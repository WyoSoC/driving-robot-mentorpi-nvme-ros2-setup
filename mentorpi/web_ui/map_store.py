"""Saved maps (mentorpi/maps/ on the host, /home/ubuntu/maps in the container): NAME.yaml + NAME.png.

Each map gets a short content hash (PNG pixels + resolution + origin, not the name), so two cars can tell
whether they are on the same map even if one copy was renamed. Positions are only comparable then.

Managing maps (Map panel -> Maps): rename() renames every file of a map (NAME.yaml/.png, and slam_toolbox's
NAME.data/.posegraph if saved while mapping) and fixes the image line; delete() removes them and records
the map's content hash in .deleted.json, so the map sync between the cars (peer_manager.py) never copies
it back, and the other cars delete their copy too (they see the list in this car's status).
"""
import hashlib
import json
import os
import re
import struct
import threading
import time

MAPS_DIR = '/home/ubuntu/maps'
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')

EXTRA_EXT = ('data', 'posegraph')   # slam_toolbox pose graph, saved next to the map while mapping
DELETED = '.deleted.json'           # content hashes of deleted maps (never copied back)

_cache = {}            # name -> (key, info)
_lock = threading.Lock()


def names():
    try:
        return sorted(f[:-5] for f in os.listdir(MAPS_DIR) if f.endswith('.yaml') and NAME_RE.match(f[:-5]))
    except OSError:
        return []


def parse_yaml(text):
    meta = {}
    for line in text.splitlines():
        k, _, v = line.partition(':')
        meta[k.strip()] = v.strip()
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')[:2]]
    return {'image': os.path.basename(meta['image']), 'resolution': float(meta['resolution']), 'origin': origin}


def meta(name):
    """resolution, origin, image file name and content hash of a saved map (cached by file times)."""
    yaml_path = os.path.join(MAPS_DIR, name + '.yaml')
    st_y = os.stat(yaml_path)
    with open(yaml_path) as f:
        m = parse_yaml(f.read())
    png_path = os.path.join(MAPS_DIR, m['image'])
    st_p = os.stat(png_path)
    key = (st_y.st_mtime_ns, st_y.st_size, st_p.st_mtime_ns, st_p.st_size)
    with _lock:
        hit = _cache.get(name)
        if hit and hit[0] == key:
            return dict(hit[1])
    with open(png_path, 'rb') as f:
        png = f.read()
    m['hash'] = content_hash(png, m)
    m['name'] = name
    if png[:8] == b'\x89PNG\r\n\x1a\n' and len(png) >= 24:      # size in m from the PNG header
        w, h = struct.unpack('>II', png[16:24])
        m['size_m'] = [round(w * m['resolution'], 1), round(h * m['resolution'], 1)]
    m['saved'] = int(st_p.st_mtime)
    m['pose_graph'] = os.path.exists(os.path.join(MAPS_DIR, name + '.posegraph'))
    with _lock:
        _cache[name] = (key, m)
    return dict(m)


def content_hash(png, m):
    h = hashlib.sha1(png)
    h.update(f"|{m['resolution']:.4f}|{m['origin'][0]:.4f}|{m['origin'][1]:.4f}".encode())
    return h.hexdigest()[:12]


def listing():
    out = []
    for n in names():
        try:
            out.append(meta(n))
        except (OSError, KeyError, ValueError):
            pass
    return out


def png_bytes(name):
    m = meta(name)
    with open(os.path.join(MAPS_DIR, m['image']), 'rb') as f:
        return f.read()


def yaml_text(name):
    with open(os.path.join(MAPS_DIR, name + '.yaml')) as f:
        return f.read()


def by_hash(h):
    for m in listing():
        if m['hash'] == h:
            return m['name']
    return None


def deleted():
    """{content hash: {'name', 'time'}} of maps deleted here or on another car."""
    try:
        with open(os.path.join(MAPS_DIR, DELETED)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _write_deleted(d):
    tmp = os.path.join(MAPS_DIR, DELETED + '.tmp')
    with open(tmp, 'w') as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, os.path.join(MAPS_DIR, DELETED))


def _files(name):
    """Every file of map NAME: (path, extension)."""
    out = []
    try:
        image = meta(name)['image']
    except (OSError, KeyError, ValueError):
        image = name + '.png'
    out.append((os.path.join(MAPS_DIR, image), 'png'))
    for ext in ('yaml',) + EXTRA_EXT:
        p = os.path.join(MAPS_DIR, f'{name}.{ext}')
        if os.path.exists(p):
            out.append((p, ext))
    return out


def rename(name, to):
    if name not in names():
        raise ValueError(f'no map "{name}"')
    if not NAME_RE.match(to or ''):
        raise ValueError('name: 1-40 letters, digits, - or _')
    if to == name:
        return
    if to in names() or any(os.path.exists(os.path.join(MAPS_DIR, f'{to}.{e}')) for e in ('png',) + EXTRA_EXT):
        raise ValueError(f'a map "{to}" already exists')
    files = _files(name)                       # (meta() takes the lock itself)
    yaml = yaml_text(name)
    with _lock:
        for path, ext in files:
            if ext != 'yaml':
                os.replace(path, os.path.join(MAPS_DIR, f'{to}.{ext}'))
        lines = [f'image: {to}.png' if l.split(':')[0].strip() == 'image' else l for l in yaml.splitlines()]
        tmp = os.path.join(MAPS_DIR, f'.{to}.yaml.tmp')
        with open(tmp, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(tmp, os.path.join(MAPS_DIR, f'{to}.yaml'))
        os.remove(os.path.join(MAPS_DIR, f'{name}.yaml'))
        _cache.pop(name, None)


def delete(name):
    """Delete map NAME (all its files) and remember its content, so it is not copied back. Returns its hash."""
    if name not in names():
        raise ValueError(f'no map "{name}"')
    h = meta(name)['hash']
    files = _files(name)
    with _lock:
        d = deleted()
        d[h] = {'name': name, 'time': int(time.time())}
        _write_deleted(d)
        for path, _ in files:
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
        _cache.pop(name, None)
    return h


def remember_deleted(h, name):
    """Record that a map (content hash h) was deleted on another car."""
    with _lock:
        d = deleted()
        if h not in d:
            d[h] = {'name': name, 'time': int(time.time())}
            _write_deleted(d)


def save_copy(name, png, yaml):
    """Store a map received from another car as NAME.{png,yaml} (image line points at NAME.png)."""
    if not NAME_RE.match(name):
        raise ValueError(f'bad map name {name!r}')
    lines = [f'image: {name}.png' if l.split(':')[0].strip() == 'image' else l for l in yaml.splitlines()]
    for ext, data, mode in (('png', png, 'wb'), ('yaml', '\n'.join(lines) + '\n', 'w')):
        tmp = os.path.join(MAPS_DIR, f'.{name}.{ext}.tmp')
        with open(tmp, mode) as f:
            f.write(data)
        os.replace(tmp, os.path.join(MAPS_DIR, f'{name}.{ext}'))
    return meta(name)
