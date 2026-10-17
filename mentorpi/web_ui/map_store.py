"""Saved maps (mentorpi/maps/ on the host, /home/ubuntu/maps in the container): NAME.yaml + NAME.png.

Each map gets a short content hash (PNG pixels + resolution + origin, not the name), so two cars can tell
whether they are on the same map even if one copy was renamed. Positions are only comparable then.
"""
import hashlib
import os
import re
import threading

MAPS_DIR = '/home/ubuntu/maps'
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')

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
