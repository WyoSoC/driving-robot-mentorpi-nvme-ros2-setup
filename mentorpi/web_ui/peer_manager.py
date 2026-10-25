"""Status exchange with the other cars (runs inside web_ui_server.py, plain HTTP, no ROS).

Every car's web server answers GET /api/self with its own status (battery, mode, navigation pose / goal /
plan, map list). PeerManager asks each peer for it twice a second, at all times (also in charging mode:
the web server keeps running), and backs off to every 3 s while a peer is unreachable.

Maps are kept in sync: every SYNC_S, any map an online peer has and this car doesn't (by content hash)
is copied here (NAME, or NAME_<car> if a different map here already has that name). A map deleted on
any car (map_store.delete: its content hash is listed in that car's status, 'maps_deleted') is never
copied back and is deleted here too, unless this car is navigating on it right now.

Peers come from PEERS="host:port host:port" (start_web_ui.sh builds it from fleet.env, leaving out this
car). Positions are in the peer's map frame: they only mean something here when both cars use the same
map (same content hash, see map_store.py).
"""
import json
import threading
import time
import urllib.parse
import urllib.request

import map_store

POLL_S = 0.5
OFFLINE_POLL_S = 3.0
TIMEOUT_S = 1.5
STALE_S = 3.0           # no answer for this long -> shown as offline
SYNC_S = 30.0           # map sync interval


class Peer:
    def __init__(self, addr):
        self.addr = addr                    # host:port
        self.car = addr.split('.')[0].split(':')[0]   # until the peer tells us its name
        self.status = None
        self.last_ok = None                 # monotonic
        self.error = 'not contacted yet'

    def url(self, path, **query):
        q = ('?' + urllib.parse.urlencode(query)) if query else ''
        return f'http://{self.addr}{path}{q}'

    def online(self):
        return self.last_ok is not None and time.monotonic() - self.last_ok < STALE_S


class PeerManager:
    def __init__(self, me, peers, logger):
        self.me = me
        self.log = logger
        self.peers = [Peer(a) for a in peers.split() if a]
        self.stop_evt = threading.Event()
        self.sync_note = ''
        self.in_use = lambda name: False   # set by the web server: map in use by navigation
        for p in self.peers:
            threading.Thread(target=self._loop, args=(p,), daemon=True).start()
        if self.peers:
            threading.Thread(target=self._sync_loop, daemon=True).start()
        if self.peers:
            self.log.info('status exchange with ' + ', '.join(p.addr for p in self.peers))
        else:
            self.log.info('no other cars configured (fleet.env): status exchange off')

    def _get(self, url, timeout=TIMEOUT_S):
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read()

    def _loop(self, p):
        was_online = None
        while not self.stop_evt.is_set():
            try:
                st = json.loads(self._get(p.url('/api/self')))
                p.status, p.last_ok, p.error = st, time.monotonic(), None
                p.car = st.get('car') or p.car
            except Exception as exc:          # unreachable, timeout, bad JSON: keep the last status
                p.error = str(getattr(exc, 'reason', exc))[:120]
            online = p.online()
            if online != was_online:
                self.log.info(f'{p.car} ({p.addr}) ' + ('online' if online else f'offline: {p.error}'))
                was_online = online
            self.stop_evt.wait(POLL_S if online else OFFLINE_POLL_S)

    def _sync_loop(self):
        self.stop_evt.wait(5.0)
        while not self.stop_evt.is_set():
            try:
                self.sync_maps()
            except Exception as exc:
                self.log.warn(f'map sync: {exc}')
            self.stop_evt.wait(SYNC_S)

    def sync_maps(self):
        """Copy every map an online peer has and this car lacks (same content = same map)."""
        copied = []
        for p in self.peers:
            if not p.online() or not p.status:
                continue
            gone = map_store.deleted()
            for h in p.status.get('maps_deleted') or []:      # deleted on that car: delete here too
                name = map_store.by_hash(h)
                if name and not self.in_use(name):
                    map_store.delete(name)
                    self.log.info(f'map "{name}" deleted (deleted on {p.car})')
                elif h not in gone and not name:
                    map_store.remember_deleted(h, '?')
            gone = map_store.deleted()
            have = {m['hash'] for m in map_store.listing()}
            for m in p.status.get('maps') or []:
                if m.get('hash') in have or m.get('hash') in gone or not map_store.NAME_RE.match(m.get('name', '')):
                    continue
                try:
                    local = self.fetch_map(p.car, m['name'])
                    copied.append(f'{local} (from {p.car})')
                    have.add(m['hash'])
                except Exception as exc:
                    self.log.warn(f'map sync: cannot copy "{m.get("name")}" from {p.car}: {exc}')
        if copied:
            self.sync_note = 'copied ' + ', '.join(copied)
        return copied

    def shutdown(self):
        self.stop_evt.set()

    def find(self, car):
        for p in self.peers:
            if p.car == car:
                return p
        return None

    def summary(self):
        """For the page: every peer with its latest status and how old it is."""
        now = time.monotonic()
        return [{
            'car': p.car,
            'addr': p.addr,
            'online': p.online(),
            'age_s': round(now - p.last_ok, 1) if p.last_ok else None,
            'error': p.error,
            'status': p.status,
        } for p in self.peers]

    def get(self, car, path, timeout=TIMEOUT_S, **query):
        """GET path on another car's web server (bytes)."""
        p = self.find(car)
        if not p:
            raise LookupError(f'unknown car {car}')
        return self._get(p.url(path, **query), timeout=timeout)

    def post(self, car, path, timeout=TIMEOUT_S, **query):
        p = self.find(car)
        if not p:
            raise LookupError(f'unknown car {car}')
        req = urllib.request.Request(p.url(path, **query), data=b'', method='POST')
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read()

    def map_png(self, car, name):
        p = self.find(car)
        if not p:
            raise LookupError(f'unknown car {car}')
        return self._get(p.url('/api/maps/file', name=name, ext='png'), timeout=10)

    def fetch_map(self, car, name):
        """Copy map NAME from CAR into this car's maps. Returns the local name (NAME, or NAME_CAR when a
        different map is already saved here as NAME). Nothing is copied when the same map is already here."""
        p = self.find(car)
        if not p:
            raise LookupError(f'unknown car {car}')
        if not map_store.NAME_RE.match(name):
            raise ValueError('bad map name')
        png = self._get(p.url('/api/maps/file', name=name, ext='png'), timeout=10)
        yaml = self._get(p.url('/api/maps/file', name=name, ext='yaml'), timeout=10).decode()
        want = map_store.content_hash(png, map_store.parse_yaml(yaml))
        local = {m['hash']: m['name'] for m in map_store.listing()}
        if want in local:
            return local[want]
        if want in map_store.deleted():
            raise ValueError(f'map "{name}" was deleted')
        target = name if name not in map_store.names() else f'{name}_{car}'[:40]
        if target in map_store.names():
            raise ValueError(f'a different map is already saved as "{target}"')
        map_store.save_copy(target, png, yaml)
        self.log.info(f'map "{name}" copied from {car} as "{target}"')
        return target
