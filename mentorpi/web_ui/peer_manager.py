"""Status exchange with the other cars (runs inside web_ui_server.py, plain HTTP, no ROS).

Every car's web server answers GET /api/self with its own status (battery, mode, navigation pose / goal /
plan, map list). PeerManager asks each peer for it twice a second, at all times (also in charging mode:
the web server keeps running), and backs off to every 3 s while a peer is unreachable.

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
        for p in self.peers:
            threading.Thread(target=self._loop, args=(p,), daemon=True).start()
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
        target = name if name not in map_store.names() else f'{name}_{car}'[:40]
        if target in map_store.names():
            raise ValueError(f'a different map is already saved as "{target}"')
        map_store.save_copy(target, png, yaml)
        self.log.info(f'map "{name}" copied from {car} as "{target}"')
        return target
