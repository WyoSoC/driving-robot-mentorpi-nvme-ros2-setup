"""The shared map (runs inside web_ui_server.py): one map that the mapping cars build together and that
is the same on every car.

Each car still runs its own SLAM: that is how it knows where it is. On top of it:

1. Keyframes: every KEY_M driven or KEY_DEG turned, a car stores one lidar scan with its pose in its own
   SLAM frame (ranges in mm). The other cars fetch them (GET /api/shared/frames?since=N), so every car
   ends up with the same keyframes of all cars.
2. One frame: the shared map is in the SLAM frame of the leader, the car whose name sorts first among
   the cars whose maps are lined up (team.py). The leader also decides how each other car's frame sits
   in its own (its team.py match, GET /api/shared/state); every car uses the leader's numbers. A car
   whose map isn't lined up yet maps on its own.
3. Same map everywhere: each car adds up all keyframes in the leader's frame on an occupancy grid
   (integer log-odds: a hit adds HIT, a ray passing through a cell adds MISS). Integer sums don't depend
   on the order the keyframes arrived in, so cars with the same keyframes and the same transforms hold
   the same map, cell for cell (status: hash of the map; the page shows whether the other car has the
   same).
4. When the leader's estimate of a car's frame moves (it refines the match as the maps grow), the map is
   rebuilt from all keyframes, so slow drift between the cars doesn't smear walls.

The map is shown on the page, Nav2 plans on it and the explorer picks its frontiers from it (team.py
publishes it as /team/map, in this car's own frame). Not covered yet: corrections that this car's SLAM
makes later (loop closure) don't move keyframes stored before.
"""
import base64
import hashlib
import json
import math
import os
import threading
import time

import numpy as np

import map_merge

KEY_M = 0.20               # keyframe every this far driven ...
KEY_DEG = 10.0             # ... or this far turned
MAX_R = 6.0                # m of each ray used
RES = 0.05
CANVAS_M = 40.0            # the grid covers +-20 m around the leader's start
HIT, MISS = 9, -4          # log-odds steps (x10): p(hit) ~0.7, p(miss) ~0.4
OCC_L, FREE_L = 5, -4      # occupied from one hit, free from one miss (more evidence wins)
LOOP_S = 0.5
FETCH_S = 1.0
FETCH_MAX = 200            # keyframes per request
T_MOVE_M, T_MOVE_DEG = 0.03, 0.5   # leader: publish a new transform only if it moved this much
NICE = 10


def encode_frame(f):
    return {'car': f['car'], 'seq': f['seq'], 'pose': f['pose'], 'mount': f['mount'],
            'amin': f['amin'], 'inc': f['inc'],
            'r': base64.b64encode(np.asarray(f['r'], '<u2').tobytes()).decode()}


def decode_frame(d):
    out = dict(d)
    out['r'] = np.frombuffer(base64.b64decode(d['r']), '<u2')
    return out


def make_frame(car, seq, pose, mount, ranges, amin, inc):
    """A keyframe exactly as the other cars will receive it (ranges in mm, 0 = no return)."""
    r = np.asarray(ranges, float)
    mm = np.where(np.isfinite(r) & (r > 0.05) & (r < 60.0), np.round(r * 1000.0), 0).astype('<u2')
    f = {'car': car, 'seq': seq, 'pose': [float(v) for v in pose], 'mount': [float(v) for v in mount],
         'amin': float(amin), 'inc': float(inc), 'r': mm}
    return decode_frame(json.loads(json.dumps(encode_frame(f))))   # same floats as after the trip


class Grid:
    """Integer log-odds occupancy grid in the shared frame."""
    def __init__(self):
        n = int(CANVAS_M / RES)
        self.n = n
        self.origin = (-CANVAS_M / 2, -CANVAS_M / 2)
        self.L = np.zeros((n, n), np.int32)
        self.steps = np.arange(0.0, MAX_R, RES / 2)

    def add(self, f, T):
        """Add keyframe f (pose in its car's frame) with T = its car's frame -> shared frame."""
        x, y, th = map_merge.compose(T, f['pose'])
        mx, my, myaw = f['mount']
        lx, ly = x + math.cos(th) * mx - math.sin(th) * my, y + math.sin(th) * mx + math.cos(th) * my
        r = f['r'].astype(float) / 1000.0
        a = th + myaw + f['amin'] + np.arange(len(r)) * f['inc']
        ok = r > 0
        r, a = r[ok], a[ok]
        hit = r <= MAX_R
        free_len = np.minimum(r, MAX_R) - RES / 2
        c, s = np.cos(a), np.sin(a)
        t = self.steps[None, :]
        along = t < free_len[:, None]
        px = lx + c[:, None] * t
        py = ly + s[:, None] * t
        free = np.unique(self._cells(px[along], py[along]))
        hits = np.unique(self._cells(lx + c[hit] * r[hit], ly + s[hit] * r[hit]))
        free = np.setdiff1d(free, hits, assume_unique=True)
        free, hits = free[free >= 0], hits[hits >= 0]
        self.L.flat[free] += MISS
        self.L.flat[hits] += HIT

    def _cells(self, x, y):
        c = np.floor((x - self.origin[0]) / RES).astype(np.int64)
        r = np.floor((y - self.origin[1]) / RES).astype(np.int64)
        ok = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        return np.where(ok, r * self.n + c, -1)

    def to_grid(self):
        """map_merge grid (cropped to the mapped part) or None."""
        known = (self.L >= OCC_L) | (self.L <= FREE_L)
        if not known.any():
            return None
        rows, cols = np.nonzero(known)
        r0, r1 = max(rows.min() - 10, 0), min(rows.max() + 11, self.n)
        c0, c1 = max(cols.min() - 10, 0), min(cols.max() + 11, self.n)
        L = self.L[r0:r1, c0:c1]
        data = np.full(L.shape, map_merge.UNK, np.uint8)
        data[L <= FREE_L] = map_merge.FREE
        data[L >= OCC_L] = map_merge.OCC
        return {'data': data, 'res': RES, 'origin': [self.origin[0] + c0 * RES, self.origin[1] + r0 * RES]}


def grid_hash(g):
    if g is None:
        return None
    h = hashlib.md5(g['data'].tobytes())
    h.update(json.dumps([round(v, 6) for v in g['origin']]).encode())
    return h.hexdigest()[:10]


def close(T1, T2, m=T_MOVE_M, deg=T_MOVE_DEG):
    dyaw = abs((T1[2] - T2[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(T1[0] - T2[0], T1[1] - T2[1]) < m and math.degrees(dyaw) < deg


class SharedMap:
    def __init__(self, node, slam, nav, team, peers, me, start=True):
        self.node, self.slam, self.nav, self.team, self.peers, self.me = node, slam, nav, team, peers, me
        self.log = node.get_logger()
        self.lock = threading.Lock()
        self.frames = {}           # car -> {seq: frame}
        self.seq = 0
        self.last_key = None       # pose of this car's last keyframe
        self.mount = None
        self.T_pub = {}            # leader: car -> its frame in mine, as published
        self.leader_state = None   # follower: the leader's /api/shared/state
        self.leader = me
        self.T_used = {}           # transforms the grid was built with
        self.added = set()         # (car, seq) in the grid
        self.grid = Grid()
        self.out = None            # (map_merge grid in the shared frame, hash)
        self.version = 0
        self.fetched_at = 0.0
        self.session = None        # SLAM start time: a new mapping run starts a new shared map
        if start:
            threading.Thread(target=self._loop, daemon=True).start()

    # ---- this car's keyframes ----------------------------------------------------------
    def capture(self):
        pose, scan = self.slam.pose, self.nav.scan
        if pose is None or scan is None or time.monotonic() - scan[0] > 1.0:
            return
        if self.last_key is not None:
            dyaw = abs((pose[2] - self.last_key[2] + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(pose[0] - self.last_key[0], pose[1] - self.last_key[1]) < KEY_M and \
                    math.degrees(dyaw) < KEY_DEG:
                return
        msg = scan[1]
        if self.mount is None:
            self.mount = self._mount(msg.header.frame_id)
        self.seq += 1
        f = make_frame(self.me, self.seq, pose, self.mount, msg.ranges, msg.angle_min, msg.angle_increment)
        with self.lock:
            self.frames.setdefault(self.me, {})[self.seq] = f
        self.last_key = list(pose)

    def _mount(self, frame_id):
        try:
            from rclpy.time import Time
            t = self.slam.tf.lookup_transform('base_footprint', frame_id, Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return [t.transform.translation.x, t.transform.translation.y, yaw]
        except Exception:
            return [0.0, 0.0, 0.0]

    def frames_json(self, since=0):
        """This car's keyframes after seq `since` (GET /api/shared/frames)."""
        with self.lock:
            mine = self.frames.get(self.me, {})
            seqs = sorted(s for s in mine if s > since)[:FETCH_MAX]
            return {'car': self.me, 'session': self.session,
                    'frames': [encode_frame(mine[s]) for s in seqs], 'last': self.seq}

    # ---- who leads, and the transforms -----------------------------------------------------
    def state_json(self):
        """GET /api/shared/state: the leader's transforms, this car's map hash and keyframe counts."""
        with self.lock:
            return {'car': self.me, 'leader': self.leader,
                    'T': {c: list(T) for c, T in self.T_pub.items()} if self.leader == self.me else {},
                    'hash': self.out[1] if self.out else None,
                    'frames': {c: len(fs) for c, fs in self.frames.items()}}

    def transforms(self):
        """{car: its frame -> shared frame} for the cars in the shared map, and the leader's name."""
        mates = {m.car: m.T for m in self.team.matched()} if self.team else {}
        leader = min([self.me] + list(mates))
        if leader == self.me:
            for car, T in mates.items():        # publish only real moves: every change rebuilds the map
                if car not in self.T_pub or not close(T, self.T_pub[car]):
                    self.T_pub[car] = tuple(T)
            for car in list(self.T_pub):
                if car not in mates:
                    del self.T_pub[car]
            out = {self.me: (0.0, 0.0, 0.0)}
            out.update(self.T_pub)
            return out, leader
        self.T_pub = {}
        st = self.leader_state
        if st and st.get('car') == leader and st.get('leader') == leader and self.me in st.get('T', {}):
            out = {leader: (0.0, 0.0, 0.0)}
            out.update({c: tuple(T) for c, T in st['T'].items()})
            return out, leader
        return {self.me: (0.0, 0.0, 0.0)}, self.me    # not lined up yet (or the leader doesn't know us)

    # ---- fetching the other cars' keyframes ---------------------------------------------
    def fetch(self):
        cars = [m.car for m in self.team.matched()] if self.team else []
        for p in self.peers.summary():
            if p['online'] and (p['status'] or {}).get('slam') and p['car'] not in cars:
                cars.append(p['car'])
        for car in cars:
            try:
                have = self.frames.get(car, {})
                d = json.loads(self.peers.get(car, '/api/shared/frames', timeout=5.0,
                                              since=max(have) if have else 0))
                new = [decode_frame(f) for f in d['frames']]
                with self.lock:
                    fs = self.frames.setdefault(car, {})
                    if have and new and min(f['seq'] for f in new) == 1:   # that car started a new run
                        fs.clear()
                    for f in new:
                        fs[f['seq']] = f
            except Exception as exc:
                self.log.debug(f'shared map: frames of {car}: {exc}')
        leader = min([self.me] + [m.car for m in self.team.matched()]) if self.team else self.me
        if leader != self.me:
            try:
                self.leader_state = json.loads(self.peers.get(leader, '/api/shared/state', timeout=5.0))
            except Exception as exc:
                self.log.debug(f'shared map: state of {leader}: {exc}')
        else:
            self.leader_state = None

    # ---- building the map -------------------------------------------------------------
    def update(self):
        T, leader = self.transforms()
        with self.lock:
            frames = {c: dict(fs) for c, fs in self.frames.items() if c in T}
        rebuild = leader != self.leader or any(c not in self.T_used or self.T_used[c] != T[c] for c in T) \
            or any(c not in T for c in self.T_used)
        if rebuild:
            grid, added = Grid(), set()
        else:
            grid, added = self.grid, self.added
        new = [(c, s) for c, fs in frames.items() for s in sorted(fs) if (c, s) not in added]
        if not new and not rebuild:
            return
        for c, s in new:
            grid.add(frames[c][s], T[c])
            added.add((c, s))
        out = grid.to_grid()
        with self.lock:
            self.grid, self.added, self.T_used, self.leader = grid, added, dict(T), leader
            self.out = (out, grid_hash(out))
            self.version += 1

    def _loop(self):
        try:
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), NICE)
        except (OSError, AttributeError):
            pass
        while True:
            time.sleep(LOOP_S)
            try:
                self.step()
            except Exception as exc:
                self.log.warn(f'shared map: {exc!r}')

    def step(self):
        running = self.slam.running()
        session = getattr(self.slam, 'started_at', None) if running else None
        if session != self.session:              # mapping (re)started or stopped: start over
            with self.lock:
                self.frames, self.seq, self.last_key, self.T_pub, self.leader_state = {}, 0, None, {}, None
                self.grid, self.added, self.T_used, self.out, self.leader = Grid(), set(), {}, None, self.me
                self.session = session
        if not running:
            return
        self.capture()
        if time.monotonic() - self.fetched_at >= FETCH_S:
            self.fetched_at = time.monotonic()
            self.fetch()
        self.update()

    # ---- for team.py (Nav2, explorer, page) and the page -------------------------------
    def joined(self):
        return len(self.T_used) > 1

    def shared_grid(self):
        """The shared map (leader's frame) and its hash."""
        with self.lock:
            return self.out if self.out else (None, None)

    def grid_in_my_frame(self):
        """The shared map in this car's own SLAM frame (for Nav2, the explorer and the page)."""
        with self.lock:
            out, T_me = self.out, self.T_used.get(self.me)
        if not out or out[0] is None or T_me is None:
            return None
        if T_me == (0.0, 0.0, 0.0):
            return out[0]
        return map_merge.resample(out[0], map_merge.invert(T_me))

    def status(self):
        with self.lock:
            counts = {c: len(fs) for c, fs in self.frames.items()}
            used = sorted(self.T_used)
            h = self.out[1] if self.out else None
        same = {}
        for p in self.peers.summary():
            sh = ((p['status'] or {}).get('shared') or {})
            if p['online'] and sh.get('hash') and p['car'] in used:
                same[p['car']] = sh['hash'] == h
        return {'joined': len(used) > 1, 'leader': self.leader, 'cars': used, 'frames': counts,
                'hash': h, 'same_as': same}
