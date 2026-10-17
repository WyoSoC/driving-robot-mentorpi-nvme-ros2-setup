"""Mapping together (runs inside web_ui_server.py): two cars exploring one space at once.

Each car runs its own SLAM, so each live map has its own frame. While this car maps, every TEAM_S it
fetches every other mapping car's live map (GET /api/slam/grid on that car) and lines it up with its own
(map_merge.align; after the first match only a quick refine from the last result). A match is used once
two checks in a row agree (within STABLE_M / STABLE_DEG), so a one-off wrong fit early on, when the maps
are small, is never used.

With a match, for the explorer (explore_manager.py):
- covered(): the cells of this car's map that are unmapped here but mapped by the other car. A frontier
  that only leads there is not worth driving to.
- claims(): where the other car is and the frontier it is driving to, in this car's frame. Frontiers
  near its goal are left to it, and frontiers closer to it than to this car cost more.
For fleet.py: to_mine(car) gives the transform from the other car's frame into this car's, so the other
car is in the costmap and the right-of-way rule works while exploring too.

The alignment (~5 s for a full search, ~0.5 s to refine) runs in a separate low-priority process, so it
never delays the web server's driving relay or sensors. A full search runs at most every FULL_EVERY_S.

The cars can start anywhere. Until their maps overlap each explores alone (the other car is an ordinary
obstacle to lidar and camera); cars that start in the same room line up within a few seconds.
"""
import base64
import concurrent.futures
import json
import multiprocessing
import math
import os
import threading
import time
import urllib.request

import numpy as np

import map_merge

TEAM_S = 10.0            # s between live map checks
FETCH_TIMEOUT = 5.0
STABLE_M = 0.30          # two alignments within this (and STABLE_DEG) = a match to use
STABLE_DEG = 8.0
LOST_AFTER = 3           # failed checks in a row -> match dropped
NICE = 10                # the alignment process runs at lower priority than driving
FULL_EVERY_S = 20.0      # s between full searches while not matched
ALIGN_TIMEOUT = 60.0


def _worker_init():
    os.nice(NICE)


def _align_job(mine, other, start):
    """In the worker process: refine from `start` if given and good, else full search."""
    if start is not None:
        T, q = map_merge.refine(mine, other, start)
        if map_merge.good_match(q):
            return T, q, 'refine'
    T, q = map_merge.align(mine, other)
    return T, q, 'full'


def grid_from_msg(msg):
    """nav_msgs/OccupancyGrid -> map_merge grid (free < 25, occupied >= 65, else unknown)."""
    h, w = msg.info.height, msg.info.width
    g = np.array(msg.data, dtype=np.int16).reshape(h, w)
    data = np.full((h, w), map_merge.UNK, np.uint8)
    data[(g >= 0) & (g < 25)] = map_merge.FREE
    data[g >= 65] = map_merge.OCC
    o = msg.info.origin.position
    return {'data': data, 'res': float(msg.info.resolution), 'origin': [float(o.x), float(o.y)]}


def encode_grid(grid):
    return {'res': grid['res'], 'origin': grid['origin'],
            'png': base64.b64encode(map_merge.to_png(grid)).decode()}


def decode_grid(d):
    return map_merge.from_png(base64.b64decode(d['png']), d['res'], d['origin'])


def close(T1, T2):
    dyaw = abs((T1[2] - T2[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(T1[0] - T2[0], T1[1] - T2[1]) < STABLE_M and math.degrees(dyaw) < STABLE_DEG


class Mate:
    """What this car knows about one other mapping car."""
    def __init__(self, car):
        self.car = car
        self.grid = None           # its live map (its frame)
        self.T = None              # its frame -> this car's frame (used)
        self.candidate = None      # last alignment, waiting for a second one that agrees
        self.quality = None
        self.fails = 0
        self.checked_at = None
        self.matched_at = None
        self.pose = None           # in its frame
        self.goal = None
        self.note = 'not checked yet'
        self.full_at = None


class Team:
    def __init__(self, node, slam, peers, me):
        self.node, self.slam, self.peers, self.me = node, slam, peers, me
        self.log = node.get_logger()
        self.mates = {}            # car -> Mate
        self.lock = threading.Lock()
        self._grid_cache = (None, None)   # (map version, encoded)
        self.pool = None
        threading.Thread(target=self._loop, daemon=True).start()

    def _run(self, mine, other, start):
        if self.pool is None:
            self.pool = concurrent.futures.ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context('spawn'), initializer=_worker_init)
        try:
            return self.pool.submit(_align_job, mine, other, start).result(timeout=ALIGN_TIMEOUT)
        except Exception:
            self.pool.shutdown(wait=False, cancel_futures=True)   # broken or stuck worker: start a new one
            self.pool = None
            raise

    # ---- this car's live map, for the other cars (GET /api/slam/grid) ----------------------
    def my_grid_json(self):
        msg = self.slam.grid
        if msg is None or not self.slam.running():
            return None
        key = id(msg)
        if self._grid_cache[0] != key:
            self._grid_cache = (key, encode_grid(grid_from_msg(msg)))
        out = dict(self._grid_cache[1])
        out['car'] = self.me
        out['pose'] = self.slam.pose
        return out

    # ---- matching loop -----------------------------------------------------------------
    def _loop(self):
        try:
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), NICE)   # fetching + bookkeeping
        except (OSError, AttributeError):
            pass
        while True:
            time.sleep(TEAM_S)
            try:
                self.check()
            except Exception as exc:
                self.log.warn(f'team: {exc}')

    def mapping_peers(self):
        out = []
        for p in self.peers.summary():
            st = p['status'] or {}
            if p['online'] and st.get('slam'):
                out.append(p)
        return out

    def check(self):
        if not self.slam.running() or self.slam.grid is None:
            with self.lock:
                self.mates.clear()
            return
        mine = grid_from_msg(self.slam.grid)
        seen = set()
        for p in self.mapping_peers():
            car = p['car']
            seen.add(car)
            m = self.mates.get(car) or Mate(car)
            try:
                d = json.loads(self.peers.get(car, '/api/slam/grid', timeout=FETCH_TIMEOUT))
                m.grid, m.pose = decode_grid(d), d.get('pose')
            except Exception as exc:
                m.note = f'no live map: {exc}'
                with self.lock:
                    self.mates[car] = m
                continue
            self._match(mine, m)
            with self.lock:
                self.mates[car] = m
        with self.lock:
            for car in list(self.mates):
                if car not in seen:
                    del self.mates[car]

    def _match(self, mine, m):
        t0 = time.time()
        start = m.T if m.T is not None else m.candidate
        if start is None and m.full_at and t0 - m.full_at < FULL_EVERY_S:
            return                                   # not matched: full search only every FULL_EVERY_S
        T, q, how = self._run(mine, m.grid, start)
        if how == 'full':
            m.full_at = t0
        m.checked_at, m.quality = time.time(), q
        took = time.time() - t0
        if T is None or not map_merge.good_match(q):
            m.fails += 1
            m.candidate = None
            if m.T is not None and m.fails >= LOST_AFTER:
                self.log.info(f'team: lost the match with {m.car}')
                m.T = None
            m.note = ('maps don\'t overlap yet' if m.T is None else 'match kept (last check failed)') + f' ({took:.1f} s)'
            return
        m.fails = 0
        if m.T is not None and close(T, m.T):
            m.T = T
            m.note = f'matched ({took:.1f} s)'
        elif m.candidate is not None and close(T, m.candidate):
            if m.T is None:
                self.log.info(f'team: map of {m.car} lined up: x={T[0]:.2f} y={T[1]:.2f} '
                              f'yaw={math.degrees(T[2]):.0f} deg')
            m.T, m.matched_at, m.candidate = T, time.time(), None
            m.note = f'matched ({took:.1f} s)'
        else:
            m.candidate = T
            m.note = 'lined up once, checking again' + (' (moved: re-checking)' if m.T is not None else '')

    # ---- for the explorer and fleet.py ------------------------------------------------------
    def to_mine(self, car):
        with self.lock:
            m = self.mates.get(car)
            return m.T if m else None

    def matched(self):
        with self.lock:
            return [m for m in self.mates.values() if m.T is not None]

    def covered(self, grid_shape, res, origin):
        """bool mask over this car's grid: mapped (free or wall) by a matched other car."""
        out = np.zeros(grid_shape, bool)
        h, w = grid_shape
        r, c = np.mgrid[0:h, 0:w]
        xy = np.stack([origin[0] + (c.ravel() + 0.5) * res, origin[1] + (r.ravel() + 0.5) * res], axis=-1)
        for m in self.matched():
            v = map_merge.lookup(m.grid, map_merge.transform(xy, map_merge.invert(m.T)))
            out |= (v != map_merge.UNK).reshape(h, w)
        return out

    def claims(self):
        """[{car, pose: [x, y], goal: [x, y] or None}] of matched cars, in this car's frame."""
        out = []
        for p in self.peers.summary():
            T = self.to_mine(p['car'])
            st = p['status'] or {}
            nv = st.get('nav') or {}
            if T is None or not p['online'] or not nv.get('pose'):
                continue
            pose = map_merge.transform(np.array([nv['pose'][:2]]), T)[0].tolist()
            goal = None
            if nv.get('goal') and nv.get('goal_state') in ('sending', 'active', 'yielding'):
                goal = map_merge.transform(np.array([nv['goal'][:2]]), T)[0].tolist()
            out.append({'car': p['car'], 'pose': pose, 'goal': goal})
        return out

    def merged_grid(self, mine):
        """This car's map with every matched car's map added (for saving the team map)."""
        out = mine
        for m in self.matched():
            out = map_merge.merge(out, m.grid, m.T)
        return out

    def status(self):
        with self.lock:
            return [{'car': m.car, 'matched': m.T is not None,
                     'T': [round(m.T[0], 2), round(m.T[1], 2), round(m.T[2], 3)] if m.T else None,
                     'agree': (round(min(m.quality['ab']['agree'], m.quality['ba']['agree']), 2)
                               if m.quality else None),
                     'checked_s_ago': round(time.time() - m.checked_at, 1) if m.checked_at else None,
                     'note': m.note} for m in self.mates.values()]
