"""Two cars in a new place (tests/run.sh test_new_place.py): no saved map, the cars start close together
(their first scans overlap), then drive off in opposite directions (west wing / east wing of a made-up
apartment). Each car maps in its own frame; the real team.py matching and shared_map.py run on their live
maps through a fake link. Checks: the maps line up early, the match is KEPT while the cars' maps grow
apart (they then only share the start room), the transform stays right, both cars hold the same shared
map, and it covers both wings. Doesn't drive."""
import json, math, sys, time, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import map_merge as mm
import shared_map as sm
import team as tm

rng = np.random.default_rng(5)
RES = 0.05
# SLAM drift of jpi2 while it drives (deg per m), corrected later in its own map (as slam_toolbox's loop
# closure does) but not in the keyframes it stored meanwhile; re-anchoring must fix those.
DRIFT_DEG_PER_M = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
REANCHOR = (sys.argv[2] != '0') if len(sys.argv) > 2 else True


def apartment():
    """16 x 10 m: west wing | start room | east wing, doorways, furniture (nothing symmetric)."""
    W, H = int(16 / RES), int(10 / RES)
    d = np.full((H, W), mm.FREE, np.uint8)
    def wall(x0, y0, x1, y1):
        d[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = mm.OCC
    def gap(x0, y0, x1, y1):
        d[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = mm.FREE
    wall(0, 0, 16, 0.1); wall(0, 9.9, 16, 10); wall(0, 0, 0.1, 10); wall(15.9, 0, 16, 10)
    wall(5.5, 0, 5.6, 10); gap(5.5, 4.0, 5.6, 5.0)            # start room | west, door 1 m
    wall(10.5, 0, 10.6, 10); gap(10.5, 6.0, 10.6, 7.0)        # start room | east, door 1 m
    wall(0, 5.0, 3.0, 5.1); wall(13.0, 3.5, 16, 3.6)          # half walls in the wings
    wall(2.0, 7.5, 3.5, 8.3); wall(1.0, 1.5, 1.8, 3.0)        # west furniture
    wall(12.0, 8.0, 13.6, 8.6); wall(14.2, 1.0, 15.0, 2.2); wall(11.5, 1.0, 12.0, 1.4)   # east furniture
    wall(7.0, 8.0, 8.4, 8.8); wall(9.2, 1.0, 9.9, 2.6); wall(6.2, 1.2, 6.6, 1.6)         # start room furniture
    return {'data': d, 'res': RES, 'origin': [0.0, 0.0]}


truth = apartment()
N_RAYS = 450


def scan(x, y, yaw, max_r=8.0):
    a = -math.pi + np.arange(N_RAYS) * (2 * math.pi / N_RAYS)
    t = np.arange(0.05, max_r, RES / 2)
    g = yaw + a
    xy = np.stack([x + np.cos(g)[:, None] * t, y + np.sin(g)[:, None] * t], -1).reshape(-1, 2)
    occ = (mm.lookup(truth, xy) == mm.OCC).reshape(N_RAYS, len(t))
    first = np.where(occ.any(axis=1), occ.argmax(axis=1), -1)
    r = np.where(first >= 0, t[np.maximum(first, 0)], np.inf)
    r = r + rng.normal(0, 0.01, N_RAYS)
    r[rng.random(N_RAYS) < 0.05] = np.inf
    return r, -math.pi, 2 * math.pi / N_RAYS


def route(points, step=0.2):
    """Poses every `step` m along the waypoints, facing the way of travel."""
    out = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        yaw = math.atan2(y1 - y0, x1 - x0)
        out += [[x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw] for i in range(n)]
    return out


# start 0.6 m apart in the start room, then west / east
ROUTE = {'jpi1': route([(8.0, 4.4), (7.0, 4.5), (5.0, 4.5), (3.5, 4.0), (1.0, 3.8), (1.0, 6.5),
                         (4.5, 6.5), (4.5, 9.0), (1.0, 9.0)]),
         'jpi2': route([(8.0, 5.0), (9.0, 5.5), (10.0, 6.5), (11.5, 6.5), (13.5, 5.0), (15.0, 5.0),
                         (15.0, 8.5), (11.5, 9.0), (11.5, 2.5), (13.0, 2.5)])}
F = {'jpi1': (2.0, -1.0, math.radians(30)), 'jpi2': (-3.0, 4.0, math.radians(-100))}   # own frame -> true


class Link:
    """Fake peers of one car: the other cars' web server endpoints, through JSON."""
    def __init__(self, me): self.me, self.cars = me, {}
    def summary(self):
        out = []
        for c, car in self.cars.items():
            if c == self.me:
                continue
            pose = car.slam.pose
            out.append({'car': c, 'online': True, 'age_s': 0.1,
                        'status': {'slam': True, 'shared': {'hash': car.shared.out[1] if car.shared.out else None},
                                   'nav': {'running': True, 'slam_mode': True, 'pose_set': True, 'pose': pose,
                                           'goal_state': 'idle', 'goal': None, 'plan': []}}})
        return out
    def get(self, c, path, timeout=0, **q):
        car = self.cars[c]
        if path == '/api/slam/grid':
            d = car.team.my_grid_json()
        elif path == '/api/shared/frames':
            d = car.shared.frames_json(int(q.get('since', 0)))
        else:
            d = car.shared.state_json()
        return json.dumps(d).encode()


def msg_of(g):
    info = types.SimpleNamespace(height=g['data'].shape[0], width=g['data'].shape[1], resolution=g['res'],
                                 origin=types.SimpleNamespace(position=types.SimpleNamespace(x=g['origin'][0], y=g['origin'][1])))
    d = np.full(g['data'].shape, -1, np.int8); d[g['data'] == mm.FREE] = 0; d[g['data'] == mm.OCC] = 100
    return types.SimpleNamespace(info=info, data=d.ravel().tolist())


class Car:
    def __init__(self, me, link):
        log = types.SimpleNamespace(info=lambda *a: print('   ', me, *a), warn=print, debug=lambda *a: None)
        self.node = types.SimpleNamespace(get_logger=lambda: log)
        self.me, self.own = me, sm.Grid()       # its live map (SLAM stand-in) in its own frame
        self.slam = types.SimpleNamespace(pose=None, grid=None, running=lambda: True, started_at=1.0,
                                          tf=types.SimpleNamespace(lookup_transform=lambda *a: (_ for _ in ()).throw(Exception())))
        self.nav = types.SimpleNamespace(scan=None)
        t = tm.Team.__new__(tm.Team)
        t.node, t.slam, t.peers, t.me, t.log = self.node, self.slam, link, me, log
        t.mates, t.lock, t._grid_cache, t.pool = {}, __import__('threading').Lock(), (None, None), None
        t._run = lambda mine, other, start, **k: tm._align_job(mine, other, start, **k)
        t.shared, t.team_msg = None, None
        self.team = t
        self.shared = sm.SharedMap(self.node, self.slam, self.nav, t, link, me, start=False)
        self.shared.session = 1.0
        t.shared = self.shared
        self.driven = 0.0
        self.drift = 0.0
        self.start = None

    def at(self, p_true, prev):
        if prev is not None:
            dm = math.hypot(p_true[0] - prev[0], p_true[1] - prev[1])
            self.driven += dm
            if self.me == 'jpi2':
                self.drift += math.radians(DRIFT_DEG_PER_M) * dm
        r, amin, inc = scan(*p_true)
        own = list(mm.compose(mm.invert(F[self.me]), p_true))      # corrected pose (own frame)
        if self.start is None:
            self.start = own
        # the pose the car believed when it stored the keyframe: bent around its start by the drift
        sx, sy = self.start[0], self.start[1]
        c, s_ = math.cos(self.drift), math.sin(self.drift)
        believed = [sx + c * (own[0] - sx) - s_ * (own[1] - sy), sy + s_ * (own[0] - sx) + c * (own[1] - sy),
                    own[2] + self.drift]
        self.slam.pose = believed
        self.nav.scan = (time.monotonic(), types.SimpleNamespace(ranges=r, angle_min=amin, angle_increment=inc,
                                                                header=types.SimpleNamespace(frame_id='laser')))
        n = len(self.shared.frames.get(self.me, {}))
        self.shared.capture()
        fs = self.shared.frames.get(self.me, {})
        if len(fs) > n:                          # its own live map (corrected poses) grows
            self.own.add(dict(fs[max(fs)], pose=own), (0.0, 0.0, 0.0))
            self.slam.grid = msg_of(self.own.to_grid())
        self.slam.pose = own                     # after the correction the car knows where it is


def main():
    links = {c: Link(c) for c in ('jpi1', 'jpi2')}
    cars = {c: Car(c, links[c]) for c in ('jpi1', 'jpi2')}
    for l in links.values():
        l.cars = cars
    T_true = mm.compose(mm.invert(F['jpi1']), F['jpi2'])          # jpi2 frame -> jpi1 frame
    n = max(len(r) for r in ROUTE.values())
    joined_at, lost, history = None, 0, []
    prev = {c: None for c in cars}
    for i in range(n):
        for c, car in cars.items():
            p = ROUTE[c][min(i, len(ROUTE[c]) - 1)]
            car.at(p, prev[c]); prev[c] = p
        if i % 5 == 0:                           # team.py checks every 5 s; the cars drive 1 m in that time
            for car in cars.values():
                car.team.check()
        if REANCHOR:
            for car in cars.values():
                for _ in range(3):
                    car.shared.reanchor()
        for car in cars.values():
            car.shared.fetch()
        for car in cars.values():
            car.shared.update()
        for car in cars.values():
            car.shared.fetch(); car.shared.update()
        j = all(car.shared.joined() for car in cars.values())
        if j and joined_at is None:
            joined_at = i
        if joined_at is not None and not j:
            lost += 1
        if i % 5 == 0:
            T = cars['jpi1'].team.to_mine('jpi2')
            # error where the cars are (jpi2's start), not at the far-away frame origin
            p0 = np.array([cars['jpi2'].start[:2]])
            err = (float(np.hypot(*(mm.transform(p0, T) - mm.transform(p0, T_true))[0])),
                   math.degrees(abs(T[2] - T_true[2]))) if T else None
            m1 = cars['jpi1'].team.mates.get('jpi2')
            q = m1.quality if m1 else None
            ov = max(q['ab']['overlap'], q['ba']['overlap']) if q else None
            history.append((i, j, err, ov))
            d = math.hypot(prev['jpi1'][0] - prev['jpi2'][0], prev['jpi1'][1] - prev['jpi2'][1])
            print(f'  step {i:3d}: cars {d:4.1f} m apart, joined {j}, '
                  f'T error {"-" if not err else f"{err[0]*100:.1f} cm {err[1]:.2f} deg"}, '
                  f'overlap {"-" if ov is None else f"{ov:.2f}"}, note "{m1.note if m1 else "-"}"')
    s1, s2 = cars['jpi1'].shared.status(), cars['jpi2'].shared.status()
    g, h = cars['jpi1'].shared.shared_grid()
    # coverage: true free floor of each wing mapped in the shared map
    def covered(x0, x1):
        cells = mm.cells_xy(truth, truth['data'] == mm.FREE)
        cells = cells[(cells[:, 0] > x0) & (cells[:, 0] < x1)]
        mine = mm.transform(cells, mm.invert(F['jpi1']))
        return np.mean(mm.lookup(g, mine) == mm.FREE)
    q = mm.quality(truth, g, F['jpi1'])
    print(f'drift {DRIFT_DEG_PER_M} deg/m (jpi2), re-anchoring {REANCHOR}: keyframes re-anchored '
          f'jpi1 {s1["reanchored"]}, jpi2 {s2["reanchored"]}')
    print(f'joined at step {joined_at}, dropped afterwards in {lost} steps; hash {s1["hash"]} / {s2["hash"]}')
    print(f'shared map vs truth: walls agree {q["ab"]["agree"]:.2f}, clash {q["ab"]["clash"]:.2f}; '
          f'free floor mapped: west {covered(0, 5.5):.0%}, start room {covered(5.6, 10.5):.0%}, east {covered(10.6, 16):.0%}')
    ok = (joined_at is not None and joined_at <= 10 and lost == 0 and s1['hash'] == s2['hash']
          and q['ab']['agree'] > 0.9 and q['ab']['clash'] < 0.06 and covered(0, 5.5) > 0.3 and covered(10.6, 16) > 0.3)
    print('ALL OK' if ok else 'FAILED')


if __name__ == '__main__':
    main()
