"""The shared map without driving (tests/run.sh test_shared.py): two simulated cars drive through a saved
map, each in its own SLAM frame (rotated + shifted), take lidar scans (ray cast on the map, with noise),
and exchange keyframes through a fake link between their SharedMap objects. Checks: both cars hold the
same map (same hash), it lines up with the real room, the order keyframes arrive in doesn't matter, a
new transform from the leader rebuilds both maps the same way, and the follower gets the map in its own
frame."""
import json, math, sys, time, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import map_merge as mm, map_store
import shared_map as sm

rng = np.random.default_rng(3)
NAME = 'livingroom' if 'livingroom' in map_store.names() else map_store.names()[0]
m = map_store.meta(NAME)
truth = mm.from_png(map_store.png_bytes(NAME), m['resolution'], m['origin'])
N_RAYS = 450


def scan(x, y, yaw, max_r=8.0):
    """Simulated lidar at a pose in the true map: ranges (inf = no return), angle_min = -pi."""
    a = -math.pi + np.arange(N_RAYS) * (2 * math.pi / N_RAYS)
    t = np.arange(0.05, max_r, truth['res'] / 2)
    g = yaw + a
    xy = np.stack([x + np.cos(g)[:, None] * t, y + np.sin(g)[:, None] * t], -1).reshape(-1, 2)
    occ = (mm.lookup(truth, xy) == mm.OCC).reshape(N_RAYS, len(t))
    first = np.where(occ.any(axis=1), occ.argmax(axis=1), -1)
    r = np.where(first >= 0, t[np.maximum(first, 0)], np.inf)
    r = r + rng.normal(0, 0.01, N_RAYS)
    r[rng.random(N_RAYS) < 0.05] = np.inf
    return r, -math.pi, 2 * math.pi / N_RAYS


def path(start, n, step=0.22):
    """A random walk on free floor (true frame), at least 25 cm from walls."""
    free = mm.cells_xy(truth, truth['data'] == mm.FREE)
    dist = __import__('cv2').distanceTransform((truth['data'] != mm.OCC).astype(np.uint8), 2, 3) * truth['res']
    ok = lambda p: mm.lookup(truth, np.array([p[:2]]))[0] == mm.FREE and \
        dist[int((p[1] - truth['origin'][1]) / truth['res']), int((p[0] - truth['origin'][0]) / truth['res'])] > 0.25
    p, out = list(start), []
    for _ in range(n):
        for _ in range(50):
            yaw = p[2] + rng.normal(0, 0.5)
            q = [p[0] + step * math.cos(yaw), p[1] + step * math.sin(yaw), yaw]
            if ok(q):
                p = q
                break
        else:
            p = [p[0], p[1], p[2] + math.pi / 2]
        out.append(list(p))
    return out


# each car's SLAM frame -> true frame
F = {'jpi1': (0.4, -0.3, math.radians(20)), 'jpi2': (-1.0, 0.8, math.radians(-125))}


class FakeTeam:
    def __init__(self, me): self.me, self.T = me, {}
    def matched(self):
        return [types.SimpleNamespace(car=c, T=T) for c, T in self.T.items()]


class Link:
    """Fake peers: GET on the other car's SharedMap (through JSON, as over HTTP)."""
    def __init__(self, me): self.me, self.cars = me, {}
    def summary(self):
        return [{'car': c, 'online': True, 'age_s': 0.1,
                 'status': {'slam': True, 'shared': {'hash': s.out[1] if s.out else None}}}
                for c, s in self.cars.items()]
    def get(self, car, path, timeout=0, **q):
        s = self.cars[car]
        d = s.frames_json(int(q.get('since', 0))) if path == '/api/shared/frames' else s.state_json()
        return json.dumps(d).encode()


def make(me):
    log = types.SimpleNamespace(info=print, warn=print, debug=lambda *a: None)
    node = types.SimpleNamespace(get_logger=lambda: log)
    slam = types.SimpleNamespace(pose=None, running=lambda: True, started_at=1.0,
                                 tf=types.SimpleNamespace(lookup_transform=lambda *a: (_ for _ in ()).throw(Exception())))
    nav = types.SimpleNamespace(scan=None)
    team, link = FakeTeam(me), Link(me)
    s = sm.SharedMap(node, slam, nav, team, link, me, start=False)
    s.session = 1.0
    return s


def put(s, car, p_true):
    """Car at a true pose: its SLAM pose (own frame) and scan."""
    r, amin, inc = scan(*p_true)
    s.slam.pose = list(mm.compose(mm.invert(F[car]), p_true))
    s.nav.scan = (time.monotonic(), types.SimpleNamespace(ranges=r, angle_min=amin, angle_increment=inc,
                                                         header=types.SimpleNamespace(frame_id='laser')))


def main():
    A, B = make('jpi1'), make('jpi2')
    A.peers.cars, B.peers.cars = {'jpi2': B}, {'jpi1': A}
    free = mm.cells_xy(truth, truth['data'] == mm.FREE)
    pa = path([*free[len(free) // 3], 0.0], 30)
    pb = path([*free[2 * len(free) // 3], 2.0], 30)
    # each car's estimate of the other's frame (team.py match), with different small errors
    T_ab = mm.compose(mm.invert(F['jpi1']), F['jpi2'])           # jpi2 frame -> jpi1 frame (true)
    noisy = lambda T, e: (T[0] + e[0], T[1] + e[1], T[2] + e[2])
    print(f'map {NAME}; jpi1 drives {len(pa)} steps, jpi2 {len(pb)}')

    t_build = []
    for i, (qa, qb) in enumerate(zip(pa, pb)):
        if i == 8:   # maps lined up after a few steps (as team.py would)
            A.team.T = {'jpi2': noisy(T_ab, (0.01, -0.01, 0.003))}
            B.team.T = {'jpi1': noisy(mm.invert(T_ab), (-0.02, 0.01, -0.004))}
        put(A, 'jpi1', qa); put(B, 'jpi2', qb)
        A.capture(); B.capture()
        A.fetch(); B.fetch()
        t0 = time.time(); A.update(); B.update(); t_build.append(time.time() - t0)
        if i == 8:
            B.fetch(); B.update()   # the follower reads the leader's transforms after the leader's update
    A.fetch(); B.fetch(); A.update(); B.update()
    sa, sb = A.status(), B.status()
    print(f'1. keyframes: jpi1 {sa["frames"]}, jpi2 {sb["frames"]}; leader {sa["leader"]}/{sb["leader"]}; '
          f'joined {sa["joined"]}/{sb["joined"]}; hash {sa["hash"]} / {sb["hash"]}')
    assert sa['joined'] and sb['joined'] and sa['leader'] == sb['leader'] == 'jpi1'
    assert sa['hash'] == sb['hash'], 'both cars hold the same map'
    ga, _ = A.shared_grid()
    gb, _ = B.shared_grid()
    assert (ga['data'] == gb['data']).all()

    # 2. the shared map (jpi1's frame) against the real room
    q = mm.quality(truth, ga, F['jpi1'])
    walls_ok = min(q['ab']['agree'], q['ba']['agree'])
    print(f'2. shared map vs the real room: walls agree {q["ab"]["agree"]:.2f} / {q["ba"]["agree"]:.2f}, '
          f'clash {q["ab"]["clash"]:.2f}; {np.count_nonzero(ga["data"] != mm.UNK) * 0.0025:.1f} m2 mapped')
    assert q['ab']['agree'] > 0.8 and q['ab']['clash'] < 0.1
    # both cars' parts are in it
    for car, pts in (('jpi1', pa), ('jpi2', pb)):
        xy = mm.transform(np.array([p[:2] for p in pts]), mm.compose(mm.invert(F['jpi1']), (0, 0, 0)))
        on = np.mean(mm.lookup(ga, xy) == mm.FREE)
        print(f'   {car}\'s path on free floor of the shared map: {on:.0%}')
        assert on > 0.9

    # 3. order doesn't matter
    g = sm.Grid()
    T = A.T_used
    allf = [(c, f) for c, fs in A.frames.items() for f in fs.values()]
    for c, f in reversed(allf):
        g.add(f, T[c])
    print(f'3. keyframes added in reverse order: same map {sm.grid_hash(g.to_grid()) == sa["hash"]}')
    assert sm.grid_hash(g.to_grid()) == sa['hash']

    # 4. the leader's estimate moves: both rebuild, still the same
    A.team.T = {'jpi2': noisy(T_ab, (0.06, 0.0, 0.0))}
    A.update(); B.fetch(); B.update()
    sa, sb = A.status(), B.status()
    print(f'4. new transform from the leader: hash {sa["hash"]} / {sb["hash"]}')
    assert sa['hash'] == sb['hash']
    small = noisy(T_ab, (0.065, 0.0, 0.0))   # below T_MOVE_M: no rebuild
    v = A.version; A.team.T = {'jpi2': small}; A.update()
    print(f'   a 5 mm change is not published (no rebuild): {A.version == v}')
    assert A.version == v

    # 5. the follower's map in its own frame lines up with the room through its own frame
    gb_own = B.grid_in_my_frame()
    q2 = mm.quality(truth, gb_own, F['jpi2'])
    print(f'5. jpi2\'s copy in its own frame vs the room: walls agree {q2["ab"]["agree"]:.2f}')
    assert q2['ab']['agree'] > 0.8

    # 6. cost
    t0 = time.time(); g = sm.Grid()
    for c, f in allf:
        g.add(f, T[c])
    full = time.time() - t0
    print(f'6. {len(allf)} keyframes: full rebuild {full:.2f} s ({full / len(allf) * 1000:.0f} ms each); '
          f'each update step up to {max(t_build) * 1000:.0f} ms')
    print('ALL OK')


if __name__ == '__main__':
    main()
