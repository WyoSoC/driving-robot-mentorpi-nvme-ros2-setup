"""Mapping together without driving (tests/run.sh test_team.py): one real saved map is split into two
overlapping halves, as two cars would have mapped them; the second half is put into its own frame
(rotated + shifted, like that car's SLAM). Checks: the halves are lined up again, the explorer drops the
edge the other car has already mapped, claimed frontiers are left to the other car, and fleet.py sees the
other car in this car's frame."""
import json, math, sys, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import map_merge as mm, map_store
import explore_manager as em
import team as tm


def main():
    name = 'explore_test2' if 'explore_test2' in map_store.names() else map_store.names()[0]
    m = map_store.meta(name)
    truth = mm.from_png(map_store.png_bytes(name), m['resolution'], m['origin'])
    res = truth['res']
    known = np.argwhere(truth['data'] != mm.UNK)
    xmin = truth['origin'][0] + known[:, 1].min() * res; xmax = truth['origin'][0] + known[:, 1].max() * res
    mid = (xmin + xmax) / 2
    print(f'map {name}: x {xmin:.2f}..{xmax:.2f}, split at {mid:.2f} with 1 m overlap')

    def crop(g, keep):
        h, w = g['data'].shape
        r, c = np.mgrid[0:h, 0:w]
        x = g['origin'][0] + (c + 0.5) * res
        d = g['data'].copy(); d[~keep(x)] = mm.UNK
        return dict(g, data=d)
    A = crop(truth, lambda x: x < mid + 0.5)                      # car A's map: left part, truth frame
    B_truth = crop(truth, lambda x: x > mid - 0.5)               # car B's part, still truth frame
    TB = (1.7, -0.8, math.radians(35))                           # B's own frame -> truth (= A's frame)
    # rasterize B in its own frame
    inv = mm.invert(TB)
    pts = mm.transform(mm.cells_xy(B_truth, B_truth['data'] != mm.UNK), inv)
    lo, hi = pts.min(axis=0) - 0.5, pts.max(axis=0) + 0.5
    w, h = np.ceil((hi - lo) / res).astype(int)
    B = {'data': np.full((h, w), mm.UNK, np.uint8), 'res': res, 'origin': [float(lo[0]), float(lo[1])]}
    xy = mm.cells_xy(B, np.ones((h, w), bool))
    B['data'] = mm.lookup(B_truth, mm.transform(xy, TB)).reshape(h, w)

    # 1. lining up
    T, q = mm.align(A, B)
    err = math.hypot(T[0] - TB[0], T[1] - TB[1]); erry = math.degrees(abs(T[2] - TB[2]))
    print(f'1. lined up: error {err*100:.1f} cm, {erry:.1f} deg, match={mm.good_match(q)}')
    assert mm.good_match(q) and err < 0.10 and erry < 2

    # 2. frontiers on A, alone and with B's map
    def to_int8(g):
        out = np.full(g['data'].shape, -1, np.int8); out[g['data'] == mm.FREE] = 0; out[g['data'] == mm.OCC] = 100
        return out
    gridA = to_int8(A)
    freeA = mm.cells_xy(A, A['data'] == mm.FREE)
    car = freeA[np.argmin(np.abs(freeA[:, 0] - (mid - 1.0)) + np.abs(freeA[:, 1] - np.median(freeA[:, 1])))]
    alone, _ = em.frontier_goals(gridA, res, A['origin'], car, car, 0, [], car_yaw=0.0)
    team = types.SimpleNamespace()
    mate = tm.Mate('jpi2'); mate.grid, mate.T = B, T
    t = tm.Team.__new__(tm.Team); t.mates = {'jpi2': mate}; t.lock = __import__('threading').Lock()
    cov = t.covered(gridA.shape, res, A['origin'])
    together, _ = em.frontier_goals(gridA, res, A['origin'], car, car, 0, [], car_yaw=0.0, covered=cov)
    cut = lambda fs: [f for f in fs if abs(f['center'][0] - (mid + 0.5)) < 0.4]
    print(f'2. frontiers on the cut edge: alone {len(cut(alone))}, with the other car\'s map {len(cut(together))};'
          f' all frontiers: {len(alone)} -> {len(together)}')
    assert len(cut(alone)) >= 1 and len(cut(together)) == 0

    # 3. claims: a frontier near the other car's goal is left to it
    if alone:
        f0 = alone[0]
        claimed, _ = em.frontier_goals(gridA, res, A['origin'], car, car, 0, [], car_yaw=0.0,
                                       claims=[{'car': 'jpi2', 'pose': [0, 0], 'goal': f0['center']}])
        left = any(math.hypot(f['center'][0] - f0['center'][0], f['center'][1] - f0['center'][1]) < 0.1 for f in claimed)
        print(f'3. best frontier claimed by the other car -> skipped: {not left}')
        assert not left

    # 4. Team state machine: a match is used only after two checks agree; my_grid_json round trip
    class FakePeers:
        def __init__(s, payload): s.payload = payload
        def summary(s): return [{'car': 'jpi2', 'online': True, 'age_s': 0.2, 'status': {'slam': True,
            'nav': {'running': True, 'slam_mode': True, 'pose_set': True, 'pose': [0.5, 0.2, 0.0],
                    'goal_state': 'active', 'goal': [1.0, 0.0, 0.0], 'plan': [[0.5, 0.2], [0.8, 0.2]]}}}]
        def get(s, car, path, timeout=0): return json.dumps(s.payload).encode()
    log = types.SimpleNamespace(info=print, warn=print)
    node = types.SimpleNamespace(get_logger=lambda: log)
    payload = tm.encode_grid(B); payload['pose'] = [0.5, 0.2, 0.0]
    def msg_of(g):
        info = types.SimpleNamespace(height=g['data'].shape[0], width=g['data'].shape[1], resolution=g['res'],
                                     origin=types.SimpleNamespace(position=types.SimpleNamespace(x=g['origin'][0], y=g['origin'][1])))
        return types.SimpleNamespace(info=info, data=to_int8(g).ravel().tolist())
    slam = types.SimpleNamespace(grid=msg_of(A), pose=[0, 0, 0], running=lambda: True)
    team = tm.Team.__new__(tm.Team)
    team.node, team.slam, team.peers, team.me, team.log = node, slam, FakePeers(payload), 'jpi1', log
    team.mates, team.lock, team._grid_cache, team.pool = {}, __import__('threading').Lock(), (None, None), None
    team.check(); s1 = team.status()[0]
    team.check(); s2 = team.status()[0]
    print(f'4. first check: matched={s1["matched"]} ({s1["note"]}); second: matched={s2["matched"]} ({s2["note"]})')
    assert not s1['matched'] and s2['matched']
    g = tm.decode_grid(team.my_grid_json())
    assert (g['data'] == A['data']).all(), 'live map round trip'
    # 5. fleet.py in exploration: the other car in this car's frame
    import fleet
    nav = types.SimpleNamespace(status=lambda: {'running': True, 'slam_mode': True, 'map': None, 'pose_set': True, 'pose': [0, 0, 0]})
    f = fleet.Fleet.__new__(fleet.Fleet); f.nav, f.peers, f.team = nav, team.peers, team
    o = f.others()[0]
    want = mm.transform(np.array([[0.5, 0.2]]), team.to_mine('jpi2'))[0]
    print(f'5. other car in this frame: ({o["x"]:.2f}, {o["y"]:.2f}), expected ({want[0]:.2f}, {want[1]:.2f}); plan points {len(o["plan"])}')
    assert abs(o['x'] - want[0]) < 1e-6 and len(o['plan']) == 2
    print('ALL OK')


if __name__ == '__main__':   # team.py starts a worker process that imports this file
    main()
