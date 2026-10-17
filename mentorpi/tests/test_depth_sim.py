"""Simulated depth images through depth_obstacles.fit_floor / classify (car 1 camera intrinsics + TF)."""
import sys, math, numpy as np
sys.path.insert(0, '/home/ubuntu/web_ui')
import depth_obstacles as do

fx, fy, cx, cy, W, Hh = 572.8, 572.3, 336.4, 251.0, 640, 480
t_cam = np.array([0.064, 0.012, 0.121])
def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
R_nom = quat_R(-0.5, 0.5, -0.5, 0.5)                    # optical -> base (from the car's TF)
def pitch(deg):                                          # extra rotation about base y (nose down > 0)
    a = math.radians(deg); return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])

objects = {   # name: (height function, expected class)
    'rug 1.5 cm':        'drivable',
    'threshold 3 cm':    'bump',
    'hump 4 cm gentle':  'bump',
    'book 4 cm (edge)':  'obstacle',
    'shoe 10 cm':        'obstacle',
}
def height(x, y):
    z = np.zeros_like(x)
    z = np.where((x > 0.5) & (x < 2.4) & (y > -0.75) & (y < -0.45), 0.015, z)                 # rug, right
    z = np.where((x > 1.00) & (x < 1.04) & (y > 0.30) & (y < 0.70), 0.03, z)                  # threshold, left
    hump = 0.04 * 0.5 * (1 - np.cos(2 * np.pi * np.clip((x - 1.3) / 0.4, 0, 1)))              # hump, centre
    z = np.where((y > -0.12) & (y < 0.12) & (x > 1.3) & (x < 1.7), np.maximum(z, hump), z)
    z = np.where((x > 1.15) & (x < 1.35) & (y > -0.40) & (y < -0.20), 0.04, z)                # book, right
    z = np.where((x > 0.70) & (x < 0.80) & (y > 0.10) & (y < 0.30), 0.10, z)                  # shoe, left
    return z
regions = {'rug 1.5 cm': (-0.75, -0.45), 'threshold 3 cm': (0.30, 0.70), 'hump 4 cm gentle': (-0.12, 0.12),
           'book 4 cm (edge)': (-0.40, -0.20), 'shoe 10 cm': (0.10, 0.30)}

def render(pitch_err_deg, noise=0.003):
    us, vs = np.meshgrid(np.arange(0, W, do.STEP), np.arange(0, Hh, do.STEP))
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], -1).reshape(-1, 3)
    R_true = pitch(pitch_err_deg) @ R_nom
    dirs = rays @ R_true.T
    depth = np.zeros(len(rays))
    hit = np.zeros(len(rays), bool)
    for t in np.arange(0.15, 2.6, 0.004):
        P = t_cam + t * dirs
        newly = ~hit & (P[:, 2] <= height(P[:, 0], P[:, 1]))
        depth[newly] = t; hit |= newly
    rng = np.random.default_rng(1)
    depth = np.where(hit, depth * (1 + rng.normal(0, noise, len(depth))), 0)
    return rays, depth

for err, glossy in ((0.0, False), (1.0, False), (0.0, True)):
    rays, d = render(err)
    if glossy:   # glossy floor: no returns from the floor itself (only objects)
        P = (rays * d[:, None]) @ (pitch(err) @ R_nom).T + t_cam
        d = np.where(height(P[:, 0], P[:, 1]) > 0.001, d, 0)
        print('\n(glossy floor: the camera sees no floor)', end='')
    ok = (d > do.MIN_RANGE) & (d < do.MAX_RANGE)
    p = (rays[ok] * d[ok, None]) @ R_nom.T + t_cam        # the module only knows the nominal mounting
    fl = do.fit_floor(p)
    if fl is not None:
        p[:, 2] -= fl[0] + fl[1] * p[:, 0] + fl[2] * p[:, 1]
    half = math.atan2(max(cx, W - cx), fx) + math.radians(2); n = int(2 * half / do.BIN) + 1
    obst, bump, no, nb = do.classify(p, -half, n)
    ang = -half + np.arange(n) * do.BIN
    print(f'\n=== camera tilted {err:.0f} deg off its nominal mounting; floor fit:',
          None if fl is None else f'offset {fl[0]*100:.1f} cm, pitch {math.degrees(math.atan(fl[1])):.2f} deg')
    boxes = {'rug 1.5 cm': (0.5, 2.4, -0.75, -0.45), 'threshold 3 cm': (1.0, 1.04, 0.30, 0.70),
             'hump 4 cm gentle': (1.3, 1.7, -0.12, 0.12), 'book 4 cm (edge)': (1.15, 1.35, -0.40, -0.20),
             'shoe 10 cm': (0.70, 0.80, 0.10, 0.30)}
    def where(rr):
        out = {}
        for a, r in zip(ang, rr):
            if not np.isfinite(r): continue
            x, y = (r + do.CELL / 2) * math.cos(a), (r + do.CELL / 2) * math.sin(a)   # cell centre
            name = next((k for k, (x0, x1, y0, y1) in boxes.items()
                         if x0 - 0.06 <= x <= x1 + 0.06 and y0 - 0.04 <= y <= y1 + 0.04), 'open floor')
            out[name] = out.get(name, 0) + 1
        return out
    wo, wb = where(obst), where(bump)
    for name in list(boxes) + ['open floor']:
        o, b = wo.get(name, 0), wb.get(name, 0)
        got = 'obstacle' if o else 'bump' if b else 'drivable'
        exp = objects.get(name, 'drivable')
        print(f'  {name:18s} expected {exp:9s} got {got:9s} (obstacle dirs {o:3d}, bump dirs {b:3d})  {"OK" if got == exp else "WRONG"}')

import time as _t
rays, d = render(0.0)
ok = (d > do.MIN_RANGE) & (d < do.MAX_RANGE)
t0 = _t.perf_counter()
for _ in range(20):
    p = (rays[ok] * d[ok, None]) @ R_nom.T + t_cam
    fl = do.fit_floor(p)
    if fl is not None: p[:, 2] -= fl[0] + fl[1] * p[:, 0] + fl[2] * p[:, 1]
    do.classify(p, -half, n)
print(f'\nprocessing time per image: {(_t.perf_counter() - t0) / 20 * 1000:.1f} ms')
