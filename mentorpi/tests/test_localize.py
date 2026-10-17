"""localize.find on the saved maps: simulated lidar scans from random free poses (with noise, missing
points and a car-sized box that is not on the map) must give back the pose, or say "not sure".
A wrong pose reported as sure is a failure. Doesn't drive."""
import glob
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, '/home/ubuntu/web_ui')
import localize
import map_merge

MAPS = '/home/ubuntu/maps'
N_POSES = 12
rng = np.random.default_rng(7)


def raycast(grid, x, y, yaw, n=450, max_r=8.0):
    """Ranges of a simulated 360 deg lidar (angle_min = -pi) at a pose; walls = occupied cells."""
    a = -math.pi + np.arange(n) * (2 * math.pi / n)
    steps = np.arange(0.05, max_r, grid['res'] / 2)
    out = np.full(n, np.inf)
    for i, ang in enumerate(a):
        g = yaw + ang
        xy = np.stack([x + steps * math.cos(g), y + steps * math.sin(g)], -1)
        hit = np.flatnonzero(map_merge.lookup(grid, xy) == map_merge.OCC)
        if len(hit):
            out[i] = steps[hit[0]]
    return a, out


def add_box(a, r, bx, by, half=0.12):
    """A 24 cm box at (bx, by) in the car's frame (another car, not on the map)."""
    for i, ang in enumerate(a):
        for t in np.arange(0.1, r[i] if np.isfinite(r[i]) else 8.0, 0.02):
            px, py = t * math.cos(ang), t * math.sin(ang)
            if abs(px - bx) < half and abs(py - by) < half:
                r[i] = t
                break
    return r


fails = 0
for y_path in sorted(glob.glob(f'{MAPS}/*.yaml')):
    meta = yaml.safe_load(open(y_path))
    png = os.path.join(MAPS, meta['image'])
    grid = map_merge.from_png(open(png, 'rb').read(), meta['resolution'], meta['origin'][:2])
    field = localize._Field(grid, grid['res'])
    free = np.flatnonzero(field.free & (field.dist >= 0.25))
    name = os.path.basename(y_path)[:-5]
    ok = unsure = wrong = 0
    took = []
    for k in range(N_POSES):
        x, y = field.centre(rng.choice(free))
        yaw = rng.uniform(-math.pi, math.pi)
        a, r = raycast(grid, x, y, yaw)
        r = r + rng.normal(0, 0.01, len(r))
        r[rng.random(len(r)) < 0.1] = np.inf
        if k % 2:
            r = add_box(a, r, 0.6, 0.3)
        pts = localize.scan_points(r, a[0], a[1] - a[0])
        res = localize.find(grid, pts)
        took.append(res['took_s'])
        p = res['pose']
        err = math.hypot(p[0] - x, p[1] - y)
        derr = abs(math.degrees((p[2] - yaw + math.pi) % (2 * math.pi) - math.pi))
        right = err < 0.15 and derr < 5
        if res['sure'] and right:
            ok += 1
        elif res['sure']:
            wrong += 1
            print(f'  WRONG {name}: true ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}) got {p} {res}')
        else:
            unsure += 1
            print(f'  not sure {name}: right={right} err={err:.2f} m {derr:.0f} deg; {res["why"]}')
    fails += wrong
    print(f'{name}: {ok} found, {unsure} not sure, {wrong} wrong; {np.mean(took):.1f} s each')
print('ALL OK' if fails == 0 else f'{fails} WRONG')
