"""Replay a run's depth images (run_recorder.py: runs/<id>/depth/) through depth_obstacles.py, offline.

Answers, on real floors: does the floor fit work (and why not), what does the camera call an obstacle, a bump
or a drop, and what changes with other settings. Nothing moves; no ROS needed. In the container:

  python3 depth_replay.py <run id or folder>                  summary over the run
  python3 depth_replay.py <run> --at 213                      the frame nearest t = 213 s: floor fit and the
                                                              height profile straight ahead
  python3 depth_replay.py <run> --set FLOOR_MAX_TILT=0.1 --set BUMP_H=0.06    try other settings
  python3 depth_replay.py <run> --frames                      one line per frame

Settings are depth_obstacles.py's constants (angles in rad, lengths in m).
"""
import argparse
import collections
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import depth_obstacles as do

RUNS_DIR = '/home/ubuntu/shared/runs'


def load_run(arg):
    d = arg if os.path.isdir(arg) else os.path.join(RUNS_DIR, arg)
    dd = os.path.join(d, 'depth')
    with open(os.path.join(dd, 'camera.json')) as f:
        cam = json.load(f)
    index = []
    with open(os.path.join(dd, 'index.jsonl')) as f:
        for line in f:
            try:
                index.append(json.loads(line))
            except ValueError:
                pass
    return dd, cam, index


def points(dd, cam, name):
    """Depth image -> points (N x 3, base_footprint) at the module's pixel step, like on the car."""
    img = cv2.imread(os.path.join(dd, name), cv2.IMREAD_UNCHANGED)
    k = max(1, do.STEP // cam['step'])
    img = img[::k, ::k]
    h, w = img.shape
    u, v = np.meshgrid(np.arange(w) * k, np.arange(h) * k)
    rays = np.stack([(u - cam['cx']) / cam['fx'], (v - cam['cy']) / cam['fy'], np.ones_like(u, float)], -1).reshape(-1, 3)
    d = img.reshape(-1).astype(np.float32) * cam['depth_unit_m']
    ok = (d > do.MIN_RANGE) & (d < do.MAX_RANGE)
    R, t = np.array(cam['R_optical_to_base']), np.array(cam['t_camera_in_base_m'])
    return (rays[ok] * d[ok, None]) @ R.T + t, float(ok.mean())


def analyse(dd, cam, name):
    p, valid = points(dd, cam, name)
    why = {}
    fl = do.fit_floor(p, why)
    q = p.copy()
    if fl is not None:
        q[:, 2] -= fl[0] + fl[1] * q[:, 0] + fl[2] * q[:, 1]
    half = math.atan2(max(cam['cx'], cam['width'] - cam['cx']), cam['fx']) + math.radians(2)
    n = int(2 * half / do.BIN) + 1
    obst, bump, n_obst, n_bump = do.classify(q, -half, n)
    edge, low = do.find_drops(q, (float(cam['t_camera_in_base_m'][0]), float(cam['t_camera_in_base_m'][2]))) \
        if fl is not None else (np.full(1, np.inf), None)
    return {'p': q, 'valid': valid, 'floor': fl, 'why': why, 'obst': obst, 'bump': bump, 'angles': -half + np.arange(n) * do.BIN,
            'drop_dirs': int(np.isfinite(edge).sum())}


def profile(q):
    ahead = (np.abs(q[:, 1]) < 0.08) & (q[:, 0] > 0)
    rows = []
    for x0 in np.arange(0.2, 2.0, 0.05):
        m = ahead & (q[:, 0] >= x0) & (q[:, 0] < x0 + 0.05)
        if m.sum() >= 2:
            rows.append((round(float(x0), 2), round(float(np.median(q[m, 2])) * 100, 1), round(float(q[m, 2].max()) * 100, 1), int(m.sum())))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--at', type=float, help='look at the frame nearest this time (s)')
    ap.add_argument('--frames', action='store_true', help='one line per frame')
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='override a depth_obstacles setting')
    a = ap.parse_args()
    for kv in a.set:
        k, v = kv.split('=', 1)
        if not hasattr(do, k):
            sys.exit(f'no setting {k} in depth_obstacles.py')
        setattr(do, k, type(getattr(do, k))(float(v)))
        print(f'setting {k} = {getattr(do, k)}')
    dd, cam, index = load_run(a.run)
    if not index:
        sys.exit('no depth frames in this run')
    if a.at is not None:
        e = min(index, key=lambda e: abs(e['t'] - a.at))
        r = analyse(dd, cam, e['file'])
        print(f"frame t = {e['t']} s ({e['file']}), valid depth {r['valid'] * 100:.0f}% of pixels")
        print('floor fit:', 'plane offset %.1f cm, pitch %.2f deg, roll %.2f deg' % (
            r['floor'][0] * 100, math.degrees(math.atan(r['floor'][1])), math.degrees(math.atan(r['floor'][2])))
            if r['floor'] is not None else f"none: {r['why']}")
        print('straight ahead (|y| < 8 cm), x m: median / max height cm (points)')
        for x, med, mx, cnt in profile(r['p']):
            print(f'  {x:4.2f}: {med:6.1f} {mx:6.1f}  ({cnt})')
        mid = len(r['obst']) // 2
        print('nearest obstacle, central directions (m):', np.round(r['obst'][mid - 5:mid + 6], 2).tolist())
        print('nearest bump, central directions (m):    ', np.round(r['bump'][mid - 5:mid + 6], 2).tolist())
        print('drop directions:', r['drop_dirs'])
        return
    stages = collections.Counter()
    fits, n_obst_dirs, n_drop_frames = [], [], 0
    for e in index:
        r = analyse(dd, cam, e['file'])
        stages[r['why'].get('stage', 'ok')] += 1
        if r['floor'] is not None:
            fits.append(r['floor'])
        no = int(np.isfinite(r['obst']).sum())
        n_obst_dirs.append(no)
        n_drop_frames += r['drop_dirs'] > 0
        if a.frames:
            fl = 'floor ok' if r['floor'] is not None else f"floor: {r['why'].get('stage')}"
            print(f"{e['t']:7.1f}s  {fl:52s} obstacle dirs {no:3d}  bump dirs {int(np.isfinite(r['bump']).sum()):3d}  drop dirs {r['drop_dirs']}")
    n = len(index)
    print(f'{n} depth frames, {index[0]["t"]:.0f}-{index[-1]["t"]:.0f} s')
    print('floor fit:')
    for k, c in stages.most_common():
        print(f'  {c * 100 / n:5.1f}%  {k}')
    if fits:
        f = np.array(fits)
        print(f'  fitted planes: offset {np.median(f[:, 0]) * 100:.1f} cm, pitch {math.degrees(math.atan(np.median(f[:, 1]))):.2f} deg, '
              f'roll {math.degrees(math.atan(np.median(f[:, 2]))):.2f} deg (medians)')
    print(f'obstacle directions per frame: median {int(np.median(n_obst_dirs))}, max {max(n_obst_dirs)}')
    print(f'frames with drops: {n_drop_frames}')


if __name__ == '__main__':
    main()
