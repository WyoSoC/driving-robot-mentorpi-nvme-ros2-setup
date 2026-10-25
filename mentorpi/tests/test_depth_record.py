"""Depth frames in run recordings and the offline replay (tests/run.sh test_depth_record.py): fit_floor says
why it found no floor (glossy floor, camera tilted); the recorder keeps one depth image a second with the
camera's intrinsics and mounting; depth_replay.py reads them back and gets the same floor fit as the car.
Synthetic depth images (car 1's camera), nothing moves."""
import io
import json
import math
import os
import shutil
import sys
import tempfile
import types
from contextlib import redirect_stdout
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import depth_obstacles as do
import run_recorder as rr
import depth_replay

fx, fy, cx, cy, W, H = 572.8, 572.3, 336.4, 251.0, 640, 480
t_cam = np.array([0.080, 0.0, 0.110])
def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
R_nom = quat_R(-0.5, 0.5, -0.5, 0.5)                    # optical -> base_footprint (the car's TF)
def pitch(deg):
    a = math.radians(deg); return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def depth_image(tilt_deg=0.0, glossy=False):
    """16-bit mm depth image of a flat floor (camera tilted tilt_deg nose down from its nominal mounting)."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)
    dirs = rays @ (pitch(tilt_deg) @ R_nom).T
    t = np.where(dirs[..., 2] < -1e-6, -t_cam[2] / np.minimum(dirs[..., 2], -1e-6), 0)   # ray hits the floor
    d = np.where((t > 0) & (t < 3.0), t, 0)           # optical z of rays is 1: depth = t
    if glossy:
        d[:] = 0
    return (d * 1000).astype(np.uint16)


def points(img):
    k = do.STEP
    sub = img[::k, ::k].astype(np.float32) * 0.001
    u, v = np.meshgrid(np.arange(0, W, k), np.arange(0, H, k))
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    d = sub.reshape(-1); ok = (d > do.MIN_RANGE) & (d < do.MAX_RANGE)
    return (rays[ok] * d[ok, None]) @ R_nom.T + t_cam


# 1. fit_floor and its reasons
for name, img, want in (('flat floor', depth_image(), 'ok'),
                        ('glossy floor (no returns)', depth_image(glossy=True), 'no floor near the expected height'),
                        ('camera 2.5 deg lower than its mounting says', depth_image(2.5),
                         'floor found only in a narrow band of distances (camera tilted differently from its mounting?)'),
                        ('camera 5 deg lower, wider band', depth_image(5.0), None)):
    why = {}
    fl = do.fit_floor(points(img), why)
    got = 'ok' if fl is not None else why['stage']
    print(f'{name:45s} -> {got} {why if why else ""}')
    assert want is None or got == want, (name, got, why)

# 2. the recorder keeps depth frames (1 a second) with the camera's intrinsics and mounting
tmp = tempfile.mkdtemp()
d = os.path.join(tmp, '20260101_000000_test')
os.makedirs(os.path.join(d, 'depth'))
rec = rr.RunRecorder.__new__(rr.RunRecorder)
rec.node = types.SimpleNamespace(depth_obs=types.SimpleNamespace(K=(fx, fy, cx, cy), R=R_nom, t=t_cam))
rec.run = {'dir': d, 't0': 0.0, 'depth_at': -10.0, 'depth': 0}
img = depth_image()
msg = types.SimpleNamespace(width=W, height=H, step=W * 2, encoding='16UC1', data=img.tobytes())
t_now = [100.0]
rr.time.monotonic = lambda: t_now[0]
for i in range(5):                         # 5 frames 0.4 s apart: 2 kept (1 a second)
    why = {}; fl = do.fit_floor(points(img), why)
    rec._on_depth(msg, fl, why)
    t_now[0] += 0.4
files = sorted(os.listdir(os.path.join(d, 'depth')))
print('recorded:', files)
assert rec.run['depth'] == 2 and 'camera.json' in files and 'index.jsonl' in files, files
cam = json.load(open(os.path.join(d, 'depth', 'camera.json')))
assert cam['width'] == 320 and abs(cam['fx'] - fx / 2) < 1e-6
idx = [json.loads(l) for l in open(os.path.join(d, 'depth', 'index.jsonl'))]
assert all(e['floor'] is not None for e in idx), idx

# 3. replay: same floor fit offline
dd, cam, index = depth_replay.load_run(d)
r = depth_replay.analyse(dd, cam, index[0]['file'])
print('replay floor:', np.round(r['floor'], 4), 'obstacle dirs', int(np.isfinite(r['obst']).sum()))
assert r['floor'] is not None and abs(r['floor'][0]) < 0.005 and int(np.isfinite(r['obst']).sum()) == 0
out = io.StringIO()
sys.argv = ['depth_replay.py', d]
with redirect_stdout(out):
    depth_replay.main()
print(out.getvalue().strip())
assert '100.0%  ok' in out.getvalue()
# a setting changed for the replay (here a wider floor band): the 2.5 deg tilted camera's floor is found
sys.argv = ['depth_replay.py', d, '--set', 'FLOOR_BAND=0.08']
with redirect_stdout(io.StringIO()):
    depth_replay.main()
assert abs(do.FLOOR_BAND - 0.08) < 1e-9
fl = do.fit_floor(points(depth_image(2.5)))
print('2.5 deg tilt with an 8 cm band:', None if fl is None else f'pitch {math.degrees(math.atan(fl[1])):.2f} deg')
assert fl is not None and abs(abs(math.degrees(math.atan(fl[1]))) - 2.5) < 0.3
shutil.rmtree(tmp)
print('OK: depth recording and replay')
