"""Stairs / ledges with the depth camera (tests/run.sh test_drops.py), depth_obstacles.find_drops on simulated
points: rays from the measured camera position (car_geometry.py) with car 1's intrinsics, every 4th pixel,
1 cm noise, onto a floor that ends at an edge with a lower floor further down. The edge and the lower floor
are found where they are; a flat floor, a 3 cm step down (threshold) and scattered points below the floor
(mirror images in a glossy floor) are not drops; two of three frames confirm; a near edge stops the car."""
import math, sys, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import car_geometry as g
from builtin_interfaces.msg import Time
import depth_obstacles as do

fx, fy, cx, cy, W, H = 572.8, 572.3, 336.4, 251.0, 640, 480
CAM = np.array([g.CAMERA_X, 0.012, g.CAMERA_Z])
rng = np.random.default_rng(6)
u, v = np.meshgrid(np.arange(0, W, 4), np.arange(0, H, 4))
dirs = np.stack([np.ones(u.size), -(u.ravel() - cx) / fx, -(v.ravel() - cy) / fy], -1)   # base frame: x fwd, y left, z up
dirs /= np.linalg.norm(dirs, axis=1)[:, None]


def scene(edge_x=None, drop=0.17, edge_angle_deg=0.0, max_r=2.5):
    """Points the camera sees: floor z=0; past a straight edge (x' > edge_x, rotated by edge_angle) the floor
    is `drop` lower. Rays that see nothing (no hit within max_r) are dropped."""
    pts = []
    ca, sa = math.cos(math.radians(edge_angle_deg)), math.sin(math.radians(edge_angle_deg))
    for d in dirs:
        if d[2] >= -1e-3:
            continue                                      # looks up: walls far away, not used here
        t0 = -CAM[2] / d[2]
        p = CAM + t0 * d
        if edge_x is not None and ca * p[0] + sa * p[1] > edge_x:   # passes over the edge: lower floor
            t1 = -(CAM[2] + drop) / d[2]
            p = CAM + t1 * d
        if math.hypot(p[0], p[1]) < max_r:
            pts.append(p + rng.normal(0, 0.01 * max(0.3, math.hypot(p[0], p[1])), 3) * [1, 1, 0.5])
    return np.array(pts)


def path_bins(edge, low):
    ang = -math.pi / 2 + (np.arange(len(edge)) + 0.5) * math.radians(do.DROP_BIN_DEG)
    return np.abs(np.degrees(ang)) < 15


p = scene(edge_x=0.8)
edge, low = do.find_drops(p, (CAM[0], CAM[2]))
sel = path_bins(edge, low) & np.isfinite(edge)
print(f'1. edge 0.80 m ahead, 17 cm down: {sel.sum()} directions ahead see a drop; edge {np.median(edge[sel]):.2f} m '
      f'(true 0.80), lower floor from {np.median(low[sel]):.2f} m')
assert sel.sum() >= 5 and abs(np.median(edge[sel]) - 0.8) < 0.12 and np.median(low[sel]) > 0.9
edge, low = do.find_drops(scene(edge_x=None), (CAM[0], CAM[2]))
print(f'2. flat floor: {int(np.isfinite(edge).sum())} directions with a drop')
assert not np.isfinite(edge).any()
edge, low = do.find_drops(scene(edge_x=0.8, drop=0.03), (CAM[0], CAM[2]))
print(f'3. 3 cm step down (threshold): {int(np.isfinite(edge).sum())} directions with a drop')
assert not np.isfinite(edge).any()
noise = scene(edge_x=None)
fake = np.c_[rng.uniform(0.4, 1.4, 25), rng.uniform(-0.4, 0.4, 25), rng.uniform(-0.6, -0.08, 25)]
edge, low = do.find_drops(np.vstack([noise, fake]), (CAM[0], CAM[2]))
print(f'4. flat floor + 25 scattered points below it (mirror images): {int(np.isfinite(edge).sum())} directions with a drop')
assert not np.isfinite(edge).any()
p = scene(edge_x=0.6, edge_angle_deg=35)
edge, low = do.find_drops(p, (CAM[0], CAM[2]))
print(f'5. edge at 35 deg to the car: {int(np.isfinite(edge).sum())} directions see it')
assert np.isfinite(edge).sum() >= 5

# 6. per frame: 2 of 3 confirm, marks, stop when right ahead
class Haz:
    def __init__(self): self.pts = []
    def add_marks_base(self, pts): self.pts += pts
d = do.DepthObstacles.__new__(do.DepthObstacles)
d.drop_hist, d.drop_dirs, d.hazards, d.stops = [], 0, Haz(), []
d.on_drop_ahead = lambda note: d.stops.append(note)
d.drop_pub = types.SimpleNamespace(publish=lambda m: None)
e1 = do.find_drops(scene(edge_x=0.8), (CAM[0], CAM[2]))
d._drops(*e1, Time())
print(f'6. one frame: marks {len(d.hazards.pts)} (needs 2 of 3)')
assert not d.hazards.pts
d._drops(*e1, Time())
print(f'   two frames: {d.drop_dirs} directions confirmed, {len(d.hazards.pts)} no-go marks, stop: {d.stops or "no (edge far)"}')
assert d.drop_dirs >= 5 and d.hazards.pts and not d.stops
near = do.find_drops(scene(edge_x=g.FRONT + 0.2), (CAM[0], CAM[2]))
d.drop_hist = []; d._drops(*near, Time()); d._drops(*near, Time())
print(f'7. edge 20 cm past the bumper: stop: {d.stops}')
assert d.stops
print('ALL OK')
