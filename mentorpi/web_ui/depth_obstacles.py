"""Obstacles from the depth camera, compared with (and added to) the lidar.

Why: the lidar scans one flat plane 16 cm above the floor. Anything lower (a shoe, a cable bundle, a toy)
or an overhang above it is invisible to it. The depth camera (Angstrong HP60C, 11 cm high, looking straight
ahead, 58 deg wide) sees a 3-D picture of what is in front of the car.

What this does with a depth frame, at most RATE_HZ times a second (the camera sends ~12; 12 Hz of this
plus Nav2's two costmap layers overloaded the Pi 5 while navigating):
1. Every 4th pixel becomes a 3-D point (camera intrinsics), moved into the car frame (base_footprint, TF).
2. Floor: where the camera sees floor (carpet, matte floors; glossy floors return nothing), a plane is
   fitted to it and heights are measured from that plane. This removes small camera tilt / height errors
   that would otherwise make the floor 2 m away look like a 3-4 cm bump. Kept (smoothed) while no floor
   is visible.
3. Threat level (classify()): the points go into a height grid in front of the car (direction x 5 cm of
   distance, highest point per cell). Walking outward along each direction:
     drivable  - up to CLIMB_H above the floor (rugs, carpet, floor noise)
     obstacle  - higher than BUMP_H, or a step of more than STEP_H within one 5 cm cell (a vertical edge
                 the wheels can't climb: shoes, boxes, walls), or anything up to MAX_H (overhangs that
                 would hit the body)
     bump      - in between and rising gently (thresholds, cables, humps): the car can drive over it
   Thresholds grow with distance by NOISE_PER_M (the camera's depth noise).
4. Outputs: /depth/scan = nearest OBSTACLE per 0.5 deg direction (LaserScan in base_footprint, for Nav2
   and the collision monitor: bumps don't block); /depth/bumps = nearest bump (for the page).
   Noise filter: a direction counts only if seen in at least 2 of the last 3 frames (median of 3).

Fusion: Nav2 uses /scan_raw and /depth/scan as two separate costmap layers (each clears only its own
marks; one shared layer would let the lidar, whose plane passes over a shoe, erase the camera's mark of
it), and the collision monitor watches both. See nav/nav2_params.yaml.

Comparison (/depth/compare, JSON String, 2 Hz): inside the camera's view, per direction:
  both     - lidar and camera see an obstacle at about the same distance (within AGREE_M)
  camera   - only the camera sees one (or a closer one): low objects / overhangs the lidar misses
  lidar    - only the lidar sees one: things the camera misses (dark or shiny surfaces, too close,
             outside its height band, thin legs)
plus running totals since start, so the page can show how the two sensors compare.
Drops (find_drops(); stairs, ledges): only while the floor plane was measured in the last DROP_FLOOR_S, in
direction bins of DROP_BIN_DEG within DROP_RANGE: at least DROP_MIN_PTS points more than DROP_H below the
floor (plus the distance noise), their heights within DROP_FLAT_M of each other (a lower floor is flat;
stray mirror images in a glossy floor are not), seen in 2 of the last 3 frames. The edge lies between the
last floor point seen in that direction and where the ray to the lower floor crossed floor height (the camera
can't see the floor right past an edge), so that whole stretch out to the lower floor is marked no-go
(stall_detector's marks -> Nav2's bump_layer, kept for the run). A drop edge within DROP_STOP_M of the
front bumper in the car's path while it drives forward stops it at once and backs it out (explorer.bumped).
The camera's nearest visible floor is ~23 cm past the bumper and it looks forward only: reversing is not
covered (lift_detector.py also stops the car when its nose or tail tips over an edge).
"""
import json
import math
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

import car_geometry

DEPTH_TOPIC = '/ascamera/camera_publisher/depth0/image_raw'
INFO_TOPIC = '/ascamera/camera_publisher/depth0/camera_info'
LIDAR_TOPIC = '/scan_raw'
BASE = 'base_footprint'
STEP = 4                 # use every 4th pixel (160 x 120 points)
MIN_RANGE = 0.15         # m from the camera: closer readings are unreliable
MAX_RANGE = 2.5          # m
MAX_H = 0.22             # m: higher = above the car (lidar at 0.16 m, top of the car ~0.2 m)
# Threat levels (heights above the floor plane). The car's wheels climb small, gentle humps.
CLIMB_H = 0.02           # m: up to this is just floor (rugs, carpet, noise)
BUMP_H = 0.05            # m: higher than this is an obstacle, however gentle
STEP_H = 0.025           # m: a rise of more than this within one cell (5 cm) is a vertical edge: obstacle
NOISE_PER_M = 0.006      # m of extra tolerance per m of distance (depth noise ~0.5 % of range)
CELL = 0.05              # m: height grid cell along each direction
MIN_PTS = 2              # points needed in a cell (single stray points are ignored)
# Floor plane fit
FLOOR_BAND = 0.025       # m: points this close to the expected floor are floor candidates
FLOOR_MIN_PTS = 150
FLOOR_MAX_OFFSET = 0.03  # m, FLOOR_MAX_TILT rad: larger corrections are not believed
FLOOR_MAX_TILT = math.radians(3)
FLOOR_ALPHA = 0.2        # smoothing of the fitted plane
FLOOR_HIST = 50          # frames (10 s) for the floor statistics on the page
FOV_HALF = math.radians(29)           # camera: 58 deg wide
FLOOR_MIN_SECTORS = 7                 # floor points in at least 7 of 10 sectors across the view
DROP_H = -0.06           # m: below the floor by this much = a lower floor (stairs, ledge)
DROP_RANGE = 2.4         # m: lower floor looked for within this (camera 11 cm high: past an edge 0.8 m away
                         # a floor 17 cm lower shows only from ~1.9 m)
DROP_BIN_DEG = 2.0       # direction bins for drops
DROP_MIN_PTS = 6         # points below the floor needed in a bin (every 4th pixel is used)
DROP_FLAT_M = 0.04       # their heights (10..90 %) within this: a flat lower floor, not scattered noise
DROP_FLOOR_S = 2.0       # the floor plane must have been measured this recently
DROP_FLOOR_BAND = 0.02   # m: points this close to the floor count as floor (last floor before the edge)
DROP_STOP_M = 0.30       # m past the front bumper: a drop edge this close in the car's path stops it
MARK_STEP = 0.03
BIN = math.radians(0.5)


def fit_floor(p, why=None):
    """Plane z = a + b x + c y through the floor points (robust: fit, drop outliers, refit), or None.
    why (a dict): filled with the reason when there is no plane, for the page and depth_replay.py."""
    why = why if why is not None else {}
    why.clear()
    r = np.hypot(p[:, 0], p[:, 1])
    near = (r > 0.25) & (r < 1.8)
    sel = (np.abs(p[:, 2]) < FLOOR_BAND) & near
    if sel.sum() < FLOOR_MIN_PTS:
        why.update(stage='no floor near the expected height', candidates=int(sel.sum()), need=FLOOR_MIN_PTS,
                   points_0_25_to_1_8m=int(near.sum()),
                   median_height_cm=round(float(np.median(p[near, 2])) * 100, 1) if near.any() else None)
        return None
    # Floor is flat: keep points from 5 cm patches whose heights vary by < 8 mm. The lower part of a
    # vertical face (shoe, box) is also near floor height, but varies a lot within its patch.
    cx, cy = np.floor(p[:, 0] / 0.05).astype(int), np.floor(p[:, 1] / 0.05).astype(int)
    key = (cx - cx.min()) * 100000 + (cy - cy.min())
    _, inv = np.unique(key[sel], return_inverse=True)
    zmin = np.full(inv.max() + 1, np.inf); zmax = np.full(inv.max() + 1, -np.inf); cnt = np.zeros(inv.max() + 1)
    np.minimum.at(zmin, inv, p[sel, 2]); np.maximum.at(zmax, inv, p[sel, 2]); np.add.at(cnt, inv, 1)
    flat = ((zmax - zmin) < 0.008) & (cnt >= 3)
    idx = np.flatnonzero(sel)
    sel = np.zeros(len(p), bool); sel[idx[flat[inv]]] = True
    if sel.sum() < FLOOR_MIN_PTS:
        why.update(stage='floor too rough (few flat 5 cm patches)', flat_points=int(sel.sum()), need=FLOOR_MIN_PTS)
        return None
    # Real floor fills the view; a rug strip or a table top on one side is not the floor (glossy floors
    # return nothing, and the fit would latch onto the rug).
    ang = np.arctan2(p[sel, 1], p[sel, 0])
    sectors = np.unique(np.clip(((ang + FOV_HALF) / (2 * FOV_HALF) * 10).astype(int), 0, 9))
    if len(sectors) < FLOOR_MIN_SECTORS or np.ptp(r[sel]) < 0.6:
        why.update(stage=('floor seen only in part of the view' if len(sectors) < FLOOR_MIN_SECTORS else
                          'floor found only in a narrow band of distances (camera tilted differently from its mounting?)'),
                   sectors=int(len(sectors)), need=FLOOR_MIN_SECTORS,
                   depth_spread_m=round(float(np.ptp(r[sel])), 2), need_spread_m=0.6,
                   from_m=round(float(r[sel].min()), 2), to_m=round(float(r[sel].max()), 2))
        return None
    q = p[sel]
    for _ in range(2):
        A = np.c_[np.ones(len(q)), q[:, 0], q[:, 1]]
        coef, *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
        res = np.abs(A @ coef - q[:, 2])
        keep = res < 0.008
        if keep.sum() < FLOOR_MIN_PTS:
            why.update(stage='floor points not on one plane', on_plane=int(keep.sum()), need=FLOOR_MIN_PTS)
            return None
        q = q[keep]
    a, b, c = coef
    if abs(a) > FLOOR_MAX_OFFSET or abs(b) > math.tan(FLOOR_MAX_TILT) or abs(c) > math.tan(FLOOR_MAX_TILT):
        why.update(stage='plane too far from the nominal camera mounting', offset_cm=round(float(a) * 100, 1),
                   pitch_deg=round(math.degrees(math.atan(b)), 2), roll_deg=round(math.degrees(math.atan(c)), 2),
                   limits=f'{FLOOR_MAX_OFFSET * 100:.0f} cm, {math.degrees(FLOOR_MAX_TILT):.0f} deg')
        return None
    return np.array([a, b, c])


def find_drops(p, cam_xz, max_range=DROP_RANGE):
    """Points (N x 3, base_footprint, heights above the floor) and the camera's (x, z) -> per direction bin
    (DROP_BIN_DEG, from -90 deg) the edge distance and the lower floor's distance (inf = no drop)."""
    n = int(180 / DROP_BIN_DEG)
    edge, low = np.full(n, np.inf), np.full(n, np.inf)
    r = np.hypot(p[:, 0], p[:, 1])
    a = np.arctan2(p[:, 1], p[:, 0])
    bi = np.clip(((a + math.pi / 2) / math.radians(DROP_BIN_DEG)).astype(int), 0, n - 1)
    tol = NOISE_PER_M * r
    below = (p[:, 2] < DROP_H - tol) & (r < max_range) & (p[:, 0] > 0)
    floor = (np.abs(p[:, 2]) < DROP_FLOOR_BAND + tol) & (p[:, 0] > 0)
    for b in np.unique(bi[below]):
        m = below & (bi == b)
        if m.sum() < DROP_MIN_PTS:
            continue
        h = p[m, 2]
        if np.percentile(h, 90) - np.percentile(h, 10) > DROP_FLAT_M + NOISE_PER_M * float(np.median(r[m])):
            continue                                   # not one flat lower surface
        near = int(np.argmin(r[m]))
        px, py, pz = p[m][near]
        # where the ray from the camera to this point crossed floor height: the edge is at or before it
        cx, cz = cam_xz
        t = cz / (cz - pz)
        cross = math.hypot(cx + t * (px - cx), t * py)
        before = floor & (bi == b) & (r < r[m][near])
        last_floor = float(r[before].max()) if before.any() else None
        # the edge is past the last floor seen and no farther than the crossing: mark from the nearer one
        edge[b] = last_floor if last_floor is not None and last_floor < cross else cross
        low[b] = float(r[m][near])
    return edge, low


def classify(p, angle_min, n_bins):
    """Points (N x 3, base_footprint, heights above the floor) -> per direction bin the distance of the
    nearest obstacle and of the nearest bump before it (inf = none), plus cell counts for the page."""
    r = np.hypot(p[:, 0], p[:, 1])
    h = p[:, 2]
    sel = (r < MAX_RANGE) & (h > -0.1) & (h <= MAX_H)
    r, h = r[sel], h[sel]
    a = np.arctan2(p[sel, 1], p[sel, 0])
    ai = ((a - angle_min) / BIN).astype(int)
    ri = (r / CELL).astype(int)
    n_r = int(MAX_RANGE / CELL) + 1
    ok = (ai >= 0) & (ai < n_bins) & (ri < n_r)
    ai, ri, h = ai[ok], ri[ok], h[ok]
    H = np.full((n_bins, n_r), -np.inf, dtype=np.float32)
    N = np.zeros((n_bins, n_r), dtype=np.int32)
    np.maximum.at(H, (ai, ri), h)
    np.add.at(N, (ai, ri), 1)
    valid = N >= MIN_PTS
    tol = NOISE_PER_M * (np.arange(n_r) * CELL)[None, :]
    # height of the nearest valid cell before this one (floor = 0 in front of the car where nothing is seen)
    idx = np.where(valid, np.arange(n_r)[None, :], -1)
    last = np.maximum.accumulate(idx, axis=1)
    prev_idx = np.concatenate([np.full((n_bins, 1), -1), last[:, :-1]], axis=1)
    prev_h = np.where(prev_idx >= 0, np.take_along_axis(np.where(valid, H, 0.0), np.maximum(prev_idx, 0), axis=1), 0.0)
    high = H > CLIMB_H + tol
    obstacle = valid & ((H > BUMP_H + tol) | (high & (H - prev_h > STEP_H + tol)))
    bump = valid & high & ~obstacle
    rng = (np.arange(n_r) * CELL)[None, :].repeat(n_bins, 0)
    obst_r = np.where(obstacle, rng, np.inf).min(axis=1)
    bump_r = np.where(bump & (rng < obst_r[:, None]), rng, np.inf).min(axis=1)
    # inside a cell use the nearest real point distance (cells start at ri*CELL)
    return obst_r.astype(np.float32), bump_r.astype(np.float32), int(obstacle.sum()), int(bump.sum())
HISTORY = 3              # frames for the median filter
RATE_HZ = 5.0            # processed depth frames per second (0.18 m/s -> 4 cm between frames)
AGREE_M = 0.15           # m: lidar and camera "agree" within this
COMPARE_PERIOD = 0.5     # s
LIDAR_X = car_geometry.LIDAR_X   # lidar_frame origin in base_footprint (TF), same orientation


class DepthObstacles:
    def __init__(self, node, tf_buffer):
        self.node, self.tf = node, tf_buffer
        self.log = node.get_logger()
        self.K = None
        self.rays = None          # unit rays (optical frame) for the sampled pixels
        self.R = self.t = None    # optical -> base_footprint
        self.hist = []
        self.scan = None          # latest filtered ranges (np array)
        self.angles = None
        self.lidar = None
        self.drops = 0
        self.drop_hist = []
        self.drop_dirs = 0
        self.hazards = None        # stall_detector.StallDetector: no-go marks (set by the web server)
        self.on_drop_ahead = None  # callback(note): a drop right in front while driving forward
        self.drop_pub = node.create_publisher(LaserScan, '/depth/drops', 1)
        self.floor = np.zeros(3)      # fitted floor plane a + b x + c y (smoothed)
        self.floor_seen = 0.0         # monotonic time the floor was last fitted
        self.floor_why = {}           # why the last fit failed (fit_floor)
        self.floor_hist = []          # last FLOOR_HIST frames: fitted or the failure stage
        self.on_frame = None          # run_recorder.py: called with each processed frame while recording
        self.bump_hist = []
        self.bumps = None
        self.cells = (0, 0)
        self.frames = 0
        self.last_frame = 0.0
        self.totals = {'both': 0, 'camera': 0, 'lidar': 0}
        self.last_compare = 0.0
        self.compare = {}
        self.pub = node.create_publisher(LaserScan, '/depth/scan', 5)
        self.bump_pub = node.create_publisher(LaserScan, '/depth/bumps', 5)
        self.cmp_pub = node.create_publisher(String, '/depth/compare', 1)
        node.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        node.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        node.create_subscription(LaserScan, LIDAR_TOPIC, self.on_lidar, qos_profile_sensor_data)
        node.create_timer(COMPARE_PERIOD, self.publish_compare)

    # ---- setup -------------------------------------------------------------------
    def on_info(self, msg):
        if self.K is not None:
            return
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.log.info(f'depth obstacles: camera {msg.width}x{msg.height}, fx {msg.k[0]:.0f}')

    def _setup(self, msg):
        fx, fy, cx, cy = self.K
        us = np.arange(0, msg.width, STEP, dtype=np.float32)
        vs = np.arange(0, msg.height, STEP, dtype=np.float32)
        u, v = np.meshgrid(us, vs)
        self.rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1).reshape(-1, 3)
        half = math.atan2(max(cx, msg.width - cx), fx) + math.radians(2)
        n = int(2 * half / BIN) + 1
        self.angles = -half + np.arange(n) * BIN
        self.angle_min, self.n_bins = -half, n

    def _transform(self, frame):
        t = self.tf.lookup_transform(BASE, frame, Time())
        q, tr = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)
        # direction from the robot description (TF), position as measured on the car (car_geometry.py:
        # the description has the camera 1.6 cm further back and 1.1 cm higher)
        self.t = np.array([car_geometry.CAMERA_X, tr.y, car_geometry.CAMERA_Z], dtype=np.float32)
        self.log.info(f'depth obstacles: camera at {np.round(self.t, 3).tolist()} m in {BASE} '
                      f'(TF said {[round(tr.x, 3), round(tr.y, 3), round(tr.z, 3)]})')

    # ---- per frame -----------------------------------------------------------------
    def on_lidar(self, msg):
        self.lidar = msg

    def on_depth(self, msg):
        if self.K is None or msg.encoding not in ('16UC1', 'mono16'):
            return
        now = time.monotonic()
        if now - self.last_frame < 1.0 / RATE_HZ - 0.01:
            return
        self.last_frame = now
        if self.rays is None:
            self._setup(msg)
        if self.R is None:
            try:
                self._transform(msg.header.frame_id)
            except Exception:
                return
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        d = d[::STEP, ::STEP].reshape(-1).astype(np.float32) * 0.001          # m along the optical axis
        ok = (d > MIN_RANGE) & (d < MAX_RANGE)
        p = (self.rays[ok] * d[ok, None]) @ self.R.T + self.t                 # points in base_footprint
        why = {}
        fl = fit_floor(p, why)
        self.floor_why = why
        self.floor_hist.append(why.get('stage', 'ok'))
        del self.floor_hist[:-FLOOR_HIST]
        if self.on_frame is not None:
            try:
                self.on_frame(msg, fl, why)
            except Exception as exc:
                self.log.warn(f'depth obstacles: recording a frame: {exc!r}')
        if fl is not None:
            self.floor = (1 - FLOOR_ALPHA) * self.floor + FLOOR_ALPHA * fl if self.floor_seen else fl
            self.floor_seen = now
        p[:, 2] -= self.floor[0] + self.floor[1] * p[:, 0] + self.floor[2] * p[:, 1]   # height above floor
        near = np.hypot(p[:, 0], p[:, 1]) < 1.0
        self.drops = int(np.count_nonzero((p[:, 2] < DROP_H) & near))
        if self.floor_seen and now - self.floor_seen < DROP_FLOOR_S:
            self._drops(*find_drops(p, (float(self.t[0]), float(self.t[2]))), msg.header.stamp)
        ranges, bumps, n_obst, n_bump = classify(p, self.angle_min, self.n_bins)
        self.cells = (n_obst, n_bump)

        self.hist.append(ranges)
        del self.hist[:-HISTORY]
        filt = np.median(np.stack(self.hist), axis=0) if len(self.hist) == HISTORY else ranges
        self.bump_hist.append(bumps)
        del self.bump_hist[:-HISTORY]
        self.bumps = np.median(np.stack(self.bump_hist), axis=0) if len(self.bump_hist) == HISTORY else bumps
        self.scan = filt
        self.frames += 1

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = BASE
        out.angle_min = float(self.angle_min)
        out.angle_max = float(self.angle_min + (self.n_bins - 1) * BIN)
        out.angle_increment = float(BIN)
        out.time_increment = 0.0
        out.scan_time = 1.0 / RATE_HZ
        out.range_min = 0.05
        out.range_max = float(MAX_RANGE)
        out.ranges = [float(x) for x in filt]
        self.pub.publish(out)
        out.ranges = [float(x) for x in self.bumps]
        self.bump_pub.publish(out)

    # ---- drops ------------------------------------------------------------------------
    def _drops(self, edge, low, stamp):
        """2 of the last 3 frames -> confirmed drop bins: publish, mark no-go, stop if right ahead."""
        self.drop_hist.append((edge, low))
        del self.drop_hist[:-HISTORY]
        if len(self.drop_hist) < 2:
            return
        E = np.stack([e for e, _ in self.drop_hist]); L = np.stack([l for _, l in self.drop_hist])
        seen = np.isfinite(E).sum(axis=0) >= 2
        edge, low = np.full(E.shape[1], np.inf), np.full(E.shape[1], np.inf)
        if seen.any():
            Es, Ls = E[:, seen], L[:, seen]
            edge[seen] = np.where(np.isfinite(Es), Es, np.inf).min(axis=0)     # nearest edge seen
            low[seen] = np.where(np.isfinite(Ls), Ls, -np.inf).max(axis=0)
        out = LaserScan()
        out.header.stamp, out.header.frame_id = stamp, BASE
        n = len(edge)
        out.angle_min, out.angle_increment = -math.pi / 2, math.radians(DROP_BIN_DEG)
        out.angle_max = out.angle_min + (n - 1) * out.angle_increment
        out.range_min, out.range_max = 0.05, float(DROP_RANGE * 2)
        out.ranges = [float(x) for x in edge]
        self.drop_pub.publish(out)
        if not seen.any():
            return
        self.drop_dirs = int(seen.sum())
        ang = -math.pi / 2 + (np.arange(n) + 0.5) * math.radians(DROP_BIN_DEG)
        pts = []
        for b in np.flatnonzero(seen):
            for d in np.arange(edge[b], min(low[b], edge[b] + 0.8) + 1e-6, MARK_STEP):
                pts.append((d * math.cos(ang[b]), d * math.sin(ang[b])))
        if self.hazards is not None and pts:
            self.hazards.add_marks_base(pts)
        # right in front of the car while it drives forward: stop now
        ex, ey = edge[seen] * np.cos(ang[seen]), edge[seen] * np.sin(ang[seen])
        close = (ex - car_geometry.FRONT < DROP_STOP_M) & (np.abs(ey) < car_geometry.HALF_WIDTH + 0.05)
        if close.any() and self.on_drop_ahead is not None:
            gap = float(np.min(ex[close]) - car_geometry.FRONT)
            self.on_drop_ahead(f'drop ahead ({max(gap, 0) * 100:.0f} cm past the bumper, depth camera)')

    # ---- comparison ------------------------------------------------------------------
    def lidar_bins(self):
        """Lidar ranges (from base_footprint) in the camera's direction bins (nearest per bin)."""
        s = self.lidar
        if s is None or self.angles is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(len(r), dtype=np.float32) * s.angle_increment
        ok = np.isfinite(r) & (r > max(s.range_min, 0.05)) & (r < s.range_max)
        x, y = r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])
        rb, ab = np.hypot(x, y), np.arctan2(y, x)
        out = np.full(self.n_bins, np.inf, dtype=np.float32)
        inside = (ab >= self.angle_min) & (ab < self.angle_min + self.n_bins * BIN)
        idx = ((ab[inside] - self.angle_min) / BIN).astype(int)
        np.minimum.at(out, idx, rb[inside])
        # the lidar has ~0.8 deg between points: fill single empty bins from their neighbours
        gaps = ~np.isfinite(out)
        nb = np.minimum(np.roll(out, 1), np.roll(out, -1))
        out[gaps] = nb[gaps]
        return out

    def publish_compare(self):
        now = time.monotonic()
        alive = now - self.last_frame < 1.0
        floor_age = now - self.floor_seen if self.floor_seen else None
        res = {'alive': alive, 'frames': self.frames, 'max_range': MAX_RANGE,
               'levels_m': {'drivable_up_to': CLIMB_H, 'bump_up_to': BUMP_H, 'max_step': STEP_H},
               'floor': {'seen_s_ago': round(floor_age, 1) if floor_age is not None else None,
                         'ok_pct': round(100 * self.floor_hist.count('ok') / len(self.floor_hist)) if self.floor_hist else None,
                         'why': self.floor_why or None,
                         'drops_checked': bool(self.floor_seen and now - self.floor_seen < DROP_FLOOR_S),
                         'offset_cm': round(float(self.floor[0]) * 100, 1),
                         'pitch_deg': round(math.degrees(math.atan(self.floor[1])), 2),
                         'roll_deg': round(math.degrees(math.atan(self.floor[2])), 2)},
               'bump_dirs': int(np.isfinite(self.bumps).sum()) if self.bumps is not None else 0,
               'nearest_bump_m': (round(float(np.min(self.bumps)), 2)
                                  if self.bumps is not None and np.isfinite(self.bumps).any() else None)}
        lid = self.lidar_bins() if alive else None
        if alive and lid is not None and self.scan is not None:
            cam = self.scan
            lim = MAX_RANGE
            c_ok, l_ok = cam < lim, lid < lim
            camc, lidc = np.where(c_ok, cam, 99.0), np.where(l_ok, lid, 99.0)   # no inf - inf
            both = c_ok & l_ok & (np.abs(camc - lidc) <= AGREE_M)
            camera = c_ok & (camc < lidc - AGREE_M)
            lidar = l_ok & (lidc < camc - AGREE_M)
            counts = {'both': int(both.sum()), 'camera': int(camera.sum()), 'lidar': int(lidar.sum())}
            for k in counts:
                self.totals[k] += counts[k]
            seen = sum(counts.values())
            cam_only = np.where(camera, cam, np.inf)
            res.update({
                'counts': counts,
                'agree_pct': round(100 * counts['both'] / seen) if seen else None,
                'totals': dict(self.totals),
                'nearest_camera_only_m': round(float(cam_only.min()), 2) if np.isfinite(cam_only.min()) else None,
                'nearest_camera_m': round(float(cam.min()), 2) if np.isfinite(cam.min()) else None,
                'nearest_lidar_m': round(float(lid.min()), 2) if np.isfinite(lid.min()) else None,
                'mean_diff_m': round(float(np.mean(cam[both] - lid[both])), 3) if both.any() else None,
                'drop_points': self.drops,
                'drop_dirs': self.drop_dirs,
            })
        self.compare = res
        if now - self.last_compare >= COMPARE_PERIOD - 0.05:
            self.last_compare = now
            self.cmp_pub.publish(String(data=json.dumps(res)))
