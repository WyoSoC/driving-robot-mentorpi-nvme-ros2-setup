"""Obstacles from the depth camera, compared with (and added to) the lidar.

Why: the lidar scans one flat plane 16 cm above the floor. Anything lower (a shoe, a cable bundle, a toy)
or an overhang above it is invisible to it. The depth camera (Angstrong HP60C, 12 cm high, looking straight
ahead, 58 deg wide) sees a 3-D picture of what is in front of the car.

What this does with a depth frame, at most RATE_HZ times a second (the camera sends ~12; 12 Hz of this
plus Nav2's two costmap layers overloaded the Pi 5 while navigating):
1. Every 4th pixel becomes a 3-D point (camera intrinsics), moved into the car frame (base_footprint, TF).
2. Points between MIN_H and MAX_H above the floor and within MAX_RANGE are obstacles: the band the car's
   body occupies. The floor itself (and its noise) and anything above the car are left out.
3. For each 0.5 deg direction the nearest obstacle -> a LaserScan in base_footprint: /depth/scan.
4. Noise filter: a direction counts only if seen in at least 2 of the last 3 frames (median of 3).

Fusion: Nav2 uses /scan_raw and /depth/scan as two separate costmap layers (each clears only its own
marks; one shared layer would let the lidar, whose plane passes over a shoe, erase the camera's mark of
it), and the collision monitor watches both. See nav/nav2_params.yaml.

Comparison (/depth/compare, JSON String, 2 Hz): inside the camera's view, per direction:
  both     - lidar and camera see an obstacle at about the same distance (within AGREE_M)
  camera   - only the camera sees one (or a closer one): low objects / overhangs the lidar misses
  lidar    - only the lidar sees one: things the camera misses (dark or shiny surfaces, too close,
             outside its height band, thin legs)
plus running totals since start, so the page can show how the two sensors compare.
Drop candidates: points clearly BELOW the floor (stairs) are counted, not used for driving: on glossy
floors the camera gets no floor returns at all, so this can't be trusted yet.
"""
import json
import math
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

DEPTH_TOPIC = '/ascamera/camera_publisher/depth0/image_raw'
INFO_TOPIC = '/ascamera/camera_publisher/depth0/camera_info'
LIDAR_TOPIC = '/scan_raw'
BASE = 'base_footprint'
STEP = 4                 # use every 4th pixel (160 x 120 points)
MIN_RANGE = 0.15         # m from the camera: closer readings are unreliable
MAX_RANGE = 2.5          # m
MIN_H = 0.04             # m above the floor: lower = floor / noise
MAX_H = 0.22             # m: higher = above the car (lidar at 0.16 m, top of the car ~0.2 m)
DROP_H = -0.06           # m: below the floor by this much = possible drop (stairs); counted only
BIN = math.radians(0.5)
HISTORY = 3              # frames for the median filter
RATE_HZ = 5.0            # processed depth frames per second (0.18 m/s -> 4 cm between frames)
AGREE_M = 0.15           # m: lidar and camera "agree" within this
COMPARE_PERIOD = 0.5     # s
LIDAR_X = -0.010         # lidar_frame origin in base_footprint (TF), same orientation


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
        self.frames = 0
        self.last_frame = 0.0
        self.totals = {'both': 0, 'camera': 0, 'lidar': 0}
        self.last_compare = 0.0
        self.compare = {}
        self.pub = node.create_publisher(LaserScan, '/depth/scan', 5)
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
        self.t = np.array([tr.x, tr.y, tr.z], dtype=np.float32)
        self.log.info(f'depth obstacles: camera at {np.round(self.t, 3).tolist()} m in {BASE}')

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
        h = p[:, 2]
        obst = (h >= MIN_H) & (h <= MAX_H)
        r = np.hypot(p[obst, 0], p[obst, 1])
        a = np.arctan2(p[obst, 1], p[obst, 0])
        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        idx = np.clip(((a - self.angle_min) / BIN).astype(int), 0, self.n_bins - 1)
        np.minimum.at(ranges, idx, r)
        near = np.hypot(p[:, 0], p[:, 1]) < 1.0
        self.drops = int(np.count_nonzero((h < DROP_H) & near))

        self.hist.append(ranges)
        del self.hist[:-HISTORY]
        filt = np.median(np.stack(self.hist), axis=0) if len(self.hist) == HISTORY else ranges
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
        res = {'alive': alive, 'frames': self.frames, 'band_m': [MIN_H, MAX_H], 'max_range': MAX_RANGE}
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
            })
        self.compare = res
        if now - self.last_compare >= COMPARE_PERIOD - 0.05:
            self.last_compare = now
            self.cmp_pub.publish(String(data=json.dumps(res)))
