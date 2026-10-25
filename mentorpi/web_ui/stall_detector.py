"""The car pushes against something (runs inside web_ui_server.py): the motors turn, the car doesn't move.

The car has no wheel encoders (Hiwonder's /odom adds up the commanded speed, so it "moves" while jammed).
The lidar is the honest sign: driving, the distances ahead and behind change; jammed, they stay the same.

Blocked = during a mission (autonomous mapping, a navigation goal, or the explorer's own escape move; not
while a person drives with the joystick) the motors were told to drive at >= CMD_V_MIN in one direction for
the whole last WINDOW_S, and the lidar ranges ahead and behind changed by less than STILL_M (median).

Then:
1. the motors stop;
2. the place just past the pushing bumper is marked as an obstacle (/bump/cloud -> Nav2's bump_layer, both
   costmaps): it is something the sensors don't see, or the planner would not have driven into it. Marks
   stay for this mapping / navigation run (cleared when either starts again);
3. the car backs out the opposite way (explore_manager.bumped(): backwards after hitting something ahead,
   forwards after hitting something behind while reversing) and goes on: autonomous mapping picks its next
   goal, a navigation goal is paused and resumed.
Each bump is logged (the run recording's nav log has the explorer's lines; events.jsonl the message).
"""
import math
import threading
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import car_geometry as g

WINDOW_S = 1.5             # commanded to drive this long ...
CMD_V_MIN = 0.04           # ... at least this fast (m/s), always the same direction ...
STILL_M = 0.025            # ... and the lidar changed less than this (median, beams ahead and behind)
SECTOR_DEG = 45            # beams within this of straight ahead / behind are compared
BIN_DEG = 1.0
COOLDOWN_S = 5.0           # after a bump: no new one for this long (the back-out move starts)
MARK_AHEAD = 0.03          # m past the bumper: where the obstacle is marked
MARK_STEP = 0.03
PUBLISH_S = 1.0


def binned(ranges, angles, bin_deg=BIN_DEG):
    """Ranges per fixed direction bin (nearest valid reading, NaN if none): the MS200 sends 448 to 450
    readings per turn, so scans are compared by direction, not by index."""
    r = np.asarray(ranges, float)
    a = (np.asarray(angles) + math.pi) % (2 * math.pi)
    idx = np.minimum((a / math.radians(bin_deg)).astype(int), int(360 / bin_deg) - 1)
    ok = np.isfinite(r) & (r > 0.12) & (r < 6)
    out = np.full(int(360 / bin_deg), np.inf)
    np.minimum.at(out, idx[ok], r[ok])
    out[np.isinf(out)] = np.nan
    return out


def lidar_still(r_then, r_now, angles_then, angles_now=None, sector_deg=SECTOR_DEG):
    """Median change (m) of the ranges ahead and behind between two scans, or None if too few readings."""
    b0 = binned(r_then, angles_then)
    b1 = binned(r_now, angles_then if angles_now is None else angles_now)
    centre = np.radians((np.arange(len(b0)) + 0.5) * BIN_DEG) - math.pi      # bin direction, 0 = ahead
    a = np.abs(centre)
    sel = (a <= math.radians(sector_deg)) | (a >= math.pi - math.radians(sector_deg))
    ok = sel & np.isfinite(b0) & np.isfinite(b1)
    if ok.sum() < 20:
        return None
    return float(np.median(np.abs(b1[ok] - b0[ok])))


def pushing(cmds, now, window=WINDOW_S, v_min=CMD_V_MIN):
    """+1 / -1 if every motor command of the last `window` s drove forward / backward at >= v_min, else 0."""
    recent = [v for t, v in cmds if now - t <= window]
    if not cmds or now - cmds[0][0] < window or len(recent) < 5 or now - cmds[-1][0] > 0.4:
        return 0
    if all(v >= v_min for v in recent):
        return 1
    if all(v <= -v_min for v in recent):
        return -1
    return 0


class StallDetector:
    def __init__(self, node, tf, explorer, nav, slam):
        self.node, self.tf, self.explorer, self.nav, self.slam = node, tf, explorer, nav, slam
        self.log = node.get_logger()
        self.cmds = []             # (t, v) of /cmd_vel (what the motors get)
        self.scans = []            # (t, ranges) of /scan_raw
        self.marks = []            # [x, y] in map frame
        self.session = None
        self.last_bump = 0.0
        self.bumps = 0
        self.note = ''
        self.lock = threading.Lock()
        self.pub = node.create_publisher(PointCloud2, '/bump/cloud', 1)
        node.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        node.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        node.create_timer(0.25, self.check)
        node.create_timer(PUBLISH_S, self.publish)

    def on_cmd(self, msg):
        now = time.monotonic()
        with self.lock:
            self.cmds.append((now, msg.linear.x))
            self.cmds = [c for c in self.cmds if now - c[0] <= WINDOW_S + 1.0]

    def on_scan(self, msg):
        now = time.monotonic()
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        with self.lock:
            self.scans.append((now, np.asarray(msg.ranges, dtype=np.float32), angles))
            self.scans = [s for s in self.scans if now - s[0] <= WINDOW_S + 1.0]

    def in_mission(self):
        ex = self.explorer
        return ex.active() or ex.recovering or self.nav.goal_active()

    def check(self):
        try:
            self._check()
        except Exception as exc:
            self.log.warn(f'stall check: {exc!r}')

    def _check(self):
        session = (getattr(self.slam, 'started_at', None) if self.slam.running() else None,
                   self.nav.started_at if self.nav.running() else None)
        if session != self.session:          # a new mapping / navigation run: forget the old marks
            self.session, self.marks = session, []
        now = time.monotonic()
        if now - self.last_bump < COOLDOWN_S or not self.in_mission():
            return
        if now - self.node.last_cmd_time < 1.0 and self.node.moving:
            return                           # a person drives with the joystick
        with self.lock:
            cmds, scans = list(self.cmds), list(self.scans)
        direction = pushing(cmds, now)
        if not direction or not scans:
            return
        then = [s for s in scans if now - s[0] >= WINDOW_S]
        if not then or now - scans[-1][0] > 0.5:
            return
        change = lidar_still(then[-1][1], scans[-1][1], then[-1][2], scans[-1][2])
        if change is None or change >= STILL_M:
            return
        self.bump(direction, change)

    def bump(self, direction, change):
        self.last_bump = time.monotonic()
        self.bumps += 1
        where = 'ahead' if direction > 0 else 'behind'
        self.note = f'bumped into something {where} (motors on, lidar still: {change * 100:.1f} cm change)'
        self.log.warn(f'stall: {self.note}')
        self.node.cmd_pub.publish(Twist())   # motors off now
        self._mark(direction)
        self.explorer.bumped(direction, self.note)

    def _mark(self, direction):
        """Obstacle marks along the pushing bumper, in the map frame."""
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x = g.FRONT + MARK_AHEAD if direction > 0 else g.BACK - MARK_AHEAD
        c, s = math.cos(yaw), math.sin(yaw)
        for y in np.arange(-g.HALF_WIDTH, g.HALF_WIDTH + 1e-6, MARK_STEP):
            self.marks.append([tr.x + c * x - s * y, tr.y + s * x + c * y])

    def publish(self):
        """The marks as a cloud in base_footprint (like fleet.py), for Nav2's bump_layer."""
        if not self.marks or not self.nav.running():
            return
        try:
            t = self.tf.lookup_transform('base_footprint', 'map', Time())
        except Exception:
            return
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        xy = np.asarray(self.marks)
        pts = np.stack([tr.x + c * xy[:, 0] - s * xy[:, 1], tr.y + s * xy[:, 0] + c * xy[:, 1],
                        np.full(len(xy), 0.05)], -1).astype(np.float32)
        hdr = t.header
        hdr.frame_id = 'base_footprint'
        self.pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts.tolist()))

    def status(self):
        return {'bumps': self.bumps, 'marks': len(self.marks), 'note': self.note}
