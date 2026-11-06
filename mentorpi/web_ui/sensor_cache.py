"""Full sensor data for the API (robot_api.py: GET /api/v1/sensors, /api/v1/stream?full=1).

The latest message of every sensor topic, every field as ROS has it (converted to JSON: NaN / inf -> null),
with when it arrived and how often it comes. Subscribed only while a program asks (first request: waits up to
WAIT_S for the first messages), unsubscribed IDLE_S after the last request, so the car does no extra work
when nobody uses it.
"""
import array
import base64
import math
import threading
import time
from collections import deque

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, LaserScan
from std_msgs.msg import Bool, UInt16

try:
    from ros_robot_controller_msgs.msg import ButtonState
except ImportError:
    ButtonState = None

IDLE_S = 60.0
WAIT_S = 4.0       # new subscriptions take 1-3 s to connect (DDS discovery)
QUIET = {'depth_drops', 'command', 'button', 'odom_lidar'}   # only publish now and then: not waited for

# name: (topic, type, what it is)
TOPICS = {
    'lidar': ('/scan_raw', LaserScan, 'MS200 lidar, 360°, ~10 Hz: ranges (m) and intensities'),
    'imu_raw': ('/ros_robot_controller/imu_raw', Imu, "the controller board's IMU as it comes (not calibrated)"),
    'imu': ('/imu', Imu, 'IMU with the calibration (imu_calib.yaml) applied'),
    'imu_corrected': ('/imu_corrected', Imu, 'IMU with the gyro start-up bias removed (what the EKF uses)'),
    'imu_rpy': ('/imu/rpy/filtered', Vector3Stamped, 'roll, pitch, yaw (rad) from the IMU orientation filter'),
    'imu_steady': ('/imu/steady_state', Bool, 'the IMU filter thinks the car stands still'),
    'odom_wheels': ('/odom_raw', Odometry, 'wheel odometry (motor encoders + steering angle)'),
    'odom': ('/odom', Odometry, 'EKF: wheels + IMU + lidar speed fused (what everything else uses)'),
    'odom_lidar': ('/odom_lidar', Odometry, 'lidar odometry (rf2o, scan matching)'),
    'battery_mv': ('/ros_robot_controller/battery', UInt16, 'battery voltage in mV (2S pack), ~1 Hz'),
    'depth_scan': ('/depth/scan', LaserScan, 'obstacles seen by the depth camera, as a scan (base_footprint)'),
    'depth_bumps': ('/depth/bumps', LaserScan, 'low bumps the depth camera sees (can drive over)'),
    'depth_drops': ('/depth/drops', LaserScan, 'drops / stair edges the depth camera sees (only when it sees one)'),
    'rgb_camera_info': ('/ascamera/camera_publisher/rgb0/camera_info', CameraInfo, 'colour camera intrinsics'),
    'depth_camera_info': ('/ascamera/camera_publisher/depth0/camera_info', CameraInfo, 'depth camera intrinsics'),
    'command': ('/app/cmd_vel', Twist, 'the drive command the car is executing (only while driving): linear.x '
                                       'm/s, angular.z = front wheel steering angle (rad, + left)'),
}
if ButtonState is not None:
    TOPICS['button'] = ('/ros_robot_controller/button', ButtonState, "the controller board's buttons (last press)")


def to_json(v):
    """A ROS message (or field) as JSON-safe Python: every field, NaN / inf -> None."""
    if hasattr(v, 'get_fields_and_field_types'):
        return {k: to_json(getattr(v, k)) for k in v.get_fields_and_field_types()}
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, (bytes, bytearray)):
        return base64.b64encode(v).decode()
    if isinstance(v, (list, tuple, array.array)) or type(v).__name__ == 'ndarray':
        return [to_json(x) for x in v]
    if type(v).__module__ == 'numpy':
        return to_json(v.item())
    return v


class SensorCache:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.subs = {}
        self.latest = {}               # name -> (msg, wall time, monotonic time)
        self.times = {k: deque(maxlen=30) for k in TOPICS}
        self.counts = {k: 0 for k in TOPICS}
        self.asked_at = 0.0
        node.create_timer(5.0, self._idle_check)

    def _cb(self, name):
        def cb(msg):
            now = time.monotonic()
            self.latest[name] = (msg, time.time(), now)
            self.times[name].append(now)
            self.counts[name] += 1
        return cb

    def _subscribe(self):
        with self.lock:
            if self.subs:
                return False
            for name, (topic, typ, _) in TOPICS.items():
                qos = 10 if typ in (UInt16, Twist, Bool) or name == 'button' else qos_profile_sensor_data
                self.subs[name] = self.node.create_subscription(typ, topic, self._cb(name), qos)
            self.node.get_logger().info(f'API: full sensor data subscribed ({len(self.subs)} topics)')
            return True

    def _idle_check(self):
        if self.subs and time.monotonic() - self.asked_at > IDLE_S:
            with self.lock:
                for s in self.subs.values():
                    self.node.destroy_subscription(s)
                self.subs = {}
                self.latest = {}
            self.node.get_logger().info('API: full sensor data unsubscribed (nobody asked for a minute)')

    def rate(self, name):
        t = self.times[name]
        if len(t) < 2 or time.monotonic() - t[-1] > 3:
            return None
        return round((len(t) - 1) / (t[-1] - t[0]), 1) if t[-1] > t[0] else None

    def snapshot(self):
        """{name: {topic, type, about, age_s, hz, received, msg}} for every sensor topic (msg None: no data)."""
        self.asked_at = time.monotonic()
        if self._subscribe():
            end = time.monotonic() + WAIT_S
            while time.monotonic() < end and not all(k in self.latest for k in TOPICS if k not in QUIET):
                time.sleep(0.05)
        now = time.monotonic()
        out = {}
        for name, (topic, typ, about) in TOPICS.items():
            got = self.latest.get(name)
            out[name] = {'topic': topic, 'type': f'{typ.__module__.split(".")[0]}/{typ.__name__}', 'about': about,
                         'age_s': round(now - got[2], 3) if got else None, 'hz': self.rate(name),
                         'received': self.counts[name], 'msg': to_json(got[0]) if got else None}
        return out
