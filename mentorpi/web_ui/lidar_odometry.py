"""Odometry measured by the lidar (runs inside web_ui_server.py): rf2o_laser_odometry -> /odom_lidar.

ROS gets no wheel feedback (the motor encoders only feed the board's speed loop): /odom_raw adds up the
COMMANDED speed (made accurate by the speed map, README "Odometry"), and the EKF (/odom) adds the gyro's
turn rate (system/patch_ekf.py). Nothing measures how far the car really went. rf2o estimates the car's motion from how
consecutive lidar scans shift (range flow), so it sees a jammed car stand still and a carried car move.

It is published on /odom_lidar, not on /odom_rf2o (Hiwonder's EKF input name: rf2o's raw pose, with zero
covariance, would have taken over the cars' odometry). What the EKF gets instead is /odom_lidar_speed: rf2o's
forward speed averaged over SPEED_AVG_S, with a standard deviation of SPEED_SIGMA (system/patch_ekf.py fuses
it next to the commanded speed). Not while the car is lifted (lift_detector.py: then the lidar odometry is
wrong), nor when rf2o's messages are stale, nor in its first seconds after a (re)start.

Chosen on the replay of jpi1's 9.3-minute run (2026-10-24 21:44) against SLAM, the commanded speed fused with
the averaged lidar speed the way the EKF weighs them (5 s windows): sigma 2 cm/s took the position error
while driving from 13.5 to 10.5 cm (median) and 42.8 to 26.2 cm (90 %), the drift while the car really
stood (commanded to move) from 66.9 to 27.4 cm (90 %); the distance came out 9 % short (rf2o counts short).
rf2o is known to drift in featureless places (long plain corridors).

Runs while the sensors run (not in charging mode); restarted if it exits.
"""
import os
import signal
import subprocess
import threading
import time
from collections import deque

from nav_msgs.msg import Odometry

TOPIC = '/odom_lidar'
CMD = ['ros2', 'run', 'rf2o_laser_odometry', 'rf2o_laser_odometry_node', '--ros-args',
       '-r', '__node:=rf2o_laser_odometry',
       '-p', 'laser_scan_topic:=/scan_rf2o',      # scan_filter.py: -pi..pi, fixed 450 readings
       '-p', f'odom_topic:={TOPIC}', '-p', 'publish_tf:=false',   # never /odom_rf2o or a TF: the EKF owns odom
       '-p', 'base_frame_id:=base_footprint', '-p', 'odom_frame_id:=odom', '-p', "init_pose_from_topic:=''",
       '-p', 'freq:=10.0', '--log-level', 'WARN']
LOG = '/tmp/rf2o.log'
CHECK_S = 5.0
SPEED_TOPIC = '/odom_lidar_speed'
SPEED_AVG_S = 0.5          # rf2o's speed is noisy per message (sigma ~4 cm/s standing): averaged ...
SPEED_MIN_N = 3            # ... over at least this many messages ...
SPEED_SIGMA = 0.02         # ... and reported with this standard deviation (m/s)
SETTLE_S = 3.0             # after rf2o (re)starts
BIG = 1e6


class LidarOdometry:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.proc = None
        self.starts = 0
        self.started_at = 0.0
        self.window = deque()      # (time, forward speed) of rf2o's messages
        self.sent = 0
        self.speed_pub = node.create_publisher(Odometry, SPEED_TOPIC, 10)
        node.create_subscription(Odometry, TOPIC, self.on_odom, 10)
        # one left over from an earlier web server would publish /odom_lidar from another start point
        subprocess.run(['pkill', '-INT', '-f', '[r]f2o_laser_odometry_node'], check=False)
        threading.Thread(target=self._loop, daemon=True).start()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def _loop(self):
        time.sleep(10.0)                       # let the sensors come up after a restart
        while True:
            try:
                want = getattr(self.node, 'charging', 'off') == 'off'
                if want and not self.running():
                    self.proc = subprocess.Popen(CMD, start_new_session=True, stdout=open(LOG, 'w'),
                                                 stderr=subprocess.STDOUT)
                    self.starts += 1
                    self.started_at = time.monotonic()
                    self.window.clear()
                    self.log.info(f'lidar odometry (rf2o) started -> {TOPIC}')
                elif not want and self.running():
                    self.stop()
            except Exception as exc:
                self.log.warn(f'lidar odometry: {exc!r}')
            time.sleep(CHECK_S)

    def on_odom(self, msg):
        """rf2o's odometry -> /odom_lidar_speed for the EKF (averaged forward speed, honest covariance)."""
        now = time.monotonic()
        w = self.window
        w.append((now, msg.twist.twist.linear.x))
        while w and now - w[0][0] > SPEED_AVG_S:
            w.popleft()
        lifted = getattr(self.node, 'lifted', lambda: False)()
        if lifted or now - self.started_at < SETTLE_S or len(w) < SPEED_MIN_N:
            return
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = 'odom', 'base_footprint'
        out.twist.twist.linear.x = sum(v for _, v in w) / len(w)
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 7] = BIG
        cov[0] = SPEED_SIGMA ** 2
        out.twist.covariance = cov
        out.pose.covariance = [BIG if i % 7 == 0 else 0.0 for i in range(36)]
        self.speed_pub.publish(out)
        self.sent += 1

    def stop(self):
        if self.running():
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
            self.log.info('lidar odometry (rf2o) stopped')
        self.proc = None

    def status(self):
        return {'running': self.running(), 'topic': TOPIC, 'starts': self.starts, 'speed_msgs': self.sent}
