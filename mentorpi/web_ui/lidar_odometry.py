"""Odometry measured by the lidar (runs inside web_ui_server.py): rf2o_laser_odometry -> /odom_lidar.

ROS gets no wheel feedback (the motor encoders only feed the board's speed loop): /odom_raw adds up the
COMMANDED speed (made accurate by the speed map, README "Odometry"), and the EKF (/odom) adds the gyro's
turn rate (system/patch_ekf.py). Nothing measures how far the car really went. rf2o estimates the car's motion from how
consecutive lidar scans shift (range flow), so it sees a jammed car stand still and a carried car move.

It is published on /odom_lidar, not on /odom_rf2o: Hiwonder's ekf.yaml already lists /odom_rf2o as an input,
so that name would change the cars' odometry (and with it SLAM, navigation and every behaviour built on it)
at once. First compare (Odometry panel: green = /odom, blue = /odom_lidar; run recordings), then decide.
rf2o is known to drift in featureless places (long plain corridors) and goes wrong while the car is carried.

Runs while the sensors run (not in charging mode); restarted if it exits.
"""
import os
import signal
import subprocess
import threading
import time

TOPIC = '/odom_lidar'
CMD = ['ros2', 'run', 'rf2o_laser_odometry', 'rf2o_laser_odometry_node', '--ros-args',
       '-r', '__node:=rf2o_laser_odometry',
       '-p', 'laser_scan_topic:=/scan_rf2o',      # scan_filter.py: -pi..pi, fixed 450 readings
       '-p', f'odom_topic:={TOPIC}', '-p', 'publish_tf:=false',   # never /odom_rf2o or a TF: the EKF owns odom
       '-p', 'base_frame_id:=base_footprint', '-p', 'odom_frame_id:=odom', '-p', "init_pose_from_topic:=''",
       '-p', 'freq:=10.0', '--log-level', 'WARN']
LOG = '/tmp/rf2o.log'
CHECK_S = 5.0


class LidarOdometry:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.proc = None
        self.starts = 0
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
                    self.log.info(f'lidar odometry (rf2o) started -> {TOPIC}')
                elif not want and self.running():
                    self.stop()
            except Exception as exc:
                self.log.warn(f'lidar odometry: {exc!r}')
            time.sleep(CHECK_S)

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
        return {'running': self.running(), 'topic': TOPIC, 'starts': self.starts}
