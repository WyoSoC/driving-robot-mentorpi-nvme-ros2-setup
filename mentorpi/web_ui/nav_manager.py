"""Navigation for the MentorPi web UI (runs inside web_ui_server.py, in the MentorPi container).

- start(map) / stop(): runs Nav2 (nav/nav.launch.py: map_server + AMCL + Hybrid-A* planner + Regulated
  Pure Pursuit + collision monitor, set up for this car) on a map saved with the Map panel. Not
  together with SLAM (both own the map -> odom transform).
- set_pose(x, y, yaw): AMCL's starting guess ("Set position" on the page).
- go(x, y, yaw) / cancel(): NavigateToPose goal; feedback (distance left, recoveries) for the page.
- Gate: Nav2's output (/nav/cmd_vel, after its collision monitor) reaches the motors (/cmd_vel, speed +
  turn rate, which odom_publisher clamps to 0.2 m/s and 0.5 rad/s) only while a goal is active and
  fresh. The page's E-stop and any joystick input cancel the goal; leaving it stops the car.
"""
import math
import os
import re
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.time import Time

MAPS_DIR = '/home/ubuntu/maps'
LAUNCH = ['ros2', 'launch', '/home/ubuntu/nav/nav.launch.py']
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
CMD_FRESH_S = 0.5          # Nav2 command older than this -> stop
PLAN_STEP = 0.05           # m between plan points sent to the page


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NavManager:
    def __init__(self, node, tf_buffer, slam):
        self.node, self.tf, self.slam = node, tf_buffer, slam
        self.lock = threading.Lock()
        self.proc = None
        self.map_name = None
        self.started_at = None
        self.message = ''
        self.goal = None           # [x, y, yaw]
        self.goal_handle = None
        self.goal_state = 'idle'   # idle | sending | active | succeeded | aborted | canceled | rejected
        self.feedback = {}
        self.plan = []
        self.pose = None
        self.pose_set = False
        self.last_nav_cmd = None
        self.moving = False

        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 1)
        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        node.create_subscription(Twist, '/nav/cmd_vel', self.on_nav_cmd, 1)
        node.create_subscription(Path, '/plan', self.on_plan, 1)
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        node.create_timer(0.1, self.gate_watchdog)
        node.create_timer(0.2, self.update_pose)
        self._kill_strays()

    # ---- process ---------------------------------------------------------------
    def _kill_strays(self):
        subprocess.run(['pkill', '-INT', '-f', 'nav/nav.launch.py'], check=False)

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def maps(self):
        try:
            return sorted(f[:-5] for f in os.listdir(MAPS_DIR) if f.endswith('.yaml'))
        except OSError:
            return []

    def map_meta(self, name):
        """resolution + origin from a saved map yaml (the page draws the PNG with it)."""
        meta = {}
        with open(os.path.join(MAPS_DIR, name + '.yaml')) as f:
            for line in f:
                k, _, v = line.partition(':')
                meta[k.strip()] = v.strip()
        origin = [float(x) for x in meta['origin'].strip('[]').split(',')[:2]]
        return {'image': meta['image'], 'resolution': float(meta['resolution']), 'origin': origin}

    def start(self, name):
        with self.lock:
            if self.running():
                return False, f'already navigating on "{self.map_name}"'
            if not NAME_RE.match(name or '') or name not in self.maps():
                return False, f'no saved map "{name}"'
            if self.slam.running():
                return False, 'stop mapping first'
            self.proc = subprocess.Popen(LAUNCH + [f'map:={os.path.join(MAPS_DIR, name + ".yaml")}'],
                                         start_new_session=True,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.map_name, self.started_at = name, time.time()
            self.goal, self.goal_state, self.plan, self.pose, self.pose_set = None, 'idle', [], None, False
            self.message = f'navigation starting on "{name}" (~20 s); then set the car\'s position on the map'
            self.node.get_logger().info(f'navigation started on map {name}')
            return True, self.message

    def stop(self):
        self.cancel()
        with self.lock:
            if not self.running():
                return False, 'not navigating'
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc = None
            self.message = 'navigation stopped'
            self.node.get_logger().info('navigation stopped')
            return True, self.message

    def shutdown(self):
        if self.running():
            self.stop()

    # ---- localization ----------------------------------------------------------
    def set_pose(self, x, y, yaw):
        if not self.running():
            return False, 'start navigation first'
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2          # +-25 cm
        cov[35] = math.radians(15) ** 2      # +-15 deg
        m.pose.covariance = cov
        for _ in range(3):
            self.pose_pub.publish(m)
            time.sleep(0.05)
        self.pose_set = True
        self.message = f'position set to ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f} deg; drive a little to let it settle'
        return True, self.message

    def update_pose(self):
        if not self.running():
            self.pose = None
            return
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return
        tr = t.transform.translation
        self.pose = [tr.x, tr.y, yaw_of(t.transform.rotation)]

    def on_plan(self, msg):
        pts = []
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            if not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) >= PLAN_STEP:
                pts.append([round(x, 3), round(y, 3)])
        self.plan = pts

    # ---- goals -----------------------------------------------------------------
    def go(self, x, y, yaw):
        if not self.running():
            return False, 'start navigation first'
        if not self.pose_set:
            return False, 'set the car\'s position on the map first'
        if not self.client.wait_for_server(timeout_sec=2.0):
            return False, 'navigation is still starting, try again in a few seconds'
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal, self.goal_state, self.feedback, self.plan = [x, y, yaw], 'sending', {}, []
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)
        self.message = f'going to ({x:.2f}, {y:.2f})'
        self.node.get_logger().info(self.message)
        return True, self.message

    def on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.goal_state, self.message = 'rejected', 'goal rejected'
            return
        self.goal_handle, self.goal_state = handle, 'active'
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        fb = msg.feedback
        self.feedback = {'distance_left': round(fb.distance_remaining, 2),
                         'recoveries': fb.number_of_recoveries,
                         'time_s': fb.navigation_time.sec}

    def on_result(self, fut):
        status = fut.result().status
        self.goal_handle = None
        self.goal_state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                           GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'aborted')
        self.message = {'succeeded': 'arrived', 'canceled': 'goal canceled',
                        'aborted': 'could not reach the goal (no path, or stuck)'}[self.goal_state]
        self.node.get_logger().info(f'navigation goal {self.goal_state}')
        self.stop_motors()

    def cancel(self, reason='goal canceled'):
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            self.message = reason
        if self.goal_state in ('sending', 'active'):
            self.goal_state = 'canceled'
        self.goal_handle = None
        self.stop_motors()
        return True, reason

    # ---- the gate --------------------------------------------------------------
    def goal_active(self):
        return self.goal_state in ('sending', 'active') and self.running()

    def on_nav_cmd(self, msg):
        self.last_nav_cmd = time.monotonic()
        if not self.goal_active():
            return
        self.cmd_pub.publish(msg)
        self.moving = abs(msg.linear.x) > 1e-3

    def gate_watchdog(self):
        stale = self.last_nav_cmd is None or time.monotonic() - self.last_nav_cmd > CMD_FRESH_S
        if self.moving and (stale or not self.goal_active()):
            self.stop_motors()
            if self.goal_active():
                self.node.get_logger().warn('no fresh navigation command: car stopped')
        if self.goal_state in ('sending', 'active') and not self.running():
            self.goal_state, self.message = 'aborted', 'navigation stopped'

    def stop_motors(self):
        self.cmd_pub.publish(Twist())
        self.moving = False

    # ---- status for the web page -----------------------------------------------
    def status(self):
        meta = None
        if self.map_name:
            try:
                meta = self.map_meta(self.map_name)
            except (OSError, KeyError, ValueError):
                meta = None
        return {
            'running': self.running(),
            'uptime_s': round(time.time() - self.started_at) if self.running() else None,
            'map_name': self.map_name if self.running() else None,
            'map': meta if self.running() else None,
            'maps': self.maps(),
            'pose': self.pose,
            'pose_set': self.pose_set,
            'goal': self.goal,
            'goal_state': self.goal_state,
            'feedback': self.feedback,
            'plan': self.plan if self.goal_active() else [],
            'message': self.message,
        }
