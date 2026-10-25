"""Navigation for the MentorPi web UI (runs inside web_ui_server.py, in the MentorPi container).

- start(map) / stop(): runs Nav2 (nav/nav.launch.py: map_server + AMCL + Hybrid-A* planner + Regulated
  Pure Pursuit + collision monitor, set up for this car) on a map saved with the Map panel. Not
  together with SLAM (both own the map -> odom transform).
- set_pose(x, y, yaw): AMCL's starting guess ("Set position" on the page).
- find_position(): finds the car on the map from one lidar scan (localize.py), no guess needed; runs by
  itself once Nav2 is up, and from "Find position" on the page. If it is not sure, the best guess is
  shown and the position is set by hand.
- go(x, y, yaw) / cancel(): NavigateToPose goal; feedback (distance left, recoveries) for the page.
- Tight start: if a goal fails before the car has moved (it stands so close to a wall or the other car
  that Nav2 sees its start inside an obstacle and finds no path at all), the car creeps NUDGE_M straight
  ahead (or back, whichever side the lidar shows room) and sends the goal again (state "nudging"; at
  most NUDGE_MAX times per goal).
- pause(reason) / resume(): right of way (fleet.py): the goal is taken back from Nav2 (state "yielding",
  car stopped) and sent again later, so Nav2 doesn't count the wait as being stuck.
- Gate: Nav2's output (/nav/cmd_vel, after its collision monitor) reaches the motors (/cmd_vel, speed +
  turn rate, which odom_publisher clamps to 0.2 m/s and 0.5 rad/s) only while a goal is active and
  fresh. The page's E-stop and any joystick input cancel the goal; leaving it stops the car.
- Steering (shape_steering): the wheels' angle follows the path's curvature (turn rate / speed; Hiwonder's
  odom_publisher computes atan(0.145 * w / v)). Nav2 slows to a few cm/s at every forward / reverse switch
  and near obstacles while still sending a turn rate, so the ratio jumped to full lock and back (recorded on
  jpi2: steering -69..+73 deg, a dozen switches in 20 s). The gate keeps the curvature within what the wheels
  can do (MAX_STEER_DEG), holds it while slower than STEER_V_MIN, and changes it by at most KAPPA_STEP per
  command (full left to full right in about a second).
"""
import math
import os
import signal
import subprocess
import threading
import time

import numpy as np

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import Trigger

import car_geometry
import localize
import map_merge
import map_store

MAPS_DIR = map_store.MAPS_DIR
LAUNCH = ['ros2', 'launch', '/home/ubuntu/nav/nav.launch.py']
NAV_LOG = '/tmp/nav.log'   # Nav2's output (inside the container), overwritten at every start
NAME_RE = map_store.NAME_RE
CMD_FRESH_S = 0.5          # Nav2 command older than this -> stop
WHEELBASE = 0.145          # m, as in Hiwonder's ackermann.py (steering = atan(WHEELBASE * w / v))
MAX_STEER_DEG = 30.0       # the wheels' real limit
KAPPA_MAX = math.tan(math.radians(MAX_STEER_DEG)) / WHEELBASE   # 1/m, ~4 (radius 0.25 m)
STEER_V_MIN = 0.03         # m/s: slower -> keep the wheels where they are
KAPPA_STEP = 0.8           # 1/m per command (10 Hz): full range (2 x KAPPA_MAX) in ~1 s
PLAN_STEP = 0.05           # m between plan points sent to the page
SCAN_TOPIC = '/scan_raw'
SCAN_FRESH_S = 1.0
NUDGE_M = 0.15             # tight start: creep this far, then try the goal again (once)
NUDGE_SPEED = 0.06
NUDGE_MAX = 2              # creeps per goal
NUDGE_ROOM = 0.25          # m free needed past the bumper in that direction
BUMPER = car_geometry.BUMPER   # lidar centre -> front / rear bumper


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def shape_steering(v, w, kappa):
    """(speed, turn rate) from Nav2 + the curvature now -> (speed, turn rate to send, new curvature)."""
    if abs(v) >= STEER_V_MIN:
        want = max(-KAPPA_MAX, min(KAPPA_MAX, w / v))
        kappa += max(-KAPPA_STEP, min(KAPPA_STEP, want - kappa))
    return v, v * kappa, kappa


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
        self.goal_state = 'idle'   # idle | sending | active | yielding | succeeded | aborted | canceled | rejected
        self.feedback = {}
        self.plan = []
        self.pose = None
        self.pose_set = False
        self.last_nav_cmd = None
        self.moving = False
        self.slam_mode = False
        self.goal_seq = 0
        self.kappa = 0.0           # curvature the wheels are set to (shape_steering)
        self.scan = None
        self.finding = False
        self.found = None          # last find_position() result (pose, sure, why, ...)
        self.auto_find = False     # find the position by itself once Nav2 is up
        self.goal_from = None      # where the car was when the goal was given (tight start check)
        self.nudged = 0

        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 1)
        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        node.create_subscription(Twist, '/nav/cmd_vel', self.on_nav_cmd, 1)
        node.create_subscription(Path, '/plan', self.on_plan, 1)
        node.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')   # plan only (explorer)
        self.active_cli = node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.nav_active = False    # Nav2's lifecycle manager says all its nodes are active
        self._asking = False
        node.create_timer(0.1, self.gate_watchdog)
        node.create_timer(0.2, self.update_pose)
        node.create_timer(1.0, self.auto_find_tick)
        self._kill_strays()

    # ---- process ---------------------------------------------------------------
    def _kill_strays(self):
        subprocess.run(['pkill', '-INT', '-f', 'nav/nav.launch.py'], check=False)

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def maps(self):
        return map_store.names()

    def map_meta(self, name):
        """resolution, origin, image and content hash of a saved map (the page draws the PNG with it)."""
        return map_store.meta(name)

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
                                         stdout=open(NAV_LOG, 'w'), stderr=subprocess.STDOUT)
            self.map_name, self.started_at, self.slam_mode = name, time.time(), False
            self.nav_active = False
            self.goal, self.goal_state, self.plan, self.pose, self.pose_set = None, 'idle', [], None, False
            self.found, self.auto_find = None, True
            self.message = f'navigation starting on "{name}" (~20 s); then it finds its position on the map'
            self.node.get_logger().info(f'navigation started on map {name}')
            return True, self.message

    def start_on_slam(self):
        """Nav2 on the live map of the running SLAM (exploration). SLAM localizes: no position to set."""
        with self.lock:
            if self.running():
                return False, 'navigation is already running'
            if not self.slam.running():
                return False, 'start mapping first'
            self.proc = subprocess.Popen(LAUNCH + ['slam:=true'], start_new_session=True,
                                         stdout=open(NAV_LOG, 'w'), stderr=subprocess.STDOUT)
            self.map_name, self.started_at, self.slam_mode = None, time.time(), True
            self.nav_active = False
            self.goal, self.goal_state, self.plan, self.pose, self.pose_set = None, 'idle', [], None, True
            self.found, self.auto_find = None, False
            self.message = 'navigation starting on the live map'
            self.node.get_logger().info('navigation started on the live SLAM map')
            return True, self.message

    def ready(self):
        """Nav2 accepts goals: its action server exists AND its lifecycle manager reports every node active.
        (The server exists a second before bt_navigator is active; a goal sent then is rejected, which the
        explorer used to take for a tight spot and backed up into unmapped space.)"""
        if not self.running() or not self.client.server_is_ready():
            return False
        if self.nav_active:
            return True
        if not self._asking and self.active_cli.service_is_ready():
            self._asking = True
            fut = self.active_cli.call_async(Trigger.Request())

            def done(f):
                self._asking = False
                try:
                    self.nav_active = bool(f.result().success)
                except Exception:
                    pass
            fut.add_done_callback(done)
        return False

    def plan(self, x, y, yaw, timeout=4.0):
        """A path from where the car is to (x, y, yaw) without driving it: [[x, y, yaw], ...], or None."""
        if not self.running() or not self.plan_client.server_is_ready():
            return None
        g = ComputePathToPose.Goal()
        g.use_start, g.planner_id = False, 'GridBased'
        g.goal.header.frame_id = 'map'
        g.goal.pose.position.x, g.goal.pose.position.y = float(x), float(y)
        g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        done, box = threading.Event(), {}

        def on_result(f):
            try:
                box['path'] = f.result().result.path.poses
            except Exception:
                pass
            done.set()

        def on_goal(f):
            h = f.result()
            if not h or not h.accepted:
                done.set()
                return
            h.get_result_async().add_done_callback(on_result)
        self.plan_client.send_goal_async(g).add_done_callback(on_goal)
        if not done.wait(timeout) or not box.get('path'):
            return None
        return [[p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)] for p in box['path']]

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
    def set_pose(self, x, y, yaw, sigma_m=0.25, sigma_deg=15.0):
        if not self.running():
            return False, 'start navigation first'
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = sigma_m ** 2       # by hand: +-25 cm
        cov[35] = math.radians(sigma_deg) ** 2   # by hand: +-15 deg
        m.pose.covariance = cov
        for _ in range(3):
            self.pose_pub.publish(m)
            time.sleep(0.05)
        self.pose_set = True
        self.message = f'position set to ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f} deg; drive a little to let it settle'
        return True, self.message

    # ---- finding the position from the lidar (localize.py) ------------------------------
    def on_scan(self, msg):
        self.scan = (time.monotonic(), msg)

    def auto_find_tick(self):
        if (self.auto_find and self.ready() and not self.slam_mode and not self.pose_set
                and not self.finding and self.scan and time.monotonic() - self.scan[0] < SCAN_FRESH_S):
            self.auto_find = False
            threading.Thread(target=self.find_position, daemon=True).start()

    def find_position(self):
        """Find the car on the map from the current lidar scan; set it in AMCL if sure."""
        if not self.running() or self.slam_mode:
            return False, 'start navigation on a saved map first'
        if self.goal_active():
            return False, 'not while driving to a goal'
        if self.finding:
            return False, 'already looking'
        if not self.scan or time.monotonic() - self.scan[0] > SCAN_FRESH_S:
            return False, 'no lidar scan'
        self.finding = True
        try:
            msg = self.scan[1]
            mount = (0.0, 0.0, 0.0)
            try:
                t = self.tf.lookup_transform('base_footprint', msg.header.frame_id, Time())
                mount = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
            except Exception:
                pass
            pts = localize.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, mount)
            m = self.map_meta(self.map_name)
            grid = map_merge.from_png(map_store.png_bytes(self.map_name), m['resolution'], m['origin'][:2])
            res = localize.find(grid, pts)
            self.found = res
            if res['pose'] is None:
                self.message = f'could not find the position: {res["why"]}; set it by hand'
                return False, self.message
            x, y, yaw = res['pose']
            where = f'({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f} deg'
            if not res['sure']:
                self.message = (f'not sure where the car is (best guess {where}, shown dashed): {res["why"]}. '
                                f'Set the position by hand')
                self.node.get_logger().info(f'find position: not sure: {res}')
                return False, self.message
            self.set_pose(x, y, yaw, sigma_m=0.1, sigma_deg=5.0)
            self.message = (f'found its position on the map: {where} ({res["inliers"]:.0%} of the scan on '
                            f'walls, {res["took_s"]:.1f} s). Check the lidar dots line up with the walls')
            self.node.get_logger().info(f'find position: {res}')
            return True, self.message
        except Exception as exc:
            self.message = f'could not find the position: {exc}'
            return False, self.message
        finally:
            self.finding = False

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
    def go(self, x, y, yaw, retry=False):
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
        if not retry:
            self.goal_from, self.nudged = list(self.pose) if self.pose else None, 0
        else:
            self.goal_from = list(self.pose) if self.pose else None
        self.goal_seq += 1
        seq = self.goal_seq
        fut = self.client.send_goal_async(goal, feedback_callback=lambda m: self.on_feedback(m, seq))
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))
        self.message = f'going to ({x:.2f}, {y:.2f})'
        self.node.get_logger().info(self.message)
        return True, self.message

    # Answers for an older goal (replaced or canceled meanwhile) are ignored: seq != self.goal_seq.
    def on_goal_response(self, fut, seq):
        handle = fut.result()
        if seq != self.goal_seq or self.goal_state != 'sending':
            if handle.accepted and seq != self.goal_seq and self.goal_state in ('sending', 'active'):
                pass   # a newer goal preempts it in bt_navigator
            elif handle.accepted:
                handle.cancel_goal_async()   # canceled before it was accepted
            return
        if not handle.accepted:
            self.goal_state, self.message = 'rejected', 'goal rejected'
            return
        self.goal_handle, self.goal_state = handle, 'active'
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_feedback(self, msg, seq):
        if seq != self.goal_seq:
            return
        fb = msg.feedback
        self.feedback = {'distance_left': round(fb.distance_remaining, 2),
                         'recoveries': fb.number_of_recoveries,
                         'time_s': fb.navigation_time.sec}

    def on_result(self, fut, seq):
        if seq != self.goal_seq or self.goal_state != 'active':
            return
        status = fut.result().status
        self.goal_handle = None
        self.goal_state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                           GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'aborted')
        self.message = {'succeeded': 'arrived', 'canceled': 'goal canceled',
                        'aborted': 'could not reach the goal (no path, or stuck)'}[self.goal_state]
        self.node.get_logger().info(f'navigation goal {self.goal_state}')
        self.stop_motors()
        if self.goal_state == 'aborted' and not self.slam_mode and self.nudged < NUDGE_MAX and self._not_moved():
            self.nudged += 1
            self.goal_state = 'nudging'
            threading.Thread(target=self._nudge, args=(self.goal_seq,), daemon=True).start()

    # ---- tight start ----------------------------------------------------------------
    def _not_moved(self):
        a, b = self.goal_from, self.pose
        return a is not None and b is not None and math.hypot(a[0] - b[0], a[1] - b[1]) < 0.05

    def _clearance(self, center_deg, half_width_deg=25):
        if not self.scan or time.monotonic() - self.scan[0] > SCAN_FRESH_S:
            return None
        m = self.scan[1]
        r = np.asarray(m.ranges, dtype=float)
        a = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment)
        d = (a - center_deg + 180.0) % 360.0 - 180.0
        sel = (np.abs(d) <= half_width_deg) & np.isfinite(r) & (r > 0.05)
        return float(r[sel].min()) if sel.any() else 8.0

    def _nudge(self, seq):
        """Creep NUDGE_M ahead (or back) out of a tight start, then send the goal again."""
        room = {d: (self._clearance(d) or 0.0) - BUMPER[d] for d in (0, 180)}
        d = 0 if room[0] >= NUDGE_ROOM else 180 if room[180] >= NUDGE_ROOM else None
        if d is None:
            self.goal_state = 'aborted'
            self.message = (f'no path: the car stands in a tight spot and has no room to creep out '
                            f'(ahead {room[0]:.2f} m, behind {room[180]:.2f} m); move it by hand')
            return
        self.message = f'tight start: creeping {"forward" if d == 0 else "back"} {NUDGE_M:.2f} m, then trying again'
        self.node.get_logger().info(f'{self.message} (room ahead {room[0]:.2f} m, behind {room[180]:.2f} m)')
        start, t0, moved = self.pose, time.monotonic(), 0.0
        tw = Twist()
        tw.linear.x = NUDGE_SPEED if d == 0 else -NUDGE_SPEED
        while (self.goal_state == 'nudging' and seq == self.goal_seq and self.running()
               and time.monotonic() - t0 < NUDGE_M / NUDGE_SPEED + 3.0):
            c = self._clearance(d)
            if c is None or c - BUMPER[d] < 0.10:
                break
            if start and self.pose:
                moved = math.hypot(self.pose[0] - start[0], self.pose[1] - start[1])
                if moved >= NUDGE_M:
                    break
            self.cmd_pub.publish(tw)
            time.sleep(0.1)
        self.stop_motors()
        time.sleep(0.5)
        if self.goal_state != 'nudging' or seq != self.goal_seq:
            return                                 # canceled (E-STOP, joystick, Cancel) meanwhile
        self.node.get_logger().info(f'tight start: moved {moved:.2f} m, sending the goal again')
        self.goal_state = 'idle'
        ok, msg = self.go(*self.goal, retry=True)
        if not ok:
            self.goal_state, self.message = 'aborted', msg

    def cancel(self, reason='goal canceled'):
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            self.message = reason
        if self.goal_state in ('sending', 'active', 'yielding', 'nudging'):
            self.goal_state = 'canceled'
        self.goal_handle = None
        self.stop_motors()
        return True, reason

    def pause(self, reason):
        """Take the goal back from Nav2 and stop; resume() sends it again."""
        if not self.goal_active():
            return
        handle = self.goal_handle
        self.goal_seq += 1             # answers for the paused goal are ignored from now on
        if handle is not None:
            handle.cancel_goal_async()
        self.goal_handle, self.goal_state, self.message = None, 'yielding', reason
        self.stop_motors()

    def resume(self):
        if self.goal_state != 'yielding' or not self.goal:
            return False, 'nothing to resume'
        self.goal_state = 'idle'
        ok, msg = self.go(*self.goal)
        if not ok:
            self.goal_state = 'yielding'   # try again on the next tick
        return ok, msg

    # ---- the gate --------------------------------------------------------------
    def goal_active(self):
        return self.goal_state in ('sending', 'active') and self.running()

    def on_nav_cmd(self, msg):
        self.last_nav_cmd = time.monotonic()
        if not self.goal_active():
            return
        out = Twist()
        out.linear.x, out.angular.z, self.kappa = shape_steering(msg.linear.x, msg.angular.z, self.kappa)
        self.cmd_pub.publish(out)
        self.moving = abs(out.linear.x) > 1e-3

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
        if self.map_name and self.running():
            try:
                meta = self.map_meta(self.map_name)
            except (OSError, KeyError, ValueError):
                meta = None
        return {
            'running': self.running(),
            'slam_mode': self.slam_mode and self.running(),
            'uptime_s': round(time.time() - self.started_at) if self.running() else None,
            'map_name': self.map_name if self.running() else None,
            'map': meta if self.running() else None,
            'maps': self.maps(),
            'pose': self.pose,
            'pose_set': self.pose_set,
            'finding': self.finding,
            'found': self.found if self.running() and not self.slam_mode else None,
            'goal': self.goal,
            'goal_state': self.goal_state,
            'feedback': self.feedback,
            'plan': self.plan if self.goal_active() else [],
            'message': self.message,
        }
