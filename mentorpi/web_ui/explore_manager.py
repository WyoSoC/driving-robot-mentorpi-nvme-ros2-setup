"""Autonomous mapping of one car (runs inside web_ui_server.py): frontier exploration.

start(radius): starts SLAM (if not running) and Nav2 on the live SLAM map (nav.launch.py slam:=true), then
repeatedly drives to the best "frontier" (the edge between mapped free floor and unmapped space) until none
is left within `radius` m of the start, drives back to the start and saves the map
(explore_YYYYMMDD_HHMM_<car>, local time; the other cars copy it, see peer_manager.py).

Goals are only ever placed on mapped free floor, away from walls (CLEAR_OCC) and from unmapped space
(CLEAR_UNKNOWN), and the planner may not plan through unmapped space (allow_unknown: false).
Note: the lidar can't see a drop (stairs): the floor past a stair edge looks free. Use the radius as a fence.

Success is measured by what was mapped, not by arriving: a goal that is aborted but mapped new area
(>= GAIN_OK) counts as useful (only that goal point is avoided afterwards). A goal "fails" when it adds
nothing; then its whole frontier is skipped. The car is "stuck" only when it neither moves nor maps
anything for STUCK_S. Goals that need the car to turn around cost extra (TURN_COST), so it keeps going
roughly the way it faces instead of hunting left and right.

With another car mapping at the same time (team.py, once their live maps are lined up): frontiers are
picked on the team map (this car's map + what the other car mapped), so both cars see the same unmapped
edges and each can drive into the part the other mapped; frontiers near the other car's goal are left
to it (CLAIM_R), frontiers nearer to it than to this car cost extra (SHARE_COST), and at the end the
car whose name sorts first also saves the merged map (team_YYYYMMDD_HHMM).

It stops (navigation stopped, SLAM kept so the map can be saved) on: Stop, the joystick or E-stop (they
cancel the goal), low battery, charging mode, SLAM or navigation stopping, MAX_FAILS useless goals in a
row, or MAX_TIME.
"""
import math
import threading
import time
from datetime import datetime

import numpy as np
from geometry_msgs.msg import Twist

import car_geometry
import map_merge
import map_store
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan

FREE_MAX = 25            # occupancy 0..100: below = free
OCC_MIN = 65             # at or above = wall / obstacle
CLEAR_OCC = 0.30         # m, goal at least this far from walls
CLEAR_UNKNOWN = 0.15     # m, and from unmapped space
PASS_OCC = 0.14          # m, cells the car can pass (connectivity check)
GOAL_NEAR = 1.0          # m, goal at most this far from its frontier
MIN_FRONTIER = 0.50      # m, smaller frontiers are ignored (noise, slow to chase)
MIN_GOAL_DIST = 0.70     # m, closer goals: short awkward hops for a car
BLACKLIST_R = 0.5        # m, around failed goals
GOAL_TIMEOUT = 180       # s per goal, unless it keeps mapping new area
STUCK_S = 40             # s without moving STUCK_M and without mapping GAIN_STEP -> give up on the goal
STUCK_M = 0.15
GAIN_OK = 0.5            # m2 newly mapped while going for a goal: the attempt was useful
GAIN_STEP = 0.2          # m2 newly mapped counts as progress (not stuck)
PLAN_TOP = 5             # the best frontiers by straight-line score get a real plan (route_cost) ...
PLAN_BUDGET_S = 8.0      # ... within this time
REV_COST = 2.0           # m extra per m the route drives in reverse (the car can't see behind)
CUSP_COST = 1.0          # m extra per forward / reverse switch on the route
TURN_COST = 0.8          # m of extra distance per radian the car must turn towards a goal
FAILED_FRONTIER_R = 0.8  # m, frontiers whose centre is this close to a failed one are skipped
MAX_FAILS = 6            # failed goals in a row -> give up
MAX_TIME = 20 * 60       # s
START_TIMEOUT = 90       # s for SLAM + Nav2 to come up
NO_FRONTIER_CHECKS = 3   # consecutive empty checks -> done
STEP_S = 1.0
SMOOTH_CELLS = 2          # closing iterations on the free space
# Escape: when goals fail while the car stands still in a tight spot (the planner won't start from there),
# drive straight out (backwards if there is room behind, else forwards) before trying the next goal.
ESCAPE_M = 0.25          # m to drive straight
ESCAPE_SPEED = 0.06      # m/s
ESCAPE_CLEAR = 0.30      # m free needed in the driving direction (lidar, from the lidar centre + body)
ESCAPE_MAX = 3           # escapes in a row without any goal succeeding -> stop escaping
RETRACE_M = 0.8          # m to drive back along the car's own trail (known to fit) before replanning
RETRACE_LOOKAHEAD = 0.30 # m, pure pursuit on the trail
MIN_TURN_R = 0.40        # m, tightest turn of the car
JOIN_WAIT_S = 30.0       # s that cars started together wait for their maps to line up before driving
CLAIM_R = 1.0            # m, frontiers this close to another car's goal are left to it
SHARE_COST = 1.0         # extra cost per m that a frontier is closer to another car than to this one
SCAN_TOPIC = '/scan_raw'  # lidar_frame = base_footprint orientation (0 = forward, counter-clockwise)


def frontier_goals(grid, res, origin, car_xy, fence_xy, radius, blacklist, failed_centers=(), car_yaw=None,
                   claims=()):
    """Frontier clusters and a goal for each. grid: int8 [h, w] (-1 unknown, 0..100), row 0 = min y.
    grid is the team map when mapping together (team.py). claims: other
    cars [{'pose': [x, y], 'goal': [x, y] or None}] in this frame.
    Returns ([{'goal': [x, y, yaw], 'center': [x, y], 'size': m, 'dist': m}], frontier cells [[x, y], ...])."""
    h, w = grid.shape
    occ = grid >= OCC_MIN
    # Close the thin unmapped gaps between lidar rays (up to ~4 cells), so the frontier is the outer edge of
    # the mapped floor, not streaks inside it. Walls stay walls.
    free = ndimage.binary_closing((grid >= 0) & (grid < FREE_MAX), structure=np.ones((3, 3)), iterations=SMOOTH_CELLS)
    free &= ~occ
    unknown = ~free & ~occ & (grid < 0)
    to_xy = lambda r, c: (origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res)
    cross = ndimage.generate_binary_structure(2, 1)
    frontier = free & ndimage.binary_dilation(unknown, structure=cross)

    d_occ = ndimage.distance_transform_edt(~occ) * res
    d_unk = ndimage.distance_transform_edt(~unknown) * res
    passable = free & (d_occ >= PASS_OCC)
    safe = passable & (d_occ >= CLEAR_OCC) & (d_unk >= CLEAR_UNKNOWN)

    # cells reachable from the car (connected passable floor)
    cr = int((car_xy[1] - origin[1]) / res)
    cc = int((car_xy[0] - origin[0]) / res)
    labels, _ = ndimage.label(passable)
    car_label = 0
    if 0 <= cr < h and 0 <= cc < w:
        r0, r1, c0, c1 = max(cr - 10, 0), min(cr + 11, h), max(cc - 10, 0), min(cc + 11, w)
        win = labels[r0:r1, c0:c1]
        if win.any():   # nearest passable cell within 0.5 m (the car itself may stand close to a wall)
            rr, ccs = np.nonzero(win)
            k = np.argmin((rr + r0 - cr) ** 2 + (ccs + c0 - cc) ** 2)
            car_label = win[rr[k], ccs[k]]
    if not car_label:
        return [], []
    reach = safe & (labels == car_label)

    clusters, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    if n == 0:
        return [], []
    sizes = ndimage.sum(frontier, clusters, index=np.arange(1, n + 1))
    centers = ndimage.center_of_mass(frontier, clusters, index=np.arange(1, n + 1))
    out, cells = [], []
    rows_all, cols_all = np.nonzero(frontier)
    for k in range(n):
        size_m = sizes[k] * res
        if size_m < MIN_FRONTIER:
            continue
        cy, cx = to_xy(*centers[k])[1], to_xy(*centers[k])[0]
        if radius and math.hypot(cx - fence_xy[0], cy - fence_xy[1]) > radius:
            continue
        if any(math.hypot(cx - fx, cy - fy) < FAILED_FRONTIER_R for fx, fy in failed_centers):
            continue
        if any(c.get('goal') and math.hypot(cx - c['goal'][0], cy - c['goal'][1]) < CLAIM_R for c in claims):
            continue
        mask = clusters == k + 1
        # goal: a reachable safe cell close to this frontier, the one nearest to the car
        rows, cols = np.nonzero(mask)
        r0, r1 = max(rows.min() - 25, 0), min(rows.max() + 26, h)
        c0, c1 = max(cols.min() - 25, 0), min(cols.max() + 26, w)
        d_front = ndimage.distance_transform_edt(~mask[r0:r1, c0:c1]) * res
        cand = reach[r0:r1, c0:c1] & (d_front <= GOAL_NEAR)
        if not cand.any():
            continue
        cr_, cc_ = np.nonzero(cand)
        gx = origin[0] + (cc_ + c0 + 0.5) * res
        gy = origin[1] + (cr_ + r0 + 0.5) * res
        dcar = np.hypot(gx - car_xy[0], gy - car_xy[1])
        ok = dcar >= MIN_GOAL_DIST
        for bx, by in blacklist:
            ok &= np.hypot(gx - bx, gy - by) > BLACKLIST_R
        if radius:
            ok &= np.hypot(gx - fence_xy[0], gy - fence_xy[1]) <= radius
        if not ok.any():
            continue
        # prefer close to the frontier (sees into it), then close to the car
        score = d_front[cr_, cc_] * 2.0 + dcar * 0.3
        score[~ok] = np.inf
        i = int(np.argmin(score))
        x, y = float(gx[i]), float(gy[i])
        # arrive the way the car naturally comes in (from where it is), not facing the unmapped space: the lidar
        # sees all round, and a fixed final heading made the planner add turns / reversing near the goal
        yaw = math.atan2(y - car_xy[1], x - car_xy[0]) if math.hypot(y - car_xy[1], x - car_xy[0]) > 0.3 \
            else (car_yaw if car_yaw is not None else math.atan2(cy - y, cx - x))
        turn = 0.0
        if car_yaw is not None:
            d = math.atan2(y - car_xy[1], x - car_xy[0]) - car_yaw
            turn = abs(math.atan2(math.sin(d), math.cos(d)))
        share = 0.0
        for c in claims:     # nearer to another car than to this one: rather its job
            mine_d = math.hypot(cx - car_xy[0], cy - car_xy[1])
            share = max(share, mine_d - math.hypot(cx - c['pose'][0], cy - c['pose'][1]))
        out.append({'goal': [round(x, 3), round(y, 3), round(yaw, 3)], 'center': [round(cx, 3), round(cy, 3)],
                    'size': round(size_m, 2), 'dist': round(float(dcar[i]), 2), 'turn': round(turn, 2),
                    'share': round(share, 2)})
        sel = mask[rows_all, cols_all]
        for r, c in list(zip(rows_all[sel], cols_all[sel]))[::3]:
            x_, y_ = to_xy(r, c)
            cells.append([round(x_, 2), round(y_, 2)])
    # best first: close, ahead of the car, large
    out.sort(key=lambda f: f['dist'] + TURN_COST * f['turn'] - 0.5 * min(f['size'], 3.0) + SHARE_COST * f['share'])
    return out, cells


def route_cost(path):
    """Planned path [[x, y, yaw], ...] -> {'length', 'reverse', 'cusps', 'cost'} (cost in m: length plus
    REV_COST per m in reverse plus CUSP_COST per forward / reverse switch)."""
    p = np.asarray(path, dtype=float)
    if len(p) < 2:
        return {'length': 0.0, 'reverse': 0.0, 'cusps': 0, 'cost': 0.0}
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    mv = seg > 1e-4
    fwd = (d[:, 0] * np.cos(p[:-1, 2]) + d[:, 1] * np.sin(p[:-1, 2]))[mv] > 0
    length, reverse = float(seg.sum()), float(seg[mv][~fwd].sum())
    cusps = int(np.sum(fwd[1:] != fwd[:-1]))
    return {'length': round(length, 2), 'reverse': round(reverse, 2), 'cusps': cusps,
            'cost': round(length + REV_COST * reverse + CUSP_COST * cusps, 2)}


class Explorer:
    def __init__(self, node, slam, nav, car='car'):
        self.node, self.slam, self.nav, self.car = node, slam, nav, car
        self.team = None           # team.Team, set by the web server (mapping together)
        self.team_saved_as = None
        self.log = node.get_logger()
        self.lock = threading.Lock()
        self.state = 'off'      # off | starting | exploring | returning | done | stopped
        self.message = ''
        self.radius = None
        self.home = None
        self.goal = None
        self.goal_sent_at = None
        self.frontiers, self.cells = [], []
        self.blacklist, self.visited, self.failed_centers = [], [], []
        self.goal_center = None
        self.progress_at, self.progress_xy = None, None
        self.progress_known = self.known_at_send = 0.0
        self.reached = self.partial = self.failed = 0
        self.send_xy = None
        self.mapped_m2 = 0.0
        self.fails = 0
        self.empty_checks = 0
        self.quick_retries = 0
        self.started_at = None
        self.saved_as = None
        self.own_cancel = False
        self.dry = False
        self.scan = None
        self.escapes = 0
        self.unreached = 0
        self.rejected = 0
        self.lifted = False        # lift_detector: the car is being carried
        self.recovering = False    # Back out while not exploring
        self.backout_req = False   # Back out asked for while exploring
        self.bump_req = 0          # stall_detector: pushed against something ahead (+1) / behind (-1)
        self.blocked_at = 0.0
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 1)
        node.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        threading.Thread(target=self._loop, daemon=True).start()

    def active(self):
        return self.state in ('starting', 'exploring', 'returning')

    # ---- control --------------------------------------------------------------
    def start(self, radius, dry=False, wait_join=False):
        """dry: everything except sending goals (test without driving; API only). wait_join: started
        together with other cars: map in place until the shared map has joined (JOIN_WAIT_S at most)."""
        with self.lock:
            if self.active():
                return False, 'already exploring'
            if self.node.charging != 'off':
                return False, 'turn charging mode off first'
            if self.nav.running() and not self.nav.slam_mode:
                return False, 'stop navigation first'
            batt = self.node.battery_st or {}
            if batt.get('level') in ('low', 'critical'):
                return False, 'battery low: charge first'
            if not self.slam.running():
                self.slam.start()
            self.radius = radius or None
            self.dry = dry
            self.wait_join, self.ready_at = wait_join, None
            self.home = None
            self.goal, self.frontiers, self.cells = None, [], []
            self.blacklist, self.visited, self.failed_centers = [], [], []
            self.goal_center = None
            self.reached = self.partial = self.failed = 0
            self.escapes = self.unreached = 0
            self.rejected = 0
            self.send_xy = None
            self.fails = self.empty_checks = self.quick_retries = 0
            self.goal_sent_at = None
            self.saved_as = self.team_saved_as = None
            self.started_at = time.monotonic()
            self.state = 'starting'
            self.message = 'starting mapping and navigation (~30 s)'
            self.log.info(f'exploration started (radius {radius or "unlimited"} m)')
            return True, self.message

    def on_lifted(self, note):
        """lift_detector.py: the car was picked up. Drop the goal, wait until it is set down."""
        self.lifted = True
        self.recovering = False
        if self.active():
            if self.goal is not None or self.nav.goal_active():
                self.own_cancel = True
                self.nav.cancel('lifted')
                self.goal = None
            self.message = note

    def on_put_down(self):
        self.lifted = False
        if self.active():
            self.progress_at = time.monotonic()
            self.message = 'set down again: going on'
            if self.state == 'returning':
                self._send(self.home)

    def bumped(self, direction, note):
        """stall_detector.py: the car pushed against something (direction +1 ahead / -1 behind) and the motors
        are off. Back out the other way and go on."""
        self.blocked_at = time.monotonic()            # an escape move that hit something ends now
        self.log.info(f'{note}: backing out')
        if self.active():
            self.bump_req = direction                 # handled in _step (cancels the goal first)
            self.message = note + ': backing out'
        elif self.nav.goal_active() and not self.recovering:
            self.nav.pause(note)                      # a navigation goal: back out, then resume it
            self.recovering = True
            threading.Thread(target=self._bump_out_nav, args=(direction, note), daemon=True).start()

    def _bump_out_nav(self, direction, note):
        try:
            time.sleep(1.1)                           # let blocked_at expire
            moved = self._straight_out(prefer=180 if direction > 0 else 0)
        finally:
            self.recovering = False
            self.cmd_pub.publish(Twist())
        self.log.info(f'{note}: {"backed out" if moved else "no room to back out"}, resuming the goal')
        self.nav.resume()

    def back_out(self):
        """Back out (Map panel): drive back along the car's own trail, else straight back or forward, slowly,
        watching the lidar. While exploring: the current goal is dropped, then exploring goes on. Otherwise
        (stopped, done, or just mapping): only the move. E-STOP, the joystick or Stop ends it."""
        if not self.slam.running() or self.slam.pose is None:
            return False, 'start mapping first (the car needs its position and trail)'
        if self.recovering or self.backout_req:
            return False, 'already backing out'
        if self.active():
            self.backout_req = True
            return True, 'backing out, then exploring goes on'
        if self.nav.goal_active():
            self.nav.cancel('backing out')
        self.recovering = True
        threading.Thread(target=self._back_out_alone, daemon=True).start()
        return True, 'backing out'

    def _back_out_alone(self):
        try:
            moved = self._escape(force=True)
            self.message = 'backed out' if moved else 'could not back out: no room behind or ahead (lidar)'
        finally:
            self.recovering = False
            self.cmd_pub.publish(Twist())

    def stop(self, reason='exploration stopped'):
        self.recovering = False
        self.backout_req = False
        self.bump_req = 0
        if self.active():
            self.mapped_m2 = self._known_m2()
        with self.lock:
            was = self.active()
            self.state = 'stopped' if was else self.state
            self.message = reason + ('; mapping still running: save the map' if self.slam.running() else '')
        if self.nav.goal_active():
            self.own_cancel = True
            self.nav.cancel(reason)
        if self.nav.running() and self.nav.slam_mode:
            self.nav.stop()
        if was:
            self.log.info(f'exploration stopped: {reason}')
        return True, self.message

    # ---- loop -------------------------------------------------------------------
    def _loop(self):
        while True:
            time.sleep(STEP_S)
            try:
                self._step()
            except Exception as exc:          # keep the thread alive; stop the car
                self.log.error(f'exploration error: {exc!r}')
                if self.active():
                    self.stop(f'exploration error: {exc}')

    def _on_scan(self, msg):
        self.scan = msg

    def _clearance(self, center_deg, half_width_deg=25):
        """Smallest lidar range within +-half_width of a direction (deg, 0 = forward), or None."""
        s = self.scan
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=float)
        a = np.degrees(s.angle_min + np.arange(len(r)) * s.angle_increment)
        d = (a - center_deg + 180.0) % 360.0 - 180.0
        sel = (np.abs(d) <= half_width_deg) & np.isfinite(r) & (r > 0.05)
        return float(r[sel].min()) if sel.any() else None

    def _escape(self, force=False, prefer=None):
        """Get out of a tight spot (the planner won't start there: 'Starting point in lethal space', or the
        car stands still in front of something): first drive back along the car's own trail (it came in
        that way, so it fits), else straight. force: asked for (Back out), also after ESCAPE_MAX.
        Returns True if the car moved."""
        if self.escapes >= ESCAPE_MAX and not force:
            return False
        if prefer == 0:                               # hit something behind: get away forwards
            return self._straight_out(prefer=0)
        if self._retrace():
            self.escapes += 1
            return True
        return self._straight_out()

    def _may_drive(self):
        return self.active() or self.recovering

    def _stuck_recovery(self, why):
        """A goal failed and the car didn't move: that is the car's spot, not the frontier's fault. Back out
        and go on (the frontier stays a target). After ESCAPE_MAX escapes in a row that didn't get the car
        anywhere: stop and ask for help, instead of writing off every frontier. True = handled."""
        goal = self.goal
        if self._escape():
            self.log.info(f'exploration goal {goal} failed ({why}) where the car stood: backed out, going on')
            self.goal = None
            if self.state == 'returning':
                self._send(self.home)
            return True
        if self.escapes >= ESCAPE_MAX:
            self.goal = None
            self.stop(f'stuck: backed out {ESCAPE_MAX} times without getting anywhere. Press "Back out" '
                      f'or move the car, then start again')
            return True
        return False                                   # no room to move: count it as a failed goal

    def _drive_ok(self, direction_deg, body):
        """False when someone drives with the joystick, exploring / Back out stopped, the car just bumped into
        something (stall_detector.py), or an obstacle is close."""
        if not self._may_drive() or self.lifted or time.monotonic() - self.blocked_at < 1.0:
            return False
        if time.monotonic() - self.node.last_cmd_time < 1.0 and self.node.moving:
            return False
        c = self._clearance(direction_deg)
        return c is None or c - body >= 0.10

    def _retrace(self):
        """Reverse (pure pursuit) along the last RETRACE_M of the trail the car drove."""
        trail, pose = list(self.slam.path), self.slam.pose
        if len(trail) < 4 or pose is None:
            return False
        # Walk back along the trail, keeping only points that lead steadily away from here: the back-and-
        # forth wiggles of the last manoeuvres are skipped, the way the car came in remains.
        pts, dist, far = [[pose[0], pose[1]]], 0.0, 0.0
        for p in reversed(trail):
            away = math.hypot(p[0] - pose[0], p[1] - pose[1])
            if away < far + 0.03:
                continue
            far = away
            dist += math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1])
            pts.append(p)
            if far >= RETRACE_M:
                break
        if dist < 0.2:
            return False
        self.message = f'tight spot: backing out along its own trail ({dist:.1f} m)'
        self.log.info(f'escape {self.escapes + 1}: {self.message}')
        start, t0, idx, moved = pose, time.monotonic(), 0, 0.0
        while time.monotonic() - t0 < dist / ESCAPE_SPEED + 6.0:
            pose = self.slam.pose
            if pose is None:
                break
            moved = math.hypot(pose[0] - start[0], pose[1] - start[1])
            # nearest trail point from where we are (never go back to earlier ones)
            while idx + 1 < len(pts) and math.hypot(pts[idx + 1][0] - pose[0], pts[idx + 1][1] - pose[1]) <= \
                    math.hypot(pts[idx][0] - pose[0], pts[idx][1] - pose[1]):
                idx += 1
            end = pts[-1]
            if math.hypot(end[0] - pose[0], end[1] - pose[1]) < 0.08 or idx == len(pts) - 1:
                break
            tgt = end
            for q in pts[idx:]:
                if math.hypot(q[0] - pose[0], q[1] - pose[1]) >= RETRACE_LOOKAHEAD:
                    tgt = q
                    break
            dx, dy = tgt[0] - pose[0], tgt[1] - pose[1]
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            xr, yr = c * dx + sn * dy, -sn * dx + c * dy          # target in the car's frame
            v = -ESCAPE_SPEED if xr < 0 else ESCAPE_SPEED          # the trail is normally behind
            direction = 180 if v < 0 else 0
            body = car_geometry.BUMPER[direction]
            if not self._drive_ok(direction, body):
                break
            kappa = 2.0 * yr / max(xr * xr + yr * yr, 1e-6)
            kappa = max(-1.0 / MIN_TURN_R, min(1.0 / MIN_TURN_R, kappa))
            twist = Twist()
            twist.linear.x, twist.angular.z = v, v * kappa
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self._halt()
        self.log.info(f'escape done (trail): moved {moved:.2f} m')
        return moved > 0.1

    def _halt(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)
        self.cmd_pub.publish(Twist())

    def _straight_out(self, prefer=None):
        """Drive straight back (or forward) ESCAPE_M if the lidar shows room. prefer: 0 forward / 180 back."""
        back, front = self._clearance(180), self._clearance(0)
        body = car_geometry.BUMPER          # lidar centre -> front / rear bumper
        options = [(c - body[d], d) for c, d in ((back, 180), (front, 0)) if c is not None]
        options = [o for o in options if o[0] >= ESCAPE_CLEAR and self._known_free(o[1], ESCAPE_M + 0.10)]
        if not options:
            self.log.info(f'escape: no room or not mapped (behind {back}, ahead {front})')
            return False
        if prefer is not None and any(d == prefer for _, d in options):
            room, direction = next(o for o in options if o[1] == prefer)
        else:
            room, direction = max(options) if not any(d == 180 for _, d in options) else \
                next(o for o in options if o[1] == 180)        # prefer backing out the way it came
        sign = -1.0 if direction == 180 else 1.0
        self.escapes += 1
        self.message = f'tight spot: driving {"back" if sign < 0 else "forward"} {ESCAPE_M:.2f} m to get out'
        self.log.info(f'escape {self.escapes}: {self.message} (room {room:.2f} m)')
        start = self.slam.pose
        t0 = time.monotonic()
        moved = 0.0
        twist = Twist()
        twist.linear.x = sign * ESCAPE_SPEED
        while self._may_drive() and time.monotonic() - self.blocked_at > 1.0 and \
                time.monotonic() - t0 < ESCAPE_M / ESCAPE_SPEED + 3.0:
            if time.monotonic() - self.node.last_cmd_time < 1.0 and self.node.moving:
                break                                   # someone is driving with the joystick
            c = self._clearance(direction)
            if c is not None and c - body[direction] < 0.12:
                break
            pose = self.slam.pose
            if start and pose:
                moved = math.hypot(pose[0] - start[0], pose[1] - start[1])
                if moved >= ESCAPE_M:
                    break
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        self.cmd_pub.publish(Twist())
        self.log.info(f'escape done: moved {moved:.2f} m')
        return moved > 0.05

    def _known_free(self, direction_deg, dist):
        """Is the strip the car would sweep driving `dist` m straight ahead (0) or back (180) mapped as free on
        its own map? The escape never drives into unmapped space (behind the car at the start: usually a wall)."""
        g, pose = self._grid(own=True), self.slam.pose
        if g is None or pose is None:
            return False
        grid, res, origin = g
        sign = 1.0 if direction_deg == 0 else -1.0
        start = car_geometry.FRONT if sign > 0 else -car_geometry.BACK
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        h, w = grid.shape
        for along in np.arange(start, start + dist + 1e-6, res / 2):
            for across in (-car_geometry.HALF_WIDTH, 0.0, car_geometry.HALF_WIDTH):
                bx, by = sign * along, across
                x, y = pose[0] + c * bx - s_ * by, pose[1] + s_ * bx + c * by
                col, row = int((x - origin[0]) / res), int((y - origin[1]) / res)
                if not (0 <= row < h and 0 <= col < w) or not (0 <= grid[row, col] < FREE_MAX):
                    return False
        return True

    def _known_m2(self):
        g = self._grid(own=True)
        if g is None:
            return 0.0
        return float(np.count_nonzero(g[0] >= 0)) * g[1] * g[1]

    def _grid(self, own=False):
        """The team map (team.py: this car's map + what the other cars mapped), or this car's own map.
        Own progress (mapped m2) is measured on the own map."""
        msg = self.slam.grid
        if not own and self.team is not None and self.team.team_msg is not None:
            msg = self.team.team_msg
        if msg is None:
            return None
        g = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        return g, msg.info.resolution, (o.x, o.y)

    def _step(self):
        if not self.active():
            return
        now = time.monotonic()
        # safety / preconditions
        if self.node.charging != 'off':
            return self.stop('charging mode')
        if (self.node.battery_st or {}).get('level') in ('low', 'critical'):
            return self.stop('battery low')
        if not self.slam.running():
            return self.stop('mapping stopped')
        if now - self.started_at > MAX_TIME:
            return self.stop(f'time limit ({MAX_TIME // 60} min)')
        if self.lifted:                                       # carried by a person (lift_detector): wait
            return
        if self.bump_req and self.state != 'starting':         # pushed against something (stall_detector)
            direction, self.bump_req = self.bump_req, 0
            if self.goal is not None or self.nav.goal_active():
                self.own_cancel = True
                self.nav.cancel('bumped into something')
                self.goal = None
            time.sleep(max(0.0, 1.1 - (time.monotonic() - self.blocked_at)))
            moved = self._escape(force=True, prefer=180 if direction > 0 else 0)
            self.message = ('bumped into something: backed out, going on' if moved
                            else 'bumped into something and no room to back out (lidar)')
            if self.state == 'returning':
                self._send(self.home)
            return
        if self.backout_req and self.state != 'starting':      # Back out pressed on the page
            self.backout_req = False
            if self.goal is not None or self.nav.goal_active():
                self.own_cancel = True
                self.nav.cancel('backing out')
                self.goal = None
            moved = self._escape(force=True)
            self.message = ('backed out, going on' if moved else 'could not back out: no room behind or ahead (lidar)')
            if self.state == 'returning':
                self._send(self.home)
            return

        if self.state == 'starting':
            if not self.nav.running():
                if not self.slam.grid or self.slam.pose is None:
                    if now - self.started_at > START_TIMEOUT:
                        return self.stop('mapping did not start')
                    return
                ok, msg = self.nav.start_on_slam()
                if not ok:
                    return self.stop(msg)
                return
            if not self.nav.ready():
                if now - self.started_at > START_TIMEOUT:
                    return self.stop('navigation did not start')
                return
            if self.wait_join and not self._joined():
                self.ready_at = self.ready_at or now
                if now - self.ready_at < JOIN_WAIT_S:
                    self.message = (f'waiting for the other car\'s map to line up '
                                    f'({JOIN_WAIT_S - (now - self.ready_at):.0f} s)')
                    return
            self.home = list(self.slam.pose)
            self.state = 'exploring'
            self.message = ('exploring' if not self.wait_join or self._joined()
                            else 'exploring alone until the maps line up')
            return

        if not self.nav.running():
            return self.stop('navigation stopped')
        gs = self.nav.goal_state
        pose = self.slam.pose

        if self.goal is not None:
            if gs == 'yielding':          # waiting for another car (fleet.py): not stuck
                self.progress_at = now
                return
            if gs in ('sending', 'active'):
                known = self._known_m2()
                moved = pose is not None and math.hypot(pose[0] - self.progress_xy[0],
                                                        pose[1] - self.progress_xy[1]) > STUCK_M
                if moved or known - self.progress_known > GAIN_STEP:
                    self.progress_xy = pose[:2] if pose is not None else self.progress_xy
                    self.progress_at, self.progress_known = now, known
                if now - self.progress_at > STUCK_S:
                    self.own_cancel = True
                    self.nav.cancel('stuck')
                    if self._stuck_recovery(f'stuck (no movement, nothing new mapped for {STUCK_S} s)'):
                        return
                    self._failed(f'stuck (no movement, nothing new mapped for {STUCK_S} s)')
                    if self.state == 'returning':
                        return self._finish()
                elif now - self.goal_sent_at > GOAL_TIMEOUT:
                    self.own_cancel = True
                    self.nav.cancel('goal took too long')
                    self._failed('took too long')
                elif self.state == 'exploring' and int(now) % 3 == 0:
                    self._check_goal_still_useful(pose)
                return
            if gs == 'canceled':
                if self.own_cancel:
                    self.own_cancel = False
                    self.goal = None
                else:   # the person at the controls (joystick / E-stop / Cancel goal)
                    return self.stop('stopped by you')
            elif gs == 'rejected':      # navigation not accepting goals yet: not a tight spot, try again
                self.goal = None
                self.rejected += 1
                if self.rejected >= 20:
                    return self.stop('navigation keeps rejecting goals')
                self.message = 'navigation not ready yet, trying again'
                return
            elif gs == 'aborted':
                moved = pose is not None and self.send_xy is not None and \
                    math.hypot(pose[0] - self.send_xy[0], pose[1] - self.send_xy[1]) > STUCK_M
                stuck_here = not moved and self._known_m2() - self.known_at_send < GAIN_OK
                was_home = self.state == 'returning'
                if stuck_here and self._stuck_recovery('could not reach it'):
                    return
                self._failed('could not reach it')
                if was_home:
                    return self._finish()
            elif gs == 'succeeded':
                if self.state == 'returning':
                    return self._finish()
                self.fails = 0
                self.quick_retries = 0
                self.reached += 1
                self.escapes = 0
                self.visited.append(self.goal[:2])
                self.goal = None
            else:
                return   # idle: the goal response has not arrived yet
            if self.fails >= MAX_FAILS:
                return self.stop(f'{MAX_FAILS} goals in a row could not be reached')
            if self.state == 'returning' and self.goal is None:   # could not get home: finish where we are
                return self._finish()

        if self.state == 'returning':
            return
        # pick the next frontier
        g = self._grid()
        if g is None or pose is None:
            return
        grid, res, origin = g
        claims = self._claims()
        fronts, cells = frontier_goals(grid, res, origin, pose[:2], self.home[:2], self.radius,
                                       self.blacklist + self.visited, self.failed_centers,
                                       car_yaw=pose[2], claims=claims)
        self.frontiers, self.cells = fronts, cells
        if not fronts:
            self.empty_checks += 1
            self.message = f'no frontier left? checking ({self.empty_checks}/{NO_FRONTIER_CHECKS})'
            if self.empty_checks >= NO_FRONTIER_CHECKS:
                left = f' ({len(self.failed_centers)} unmapped edges could not be reached)' if self.failed_centers else ''
                self.state, self.message = 'returning', f'map complete{left}: driving back to the start'
                self.goal_center = None
                self._send(self.home)
            return
        self.empty_checks = 0
        f = self._choose(fronts)
        self.goal_center = f['center']
        if self.dry:
            self.message = f'dry run: would go to {f["goal"]} ({len(fronts)} frontiers)'
            self.dry_goal = f['goal']
            return
        self.message = f'to frontier {f["size"]:.1f} m wide, {f["dist"]:.1f} m away ({len(fronts)} left)'
        self._send(f['goal'])

    def _choose(self, fronts):
        """Of the PLAN_TOP best frontiers (straight-line score), the one with the cheapest real route: the
        planner's path (no driving) costed by length, reverse driving and forward / reverse switches
        (route_cost). In tight spaces a frontier just behind the car or one needing a three-point turn
        loses to one the car can drive to forwards. No plan for any: the first (navigation decides)."""
        t0, best = time.monotonic(), None
        for f in fronts[:PLAN_TOP]:
            if time.monotonic() - t0 > PLAN_BUDGET_S:
                break
            path = self.nav.plan(*f['goal'])
            if not path:
                continue
            rc = route_cost(path)
            score = rc['cost'] - 0.5 * min(f['size'], 3.0) + SHARE_COST * f.get('share', 0.0)
            f['route'] = rc
            if best is None or score < best[0]:
                best = (score, f)
        if best is None:
            return fronts[0]
        f = best[1]
        self.log.info(f'next frontier: route {f["route"]["length"]} m, {f["route"]["reverse"]} m reverse, '
                      f'{f["route"]["cusps"]} switches ({time.monotonic() - t0:.1f} s to compare)')
        return f

    def _send(self, goal):
        self.own_cancel = False
        ok, msg = self.nav.go(*goal)
        if not ok:
            self._failed(msg)
            return
        self.goal, self.goal_sent_at, self.own_cancel = list(goal), time.monotonic(), False
        self.progress_at, self.progress_xy = time.monotonic(), (self.slam.pose or goal)[:2]
        self.progress_known = self.known_at_send = self._known_m2()
        self.send_xy = self.slam.pose[:2] if self.slam.pose else None

    def _failed(self, why):
        quick = self.goal_sent_at is not None and time.monotonic() - self.goal_sent_at < 3.0
        gain = self._known_m2() - self.known_at_send if self.goal is not None else 0.0
        if self.goal is not None and not quick and gain >= GAIN_OK:
            # did not arrive, but mapped new area on the way: a useful attempt. Avoid only this goal point.
            self.blacklist.append(self.goal[:2])
            self.partial += 1
            self.fails = 0
            self.escapes = 0
            self.log.info(f'exploration goal {self.goal} not reached ({why}) but mapped {gain:.1f} m2: useful')
            self.goal = None
            return
        if self.goal is not None and quick and self.quick_retries < 2:
            # failed at once: usually navigation had no map yet. Try again, don't blacklist.
            self.quick_retries += 1
            self.log.info(f'exploration goal {self.goal} failed at once ({why}): retrying')
            self.goal = None
            return
        if self.goal is not None:
            self.blacklist.append(self.goal[:2])
            if self.goal_center:
                self.failed_centers.append(self.goal_center)
            self.failed += 1
            self.unreached += 1
            self.log.info(f'exploration goal {self.goal} failed ({why}), nothing new mapped: skipping its frontier')
        self.fails += 1
        self.goal = None

    def _check_goal_still_useful(self, pose):
        """While driving: if the frontier we drive to has been mapped meanwhile, go to the next one."""
        g = self._grid()
        if g is None or pose is None:
            return
        grid, res, origin = g
        claims = self._claims()
        fronts, cells = frontier_goals(grid, res, origin, pose[:2], self.home[:2], self.radius,
                                       self.blacklist + self.visited, self.failed_centers,
                                       car_yaw=pose[2])
        self.frontiers, self.cells = fronts, cells
        gx, gy = self.goal[0], self.goal[1]
        if not any(math.hypot(f['goal'][0] - gx, f['goal'][1] - gy) < 1.0 for f in fronts):
            self.own_cancel = True
            self.nav.cancel('frontier mapped, next one')
            self.visited.append(self.goal[:2])
            self.goal = None
            return
        # both cars chose the same frontier: the one whose name sorts first keeps it
        c = self.goal_center
        for cl in claims:
            if c and cl.get('goal') and cl['car'] < self.car and math.hypot(c[0] - cl['goal'][0], c[1] - cl['goal'][1]) < CLAIM_R:
                self.own_cancel = True
                self.nav.cancel(f'{cl["car"]} is mapping there, next one')
                self.goal = None
                return

    def _joined(self):
        sh = self.team.shared if self.team is not None else None
        return sh is not None and sh.joined()

    def _claims(self):
        if self.team is None or not self.team.matched():
            return ()
        try:
            return self.team.claims()
        except Exception as exc:
            self.log.warn(f'team claims: {exc}')
            return ()

    def _save_team_map(self):
        """Merged map of all matched cars; saved by the car whose name sorts first."""
        if self.team is None or self.slam.grid is None:
            return None
        mates = self.team.matched()
        if not mates or any(m.car < self.car for m in mates):
            return None
        from team import grid_from_msg
        sh = self.team.shared
        merged = sh.shared_grid()[0] if sh is not None and sh.joined() else None   # leader's frame = mine
        if merged is None:
            merged = self.team.merged_grid(grid_from_msg(self.slam.grid))
        name = datetime.now().strftime('team_%Y%m%d_%H%M')
        if name in map_store.names():
            return None
        map_store.save_copy(name, map_merge.to_png(merged), map_merge.yaml_text(name, merged))
        self.log.info(f'team map saved as "{name}" ({self.car} + ' + ', '.join(m.car for m in mates) + ')')
        return name

    def _finish(self):
        self.mapped_m2 = self._known_m2()
        name = datetime.now().strftime('explore_%Y%m%d_%H%M_') + self.car
        ok, msg = self.slam.save(name)
        try:
            self.team_saved_as = self._save_team_map()
        except Exception as exc:
            self.team_saved_as = None
            self.log.warn(f'team map not saved: {exc}')
        self.saved_as = name if ok else None
        if self.nav.running() and self.nav.slam_mode:
            self.nav.stop()
        self.state = 'done'
        left = f' ({len(self.failed_centers)} edges not reachable)' if self.failed_centers else ''
        self.message = (f'map complete{left}, saved as "{name}"' if ok else f'map complete, but saving failed ({msg}): '
                        'press Save map') + (f'; team map "{self.team_saved_as}"' if self.team_saved_as else '') + \
            '; mapping still running'
        self.goal = None
        self.log.info(self.message)

    # ---- status -----------------------------------------------------------------
    def status(self):
        return {
            'state': self.state,
            'active': self.active(),
            'recovering': self.recovering or self.backout_req,
            'lifted': self.lifted,
            'plan': self.nav.plan if self.active() and self.nav.goal_active() else [],   # to the goal (own frame)
            'message': self.message,
            'radius': self.radius,
            'home': self.home,
            'goal': self.goal or (getattr(self, 'dry_goal', None) if self.dry and self.active() else None),
            'dry': self.dry,
            'frontiers': [f['center'] + [f['size']] for f in self.frontiers][:30],
            'cells': self.cells[:1500] if self.active() else [],
            'fails': self.fails,
            'goals': {'reached': self.reached, 'useful': self.partial, 'failed': self.failed},
            'mapped_m2': round(self._known_m2() if self.active() else self.mapped_m2, 1),
            'elapsed_s': round(time.monotonic() - self.started_at) if self.started_at and self.active() else None,
            'saved_as': self.saved_as,
            'team_saved_as': self.team_saved_as,
            'team': self.team.status() if self.team else [],
            'shared': self.team.shared.status() if self.team and self.team.shared else None,
        }
