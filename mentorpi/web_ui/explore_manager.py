"""Autonomous mapping of one car (runs inside web_ui_server.py): frontier exploration.

start(radius): starts SLAM (if not running) and Nav2 on the live SLAM map (nav.launch.py slam:=true), then
repeatedly drives to the best "frontier" (the edge between mapped free floor and unmapped space) until none
is left within `radius` m of the start, drives back to the start and saves the map (explore_YYYYMMDD_HHMM).

Goals are only ever placed on mapped free floor, away from walls (CLEAR_OCC) and from unmapped space
(CLEAR_UNKNOWN), and the planner may not plan through unmapped space (allow_unknown: false).
Note: the lidar can't see a drop (stairs): the floor past a stair edge looks free. Use the radius as a fence.

It stops (navigation stopped, SLAM kept so the map can be saved) on: Stop, the joystick or E-stop (they
cancel the goal), low battery, charging mode, SLAM or navigation stopping, too many failed goals, or
MAX_TIME.
"""
import math
import threading
import time
from datetime import datetime

import numpy as np
from scipy import ndimage

FREE_MAX = 25            # occupancy 0..100: below = free
OCC_MIN = 65             # at or above = wall / obstacle
CLEAR_OCC = 0.30         # m, goal at least this far from walls
CLEAR_UNKNOWN = 0.15     # m, and from unmapped space
PASS_OCC = 0.14          # m, cells the car can pass (connectivity check)
GOAL_NEAR = 1.0          # m, goal at most this far from its frontier
MIN_FRONTIER = 0.50      # m, smaller frontiers are ignored (noise, slow to chase)
MIN_GOAL_DIST = 0.70     # m, closer goals: short awkward hops for a car
BLACKLIST_R = 0.5        # m, around failed goals
GOAL_TIMEOUT = 120       # s per goal
STUCK_S = 25             # s without moving STUCK_M while driving to a goal -> give up on it
STUCK_M = 0.15
FAILED_FRONTIER_R = 0.8  # m, frontiers whose centre is this close to a failed one are skipped
MAX_FAILS = 6            # failed goals in a row -> give up
MAX_TIME = 20 * 60       # s
START_TIMEOUT = 90       # s for SLAM + Nav2 to come up
NO_FRONTIER_CHECKS = 3   # consecutive empty checks -> done
STEP_S = 1.0
SMOOTH_CELLS = 2          # closing iterations on the free space


def frontier_goals(grid, res, origin, car_xy, fence_xy, radius, blacklist, failed_centers=()):
    """Frontier clusters and a goal for each. grid: int8 [h, w] (-1 unknown, 0..100), row 0 = min y.
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
        yaw = math.atan2(cy - y, cx - x)              # face the unmapped space
        out.append({'goal': [round(x, 3), round(y, 3), round(yaw, 3)], 'center': [round(cx, 3), round(cy, 3)],
                    'size': round(size_m, 2), 'dist': round(float(dcar[i]), 2)})
        sel = mask[rows_all, cols_all]
        for r, c in list(zip(rows_all[sel], cols_all[sel]))[::3]:
            x_, y_ = to_xy(r, c)
            cells.append([round(x_, 2), round(y_, 2)])
    # best first: close and large
    out.sort(key=lambda f: f['dist'] - 0.5 * min(f['size'], 3.0))
    return out, cells


class Explorer:
    def __init__(self, node, slam, nav):
        self.node, self.slam, self.nav = node, slam, nav
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
        self.fails = 0
        self.empty_checks = 0
        self.quick_retries = 0
        self.started_at = None
        self.saved_as = None
        self.own_cancel = False
        self.dry = False
        threading.Thread(target=self._loop, daemon=True).start()

    def active(self):
        return self.state in ('starting', 'exploring', 'returning')

    # ---- control --------------------------------------------------------------
    def start(self, radius, dry=False):
        """dry: everything except sending goals (test without driving; API only)."""
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
            self.home = None
            self.goal, self.frontiers, self.cells = None, [], []
            self.blacklist, self.visited, self.failed_centers = [], [], []
            self.goal_center = None
            self.fails = self.empty_checks = self.quick_retries = 0
            self.goal_sent_at = None
            self.saved_as = None
            self.started_at = time.monotonic()
            self.state = 'starting'
            self.message = 'starting mapping and navigation (~30 s)'
            self.log.info(f'exploration started (radius {radius or "unlimited"} m)')
            return True, self.message

    def stop(self, reason='exploration stopped'):
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

    def _grid(self):
        msg = self.slam.grid
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
            self.home = list(self.slam.pose)
            self.state, self.message = 'exploring', 'exploring'
            return

        if not self.nav.running():
            return self.stop('navigation stopped')
        gs = self.nav.goal_state
        pose = self.slam.pose

        if self.goal is not None:
            if gs in ('sending', 'active'):
                if pose is not None and (self.progress_at is None or
                                         math.hypot(pose[0] - self.progress_xy[0], pose[1] - self.progress_xy[1]) > STUCK_M):
                    self.progress_xy, self.progress_at = pose[:2], now
                if now - self.progress_at > STUCK_S:
                    self.own_cancel = True
                    self.nav.cancel('stuck')
                    self._failed(f'stuck (no progress for {STUCK_S} s)')
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
            elif gs in ('aborted', 'rejected'):
                self._failed('could not reach it')
            elif gs == 'succeeded':
                if self.state == 'returning':
                    return self._finish()
                self.fails = 0
                self.quick_retries = 0
                self.visited.append(self.goal[:2])
                self.goal = None
            else:
                return   # idle: the goal response has not arrived yet
            if self.fails >= MAX_FAILS:
                return self.stop(f'{MAX_FAILS} goals in a row could not be reached')
            if self.state == 'returning':   # could not get home: finish where we are
                return self._finish()

        if self.state == 'returning':
            return
        # pick the next frontier
        g = self._grid()
        if g is None or pose is None:
            return
        grid, res, origin = g
        fronts, cells = frontier_goals(grid, res, origin, pose[:2], self.home[:2], self.radius,
                                       self.blacklist + self.visited, self.failed_centers)
        self.frontiers, self.cells = fronts, cells
        if not fronts:
            self.empty_checks += 1
            self.message = f'no frontier left? checking ({self.empty_checks}/{NO_FRONTIER_CHECKS})'
            if self.empty_checks >= NO_FRONTIER_CHECKS:
                self.state, self.message = 'returning', 'map complete: driving back to the start'
                self.goal_center = None
                self._send(self.home)
            return
        self.empty_checks = 0
        f = fronts[0]
        self.goal_center = f['center']
        if self.dry:
            self.message = f'dry run: would go to {f["goal"]} ({len(fronts)} frontiers)'
            self.dry_goal = f['goal']
            return
        self.message = f'to frontier {f["size"]:.1f} m wide, {f["dist"]:.1f} m away ({len(fronts)} left)'
        self._send(f['goal'])

    def _send(self, goal):
        self.own_cancel = False
        ok, msg = self.nav.go(*goal)
        if not ok:
            self._failed(msg)
            return
        self.goal, self.goal_sent_at, self.own_cancel = list(goal), time.monotonic(), False
        self.progress_at, self.progress_xy = time.monotonic(), (self.slam.pose or goal)[:2]

    def _failed(self, why):
        quick = self.goal_sent_at is not None and time.monotonic() - self.goal_sent_at < 3.0
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
            self.log.info(f'exploration goal {self.goal} failed: {why}')
        self.fails += 1
        self.goal = None

    def _check_goal_still_useful(self, pose):
        """While driving: if the frontier we drive to has been mapped meanwhile, go to the next one."""
        g = self._grid()
        if g is None or pose is None:
            return
        grid, res, origin = g
        fronts, cells = frontier_goals(grid, res, origin, pose[:2], self.home[:2], self.radius,
                                       self.blacklist + self.visited, self.failed_centers)
        self.frontiers, self.cells = fronts, cells
        gx, gy = self.goal[0], self.goal[1]
        if not any(math.hypot(f['goal'][0] - gx, f['goal'][1] - gy) < 1.0 for f in fronts):
            self.own_cancel = True
            self.nav.cancel('frontier mapped, next one')
            self.visited.append(self.goal[:2])
            self.goal = None

    def _finish(self):
        name = datetime.now().strftime('explore_%Y%m%d_%H%M')
        ok, msg = self.slam.save(name)
        self.saved_as = name if ok else None
        if self.nav.running() and self.nav.slam_mode:
            self.nav.stop()
        self.state = 'done'
        self.message = (f'map complete, saved as "{name}"' if ok else f'map complete, but saving failed ({msg}): '
                        'press Save map') + '; mapping still running'
        self.goal = None
        self.log.info(self.message)

    # ---- status -----------------------------------------------------------------
    def status(self):
        return {
            'state': self.state,
            'active': self.active(),
            'message': self.message,
            'radius': self.radius,
            'home': self.home,
            'goal': self.goal or (getattr(self, 'dry_goal', None) if self.dry and self.active() else None),
            'dry': self.dry,
            'frontiers': [f['center'] + [f['size']] for f in self.frontiers][:30],
            'cells': self.cells[:1500] if self.active() else [],
            'fails': self.fails,
            'elapsed_s': round(time.monotonic() - self.started_at) if self.started_at and self.active() else None,
            'saved_as': self.saved_as,
        }
