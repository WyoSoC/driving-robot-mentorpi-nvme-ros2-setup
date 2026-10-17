"""Odometry calibration with the lidar and IMU as references (runs inside the MentorPi container;
normally via ../calibrate_odom.sh, which installs the result).

The car has no wheel-encoder feedback: /odom_raw is integrated from the *commanded* speed and
steering (odom_publisher), scaled by linear/angular_correction_factor. This measures how far
and how much the car really moved:

  linear   drive straight toward a flat wall: lidar distance change vs /odom_raw distance,
           forward then back again (2 samples)
  angular  drive a left and a right circle: IMU yaw change vs /odom_raw yaw change
  speedmap drive back and forth toward a flat wall at several motor speeds; the lidar measures the
           real speed of each (the motors have no speed feedback and respond non-linearly)
  steer    drive straight toward a flat wall and back at 0.1 m/s: IMU heading change per metre ->
           steering centre trim (servo offset, us) that makes "straight" straight
  check    no driving: show what the car sees and whether it is ready

Prints "RESULT motor_speed_map=real,sent,..." (speedmap), "RESULT motor_speed_factor=..." (linear: real speed / commanded speed; the controller divides
motor commands by it, so commanded = real and /odom_raw needs no linear correction) or
"RESULT angular_correction_factor=..." (new values = current value x measured ratio). Drives at 0.2 m/s; stops if something is too close, on timeout,
on Ctrl+C and on exit.

Usage: python3 odom_calib.py check|linear|angular CURRENT_LINEAR CURRENT_ANGULAR [CURRENT_MOTOR]
"""
import math
import signal
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

MODE = sys.argv[1]
CUR_LIN, CUR_ANG = float(sys.argv[2]), float(sys.argv[3])
CUR_MOTOR = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
CUR_TRIM = int(float(sys.argv[5])) if len(sys.argv) > 5 else 0   # steering servo offset, us
WHEELBASE = 0.145           # m, controller/ackermann.py
SPEED = 0.2                 # m/s
STEER = 0.5                 # rad/s commanded turn rate for the circles (0.4 m radius as commanded;
                            # car 2 really turned ~0.8x that, ~0.5 m radius, ~1.1 m wide circles)
WALL_MIN, WALL_MAX = 1.0, 4.0
STOP_FRONT = 0.45           # m, abort if the front gets this close
STOP_NEAR = 0.25            # m, abort during circles if anything is this close
STOP_BACK = 0.45            # m, abort when reversing if anything behind is this close (lidar to obstacle)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runaway(Exception):
    pass


class Calib:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('odom_calib')
        self.cmd = self.n.create_publisher(Twist, '/app/cmd_vel', 1)
        self.scan = None
        self.odom = None
        self.imu_yaw = None
        self.n.create_subscription(LaserScan, '/scan_raw', self._scan, qos_profile_sensor_data)
        self.n.create_subscription(Odometry, '/odom_raw', lambda m: setattr(self, 'odom', m), 10)
        self.gyro_yaw, self._gyro_t = 0.0, None   # integrated gyro z: the filtered /imu orientation went
        self.n.create_subscription(Imu, '/imu', self._imu, 50)   # wrong while driving (car 2: +-175 deg/pass)
        self.lock = threading.Lock()
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _imu(self, m):
        self.imu_yaw = yaw_of(m.orientation)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._gyro_t is not None and 0 < t - self._gyro_t < 0.5:
            self.gyro_yaw += m.angular_velocity.z * (t - self._gyro_t)
        self._gyro_t = t

    def wall_angle(self, half_deg=20.0):
        """Angle (rad) of the wall's normal relative to the car's heading, from a line fit to the
        lidar points within +-half_deg ahead (outliers > 5 cm dropped). + = wall normal to the left."""
        with self.lock:
            m = self.scan
        if m is None:
            return None
        pts = [(r * math.cos(a), r * math.sin(a), a) for i, r in enumerate(m.ranges)
               for a in [wrap(m.angle_min + i * m.angle_increment)]
               if abs(a) <= math.radians(half_deg) and m.range_min < r < m.range_max and math.isfinite(r)]
        centre = [x for x, _, a in pts if abs(a) <= math.radians(5)]
        if len(centre) < 3:
            return None
        cx = statistics.median(centre)   # keep only points near the wall straight ahead (not a doorway beside it)
        pts = [(x, y) for x, y, _ in pts if abs(x - cx) < 0.25]
        for _ in range(2):
            if len(pts) < 10:
                return None
            n = len(pts); my = sum(y for _, y in pts) / n; mx = sum(x for x, _ in pts) / n
            syy = sum((y - my) ** 2 for _, y in pts)
            a = sum((y - my) * (x - mx) for x, y in pts) / syy if syy > 0 else 0.0   # x = a*y + b
            b = mx - a * my
            pts = [(x, y) for x, y in pts if abs(x - (a * y + b)) < 0.05]
        return math.atan(-a)

    def _spin(self):
        try:
            rclpy.spin(self.n)
        except Exception:
            pass

    def _scan(self, m):
        with self.lock:
            self.scan = m

    def sector(self, center_deg, width_deg=3.0):
        """Median range in a narrow sector (0 = straight ahead, + = left)."""
        with self.lock:
            m = self.scan
        if m is None:
            return None
        c = math.radians(center_deg)
        v = [r for i, r in enumerate(m.ranges)
             if abs(wrap(m.angle_min + i * m.angle_increment - c)) <= math.radians(width_deg)
             and m.range_min < r < m.range_max and math.isfinite(r)]
        return statistics.median(v) if len(v) >= 2 else None

    def guard(self, direction, width_deg=35.0):
        """Nearest obstacle (2nd-closest point, so one stray return is ignored) in a wide wedge ahead
        (direction > 0) or behind (< 0). Not a median: a chair leg is only 1-2 points of a sector."""
        with self.lock:
            m = self.scan
        if m is None:
            return None
        c = 0.0 if direction > 0 else math.pi
        v = sorted(r for i, r in enumerate(m.ranges)
                   if abs(wrap(m.angle_min + i * m.angle_increment - c)) <= math.radians(width_deg)
                   and m.range_min < r < m.range_max and math.isfinite(r))
        return v[1] if len(v) >= 2 else None

    def nearest(self):
        with self.lock:
            m = self.scan
        v = [r for r in m.ranges if m.range_min < r < m.range_max and math.isfinite(r)] if m else []
        return min(v) if v else None

    def averaged(self, fn, seconds=2.0):
        vals, end = [], time.time() + seconds
        while time.time() < end:
            x = fn()
            if x is not None:
                vals.append(x)
            time.sleep(0.1)
        return statistics.median(vals) if vals else None

    def wait_ready(self):
        end = time.time() + 10
        while time.time() < end and (self.scan is None or self.odom is None or self.imu_yaw is None):
            time.sleep(0.1)
        missing = [k for k, v in (('lidar', self.scan), ('odom_raw', self.odom), ('imu', self.imu_yaw)) if v is None]
        if missing:
            sys.exit(f'no data from: {", ".join(missing)}')

    def drive(self, speed, steer=0.0):
        t = Twist()
        t.linear.x, t.angular.z = speed, steer
        self.cmd.publish(t)

    def stop(self):
        for _ in range(10):
            self.drive(0.0)
            time.sleep(0.05)

    def odom_xy(self):
        p = self.odom.pose.pose.position
        return p.x, p.y

    # ---- linear --------------------------------------------------------------
    def straight(self, direction):
        """Drive ~0.8 m (direction +1 toward the wall, -1 back). Return (lidar_dist, odom_dist)."""
        d0 = self.averaged(lambda: self.sector(0))
        if d0 is None:
            sys.exit('no lidar return straight ahead')
        travel = min(0.8, d0 - 0.6) if direction > 0 else 0.8
        if travel < 0.4:
            sys.exit(f'wall too close ({d0:.2f} m) to drive toward it')
        x0, y0 = self.odom_xy()
        end = time.time() + travel / SPEED * 2 + 3
        while True:
            x, y = self.odom_xy()
            if math.hypot(x - x0, y - y0) >= travel:
                break
            ahead = self.sector(0)   # also stop on the real (lidar) distance: an uncalibrated car
            if ahead is not None and abs(d0 - ahead) >= travel:   # can drive much further than odometry says
                break
            front = self.guard(direction)
            if front is not None and front < (STOP_FRONT if direction > 0 else STOP_BACK):
                self.stop()
                sys.exit(f'stopped: obstacle {front:.2f} m {"ahead" if direction > 0 else "behind"}')
            if time.time() > end:
                self.stop()
                sys.exit('stopped: timeout (is the car moving?)')
            self.drive(direction * SPEED)
            time.sleep(0.05)
        self.stop()
        time.sleep(1.5)  # settle
        x, y = self.odom_xy()
        d1 = self.averaged(lambda: self.sector(0))
        real, odo = abs(d0 - d1), math.hypot(x - x0, y - y0)
        print(f'  {"forward" if direction > 0 else "back   "}: lidar {d0:.3f} -> {d1:.3f} m = {real:.3f} m moved, '
              f'odometry {odo:.3f} m  (ratio {real / odo:.3f})', flush=True)
        return real, odo

    def linear(self):
        d0 = self.averaged(lambda: self.sector(0))
        if d0 is None or not WALL_MIN <= d0 <= WALL_MAX:
            sys.exit(f'need a flat wall straight ahead {WALL_MIN}-{WALL_MAX} m away (lidar sees {d0})')
        ratios = []
        for direction in (+1, -1):
            real, odo = self.straight(direction)
            ratios.append(real / odo)
        r = statistics.fmean(ratios)
        if abs(ratios[0] - ratios[1]) > 0.08:
            print(f'  WARNING: forward/back differ ({ratios[0]:.3f} vs {ratios[1]:.3f}); '
                  'check the wall is flat and square to the car', flush=True)
        if abs(CUR_LIN - 1.0) > 1e-6:
            print(f'  NOTE: linear_correction_factor is {CUR_LIN}; the result assumes odometry = commanded speed', flush=True)
        print(f'RESULT motor_speed_factor={CUR_MOTOR * r:.4f}  (current {CUR_MOTOR}, measured ratio {r:.4f})')

    # ---- speed map -----------------------------------------------------------
    def scan_front(self):
        """(stamp, median range straight ahead) of the latest scan."""
        with self.lock:
            m = self.scan
        if m is None:
            return None
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return st, self.sector(0)

    def run_at(self, sent, direction, target):
        """Drive at motor speed `sent` (m/s as the factory code understands it) until the wall ahead is
        at `target` m (absolute, so passes don't add up and walk the car away). Return the real
        steady speed in m/s (lidar range rate after the first second), 0 if it barely moved."""
        d0 = self.averaged(lambda: self.sector(0), 1.0)
        if d0 is None:
            sys.exit('no lidar return straight ahead')
        samples, seen, t0, glitches = [], set(), time.time(), 0
        while True:
            el = time.time() - t0
            f = self.scan_front()
            if f and f[1] is not None and f[0] not in seen:
                seen.add(f[0])
                # drop impossible jumps (> 2 m/s; the car tops out ~1.6 m/s): the narrow sector ahead
                # sometimes sees past the wall (gap, door edge) when the heading changes a little
                last = samples[-1] if samples else (f[0] - 0.1, d0)
                if abs(f[1] - last[1]) <= 2.0 * max(f[0] - last[0], 0.1) + 0.05:
                    samples.append((f[0], f[1]))
                else:
                    glitches += 1
                    if glitches > 8:
                        self.stop()
                        sys.exit('stopped: the wall reading ahead keeps jumping; is the car still square to a '
                                 'flat wall, with no door or gap next to where it points?')
                    continue
                if (f[1] <= target) if direction > 0 else (f[1] >= target):
                    break
            guard = self.guard(direction)
            if guard is not None and guard < (STOP_FRONT if direction > 0 else STOP_BACK):
                self.stop()
                sys.exit(f'stopped: obstacle {guard:.2f} m {"ahead" if direction > 0 else "behind"}')
            if len(samples) >= 6 and samples[-1][0] - samples[-6][0] >= 0.35:   # runaway: much faster than sent
                w = samples[-6:]
                fast = abs(w[-1][1] - w[0][1]) / (w[-1][0] - w[0][0])
                steady = all((b[1] - a[1]) * (w[-1][1] - w[0][1]) >= 0 for a, b in zip(w, w[1:]))
                if fast > max(3 * sent, 0.3) and steady and abs(w[-1][1] - d0) > 0.15:
                    self.stop()
                    time.sleep(1.0)
                    trace = ' '.join(f'{r:.2f}' for _, r in samples[-10:])
                    print(f'    sent {sent:.3f} {"fwd" if direction > 0 else "back"}: RUNAWAY, ~{fast:.2f} m/s real; '
                          f'stopped (wall distance, last readings: {trace})', flush=True)
                    raise Runaway(fast)
            if el > 10:
                break
            self.drive(direction * sent)
            time.sleep(0.05)
        self.stop()
        time.sleep(1.0)
        if len(samples) < 4:
            return 0.0
        ts = samples[0][0]
        late = [(s - ts, r) for s, r in samples if s - ts >= 1.0] or [(s - ts, r) for s, r in samples]
        if len(late) < 3:
            late = [(s - ts, r) for s, r in samples[len(samples) // 2:]]
        n = len(late); mt = sum(a for a, _ in late) / n; mr = sum(b for _, b in late) / n
        den = sum((a - mt) ** 2 for a, _ in late)
        v = abs(sum((a - mt) * (b - mr) for a, b in late) / den) if den > 0 else 0.0
        moved = abs(samples[-1][1] - d0)
        print(f'    sent {sent:.3f} {"fwd" if direction > 0 else "back"}: real {v:.3f} m/s, moved {moved:.2f} m'
              f'{f", {glitches} lidar glitches ignored" if glitches else ""}', flush=True)
        return v if moved > 0.05 else 0.0

    def back_off(self):
        """After a runaway pass, slowly return toward the start distance."""
        ahead = self.averaged(lambda: self.sector(0), 0.5)
        if ahead is not None and ahead < 1.5:
            behind = self.guard(-1)
            if behind is None or behind > 1.0:
                self.run_at(0.07, -1, 1.5)

    def speedmap(self):
        d = self.averaged(lambda: self.sector(0))
        if d is None or not 1.5 <= d <= WALL_MAX:
            sys.exit(f'need a flat wall straight ahead 1.5-{WALL_MAX} m away (lidar sees {d})')
        home = d
        behind = self.guard(-1)
        if behind is not None and behind < 1.0:
            sys.exit(f'something is {behind:.2f} m behind the car; needs ~1.3 m free behind')
        pts = []
        for sent in (0.05, 0.07, 0.09, 0.11, 0.13, 0.16, 0.20):
            try:
                v = (self.run_at(sent, +1, home - 0.7) + self.run_at(sent, -1, home)) / 2
            except Runaway:
                print(f'  sent {sent:.3f} m/s -> RUNAWAY: the motor speed control loses control here; '
                      'map ends below this speed', flush=True)
                self.back_off()
                break
            print(f'  sent {sent:.3f} m/s -> real {v:.3f} m/s (ratio {v / sent:.2f})', flush=True)
            if v > 0.02 and (not pts or v > pts[-1][0] + 0.005):
                pts.append((v, sent))
        if len(pts) < 2:
            sys.exit('car hardly moved at any speed; nothing measured')
        flat = ','.join(f'{r:.4f},{s:.4f}' for r, s in pts)
        print(f'RESULT motor_speed_map={flat}')

    # ---- steering centre -----------------------------------------------------
    def straight_yaw(self, direction, target, speed=0.1):
        """Drive straight until the wall ahead is at `target` m. Return (metres moved, yaw change)."""
        d0 = self.averaged(lambda: self.sector(0), 1.0)
        w0, g0 = self.averaged(self.wall_angle, 1.0), self.gyro_yaw
        if w0 is None:
            sys.exit('cannot see a flat wall ahead (+-20 deg)')
        t0 = time.time()
        while True:
            f = self.sector(0)
            if f is not None and ((f <= target) if direction > 0 else (f >= target)):
                break
            g = self.guard(direction)
            if g is not None and g < (STOP_FRONT if direction > 0 else STOP_BACK):
                self.stop()
                sys.exit(f'stopped: obstacle {g:.2f} m {"ahead" if direction > 0 else "behind"}')
            if time.time() - t0 > 15:
                self.stop()
                sys.exit('stopped: timeout (is the car moving?)')
            self.drive(direction * speed)
            time.sleep(0.05)
        self.stop()
        time.sleep(1.5)
        d1 = self.averaged(lambda: self.sector(0), 1.0)
        w1 = self.averaged(self.wall_angle, 1.0)
        if w1 is None:
            sys.exit('lost the wall ahead')
        dyaw = -(w1 - w0)                    # car turns left -> the wall's normal swings right
        dgyro = self.gyro_yaw - g0
        moved = abs(d1 - d0)
        print(f'  {"forward" if direction > 0 else "back   "}: moved {moved:.2f} m, heading changed '
              f'{math.degrees(dyaw):+.2f} deg by lidar, {math.degrees(dgyro):+.2f} by gyro '
              f'({math.degrees(dyaw) / max(moved, 0.01):+.2f} deg/m)', flush=True)
        if abs(dyaw) > math.radians(15) or abs(dgyro) > math.radians(15):
            sys.exit('heading changed > 15 deg in one pass: not plausible for a steering trim; nothing saved')
        if abs(dyaw - dgyro) > max(math.radians(1.5), 0.3 * abs(dyaw)):
            print('    (gyro disagrees with the lidar; using the lidar, it sees the wall directly)', flush=True)
        return moved, dyaw

    def steer(self):
        home = self.averaged(lambda: self.sector(0))
        if home is None or not 1.5 <= home <= WALL_MAX:
            sys.exit(f'need a flat wall straight ahead 1.5-{WALL_MAX} m away (lidar sees {home})')
        behind = self.guard(-1)
        if behind is not None and behind < 1.0:
            sys.exit(f'something is {behind:.2f} m behind the car; needs ~1.3 m free behind')
        ks, kb = [], []
        for _ in range(2):
            m1, y1 = self.straight_yaw(+1, home - 0.7)
            m2, y2 = self.straight_yaw(-1, home)
            ks.append(y1 / max(m1, 0.05))        # path curvature driving forward, rad/m (+ = curves left)
            kb.append(-y2 / max(m2, 0.05))       # reversing: steering play makes it differ; reported only
        k = statistics.fmean(ks)
        print(f'  reversing: {statistics.fmean([math.degrees(x) for x in kb]):+.1f} deg/m (not used: steering play '
              'lets the wheels settle differently when pushed backwards)', flush=True)
        delta = math.degrees(math.atan(WHEELBASE * k))       # steering angle the car actually has
        us = 2000 * delta / 180                               # ackermann.py: 1500 + 2000*deg/180
        new = int(round(max(-100, min(100, CUR_TRIM - us))))
        passes = ' '.join(f'{math.degrees(x):+.1f}' for x in ks)
        print(f'  forward curvature {math.degrees(k):+.1f} deg/m (passes: {passes}) '
              f'= steering {delta:+.2f} deg off centre ({"left" if delta > 0 else "right"})', flush=True)
        if abs(us) < 2:
            print('  steering centre is good (< 2 us off)', flush=True)
        print(f'RESULT steer_trim_us={new}  (current {CUR_TRIM}, change {new - CUR_TRIM:+d})')

    # ---- angular -------------------------------------------------------------
    def circle(self, steer):
        imu_prev, odo_prev = self.gyro_yaw, yaw_of(self.odom.pose.pose.orientation)
        imu_turn = odo_turn = 0.0
        end = time.time() + 2 * math.pi * 0.4 / SPEED * 2.5
        while abs(imu_turn) < math.radians(340):
            near = self.nearest()
            if near is not None and near < STOP_NEAR:
                self.stop()
                sys.exit(f'stopped: obstacle {near:.2f} m away; needs ~1.2 m clear around the car')
            if time.time() > end:
                self.stop()
                sys.exit('stopped: timeout (is the car moving?)')
            self.drive(SPEED, steer)
            time.sleep(0.05)
            imu, odo = self.gyro_yaw, yaw_of(self.odom.pose.pose.orientation)
            imu_turn += imu - imu_prev
            odo_turn += wrap(odo - odo_prev)
            imu_prev, odo_prev = imu, odo
        self.stop()
        time.sleep(1.0)
        imu_turn += self.gyro_yaw - imu_prev
        odo_turn += wrap(yaw_of(self.odom.pose.pose.orientation) - odo_prev)
        print(f'  {"left " if steer > 0 else "right"} circle: IMU {math.degrees(imu_turn):+.1f} deg, '
              f'odometry {math.degrees(odo_turn):+.1f} deg  (ratio {imu_turn / odo_turn:.3f})', flush=True)
        return imu_turn / odo_turn

    def angular(self):
        near = self.averaged(self.nearest, 1.0)
        if near is not None and near < 0.6:
            sys.exit(f'something is {near:.2f} m from the lidar; needs ~1.2 m clear around the car')
        ratios = [self.circle(+STEER), self.circle(-STEER)]
        r = statistics.fmean(ratios)
        if abs(ratios[0] - ratios[1]) > 0.1:
            print(f'  NOTE: left/right differ ({ratios[0]:.3f} vs {ratios[1]:.3f}); '
                  'the steering may not be centred', flush=True)
        print(f'RESULT angular_correction_factor={CUR_ANG * r:.4f}  (current {CUR_ANG}, measured ratio {r:.4f})')

    def check(self):
        f, b, near = (self.averaged(lambda: self.sector(0)), self.averaged(lambda: self.guard(-1)),
                      self.averaged(self.nearest, 1.0))
        fmt = lambda v: f'{v:.2f} m' if v is not None else 'nothing'
        g0 = self.gyro_yaw
        wa = self.averaged(self.wall_angle, 1.0)
        drift = math.degrees(self.gyro_yaw - g0)
        print(f'  ahead {fmt(f)}, behind {fmt(b)}, nearest {fmt(near)}')
        print(f'  wall ahead: {"not seen" if wa is None else f"{math.degrees(wa):+.1f} deg off square"}; '
              f'gyro drift standing still {drift:+.2f} deg/s')
        print(f'  linear test: {"ready" if f and WALL_MIN <= f <= WALL_MAX else f"needs a wall {WALL_MIN}-{WALL_MAX} m ahead"}'
              f'{"" if b is None or b > 1.3 else f", but needs 1.3 m free behind (has {b:.2f} m)"}')
        print(f'  angular test: {"ready" if near is None or near >= 0.6 else "needs ~1.2 m clear around the car"}')


def main():
    c = Calib()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: (c.stop(), sys.exit(130)))
    try:
        c.wait_ready()
        {'check': c.check, 'linear': c.linear, 'angular': c.angular, 'speedmap': c.speedmap,
         'steer': c.steer}[MODE]()
    finally:
        c.stop()
        rclpy.try_shutdown()
        c.spinner.join(timeout=5)  # avoid rclpy aborting the process at exit


if __name__ == '__main__':
    main()
