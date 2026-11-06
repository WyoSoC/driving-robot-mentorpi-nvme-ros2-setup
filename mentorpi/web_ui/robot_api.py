"""The car's API for programs on other computers (runs inside web_ui_server.py): /api/v1/... on the page's port.

Everything a program needs to drive the car and read it, over plain HTTP + JSON (README "API"; a Python client
and an example are in mentorpi/client/):

  GET  /api/v1                 what this is: car, limits, endpoints
  GET  /api/v1/state           everything at once: drive, odometry, IMU, lidar summary, battery, charging, safety,
                               depth camera, SLAM / navigation / mapping
  GET  /api/v1/stream?hz=5     the same state, pushed hz times a second (Server-Sent Events: "data: {json}")
  GET  /api/v1/battery         voltage, charge %, charging / in use, time to empty / full, charging mode
  GET  /api/v1/odom            EKF odometry: x, y, yaw, speed, turn rate
  GET  /api/v1/imu             roll, pitch, accelerations, turn rates
  GET  /api/v1/lidar           the whole lidar scan (ranges in m, null = no reading)
  GET  /api/v1/camera.jpg?view=rgb|depth        one picture (depth: colourised)
  GET  /api/v1/video?view=rgb|depth&width=&quality=   MJPEG stream (multipart/x-mixed-replace)
  GET  /api/v1/depth.png       the latest depth image, 16-bit PNG, millimetres (0 = no reading)
  GET  /api/v1/depth/camera    depth camera intrinsics and mounting (for turning depth pixels into points)
  POST /api/v1/drive           {"speed": m/s, "steer": deg (+ left), "duration": s, "safe": true, "dry": false}
  POST /api/v1/stop            stop now
  POST /api/v1/horn            beep
  POST /api/v1/charging        {"on": true | false}: charging mode (stops / restarts the robot software)

Driving (the car is an Ackermann car: it steers its front wheels, it can't turn on the spot):
- A drive command holds for `duration` seconds (default 0.5, at most MAX_DURATION_S), then the car stops by
  itself. Send the next command before it runs out to keep going (a program that stops talking stops the car).
- Limits are the page's: speed up to MAX_SPEED m/s, steering up to MAX_STEER_DEG degrees.
- safe (default on): the lidar watches the path the car will sweep (along the steering arc, car width + margin)
  and refuses / stops when something is closer than the stopping distance (STOP_BASE_M + speed-dependent).
  The lidar is 16 cm up: low things (cables, thresholds) it doesn't see. safe=false turns this off.
- dry=true: check and answer without moving the car.
- Refused while charging mode is on, while the car is lifted, during autonomous mapping, and without lidar
  data (safe mode). A navigation goal is cancelled (as the page's joystick does).
- The page wins: driving or E-STOP on the page cancels the API's command.
"""
import json
import math
import threading
import time
import urllib.request

import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

import car_geometry as g

VERSION = 1
MAX_SPEED = 0.7                 # m/s (the page's limit)
MAX_STEER_DEG = 40.0
MAX_DURATION_S = 3.0
DEFAULT_DURATION_S = 0.5
WHEELBASE = 0.145               # m (nav_manager.py)
STOP_BASE_M = 0.15              # free space always kept past the bumper
STOP_LATENCY_S = 0.25           # lidar + command delay
STOP_DECEL = 1.0                # m/s^2, braking
SIDE_MARGIN_M = 0.03            # beyond the car's half width
SCAN_STALE_S = 1.0
VIDEO = 'http://127.0.0.1:8080'
VIEWS = {'rgb': '/ascamera/camera_publisher/rgb0/image', 'depth': '/web/depth_color'}
LOW_V, CRITICAL_V = 6.9, 6.6    # the page's battery warnings


def stop_distance(speed):
    v = abs(speed)
    return STOP_BASE_M + v * STOP_LATENCY_S + v * v / (2 * STOP_DECEL)


def path_obstacle(ranges, angle_min, angle_inc, speed, steer_rad, lidar_x=g.LIDAR_X):
    """Nearest lidar reading in the path the car sweeps driving at `speed` with `steer_rad`.
    Returns (distance along the path past the bumper in m, or inf; bearing of that reading in deg).
    The car turns about a point on its rear axle line (base_footprint origin) at radius WHEELBASE / tan(steer);
    a reading is in the path when its distance from that arc is within the car's half width + margin."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > 0.02)
    r = np.where(ok, r, 0.0)
    x = lidar_x + r * np.cos(a)
    y = r * np.sin(a)
    inside = (x >= g.BACK) & (x <= g.FRONT) & (np.abs(y) <= g.HALF_WIDTH)
    ok &= ~inside                                  # touching the car / the car itself (scan_filter.py)
    back = speed < 0
    if back:
        x = -x                                     # reversing: mirror; the turning centre stays on the same side
    bumper = -g.BACK if back else g.FRONT
    half = g.HALF_WIDTH + SIDE_MARGIN_M
    k = math.tan(steer_rad) / WHEELBASE            # curvature, + left
    if abs(k) < 1e-3:
        lateral = np.abs(y)
        along = x - bumper
    else:
        R = 1.0 / k
        rho = np.hypot(x, y - R)                   # distance from the turning centre (0, R)
        lateral = np.abs(rho - abs(R))
        theta = np.arctan2(x, np.sign(R) * (R - y))   # angle travelled along the arc to reach the reading
        theta = np.where(theta < -math.pi / 2, theta + 2 * math.pi, theta)
        along = abs(R) * theta - bumper
    hit = ok & (lateral <= half) & (along > -0.02)
    if not hit.any():
        return math.inf, None
    i = int(np.argmin(np.where(hit, along, np.inf)))
    return max(0.0, float(along[i])), round(math.degrees(math.atan2(math.sin(a[i]), math.cos(a[i]))), 1)


def sector_min(ranges, angle_min, angle_inc, lo_deg, hi_deg):
    r = np.asarray(ranges, dtype=float)
    a = np.degrees((angle_min + np.arange(len(r)) * angle_inc + math.pi) % (2 * math.pi) - math.pi)
    m = np.isfinite(r) & (r > 0.02) & (a >= lo_deg) & (a <= hi_deg) if lo_deg <= hi_deg else \
        np.isfinite(r) & (r > 0.02) & ((a >= lo_deg) | (a <= hi_deg))
    return round(float(r[m].min()), 3) if m.any() else None


def r3(v, n=3):
    return None if v is None or not math.isfinite(v) else round(float(v), n)


class RobotAPI:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.cmd = None                 # {'speed', 'steer', 'until', 'safe', 'who', 'started'}
        self.last = {'result': None}    # the last drive outcome, for state
        self.scan = None
        self.scan_at = 0.0
        self.imu = {}
        self.imu_at = 0.0
        self.rpy = None
        self.gyro = None
        self.odom = None
        self.depth_msg = None
        node.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu_corrected', self.on_imu_corrected, qos_profile_sensor_data)
        node.create_subscription(Vector3Stamped, '/imu/rpy/filtered', self.on_rpy, qos_profile_sensor_data)
        try:
            from ros_robot_controller_msgs.msg import BuzzerState
            self.buzzer_type = BuzzerState
            self.buzzer_pub = node.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        except ImportError:
            self.buzzer_pub = None
        node.create_timer(0.1, self.tick)

    # ---- sensors -------------------------------------------------------------------------
    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.monotonic()

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.imu = {'accel': [r3(a.x), r3(a.y), r3(a.z)]}
        self.imu_at = time.monotonic()

    def on_imu_corrected(self, msg):
        w = msg.angular_velocity
        self.gyro = [r3(math.degrees(w.x), 2), r3(math.degrees(w.y), 2), r3(math.degrees(w.z), 2)]

    def on_rpy(self, msg):
        self.rpy = (msg.vector.x, msg.vector.y)

    def on_odom(self, msg):
        """Called from web_ui_server.py's /odom handler."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = {'x': r3(p.x), 'y': r3(p.y), 'yaw_deg': r3(math.degrees(yaw), 1),
                     'speed': r3(msg.twist.twist.linear.x), 'turn_rate_deg_s': r3(math.degrees(msg.twist.twist.angular.z), 1),
                     't': time.time()}

    def scan_fresh(self):
        return self.scan is not None and time.monotonic() - self.scan_at < SCAN_STALE_S

    # ---- driving -------------------------------------------------------------------------
    def why_not(self, speed):
        n = self.node
        if n.charging != 'off':
            return 'charging mode is on (the robot software is stopped): switch it off first'
        if n.lift.lifted:
            return 'the car is lifted: put it down first'
        if n.explorer.active():
            return 'autonomous mapping is running: stop it first'
        return None

    def check_path(self, speed, steer_rad):
        """(blocked reason or None, nearest in path m)."""
        if not self.scan_fresh():
            return 'no lidar data (safe mode needs it; safe=false drives without)', None
        s = self.scan
        d, bearing = path_obstacle(s.ranges, s.angle_min, s.angle_increment, speed, steer_rad)
        need = stop_distance(speed)
        if speed != 0 and d < need:
            return (f'obstacle {d:.2f} m {"ahead" if speed > 0 else "behind"} in the path (bearing {bearing}°), '
                    f'needs {need:.2f} m at {abs(speed):.2f} m/s'), d
        return None, d

    def drive(self, speed, steer_deg=0.0, duration=DEFAULT_DURATION_S, safe=True, dry=False, who='api'):
        try:
            speed, steer_deg, duration = float(speed), float(steer_deg), float(duration)
        except (TypeError, ValueError):
            return {'ok': False, 'message': 'speed, steer and duration must be numbers'}
        if not all(map(math.isfinite, (speed, steer_deg, duration))):
            return {'ok': False, 'message': 'speed, steer and duration must be finite'}
        limited = abs(speed) > MAX_SPEED or abs(steer_deg) > MAX_STEER_DEG or not 0 < duration <= MAX_DURATION_S
        speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
        steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))
        duration = max(0.05, min(MAX_DURATION_S, duration))
        steer = math.radians(steer_deg)
        out = {'speed': speed, 'steer': steer_deg, 'duration': duration, 'safe': bool(safe), 'dry': bool(dry),
               'stop_distance_m': r3(stop_distance(speed), 2)}
        if limited:
            out['limited'] = f'clamped to speed <= {MAX_SPEED} m/s, steer <= {MAX_STEER_DEG}°, 0 < duration <= {MAX_DURATION_S} s'
        why = self.why_not(speed)
        if why:
            return self._result(out | {'ok': False, 'message': why})
        if safe:
            blocked, d = self.check_path(speed, steer)
            out['path_clear_m'] = r3(d, 2)
            if blocked:
                if not dry:
                    self.stop('blocked')
                return self._result(out | {'ok': False, 'blocked': True, 'message': blocked})
        if dry:
            return self._result(out | {'ok': True, 'message': 'dry run: would drive'})
        n = self.node
        if (speed or steer) and (n.nav.goal_active() or n.nav.goal_state in ('yielding', 'nudging')):
            n.nav.cancel('goal canceled: driven through the API')
            out['note'] = 'navigation goal canceled'
        now = time.monotonic()
        with self.lock:
            started = self.cmd['started'] if self.cmd else now
            self.cmd = {'speed': speed, 'steer': steer, 'until': now + duration, 'safe': bool(safe), 'who': who,
                        'started': started}
        self._send(speed, steer)
        return self._result(out | {'ok': True, 'message': 'driving'})

    def _result(self, r):
        self.last = {'result': r, 't': time.time()}
        return r

    def _send(self, speed, steer):
        t = Twist()
        t.linear.x, t.angular.z = speed, steer
        self.node.send_drive(t)

    def stop(self, why='stop'):
        with self.lock:
            had = self.cmd is not None
            self.cmd = None
        self.node.stop()
        if had and why != 'stop':
            self.node.get_logger().info(f'API drive stopped: {why}')
        return {'ok': True, 'message': 'stopped' if why == 'stop' else f'stopped: {why}'}

    def cancel_by_page(self):
        """The page drove (or pressed E-STOP): the person there wins."""
        if self.cmd is not None:
            with self.lock:
                self.cmd = None
            self.last = {'result': {'ok': False, 'message': 'canceled: someone drove from the page'}, 't': time.time()}
            self.node.get_logger().info('API drive canceled by the page')

    def tick(self):
        c = self.cmd
        if c is None:
            return
        if time.monotonic() >= c['until']:
            self.stop('duration over')
            self.last = {'result': {'ok': True, 'message': 'done (duration over)'}, 't': time.time()}
            return
        why = self.why_not(c['speed'])
        if why is None and c['safe']:
            why, _ = self.check_path(c['speed'], c['steer'])
        if why:
            self.stop(why)
            self.last = {'result': {'ok': False, 'blocked': True, 'message': why}, 't': time.time()}
            return
        self._send(c['speed'], c['steer'])

    def horn(self):
        if self.buzzer_pub is None:
            return {'ok': False, 'message': 'no buzzer message type'}
        m = self.buzzer_type()
        m.freq, m.on_time, m.off_time, m.repeat = 1900, 0.1, 0.05, 2
        self.buzzer_pub.publish(m)
        return {'ok': True, 'message': 'beep'}

    # ---- reading -------------------------------------------------------------------------
    def battery(self):
        n = self.node
        v = n.battery_v
        b = dict(n.battery_st or {})
        return {'voltage': r3(v, 2), 'level': None if v is None else 'critical' if v < CRITICAL_V else 'low' if v < LOW_V else 'ok',
                'charging_mode': n.charging, **b}

    def imu_state(self):
        return {'roll_deg': r3(math.degrees(self.rpy[0]), 1) if self.rpy else None,
                'pitch_deg': r3(math.degrees(self.rpy[1]), 1) if self.rpy else None,
                'yaw_deg': self.odom['yaw_deg'] if self.odom else None,
                'accel_m_s2': self.imu.get('accel'), 'gyro_deg_s': self.gyro,
                'age_s': r3(time.monotonic() - self.imu_at, 2) if self.imu_at else None}

    def lidar_summary(self):
        if self.scan is None:
            return None
        s = self.scan
        args = (s.ranges, s.angle_min, s.angle_increment)
        r = np.asarray(s.ranges, dtype=float)
        ok = np.isfinite(r) & (r > 0.02)
        return {'front_m': sector_min(*args, -30, 30), 'left_m': sector_min(*args, 60, 120),
                'back_m': sector_min(*args, 150, -150), 'right_m': sector_min(*args, -120, -60),
                'nearest_m': r3(float(r[ok].min())) if ok.any() else None,
                'readings': int(ok.sum()), 'age_s': r3(time.monotonic() - self.scan_at, 2)}

    def lidar(self):
        if self.scan is None:
            return None
        s = self.scan
        return {'frame': 'lidar: same orientation as the car (x forward, y left), '
                         f'{g.LIDAR_X} m along x from base_footprint, {g.LIDAR_TOP_Z} m up',
                'angle_min': s.angle_min, 'angle_increment': s.angle_increment, 'range_min': s.range_min,
                'range_max': s.range_max, 'stamp': s.header.stamp.sec + s.header.stamp.nanosec * 1e-9,
                'age_s': r3(time.monotonic() - self.scan_at, 2),
                'ranges': [round(x, 3) if math.isfinite(x) and x > 0 else None for x in s.ranges]}

    def drive_state(self):
        c = self.cmd
        now = time.monotonic()
        return {'active': c is not None,
                'speed': c['speed'] if c else 0.0, 'steer': r3(math.degrees(c['steer']), 1) if c else 0.0,
                'remaining_s': r3(max(0.0, c['until'] - now), 2) if c else 0.0,
                'last': self.last.get('result')}

    def state(self):
        n = self.node
        nav = n.nav.status()
        dc = n.depth_obs.compare or {}
        floor = dc.get('floor') or {}
        return {
            'car': n.car_name, 't': time.time(),
            'mode': ('charging' if n.charging != 'off' else 'exploring' if n.explorer.active()
                     else 'navigating' if nav['running'] else 'mapping' if n.slam.running() else 'idle'),
            'drive': self.drive_state(),
            'odom': self.odom,
            'map_pose': (n.slam.pose if n.slam.running() else nav.get('pose')),
            'imu': self.imu_state(),
            'lidar': self.lidar_summary(),
            'battery': self.battery(),
            'safety': {'lifted': n.lift.lifted, 'bumps': n.stall.bumps, 'note': n.stall.note},
            'depth': {k: dc.get(k) for k in ('nearest_camera_m', 'nearest_lidar_m', 'nearest_camera_only_m', 'drop_dirs')
                      if k in dc} | ({'floor_ok_pct': floor.get('ok_pct'), 'floor_why': floor.get('why')} if floor else {}),
            'slam': n.slam.running(),
            'nav': {k: nav.get(k) for k in ('running', 'map_name', 'goal', 'goal_state')},
            'explore': {k: v for k, v in n.explorer.status().items() if k in ('state', 'message', 'mapped_m2')},
        }

    def depth_png(self):
        m = self.depth_msg
        if m is None:
            return None
        img = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        ok, png = cv2.imencode('.png', img)
        return png.tobytes() if ok else None

    def depth_camera(self):
        d = self.node.depth_obs
        K = getattr(d, 'K', None)
        if K is None:
            return None
        m = self.depth_msg
        return {'fx': K[0], 'fy': K[1], 'cx': K[2], 'cy': K[3],
                'width': m.width if m else None, 'height': m.height if m else None, 'depth_unit_m': 0.001,
                'R_optical_to_base': np.asarray(d.R).round(6).tolist() if getattr(d, 'R', None) is not None else None,
                't_camera_in_base_m': np.asarray(d.t).round(4).tolist() if getattr(d, 't', None) is not None else None,
                'note': 'point in base_footprint = R @ (depth * [(u - cx) / fx, (v - cy) / fy, 1]) + t'}

    def video_url(self, view, q):
        url = f'{VIDEO}/stream?topic={VIEWS[view]}&type=mjpeg&quality={int(q.get("quality", 70))}'
        for k in ('width', 'height'):
            if q.get(k):
                url += f'&{k}={int(q[k])}'
        return url

    def snapshot(self, view):
        with urllib.request.urlopen(f'{VIDEO}/snapshot?topic={VIEWS[view]}', timeout=5) as r:
            return r.read()

    def index(self):
        return {'api': VERSION, 'car': self.node.car_name,
                'limits': {'speed_m_s': MAX_SPEED, 'steer_deg': MAX_STEER_DEG, 'duration_s': MAX_DURATION_S},
                'docs': 'mentorpi/web_ui/robot_api.py and README "API"', 'endpoints': [
                    'GET /api/v1/state', 'GET /api/v1/stream?hz=5', 'GET /api/v1/battery', 'GET /api/v1/odom',
                    'GET /api/v1/imu', 'GET /api/v1/lidar', 'GET /api/v1/camera.jpg?view=rgb|depth',
                    'GET /api/v1/video?view=rgb|depth&width=&quality=', 'GET /api/v1/depth.png', 'GET /api/v1/depth/camera',
                    'POST /api/v1/drive {speed, steer, duration, safe, dry}', 'POST /api/v1/stop', 'POST /api/v1/horn',
                    'POST /api/v1/charging {on}']}


# ---- HTTP (called from web_ui_server.py's handler) -----------------------------------------------------
def _query(h):
    import urllib.parse
    url = urllib.parse.urlparse(h.path)
    return url.path, {k: v[-1] for k, v in urllib.parse.parse_qs(url.query).items()}


def _body(h, q):
    """JSON body (or the query string) as a dict."""
    n = int(h.headers.get('Content-Length') or 0)
    if n:
        try:
            b = json.loads(h.rfile.read(min(n, 65536)) or b'{}')
            if isinstance(b, dict):
                return {**q, **b}
        except ValueError:
            pass
    return q


def _bool(v, default):
    if v is None:
        return default
    if isinstance(v, bool):
        return v
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def handle_get(h, api):
    path, q = _query(h)
    if path in ('/api/v1', '/api/v1/'):
        return h._send(200, api.index())
    if path == '/api/v1/state':
        return h._send(200, api.state())
    if path == '/api/v1/battery':
        return h._send(200, api.battery())
    if path == '/api/v1/odom':
        return h._send(200 if api.odom else 503, api.odom or {'error': 'no odometry yet'})
    if path == '/api/v1/imu':
        return h._send(200, api.imu_state())
    if path == '/api/v1/lidar':
        s = api.lidar()
        return h._send(200 if s else 503, s or {'error': 'no lidar data'})
    if path == '/api/v1/depth/camera':
        c = api.depth_camera()
        return h._send(200 if c else 503, c or {'error': 'depth camera not ready'})
    if path == '/api/v1/depth.png':
        png = api.depth_png()
        return h._send(200, png, 'image/png') if png else h._send(503, {'error': 'no depth image yet'})
    if path in ('/api/v1/camera.jpg', '/api/v1/video'):
        view = q.get('view', 'rgb')
        if view not in VIEWS:
            return h._send(400, {'error': 'view=rgb or view=depth'})
        if path == '/api/v1/camera.jpg':
            try:
                return h._send(200, api.snapshot(view), 'image/jpeg')
            except Exception as exc:
                return h._send(503, {'error': f'camera: {exc}'})
        return _proxy_stream(h, api.video_url(view, q))
    if path == '/api/v1/stream':
        return _sse(h, api, q)
    return h._send(404, {'error': 'not found', 'see': 'GET /api/v1'})


def handle_post(h, api):
    path, q = _query(h)
    b = _body(h, q)
    if path == '/api/v1/drive':
        r = api.drive(b.get('speed', 0), b.get('steer', 0), b.get('duration', DEFAULT_DURATION_S),
                      _bool(b.get('safe'), True), _bool(b.get('dry'), False), who=h._who())
        return h._send(200 if r['ok'] else 409, r)
    if path == '/api/v1/stop':
        return h._send(200, api.stop())
    if path == '/api/v1/horn':
        return h._send(200, api.horn())
    if path == '/api/v1/charging':
        if 'on' not in b:
            return h._send(400, {'ok': False, 'message': 'body {"on": true} or {"on": false}'})
        on = _bool(b['on'], False)
        api.stop('charging mode')
        api.node.request_charging(on)
        return h._send(200, {'ok': True, 'message': f'charging mode {"on" if on else "off"} requested '
                                                    '(the robot software stops / starts: takes ~10-60 s)'})
    return h._send(404, {'error': 'not found', 'see': 'GET /api/v1'})


def _proxy_stream(h, url):
    """Pass web_video_server's MJPEG stream through, so a program needs only this port."""
    try:
        up = urllib.request.urlopen(url, timeout=10)
    except Exception as exc:
        return h._send(503, {'error': f'video: {exc}'})
    try:
        h.send_response(200)
        h.send_header('Content-Type', up.headers.get('Content-Type', 'multipart/x-mixed-replace;boundary=boundarydonotcross'))
        h.end_headers()
        while chunk := up.read1(65536):
            h.wfile.write(chunk)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        up.close()


def _sse(h, api, q):
    try:
        hz = max(0.2, min(20.0, float(q.get('hz', 5))))
    except ValueError:
        hz = 5.0
    h.send_response(200)
    h.send_header('Content-Type', 'text/event-stream')
    h.end_headers()
    try:
        while True:
            h.wfile.write(b'data: ' + json.dumps(api.state()).encode() + b'\n\n')
            h.wfile.flush()
            time.sleep(1.0 / hz)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
