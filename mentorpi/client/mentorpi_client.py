"""Python client for the car's API (web_ui/robot_api.py), for programs on another computer.

Only the Python standard library (3.8+); copy this one file next to your program. numpy / OpenCV are optional,
for turning pictures into arrays (see example.py).

    from mentorpi_client import Car
    car = Car('http://192.168.1.50:8000')         # or https://<car>.<tailnet>.ts.net, or https://<car ip>
    print(car.battery())                           # voltage, charge %, charging / in use, charging mode
    print(car.state()['lidar'])                    # nearest thing front / left / back / right
    full = car.sensors()                           # FULL data: every field of every sensor's latest message
    full['topics']['imu_raw']['msg']               # e.g. the raw IMU: orientation, angular_velocity, ...
    open('pic.jpg', 'wb').write(car.picture())     # one camera picture (JPEG)
    for jpeg in car.video():                       # live video, one JPEG per frame
        ...
    for s in car.stream(hz=5):                     # live state, 5 times a second
        ...
    car.drive_for(0.15, steer=0, seconds=2)        # drive 2 s at 0.15 m/s, straight; stops for obstacles
    car.stop()

Driving: a command holds for `duration` seconds (at most 3), then the car stops by itself, so a program that
dies stops the car. drive_for() keeps sending for longer drives. safe=True (default): the car refuses / stops
when the lidar sees something in its path. Speeds in m/s (+ forward), steering in degrees (+ left, at most 40;
the car steers its front wheels: it can't turn on the spot).
"""
import json
import socket
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request


class CarError(Exception):
    pass


class Car:
    def __init__(self, url, verify=True, timeout=5.0):
        """url: the page's address, e.g. http://<car ip>:8000, https://<car>.<tailnet>.ts.net or https://<car ip>.
        verify=False: accept the self-signed certificate of https://<car ip> (local network)."""
        self.base = url.rstrip('/') + '/api/v1'
        self.timeout = timeout
        self.ctx = None
        self.resolved = False
        if not verify:          # also for http://<car ip>, which redirects to https://<car ip>
            self.ctx = ssl.create_default_context()
            self.ctx.check_hostname = False
            self.ctx.verify_mode = ssl.CERT_NONE

    def _resolve(self):
        """Follow a redirect once (http://<car ip> -> https://<car ip>) and keep the address it led to, so drive
        commands (POST) go straight there."""
        self.resolved = True
        try:
            with urllib.request.urlopen(self.base, timeout=self.timeout, context=self.ctx) as r:
                final = r.geturl().rstrip('/')
        except (urllib.error.URLError, socket.timeout, ValueError):
            return
        if final != self.base and final.endswith('/api/v1'):
            self.base = final

    # ---- plumbing ------------------------------------------------------------------------------
    def _open(self, path, body=None, timeout=None, query=None):
        if not self.resolved:
            self._resolve()
        url = self.base + path + ('?' + urllib.parse.urlencode(query) if query else '')
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data, {'Content-Type': 'application/json'} if data is not None else {},
                                     method='POST' if data is not None else 'GET')
        try:
            return urllib.request.urlopen(req, timeout=timeout or self.timeout, context=self.ctx)
        except urllib.error.HTTPError as e:
            if e.code == 409:              # drive refused (obstacle, charging, ...): the answer says why
                return e
            raise CarError(f'{e.code} {path}: {e.read()[:300].decode(errors="replace")}') from None
        except (urllib.error.URLError, socket.timeout) as e:
            raise CarError(f'cannot reach {url}: {getattr(e, "reason", e)}') from None

    def _get(self, path, **query):
        with self._open(path, query=query) as r:
            return json.load(r)

    def _post(self, path, body):
        with self._open(path, body=body) as r:
            return json.load(r)

    def _bytes(self, path, **query):
        with self._open(path, query=query, timeout=10) as r:
            return r.read()

    # ---- reading -------------------------------------------------------------------------------
    def info(self):
        """Car name, limits, endpoints."""
        return self._get('')

    def state(self):
        """Everything at once: mode, drive, odom, map_pose, imu, lidar (summary), battery, safety, depth, slam,
        nav, explore."""
        return self._get('/state')

    def sensors(self, images=False):
        """FULL sensor data. 'topics': the latest message of every sensor topic, every field as ROS has it
        (lidar, imu_raw, imu, imu_corrected, imu_rpy, imu_steady, odom_wheels, odom, odom_lidar, battery_mv,
        depth_scan, depth_bumps, depth_drops, rgb_camera_info, depth_camera_info, command, button), each with
        topic, type, about, received (car clock), age_s, hz, count and msg (None: no message yet; most messages
        have header.stamp = when the sensor measured). Also 'battery' (estimate), 'depth_compare',
        'frames' (where the sensors sit on the car) and 'state'. images=True: also 'rgb_jpeg_base64' and
        'depth_png_base64'. The first call takes 1-4 s (the car starts listening to every topic)."""
        with self._open('/sensors', query={'images': int(bool(images))}, timeout=max(self.timeout, 15)) as r:
            return json.load(r)

    def depth_points(self, step=4, ply=False):
        """3D points (m) from the depth image, in the car's frame (x forward, y left, z up): a list of [x, y, z],
        every step-th pixel (step 4: ~19000 points). ply=True: the PLY file's bytes (MeshLab, CloudCompare, Open3D)."""
        if ply:
            return self._bytes('/depth/points', step=step, format='ply')
        return self._get('/depth/points', step=step)['points']

    def battery(self):
        """voltage, level (ok / low / critical), soc (%), state (charging / discharging / steady / full /
        collecting), eta_min + eta_kind (to empty / to full), charging_mode (on / switching / off)."""
        return self._get('/battery')

    def odom(self):
        """EKF odometry since start: x, y (m), yaw_deg, speed (m/s), turn_rate_deg_s."""
        return self._get('/odom')

    def imu(self):
        """roll_deg, pitch_deg, yaw_deg, accel_m_s2 [x, y, z], gyro_deg_s [x, y, z]."""
        return self._get('/imu')

    def lidar(self):
        """The whole scan: angle_min, angle_increment (rad), ranges (m, None = no reading), frame."""
        return self._get('/lidar')

    def lidar_points(self):
        """The scan as (x, y) points in m, in the car's frame (x forward, y left)."""
        import math
        s = self.lidar()
        x0 = -0.010                        # lidar centre from the car's origin (car_geometry.py)
        return [(x0 + r * math.cos(s['angle_min'] + i * s['angle_increment']),
                 r * math.sin(s['angle_min'] + i * s['angle_increment']))
                for i, r in enumerate(s['ranges']) if r is not None]

    def picture(self, view='rgb'):
        """One camera picture, JPEG bytes. view='depth': the depth image, colourised."""
        return self._bytes('/camera.jpg', view=view)

    def depth_png(self):
        """The latest depth image: 16-bit PNG bytes, millimetres (0 = no reading). With OpenCV:
        cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)."""
        return self._bytes('/depth.png')

    def depth_camera(self):
        """Depth camera intrinsics (fx, fy, cx, cy) and mounting (R, t), to turn depth pixels into points."""
        return self._get('/depth/camera')

    def video(self, view='rgb', width=None, quality=70, with_time=False):
        """Live video: yields one JPEG (bytes) per frame until you stop iterating. with_time=True: yields
        (jpeg, stamp), stamp = when the camera took the frame (car clock, s since 1970; None if not sent)."""
        q = {'view': view, 'quality': quality}
        if width:
            q['width'] = width
        r = self._open('/video', query=q, timeout=10)
        try:
            while True:
                length, ts = None, None
                while True:                                    # part headers
                    line = r.readline()
                    if not line:
                        return
                    line = line.strip()
                    low = line.lower()
                    if low.startswith(b'content-length:'):
                        length = int(line.split(b':', 1)[1])
                    elif low.startswith(b'x-timestamp:'):
                        try:
                            ts = float(line.split(b':', 1)[1])
                        except ValueError:
                            pass
                    elif not line and length is not None:
                        break
                jpeg = r.read(length)
                yield (jpeg, ts) if with_time else jpeg
        finally:
            r.close()

    def stream(self, hz=5, full=False):
        """Live state (like state()), hz times a second, until you stop iterating. full=True: the full sensor
        data (like sensors()) each time."""
        r = self._open('/stream', query={'hz': hz, 'full': int(bool(full))}, timeout=15)
        try:
            for line in r:
                if line.startswith(b'data: '):
                    yield json.loads(line[6:])
        finally:
            r.close()

    # ---- driving -------------------------------------------------------------------------------
    def drive(self, speed, steer=0.0, duration=0.5, safe=True, dry=False):
        """One drive command: speed m/s (+ forward), steer degrees (+ left), held for duration s (<= 3), then the
        car stops by itself. Returns the answer: ok, message, and blocked=True when an obstacle is in the path.
        dry=True: only check (would it drive? how much room is there?)."""
        return self._post('/drive', {'speed': speed, 'steer': steer, 'duration': duration, 'safe': safe, 'dry': dry})

    def drive_for(self, speed, steer=0.0, seconds=1.0, safe=True, every=0.25):
        """Drive for `seconds` (any length), sending a command every `every` s. Stops at the end, on an obstacle,
        on Ctrl-C, or when the page takes over. Returns the last answer."""
        end = time.monotonic() + seconds
        r = {'ok': True}
        try:
            while True:
                left = end - time.monotonic()
                if left <= 0:
                    break
                r = self.drive(speed, steer, duration=min(left, every * 2.5), safe=safe)
                if not r.get('ok'):
                    return r
                time.sleep(min(every, left))
                st = self._get('/state')['drive']
                if not st['active'] and time.monotonic() < end - 0.05:      # stopped by the car (obstacle, page)
                    return st['last'] or r
        finally:
            self.stop()
        return r

    def stop(self):
        return self._post('/stop', {})

    def horn(self):
        return self._post('/horn', {})

    def charging(self, on):
        """Charging mode on: the robot software stops (sensors off) so the battery charges faster; off: back on
        (~10-60 s)."""
        return self._post('/charging', {'on': bool(on)})

    # ---- more (the page's own endpoints) --------------------------------------------------------
    def say(self, text):
        """The car speaks (its voice module)."""
        return self._raw_post('/api/audio/say', text=text)

    def nav_go(self, x, y, yaw=0.0):
        """Navigation (needs navigation running on a saved map: page -> Mission): drive to x, y (m) on the map."""
        return self._raw_post('/api/nav/go', x=x, y=y, yaw=yaw)

    def nav_cancel(self):
        return self._raw_post('/api/nav/cancel')

    def _raw_post(self, path, **query):
        if not self.resolved:
            self._resolve()
        url = self.base[:-len('/api/v1')] + path + ('?' + urllib.parse.urlencode(query) if query else '')
        req = urllib.request.Request(url, b'', method='POST')
        try:
            with urllib.request.urlopen(req, timeout=self.timeout, context=self.ctx) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            return json.loads(e.read() or b'{}')
