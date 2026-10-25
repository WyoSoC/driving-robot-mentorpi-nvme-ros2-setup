"""Recording of every mapping and navigation run (runs inside web_ui_server.py), to look at a run afterwards:
where the car went, what it saw, and why it stopped, backed up or got stuck.

A run lasts while something drives the car with a map:
- "autonomous mapping"  the explorer (from this page, from another car, or a dry run)
- "mapping"             SLAM started by hand (driven with the keys / gamepad); becomes "autonomous mapping"
                        if the explorer starts during it, and stays so when the explorer ends and SLAM goes on
- "navigation"          Nav2 on a saved map
A run with no movement for IDLE_END_S is closed ("idle"); the next one starts when the car moves again.
A run in which the car never moved (and that lasted < SHORT_S) is thrown away.

Folder RUNS_DIR/YYYYMMDD_HHMMSS_<car>/:
- events.jsonl  one JSON object per line, t = seconds since the start:
                {"type": "state"}   explorer state / message / goal, navigation goal state, waiting for
                                    another car, shared map joined: whenever any of it changes
                {"type": "pose"}    the car's pose (map frame: SLAM, else Nav2) every POSE_S
                {"type": "nav2"}    every WARN / ERROR line of Nav2 (nav.log), with the pose at that moment
                {"type": "warn"}    bump, lift, tip (stall / lift detectors), with the pose
- frames/       camera pictures, FRAME_S apart, FRAME_W x FRAME_H JPEG; the name is t (e.g. 0012.0.jpg)
- depth/        depth images, DEPTH_S apart: every DEPTH_STEP-th pixel (320 x 240), 16-bit PNG in mm (lossless;
                ~60 KB); depth/camera.json: intrinsics of these images and the camera's mounting (R, t in
                base_footprint); depth/index.jsonl: per image the floor fit (plane or why not). Replay offline:
                depth_replay.py (floor fit, obstacles, bumps, drops on real floors)
- map.png, map.json   the map (the SLAM map as the page shows it, every MAP_S and at the end; the saved
                map when navigating); map.json: resolution, origin (x, y of the bottom left corner), map name,
                view (mapping together: x, y, yaw from the car's frame, which the poses are in, to the map's)
- nav.log       Nav2's whole output (it is overwritten when navigation starts next time)
- bag/          ros2 bag (sqlite, not compressed: compressing at the end took > 15 s on the busy Pi and
                a cut-off bag can't be read; ~16 MB a minute): lidar, depth obstacles, odometry, TF,
                both costmaps, the maps, Nav2's plan, the controller's lookahead point and collision arc,
                the drive commands, the EKF inputs (TOPICS; the EKF can be replayed)
- summary.json  duration, distance, area mapped, goals, the saved map, warnings, Nav2 warnings by kind
- run.json      what was started (kind, radius, dry run, together, map)

The newest KEEP runs are kept, and at most MAX_TOTAL_MB of them. RUNS_DIR is the container's
/home/ubuntu/shared (~/docker/tmp on the car).
HTTP (web_ui_server.py): GET /api/runs (list), /api/runs/<id> (details + timeline), /api/runs/<id>/map.png,
/api/runs/<id>/frame/<name>.jpg, /api/runs/<id>/download (zip); POST /api/runs/<id>/delete.
"""
import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import zipfile
from datetime import datetime

RUNS_DIR = '/home/ubuntu/shared/runs'
KEEP = 50
MAX_TOTAL_MB = 10000
SAMPLE_S = 0.5
POSE_S = 1.0
MAP_S = 30.0
FRAME_S = 1.0
FRAME_W, FRAME_H, FRAME_Q = 320, 240, 60
IDLE_END_S = 600.0         # no movement this long: close the run
MOVE_M = 0.05              # "moved": this far, or MOVE_RAD turned
MOVE_RAD = 0.1
RESTART_M = 0.2            # after an idle end: a new run once the car is this far from where it stood
SHORT_S = 60.0
DEPTH_S = 1.0
DEPTH_STEP = 2
RGB_TOPIC = '/ascamera/camera_publisher/rgb0/image'
NAV_LOG = '/tmp/nav.log'
TOPICS = ['/scan_raw', '/scan_nav', '/depth/scan', '/depth/bumps', '/odom', '/tf', '/tf_static',
          '/map', '/team/map', '/local_costmap/costmap', '/global_costmap/costmap',
          '/plan', '/received_global_plan', '/lookahead_point', '/lookahead_collision_arc',
          '/nav/cmd_vel', '/cmd_vel', '/fleet/cloud', '/bump/cloud', '/imu', '/depth/drops', '/odom_lidar',
          '/odom_raw', '/odom_lidar_speed', '/imu_corrected']   # EKF inputs: replayable
ANSI = re.compile(r'\x1b\[[0-9;]*m')
NAV2_LINE = re.compile(r'\[(WARN|ERROR)\]\s*\[[0-9.]+\]\s*\[([^\]]+)\]:\s*(.*)')
RUN_ID = re.compile(r'^\d{8}_\d{6}_[A-Za-z0-9_-]{1,40}$')
FRAME_NAME = re.compile(r'^\d{4,6}\.\d\.jpg$')


def kind(msg):
    """A Nav2 message without its numbers, for counting ('failed to generate a valid path to (N, N)')."""
    return re.sub(r'-?\d+(\.\d+)?', 'N', msg)[:160]


def dir_size(d):
    return sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(d) for f in fs)


def compact_events(lines):
    """events.jsonl -> {'poses': [[t, x, y, yaw]], 'events': [other events]} for the page."""
    poses, events = [], []
    for line in lines:
        try:
            ev = json.loads(line)
        except ValueError:
            continue           # a line cut off by a crash
        if ev.get('type') == 'pose' and ev.get('pose'):
            poses.append([ev['t']] + ev['pose'][:3])
        else:
            events.append(ev)
    return {'poses': poses, 'events': events}


class RunRecorder:
    def __init__(self, node, explorer, nav, slam, me, team=None):
        self.node, self.explorer, self.nav, self.slam, self.me, self.team = node, explorer, nav, slam, me, team
        self.log = node.get_logger()
        self.run = None            # current run: {'id', 'dir', 't0', 'events', 'bag', ...}
        self.idle_at = None        # where the car stood when a run was closed for being idle
        self.cam_sub = None
        self.bridge = None
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- what is running ----------------------------------------------------------------
    def _want(self):
        if self.explorer.active():
            return 'autonomous mapping'
        if self.slam.running():
            return 'mapping'
        if self.nav.running():
            return 'navigation'
        return None

    def _pose(self):
        p = self.slam.pose if self.slam.running() else self.nav.pose
        return [round(v, 3) for v in p[:3]] if p else None

    # ---- start / end -------------------------------------------------------------------
    def _begin(self, what):
        ex = self.explorer
        rid = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{self.me}'
        d = os.path.join(RUNS_DIR, rid)
        os.makedirs(os.path.join(d, 'frames'), exist_ok=True)
        os.makedirs(os.path.join(d, 'depth'), exist_ok=True)
        info = {'id': rid, 'car': self.me, 'kind': what, 'started': time.time()}
        if what == 'autonomous mapping':
            info.update(radius_m=ex.radius, dry_run=ex.dry, together=getattr(ex, 'wait_join', False))
        if what == 'navigation':
            info['map'] = self.nav.map_name
        self._write_json(d, 'run.json', info)
        bag = None
        try:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(d, 'bag')] + TOPICS,
                                   start_new_session=True, stdout=open(os.path.join(d, 'bag.log'), 'w'),
                                   stderr=subprocess.STDOUT)
        except OSError as exc:
            self.log.warn(f'run recording: no bag ({exc})')
        with self.lock:
            self.run = {'id': rid, 'dir': d, 'info': info, 't0': time.monotonic(),
                        'events': open(os.path.join(d, 'events.jsonl'), 'w'),
                        'bag': bag, 'last': None, 'pose_at': 0.0, 'nav_pos': None, 'counts': {}, 'n_nav2': 0,
                        'map_at': 0.0, 'frame_at': 0.0, 'frames': 0, 'dist': 0.0, 'prev': None,
                        'still': None, 'moved_at': time.monotonic(), 'warn': self._warn_counts(), 'warnings': 0,
                        'depth_at': 0.0, 'depth': 0}
        if what == 'navigation':
            self._copy_saved_map(self.run)
        self._camera(True)
        dobs = getattr(self.node, 'depth_obs', None)
        if dobs is not None:
            dobs.on_frame = self._on_depth
        self.log.info(f'run recording started ({what}): {d}')
        self._prune()

    def _end(self, reason=None):
        with self.lock:
            r, self.run = self.run, None
        if r is None:
            return
        self._camera(False)
        dobs = getattr(self.node, 'depth_obs', None)
        if dobs is not None:
            dobs.on_frame = None
        self._tail_nav_log(r)
        if r['info']['kind'] != 'navigation':
            self._save_map(r)
        if r['bag'] is not None and r['bag'].poll() is None:
            os.killpg(r['bag'].pid, signal.SIGINT)        # let ros2 bag close the file properly
            try:
                r['bag'].wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(r['bag'].pid, signal.SIGKILL)
        try:
            shutil.copy(NAV_LOG, os.path.join(r['dir'], 'nav.log'))
        except OSError:
            pass
        duration = time.monotonic() - r['t0']
        r['events'].close()
        if r['dist'] < MOVE_M and duration < SHORT_S and r['info']['kind'] != 'autonomous mapping':
            shutil.rmtree(r['dir'], ignore_errors=True)
            self.log.info(f'run recording dropped (the car did not move): {r["dir"]}')
            return
        summary = {'id': r['id'], 'kind': r['info']['kind'], 'duration_s': round(duration),
                   'distance_m': round(r['dist'], 2), 'frames': r['frames'], 'depth_frames': r['depth'],
                   'end_reason': reason,
                   'warnings': r['warnings'], 'nav2_messages': r['n_nav2'],
                   'nav2_by_kind': dict(sorted(r['counts'].items(), key=lambda kv: -kv[1]))}
        if r['info']['kind'] == 'autonomous mapping':
            ex = self.explorer.status()
            summary.update(end_state=ex.get('state'), end_message=ex.get('message'), mapped_m2=ex.get('mapped_m2'),
                           goals=ex.get('goals'), saved_as=ex.get('saved_as'), team_saved_as=ex.get('team_saved_as'))
        elif r['info']['kind'] == 'mapping':
            summary['mapped_m2'] = r.get('mapped_m2')
        else:
            summary.update(end_state=self.nav.goal_state, end_message=self.nav.message)
        self._write_json(r['dir'], 'summary.json', summary)
        self.log.info(f'run recording finished: {r["dir"]} ({summary["duration_s"]} s, {summary["distance_m"]} m, '
                      f'{r["n_nav2"]} Nav2 warnings / errors)')

    def _upgrade(self, r):
        """Mapping by hand -> the explorer took over."""
        ex = self.explorer
        r['info'].update(kind='autonomous mapping', radius_m=ex.radius, dry_run=ex.dry,
                         together=getattr(ex, 'wait_join', False), explorer_from_s=round(time.monotonic() - r['t0']))
        self._write_json(r['dir'], 'run.json', r['info'])

    @staticmethod
    def _write_json(d, name, obj):
        tmp = os.path.join(d, name + '.tmp')
        with open(tmp, 'w') as f:
            json.dump(obj, f, indent=1)
        os.replace(tmp, os.path.join(d, name))

    def _prune(self):
        try:
            runs = sorted(n for n in os.listdir(RUNS_DIR) if os.path.isdir(os.path.join(RUNS_DIR, n)))
        except OSError:
            return
        for n in runs[:-KEEP]:
            shutil.rmtree(os.path.join(RUNS_DIR, n), ignore_errors=True)
        runs = runs[-KEEP:]
        size = {n: dir_size(os.path.join(RUNS_DIR, n)) for n in runs}
        while len(runs) > 1 and sum(size[n] for n in runs) > MAX_TOTAL_MB * 1e6:
            shutil.rmtree(os.path.join(RUNS_DIR, runs[0]), ignore_errors=True)
            runs.pop(0)

    # ---- map and camera ----------------------------------------------------------------
    def _save_map(self, r):
        png, meta, grid = self.slam.png, self.slam.meta, self.slam.grid
        view = getattr(self.slam, 'view', None) if getattr(self.slam, 'team_map', None) is not None else None
        if not png or not meta:
            return
        with open(os.path.join(r['dir'], 'map.png.tmp'), 'wb') as f:
            f.write(png)
        os.replace(os.path.join(r['dir'], 'map.png.tmp'), os.path.join(r['dir'], 'map.png'))
        self._write_json(r['dir'], 'map.json', {'resolution': meta['resolution'], 'origin': meta['origin'][:2],
                                                'width': meta['width'], 'height': meta['height'], 'map': None,
                                                'view': view})   # team map: car frame -> map frame
        if grid is not None:
            import numpy as np
            g = np.asarray(grid.data, dtype=np.int8)
            r['mapped_m2'] = round(int(np.count_nonzero((g >= 0) & (g < 30))) * grid.info.resolution ** 2, 1)  # seen free

    def _copy_saved_map(self, r):
        name = self.nav.map_name
        try:
            import map_store
            m = map_store.meta(name)
            with open(os.path.join(r['dir'], 'map.png'), 'wb') as f:
                f.write(map_store.png_bytes(name))
            self._write_json(r['dir'], 'map.json', {'resolution': m['resolution'], 'origin': list(m['origin'][:2]),
                                                    'map': name})
        except Exception as exc:
            self.log.warn(f'run recording: cannot copy the map "{name}": {exc!r}')

    def _camera(self, on):
        """Subscribe to the colour camera only while recording (a raw image costs CPU even when unused)."""
        try:
            if on and self.cam_sub is None:
                from rclpy.qos import qos_profile_sensor_data
                from sensor_msgs.msg import Image
                if self.bridge is None:
                    from cv_bridge import CvBridge
                    self.bridge = CvBridge()
                self.cam_sub = self.node.create_subscription(Image, RGB_TOPIC, self._on_image,
                                                             qos_profile_sensor_data)
            elif not on and self.cam_sub is not None:
                sub, self.cam_sub = self.cam_sub, None
                self.node.destroy_subscription(sub)
        except Exception as exc:
            self.log.warn(f'run recording: camera: {exc!r}')

    def _on_image(self, msg):
        r = self.run
        now = time.monotonic()
        if r is None or now - r['frame_at'] < FRAME_S:
            return
        r['frame_at'] = now
        try:
            import cv2
            enc = msg.encoding.lower()
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8' if enc in ('rgb8', 'bgr8') else 'passthrough')
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            img = cv2.resize(img, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, FRAME_Q])
            if ok:
                with open(os.path.join(r['dir'], 'frames', f'{now - r["t0"]:06.1f}.jpg'), 'wb') as f:
                    f.write(jpg.tobytes())
                r['frames'] += 1
        except Exception as exc:
            if r['frames'] == 0:
                self.log.warn(f'run recording: camera frame: {exc!r}')
                r['frame_at'] = now + 30          # don't log this every second

    def _on_depth(self, msg, floor, why):
        """depth_obstacles.py, each processed depth frame: keep one every DEPTH_S (16-bit PNG, mm)."""
        r = self.run
        now = time.monotonic()
        if r is None or now - r['depth_at'] < DEPTH_S:
            return
        r['depth_at'] = now
        import cv2
        import numpy as np
        dd = os.path.join(r['dir'], 'depth')
        if r['depth'] == 0:
            dobs = self.node.depth_obs
            fx, fy, cx, cy = dobs.K
            k = DEPTH_STEP
            self._write_json(dd, 'camera.json', {
                'note': 'pixel (u, v) of these images = (k*u, k*v) of the camera; intrinsics below are for these',
                'step': k, 'width': (msg.width + k - 1) // k, 'height': (msg.height + k - 1) // k,
                'fx': fx / k, 'fy': fy / k, 'cx': cx / k, 'cy': cy / k, 'depth_unit_m': 0.001,
                'R_optical_to_base': np.asarray(dobs.R).round(6).tolist(),
                't_camera_in_base_m': np.asarray(dobs.t).round(4).tolist(), 'encoding': msg.encoding})
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        img = np.ascontiguousarray(img[::DEPTH_STEP, ::DEPTH_STEP])
        ok, png = cv2.imencode('.png', img)
        if not ok:
            return
        t = now - r['t0']
        name = f'{t:06.1f}.png'
        with open(os.path.join(dd, name), 'wb') as f:
            f.write(png.tobytes())
        with open(os.path.join(dd, 'index.jsonl'), 'a') as f:
            f.write(json.dumps({'t': round(t, 2), 'file': name,
                                'floor': None if floor is None else [round(float(v), 5) for v in floor],
                                'why': why or None}) + '\n')
        r['depth'] += 1

    # ---- while running -----------------------------------------------------------------
    def _write(self, r, ev):
        ev['t'] = round(time.monotonic() - r['t0'], 2)
        r['events'].write(json.dumps(ev) + '\n')

    def _warn_counts(self):
        lift, stall = getattr(self.node, 'lift', None), getattr(self.node, 'stall', None)
        return {'lift': getattr(lift, 'lifts', 0), 'tip': getattr(lift, 'tips', 0), 'bump': getattr(stall, 'bumps', 0)}

    def _sample(self, r):
        ex, nv = self.explorer, self.nav
        sh = self.team.shared if self.team is not None else None
        st = {'state': ex.state, 'message': ex.message, 'goal': ex.goal,
              'nav_goal_state': nv.goal_state, 'nav_message': nv.message,
              'joined': sh.joined() if sh is not None else None}
        if st != r['last']:
            r['last'] = dict(st)
            self._write(r, dict(st, type='state'))
        now = time.monotonic()
        pose = self._pose()
        if pose is not None:
            if r['prev'] is not None:
                r['dist'] += math.hypot(pose[0] - r['prev'][0], pose[1] - r['prev'][1])
            r['prev'] = pose
            s = r['still']
            if s is None or math.hypot(pose[0] - s[0], pose[1] - s[1]) > MOVE_M or \
                    abs(math.remainder(pose[2] - s[2], math.tau)) > MOVE_RAD:
                r['still'], r['moved_at'] = pose, now
            if now - r['pose_at'] >= POSE_S:
                r['pose_at'] = now
                self._write(r, {'type': 'pose', 'pose': pose})
        counts = self._warn_counts()
        for k, n in counts.items():
            if n > r['warn'].get(k, 0):
                lift, stall = getattr(self.node, 'lift', None), getattr(self.node, 'stall', None)
                note = getattr(stall if k == 'bump' else lift, 'note', '')
                self._write(r, {'type': 'warn', 'kind': k, 'note': note, 'pose': pose})
                r['warnings'] += 1
        r['warn'] = counts
        if r['info']['kind'] != 'navigation' and now - r['map_at'] >= MAP_S:
            r['map_at'] = now
            self._save_map(r)
        self._tail_nav_log(r)
        r['events'].flush()

    def _idle(self, r):
        """No movement for IDLE_END_S, no goal, explorer not running."""
        return (time.monotonic() - r['moved_at'] > IDLE_END_S and not self.explorer.active()
                and self.nav.goal_state not in ('sending', 'active', 'yielding', 'nudging'))

    def _tail_nav_log(self, r):
        """New WARN / ERROR lines of nav.log (a new file when navigation starts: follow it)."""
        try:
            st = os.stat(NAV_LOG)
        except OSError:
            return
        pos = r['nav_pos']
        if pos is None or pos[0] != st.st_ino or pos[1] > st.st_size:
            if pos is None and st.st_mtime < time.time() - (time.monotonic() - r['t0']) - 1:
                pos = (st.st_ino, st.st_size)   # an old log from before this run: only what comes next
            else:
                pos = (st.st_ino, 0)
        if st.st_size <= pos[1]:
            r['nav_pos'] = pos
            return
        with open(NAV_LOG, 'rb') as f:
            f.seek(pos[1])
            data = f.read()
        end = data.rfind(b'\n') + 1                 # whole lines only
        r['nav_pos'] = (pos[0], pos[1] + end)
        pose = self._pose()
        for line in data[:end].decode(errors='replace').splitlines():
            m = NAV2_LINE.search(ANSI.sub('', line))
            if not m:
                continue
            level, who, msg = m.groups()
            k = f'{who}: {kind(msg)}'
            r['counts'][k] = r['counts'].get(k, 0) + 1
            r['n_nav2'] += 1
            self._write(r, {'type': 'nav2', 'level': level, 'node': who, 'msg': msg.strip(), 'pose': pose})

    def step(self):
        """One look at what runs (every SAMPLE_S)."""
        want, r = self._want(), self.run
        if r is not None:
            have = r['info']['kind']
            if want is None:
                self._end('stopped')
            elif have == 'mapping' and want == 'autonomous mapping':
                self._upgrade(r)
            elif have == 'autonomous mapping' and want == 'mapping':
                pass                          # explorer done, SLAM goes on: same run
            elif want != have:
                self._end(f'{want} started')
            elif self._idle(r):
                self.idle_at = r['still'] or [0, 0, 0]
                self._end('idle')
        if self.run is None and want is not None:
            if self.idle_at is not None and want != 'autonomous mapping':
                p = self._pose()
                if p is None or math.hypot(p[0] - self.idle_at[0], p[1] - self.idle_at[1]) < RESTART_M:
                    return
            self.idle_at = None
            self._begin(want)
        if want is None:
            self.idle_at = None
        if self.run is not None:
            self._sample(self.run)

    def _loop(self):
        while True:
            time.sleep(SAMPLE_S)
            try:
                self.step()
            except Exception as exc:
                self.log.warn(f'run recording: {exc!r}')

    # ---- HTTP -------------------------------------------------------------------------
    def listing(self):
        out = []
        try:
            names = sorted((n for n in os.listdir(RUNS_DIR) if os.path.isdir(os.path.join(RUNS_DIR, n))), reverse=True)
        except OSError:
            names = []
        for n in names:
            d = os.path.join(RUNS_DIR, n)
            item = {'id': n, 'kind': 'autonomous mapping', 'recording': self.run is not None and self.run['id'] == n}
            for f in ('run.json', 'summary.json'):
                try:
                    with open(os.path.join(d, f)) as fh:
                        item.update(json.load(fh))
                except (OSError, ValueError):
                    pass
            item['id'] = n                         # the folder is the run
            item['size_mb'] = round(dir_size(d) / 1e6, 1)
            out.append(item)
        return {'car': self.me, 'dir': RUNS_DIR, 'runs': out}

    def run_dir(self, rid):
        """The folder of run rid, or None (rid is checked: no paths from the request)."""
        if not RUN_ID.match(rid or ''):
            return None
        d = os.path.join(RUNS_DIR, rid)
        return d if os.path.isdir(d) else None

    def detail(self, rid):
        d = self.run_dir(rid)
        if d is None:
            return None
        out = {'id': rid, 'kind': 'autonomous mapping', 'recording': self.run is not None and self.run['id'] == rid}
        for f in ('run.json', 'summary.json', 'map.json'):
            try:
                with open(os.path.join(d, f)) as fh:
                    if f == 'map.json':
                        out['map_info'] = json.load(fh)
                    else:
                        out.update(json.load(fh))
            except (OSError, ValueError):
                pass
        out['id'] = rid
        if out['recording']:
            out['duration_s'] = round(time.monotonic() - self.run['t0'])
        try:
            with open(os.path.join(d, 'events.jsonl')) as fh:
                out.update(compact_events(fh))
        except OSError:
            out.update(poses=[], events=[])
        try:
            out['frames'] = sorted(n for n in os.listdir(os.path.join(d, 'frames')) if FRAME_NAME.match(n))
        except OSError:
            out['frames'] = []
        out['has_map'] = os.path.exists(os.path.join(d, 'map.png'))
        if not out['has_map'] and self._saved_map(out):        # older runs: the map saved at the end
            import map_store
            m = map_store.meta(self._saved_map(out))
            out['has_map'], out['map_info'] = True, {'resolution': m['resolution'], 'origin': list(m['origin'][:2]),
                                                     'map': self._saved_map(out)}
        out['size_mb'] = round(dir_size(d) / 1e6, 1)
        return out

    @staticmethod
    def _saved_map(info):
        """The map an autonomous mapping run saved, if it still exists (same frame as the run's poses)."""
        import map_store
        name = info.get('saved_as')
        return name if name and name in map_store.names() else None

    def file(self, rid, name):
        """map.png or frames/<t>.jpg of a run: (bytes, content type) or None."""
        d = self.run_dir(rid)
        if d is None:
            return None
        if name == 'map.png':
            path, ctype = os.path.join(d, 'map.png'), 'image/png'
            if not os.path.exists(path):
                try:
                    with open(os.path.join(d, 'summary.json')) as f:
                        saved = self._saved_map(json.load(f))
                    if saved:
                        import map_store
                        return map_store.png_bytes(saved), ctype
                except (OSError, ValueError, KeyError):
                    pass
        elif FRAME_NAME.match(name):
            path, ctype = os.path.join(d, 'frames', name), 'image/jpeg'
        else:
            return None
        try:
            with open(path, 'rb') as f:
                return f.read(), ctype
        except OSError:
            return None

    def write_zip(self, rid, out):
        """Write run rid as a zip (stored, not compressed: the bag and JPEGs don't shrink) to the stream out."""
        d = self.run_dir(rid)
        with zipfile.ZipFile(out, 'w', zipfile.ZIP_STORED, allowZip64=True) as z:
            for dp, _, fs in os.walk(d):
                for f in sorted(fs):
                    p = os.path.join(dp, f)
                    try:
                        z.write(p, os.path.join(rid, os.path.relpath(p, d)))
                    except OSError:
                        pass              # a file that went away while zipping (a live run)

    def delete(self, rid):
        d = self.run_dir(rid)
        if d is None:
            return False, 'no such run'
        if self.run is not None and self.run['id'] == rid:
            return False, 'this run is being recorded; stop it first'
        shutil.rmtree(d, ignore_errors=True)
        self.log.info(f'run deleted: {rid}')
        return True, f'deleted run {rid}'
