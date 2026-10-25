"""Recording of every autonomous mapping run (runs inside web_ui_server.py), to find out afterwards why the
car stopped, backed up or got stuck.

It watches the explorer: when autonomous mapping starts (from this page, from another car, or a dry run)
a run folder RUNS_DIR/YYYYMMDD_HHMMSS_<car>/ is started; when it ends, however it ends, the run is closed:

- events.jsonl  one JSON object per line, t = seconds since the start:
                {"type": "state"}   explorer state / message / goal, navigation goal state, waiting for
                                    another car, shared map joined: whenever any of it changes
                {"type": "pose"}    the car's pose (SLAM map frame) every POSE_S
                {"type": "nav2"}    every WARN / ERROR line of Nav2 (nav.log), with the pose at that moment
- nav.log       Nav2's whole output (it is overwritten when navigation starts next time)
- bag/          ros2 bag (sqlite, not compressed: compressing at the end took > 15 s on the busy Pi and
                a cut-off bag can't be read; ~16 MB a minute, the disk has room): lidar, depth obstacles, odometry, TF, both costmaps, the maps, Nav2's plan,
                the controller's lookahead point and collision arc, the drive commands (TOPICS)
- summary.json  duration, goals, area mapped, the saved map, Nav2 warnings counted by kind
- run.json      what was started (radius, dry run, together)

The newest KEEP runs are kept, and at most MAX_TOTAL_MB of them. RUNS_DIR is the container's /home/ubuntu/shared (~/docker/tmp on the car).
GET /api/runs lists them (summaries).
"""
import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime

RUNS_DIR = '/home/ubuntu/shared/runs'
KEEP = 20
MAX_TOTAL_MB = 10000
SAMPLE_S = 0.5
POSE_S = 1.0
NAV_LOG = '/tmp/nav.log'
TOPICS = ['/scan_raw', '/scan_nav', '/depth/scan', '/depth/bumps', '/odom', '/tf', '/tf_static',
          '/map', '/team/map', '/local_costmap/costmap', '/global_costmap/costmap',
          '/plan', '/received_global_plan', '/lookahead_point', '/lookahead_collision_arc',
          '/nav/cmd_vel', '/cmd_vel', '/fleet/cloud', '/bump/cloud', '/imu', '/depth/drops', '/odom_lidar', '/odom_raw']
ANSI = re.compile(r'\x1b\[[0-9;]*m')
NAV2_LINE = re.compile(r'\[(WARN|ERROR)\]\s*\[[0-9.]+\]\s*\[([^\]]+)\]:\s*(.*)')


def kind(msg):
    """A Nav2 message without its numbers, for counting ('failed to generate a valid path to (N, N)')."""
    return re.sub(r'-?\d+(\.\d+)?', 'N', msg)[:160]


class RunRecorder:
    def __init__(self, node, explorer, nav, slam, me, team=None):
        self.node, self.explorer, self.nav, self.slam, self.me, self.team = node, explorer, nav, slam, me, team
        self.log = node.get_logger()
        self.run = None            # current run: {'id', 'dir', 't0', 'events', 'bag', ...}
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- start / end -------------------------------------------------------------------
    def _begin(self):
        ex = self.explorer
        rid = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{self.me}'
        d = os.path.join(RUNS_DIR, rid)
        os.makedirs(d, exist_ok=True)
        info = {'id': rid, 'car': self.me, 'kind': 'autonomous mapping', 'started': time.time(),
                'radius_m': ex.radius, 'dry_run': ex.dry, 'together': getattr(ex, 'wait_join', False)}
        with open(os.path.join(d, 'run.json'), 'w') as f:
            json.dump(info, f, indent=1)
        bag = None
        try:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(d, 'bag')] + TOPICS,
                                   start_new_session=True, stdout=open(os.path.join(d, 'bag.log'), 'w'),
                                   stderr=subprocess.STDOUT)
        except OSError as exc:
            self.log.warn(f'run recording: no bag ({exc})')
        self.run = {'id': rid, 'dir': d, 't0': time.monotonic(), 'events': open(os.path.join(d, 'events.jsonl'), 'w'),
                    'bag': bag, 'last': None, 'pose_at': 0.0, 'nav_pos': None, 'counts': {}, 'n_nav2': 0}
        self.log.info(f'run recording started: {d}')
        self._prune()

    def _end(self):
        r, self.run = self.run, None
        if r is None:
            return
        self._tail_nav_log(r)
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
        ex = self.explorer.status()
        summary = {'id': r['id'], 'duration_s': round(time.monotonic() - r['t0']),
                   'end_state': ex.get('state'), 'end_message': ex.get('message'),
                   'mapped_m2': ex.get('mapped_m2'), 'goals': ex.get('goals'), 'saved_as': ex.get('saved_as'),
                   'team_saved_as': ex.get('team_saved_as'), 'nav2_messages': r['n_nav2'],
                   'nav2_by_kind': dict(sorted(r['counts'].items(), key=lambda kv: -kv[1]))}
        with open(os.path.join(r['dir'], 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=1)
        r['events'].close()
        self.log.info(f'run recording finished: {r["dir"]} ({summary["duration_s"]} s, '
                      f'{r["n_nav2"]} Nav2 warnings / errors)')

    def _prune(self):
        try:
            runs = sorted(n for n in os.listdir(RUNS_DIR) if os.path.isdir(os.path.join(RUNS_DIR, n)))
        except OSError:
            return
        for n in runs[:-KEEP]:
            shutil.rmtree(os.path.join(RUNS_DIR, n), ignore_errors=True)
        runs = runs[-KEEP:]
        size = {n: sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(os.path.join(RUNS_DIR, n))
                       for f in fs) for n in runs}
        while len(runs) > 1 and sum(size[n] for n in runs) > MAX_TOTAL_MB * 1e6:
            shutil.rmtree(os.path.join(RUNS_DIR, runs[0]), ignore_errors=True)
            runs.pop(0)

    # ---- while running -----------------------------------------------------------------
    def _write(self, r, ev):
        ev['t'] = round(time.monotonic() - r['t0'], 2)
        r['events'].write(json.dumps(ev) + '\n')

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
        if now - r['pose_at'] >= POSE_S and self.slam.pose is not None:
            r['pose_at'] = now
            self._write(r, {'type': 'pose', 'pose': [round(v, 3) for v in self.slam.pose]})
        self._tail_nav_log(r)
        r['events'].flush()

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
        pose = [round(v, 3) for v in self.slam.pose] if self.slam.pose else None
        for line in data[:end].decode(errors='replace').splitlines():
            m = NAV2_LINE.search(ANSI.sub('', line))
            if not m:
                continue
            level, who, msg = m.groups()
            k = f'{who}: {kind(msg)}'
            r['counts'][k] = r['counts'].get(k, 0) + 1
            r['n_nav2'] += 1
            self._write(r, {'type': 'nav2', 'level': level, 'node': who, 'msg': msg.strip(), 'pose': pose})

    def _loop(self):
        while True:
            time.sleep(SAMPLE_S)
            try:
                active = self.explorer.active()
                if active and self.run is None:
                    self._begin()
                elif not active and self.run is not None:
                    self._end()
                if self.run is not None:
                    self._sample(self.run)
            except Exception as exc:
                self.log.warn(f'run recording: {exc!r}')

    # ---- GET /api/runs -------------------------------------------------------------------
    def listing(self):
        out = []
        try:
            names = sorted((n for n in os.listdir(RUNS_DIR) if os.path.isdir(os.path.join(RUNS_DIR, n))), reverse=True)
        except OSError:
            names = []
        for n in names:
            d = os.path.join(RUNS_DIR, n)
            item = {'id': n, 'recording': self.run is not None and self.run['id'] == n}
            for f in ('run.json', 'summary.json'):
                try:
                    with open(os.path.join(d, f)) as fh:
                        item.update(json.load(fh))
                except (OSError, ValueError):
                    pass
            out.append(item)
        return {'car': self.me, 'dir': RUNS_DIR, 'runs': out}
