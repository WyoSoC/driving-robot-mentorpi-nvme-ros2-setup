"""SLAM for the MentorPi web UI (runs inside web_ui_server.py, in the MentorPi container).

- start/stop: runs only Hiwonder's slam_toolbox node (slam/launch/include/slam_base.launch.py,
  config slam/config/slam.yaml via nav/slam.launch.py (map every 2 s, not 0.2 s): lidar /scan_raw, frames map -> odom -> base_footprint) next to the
  running bringup. (Hiwonder's own slam.launch.py would start a second copy of the drivers.)
- /map (OccupancyGrid) is rendered to a PNG on the car; the browser fetches the image instead of
  the raw grid (hundreds of KB per update, too heavy over a slow link).
- the car's pose in the map frame comes from TF map -> base_footprint; its path is kept here.
- save: map image + yaml (nav2 map_server format) and slam_toolbox's pose graph
  (.posegraph/.data, to continue mapping later) into MAPS_DIR, a per-car host folder
  (mentorpi/maps/, mounted by run_container.sh). After Stop, the map still on screen is saved as
  PNG + yaml (no pose graph).
"""
import math
import os
import re
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

MAPS_DIR = '/home/ubuntu/maps'
LAUNCH = ['ros2', 'launch', '/home/ubuntu/nav/slam.launch.py']   # Hiwonder's slam_toolbox setup, map every 2 s
RENDER_PERIOD = 1.0        # s, at most one PNG per second
PATH_STEP = 0.05           # m between stored path points
PATH_MAX = 5000
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


class SlamManager:
    def __init__(self, node):
        self.node = node
        self.proc = None
        self.started_at = None
        self.lock = threading.Lock()
        self.png = None              # latest rendered map
        self.meta = None             # width, height, resolution, origin, version
        self.version = 0
        self.last_render = 0.0
        self.pending_map = None
        self.team_map = None     # shown instead of the own map while mapping together
        self.view = None         # [x, y, yaw]: this car's frame -> the frame of the map shown (shared map)
        self.grid = None
        self.pose = None             # x, y, yaw in map frame
        self.path = []
        self.message = ''
        os.makedirs(MAPS_DIR, exist_ok=True)
        self._kill_strays()

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf, node)
        self.save_cli = node.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_cli = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        node.create_timer(0.2, self.update_pose)
        node.create_timer(0.5, self.render_pending)

    # ---- process -------------------------------------------------------------
    def _kill_strays(self):
        """A previous web server may have left slam_toolbox running."""
        subprocess.run(['pkill', '-INT', '-f', 'slam_base.launch.py'], check=False)
        subprocess.run(['pkill', '-INT', '-f', 'sync_slam_toolbox_node'], check=False)

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        with self.lock:
            if self.running():
                return 'already running'
            self.path, self.pose, self.png, self.meta, self.grid = [], None, None, None, None
            self.team_map = None
            self.proc = subprocess.Popen(LAUNCH, cwd=MAPS_DIR, start_new_session=True,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.started_at = time.time()
            self.message = 'mapping started'
            self.node.get_logger().info('SLAM started')
            return self.message

    def stop(self):
        with self.lock:
            if not self.running():
                return 'not running'
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc = None
            self.message = 'mapping stopped (map kept on screen; save it before starting again)'
            self.node.get_logger().info('SLAM stopped')
            return self.message

    def shutdown(self):
        if self.running():
            self.stop()

    # ---- map + pose ----------------------------------------------------------
    def on_map(self, msg):
        if self.team_map is None:
            self.pending_map = msg
        self.grid = msg          # latest raw map (exploration)

    def show_team_map(self, msg, view=None):
        """Show the team map (team.py: this car's map + the other cars', in this car's frame) instead of
        this car's own; None = own map again."""
        self.team_map = msg
        self.view = list(view) if msg is not None and view is not None else None
        self.pending_map = msg if msg is not None else self.grid

    def render_pending(self):
        msg = self.pending_map
        if msg is None or time.monotonic() - self.last_render < RENDER_PERIOD:
            return
        self.pending_map = None
        self.last_render = time.monotonic()
        self._render(msg)

    def _render(self, msg):
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)   # -1 unknown, 0 free .. 100 occupied
        img = np.full((h, w), 205, np.uint8)                      # unknown: grey
        known = grid >= 0
        img[known] = (255 - grid[known].astype(np.int32) * 255 // 100).astype(np.uint8)  # free white .. occupied black
        img = np.flipud(img)                                      # row 0 = top of the image = max y
        ok, png = cv2.imencode('.png', img)
        if not ok:
            return
        o = msg.info.origin.position
        self.version += 1
        self.png = png.tobytes()
        self.meta = {'width': w, 'height': h, 'resolution': msg.info.resolution,
                     'origin': [o.x, o.y], 'version': self.version}

    def update_pose(self):
        if not self.running():
            return
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = [tr.x, tr.y, yaw]
        if not self.path or math.hypot(tr.x - self.path[-1][0], tr.y - self.path[-1][1]) > PATH_STEP:
            self.path.append([round(tr.x, 3), round(tr.y, 3)])
            del self.path[:-PATH_MAX]

    # ---- saving --------------------------------------------------------------
    def saved_maps(self):
        try:
            return sorted(f[:-5] for f in os.listdir(MAPS_DIR) if f.endswith('.yaml'))
        except OSError:
            return []

    def _call(self, client, request, timeout=20.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{client.srv_name} not available')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()

    def save_rendered(self, path):
        """Save the rendered map (PNG + nav2 map yaml; no pose graph)."""
        png, meta = self.png, self.meta
        if not png or not meta:
            return False, 'no map to save: start mapping first'
        with open(path + '.png', 'wb') as f:
            f.write(png)
        with open(path + '.yaml', 'w') as f:   # 205 (unknown) -> (255-205)/255 = 0.196, not < free_thresh
            f.write(f"image: {os.path.basename(path)}.png\nmode: trinary\nresolution: {meta['resolution']:.3f}\n"
                    f"origin: [{meta['origin'][0]:.4f}, {meta['origin'][1]:.4f}, 0.0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")
        self.message = f'saved map "{os.path.basename(path)}" (image + yaml; mapping was stopped, no pose graph)'
        return True, self.message

    def save(self, name):
        if not NAME_RE.match(name or ''):
            return False, 'name: 1-40 letters, digits, - or _'
        path = os.path.join(MAPS_DIR, name)
        if not self.running():
            return self.save_rendered(path)
        # While mapping: image + yaml from the latest map, written here (slam_toolbox's save_map runs
        # nav2's map_saver_cli, which failed with 255 here), plus slam_toolbox's pose graph.
        if self.grid is not None:
            self._render(self.grid)
        ok, msg = self.save_rendered(path)
        if not ok:
            return ok, msg
        try:
            req2 = SerializePoseGraph.Request()
            req2.filename = path
            graph_ok = self._call(self.serialize_cli, req2).result == 0
        except Exception:
            graph_ok = False
        self.message = f'saved map "{name}"' + ('' if graph_ok else ' (image + yaml; pose graph not saved)')
        return True, self.message

    # ---- status for the web page ---------------------------------------------
    def status(self, since_path=0):
        return {
            'running': self.running(),
            'uptime_s': round(time.time() - self.started_at) if self.running() else None,
            'map': self.meta,
            'pose': self.pose,
            'view': self.view,
            'path_len': len(self.path),
            'path': self.path[since_path:] if since_path <= len(self.path) else self.path,
            'maps': self.saved_maps(),
            'message': self.message,
        }
