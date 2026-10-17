#!/usr/bin/env python3
"""MentorPi web UI server (runs inside the MentorPi container).

- Serves index.html on HTTP_PORT.
- Relays /web/cmd_vel -> /app/cmd_vel with clamping and a watchdog: if the
  browser stops sending while the car is moving (tab closed, Wi-Fi drop),
  the car is stopped after CMD_TIMEOUT seconds.
- Publishes /web/depth_color (bgr8) so web_video_server can stream the
  16-bit depth image as MJPEG. Only runs while someone is watching.
- Publishes /web/battery_status (std_msgs/String, JSON, 1 Hz): charge %, charging /
  in use / steady, time to empty or full. See battery_estimator.py.
- Charging mode (manual): GET /api/charging -> {"mode": "on" | "switching" | "off"},
  POST /api/charging?on=1|0 writes state/charging_request; the host (system/charging_mode.sh via
  mentorpi-charging.path) stops / restarts the robot software. This server keeps running.
- Navigation (nav_manager.py, Nav2 in nav/): GET /api/nav (status, pose, plan), GET /api/nav/map.png?name=,
  POST /api/nav/start?map=NAME | /api/nav/stop | /api/nav/pose?x=&y=&yaw= | /api/nav/go?x=&y=&yaw= |
  /api/nav/cancel. Joystick input from the page cancels a navigation goal.
- SLAM (slam_manager.py): GET /api/slam (status, pose, path), GET /api/slam/map.png,
  POST /api/slam/start | /api/slam/stop | /api/slam/save?name=NAME
- Exploration (explore_manager.py): POST /api/explore/start?radius=M (0 = no limit) | /api/explore/stop;
  its status is in GET /api/slam ("explore"). Maps by itself: SLAM + Nav2 on the live map + frontiers.
- Other cars (peer_manager.py, peers from fleet.env): GET /api/self = this car's status (what the other
  cars poll), GET /api/fleet = this car + the latest status of every other car, GET /api/maps = maps here
  and on the other cars, GET /api/maps/file?name=&ext=png|yaml (a map for another car to copy).
  A map of another car: GET /api/nav/map.png?name=&car=, POST /api/nav/start?map=&car= copies it here first.

/app/cmd_vel contract (odom_publisher_node, MentorPi_Acker):
  linear.x  = speed in m/s
  angular.z = front wheel steering angle in rad (positive = left)
"""
import functools
import http.server
import json
import math
import urllib.parse
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt16

import map_store
from battery_estimator import BatteryEstimator
from explore_manager import Explorer
from nav_manager import NavManager
from peer_manager import PeerManager
from slam_manager import SlamManager

HTTP_PORT = int(os.environ.get('WEB_UI_PORT', 8000))
CAR_NAME = os.environ.get('CAR_NAME') or os.uname().nodename.split('.')[0]
PEERS = os.environ.get('PEERS', '')
WEB_ROOT = os.path.dirname(os.path.abspath(__file__))

MAX_SPEED = 0.7               # m/s, joystick_control default
MAX_STEER = math.radians(40)  # same limit as joystick_control
CMD_TIMEOUT = 0.5             # s without a web command while moving -> stop

DEPTH_TOPIC = '/ascamera/camera_publisher/depth0/image_raw'
DEPTH_MAX_MM = 4000           # colour scale: 0 m (red) .. 4 m (blue)
DEPTH_PERIOD = 0.1            # s, colorize at most 10 Hz

BATTERY_TOPIC = '/ros_robot_controller/battery'   # UInt16, mV, ~1 Hz
DRIVING_SPEED = 0.02          # m/s from /odom; above -> motors loaded, voltage sags
STATE_DIR = '/home/ubuntu/mentorpi_state'   # host mentorpi/state/ (run_container.sh)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class WebBridge(Node):
    def __init__(self):
        super().__init__('web_ui_bridge')

        self.cmd_pub = self.create_publisher(Twist, '/app/cmd_vel', 1)
        self.create_subscription(Twist, '/web/cmd_vel', self.on_cmd, 1)
        self.last_cmd_time = 0.0
        self.moving = False
        self.create_timer(0.1, self.watchdog)

        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, '/web/depth_color', 1)
        self.last_depth_time = 0.0
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)

        self.battery = BatteryEstimator()
        self.battery_v = None
        self.battery_st = None
        self.speed = 0.0
        self.battery_pub = self.create_publisher(String, '/web/battery_status', 1)
        self.create_subscription(UInt16, BATTERY_TOPIC, self.on_battery, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(1.0, self.publish_battery)
        self.charging = 'off'
        self.create_timer(1.0, self.read_charging_mode)

        self.slam = SlamManager(self)
        self.nav = NavManager(self, self.slam.tf, self.slam)
        self.explorer = Explorer(self, self.slam, self.nav)
        self.peers = PeerManager(CAR_NAME, PEERS, self.get_logger())

    def self_status(self):
        """What the other cars see of this one (GET /api/self)."""
        nav = self.nav.status()
        slam_on = self.slam.running()
        mode = ('charging' if self.charging != 'off' else 'exploring' if self.explorer.active()
                else 'navigating' if nav['running']
                else 'mapping' if slam_on else 'idle')
        return {
            'car': CAR_NAME,
            'mode': mode,
            'charging': self.charging,
            'battery': self.battery_st,
            'speed': round(self.speed, 3),
            'nav': {k: nav[k] for k in ('running', 'map_name', 'pose', 'pose_set', 'goal', 'goal_state', 'plan',
                                        'feedback')}
                   | {'map_hash': nav['map']['hash'] if nav['map'] else None},
            'slam': slam_on,
            'maps': [{k: m[k] for k in ('name', 'hash', 'resolution', 'origin')} for m in map_store.listing()],
        }

    def read_charging_mode(self):
        try:
            with open(os.path.join(STATE_DIR, 'charging')) as f:
                self.charging = f.read().strip() or 'switching'
        except FileNotFoundError:
            self.charging = 'off'
        except OSError:
            pass
        self.battery.set_charging_mode(self.charging == 'on')

    def request_charging(self, on):
        if on and self.explorer.active():
            self.explorer.stop('charging mode')
        if on and self.nav.running():
            self.nav.stop()
        if self.slam.running():
            self.slam.stop()
        with open(os.path.join(STATE_DIR, 'charging_request'), 'w') as f:
            f.write('on\n' if on else 'off\n')
        self.get_logger().info(f'charging mode {"on" if on else "off"} requested')

    def on_cmd(self, msg):
        if (msg.linear.x or msg.angular.z) and self.nav.goal_active():
            self.nav.cancel('goal canceled: manual driving')   # the person at the joystick wins
            self.get_logger().info('navigation goal canceled by manual driving')
        out = Twist()
        out.linear.x = clamp(msg.linear.x, MAX_SPEED)
        out.angular.z = clamp(msg.angular.z, MAX_STEER)
        self.cmd_pub.publish(out)
        self.last_cmd_time = time.monotonic()
        self.moving = out.linear.x != 0.0

    def watchdog(self):
        if self.moving and time.monotonic() - self.last_cmd_time > CMD_TIMEOUT:
            self.stop()
            self.get_logger().warn('Web control timed out, car stopped')

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.moving = False

    def on_battery(self, msg):
        self.battery_v = msg.data / 1000.0
        self.battery.add(time.monotonic(), self.battery_v)

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        if abs(msg.twist.twist.linear.x) > DRIVING_SPEED:
            self.battery.driving(time.monotonic())

    def publish_battery(self):
        if self.battery_v is not None:
            self.battery_st = self.battery.status(self.battery_v)
            self.battery_pub.publish(String(data=json.dumps(self.battery_st)))

    def on_depth(self, msg):
        if self.depth_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self.last_depth_time < DEPTH_PERIOD:
            return
        self.last_depth_time = now

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        valid = depth > 0
        scaled = np.clip(depth.astype(np.float32) * (255.0 / DEPTH_MAX_MM), 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_JET)
        color[~valid] = 0
        out = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        out.header = msg.header
        self.depth_pub.publish(out)


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    node = None   # set in main()

    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype='application/json'):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        slam = self.node.slam
        if url.path == '/api/charging':
            return self._send(200, {'mode': self.node.charging})
        if url.path == '/api/slam':
            q = urllib.parse.parse_qs(url.query)
            return self._send(200, slam.status(int(q.get('since', ['0'])[0] or 0)) | {'explore': self.node.explorer.status()})
        if url.path == '/api/slam/map.png':
            png = slam.png
            return self._send(200, png, 'image/png') if png else self._send(404, {'error': 'no map yet'})
        if url.path == '/api/nav':
            return self._send(200, self.node.nav.status())
        if url.path == '/api/self':
            return self._send(200, self.node.self_status())
        if url.path == '/api/fleet':
            return self._send(200, {'car': CAR_NAME, 'peers': self.node.peers.summary()})
        if url.path == '/api/maps':
            return self._send(200, self.maps_everywhere())
        if url.path == '/api/maps/file':
            q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
            name, ext = q.get('name', ''), q.get('ext', '')
            if name not in map_store.names() or ext not in ('png', 'yaml'):
                return self._send(404, {'error': 'no such map'})
            try:
                if ext == 'png':
                    return self._send(200, map_store.png_bytes(name), 'image/png')
                return self._send(200, map_store.yaml_text(name).encode(), 'text/plain')
            except (OSError, KeyError, ValueError):
                return self._send(404, {'error': 'map files missing'})
        if url.path == '/api/nav/map.png':
            q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
            name, car = q.get('name', ''), q.get('car', '')
            if car and car != CAR_NAME:
                try:
                    return self._send(200, self.node.peers.map_png(car, name), 'image/png')
                except Exception as exc:
                    return self._send(404, {'error': f'cannot get the map from {car}: {exc}'})
            if name not in self.node.nav.maps():
                return self._send(404, {'error': 'no such map'})
            try:
                img = self.node.nav.map_meta(name)['image']
                with open(os.path.join('/home/ubuntu/maps', os.path.basename(img)), 'rb') as f:
                    return self._send(200, f.read(), 'image/png')
            except (OSError, KeyError):
                return self._send(404, {'error': 'map image missing'})
        return super().do_GET()

    def maps_everywhere(self):
        """Maps here, then maps on the other cars that are not here (same content = same map)."""
        here = [dict(m, car=CAR_NAME, local=True) for m in map_store.listing()]
        seen = {m['hash'] for m in here}
        out = list(here)
        for p in self.node.peers.summary():
            for m in ((p['status'] or {}).get('maps') or []):
                if m.get('hash') not in seen:
                    seen.add(m.get('hash'))
                    out.append(dict(m, car=p['car'], local=False, online=p['online']))
        return {'car': CAR_NAME, 'maps': out}

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        slam = self.node.slam
        if url.path == '/api/charging':
            on = urllib.parse.parse_qs(url.query).get('on', [''])[0]
            if on not in ('0', '1'):
                return self._send(400, {'ok': False, 'message': 'on=1 or on=0'})
            try:
                self.node.request_charging(on == '1')
            except OSError as exc:
                return self._send(500, {'ok': False, 'message': f'cannot write request: {exc}'})
            return self._send(200, {'ok': True})
        if url.path.startswith('/api/nav/'):
            q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
            nav, action = self.node.nav, url.path[len('/api/nav/'):]
            try:
                if action == 'start':
                    name, car = q.get('map', ''), q.get('car', '')
                    if car and car != CAR_NAME:
                        if nav.running():
                            return self._send(400, {'ok': False, 'message': f'already navigating on "{nav.map_name}"'})
                        try:
                            name = self.node.peers.fetch_map(car, name)
                        except Exception as exc:
                            return self._send(400, {'ok': False, 'message': f'cannot copy the map from {car}: {exc}'})
                    ok, msg = nav.start(name)
                elif action == 'stop':
                    ok, msg = nav.stop()
                elif action == 'cancel':
                    ok, msg = nav.cancel()
                elif action in ('pose', 'go'):
                    x, y, yaw = float(q['x']), float(q['y']), float(q.get('yaw', 0))
                    if not all(map(math.isfinite, (x, y, yaw))):
                        raise ValueError
                    ok, msg = (nav.set_pose if action == 'pose' else nav.go)(x, y, yaw)
                else:
                    return self._send(404, {'error': 'not found'})
            except (KeyError, ValueError):
                return self._send(400, {'ok': False, 'message': 'x, y (m) and yaw (rad) needed'})
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        if url.path == '/api/explore/start':
            try:
                radius = float(urllib.parse.parse_qs(url.query).get('radius', ['0'])[0] or 0)
                if not math.isfinite(radius) or radius < 0:
                    raise ValueError
            except ValueError:
                return self._send(400, {'ok': False, 'message': 'radius in m (0 = no limit)'})
            dry = urllib.parse.parse_qs(url.query).get('dry', ['0'])[0] == '1'
            ok, msg = self.node.explorer.start(radius, dry=dry)
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        if url.path == '/api/explore/stop':
            ok, msg = self.node.explorer.stop()
            return self._send(200, {'ok': ok, 'message': msg})
        if url.path == '/api/slam/stop' and (self.node.explorer.active() or self.node.nav.slam_mode):
            self.node.explorer.stop('mapping stopped')
        if url.path == '/api/slam/start' and self.node.nav.running():
            return self._send(400, {'ok': False, 'message': 'stop navigation first'})
        if url.path == '/api/slam/start':
            return self._send(200, {'ok': True, 'message': slam.start()})
        if url.path == '/api/slam/stop':
            return self._send(200, {'ok': True, 'message': slam.stop()})
        if url.path == '/api/slam/save':
            name = urllib.parse.parse_qs(url.query).get('name', [''])[0]
            ok, msg = slam.save(name)
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        return self._send(404, {'error': 'not found'})

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


def serve_http():
    handler = functools.partial(QuietHandler, directory=WEB_ROOT)
    for attempt in range(20):  # a previous instance may still be releasing the port
        try:
            server = http.server.ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), handler)
            break
        except OSError as exc:
            if attempt == 19:
                # Without HTTP the page is down while the service looks healthy: exit, let systemd restart.
                print(f'cannot open port {HTTP_PORT}: {exc}', flush=True)
                os._exit(1)
            time.sleep(1)
    server.serve_forever()


def main():
    rclpy.init()
    node = WebBridge()
    QuietHandler.node = node
    threading.Thread(target=serve_http, daemon=True).start()
    node.get_logger().info(f'Web UI on http://0.0.0.0:{HTTP_PORT}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()  # fails harmlessly if ROS already shut down (e.g. SIGTERM)
        except Exception:
            pass
        node.nav.shutdown()
        node.peers.shutdown()
        node.slam.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
