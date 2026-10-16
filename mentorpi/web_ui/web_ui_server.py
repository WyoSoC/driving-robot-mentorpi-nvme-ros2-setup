#!/usr/bin/env python3
"""MentorPi web UI server (runs inside the MentorPi container).

- Serves index.html on HTTP_PORT.
- Relays /web/cmd_vel -> /app/cmd_vel with clamping and a watchdog: if the
  browser stops sending while the car is moving (tab closed, Wi-Fi drop),
  the car is stopped after CMD_TIMEOUT seconds.
- Publishes /web/depth_color (bgr8) so web_video_server can stream the
  16-bit depth image as MJPEG. Only runs while someone is watching.

/app/cmd_vel contract (odom_publisher_node, MentorPi_Acker):
  linear.x  = speed in m/s
  angular.z = front wheel steering angle in rad (positive = left)
"""
import functools
import http.server
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

HTTP_PORT = int(os.environ.get('WEB_UI_PORT', 8000))
WEB_ROOT = os.path.dirname(os.path.abspath(__file__))

MAX_SPEED = 0.7               # m/s, joystick_control default
MAX_STEER = math.radians(40)  # same limit as joystick_control
CMD_TIMEOUT = 0.5             # s without a web command while moving -> stop

DEPTH_TOPIC = '/ascamera/camera_publisher/depth0/image_raw'
DEPTH_MAX_MM = 4000           # colour scale: 0 m (red) .. 4 m (blue)
DEPTH_PERIOD = 0.1            # s, colorize at most 10 Hz


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

    def on_cmd(self, msg):
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
    def log_message(self, *args):
        pass

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


def serve_http():
    handler = functools.partial(QuietHandler, directory=WEB_ROOT)
    server = http.server.ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), handler)
    server.serve_forever()


def main():
    rclpy.init()
    node = WebBridge()
    threading.Thread(target=serve_http, daemon=True).start()
    node.get_logger().info(f'Web UI on http://0.0.0.0:{HTTP_PORT}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
