"""Lidar scan for obstacle avoidance (runs inside web_ui_server.py): /scan_raw -> /scan_nav.

Readings that land inside the car's own rectangle (car_geometry.py: something touching the car, a cable,
a curtain, or the car itself) are set to NaN, so Nav2 neither marks nor clears with them. Everything
outside the rectangle is kept, however close. Nav2's obstacle layers and the collision monitor use
/scan_nav; localization (AMCL) and SLAM keep /scan_raw.

The lidar frame has the same orientation as base_footprint (TF), so a reading at angle a and range r is
at x = LIDAR_X + r cos a, y = r sin a.
"""
import math

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import car_geometry as g

IN_TOPIC, OUT_TOPIC = '/scan_raw', '/scan_nav'


def inside_car(ranges, angle_min, angle_inc, lidar_x=g.LIDAR_X):
    """bool per reading: does it land inside the car's rectangle (base_footprint)?"""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    with np.errstate(invalid='ignore'):
        x = lidar_x + r * np.cos(a)
        y = r * np.sin(a)
        return np.isfinite(r) & (x >= g.BACK) & (x <= g.FRONT) & (np.abs(y) <= g.HALF_WIDTH)


class ScanFilter:
    def __init__(self, node):
        self.pub = node.create_publisher(LaserScan, OUT_TOPIC, qos_profile_sensor_data)
        node.create_subscription(LaserScan, IN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.dropped = 0           # readings inside the car in the last scan (for status / debugging)

    def on_scan(self, msg):
        bad = inside_car(msg.ranges, msg.angle_min, msg.angle_increment)
        self.dropped = int(bad.sum())
        if self.dropped:
            r = np.asarray(msg.ranges, dtype=np.float32)
            r[bad] = math.nan
            msg.ranges = r.tolist()
        self.pub.publish(msg)
