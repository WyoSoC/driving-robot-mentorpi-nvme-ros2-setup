"""Patch the factory odom_publisher inside a freshly started MentorPi container.

Bug: odom_publisher applies linear_correction_factor only to the x/y position it publishes on
/odom_raw, and never uses angular_correction_factor. The EKF (ekf.yaml) fuses only the
*velocities* from /odom_raw (vx, vy, vyaw), so the calibration in calibrate_params.yaml had no
effect on /odom, the web UI trail or SLAM.

Fix: scale the commanded speed and turn rate by the factors inside the odometry loop (the
velocities the EKF uses, and the integrated position).

Second fix: the motors ran ~2.3x faster than commanded (car 1, lidar-measured; the board's motor
type setting does not match the fitted motors). New parameter motor_speed_factor = real speed /
commanded speed, measured by calibrate_odom.sh linear; motor commands are divided by it so a
commanded speed is the real speed (speed limits mean something). Speed and turn rate are scaled
together, so the steering angle does not change.

Run by run_container.sh: python3 patch_odom.py [FILE]
"""
import sys

FILE = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ubuntu/ros2_ws/src/driver/controller/controller/odom_publisher_node.py"
MARK = "# mentorpi patch: correction factors on velocities"

src = open(FILE).read()
if MARK in src:
    print("odom_publisher already patched")
    sys.exit(0)

replacements = [
    ("""            delta_x = self.linear_x * self.dt * math.cos(self.pose_yaw)
            delta_y = self.linear_x * self.dt * math.sin(self.pose_yaw)
            delta_yaw = self.angular_z * self.dt""",
     f"""            {MARK}
            v = self.linear_factor * self.linear_x
            w = self.angular_factor * self.angular_z
            delta_x = v * self.dt * math.cos(self.pose_yaw)
            delta_y = v * self.dt * math.sin(self.pose_yaw)
            delta_yaw = w * self.dt"""),
    ("""            self.odom.pose.pose.position.x = self.linear_factor * self.x
            self.odom.pose.pose.position.y = self.linear_factor * self.y""",
     """            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y"""),
    ("""            self.odom.twist.twist.linear.x = self.linear_x""",
     """            self.odom.twist.twist.linear.x = v"""),
    ("""            self.odom.twist.twist.angular.z = self.angular_z""",
     """            self.odom.twist.twist.angular.z = w"""),
]
replacements += [
    ("""        self.declare_parameter('angular_correction_factor', 1.00)""",
     """        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('motor_speed_factor', 1.00)  # mentorpi patch: real / commanded speed"""),
    ("""        self.angular_factor = self.get_parameter('angular_correction_factor').value""",
     """        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.motor_factor = self.get_parameter('motor_speed_factor').value or 1.0
        self.get_logger().info(f'motor_speed_factor {self.motor_factor} (mentorpi patch)')"""),
    ("""\n            speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)""",
     """\n            speeds = self.ackermann.set_velocity(self.linear_x / self.motor_factor, self.angular_z / self.motor_factor)"""),
]
for old, new in replacements:
    if src.count(old) != 1:
        sys.exit(f"patch_odom: factory code changed, cannot apply patch:\n{old}")
    src = src.replace(old, new)
open(FILE, "w").write(src)
print("odom_publisher patched: correction factors on /odom_raw velocities, motor_speed_factor on motor commands")
