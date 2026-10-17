"""Patch the factory odom_publisher inside a freshly started MentorPi container.

Bug: odom_publisher applies linear_correction_factor only to the x/y position it publishes on
/odom_raw, and never uses angular_correction_factor. The EKF (ekf.yaml) fuses only the
*velocities* from /odom_raw (vx, vy, vyaw), so the calibration in calibrate_params.yaml had no
effect on /odom, the web UI trail or SLAM.

Fix: scale the commanded speed and turn rate by the factors inside the odometry loop (the
velocities the EKF uses, and the integrated position).

Second fix: real speed does not follow the command (car 1, lidar-measured: 0.2 m/s commanded ->
0.46 m/s real, 0.087 -> 0.08; the motors have no speed feedback, so the response is non-linear).
New parameter motor_speed_map = [real0, sent0, real1, sent1, ...] (m/s, increasing), measured by
calibrate_odom.sh speedmap: the speed to send for a wanted real speed is interpolated from it, so a
commanded speed is the real speed and speed limits mean something. Fallback without a map:
motor_speed_factor (real / commanded, default 1.0). Speed and turn rate are scaled together, so the
steering angle does not change.

Third fix: the car steered ~25% less than commanded (car 2, IMU-measured circles). New parameter
steer_gain multiplies the turn rate used for the steering servo only (rear-wheel speeds and
odometry keep the commanded turn rate), so a commanded turn rate is the real one.

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
        self.declare_parameter('motor_speed_factor', 1.00)  # mentorpi patch: real / commanded speed
        self.declare_parameter('motor_speed_map', [0.0])     # mentorpi patch: [real, sent, ...]
        self.declare_parameter('steer_gain', 1.00)           # mentorpi patch: servo turn rate x gain"""),
    ("""        self.angular_factor = self.get_parameter('angular_correction_factor').value""",
     """        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.motor_factor = self.get_parameter('motor_speed_factor').value or 1.0
        m = list(self.get_parameter('motor_speed_map').value or [])
        self.motor_map = [(m[i], m[i + 1]) for i in range(0, len(m) - 1, 2)] if len(m) >= 4 else []
        self.steer_gain = self.get_parameter('steer_gain').value or 1.0
        self.get_logger().info(f'motor_speed_factor {self.motor_factor}, motor_speed_map {self.motor_map}, '
                               f'steer_gain {self.steer_gain} (mentorpi patch)')"""),
    ("""    def cmd_vel_callback(self, msg):""",
     """    def motor_scale(self, v):
        # mentorpi patch: sent / wanted speed ratio for a wanted real speed v
        v = abs(v)
        if v < 1e-6:
            return 1.0
        pts = self.motor_map
        if not pts:
            return 1.0 / self.motor_factor
        if v <= pts[0][0]:                       # below the slowest measured point: same ratio
            return pts[0][1] / pts[0][0]
        for (r0, s0), (r1, s1) in zip(pts, pts[1:]):
            if v <= r1:
                return (s0 + (s1 - s0) * (v - r0) / (r1 - r0)) / v
        return pts[-1][1] / pts[-1][0]           # above the fastest: same ratio

    def cmd_vel_callback(self, msg):"""),
    ("""\n            speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)""",
     """\n            k = self.motor_scale(self.linear_x)
            speeds = self.ackermann.set_velocity(self.linear_x * k, self.angular_z * k)
            if self.steer_gain != 1.0 and speeds[0] is not None:   # stronger servo command only
                servo = self.ackermann.set_velocity(self.linear_x * k, self.angular_z * k * self.steer_gain)[0]
                speeds = (servo, speeds[1])"""),
]
for old, new in replacements:
    if src.count(old) != 1:
        sys.exit(f"patch_odom: factory code changed, cannot apply patch:\n{old}")
    src = src.replace(old, new)
open(FILE, "w").write(src)
print("odom_publisher patched: correction factors on /odom_raw velocities, motor speed map + steer gain on commands")
