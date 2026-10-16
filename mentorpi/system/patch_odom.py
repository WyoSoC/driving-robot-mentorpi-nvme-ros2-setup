"""Patch the factory odom_publisher inside a freshly started MentorPi container.

Bug: odom_publisher applies linear_correction_factor only to the x/y position it publishes on
/odom_raw, and never uses angular_correction_factor. The EKF (ekf.yaml) fuses only the
*velocities* from /odom_raw (vx, vy, vyaw), so the calibration in calibrate_params.yaml had no
effect on /odom, the web UI trail or SLAM.

Fix: scale the commanded speed and turn rate by the factors inside the odometry loop (the
velocities the EKF uses, and the integrated position). Motor commands are not touched.

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
for old, new in replacements:
    if src.count(old) != 1:
        sys.exit(f"patch_odom: factory code changed, cannot apply patch:\n{old}")
    src = src.replace(old, new)
open(FILE, "w").write(src)
print("odom_publisher patched: correction factors apply to /odom_raw velocities")
