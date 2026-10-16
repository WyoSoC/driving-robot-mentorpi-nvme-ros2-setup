"""Steering test (runs inside the MentorPi container; normally via ../steer_test.sh).

Sends steering angles at zero speed through /app/cmd_vel, the same path as the web UI, and logs
the servo pulse the controller sends. Only the steering servo moves; the drive motors stay off.

Usage: python3 steer_test.py DEG:SECONDS [DEG:SECONDS ...]   (+ = left), e.g. 40:2 -40:2 0:1
"""
import math, sys, time, rclpy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import SetPWMServoState
rclpy.init(); n = rclpy.create_node("steer_test")
pub = n.create_publisher(Twist, "/app/cmd_vel", 1)
seen = []
n.create_subscription(SetPWMServoState, "/ros_robot_controller/pwm_servo/set_state",
                      lambda m: seen.append(m.state[0].position[0]) if m.state and m.state[0].position else None, 10)
end = time.time() + 2
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)  # discovery
for step in sys.argv[1:]:
    deg, hold = (float(x) for x in step.split(":"))
    seen.clear(); t = Twist(); t.angular.z = math.radians(deg)
    end = time.time() + hold
    while time.time() < end:
        pub.publish(t); rclpy.spin_once(n, timeout_sec=0.1)
    print(f"steer {deg:+5.1f} deg for {hold:.0f} s -> servo pulse {sorted(set(seen))}", flush=True)
