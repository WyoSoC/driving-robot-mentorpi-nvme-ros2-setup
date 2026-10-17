"""Drive each rear motor on its own, in speed steps, WITH THE WHEELS OFF THE GROUND (runs inside the
MentorPi container; normally via ../tools/motor_test.sh).

Finds a motor whose speed control runs away (car 1: fine below ~0.6 wheel rev/s, much too fast above):
watch which wheel suddenly races. Sends wheel speeds straight to the board (ros_robot_controller/set_motor,
rev/s), bypassing the speed map. Ackermann layout (controller/ackermann.py): motor 2 = left rear,
motor 4 = right rear (mounted mirrored, so it gets the negative value for forward).

Usage: python3 motor_test.py [left|right|both] [STEP_SECONDS]
"""
import signal
import sys
import time

import rclpy
from ros_robot_controller_msgs.msg import MotorsState, MotorState

WHICH = sys.argv[1] if len(sys.argv) > 1 else 'both'
STEP = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
RPS = [0.2, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0]   # wheel rev/s; 0.6 rev/s ~ 0.13 m/s with the 67 mm wheels
MOTORS = {'left': (2, 1.0), 'right': (4, -1.0)}

rclpy.init()
node = rclpy.create_node('motor_test')
pub = node.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)


def send(speeds):
    msg = MotorsState()
    for mid in (1, 2, 3, 4):
        m = MotorState()
        m.id, m.rps = mid, float(speeds.get(mid, 0.0))
        msg.data.append(m)
    pub.publish(msg)


def stop(*_):
    for _ in range(5):
        send({})
        time.sleep(0.05)


def bail(*_):
    stop()
    sys.exit(130)


for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, bail)

try:
    time.sleep(1.0)   # let the publisher connect
    for name in (['left', 'right'] if WHICH == 'both' else [WHICH]):
        mid, sign = MOTORS[name]
        print(f'== {name} rear wheel (motor {mid})', flush=True)
        for rps in RPS:
            print(f'  {rps:.1f} rev/s  (~{rps * 3.1416 * 0.067:.2f} m/s)', flush=True)
            end = time.time() + STEP
            while time.time() < end:
                send({mid: sign * rps})
                time.sleep(0.1)
        stop()
        time.sleep(1.5)
finally:
    stop()
    rclpy.try_shutdown()
print('done, motors stopped')
