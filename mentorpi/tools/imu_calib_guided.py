"""Guided accelerometer calibration for the MentorPi (runs inside the MentorPi container).

Wraps imu_calib's do_calib: for each of the 6 poses it shows the live accelerometer reading,
waits until the car is in the right pose and still, then tells do_calib to record. If the car
moves while recording, the run is aborted. Results go to OUTPUT_FILE (a temporary file);
calibrate_imu.sh installs it only after a successful run.

Usage (normally via ../calibrate_imu.sh): python3 -u imu_calib_guided.py OUTPUT_FILE
"""
import collections
import os
import signal
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Imu

OUTPUT = sys.argv[1]
G = 9.80665
IN_POSE = 8.0       # target axis must read at least this (m/s^2) with the right sign
OFF_AXIS = 2.5      # the other two axes must stay below this
STILL_STD = 0.25    # std dev over the last second while "still"
HOLD_S = 1.5        # pose must be held this long before recording starts
# Test mode: accept any pose, to exercise the full flow with the car standing still. The result
# is then nonsense and calibrate_imu.sh's sanity check is expected to reject it.
TEST = os.environ.get("IMU_CALIB_TEST") == "1"

# do_calib's order. Hints assume ROS convention (X forward, Y left, Z up); the live check is
# what counts, so if a hint is wrong just turn the car until the reading says OK.
POSES = [
    ("X+", 0, +1, "front of the car pointing UP (car standing on its rear end)"),
    ("X-", 0, -1, "front of the car pointing DOWN (car standing on its front end)"),
    ("Y+", 1, +1, "LEFT side up (car lying on its right side)"),
    ("Y-", 1, -1, "RIGHT side up (car lying on its left side)"),
    ("Z+", 2, +1, "normal, on its wheels"),
    ("Z-", 2, -1, "upside down (support the body so the lidar/camera carry no weight)"),
]

samples = collections.deque(maxlen=50)  # ~1 s at 48 Hz
lock = threading.Lock()


def on_imu(msg):
    a = msg.linear_acceleration
    with lock:
        samples.append((a.x, a.y, a.z))


def snapshot():
    with lock:
        return list(samples)


def pose_state(axis, sign):
    s = snapshot()
    if len(s) < 25:
        return False, None, "waiting for IMU data..."
    mean = [statistics.fmean(v[i] for v in s) for i in range(3)]
    std = max(statistics.pstdev(v[i] for v in s) for i in range(3))
    others = [abs(mean[i]) for i in range(3) if i != axis]
    ok_pose = TEST or (sign * mean[axis] >= IN_POSE and max(others) <= OFF_AXIS)
    still = std <= STILL_STD
    up = max(range(3), key=lambda i: abs(mean[i]))
    now_up = "XYZ"[up] + ("+" if mean[up] > 0 else "-")
    line = (f"x={mean[0]:+6.2f} y={mean[1]:+6.2f} z={mean[2]:+6.2f}  "
            f"up now: {now_up}  {'pose OK' if ok_pose else 'turn the car'}"
            f"{', still' if still else ', moving'}")
    return ok_pose and still, mean, line


def spin(node):
    try:
        rclpy.spin(node)
    except Exception:  # context shut down on exit
        pass


def main():
    # Turn SIGTERM/SIGHUP into SystemExit so the finally block below stops do_calib.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(1))
    rclpy.init()
    node = rclpy.create_node("imu_calib_guide")
    node.create_subscription(Imu, "/ros_robot_controller/imu_raw", on_imu, 10)
    threading.Thread(target=spin, args=(node,), daemon=True).start()

    try:
        run(node)
    finally:
        if CALIB[0] is not None and CALIB[0].poll() is None:
            CALIB[0].kill()
        rclpy.try_shutdown()


CALIB = [None]


def run(node):
    # Start do_calib directly (not via `ros2 run`, whose child can outlive a kill).
    exe = os.path.join(get_package_prefix("imu_calib"), "lib", "imu_calib", "do_calib")
    calib = CALIB[0] = subprocess.Popen(
        [exe, "--ros-args",
         "-r", "imu:=/ros_robot_controller/imu_raw", "-p", f"output_file:={OUTPUT}"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    out = collections.deque()
    threading.Thread(target=lambda: [out.append(l) for l in calib.stdout], daemon=True).start()

    def wait_output(word, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if any(word in l for l in out):
                return True
            if calib.poll() is not None:
                return any(word in l for l in out)
            time.sleep(0.1)
        return False

    def abort(msg):
        print(f"\n\nABORTED: {msg}\nThe calibration file was NOT changed.", flush=True)
        calib.kill()
        sys.exit(1)

    print(__doc__.split("\n\n")[1].replace("\n", " "), "\n", flush=True)
    for n, (label, axis, sign, hint) in enumerate(POSES, 1):
        print(f"[{n}/6] {label} up: {hint}", flush=True)
        held_since = None
        while True:
            ready, _, line = pose_state(axis, sign)
            print("\r   " + line + " " * 6, end="", flush=True)
            if ready:
                held_since = held_since or time.time()
                if time.time() - held_since >= HOLD_S:
                    break
            else:
                held_since = None
            if calib.poll() is not None:
                abort("do_calib exited unexpectedly:\n" + "".join(out))
            time.sleep(0.2)
        print(f"\n   Recording - keep the car still (~{500 / 48:.0f} s)...", end="", flush=True)
        done_before = sum(" Done." in l for l in out)
        calib.stdin.write("\n")
        calib.stdin.flush()
        end = time.time() + 60
        while sum(" Done." in l for l in out) <= done_before:
            _, mean, _ = pose_state(axis, sign)
            if not TEST and mean is not None and (sign * mean[axis] < IN_POSE - 0.5):
                abort(f"the car left the {label} pose while recording. Run it again.")
            if time.time() > end:
                abort("timed out waiting for do_calib to finish recording.")
            time.sleep(0.2)
        print(" done.\n", flush=True)

    if not wait_output("Saving calibration file... Success!", 60):
        abort("do_calib could not compute or save the calibration:\n" + "".join(out))
    calib.wait(timeout=10)
    print("Calibration computed and saved to", OUTPUT, flush=True)


if __name__ == "__main__":
    main()
