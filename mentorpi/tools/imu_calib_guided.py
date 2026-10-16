"""Guided accelerometer calibration for the MentorPi (runs inside the MentorPi container).

For each of the 6 poses it shows the live accelerometer reading, waits until the car is in
the right pose and still, then records 500 samples. If the car moves while recording, the run
is aborted. Results go to OUTPUT_FILE (a temporary file); calibrate_imu.sh installs it only
after a successful run.

Model: per axis, corrected = scale * raw - bias, with offset = (up + down) / 2 and
scale = g / ((up - down) / 2). Written in imu_calib's format (SM diagonal, bias) for
apply_calib, which computes SM * raw - bias. This replaces imu_calib's do_calib, whose
12-parameter fit assumes perfectly square poses: a car body lying on its side or end is
tilted a few degrees, and do_calib turned that into false cross-axis terms and offsets
(car 2: 2.7% magnitude error at rest vs 0.04% with this model from the same poses).

Usage (normally via ../calibrate_imu.sh): python3 -u imu_calib_guided.py OUTPUT_FILE
"""
import collections
import os
import signal
import statistics
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

OUTPUT = sys.argv[1]
G = 9.80665
SAMPLES = 500       # per pose (~11 s at 48 Hz)
IN_POSE = 8.0       # target axis must read at least this (m/s^2) with the right sign
OFF_AXIS = 2.5      # the other two axes must stay below this (~15 deg); small tilts are fine
STILL_STD = 0.25    # std dev over the last second while "still"
HOLD_S = 1.5        # pose must be held this long before recording starts
# Test mode: accept any pose, to exercise the full flow with the car standing still. The result
# is then nonsense and calibrate_imu.sh's sanity check is expected to reject it.
TEST = os.environ.get("IMU_CALIB_TEST") == "1"

# Axes as mounted on the MentorPi M1 controller board (verified on car 2):
# X+ = right side of the car, Y+ = front, Z+ = up. The live check is what counts.
POSES = [
    ("X+", 0, +1, "lying on its LEFT side (right side facing up)"),
    ("X-", 0, -1, "lying on its RIGHT side (left side facing up)"),
    ("Y+", 1, +1, "standing on its REAR end, front pointing up"),
    ("Y-", 1, -1, "standing on its FRONT end, rear pointing up"),
    ("Z+", 2, +1, "normal, on its wheels"),
    ("Z-", 2, -1, "upside down (support the body so the lidar/camera carry no weight)"),
]

window = collections.deque(maxlen=50)  # ~1 s, for the live pose check
recording = []                         # samples of the pose being recorded
lock = threading.Lock()
rec_on = False


def on_imu(msg):
    a = msg.linear_acceleration
    v = (a.x, a.y, a.z)
    with lock:
        window.append(v)
        if rec_on:
            recording.append(v)


def pose_state(axis, sign):
    with lock:
        s = list(window)
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


def fail(msg):
    print(f"\n\nABORTED: {msg}\nThe calibration file was NOT changed.", flush=True)
    sys.exit(1)


def record_pose(n, label, axis, sign, hint):
    global rec_on
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
        time.sleep(0.2)
    print(f"\n   Recording - keep the car still (~{SAMPLES / 48:.0f} s)...", end="", flush=True)
    with lock:
        recording.clear()
        rec_on = True
    end = time.time() + 60
    while True:
        with lock:
            got = len(recording)
        if got >= SAMPLES:
            break
        _, mean, _ = pose_state(axis, sign)
        if not TEST and mean is not None and sign * mean[axis] < IN_POSE - 0.5:
            fail(f"the car left the {label} pose while recording. Run it again.")
        if time.time() > end:
            fail("timed out waiting for IMU samples.")
        time.sleep(0.2)
    with lock:
        rec_on = False
        s = recording[:SAMPLES]
    mean = [statistics.fmean(v[i] for v in s) for i in range(3)]
    print(f" done. (x={mean[0]:+.3f} y={mean[1]:+.3f} z={mean[2]:+.3f})\n", flush=True)
    return mean[axis]


def main():
    # Turn SIGTERM/SIGHUP into SystemExit so cleanup runs.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(1))
    rclpy.init()
    node = rclpy.create_node("imu_calib_guide")
    node.create_subscription(Imu, "/ros_robot_controller/imu_raw", on_imu, qos_profile_sensor_data)
    spinner = threading.Thread(target=spin, args=(node,), daemon=True)
    spinner.start()
    try:
        print(__doc__.split("\n\n")[1].replace("\n", " "), "\n", flush=True)
        readings = {}
        for n, (label, axis, sign, hint) in enumerate(POSES, 1):
            readings[label] = record_pose(n, label, axis, sign, hint)
        scale, bias = [], []
        for a in "XYZ":
            up, down = readings[a + "+"], readings[a + "-"]
            half = (up - down) / 2
            if half <= 0:
                fail(f"{a} axis: up/down readings {up:+.3f}/{down:+.3f} make no sense.")
            s = G / half
            scale.append(s)
            bias.append(s * (up + down) / 2)
            print(f"{a}: up {up:+.4f}  down {down:+.4f}  -> scale {s:.5f}  bias {bias[-1]:+.5f}")
        sm = [scale[0], 0.0, 0.0, 0.0, scale[1], 0.0, 0.0, 0.0, scale[2]]
        with open(OUTPUT, "w") as f:
            f.write("SM:\n" + "".join(f"  - {v!r}\n" for v in sm))
            f.write("bias:\n" + "".join(f"  - {v!r}\n" for v in bias))
        print("Calibration computed and saved to", OUTPUT, flush=True)
    finally:
        # Stop the spin thread before the interpreter exits; otherwise rclpy aborts the process
        # ("terminate called without an active exception") and the caller sees a failure.
        rclpy.try_shutdown()
        spinner.join(timeout=5)
        node.destroy_node()


if __name__ == "__main__":
    main()
