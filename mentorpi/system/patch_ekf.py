"""Patch the factory EKF config (controller/config/ekf.yaml) inside a freshly started MentorPi container.

Bug: the EKF took the heading (orientation yaw) and turn rate from /imu, the output of
imu_complementary_filter. That filter re-estimates the gyro bias whenever it thinks the car is steady
(turning < 0.2 rad/s at a steady rate, not accelerating), so slow turns are absorbed within ~2 s and
leave a phantom counter-turn afterwards; its orientation also jumped ~180 deg while the car
accelerated. And the EKF output heading came out mirrored (car 1, hand-turned 90 deg left: clean gyro
+89, /imu +41, /odom -41; after turning back: +1 / -35 / +35).

Fix: fuse only the turn rate (vyaw) from /imu_corrected (imu_calib: gyro bias removed once at start,
while the car stands still; 0.01-0.03 deg/s drift). The heading is the EKF's own integral of it.
Speed still comes from /odom_raw, whose "stopped" covariance pins the turn rate to 0 while parked.
/imu (the filter) stays for the web page's attitude display.

Run by run_container.sh: python3 patch_ekf.py [FILE ...]
"""
import sys

FILES = sys.argv[1:] or [
    "/home/ubuntu/ros2_ws/src/driver/controller/config/ekf.yaml",
    "/home/ubuntu/ros2_ws/install/controller/share/controller/config/ekf.yaml",
]
MARK = "# mentorpi patch: gyro turn rate from /imu_corrected"

OLD_TOPIC = "      imu0: imu\n"
NEW_TOPIC = f"      {MARK}\n      imu0: imu_corrected\n"
OLD_CFG = """      imu0_config: [false, false, false,
                    false, false, true,
                    false, false, false,
                    false, false, true,
                    false,  false, false]"""
NEW_CFG = """      imu0_config: [false, false, false,
                    false, false, false,
                    false, false, false,
                    false, false, true,
                    false,  false, false]"""

done = 0
for path in FILES:
    try:
        src = open(path).read()
    except FileNotFoundError:
        continue
    if MARK in src:
        done += 1
        continue
    for old in (OLD_TOPIC, OLD_CFG):
        if src.count(old) != 1:
            sys.exit(f"patch_ekf: factory ekf.yaml changed ({path}), cannot apply patch:\n{old}")
    src = src.replace(OLD_TOPIC, NEW_TOPIC).replace(OLD_CFG, NEW_CFG)
    open(path, "w").write(src)
    done += 1
if not done:
    sys.exit("patch_ekf: no ekf.yaml found")
print(f"ekf patched ({done} files): turn rate from /imu_corrected, no /imu orientation")
