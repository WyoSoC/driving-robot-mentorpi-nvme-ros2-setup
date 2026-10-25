"""Patch the factory EKF config (controller/config/ekf.yaml) inside a freshly started MentorPi container.

Bug: the EKF took the heading (orientation yaw) and turn rate from /imu, the output of
imu_complementary_filter. That filter re-estimates the gyro bias whenever it thinks the car is steady
(turning < 0.2 rad/s at a steady rate, not accelerating), so slow turns are absorbed within ~2 s and
leave a phantom counter-turn afterwards; its orientation also jumped ~180 deg while the car
accelerated. And the EKF output heading came out mirrored (car 1, hand-turned 90 deg left: clean gyro
+89, /imu +41, /odom -41; after turning back: +1 / -35 / +35).

Fix: fuse only the turn rate (vyaw) from /imu_corrected (imu_calib: gyro bias removed once at start,
while the car stands still; 0.01-0.03 deg/s drift). The heading is the EKF's own integral of it.
Speed still comes from /odom_raw, but not its turn rate: its "stopped" covariance pinned the turn
rate to 0 the moment the command reached 0, while the car was still rolling and turning (car 2,
23.8 m loop with 27 stops: EKF heading +27 deg off at the start point, the gyro alone +9.5). So the
heading is the gyro alone; parked it drifts ~1-2 deg/min, which SLAM corrects.
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
OLD_ODOM = """      odom0_config: [false, false, false,
                     false, false, false,
                     true, true, false,
                     false, false, true,
                     false, false, false]"""
NEW_ODOM = """      odom0_config: [false, false, false,
                     false, false, false,
                     true, true, false,
                     false, false, false,
                     false, false, false]"""
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

# Second patch (2026-10-25): measured forward speed from the lidar. The factory odom1 input (/odom_rf2o, as a
# pose) was never published. It becomes /odom_lidar_speed (web_ui/lidar_odometry.py: rf2o's forward speed
# averaged over 0.5 s, sigma 2 cm/s), fused as forward speed only, next to the commanded speed of /odom_raw
# (which stays: it keeps a parked car at exactly 0, and is the fallback when the lidar speed is missing).
MARK2 = "# mentorpi patch: forward speed from the lidar (/odom_lidar_speed)"
OLD_ODOM1 = """      odom1: odom_rf2o
      odom1_config: [true, true, false,
                     false, false, true,
                     false, false, false,
                     false, false, false,
                     false, false, false]
      odom1_differential: false
      odom1_relative: true"""
NEW_ODOM1 = f"""      {MARK2}
      odom1: odom_lidar_speed
      odom1_config: [false, false, false,
                     false, false, false,
                     true, false, false,
                     false, false, false,
                     false, false, false]
      odom1_differential: false
      odom1_relative: false"""

done = 0
for path in FILES:
    try:
        src = open(path).read()
    except FileNotFoundError:
        continue
    if MARK not in src:
        for old in (OLD_TOPIC, OLD_CFG, OLD_ODOM):
            if src.count(old) != 1:
                sys.exit(f"patch_ekf: factory ekf.yaml changed ({path}), cannot apply patch:\n{old}")
        src = src.replace(OLD_TOPIC, NEW_TOPIC).replace(OLD_CFG, NEW_CFG).replace(OLD_ODOM, NEW_ODOM)
    if MARK2 not in src:
        if src.count(OLD_ODOM1) != 1:
            sys.exit(f"patch_ekf: factory odom1 block not found ({path}), cannot add the lidar speed")
        src = src.replace(OLD_ODOM1, NEW_ODOM1)
    open(path, "w").write(src)
    done += 1
if not done:
    sys.exit("patch_ekf: no ekf.yaml found")
print(f"ekf patched ({done} files): heading from the gyro (/imu_corrected) only, forward speed also from the lidar")
