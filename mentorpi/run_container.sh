#!/bin/bash
# Start or restart the MentorPi factory Docker container.
# Adapted from the Hiwonder SD card factory image for the NVMe setup.
#
# Per-car settings are applied from files next to this script:
#   car.env        ROS_DOMAIN_ID (patched into the image's .typerc on every start) and
#                  STEER_SERVO_PORT (patched into ros_robot_controller, system/patch_controller.py)
#   calibration/   IMU + odometry calibration, bind-mounted over the image defaults
#   web_ui/        web control UI, bind-mounted at /home/ubuntu/web_ui
#   maps/          SLAM maps saved from the web UI (per car), bind-mounted at /home/ubuntu/maps

set -e

DIR=$(cd "$(dirname "$0")" && pwd)
CONTAINER_NAME="MentorPi"
IMAGE="mentorpi:factory"
TYPERC=/home/ubuntu/ros2_ws/.typerc
# The nodes read config from the install tree (plain copies, not symlinks to src/), so the
# per-car calibration is mounted there; the src/ mounts are for Hiwonder tools that use src/.
WS_SHARE=/home/ubuntu/ros2_ws/install

if [ ! -f "$DIR/car.env" ]; then
    echo "Missing $DIR/car.env (copy car.env.example and set a unique ROS_DOMAIN_ID)" >&2
    exit 1
fi
source "$DIR/car.env"

# Allow X11 connections from the container (no-op when there is no display, e.g. at boot)
xhost + >/dev/null 2>&1 || true

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing ${CONTAINER_NAME} container..."
    docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

mkdir -p "$HOME/docker/tmp" "$DIR/maps"
UID_NUM=$(id -u)

echo "Starting ${CONTAINER_NAME} (${CAR_NAME}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."
docker run -d \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network host \
    -e DISPLAY=:0 \
    -e PULSE_SERVER=unix:/run/user/${UID_NUM}/pulse/native \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e XAUTHORITY="" \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /run/user/${UID_NUM}/pulse:/run/user/${UID_NUM}/pulse \
    -v /var/lib/dbus:/var/lib/dbus \
    -v "$HOME/docker/tmp:/home/ubuntu/shared" \
    -v "$DIR/web_ui:/home/ubuntu/web_ui" \
    -v "$DIR/maps:/home/ubuntu/maps" \
    -v "$DIR/calibration/imu_calib.yaml:/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml" \
    -v "$DIR/calibration/calibrate_params.yaml:/home/ubuntu/ros2_ws/src/driver/controller/config/calibrate_params.yaml" \
    -v "$DIR/calibration/imu_calib.yaml:$WS_SHARE/calibration/share/calibration/config/imu_calib.yaml" \
    -v "$DIR/calibration/calibrate_params.yaml:$WS_SHARE/controller/share/controller/config/calibrate_params.yaml" \
    -v "$DIR/calibration/servo_config.yaml:/home/ubuntu/software/Servo_upper_computer/servo_config.yaml" \
    "${IMAGE}" \
    tail -f /dev/null >/dev/null

# The image hardcodes ROS_DOMAIN_ID=0; give each car its own domain.
docker exec "${CONTAINER_NAME}" sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}/" "$TYPERC"

# Which RRC Lite PWM port the steering servo is plugged into (our standard: 4; factory: 3).
docker exec -i -u ubuntu -e BATTERY_ALARM_MV="${BATTERY_ALARM_MV:-6600}" "${CONTAINER_NAME}" \
    python3 - "${STEER_SERVO_PORT:-4}" < "$DIR/system/patch_controller.py"

# Odometry correction factors (calibration/calibrate_params.yaml) on the velocities the EKF uses.
docker exec -i -u ubuntu "${CONTAINER_NAME}" python3 - < "$DIR/system/patch_odom.py"

# rosbridge leak when a browser disconnects mid-setup (see system/patch_rosbridge.py).
docker exec -i "${CONTAINER_NAME}" python3 - < "$DIR/system/patch_rosbridge.py"

echo "${CONTAINER_NAME} container started."
echo "To launch ROS2: $DIR/start_node.sh"
