#!/bin/bash
# Start or restart the MentorPi factory Docker container.
# Adapted from the Hiwonder SD card factory image for the NVMe setup.

set -e

CONTAINER_NAME="MentorPi"
IMAGE="mentorpi:factory"

# Allow X11 connections from the container
xhost + 2>/dev/null || true

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing ${CONTAINER_NAME} container..."
    docker rm -f "${CONTAINER_NAME}"
fi

echo "Starting ${CONTAINER_NAME}..."
docker run -d \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network host \
    -e DISPLAY=:0 \
    -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e XAUTHORITY="" \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /run/user/1000/pulse:/run/user/1000/pulse \
    -v /var/lib/dbus:/var/lib/dbus \
    -v /home/gojian/docker/tmp:/home/ubuntu/shared \
    "${IMAGE}" \
    tail -f /dev/null

echo "${CONTAINER_NAME} container started."
echo "To launch ROS2: docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c 'source ~/.zshrc; ros2 launch bringup bringup.launch.py'"
