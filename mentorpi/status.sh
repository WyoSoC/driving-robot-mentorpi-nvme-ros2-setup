#!/bin/bash
# Health check for the MentorPi system: services, devices, ROS topics, web UI.

DIR=$(cd "$(dirname "$0")" && pwd)
source "$DIR/car.env" 2>/dev/null || true
PORT=${WEB_UI_PORT:-8000}

echo "== ${CAR_NAME:-?}  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?})"
echo; echo "-- services"
for u in mentorpi-container mentorpi-bringup mentorpi-webui; do
    printf "  %-20s %s / %s\n" "$u" "$(systemctl is-active $u)" "$(systemctl is-enabled $u 2>/dev/null)"
done

echo; echo "-- devices"
for d in /dev/rrc /dev/ldlidar; do
    [ -e $d ] && echo "  $d -> $(readlink $d)" || echo "  $d MISSING"
done
lsusb -d 3482:6723 >/dev/null && echo "  depth camera present" || echo "  depth camera MISSING"

echo; echo "-- container"
if ! docker ps --format '{{.Names}}' | grep -q '^MentorPi$'; then
    echo "  MentorPi container not running"; exit 1
fi
docker exec -u ubuntu MentorPi /bin/zsh -c '
source ~/.zshrc >/dev/null 2>&1
echo "  ROS nodes: $(ros2 node list 2>/dev/null | wc -l)"
for t in /odom /imu /scan_raw /ascamera/camera_publisher/rgb0/image; do
    r=$(timeout 6 ros2 topic hz $t 2>/dev/null | grep -m1 "average rate" | awk "{print \$3}")
    [ -n "$r" ] && r="$r Hz" || r="NO DATA"
    printf "  %-42s %s\n" $t "$r"
done
b=$(timeout 6 ros2 topic echo --once --field data /ros_robot_controller/battery 2>/dev/null | head -1)
echo "  battery: ${b:-?} mV"'

echo; echo "-- web"
for p in 9090 8080 $PORT; do
    ss -ltn | grep -q ":$p " && echo "  port $p listening" || echo "  port $p NOT listening"
done
for ip in $(hostname -I); do
    if [[ $ip == *.* && $ip != 172.17.* ]]; then echo "  http://$ip:$PORT"; fi
done
