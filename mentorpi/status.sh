#!/bin/bash
# Health check for the MentorPi system: services, devices, ROS topics, web UI.

DIR=$(cd "$(dirname "$0")" && pwd)
source "$DIR/car.env" 2>/dev/null || true
PORT=${WEB_UI_PORT:-8000}

echo "== ${CAR_NAME:-?}  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?})"
echo; echo "-- services"
for u in mentorpi-container mentorpi-bringup mentorpi-webui; do
    s=$(systemctl is-active $u)
    [ "$u" = mentorpi-bringup ] && [ -e "$DIR/state/charging" ] && s="stopped (charging mode)"
    printf "  %-20s %s / %s\n" "$u" "$s" "$(systemctl is-enabled $u 2>/dev/null)"
done
[ -e "$DIR/state/charging" ] && echo "  CHARGING MODE: only the board (battery) and rosbridge run; turn it off on the web page"

echo; echo "-- devices"
for d in /dev/rrc /dev/ldlidar; do
    [ -e $d ] && echo "  $d -> $(readlink $d)" || echo "  $d MISSING"
done
lsusb -d 3482:6723 >/dev/null && echo "  depth camera present" || echo "  depth camera MISSING"

echo; echo "-- calibration (this car only: calibration/, not in git)"
C="$DIR/calibration"
if [ -f "$C/imu_calib.yaml" ]; then
    cmp -s "$C/imu_calib.yaml" "$DIR/calibration.defaults/imu_calib.yaml" && echo "  IMU: factory defaults" \
        || echo "  IMU: calibrated ($(date -r "$C/imu_calib.yaml" +%F))"
else echo "  IMU: MISSING"; fi
val() { grep -E "^\s*$1:" "$C/calibrate_params.yaml" 2>/dev/null | sed -E 's/^[^:]*:\s*//'; }
m=$(val motor_speed_map); n=$(echo "$m" | tr -cd ',' | wc -c)
if [ -n "$m" ]; then
    echo "  speed map: $(( (n + 1) / 2 )) points, up to $(echo "$m" | tr -d '[]' | awk -F, '{print $(NF-1)}') m/s real"
else
    echo "  speed map: none (motor_speed_factor $(val motor_speed_factor | grep . || echo 1.0))"
fi
g=$(val steer_gain); tr_=$(grep -E '^3:' "$C/servo_config.yaml" 2>/dev/null | sed -E 's/^3:\s*//')
echo "  steering: port ${STEER_SERVO_PORT:-4}, centre trim ${tr_:-0} us, gain ${g:-1.0}"
echo "  odometry factors: linear $(val linear_correction_factor), angular $(val angular_correction_factor)"

echo; echo "-- container"
if ! docker ps --format '{{.Names}}' | grep -q '^MentorPi$'; then
    echo "  MentorPi container not running"; exit 1
fi
docker exec -u ubuntu MentorPi /bin/zsh -c '
source ~/.zshrc >/dev/null 2>&1
echo "  ROS nodes: $(ros2 node list 2>/dev/null | wc -l)"
for t in /odom /imu /scan_raw /ascamera/camera_publisher/rgb0/image; do
    r=$(timeout -k 2 6 ros2 topic hz $t 2>/dev/null | grep -m1 "average rate" | awk "{print \$3}")
    [ -n "$r" ] && r="$r Hz" || r="NO DATA"
    printf "  %-42s %s\n" $t "$r"
done
b=$(timeout -k 2 6 ros2 topic echo --once --field data /ros_robot_controller/battery 2>/dev/null | head -1)
echo "  battery: ${b:-?} mV"'

echo; echo "-- web"
for p in 9090 8080 $PORT; do
    ss -ltn | grep -q ":$p " && echo "  port $p listening" || echo "  port $p NOT listening"
done
for ip in $(hostname -I); do
    if [[ $ip == *.* && $ip != 172.17.* ]]; then echo "  http://$ip:$PORT"; fi
done
