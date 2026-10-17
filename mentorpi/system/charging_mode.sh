#!/bin/bash
# Manual charging mode: stop the robot software (camera, lidar, SLAM, odometry, driving) so more of
# the charger's current goes into the battery; keep only the board node (battery voltage) and
# rosbridge, so the web page stays up and shows the battery. The web server stays on throughout.
#
#   charging_mode.sh on | off | status
#   charging_mode.sh apply      do what the web page asked (state/charging_request); run as root by
#                               mentorpi-charging.service, triggered by mentorpi-charging.path
#
# State (state/, per car, cleared at every container start: a reboot always comes up normal):
#   state/charging_request   "on" / "off", written by web_ui_server.py
#   state/charging           "on" / "switching"; absent = normal mode
set -u
DIR=$(cd "$(dirname "$0")/.." && pwd)
S="$DIR/state"
mkdir -p "$S"
want=${1:-status}
[ "$want" = apply ] && want=$(tr -d '[:space:]' < "$S/charging_request" 2>/dev/null)
cur=$(tr -d '[:space:]' < "$S/charging" 2>/dev/null); cur=${cur:-off}
in_container() { docker exec -d -u ubuntu MentorPi /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec $1"; }

case "$want" in
on)
    [ "$cur" = on ] && exit 0
    echo switching > "$S/charging"
    echo "charging mode on: stopping the robot software"
    systemctl stop mentorpi-bringup        # its stop step ends every ROS process (not the web server)
    in_container "ros2 launch ros_robot_controller ros_robot_controller.launch.py"   # battery voltage
    in_container "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"      # web page link
    echo on > "$S/charging"
    echo "charging mode on: only the board node and rosbridge are running"
    ;;
off)
    [ "$cur" = off ] && exit 0
    echo switching > "$S/charging"
    echo "charging mode off: starting the robot software"
    systemctl start mentorpi-bringup       # start_node.sh first ends the charging-mode processes
    rm -f "$S/charging"
    ;;
status)
    echo "$cur"
    ;;
"")
    ;;
*)
    echo "usage: $0 on|off|status|apply" >&2; exit 1
    ;;
esac
