#!/bin/bash
# Odometry calibration with the lidar (distance) and IMU (rotation) as references.
# THE CAR DRIVES (0.2 m/s). Run on the car, in a terminal, with the robot system running.
#
#   calibrate_odom.sh check     no driving: is the car placed right?
#   calibrate_odom.sh linear    drives ~0.8 m toward a flat wall and back -> motor_speed_factor
#                               (real/commanded speed; motor commands are divided by it)
#   calibrate_odom.sh angular   drives one left and one right circle (~0.8 m wide)
#                               -> angular_correction_factor (run linear first)
#   add --yes to skip the questions (start + save), e.g. when someone at the car confirmed
#
# Results go to calibration/calibrate_params.yaml (old file kept as .bak-<date>) and bringup is
# restarted so odom_publisher loads them.

set -euo pipefail

DIR=$(cd "$(dirname "$0")" && pwd)
PARAMS="$DIR/calibration/calibrate_params.yaml"
MODE=${1:-}
YES=0; [ "${2:-}" = "--yes" ] && YES=1
case "$MODE" in check|linear|angular) ;; *) sed -n 2,11p "$0"; exit 1 ;; esac

docker ps --format '{{.Names}}' | grep -q '^MentorPi$' || { echo "MentorPi container not running" >&2; exit 1; }
cur() { v=$(grep -E "^\s*$1:" "$PARAMS" | awk '{print $2}'); echo "${v:-1.0}"; }
LIN=$(cur linear_correction_factor); ANG=$(cur angular_correction_factor); MOT=$(cur motor_speed_factor)
echo "Current factors: motor speed $MOT, linear $LIN, angular $ANG"

if [ "$MODE" = linear ]; then cat <<'EOF'
Linear calibration: place the car on the floor, wheels straight, facing a FLAT WALL square-on
1.0-4.0 m away, nothing in between, and ~1.3 m free behind it. It drives ~0.8 m toward the wall
at 0.2 m/s, stops, then drives back. It stops by itself if anything gets within 45 cm.
EOF
elif [ "$MODE" = angular ]; then cat <<'EOF'
Angular calibration: place the car on the floor with ~1.2 m clear all around. It drives one
full left circle and one right circle (~0.8 m wide) at 0.2 m/s. Stops if anything is within 25 cm.
EOF
fi
if [ "$MODE" != check ] && [ "$YES" = 0 ]; then
    read -rp "THE CAR WILL DRIVE. Type 'go' to start: " a; [ "$a" = go ] || { echo "Cancelled."; exit 1; }
fi

OUT=$(mktemp)
docker exec -i -u ubuntu MentorPi /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec python3 -u - $MODE $LIN $ANG $MOT" \
    < "$DIR/tools/odom_calib.py" | tee "$OUT" || { echo "Calibration did not complete; nothing changed." >&2; exit 1; }
[ "$MODE" = check ] && exit 0

line=$(grep '^RESULT ' "$OUT" | head -1); rm -f "$OUT"
[ -n "$line" ] || { echo "No result; nothing changed." >&2; exit 1; }
key=$(echo "$line" | sed -E 's/^RESULT ([a-z_]+)=.*/\1/'); val=$(echo "$line" | sed -E 's/^RESULT [a-z_]+=([0-9.]+).*/\1/')
lo=0.7; hi=1.3; [ "$key" = motor_speed_factor ] && { lo=0.3; hi=3.0; }
python3 -c "import sys; v,lo,hi=map(float,sys.argv[1:]); sys.exit(0 if lo <= v <= hi else 1)" "$val" $lo $hi \
    || { echo "New $key=$val is outside $lo-$hi, looks wrong; nothing changed." >&2; exit 1; }

if [ "$YES" = 0 ]; then
    read -rp "Save $key=$val? [y/N] " a; [ "$a" = y ] || { echo "Not saved."; exit 0; }
fi
cp "$PARAMS" "$PARAMS.bak-$(date +%Y%m%d-%H%M%S)"
# rewrite in place (same inode): the file is bind-mounted into the container, sed -i would replace it
python3 - "$PARAMS" "$key" "$val" <<'PY'
import re, sys
p, k, v = sys.argv[1:]
t = open(p).read()
if re.search(r'(?m)^\s*' + k + r':', t):
    t = re.sub(r'(?m)^(\s*' + k + r':).*$', r'\g<1> ' + v, t)
else:   # add it under ros__parameters, same indent as the other keys
    ind = re.search(r'(?m)^(\s*)angular_correction_factor:', t).group(1)
    t = t.rstrip('\n') + f'\n{ind}{k}: {v}\n'
open(p, 'w').write(t)
PY
echo "Saved $key=$val in $PARAMS"
sudo systemctl restart mentorpi-bringup
echo "Restarted bringup to load it."
