#!/bin/bash
# Speed and odometry calibration with the lidar (distance) and IMU (rotation) as references.
# THE CAR DRIVES. Run on the car, in a terminal, with the robot system running. Order:
#
#   calibrate_odom.sh check     no driving: is the car placed right?
#   calibrate_odom.sh speedmap  back and forth toward a flat wall at 7 motor speeds -> motor_speed_map
#                               (lidar-measured real speed; afterwards commanded = real speed)
#   calibrate_odom.sh linear    drives ~0.8 m toward the wall and back at 0.2 m/s: with a speed map
#                               it only verifies (ratio should be ~1.00), else -> motor_speed_factor
#   calibrate_odom.sh steer     straight toward the wall and back twice at 0.1 m/s -> steering centre
#                               trim (calibration/servo_config.yaml, key 3 = steering, us)
#   calibrate_odom.sh angular   one left and one right circle (~0.8 m wide) -> angular_correction_factor
#   add --yes to skip the questions (start + save), e.g. when someone at the car confirmed
#
# Results go to calibration/calibrate_params.yaml (old file kept as .bak-<date>) and bringup is
# restarted so odom_publisher loads them.

set -euo pipefail

DIR=$(cd "$(dirname "$0")" && pwd)
PARAMS="$DIR/calibration/calibrate_params.yaml"
MODE=${1:-}
YES=0; [ "${2:-}" = "--yes" ] && YES=1
case "$MODE" in check|speedmap|steer|linear|angular) ;; *) sed -n 2,16p "$0"; exit 1 ;; esac

docker ps --format '{{.Names}}' | grep -q '^MentorPi$' || { echo "MentorPi container not running" >&2; exit 1; }
cur() { v=$(grep -E "^\s*$1:" "$PARAMS" | sed -E 's/^[^:]*:\s*//'); echo "${v:-$2}"; }
LIN=$(cur linear_correction_factor 1.0); ANG=$(cur angular_correction_factor 1.0)
MOT=$(cur motor_speed_factor 1.0); MAP=$(cur motor_speed_map "")
SERVO="$DIR/calibration/servo_config.yaml"
[ -f "$SERVO" ] || cp "$DIR/calibration.defaults/servo_config.yaml" "$SERVO"
TRIM=$(grep -E '^3:' "$SERVO" | sed -E 's/^3:\s*//'); TRIM=${TRIM:-0}
echo "Current: motor_speed_map ${MAP:-none}, motor_speed_factor $MOT, linear $LIN, angular $ANG, steering trim $TRIM us"

# rewrite in place (same inode): the file is bind-mounted into the container, sed -i would replace it
setparam() {  # setparam KEY VALUE   (VALUE "" removes the key)
    python3 - "$PARAMS" "$1" "$2" <<'PY'
import re, sys
p, k, v = sys.argv[1:]
t = open(p).read()
if not v:
    t = re.sub(r'(?m)^\s*' + k + r':.*\n?', '', t)
elif re.search(r'(?m)^\s*' + k + r':', t):
    t = re.sub(r'(?m)^(\s*' + k + r':).*$', lambda m: m.group(1) + ' ' + v, t)
else:   # add it under ros__parameters, same indent as the other keys
    ind = re.search(r'(?m)^(\s*)angular_correction_factor:', t).group(1)
    t = t.rstrip('\n') + f'\n{ind}{k}: {v}\n'
open(p, 'w').write(t)
PY
}
restart_bringup() {
    sudo systemctl restart mentorpi-bringup
    echo -n "Restarted bringup, waiting for odometry + IMU ..."
    end=$((SECONDS + 120))
    until docker exec -u ubuntu MentorPi /bin/zsh -c 'source ~/.zshrc >/dev/null 2>&1; timeout -k 2 5 ros2 topic echo --once /imu >/dev/null 2>&1 && timeout -k 2 5 ros2 topic echo --once /odom_raw >/dev/null 2>&1'; do
        [ $SECONDS -lt $end ] || { echo " timed out" >&2; exit 1; }
    done
    echo " ok"
}

case "$MODE" in
speedmap) cat <<'EOF'
Speed map: place the car on the floor, wheels straight, facing a FLAT WALL square-on 1.5-4.0 m
away, nothing in between, ~1.3 m free behind. It drives ~0.7 m toward the wall and back at 7
speeds (slow to ~0.5 m/s real), about 2 minutes. Stops by itself if anything gets within 45 cm.
EOF
;;
linear) cat <<'EOF'
Linear: place the car on the floor, wheels straight, facing a FLAT WALL square-on 1.0-4.0 m away,
nothing in between, ~1.3 m free behind. It drives ~0.8 m toward the wall at 0.2 m/s, stops, then
drives back. It stops by itself if anything gets within 45 cm.
EOF
;;
steer) cat <<'EOF'
Steering centre: place the car on the floor, wheels straight, facing a FLAT WALL square-on 1.5-4.0 m
away, ~1.3 m free behind. It drives ~0.7 m toward the wall and back, twice, at 0.1 m/s, and measures
how much it turns. Stops by itself if anything gets within 45 cm.
EOF
;;
angular) cat <<'EOF'
Angular: place the car on the floor with ~1.5 m clear to the left and right and ~1 m ahead and
behind. It drives one full left circle and one right circle (~1.1 m wide) at 0.2 m/s. Stops if
anything is within 25 cm.
EOF
;;
esac
if [ "$MODE" != check ] && [ "$YES" = 0 ]; then
    read -rp "THE CAR WILL DRIVE. Type 'go' to start: " a; [ "$a" = go ] || { echo "Cancelled."; exit 1; }
fi

BACKUP="$PARAMS.bak-$(date +%Y%m%d-%H%M%S)"
if [ "$MODE" = speedmap ]; then
    # measure the raw motor response: no map, no factor
    cp "$PARAMS" "$BACKUP"
    trap 'echo "Interrupted: restoring previous settings"; cp "$BACKUP" "$PARAMS.tmp"; cat "$PARAMS.tmp" > "$PARAMS"; rm -f "$PARAMS.tmp"; sudo systemctl restart mentorpi-bringup' INT TERM
    setparam motor_speed_map ""; setparam motor_speed_factor ""
    restart_bringup
fi

OUT=$(mktemp)
if ! docker exec -i -u ubuntu MentorPi /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec python3 -u - $MODE $LIN $ANG $MOT $TRIM" \
    < "$DIR/tools/odom_calib.py" | tee "$OUT" || ! grep -q . "$OUT"; then
    echo "Calibration did not complete." >&2
fi
trap - INT TERM
restore() { cat "$BACKUP" > "$PARAMS"; echo "Previous settings restored."; restart_bringup; }
[ "$MODE" = check ] && exit 0
if [ "$MODE" = linear ] && [ -n "$MAP" ]; then
    echo "(speed map in use: this was a check, ratio ~1.00 means commanded = real speed; nothing saved)"
    exit 0
fi

line=$(grep '^RESULT ' "$OUT" | head -1 || true); rm -f "$OUT"
if [ -z "$line" ]; then
    echo "No result; nothing changed." >&2
    [ "$MODE" = speedmap ] && restore
    exit 1
fi
key=$(echo "$line" | sed -E 's/^RESULT ([a-z_]+)=.*/\1/'); val=$(echo "$line" | sed -E 's/^RESULT [a-z_]+=(-?[0-9.,]+).*/\1/')
if [ "$key" = steer_trim_us ]; then
    [ "$val" -ge -100 ] && [ "$val" -le 100 ] || { echo "New trim $val is outside -100..100 us; nothing changed." >&2; exit 1; }
elif [ "$key" = motor_speed_map ]; then
    val="[$val]"
else
    lo=0.7; hi=1.3; [ "$key" = motor_speed_factor ] && { lo=0.3; hi=3.0; }
    python3 -c "import sys; v,lo,hi=map(float,sys.argv[1:]); sys.exit(0 if lo <= v <= hi else 1)" "$val" $lo $hi \
        || { echo "New $key=$val is outside $lo-$hi, looks wrong; nothing changed." >&2; exit 1; }
fi

if [ "$YES" = 0 ]; then
    read -rp "Save $key=$val? [y/N] " a
    if [ "$a" != y ]; then
        echo "Not saved."
        [ "$MODE" = speedmap ] && restore
        exit 0
    fi
fi
if [ "$key" = steer_trim_us ]; then
    cp "$SERVO" "$SERVO.bak-$(date +%Y%m%d-%H%M%S)"
    python3 - "$SERVO" "$val" <<'PY'
import re, sys
p, v = sys.argv[1:]
t = open(p).read()
t = re.sub(r'(?m)^3:.*$', '3: ' + v, t) if re.search(r'(?m)^3:', t) else t.rstrip('\n') + f'\n3: {v}\n'
open(p, 'w').write(t)   # same inode: bind-mounted into the container
PY
    echo "Saved steering trim $val us in $SERVO"
    restart_bringup
    exit 0
fi
[ "$MODE" = speedmap ] || cp "$PARAMS" "$BACKUP"
setparam "$key" "$val"
[ "$key" = motor_speed_map ] && setparam linear_correction_factor 1.0
echo "Saved $key=$val in $PARAMS (previous: $BACKUP)"
restart_bringup
