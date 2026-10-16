#!/bin/bash
# Guided IMU (accelerometer) calibration for this car. Run it on the car, in a terminal,
# with the robot system running. You will place the car in 6 poses, ~11 s each.
#
# The result replaces calibration/imu_calib.yaml only if the run succeeds and the values look
# sane; the previous file is kept as calibration/imu_calib.yaml.bak-<date>. Bringup is then
# restarted so the new values are used.

set -euo pipefail

DIR=$(cd "$(dirname "$0")" && pwd)
CONTAINER=MentorPi
TMP=/tmp/imu_calib_new.yaml
CALIB="$DIR/calibration/imu_calib.yaml"

docker ps --format '{{.Names}}' | grep -q "^$CONTAINER$" || { echo "MentorPi container not running" >&2; exit 1; }
systemctl is-active -q mentorpi-bringup || { echo "mentorpi-bringup not running" >&2; exit 1; }

cleanup() { docker exec "$CONTAINER" pkill -f "python3 -u - $TMP" 2>/dev/null || true; }
trap 'cleanup; echo; echo "Stopped. Calibration unchanged."; exit 130' INT TERM

cat <<'EOF'
IMU calibration
- Use a flat, level surface. A box or a stack of books helps to hold the car in each pose;
  a few degrees of tilt is fine.
- Keep the car switched on. Mind the cables; the wheels won't be driven.
- The script waits until the car is in the requested pose and still, then records ~11 s.
- Press Ctrl+C to stop at any time; nothing is changed unless all 6 poses succeed.

EOF
read -rp "Press Enter to start... "
echo

docker exec "$CONTAINER" rm -f "$TMP"
if ! docker exec -i -u ubuntu -e IMU_CALIB_TEST="${IMU_CALIB_TEST:-0}" "$CONTAINER" \
        /bin/zsh -c "source ~/.zshrc >/dev/null 2>&1; exec python3 -u - $TMP" < "$DIR/tools/imu_calib_guided.py"; then
    cleanup
    echo "Calibration did not complete; $CALIB was NOT changed." >&2
    exit 1
fi

NEW=$(mktemp)
docker cp "$CONTAINER:$TMP" "$NEW" >/dev/null
# Sanity check: scale/misalignment matrix near identity, bias below 1.5 m/s^2.
python3 - "$NEW" <<'EOF' || { echo "Values look wrong; calibration NOT installed (result kept in $NEW)" >&2; exit 1; }
import sys, re
t = open(sys.argv[1]).read()
nums = [float(x) for x in re.findall(r"-?\d+\.\d+(?:e-?\d+)?", t)]
sm, bias = nums[:9], nums[9:12]
assert len(nums) == 12, "unexpected file format"
for i in range(9):
    lim = (0.9, 1.1) if i in (0, 4, 8) else (-0.1, 0.1)
    assert lim[0] <= sm[i] <= lim[1], f"SM[{i}]={sm[i]:.4f} outside {lim}"
assert all(abs(b) < 1.5 for b in bias), f"bias {bias} too large"
print("    SM diagonal: " + ", ".join(f"{sm[i]:.4f}" for i in (0, 4, 8)))
print("    bias (m/s^2): " + ", ".join(f"{b:+.4f}" for b in bias))
EOF

BAK="$CALIB.bak-$(date +%Y%m%d-%H%M%S)"
cp "$CALIB" "$BAK"
cat "$NEW" > "$CALIB"   # write in place: the file is bind-mounted into the container
rm -f "$NEW"
echo "Installed new calibration ($CALIB); previous values in $BAK"

echo "Restarting bringup to load it..."
sudo systemctl restart mentorpi-bringup
for i in $(seq 60); do
    n=$(docker exec -u ubuntu "$CONTAINER" /bin/zsh -c 'source ~/.zshrc >/dev/null 2>&1; ros2 node list 2>/dev/null | wc -l' 2>/dev/null || echo 0)
    [ "$n" -ge 26 ] && break; sleep 3
done
echo "Put the car flat on its wheels. Corrected reading should be about x=0, y=0, z=9.81:"
sleep 3
docker exec -u ubuntu "$CONTAINER" /bin/zsh -c 'source ~/.zshrc >/dev/null 2>&1; timeout 10 ros2 topic echo --once /imu_corrected --field linear_acceleration'
