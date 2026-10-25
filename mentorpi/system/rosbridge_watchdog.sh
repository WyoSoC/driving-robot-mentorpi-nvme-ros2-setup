#!/bin/bash
# Run every minute by mentorpi-watchdog.timer (as root). Restarts the robot software if
# rosbridge (port 9090, the web UI's link to ROS) is gone or its memory runs away.
# Background: rosbridge 1.3.2 can leak when a browser disconnects mid-setup; on car 1 it grew
# to 6.6 GB and was killed by the kernel (2026-10-16). system/patch_rosbridge.py fixes the
# known cause; this is the safety net. Each action is logged: journalctl -u mentorpi-watchdog
MAX_RSS_KB=${MAX_RSS_KB:-1000000}   # 1 GB (normal: ~80 MB)
GRACE_S=180                         # give bringup time to start

S="$(dirname "$0")/../state"
[ -e "$S/charging" ] && exit 0   # charging mode: robot software stopped on purpose
# a charging-mode switch asked for or under way (it stops rosbridge first): restarting now would undo it
grep -qx on "$S/charging_request" 2>/dev/null && exit 0
systemctl is-active -q mentorpi-charging.service && exit 0
systemctl is-active -q mentorpi-bringup || exit 0
since=$(systemctl show mentorpi-bringup -p ActiveEnterTimestamp --value)
[ -n "$since" ] && [ $(( $(date +%s) - $(date -d "$since" +%s) )) -ge $GRACE_S ] || exit 0

pid=$(docker exec MentorPi pgrep -f "lib/rosbridge_server/rosbridge_websocket" 2>/dev/null | head -1)
rss=0
[ -n "$pid" ] && rss=$(docker exec MentorPi awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null)
listening=0; ss -ltn | grep -q ":9090 " && listening=1

reason=""
[ "$listening" = 1 ] || reason="rosbridge not listening on :9090"
[ "${rss:-0}" -gt "$MAX_RSS_KB" ] && reason="rosbridge using ${rss} kB (> ${MAX_RSS_KB} kB)"
[ -z "$reason" ] && { [ -n "$DRY_RUN" ] && echo "ok: listening=$listening rss=${rss} kB"; exit 0; }
[ -n "$DRY_RUN" ] && { echo "would restart: $reason"; exit 0; }

echo "$reason: restarting mentorpi-bringup and mentorpi-webui"
systemctl restart mentorpi-bringup
systemctl restart mentorpi-webui
