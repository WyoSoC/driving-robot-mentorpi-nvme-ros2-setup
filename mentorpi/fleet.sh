#!/bin/bash
# Keep all cars on the same version, from any one car (or a laptop with ssh access).
#
#   fleet.sh status    version, local changes, services and sensor rates of every car
#   fleet.sh update    git pull + install.sh + restart the robot system on every car, then status
#
# Cars are listed in fleet.env next to this script (not in git), e.g.
#   FLEET="gojian@jpi1.example.ts.net gojian@jpi2.example.ts.net"
# Needs ssh key access and passwordless sudo on each car (see README).
set -uo pipefail
DIR=$(cd "$(dirname "$0")" && pwd -P)
[ -f "$DIR/fleet.env" ] || { echo "Create $DIR/fleet.env with FLEET=\"user@car1 user@car2 ...\"" >&2; exit 1; }
source "$DIR/fleet.env"
REPO='~/driving-robot-mentorpi-nvme-ros2-setup'

status_cmd="cd $REPO && echo \"  version \$(git log -1 --format='%h %s' | cut -c1-60)\" && \
  { git fetch -q 2>/dev/null; b=\$(git rev-list --count HEAD..origin/main 2>/dev/null); [ \"\${b:-0}\" = 0 ] || echo \"  BEHIND origin by \$b commit(s)\"; } && \
  { c=\$(git status --short | wc -l); [ \$c = 0 ] || echo \"  \$c uncommitted change(s)\"; } && \
  ~/mentorpi/status.sh | grep -E 'ROS_DOMAIN|active|Hz|NO DATA|battery|NOT listening' | sed 's/^/  /'"
update_cmd="cd $REPO && git pull -q --ff-only && ~/mentorpi/install.sh >/dev/null && \
  sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui && \
  for i in \$(seq 60); do ss -ltn | grep -q ':9090 ' && break; sleep 3; done; echo '  updated and restarted'"

case "${1:-status}" in
    status) cmd=$status_cmd ;;
    update) cmd="$update_cmd && sleep 20 && $status_cmd" ;;
    *) sed -n 2,9p "$0"; exit 1 ;;
esac
for car in $FLEET; do
    echo "== $car"
    host=${car#*@}; host=${host%%.*}
    if [ "$host" = "$(hostname -s)" ]; then
        bash -c "$cmd" || echo "  FAILED"                     # this car: no ssh needed
    else
        ssh -o BatchMode=yes -o ConnectTimeout=10 "$car" "$cmd" || echo "  FAILED (unreachable, or ssh key not loaded?)"
    fi
done
