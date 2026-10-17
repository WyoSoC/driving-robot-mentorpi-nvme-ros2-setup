#!/bin/bash
# Start the MentorPi web control UI inside the MentorPi container.
# web_ui/ is bind-mounted into the container by run_container.sh.
# Requires the container and bringup to be running.
#
# Usage: start_web_ui.sh               start in the background
#        start_web_ui.sh --foreground  run attached (used by mentorpi-webui.service)

set -e

DIR=$(cd "$(dirname "$0")" && pwd)
CONTAINER=MentorPi
source "$DIR/../car.env" 2>/dev/null || true
PORT=${WEB_UI_PORT:-8000}
CAR=${CAR_NAME:-$(hostname -s)}
# Other cars to exchange status with: every car in fleet.env except this one (same web UI port assumed).
# PEERS="host:port ..." in car.env overrides this.
if [ -z "${PEERS+x}" ] && [ -f "$DIR/../fleet.env" ]; then
    source "$DIR/../fleet.env"
    PEERS=""
    for h in $FLEET; do
        h=${h#*@}
        [ "${h%%.*}" = "$(hostname -s)" ] && continue
        PEERS+="$h:$PORT "
    done
fi
ENVS=(-e WEB_UI_PORT="$PORT" -e CAR_NAME="$CAR" -e PEERS="${PEERS:-}")
RUN="source ~/.zshrc >/dev/null 2>&1; exec python3 web_ui_server.py"

docker exec "$CONTAINER" pkill -f web_ui_server.py 2>/dev/null || true

if [ "$1" = "--foreground" ]; then
    exec docker exec -u ubuntu -w /home/ubuntu/web_ui "${ENVS[@]}" "$CONTAINER" /bin/zsh -c "$RUN"
fi

docker exec -d -u ubuntu -w /home/ubuntu/web_ui "${ENVS[@]}" "$CONTAINER" \
    /bin/zsh -c "$RUN > /tmp/web_ui.log 2>&1"

echo "MentorPi web UI starting:"
for ip in $(hostname -I); do
    if [[ $ip == *.* && $ip != 172.17.* ]]; then echo "  http://$ip:$PORT"; fi
done
echo "Server log: docker exec $CONTAINER cat /tmp/web_ui.log"
