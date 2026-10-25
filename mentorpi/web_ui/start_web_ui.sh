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
TZ_HOST=$(timedatectl show -p Timezone --value 2>/dev/null || true)   # map names use local time
# This car's secure address (Tailscale serve, README "HTTPS"), for the page's talk button on the http page.
HTTPS_URL=$(tailscale status --self --json 2>/dev/null | python3 -c 'import json, sys; d = json.load(sys.stdin); print("https://" + d["CertDomains"][0] + "/")' 2>/dev/null || true)
# VLM servers (vlm_client.py): VLM_HOSTS="server1 server2 server3" (short tailnet names, in fleet.env or car.env)
# -> https://<name>.<tailnet>:11434 (Tailscale serve in front of Ollama). VLM_URLS="url ..." overrides.
[ -f "$DIR/../fleet.env" ] && source "$DIR/../fleet.env"
source "$DIR/../car.env" 2>/dev/null || true
if [ -z "${VLM_URLS:-}" ] && [ -n "${VLM_HOSTS:-}" ]; then
    SUFFIX=$(tailscale status --json 2>/dev/null | python3 -c 'import json, sys; print(json.load(sys.stdin)["MagicDNSSuffix"])' 2>/dev/null || true)
    for h in $VLM_HOSTS; do
        [[ $h == *.* || -z $SUFFIX ]] && VLM_URLS+="https://$h:11434 " || VLM_URLS+="https://$h.$SUFFIX:11434 "
    done
fi
ENVS=(-e WEB_UI_PORT="$PORT" -e CAR_NAME="$CAR" -e PEERS="${PEERS:-}" -e TZ="${TZ_HOST:-UTC}" -e HTTPS_URL="${HTTPS_URL:-}"
      -e VLM_URLS="${VLM_URLS:-}" -e VLM_MODEL="${VLM_MODEL:-}")
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
