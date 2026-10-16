#!/bin/bash
# Install / update the MentorPi system on this car. Safe to re-run.
#
# Usage:
#   ./install.sh [--image FILE] [--name NAME] [--domain-id N] [--no-enable]
#
#   --image FILE     mentorpi-factory.tar.zst to load if the Docker image is missing
#   --name NAME      CAR_NAME for a new car.env (default: hostname, e.g. jpi2)
#   --domain-id N    ROS_DOMAIN_ID for a new car.env (default: the number in the
#                    hostname, e.g. jpi2 -> 2; must be unique per car, 0-101)
#   --no-enable      install services but do not enable them at boot
#
# Run as the normal user (not root); sudo is used where needed.

set -euo pipefail

DIR=$(cd "$(dirname "$0")" && pwd)
IMAGE=mentorpi:factory
IMAGE_FILE="" NAME="" DOMAIN="" ENABLE=1

while [ $# -gt 0 ]; do
    case "$1" in
        --image) IMAGE_FILE=$2; shift 2 ;;
        --name) NAME=$2; shift 2 ;;
        --domain-id) DOMAIN=$2; shift 2 ;;
        --no-enable) ENABLE=0; shift ;;
        -h|--help) sed -n 2,13p "$0"; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

step() { echo; echo "==> $*"; }
ok() { echo "    ok: $*"; }
warn() { echo "    WARNING: $*"; }

[ "$(id -u)" -ne 0 ] || { echo "Run as your normal user, not root." >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || warn "expected aarch64 (Raspberry Pi 5), got $(uname -m)"

step "Docker"
if ! command -v docker >/dev/null; then
    sudo apt-get update && sudo apt-get install -y docker.io
fi
sudo systemctl enable --now docker >/dev/null 2>&1
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    echo "Added $USER to the docker group. Log out and back in (or reboot), then re-run ./install.sh"
    exit 0
fi
ok "$(docker --version)"

step "Docker image $IMAGE"
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
    ok "already present"
else
    [ -n "$IMAGE_FILE" ] || { echo "Image missing. Re-run with --image /path/to/mentorpi-factory.tar.zst" >&2; exit 1; }
    command -v zstd >/dev/null || sudo apt-get install -y zstd
    if [ -f "$IMAGE_FILE.sha256" ]; then
        echo "    verifying checksum (a few minutes)..."
        (cd "$(dirname "$IMAGE_FILE")" && sha256sum -c --quiet "$(basename "$IMAGE_FILE").sha256")
    fi
    echo "    loading image (several minutes)..."
    zstd -dc "$IMAGE_FILE" | docker load
    ok "loaded"
fi

step "Per-car config (car.env)"
if [ -f "$DIR/car.env" ]; then
    ok "keeping existing car.env"
else
    NAME=${NAME:-$(hostname -s)}
    DOMAIN=${DOMAIN:-$(hostname -s | grep -oE '[0-9]+$' | sed 's/^0*\([0-9]\)/\1/' || true)}
    [[ "$DOMAIN" =~ ^[0-9]+$ ]] && [ "$DOMAIN" -le 101 ] || { echo "Cannot derive ROS_DOMAIN_ID from hostname '$(hostname -s)'. Re-run with --domain-id N (unique per car, 0-101)" >&2; exit 1; }
    sed -e "s/^CAR_NAME=.*/CAR_NAME=$NAME/" \
        -e "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$DOMAIN/" "$DIR/car.env.example" > "$DIR/car.env"
    ok "created car.env"
fi
grep -E '^(CAR_NAME|ROS_DOMAIN_ID|WEB_UI_PORT)=' "$DIR/car.env" | sed 's/^/    /'

step "Calibration"
mkdir -p "$DIR/calibration"
for f in imu_calib.yaml calibrate_params.yaml; do
    if [ -f "$DIR/calibration/$f" ]; then
        ok "keeping calibration/$f"
    else
        cp "$DIR/calibration.defaults/$f" "$DIR/calibration/$f"
        warn "calibration/$f copied from defaults; recalibrate this car (see README)"
    fi
done

step "udev rules (/dev/rrc, /dev/ldlidar, depth camera access)"
sudo install -m 644 "$DIR/system/99-mentorpi.rules" "$DIR/system/56-angstrong-camera.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
sudo udevadm trigger --action=add --subsystem-match=usb
sudo udevadm settle
ok "installed"

step "Keep ROS 2 shared memory across logouts (logind RemoveIPC=no)"
sudo install -D -m 644 "$DIR/system/90-mentorpi-keep-ipc.conf" /etc/systemd/logind.conf.d/90-mentorpi-keep-ipc.conf
sudo systemctl reload systemd-logind 2>/dev/null || warn "reload systemd-logind failed; takes effect after reboot"
ok "$(systemd-analyze cat-config systemd/logind.conf 2>/dev/null | grep -E '^RemoveIPC=' | tail -1)"

step "Hardware check"
check() {  # vid:pid description [device]
    if lsusb -d "$1" >/dev/null 2>&1; then ok "$2 ($1)${3:+ -> $3}"; else warn "$2 ($1) not found on USB"; fi
}
check 1a86:55d4 "STM32 motor controller" /dev/rrc
check 1a86:7523 "MS200 lidar" /dev/ldlidar
[ "$(lsusb -d 1a86:7523 | wc -l)" -le 1 ] || ok "extra CH340 found (voice module?); /dev/ldlidar -> $(readlink /dev/ldlidar 2>/dev/null || echo MISSING)"
check 3482:6723 "Angstrong HP60C depth camera"

step "systemd services"
mkdir -p "$HOME/docker/tmp"
for unit in mentorpi-container.service mentorpi-bringup.service mentorpi-webui.service \
            mentorpi-watchdog.service mentorpi-watchdog.timer; do
    sed -e "s|@USER@|$USER|g" -e "s|@DIR@|$DIR|g" "$DIR/system/$unit" \
        | sudo tee "/etc/systemd/system/$unit" >/dev/null
done
sudo systemctl daemon-reload
if [ "$ENABLE" = 1 ]; then
    sudo systemctl enable mentorpi-container mentorpi-bringup mentorpi-webui >/dev/null 2>&1
    sudo systemctl enable --now mentorpi-watchdog.timer >/dev/null 2>&1
    ok "installed and enabled at boot"
else
    ok "installed (not enabled)"
fi

step "Done"
echo "    Start now:  sudo systemctl start mentorpi-webui   (starts container + bringup too)"
echo "    Status:     $DIR/status.sh"
