#!/bin/bash
# Package this MentorPi system for transfer to another (identical) car.
#
# Usage: ./make_bundle.sh [OUTPUT_DIR]     (default ~/mentorpi_transfer)
#
# Produces in OUTPUT_DIR:
#   mentorpi-kit.tar.gz              scripts, web UI, udev rules, services, docs
#   mentorpi-factory.tar.zst(+.sha256)  the Docker image (~GBs; skipped if up to date)
# Per-car files (car.env, calibration/) are NOT included.

set -euo pipefail

DIR=$(cd "$(dirname "$0")" && pwd -P)   # real path: ~/mentorpi may be a symlink into the git clone
OUT=${1:-$HOME/mentorpi_transfer}
IMAGE=mentorpi:factory
IMG_FILE="$OUT/mentorpi-factory.tar.zst"
mkdir -p "$OUT"

echo "==> Kit"
tar -C "$(dirname "$DIR")" -czf "$OUT/mentorpi-kit.tar.gz" \
    --exclude=mentorpi/car.env --exclude=mentorpi/calibration --exclude=mentorpi/maps --exclude=mentorpi/.git \
    --exclude='__pycache__' --exclude=mentorpi/tool/Log \
    "$(basename "$DIR")"
echo "    $OUT/mentorpi-kit.tar.gz ($(du -h "$OUT/mentorpi-kit.tar.gz" | cut -f1))"

echo "==> Docker image"
IMG_ID=$(docker image inspect -f '{{.Id}}' "$IMAGE")
if [ -f "$IMG_FILE" ] && [ "$(cat "$IMG_FILE.id" 2>/dev/null)" = "$IMG_ID" ]; then
    echo "    up to date: $IMG_FILE"
else
    echo "    exporting $IMAGE (takes a while)..."
    docker save "$IMAGE" | zstd -T0 -6 -q -f -o "$IMG_FILE"
    (cd "$OUT" && sha256sum "$(basename "$IMG_FILE")" > "$(basename "$IMG_FILE").sha256")
    echo "$IMG_ID" > "$IMG_FILE.id"
    echo "    $IMG_FILE ($(du -h "$IMG_FILE" | cut -f1))"
fi

cp "$DIR/README.md" "$OUT/README.md"
echo
echo "Copy $OUT to the new car (USB drive, or: rsync -aP $OUT/ user@newcar:mentorpi_transfer/)"
echo "Then on the new car:"
echo "  tar -C ~ -xzf ~/mentorpi_transfer/mentorpi-kit.tar.gz"
echo "  ~/mentorpi/install.sh --image ~/mentorpi_transfer/mentorpi-factory.tar.zst"
