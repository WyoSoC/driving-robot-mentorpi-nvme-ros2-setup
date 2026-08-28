#!/bin/bash
# Install udev rules for MentorPi hardware devices.
# Run once after first boot, then reboot or run: sudo udevadm control --reload-rules

set -e

RULES_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Installing MentorPi udev rules..."

sudo cp "$RULES_DIR/99-ttyACM0.rules"           /etc/udev/rules.d/
sudo cp "$RULES_DIR/lidar.rules"                 /etc/udev/rules.d/
sudo cp "$RULES_DIR/99-usb-cam.rules"            /etc/udev/rules.d/
sudo cp "$RULES_DIR/99-obsensor-libusb.rules"    /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done. Device mappings:"
echo "  STM32 controller  -> /dev/ttyACM0"
echo "  LiDAR (MS200)     -> /dev/ttyUSB0"
echo "  USB camera        -> /dev/video*"
echo "  Orbbec depth cam  -> (obsensor)"
