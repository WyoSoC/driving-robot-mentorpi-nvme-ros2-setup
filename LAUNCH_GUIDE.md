# MentorPi M1 — Launch Guide

**Platform:** Raspberry Pi 5 · Ubuntu 25.04 · Ackermann steering  
**Hardware:** MS200 LiDAR (`/dev/ldlidar` → ttyUSB0) · STM32 controller (`/dev/rrc` → ttyACM0) · Angstrong HP60C depth camera (`ascamera`)

---

## Option 1 — Docker (Factory Image)

The factory image is stored as the `mentorpi:factory` Docker image. It runs the original Hiwonder Ubuntu 22.04 ROS2 Humble environment.

> With [`mentorpi/install.sh`](mentorpi/README.md) the container, bringup and web UI start at boot via systemd,
> so the manual commands below are only needed for debugging. Health check: `~/mentorpi/status.sh`.

### Prerequisites

```bash
# Verify Docker is running and the image exists
docker ps
docker images | grep mentorpi
```

### Start and launch

```bash
# Start the container (removes any old one, starts fresh)
~/mentorpi/run_container.sh

# Launch ROS2 bringup inside the container
~/mentorpi/start_node.sh
```

`start_node.sh` does:
1. `xhost +` — allow X11 display access
2. Runs `~/.stop_ros.sh` inside the container (kills any previous ROS nodes)
3. Runs `ros2 launch bringup bringup.launch.py` as the `ubuntu` user

### Systemd auto-start (runs on boot)

```bash
# Check status
systemctl status mentorpi-container.service

# Enable/disable auto-start
sudo systemctl enable mentorpi-container.service
sudo systemctl disable mentorpi-container.service

# Restart manually
sudo systemctl restart mentorpi-container.service
```

### Useful Docker shell commands

```bash
# Open a shell inside the container
docker exec -it -u ubuntu MentorPi /bin/zsh

# Inside container: activate ROS2 environment
source ~/.zshrc

# Inside container: stop all ROS nodes
~/.stop_ros.sh

# Inside container: run a specific launch
ros2 launch bringup bringup.launch.py

# Inside container: run joystick only
ros2 launch peripherals joystick_control.launch.py

# Inside container: keyboard teleop
ros2 run peripherals teleop_key_control
```

### Shared folder

Files placed in `~/docker/tmp/` on the host appear at `/home/ubuntu/shared/` inside the container.

---

## Option 2 — Native (Conda ROS2 Humble)

Built from source in `~/ros2_ws/` using a conda environment. No Docker required.

### Prerequisites

The conda environment `ros2_humble` must exist. If rebuilding from scratch:

```bash
~/ros2_ws/build_workspace.sh
```

### Activate environment

```bash
# Activate ROS2 workspace (sets all env vars, sources workspace)
source ~/ros2_ws/setup_env.sh

# Or use the shell alias (available after opening a new terminal)
ros2env
```

After activation, environment variables are set:

| Variable | Value |
|---|---|
| `MACHINE_TYPE` | `MentorPi_Acker` |
| `LIDAR_TYPE` | `MS200` |
| `DEPTH_CAMERA_TYPE` | `ascamera` |
| `ROS_DOMAIN_ID` | `0` |
| `need_compile` | `True` |

### Launch full bringup

```bash
source ~/ros2_ws/setup_env.sh
ros2 launch bringup bringup.launch.py
```

This starts:
- `startup_check` — verifies hardware is reachable
- `controller` — motor driver, odometry, EKF (via `ttyACM0`)
- `lidar` — MS200 LiDAR (via `oradar_lidar` on `ttyUSB0`)
- `depth_camera` — Angstrong HP60C (`ascamera`) — *skipped, not yet built*
- `rosbridge_server` — WebSocket bridge on port 9090
- `web_video_server` — HTTP video stream — *skipped, FFmpeg compat issue*
- `start_app` — application node
- `joystick_control` — gamepad input
- `init_pose` — initializes robot servo positions

### Launch individual components

```bash
source ~/ros2_ws/setup_env.sh

# Motor controller only (STM32 bridge + odometry)
ros2 launch controller controller.launch.py

# LiDAR only (MS200)
ros2 launch peripherals lidar.launch.py

# Joystick only
ros2 launch peripherals joystick_control.launch.py

# Keyboard teleop
ros2 run peripherals teleop_key_control

# Navigation stack
ros2 launch navigation navigation.launch.py
```

### Rebuild workspace

```bash
# Use the alias (set in ~/.bashrc)
buildros

# Or run directly
~/ros2_ws/build_workspace.sh
```

---

## Joystick Controller

### Status

The joystick is **supported in both options** but requires a physical USB/Bluetooth gamepad to be connected. The expected device is `/dev/input/js0`.

```bash
# Check if joystick is detected
ls /dev/input/js*

# Test raw joystick input
cat /dev/input/js0 | xxd | head    # raw bytes
# or
ros2env && ros2 run joy joy_node    # then: ros2 topic echo /joy
```

### Button/Axis mapping

The node expects a **PS4-style** layout (cross/circle/square/triangle + L1/R1/L2/R2 + start/select):

| Control | Function |
|---|---|
| **Left stick Y** | Forward / Reverse (linear.x) |
| **Right stick X** | Steer left / right |
| **Start button** | Buzzer beep (confirmation) |
| L1, R1, L2, R2 | (reserved, no action yet) |
| Cross, Circle, Square, Triangle | (reserved, no action yet) |

Speed limits (configurable via launch args):
- Max linear: `0.5 m/s` (default)
- Max angular: `2.0 rad/s` (default)
- Steering servo: PWM servo ID 3, range ±45°

### Launch with custom speed limits

```bash
ros2 launch peripherals joystick_control.launch.py \
  max_linear:=0.3 \
  max_angular:=1.5
```

### Troubleshooting

**No `/dev/input/js0`:**
- Plug in the gamepad (USB or pair Bluetooth)
- Check `dmesg | tail -20` for device detection
- Bluetooth: `bluetoothctl` → `scan on` → `pair <MAC>` → `connect <MAC>`

**In Docker:** The container runs with `--privileged -v /dev:/dev` so any `/dev/input/js0` on the host is automatically visible inside.

**In native:** The `joy` package is installed in the conda env and will publish to `/joy` topic when a device is connected at `/dev/input/js0`.

---

## Quick Reference

| Task | Command |
|---|---|
| Start Docker option | `~/mentorpi/run_container.sh && ~/mentorpi/start_node.sh` |
| Shell into Docker | `docker exec -it -u ubuntu MentorPi /bin/zsh` |
| Activate native env | `source ~/ros2_ws/setup_env.sh` or `ros2env` |
| Launch native bringup | `ros2env && ros2 launch bringup bringup.launch.py` |
| Keyboard control | `ros2env && ros2 run peripherals teleop_key_control` |
| Joystick control | `ros2env && ros2 launch peripherals joystick_control.launch.py` |
| Rebuild workspace | `buildros` |
| Check ROS2 nodes | `ros2 node list` |
| Check topics | `ros2 topic list` |
| Monitor motor cmds | `ros2 topic echo /controller/cmd_vel` |

## Known Limitations (Native Option 2)

| Package | Status | Reason |
|---|---|---|
| `ascamera` (depth camera) | Not built | Qt6 cross-compilation issue in conda PCL |
| `web_video_server` | Not built | FFmpeg 4.x API removed in conda FFmpeg 7.x |
| `ydlidar_ros2_driver` | Not built | Not needed (robot uses MS200/oradar) |
| `simulations`, `large_models`, `yolov5_ros2` | Not built | Heavy deps, not needed for hardware use |
