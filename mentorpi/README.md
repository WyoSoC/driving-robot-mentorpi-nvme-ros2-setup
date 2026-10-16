# MentorPi M1 — System Guide

Hiwonder MentorPi M1 (Ackermann steering) on a Raspberry Pi 5, Ubuntu 25.04 on NVMe.
The robot software is the Hiwonder factory ROS 2 Humble stack running in a Docker
container (`mentorpi:factory`), plus a browser-based control UI.

Everything needed to run the car, and to copy this setup to another identical car,
lives in this folder. On a car it is reached as `~/mentorpi`: either a symlink to
`mentorpi/` in a clone of this repo, or the folder unpacked from a transfer kit.
Per-car files (`car.env`, `calibration/`) are created by `install.sh` and are not in git.

---

## Status (2026-10-16)

| Area | State |
|---|---|
| Docker factory stack (controller, lidar, IMU, odometry, joystick, rosbridge, video server) | Working |
| Depth camera (Angstrong HP60C): RGB + depth | Working (needed a host udev rule) |
| Web control UI with live video, sensors, driving | Working; tested in browser and phone layout; steering and E-stop verified, **throttle not yet driven on the ground** |
| Auto-start at boot (systemd) | Working via `systemctl`; reboot test pending |
| Transfer kit (`install.sh`, `make_bundle.sh`) | Working; used to set up car 2 (jpi2) |
| Native conda ROS 2 build (`~/ros2_ws`, "Option 2") | **Not working**; see [Native build](#native-conda-build-not-in-use) |

### Fixes made on this car
1. **udev symlinks**: the factory software expects `/dev/rrc` (STM32 controller) and
   `/dev/ldlidar` (MS200 lidar). Without them the motor controller crashes and the lidar
   doesn't connect. Added in `system/99-mentorpi.rules`.
2. **Depth camera access**: the camera (`3482:6723`) failed with `uvc_open: Access denied`
   until the vendor rules were installed on the host (`system/56-angstrong-camera.rules`).
3. **Boot service**: the old `mentorpi-container.service` started bringup before the
   container existed and failed every boot. It's replaced by three ordered services.
4. **Shared memory removed at logout**: the container shares the host `/dev/shm`, and its ROS 2
   nodes run as uid 1000 like the host user. Ubuntu's default `RemoveIPC=yes` deleted that memory
   whenever the user's last login session closed (e.g. SSH logout), after which no new subscriber
   (status.sh, web UI, video) got data. `install.sh` sets `RemoveIPC=no`
   (`system/90-mentorpi-keep-ipc.conf`).
5. **Per-car settings**: `ROS_DOMAIN_ID` and calibration are now outside the image so
   several cars can share a network.

---

## Daily use

The car starts everything on boot. Open the web UI from any device on the network:

```
http://<car-ip>:8000
```

| Task | Command |
|---|---|
| Health check | `~/mentorpi/status.sh` |
| Start / restart everything | `sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui` |
| Stop everything | `sudo systemctl stop mentorpi-container` (stops bringup and web UI too) |
| Restart only the robot software | `sudo systemctl restart mentorpi-bringup` |
| Logs | `journalctl -u mentorpi-bringup -f` / `journalctl -u mentorpi-webui -f` |
| Shell inside the container | `docker exec -it -u ubuntu MentorPi /bin/zsh` |
| Disable auto-start | `sudo systemctl disable mentorpi-container mentorpi-bringup mentorpi-webui` |

Manual start without systemd (in this order):

```bash
~/mentorpi/run_container.sh
~/mentorpi/start_node.sh            # foreground; Ctrl+C to stop
~/mentorpi/web_ui/start_web_ui.sh   # in another terminal, after bringup is up
```

### Web UI

- **Video**: RGB camera or colorized depth (red = near, blue = 4 m+), with an overlay of
  commanded speed, steering, measured speed and front clearance.
- **Driving**: drag the round pad (touch or mouse), or `W A S D` / arrow keys.
  Speed limit slider 0.1–0.7 m/s (default 0.3, remembered per browser).
  **Space** or the red button = E-stop (locks controls until released).
- **Sensors**: lidar radar view (1–8 m), IMU attitude and heading, odometry trail,
  battery voltage (2S pack, % is approximate), per-sensor health lights, event log.
- **Safety**: the page stops the car when you let go of the controls, switch tabs or lose
  the connection. On the robot, `web_ui_server.py` also stops the car if no command arrives
  for 0.5 s while moving (e.g. Wi-Fi drop), and caps speed at 0.7 m/s and steering at 40°.
- **No authentication**: anyone who can reach port 8000/9090 can drive the car.

---

## Layout

```
~/mentorpi/
├── README.md                this guide
├── car.env                  PER-CAR: name, ROS_DOMAIN_ID, web port (not transferred)
├── car.env.example          template for new cars
├── calibration/             PER-CAR: imu_calib.yaml, calibrate_params.yaml (not transferred)
├── calibration.defaults/    factory values from the image, used for a new car
├── run_container.sh         starts the container, applies car.env, mounts calibration + web_ui
├── start_node.sh            runs the ROS 2 bringup in the container (foreground)
├── status.sh                health check
├── install.sh               installs/updates everything on a car (idempotent)
├── make_bundle.sh           packages kit + Docker image for transfer
├── system/                  udev rules + systemd unit templates
├── web_ui/
│   ├── index.html           the control page (no external dependencies)
│   ├── web_ui_server.py     serves the page, drive watchdog, depth colorizer
│   └── start_web_ui.sh
```

Boot chain (all `Type`/ordering handled by systemd):

```
docker.service
  └─ mentorpi-container.service   oneshot: run_container.sh
       └─ mentorpi-bringup.service   start_node.sh (waits for /dev/rrc, /dev/ldlidar)
            └─ mentorpi-webui.service   waits for rosbridge :9090, then web_ui_server.py
```

---

## Multiple cars on one network

ROS 2 cars on the same network and the same `ROS_DOMAIN_ID` **see each other's topics**:
one car's web UI or joystick would also drive the other car. Every car must have a
unique `ROS_DOMAIN_ID` in `car.env`. Convention: the car is named after its hostname and
its `ROS_DOMAIN_ID` is the number in the hostname (`install.sh` does this by default).

| Car (hostname) | ROS_DOMAIN_ID | Notes |
|---|---|---|
| jpi1 | 1 | first car; source of the transfer bundle |
| jpi2 | 2 | installed 2026-10-16; has an extra CH340 voice module (ignored by the udev rules) |

To change it later: edit `car.env`, then `sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui`.
If you use ROS 2 tools from a laptop, set the same `ROS_DOMAIN_ID` there.

---

## Calibration (per car)

`calibration/` is bind-mounted into the container over the image's files, so values
written by the calibration tools persist on the host and stay with this car.

- `imu_calib.yaml`: accelerometer scale/misalignment (`SM`) and `bias`.
  The defaults came from the factory image of car 1. On a new car, either copy the
  `imu_calib.yaml` from that car's own Hiwonder SD card
  (`/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml`), or recalibrate:
  ```bash
  docker exec -it -u ubuntu MentorPi /bin/zsh
  ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw \
      -p calib_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml
  # follow the prompts (place the car on each side), then restart mentorpi-bringup
  ```
- `calibrate_params.yaml`: `linear_correction_factor` / `angular_correction_factor` for
  odometry (factory 1.0 / 1.0). Tune with `ros2 launch calibration linear_calib.launch.py`
  and `angular_calib.launch.py` (Hiwonder tools), then copy the values into this file.

---

## Transfer to another identical car

### What you need
- The target car: Raspberry Pi 5 MentorPi M1 (Ackermann), with the same USB devices
  (STM32 controller `1a86:55d4`, MS200 lidar `1a86:7523`, HP60C camera `3482:6723`).
- Ubuntu 25.04 arm64 installed on the target (NVMe or SD), a normal user with sudo,
  and network access (for `apt install docker.io` if Docker isn't installed).
- About 30 GB free disk space on the target (image archive + loaded image).

Two ways to get the files onto the new car:
- **git** (code only): clone this repo and `ln -s <clone>/mentorpi ~/mentorpi`. You still need the
  Docker image from step 1 (`mentorpi-factory.tar.zst`); it is not on a public registry.
- **transfer kit** (code + image): steps 1–3 below.

### 1. On this car: build the bundle
```bash
~/mentorpi/make_bundle.sh            # writes ~/mentorpi_transfer/
```
Contents: `mentorpi-kit.tar.gz` (small), `mentorpi-factory.tar.zst` (Docker image, several GB)
with a `.sha256` checksum, and this README. Re-run after making changes. The image is only
re-exported if it changed.

### 2. Copy to the new car
USB drive, or over the network:
```bash
rsync -aP ~/mentorpi_transfer/ <user>@<new-car-ip>:mentorpi_transfer/
```

### 3. On the new car: install
```bash
tar -C ~ -xzf ~/mentorpi_transfer/mentorpi-kit.tar.gz
~/mentorpi/install.sh --image ~/mentorpi_transfer/mentorpi-factory.tar.zst
```
Name and `ROS_DOMAIN_ID` default to the hostname and its number (jpi2 → 2). Override with
`--name` / `--domain-id` if the hostname has no number.
The installer:
- installs Docker if needed. The first time, it adds you to the `docker` group and asks
  you to log out and back in, then run the same command again.
- verifies the checksum and loads the image.
- creates `car.env` and copies the default calibration.
- installs the udev rules and checks that the three USB devices are present.
- installs and enables the three boot services.

### 4. Start and verify
```bash
sudo systemctl start mentorpi-webui
~/mentorpi/status.sh
```
Expect: three services `active`, `/dev/rrc` and `/dev/ldlidar` present, `/odom` ~28 Hz,
`/imu` ~48 Hz, `/scan_raw` ~10 Hz, camera ~10 Hz, ports 9090/8080/8000 listening.
Then open `http://<new-car-ip>:8000`.

### 5. After transfer
- [ ] IMU calibration for this car (see [Calibration](#calibration-per-car))
- [ ] Confirm `ROS_DOMAIN_ID` differs from every other car (in the table above)
- [ ] First throttle test with the **wheels off the ground**, then on the floor at 0.1–0.3 m/s
- [ ] Reboot once and check `status.sh` (verifies auto-start)

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ros_robot_controller` dies: `No such file or directory: '/dev/rrc'` | udev rule missing or controller unplugged. `ls -l /dev/rrc`; re-run `install.sh` |
| Lidar: no `/scan_raw` | `/dev/ldlidar` missing (same as above). The topic is `/scan_raw`, not `/scan` |
| Camera: `uvc_open: Access denied` | `56-angstrong-camera.rules` missing; re-run `install.sh`, replug the camera |
| Camera not on USB (`lsusb -d 3482:6723` empty) | Cable/port; it works on USB 2 or 3 ports |
| Web UI loads but "Disconnected" | rosbridge (9090) not up: `journalctl -u mentorpi-bringup` |
| Video "unavailable" | web_video_server (8080) not up, or camera not streaming |
| Car stops by itself while driving from web | Watchdog: browser commands stopped >0.5 s (weak Wi-Fi). Server log: `journalctl -u mentorpi-webui` |
| Nodes all listed but `status.sh` shows NO DATA, web UI shows no video/sensors (often after an SSH logout) | `/dev/shm` was cleared by logind (`ls /dev/shm \| wc -l` is ~0). Re-run `install.sh` (sets `RemoveIPC=no`), then `sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui` |
| Two cars react to the same command | Same `ROS_DOMAIN_ID`; give each car its own |
| `install.sh` stops after adding docker group | Expected the first time; log out/in, run again |

Device IDs: STM32 controller `1a86:55d4` → `/dev/rrc` (ttyACM0, 1 Mbaud);
MS200 lidar `1a86:7523` → `/dev/ldlidar` (ttyUSB0, 230400 baud); HP60C camera `3482:6723`;
gamepad `2563:0526` (ShanWan).

---

## Technical reference

| Port | Service |
|---|---|
| 8000 | Web UI (`web_ui_server.py`) |
| 9090 | rosbridge WebSocket |
| 8080 | web_video_server (MJPEG) |

Driving: `web_ui_server.py` relays `/web/cmd_vel` → `/app/cmd_vel` (`geometry_msgs/Twist`):
`linear.x` = speed m/s, `angular.z` = **front-wheel steering angle in rad** (+ = left).
`odom_publisher_node` converts this to motor speeds and steering servo (PWM servo 3, ±40°;
wheelbase 0.145 m). With `linear.x = 0` it only turns the wheels. The controller itself
has no command timeout; the web server's watchdog provides it.

Main topics: `/odom` (nav_msgs/Odometry), `/imu`, `/imu/rpy/filtered`, `/scan_raw`
(LaserScan, 0.05–12 m), `/ascamera/camera_publisher/rgb0/image`,
`/ascamera/camera_publisher/depth0/image_raw` (16UC1 mm), `/web/depth_color`,
`/ros_robot_controller/battery` (UInt16 mV), `/joy`.

---

## Native conda build (not in use)

A from-source build in `~/ros2_ws` with a conda ROS 2 Humble env (`ros2env`, `buildros`).
26 packages build, but bringup does not run. Changes made on 2026-10-15:
- added `src/mentorpi_description` (copied from the image) and installed
  `ros-humble-xacro`, `ros-humble-joint-state-publisher` in the env
- `bringup.launch.py`: depth camera and web_video_server are launched only if installed
- added the missing `get_package_share_directory` import to 7 launch files
- restored pip `numpy==2.4.6` after a conda install broke it (avoid conda installs that touch numpy)

Remaining blockers: ROS libraries built for numpy 1.x vs numpy 2.x required by OpenCV 5 /
mediapipe (`numpy._core.multiarray failed to import`); `oradar_scan` and `apply_calib`
can't find shared libraries (exit 127); `imu_complementary_filter` not installed;
`ascamera` and `web_video_server` not built. The Docker stack is the supported path and
the native build is not part of the transfer kit. `~/ros2_ws/LAUNCH_GUIDE.md` describes
both options as they were in August.
