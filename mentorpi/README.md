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
5. **rosbridge memory leak**: a browser that disconnects while its subscriptions are being set up
   (closed or reloaded within a second or two) left rosbridge sending to the dead connection; on car 1
   it grew to 6.6 GB and was killed, disconnecting the web UI. Patched at container start
   (`system/patch_rosbridge.py`), plus a once-a-minute watchdog (`mentorpi-watchdog.timer`) that
   restarts the robot software if rosbridge is gone or above 1 GB.
6. **Per-car settings**: `ROS_DOMAIN_ID` and calibration are now outside the image so
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
├── car.env                  PER-CAR: name, ROS_DOMAIN_ID, steering port, web port (not transferred)
├── car.env.example          template for new cars
├── calibration/             PER-CAR: imu_calib.yaml, calibrate_params.yaml (not transferred)
├── calibration.defaults/    factory values from the image, used for a new car
├── run_container.sh         starts the container, applies car.env, mounts calibration + web_ui
├── start_node.sh            runs the ROS 2 bringup in the container (foreground)
├── status.sh                health check
├── calibrate_imu.sh         guided IMU calibration (tools/imu_calib_guided.py)
├── calibrate_odom.sh        odometry calibration with lidar + IMU (tools/odom_calib.py)
├── fleet.sh                 update / check all cars (car list in fleet.env, not in git)
├── maps/                    PER-CAR: SLAM maps saved from the web UI (not transferred)
├── tools/steer_test.sh      steering check (front wheels off the ground)
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

## Keeping all cars in sync

All cars run the same commit of this repo; per-car files (`car.env`, `calibration/`, `maps/`) stay
on each car. Make changes in one clone, commit + push, then from any car:

```bash
~/mentorpi/fleet.sh update   # git pull + install.sh + restart on every car, then status
~/mentorpi/fleet.sh status   # version, uncommitted changes, services and sensor rates per car
```

The car list is in `~/mentorpi/fleet.env` (not in git), e.g.
`FLEET="gojian@jpi1.<tailnet>.ts.net gojian@jpi2.<tailnet>.ts.net"`. Needs ssh keys between the
cars and passwordless sudo.

## Multiple cars on one network

ROS 2 cars on the same network and the same `ROS_DOMAIN_ID` **see each other's topics**:
one car's web UI or joystick would also drive the other car. Every car must have a
unique `ROS_DOMAIN_ID` in `car.env`. Convention: the car is named after its hostname and
its `ROS_DOMAIN_ID` is the number in the hostname (`install.sh` does this by default).

| Car (hostname) | ROS_DOMAIN_ID | Notes |
|---|---|---|
| jpi1 | 1 | first car; source of the transfer bundle; steering port 4 (standard; to be verified) |
| jpi2 | 2 | installed 2026-10-16; extra CH340 voice module (ignored); steering port 4 (verified) |

To change it later: edit `car.env`, then `sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui`.
If you use ROS 2 tools from a laptop, set the same `ROS_DOMAIN_ID` there.

---

## Calibration (per car)

`calibration/` holds this car's values. `run_container.sh` mounts the files over the copies the
nodes actually read (`~/ros2_ws/install/.../share/...`), and also over the `src/` copies used
by Hiwonder's tools. Before 2026-10-16 only the `src/` copies were mounted, so per-car values
had no effect.

### IMU (accelerometer): `imu_calib.yaml`

Scale/misalignment matrix `SM` and `bias`, applied by `apply_calib` to
`/ros_robot_controller/imu_raw` → `/imu_corrected`. A new car starts with car 1's factory values.
Either copy `imu_calib.yaml` from that car's own Hiwonder SD card
(`/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml`), or calibrate:

```bash
~/mentorpi/calibrate_imu.sh      # on the car, in a terminal (ssh is fine), system running
```

You place the car in 6 poses, about 11 s each. The script shows which way is "up" right now and
starts recording only when the car is in the requested pose and still. Axes of the MentorPi M1
controller board: **X+ = right side, Y+ = front, Z+ = up**.

| Step | Axis up | Car pose |
|---|---|---|
| 1 | X+ | lying on its LEFT side (right side facing up) |
| 2 | X- | lying on its RIGHT side |
| 3 | Y+ | standing on its REAR end, front pointing up |
| 4 | Y- | standing on its FRONT end |
| 5 | Z+ | normal, on its wheels |
| 6 | Z- | upside down; support the body so the lidar/camera carry no weight |

A few degrees of tilt is fine (a car body never sits perfectly square on its side). If the car
moves while recording, or the results look wrong (scale outside 0.9–1.1, bias above 1.5 m/s²),
nothing is changed. On success the old file is kept as `calibration/imu_calib.yaml.bak-<date>`
and bringup is restarted; at rest the corrected reading (`/imu_corrected`) should have a
magnitude of about 9.81. To undo: copy the `.bak` file back and `sudo systemctl restart mentorpi-bringup`.

Method: per axis, scale = g / ((up − down) / 2) and offset = (up + down) / 2, written as a
diagonal `SM` plus `bias` (`apply_calib` computes `SM × raw − bias`). Hiwonder's `imu_calib do_calib`
is **not** used: its 12-parameter fit assumes perfectly square poses and turned the few degrees
of tilt into false cross-axis terms. On car 2 it gave a 2.7% error at rest, against 0.04% for the
per-axis model computed from the same poses. (If you run `do_calib` by hand, its parameter is
`output_file`, not `calib_file`.)

### Steering servo port: `STEER_SERVO_PORT` in `car.env`

The software drives the steering servo as PWM servo 3, but which RRC Lite port it is plugged
can differ. Hiwonder's docs put it on J5 (port 3); **our standard, and the default for all
cars, is port 4** (how car 2 came wired). `run_container.sh` patches `ros_robot_controller` at every start
(`system/patch_controller.py`) to send servo 3 to `STEER_SERVO_PORT` (swapping 3 and 4).
Unset means 4. Check with the front wheels off the ground:

```bash
~/mentorpi/tools/steer_test.sh         # full left 2 s, full right 2 s, centre
```

If the wheels don't move, try the other port in `car.env`, then
`sudo systemctl restart mentorpi-container && sudo systemctl start mentorpi-webui`.
The same patch fixes a factory bug: calling the `pwm_servo/get_state` service crashed the
whole controller node (no IMU, battery or motors until bringup restarted).

### Odometry: `calibrate_params.yaml`

How odometry works here: there is **no wheel-encoder feedback**. `odom_publisher` integrates the
*commanded* speed and steering into `/odom_raw`; the EKF (`ekf_filter_node`) fuses its velocities
with the IMU heading into `/odom` (the web UI trail). So distances follow the command (wrong if the
wheels slip or the car is slower than asked), turns follow the gyro.

`motor_speed_map` ([real, sent, ...] m/s, measured by `speedmap`) corrects the motor commands so a
commanded speed is the real speed and speed limits mean what they say (car 2: real = 0.89-0.98 x
sent). The motors (JGA27-310R, 20:1, 450 RPM) have encoders and the board runs a speed loop; car 1
runs away above ~0.12 m/s sent (suspected encoder wiring), so its map only covers slow speeds.
Fallback without a map: `motor_speed_factor` (real / commanded). Steering centre: the board's PWM
servo offset, per car in `calibration/servo_config.yaml` (key 3 = steering, us, -100..100), loaded
at controller start; `steer` measures the heading drift per metre with the IMU and sets it. `steer_gain`
(measured by `angular`) strengthens the steering servo command so a commanded turn rate is the real
one (car 2 steered ~25% less than commanded: gain 1.35); `angular_correction_factor` then stays 1.0. `angular_correction_factor` scales the odometry turn rate
(`linear_correction_factor` stays 1.0 once the motor factor is set). In the factory code the
correction factors had no effect on `/odom`; `system/patch_odom.py` fixes both at container start.
Measure them with the lidar and IMU as references, **linear first; the car drives at 0.2 m/s, on
the floor only (never on a table: the lidar can't see the edge)**:

```bash
~/mentorpi/calibrate_odom.sh check     # no driving: is the car placed right?
~/mentorpi/calibrate_odom.sh speedmap  # facing a flat wall 1.5-4 m ahead, 1.3 m free behind -> motor_speed_map
~/mentorpi/calibrate_odom.sh steer     # same placement: steering centre trim (calibration/servo_config.yaml)
~/mentorpi/calibrate_odom.sh linear    # check: with a speed map the ratio should be ~1.00
~/mentorpi/calibrate_odom.sh angular   # ~1.5 m clear left/right, 1 m ahead/behind: two circles -> steer_gain
```

Each asks for `go` before driving and before saving, keeps a `.bak-<date>` copy and restarts
bringup.

### Map (SLAM)

The web UI's **Map** panel runs lidar SLAM (`slam_toolbox`, Hiwonder's `slam/config/slam.yaml`)
next to the running system: **Start mapping**, drive slowly (≤ 0.3 m/s) and pass your start point
again now and then so loops close, then **Save map**. Maps are saved per car in
`~/mentorpi/maps/` (`NAME.pgm/.yaml` for navigation, `NAME.posegraph/.data` to continue mapping)
and are not transferred to other cars. The page shows the map rendered on the car (`/api/slam`).

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
| Steering doesn't move at all (drive motors work) | Servo on another port: see [Steering servo port](#steering-servo-port-steer_servo_port-in-carenv) |
| Web UI "Disconnected", port 9090 not listening; kernel log shows `Out of memory: Killed process ... (python3)` | rosbridge leaked memory after a browser dropped mid-connect or stopped reading (fixed by `system/patch_rosbridge.py`: cleanup + max 20 queued messages per client). `mentorpi-watchdog.timer` restarts the robot software automatically if it happens again: `journalctl -u mentorpi-watchdog` |
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
