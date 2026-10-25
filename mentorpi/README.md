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
- **Battery warnings**: the page shows "low" below 6.9 V and "critical" below 6.6 V. The board's own
  (loud) beeper only sounds below 6.6 V (factory: 6.9 V, which beeps at ~35% under load); set
  `BATTERY_ALARM_MV=...` in `car.env` to change it per car (6000-7400), then restart the container.
- **Charging mode** (button next to the battery): stops camera, lidar, mapping and driving so more of
  the charger's ~2 A goes into the battery (~1.5x faster than charging with everything running;
  switched off is fastest). The page and battery reading stay on; "Resume normal mode" restarts the
  robot software (~30 s). Manual only: a reboot or container restart always comes up normal.
  From a shell: `sudo ~/mentorpi/system/charging_mode.sh on|off|status`.
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
├── tests/run.sh             tests run in the container, nothing drives: test_fleet.py (cars avoiding
│                            each other), test_merge.py (map merging), test_localize.py (finding the car on a map),
│                            test_depth_sim.py (depth threat levels)
├── install.sh               installs/updates everything on a car (idempotent)
├── make_bundle.sh           packages kit + Docker image for transfer
├── system/                  udev rules + systemd unit templates
├── web_ui/
│   ├── index.html           the control page (no external dependencies)
│   ├── web_ui_server.py     serves the page, drive watchdog, depth colorizer
│   ├── peer_manager.py      status exchange with the other cars (fleet.env)
│   ├── fleet.py             cars avoid each other: costmap marks + right of way
│   ├── map_merge.py         line up two maps of the same place and merge them
│   ├── team.py              mapping together: live maps lined up, frontiers split
│   ├── shared_map.py        the shared map: keyframes exchanged, same map on every car
│   ├── localize.py          find the car on a saved map from one lidar scan
│   ├── car_geometry.py      the car's measured shape and sensor positions (used everywhere)
│   ├── scan_filter.py       /scan_nav: lidar without readings inside the car's rectangle
│   ├── run_recorder.py      records every autonomous mapping run (timeline, Nav2 log, bag)
│   ├── explore_manager.py   autonomous mapping (frontier exploration)
│   ├── depth_obstacles.py   depth camera obstacles (/depth/scan) + comparison with the lidar
│   ├── map_store.py         saved maps + content hash (same map on two cars?)
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

### Cars know about each other (status exchange)

Separate ROS domains keep the cars' topics apart, so they share status through their web servers
instead (`web_ui/peer_manager.py`). Each web server asks every other car in `fleet.env` for
`GET /api/self` twice a second (every 3 s while one is unreachable). This keeps running in charging
mode, because the web server always stays on. The other car's status contains its battery, mode
(idle / mapping / navigating / charging), speed, navigation map, position, goal and planned path,
and its list of saved maps.

On the page:

- **Header:** a badge per other car with its battery and mode. Click it to open that car's page.
- **Map panel, Navigate tab:**
  - The map list shows the maps on this car and on the other cars.
  - Picking a map previews it.
  - Starting navigation on another car's map first copies it here. If a different map already
    has that name here, the copy is saved as `NAME_car`.
  - Other cars that navigate on the **same map** appear in blue, with their position, planned path
    and goal. "Same map" is decided by a hash of the map's contents, not its name.

Positions only line up when both cars localise on the same map, so copy one map to both cars
rather than mapping the room twice. `PEERS="host:port …"` in `car.env` overrides the list from
`fleet.env` (`PEERS=` turns the exchange off). No ROS changes are involved, and nothing a car
receives here drives it, except that a car waits for another one in its way (next section).

### Cars avoid each other

When both cars navigate on the **same map** and both know where they are (`web_ui/fleet.py`):

- **Costmap:** the other car's body is drawn into this car's Nav2 costmaps (`fleet_layer`, fed from
  the status exchange, 5 times a second). The planner routes around it even where the lidar can't
  see it, such as around a corner. The body is grown by how far the other car can have moved since
  its status was sent (0.2 m/s × age). An empty 360° scan wipes the old marks each cycle, so a car
  that has moved on leaves no ghost behind.
- **Right of way:** this car pauses its goal when the next 1.2 m of its planned path runs within 0.3 m
  of another car that is **driving to a goal**. The motors stop, the page shows "waiting for jpi2 to
  pass", and the header badge on the other car's page shows the same. The goal is taken back from Nav2
  while waiting, so Nav2 doesn't count the wait as being stuck. It is sent again once the way has
  been clear for 2 s.
  - If both cars block each other (head-on), the car whose name sorts first (jpi1) keeps going.
  - A parked, paused or idle car is an ordinary obstacle: the planner goes around it.
  - After 30 s the waiting car goes on anyway: its planner sees the other car and replans, or reports
    no path.
- E-STOP, Cancel goal or touching the joystick also cancels a waiting goal; it never resumes by itself
  after that.

Drive-tested with both cars on one saved map (both found their own position from the lidar):
- **Crossing**, cars parked side by side 40 cm apart against a wall: both crept 15 cm out of the tight
  start; jpi2 waited ("waiting for jpi1 to pass"), then went on and arrived. jpi1 gave up then (the other
  car's mark, grown all round, reached its nose; fixed: a driving car's mark is only lengthened).
- **Head-on**, 1.2 m apart, driving towards each other's places: jpi1 waited while jpi2 turned across
  its path, went 2 s after it was clear; both arrived in 25 s.

Tested without driving (`tests/run.sh test_fleet.py`):
- A real Nav2 costmap node received fake car positions. A parked car is marked, and its marks are
  gone after it moves.
- A crossing car makes this car wait, and it resumes 2 s after the other has passed.
- Head-on, jpi1 goes and jpi2 waits.
- An offline car leaves nothing behind.

A supervised drive with both cars is still to come.

Not covered yet:
- Two cars in a narrow corridor head-on: the waiting car doesn't back out of the way.
- During exploration each car has its own map, so none of this applies until the maps are lined up
  (next section).

### Merging maps (Map panel → Merge)

Two maps of the same place, made by either car or in different runs, each have their own coordinate
frame. `web_ui/map_merge.py` lines them up by their walls:
1. A coarse search tries every 2° of rotation, with the best shift for each found by FFT
   correlation at 10 cm.
2. The best candidates are then refined to 2.5 cm and 0.25°.

The match is judged both ways:
- **Agree:** the share of walls that land on the other map's walls.
- **Clash:** the share that land on its open floor.

It merges only when all of these hold:
- agree is ≥ 80% both ways;
- clash is ≤ 12%;
- the two maps share at least 1.5 m² of floor;
- the agreeing walls extend in two directions.

On the page: pick two maps, **Line up**, check the overlay (blue: first map, red: second, purple: both),
then **Save merged**. The merged map keeps the first map's position and direction, and syncs to the
other cars like any map.

Measured on the 5 maps of the living room made by both cars:
- All 10 pairs matched (agree 85–100%, clash ≤ 9%), about 3–4 s each on the Pi.
- Chaining A→B→C landed within 7 cm and 1° of A→C directly, so the matches agree with each other.
- Wrong pairs were all rejected (agree 54–72%, clash 9–35%): a map against its own mirror image, and
  against a made-up room.

Test: `tests/run.sh test_merge.py`. API: `POST /api/maps/align?a=&b=`, `GET /api/maps/merge.png?a=&b=`, `POST /api/maps/merge?a=&b=&name=`.

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

How odometry works here: the motors have encoders, but only the board uses them (speed loop);
ROS gets no wheel feedback. `odom_publisher` integrates the *commanded* speed and steering into
`/odom_raw` (made accurate by the speed map and steering calibration below). The EKF
(`ekf_filter_node`) takes the speed from `/odom_raw` and the **turn rate from the gyro**
(`/imu_corrected`, bias removed at start-up); its output `/odom` is the web UI trail and SLAM's
odometry. `system/patch_ekf.py` sets this up: the factory config used the heading of
`imu_complementary_filter` (`/imu`), which swallows slow turns (it re-estimates the gyro bias
whenever turning < 11 deg/s) and came out mirrored in the EKF (car 1 turned 90 deg left by hand:
gyro +89, /imu +41, /odom -41). The heading is the gyro alone (`/odom_raw`'s turn rate lost
heading at every stop: car 2, 23.8 m loop, EKF +27 deg vs gyro +9.5); parked it drifts ~1-2 deg/min,
which SLAM corrects.

`motor_speed_map` ([real, sent, ...] m/s, measured by `speedmap`) corrects the motor commands so a
commanded speed is the real speed and speed limits mean what they say (both cars: real = 0.89-0.98 x
sent, steady up to 0.2 m/s). The motors (JGA27-310R, 20:1, 450 RPM) have encoders and the board runs
a speed loop. (An apparent "runaway" was a lidar artefact: the narrow sector ahead saw past the wall
when the heading drifted; the tool now drops impossible jumps.)
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

The web UI's **Map** panel runs lidar SLAM (`slam_toolbox` with Hiwonder's `slam/config/slam.yaml`,
started by `nav/slam.launch.py`) next to the running system. That launch file overrides two settings:
the map is rebuilt and published every 2 s instead of every 0.2 s, which saved CPU for Nav2 while
exploring, and RViz's interactive mode is off. Its tabs:

- **Mapping:**
  - **Start mapping** only builds the map: the car does not move by itself. Drive it with the joystick,
    slowly (≤ 0.3 m/s), and pass your start point again now and then so loops close.
  - **Save map** saves it under a name.
  - **🧭 Autonomous mapping** lets the car drive and map by itself (below), alone or **together** with
    the other cars.
- **Navigate:** drive to a goal on a saved map (Navigation, below).
- **Merge:** line up two maps of the same place and save them as one.
- **Maps:** the saved maps on this car, with size and date and a preview. **Rename** or **Delete**
  one. Maps are synced between the cars: renaming or deleting does the same on the other online cars,
  found there by content. A deleted map is listed in this car's status, so the sync never copies it
  back, and a car that was offline deletes its copy when it comes back. A map that navigation is
  using can't be renamed or deleted. API: `POST /api/maps/rename?name=&to=`,
  `POST /api/maps/delete?name=`. Test: `tests/run.sh test_maps.py`.

Maps are saved in `~/mentorpi/maps/`: `NAME.png/.yaml` for navigation, plus `NAME.posegraph/.data`
(slam_toolbox's pose graph) when saved while mapping. Any map one car has and another lacks is copied
over within 30 s (`peer_manager.py`).

---

## Transfer to another identical car


### Car shape and sensor positions (`web_ui/car_geometry.py`)

Measured on the car, from the centre of the lidar: the body is a rectangle reaching 12 cm forward, 10 cm
back and 9 cm to each side. The top of the lidar is 16 cm above the floor. The front of the depth camera
is 3 cm behind the front of the car, and its centre is 11 cm above the floor. In base_footprint (the
lidar is 1 cm behind its origin) that is a 0.22 × 0.18 m body from x −0.11 to 0.11, with the camera at
x 0.08. Used for:
- Nav2's footprints and collision-monitor zones in `nav/nav2_params.yaml`, written out there and checked
  by `tests/run.sh test_geometry.py`;
- the camera position in `depth_obstacles.py`. The robot description had the camera 1.6 cm further back
  and 1.1 cm higher; its direction still comes from TF;
- the other car's body in `fleet.py`;
- the bumper distances for the creep-out and escape moves (`nav_manager.py`, `explore_manager.py`).
- the lidar readings Nav2 ignores. `web_ui/scan_filter.py` republishes `/scan_raw` as `/scan_nav`, with
  only the readings inside the car's rectangle (cables, a curtain touching the car) set to invalid. Nav2's
  obstacle layers and the collision monitor use `/scan_nav` with no minimum range. Before, a 16 cm radius
  around the lidar was ignored: anything 4 cm in front of the bumper or 7 cm beside the car never
  reached the obstacle map. No padding is added to the footprint either.

### Navigation (Nav2)

The Map panel's **Navigate** mode drives the car to a point on a saved map:
1. Pick a map (saved in Mapping mode on this car), press **Start navigation** (~20 s).
2. The car **finds its position by itself** once Nav2 is up: put it down anywhere on the mapped floor,
   facing any way. `web_ui/localize.py` matches one lidar scan against the whole map (every free spot
   and heading, then refined; ~2 s) and sets the result in AMCL. It only does so when it is sure: most
   of the scan lies on walls of the map, few rays would pass through walls, and no clearly different
   place fits almost as well (a symmetric room). Otherwise the best guess is drawn dashed and the
   message says why. **🔎 Find position** runs it again (after moving the car by hand);
   **📍 Set position** sets it by hand: click where the car is, drag the way it faces, release.
   People, the other car or moved furniture don't stop it; a car in an unmapped spot gets "not sure".
3. **🎯 Go to**: click the destination, drag the way the car should arrive, release. The green line is
   the plan; the red arrow is the car (lidar localization, AMCL).
4. **Cancel goal**, the **E-STOP**, or touching the joystick stops it.
5. **Tight start:** a car parked against a wall or right next to the other car can stand, as far as Nav2
   can tell, inside an obstacle; Nav2 then finds no path at all. If a goal fails before the car has
   moved, it creeps 15 cm straight ahead (or back, whichever side the lidar shows ≥ 25 cm of room past
   the bumper) at 6 cm/s and tries the goal once more ("tight start: creeping …").

Set up for this car (`nav/nav2_params.yaml`, started by `web_ui/nav_manager.py` via `nav/nav.launch.py`):
Test: `tests/run.sh test_localize.py` (simulated scans from random spots on every saved map, with noise,
missing points and a car-sized box that is not on the map: 60 of 60 found, none wrong). API:
`POST /api/nav/find`.

Hybrid-A* planner (only paths a car can drive: 0.40 m minimum turning radius, reverses when it must),
Regulated Pure Pursuit (0.18 m/s, slower in tight curves and near obstacles), recoveries without
turning on the spot (`nav/navigate_ackermann.xml`: clear costmaps, wait, back up 15 cm), a velocity
smoother and a lidar collision monitor (stops for anything within a few cm of the body, slows so it
never reaches an obstacle within 1.2 s). The web server is the gate: Nav2's output reaches `/cmd_vel`
only while a goal is active, and `/cmd_vel` itself is clamped to 0.2 m/s and 0.5 rad/s by
`odom_publisher`. Mapping and navigation can only run together while exploring (below).

### Depth camera obstacles (lidar + camera)

The lidar scans a single plane 16 cm above the floor, so it can't see lower things such as shoes,
cables or toys. `web_ui/depth_obstacles.py` turns every depth image (about 12 per second) into
obstacles:

1. **Points:** every 4th pixel becomes a 3-D point in the car's frame, 5 images per second.
2. **Floor calibration:** where the camera sees floor, it fits a plane to it, and heights are measured
   from that plane. That corrects camera tilt and height errors (a 1° tilt fits as 1.08°). Carpet and
   matte floors return points; glossy floors return none.

   It only accepts flat 5 cm patches, spread over at least 7 of 10 sectors across the view, so a rug
   strip or the bottoms of objects aren't taken for the floor. With no floor visible, heights come from
   the nominal camera mounting.
3. **Threat level:** the camera builds a height grid of the area in front, one cell per direction and per
   5 cm of distance. It then walks outward along each direction:

   | Class | Rule | Examples | Navigation |
   |---|---|---|---|
   | drivable | up to `CLIMB_H` (2 cm) | rugs, carpet edges, floor | ignored |
   | bump | up to `BUMP_H` (5 cm) **and** no step over `STEP_H` (2.5 cm) within 5 cm | door thresholds, cables, gentle humps | ignored (drivable), shown in yellow |
   | obstacle | taller than 5 cm, or a steeper step, up to 22 cm (the car's height) | shoes, books, boxes, overhangs | `/depth/scan`: avoided and stopped for |

   Thresholds grow by 6 mm per metre of distance, because depth noise grows with distance. The values
   are at the top of `web_ui/depth_obstacles.py`; adjust them to what the car can actually climb.
4. **Scans:** the nearest obstacle and the nearest bump in each 0.5° direction become `/depth/scan` and
   `/depth/bumps`, covering the camera's 58° view. A direction only counts when it's seen in 2 of the
   last 3 images.

Simulated scenes, with car 1's lens and mounting, a carpet or glossy floor, and the camera tilted 0° or
1°, all classify correctly: a 1.5 cm rug and open floor as drivable, a 3 cm threshold and a gentle
4 cm hump as bumps, and a 4 cm book (vertical edge) and a 10 cm shoe as obstacles. Processing takes
about 9 ms per image.

**How navigation uses both sensors:** the camera has its own layer in both obstacle maps. A shared
layer wouldn't work, because the lidar scans over a shoe and would erase the camera's mark. The
collision monitor watches the lidar and the camera, so the car also stops for low objects in front.

**Comparing the two sensors:** the Lidar + camera panel draws camera obstacles in purple, with a
Lidar / Camera / both selector. Below it is the comparison inside the camera's view, now and since
start:

- **agree:** both sensors see the same obstacle, within 15 cm.
- **camera only:** low things under the lidar plane.
- **lidar only:** things the camera misses: shiny, dark or thin objects, or things too close or
  too far.

The same comparison is available from `GET /api/depth`.

Points well below the floor (possible drops) are counted but not used for driving. The camera gets
no readings from glossy floors at all, so it can't detect drops reliably yet.

### Exploration (the car maps by itself)

Map panel, **Mapping** mode: pick a distance limit (3, 5 or 8 m, or none) and press **🧭 Autonomous mapping**.
The car then works like this (`web_ui/explore_manager.py`):

1. It starts mapping (if mapping isn't already running) and navigation on the live map
   (`nav.launch.py slam:=true`, about 30 s).
2. It drives to the best "frontier", the edge between mapped floor and unmapped space. On the map,
   frontiers are orange, the next goal is green and the start is a square.
3. It repeats until no frontier is left within the limit, then drives back to the start and saves
   the map as `explore_YYYYMMDD_HHMM`. Mapping keeps running, so you can also drive further by hand.

How goals are chosen and checked:

- Goals are only placed on mapped free floor: ≥ 0.30 m from walls, ≥ 0.15 m from unmapped space,
  and ≥ 0.7 m from the car.
- The planner never plans through unmapped space.
- A goal that fails, or takes more than 2 minutes, is blacklisted. After 6 failures in a row it gives up.

**Stop autonomous mapping**, the E-STOP, the joystick, Cancel, charging mode, low battery or the 20-minute
limit each stop it. Navigation then stops and mapping keeps running, so the map can be saved.

**Drops (stairs, ledges) are not detected.** The lidar scans a flat plane 12–15 cm high and sees over
a stair edge, so the floor past it looks free. The depth camera looks straight ahead, and on a glossy
floor it gets no return from the floor at all. Block stairs, or keep the distance limit inside the
safe area.

#### Run recordings (`web_ui/run_recorder.py`)

Every autonomous mapping run is recorded, from the page, from another car, or a dry run, to find out
afterwards why the car stopped, backed up or got stuck. One folder per run in `~/docker/tmp/runs/` on
the car (`/home/ubuntu/shared/runs` in the container): `YYYYMMDD_HHMMSS_<car>/` with

- `events.jsonl`: a timeline with seconds since the start:
  - explorer state, message and goal, navigation goal state, shared map joined (whenever they change);
  - the car's position every second;
  - every Nav2 warning or error, with the car's position at that moment;
- `nav.log`: Nav2's whole output;
- `bag/`: a ros2 bag of the lidar (raw and filtered), depth obstacles, odometry, TF, the maps and
  costmaps, Nav2's plan, the controller's lookahead point and collision arc, and the drive commands.
  It is not compressed (about 15 MB a minute);
- `summary.json`: duration, goals, area mapped, the saved map, and Nav2 warnings counted by kind;
- `run.json`: radius, dry run, together.

The newest 20 runs are kept, 10 GB at most. `GET /api/runs` lists them.

### Mapping together (two cars explore at once)

Tick **together** next to 🧭 Autonomous mapping. Every other online car that is idle starts too, and
**Stop autonomous mapping** stops them all. The cars can start anywhere, facing any way, near each other or in
different rooms. `web_ui/team.py` does the rest:

- **Before their maps overlap** each car explores on its own; its lidar and depth camera see the other
  car as an ordinary obstacle. Once the two maps share enough walls and floor, they line up (cars that
  start in the same room: within a few seconds).

- **Live maps lined up:** each car keeps its own SLAM map in its own frame. Every 10 s it fetches the
  other car's live map (`GET /api/slam/grid`) and lines it up with its own, the same way as Merge.
  A match is used only once two checks in a row agree (within 30 cm and 8°). Later checks only
  refine it, which takes well under a second.
  - A full search takes about 5 s and runs at most every 20 s while the maps don't match yet. It runs
    in a separate low-priority process, so it never slows the driving relay or the sensors.
  - On the page: "jpi2: matched" next to the map, and the other car drawn in blue in this car's map.
- **Work split:**
  - An edge whose unmapped side the other car has already mapped is not a target.
  - Frontiers within 1.5 m of the other car's goal are left to it.
  - Frontiers nearer to the other car than to this one cost extra.
  - If both cars picked the same frontier, the car whose name sorts first keeps it.
- **Avoiding each other** while exploring: costmap marks and right of way (previous section), through
  the lined-up maps.
- **At the end** each car saves its own map. The car whose name sorts first also saves the merged map
  (`team_YYYYMMDD_HHMM`).

Drive test (both cars in the living room ~1 m apart, 2.5 m limit, started together from jpi1's page):
the live maps lined up within 5–10 s (agree 0.95–0.97, kept all the way), jpi2 finished first (18 m²:
the rest within its limit was already mapped by jpi1), jpi1 went on to 25.7 m², and the team map
(`team_…`) has no doubled walls. 1 min 45 s from start to the saved team map.

#### The shared map (`web_ui/shared_map.py`)

One map that the mapping cars build together and that is the same, cell for cell, on every car:

1. **Keyframes:** every 20 cm driven or 10° turned, a car stores one lidar scan with its SLAM pose
   (ranges in mm, ~1 kB). The other cars fetch them every second (`GET /api/shared/frames?since=N`), so
   every car ends up with the same keyframes of all cars.
2. **One frame:** the map is in the SLAM frame of the leader, the car whose name sorts first among the
   cars whose maps are lined up. The leader also decides where each other car's frame sits in its own
   (its match from the live maps above; `GET /api/shared/state`), and every car uses the leader's
   numbers. It publishes a new estimate only when it moved ≥ 3 cm or 0.5°.
3. **Same map everywhere:** each car adds all keyframes up on a grid that counts, per cell, the rays
   that ended there and the rays that reached it, like slam_toolbox (Karto). A cell is a wall when at
   least 30 % of the rays reaching it ended there: thin things the lidar hits now and then (chair legs)
   stay, and a car that stood somewhere for a while fades once the others see through the place. The
   counts are integers, so the result doesn't depend on the order keyframes arrive in, and cars with
   the same keyframes and transforms hold the same map.
   Each car reports a hash of its map; the page shows "same as jpi2 ✓" (or "syncing…").
4. **Drift between the cars:** when the leader's estimate of a car's frame moves, every car rebuilds the
   map from all keyframes (5 ms per keyframe on the Pi).
5. **Re-anchoring:** a car's SLAM later corrects its own map (loop closure), but its keyframes keep the
   pose they were stored with. So each car keeps fitting its own keyframes to its own latest SLAM map
   (the scan matching of "Find position", 4 per half second). A keyframe whose fit moved ≥ 3 cm or 1°
   gets the new pose and a new revision, and the other cars fetch it again. Every car takes the old
   contribution out of the map and adds the new one. With integers this is exact, so the map stays the
   same on every car.

The page shows the shared map in the shared frame, so both cars' pages show the same picture, with this
car in red and the other in blue; the merged map saved at the end is the shared map. For driving, each
car uses its own SLAM map wherever it has seen, and the shared map only where it hasn't been
(`/team/map`, in its own frame, at most every 3 s). Nav2 plans on that map and the explorer picks
frontiers from it. The first drive test showed why. With the shared map everywhere, the other car's
scans marked this car's own spot as a wall ("starting point in lethal space"). Thin furniture the
shared map had cleared was still in the live lidar view, so the controller stopped again and again
("collision ahead"), and the cars went back and forth. Before the maps are lined up, each car maps on its own.

**Starting in a new place** (no saved map at all): put the cars near each other, so their first scans
see the same walls; any direction, a few metres apart is fine. Tick **together** and start autonomous mapping.
Each car maps in place until the shared map has joined ("waiting for the other car's map to line up",
30 s at most; usually ~10 s), then they go their separate ways, already on one map. A match that was
confirmed is kept while the maps grow apart: then only walls have to agree and not clash, without the
share of overlap needed for a first match, because the cars may share only the start area.
`tests/run.sh test_new_place.py` simulates this: a made-up 16 × 10 m apartment, cars start 0.6 m apart
in the middle room, one tours the west wing and the other the east wing (up to 14 m apart). jpi2's SLAM
drifts 1°/m and corrects it later. Result: joined after 1 m, never dropped, same map on both, walls
agree 97% with the real plan (85% without re-anchoring), 91–98% of each wing mapped. The match is
5 cm off where the cars are.

Tests: `tests/run.sh test_shared.py`. Two simulated cars drive the living-room map in their own frames.
Result: same map on both, walls agree 1.00 with the real room, independent of keyframe order, rebuilt
the same way after a new transform. Live, with both cars parked: joined within ~15 s, same hash on both
pages.

How well maps line up, from tests that split one real map into two halves (one rotated into its own
frame):
- With 1 m or more of overlap they line up within 2 cm and 0.3°.
- With 0.5 m of overlap the result is rejected rather than wrong.

A match also needs:
- At least 1.5 m² of floor that both maps saw.
- Matching walls that extend in two directions, not just one straight wall.

This rejects maps that only touch along a wall. Real pairs share 13–31 m² of floor; wrong fits shared
under 0.1 m². Test: `tests/run.sh test_team.py`.

Live check without driving: car 2 mapped while parked at its charger, and car 1 fetched that live map.
It lined up with the saved living-room map (walls agree 95%, no clashes, 2.6 m² of shared floor). A
saved map that never covered that corner was rejected. Not yet tried with two cars driving.

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
