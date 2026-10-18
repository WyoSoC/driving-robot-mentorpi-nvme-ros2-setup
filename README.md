# MentorPi M1 — NVMe Boot + ROS2 Setup

Setup scripts, configuration, and source patches for running the
[Hiwonder MentorPi M1](https://www.hiwonder.com/products/mentorpi) robotic car
from an NVMe SSD on a Raspberry Pi 5 with Ubuntu 25.04.

## Mission

**Koatlus Autonomous Navigation:** a small fleet of low-cost Ackermann cars that map an unknown indoor
space by themselves, together, and then navigate it safely: aware of each other, of obstacles at any
height the car can hit, and of drops it can fall down.

The cars work as one decentralized set of sensors: spatially apart, aware of each other, constantly
exchanging what they see, and building one shared map that is the same on every car. They can start
anywhere.

| # | Goal | State |
|---|---|---|
| 1 | Each car drives, sees and is monitored from a browser (camera, lidar, depth, IMU, odometry, battery) | Done |
| 2 | Each car maps a space by itself (frontier exploration; backs out of tight spots along its own trail) | Done, tested on both cars |
| 3 | Navigate to a goal on any saved map, from either car; the car finds its own position on the map from its lidar | Done |
| 4 | Cars know about each other: status exchanged at all times, maps synced between cars | Done |
| 5 | Obstacles from lidar **and** depth camera; camera grades threat: rug = drivable, low hump = bump, else obstacle | Done, tested live |
| 6 | Cars avoid each other: each car's planner treats the other car as an obstacle, and a right-of-way rule when they meet | Done, drive-tested with both cars (crossing and head-on, from a tight start 40 cm apart) |
| 7 | Map together: two cars explore one space at once, split the unexplored areas, merge into one map | Done, drive-tested (living room, 2.5 m limit: maps lined up within 10 s, team map in under 2 min) |
| 7b | One shared map: the cars exchange lidar keyframes and build the same map, in one frame, on every car; navigation and exploration use it | Built; same map on both cars checked live (parked); drive test next |
| 7c | Start together in a new place (no saved map): cars start near each other, line up, then split up; the match holds as they drift apart, keyframes follow each car's SLAM corrections | Built, simulated (16 × 10 m apartment, cars up to 14 m apart, with SLAM drift); drive test next |
| 8 | Never drive off a drop (stairs, ledges): downward range sensors, no-go zones on the map | Open (needs sensors; camera sees no glossy floor) |
| 9 | Later: AprilTag landmarks, traffic rules for shared corridors | Open |

Ground rule: a car only drives with a person present who has confirmed it.

Details of each part: [mentorpi/README.md](mentorpi/README.md).

## Setup

The factory software ships on an SD card running Ubuntu 22.04 in a Docker container.
This repo documents two ways to run it on a modern NVMe-boot system:

| | Option 1 — Docker | Option 2 — Native |
|---|---|---|
| **Base OS** | Ubuntu 25.04 (host) | Ubuntu 25.04 (host) |
| **ROS2** | Humble (inside container, Ubuntu 22.04) | Humble (conda / RoboStack) |
| **Fidelity** | Exact factory image | Built from source |
| **Setup effort** | `install.sh` (one command) | Build workspace (~20 min) |
| **Depth camera** | Works | Not yet (PCL/Qt6 issue) |
| **Video server** | Works | Not yet (FFmpeg 4→7 port needed) |
| **Web control UI** | Yes (live video, sensors, driving) | No |
| **Status** | **Supported; running on the cars** | Does not run bringup yet (see [mentorpi/README.md](mentorpi/README.md#native-conda-build-not-in-use)) |

---

## Hardware

| Component | Detail |
|---|---|
| **SBC** | Raspberry Pi 5 |
| **Chassis** | Ackermann steering (`MACHINE_TYPE=MentorPi_Acker`) |
| **Motor controller** | STM32 via `ros_robot_controller`, USB `1a86:55d4` → `/dev/rrc` |
| **LiDAR** | Hiwonder MS200 (`oradar_lidar` driver), USB `1a86:7523` → `/dev/ldlidar` |
| **Depth camera** | Angstrong HP60C (`ascamera`), USB `3482:6723` |
| **OS** | Ubuntu 25.04 Plucky Puffin, booting from NVMe SSD |

---

## Option 1 — Docker (Factory Image)

Everything for this option is in [`mentorpi/`](mentorpi/): three systemd services that start
the container, the ROS 2 bringup and a browser control UI at boot, udev rules, per-car
settings, a health check, and an installer for setting up more cars.
**See [mentorpi/README.md](mentorpi/README.md) for the full guide.**

Quick start on a car (Docker installed, `mentorpi:factory` image present or available as
a transfer bundle from another car):

```bash
git clone https://github.com/WyoSoC/driving-robot-mentorpi-nvme-ros2-setup.git
ln -s ~/driving-robot-mentorpi-nvme-ros2-setup/mentorpi ~/mentorpi
~/mentorpi/install.sh [--image /path/to/mentorpi-factory.tar.zst]
sudo systemctl start mentorpi-webui
~/mentorpi/status.sh                 # then open http://<car-ip>:8000
```

Each car gets its name and a unique `ROS_DOMAIN_ID` from its hostname (`jpi2` → 2).

The factory image (~6 GB compressed) is not on a public registry. Copy it from a working
car with `mentorpi/make_bundle.sh` (see "Transfer to another identical car" in the guide).

See [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) for manual launch commands.

---

## Option 2 — Native (conda / RoboStack)

### System dependencies

```bash
sudo apt-get install -y \
  libsuitesparse-dev \
  libg2o-dev \
  libmetis-dev

# libg2o_incremental is not in the Ubuntu 25.04 package — copy from Docker image:
sudo docker run --rm mentorpi:factory \
  cat /opt/ros/humble/lib/aarch64-linux-gnu/libg2o_incremental.so \
  | sudo tee /usr/lib/aarch64-linux-gnu/libg2o_incremental.so > /dev/null
```

### Install conda (Miniforge)

```bash
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
bash Miniforge3-Linux-aarch64.sh -b -p ~/Programs/Conda
~/Programs/Conda/bin/conda init bash
# Open a new terminal, then:
```

### Create the ROS2 Humble environment

```bash
# From this repo:
conda env create -f native/environment.yml
# This installs ~326 packages including ROS2 Humble via RoboStack (~10-20 min)

# Pin critical versions to avoid solver breakage on updates:
PINNED=~/Programs/Conda/envs/ros2_humble/conda-meta/pinned
echo "empy ==3.3.4"      >> $PINNED
echo "setuptools ==65.7.0" >> $PINNED
```

### Clone and patch the workspace

```bash
# Clone Hiwonder's source
mkdir -p ~/ros2_ws/src
git clone https://github.com/Hiwonder/MentorPi.git ~/ros2_ws/src \
  --recurse-submodules

# Apply source-level compatibility patches (Ubuntu 25.04 / GCC 14 / CMake 3.31)
cd ~/ros2_ws/src/third_party
patch -p1 -d async_web_server_cpp < /path/to/patches/0001-async_web_server_cpp-boost-array-include.patch
patch -p1 -d ldlidar_stl_ros2    < /path/to/patches/0002-ldlidar-linux-types-for-termios.patch
patch -p1 -d ldlidar_stl_ros2    < /path/to/patches/0003-ldlidar-pthread-include.patch
patch -p1 -d oradar_lidar        < /path/to/patches/0004-oradar-cmake-old-policies.patch
patch -p1 -d oradar_lidar        < /path/to/patches/0005-oradar-cmake-cmp0026-location.patch
```

### Copy additional libraries from Docker image

Some packages are not available via conda or apt. Copy them from the factory image:

```bash
CONDA=~/Programs/Conda/envs/ros2_humble

# filters (laser_filters dependency)
docker run --rm mentorpi:factory tar -C /opt/ros/humble -c \
  lib/libfilters.so lib/liblaser_scan_filters.so \
  include/filters share/filters | tar -C $CONDA -x

# apriltag C library (apriltag_ros dependency)
docker run --rm mentorpi:factory tar -C /opt/ros/humble -c \
  lib/libapriltag.so lib/libapriltag.so.3 lib/libapriltag.so.3.2.0 \
  include/apriltag share/apriltag | tar -C $CONDA -x
```

### Install workspace scripts

```bash
cp native/setup_env.sh      ~/ros2_ws/
cp native/build_workspace.sh ~/ros2_ws/
cp native/.typerc            ~/ros2_ws/
chmod +x ~/ros2_ws/*.sh

# Add shell aliases
echo "alias ros2env='source ~/ros2_ws/setup_env.sh'" >> ~/.bashrc
echo "alias buildros='~/ros2_ws/build_workspace.sh'" >> ~/.bashrc
source ~/.bashrc
```

### Build

```bash
buildros
# Takes ~10-20 min on first run. Skips: ascamera, web_video_server, ydlidar, simulations
```

### Run

```bash
ros2env
ros2 launch bringup bringup.launch.py
```

See [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) for all launch options including joystick and navigation.

---

## Known limitations (Option 2 — native)

| Package | Status | Notes |
|---|---|---|
| `ascamera` | Not built | conda PCL compiled with Qt6 cross-build support; `QT_HOST_PATH` not set |
| `web_video_server` | Not built | Uses FFmpeg 4.x API; conda ships FFmpeg 7.x (API breaks: `av_register_all`, `AVStream::codec`, etc.) |
| `ydlidar_ros2_driver` | Not built | Proprietary `ydlidar_sdk` unavailable; not needed (robot uses MS200/oradar) |
| `simulations`, `large_models`, `yolov5_ros2` | Not built | Heavy deps; not needed for hardware use |

---

## Repository structure

```
.
├── README.md
├── LAUNCH_GUIDE.md        # Full launch reference for both options
├── mentorpi/               # Option 1: Docker stack, web UI, installer (see its README)
│   ├── install.sh / make_bundle.sh / status.sh
│   ├── run_container.sh / start_node.sh
│   ├── car.env.example / calibration.defaults/   # per-car settings templates
│   ├── system/            # udev rules + systemd unit templates
│   └── web_ui/            # browser control UI
├── native/
│   ├── setup_env.sh        # Activate conda env + source workspace
│   ├── build_workspace.sh  # colcon build with all required cmake args
│   ├── .typerc             # Robot config env vars (MACHINE_TYPE, LIDAR_TYPE, etc.)
│   └── environment.yml     # Reproducible conda environment spec
├── patches/
│   ├── README.md           # Patch descriptions and apply instructions
│   ├── 0001-async_web_server_cpp-boost-array-include.patch
│   ├── 0002-ldlidar-linux-types-for-termios.patch
│   ├── 0003-ldlidar-pthread-include.patch
│   ├── 0004-oradar-cmake-old-policies.patch
│   └── 0005-oradar-cmake-cmp0026-location.patch
```

The native option needs the udev rules from `mentorpi/system/` as well
(`sudo install -m 644 mentorpi/system/*.rules /etc/udev/rules.d/`).

---

## References

- [Hiwonder MentorPi M1 product page](https://www.hiwonder.com/products/mentorpi)
- [Hiwonder MentorPi GitHub](https://github.com/Hiwonder/MentorPi)
- [RoboStack — ROS2 via conda](https://robostack.github.io/)
- [Nav2 documentation](https://navigation.ros.org/)
