# MentorPi M1 — NVMe Boot + ROS2 Setup

Setup scripts, configuration, and source patches for running the
[Hiwonder MentorPi M1](https://www.hiwonder.com/products/mentorpi) robotic car
from an NVMe SSD on a Raspberry Pi 5 with Ubuntu 25.04.

The factory software ships on an SD card running Ubuntu 22.04 in a Docker container.
This repo documents two ways to run it on a modern NVMe-boot system:

| | Option 1 — Docker | Option 2 — Native |
|---|---|---|
| **Base OS** | Ubuntu 25.04 (host) | Ubuntu 25.04 (host) |
| **ROS2** | Humble (inside container, Ubuntu 22.04) | Humble (conda / RoboStack) |
| **Fidelity** | Exact factory image | Built from source |
| **Setup effort** | Pull image + run script | Build workspace (~20 min) |
| **Depth camera** | Works | Not yet (PCL/Qt6 issue) |
| **Video server** | Works | Not yet (FFmpeg 4→7 port needed) |

---

## Hardware

| Component | Detail |
|---|---|
| **SBC** | Raspberry Pi 5 |
| **Chassis** | Ackermann steering (`MACHINE_TYPE=MentorPi_Acker`) |
| **Motor controller** | STM32 via `ros_robot_controller` on `/dev/ttyACM0` |
| **LiDAR** | Hiwonder MS200 (`oradar_lidar` driver) on `/dev/ttyUSB0` |
| **Depth camera** | Orbbec Astra (`ascamera`) |
| **OS** | Ubuntu 25.04 Plucky Puffin, booting from NVMe SSD |

---

## Prerequisites

```bash
# Add your user to the serial port groups
sudo usermod -aG dialout,tty $USER
# Log out and back in, or: newgrp dialout

# Install udev rules for hardware devices
bash udev/install_udev_rules.sh
```

---

## Option 1 — Docker (Factory Image)

### Pull the factory image

```bash
docker pull ghcr.io/wyoarcc/mentorpi:factory
docker tag  ghcr.io/wyoarcc/mentorpi:factory mentorpi:factory
```

### Install scripts

```bash
mkdir -p ~/mentorpi
cp docker/run_container.sh  ~/mentorpi/
cp docker/start_node.sh     ~/mentorpi/
chmod +x ~/mentorpi/*.sh
```

### (Optional) Install systemd auto-start

```bash
# Edit the service file if your username is not 'gojian'
sed -i "s/gojian/$USER/g" docker/mentorpi-container.service

sudo cp docker/mentorpi-container.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mentorpi-container.service
```

### Run

```bash
# Start container + launch ROS2
~/mentorpi/run_container.sh
~/mentorpi/start_node.sh

# Or if systemd service is installed:
sudo systemctl start mentorpi-container.service
```

See [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) for all available commands.

---

## Option 2 — Native (conda / RoboStack)

### System dependencies

```bash
sudo apt-get install -y \
  libsuitesparse-dev \
  libg2o-dev \
  libmetis-dev

# libg2o_incremental is not in the Ubuntu 25.04 package — copy from Docker image:
sudo docker run --rm ghcr.io/wyoarcc/mentorpi:factory \
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
docker run --rm ghcr.io/wyoarcc/mentorpi:factory tar -C /opt/ros/humble -c \
  lib/libfilters.so lib/liblaser_scan_filters.so \
  include/filters share/filters | tar -C $CONDA -x

# apriltag C library (apriltag_ros dependency)
docker run --rm ghcr.io/wyoarcc/mentorpi:factory tar -C /opt/ros/humble -c \
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
├── docker/
│   ├── run_container.sh            # Start/restart the factory Docker container
│   ├── start_node.sh               # Launch ROS2 bringup inside container
│   └── mentorpi-container.service  # systemd unit for auto-start on boot
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
└── udev/
    ├── 99-ttyACM0.rules             # STM32 motor controller serial port
    ├── lidar.rules                  # LiDAR USB-serial device
    ├── 99-usb-cam.rules             # USB camera
    ├── 99-obsensor-libusb.rules     # Orbbec depth camera
    └── install_udev_rules.sh        # Helper to install all rules
```

---

## References

- [Hiwonder MentorPi M1 product page](https://www.hiwonder.com/products/mentorpi)
- [Hiwonder MentorPi GitHub](https://github.com/Hiwonder/MentorPi)
- [RoboStack — ROS2 via conda](https://robostack.github.io/)
- [Nav2 documentation](https://navigation.ros.org/)
