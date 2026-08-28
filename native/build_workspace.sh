#!/bin/bash
# Build the MentorPi native ROS2 workspace.
# Run once after creating the conda environment, and again after any code changes.

set -e

CONDA_BASE=/home/gojian/Programs/Conda
ROS2_ENV=ros2_humble
WS=/home/gojian/ros2_ws

source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ROS2_ENV"
source "$CONDA_BASE/envs/$ROS2_ENV/setup.bash"

# empy 3.3.4 required — rosidl template engine is incompatible with empy 4.x
pip show empy | grep -q "3.3.4" || pip install empy==3.3.4 -q

cd "$WS"

# Skip simulation packages and heavy AI packages on first build
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DG2O_INCLUDE_DIR=/usr/include/g2o \
    -DG2O_CORE_LIB=/usr/lib/aarch64-linux-gnu/libg2o_core.so \
    -DG2O_STUFF_LIB=/usr/lib/aarch64-linux-gnu/libg2o_stuff.so \
    -DG2O_TYPES_SLAM2D_LIB=/usr/lib/aarch64-linux-gnu/libg2o_types_slam2d.so \
    -DG2O_TYPES_SLAM3D_LIB=/usr/lib/aarch64-linux-gnu/libg2o_types_slam3d.so \
    -DG2O_SOLVER_CHOLMOD_LIB=/usr/lib/aarch64-linux-gnu/libg2o_solver_cholmod.so \
    -DG2O_SOLVER_PCG_LIB=/usr/lib/aarch64-linux-gnu/libg2o_solver_pcg.so \
    -DG2O_SOLVER_CSPARSE_LIB=/usr/lib/aarch64-linux-gnu/libg2o_solver_csparse.so \
    -DG2O_INCREMENTAL_LIB=/usr/lib/aarch64-linux-gnu/libg2o_incremental.so \
    -DG2O_CSPARSE_EXTENSION_LIB=/usr/lib/aarch64-linux-gnu/libg2o_csparse_extension.so \
  --packages-ignore \
    simulations \
    large_models \
    large_models_msgs \
    yolov5_ros2 \
    ascamera \
    web_video_server \
    ydlidar_ros2_driver \
  2>&1 | tee /tmp/colcon_build.log

echo ""
echo "Build complete. Source the workspace:"
echo "  source ~/ros2_ws/setup_env.sh"
