#!/bin/bash
# Source this file to activate the MentorPi native ROS2 environment.
# Usage: source ~/ros2_ws/setup_env.sh

CONDA_BASE=/home/gojian/Programs/Conda
ROS2_ENV=ros2_humble
WS=/home/gojian/ros2_ws

# Activate conda environment
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ROS2_ENV"

# Source ROS2
source "$CONDA_BASE/envs/$ROS2_ENV/setup.bash"

# Source the built workspace (if it exists)
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# Load robot config
source "$WS/.typerc"

# Serial port access
export PATH="/home/gojian/.local/bin:$PATH"
