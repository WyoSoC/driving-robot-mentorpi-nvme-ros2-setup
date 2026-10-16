#!/bin/bash
# Run the ROS2 bringup inside the MentorPi container (foreground).
xhost + >/dev/null 2>&1 || true
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh"
exec docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 launch bringup bringup.launch.py"
