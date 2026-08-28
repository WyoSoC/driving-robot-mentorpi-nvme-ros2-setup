#!/bin/bash
xhost +
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh"
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 launch bringup bringup.launch.py"
