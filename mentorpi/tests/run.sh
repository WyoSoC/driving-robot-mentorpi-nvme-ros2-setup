#!/bin/bash
# Run a test inside the MentorPi container: tests/run.sh test_fleet.py | test_merge.py | test_depth_sim.py
# None of them drives the car. test_merge.py uses the maps saved on this car.
cd "$(dirname "$0")" || exit 1
[ -f "$1" ] || { echo "usage: $0 test_fleet.py | test_merge.py | test_depth_sim.py"; exit 1; }
docker exec -i MentorPi bash -lc 'source /opt/ros/humble/setup.bash; cd /tmp; python3 -' < "$1"
