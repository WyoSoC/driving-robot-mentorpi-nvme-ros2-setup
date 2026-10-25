#!/bin/bash
# Run a test inside the MentorPi container: tests/run.sh test_fleet.py | test_merge.py | test_team.py | test_shared.py | test_new_place.py | test_maps.py | test_geometry.py | test_steering.py | test_recovery.py | test_stall.py | test_lift.py | test_tight.py | test_drops.py | test_replan.py | test_localize.py | test_depth_sim.py | test_runs.py | test_vlm.py | test_depth_record.py
# None of them drives the car. test_merge.py uses the maps saved on this car.
cd "$(dirname "$0")" || exit 1
[ -f "$1" ] || { echo "usage: $0 test_fleet.py | test_merge.py | test_team.py | test_shared.py | test_new_place.py | test_maps.py | test_geometry.py | test_steering.py | test_recovery.py | test_stall.py | test_lift.py | test_tight.py | test_drops.py | test_replan.py | test_localize.py | test_depth_sim.py | test_runs.py | test_vlm.py | test_depth_record.py"; exit 1; }
# copied in as a file: worker processes (team.py) need a real main module
docker cp -q "$1" "MentorPi:/tmp/$1" && docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash; cd /tmp; timeout 600 python3 /tmp/$1"
