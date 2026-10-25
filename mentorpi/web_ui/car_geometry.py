"""The car's shape and sensor positions, measured on the car (MentorPi M1 Ackermann; both cars the same).

Measured from the centre of the lidar:
- the body is a rectangle: front 12 cm ahead, back 10 cm behind, 9 cm to each side;
- the lidar's top is 16 cm above the floor;
- the front of the depth camera is 3 cm behind the front of the car, its centre 11 cm above the floor.

Everything here is in base_footprint (the car's frame: x forward, y left, on the floor; the lidar centre is
LIDAR_X from its origin, from the robot description's TF). Used by depth_obstacles.py (camera position),
fleet.py (the other car's body), nav_manager.py and explore_manager.py (room past the bumpers).
nav/nav2_params.yaml can't import this: its footprints and collision-monitor zones are written out there
and tests/test_geometry.py checks they match.
"""

LIDAR_X = -0.010           # lidar centre in base_footprint (TF base_footprint -> lidar_frame)
LIDAR_TOP_Z = 0.16         # m above the floor (TF: 0.163)

# from the lidar centre (measured)
LIDAR_TO_FRONT = 0.12
LIDAR_TO_BACK = 0.10
HALF_WIDTH = 0.09
CAMERA_BEHIND_FRONT = 0.03  # front of the depth camera to the front of the car
CAMERA_Z = 0.11            # centre of the depth camera above the floor

# in base_footprint
FRONT = LIDAR_X + LIDAR_TO_FRONT        # 0.11
BACK = LIDAR_X - LIDAR_TO_BACK          # -0.11
CAMERA_X = FRONT - CAMERA_BEHIND_FRONT  # 0.08
FOOTPRINT = [[FRONT, HALF_WIDTH], [FRONT, -HALF_WIDTH], [BACK, -HALF_WIDTH], [BACK, HALF_WIDTH]]

# lidar centre -> bumper, by driving direction (deg: 0 forward, 180 backward), for lidar clearance checks
BUMPER = {0: LIDAR_TO_FRONT, 180: LIDAR_TO_BACK}
