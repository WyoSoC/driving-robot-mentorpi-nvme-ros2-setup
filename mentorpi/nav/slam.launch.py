"""slam_toolbox for the web UI's mapping (started by web_ui/slam_manager.py), instead of Hiwonder's
slam_base.launch.py: the same node, Hiwonder's slam/config/slam.yaml and the same remappings, with OVERRIDES.

Why: Hiwonder's map_update_interval is 0.2 s. slam_toolbox then rebuilds the whole occupancy grid from all
scans and publishes /map 5 times a second (measured ~4.6 Hz), which costs CPU that Nav2 needs while the car
explores (its controller and planner already missed their rates). Nothing here needs the map that often:
the page draws it once a second, Nav2 gets the team map at most every 3 s (team.py).

  ros2 launch /home/ubuntu/nav/slam.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

OVERRIDES = {
    'map_update_interval': 2.0,        # s between map rebuilds / publishes (Hiwonder: 0.2; slam_toolbox: 5.0)
    'enable_interactive_mode': False,  # interactive markers for editing the graph in RViz: not used here
}
OUT = '/tmp/slam_params.yaml'


def setup(context):
    if os.environ.get('need_compile') == 'True':
        src = os.path.join(get_package_share_directory('slam'), 'config', 'slam.yaml')
    else:
        src = '/home/ubuntu/ros2_ws/src/slam/config/slam.yaml'
    p = yaml.safe_load(open(src))
    node_key = next(iter(p))                          # 'slam_toolbox'
    params = p[node_key]['ros__parameters']
    params.update({'use_sim_time': False, 'map_frame': 'map', 'odom_frame': 'odom',
                   'base_frame': 'base_footprint'})
    params.update(OVERRIDES)
    with open(OUT, 'w') as f:
        yaml.safe_dump(p, f)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/map', 'map'), ('/map_metadata', 'map_metadata')]
    return [Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
                 output='screen', parameters=[OUT], remappings=remappings)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
