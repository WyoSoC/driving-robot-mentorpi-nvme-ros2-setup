"""Nav2 for the MentorPi (Ackermann) on a saved map. Runs next to the normal bringup (drivers, EKF);
started and stopped by the web UI (web_ui/nav_manager.py).

  ros2 launch /home/ubuntu/nav/nav.launch.py map:=/home/ubuntu/maps/livingroom.yaml
  ros2 launch /home/ubuntu/nav/nav.launch.py slam:=true    # on the live map of a running slam_toolbox
                                                             # (exploration): no map_server, no AMCL

Velocity chain (see nav2_params.yaml): controller / behaviors -> /nav/cmd_vel_raw -> velocity_smoother
-> /nav/cmd_vel_smoothed -> collision_monitor -> /nav/cmd_vel (the web server gates it to /cmd_vel).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(NAV_DIR, 'nav2_params.yaml')
RAW, SMOOTHED = '/nav/cmd_vel_raw', '/nav/cmd_vel_smoothed'


def setup(context):
    map_yaml = LaunchConfiguration('map').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() in ('true', '1')
    common = dict(output='screen', parameters=[PARAMS], respawn=False)
    localization = ['map_server', 'amcl']
    navigation = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                  'velocity_smoother', 'collision_monitor']
    nodes = [] if slam else [   # with SLAM, slam_toolbox publishes /map and map -> odom
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[PARAMS, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', parameters=[{'autostart': True, 'node_names': localization, 'bond_timeout': 10.0}]),
    ]
    return nodes + [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             remappings=[('cmd_vel', RAW)], **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=[('cmd_vel', RAW)], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             remappings=[('cmd_vel', RAW), ('cmd_vel_smoothed', SMOOTHED)], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'autostart': True, 'node_names': navigation, 'bond_timeout': 10.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('map', default_value='', description='map yaml (saved with the web UI Map panel)'),
        DeclareLaunchArgument('slam', default_value='false', description='true: navigate on the live SLAM map'),
        OpaqueFunction(function=setup),
    ])
