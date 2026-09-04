#!/usr/bin/env python3
"""
Warehouse full mission launch — follows recorded_waypoints.yaml then PnP.

Record path first with record_waypoints.launch.py + teleop + waypoint_recorder.

Usage:
  ros2 launch warehouse_orchestrator warehouse_mission.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_orchestrator')
    full_mission = os.path.join(pkg, 'launch', 'full_mission.launch.py')
    default_wp = os.path.join(pkg, 'config', 'recorded_waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('enable_fusion_rviz', default_value='true'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_wp,
            description='YAML recorded by waypoint_recorder'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_mission),
            launch_arguments={
                'enable_fusion_rviz': LaunchConfiguration('enable_fusion_rviz'),
                'enable_nav2': 'true',
                'enable_ekf': 'true',
                'enable_mission': 'true',
                'enable_pick_place': 'true',
                'enable_bootstrap_tf': 'false',
                'enable_bootstrap_base_tf': 'false',
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'x_pose': '-0.14',
                'y_pose': '-2.07',
            }.items(),
        ),
    ])
