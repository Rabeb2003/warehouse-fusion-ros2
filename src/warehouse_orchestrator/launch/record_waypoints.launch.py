#!/usr/bin/env python3
"""
Bringup for manual teleop + waypoint recording (NO auto mission).

Usage:
  # Terminal 1
  ros2 launch warehouse_orchestrator record_waypoints.launch.py

  # Terminal 2 — keyboard drive (/cmd_vel)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 3 — recorder (Enter = save pose in map/GPS frame)
  ros2 run warehouse_orchestrator waypoint_recorder
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

    return LaunchDescription([
        DeclareLaunchArgument('enable_fusion_rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_mission),
            launch_arguments={
                'enable_fusion_rviz': LaunchConfiguration('enable_fusion_rviz'),
                'enable_nav2': 'true',
                'enable_ekf': 'true',
                'enable_mission': 'false',
                'enable_pick_place': 'false',
                'enable_bootstrap_tf': 'false',
                'enable_bootstrap_base_tf': 'false',
                'x_pose': '-0.14',
                'y_pose': '-2.07',
            }.items(),
        ),
    ])
