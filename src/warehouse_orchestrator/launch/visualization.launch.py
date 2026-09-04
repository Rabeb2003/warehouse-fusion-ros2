#!/usr/bin/env python3
"""
Visualization Launch File — Warehouse Fusion Workspace
======================================================
Dedicated launch file for RViz2 visualization of both Panda arm and trailer.
This launch file ONLY starts RViz - it assumes robot_state_publishers are already
running from the main mission launch (full_mission.launch.py).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Chemins des packages ─────────────────────────────────────────────────
    warehouse_orchestrator_share = get_package_share_directory('warehouse_orchestrator')

    # RViz config
    rviz_config_fusion  = os.path.join(warehouse_orchestrator_share, 'rviz', 'fusion_mission.rviz')

    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time_arg  = DeclareLaunchArgument('use_sim_time',  default_value='true')
    enable_rviz_arg   = DeclareLaunchArgument('enable_rviz',   default_value='true')

    # ========================================================================
    # RViz2 FUSIONNÉ (Panda + Trailer)
    #   Config fusion_mission.rviz : RobotModel Panda + Trailer, TF tree.
    #   Fixed Frame = "world".
    #   NOTE: This assumes robot_state_publishers are already running from
    #   full_mission.launch.py with proper joint_states remappings.
    # ========================================================================
    rviz2_fused = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fusion',
        output='screen',
        arguments=['-d', rviz_config_fusion],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    # ========================================================================
    # ASSEMBLAGE FINAL — LaunchDescription
    # ========================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_rviz_arg,

        # ── t=0s : RViz2 fusionné
        rviz2_fused,
    ])
