#!/usr/bin/env python3
"""
Panda RViz Launch File
Launches RViz2 with Panda MoveIt configuration.
Panda robot is already spawned in Gazebo by trailer_nav2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # ========================================================================
    # ARGUMENTS
    # ========================================================================
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # ========================================================================
    # PATHS
    # ========================================================================
    panda_moveit_config_share = FindPackageShare('panda_moveit_config')
    
    # ========================================================================
    # RVIZ2 FOR PANDA
    # ========================================================================
    rviz_config = os.path.join(
        panda_moveit_config_share.find('panda_moveit_config'),
        'rviz',
        'moveit.rviz'
    )
    
    # Load robot description parameters for RViz
    robot_description = PathJoinSubstitution([
        FindPackageShare('panda_description'),
        'urdf',
        'panda.urdf.xacro'
    ])
    
    rviz2_panda = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_panda',
        arguments=['--display-config', rviz_config],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg='=== Panda RViz Launch ==='),
            LogInfo(msg='=== Panda robot must be spawned in Gazebo first ==='),
            LogInfo(msg='=== Launch trailer_nav2.launch.py first ==='),
            rviz2_panda,
        ]
    )
