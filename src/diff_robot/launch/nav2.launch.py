#!/usr/bin/env python3
"""
Nav2 Launch File — Robot Remorque (Diff Drive + Trailer)
=========================================================
Launches the entire Nav2 stack explicitly (without AMCL)
integrated with the trailer safety control nodes in a clean serial pipeline:
  controller_server -> /cmd_vel_nav2_raw
  velocity_smoother -> /cmd_vel_nav2
  trailer_aware_controller -> /cmd_vel_raw
  cmd_vel_safety_node -> /cmd_vel
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot')

    # Default file paths
    default_map = os.path.join(pkg_share, 'map', 'my_map.yaml')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_trailer_params = os.path.join(pkg_share, 'config', 'trailer_params.yaml')
    default_bt_xml = os.path.join(pkg_share, 'config', 'navigate_to_pose_trailer_recovery.xml')
    default_rviz_config = os.path.join(pkg_share, 'urdf', 'rviz.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    trailer_params_file = LaunchConfiguration('trailer_params_file')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map YAML file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'trailer_params_file',
            default_value=default_trailer_params,
            description='Full path to the trailer parameters file'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with Nav2 configuration'),

        # ── 1. Map Server Node ──────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_yaml}]),

        # ── 2. Lifecycle Manager for Map Server ──────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        # ── 3. Nav2 Planner Server ───────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # ── 4. Nav2 Controller Server ────────────────────────────────────────
        # Outputs to /cmd_vel_nav2_raw (to be smoothed by velocity_smoother)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav2_raw')]),

        # ── 5. Nav2 Smoother Server ──────────────────────────────────────────
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # ── 6. Nav2 Behavior Server (Recoveries) ─────────────────────────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav2_raw')]),

        # ── 7. Nav2 BT Navigator ─────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_bt_xml,
            }]),

        # ── 8. Nav2 Waypoint Follower ────────────────────────────────────────
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # ── 9. Nav2 Velocity Smoother ────────────────────────────────────────
        # Inputs /cmd_vel_nav2_raw, outputs smoothed command to /cmd_vel_nav2
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav2_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav2'),
            ]),

        # ── 10. Nav2 Lifecycle Manager for Navigation ────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['planner_server', 'controller_server', 'smoother_server',
                                       'behavior_server', 'bt_navigator', 'waypoint_follower',
                                       'velocity_smoother']}]),

        # ── 11. Trailer-Aware Controller (Kinematics Transformation) ─────────
        # Subscribes to /cmd_vel_nav2, transforms it and publishes to /cmd_vel_raw
        Node(
            package='diff_robot',
            executable='trailer_aware_controller_node.py',
            name='trailer_aware_controller',
            output='screen',
            parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('input_topic', '/cmd_vel_nav2'),
                ('output_topic', '/cmd_vel_raw'),
            ]),

        # ── 12. Command Velocity Safety Node (Anti-Jackknife Watchdog) ───────
        # Subscribes to /cmd_vel_raw, checks limits and publishes to /cmd_vel
        Node(
            package='diff_robot',
            executable='cmd_vel_safety_node.py',
            name='cmd_vel_safety_node',
            output='screen',
            parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('input_topic', '/cmd_vel_raw'),
                ('output_topic', '/cmd_vel'),
            ]),

        # ── 13. RViz2 (Nav2 + GPS visualization) ─────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            additional_env={
                'QT_QPA_PLATFORM': 'xcb',
                'DISPLAY': ':0',
            },
            condition=IfCondition(rviz)),
    ])
