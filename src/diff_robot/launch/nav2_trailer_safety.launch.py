#!/usr/bin/env python3
"""
Nav2 with Trailer Safety - Launch file for differential drive robot with trailer
Integrates anti-jackknife safety nodes with Nav2 navigation
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'diff_robot'
    package_share = get_package_share_directory(package_name)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    
    # File paths
    nav2_params_file = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])
    trailer_params_file = PathJoinSubstitution([package_share, 'config', 'trailer_params.yaml'])
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='Path to map yaml file'),
        DeclareLaunchArgument('use_nav2', default_value='false',
                              description='Launch Nav2 stack'),
        DeclareLaunchArgument('launch_dashboard', default_value='false',
                              description='Launch trailer dashboard GUI'),
        
        # ── 1. Base Robot Launch (Gazebo + Robot State Publisher) ────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, 'launch', 'robot_remorque.launch.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',  # RViz will be launched separately
            }.items(),
        ),
        
        # ── 2. Trailer-Aware Controller (transforms Nav2 commands) ───────────────────
        TimerAction(period=15.0, actions=[
            Node(
                package=package_name,
                executable='trailer_aware_controller_node.py',
                name='trailer_aware_controller',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('input_topic', '/cmd_vel_nav2'),
                    ('output_topic', '/cmd_vel_raw'),
                ],
            ),
        ]),
        
        # ── 3. CMD Vel Safety Node (anti-jackknife protection) ───────────────────────
        TimerAction(period=15.5, actions=[
            Node(
                package=package_name,
                executable='cmd_vel_safety_node.py',
                name='cmd_vel_safety_node',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('input_topic', '/cmd_vel_raw'),
                    ('output_topic', '/cmd_vel'),
                ],
            ),
        ]),
        
        # ── 4. Dynamic Footprint Node (articulated footprint) ────────────────────────
        TimerAction(period=16.0, actions=[
            Node(
                package=package_name,
                executable='dynamic_footprint_node.py',
                name='dynamic_footprint_node',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
            ),
        ]),
        
        # ── 5. Trailer Dashboard (optional monitoring) ─────────────────────────────
        TimerAction(period=16.5, actions=[
            Node(
                package=package_name,
                executable='trailer_dashboard_node.py',
                name='trailer_dashboard',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('launch_dashboard')),
            ),
        ]),
        
        # ── 6. Nav2 Stack (if map is provided) ───────────────────────────────────────
        # Note: Using GPS localization (from robot_remorque.launch.py) instead of AMCL
        TimerAction(period=17.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}, 
                          {'yaml_filename': map_yaml}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                           'node_names': ['map_server'], 'bond_timeout': 10.0}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
        ]),
        
        # ── 7. Nav2 Navigation Stack ─────────────────────────────────────────────────
        TimerAction(period=20.0, actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav2')],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav2'),
                    ('cmd_vel_smoothed', 'cmd_vel_raw'),
                ],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['planner_server', 'controller_server', 'smoother_server',
                                   'behavior_server', 'bt_navigator', 'waypoint_follower',
                                   'velocity_smoother'],
                     'bond_timeout': 10.0},
                ],
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            ),
        ]),
        
        # ── 8. RViz2 (Nav2 + GPS visualization) ─────────────────────────────────────
        TimerAction(period=22.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([package_share, 'urdf', 'rviz.rviz'])],
                parameters=[{'use_sim_time': use_sim_time}],
                additional_env={
                    'QT_QPA_PLATFORM': 'xcb',
                    'DISPLAY': ':0',
                },
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
            ),
        ]),

        # Additional arguments for optional components
        DeclareLaunchArgument('use_nav2', default_value='false',
                              description='Launch Nav2 stack'),
        DeclareLaunchArgument('launch_dashboard', default_value='false',
                              description='Launch trailer dashboard GUI'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Launch RViz2 with Nav2 and GPS plugins'),
    ])
