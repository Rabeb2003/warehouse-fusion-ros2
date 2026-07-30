#!/usr/bin/env python3
"""
SLAM Trailer Launch - SLAM Toolbox + Trailer Control for mapping
===================================================================
Lance:
  1. Gazebo avec le world existant
  2. Robot State Publisher (URDF robot-remorque)
  3. Spawn du robot dans Gazebo
  4. SLAM Toolbox pour la cartographie
  5. Trailer Control Node (téléopération GUI)
  6. RViz2 pour visualiser la carte en temps réel
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = get_package_share_directory('diff_robot')

    # ─── Fichiers ────────────────────────────────────────────────────────────
    urdf_file_path   = os.path.join(package_share, 'urdf', 'robot_remorque.urdf')
    rviz_config_path = os.path.join(package_share, 'urdf', 'rviz.rviz')
    world_file_path  = os.path.join(package_share, 'world', 'silverstone_track.world')
    gazebo_models    = os.path.join(package_share, 'world', 'models')
    
    # Path of parent folder containing packages to help Gazebo find meshes
    parent_share_dir = os.path.dirname(package_share)
    gazebo_model_path_val = gazebo_models + ":" + parent_share_dir

    # ─── Charger le URDF ─────────────────────────────────────────────────────
    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    # ─── Arguments ───────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    return LaunchDescription([
        # ─── Arguments ───────────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Launch trailer control GUI'),

        # ─── Gazebo Environment ────────────────────────────────────────────────
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path_val),

        # ── 1. Gazebo Server & Client ─────────────────────────────────────────
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file_path,
            ],
            output='screen'),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        # ── 2. Robot State Publisher ─────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }]),

        # ── 3. Spawn Robot ───────────────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file_path,
                    '-entity', 'robot_remorque',
                    '-x', '-0.142601',
                    '-y', '-2.065040',
                    '-z', '0.10',
                    '-Y', '-0.062120',
                ],
                output='screen'),
        ]),

        # ── 4. EKF Local (odom -> base_footprint) — DÉSACTIVÉ pour SLAM ────
        # SLAM Toolbox publie directement map -> odom TF
        # Pas besoin de fusion odométrie locale pour SLAM
        # TimerAction(period=8.0, actions=[
        #     Node(
        #         package='robot_localization',
        #         executable='ekf_node',
        #         name='ekf_filter_node_odom',
        #         output='screen',
        #         parameters=[
        #             PathJoinSubstitution([package_share, 'config', 'slam_ekf.yaml']),
        #             {'use_sim_time': use_sim_time},
        #         ],
        #         remappings=[
        #             ('odometry/filtered', 'odometry/local'),
        #         ]),
        # ]),

        # ── 4.5. Trailer Joint Publisher (beta angle) ───────────────────────
        TimerAction(period=9.0, actions=[
            Node(
                package='diff_robot',
                executable='trailer_joint_publisher.py',
                name='trailer_joint_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'hitch_joint_name': 'hitch_joint',
                    'beta_topic': '/trailer/beta',
                }],
            ),
        ]),

        # ── 5. SLAM Toolbox ────────────────────────────────────────────────────
        # SLAM Toolbox will publish map -> odom TF
        TimerAction(period=10.0, actions=[
            Node(
                parameters=[
                  {'use_sim_time': use_sim_time},
                  {'map_frame': 'map'},
                  {'odom_frame': 'odom'},
                  {'base_frame': 'base_footprint'},
                  {'scan_topic': '/scan'},
                  {'use_scan_matching': True},
                  {'use_scan_barycenter': True},
                  {'minimum_travel_distance': 0.3},
                  {'minimum_travel_heading': 0.3},
                  {'scan_buffer_size': 30},  # Increased from 10 to 30 to prevent queue full
                  {'scan_buffer_maximum_scan_distance': 10.0},
                  {'link_scan_maximum_distance': 10.0},
                  {'do_loop_closing': True},
                  {'loop_search_maximum_distance': 3.0},
                  {'resolution': 0.05},
                  {'max_laser_range': 10.0},
                  {'transform_publish_period': 0.05},
                  {'map_update_interval': 5.0},
                ],
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen'),
        ]),

        # ── 5. Odometry Publisher (joint-based - bypasses Gazebo physics limitation) ─────
        TimerAction(period=11.0, actions=[
            Node(
                package='trailer_kinematics',
                executable='odometry_publisher',
                name='odometry_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'wheel_separation': 0.402,
                    'wheel_radius': 0.085,
                    'hitch_distance': 0.50,
                    'publish_rate': 50.0,
                    'publish_tf': True,  # Required for SLAM Toolbox (odom->base_footprint)
                }],
            ),
        ]),

        # ── 5.5. Trailer Control Node (Teleoperation GUI) ─────────────────────
        TimerAction(period=12.0, actions=[
            Node(
                package='diff_robot',
                executable='trailer_control_node.py',
                name='trailer_control_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'cmd_vel_topic': '/cmd_vel_raw',
                    'max_stationary_omega': 3.0,
                }],
                additional_env={
                    'QT_QPA_PLATFORM': 'xcb',
                    'DISPLAY': ':0',
                },
            ),
        ]),

        # ── 5.5. Cmd Vel Safety Node (Anti-jackknife) ─────────────────────
        TimerAction(period=12.5, actions=[
            Node(
                package='diff_robot',
                executable='cmd_vel_safety_node.py',
                name='cmd_vel_safety_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'hitch_distance': 0.50,
                    'beta_limit_deg': 45.0,
                    'max_forward_speed': 0.26,
                    'max_backward_speed': 0.15,
                    'max_stationary_omega': 5.0,
                    'input_topic': '/cmd_vel_raw',
                    'output_topic': '/cmd_vel',
                }],
            ),
        ]),

        # ── 6. RViz2 ─────────────────────────────────────────────────────────
        TimerAction(period=15.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}],
                additional_env={
                    'QT_QPA_PLATFORM': 'xcb',
                    'DISPLAY': ':0',
                },
                condition=IfCondition(use_rviz)),
        ]),
    ])
