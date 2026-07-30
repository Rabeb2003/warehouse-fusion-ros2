#!/usr/bin/env python3
"""
Robot Remorque — Launch file pour ros2_diff_drive_robot
========================================================
Lance :
  1. Gazebo avec le world existant
  2. Robot State Publisher (nouveau URDF robot-remorque)
  3. Spawn du robot dans Gazebo
  4. GPS Localization (dual EKF + navsat_transform via robot_localization)
  5. Beta EKF Node (estimation angle attelage β)
  6. Multi-Circle Footprint Node (footprint dynamique Nav2)
  7. Hitch Angle Stabilizer (filtre CBF anti-jackknifing)
  8. RViz2 (optionnel)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    rviz         = LaunchConfiguration('rviz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation time'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),

        # Gazebo model path
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path_val),

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

        # ── 2. Robot State Publisher ────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True},
                {'publish_frequency': 50.0},
                {'frame_prefix': ''},
                {'use_tf_static': True},
            ]),

        # ── 2b. Trailer Joint Publisher (publishes all joint states) ─────────
        Node(
            package='diff_robot',
            executable='trailer_joint_publisher.py',
            name='trailer_joint_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'beta_topic': '/trailer/beta_raw',
            }]),

        # ── 2c. Wheel Synchronization Node ─ DÉSACTIVÉ (cause résistance) ───
        # Node(package='diff_robot', executable='wheel_sync_node.py', ...)

        # ── 3. Spawn robot (après 5s) ───────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file_path,
                    '-entity', 'robot_remorque',
                    '-x', '-0.142601',
                    '-y', '-2.065040',
                    '-z', '0.02',
                    '-Y', '-0.062120',
                ],
                output='screen'),
        ]),

        # ── 4. GPS Localization (dual EKF + navsat_transform) ──────────────
        # Lance ekf_filter_node_odom (6s), navsat_transform (6.5s),
        # ekf_filter_node_map (7s) — timing interne au fichier inclus
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'gps_localization.launch.py')
            ),
        ),

        # ── 5. Beta EKF Node (après 8s) ────────────────────────────────────
        TimerAction(period=8.0, actions=[
            Node(
                package='trailer_kinematics',
                executable='beta_ekf_node',
                name='beta_ekf_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'hitch_distance': 0.50,
                    'wheel_separation': 0.40,
                    'max_wheel_speed': 1.0,
                    'beta_limit_deg': 45.0,
                }]),
        ]),

        # ── 5. Multi-Circle Footprint Node (après 8.5s) ─────────────────────
        TimerAction(period=8.5, actions=[
            Node(
                package='trailer_kinematics',
                executable='multi_circle_footprint_node',
                name='multi_circle_footprint_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'hitch_distance': 0.50,
                    'robot_length': 0.6,
                    'robot_width': 0.4,
                    'trailer_length': 0.8,
                    'trailer_width': 0.5,
                    'tractor_center_x': -0.025,
                    'tractor_center_y': 0.041,
                    'hitch_offset_x': -0.3575,
                    'hitch_offset_y': 0.0414,
                    'num_circles_tractor': 3,
                    'num_circles_trailer': 3,
                    'footprint_margin': 0.03,
                    'cost_threshold': 254,  # Maximum - désactive pratiquement les avertissements
                }]),
        ]),

        # ── 6. Hitch Angle Stabilizer ─ DÉSACTIVÉ (filtrait /cmd_vel) ────────
        # Le stabilisateur est désactivé pour permettre le mouvement libre.
        # Réactiver uniquement en mode navigation autonome.
        # TimerAction(period=9.0, actions=[Node(package='trailer_kinematics', ...)])

        # ── 7. RViz2 ────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            additional_env={
                'QT_QPA_PLATFORM': 'xcb',
                'DISPLAY': ':0',
            },
            condition=IfCondition(rviz)),
    ])
