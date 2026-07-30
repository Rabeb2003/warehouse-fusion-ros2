#!/usr/bin/env python3
"""
GPS Localization — Launch file pour robot-remorque différentiel
=================================================================
Lance :
  1. ekf_filter_node_odom   -> TF odom -> base_footprint  (fusion roues + IMU)
  2. navsat_transform_node  -> convertit GPS (LLA) en cartésien
  3. ekf_filter_node_map    -> TF map -> odom              (fusion + GPS)

PRÉREQUIS :
  - Le plugin diff_drive dans l'URDF a publish_odom_tf: false
  - Le plugin IMU publie sur /imu/data
  - Le plugin GPS publie sur /gps/fix
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('diff_robot')
    ekf_config_path = os.path.join(package_share, 'config', 'gps_ekf.yaml')

    return LaunchDescription([

        # ── 1. EKF local (odom -> base_footprint) ───────────────────────
        # Délai 15s : laisser Gazebo stabiliser son horloge avant que l'EKF
        # commence à recevoir des messages — évite les "jump back in time"
        TimerAction(period=15.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_odom',
                output='screen',
                parameters=[ekf_config_path, {
                    'use_sim_time': True,
                    'smooth_lagged_data': True,
                    'history_length': 2.0,
                }],
                remappings=[
                    ('odometry/filtered', 'odometry/local'),
                    ('odom', '/diff_drive_controller/odom'),
                ]),
        ]),

        # ── 2. navsat_transform (GPS -> cartésien) ──────────────────────
        TimerAction(period=17.0, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                parameters=[ekf_config_path, {'use_sim_time': True}],
                remappings=[
                    ('imu', '/imu'),
                    ('gps/fix', '/gps/fix'),
                    ('gps/filtered', '/gps/filtered'),
                    ('odometry/gps', '/odometry/gps'),
                    ('odometry/filtered', '/odometry/local'),
                ]),
        ]),

        # ── 3. EKF global (map -> odom) ──────────────────────────────────
        # Démarré après navsat_transform pour que /odometry/gps existe déjà
        TimerAction(period=19.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_map',
                output='screen',
                parameters=[ekf_config_path, {
                    'use_sim_time': True,
                    'smooth_lagged_data': True,
                    'history_length': 2.0,
                }],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('odom', '/diff_drive_controller/odom'),
                ]),
        ]),
    ])
