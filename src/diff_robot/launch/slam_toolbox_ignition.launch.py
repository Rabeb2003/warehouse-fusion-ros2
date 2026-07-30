#!/usr/bin/env python3
"""
SLAM Toolbox Ignition — Launch file pour robot-remorque sous Gazebo Sim (Ignition)
================================================================================
Cette configuration lance la cartographie (SLAM Toolbox) avec la remorque.
L'EKF global (GPS) est désactivé, seul l'EKF local (odom -> base_footprint) tourne.
SLAM Toolbox se charge de publier la transformée map -> odom.
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('diff_robot')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    # ---- Chemins ----
    urdf_file  = os.path.join(pkg, 'urdf', 'robot_remorque.urdf')
    world_file = os.path.join(pkg, 'world', 'warehouse.world')
    rviz_config = os.path.join(pkg, 'urdf', 'rviz.rviz')
    ekf_config_path = os.path.join(pkg, 'config', 'gps_ekf.yaml')
    slam_params_path = os.path.join(pkg, 'map', 'slam_params.yaml')

    # ---- Variables d'environnement Gazebo (meshes) ----
    share_dir = os.path.dirname(pkg)
    for env_var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        if env_var in os.environ:
            os.environ[env_var] += ':' + share_dir
        else:
            os.environ[env_var] = share_dir

    # ---- Compilation URDF ----
    robot_description_xml = xacro.process_file(urdf_file).toxml()

    # ---- Arguments ----
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    x_pose_arg   = DeclareLaunchArgument('x_pose',   default_value='-0.14')
    y_pose_arg   = DeclareLaunchArgument('y_pose',   default_value='-2.07')

    # ================================================================
    # 1. Gazebo Ignition
    # ================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_file, ' -r -v 1'],
        }.items(),
    )

    # ================================================================
    # 2. Robot State Publisher
    # ================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'ignore_timestamp': False,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'source_list': ['/joint_state_broadcaster/joint_states'],
        }],
    )

    # ================================================================
    # 3. Ponts ros_gz_bridge
    # ================================================================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
    )

    # ================================================================
    # 4. Spawn du robot (après 3s)
    # ================================================================
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot_remorque',
                output='screen',
                arguments=[
                    '-name',  'robot_remorque',
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('x_pose'),
                    '-y', LaunchConfiguration('y_pose'),
                    '-z', '0.10',
                ],
            ),
        ]
    )

    # ================================================================
    # 5. Contrôleurs ros2_control
    # ================================================================
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='jsb_spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            ),
        ]
    )

    diff_drive_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='diff_ctrl_spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            ),
        ]
    )

    # ================================================================
    # 6. EKF local uniquement (odom -> base_footprint)
    #    Indispensable pour fournir l'odométrie à SLAM Toolbox.
    # ================================================================
    local_ekf = TimerAction(
        period=15.0,
        actions=[
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
        ]
    )

    # ================================================================
    # 7. Nœuds métier remorque
    # ================================================================
    trailer_joint_publisher = Node(
        package='diff_robot',
        executable='trailer_joint_publisher.py',
        name='trailer_joint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'beta_topic': '/trailer/beta_raw',
        }],
    )

    beta_ekf = Node(
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
        }],
    )

    multi_circle_footprint = Node(
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
        }],
    )

    # ================================================================
    # 8. SLAM Toolbox (publie map -> odom)
    # ================================================================
    slam_toolbox = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_path,
                    {'use_sim_time': True}
                ],
            )
        ]
    )

    # ================================================================
    # 9. RViz2 — démarré après 25s
    # ================================================================
    rviz2 = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        x_pose_arg,
        y_pose_arg,

        # ── Gazebo + RSP + Ponts
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        clock_bridge,
        scan_bridge,
        imu_bridge,

        # ── Séquence spawn + contrôleurs (minutés)
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,

        # ── Localisation EKF local
        local_ekf,

        # ── Nœuds remorque
        trailer_joint_publisher,
        beta_ekf,
        multi_circle_footprint,

        # ── SLAM Toolbox
        slam_toolbox,

        # ── RViz (après stabilisation complète)
        rviz2,

        LogInfo(msg='=== Robot Remorque SLAM Ignition Stack Started ==='),
        LogInfo(msg='Drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
        LogInfo(msg='Save map: ros2 run nav2_map_server map_saver_cli -f /home/rabeb/ros2_diff_drive_robot/src/diff_robot/map/my_map'),
    ])
