#!/usr/bin/env python3
"""
Full Mission Launch File — Warehouse Fusion Workspace
=====================================================
Launches the entire integrated simulation:
  1. Gazebo Fortress with warehouse_merged.sdf
  2. Franka Panda arm + MoveIt2 + Object Vision Detector (via ex_pick_and_place.launch.py)
  3. Static TF publisher: world -> map (Phase 2 reconciliation)
  4. Articulated Trailer Robot + Sensors + ros2_control + Kinematics + Nav2 Stack
  5. Mission Orchestrator node to coordinate waypoint navigation & loading task
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    diff_robot_share = get_package_share_directory('diff_robot')
    panda_config_share = get_package_share_directory('panda_moveit_config')
    merged_world_path = os.path.join(diff_robot_share, 'world', 'warehouse_merged.sdf')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_rviz_arg = DeclareLaunchArgument('enable_rviz', default_value='true')

    # ================================================================
    # 1. Static TF Publisher: world -> map (Phase 2)
    #    Connects Panda's world root frame with Nav2's map frame.
    # ================================================================
    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ================================================================
    # 2. Panda Arm + MoveIt 2 + Gazebo World (ex_pick_and_place.launch.py)
    # ================================================================
    panda_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(panda_config_share, 'launch', 'ex_pick_and_place.launch.py')
        ),
        launch_arguments={
            'world': merged_world_path,
            'enable_pick_place': 'true',
            'pick_place_start_delay': '30.0',
            'use_sim_time': 'true',
        }.items(),
    )

    # ================================================================
    # 3. Trailer Robot Navigation Stack
    # ================================================================
    # Trailer URDF & Nodes
    urdf_file = os.path.join(diff_robot_share, 'urdf', 'robot_remorque.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc_xml = f.read()

    trailer_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='trailer_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
    )

    trailer_jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='trailer_joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time': True,
            'source_list': ['/joint_state_broadcaster/joint_states'],
        }],
    )

    # Bridges for Trailer sensors
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

    # Spawn Trailer robot in merged Gazebo world
    spawn_trailer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot_remorque',
                output='screen',
                arguments=[
                    '-name', 'robot_remorque',
                    '-topic', 'robot_description',
                    '-x', '-0.14',
                    '-y', '-2.07',
                    '-z', '0.10',
                ],
            ),
        ]
    )

    # Spawner for trailer diff_drive_controller
    trailer_controller_spawner = TimerAction(
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

    # Trailer Kinematics Nodes
    trailer_joint_pub = Node(
        package='diff_robot',
        executable='trailer_joint_publisher.py',
        name='trailer_joint_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'beta_topic': '/trailer/beta_raw'}],
    )

    beta_ekf_node = Node(
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
            'cost_threshold': 254,
        }],
    )

    # Nav2 Stack for Trailer
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_robot_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': 'false',
        }.items(),
    )

    # ================================================================
    # 4. Mission Orchestrator Node (Phase 5)
    #    Launches after 20 seconds (when Nav2 and TF tree are active).
    # ================================================================
    mission_orchestrator_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='warehouse_orchestrator',
                executable='mission_orchestrator',
                name='mission_orchestrator',
                output='screen',
                parameters=[{
                    'dock_x': -0.2425,
                    'dock_y': 4.00,
                    'dock_yaw': 0.0,
                    'max_dock_distance': 0.50,
                    'target_drop_x': -1.0,
                    'target_drop_y': 4.0,
                }],
            ),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_rviz_arg,

        # 1. Panda Stack & Gazebo World
        panda_stack,

        # 2. Static TF world -> map
        static_world_to_map,

        # 3. Trailer Stack
        trailer_rsp,
        trailer_jsp,
        scan_bridge,
        imu_bridge,
        spawn_trailer,
        trailer_controller_spawner,
        trailer_joint_pub,
        beta_ekf_node,
        multi_circle_footprint,
        nav2_stack,

        # 4. Mission Orchestrator
        mission_orchestrator_node,

        LogInfo(msg='=== Full Warehouse Fusion Mission Launched Successfully ==='),
    ])
