#!/usr/bin/env python3
"""
Trailer Navigation Launch File
Launches the trailer robot with Nav2 navigation and RViz visualization.
This is the first step in the sequential warehouse mission.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ========================================================================
    # ARGUMENTS
    # ========================================================================
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="warehouse_merged.sdf",
            description="Gazebo world file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Launch Gazebo in headless mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Launch RViz2",
        )
    )
    
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")
    
    # ========================================================================
    # PATHS
    # ========================================================================
    diff_robot_share = FindPackageShare('diff_robot')
    warehouse_orchestrator_share = FindPackageShare('warehouse_orchestrator')
    
    # ========================================================================
    # GAZEBO IGNITION
    # ========================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments=[
            ("gz_args", [
                PathJoinSubstitution([diff_robot_share, "world", world]),
                " -r -v 1",
            ]),
        ],
    )
    
    # ========================================================================
    # BRIDGES ros_gz_bridge
    # ========================================================================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )
    
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
    )
    
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
    )
    
    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_bridge',
        arguments=['/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        parameters=[{'use_sim_time': True}],
    )
    
    # Panda bridges
    panda_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='panda_bridge',
        arguments=[
            '/wrist_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/panda/attach_object@std_msgs/msg/Empty]gz.msgs.Empty',
            '/panda/detach_object@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("/wrist_camera/image_raw", "/wrist_camera/image_raw_ign"),
        ],
    )
    
    # ========================================================================
    # TRAILER ROBOT STATE PUBLISHER
    # ========================================================================
    # Load URDF
    urdf_file = os.path.join(
        FindPackageShare('diff_robot').find('diff_robot'),
        'urdf',
        'robot_remorque.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_description_xml = infp.read()
    
    trailer_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='trailer_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,
        }],
    )
    
    # ========================================================================
    # PANDA ROBOT STATE PUBLISHER
    # ========================================================================
    panda_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panda_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments=[
            ("ros2_control_plugin", "ign"),
            ("ros2_control_command_interface", "position"),
            ("robot_state_publisher_name", "panda_robot_state_publisher"),
            ("robot_description_topic", "/panda/robot_description"),
            ("origin_xyz", "-1.0 3.5 0.4"),
            ("gazebo_grasp_plugin", "true"),
            ("enable_rviz", "false"),  # No RViz here
            ("use_sim_time", use_sim_time),
            ("log_level", "info"),
        ],
    )
    
    # ========================================================================
    # STATIC TF BRIDGES (world -> map -> odom -> base_footprint)
    # ========================================================================
    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'map',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
    
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bootstrap_map_to_odom_tf',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bootstrap_odom_to_base_tf',
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint',
            '--x', '-0.140', '--y', '-2.070', '--z', '0.100',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # ========================================================================
    # SPAWN TRAILER IN GAZEBO
    # ========================================================================
    spawn_trailer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot_remorque',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'robot_remorque',
                    '-allow-rename',
                    '-x', '-0.14',
                    '-y', '-2.07',
                    '-z', '0.1',
                ],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )
    
    # ========================================================================
    # SPAWN PANDA IN GAZEBO
    # ========================================================================
    spawn_panda = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_panda',
                arguments=[
                    '-topic', '/panda/robot_description',
                    '-name', 'panda',
                    '-allow-rename',
                    '-x', '-1.0',
                    '-y', '3.5',
                    '-z', '0.4',
                ],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )
    
    # ========================================================================
    # TRAILER CONTROLLERS
    # ========================================================================
    diff_drive_spawner = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='trailer_diff_ctrl_spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
                parameters=[os.path.join(diff_robot_share.find('diff_robot'), 'config', 'diff_drive_controller.yaml')],
            ),
        ]
    )
    
    # ========================================================================
    # EKF LOCALIZATION
    # ========================================================================
    gps_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_robot_share.find('diff_robot'), 'launch', 'gps_localization.launch.py')
        ),
    )
    
    # ========================================================================
    # BUSINESS NODES
    # ========================================================================
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
    
    beta_ekf_node = Node(
        package='trailer_kinematics',
        executable='beta_ekf_node',
        name='beta_ekf_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'hitch_distance': 0.50,
            'wheel_separation': 0.40,
            'beta_limit': 45.0,
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
            'robot_length': 0.60,
            'robot_width': 0.40,
            'trailer_length': 0.80,
            'trailer_width': 0.50,
            'hitch_offset_x': -0.357,
            'hitch_offset_y': 0.041,
            'num_circles_tractor': 3,
            'num_circles_trailer': 3,
            'circle_radius': 0.28,
        }],
    )
    
    # ========================================================================
    # PANDA BUSINESS NODES
    # ========================================================================
    object_detector = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='panda_moveit_config',
                executable='object_detector.py',
                name='object_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }],
            ),
        ]
    )
    
    planning_scene_setup = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='panda_moveit_config',
                executable='planning_scene_setup.py',
                name='planning_scene_setup',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }],
            ),
        ]
    )
    
    # ========================================================================
    # NAV2 STACK
    # ========================================================================
    nav2_stack = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diff_robot_share.find('diff_robot'), 'launch', 'nav2.launch.py')
                ),
            ),
        ]
    )
    
    # ========================================================================
    # RVIZ2
    # ========================================================================
    rviz_config = os.path.join(
        diff_robot_share.find('diff_robot'),
        'rviz',
        'nav2_trailer_gps.rviz'
    )
    
    rviz2_trailer = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_trailer',
                arguments=['--display-config', rviz_config],
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg='=== Unified Gazebo Launch with Separate RViz ==='),
            LogInfo(msg='=== Trailer + Panda in Gazebo | Separate RViz for each ==='),
            LogInfo(msg='=== Trailer spawn at t=8s | Panda spawn at t=10s ==='),
            LogInfo(msg='=== Nav2 at t=20s | Trailer RViz at t=25s ==='),
            gazebo_launch,
            clock_bridge,
            scan_bridge,
            imu_bridge,
            gps_bridge,
            panda_bridge,
            static_world_to_map,
            static_map_to_odom,
            static_odom_to_base,
            trailer_rsp,
            panda_rsp,
            trailer_joint_publisher,
            beta_ekf_node,
            multi_circle_footprint,
            gps_localization,
            spawn_trailer,
            spawn_panda,
            diff_drive_spawner,
            object_detector,
            planning_scene_setup,
            nav2_stack,
            rviz2_trailer,
        ]
    )


def IfCondition(condition):
    """Helper function for conditional launch"""
    from launch.conditions import IfCondition as IC
    return IC(condition)
