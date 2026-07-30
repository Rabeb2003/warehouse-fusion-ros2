#!/usr/bin/env python3
"""
Robot Remorque Ignition — Launch file stabilisé pour RViz2
============================================================
Corrections v2 :
  - joint_state_publisher : agrège JSB + joints passifs (remorque, hitch, etc.)
  - RViz démarré après 25s (horloge Gazebo et contrôleurs stabilisés)
  - TF statique temporaire map->odom le temps que l'EKF démarre
  - Séquencement : spawn -> JSB -> DiffDrive -> RViz
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
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

    # ---- Variables d'environnement Gazebo (pour trouver les meshes) ----
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
    #    NOTE: PAS de joint_state_publisher séparé !
    #    Le joint_state_broadcaster de ros2_control publie sur
    #    /joint_state_broadcaster/joint_states ET /joint_states.
    #    Un JSP concurrent crée un conflit qui fait clignoter le robot.
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
    # 3. Ponts ros_gz_bridge (démarrage immédiat)
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

    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_bridge',
        arguments=['/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        output='screen',
    )

    # ================================================================
    # 4. Spawn du robot (après 3s pour laisser Gazebo démarrer)
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
    # 5. Contrôleurs ros2_control — séquence: JSB puis DiffDrive
    #    Démarrage après 8s (temps que Gazebo charge le robot + plugins)
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
    # 6. TF statique temporaire map -> odom
    #    Actif pendant que l'EKF charge (15-19s).
    #    Evite que RViz affiche "No transform from X to map" en boucle.
    #    IMPORTANT : l'EKF global (ekf_filter_node_map) prend le relais.
    #    Pour éviter un conflit TF permanent (double-publication), 
    #    nous désactivons ce nœud statique.
    # ================================================================
    # static_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_odom_tf',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen',
    # )

    # ================================================================
    # 7. Localisation EKF (délais augmentés pour éviter jump-back-in-time)
    # ================================================================
    gps_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gps_localization.launch.py')
        ),
    )

    # ================================================================
    # 8. Nœuds métier remorque
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
    # 9. RViz2 — démarré après 25s
    #    Raison: l'horloge Gazebo met ~5s à stabiliser, les contrôleurs
    #    ~10s, et le premier EKF ~15s. Démarrer RViz trop tôt provoque
    #    des TF resets en cascade ("jump back in time").
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
        gps_bridge,

        # ── TF map->odom statique temporaire (désactivé pour éviter conflits avec EKF)
        # static_map_odom,

        # ── Séquence spawn + contrôleurs (minutés)
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,

        # ── Localisation EKF
        gps_localization,

        # ── Nœuds remorque
        trailer_joint_publisher,
        beta_ekf,
        multi_circle_footprint,

        # ── RViz (après stabilisation complète)
        rviz2,

        LogInfo(msg='=== Robot Remorque Ignition Stack Started ==='),
        LogInfo(msg='=== RViz2 will start in ~25 seconds (clock stabilization) ==='),
    ])
