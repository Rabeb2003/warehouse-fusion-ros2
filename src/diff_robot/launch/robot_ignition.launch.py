import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('diff_robot')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    # ---- Chemins des ressources ----
    urdf_file = os.path.join(pkg, 'urdf', 'diff_robot.urdf')
    world_file = os.path.join(pkg, 'world', 'silverstone_track.world')
    rviz_config = os.path.join(pkg, 'urdf', 'rviz.rviz')

    # Lecture du URDF (pas xacro ici, fichier .urdf direct)
    with open(urdf_file, 'r') as f:
        robot_description_xml = f.read()

    # ---- Arguments ----
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='-0.14')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='-2.07')

    # ================================================================
    # 1. Gazebo Ignition (Harmonic / Fortress)
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
        }],
    )

    # joint_state_publisher : combine JSB (roues actives) + joints fixes
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
    # 3. Spawn du robot dans Gazebo Ignition
    # ================================================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_robot',
        output='screen',
        arguments=[
            '-name',  'diff_robot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.15',
        ],
    )

    # ================================================================
    # 4. Contrôleurs ros2_control (chaîne séquentielle après spawn)
    # ================================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='jsb_spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_ctrl_spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Séquençage : spawn -> JSB -> DiffDrive
    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # ================================================================
    # 5. Ponts ROS <-> Gazebo Ignition (ros_gz_bridge)
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

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen',
    )

    # Pont TF : Ignition publie les TFs odom->base_footprint via OdometryPublisher
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen',
    )

    # ================================================================
    # 6. TFs statiques pour les capteurs
    # ================================================================
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['0', '0', '0.225', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen',
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen',
    )

    # ================================================================
    # 7. RViz2
    # ================================================================
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        use_rviz_arg,
        x_pose_arg,
        y_pose_arg,

        # Infrastructure
        gazebo,
        robot_state_publisher,
        joint_state_publisher,

        # Ponts GZ <-> ROS
        clock_bridge,
        scan_bridge,
        imu_bridge,
        odom_bridge,
        tf_bridge,

        # Spawn + contrôleurs (chaîne séquentielle)
        spawn_entity,
        load_jsb,
        load_diff_ctrl,

        # TFs statiques
        static_tf_lidar,
        static_tf_imu,

        # Visualisation
        rviz2,

        LogInfo(msg='=== Diff Robot Gazebo Ignition Stack Started ==='),
        LogInfo(msg='Téléopération: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
    ])
