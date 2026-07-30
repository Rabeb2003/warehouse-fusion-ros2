import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('diff_robot')

    # File paths
    urdf_file_path = os.path.join(package_share, 'urdf', 'diff_robot.urdf')
    rviz_config_file_path = os.path.join(package_share, 'urdf', 'rviz.rviz')
    world_file_path = os.path.join(package_share, 'world', 'silverstone_track.world')
    gazebo_models_path = os.path.join(package_share, 'world', 'models')

    # Load robot description from URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file_path,
            description='Absolute path to robot URDF file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file_path,
            description='Absolute path to RViz config file'),
        DeclareLaunchArgument(
            name='world',
            default_value=world_file_path,
            description='Absolute path to world file'),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Start RViz with the robot launch'),
        DeclareLaunchArgument(
            name='x', default_value='-0.142601', description='Initial x position of the robot'),
        DeclareLaunchArgument(
            name='y', default_value='-2.065040', description='Initial y position of the robot'),
        DeclareLaunchArgument(
            name='z', default_value='0.150008', description='Initial z position of the robot'),
        DeclareLaunchArgument(
            name='R', default_value='-0.000005', description='Initial roll orientation of the robot'),
        DeclareLaunchArgument(
            name='P', default_value='0.000040', description='Initial pitch orientation of the robot'),
        DeclareLaunchArgument(
            name='Y', default_value='-0.062120', description='Initial yaw orientation of the robot'),

        # Gazebo launch
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_models_path),
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                LaunchConfiguration('world'),
            ],
            output='screen'),

        # Spawn the robot in Gazebo
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=[
                 '-file', LaunchConfiguration('model'),
                 '-entity', 'diff_robot',
                 '-x', LaunchConfiguration('x'),
                 '-y', LaunchConfiguration('y'),
                 '-z', LaunchConfiguration('z'),
                 '-R', LaunchConfiguration('R'),
                 '-P', LaunchConfiguration('P'),
                 '-Y', LaunchConfiguration('Y')],
             output='screen'),

        # Publish the robot state (robot_description topic)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[
                 {'robot_description': robot_desc},
                 {'use_sim_time': True},
             ],
             remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")]),

        # Launch RViz
        Node(package='rviz2', executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
