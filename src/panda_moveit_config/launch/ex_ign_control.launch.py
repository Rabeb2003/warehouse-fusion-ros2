#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 and executing motions using fake ROS 2 controllers within RViz2"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("panda_description"), ".."]),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("panda_description"), ".."]),
        ),
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                (
                    "gz_args",
                    [
                        world,
                        PythonExpression(
                            ["' -s --headless-rendering' if '", headless, "' == 'true' else ''"]
                        ),
                        " -r -v ",
                        ign_verbosity,
                    ],
                ),
                (
                    "ign_args",
                    [
                        world,
                        PythonExpression(
                            ["' -s --headless-rendering' if '", headless, "' == 'true' else ''"]
                        ),
                        " -r -v ",
                        ign_verbosity,
                    ],
                ),
            ],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("panda_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("ros2_control_plugin", "ign"),
                ("ros2_control_command_interface", "position"),
                ("robot_state_publisher_name", "panda_robot_state_publisher"),
                ("robot_description_topic", "/panda/robot_description"),
                # TODO: Re-enable colligion geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "false"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # ros_ign_bridge (clock -> ROS 2)
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_ign_gazebo_create waits until robot_state_publisher has published robot_description
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_ign_gazebo",
                    executable="create",
                    output="log",
                    arguments=[
                        "-topic",
                        "/panda/robot_description",
                        "-name",
                        model,
                        "--ros-args",
                        "--log-level",
                        log_level,
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("panda_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run only the Gazebo server with headless rendering.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
