#!/usr/bin/env -S ros2 launch
"""
Pick and Place Full Simulation Launch File
Launches Gazebo Fortress with warehouse world, Franka Panda arm, MoveIt 2, RViz2, and Pick and Place Demo node.
Sequenced with TimerAction to establish /clock bridge BEFORE launching MoveIt 2 & RViz2 to prevent TF buffer time jumps.
"""

from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_pick_place = LaunchConfiguration("enable_pick_place")
    pick_place_start_delay = LaunchConfiguration("pick_place_start_delay")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    env_actions = [
        SetEnvironmentVariable("OGRE_RTT_MODE", "Copy"),
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("panda_description"), ".."]),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("panda_description"), ".."]),
        ),
    ]

    # 1. Launch Ignition Gazebo with warehouse world
    gazebo_launch = IncludeLaunchDescription(
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
    )

    # 2. Clock & Sensor Parameter Bridge
    bridge_node = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/wrist_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/panda/attach_object@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/panda/detach_object@std_msgs/msg/Empty]ignition.msgs.Empty",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/wrist_camera/image_raw", "/wrist_camera/image_raw_ign"),
        ],
    )

    # 3. Launch MoveIt 2, robot_state_publisher, controllers and RViz.
    moveit_rviz_launch = TimerAction(
        period=2.0,
        actions=[
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
                    ("origin_xyz", "-1.0 3.5 0.4"),
                    ("gazebo_grasp_plugin", "true"),
                    ("rviz_config", rviz_config),
                    ("enable_rviz", enable_rviz),
                    ("use_sim_time", use_sim_time),
                    ("log_level", log_level),
                ],
            ),
        ],
    )

    # 4. Spawn the same robot_description in Gazebo after it is published by robot_state_publisher.
    robot_spawner = TimerAction(
        period=5.0,
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
                    "-x",
                    "-1.0",
                    "-y",
                    "3.5",
                    "-z",
                    "0.4",
                    "-allow-rename",
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ],
    )

    # 5. Vision Object Detector Node
    object_detector_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="panda_moveit_config",
                executable="object_detector.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "image_topic": "/wrist_camera/image_raw_ign",
                        "rgb_image_topic": "/wrist_camera/image_raw",
                        "world_x_min": -0.80,
                        "world_x_max": -0.20,
                        "world_y_min": 3.10,
                        "world_y_max": 3.90,
                        "world_z_min": 0.35,
                        "world_z_max": 0.55,
                        "max_pose_jump": 0.08,
                    }
                ],
            ),
        ],
    )

    # 6. Planning Scene Setup Node (for RViz visualization)
    planning_scene_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="panda_moveit_config",
                executable="planning_scene_setup.py",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ],
    )

    # 7. Pick & Place Demo Node
    pnp_node = TimerAction(
        period=pick_place_start_delay,
        actions=[
            Node(
                package="panda_moveit_config",
                executable="pick_and_place_demo.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "vision_required": True,
                        "attach_max_distance": 0.12,
                        "finger_center_max_distance": 0.10,
                        "target_half_width": 0.025,
                    }
                ],
                condition=IfCondition(enable_pick_place),
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + env_actions
        + [
            gazebo_launch,
            bridge_node,
            moveit_rviz_launch,
            robot_spawner,
            object_detector_node,
            planning_scene_node,
            pnp_node,
        ]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("panda_description"),
                    "panda",
                    "worlds",
                    "warehouse.sdf",
                ]
            ),
            description="Path to Gazebo SDF world file.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name of the spawned robot model.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("panda_moveit_config"),
                    "rviz",
                    "moveit.rviz",
                ]
            ),
            description="Path to RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Enable RViz2 visualization window.",
        ),
        DeclareLaunchArgument(
            "enable_pick_place",
            default_value="true",
            description="Run the autonomous pick and place demo after controllers are available.",
        ),
        DeclareLaunchArgument(
            "pick_place_start_delay",
            default_value="40.0",
            description="Seconds to wait before starting the autonomous pick and place node.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run only the Gazebo server with headless rendering.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Gazebo verbosity (0-4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 log level.",
        ),
    ]
