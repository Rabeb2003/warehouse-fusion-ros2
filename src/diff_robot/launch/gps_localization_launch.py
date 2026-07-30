from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot')  # adapted for diff_robot package

    world_path = os.path.join(pkg_share, 'world', 'silverstone_track.world')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'gps_ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'gps_simple.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_remorque.urdf')
    urdf_meshes_path = os.path.join(pkg_share, 'urdf', 'meshes')

    # Charger le URDF et traiter les inclusions xacro
    # Mapping nécessaire pour résoudre $(find diff_robot) dans les xacro:include
    doc = xacro.process_file(urdf_path, mappings={'find diff_robot': pkg_share})
    urdf_content = doc.toxml()
    # DEBUG: Vérifier si ros2_control est présent
    print(f"DEBUG: urdf_content length: {len(urdf_content)}")
    print(f"DEBUG: urdf_content contains ros2_control: {'ros2_control' in urdf_content}")
    print(f"DEBUG: urdf_content contains xacro:include: {'xacro:include' in urdf_content}")
    # Remplacer package://diff_robot/urdf/meshes/ par file://chemin_absolu
    urdf_content = urdf_content.replace('package://diff_robot/urdf/meshes/', f'file://{urdf_meshes_path}/')

    # 1. Simulateur Gazebo Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. robot_state_publisher : publie robot_description et TF statiques
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}],
        output='screen'
    )

    # 3. joint_state_publisher : publie les états des joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. Pont ROS 2 <-> Gazebo : doit démarrer tôt, les EKF en dépendent
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 5. Spawn du robot dans Gazebo Fortress (remplace spawn_entity.py de Classic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'tractor_trailer', '-z', '0.1'],
        output='screen'
    )

    # 6. Controller Manager et contrôleurs (après spawn du robot)
    # DÉSACTIVÉ - gz_ros2_control/ign_ros2_control ne fonctionne pas avec Gazebo Fortress
    # Utilisation de l'odométrie Gazebo directement via le bridge
    # spawn_controllers = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )

    # spawn_diff_drive = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller'],
    #     output='screen'
    # )

    # 7. EKF locale (odom -> base_footprint)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 8. navsat_transform_node : doit démarrer APRÈS que le GPS ait eu le
    #    temps de stabiliser sa covariance (cf. bruit gaussien non nul
    #    ajouté au capteur, fichier 2/8) -> décalage volontaire
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', 'gps/fix'),
            ('imu', 'imu'),
            ('odometry/filtered', 'odometry/global'),
        ]
    )

    # 9. EKF globale (map -> odom), démarre après navsat_transform
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # 10. RViz, en dernier, une fois la TF map->odom->base_footprint stable
    # Désactivé temporairement - RViz crash
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen'
    # )

    return LaunchDescription([
        # GAZEBO_MODEL_PATH pour que Gazebo trouve les meshes
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', urdf_meshes_path),
        gazebo,
        TimerAction(period=1.0, actions=[robot_state_publisher]),
        TimerAction(period=1.5, actions=[joint_state_publisher]),
        TimerAction(period=2.0, actions=[bridge]),
        TimerAction(period=3.0, actions=[spawn_robot]),
        # ros2_control désactivé - utilisation odométrie Gazebo
        TimerAction(period=4.0, actions=[ekf_local]),
        TimerAction(period=4.5, actions=[navsat_transform]),
        TimerAction(period=5.0, actions=[ekf_global]),
        # RViz désactivé
    ])
