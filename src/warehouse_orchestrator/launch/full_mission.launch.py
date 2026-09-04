#!/usr/bin/env python3
"""
Full Mission Launch File — Warehouse Fusion Workspace
======================================================
Fussionne exactement la logique des deux launches originaux du projet trailer :
  1. robot_remorque_ignition.launch.py  — Gazebo + RSP + bridges + spawn + controllers + EKF + kinematics
  2. nav2.launch.py                     — Nav2 stack complet (map_server, planner, controller, BT, safety)

Et y ajoute :
  3. ex_pick_and_place.launch.py — Panda arm + MoveIt2 (Gazebo déjà lancé, il ne recrée pas Ignition)
  4. TF statique world -> map         — Pont entre le repère racine du Panda (world) et le repère Nav2 (map)
  5. Mission Orchestrator             — Synchronise navigation trailer + tâche pick-and-place Panda

Séquencement critique (temps relatifs au lancement) :
   t=0s  : Gazebo Fortress (via Panda stack)
   t=0s  : Trailer RSP, bridges clock/scan/imu/gps, TF world->map
   t=3s  : Panda arm spawné dans Gazebo (via ex_pick_and_place)
   t=8s  : Trailer spawné dans Gazebo (ros_gz_sim create -topic /trailer_robot_description)
   t=14s : joint_state_broadcaster spawné (trailer)
   t=16s : diff_drive_controller spawné (trailer)
   t=15s : EKF local odom démarre (robot_localization)
   t=17s : navsat_transform démarre
   t=19s : EKF global map démarre — TF complet: world->map->odom->base_footprint
   t=20s : Nav2 stack (map_server, planner, controller, BT, safety nodes)
   t=25s : RViz2 fusionné (Panda + Trailer)
   t=55s : Mission Orchestrator (envoie 1er goal Nav2 au trailer)
"""

import os
import xacro
import yaml
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_share: str, relpath: str):
    with open(os.path.join(package_share, relpath), 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ── Chemins des packages ─────────────────────────────────────────────────
    diff_robot_share         = get_package_share_directory('diff_robot')
    panda_config_share       = get_package_share_directory('panda_moveit_config')
    warehouse_orchestrator_share = get_package_share_directory('warehouse_orchestrator')

    # Le monde unifié (entrepôt + station Panda)
    merged_world_path   = os.path.join(diff_robot_share, 'world', 'warehouse_merged.sdf')

    # URDF trailer — compilé via xacro (comme dans robot_remorque_ignition.launch.py original)
    urdf_file           = os.path.join(diff_robot_share, 'urdf', 'robot_remorque.urdf')
    robot_description_xml = xacro.process_file(urdf_file).toxml()

    # Fichiers de configuration Nav2
    nav2_params_file    = os.path.join(diff_robot_share, 'config', 'nav2_params.yaml')
    trailer_params_file = os.path.join(diff_robot_share, 'config', 'trailer_params.yaml')
    diff_drive_params_file = os.path.join(diff_robot_share, 'config', 'diff_drive_controller.yaml')
    default_map         = os.path.join(diff_robot_share, 'map', 'my_map.yaml')
    default_bt_xml      = os.path.join(diff_robot_share, 'config', 'navigate_to_pose_trailer_recovery.xml')

    # RViz fusionné (Panda + Trailer + Nav2) — fichier dédié warehouse_orchestrator
    rviz_config_fusion  = os.path.join(warehouse_orchestrator_share, 'rviz', 'fusion_mission.rviz')

    # RViz Panda MoveIt — fichier dédié panda_moveit_config
    rviz_config_panda   = os.path.join(panda_config_share, 'rviz', 'moveit.rviz')

    # ── Variables d'environnement Gazebo (meshes diff_robot) ────────────────
    share_dir = os.path.dirname(diff_robot_share)
    for env_var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        if env_var in os.environ:
            os.environ[env_var] += ':' + share_dir
        else:
            os.environ[env_var] = share_dir

    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time_arg  = DeclareLaunchArgument('use_sim_time',  default_value='true')
    enable_fusion_rviz_arg = DeclareLaunchArgument('enable_fusion_rviz', default_value='true',
        description='Start fused RViz (map/costmaps/scan/odom/MotionPlanning)')
    enable_panda_rviz_arg = DeclareLaunchArgument('enable_panda_rviz', default_value='false')
    enable_bootstrap_tf_arg = DeclareLaunchArgument(
        'enable_bootstrap_tf', default_value='false',
        description='Static map->odom. Ignored when enable_ekf:=true (EKF owns map->odom).')
    enable_nav2_arg   = DeclareLaunchArgument('enable_nav2',   default_value='false')
    enable_ekf_arg    = DeclareLaunchArgument('enable_ekf',    default_value='false')
    # Auto dock goal via mission_orchestrator. Default false so you can use
    # RViz "Nav2 Goal" manually without a path appearing by itself.
    enable_mission_arg = DeclareLaunchArgument('enable_mission', default_value='false',
        description='If true (and enable_nav2), auto dock→PnP mission after Nav2 is up')
    enable_pick_place_arg = DeclareLaunchArgument(
        'enable_pick_place', default_value='false',
        description='Start Panda pick_and_place_demo (waits /trailer/docking_status)')
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            warehouse_orchestrator_share, 'config', 'recorded_waypoints.yaml'),
        description='YAML path of recorded waypoints (map frame) for mission_orchestrator')
    x_pose_arg        = DeclareLaunchArgument('x_pose',        default_value='-0.14')
    y_pose_arg        = DeclareLaunchArgument('y_pose',        default_value='-2.07')

    # Keep legacy alias so enable_rviz:=true still works for users
    enable_rviz_arg   = DeclareLaunchArgument('enable_rviz',   default_value='true',
        description='Deprecated alias of enable_fusion_rviz')

    # ========================================================================
    # BLOC 1 — PANDA ARM + GAZEBO FORTRESS + MoveIt2
    #   ex_pick_and_place.launch.py lance Ignition avec le monde fusionné,
    #   spawne le bras Panda, démarre MoveIt2, RViz (désactivé ici car on
    #   lance le nôtre), le détecteur de vision et le pick_and_place_demo.
    # ========================================================================
    panda_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(panda_config_share, 'launch', 'ex_pick_and_place.launch.py')
        ),
        launch_arguments={
            'world':               merged_world_path,
            'enable_pick_place':   LaunchConfiguration('enable_pick_place'),
            'pick_place_start_delay': '45.0',
            'use_sim_time':        'true',
            'enable_rviz':         'false',
            'headless':            'false',
        }.items(),
    )

    # ========================================================================
    # BLOC 2A — TF STATIQUE world -> map (identité, PERMANENT)
    #   Pont entre l'univers Panda (root=world) et l'univers Nav2 (root=map).
    #   use_sim_time: True — synchronisé avec l'horloge simulation.
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
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ========================================================================
    # BLOC 2B — TF BOOTSTRAP map -> odom -> base_footprint
    #   Connecte le trailer à l'arbre TF dès t=0 avant que l'EKF démarre.
    #   Desactive par defaut: l'EKF doit etre l'unique source map->odom
    #   et odom->base_footprint en mission normale.
    #   use_sim_time: True — synchronisé avec l'horloge simulation.
    # ========================================================================
    # Bootstrap map->odom: ONLY when EKF is off (EKF map owns map->odom).
    # enable_bootstrap_tf:=true + enable_ekf:=true caused TF fighting / broken trailer tree.
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
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_bootstrap_tf'), "' == 'true' and '",
                LaunchConfiguration('enable_ekf'), "' != 'true'",
            ])
        ),
        output='screen',
    )

    # Bootstrap odom->base: early bridge until EKF local is up. Auto-on when EKF
    # is enabled so RViz has a connected tree before t≈15–25s.
    enable_bootstrap_base_tf_arg = DeclareLaunchArgument(
        'enable_bootstrap_base_tf', default_value='false',
        description='Static odom->base_footprint. Prefer true only if EKF is delayed.')

    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bootstrap_odom_to_base_tf',
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint',
            '--x', LaunchConfiguration('x_pose'),
            '--y', LaunchConfiguration('y_pose'),
            '--z', '0.10',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_bootstrap_base_tf')),
        output='screen',
    )

    # ========================================================================
    # BLOC 3 — TRAILER : Robot State Publisher
    #   Publie l'arbre TF du trailer sur /trailer_robot_description et /tf.
    #   S'abonne a /trailer/joint_states (alimente par joint_state_splitter).
    #   ignore_timestamp: True — évite les erreurs "Moved backwards in time"
    #   NOTE: Utilise /trailer_robot_description pour éviter le conflit avec Panda
    # ========================================================================
    trailer_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='trailer_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time':      True,
            'publish_frequency': 50.0,
            'ignore_timestamp':  True,
        }],
        remappings=[
            ('robot_description', '/trailer_robot_description'),
            ('joint_states', '/trailer/joint_states'),
        ],
    )

    # ========================================================================
    # BLOC 3b — JOINT STATE SPLITTER
    #   Gazebo héberge panda + trailer dans le même processus : les deux
    #   joint_state_broadcaster publient sur /joint_states. Ce nœud sépare
    #   les messages (prefix panda_ vs reste) vers /panda/joint_states et
    #   /trailer/joint_states pour les deux robot_state_publisher / RViz.
    # ========================================================================
    joint_state_splitter = Node(
        package='warehouse_orchestrator',
        executable='joint_state_splitter',
        name='joint_state_splitter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topics': [
                '/joint_states',
                '/joint_state_broadcaster/joint_states',
            ],
            'panda_output_topic': '/panda/joint_states',
            'trailer_output_topic': '/trailer/joint_states',
            'panda_prefix': 'panda_',
        }],
    )

    # Teleop / Nav2 publish /cmd_vel ; diff_drive listens on
    # /diff_drive_controller/cmd_vel_unstamped (no Gazebo remapping — it is
    # process-wide and conflicts with multi-robot).
    cmd_vel_relay = Node(
        package='warehouse_orchestrator',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cmd_vel',
            'output_topic': '/diff_drive_controller/cmd_vel_unstamped',
        }],
    )

    # MotionPlanning (RViz) expects /robot_description + /robot_description_semantic.
    # Panda RSP only publishes /panda/robot_description.
    robot_description_relay = Node(
        package='warehouse_orchestrator',
        executable='robot_description_relay',
        name='robot_description_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/panda/robot_description',
            'output_topic': '/robot_description',
            'semantic_output_topic': '/robot_description_semantic',
            'move_group_node': '/move_group',
        }],
    )

    # ========================================================================
    # BLOC 4 — BRIDGES ros_gz_bridge
    #   clock_bridge : indispensable pour use_sim_time (horloge Gazebo -> ROS)
    #   scan_bridge  : LiDAR -> /scan (Nav2 costmaps)
    #   imu_bridge   : IMU -> /imu (EKF localization)
    #   gps_bridge   : GPS -> /gps/fix (navsat_transform -> EKF global)
    #   (Le bridge Panda /wrist_camera est géré par ex_pick_and_place)
    # clock_bridge SUPPRIMÉ : ex_pick_and_place.launch.py publie déjà /clock
    # Deux publishers simultanés sur /clock causent les "jump back in time" EKF.
    # Le bridge Panda (/clock via ignition.msgs.Clock) est suffisant.

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

    # ========================================================================
    # BLOC 5 — SPAWN TRAILER dans Gazebo (t=8s)
    #   Délai augmenté à 8s car Panda prend ~5s à spawner et initialiser
    #   son controller_manager. On attend que Gazebo soit stable.
    #   Le spawner lit /trailer_robot_description publié par trailer_rsp.
    # ========================================================================
    spawn_trailer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot_remorque',
                output='screen',
                arguments=[
                    '-name',  'robot_remorque',
                    '-topic', '/trailer_robot_description',   # Topic séparé pour éviter conflit Panda
                    '-world', 'mecanum_warehouse',
                    '-x', LaunchConfiguration('x_pose'),
                    '-y', LaunchConfiguration('y_pose'),
                    '-z', '0.10',
                ],
            ),
        ]
    )

    # ========================================================================
    # BLOC 6 — CONTRÔLEURS TRAILER
    #   t=22s : joint_state_broadcaster  (fournit /joint_states → splitter → RSP)
    #   t=28s : diff_drive_controller
    #   Do NOT start before gz_ros2_control plugin finishes (~t=15–20s with Panda).
    # ========================================================================
    # Robust activator: waits for gz_ros2_control CM then load/configure/activate.
    # Starts after trailer spawn (t=8s) + plugin init (~10–15s under dual robot).
    trailer_controller_activator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='warehouse_orchestrator',
                executable='controller_activator',
                name='trailer_controller_activator',
                output='screen',
                parameters=[{
                    # Wall clock — must not use sim time (RTF<<1 freezes timers).
                    'use_sim_time': False,
                    'controller_manager': '/trailer_controller_manager',
                    'controllers': [
                        'joint_state_broadcaster',
                        'diff_drive_controller',
                    ],
                    'timeout_sec': 360.0,
                    'poll_period_sec': 2.0,
                    'service_wait_sec': 60.0,
                    'call_timeout_sec': 60.0,
                }],
            ),
        ],
    )

    # ========================================================================
    # BLOC 7 — LOCALISATION EKF (identique à robot_remorque_ignition original)
    #   Délais: EKF local t=15s, navsat_transform t=17s, EKF global t=19s
    #   Résultat: chaîne TF complète world->map->odom->base_footprint
    # ========================================================================
    gps_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_robot_share, 'launch', 'gps_localization.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_ekf')),
    )

    # ========================================================================
    # BLOC 8 — NŒUDS MÉTIER REMORQUE (identiques à l'original)
    # ========================================================================
    trailer_joint_publisher = Node(
        package='diff_robot',
        executable='trailer_joint_publisher.py',
        name='trailer_joint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'beta_topic':   '/trailer/beta_raw',
        }],
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    beta_ekf_node = Node(
        package='trailer_kinematics',
        executable='beta_ekf_node',
        name='beta_ekf_node',
        output='screen',
        parameters=[{
            'use_sim_time':    True,
            'hitch_distance':  0.50,
            'wheel_separation': 0.40,
            'max_wheel_speed': 1.0,
            'beta_limit_deg':  55.0,
            'joint_states_topic': '/trailer/joint_states',
            'odom_topic': '/diff_drive_controller/odom',
        }],
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    multi_circle_footprint = Node(
        package='trailer_kinematics',
        executable='multi_circle_footprint_node',
        name='multi_circle_footprint_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'hitch_distance':       0.50,
            'robot_length':         0.6,
            'robot_width':          0.4,
            'trailer_length':       0.8,
            'trailer_width':        0.5,
            'tractor_center_x':     -0.025,
            'tractor_center_y':     0.041,
            'hitch_offset_x':       -0.3575,
            'hitch_offset_y':       0.0414,
            'num_circles_tractor':  3,
            'num_circles_trailer':  3,
            'footprint_margin':     0.01,
            'cost_threshold':       254,
        }],
    )

    # ========================================================================
    # BLOC 9 — NAV2 STACK COMPLET (identique à nav2.launch.py original)
    # Nav2 after trailer controllers are up (activator starts t=25s, typically done ~40–60s).
    nav2_stack = TimerAction(
        period=55.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'yaml_filename': default_map},
                ]),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart':    True,
                    'node_names':   ['map_server'],
                }]),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}]),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                remappings=[('cmd_vel', 'cmd_vel_nav2_raw')]),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}]),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                remappings=[('cmd_vel', 'cmd_vel_nav2_raw')]),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {
                    'use_sim_time':              True,
                    'default_nav_to_pose_bt_xml': default_bt_xml,
                }]),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}]),

            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                remappings=[
                    ('cmd_vel',         'cmd_vel_nav2_raw'),
                    ('cmd_vel_smoothed', 'cmd_vel_nav2'),
                ]),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart':    True,
                    'node_names':   [
                        'planner_server', 'controller_server', 'smoother_server',
                        'behavior_server', 'bt_navigator', 'waypoint_follower',
                        'velocity_smoother',
                    ],
                }]),

            # ── Pipeline sécurité cmd_vel ──────────────────────────────────
            Node(
                package='diff_robot',
                executable='trailer_aware_controller_node.py',
                name='trailer_aware_controller',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': True}],
                remappings=[
                    ('input_topic',  '/cmd_vel_nav2'),
                    ('output_topic', '/cmd_vel_raw'),
                ]),

            Node(
                package='diff_robot',
                executable='cmd_vel_safety_node.py',
                name='cmd_vel_safety_node',
                output='screen',
                parameters=[trailer_params_file, {'use_sim_time': True}],
                remappings=[
                    ('input_topic',  '/cmd_vel_raw'),
                    ('output_topic', '/cmd_vel'),
                ]),
        ],
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    # ========================================================================
    # BLOC 10 — RViz2 FUSIONNÉ (Trailer + MoveIt MotionPlanning)
    #   NOTE: pas de TimerAction+condition (bug Humble: RViz ne démarre jamais).
    #   Les robot_description sont latched → OK de démarrer dès t=0.
    # ========================================================================
    rviz_moveit_params = os.path.join(
        warehouse_orchestrator_share, 'config', 'rviz_moveit_params.yaml')

    rviz2_fused = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fusion',
        output='screen',
        arguments=['-d', rviz_config_fusion],
        parameters=[rviz_moveit_params],
        # enable_fusion_rviz OR legacy enable_rviz (panda include forces enable_rviz:=false
        # in its own scope, but keep both args for CLI compatibility)
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_fusion_rviz'), "' == 'true' or '",
                LaunchConfiguration('enable_rviz'), "' == 'true'"
            ])
        ),
    )

    # ========================================================================
    # BLOC 10B — RViz2 PANDA MOVEIT (Panda arm + MotionPlanning) — t=25s
    #   Démarré avec namespace "panda" pour éviter les conflits avec MoveIt.
    #   Config moveit.rviz : RobotModel Panda, MotionPlanning display.
    #   Remappings nécessaires pour les services/actions MoveIt.
    # ========================================================================
    rviz2_panda = Node(
        package='rviz2',
        executable='rviz2',
        namespace='panda',
        name='rviz2_panda',
        output='screen',
        arguments=['-d', rviz_config_panda],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('get_planner_params', '/panda/get_planner_params'),
            ('query_planner_interface', '/panda/query_planner_interface'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_panda_rviz')),
    )

    # ========================================================================
    # BLOC 11 — MISSION ORCHESTRATOR — t=70s (OPTIONAL)
    #   Corridor waypoints → dock at Panda table → /trailer/docking_status → PnP
    # ========================================================================
    mission_orchestrator_node = TimerAction(
        period=70.0,
        actions=[
            Node(
                package='warehouse_orchestrator',
                executable='mission_orchestrator',
                name='mission_orchestrator',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'dock_x': -0.24,
                    'dock_y': 2.80,
                    'dock_yaw': 1.57,
                    'max_dock_distance': 0.60,
                    'target_drop_x': -0.5,
                    'target_drop_y': 3.2,
                    'use_corridor_waypoints': True,
                    'send_return_goal': False,
                    'mission_start_delay_sec': 5.0,
                    'waypoints_file': ParameterValue(
                        LaunchConfiguration('waypoints_file'), value_type=str),
                }],
            ),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_nav2'), "' == 'true' and '",
                LaunchConfiguration('enable_mission'), "' == 'true'"
            ])
        ),
    )

    # ========================================================================
    # ASSEMBLAGE FINAL — LaunchDescription
    # ========================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_fusion_rviz_arg,
        enable_rviz_arg,
        enable_panda_rviz_arg,
        enable_bootstrap_tf_arg,
        enable_bootstrap_base_tf_arg,
        enable_nav2_arg,
        enable_ekf_arg,
        enable_mission_arg,
        enable_pick_place_arg,
        waypoints_file_arg,
        x_pose_arg,
        y_pose_arg,

        # ── t=0s : Panda arm + Gazebo Fortress (monde fusionné)
        panda_stack,

        # ── t=0s : TF world->map (pont Panda<->Nav2) — PERMANENT
        static_world_to_map,
        # ── Option debug : TF bootstrap map->odom (base via diff_drive odom TF)
        static_map_to_odom,
        static_odom_to_base,

        # ── t=0s : Trailer RSP (publie les TF a partir de /trailer/joint_states)
        trailer_rsp,

        # ── t=0s : Split shared /joint_states -> /panda + /trailer
        joint_state_splitter,

        # ── t=0s : Relay teleop /cmd_vel -> diff_drive
        cmd_vel_relay,

        # ── t=0s : /robot_description(+_semantic) for MoveIt RViz
        robot_description_relay,

        # ── t=0s : Bridges capteurs trailer (PAS de clock_bridge, déjà dans panda_stack)
        scan_bridge,
        imu_bridge,
        gps_bridge,

        # ── t=0s : Nœuds métier remorque (démarrent mais attendent les données)
        trailer_joint_publisher,
        beta_ekf_node,
        multi_circle_footprint,

        # ── t=0s : EKF localization (délais internes : 15s, 17s, 19s)
        gps_localization,

        # ── t=8s  : Spawn trailer dans Gazebo (après stabilisation Panda)
        spawn_trailer,

        # ── t=18s : Activate trailer joint_state_broadcaster + diff_drive
        trailer_controller_activator,

        # ── t=40s : Nav2 stack (after trailer controllers are up)
        nav2_stack,

        # ── t=25s : RViz2 fusionné (Trailer + Nav2)
        rviz2_fused,

        # ── t=25s : RViz2 Panda MoveIt (Panda arm + MotionPlanning)
        rviz2_panda,

        # ── t=55s : Mission Orchestrator
        mission_orchestrator_node,

        LogInfo(msg='=== Warehouse Fusion Mission — Tous les stacks lancés ==='),
        LogInfo(msg='=== Trailer spawné à t=8s | RViz à t=10s | Nav2 à t=20s | Mission à t=55s ==='),
    ])
