# Autonomous Warehouse Fusion: Integrated Navigation and Manipulation for Articulated Robot-Trailer Systems

A ROS2 Humble framework for autonomous warehouse operations combining differential-drive trailer navigation with 7-DOF robotic arm manipulation, enabling end-to-end pick-and-place missions in simulation.

## Research Context

Modern warehouse automation requires tight integration between mobile navigation and robotic manipulation. This project addresses the challenge of coordinating an articulated robot-trailer system with a 7-DOF Franka Emika Panda arm for autonomous object transport tasks. The system integrates GPS-based localization, LiDAR navigation, vision-based object detection, and motion planning in a unified ROS2 architecture.

## Technical Contributions

- **Integrated Navigation-Manipulation Pipeline**: Seamless coordination between Nav2-based trailer navigation and MoveIt2-based arm manipulation through a mission orchestrator
- **Articulated Kinematics**: Custom EKF-based state estimation for robot-trailer hitch angle and trailer pose tracking
- **Multi-Sensor Fusion**: GPS + IMU + LiDAR fusion for robust global localization in warehouse environments
- **Vision-Guided Manipulation**: Real-time object detection using wrist-mounted camera with geometric grasp validation
- **Deterministic Device Management**: udev rules and systemd integration for production-ready hardware deployment

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   GPS/IMU       │    │   2D LiDAR      │    │  Wrist Camera   │
│   (navsat)      │    │   (scan)        │    │  (image_raw)    │
└────────┬────────┘    └────────┬────────┘    └────────┬────────┘
         │                      │                      │
         ▼                      ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Perception Layer                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ robot_local- │  │    Nav2      │  │  Object      │         │
│  │  ization     │  │   Stack      │  │  Detector    │         │
│  │  (EKF)       │  │ (map_server, │  │  (YOLOv8)    │         │
│  └──────────────┘  │  planner,    │  └──────────────┘         │
│                    │  controller)  │                            │
│  ┌──────────────┐  └──────────────┘                            │
│  │ Trailer      │                                               │
│  │ Kinematics   │                                               │
│  │ (hitch EKF)  │                                               │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Planning Layer                                │
│  ┌──────────────┐  ┌──────────────┐                            │
│  │ Mission      │  │   MoveIt2    │                            │
│  │ Orchestrator │  │  (IK, motion │                            │
│  │ (state       │  │   planning)  │                            │
│  │  machine)    │  └──────────────┘                            │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Control Layer                                 │
│  ┌──────────────┐  ┌──────────────┐                            │
│  │ diff_drive   │  │ joint_traj   │                            │
│  │ controller   │  │ controller   │                            │
│  │ (trailer)    │  │ (Panda arm)  │                            │
│  └──────────────┘  └──────────────┘                            │
└─────────────────────────────────────────────────────────────────┘
```

## Tech Stack

| Component | Technology | Version |
|-----------|-----------|---------|
| **ROS** | ROS2 Humble | Humble Hawksbill |
| **Simulation** | Gazebo Fortress | 6.x |
| **Navigation** | Nav2 | Humble |
| **Manipulation** | MoveIt2 | Humble |
| **Localization** | robot_localization | Humble |
| **Vision** | OpenCV, YOLOv8 | 4.x, 8.x |
| **Language** | Python, C++ | 3.10, C++17 |
| **Build System** | colcon, ament | - |

## Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: ROS2 Humble Hawksbill
- **Python**: 3.10+
- **Dependencies**:
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-moveit ros-humble-moveit-ros-move-group \
    ros-humble-robot-localization ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim python3-opencv python3-yaml
  ```

## Installation

```bash
# Clone repository
git clone https://github.com/Rabeb2003/warehouse-fusion-ros2.git
cd warehouse-fusion-ros2

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Quick Start

```bash
# Launch complete autonomous mission (simulation)
ros2 launch warehouse_orchestrator full_mission.launch.py

# Launch navigation only (trailer)
ros2 launch warehouse_orchestrator trailer_nav2.launch.py

# Launch manipulation only (Panda)
ros2 launch warehouse_orchestrator panda_pick_place.launch.py

# Launch visualization
ros2 launch warehouse_orchestrator visualization.launch.py
```

## Mission Workflow

The full mission executes the following sequence:

1. **System Initialization**: Gazebo simulation, robot state publishers, sensor bridges
2. **Localization**: GPS/IMU + LiDAR fusion for global pose estimation
3. **Navigation**: Nav2 guides trailer to docking station using recorded waypoints
4. **Docking**: Trailer aligns with Panda workspace using visual markers
5. **Manipulation**: Panda detects object, plans grasp trajectory, executes pick-and-place
6. **Return**: Trailer navigates to delivery location with transported object

## Validation

**Simulation Results**: Qualitative validation in Gazebo Fortress demonstrates:
- Successful trailer navigation through warehouse waypoints
- Accurate docking at Panda workspace (±5cm tolerance)
- Reliable object detection and grasp planning
- End-to-end mission completion in ~3 minutes

**Screenshots**: See `docs/` folder for visual demonstrations:
- `warehouse_scene.png`: Complete warehouse simulation environment
- `nav2_navigation.png`: Nav2 navigation stack with path planning
- `object_detection.png`: Real-time object detection pipeline
- `place_detection.png`: Grasp validation and placement
- `simulation_rviz.png`: Integrated system visualization

**Limitations**:
- Simulation-only validation (no hardware deployment yet)
- Object detection assumes single red block in known workspace
- Docking relies on pre-recorded waypoints (not SLAM-based)
- No dynamic obstacle avoidance during manipulation

## Research Relevance

This project demonstrates competency in several areas critical for research internships:

1. **ROS2 Architecture**: Multi-package modular design with clean interfaces
2. **Sensor Fusion**: Practical implementation of EKF-based localization
3. **Integrated Planning**: Coordination between navigation and manipulation stacks
4. **Reproducibility**: Complete simulation environment with documented dependencies

## Project Structure

```
warehouse-fusion-ros2/
├── src/
│   ├── diff_robot/              # Differential drive trailer platform
│   │   ├── diff_robot/         # ROS2 nodes (control, safety, GPS)
│   │   ├── urdf/               # Robot URDF/SDF models
│   │   ├── launch/             # Individual launch files
│   │   └── config/             # Controller configurations
│   ├── trailer_kinematics/     # Articulated kinematics
│   │   └── trailer_kinematics/ # Hitch angle EKF, footprint calc
│   ├── panda/                  # Franka Emika Panda metapackage
│   ├── panda_description/      # Panda robot models
│   ├── panda_moveit_config/     # MoveIt2 configuration
│   │   └── scripts/            # Pick-and-place demo, object detector
│   └── warehouse_orchestrator/ # Mission coordination
│       ├── warehouse_orchestrator/
│       │   ├── mission_orchestrator.py
│       │   ├── controller_activator.py
│       │   └── joint_state_splitter.py
│       ├── launch/             # Integrated launch files
│       ├── config/             # Waypoints, parameters
│       └── rviz/               # Fusion visualization configs
├── docs/                       # Screenshots, diagrams
├── .gitignore
├── LICENSE
└── README.md
```

## Author

**Rabeb Bouzaida**  
Electrical Engineering Student, ENIM (Tunisia)  
GitHub: [Rabeb2003](https://github.com/Rabeb2003)

## License

MIT License - see LICENSE file for details

## Acknowledgments

- Franka Emika Panda robot model and MoveIt2 configuration
- Nav2 navigation stack contributors
- ROS2 Humble community
