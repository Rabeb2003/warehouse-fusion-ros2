#!/usr/bin/env python3
"""
Pick and Place Autonomous Demo Node for Franka Emika Panda in ROS 2 Humble & Gazebo Fortress.
Executes 4 steps:
  1. Open Gripper & Move to Pre-Grasp Pose (above object on table)
  2. Approach & Close Gripper (Grasp)
  3. Lift & Transfer to Place Platform
  4. Lower, Open Gripper (Place) & Retreat to Home
"""

import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class PickAndPlaceDemoNode(Node):
    def __init__(self):
        super().__init__('panda_pick_and_place_demo')
        self.get_logger().info("==================================================")
        self.get_logger().info("  STARTING FRANKA PANDA PICK AND PLACE DEMO NODE  ")
        self.get_logger().info("==================================================")

        self.declare_parameter('vision_required', True)
        self.declare_parameter('vision_timeout', 20.0)
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('tcp_frame', 'panda_hand_tcp')
        self.declare_parameter('left_finger_frame', 'panda_leftfinger')
        self.declare_parameter('right_finger_frame', 'panda_rightfinger')
        self.declare_parameter('attach_max_distance', 0.12)
        self.declare_parameter('finger_center_max_distance', 0.10)
        self.declare_parameter('target_half_width', 0.025)
        self.vision_required = bool(self.get_parameter('vision_required').value)
        self.vision_timeout = float(self.get_parameter('vision_timeout').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        self.left_finger_frame = self.get_parameter('left_finger_frame').value
        self.right_finger_frame = self.get_parameter('right_finger_frame').value
        self.attach_max_distance = float(self.get_parameter('attach_max_distance').value)
        self.finger_center_max_distance = float(
            self.get_parameter('finger_center_max_distance').value
        )
        self.target_half_width = float(self.get_parameter('target_half_width').value)

        # Vision subscriber for real-time detected object pose
        self.detected_object_pose = None
        self.last_detection_monotonic = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.object_pose_callback,
            10
        )

        # Action clients for arm and gripper trajectory execution
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_trajectory_controller/follow_joint_trajectory'
        )
        self._controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gazebo DetachableJoint publishers for 100% rigid grasping
        self._attach_pub = self.create_publisher(Empty, '/panda/attach_object', 10)
        self._detach_pub = self.create_publisher(Empty, '/panda/detach_object', 10)

        self.arm_joints = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        self.gripper_joints = ['panda_finger_joint1', 'panda_finger_joint2']
        self.required_controllers = [
            'joint_state_broadcaster',
            'joint_trajectory_controller',
            'gripper_trajectory_controller',
        ]

    def object_pose_callback(self, msg: PoseStamped):
        self.detected_object_pose = msg.pose
        self.last_detection_monotonic = time.monotonic()

    def wait_for_detected_target(self):
        self.get_logger().info("Waiting for a real /detected_object_pose from the red detector...")
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < self.vision_timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.detected_object_pose is None:
                continue

            age = time.monotonic() - self.last_detection_monotonic
            if age > 2.0:
                continue

            target = self.detected_object_pose
            if self.is_target_pose_reasonable(target):
                self.get_logger().info(
                    f"Vision target accepted: X={target.position.x:.3f}, "
                    f"Y={target.position.y:.3f}, Z={target.position.z:.3f}"
                )
                return target

            self.get_logger().warn(
                f"Ignoring unreasonable vision pose: X={target.position.x:.3f}, "
                f"Y={target.position.y:.3f}, Z={target.position.z:.3f}",
                throttle_duration_sec=2.0,
            )

        message = "No valid red object detection arrived before timeout"
        if self.vision_required:
            raise RuntimeError(message)

        self.get_logger().warn(f"{message}; using configured fallback coordinates")
        return None

    def is_target_pose_reasonable(self, pose):
        return (
            0.20 <= pose.position.x <= 0.80
            and -0.40 <= pose.position.y <= 0.40
            and 0.35 <= pose.position.z <= 0.55
        )

    def get_frame_position(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Cannot read transform {self.target_frame} -> {frame_id}: {exc}")
            return None

        t = transform.transform.translation
        return t.x, t.y, t.z

    def get_tcp_position(self):
        return self.get_frame_position(self.tcp_frame)

    def distance_points(self, first, second):
        dx = first[0] - second[0]
        dy = first[1] - second[1]
        dz = first[2] - second[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def tcp_distance_to_target(self, target):
        tcp = self.get_tcp_position()
        if tcp is None:
            return None

        return self.distance_points(
            tcp,
            (target.position.x, target.position.y, target.position.z),
        )

    def grasp_geometry_to_target(self, target):
        target_point = (target.position.x, target.position.y, target.position.z)
        tcp = self.get_frame_position(self.tcp_frame)
        left = self.get_frame_position(self.left_finger_frame)
        right = self.get_frame_position(self.right_finger_frame)
        if tcp is None or left is None or right is None:
            return None

        finger_center = (
            0.5 * (left[0] + right[0]),
            0.5 * (left[1] + right[1]),
            0.5 * (left[2] + right[2]),
        )
        width_axis = (
            right[0] - left[0],
            right[1] - left[1],
            right[2] - left[2],
        )
        finger_gap = self.distance_points(left, right)
        lateral_offset = 0.0
        between_fingers = True
        if finger_gap > 1e-6:
            width_unit = (
                width_axis[0] / finger_gap,
                width_axis[1] / finger_gap,
                width_axis[2] / finger_gap,
            )
            center_to_target = (
                target_point[0] - finger_center[0],
                target_point[1] - finger_center[1],
                target_point[2] - finger_center[2],
            )
            lateral_offset = abs(
                center_to_target[0] * width_unit[0]
                + center_to_target[1] * width_unit[1]
                + center_to_target[2] * width_unit[2]
            )
            between_fingers = lateral_offset <= (0.5 * finger_gap + self.target_half_width)

        return {
            'tcp_distance': self.distance_points(tcp, target_point),
            'finger_center_distance': self.distance_points(finger_center, target_point),
            'finger_gap': finger_gap,
            'lateral_offset': lateral_offset,
            'between_fingers': between_fingers,
        }

    def wait_for_active_controllers(self, timeout_sec=120.0):
        self.get_logger().info("Waiting for controller_manager/list_controllers service...")
        start_time = time.monotonic()
        while not self._controllers_client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start_time > timeout_sec:
                raise RuntimeError("controller_manager/list_controllers service not available")

        last_log_time = 0.0
        while time.monotonic() - start_time <= timeout_sec:
            future = self._controllers_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.done() and future.result() is not None:
                states = {
                    controller.name: controller.state
                    for controller in future.result().controller
                }
                inactive = [
                    name for name in self.required_controllers
                    if states.get(name) != 'active'
                ]
                if not inactive:
                    self.get_logger().info("All required controllers are ACTIVE.")
                    return

                now = time.monotonic()
                if now - last_log_time > 5.0:
                    state_text = ", ".join(
                        f"{name}={states.get(name, 'missing')}"
                        for name in self.required_controllers
                    )
                    self.get_logger().info(f"Controllers not ready yet: {state_text}")
                    last_log_time = now

            time.sleep(0.5)

        raise RuntimeError("Timed out waiting for active Panda controllers")

    def wait_for_servers(self):
        self.wait_for_active_controllers()

        self.get_logger().info("Waiting for joint_trajectory_controller action server...")
        self._arm_client.wait_for_server()
        self.get_logger().info("Arm controller connected!")
        
        self.get_logger().info("Waiting for gripper_trajectory_controller action server...")
        self._gripper_client.wait_for_server()
        self.get_logger().info("Gripper controller connected!")
        time.sleep(1.0)

    def send_arm_trajectory(self, joint_positions, duration_sec=3.0, retries=5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        for attempt in range(retries):
            self.get_logger().info(f"Sending arm goal (Attempt {attempt+1}/{retries}): {joint_positions}")
            future = self._arm_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                result = res_future.result().result
                if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    return True
                self.get_logger().warn(
                    f"Arm trajectory finished with error_code={result.error_code}; retrying..."
                )
                time.sleep(2.0)
                continue
            
            self.get_logger().warn("Arm goal rejected (controller activating?), retrying in 2 seconds...")
            time.sleep(2.0)
            
        self.get_logger().error("Arm goal rejected after max retries!")
        return False

    def send_gripper_command(self, finger_position, duration_sec=1.5, retries=5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [finger_position, finger_position]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        for attempt in range(retries):
            self.get_logger().info(f"Setting gripper fingers (Attempt {attempt+1}/{retries}) to: {finger_position} m")
            future = self._gripper_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                result = res_future.result().result
                if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    return True
                self.get_logger().warn(
                    f"Gripper trajectory finished with error_code={result.error_code}; retrying..."
                )
                time.sleep(2.0)
                continue
                
            self.get_logger().warn("Gripper goal rejected (controller activating?), retrying in 2 seconds...")
            time.sleep(2.0)

        self.get_logger().error("Gripper goal rejected after max retries!")
        return False

    def run_sequence(self):
        # STEP 0: Lock the target from a real camera detection before moving.
        # The previous observation pose tilted the wrist camera into a geometry
        # where ray-plane projection became unstable and produced fake far targets.
        self.get_logger().info(">>> STEP 0: Waiting for live red-object detection")
        target_x, target_y, target_z = 0.5, 0.0, 0.425
        target = self.wait_for_detected_target()
        if target is not None:
            target_x = target.position.x
            target_y = target.position.y
            target_z = target.position.z
        target_reference = target
        if target_reference is None:
            target_reference = Pose()
            target_reference.position.x = target_x
            target_reference.position.y = target_y
            target_reference.position.z = target_z

        # Compute dynamic base rotation angle based on camera object coordinates
        angle_base = math.atan2(target_y, target_x)

        # Dynamic Joint configurations (radians)
        home_pose = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        pre_grasp_pose = [angle_base, 0.45, 0.0, -1.8, 0.0, 2.25, 0.785]
        grasp_pose = [angle_base, 0.78, 0.0, -1.45, 0.0, 2.25, 0.785]
        lift_pose = [angle_base, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785]
        transfer_pose = [1.57, 0.45, 0.0, -1.8, 0.0, 2.25, 0.785]
        descend_place_pose = [1.57, 0.78, 0.0, -1.45, 0.0, 2.25, 0.785]

        # ----------------------------------------------------
        # STEP 1: OPEN GRIPPER & MOVE TO PRE-GRASP
        # ----------------------------------------------------
        self.get_logger().info(">>> STEP 1: Opening Gripper & Moving to Vision Pre-Grasp Pose")
        if not self.send_gripper_command(finger_position=0.035, duration_sec=1.5):
            raise RuntimeError("Failed to open gripper")
        time.sleep(1.0)
        if not self.send_arm_trajectory(pre_grasp_pose, duration_sec=4.0):
            raise RuntimeError("Failed to reach pre-grasp pose")
        time.sleep(1.0)

        # ----------------------------------------------------
        # STEP 2: APPROACH TABLE SURFACE & CLOSE GRIPPER (GRASP)
        # ----------------------------------------------------
        self.get_logger().info(">>> STEP 2: Descending to Table Surface & Closing Gripper (Grasp)")
        if not self.send_arm_trajectory(grasp_pose, duration_sec=4.0):
            raise RuntimeError("Failed to reach grasp pose")
        time.sleep(1.0)
        if not self.send_gripper_command(finger_position=0.012, duration_sec=2.0):
            raise RuntimeError("Failed to close gripper")
        time.sleep(0.5)

        grasp_geometry = self.grasp_geometry_to_target(target_reference)
        if grasp_geometry is None:
            raise RuntimeError("Cannot validate grasp geometry because gripper TF transforms are unavailable")

        self.get_logger().info(
            "Grasp geometry check: "
            f"TCP-object={grasp_geometry['tcp_distance']:.3f} m, "
            f"finger-center-object={grasp_geometry['finger_center_distance']:.3f} m, "
            f"finger-gap={grasp_geometry['finger_gap']:.3f} m, "
            f"lateral-offset={grasp_geometry['lateral_offset']:.3f} m"
        )

        if (
            grasp_geometry['tcp_distance'] > self.attach_max_distance
            or grasp_geometry['finger_center_distance'] > self.finger_center_max_distance
            or not grasp_geometry['between_fingers']
        ):
            raise RuntimeError(
                "Refusing fake attach: detected object is not inside the gripper. "
                f"TCP-object={grasp_geometry['tcp_distance']:.3f} m "
                f"(limit {self.attach_max_distance:.3f} m), "
                f"finger-center-object={grasp_geometry['finger_center_distance']:.3f} m "
                f"(limit {self.finger_center_max_distance:.3f} m), "
                f"between_fingers={grasp_geometry['between_fingers']}"
            )

        # Trigger rigid Gazebo DetachableJoint attachment
        self.get_logger().info("Triggering Gazebo DetachableJoint attach_object...")
        self._attach_pub.publish(Empty())
        time.sleep(1.0)

        # ----------------------------------------------------
        # STEP 3: LIFT & TRANSFER TO PLACE PLATFORM
        # ----------------------------------------------------
        self.get_logger().info(">>> STEP 3: Lifting Object & Transferring to Place Platform")
        if not self.send_arm_trajectory(lift_pose, duration_sec=3.0):
            raise RuntimeError("Failed to lift object")
        time.sleep(1.0)
        if not self.send_arm_trajectory(transfer_pose, duration_sec=5.0):
            raise RuntimeError("Failed to reach transfer pose")
        time.sleep(1.0)
        if not self.send_arm_trajectory(descend_place_pose, duration_sec=3.0):
            raise RuntimeError("Failed to reach place pose")
        time.sleep(1.0)

        # ----------------------------------------------------
        # STEP 4: PLACE, OPEN GRIPPER & RETREAT TO HOME
        # ----------------------------------------------------
        self.get_logger().info(">>> STEP 4: Opening Gripper (Release) & Retreating to Home")
        # Trigger rigid Gazebo DetachableJoint detachment
        self.get_logger().info("Triggering Gazebo DetachableJoint detach_object...")
        self._detach_pub.publish(Empty())
        time.sleep(0.5)
        if not self.send_gripper_command(finger_position=0.035, duration_sec=1.5):
            raise RuntimeError("Failed to open gripper at place pose")
        time.sleep(1.5)
        if not self.send_arm_trajectory(lift_pose, duration_sec=3.0):
            raise RuntimeError("Failed to retreat from place pose")
        time.sleep(1.0)
        if not self.send_arm_trajectory(home_pose, duration_sec=4.0):
            raise RuntimeError("Failed to return home")

        self.get_logger().info("==================================================")
        self.get_logger().info("  PICK AND PLACE TASK SUCCESSFULLY COMPLETED !   ")
        self.get_logger().info("==================================================")

def main(args=None):
    rclpy.init(args=args)
    demo = PickAndPlaceDemoNode()
    try:
        demo.wait_for_servers()
        demo.run_sequence()
    except Exception as e:
        demo.get_logger().error(f"Error during Pick and Place execution: {e}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
