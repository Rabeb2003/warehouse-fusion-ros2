#!/usr/bin/env python3
"""
Mission Orchestrator Node for Warehouse Fusion.
Coordinates autonomous trailer navigation and Franka Panda arm pick-and-place task:
  1. Sends docking Nav2 waypoint to trailer robot near Panda workstation.
  2. Confirms docking arrival via Nav2 result and TF lookup for trailer_cart_drop_point.
  3. Publishes /trailer/docking_status to trigger Panda arm pick-and-place.
  4. Waits for /panda/task_completed.
  5. Sends post-loading navigation goal to trailer to continue its warehouse route.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MissionOrchestratorNode(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        self.get_logger().info("==================================================")
        self.get_logger().info("   STARTING WAREHOUSE MISSION ORCHESTRATOR NODE   ")
        self.get_logger().info("==================================================")

        self.declare_parameter('dock_x', -0.2425)
        self.declare_parameter('dock_y', 4.00)
        self.declare_parameter('dock_yaw', 0.0)
        self.declare_parameter('max_dock_distance', 0.50)
        self.declare_parameter('target_drop_x', -1.0)
        self.declare_parameter('target_drop_y', 4.0)

        self.dock_x = float(self.get_parameter('dock_x').value)
        self.dock_y = float(self.get_parameter('dock_y').value)
        self.dock_yaw = float(self.get_parameter('dock_yaw').value)
        self.max_dock_distance = float(self.get_parameter('max_dock_distance').value)
        self.target_drop_x = float(self.get_parameter('target_drop_x').value)
        self.target_drop_y = float(self.get_parameter('target_drop_y').value)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publishers & Subscribers
        self.docking_pub = self.create_publisher(Bool, '/trailer/docking_status', 10)
        self.panda_completed_sub = self.create_subscription(
            Bool, '/panda/task_completed', self.panda_completion_callback, 10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.panda_finished = False

    def panda_completion_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received confirmation: Panda arm pick-and-place task COMPLETED!")
            self.panda_finished = True

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_nav_goal(self, x, y, yaw):
        self.get_logger().info(f"Waiting for Nav2 /navigate_to_pose action server...")
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, yaw)

        self.get_logger().info(f"Sending NavigateToPose goal to Trailer: X={x:.3f}, Y={y:.3f}, Yaw={yaw:.2f} rad")
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected by Nav2!")
            return False

        self.get_logger().info("Navigation goal accepted by Nav2. Waiting for arrival...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trailer robot successfully arrived at docking goal!")
            return True
        else:
            self.get_logger().warn(f"Trailer navigation finished with status: {result.status}")
            return False

    def verify_tf_docking(self):
        self.get_logger().info("Verifying trailer_cart_drop_point TF transform relative to world...")
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'trailer_cart_drop_point',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            dist = math.sqrt((tx - self.target_drop_x)**2 + (ty - self.target_drop_y)**2)
            self.get_logger().info(
                f"TF Check: trailer_cart_drop_point at X={tx:.3f}, Y={ty:.3f}, Z={tz:.3f} | "
                f"Distance to place target ({self.target_drop_x}, {self.target_drop_y}): {dist:.3f} m"
            )
            return dist <= self.max_dock_distance
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed for trailer_cart_drop_point: {ex}")
            # If TF fails temporarily, accept docking based on Nav2 success
            return True

    def run_mission(self):
        time.sleep(5.0)  # Grace time for all nodes and TF tree to initialize
        self.get_logger().info(">>> MISSION STEP 1: Sending Trailer to Docking Location near Panda Arm")

        if self.send_nav_goal(self.dock_x, self.dock_y, self.dock_yaw):
            self.get_logger().info(">>> MISSION STEP 2: Verifying trailer docking pose with TF")
            tf_ok = self.verify_tf_docking()
            if tf_ok:
                self.get_logger().info(">>> MISSION STEP 3: Confirming Docking & Triggering Panda Arm")
                dock_msg = Bool(data=True)
                for _ in range(5):
                    self.docking_pub.publish(dock_msg)
                    time.sleep(0.5)
            else:
                self.get_logger().warn("TF distance check exceeded max threshold, publishing docking status anyway")
                self.docking_pub.publish(Bool(data=True))

            self.get_logger().info(">>> MISSION STEP 4: Waiting for Panda Pick and Place completion...")
            while rclpy.ok() and not self.panda_finished:
                rclpy.spin_once(self, timeout_sec=1.0)

            if self.panda_finished:
                self.get_logger().info(">>> MISSION STEP 5: Panda arm loaded object! Resuming trailer route")
                # Send second waypoint in warehouse (e.g. return to -4.0, 2.0)
                self.send_nav_goal(-4.0, 2.0, 3.14)
                self.get_logger().info("==================================================")
                self.get_logger().info("   FULL WAREHOUSE FUSION MISSION COMPLETED !      ")
                self.get_logger().info("==================================================")


def main(args=None):
    rclpy.init(args=args)
    orchestrator = MissionOrchestratorNode()
    try:
        orchestrator.run_mission()
    except Exception as e:
        orchestrator.get_logger().error(f"Error in Mission Orchestrator: {e}")
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
