#!/usr/bin/env python3
"""
Mission Orchestrator — follow recorded (or default) waypoints, then pick-and-place.

Flow:
  1. Load waypoints from YAML (recorded via waypoint_recorder + teleop), else defaults.
  2. Sequential NavigateToPose for each waypoint (trailer BT recoveries).
  3. /trailer/docking_status → Panda PnP → /panda/task_completed.
"""

from __future__ import annotations

import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MissionOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_orchestrator')
        self.get_logger().info('=== Warehouse Mission Orchestrator ===')

        self.declare_parameter('dock_x', -0.24)
        self.declare_parameter('dock_y', 2.80)
        self.declare_parameter('dock_yaw', 1.5708)
        self.declare_parameter('max_dock_distance', 0.60)
        self.declare_parameter('target_drop_x', -0.5)
        self.declare_parameter('target_drop_y', 3.2)
        self.declare_parameter('use_corridor_waypoints', True)
        self.declare_parameter('send_return_goal', False)
        self.declare_parameter('mission_start_delay_sec', 8.0)
        self.declare_parameter('goal_attempts', 5)
        self.declare_parameter('gap_x', 1.50)
        self.declare_parameter('gap_y_south', -1.50)
        self.declare_parameter('gap_y_north', 1.20)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('waypoint_dedup_distance', 0.25)
        self.declare_parameter('panda_wait_timeout_sec', 30.0)
        self.declare_parameter('docking_signal_period_sec', 1.0)

        self.dock_x = float(self.get_parameter('dock_x').value)
        self.dock_y = float(self.get_parameter('dock_y').value)
        self.dock_yaw = float(self.get_parameter('dock_yaw').value)
        self.max_dock_distance = float(self.get_parameter('max_dock_distance').value)
        self.target_drop_x = float(self.get_parameter('target_drop_x').value)
        self.target_drop_y = float(self.get_parameter('target_drop_y').value)
        self.use_corridor_waypoints = bool(self.get_parameter('use_corridor_waypoints').value)
        self.send_return_goal = bool(self.get_parameter('send_return_goal').value)
        self.start_delay = float(self.get_parameter('mission_start_delay_sec').value)
        self.goal_attempts = int(self.get_parameter('goal_attempts').value)
        self.gap_x = float(self.get_parameter('gap_x').value)
        self.gap_y_south = float(self.get_parameter('gap_y_south').value)
        self.gap_y_north = float(self.get_parameter('gap_y_north').value)
        self.waypoints_file = str(self.get_parameter('waypoints_file').value).strip()
        self.dedup_distance = float(self.get_parameter('waypoint_dedup_distance').value)
        self.panda_wait_timeout = float(self.get_parameter('panda_wait_timeout_sec').value)
        self.docking_signal_period = float(
            self.get_parameter('docking_signal_period_sec').value
        )

        self._nav_pose = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        sync_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.docking_pub = self.create_publisher(
            Bool, '/trailer/docking_status', sync_qos
        )
        self.create_subscription(
            Bool, '/panda/task_completed', self._on_panda_done, sync_qos
        )
        self.create_subscription(
            Bool, '/panda/task_completed', self._on_panda_done, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.panda_finished = False

    def _on_panda_done(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info('Panda pick-and-place COMPLETED')
            self.panda_finished = True

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        return Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation = self._yaw_to_quat(yaw)
        return p

    def _dedupe_waypoints(
        self, wps: list[tuple[float, float, float]]
    ) -> list[tuple[float, float, float]]:
        if not wps or self.dedup_distance <= 0.0:
            return wps
        out = [wps[0]]
        for x, y, yaw in wps[1:]:
            lx, ly, _ = out[-1]
            if math.hypot(x - lx, y - ly) >= self.dedup_distance:
                out.append((x, y, yaw))
        if len(out) < len(wps):
            self.get_logger().info(
                f'Deduped waypoints: {len(wps)} → {len(out)} '
                f'(min distance {self.dedup_distance:.2f} m)')
        return out

    def _load_waypoints(self) -> list[tuple[float, float, float]]:
        path = os.path.expanduser(self.waypoints_file)
        if path and os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            wps = data.get('waypoints') or []
            out: list[tuple[float, float, float]] = []
            for wp in wps:
                out.append((float(wp['x']), float(wp['y']), float(wp.get('yaw', self.dock_yaw))))
            out = self._dedupe_waypoints(out)
            if out:
                self.get_logger().info(
                    f'Loaded {len(out)} waypoints from {path} | '
                    f'first=({out[0][0]:.2f},{out[0][1]:.2f}) '
                    f'last=({out[-1][0]:.2f},{out[-1][1]:.2f})')
                return out
            self.get_logger().warn(f'Empty waypoints in {path} — using defaults')

        if self.waypoints_file:
            self.get_logger().warn(
                f'waypoints_file not found: {path} — using default corridor')

        return [
            (self.gap_x, self.gap_y_south, self.dock_yaw),
            (self.gap_x, self.gap_y_north, self.dock_yaw),
            (self.dock_x, self.dock_y, self.dock_yaw),
        ]

    def _wait_nav_servers(self) -> bool:
        self.get_logger().info('Waiting for /navigate_to_pose…')
        if not self._nav_pose.wait_for_server(timeout_sec=90.0):
            self.get_logger().error('/navigate_to_pose not available')
            return False
        return True

    def send_nav_goal(self, x: float, y: float, yaw: float, attempts: int | None = None) -> bool:
        attempts = self.goal_attempts if attempts is None else attempts
        for attempt in range(1, attempts + 1):
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(x, y, yaw)
            self.get_logger().info(
                f'Nav goal ({attempt}/{attempts}): x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
            fut = self._nav_pose.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().warn('Goal rejected — retrying…')
                time.sleep(3.0)
                continue
            result_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_fut)
            result = result_fut.result()
            if result is None:
                self.get_logger().warn('No result — retrying…')
                time.sleep(2.0)
                continue
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Nav goal SUCCEEDED')
                return True
            self.get_logger().warn(
                f'Nav finished status={result.status} — retrying next attempt…')
            time.sleep(2.0)
        return False

    def send_waypoint_path(self) -> bool:
        waypoints = self._load_waypoints()
        if not waypoints:
            self.get_logger().error('No waypoints to follow')
            return False

        last_x, last_y, last_yaw = waypoints[-1]
        self.get_logger().info(
            f'Following {len(waypoints)} NavigateToPose goals → '
            f'last ({last_x:.2f}, {last_y:.2f}) [map/GPS]')

        for i, (x, y, yaw) in enumerate(waypoints, start=1):
            self.get_logger().info(f'--- Waypoint {i}/{len(waypoints)} ---')
            if not self.send_nav_goal(x, y, yaw):
                self.get_logger().error(f'Waypoint {i} failed')
                if i < len(waypoints):
                    self.get_logger().warn('Attempting last waypoint recovery…')
                    return self.send_nav_goal(last_x, last_y, last_yaw)
                return False

        self.dock_x, self.dock_y, self.dock_yaw = last_x, last_y, last_yaw
        self.get_logger().info('Waypoint path SUCCEEDED')
        return True

    def verify_tf_docking(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', 'trailer_cart_drop_point',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0),
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            dist = math.hypot(tx - self.target_drop_x, ty - self.target_drop_y)
            self.get_logger().info(
                f'Drop point TF ({tx:.2f},{ty:.2f}) dist to target={dist:.2f} m')
            return dist <= self.max_dock_distance
        except TransformException as ex:
            self.get_logger().warn(f'TF check skipped: {ex}')
            return True

    def wait_for_panda_listener(self) -> bool:
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < self.panda_wait_timeout:
            subscribers = self.docking_pub.get_subscription_count()
            if subscribers > 0:
                self.get_logger().info(
                    f'Panda docking listener connected ({subscribers} subscriber)'
                )
                return True
            self.get_logger().warn(
                'Waiting for Panda pick_and_place_demo subscriber on '
                '/trailer/docking_status...',
                throttle_duration_sec=5.0,
            )
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().error(
            'No Panda subscriber on /trailer/docking_status. '
            'Start full_mission with enable_pick_place:=true, or run '
            'panda_moveit_config pick_and_place_demo.py.'
        )
        return False

    def trigger_panda(self) -> None:
        msg = Bool(data=True)
        self.docking_pub.publish(msg)
        self.get_logger().info('Published /trailer/docking_status = True')

    def keep_panda_triggered_until_done(self) -> None:
        msg = Bool(data=True)
        last_publish = 0.0
        while rclpy.ok() and not self.panda_finished:
            now = time.monotonic()
            if now - last_publish >= self.docking_signal_period:
                self.docking_pub.publish(msg)
                last_publish = now
            rclpy.spin_once(self, timeout_sec=0.2)

    def run_mission(self) -> None:
        self.get_logger().info(f'Mission start delay {self.start_delay:.0f}s…')
        time.sleep(self.start_delay)
        if not self._wait_nav_servers():
            return

        self.get_logger().info('STEP 1: Navigate trailer along waypoint path')
        ok = (
            self.send_waypoint_path()
            if self.use_corridor_waypoints
            else self.send_nav_goal(self.dock_x, self.dock_y, self.dock_yaw)
        )
        if not ok:
            self.get_logger().error('Navigation failed — mission abort')
            return

        self.get_logger().info('=== MISSION 1 COMPLETE: trailer at Panda table ===')
        self.verify_tf_docking()

        self.get_logger().info('=== MISSION 2 START: Panda pick red block → trailer ===')
        if not self.wait_for_panda_listener():
            return
        self.trigger_panda()

        self.get_logger().info('Waiting for /panda/task_completed (Mission 2)…')
        self.keep_panda_triggered_until_done()

        if self.panda_finished:
            self.get_logger().info('=== FULL MISSION COMPLETE (nav + pick-and-place) ===')
            if self.send_return_goal:
                self.send_nav_goal(-0.14, -2.07, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestratorNode()
    try:
        node.run_mission()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Mission error: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
