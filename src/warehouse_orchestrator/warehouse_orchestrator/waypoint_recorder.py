#!/usr/bin/env python3
"""
Interactive waypoint recorder for trailer navigation (map / GPS frame).

Drive with teleop_twist_keyboard, then save poses in map frame.

Controls (this terminal):
  Enter / s  — save current pose (map → base_footprint)
  d          — delete last waypoint
  w          — write/flush file now
  q          — save and quit

Also:  ros2 service call /save_waypoint std_srvs/srv/Trigger {}
"""

from __future__ import annotations

import math
import os
import select
import sys
import termios
import tty

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class WaypointRecorder(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_recorder')

        default_out = os.path.join(
            get_package_share_directory('warehouse_orchestrator'),
            'config',
            'recorded_waypoints.yaml',
        )
        self.declare_parameter('output_file', default_out)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('min_save_distance', 0.25)  # ignore duplicate Enter presses

        self.output_file = os.path.expanduser(
            str(self.get_parameter('output_file').value))
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.min_save_distance = float(self.get_parameter('min_save_distance').value)

        self.waypoints: list[dict] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, 'save_waypoint', self._on_save_srv)

        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self._load_existing()
        self.get_logger().info(f'Recording to: {self.output_file}')
        self.get_logger().info(
            f'TF {self.parent_frame} → {self.child_frame} | '
            'Keys: [Enter/s]=save [d]=delete [w]=write [q]=quit')

    def _load_existing(self) -> None:
        if not os.path.isfile(self.output_file):
            return
        with open(self.output_file, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        self.waypoints = list(data.get('waypoints') or [])
        if self.waypoints:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} existing waypoints from file')

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF unavailable: {ex}', throttle_duration_sec=2.0)
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation.z, tf.transform.rotation.w)
        return x, y, yaw

    def save_current(self) -> bool:
        pose = self._lookup_pose()
        if pose is None:
            return False
        x, y, yaw = pose
        if self.waypoints and self.min_save_distance > 0.0:
            last = self.waypoints[-1]
            dx = x - float(last['x'])
            dy = y - float(last['y'])
            if math.hypot(dx, dy) < self.min_save_distance:
                self.get_logger().warn(
                    f'Too close to WP#{len(self.waypoints)} '
                    f'(<{self.min_save_distance:.2f} m) — not saved')
                return False
        wp = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}
        self.waypoints.append(wp)
        self._write_file()
        self.get_logger().info(
            f'Saved WP#{len(self.waypoints)}: x={wp["x"]:.3f} y={wp["y"]:.3f} yaw={wp["yaw"]:.3f}')
        return True

    def delete_last(self) -> None:
        if not self.waypoints:
            self.get_logger().warn('No waypoints to delete')
            return
        removed = self.waypoints.pop()
        self._write_file()
        self.get_logger().info(f'Deleted last: {removed} ({len(self.waypoints)} left)')

    def _write_file(self) -> None:
        data = {
            'frame_id': self.parent_frame,
            'waypoints': self.waypoints,
        }
        with open(self.output_file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(f'Wrote {len(self.waypoints)} waypoints → {self.output_file}')

    def _on_save_srv(self, _req, res):
        ok = self.save_current()
        res.success = ok
        res.message = 'saved' if ok else 'tf failed'
        return res


def _read_key_nonblocking() -> str | None:
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not dr:
        return None
    return sys.stdin.read(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRecorder()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print('\n=== Waypoint Recorder ===')
        print('Drive with teleop in another terminal.')
        print('Here: Enter/s=save  d=delete  w=write  q=quit\n')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = _read_key_nonblocking()
            if key is None:
                continue
            if key in ('\n', '\r', 's', 'S'):
                node.save_current()
            elif key in ('d', 'D'):
                node.delete_last()
            elif key in ('w', 'W'):
                node._write_file()
            elif key in ('q', 'Q'):
                node._write_file()
                break
    except KeyboardInterrupt:
        node._write_file()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
