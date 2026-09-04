#!/usr/bin/env python3
"""Relay /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped for teleop."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(Twist, out, 10)
        self.create_subscription(Twist, inp, self._cb, 10)
        self.get_logger().info(f'Relaying {inp} -> {out}')

    def _cb(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
