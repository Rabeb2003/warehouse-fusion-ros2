#!/usr/bin/env python3
"""Publish latched /robot_description(+_semantic) for MoveIt RViz MotionPlanning.

Panda RSP remaps description to /panda/robot_description. MotionPlanning expects
the standard /robot_description topic (and semantic) when Robot Description is
set to 'robot_description'.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String


class RobotDescriptionRelay(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_relay')
        self.declare_parameter('input_topic', '/panda/robot_description')
        self.declare_parameter('output_topic', '/robot_description')
        self.declare_parameter('semantic_output_topic', '/robot_description_semantic')
        self.declare_parameter('move_group_node', '/move_group')

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        sem_out = self.get_parameter('semantic_output_topic').value
        mg = self.get_parameter('move_group_node').value

        self._pub = self.create_publisher(String, out, latched)
        self._sem_pub = self.create_publisher(String, sem_out, latched)
        self.create_subscription(String, inp, self._on_desc, latched)
        self.create_timer(1.0, self._try_fetch_semantic)
        self._semantic_done = False
        self._mg = mg
        self.get_logger().info(f'Relay {inp} -> {out}; semantic from {mg}')

    def _on_desc(self, msg: String) -> None:
        self._pub.publish(msg)

    def _try_fetch_semantic(self) -> None:
        if self._semantic_done:
            return
        cli = self.create_client(GetParameters, f'{self._mg}/get_parameters')
        if not cli.wait_for_service(timeout_sec=0.1):
            return
        req = GetParameters.Request()
        req.names = ['robot_description_semantic']
        fut = cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                if res and res.values and res.values[0].string_value:
                    msg = String()
                    msg.data = res.values[0].string_value
                    self._sem_pub.publish(msg)
                    self._semantic_done = True
                    self.get_logger().info(
                        f'Published robot_description_semantic ({len(msg.data)} chars)')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'semantic fetch failed: {exc}')

        fut.add_done_callback(_done)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
