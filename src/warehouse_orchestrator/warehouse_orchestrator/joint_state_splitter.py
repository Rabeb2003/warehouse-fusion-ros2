#!/usr/bin/env python3
"""Split a shared JointState stream into panda / trailer topics.

Gazebo runs both gz_ros2_control plugins in one process, so remappings and
identically-named joint_state_broadcasters collide. This node fans out
messages by joint-name prefix so each robot_state_publisher gets a clean
stream.
"""

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateSplitter(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_splitter')

        self.declare_parameter('input_topics', ['/joint_states', '/joint_state_broadcaster/joint_states'])
        self.declare_parameter('panda_output_topic', '/panda/joint_states')
        self.declare_parameter('trailer_output_topic', '/trailer/joint_states')
        self.declare_parameter('panda_prefix', 'panda_')

        input_topics = self.get_parameter('input_topics').value
        panda_topic = self.get_parameter('panda_output_topic').value
        trailer_topic = self.get_parameter('trailer_output_topic').value
        self._panda_prefix = self.get_parameter('panda_prefix').value

        # JSB publishes RELIABLE + TRANSIENT_LOCAL; match it on input.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._panda_pub = self.create_publisher(JointState, panda_topic, out_qos)
        self._trailer_pub = self.create_publisher(JointState, trailer_topic, out_qos)
        for topic in input_topics:
            self.create_subscription(JointState, topic, self._on_joint_state, sensor_qos)

        self.get_logger().info(
            f'Splitting {list(input_topics)} -> {panda_topic} / {trailer_topic} '
            f'(panda prefix="{self._panda_prefix}")'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        panda_idx: List[int] = []
        trailer_idx: List[int] = []
        for i, name in enumerate(msg.name):
            if name.startswith(self._panda_prefix):
                panda_idx.append(i)
            else:
                trailer_idx.append(i)

        if panda_idx:
            self._panda_pub.publish(self._subset(msg, panda_idx))
        if trailer_idx:
            self._trailer_pub.publish(self._subset(msg, trailer_idx))

    @staticmethod
    def _subset(msg: JointState, indices: List[int]) -> JointState:
        out = JointState()
        out.header = msg.header
        out.name = [msg.name[i] for i in indices]
        if msg.position:
            out.position = [msg.position[i] for i in indices]
        if msg.velocity:
            out.velocity = [msg.velocity[i] for i in indices]
        if msg.effort:
            out.effort = [msg.effort[i] for i in indices]
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
