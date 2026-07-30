#!/usr/bin/env python3
"""
Trailer Joint Publisher
Subscribes to joint states from Gazebo, extracts the hitch joint position,
and publishes it as the beta angle for the safety stabilizer node.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class TrailerJointPublisher(Node):
    def __init__(self):
        super().__init__('trailer_joint_publisher')
        
        # Parameters
        self.declare_parameter('hitch_joint_name', 'hitch_joint')
        self.declare_parameter('beta_topic', '/trailer/beta')
        
        self.hitch_joint_name = self.get_parameter('hitch_joint_name').value
        beta_topic = self.get_parameter('beta_topic').value
        
        # Publisher for beta angle
        self.beta_pub = self.create_publisher(Float64, beta_topic, 10)
        
        # Subscriber to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.get_logger().info('Trailer Joint Publisher started')
        self.get_logger().info(f'Subscribed to /joint_states to extract {self.hitch_joint_name}')
        self.get_logger().info(f'Publishing beta angle to {beta_topic}')
        
    def joint_state_callback(self, msg):
        """Extract hitch joint position and publish as beta"""
        if self.hitch_joint_name in msg.name:
            idx = msg.name.index(self.hitch_joint_name)
            if idx < len(msg.position):
                beta_msg = Float64()
                beta_msg.data = msg.position[idx]
                self.beta_pub.publish(beta_msg)


def main():
    rclpy.init()
    node = TrailerJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
