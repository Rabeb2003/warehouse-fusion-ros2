#!/usr/bin/env python3
"""
CMD Vel Safety Node - Anti-Jackknife Protection for Differential Drive Robot with Trailer
Adapted from Husky project for differential drive (not skid-steer)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SafetyNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_node')
        
        # Parameters
        self.declare_parameter('max_forward_speed', 0.26)
        self.declare_parameter('max_backward_speed', 0.15)
        self.declare_parameter('max_reverse_speed', 0.15)
        self.declare_parameter('max_stationary_omega', 1.0)
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('hitch_distance', 0.50)
        self.declare_parameter('beta_limit_deg', 45.0)
        
        self.max_v = self.get_parameter('max_forward_speed').value
        max_backward = self.get_parameter('max_backward_speed').value
        max_reverse = self.get_parameter('max_reverse_speed').value
        self.max_rev = max_backward if max_backward > 0 else max_reverse
        self.max_omega = self.get_parameter('max_stationary_omega').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.d = self.get_parameter('hitch_distance').value
        self.beta_limit_deg = self.get_parameter('beta_limit_deg').value
        
        # Calculate minimum turning radius based on hitch distance and beta limit
        beta_limit_rad = math.radians(self.beta_limit_deg)
        self.r_min = self.d / math.sin(beta_limit_rad)
        self.lambda_max = 1.0 / self.r_min  # 1/R_min
        
        # Current beta angle from trailer
        self.beta = 0.0
        
        # Subscribers and publishers
        self.sub = self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.beta_sub = self.create_subscription(Float64, '/trailer/beta', self.beta_callback, 10)
        self.pub = self.create_publisher(Twist, output_topic, 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(Float64, '/cmd_vel_safety/status', 10)
        
        self.get_logger().info(
            f'CMD Vel Safety Node started: d={self.d:.2f}m, '
            f'beta_limit={self.beta_limit_deg:.1f}°, R_min={self.r_min:.2f}m'
        )
    
    def beta_callback(self, msg):
        self.beta = msg.data
    
    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Limit velocity
        v_limited = max(-self.max_rev, min(self.max_v, v))
        
        # Adaptive omega limit based on beta angle
        beta_abs = abs(self.beta)
        beta_limit_rad = math.radians(self.beta_limit_deg)
        
        # Reduce omega_max as beta approaches the mechanical limit.
        beta_ratio = min(1.0, beta_abs / beta_limit_rad)
        omega_max_adaptive = self.max_omega * (1.0 - 0.35 * beta_ratio)
        
        if abs(v_limited) > 0.05:
            omega_max_kinematic = (math.sin(beta_limit_rad) / self.d) * abs(v_limited)
            omega_max = min(omega_max_adaptive, omega_max_kinematic)
        else:
            omega_max = omega_max_adaptive  # Allow controlled stationary rotation
        
        # Limit omega
        omega_limited = max(-omega_max, min(omega_max, omega))
        
        # Publish safe command
        out = Twist()
        out.linear.x = v_limited
        out.angular.z = omega_limited
        self.pub.publish(out)
        
        # Publish status (0=safe, 1=warning, 2=danger)
        beta_abs_deg = math.degrees(beta_abs)
        status = 0.0 if beta_abs_deg < 30.0 else (1.0 if beta_abs_deg < 40.0 else 2.0)
        self.status_pub.publish(Float64(data=status))
        
        self.get_logger().info(
            f'v={v_limited:.2f} omega={omega_limited:.3f} omega_max={omega_max:.3f} beta={math.degrees(self.beta):.1f}°',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
