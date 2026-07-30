#!/usr/bin/env python3
"""
Hitch Angle Stabilizer Node (Anti-Jackknifing CBF Safety Filter)
================================================================
Implements a Control Barrier Function (CBF) to ensure the articulation angle (beta)
remains within safe boundaries, avoiding jackknifing and sudden unsafe turns.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class HitchAngleStabilizer(Node):
    def __init__(self):
        super().__init__('hitch_angle_stabilizer')
        
        # Declare parameters
        self.declare_parameter('hitch_distance', 0.50)      # d (m)
        self.declare_parameter('beta_max_deg', 40.0)         # Max angle forward (deg)
        self.declare_parameter('beta_max_deg_rev', 20.0)     # Max angle reverse (deg)
        self.declare_parameter('cbf_gain', 2.0)              # k (forward)
        self.declare_parameter('cbf_gain_rev', 3.0)          # k (reverse)
        self.declare_parameter('max_reverse_speed', 0.15)    # m/s
        self.declare_parameter('enable_linear_scaling', True)
        
        # Get parameter values
        self.d = self.get_parameter('hitch_distance').value
        self.beta_max = math.radians(self.get_parameter('beta_max_deg').value)
        self.beta_max_rev = math.radians(self.get_parameter('beta_max_deg_rev').value)
        self.k = self.get_parameter('cbf_gain').value
        self.k_rev = self.get_parameter('cbf_gain_rev').value
        self.max_rev_speed = self.get_parameter('max_reverse_speed').value
        self.linear_scaling = self.get_parameter('enable_linear_scaling').value
        
        # State
        self.beta = 0.0
        
        # Subscribers
        self.beta_sub = self.create_subscription(
            Float64, '/trailer/beta', self.beta_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publisher
        self.safe_cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_safe', 10
        )
        
        self.get_logger().info('Hitch Angle Stabilizer (Anti-Jackknifing CBF) Node initialized')
        self.get_logger().info(f'  hitch_distance (d) = {self.d:.3f} m')
        self.get_logger().info(f'  beta_max = {math.degrees(self.beta_max):.1f}° (forward) / {math.degrees(self.beta_max_rev):.1f}° (reverse)')
        self.get_logger().info(f'  cbf_gain (k) = {self.k:.2f} (forward) / {self.k_rev:.2f} (reverse)')
        
    def beta_callback(self, msg):
        self.beta = msg.data
        
    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Determine parameters based on direction
        is_reversing = v < 0.0
        
        if is_reversing:
            b_max = self.beta_max_rev
            gain = self.k_rev
            # Limit reverse speed
            v_cmd = max(v, -self.max_rev_speed)
        else:
            b_max = self.beta_max
            gain = self.k
            v_cmd = v
            
        # 1. Scaling linear velocity in curves (prudent turning)
        if self.linear_scaling and not is_reversing:
            # Scale linear speed: decreases as beta approaches beta_max
            scale_factor = 1.0 - (abs(self.beta) / b_max) ** 2
            scale_factor = max(0.1, min(1.0, scale_factor))  # Keep at least 10% speed
            v_safe = v_cmd * scale_factor
        else:
            v_safe = v_cmd
            
        # 2. Control Barrier Function (CBF) constraints on omega
        # Dynamics: beta_dot = omega - (v/d)*sin(beta)
        # We need:
        # -gain * (b_max + beta) <= beta_dot <= gain * (b_max - beta)
        #
        # Substituting beta_dot:
        # -gain * (b_max + beta) <= omega - (v_safe/d)*sin(beta) <= gain * (b_max - beta)
        #
        # Thus:
        # omega_min = (v_safe/d)*sin(beta) - gain * (b_max + beta)
        # omega_max = (v_safe/d)*sin(beta) + gain * (b_max - beta)
        
        sin_beta = math.sin(self.beta)
        drift_term = (v_safe / self.d) * sin_beta
        
        omega_max = drift_term + gain * (b_max - self.beta)
        omega_min = drift_term - gain * (b_max + self.beta)
        
        # Ensure bounds are mathematically valid (omega_min should be <= omega_max)
        if omega_min > omega_max:
            omega_min, omega_max = omega_max, omega_min
            
        # Apply safety bounds
        omega_safe = max(omega_min, min(omega_max, omega))
        
        # Log if commands are active and being modified
        if abs(omega - omega_safe) > 0.05 and (abs(v) > 0.01 or abs(omega) > 0.01):
            self.get_logger().warn(
                f"Jackknifing/Torpillage prévenu ! commandé: {omega:.2f} rad/s -> sécurisé: {omega_safe:.2f} rad/s (beta: {math.degrees(self.beta):.1f}°)",
                throttle_duration_sec=1.0
            )
            
        # Publish safe command
        safe_msg = Twist()
        safe_msg.linear.x = v_safe
        safe_msg.linear.y = msg.linear.y
        safe_msg.linear.z = msg.linear.z
        safe_msg.angular.x = msg.angular.x
        safe_msg.angular.y = msg.angular.y
        safe_msg.angular.z = omega_safe
        
        self.safe_cmd_pub.publish(safe_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HitchAngleStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
