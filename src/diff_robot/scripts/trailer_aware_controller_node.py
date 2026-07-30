#!/usr/bin/env python3
"""
Trailer-Aware Controller Node for Differential Drive Robot with Trailer
Converts Nav2 commands to trailer-safe commands using inverse kinematics
Adapted for differential drive (not skid-steer)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TrailerAwareControllerNode(Node):
    def __init__(self):
        super().__init__('trailer_aware_controller_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/cmd_vel_nav2')
        self.declare_parameter('output_topic', '/cmd_vel_raw')
        self.declare_parameter('hitch_distance', 0.50)
        self.declare_parameter('beta_limit_deg', 45.0)
        self.declare_parameter('max_beta_deg', 35.0)
        self.declare_parameter('beta_lookahead_time', 1.0)
        self.declare_parameter('max_stationary_omega', 1.0)
        self.declare_parameter('use_inverse_kinematics', True)
        self.declare_parameter('adaptive_velocity_reduction', True)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.d = self.get_parameter('hitch_distance').value
        self.beta_limit_deg = self.get_parameter('beta_limit_deg').value
        self.max_beta_deg = self.get_parameter('max_beta_deg').value
        self.lookahead_time = float(self.get_parameter('beta_lookahead_time').value)
        self.max_stationary_omega = float(self.get_parameter('max_stationary_omega').value)
        self.use_inverse_kinematics = bool(self.get_parameter('use_inverse_kinematics').value)
        self.adaptive_velocity_reduction = bool(
            self.get_parameter('adaptive_velocity_reduction').value
        )
        
        # Calculate minimum turning radius
        beta_limit_rad = math.radians(self.beta_limit_deg)
        self.r_min = self.d / math.sin(beta_limit_rad)
        self.max_beta = math.radians(self.max_beta_deg)
        
        # State
        self.beta = 0.0
        self.last_cmd = Twist()
        self.last_v_out = 0.0
        self.last_omega_out = 0.0
        
        # Subscribers and publishers
        self.create_subscription(Float64, '/trailer/beta', self.beta_callback, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        
        self.get_logger().info(
            f'Trailer-aware controller: {self.input_topic} -> {self.output_topic}, '
            f'd={self.d:.2f} m, R_min={self.r_min:.2f} m, '
            f'max_beta={self.max_beta_deg:.1f} deg, '
            f'inverse_kin={self.use_inverse_kinematics}'
        )
    
    def beta_callback(self, msg):
        self.beta = msg.data
    
    def cmd_callback(self, msg):
        safe_cmd = self.transform_cmd(msg)
        self.cmd_pub.publish(safe_cmd)
    
    def transform_cmd(self, cmd):
        v = cmd.linear.x
        omega = cmd.angular.z
        abs_beta = abs(self.beta)
        
        # If using inverse kinematics, compute required tractor omega for desired trailer path
        if self.use_inverse_kinematics and abs(v) > 1e-3:
            v, omega = self.inverse_kinematics_transform(v, omega, abs_beta)
        
        # Adaptive velocity reduction based on beta
        if self.adaptive_velocity_reduction:
            v = self.adapt_velocity(v, abs_beta)
        
        # Enforce kinematic limits
        omega_max = self.compute_omega_max(v)
        if omega_max > 1e-6:
            omega = max(-omega_max, min(omega_max, omega))
        else:
            omega = 0.0
        
        out = Twist()
        out.linear.x = v
        out.angular.z = omega
        self.last_v_out = v
        self.last_omega_out = omega
        
        return out
    
    def inverse_kinematics_transform(self, v, omega, abs_beta):
        """
        Transform Nav2 command using trailer inverse kinematics.
        For a desired path curvature, compute the tractor omega that achieves it
        while keeping the trailer beta within safe limits.
        
        Kinematic model for differential drive with trailer:
        beta_dot = -omega - (v/d) * sin(beta)
        """
        # Current beta dynamics: beta_dot = -omega - (v/d) * sin(beta)
        # For steady-state turning (beta_dot = 0): omega = -(v/d) * sin(beta)
        
        # Predict steady-state beta for current command
        if abs(omega) > 1e-3:
            # Current turning radius
            R = self.compute_turning_radius(v, omega)
            if not math.isinf(R):
                # Steady-state beta for this radius
                beta_steady = math.copysign(math.atan(self.d / R), omega)
                abs_beta_steady = abs(beta_steady)
                
                # If steady-state beta would be too high, reduce curvature
                if abs_beta_steady > self.max_beta:
                    # Compute max safe curvature
                    max_curvature = math.tan(self.max_beta) / self.d
                    omega_safe = v * max_curvature * (1.0 if omega > 0 else -1.0)
                    
                    # Smooth transition
                    alpha = 0.3
                    omega = (1.0 - alpha) * omega + alpha * omega_safe
        
        # Add beta correction term to align trailer with tractor
        if abs_beta > 0.1:
            # Proportional correction to reduce beta
            correction_gain = 0.5
            omega_correction = correction_gain * (v / self.d) * math.sin(self.beta)
            omega += omega_correction
        
        return v, omega
    
    def adapt_velocity(self, v, abs_beta):
        """Reduce velocity when beta is high to maintain stability."""
        if abs_beta < 0.2:
            return v  # No reduction for small beta
        
        # Linear reduction: scale = 1.0 at beta=0, scale = 0.5 at beta=max_beta
        scale = 1.0 - 0.5 * (abs_beta / self.max_beta)
        scale = max(0.5, min(1.0, scale))
        
        return v * scale
    
    def compute_omega_max(self, v):
        """Compute maximum angular velocity based on kinematic limits."""
        if abs(v) < 0.03:
            return self.max_stationary_omega

        beta_limit_rad = math.radians(self.beta_limit_deg)
        k_omega = math.sin(beta_limit_rad) / self.d
        return k_omega * abs(v)
    
    def compute_turning_radius(self, v, omega):
        """Compute turning radius from v and omega."""
        return abs(v / omega) if abs(omega) > 1e-6 else float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = TrailerAwareControllerNode()
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
