#! /usr/bin/env python3

"""
    Keyboard Teleop for Trailer Robot
    Adapted from original by Gaurav Gupta
    Includes kinematic limits for trailer hitch
"""


import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class _Getch:
    """
    Gets a single character from standard input. 
    Does not echo to the screen.
    """

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('trailer_keyboard_teleop')
        
        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('hitch_distance', 0.50)
        self.declare_parameter('beta_limit_deg', 45.0)
        self.declare_parameter('max_stationary_omega', 3.0)
        
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.d = self.get_parameter('hitch_distance').value
        self.beta_limit_deg = self.get_parameter('beta_limit_deg').value
        self.max_stationary_omega = self.get_parameter('max_stationary_omega').value
        
        self.vel_publisher = self.create_publisher(Twist, cmd_topic, 1)
        timer_period = 0.05
        self.timer = self.create_timer(
            timer_period, self.velocity_publish_event)
        self.cmd_vel_msg = Twist()
        
        # Beta subscription
        self.beta = 0.0
        self.create_subscription(Float64, '/trailer/beta', self.beta_callback, 10)

    def beta_callback(self, msg):
        self.beta = msg.data

    def set_vel(self, v, w):
        # Apply kinematic limits
        omega_out = self.saturate_omega(v, w)
        self.get_logger().info(f"v: {v:.2f}, w: {w:.2f} -> omega_out: {omega_out:.2f} rad/s, beta: {math.degrees(self.beta):.1f}°")
        self.cmd_vel_msg.linear.x = v
        self.cmd_vel_msg.angular.z = omega_out

    def saturate_omega(self, v: float, omega: float) -> float:
        """Saturate omega based on kinematic limits."""
        # Allow stationary rotation
        if abs(v) < 1e-3:
            if abs(omega) < 1e-6:
                return 0.0
            return max(-self.max_stationary_omega, min(self.max_stationary_omega, omega))
        
        # For moving robot, use a more permissive limit
        # Conservative estimate: omega_max = 2 * v / d
        omega_max = 2.0 * abs(v) / self.d
        
        if omega_max <= 0:
            return 0.0
        return max(-omega_max, min(omega_max, omega))

    def velocity_publish_event(self):
        self.vel_publisher.publish(self.cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    getch = _Getch()
    publish_node = TeleopPublisher()
    thread = threading.Thread(target=rclpy.spin,
                              args=(publish_node, ), daemon=True)
    # Thread for node's timer callback
    thread.start()
    v = 0.0
    w = 0.0
    dv = 0.1  # Linear velocity increment
    dw = 0.1  # Angular velocity increment
    
    publish_node.get_logger().info("\n" + "="*60)
    publish_node.get_logger().info("  TRAILER ROBOT KEYBOARD TELEOP")
    publish_node.get_logger().info("="*60)
    publish_node.get_logger().info("  w : increment linear velocity by %.1f" % dv)
    publish_node.get_logger().info("  s : decrement linear velocity by %.1f" % dv)
    publish_node.get_logger().info("  a : increment angular velocity by %.1f" % dw)
    publish_node.get_logger().info("  d : decrement angular velocity by %.1f" % dw)
    publish_node.get_logger().info("  space : zero velocity command")
    publish_node.get_logger().info("  q : QUIT")
    publish_node.get_logger().info("="*60)
    publish_node.get_logger().info(f"  Hitch distance: {publish_node.d:.2f}m")
    publish_node.get_logger().info(f"  Beta limit: {publish_node.beta_limit_deg:.1f}°")
    publish_node.get_logger().info(f"  Max stationary omega: {publish_node.max_stationary_omega:.2f} rad/s")
    publish_node.get_logger().info("="*60 + "\n")
    
    try:
        while (rclpy.ok()):
            key_in = getch()
            if key_in == "w":
                v += dv
            elif key_in == "s":
                v -= dv
            elif key_in == "d":
                w -= dw
            elif key_in == "a":
                w += dw
            elif key_in == "q":
                break
            else:
                v = 0.0
                w = 0.0
            publish_node.set_vel(v, w)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
