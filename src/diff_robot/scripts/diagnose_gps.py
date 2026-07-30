#!/usr/bin/env python3
"""
Diagnostic script to check GPS covariance and identify position jump issues
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import sys


class GPSDiagnostic(Node):
    def __init__(self):
        super().__init__('gps_diagnostic')
        
        self.gps_received = False
        self.odom_received = False
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('Waiting for GPS and odom messages...')
        
        # Timer to check after 5 seconds
        self.timer = self.create_timer(5.0, self.check_status)
    
    def gps_callback(self, msg):
        self.gps_received = True
        self.get_logger().info(f'GPS Fix received:')
        self.get_logger().info(f'  Position: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}')
        self.get_logger().info(f'  Covariance type: {msg.position_covariance_type}')
        self.get_logger().info(f'  Covariance: {msg.position_covariance}')
        
        # Check if covariance is all zeros
        if all(v == 0.0 for v in msg.position_covariance):
            self.get_logger().error('❌ GPS covariance is ALL ZEROS - this will cause EKF position jumps!')
        elif msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            self.get_logger().error('❌ GPS covariance type is UNKNOWN - EKF may treat measurements as perfect!')
        else:
            self.get_logger().info('✅ GPS covariance looks reasonable')
    
    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'Odom received:')
            self.get_logger().info(f'  Position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}')
            self.get_logger().info(f'  Publishers count: Check with "ros2 topic info /odom -v"')
    
    def check_status(self):
        if not self.gps_received:
            self.get_logger().error('❌ No GPS messages received after 5 seconds')
        if not self.odom_received:
            self.get_logger().error('❌ No odom messages received after 5 seconds')
        
        self.get_logger().info('\n=== DIAGNOSTIC COMPLETE ===')
        self.get_logger().info('Run these commands for more info:')
        self.get_logger().info('  ros2 topic info /odom -v')
        self.get_logger().info('  ros2 topic echo /gps/fix --once')
        self.get_logger().info('  ros2 run tf2_ros tf2_echo map odom')
        
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GPSDiagnostic()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
