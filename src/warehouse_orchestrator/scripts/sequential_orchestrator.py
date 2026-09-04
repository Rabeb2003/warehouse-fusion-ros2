#!/usr/bin/env python3
"""
Sequential Mission Orchestrator
Waits for trailer navigation to complete, then launches Panda pick and place.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import subprocess
import sys


class SequentialOrchestrator(Node):
    def __init__(self):
        super().__init__('sequential_orchestrator')
        
        self.get_logger().info('==================================================')
        self.get_logger().info('   SEQUENTIAL MISSION ORCHESTRATOR')
        self.get_logger().info('==================================================')
        
        # Navigation goal for trailer (near Panda arm)
        self.docking_pose = PoseStamped()
        self.docking_pose.header.frame_id = 'map'
        self.docking_pose.pose.position.x = -1.0
        self.docking_pose.pose.position.y = 3.5
        self.docking_pose.pose.position.z = 0.0
        self.docking_pose.pose.orientation.w = 1.0
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Wait for navigation to be ready
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')
        
        # Send navigation goal
        self.get_logger().info('Sending trailer to docking location...')
        self.navigator.goToPose(self.docking_pose)
        
        # Wait for goal completion
        self.get_logger().info('Waiting for navigation to complete...')
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f}m'
                )
        
        # Check result
        result = self.navigator.getResult()
        if result == 1:  # Success
            self.get_logger().info('Navigation completed successfully!')
            self.get_logger().info('Trailer is now at docking position.')
            self.get_logger().info('Panda pick and place is already running via panda_stack.')
        else:
            self.get_logger().error('Navigation failed!')


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = SequentialOrchestrator()
    
    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
