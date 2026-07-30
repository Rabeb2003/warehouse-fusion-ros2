#!/usr/bin/env python3
"""
Wheel Synchronization Node
Synchronizes front wheels and trailer wheels with rear wheels for 4WD + trailer motorization
Uses Gazebo apply_joint_effort service to drive additional wheels
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort


class WheelSyncNode(Node):
    def __init__(self):
        super().__init__('wheel_sync_node')
        
        # Subscribe to joint states to get rear wheel velocities
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Gazebo service client for applying joint effort
        self.effort_client = self.create_client(
            ApplyJointEffort, '/apply_joint_effort')
        
        # Store rear wheel velocities
        self.rear_left_vel = 0.0
        self.rear_right_vel = 0.0
        
        # PID-like control parameters
        self.kp = 5.0  # Proportional gain
        self.kd = 0.5  # Derivative gain
        self.last_front_left_vel = 0.0
        self.last_front_right_vel = 0.0
        self.last_trailer_left_vel = 0.0
        self.last_trailer_right_vel = 0.0
        
        # Wait for service
        self.get_logger().info('Waiting for Gazebo apply_joint_effort service...')
        self.effort_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info('Wheel Sync Node started')
        self.get_logger().info('Synchronizing front and trailer wheels with rear wheels')
    
    def joint_state_callback(self, msg):
        """Read joint states and apply effort to synchronize wheels"""
        try:
            # Find rear wheel indices
            rear_left_vel = 0.0
            rear_right_vel = 0.0
            front_left_vel = 0.0
            front_right_vel = 0.0
            trailer_left_vel = 0.0
            trailer_right_vel = 0.0
            
            # Check if velocity array has data
            if len(msg.velocity) > 0:
                if 'rear_left_joint' in msg.name:
                    idx = msg.name.index('rear_left_joint')
                    if idx < len(msg.velocity):
                        rear_left_vel = msg.velocity[idx]
                
                if 'rear_right_joint' in msg.name:
                    idx = msg.name.index('rear_right_joint')
                    if idx < len(msg.velocity):
                        rear_right_vel = msg.velocity[idx]
                
                if 'front_left_joint' in msg.name:
                    idx = msg.name.index('front_left_joint')
                    if idx < len(msg.velocity):
                        front_left_vel = msg.velocity[idx]
                
                if 'front_right_joint' in msg.name:
                    idx = msg.name.index('front_right_joint')
                    if idx < len(msg.velocity):
                        front_right_vel = msg.velocity[idx]
                
                if 'trailer_wheel_left_joint' in msg.name:
                    idx = msg.name.index('trailer_wheel_left_joint')
                    if idx < len(msg.velocity):
                        trailer_left_vel = msg.velocity[idx]
                
                if 'trailer_wheel_right_joint' in msg.name:
                    idx = msg.name.index('trailer_wheel_right_joint')
                    if idx < len(msg.velocity):
                        trailer_right_vel = msg.velocity[idx]
            
            # Calculate effort using PD control to match velocities
            front_left_effort = self.kp * (rear_left_vel - front_left_vel) + self.kd * (self.last_front_left_vel - front_left_vel)
            front_right_effort = self.kp * (rear_right_vel - front_right_vel) + self.kd * (self.last_front_right_vel - front_right_vel)
            trailer_left_effort = self.kp * (rear_left_vel - trailer_left_vel) + self.kd * (self.last_trailer_left_vel - trailer_left_vel)
            trailer_right_effort = self.kp * (rear_right_vel - trailer_right_vel) + self.kd * (self.last_trailer_right_vel - trailer_right_vel)
            
            # Store last velocities
            self.last_front_left_vel = front_left_vel
            self.last_front_right_vel = front_right_vel
            self.last_trailer_left_vel = trailer_left_vel
            self.last_trailer_right_vel = trailer_right_vel
            
            # Apply effort to front and trailer wheels
            self.apply_effort('front_left_joint', front_left_effort)
            self.apply_effort('front_right_joint', front_right_effort)
            self.apply_effort('trailer_wheel_left_joint', trailer_left_effort)
            self.apply_effort('trailer_wheel_right_joint', trailer_right_effort)
            
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {e}')
    
    def apply_effort(self, joint_name, effort):
        """Apply effort to joint using Gazebo service"""
        try:
            request = ApplyJointEffort.Request()
            request.joint_name = joint_name
            request.effort = effort
            request.duration.sec = 0
            request.duration.nanosec = 50000000  # Apply for 50ms
            request.start_time.sec = 0
            request.start_time.nanosec = 0  # Start immediately
            
            future = self.effort_client.call_async(request)
            
        except Exception as e:
            self.get_logger().error(f'Error applying effort to {joint_name}: {e}')


def main():
    rclpy.init()
    node = WheelSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
