#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Parameters
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.40).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.075).value
        self.hitch_distance = self.declare_parameter('hitch_distance', 0.50).value
        self.publish_rate = self.declare_parameter('publish_rate', 50.0).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        
        # State - Robot (base_footprint)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # State - Trailer (trailer_footprint)
        self.trailer_x = 0.0
        self.trailer_y = 0.0
        self.trailer_theta = 0.0
        self.last_trailer_left_pos = 0.0
        self.last_trailer_right_pos = 0.0
        self.beta = 0.0  # Angle d'attelage
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.trailer_odom_pub = self.create_publisher(Odometry, '/odometry/trailer', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)
        
        self.get_logger().info('Odometry Publisher started (robot + trailer)')

    def joint_callback(self, msg):
        try:
            # Robot wheels
            left_idx = msg.name.index('rear_left_joint')
            right_idx = msg.name.index('rear_right_joint')
            
            # Trailer wheels
            trailer_left_idx = msg.name.index('trailer_wheel_left_joint')
            trailer_right_idx = msg.name.index('trailer_wheel_right_joint')
            
            # Hitch joint
            hitch_idx = msg.name.index('hitch_joint')
            
            # Get current positions and velocities
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]
            left_vel = msg.velocity[left_idx]
            right_vel = msg.velocity[right_idx]
            trailer_left_pos = msg.position[trailer_left_idx]
            trailer_right_pos = msg.position[trailer_right_idx]
            trailer_left_vel = msg.velocity[trailer_left_idx]
            trailer_right_vel = msg.velocity[trailer_right_idx]
            beta = msg.position[hitch_idx]
            
            # Update time
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            # Calculate robot velocities from position differences (more stable than joint velocities)
            delta_left = left_pos - self.last_left_pos
            delta_right = right_pos - self.last_right_pos
            
            if dt > 0.001:  # Avoid division by zero
                linear_vel = (delta_right + delta_left) * self.wheel_radius / (2.0 * dt)
                angular_vel = (delta_right - delta_left) * self.wheel_radius / (self.wheel_separation * dt)
            else:
                linear_vel = 0.0
                angular_vel = 0.0
            
            # Calculate trailer velocities from position differences
            delta_trailer_left = trailer_left_pos - self.last_trailer_left_pos
            delta_trailer_right = trailer_right_pos - self.last_trailer_right_pos
            
            if dt > 0.001:
                trailer_linear_vel = (delta_trailer_right + delta_trailer_left) * self.wheel_radius / (2.0 * dt)
            else:
                trailer_linear_vel = 0.0
            
            if dt > 0:
                # Update robot pose
                self.x += linear_vel * math.cos(self.theta) * dt
                self.y += linear_vel * math.sin(self.theta) * dt
                self.theta += angular_vel * dt
                
                # Update trailer pose (kinematic model for trailer)
                self.beta = beta
                trailer_angular_vel = linear_vel * math.sin(self.beta) / self.hitch_distance
                
                self.trailer_theta += trailer_angular_vel * dt
                self.trailer_x += trailer_linear_vel * math.cos(self.trailer_theta) * dt
                self.trailer_y += trailer_linear_vel * math.sin(self.trailer_theta) * dt
            
            # Store for next iteration
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_trailer_left_pos = trailer_left_pos
            self.last_trailer_right_pos = trailer_right_pos
            self.last_time = current_time
            
            # Store velocities for publishing
            self.linear_vel = linear_vel
            self.angular_vel = angular_vel
            self.trailer_linear_vel = trailer_linear_vel
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Joint state error: {e}')

    def publish_odometry(self):
        # Publish robot odometry
        self.publish_robot_odometry()
        
        # Publish trailer odometry
        self.publish_trailer_odometry()

    def publish_robot_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        if hasattr(self, 'linear_vel'):
            odom.twist.twist.linear.x = self.linear_vel
            odom.twist.twist.angular.z = self.angular_vel
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0
        
        # Covariance
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[35] = 0.01
        
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[35] = 0.01
        
        self.odom_pub.publish(odom)
        
        # Publish TF for robot (only if enabled - let SLAM Toolbox handle map-odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(t)

    def publish_trailer_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'trailer_footprint'
        
        # Position
        odom.pose.pose.position.x = self.trailer_x
        odom.pose.pose.position.y = self.trailer_y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.trailer_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.trailer_theta / 2.0)
        
        # Velocity
        if hasattr(self, 'trailer_linear_vel'):
            odom.twist.twist.linear.x = self.trailer_linear_vel
        else:
            odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Covariance
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.0002
        odom.pose.covariance[7] = 0.0002
        odom.pose.covariance[35] = 0.02
        
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 0.0002
        odom.twist.covariance[35] = 0.02
        
        self.trailer_odom_pub.publish(odom)
        
        # Publish TF for trailer
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'trailer_footprint'
        
        t.transform.translation.x = self.trailer_x
        t.transform.translation.y = self.trailer_y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.trailer_theta / 2.0)
        t.transform.rotation.w = math.cos(self.trailer_theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
