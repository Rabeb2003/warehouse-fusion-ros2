#!/usr/bin/env python3
"""
Dynamic Footprint Node - Models differential drive robot + trailer as series of disks
Adapted for differential drive (not skid-steer)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math


class DynamicFootprint(Node):
    def __init__(self):
        super().__init__('dynamic_footprint_node')
        
        # Parameters for differential drive robot + trailer
        self.declare_parameter('robot_length', 0.6)
        self.declare_parameter('robot_width', 0.4)
        self.declare_parameter('trailer_length', 0.8)
        self.declare_parameter('trailer_width', 0.5)
        self.declare_parameter('hitch_distance', 0.50)  # d: hitch -> trailer axle
        self.declare_parameter('hitch_offset', 0.0)  # offset from robot center to hitch
        
        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value
        self.trailer_length = self.get_parameter('trailer_length').value
        self.trailer_width = self.get_parameter('trailer_width').value
        self.d = self.get_parameter('hitch_distance').value
        self.hitch_offset = self.get_parameter('hitch_offset').value
        
        # Disk radius for footprint approximation
        self.declare_parameter('disk_radius', 0.12)
        self.disk_radius = self.get_parameter('disk_radius').value
        
        # Number of disks per segment
        self.robot_disks = max(3, int(self.robot_length / (2 * self.disk_radius)))
        self.trailer_disks = max(3, int(self.trailer_length / (2 * self.disk_radius)))
        
        # Robot state
        self.robot_pose = None
        self.trailer_angle = 0.0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.trailer_angle_sub = self.create_subscription(
            Float64,
            '/trailer/beta',
            self.trailer_angle_callback,
            10
        )
        
        # Publisher for dynamic footprint
        self.footprint_pub = self.create_publisher(
            Polygon,
            '/local_costmap/published_footprint',
            10
        )
        
        # Timer to update footprint
        self.footprint_timer = self.create_timer(0.1, self.update_footprint)
        
        self.get_logger().info(
            f'Dynamic Footprint Node initialized: '
            f'robot={self.robot_length}x{self.robot_width}m, '
            f'trailer={self.trailer_length}x{self.trailer_width}m, '
            f'd={self.d:.2f}m'
        )
    
    def odom_callback(self, msg):
        """Callback for robot odometry"""
        self.robot_pose = msg.pose.pose
    
    def trailer_angle_callback(self, msg):
        """Callback for trailer angle beta"""
        self.trailer_angle = msg.data
    
    def update_footprint(self):
        """Update and publish dynamic footprint"""
        if self.robot_pose is None:
            return
        
        # Compute dynamic footprint
        footprint = self.compute_footprint(self.robot_pose, self.trailer_angle)
        
        # Publish
        self.footprint_pub.publish(footprint)
    
    def compute_footprint(self, robot_pose, trailer_angle):
        """Compute dynamic footprint based on trailer angle"""
        footprint = Polygon()
        
        # Extract robot pose
        x_r, y_r, theta_r = self.pose_to_tuple(robot_pose)
        
        # Disks for robot (from rear to front)
        for i in range(self.robot_disks):
            x_local = -self.robot_length/2 + i * (self.robot_length / (self.robot_disks - 1))
            disk_pos = self.transform_point(x_local, 0, x_r, y_r, theta_r)
            footprint.points.append(Point32(x=disk_pos[0], y=disk_pos[1]))
        
        # Disks for trailer (offset by hitch angle)
        trailer_pose = self.compute_trailer_pose(x_r, y_r, theta_r, trailer_angle)
        x_t, y_t, theta_t = trailer_pose
        
        for i in range(self.trailer_disks):
            x_local = -self.trailer_length/2 + i * (self.trailer_length / (self.trailer_disks - 1))
            disk_pos = self.transform_point(x_local, 0, x_t, y_t, theta_t)
            footprint.points.append(Point32(x=disk_pos[0], y=disk_pos[1]))
        
        return footprint
    
    def pose_to_tuple(self, pose):
        """Convert pose to tuple (x, y, theta)"""
        x = pose.position.x
        y = pose.position.y
        
        # Extract angle from quaternion
        q = pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return (x, y, theta)
    
    def compute_trailer_pose(self, x_r, y_r, theta_r, beta):
        """Compute trailer pose based on robot pose and hitch angle"""
        # Hitch position relative to robot center
        hitch_x = x_r + self.hitch_offset * math.cos(theta_r)
        hitch_y = y_r + self.hitch_offset * math.sin(theta_r)
        
        # Trailer position (trailer axle is at distance d from hitch)
        # Trailer orientation is theta_r + beta
        theta_t = theta_r + beta
        x_t = hitch_x + self.d * math.cos(theta_t)
        y_t = hitch_y + self.d * math.sin(theta_t)
        
        return (x_t, y_t, theta_t)
    
    def transform_point(self, x_local, y_local, x_origin, y_origin, theta):
        """Transform local point to global coordinates"""
        x_global = x_origin + x_local * math.cos(theta) - y_local * math.sin(theta)
        y_global = y_origin + x_local * math.sin(theta) + y_local * math.cos(theta)
        return (x_global, y_global)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFootprint()
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
