#!/usr/bin/env python3
"""
Planning Scene Setup Node for MoveIt 2
Adds the red target block to the MoveIt planning scene for RViz visualization
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header

class PlanningSceneSetup(Node):
    def __init__(self):
        super().__init__('planning_scene_setup')
        self.get_logger().info("Setting up MoveIt Planning Scene with red target block...")
        
        # Publisher for planning scene
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Wait a bit for MoveIt to be ready
        import time
        time.sleep(3.0)
        
        # Add the red target block to the planning scene
        self.add_target_block()

    def make_color(self, object_id, red, green, blue, alpha=1.0):
        object_color = ObjectColor()
        object_color.id = object_id
        object_color.color = ColorRGBA(r=red, g=green, b=blue, a=alpha)
        return object_color
        
    def add_target_block(self):
        """Add the red target block to the planning scene"""
        
        # Create collision object
        co = CollisionObject()
        co.header = Header(frame_id="world")
        co.id = "target_block"
        
        # Define box shape (matching warehouse.sdf: 0.04 x 0.04 x 0.05)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.04, 0.04, 0.05]  # [x, y, z]
        co.primitives = [box]
        
        # Set pose (matching warehouse.sdf: 0.5, 0.0, 0.425)
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=0.425)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        co.primitive_poses = [pose]
        
        co.operation = CollisionObject.ADD
        
        # Create planning scene message
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [self.make_color(co.id, 0.9, 0.02, 0.02, 1.0)]
        
        # Publish to planning scene
        self.scene_pub.publish(scene)
        self.get_logger().info("✅ Added red target block to MoveIt planning scene at (0.5, 0.0, 0.425)")
        
        # Also add the pick table for visualization
        self.add_pick_table()
        self.add_place_table()
        
    def add_pick_table(self):
        """Add the pick table to the planning scene"""
        
        co = CollisionObject()
        co.header = Header(frame_id="world")
        co.id = "pick_table"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.5, 0.6, 0.4]  # matching warehouse.sdf
        co.primitives = [box]
        
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=0.2)  # matching warehouse.sdf
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        co.primitive_poses = [pose]
        
        co.operation = CollisionObject.ADD
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [self.make_color(co.id, 0.36, 0.36, 0.40, 0.85)]
        
        self.scene_pub.publish(scene)
        self.get_logger().info("✅ Added pick table to MoveIt planning scene")
        
    def add_place_table(self):
        """Add the place table to the planning scene"""
        
        co = CollisionObject()
        co.header = Header(frame_id="world")
        co.id = "place_table"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.4, 0.4, 0.4]  # matching warehouse.sdf
        co.primitives = [box]
        
        pose = Pose()
        pose.position = Point(x=0.0, y=0.5, z=0.2)  # matching warehouse.sdf
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        co.primitive_poses = [pose]
        
        co.operation = CollisionObject.ADD
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [self.make_color(co.id, 0.24, 0.43, 0.68, 0.85)]
        
        self.scene_pub.publish(scene)
        self.get_logger().info("✅ Added place table to MoveIt planning scene")

def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneSetup()
    try:
        # Keep node alive for a few seconds to ensure message is sent
        import time
        time.sleep(5.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
