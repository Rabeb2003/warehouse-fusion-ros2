#!/usr/bin/env python3
"""Sweep pose finder - teste plusieurs poses et détecte laquelle voit le rouge"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Image
import numpy as np
import time

class PoseSweeper(Node):
    def __init__(self):
        super().__init__('pose_sweeper')
        self._client = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.red_pixel_count = 0
        self.create_subscription(Image, '/wrist_camera/image_raw', self.img_cb, 10)
        self.joints = ['panda_joint1','panda_joint2','panda_joint3',
                       'panda_joint4','panda_joint5','panda_joint6','panda_joint7']

    def img_cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            r, g, b = arr[:,:,0].astype(int), arr[:,:,1].astype(int), arr[:,:,2].astype(int)
            red_mask = (r > 100) & (g < 80) & (b < 80)
            self.red_pixel_count = int(red_mask.sum())
        except Exception as e:
            self.get_logger().warn(f"Image processing error: {e}")

    def send_pose(self, positions, duration=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration)
        goal.trajectory.points = [pt]
        
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if gh and gh.accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf)
            return True
        return False

    def sweep(self):
        self._client.wait_for_server()
        self.get_logger().info("Starting pose sweep...")
        
        # Grille de candidats à tester (j2 = inclinaison épaule, j4 = coude)
        # Étendue plus large pour couvrir tout l'espace de la pick table
        candidates = []
        for j2 in [-0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6]:
            for j4 in [-2.6, -2.4, -2.2, -2.0, -1.8, -1.6, -1.4, -1.2]:
                for j6 in [1.2, 1.4, 1.6, 1.8, 2.0]:
                    candidates.append([0.0, j2, 0.0, j4, 0.0, j6, 0.785])

        self.get_logger().info(f"Testing {len(candidates)} pose candidates...")
        
        best = None
        for i, pose in enumerate(candidates):
            self.get_logger().info(f"[{i+1}/{len(candidates)}] Testing pose: {pose}")
            
            if self.send_pose(pose, duration=2.0):
                time.sleep(1.5)  # Attendre stabilisation caméra
                
                # Échantillonner plusieurs images
                red_counts = []
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.2)
                    red_counts.append(self.red_pixel_count)
                
                avg_red = sum(red_counts) // len(red_counts)
                self.get_logger().info(f"  -> Red pixels: {avg_red}")
                
                if best is None or avg_red > best[1]:
                    best = (pose, avg_red)
            else:
                self.get_logger().warn(f"  -> Failed to send pose")

        if best:
            self.get_logger().info(f"🏆 MEILLEURE POSE: {best[0]} avec {best[1]} pixels rouges")
            self.get_logger().info(f"Collez ces valeurs dans pick_and_place_demo.py comme observe_pose")
        else:
            self.get_logger().error("Aucune pose valide trouvée")

def main():
    rclpy.init()
    node = PoseSweeper()
    try:
        node.sweep()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
