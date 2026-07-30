#!/usr/bin/env python3
"""
Manual Trajectory Recorder for Franka Panda
- Records joint states while you manually move the robot in RViz/Gazebo
- Saves trajectory to YAML file for replay
- Useful for creating pick and place motions manually
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
from datetime import datetime
import os

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        self.recording = False
        self.trajectory = []
        self.start_time = None
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Create output directory
        self.output_dir = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'recorded_trajectories'
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info("Trajectory Recorder Ready")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info("Commands: start, stop, save, quit")
        
        # Start input thread for commands
        import threading
        self.input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.input_thread.start()
    
    def joint_callback(self, msg):
        if self.recording:
            # Record joint positions for panda arm joints
            panda_joints = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                          'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
            
            joint_positions = []
            for joint_name in panda_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    joint_positions.append(msg.position[idx])
            
            if len(joint_positions) == 7:
                timestamp = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
                self.trajectory.append({
                    'time': timestamp,
                    'positions': joint_positions
                })
    
    def command_loop(self):
        while rclpy.ok():
            try:
                cmd = input("recorder> ").strip().lower()
                
                if cmd == 'start':
                    self.start_recording()
                elif cmd == 'stop':
                    self.stop_recording()
                elif cmd == 'save':
                    self.save_trajectory()
                elif cmd == 'quit':
                    if self.recording:
                        self.stop_recording()
                    self.save_trajectory()
                    rclpy.shutdown()
                    break
                elif cmd == 'help':
                    print("Commands: start, stop, save, quit, help")
                else:
                    print(f"Unknown command: {cmd}")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def start_recording(self):
        if not self.recording:
            self.recording = True
            self.trajectory = []
            self.start_time = self.get_clock().now().nanoseconds
            self.get_logger().info("🔴 Recording started - Move robot in RViz/Gazebo")
        else:
            self.get_logger().warn("Already recording")
    
    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.get_logger().info(f"⏹️ Recording stopped - {len(self.trajectory)} points recorded")
        else:
            self.get_logger().warn("Not recording")
    
    def save_trajectory(self):
        if not self.trajectory:
            self.get_logger().warn("No trajectory to save")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f'trajectory_{timestamp}.yaml')
        
        trajectory_data = {
            'trajectory': self.trajectory,
            'joint_names': ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                          'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
            'recorded_at': timestamp
        }
        
        with open(filename, 'w') as f:
            yaml.dump(trajectory_data, f, default_flow_style=None)
        
        self.get_logger().info(f"💾 Trajectory saved to: {filename}")
        self.trajectory = []

def main(args=None):
    rclpy.init(args=args)
    recorder = TrajectoryRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if recorder.recording:
            recorder.stop_recording()
            recorder.save_trajectory()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
