#!/usr/bin/env python3
"""
Trailer Dashboard Node - Real-time monitoring for differential drive robot with trailer
Adapted from Husky project for differential drive
"""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String


class TrailerDashboardNode(Node):
    def __init__(self):
        super().__init__('trailer_dashboard_node')
        
        self.declare_parameter('beta_limit_deg', 45.0)
        self.declare_parameter('hitch_distance', 0.50)
        self.beta_limit_deg = float(self.get_parameter('beta_limit_deg').value)
        self.d = float(self.get_parameter('hitch_distance').value)
        
        self.cmd_vel = None
        self.odom = None
        self.beta = None
        self.metrics = None
        self.risk = 'Waiting for /trailer/risk...'
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/trailer/beta', self.beta_callback, 10)
        self.create_subscription(Float64MultiArray, '/trailer/metrics', self.metrics_callback, 10)
        self.create_subscription(String, '/trailer/risk', self.risk_callback, 10)
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg
    
    def odom_callback(self, msg: Odometry):
        self.odom = msg
    
    def beta_callback(self, msg: Float64):
        self.beta = msg.data
    
    def metrics_callback(self, msg: Float64MultiArray):
        self.metrics = msg.data
    
    def risk_callback(self, msg: String):
        self.risk = msg.data


class TrailerDashboardApp:
    def __init__(self, node: TrailerDashboardNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Diff Drive Robot + Trailer Dashboard')
        self.root.geometry('720x430')
        self.root.minsize(680, 390)
        
        self.values = {}
        self.progress = None
        self.status_label = None
        
        self.build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.update_loop()
    
    def build_ui(self):
        main = ttk.Frame(self.root, padding=14)
        main.pack(fill=tk.BOTH, expand=True)
        
        title = ttk.Label(main, text='Diff Drive Robot + Trailer Live Dashboard', 
                         font=('TkDefaultFont', 15, 'bold'))
        title.pack(anchor=tk.W, pady=(0, 10))
        
        grid = ttk.Frame(main)
        grid.pack(fill=tk.BOTH, expand=True)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        
        self.add_section(grid, 'Command /cmd_vel', 0, 0, [
            ('linear_x', 'Linear x', 'waiting'),
            ('angular_z', 'Angular z', 'waiting'),
        ])
        self.add_section(grid, 'Odometry /odom', 0, 1, [
            ('odom_x', 'X', 'waiting'),
            ('odom_y', 'Y', 'waiting'),
            ('theta', 'Theta', 'waiting'),
            ('distance', 'Distance', 'waiting'),
        ])
        self.add_section(grid, 'Trailer Angle', 1, 0, [
            ('beta', 'Beta', 'waiting'),
            ('beta_dot', 'Beta dot', 'waiting'),
            ('limit', 'Limit', f'{self.node.beta_limit_deg:.1f} deg'),
            ('margin', 'Margin', 'waiting'),
        ])
        self.add_section(grid, 'Motion Metrics', 1, 1, [
            ('velocity', 'Velocity', 'waiting'),
            ('omega', 'Omega', 'waiting'),
            ('radius', 'Turn radius', 'waiting'),
            ('omega_max', 'Omega max', 'waiting'),
        ])
        
        bottom = ttk.Frame(main)
        bottom.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(bottom, text='Beta usage').pack(anchor=tk.W)
        self.progress = ttk.Progressbar(bottom, orient=tk.HORIZONTAL, mode='determinate', maximum=100.0)
        self.progress.pack(fill=tk.X, pady=(3, 8))
        
        self.status_label = ttk.Label(bottom, text='Risk: waiting', font=('TkDefaultFont', 11, 'bold'))
        self.status_label.pack(anchor=tk.W)
    
    def add_section(self, parent, title, row, column, rows):
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.grid(row=row, column=column, sticky='nsew', padx=6, pady=6)
        frame.columnconfigure(1, weight=1)
        
        for index, (key, label, default) in enumerate(rows):
            ttk.Label(frame, text=label).grid(row=index, column=0, sticky=tk.W, pady=3)
            value = ttk.Label(frame, text=default)
            value.grid(row=index, column=1, sticky=tk.E, pady=3)
            self.values[key] = value
    
    def update_loop(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.refresh_values()
        self.root.after(100, self.update_loop)
    
    def refresh_values(self):
        if self.node.cmd_vel is not None:
            self.values['linear_x'].config(text=f'{self.node.cmd_vel.linear.x:.2f} m/s')
            self.values['angular_z'].config(text=f'{self.node.cmd_vel.angular.z:.2f} rad/s')
        
        if self.node.odom is not None:
            self.values['odom_x'].config(text=f'{self.node.odom.pose.pose.position.x:.2f} m')
            self.values['odom_y'].config(text=f'{self.node.odom.pose.pose.position.y:.2f} m')
            
            # Extract theta from quaternion
            q = self.node.odom.pose.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.values['theta'].config(text=f'{math.degrees(theta):.1f} deg')
        
        if self.node.beta is not None:
            beta_deg = math.degrees(self.node.beta)
            margin = self.node.beta_limit_deg - abs(beta_deg)
            usage = min(100.0, abs(beta_deg) / self.node.beta_limit_deg * 100.0)
            
            self.values['beta'].config(text=f'{beta_deg:.1f} deg')
            self.values['margin'].config(text=f'{margin:.1f} deg')
            self.progress['value'] = usage
        
        if self.node.metrics is not None and len(self.node.metrics) >= 4:
            # Assuming metrics format: [v, omega, beta_dot, omega_max, ...]
            velocity = self.node.metrics[0] if len(self.node.metrics) > 0 else 0.0
            omega = self.node.metrics[1] if len(self.node.metrics) > 1 else 0.0
            beta_dot = self.node.metrics[2] if len(self.node.metrics) > 2 else 0.0
            omega_max = self.node.metrics[3] if len(self.node.metrics) > 3 else 0.0
            
            radius = abs(velocity / omega) if abs(omega) > 1e-6 else float('inf')
            radius_text = 'inf' if math.isinf(radius) else f'{radius:.2f} m'
            
            self.values['beta_dot'].config(text=f'{math.degrees(beta_dot):.1f} deg/s')
            self.values['velocity'].config(text=f'{velocity:.2f} m/s')
            self.values['omega'].config(text=f'{omega:.2f} rad/s')
            self.values['omega_max'].config(text=f'{omega_max:.2f} rad/s')
            self.values['radius'].config(text=radius_text)
        
        self.status_label.config(text=f'Risk: {self.node.risk}')
    
    def close(self):
        self.root.destroy()
    
    def run(self):
        import signal
        signal.signal(signal.SIGINT, lambda *_: self.root.quit())
        signal.signal(signal.SIGTERM, lambda *_: self.root.quit())
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TrailerDashboardNode()
    app = TrailerDashboardApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
