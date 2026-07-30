#!/usr/bin/env python3
"""Panneau de controle : avancer, reculer, tourner gauche/droite."""
import math
import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class TrailerControlGUI(Node):
    def __init__(self):
        super().__init__('trailer_control_node')
        
        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('hitch_distance', 0.50)
        self.declare_parameter('beta_limit_deg', 45.0)
        self.declare_parameter('max_stationary_omega', 3.0)
        
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.d = self.get_parameter('hitch_distance').value
        self.beta_limit_deg = self.get_parameter('beta_limit_deg').value
        self.max_stationary_omega = self.get_parameter('max_stationary_omega').value
        
        self.publisher_ = self.create_publisher(Twist, cmd_topic, 10)
        
        self.beta = 0.0
        self.create_subscription(Float64, '/trailer/beta', self.beta_callback, 10)
        
        self.linear_speed = 0.70
        self.angular_speed = 4.0
        self.turn_linear_speed = 0.50
        self.toggle_mode = False
        self._active_drive_key = None
        
        self._cmd_lock = threading.Lock()
        self.active_lin = 0.0
        self.active_ang = 0.0
        self._last_logged = (0.0, 0.0)
        
        self.create_timer(0.05, self.timer_publish)
        
        self.get_logger().info('Trailer Control Node started')
        self.get_logger().info(f'd={self.d:.2f}m, beta_limit={self.beta_limit_deg:.1f}°')
        
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        self.gui_thread.start()
    
    def beta_callback(self, msg):
        self.beta = msg.data
    
    def timer_publish(self):
        with self._cmd_lock:
            lin = self.active_lin
            ang = self.active_ang
        self._publish(lin, ang)
        if abs(lin) > 1e-6 or abs(ang) > 1e-6:
            key = (round(lin, 2), round(ang, 2))
            if key != self._last_logged:
                self._last_logged = key
                omega_out = self.saturate_omega(lin, ang)
                self.get_logger().info(
                    f'CMD: v={lin:.2f}, omega={omega_out:.2f} rad/s'
                )
        elif self._last_logged != (0.0, 0.0):
            self._last_logged = (0.0, 0.0)
            self.get_logger().info('CMD: stop')
    
    def _publish(self, v: float, omega: float):
        omega_out = self.saturate_omega(v, omega)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega_out
        self.publisher_.publish(cmd)
    
    def saturate_omega(self, v: float, omega: float) -> float:
        """Saturate omega based on kinematic limits."""
        # Allow stationary rotation
        if abs(v) < 1e-3:
            if abs(omega) < 1e-6:
                return 0.0
            return max(-self.max_stationary_omega, min(self.max_stationary_omega, omega))
        
        # For moving robot, use a more permissive limit
        # Only limit if omega would cause jackknife (beta > 45°)
        # Conservative estimate: omega_max = 2 * v / d
        omega_max = 2.0 * abs(v) / self.d
        
        if omega_max <= 0:
            return 0.0
        return max(-omega_max, min(omega_max, omega))
    
    def _set_motion(self, lin: float, ang: float):
        with self._cmd_lock:
            self.active_lin = lin
            self.active_ang = ang
    
    def start_motion(self, lin_dir: float, ang_dir: float, use_turn_speed: bool = False):
        v_base = self.turn_linear_speed if use_turn_speed else self.linear_speed
        lin = lin_dir * v_base
        ang = ang_dir * self.angular_speed
        self._set_motion(lin, ang)
    
    def stop_motion(self):
        self._set_motion(0.0, 0.0)
        self._active_drive_key = None
    
    def on_drive_press(self, lin_dir: float, ang_dir: float, use_turn_speed: bool = False):
        key = (lin_dir, ang_dir, use_turn_speed)
        if self.toggle_mode:
            with self._cmd_lock:
                moving = abs(self.active_lin) > 1e-6 or abs(self.active_ang) > 1e-6
                same = self._active_drive_key == key
            if moving and same:
                self.stop_motion()
            else:
                self.start_motion(lin_dir, ang_dir, use_turn_speed)
                self._active_drive_key = key
        else:
            self.start_motion(lin_dir, ang_dir, use_turn_speed)
            self._active_drive_key = key
    
    def on_drive_release(self):
        if not self.toggle_mode:
            self.stop_motion()
    
    def run_gui(self):
        try:
            self._build_gui()
        except Exception as exc:
            self.get_logger().error(f'Erreur interface teleop: {exc}')
    
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('Controle Robot + Remorque')
        self.root.geometry('520x560')
        self.root.configure(bg='#2c3e50')

        tk.Label(
            self.root,
            text='Maintenez un bouton pour conduire (relacher = STOP)',
            font=('Helvetica', 11, 'bold'), fg='white', bg='#2c3e50',
        ).pack(pady=6)

        mode_frame = tk.Frame(self.root, bg='#2c3e50')
        mode_frame.pack(pady=2)
        self.toggle_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            mode_frame,
            text='Mode clic (decoche = maintenir le bouton)',
            variable=self.toggle_var,
            command=self._on_toggle_changed,
            fg='white', bg='#2c3e50', selectcolor='#34495e',
            activebackground='#2c3e50', activeforeground='white',
            font=('Helvetica', 10),
        ).pack()

        speed_frame = tk.Frame(self.root, bg='#2c3e50')
        speed_frame.pack(pady=8)

        tk.Label(speed_frame, text='Vitesse lineaire (v):', fg='white', bg='#2c3e50').grid(row=0, column=0)
        self.lin_scale = tk.Scale(
            speed_frame, from_=0.15, to=1.2, resolution=0.05, orient=tk.HORIZONTAL,
            command=self.update_speeds, bg='#34495e', fg='white', length=220,
        )
        self.lin_scale.set(self.linear_speed)
        self.lin_scale.grid(row=0, column=1)

        tk.Label(speed_frame, text='Vitesse angulaire (omega):', fg='white', bg='#2c3e50').grid(row=1, column=0)
        self.ang_scale = tk.Scale(
            speed_frame, from_=0.15, to=3.0, resolution=0.05, orient=tk.HORIZONTAL,
            command=self.update_speeds, bg='#34495e', fg='white', length=220,
        )
        self.ang_scale.set(self.angular_speed)
        self.ang_scale.grid(row=1, column=1)

        self.status = tk.Label(self.root, text='', fg='#f1c40f', bg='#2c3e50', font=('Helvetica', 10))
        self.status.pack(pady=4)
        self.motion_status = tk.Label(
            self.root, text='Etat: ARRETE', fg='#e74c3c', bg='#2c3e50', font=('Helvetica', 11, 'bold'),
        )
        self.motion_status.pack(pady=2)
        self.update_speeds(None)

        btn_frame = tk.Frame(self.root, bg='#2c3e50')
        btn_frame.pack(pady=12)

        def bind_drive(btn, lin, ang, turn=False):
            btn.bind(
                '<ButtonPress-1>',
                lambda e, l=lin, a=ang, t=turn: self._gui_press(l, a, t),
            )
            btn.bind('<ButtonRelease-1>', lambda e: self._gui_release())

        fwd = {'font': ('Helvetica', 11, 'bold'), 'width': 11, 'height': 2,
               'bg': '#3498db', 'fg': 'white'}
        turn = {'font': ('Helvetica', 11, 'bold'), 'width': 11, 'height': 2,
                'bg': '#2ecc71', 'fg': 'white'}
        stop_s = {'font': ('Helvetica', 11, 'bold'), 'width': 11, 'height': 2,
                  'bg': '#e74c3c', 'fg': 'white'}

        b_fwd = tk.Button(btn_frame, text='Avancer', **fwd)
        b_fwd.grid(row=0, column=1, padx=4, pady=4)
        bind_drive(b_fwd, 1, 0)

        b_ag = tk.Button(btn_frame, text='Avancer +\nGauche', **turn)
        b_ag.grid(row=0, column=0, padx=4, pady=4)
        bind_drive(b_ag, 1, 1, True)

        b_ad = tk.Button(btn_frame, text='Avancer +\nDroite', **turn)
        b_ad.grid(row=0, column=2, padx=4, pady=4)
        bind_drive(b_ad, 1, -1, True)

        b_left = tk.Button(btn_frame, text='Tourner\nGAUCHE', **turn)
        b_left.grid(row=1, column=0, padx=4, pady=4)
        bind_drive(b_left, 0, 1)  # pivot gauche sur place

        b_stop = tk.Button(btn_frame, text='STOP', command=self._gui_stop, **stop_s)
        b_stop.grid(row=1, column=1, padx=4, pady=4)

        b_right = tk.Button(btn_frame, text='Tourner\nDROITE', **turn)
        b_right.grid(row=1, column=2, padx=4, pady=4)
        bind_drive(b_right, 0, -1)  # pivot droite sur place

        b_rev = tk.Button(btn_frame, text='Reculer', **fwd)
        b_rev.grid(row=2, column=1, padx=4, pady=4)
        bind_drive(b_rev, -1, 0)

        b_pivot_l = tk.Button(btn_frame, text='Pivot\nGauche', **fwd)
        b_pivot_l.grid(row=3, column=0, padx=4, pady=4)
        bind_drive(b_pivot_l, 0, 1)

        b_pivot_r = tk.Button(btn_frame, text='Pivot\nDroite', **fwd)
        b_pivot_r.grid(row=3, column=2, padx=4, pady=4)
        bind_drive(b_pivot_r, 0, -1)

        self.root.protocol('WM_DELETE_WINDOW', self.on_closing)
        self.root.mainloop()
    
    def _on_toggle_changed(self):
        self.toggle_mode = self.toggle_var.get()
        if not self.toggle_mode:
            self.stop_motion()
    
    def _gui_press(self, lin_dir: float, ang_dir: float, use_turn_speed: bool):
        self.on_drive_press(lin_dir, ang_dir, use_turn_speed)
        self._refresh_motion_label()
    
    def _gui_release(self):
        self.on_drive_release()
        self._refresh_motion_label()
    
    def _gui_stop(self):
        self.stop_motion()
        self._refresh_motion_label()
    
    def _refresh_motion_label(self):
        with self._cmd_lock:
            moving = abs(self.active_lin) > 1e-6 or abs(self.active_ang) > 1e-6
            lin, ang = self.active_lin, self.active_ang
        if moving:
            self.motion_status.config(
                text=f'Etat: EN MARCHE  v={lin:.2f}  omega={ang:.2f}',
                fg='#2ecc71',
            )
        else:
            self.motion_status.config(text='Etat: ARRETE', fg='#e74c3c')
    
    def update_speeds(self, _):
        self.linear_speed = self.lin_scale.get()
        self.angular_speed = self.ang_scale.get()
        self.turn_linear_speed = max(0.20, self.linear_speed * 0.75)
        # Compute omega_max based on kinematic limits
        beta_limit_rad = math.radians(self.beta_limit_deg)
        k_omega = math.sin(beta_limit_rad) / self.d
        om = k_omega * abs(self.linear_speed)
        self.status.config(
            text=f'v={self.linear_speed:.2f} m/s | omega={self.angular_speed:.2f} rad/s | '
                 f'omega_max={om:.3f} | pivot_max={self.max_stationary_omega:.2f} | '
                 f'beta={math.degrees(self.beta):.1f} deg'
        )

    def on_closing(self):
        self.stop_motion()
        self.root.destroy()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrailerControlGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
