#!/usr/bin/env python3
"""
Beta EKF Node - Estimation de l'angle d'attelage remorque
=========================================================
Ce nœud implémente un filtre de Kalman étendu (EKF) pour estimer l'angle β
entre le tracteur et la remorque, en fusionnant plusieurs sources de mesure :
- Mesure directe du joint hitch (β_joint)
- Estimation cinématique à partir des roues (β_wheels)
- Mesure GPS si disponible (β_GPS)

Équation cinématique de base :
    β̇ = ω - (v/d) * sin(β)

où :
    β : angle d'attelage (θ_tracteur - θ_remorque)
    ω : vitesse angulaire du tracteur
    v : vitesse linéaire du tracteur
    d : distance entre point d'attelage et essieu remorque
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Header
import numpy as np
import math


class BetaEKFNode(Node):
    """Filtre de Kalman étendu pour l'angle d'attelage remorque."""

    def __init__(self):
        super().__init__('beta_ekf_node')

        # ─── Paramètres géométriques ────────────────────────────────────────
        self.declare_parameter('hitch_distance', 0.50)  # d : distance attelage-essieu (m)
        self.declare_parameter('wheel_separation', 0.40)  # L : entraxe roues (m)
        self.declare_parameter('max_wheel_speed', 1.0)  # vmax : vitesse max roue (m/s)
        self.declare_parameter('beta_limit_deg', 45.0)  # βmax : limite mécanique (degrés)
        self.declare_parameter('process_noise', 0.01)  # Bruit de processus
        self.declare_parameter('joint_measurement_noise', 0.05)  # Bruit mesure joint
        self.declare_parameter('wheel_measurement_noise', 0.1)  # Bruit mesure cinématique

        self.d = self.get_parameter('hitch_distance').value
        self.L = self.get_parameter('wheel_separation').value
        self.vmax = self.get_parameter('max_wheel_speed').value
        self.beta_max = math.radians(self.get_parameter('beta_limit_deg').value)

        # ─── État EKF ───────────────────────────────────────────────────────
        # État : [β, β̇]
        self.x = np.array([0.0, 0.0])  # État initial
        self.P = np.eye(2) * 0.1  # Covariance initiale
        self.Q = np.eye(2) * self.get_parameter('process_noise').value  # Bruit processus

        # ─── Variables pour calculs ───────────────────────────────────────────
        self.current_v = 0.0
        self.current_omega = 0.0
        self.beta_joint = 0.0
        self.last_time = self.get_clock().now()

        # ─── QoS Profile pour odom (reliable, volatile) ────────────────
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ─── Subscribers ─────────────────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # ─── Publishers ───────────────────────────────────────────────────────
        self.beta_pub = self.create_publisher(Float64, '/trailer/beta', 10)
        self.beta_dot_pub = self.create_publisher(Float64, '/trailer/beta_dot', 10)
        self.r_min_pub = self.create_publisher(Float64, '/trailer/r_min', 10)
        self.omega_max_pub = self.create_publisher(Float64, '/trailer/omega_max', 10)
        self.icr_pub = self.create_publisher(TwistStamped, '/trailer/icr', 10)

        # ─── Timer pour publication (50 Hz) ───────────────────────────────────
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('Beta EKF Node initialized')
        self.get_logger().info(f'  hitch_distance d = {self.d:.3f} m')
        self.get_logger().info(f'  wheel_separation L = {self.L:.3f} m')
        self.get_logger().info(f'  beta_limit = {math.degrees(self.beta_max):.1f} deg')

    def joint_state_callback(self, msg):
        """Récupère l'angle du joint hitch."""
        if 'hitch_joint' in msg.name:
            idx = msg.name.index('hitch_joint')
            self.beta_joint = msg.position[idx]

    def odom_callback(self, msg):
        """Récupère la vitesse et la vitesse angulaire du tracteur."""
        self.current_v = msg.twist.twist.linear.x
        self.current_omega = msg.twist.twist.angular.z

    def predict_step(self, dt):
        """
        Étape de prédiction EKF : β̇ = ω - (v/d) * sin(β)
        
        Modèle de transition :
            x = [β, β̇]
            ẋ = [β̇, - (v/d) * cos(β) * β̇ - (a/d) * sin(β)]
        """
        beta, beta_dot = self.x
        v = self.current_v
        omega = self.current_omega

        # Équation cinématique principale
        beta_dot_pred = omega - (v / self.d) * math.sin(beta)

        # Dérivée de β̇ (pour linéarisation)
        # β̈ = - (v/d) * cos(β) * β̇ - (a/d) * sin(β)
        # On néglige l'accélération a pour simplifier
        beta_ddot = - (v / self.d) * math.cos(beta) * beta_dot

        # Mise à jour de l'état prédit (Euler)
        self.x[0] += beta_dot_pred * dt
        self.x[1] += beta_ddot * dt

        # Jacobien de la fonction de transition F = ∂f/∂x
        F = np.array([
            [1.0, dt],
            [-(v / self.d) * math.cos(beta) * dt, 1.0]
        ])

        # Mise à jour de la covariance
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_step_joint(self):
        """
        Étape de mise à jour avec mesure du joint hitch.
        z = β_joint
        """
        beta_joint_noise = self.get_parameter('joint_measurement_noise').value
        R = np.array([[beta_joint_noise**2]])

        # Jacobien de la mesure H = ∂h/∂x
        H = np.array([[1.0, 0.0]])

        # Innovation
        y = np.array([self.beta_joint - self.x[0]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Mise à jour de l'état et covariance
        self.x += K @ y
        self.P = (np.eye(2) - K @ H) @ self.P

    def update_step_wheels(self):
        """
        Étape de mise à jour avec estimation cinématique des roues.
        β_wheels estimé à partir de la différence de vitesse des roues arrière.
        """
        # Pour l'instant, on utilise une estimation simplifiée
        # β_wheels ≈ arctan(ω * d / v) en régime établi
        if abs(self.current_v) > 0.1:
            beta_wheels = math.atan2(self.current_omega * self.d, self.current_v)
        else:
            beta_wheels = self.x[0]

        beta_wheel_noise = self.get_parameter('wheel_measurement_noise').value
        R = np.array([[beta_wheel_noise**2]])

        H = np.array([[1.0, 0.0]])
        y = np.array([beta_wheels - self.x[0]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(2) - K @ H) @ self.P

    def compute_r_min(self):
        """
        Calcule le rayon minimal admissible : R_min = d / sin(β_max)
        """
        if abs(math.sin(self.beta_max)) > 0.01:
            r_min = self.d / math.sin(self.beta_max)
        else:
            r_min = float('inf')
        return r_min

    def compute_omega_max(self):
        """
        Calcule la vitesse angulaire maximale : ω_max(v) = 2(v_max - |v|) / L
        """
        v = abs(self.current_v)
        if v < self.vmax:
            omega_max = 2 * (self.vmax - v) / self.L
        else:
            omega_max = 0.0
        return omega_max

    def compute_icr(self):
        """
        Calcule la position de l'ICR (Instantaneous Center of Rotation).
        
        Pour le tracteur :
            R = v / ω
            ICR = (0, R) dans le repère base_link
        """
        v = self.current_v
        omega = self.current_omega

        if abs(omega) > 0.01:
            R = v / omega
            icr_x = 0.0
            icr_y = R
        else:
            icr_x = 0.0
            icr_y = float('inf')

        return icr_x, icr_y

    def timer_callback(self):
        """Callback principal : prédiction EKF et publication."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0.0 and dt < 1.0:  # Éviter dt invalide
            # Étape de prédiction
            self.predict_step(dt)

            # Étapes de mise à jour
            self.update_step_joint()
            self.update_step_wheels()

            # Clamp β dans les limites physiques
            self.x[0] = np.clip(self.x[0], -self.beta_max, self.beta_max)

        # ─── Publication des résultats ───────────────────────────────────────
        beta_msg = Float64()
        beta_msg.data = self.x[0]
        self.beta_pub.publish(beta_msg)

        beta_dot_msg = Float64()
        beta_dot_msg.data = self.x[1]
        self.beta_dot_pub.publish(beta_dot_msg)

        r_min_msg = Float64()
        r_min_msg.data = self.compute_r_min()
        self.r_min_pub.publish(r_min_msg)

        omega_max_msg = Float64()
        omega_max_msg.data = self.compute_omega_max()
        self.omega_max_pub.publish(omega_max_msg)

        icr_x, icr_y = self.compute_icr()
        icr_msg = TwistStamped()
        icr_msg.header.stamp = self.get_clock().now().to_msg()
        icr_msg.header.frame_id = 'base_link'
        icr_msg.twist.linear.x = icr_x
        icr_msg.twist.linear.y = icr_y
        self.icr_pub.publish(icr_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BetaEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
