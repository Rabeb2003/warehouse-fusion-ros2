#!/usr/bin/env python3
"""
Multi-Circle Dynamic Footprint Node for Robot + Trailer
========================================================
Implements modern multi-circle coverage for articulated vehicle footprint.
Based on 2024 GPU study showing 36% trajectory length reduction and 65% acceleration reduction.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import math


class MultiCircleFootprintNode(Node):
    def __init__(self):
        super().__init__('multi_circle_footprint_node')
        
        # Paramètres géométriques
        self.declare_parameter('hitch_distance', 0.50)  # d : distance attelage-essieu (m)
        self.declare_parameter('robot_length', 0.6)    # Lr : longueur tracteur (m)
        self.declare_parameter('robot_width', 0.4)     # Wr : largeur tracteur (m)
        self.declare_parameter('trailer_length', 0.8)  # Lt : longueur remorque (m)
        self.declare_parameter('trailer_width', 0.5)   # Wt : largeur remorque (m)
        self.declare_parameter('tractor_center_x', -0.025)  # Centre collision tracteur / base_link
        self.declare_parameter('tractor_center_y', 0.041)
        self.declare_parameter('hitch_offset_x', -0.3575)   # Position attelage / base_link
        self.declare_parameter('hitch_offset_y', 0.0414)
        self.declare_parameter('num_circles_tractor', 3)  # N1 : nombre cercles tracteur
        self.declare_parameter('num_circles_trailer', 3)  # N2 : nombre cercles remorque
        self.declare_parameter('footprint_margin', 0.03)  # Marge incluse dans les cercles
        self.declare_parameter('cost_threshold', 100)  # Seuil de coût pour rejet
        
        self.d = self.get_parameter('hitch_distance').value
        self.Lr = self.get_parameter('robot_length').value
        self.Wr = self.get_parameter('robot_width').value
        self.Lt = self.get_parameter('trailer_length').value
        self.Wt = self.get_parameter('trailer_width').value
        self.tractor_center_x = self.get_parameter('tractor_center_x').value
        self.tractor_center_y = self.get_parameter('tractor_center_y').value
        self.hitch_x = self.get_parameter('hitch_offset_x').value
        self.hitch_y = self.get_parameter('hitch_offset_y').value
        self.N1 = self.get_parameter('num_circles_tractor').value
        self.N2 = self.get_parameter('num_circles_trailer').value
        self.footprint_margin = self.get_parameter('footprint_margin').value
        self.cost_threshold = self.get_parameter('cost_threshold').value
        
        # État
        self.beta = 0.0
        self.local_costmap = None
        
        # Buffer TF et Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Couvre la demi-largeur du convoi plus une marge de sécurité.
        self.circle_radius = max(self.Wr, self.Wt) / 2.0 + self.footprint_margin
        
        # Subscribers
        self.beta_sub = self.create_subscription(
            Float64, '/trailer/beta', self.beta_callback, 10
        )
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10
        )
        
        # Publishers
        self.local_costmap_footprint_pub = self.create_publisher(
            Polygon, '/local_costmap/footprint', 10
        )
        self.global_costmap_footprint_pub = self.create_publisher(
            Polygon, '/global_costmap/footprint', 10
        )
        self.visual_footprint_pub = self.create_publisher(
            PolygonStamped, '/trailer/footprint', 10
        )
        self.visual_footprint_local_pub = self.create_publisher(
            PolygonStamped, '/trailer/footprint_local', 10
        )
        self.circle_centers_pub = self.create_publisher(
            PolygonStamped, '/trailer/circle_centers', 10  # Pour visualisation
        )
        
        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update_footprint)
        
        self.get_logger().info('Multi-Circle Footprint Node initialized')
        self.get_logger().info(f'  hitch_distance = {self.d:.3f} m')
        self.get_logger().info(f'  robot_length = {self.Lr:.3f} m')
        self.get_logger().info(f'  robot_width = {self.Wr:.3f} m')
        self.get_logger().info(f'  trailer_length = {self.Lt:.3f} m')
        self.get_logger().info(f'  trailer_width = {self.Wt:.3f} m')
        self.get_logger().info(f'  hitch_offset = ({self.hitch_x:.3f}, {self.hitch_y:.3f}) m')
        self.get_logger().info(f'  num_circles_tractor = {self.N1}')
        self.get_logger().info(f'  num_circles_trailer = {self.N2}')
        self.get_logger().info(f'  circle_radius = {self.circle_radius:.3f} m')
        self.get_logger().info('  publishing Nav2 footprint to /local_costmap/footprint and /global_costmap/footprint')
        
    def beta_callback(self, msg):
        self.beta = msg.data
        
    def costmap_callback(self, msg):
        self.local_costmap = msg
        
    def compute_circle_centers(self):
        """
        Calcule les centres des cercles pour tracteur et remorque.
        """
        centers = []
        
        # Cercles du tracteur, bornés à l'intérieur de sa boîte collision URDF.
        for x_local in self.linspace_inside_box(self.Lr, self.N1):
            x = self.tractor_center_x + x_local
            y = self.tractor_center_y
            centers.append((x, y, 'tractor'))
        
        # Cercles de la remorque : +X du trailer_base_link pointe vers l'arrière
        # du convoi à beta=0 à cause du yaw pi dans le joint d'attelage URDF.
        cos_beta = math.cos(self.beta)
        sin_beta = math.sin(self.beta)
        rear_dir_x = -cos_beta
        rear_dir_y = -sin_beta

        for distance_from_hitch in self.linspace_inside_trailer(self.Lt, self.N2):
            x = self.hitch_x + distance_from_hitch * rear_dir_x
            y = self.hitch_y + distance_from_hitch * rear_dir_y
            centers.append((x, y, 'trailer'))
        
        return centers

    def linspace_inside_box(self, length, count):
        """Positions longitudinales de centres de cercles dans une boîte."""
        if count <= 1:
            return [0.0]

        half_span = max(0.0, length / 2.0 - self.circle_radius)
        if half_span == 0.0:
            return [0.0 for _ in range(count)]

        return [
            -half_span + (2.0 * half_span * i / (count - 1))
            for i in range(count)
        ]

    def linspace_inside_trailer(self, length, count):
        """Distances positives depuis l'attelage vers l'arrière de la remorque."""
        if count <= 1:
            return [length / 2.0]

        half_span = max(0.0, length / 2.0 - self.circle_radius)
        return [
            length / 2.0 - half_span + (2.0 * half_span * i / (count - 1))
            for i in range(count)
        ]
        
    def evaluate_costmap(self, centers):
        """
        Évalue le coût à chaque centre de cercle dans la costmap locale.
        """
        if self.local_costmap is None:
            return 0.0
        
        max_cost = 0.0
        
        try:
            # Récupérer la transformation de base_link vers le repère de la costmap
            transform = self.tf_buffer.lookup_transform(
                self.local_costmap.header.frame_id,
                'base_link',
                rclpy.time.Time()
            )
            
            # Extraire la translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            
            # Extraire la rotation (quaternion vers yaw en 2D)
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Résolution et origine de la costmap
            res = self.local_costmap.info.resolution
            origin_x = self.local_costmap.info.origin.position.x
            origin_y = self.local_costmap.info.origin.position.y
            width = self.local_costmap.info.width
            height = self.local_costmap.info.height
            data = self.local_costmap.data
            
            for cx, cy, _ in centers:
                # Transformer le point du repère base_link vers le repère de la costmap
                ox = tx + cx * math.cos(yaw) - cy * math.sin(yaw)
                oy = ty + cx * math.sin(yaw) + cy * math.cos(yaw)
                
                # Convertir en coordonnées de grille
                mx = int((ox - origin_x) / res)
                my = int((oy - origin_y) / res)
                
                # Vérifier les limites de la grille
                if 0 <= mx < width and 0 <= my < height:
                    cost = data[my * width + mx]
                    if cost > max_cost:
                        max_cost = float(cost)
                        
            # Si le coût dépasse le seuil, publier un avertissement
            if max_cost >= self.cost_threshold:
                self.get_logger().warn(
                    f"Risque de collision détecté ! Coût max : {max_cost:.1f} (Seuil : {self.cost_threshold})",
                    throttle_duration_sec=2.0
                )
                
        except Exception as e:
            # Ne pas polluer les logs si TF n'est pas encore disponible au démarrage
            self.get_logger().debug(f"Impossible de transformer le footprint : {e}")
            
        return max_cost
        
    def compute_circle_envelope(self, centers):
        """
        Calcule un polygone approximant l'union des cercles.
        Pour compatibilité Nav2, on utilise l'enveloppe convexe des centres
        étendue par le rayon.
        """
        # Créer des points sur le périmètre de chaque cercle
        envelope_points = []
        num_points_per_circle = 8
        
        for cx, cy, _ in centers:
            for i in range(num_points_per_circle):
                angle = 2.0 * math.pi * i / num_points_per_circle
                px = cx + self.circle_radius * math.cos(angle)
                py = cy + self.circle_radius * math.sin(angle)
                envelope_points.append((px, py))
        
        # Calculer l'enveloppe convexe
        hull = self.compute_convex_hull(envelope_points)
        return hull
        
    def compute_convex_hull(self, points):
        """
        Calcule l'enveloppe convexe (chaîne monotone).
        """
        unique_points = sorted(set(points))
        if len(unique_points) <= 1:
            return unique_points

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower = []
        for p in unique_points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        upper = []
        for p in reversed(unique_points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        return lower[:-1] + upper[:-1]
        
    def update_footprint(self):
        """Calcule et publie le footprint multi-cercles."""
        # Calculer les centres des cercles
        centers = self.compute_circle_centers()
        
        # Évaluer le coût (optionnel - pour vérification de sécurité)
        max_cost = self.evaluate_costmap(centers)
        
        # Calculer l'enveloppe des cercles pour compatibilité Nav2
        hull_points = self.compute_circle_envelope(centers)
        
        # Créer le message PolygonStamped pour l'enveloppe
        footprint = PolygonStamped()
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.header.frame_id = 'base_link'
        
        for px, py in hull_points:
            footprint.polygon.points.append(
                Point32(x=float(px), y=float(py), z=0.0)
            )
        
        # Les costmaps Nav2 écoutent geometry_msgs/Polygon sur leur topic
        # "footprint". Les topics "published_footprint" sont des sorties Nav2.
        nav2_footprint = Polygon()
        nav2_footprint.points = footprint.polygon.points
        self.local_costmap_footprint_pub.publish(nav2_footprint)
        self.global_costmap_footprint_pub.publish(nav2_footprint)

        # Topics stamped conservés pour RViz et diagnostic.
        self.visual_footprint_pub.publish(footprint)
        self.visual_footprint_local_pub.publish(footprint)
        
        # Publier les centres des cercles pour visualisation
        centers_msg = PolygonStamped()
        centers_msg.header.stamp = self.get_clock().now().to_msg()
        centers_msg.header.frame_id = 'base_link'
        
        for cx, cy, _ in centers:
            centers_msg.polygon.points.append(
                Point32(x=float(cx), y=float(cy), z=0.0)
            )
        
        self.circle_centers_pub.publish(centers_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCircleFootprintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
