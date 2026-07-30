#!/usr/bin/env python3
"""Detect the red target block from the Ignition camera stream."""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401 - registers geometry_msgs conversions in tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RedObjectDetector(Node):
    def __init__(self):
        super().__init__("red_object_detector")

        self.declare_parameter("image_topic", "/wrist_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/wrist_camera/camera_info")
        self.declare_parameter("camera_frame", "panda_wrist_camera_optical_frame")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("target_center_z", 0.425)
        self.declare_parameter("horizontal_fov", 1.25)
        self.declare_parameter("min_area", 35.0)
        self.declare_parameter("min_saturation", 60)
        self.declare_parameter("min_value", 45)
        self.declare_parameter("rgb_image_topic", "/wrist_camera/image_rgb")
        self.declare_parameter("detected_image_topic", "/wrist_camera/detected_object_image")
        self.declare_parameter("mask_topic", "/wrist_camera/red_mask")
        self.declare_parameter("world_x_min", 0.20)
        self.declare_parameter("world_x_max", 0.80)
        self.declare_parameter("world_y_min", -0.40)
        self.declare_parameter("world_y_max", 0.40)
        self.declare_parameter("world_z_min", 0.35)
        self.declare_parameter("world_z_max", 0.55)
        self.declare_parameter("max_pose_jump", 0.08)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.target_center_z = float(self.get_parameter("target_center_z").value)
        self.horizontal_fov = float(self.get_parameter("horizontal_fov").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.min_saturation = int(self.get_parameter("min_saturation").value)
        self.min_value = int(self.get_parameter("min_value").value)
        self.rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self.detected_image_topic = self.get_parameter("detected_image_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.world_x_min = float(self.get_parameter("world_x_min").value)
        self.world_x_max = float(self.get_parameter("world_x_max").value)
        self.world_y_min = float(self.get_parameter("world_y_min").value)
        self.world_y_max = float(self.get_parameter("world_y_max").value)
        self.world_z_min = float(self.get_parameter("world_z_min").value)
        self.world_z_max = float(self.get_parameter("world_z_max").value)
        self.max_pose_jump = float(self.get_parameter("max_pose_jump").value)

        self.frame_count = 0
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_valid_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
        self.rgb_pub = self.create_publisher(
            Image, self.rgb_image_topic, 10
        )
        self.viz_pub = self.create_publisher(
            Image, self.detected_image_topic, 10
        )
        self.mask_pub = self.create_publisher(
            Image, self.mask_topic, 10
        )

        self.get_logger().info(
            f"Red detector active: image={self.image_topic}, info={self.camera_info_topic}, "
            f"camera_frame={self.camera_frame}, world_frame={self.world_frame}"
        )
        self.get_logger().info(
            f"RViz image topics: {self.rgb_image_topic} or {self.detected_image_topic}. "
            f"Ignition bridge input stays on {self.image_topic}."
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def decode_image(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "r8g8b8": 3,
            "bgr8": 3,
            "b8g8r8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported encoding '{msg.encoding}'")

        row_step = msg.step if msg.step else msg.width * channels
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape((msg.height, row_step))
        pixels = rows[:, : msg.width * channels].reshape(
            (msg.height, msg.width, channels)
        )

        if encoding in ("rgb8", "r8g8b8"):
            rgb = pixels.copy()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif encoding in ("bgr8", "b8g8r8"):
            bgr = pixels.copy()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        elif encoding == "rgba8":
            rgb = cv2.cvtColor(pixels, cv2.COLOR_RGBA2RGB)
            bgr = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            bgr = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
            rgb = cv2.cvtColor(pixels, cv2.COLOR_BGRA2RGB)
        else:
            gray = pixels.reshape((msg.height, msg.width))
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        return bgr, rgb

    def get_camera_matrix(self, msg: Image):
        if self.camera_matrix is not None:
            return self.camera_matrix

        fx = msg.width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        fy = fx
        cx = msg.width / 2.0
        cy = msg.height / 2.0
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    def image_callback(self, msg: Image):
        self.frame_count += 1
        try:
            bgr_image, rgb_image = self.decode_image(msg)
        except Exception as exc:
            self.get_logger().error(f"Image decode error: {exc}")
            return

        self.publish_image(self.rgb_pub, rgb_image, "rgb8", msg.header)

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, self.min_saturation, self.min_value])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([165, self.min_saturation, self.min_value])
        upper_red2 = np.array([180, 255, 255])

        hsv_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2),
        )
        red = rgb_image[:, :, 0].astype(np.int16)
        green = rgb_image[:, :, 1].astype(np.int16)
        blue = rgb_image[:, :, 2].astype(np.int16)
        dominance_mask = (
            (red > 70)
            & (red > green + 35)
            & (red > blue + 35)
            & (red > 1.35 * np.maximum(green, blue))
        ).astype(np.uint8) * 255

        mask = cv2.bitwise_or(hsv_mask, dominance_mask)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        self.publish_image(self.mask_pub, mask, "mono8", msg.header)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = rgb_image.copy()
        red_pixels = int(cv2.countNonZero(mask))

        if not contours:
            self.log_no_detection(msg, red_pixels, 0.0, hsv)
            self.publish_image(self.viz_pub, annotated, "rgb8", msg.header)
            return

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area < self.min_area:
            self.log_no_detection(msg, red_pixels, area, hsv)
            self.publish_image(self.viz_pub, annotated, "rgb8", msg.header)
            return

        x, y, w, h = cv2.boundingRect(largest_contour)
        cx = x + w // 2
        cy = y + h // 2
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(
            annotated,
            "RED BLOCK",
            (x, max(15, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        pose_world = self.pixel_to_world_pose(cx, cy, msg)
        if pose_world is not None:
            if self.is_pose_reasonable(pose_world):
                self.pose_pub.publish(pose_world)
                self.last_valid_pose = pose_world.pose
                self.get_logger().info(
                    f"VISION DETECTED: pixel=({cx},{cy}) area={area:.0f} "
                    f"world=({pose_world.pose.position.x:.3f}, "
                    f"{pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f})",
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().warn(
                    f"Rejected unstable red projection: pixel=({cx},{cy}) area={area:.0f} "
                    f"world=({pose_world.pose.position.x:.3f}, "
                    f"{pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f})",
                    throttle_duration_sec=2.0,
                )
        else:
            self.get_logger().warn(
                f"Red pixels detected but 3D projection failed: pixel=({cx},{cy}) area={area:.0f}",
                throttle_duration_sec=2.0,
            )

        self.publish_image(self.viz_pub, annotated, "rgb8", msg.header)

    def pixel_to_world_pose(self, pixel_x: int, pixel_y: int, msg: Image):
        camera_matrix = self.get_camera_matrix(msg)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx0 = camera_matrix[0, 2]
        cy0 = camera_matrix[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            return None

        ray_x = (pixel_x - cx0) / fx
        ray_y = (pixel_y - cy0) / fy

        origin_cam = PoseStamped()
        origin_cam.header.stamp = Time().to_msg()
        origin_cam.header.frame_id = self.camera_frame
        origin_cam.pose.orientation.w = 1.0

        ray_cam = PoseStamped()
        ray_cam.header = origin_cam.header
        ray_cam.pose.position.x = float(ray_x)
        ray_cam.pose.position.y = float(ray_y)
        ray_cam.pose.position.z = 1.0
        ray_cam.pose.orientation.w = 1.0

        try:
            origin_world = self.tf_buffer.transform(
                origin_cam, self.world_frame, timeout=Duration(seconds=0.5)
            )
            ray_world = self.tf_buffer.transform(
                ray_cam, self.world_frame, timeout=Duration(seconds=0.5)
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF projection failed: {exc}", throttle_duration_sec=2.0)
            return None

        ox = origin_world.pose.position.x
        oy = origin_world.pose.position.y
        oz = origin_world.pose.position.z
        dx = ray_world.pose.position.x - ox
        dy = ray_world.pose.position.y - oy
        dz = ray_world.pose.position.z - oz
        if abs(dz) < 1.0e-6:
            return None

        t = (self.target_center_z - oz) / dz
        if t <= 0.0:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = ox + t * dx
        pose.pose.position.y = oy + t * dy
        pose.pose.position.z = self.target_center_z
        pose.pose.orientation.w = 1.0
        return pose

    def is_pose_reasonable(self, pose_stamped: PoseStamped):
        pose = pose_stamped.pose
        if not (
            self.world_x_min <= pose.position.x <= self.world_x_max
            and self.world_y_min <= pose.position.y <= self.world_y_max
            and self.world_z_min <= pose.position.z <= self.world_z_max
        ):
            return False

        if self.last_valid_pose is None:
            return True

        dx = pose.position.x - self.last_valid_pose.position.x
        dy = pose.position.y - self.last_valid_pose.position.y
        dz = pose.position.z - self.last_valid_pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.max_pose_jump

    def publish_image(self, publisher, image: np.ndarray, encoding: str, header):
        contiguous = np.ascontiguousarray(image)
        msg = Image()
        msg.header = header
        msg.height = int(contiguous.shape[0])
        msg.width = int(contiguous.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        channels = 1 if contiguous.ndim == 2 else int(contiguous.shape[2])
        msg.step = msg.width * channels
        msg.data = contiguous.tobytes()
        publisher.publish(msg)

    def log_no_detection(self, msg: Image, red_pixels: int, largest_area: float, hsv):
        mean_hsv = cv2.mean(hsv)[:3]
        self.get_logger().warn(
            f"Aucun rouge detecte: encoding={msg.encoding}, red_pixels={red_pixels}, "
            f"largest_area={largest_area:.1f}, mean_hsv={tuple(round(v, 2) for v in mean_hsv)}",
            throttle_duration_sec=2.0,
        )
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Camera feed active: {self.frame_count} frames from {self.image_topic}"
            )


def main(args=None):
    rclpy.init(args=args)
    detector = RedObjectDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
