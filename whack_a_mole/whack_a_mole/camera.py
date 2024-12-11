import pyrealsense2 as rs
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import Empty

from sensor_msgs.msg import Image, CameraInfo

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from whack_a_mole.mole_detection import OpenCVClient
from whack_a_mole.constants import COLORS, COLORS_HSV


class Camera(Node):

    def __init__(self):

        super().__init__("color_camera")

        self.declare_parameter("clipping_distance", 1400)
        self.declare_parameter("box_start_x", 1)
        self.declare_parameter("box_end_x", 1)
        self.declare_parameter("box_start_y", 1)
        self.declare_parameter("box_end_y", 1)

        self.crop = [
            [self.get_parameter("box_start_x").value,
            self.get_parameter("box_start_y").value],
            [self.get_parameter("box_end_x").value,
            self.get_parameter("box_end_y").value]
        ]

        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.get_depth_info,
            qos_profile=QoSProfile(depth=10),
        )

        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.get_color_info,
            qos_profile=QoSProfile(depth=10),
        )

        self.image_pub = self.create_publisher(
            Image, "filtered_image", qos_profile=QoSProfile(depth=10)
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10,
        )
        
        self.create_service(Empty, 'toggle_tf_publish', self.toggle_tf_publish)
        self.clipping_distance = self.get_parameter("clipping_distance").value
        self.cv2_client = OpenCVClient(
            clipping_distance=self.clipping_distance,
            crop=self.crop
        )
        self.tf_broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))

        self.prev_xyz = np.array([0, 0, 0], dtype=float)

        self.freq = 30.0  # Hz

        self.create_timer(1 / self.freq, self.timer_callback)

        self.running_avg = (
            np.zeros((len(COLORS.keys()), int(self.freq * 10), 2))
            + (np.array(self.crop[0]) + np.array(self.crop[1])) / 2
        )

        self.update_colors = True

    def timer_callback(self):
        """Timer callback to detect the color of the object and broadcast the frame."""
        if self.cv2_client.color_image.shape[0] == 0 or self.cv2_client.depth_image.shape[0] == 0:
            return

        for color in COLORS_HSV.keys():

            self.broadcast_color(
                lower_HSV=COLORS_HSV[color][0],
                higher_HSV=COLORS_HSV[color][1],
                color=color,
            )

            self.detect_illumination(color)

    def toggle_tf_publish(self, request, response):
        self.update_colors = not self.update_colors
        return response

    def detect_illumination(self, color: str):
        """
        Detects the illumination of the color and broadcasts.
        
        Args:
            color (str): The color of the object
        
        """
        color_index = COLORS[color]
        median_centroid = np.median(self.running_avg[color_index], axis=0)
        median_centroid = np.array(median_centroid, dtype=int)

    def broadcast_color(self, lower_HSV, higher_HSV, color: str):
        """
        Broadcasts the color frame.
        
        Args:
            lower_HSV (np.array): The lower HSV range
            higher_HSV (np.array): The higher HSV range
            color (str): The color of the object
        """
        color_index = COLORS[color]

        x_c, y_c, bg_removed = self.find_color_centroid(lower_HSV, higher_HSV)
        msg = CvBridge().cv2_to_imgmsg(bg_removed, encoding="bgr8")
        self.image_pub.publish(msg)

        if x_c != 0 and y_c != 0 and self.update_colors:

            self.running_avg[color_index, :-1] = self.running_avg[
                color_index, 1:
            ]  # Shift all elements down by 1
            self.running_avg[color_index, -1] = np.array([x_c, y_c])

        median_centroid = np.median(self.running_avg[color_index], axis=0)
        median_centroid = np.array(median_centroid, dtype=int)

        self.broadcast_color_frame(
            "camera_depth_frame", color + "_frame", median_centroid[0], median_centroid[1]
        )

    def log(self, *message):
        """Custom logger function."""
        self.get_logger().info(f"[CAMERA NODE]: {", ".join(str(i) for i in message)}")

    def camera_info_callback(self, msg):
        """
        Callback for camera info subscriber

        Args:
            msg (CameraInfo): Camera Info message
        """
        self.cv2_client.camera_intrinsics = msg.k

    def get_depth_info(self, msg: Image):
        """
        Callback for depth image subscriber
        
        Args:
            msg (Image): Depth image message
        """
        self.cv2_client.depth_image = CvBridge().imgmsg_to_cv2(msg)

    def get_color_info(self, msg: Image):
        """
        Callback for color image subscriber
        
        Args:
            msg (Image): Color image message
        """
        self.cv2_client.color_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def broadcast_color_frame(self, base_frame, child_frame, x_c, y_c):

        x, y, z = self.get_3d_coordinates_at_pixel(x_c, y_c, frame_name=child_frame)
        if x == -1:
            x, y, z = self.prev_xyz[0], self.prev_xyz[1], self.prev_xyz[2]
        else:
            self.prev_xyz[0], self.prev_xyz[1], self.prev_xyz[2] = x, y, z

        # self.log(child_frame,x,y,z)

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = z
        transform.transform.translation.y = -x
        transform.transform.translation.z = -y

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)


def entry(args=None):

    rclpy.init(args=args)

    camera_node = Camera()

    rclpy.spin(camera_node)
