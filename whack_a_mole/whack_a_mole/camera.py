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

global COLORS, COLORS_HSV, CROP

COLORS = {"GREEN": 0, "YELLOW": 1, "BLUE": 2, "RED": 3}

COLORS_HSV = {
    "GREEN": [np.array([47, 90, 132]), np.array([94, 255, 191])],
    "YELLOW": [np.array([24, 100, 101]), np.array([52, 255, 230])],
    "BLUE": [np.array((95, 157, 85)), np.array((161, 255, 189))],
    "RED": [np.array((0, 132, 4)), np.array((9, 241, 221))],
}

CROP = [(300, 400), (700, 720)]


class Camera(Node):

    def __init__(self):

        super().__init__("color_camera")

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

        self.tf_broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))

        self.color_image = np.array([])
        self.depth_image = np.array([])
        self.camera_intrinsics = np.array([])

        self.prev_xyz = np.array([0, 0, 0], dtype=float)

        self.freq = 30.0  # Hz

        self.create_timer(1 / self.freq, self.timer_callback)

        self.running_avg = (
            np.zeros((len(COLORS.keys()), int(self.freq * 10), 2))
            + (np.array(CROP[0]) + np.array(CROP[1])) / 2
        )

        self.clipping_distance = 1180
        
        self.update_colors = True

    def timer_callback(self):

        if self.color_image.shape[0] == 0:
            self.log("Image shape color")
            self.log("Waiting for image data...")
            return
        
        if self.depth_image.shape[0] == 0:
            self.log("Image shape depth")
            self.log("Waiting for depth data...")
            return

        for color in COLORS_HSV.keys():

            self.broadcast_color(
                lower_HSV=COLORS_HSV[color][0],
                higher_HSV=COLORS_HSV[color][1],
                color=color,
            )

    def toggle_tf_publish(self, request, response):
        
        self.update_colors = not self.update_colors
        return response

    def broadcast_color(self, lower_HSV, higher_HSV, color: str):

        color_index = COLORS[color]

        # if (color == "RED"):

        #     self.log(COLORS_HSV["RED"][0].shape)

        x_c, y_c = self.find_color_centroid(lower_HSV, higher_HSV)

        if x_c != 0 and y_c != 0 and self.update_colors:

            self.running_avg[color_index, :-1] = self.running_avg[
                color_index, 1:
            ]  # Shift all elements down by 1
            self.running_avg[color_index, -1] = np.array([x_c, y_c])

        avg_centroid = np.median(self.running_avg[color_index], axis=0)
        avg_centroid = np.array(avg_centroid, dtype=int)

        self.broadcast_color_frame(
            "camera_depth_frame", color + "_frame", avg_centroid[0], avg_centroid[1]
        )

    def log(self, *message):

        self.get_logger().info(f"[CAMERA NODE]: {", ".join(str(i) for i in message)}")

    def camera_info_callback(self, msg):
        # Save the camera intrinsics
        self.camera_intrinsics = msg.k

    def get_depth_info(self, msg: Image):

        self.depth_image = CvBridge().imgmsg_to_cv2(msg)

    def get_color_info(self, msg: Image):

        self.color_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def find_color_centroid(self, lower_HSV: np.array, higher_HSV: np.array):
        """
        Returns the pixel indecies of the centroid of a color defined in lower_HSV , higher_HSV range
        """

        depth_image_3d = np.dstack(
            (self.depth_image, self.depth_image, self.depth_image)
        )  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where(
            (depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0),
            0,
            self.color_image,
        )

        if lower_HSV.shape[0] == 2:
            mask1 = cv2.inRange(
                cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV), lower_HSV[0], higher_HSV[0]
            )
            mask2 = cv2.inRange(
                cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV), lower_HSV[1], higher_HSV[1]
            )
            mask = mask1 + mask2

        else:

            mask = cv2.inRange(
                cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV), lower_HSV, higher_HSV
            )

        if not np.any(mask):
            return np.array([-1, -1])

        # Find Centroid
        x_c, y_c = self.find_centroid_cropped(mask, (*CROP[0], *CROP[1]))

        x_c = int(x_c)
        y_c = int(y_c)

        for color in COLORS:
            color_index = COLORS[color]

            avg_centroid = np.median(self.running_avg[color_index], axis=0)
            avg_centroid = np.array(avg_centroid, dtype=int)

            cv2.circle(
                bg_removed,
                (avg_centroid[0], avg_centroid[1]),
                5,
                (0, 0, 255),
                thickness=10,
            )

            cv2.putText(
                bg_removed,
                f"{color}",
                (avg_centroid[0], avg_centroid[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

        cv2.rectangle(bg_removed, CROP[0], CROP[1], (0, 0, 255), thickness=5)

        msg = CvBridge().cv2_to_imgmsg(bg_removed, encoding="bgr8")

        self.image_pub.publish(msg)

        return x_c, y_c

    def find_centroid(self, masked_image):

        # Get all countours
        # https://docs.opencv.org/4.6.0/d4/d73/tutorial_py_contours_begin.html

        image = np.array(masked_image, dtype=np.uint8)
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Find the centroid
        # https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/

        # Find the largest area in the stream

        largest_area = -1
        largest_index = -1

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])

            # Check if the current contour has the largest area so far
            if area > largest_area:
                largest_area = area
                largest_index = i

        # If no valid contour was found (largest_area remains 0)
        if largest_area == 0:
            return 0, 0

        # Calculate the centroid

        try:
            moments = cv2.moments(contours[largest_index])
        except:
            print(len(contours))

        x_c = (moments["m10"]) / (largest_area)
        y_c = (moments["m01"]) / (largest_area)

        return x_c, y_c

    def find_centroid_cropped(self, masked_image, crop_indices):
        """
        Find the centroid of the largest contour within a specified crop region.

        Args:
        - masked_image (np.array): The masked image to process.
        - crop_indices (tuple): A tuple (x1, y1, x2, y2) specifying the crop region.

        Returns:
        - (int, int): The (x, y) coordinates of the centroid in the original image.
        """

        # Unpack the crop region indices
        x1, y1, x2, y2 = crop_indices

        # Crop the masked image based on the provided indices
        cropped_image = masked_image[y1:y2, x1:x2]

        # Convert cropped image to numpy array (if necessary)
        image = np.array(cropped_image, dtype=np.uint8)

        # Find all contours in the cropped region
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour area
        largest_area = -1
        largest_index = -1

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])

            # Check if the current contour has the largest area so far
            if area > largest_area:
                largest_area = area
                largest_index = i

        # If no valid contour was found (largest_area remains 0)
        if (largest_area == 0) or (largest_area == -1):
            return 0, 0

        # Get the centroid in the cropped region

        moments = cv2.moments(contours[largest_index])

        if moments["m00"] == 0:  # To avoid division by zero
            return 0, 0

        # Calculate the centroid in the cropped image
        x_c = int(moments["m10"] / moments["m00"])
        y_c = int(moments["m01"] / moments["m00"])

        # Convert the cropped image centroid back to original image coordinates
        original_x_c = x1 + x_c
        original_y_c = y1 + y_c

        return original_x_c, original_y_c

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

    def get_3d_coordinates_at_pixel(self, x, y, frame_name=""):
        """Convert the pixel coordinates (x, y) to 3D camera coordinates in meters."""

        if self.camera_intrinsics.shape[0] == 0:
            self.log("Camera intrinsics not yet received.")
            return -1, -1, -1

        # Get the depth value at the specified pixel (x, y)
        depth_in_meters = self.depth_image[y, x] / 1000

        if depth_in_meters == 0:  # No depth data at this pixel
            self.log(f"[{frame_name}] No valid depth data at pixel ({x}, {y}).")
            return -1, -1, -1

        # Extract intrinsic parameters from the camera info (in a 3x3 matrix form)
        f_x, f_y = (
            self.camera_intrinsics[0],
            self.camera_intrinsics[4],
        )  # Focal lengths (in pixels)
        c_x, c_y = (
            self.camera_intrinsics[2],
            self.camera_intrinsics[5],
        )  # Optical center (in pixels)

        # Convert to 3D coordinates (X, Y, Z) in camera frame
        X = (x - c_x) * depth_in_meters / f_x
        Y = (y - c_y) * depth_in_meters / f_y
        Z = depth_in_meters  # Depth in meters
        return X, Y, Z


def entry(args=None):

    rclpy.init(args=args)

    camera_node = Camera()

    rclpy.spin(camera_node)
