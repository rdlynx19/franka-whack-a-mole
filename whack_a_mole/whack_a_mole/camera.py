import pyrealsense2 as rs
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image , PointCloud2
from realsense2_camera_msgs.msg import RGBD

from std_msgs.msg import Header

from time import sleep

class Camera(Node):

    def __init__(self):
        
        super().__init__('color_camera')

        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.get_depth_info,
            qos_profile= QoSProfile(depth = 10)
        )

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.get_color_info,
            qos_profile= QoSProfile(depth = 10)
        )

        self.image_pub = self.create_publisher(
            Image,
            "filtered_image",
            qos_profile= QoSProfile(depth = 10)
        )
        

        self.color_image = np.array([])
        self.depth_image = np.array([])

        self.freq = 10.0 # Hz

        self.create_timer(
            1/self.freq,
            self.timer_callback
        )

    def timer_callback(self):

        if (self.color_image.shape[0] == 0 or self.depth_image.shape[0] == 0): self.log("Waiting for image data...") ;return

        lower_HSV = np.array((115, 103, 83))
        higher_HSV = np.array((130, 255, 179) )

        # lower_HSV = np.array([255, 0, 255])  # Lower bound of HSV (e.g., a certain shade of yellow)
        # higher_HSV = np.array([255, 255, 255])  # Upper bound of HSV

        # lower_HSV = np.array((15,0,0))  # Lower bound of red (from 0)
        # higher_HSV = np.array((36, 255, 255))  # Upper bound of red (up to 10)

        
        self.find_color_centroid(lower_HSV,higher_HSV)
        

    def log(self,*message):

        self.get_logger().info(f"[CAMERA NODE]: {",".join(str(i) for i in message)}")

    def get_depth_info(self, msg : Image):
        
        self.depth_image = CvBridge().imgmsg_to_cv2(msg)

        # center_x, center_y = depth_array.shape[1] // 2, depth_array.shape[0] // 2
        # depth_value = depth_array[center_y, center_x]
        # self.get_logger().info(f"Depth at center ({center_x}, {center_y}): {depth_value} mm")

    def get_color_info(self, msg : Image):

        self.color_image = CvBridge().imgmsg_to_cv2(msg,desired_encoding="bgr8")

    

    def find_color_centroid(self,lower_HSV : np.array, higher_HSV : np.array):
        

        # self.image_pub.publish(CvBridge().cv2_to_imgmsg(self.color_image,encoding="rgb8"))

        mask = cv2.inRange(cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV),lower_HSV,higher_HSV)

        image_filtered = cv2.bitwise_and(self.color_image,self.color_image, mask = mask)

        msg = CvBridge().cv2_to_imgmsg(image_filtered,encoding="bgr8")

        self.image_pub.publish(msg)



def entry(args = None):

    rclpy.init(args=args)

    camera_node = Camera()

    rclpy.spin(camera_node)
