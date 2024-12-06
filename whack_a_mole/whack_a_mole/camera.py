import pyrealsense2 as rs
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image , PointCloud2
from realsense2_camera_msgs.msg import RGBD

class Camera(Node):

    def __init__(self):
        
        super().__init__('color_camera')

        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.process_image,
            qos_profile= QoSProfile(depth = 10)
        )


    def log(self,*message):

        self.get_logger().info(f"[CAMERA NODE]: {",".join(str(i) for i in message)}")

    def process_image(self, msg : Image):
        
        # data = np.array(msg.data).reshape((msg.width,msg.height,2))

        depth_array = CvBridge().imgmsg_to_cv2(msg)

        
        # self.log(f"{np.max(data[:,:,1]), np.min(data[:,:,1])}")

        # self.log(depth_array)

        center_x, center_y = depth_array.shape[1] // 2, depth_array.shape[0] // 2
        depth_value = depth_array[center_y, center_x]
        self.get_logger().info(f"Depth at center ({center_x}, {center_y}): {depth_value} mm")




def entry(args = None):

    rclpy.init(args=args)

    camera_node = Camera()

    rclpy.spin(camera_node)
