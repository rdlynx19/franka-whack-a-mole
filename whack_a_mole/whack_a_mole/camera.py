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

        masked_image = cv2.bitwise_and(self.color_image,self.color_image, mask = mask)


        if(not np.any(mask)): return np.array([-1,-1,-1])

        #find centroid

        #Find Centroid
        x_c , y_c = self.find_centroid(mask)

        x_c = int(x_c)
        y_c = int(y_c)

        cv2.circle(masked_image,(x_c,y_c),5,(0, 0, 255),thickness = 10)


        msg = CvBridge().cv2_to_imgmsg(masked_image,encoding="bgr8")

        self.image_pub.publish(msg)

        


    def find_centroid(self,masked_image):

                # Get all countours
                # https://docs.opencv.org/4.6.0/d4/d73/tutorial_py_contours_begin.html

                image = np.array(masked_image,dtype=np.uint8)
                contours,_  = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                
                #Find the centroid
                #https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
                
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
                                
                
                #Calculate the centroid
                

                moments = cv2.moments(contours[largest_index])

                

                x_c = (moments["m10"])/ (largest_area)
                y_c = (moments["m01"])/(largest_area)

                return x_c,y_c



def entry(args = None):

    rclpy.init(args=args)

    camera_node = Camera()

    rclpy.spin(camera_node)
