"""Test the Camera node."""

import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

from launch_testing.actions import ReadyToTest
from launch_testing_ros import WaitForTopics

import pytest

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image

from std_srvs.srv import Empty


@pytest.mark.rostest
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    camera_node = Node(
        package='whack_a_mole',
        executable='camera',
        name='color_camera',
        parameters=[{
            'clipping_distance': 1400,
            'box_start_x': 10,
            'box_end_x': 100,
            'box_start_y': 10,
            'box_end_y': 100
        }],
        remappings=[
            (
                '/camera/camera/aligned_depth_to_color/image_raw',
                'test_depth_image'),
            ('/camera/camera/color/image_raw', 'test_color_image'),
            ('/camera/camera/color/camera_info', 'test_camera_info')
        ]
    )

    return (
        LaunchDescription([
            camera_node,
            ReadyToTest()
        ]),
        {'camera_node': camera_node}
    )


class TestCameraNode(unittest.TestCase):
    """Test the Camera node."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 before running any tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after tests."""
        rclpy.shutdown()

    def setUp(self):
        """Set up the test node."""
        self.node = RclpyNode('test_camera_node')

    def tearDown(self):
        """Destroy the test node."""
        self.node.destroy_node()

    # --------------- Begin Citation [1] -------------------#
    def test_camera_node_startup(self):
        """Test that the Camera node starts and subscribes to topics."""
        # Wait for the Camera node to subscribe to the expected topics
        topic_checker = WaitForTopics(
            ['test_depth_image', 'test_color_image'],
            timeout=5.0, node=self.node)
        self.assertTrue(
            topic_checker.wait(),
            'Camera node did not subscribe to expected topics.')

    def test_toggle_tf_publish_service(self):
        """Test the toggle_tf_publish service of the Camera node."""
        client = self.node.create_client(Empty, 'toggle_tf_publish')

        # Wait for service to be available
        self.assertTrue(client.wait_for_service
                        (timeout_sec=5.0),
                        'Service toggle_tf_publish not available.')

        # Call the service and check the response
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        self.assertIsNotNone(
            future.result(),
            'Service call to toggle_tf_publish failed.')

    def test_image_processing(self, launch_service, camera_node, proc_output):
        """Test that the Camera node publishes processed images."""
        processed_image_received = False

        def processed_image_callback(msg):
            nonlocal processed_image_received
            processed_image_received = True

        self.node.create_subscription(
            Image,
            'filtered_image',
            processed_image_callback,
            QoSProfile(depth=10)
        )

        # Publish a test color image
        pub = self.node.create_publisher(
            Image,
            'test_color_image',
            QoSProfile(depth=10))
        test_image = Image()
        test_image.header.frame_id = 'test_frame'
        pub.publish(test_image)

        # Wait for the callback to confirm the image was processed
        end_time = self.node.get_clock().now().seconds + 5.0
        while (
            not processed_image_received and
                self.node.get_clock().now().seconds < end_time):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(
            processed_image_received,
            'Processed image was not published.')
        # --------------- End Citation [1] -------------------#
