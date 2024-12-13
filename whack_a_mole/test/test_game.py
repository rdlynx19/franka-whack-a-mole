import unittest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
import rclpy
from whack_a_mole_interfaces.srv import TargetFrame
from object_mover_interfaces.srv import PickPose
from std_srvs.srv import Empty

@pytest.mark.rostest
def generate_test_description():
    # Launch the Game node
    game_node = Node(
        package='your_package_name',
        executable='game',
        output='screen'
    )

    return (
        LaunchDescription([
            game_node,
            ReadyToTest()
        ]),
        {'game_node': game_node}
    )

class TestGameNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    # --------------- Begin Citation [1] -------------------#
    def test_play_game_service(self, launch_service, game_node, proc_output):
        """Test the 'play' service of the Game node."""
        client = self.node.create_client(TargetFrame, 'play')
        self.assertTrue(client.wait_for_service(timeout_sec=5), "Play service not available")

        # Send a request to the play service
        request = TargetFrame.Request()
        request.color.data = 'tag_frame'

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
        
        # Check the response
        response = future.result()
        self.assertIsNotNone(response, "No response from play service")

    def test_pick_client_interaction(self, launch_service, game_node, proc_output):
        """Test the interaction with the pick client."""
        pick_client = self.node.create_client(PickPose, 'pick')
        self.assertTrue(pick_client.wait_for_service(timeout_sec=5), "Pick service not available")

        # Send a request to the pick service
        request = PickPose.Request()
        request.pick_point.position.x = 0.1
        request.pick_point.position.y = 0.1
        request.pick_point.position.z = 0.1
        request.pick_point.orientation.x = 0.0
        request.pick_point.orientation.y = 0.0
        request.pick_point.orientation.z = 0.0
        request.pick_point.orientation.w = 1.0

        future = pick_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)

        # Check the response
        response = future.result()
        self.assertIsNotNone(response, "No response from pick service")
    # --------------- End Citation [1] -------------------#