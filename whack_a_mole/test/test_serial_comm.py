import unittest
from unittest.mock import MagicMock, patch
import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from whack_a_mole_interfaces.action import ActuateServo
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
from launch_testing.legacy import WaitForTopics

@pytest.mark.rostest
def generate_test_description():
    comm_node = LaunchNode(
        package='whack_a_mole',
        executable='comm_node',
        name='comm_node',
        output='screen',
    )

    return (
        LaunchDescription([
            comm_node,
            ReadyToTest(),
        ]),
        {'comm_node': comm_node},
    )

class TestCommunicationNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 before running any tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after tests."""
        rclpy.shutdown()

    def setUp(self):
        """Set up the test node and action client."""
        self.node = rclpy.create_node('test_comm_node')
        self.action_client = ActionClient(self.node, ActuateServo, 'swing_hammer')

    def tearDown(self):
        """Destroy the test node."""
        self.node.destroy_node()

    # --------------- Begin Citation [1] -------------------#
    @patch('serial.Serial')
    def test_serial_data_transmission(self, mock_serial):
        """Test that the node sends the correct serial data when handling goals."""
        # Mock the serial communication
        mock_serial_instance = mock_serial.return_value
        mock_serial_instance.write = MagicMock()

        # Wait for the action server to be available
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))

        # Send a goal to the action server
        goal_msg = ActuateServo.Goal()
        goal_msg.position = 'raise'
        future = self.action_client.send_goal_async(goal_msg)

        # Spin until the goal is sent
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        # Verify the correct data was sent to the serial port
        mock_serial_instance.write.assert_called_once_with(b'r')

    @patch('serial.Serial')
    def test_serial_data_reception(self, mock_serial):
        """Test that the node correctly processes received serial data."""
        # Mock the serial communication
        mock_serial_instance = mock_serial.return_value
        mock_serial_instance.readline = MagicMock(side_effect=[
            b'r\n',
            b'h\n',
            b'd\n',  # Simulate the acknowledgment
        ])

        # Wait for the action server to be available
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))

        # Send a goal to the action server
        goal_msg = ActuateServo.Goal()
        goal_msg.position = 'hit'
        future = self.action_client.send_goal_async(goal_msg)

        # Spin until the goal is sent
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        # Simulate waiting for acknowledgment ('d')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        # Check the result
        result = result_future.result().result
        self.assertTrue(result.res)

        # Verify the logger processed the received data
        self.assertEqual(mock_serial_instance.readline.call_count, 3)
    # --------------- End Citation [1] -------------------#    

