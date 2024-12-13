"""Test the Hint node."""

import unittest
from unittest.mock import MagicMock, patch

from launch import LaunchDescription

from launch_ros.actions import Node

from launch_testing.actions import ReadyToTest

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor

from whack_a_mole.arduino_hint import Hint


@pytest.mark.rostest
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    hint_node = Node(
        package='whack_a_mole',
        executable='hint',
        name='hint',
        output='screen'
    )

    return (
        LaunchDescription([
            hint_node,
            ReadyToTest()
        ]),
        {'hint_node': hint_node}
    )


class TestHintNode(unittest.TestCase):
    """Test the Hint node."""

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
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Destroy the test node."""
        self.node.destroy_node()

    # --------------- Begin Citation [1] -------------------#
    @patch('serial.Serial')
    def test_read_serial_data(self, mock_serial):
        """Test the read_serial_data method of the Hint node."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.readline.side_effect = [
            b'0\n', b'1\n', b'2\n', b'3\n'
        ]
        mock_serial.return_value = mock_serial_instance

        hint_node = Hint()

        executor = SingleThreadedExecutor()
        executor.add_node(hint_node)

        try:
            rclpy.spin_once(hint_node, timeout_sec=1)

            msg = hint_node.read_serial_data()
            self.assertEqual(msg.data, '0')

            msg = hint_node.read_serial_data()
            self.assertEqual(msg.data, '1')

            msg = hint_node.read_serial_data()
            self.assertEqual(msg.data, '2')

            msg = hint_node.read_serial_data()
            self.assertEqual(msg.data, '3')
        finally:
            executor.shutdown()
            hint_node.destroy_node()
    # --------------- End Citation [1] -------------------#
