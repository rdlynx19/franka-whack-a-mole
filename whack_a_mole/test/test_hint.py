import unittest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
import rclpy
from std_srvs.srv import Empty
from whack_a_mole_interfaces.srv import TargetFrame
from whack_a_mole.arduino_hint import Hint
from unittest.mock import patch, MagicMock

@pytest.mark.rostest
def generate_test_description():
    # Launch the Hint node
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