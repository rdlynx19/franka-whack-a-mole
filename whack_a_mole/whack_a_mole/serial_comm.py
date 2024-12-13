"""
Communication Node
==================

This module implements the `CommunicationNode` class, which facilitates serial communication with an Arduino to control a hammer's actuation in the Whack-A-Mole game.

Classes
-------
CommunicationNode
    A node that manages communication with the Arduino and handles hammer actuation using a ROS2 action server.

Functions
---------
node_main(args=None)
    Entry point to run the `CommunicationNode`.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from whack_a_mole_interfaces.action import ActuateServo
import serial
import serial.tools.list_ports as list_ports

ser_comm = serial.Serial()


class CommunicationNode(Node):
    """
    ROS2 Node for Arduino communication and hammer actuation control.

    This node:
    - Establishes a serial connection with the Arduino.
    - Handles hammer actuation through a ROS2 action server.
    """

    def __init__(self):
        """
        Initializes the CommunicationNode.

        - Discovers and connects to the Arduino via serial port.
        - Sets up the `swing_hammer` ROS2 action server.

        :arg None: No arguments are passed during initialization.
        :type None: NoneType

        :raises RuntimeError: If the Arduino cannot be found during serial port discovery.
        """
        super().__init__('comm_node')

        # Create an action server for swinging the hammer
        self.hammer_action_server = ActionServer(
            self, 
            ActuateServo,
            'swing_hammer',
            self.swing_callback
        )

        # Expected Arduino serial number
        ARDUINO_SERIAL = "F412FA6FA49C"

        # Discover available ports
        ports = list(list_ports.comports(True))  # Get list of ports
        arduino_port = None

        # Find the port connected to the Arduino
        for p in ports:
            if p.serial_number == ARDUINO_SERIAL:
                arduino_port = p.device
                break

        # Check if the Arduino was detected
        if arduino_port is None:
            self.get_logger().error("Error finding Arduino")

        # Establish serial communication with the Arduino
        self.connect_serial_port(serial_port=arduino_port, baud_rate=115200)

    async def swing_callback(self, goal_handle):
        """
        Handles the `swing_hammer` action server callback.

        This callback determines whether to "raise" or "hit" the hammer based on the request. It sends the appropriate command to the Arduino and waits for completion.

        :arg goal_handle: The action handle containing the request details.
        :type goal_handle: `ActuateServo.GoalHandle`

        :return: The result of the hammer actuation.
        :rtype: `ActuateServo.Result`

        :raises ValueError: If the request position is invalid or not of type `String`.
        """
        # Ensure the request position is a String
        if not isinstance(goal_handle.request.position, String):
            self.get_logger().error(f"Expected String for position, got: {type(goal_handle.request.position)}")
            result = ActuateServo.Result()
            result.res = False  # Signal an error in the result
            goal_handle.abort()  # Abort the goal
            return result

        # Process the request position
        if goal_handle.request.position.data == 'raise':
            # Command Arduino to raise the hammer
            msg = String()
            msg.data = 'r'
            self.write_serial_data(msg)
            self.get_logger().info('Raising the hammer')
        elif goal_handle.request.position.data == 'hit':
            # Command Arduino to hit
            msg = String()
            msg.data = 'h'
            self.write_serial_data(msg)
            self.get_logger().info('Hitting the hammer') 
        else:
            # Handle invalid input
            self.get_logger().error("No valid string option sent. Send 'raise' or 'hit'!")

        self.get_logger().info('Arduino finished moving hammer, proceeding to the next step')

        # Return success result
        result = ActuateServo.Result()
        result.res = True
        goal_handle.succeed()
        return result

    def write_serial_data(self, msg: String):
        """
        Sends a message to the Arduino over the serial connection.

        :arg msg: The message to send.
        :type msg: `String`

        :raises Exception: If the serial communication fails.
        """
        try:
            msg = str(msg.data)
            ser_comm.write(msg.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f'Cannot write serial data!: {e}')

    async def read_serial_data(self):
        """
        Reads data from the Arduino until a specific condition is met.

        The function waits for the Arduino to send the character `'d'`.

        :return: `True` if `'d'` is received; `False` otherwise.
        :rtype: bool

        :raises Exception: If there is an error during serial reading.
        """
        try:
            msg = String()
            msg.data = ser_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            while msg.data != 'd':  # Loop until `'d'` is received
                msg.data = ser_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
                self.get_logger().info(f'Received: {msg.data}')
            return True 
        except Exception as e:
            self.get_logger().error(f'Cannot read serial data!: {e}')
            return False

    def connect_serial_port(self, serial_port, baud_rate):
        """
        Configures and opens the serial connection.

        :arg serial_port: The serial port to connect to.
        :type serial_port: str
        :arg baud_rate: The baud rate for serial communication.
        :type baud_rate: int

        :raises Exception: If the serial connection cannot be established.
        """
        ser_comm.port = serial_port
        ser_comm.baudrate = baud_rate
        ser_comm.timeout = 1
        ser_comm.open()


def node_main(args=None):
    """
    Entry point to run the `CommunicationNode`.

    Initializes the ROS2 node and starts spinning.

    :arg args: Optional arguments for the ROS2 node.
    :type args: list or NoneType
    """
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    node_main(args=sys.argv)
