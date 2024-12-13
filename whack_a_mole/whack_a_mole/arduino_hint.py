"""Hint node that receives feedback from the Arduino on which button is lit."""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
from std_msgs.msg import String
from std_srvs.srv import Empty
from whack_a_mole_interfaces.srv import TargetFrame


mole_comm = serial.Serial()


class Hint(Node):
    """
    ROS2 Hint Node for serial communication and service interaction.

    Responsibilities:
    - Communicates with an Arduino device over a serial port.
    - Handles client calls to start a game and process received messages.
    - Interfaces with the `play` and `toggle_tf_publish` services.

    **Services**:
    - `call_play` (`Empty`): Initiates the game and reads serial input from the Arduino.

    **Clients**:
    - `play` (`TargetFrame`): Sends color data for target frames to a service.
    - `toggle_tf_publish` (`Empty`): Toggles the transform publisher.

    """

    def __init__(self):
        """
        Initialize the Hint node, sets up serial comm, and creates ROS2 service and clients.

        :arg None: No arguments are passed during initialization.
        :type None: NoneType

        :raises RuntimeError: If the Arduino cannot be found during serial port discovery.
        """
        super().__init__('hint')

        self.play_client = self.create_client(TargetFrame, 'play')

        self.toggle_client = self.create_client(Empty, 'toggle_tf_publish')

        ARDUINO_SERIAL = '34336313537351203262'

        # ports discovery
        ports = list(list_ports.comports(True))  # get list of ports
        arduino_port = None

        # grab the port that's connected to the Arduino
        for p in ports:
            if p.serial_number == ARDUINO_SERIAL:
                arduino_port = p.device
                break

        # check Arduino has been found
        if arduino_port is None:
            self.get_logger().error('error finding arduino')

        self.connect_serial_port(serial_port=arduino_port, baud_rate=115200)

        self.call_play = self.create_service(
            Empty,
            'call_play',
            self.call_play_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    async def call_play_callback(self, request, response):
        """
        Start playing. Callback function for the `call_play` service.

        This function starts the game by:
        - Sending a start signal (`'s'`) to the Arduino over the serial port.
        - Calling the `toggle_tf_publish` client asynchronously.
        - Continuously reading data from the Arduino and interpreting it as color frames.
        - Passing the interpreted frames to the `play` service.

        :arg request: An empty request from the `call_play` service.
        :type request: `Empty.Request`
        :arg response: An empty response returned to the service caller.
        :type response: `Empty.Response`

        :raises Exception: If the game cannot be started or serial communication fails.
        :return: The response to the service.
        :rtype: `Empty.Response`
        """
        try:
            start = str('s')
            mole_comm.write(start.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Could not start the game: {e}')
        empty_req = Empty.Request()
        await self.toggle_client.call_async(empty_req)
        while True:
            msg = String()
            msg = await self.read_serial_data()
            self.get_logger().info(f'{msg}')
            if msg.data == '0':
                msg.data = 'YELLOW_frame'
            elif msg.data == '1':
                msg.data = 'BLUE_frame'
            elif msg.data == '2':
                msg.data = 'GREEN_frame'
            elif msg.data == '3':
                msg.data = 'RED_frame'
            frame_req = TargetFrame.Request()
            frame_req.color = msg
            self.get_logger().info(f'Passed the msg {msg}')
            await self.play_client.call_async(frame_req)
            self.get_logger().info('Returned from play client')

    async def read_serial_data(self):
        """
        Read serial data from the Arduino device.

        This function continuously reads from the serial port
        until valid data (`0`, `1`, `2`, or `3`) is received.

        :return: A message containing the valid serial data read from the device.
        :rtype: `String`

        :raises Exception: If there is an error reading data from the serial port.
        """
        try:
            msg = String()
            msg.data = mole_comm.readline().decode('utf-8').rstrip('\n').rstrip('\r')
            self.get_logger().info(f'{type(msg.data)}')
            while msg.data != '0' and msg.data != '1' and msg.data != '2' and msg.data != '3':
                msg.data = mole_comm.readline().decode('utf-8').rstrip('\n').rstrip('\r')
                self.get_logger().info(f'Received data from: {msg.data}')
            return msg
        except Exception as e:
            self.get_logger().error(f'Exception{e}')
            self.get_logger().error('Cannot read serial data!')
            return

    def connect_serial_port(self, serial_port, baud_rate):
        """
        Configure and opens the serial port for communication with the Arduino device.

        :arg serial_port: The name of the serial port to connect to.
        :type serial_port: str
        :arg baud_rate: The baud rate for serial communication.
        :type baud_rate: int
        """
        mole_comm.port = serial_port
        mole_comm.baudrate = baud_rate
        mole_comm.timeout = 1
        mole_comm.open()


def hint_main(args=None):
    """
    Entry point for starting the Hint node.

    :arg args: Optional command-line arguments passed to the ROS2 program.
    :type args: list or NoneType
    """
    rclpy.init(args=args)
    node = Hint()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    hint_main(args=sys.argv)
