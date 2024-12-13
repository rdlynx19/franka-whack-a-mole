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
    """Node that receives feedback from the Arduino on which button is lit."""

    def __init__(self):
        """Initialize the node."""
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

        self.connect_serial_port(
            serial_port=arduino_port,
            baud_rate=115200)

        self.call_play = self.create_service(
            Empty,
            'call_play',
            self.call_play_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

    async def call_play_callback(self, request, response):
        """Play game."""
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
            if (msg.data == '0'):
                msg.data = 'YELLOW_frame'
            elif (msg.data == '1'):
                msg.data = 'BLUE_frame'
            elif (msg.data == '2'):
                msg.data = 'GREEN_frame'
            elif (msg.data == '3'):
                msg.data = 'RED_frame'
            frame_req = TargetFrame.Request()
            frame_req.color = msg
            self.get_logger().info(f'Passed the msg {msg}')
            await self.play_client.call_async(frame_req)
            self.get_logger().info('Returned from play client')
        return response

    async def read_serial_data(self):
        """Read serial data."""
        try:
            msg = String()
            msg.data = (
                mole_comm.readline().decode('utf-8').rstrip('\n').rstrip('\r')
            )
            self.get_logger().info(f'{type(msg.data)}')
            while msg.data not in {'0', '1', '2', '3'}:
                msg.data = mole_comm.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received data from: {msg.data}')
            return msg
        except Exception as e:
            self.get_logger().error(f'Exception{e}')
            self.get_logger().error("Can't read serial data!")
            return

    def connect_serial_port(self, serial_port, baud_rate):
        """Connect to the serial port."""
        mole_comm.port = serial_port
        mole_comm.baudrate = baud_rate
        mole_comm.timeout = 1
        mole_comm.open()


def hint_main(args=None):
    """Run the Hint node."""
    rclpy.init(args=args)
    node = Hint()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    hint_main(args=sys.argv)
