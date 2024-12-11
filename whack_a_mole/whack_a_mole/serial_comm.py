import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from whack_a_mole_interfaces.action import ActuateServo
import serial
import serial.tools.list_ports as list_ports

ser_comm = serial.Serial()

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('comm_node')

        self.hammer_action_server = ActionServer(
            self, 
            ActuateServo,
            'swing_hammer',
            self.swing_callback
        )
        ARDUINO_SERIAL = "F412FA6FA49C"

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
            self.get_logger.error("error finding arduino")

        self.connect_serial_port(serial_port=arduino_port, baud_rate=115200)

    async def swing_callback(self, goal_handle):
        if not isinstance(goal_handle.request.position, String):
            self.get_logger().error(f"Expected string for position, got: {type(goal_handle.request.position)}")
            result = ActuateServo.Result()
            result.res = False  # Signal an error in the result
            goal_handle.abort()  # Abort the goal
            return result

        if(goal_handle.request.position.data == 'raise'):
            msg = String()
            msg.data = 'r'
            self.write_serial_data(msg)
            self.get_logger().info('Raising the hammer')
        elif(goal_handle.request.position.data == 'hit'):
            msg = String()
            msg.data = 'h'
            self.write_serial_data(msg)
            self.get_logger().info('Hitting the hammer') 
        else:
            self.get_logger().error("No valid string option sent, Send raise or hit!")

        serial_val = await self.read_serial_data()
        self.get_logger().info('Arduino finished moving hammer, moving on to next step')
        
        result = ActuateServo.Result()
        result.res = True
        goal_handle.succeed()

        return result
        

    def write_serial_data(self, msg: String):
        try:
            msg = str(msg.data)
            ser_comm.write(msg.encode("utf-8"))
        except Exception as e:
            self.get_logger().error('Cannot write serial data!: {e}')

    async def read_serial_data(self):
        try:
            msg = String()
            msg.data = ser_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            while msg.data != 'd':
                msg.data = ser_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
                self.get_logger().info(f'{msg.data}')
            return True 
        except Exception as e:
            self.get_logger().error(f'Cannot read serial data!: {e}')
            return False


    def connect_serial_port(self, serial_port, baud_rate):
        ser_comm.port = serial_port
        ser_comm.baudrate = baud_rate
        ser_comm.timeout = 1
        ser_comm.open()

def node_main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    import sys
    node_main(args=sys.argv)