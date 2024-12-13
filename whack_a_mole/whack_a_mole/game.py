from geometry_msgs.msg import Pose, TransformStamped
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from whack_a_mole_interfaces.action import ActuateServo
from whack_a_mole_interfaces.srv import TargetFrame


class Game(Node):
    """
    ROS2 Game Node for controlling Franka to interact with a target in a game.

    The node:
    - Calculates poses for the robotic arm based on TF transforms.
    - Interacts with various ROS2 services and actions for robot motion control.
    - Handles game-specific functionality like moving to specific color targets.

    **Services**:
    - `play` (`TargetFrame`): Starts the game and moves the robot to the target frame.
    - `go_home` (`Empty`): Sends the robotic arm to the default home position.

    **Clients**:
    - `pick` (`PickPose`): Calls a service to move the robot to the provided goal pose.
    - `swing_hammer` (`ActuateServo`): An action client to actuate the hammer motion.

    **TF Transformations**:
    - Sets up a static transformation from `base_tag` to `base`.

    **Constants**:
    - READ! This node uses specific offsets for hammer and tag positions.
    - Make sure to change these values based on your setup.
    """

    def __init__(self):
        """
        Initialize the Game node.

        Sets up:
        - Motion planning interface.
        - Services for game and home functionalities.
        - Clients for picking and actuation.
        - TF transformations for base to tag calculations.

        :arg None: No arguments are passed during initialization.
        :type None: NoneType
        """
        super().__init__('game')
        self.mpi = MotionPlanningInterface(self)

        self.serv = self.create_service(
            TargetFrame,
            'play',
            self.play_game,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.hammer_client = ActionClient(
            self,
            ActuateServo,
            'swing_hammer'
        )

        self.go_home = self.create_service(
            Empty,
            'go_home',
            self.go_home_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.pick_client = self.create_client(PickPose, 'pick')
        self.static = StaticTransformBroadcaster(self)
        self.tag0_to_base = TransformStamped()
        self.tag0_to_base.header.stamp = self.get_clock().now().to_msg()
        self.tag0_to_base.header.frame_id = 'base_tag'
        self.tag0_to_base.child_frame_id = 'base'
        self.get_logger().info('Setting up the tag0 transform')

        # Base to base tag offsets
        self.tag0_to_base.transform.translation.x = 0.155
        self.tag0_to_base.transform.translation.y = -0.085
        self.tag0_to_base.transform.translation.z = 0.0
        q = [-0.5, -0.5, -0.5, 0.5]
        self.tag0_to_base.transform.rotation.x = q[0]
        self.tag0_to_base.transform.rotation.y = q[1]
        self.tag0_to_base.transform.rotation.z = q[2]
        self.tag0_to_base.transform.rotation.w = q[3]
        self.static.sendTransform(self.tag0_to_base)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    async def play_game(self, request, response):
        """
        Move the hammer to the target frame.

        This service is called with a target frame.
        The robot moves to the target,
        calculates the appropriate pose based on TF transformations,
        and executes the required motion.

        :arg request: The service request containing the target color frame.
        :type request: `TargetFrame.Request`
        :arg response: The service response.
        :type response: `TargetFrame.Response`

        :return: The response to the service caller.
        :rtype: `TargetFrame.Response`

        :raises Exception: If there is an error in TF  lookup or motion execution.
        """
        received_frame = request.color
        tag_frame = received_frame.data
        try:
            self.base_to_tag = self.buffer.lookup_transform('base', tag_frame, rclpy.time.Time())
            goal_pose = Pose()

            # Hammer offsets
            goal_pose.position.x = self.base_to_tag.transform.translation.x - 0.135
            goal_pose.position.y = self.base_to_tag.transform.translation.y - 0.084
            goal_pose.position.z = 0.32
            rotation = [3.14, 3.14, 0.0, 0.0]
            goal_pose.orientation.x = rotation[0]
            goal_pose.orientation.y = rotation[1]
            goal_pose.orientation.z = rotation[2]
            goal_pose.orientation.w = rotation[3]

            # Call the pick client
            pick_req = PickPose.Request()
            pick_req.pick_point = goal_pose
            _ = await self.pick_client.call_async(pick_req)
            self.get_logger().info(f'Moved to {tag_frame}')
        except Exception as e:
            self.get_logger().error(f'Error in move_to_tag: {e}')

        return response

    async def go_home_callback(self, request, response):
        """
        Service callback to send the robot back to its home position.

        :arg request: An empty service request.
        :type request: `Empty.Request`
        :arg response: An empty service response.
        :type response: `Empty.Response`

        :return: The response to the service caller.
        :rtype: `Empty.Response`

        :raises Exception: If there is an error during the motion to the home position.
        """
        try:
            await self.mpi.go_home_callback()
        except Exception as e:
            self.get_logger().error(f'Error in go_home_callback: {e}')
        return response


def main(args=None):
    """
    Entry point to start the Game node.

    This function initializes the ROS2 system, creates the Game node, and keeps it spinning.

    :arg args: Optional command-line arguments passed to the ROS2 system.
    :type args: list or NoneType
    """
    rclpy.init(args=args)
    game = Game()
    rclpy.spin(game)
    game.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(args=sys.argv)
