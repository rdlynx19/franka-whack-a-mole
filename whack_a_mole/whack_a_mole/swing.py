"""
Swing Node
==========

This module implements the `Swing` class, which integrates hammer actuation with the motion planner for enabling precise robotic movements and interactions in the Whack-A-Mole game.

Classes
-------
Swing
    A ROS2 node responsible for planning and executing robot arm movements and actuating the hammer.

Functions
---------
main(args=None)
    Entry point to run the `Swing` node.
"""

from geometry_msgs.msg import Pose
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from whack_a_mole_interfaces.action import ActuateServo


class Swing(Node):
    """
    ROS2 Node for integrating hammer actuation with MotionPlanningInterface.

    This node:
    - Handles motion planning for robot arm movements via the MotionPlanningInterface.
    - Actuates the hammer to perform actions like "hit."

    **Services**:
    - `pick` (`PickPose`): Service to handle object picking by planning and executing a motion trajectory.

    **Clients**:
    - `swing_hammer` (`ActuateServo`): Action client to actuate the hammer's movements.
    """

    def __init__(self):
        """
        Initializes the Swing node.

        Sets up:
        - Motion planning interface.
        - ROS2 service for object picking.
        - ROS2 action client for hammer actuation.

        :arg None: No arguments are passed during initialization.
        :type None: NoneType
        """
        super().__init__('swing')
        self.mpi = MotionPlanningInterface(self)

        # Service to handle picking tasks
        self.serv = self.create_service(
            PickPose,
            'pick',
            self.pick_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Action client for hammer actuation
        self.hammer_client = ActionClient(
            self,
            ActuateServo,
            'swing_hammer'
        )

    async def pick_callback(self, request: PickPose, response):
        """
        Service callback to execute picking operations.

        Handles robot arm movement to a target pose and actuates the hammer.

        :param request: Service request containing the pick point.
        :type request: `PickPose.Request`
        :param response: Service response to be sent back to the caller.
        :type response: `PickPose.Response`

        :return: The response to the service caller.
        :rtype: `PickPose.Response`

        **Steps**:
        1. Plan and execute trajectory to move the arm to the target pose.
        2. Actuate the hammer to "hit" the target.
        """
        # Extract the target object pose from the request
        object_pose = Pose()
        object_pose = request.pick_point

        # Step 1: Move the robotic arm to the specified pose (e.g., next to the button)
        pose1 = object_pose
        traj1 = await self.mpi.plan_path(goal_pose=pose1)
        _ = await self.mpi.exec_path(traj1)
        self.get_logger().info('Step 1: Finished moving arm to button')

        # Step 2: Actuate the hammer to "hit"
        msg = String()
        actuate_msg = ActuateServo.Goal()
        msg.data = 'hit'  # Command to signal the hammer actuation
        actuate_msg.position = msg
        goal_handle = await self.hammer_client.send_goal_async(actuate_msg)
        self.get_logger().info(f'Step 2: Actuating hammer to {msg.data}!')
        await goal_handle.get_result_async()

        return response


def main(args=None):
    """
    Entry point to start the Swing node.

    This function initializes the ROS2 system, creates the Swing node, and keeps it spinning.

    :arg args: Optional command-line arguments passed to the ROS2 system.
    :type args: list or NoneType
    """
    rclpy.init(args=args)
    node = Swing()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    main(args=sys.argv)
