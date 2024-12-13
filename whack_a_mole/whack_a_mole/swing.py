"""Integrate the hammer actuation with the MotionPlanner."""

from geometry_msgs.msg import Pose

from object_mover.MotionPlanningInterface import MotionPlanningInterface

from object_mover_interfaces.srv import PickPose

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from std_msgs.msg import String

from whack_a_mole_interfaces.action import ActuateServo


class Swing(Node):
    """Node for swinging the hammer."""

    def __init__(self):
        """Initialize the node."""
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

        # Step 2: Actuating the hammer to hit!
        msg = String()
        actuate_msg = ActuateServo.Goal()
        msg.data = 'hit'  # Command to signal the hammer actuation
        actuate_msg.position = msg
        goal_handle = await self.hammer_client.send_goal_async(actuate_msg)
        self.get_logger().info(f'Step 2: Actuating hammer to {msg.data}!')
        await goal_handle.get_result_async()

        return response


def main(args=None):
    """Run the Swing node."""
    rclpy.init(args=args)
    node = Swing()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(args=sys.argv)
