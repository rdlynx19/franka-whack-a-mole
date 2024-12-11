"""Integrate the hammer actuation with the MotionPlanner."""
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
    def __init__(self):
        super().__init__('swing')
        self.mpi = MotionPlanningInterface(self)
        self.serv = self.create_service(
            PickPose,
            'pick',
            self.pick_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.hammer_client = ActionClient(
            self,
            ActuateServo,
            'swing_hammer'
        )

    async def pick_callback(self, request: PickPose, response):
        """
        Trigger to pick object.

        :param request: service request object
        :type request: object_mover_interfaces.srv.PickPose
        :returns: response to service call
        :rtype: bool
        """
        object_pose = Pose()
        object_pose = request.pick_point

        # Step 1: Move arm to button
        pose1 = object_pose
        traj1 = await self.mpi.plan_path(goal_pose = pose1)
        _ = await self.mpi.exec_path(traj1)
        self.get_logger().info('Step 3: Finished moving arm to button')

       # Step 2: Actuating the hammer to hit!
        msg = String()
        actuate_msg = ActuateServo.Goal()
        msg.data = 'hit'
        actuate_msg.position = msg
        goal_handle = await self.hammer_client.send_goal_async(actuate_msg)
        self.get_logger().info(f'Actuating hammer to {msg.data}!')
        await goal_handle.get_result_async()
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Swing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    import sys
    main(args=sys.argv)