"""Integrate the hammer actuation with the MotionPlanner."""
import copy

from geometry_msgs.msg import Pose
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Empty
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

        # self.msg_publisher = self.create_publisher(String, 'test_writing', 1)
        # self.flag = 1


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
        pose1 = copy.deepcopy(object_pose)

        # Step 1: Move arm right above object
        pose1.position.z = object_pose.position.z + 0.2
        traj1 = await self.mpi.plan_path(goal_pose = pose1)
        _ = await self.mpi.exec_path(traj1)
        self.get_logger().info('Step 1: Finished moving arm above object')

        # Step 2: Open Grippers
        # await self.mpi.motion_planner.toggle_gripper('open')
        # self.get_logger().info('Step 2: Finished opening grippers')
         # Step: Actuating the hammer to raise!
        msg = String()
        msg.data = 'raise'
        actuate_msg = ActuateServo.Goal()
        actuate_msg.position = msg
        self.get_logger().info(f'{actuate_msg}')
        # _ = await self.hammer_client.send_goal_async(actuate_msg)
        self.get_logger().info(f'Actuating hammer to {msg.data}!')


        # Step 3: Move arm to object
        pose2 = object_pose
        traj3 = await self.mpi.plan_path(goal_pose = pose2)
        _ = await self.mpi.exec_path(traj3)
        self.get_logger().info('Step 3: Finished moving arm to object')

       # Step: Actuating the hammer to hit!
        msg.data = 'hit'
        actuate_msg.position = msg
        goal_handle = await self.hammer_client.send_goal_async(actuate_msg)
        self.get_logger().info(f'Actuating hammer to {msg.data}!')
        await goal_handle.get_result_async()
        

        # Step 4: Closing grippers
        # await self.mpi.motion_planner.toggle_gripper('close')
        # self.get_logger().info('Step 4: Finished closing grippers')

        # Step 5: Attaching box to arm in scene
        # await self.mpi.planning_scene.attach_object('box')
        # self.get_logger().info('Step 5: Finished attaching box to arm')

        # Step 6: Move arm up
        # self.get_logger().info('Start Moving Arm Up')
        # pose3 = object_pose
        # pose3.position.z = object_pose.position.z + 0.3
        # traj6 = await self.mpi.plan_path(goal_pose = pose3)
        # _ = await self.mpi.exec_path(traj6)
        # self.get_logger().info('Step 6: Finished moving arm up')

        
        # msg.data = 'raise'
        # self.get_logger().info(f'Actuating hammer to {msg.data}!')
        # self.msg_publisher.publish(msg)
        # Step 7: Move arm to other side of obstacle
        # pose4 = object_pose
        # pose4.position.y = object_pose.position.y + 0.3
        # _ = await self.mpi.plan_path(goal_pose=pose4)
        # self.get_logger().info('Step 7: Finished moving arm to other side of obstacle')

        # Step 8 Drop object
        # await self.mpi.motion_planner.toggle_gripper('open')
        # self.get_logger().info('Step 8: Finished dropping object')

        # Step 9: Detach object from arm
        # await self.mpi.planning_scene.detach_object('box')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Swing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    import sys
    main(args=sys.argv)