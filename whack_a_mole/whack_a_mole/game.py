"""Implements workflow for Franka to play the game."""
from geometry_msgs.msg import Pose, TransformStamped
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
import rclpy
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
from whack_a_mole_interfaces.action import ActuateServo
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Game(Node):
    def __init__(self):
        super().__init__('game')
        self.mpi = MotionPlanningInterface(self)
        self.serv = self.create_service(
            Empty,
            'play',
            self.play_game, #Callback for play service
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
        self.tag0_to_base.transform.translation.x = 0.155
        self.tag0_to_base.transform.translation.y = -0.085
        self.tag0_to_base.transform.translation.z = 0.0
        q = [ -0.5, -0.5, -0.5, 0.5 ]
        self.tag0_to_base.transform.rotation.x = q[0]
        self.tag0_to_base.transform.rotation.y = q[1]
        self.tag0_to_base.transform.rotation.z = q[2]
        self.tag0_to_base.transform.rotation.w = q[3]
        self.static.sendTransform(self.tag0_to_base)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


    async def play_game(self, request, response):
        try:
            self.base_to_cam = self.buffer.lookup_transform('base', 'camera_color_frame', rclpy.time.Time())
            self.base_to_tag1 = self.buffer.lookup_transform('base', 'board', rclpy.time.Time())
            self.base_to_tag2 = self.buffer.lookup_transform('base', 'mole', rclpy.time.Time())

            goal_pose = Pose()
            goal_pose.position.x = self.base_to_tag1.transform.translation.x - 0.17
            goal_pose.position.y = self.base_to_tag1.transform.translation.y 
            goal_pose.position.z = 0.15
            rotation = [3.14, 3.14, 0.0, 0.0]
            goal_pose.orientation.x = rotation[0]
            goal_pose.orientation.y = rotation[1]
            goal_pose.orientation.z = rotation[2]
            goal_pose.orientation.w = rotation[3]

            self.get_logger().info(f'Goal pose: {goal_pose}')

            # Wait for the service to be ready
            self.get_logger().info("Waiting for 'pick' service...")
            if not self.pick_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("'pick' service not available")
                return response
            self.get_logger().info("'pick' service is available")

            # Create and send request
            request = PickPose.Request()
            request.pick_point.position.x = goal_pose.position.x
            request.pick_point.position.y = goal_pose.position.y
            request.pick_point.position.z = goal_pose.position.z
            request.pick_point.orientation.x = goal_pose.orientation.x
            request.pick_point.orientation.y = goal_pose.orientation.y
            request.pick_point.orientation.z = goal_pose.orientation.z
            request.pick_point.orientation.w = goal_pose.orientation.w

            self.get_logger().info(f'Sending pick point: {request}')
            _ = await self.pick_client.call_async(request)
            self.get_logger().info(f'Pick service responded: {response}')
            
            new_goal_pose = Pose()
            new_goal_pose.position.x = self.base_to_tag2.transform.translation.x - 0.17
            new_goal_pose.position.y = self.base_to_tag2.transform.translation.y
            new_goal_pose.position.z = 0.15
            new_goal_pose.orientation.x = rotation[0]
            new_goal_pose.orientation.y = rotation[1]
            new_goal_pose.orientation.z = rotation[2]
            new_goal_pose.orientation.w = rotation[3]
            
            self.get_logger().info(f'Goal pose: {new_goal_pose}')
            
            # Create and send request
            new_request = PickPose.Request()
            new_request.pick_point.position.x = new_goal_pose.position.x
            new_request.pick_point.position.y = new_goal_pose.position.y
            new_request.pick_point.position.z = new_goal_pose.position.z
            new_request.pick_point.orientation.x = new_goal_pose.orientation.x
            new_request.pick_point.orientation.y = new_goal_pose.orientation.y
            new_request.pick_point.orientation.z = new_goal_pose.orientation.z
            new_request.pick_point.orientation.w = new_goal_pose.orientation.w
            
            self.get_logger().info(f'Sending pick point: {new_request}')
            _ = await self.pick_client.call_async(new_request)
            self.get_logger().info(f'Pick service responded: {response}')
            
            
        except Exception as e:
            self.get_logger().error(f'Error in play_game: {e}')
        return response
            
    async def go_home_callback(self, request, response):
        try:
            await self.mpi.go_home_callback()
        except Exception as e:
            self.get_logger().error(f'Error in go_home_callback: {e}')
        return response

def main(args=None):
    rclpy.init(args=args)
    game = Game()
    rclpy.spin(game)
    game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)