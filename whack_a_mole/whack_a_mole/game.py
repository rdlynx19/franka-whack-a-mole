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
        self.static = StaticTransformBroadcaster(self)
        self.base_to_tag0 = TransformStamped()
        self.base_to_tag0.header.stamp = self.get_clock().now().to_msg()
        self.base_to_tag0.header.frame_id = 'base_tag'
        self.base_to_tag0.child_frame_id = 'base'
        self.get_logger().info('Setting up the tag0 transform')
        self.base_to_tag0.transform.translation.x = 0.0
        self.base_to_tag0.transform.translation.y = 0.159
        self.base_to_tag0.transform.translation.z = -0.00194
        self.base_to_tag0.transform.rotation.z = -math.pi/3

        self.static.sendTransform(self.base_to_tag0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    async def play_game(self, request, response):
        try:
            self.base_to_cam = self.buffer.lookup_transform('base', 'camera_color_frame', rclpy.time.Time())
            self.base_to_tag1 = self.buffer.lookup_transform('base', 'board', rclpy.time.Time())
            goal_pose = Pose()
            goal_pose.position.x = self.base_to_tag1.transform.translation.x
            goal_pose.position.y = self.base_to_tag1.transform.translation.y
            goal_pose.position.z = 0.1
            goal_pose.orientation.x = 1.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 0.0

            traj1 = await self.mpi.plan_path(
                goal_pose=goal_pose,
            )
            _ = await self.mpi.exec_path(traj1)

        except tf2_ros.LookupException as e:
            self.get_logger().info(f'LookupException: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().info(f'ConnectivityException: {e}')   
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().info(f'ExtrapolationException: {e}')
        finally:
            self.get_logger().info('in finally')
            
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