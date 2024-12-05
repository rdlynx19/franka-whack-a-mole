"""Implements workflow for Franka to play the game."""
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

    async def play_game(self, request, response):
        