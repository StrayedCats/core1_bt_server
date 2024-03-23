#!/usr/bin/env python3

from core1_bt_action.action import TargetPose as TD

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

class TargetDecision(Node):
    def __init__(self):
        super().__init__('target_decision')

        self._action_server = ActionServer(
            self,
            TD,
            'find_target_pose',
            self.execute_callback)
        
        # TODO: std::msgs empty

    def execute_callback(self, goal_handle):
        goal_handle.succeed()
        result = TD.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    target_decision = TargetDecision()
    rclpy.spin(target_decision)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
