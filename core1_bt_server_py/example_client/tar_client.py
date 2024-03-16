#!/usr/bin/env python3

from core1_bt_action.action import TargetPose as TD

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray


class TargetDecisionClient(Node):
    def __init__(self):
        super().__init__('target_decision_client')
        self._action_client = ActionClient(self,
                                           TD,
                                           'find_target_pose')
        self.ally_subscription = self.create_subscription(PoseArray,
                                                          'ally_poses',
                                                          self.ally_callback,
                                                          10)
        self.enemy_subscription = self.create_subscription(PoseArray,
                                                           'enemy_poses',
                                                           self.enemy_callback,
                                                           10)

        self.ally_poses = None
        self.enemy_poses = None

    def ally_callback(self, msg):
        self.ally_poses = msg
        self.send_goal_if_ready()

    def enemy_callback(self, msg):
        self.enemy_poses = msg
        self.send_goal_if_ready()

    def send_goal_if_ready(self):
        if self.ally_poses is not None and self.enemy_poses is not None:
            self.send_goal(self.ally_poses, self.enemy_poses)
            # リクエスト送信後、次のリクエストが新しいデータに基づくようにポーズをリセット
            self.ally_poses = None
            self.enemy_poses = None

    def send_goal(self, ally_poses, enemy_poses):
        goal_msg = TD.Goal()
        goal_msg.allies = ally_poses
        goal_msg.enemies = enemy_poses

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Data rejected...')
            return

        self.get_logger().info('Data accepted!!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Received result: Closest enemy distance is {result.target_pose.position}')


def main(args=None):
    rclpy.init(args=args)
    client = TargetDecisionClient()
    rclpy.spin(client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()