#!/usr/bin/env python3

from core1_bt_action.action import TargetPose as TD

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import numpy as np


class TargetDecision(Node):
    def __init__(self):
        super().__init__('target_decision')
        self._action_server = ActionServer(
            self,
            TD,
            'find_target_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = TD.Feedback()
        result = TD.Result()

        ally_poses = goal_handle.request.allies.poses
        enemy_poses = goal_handle.request.enemies.poses
        distance_data = []

        for ally_pose in ally_poses:
            for enemy_pose in enemy_poses:
                distance = np.sqrt((ally_pose.position.x - enemy_pose.position.x) ** 2 +
                                   (ally_pose.position.y - enemy_pose.position.y) ** 2 +
                                   (ally_pose.position.z - enemy_pose.position.z) ** 2)
                feedback_msg.current_calculated_distance = distance
                goal_handle.publish_feedback(feedback_msg)
                temp_data = [ally_pose, enemy_pose, distance]
                distance_data.append(temp_data)
        distance_data = sorted(distance_data, key=lambda x: x[2])
        self.get_logger().info(f'Distance data: {distance_data}')

        if distance_data:
            result.target_pose = distance_data[0][1]
        else:
            result.target_pose = None

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    target_decision = TargetDecision()
    rclpy.spin(target_decision)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
