#!/usr/bin/env python3

from core1_bt_action.action import TargetPose as TD

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import random

class TargetDecisionClient(Node):
    def __init__(self):
        super().__init__('target_decision_client')
        self._action_client = ActionClient(self, TD, 'find_target_pose')

        self.map_width = 5
        self.scale = 100

        self.ally_poses = PoseArray()
        self.enemy_poses = PoseArray()
        self.create_timer(1.0, self.timer_callback)

        

    def timer_callback(self):
        self.ally_poses.poses = []
        self.enemy_poses.poses = []
        for i in range(5):
            x = random.uniform(0.0, self.map_width)
            y = random.uniform(0.0, self.map_width)
            self.ally_poses.poses.append(self.create_pose(x, y))

        for i in range(5):
            x = random.uniform(0.0, self.map_width)
            y = random.uniform(0.0, self.map_width)
            self.enemy_poses.poses.append(self.create_pose(x, y))

        if self.ally_poses is not None and self.enemy_poses is not None:
            self.send_goal(self.ally_poses, self.enemy_poses)

    def create_pose(self, x, y):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        return pose

    def show_poses(self, ally_poses, enemy_poses, target_pose=None):
        img = np.zeros((self.map_width * self.scale, self.map_width * self.scale, 3), np.uint8)
        for pose in ally_poses.poses:
            x = int(pose.position.x * self.scale)
            y = int(pose.position.y * self.scale)
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

        for pose in enemy_poses.poses:
            x = int(pose.position.x * self.scale)
            y = int(pose.position.y * self.scale)
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, 'e', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        if target_pose is not None:
            x = int(target_pose.position.x * self.scale)
            y = int(target_pose.position.y * self.scale)
            cv2.rectangle(img, (x-10, y-10), (x+10, y+10), (255, 255, 0), 2)

        cv2.imshow('poses', img)
        cv2.waitKey(1)

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

        self.show_poses(self.ally_poses, self.enemy_poses, result.target_pose)


def main(args=None):
    rclpy.init(args=args)
    client = TargetDecisionClient()
    rclpy.spin(client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()