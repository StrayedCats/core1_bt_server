#!/usr/bin/env python3

from core1_bt_action.action import TargetPose as TD

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import numpy as np


class TargetDecision(Node):
    def __init__(self):
        super().__init__('target_decision')

        self.map_width = 5
        self.scale = 100

        self._action_server = ActionServer(
            self,
            TD,
            'find_target_pose',
            self.execute_callback)
        
        self.image_pub = self.create_publisher(Image, 'target_decisions', 10)

    def execute_callback(self, goal_handle):
        feedback_msg = TD.Feedback()
        result = TD.Result()

        ally_poses = goal_handle.request.allies.poses
        enemy_poses = goal_handle.request.enemies.poses
        distance_data = []

        if len(enemy_poses) == 0:
            result.target_pose = Pose()
        elif len(enemy_poses) == 1:
            result.target_pose = enemy_poses[0]
        else:
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
                result.target_pose = Pose()

        img = self.draw_poses(goal_handle.request.allies, goal_handle.request.enemies, result.target_pose)
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(image_message)

        goal_handle.succeed()
        return result

    def draw_poses(self, ally_poses, enemy_poses, target_pose=None) -> np.ndarray:
        img = np.zeros((self.map_width * self.scale, self.map_width * self.scale, 3), np.uint8)
        for pose in ally_poses.poses:
            print(pose.position.x, pose.position.y)
            x = int(pose.position.x * self.scale)
            y = int(pose.position.y * self.scale)
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

        for pose in enemy_poses.poses:
            print(pose.position.x, pose.position.y)
            x = int(pose.position.x * self.scale)
            y = int(pose.position.y * self.scale)
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, 'e', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        if (target_pose is not None) or (target_pose.position.x != 0.0 and target_pose.position.y != 0.0):
            x = int(target_pose.position.x * self.scale)
            y = int(target_pose.position.y * self.scale)
            cv2.rectangle(img, (x-10, y-10), (x+10, y+10), (255, 255, 0), 2)

        return img


def main(args=None):
    rclpy.init(args=args)
    target_decision = TargetDecision()
    rclpy.spin(target_decision)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
