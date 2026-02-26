#!/usr/bin/env python3
"""
Gripper action server for the SOA arm.

Uses pymoveit2 GripperInterface to control the gripper joint position.

Usage:
    ros2 run soa_actions gripper_server
"""

import time
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import GripperInterface

from soa_interfaces.action import Gripper
from soa_actions import soa_robot


class GripperServer(Node):

    def __init__(self):
        super().__init__('gripper_server')

        # Callback group for pymoveit2 (must be reentrant)
        self._cb_group = ReentrantCallbackGroup()

        # Initialize gripper interface
        self._gripper = GripperInterface(
            node=self,
            gripper_joint_names=soa_robot.gripper_joint_names(),
            open_gripper_joint_positions=soa_robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=soa_robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=soa_robot.MOVE_GROUP_GRIPPER,
            callback_group=self._cb_group,
            gripper_command_action_name='gripper_controller/gripper_cmd',
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper_command',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Gripper action server ready')

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received Gripper goal')

        target_position = goal_handle.request.target_position
        self.get_logger().info(f'Target gripper position: {target_position}')

        result = Gripper.Result()

        # Publish initial feedback
        feedback = Gripper.Feedback()
        feedback.current_position = target_position  # Will be updated after motion
        goal_handle.publish_feedback(feedback)

        # Move gripper to target position
        self._gripper.move_to_position(target_position)
        self._gripper.wait_until_executed()

        # Publish final feedback
        feedback.current_position = target_position
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.message = f'Gripper moved to position {target_position:.4f}'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)

    node = GripperServer()

    # Use MultiThreadedExecutor for pymoveit2 concurrent callbacks
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    # Wait for initialization
    time.sleep(1.0)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
