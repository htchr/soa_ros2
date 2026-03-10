#!/usr/bin/env python3
"""
MoveToJointStates action server for the SOA 5-DOF arm.

Uses pymoveit2 to plan and execute joint-space motion to a target configuration.
Joint names may be specified in the goal; if omitted, all 5 arm joints are assumed
in order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Usage:
    ros2 run soa_actions move_to_joint_states_server
"""

import time
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State

from soa_interfaces.action import MoveToJointStates
from soa_actions import soa_robot


# Joint limits from soa_description/urdf/soa_macro.xacro
_JOINT_LIMITS = {
    'shoulder_pan':  (-1.92, 1.92),
    'shoulder_lift': (-1.75, 1.75),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.66, 1.66),
    'wrist_roll':    (-2.74, 2.84),
}


class MoveToJointStatesServer(Node):

    def __init__(self):
        super().__init__('move_to_joint_states_server')

        # Declare configurable parameters
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('allowed_planning_time', 3.0)

        # Callback group for pymoveit2 (must be reentrant)
        self._cb_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=soa_robot.joint_names(),
            base_link_name=soa_robot.base_link_name(),
            end_effector_name=soa_robot.end_effector_name(),
            group_name=soa_robot.MOVE_GROUP_ARM,
            callback_group=self._cb_group,
        )

        # Apply velocity/acceleration scaling
        self._moveit2.max_velocity = (
            self.get_parameter('max_velocity').get_parameter_value().double_value
        )
        self._moveit2.max_acceleration = (
            self.get_parameter('max_acceleration').get_parameter_value().double_value
        )

        # Increase planning budget
        self._moveit2.num_planning_attempts = (
            self.get_parameter('num_planning_attempts')
            .get_parameter_value().integer_value
        )
        self._moveit2.allowed_planning_time = (
            self.get_parameter('allowed_planning_time')
            .get_parameter_value().double_value
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            MoveToJointStates,
            'move_to_joint_states',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('MoveToJointStates action server ready')

    def _wait_until_executed(self):
        """Wait for MoveIt2 execution without rclpy.spin_once() conflict."""
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            time.sleep(0.1)
        return self._moveit2.motion_suceeded

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received MoveToJointStates goal')

        joint_positions = list(goal_handle.request.joint_positions)
        joint_names = list(goal_handle.request.joint_names)

        # Resolve default joint names
        if not joint_names:
            joint_names = soa_robot.joint_names()

        result = MoveToJointStates.Result()

        # --- Validation: length ---
        if len(joint_positions) != len(joint_names):
            goal_handle.abort()
            result.success = False
            result.message = (
                f'Length mismatch: {len(joint_positions)} positions for '
                f'{len(joint_names)} joints ({joint_names})'
            )
            self.get_logger().warn(result.message)
            return result

        # --- Validation: joint limits ---
        for name, pos in zip(joint_names, joint_positions):
            if name not in _JOINT_LIMITS:
                goal_handle.abort()
                result.success = False
                result.message = f'Unknown joint: {name}'
                self.get_logger().warn(result.message)
                return result
            lo, hi = _JOINT_LIMITS[name]
            if not (lo <= pos <= hi):
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Joint {name} position {pos:.4f} rad out of limits '
                    f'[{lo}, {hi}]'
                )
                self.get_logger().warn(result.message)
                return result

        self.get_logger().info(
            f'Target joints: {joint_names}, positions: {joint_positions}'
        )

        # --- Plan in joint space ---
        planning_time = (
            self.get_parameter('allowed_planning_time')
            .get_parameter_value().double_value
        )
        self._moveit2.allowed_planning_time = planning_time
        self._moveit2.clear_goal_constraints()

        future = self._moveit2.plan_async(
            joint_positions=joint_positions,
            joint_names=joint_names,
            start_joint_state=self._moveit2.joint_state,
        )

        if future is None:
            goal_handle.abort()
            result.success = False
            result.message = 'plan_async() returned None'
            self.get_logger().error(result.message)
            return result

        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed: no trajectory found'
            self.get_logger().error(result.message)
            return result

        # --- Execute ---
        self._moveit2.execute(trajectory)
        success = self._wait_until_executed()

        self._publish_feedback(goal_handle, joint_names, joint_positions)

        if success:
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached target joint configuration'
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Execution failed'
            self.get_logger().error(result.message)

        return result

    def _publish_feedback(self, goal_handle, joint_names, target_positions):
        """Publish max absolute joint error between current and target positions."""
        feedback = MoveToJointStates.Feedback()
        try:
            js = self._moveit2.joint_state
            if js is not None and js.name:
                name_to_pos = dict(zip(js.name, js.position))
                errors = [
                    abs(name_to_pos[n] - t)
                    for n, t in zip(joint_names, target_positions)
                    if n in name_to_pos
                ]
                feedback.max_joint_error = max(errors) if errors else -1.0
            else:
                feedback.max_joint_error = -1.0
        except Exception:
            feedback.max_joint_error = -1.0
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToJointStatesServer()

    # Use MultiThreadedExecutor for pymoveit2 concurrent callbacks
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    # Wait for MoveIt2 initialization
    time.sleep(1.0)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
