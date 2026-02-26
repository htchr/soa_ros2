#!/usr/bin/env python3
"""
MoveToPose action server for the SOA 5-DOF arm.

Uses pymoveit2 to plan and execute IK-based motion to a target pose.
Implements a fallback strategy for the 5-DOF arm:
  1. Attempt full pose (position + orientation)
  2. Fall back to position-only IK if full pose planning fails

Usage:
    ros2 run soa_actions move_to_pose_server
"""

import math
import time
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State

from soa_interfaces.action import MoveToPose
from soa_actions import soa_robot


class MoveToPoseServer(Node):

    def __init__(self):
        super().__init__('move_to_pose_server')

        # Declare configurable parameters
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('tolerance_position', 0.01)
        self.declare_parameter('tolerance_orientation', 0.1)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('max_reach', 0.4)

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

        # Increase planning budget (pymoveit2 defaults: 0.5s / 5 attempts)
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
            MoveToPose,
            'move_to_pose',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('MoveToPose action server ready')

    def _wait_until_executed(self):
        """Wait for MoveIt2 execution without rclpy.spin_once() conflict."""
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            time.sleep(0.1)
        return self._moveit2.motion_suceeded

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received MoveToPose goal')

        target = goal_handle.request.target_pose
        position = [target.position.x, target.position.y, target.position.z]
        quat_xyzw = [
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ]

        tol_pos = (
            self.get_parameter('tolerance_position')
            .get_parameter_value().double_value
        )
        tol_orient = (
            self.get_parameter('tolerance_orientation')
            .get_parameter_value().double_value
        )

        self.get_logger().info(
            f'Target position: {position}, orientation: {quat_xyzw}'
        )

        result = MoveToPose.Result()

        # --- Pre-flight validation ---
        max_reach = (
            self.get_parameter('max_reach')
            .get_parameter_value().double_value
        )
        dist = math.sqrt(sum(p ** 2 for p in position))
        if dist > max_reach:
            goal_handle.abort()
            result.success = False
            result.message = (
                f'Target distance {dist:.3f}m exceeds max reach {max_reach}m'
            )
            self.get_logger().warn(result.message)
            return result

        quat_norm = math.sqrt(sum(q ** 2 for q in quat_xyzw))
        if abs(quat_norm - 1.0) > 0.01:
            goal_handle.abort()
            result.success = False
            result.message = (
                f'Invalid quaternion (norm={quat_norm:.4f}, expected 1.0)'
            )
            self.get_logger().warn(result.message)
            return result

        # --- Attempt 1: Full pose (position + orientation) ---
        # Use plan_async() + execute() to avoid rclpy.spin_once() inside
        # move_to_pose()/plan(), which conflicts with MultiThreadedExecutor.
        self.get_logger().info('Attempt 1: Full pose target')
        future = self._moveit2.plan_async(
            position=position,
            quat_xyzw=quat_xyzw,
            tolerance_position=tol_pos,
            tolerance_orientation=tol_orient,
            start_joint_state=self._moveit2.joint_state,
        )

        if future is not None:
            while not future.done():
                time.sleep(0.1)
            trajectory = self._moveit2.get_trajectory(future)
        else:
            trajectory = None

        if trajectory is not None:
            self._moveit2.execute(trajectory)
            success = self._wait_until_executed()
        else:
            success = False

        if success:
            self._publish_feedback(goal_handle, position)
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached target with full pose'
            self.get_logger().info(result.message)
            return result

        # --- Attempt 2: Position-only fallback (5-DOF limitation) ---
        self.get_logger().info(
            'Full pose failed, Attempt 2: Position-only fallback'
        )
        # Use plan_async() instead of plan() to avoid rclpy.spin_once()
        self._moveit2.clear_goal_constraints()
        future = self._moveit2.plan_async(
            position=position,
            tolerance_position=2.0 * tol_pos,
            start_joint_state=self._moveit2.joint_state,
        )

        if future is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed for both full pose and position-only'
            self.get_logger().warn(result.message)
            return result

        # Wait for planning without rclpy.spin_once()
        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed for both full pose and position-only'
            self.get_logger().warn(result.message)
            return result

        self._moveit2.execute(trajectory)
        success = self._wait_until_executed()

        if success and self._moveit2.motion_suceeded:
            self._publish_feedback(goal_handle, position)
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached target with position-only IK (orientation ignored)'
            self.get_logger().info(result.message)
            return result

        # Both attempts failed
        goal_handle.abort()
        result.success = False
        result.message = 'Execution failed after position-only planning'
        self.get_logger().warn(result.message)
        return result

    def _publish_feedback(self, goal_handle, target_position):
        """Publish distance feedback from current EE pose to target."""
        feedback = MoveToPose.Feedback()
        try:
            # Use compute_fk_async() to avoid rclpy.spin_once() in compute_fk()
            fk_future = self._moveit2.compute_fk_async()
            if fk_future is not None:
                while not fk_future.done():
                    time.sleep(0.1)
                fk_result = self._moveit2.get_compute_fk_result(fk_future)
            else:
                fk_result = None

            if fk_result is not None:
                current = fk_result.pose.position
                dx = current.x - target_position[0]
                dy = current.y - target_position[1]
                dz = current.z - target_position[2]
                feedback.distance_to_goal = math.sqrt(dx**2 + dy**2 + dz**2)
            else:
                feedback.distance_to_goal = -1.0
        except Exception:
            feedback.distance_to_goal = -1.0
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseServer()

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
