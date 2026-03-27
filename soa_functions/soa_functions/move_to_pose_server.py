#!/usr/bin/env python3
"""
MoveToPose action server for the SOA 5-DOF arm.

Uses pymoveit2 to plan and execute IK-based motion to a target pose.
Implements a fallback strategy for the 5-DOF arm:
  1. Attempt full pose (position + orientation)
  2. Fall back to position-only IK if full pose planning fails

Usage:
    ros2 run soa_functions move_to_pose_server
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
from soa_functions import soa_robot


class MoveToPoseServer(Node):

    def __init__(self):
        super().__init__('move_to_pose_server')

        # Declare configurable parameters
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('tolerance_position', 0.01)
        self.declare_parameter('tolerance_orientation', 0.1)
        self.declare_parameter('tolerance_orientation_relaxed', 0.5)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('allowed_planning_time', 3.0)
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

    def _wait_and_publish_feedback(self, goal_handle, target_position):
        """Wait for MoveIt2 execution, publishing feedback each iteration."""
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            self._publish_feedback(goal_handle, target_position)
            time.sleep(0.1)
        # Publish one final feedback with the settled position
        self._publish_feedback(goal_handle, target_position)
        return self._moveit2.motion_suceeded

    def _plan_and_execute(self, goal_handle, position, quat_xyzw=None,
                          tol_pos=0.01, tol_orient=0.1,
                          planning_time=None) -> bool:
        """Plan and execute a single motion attempt. Returns True on success."""
        base_time = (
            self.get_parameter('allowed_planning_time')
            .get_parameter_value().double_value
        )
        self._moveit2.allowed_planning_time = (
            planning_time if planning_time is not None else base_time
        )
        self._moveit2.clear_goal_constraints()

        kwargs = dict(
            position=position,
            tolerance_position=tol_pos,
            start_joint_state=self._moveit2.joint_state,
        )
        if quat_xyzw is not None:
            kwargs['quat_xyzw'] = quat_xyzw
            kwargs['tolerance_orientation'] = tol_orient

        future = self._moveit2.plan_async(**kwargs)
        if future is None:
            return False

        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            return False

        self._moveit2.execute(trajectory)
        return self._wait_and_publish_feedback(goal_handle, position)

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
        tol_orient_relaxed = (
            self.get_parameter('tolerance_orientation_relaxed')
            .get_parameter_value().double_value
        )
        planning_time = (
            self.get_parameter('allowed_planning_time')
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

        # --- 4-attempt tolerance cascade ---
        attempts = [
            # (label, quat_xyzw, tol_pos, tol_orient, planning_time)
            ('Attempt 1: full pose (tight)',
             quat_xyzw, tol_pos, tol_orient, planning_time),
            ('Attempt 2: full pose (relaxed orientation)',
             quat_xyzw, tol_pos, tol_orient_relaxed, planning_time),
            ('Attempt 3: position-only',
             None, tol_pos * 2, None, planning_time + 2.0),
        ]

        success_messages = [
            'Reached target: full pose (tight)',
            'Reached target: full pose (relaxed orientation)',
            'Reached target: position-only IK (orientation ignored)',
        ]

        for i, (label, quat, tp, to, pt) in enumerate(attempts):
            self.get_logger().info(label)
            success = self._plan_and_execute(
                goal_handle=goal_handle,
                position=position,
                quat_xyzw=quat,
                tol_pos=tp,
                tol_orient=to if to is not None else tol_orient,
                planning_time=pt,
            )
            if success:
                goal_handle.succeed()
                result.success = True
                result.message = success_messages[i]
                self.get_logger().info(result.message)
                return result
            self.get_logger().warn(f'{label} — failed')

        goal_handle.abort()
        result.success = False
        result.message = 'Planning failed for all 3 attempts'
        self.get_logger().error(result.message)
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
