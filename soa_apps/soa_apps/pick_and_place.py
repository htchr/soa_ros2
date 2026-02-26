#!/usr/bin/env python3
"""
Pick-and-place demo using MoveToPose and Gripper action servers.

Sends a sequence of pose and gripper goals to pick up a small cube and
place it at a different location.

Prerequisites:
    # 1. Launch the MoveIt stack:
    ros2 launch soa_moveit_config soa_moveit_bringup.launch.py

    # 2. Start the action servers:
    ros2 run soa_actions move_to_pose_server
    ros2 run soa_actions gripper_server

    # 3. Run this demo:
    ros2 run soa_apps pick_and_place

Calling action servers from the command line:
    # Move to a pose:
    ros2 action send_goal /move_to_pose soa_interfaces/action/MoveToPose \\
      "{target_pose: {position: {x: 0.2, y: 0.0, z: 0.15}, \\
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" --feedback

    # Open the gripper:
    ros2 action send_goal /gripper_command soa_interfaces/action/Gripper \\
      "{target_position: 1.7453}" --feedback

    # Close the gripper:
    ros2 action send_goal /gripper_command soa_interfaces/action/Gripper \\
      "{target_position: -0.1745}" --feedback
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from soa_interfaces.action import MoveToPose, Gripper


# Gripper positions from SRDF
GRIPPER_OPEN = 1.7453
GRIPPER_CLOSED = -0.1745

# Default orientation: gripper pointing down (adjust for your setup)
DEFAULT_QUAT = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

# Pick-and-place poses (placeholder values — tune for your workspace)
# The SOA arm has roughly 0.1–0.3m reach depending on configuration
POSES = {
    'pre_pick':  Point(x=0.15, y=0.05, z=0.15),   # Above the cube
    'pick':      Point(x=0.15, y=0.05, z=0.05),    # At cube level
    'lift':      Point(x=0.15, y=0.05, z=0.15),    # Lift up after grasp
    'pre_place': Point(x=0.15, y=-0.05, z=0.15),   # Above the place location
    'place':     Point(x=0.15, y=-0.05, z=0.05),    # At place level
    'retreat':   Point(x=0.15, y=-0.05, z=0.15),   # Retreat after release
}


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        self._pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._gripper_client = ActionClient(self, Gripper, 'gripper_command')

    def send_pose_goal(self, position, orientation=None):
        """Send a MoveToPose goal and wait for the result."""
        if orientation is None:
            orientation = DEFAULT_QUAT

        goal = MoveToPose.Goal()
        goal.target_pose = Pose(position=position, orientation=orientation)

        self.get_logger().info(
            f'Sending pose goal: [{position.x:.3f}, {position.y:.3f}, {position.z:.3f}]'
        )

        self._pose_client.wait_for_server()
        future = self._pose_client.send_goal_async(
            goal, feedback_callback=self._pose_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pose goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Pose goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Pose goal failed: {result.message}')
        return result.success

    def send_gripper_goal(self, target_position):
        """Send a Gripper goal and wait for the result."""
        goal = Gripper.Goal()
        goal.target_position = target_position

        action_name = 'open' if target_position > 0.0 else 'close'
        self.get_logger().info(
            f'Sending gripper goal: {action_name} ({target_position:.4f})'
        )

        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(
            goal, feedback_callback=self._gripper_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Gripper goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Gripper goal failed: {result.message}')
        return result.success

    def _pose_feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(f'  Distance to goal: {distance:.4f}')

    def _gripper_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info(f'  Gripper position: {position:.4f}')

    def run(self):
        """Execute the pick-and-place sequence."""
        self.get_logger().info('=== Starting pick-and-place sequence ===')

        steps = [
            ('Open gripper',         lambda: self.send_gripper_goal(GRIPPER_OPEN)),
            ('Move above pick',      lambda: self.send_pose_goal(POSES['pre_pick'])),
            ('Move to pick',         lambda: self.send_pose_goal(POSES['pick'])),
            ('Close gripper',        lambda: self.send_gripper_goal(GRIPPER_CLOSED)),
            ('Lift up',              lambda: self.send_pose_goal(POSES['lift'])),
            ('Move above place',     lambda: self.send_pose_goal(POSES['pre_place'])),
            ('Move to place',        lambda: self.send_pose_goal(POSES['place'])),
            ('Open gripper',         lambda: self.send_gripper_goal(GRIPPER_OPEN)),
            ('Retreat',              lambda: self.send_pose_goal(POSES['retreat'])),
        ]

        for i, (name, action) in enumerate(steps, 1):
            self.get_logger().info(f'--- Step {i}/{len(steps)}: {name} ---')
            success = action()
            if not success:
                self.get_logger().error(
                    f'Step "{name}" failed. Aborting sequence.'
                )
                return

        self.get_logger().info('=== Pick-and-place sequence complete ===')


def main(args=None):
    rclpy.init(args=args)

    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
