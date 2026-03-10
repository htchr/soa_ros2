#!/usr/bin/env python3
"""
Execute a sequence of end-effector poses from a CSV file, with gripper commands
interspersed between poses.

The SEQUENCE list defines the full execution plan as an ordered list of Steps.
Each Step is either a 'pose' (referenced by CSV index) or a 'gripper' command.
To modify the manipulation task, edit SEQUENCE — the execution engine stays the same.

CSV format: x,y,z,qx,qy,qz,qw (one pose per row, with header)

Prerequisites:
    # 1. Launch the MoveIt stack:
    ros2 launch soa_moveit_config soa_moveit_bringup.launch.py

    # 2. Start the action servers:
    ros2 run soa_actions move_to_pose_server
    ros2 run soa_actions gripper_server

    # 3. Run this node (default CSV path):
    ros2 run soa_apps go_to_poses

    # Or with a custom CSV path:
    ros2 run soa_apps go_to_poses --ros-args -p csv_path:=/path/to/poses.csv
"""

import csv
from dataclasses import dataclass, field

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion
from soa_interfaces.action import Gripper, MoveToPose


# Gripper joint positions (from SRDF / soa_robot.py)
GRIPPER_OPEN = 1.7453
GRIPPER_CLOSED = 0.1   

DEFAULT_CSV_PATH = '/home/ubuntu/techin517/soa_ws/poses.csv'


@dataclass
class Step:
    """A single step in the manipulation sequence.

    type: 'pose' or 'gripper'
    pose_index: index into the loaded CSV poses list (used when type == 'pose')
    gripper_pos: target gripper joint position (used when type == 'gripper')
    label: human-readable description shown in log output
    """

    type: str
    pose_index: int = 0
    gripper_pos: float = 0.0
    label: str = ''


# ---------------------------------------------------------------------------
# Sequence definition — edit this list to change the manipulation task.
# Poses are referenced by their 0-based row index in the CSV file.
# ---------------------------------------------------------------------------
SEQUENCE = [
    Step('pose',    pose_index=0, label='pose 0 (approach)'),
    Step('pose',    pose_index=1, label='pose 1 (pre-grasp)'),
    Step('gripper', gripper_pos=GRIPPER_OPEN,   label='open gripper'),
    Step('pose',    pose_index=2, label='pose 2 (at object)'),
    Step('gripper', gripper_pos=GRIPPER_CLOSED, label='close gripper'),
    Step('pose',    pose_index=3, label='pose 3 (lift / place)'),
]


def load_poses(path: str) -> list:
    """Parse CSV file and return a list of geometry_msgs/Pose."""
    poses = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            poses.append(Pose(
                position=Point(
                    x=float(row['x']),
                    y=float(row['y']),
                    z=float(row['z']),
                ),
                orientation=Quaternion(
                    x=float(row['qx']),
                    y=float(row['qy']),
                    z=float(row['qz']),
                    w=float(row['qw']),
                ),
            ))
    return poses


class GoToPoses(Node):

    def __init__(self):
        super().__init__('go_to_poses')

        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)

        self._pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._gripper_client = ActionClient(self, Gripper, 'gripper_command')

    def send_pose_goal(self, pose: Pose, label: str = '') -> bool:
        """Send a MoveToPose goal and block until the result is received."""
        goal = MoveToPose.Goal()
        goal.target_pose = pose

        self.get_logger().info(
            f'Sending pose goal ({label}): '
            f'[{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}]'
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

    def send_gripper_goal(self, target_position: float, label: str = '') -> bool:
        """Send a Gripper goal and block until the result is received."""
        goal = Gripper.Goal()
        goal.target_position = target_position

        self.get_logger().info(
            f'Sending gripper goal ({label}): position={target_position:.4f}'
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
        """Load poses from CSV and execute the SEQUENCE."""
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading poses from: {csv_path}')

        poses = load_poses(csv_path)
        self.get_logger().info(f'Loaded {len(poses)} pose(s)')

        self.get_logger().info('=== Starting go_to_poses sequence ===')

        for i, step in enumerate(SEQUENCE, 1):
            self.get_logger().info(
                f'--- Step {i}/{len(SEQUENCE)}: {step.label} ---'
            )

            if step.type == 'pose':
                if step.pose_index >= len(poses):
                    self.get_logger().error(
                        f'Pose index {step.pose_index} out of range '
                        f'(CSV has {len(poses)} poses). Aborting.'
                    )
                    return
                success = self.send_pose_goal(poses[step.pose_index], step.label)

            elif step.type == 'gripper':
                success = self.send_gripper_goal(step.gripper_pos, step.label)

            else:
                self.get_logger().error(f'Unknown step type: {step.type!r}. Aborting.')
                return

            if not success:
                self.get_logger().error(
                    f'Step "{step.label}" failed. Aborting sequence.'
                )
                return

        self.get_logger().info('=== go_to_poses sequence complete ===')


def main(args=None):
    rclpy.init(args=args)

    node = GoToPoses()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
