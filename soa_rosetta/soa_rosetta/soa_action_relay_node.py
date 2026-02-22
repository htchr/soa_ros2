#!/usr/bin/env python3
"""
Action Relay Node for SOA Robot.

This node splits 6-dimensional policy actions into separate commands for
the SOA robot's arm and gripper controllers.

Architecture:
  Input: /policy_actions (std_msgs/msg/Float64MultiArray)
    - 6 values: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]

  Outputs:
    - /arm_position_controller/commands (std_msgs/msg/Float64MultiArray)
      - 5 values: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
    - /gripper_controller/commands (std_msgs/msg/Float64MultiArray)
      - 1 value: [gripper]

This relay is necessary because soa_ros2 uses separate controllers for the arm
and gripper, but LeRobot policies output a single 6D action vector.

Usage:
    ros2 run soa_rosetta soa_action_relay_node

    # With custom topic names
    ros2 run soa_rosetta soa_action_relay_node --ros-args \\
        -p input_topic:=/custom/policy_actions \\
        -p arm_output_topic:=/custom/arm_commands \\
        -p gripper_output_topic:=/custom/gripper_commands
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray


class ActionRelayNode(Node):
    """Relay node to split policy actions for SOA arm and gripper controllers."""

    def __init__(self):
        """Initialize the action relay node."""
        super().__init__('action_relay')

        # Declare parameters
        self.declare_parameter('input_topic', '/policy_actions')
        self.declare_parameter('arm_output_topic', '/arm_position_controller/commands')
        self.declare_parameter('gripper_output_topic', '/gripper_controller/commands')
        self.declare_parameter('expected_action_dim', 6)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('debug', False)

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        arm_output_topic = self.get_parameter('arm_output_topic').value
        gripper_output_topic = self.get_parameter('gripper_output_topic').value
        self.expected_action_dim = self.get_parameter('expected_action_dim').value
        self.debug = self.get_parameter('debug').value

        # Joint labels for debug logging (matches soa_act_deployment.yaml action selector)
        self.joint_labels = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'gripper',
        ]

        # Configure QoS
        qos_reliability_str = self.get_parameter('qos_reliability').value
        reliability = (
            ReliabilityPolicy.RELIABLE
            if qos_reliability_str == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        qos_profile = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            input_topic,
            self.action_callback,
            qos_profile
        )

        # Create publishers
        self.arm_publisher = self.create_publisher(
            Float64MultiArray,
            arm_output_topic,
            qos_profile
        )

        self.gripper_publisher = self.create_publisher(
            Float64MultiArray,
            gripper_output_topic,
            qos_profile
        )

        # Statistics
        self.message_count = 0
        self.error_count = 0

        self.get_logger().info(
            f'Action Relay Node started\n'
            f'  Input: {input_topic}\n'
            f'  Arm output: {arm_output_topic}\n'
            f'  Gripper output: {gripper_output_topic}\n'
            f'  Expected action dimension: {self.expected_action_dim}'
        )

    def action_callback(self, msg: Float64MultiArray):
        """
        Process incoming policy actions and split into arm and gripper commands.

        Args:
            msg: Policy action array (6D: 5 arm joints + 1 gripper)
        """
        self.message_count += 1

        # Validate action dimension
        if len(msg.data) != self.expected_action_dim:
            self.error_count += 1
            self.get_logger().warn(
                f'Unexpected action dimension: got {len(msg.data)}, '
                f'expected {self.expected_action_dim}. Skipping message.',
                throttle_duration_sec=1.0  # Throttle warnings to 1Hz
            )
            return

        # Split actions
        # First 5 dimensions: arm joints (shoulder_pan through wrist_roll)
        arm_actions = Float64MultiArray()
        arm_actions.data = list(msg.data[:5])

        # Last dimension: gripper
        gripper_actions = Float64MultiArray()
        gripper_actions.data = [msg.data[5]]

        # Publish split commands
        self.arm_publisher.publish(arm_actions)
        self.gripper_publisher.publish(gripper_actions)

        # Debug logging: show labeled values for every action
        if self.debug:
            parts = []
            for i, val in enumerate(msg.data):
                label = self.joint_labels[i] if i < len(self.joint_labels) else f'dim_{i}'
                parts.append(f'{label}={val:.4f}')
            self.get_logger().info(
                f'Action [{self.message_count}]: {", ".join(parts)} | '
                f'arm={[f"{v:.4f}" for v in arm_actions.data]} '
                f'gripper={gripper_actions.data}',
                throttle_duration_sec=0.1
            )

        # Log periodically
        if self.message_count % 100 == 0:
            self.get_logger().info(
                f'Relayed {self.message_count} actions '
                f'({self.error_count} errors)',
                throttle_duration_sec=5.0
            )


def main(args=None):
    """Run the action relay node."""
    rclpy.init(args=args)
    node = ActionRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down action relay node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
