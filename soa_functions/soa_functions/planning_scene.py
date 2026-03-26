#!/usr/bin/env python3
"""
ROS 2 node that adds a table collision object to the MoveIt planning scene.

The table is a flat box at base_link level, representing the surface the robot
is mounted on. This prevents MoveIt from planning paths through the table.

Usage:
    ros2 run soa_functions planning_scene
    ros2 run soa_functions planning_scene --ros-args \
        -p table_position:="[0.0, 0.0, -0.01]" \
        -p table_size:="[1.0, 1.0, 0.02]"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from soa_functions import soa_robot


def main():
    rclpy.init()

    node = Node("planning_scene")

    # Table parameters (position and size can be overridden at launch)
    node.declare_parameter("table_position", [0.0, 0.0, -0.01])
    node.declare_parameter("table_size", [1.0, 1.0, 0.02])

    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=soa_robot.joint_names(),
        base_link_name=soa_robot.base_link_name(),
        end_effector_name=soa_robot.end_effector_name(),
        group_name=soa_robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin executor in background thread
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    position = list(
        node.get_parameter("table_position").get_parameter_value().double_array_value
    )
    size = list(
        node.get_parameter("table_size").get_parameter_value().double_array_value
    )

    # Add table collision box
    node.get_logger().info(
        f"Adding table collision box: position={position}, size={size}"
    )
    moveit2.add_collision_box(
        id="table",
        position=position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        size=size,
    )
    node.get_logger().info("Table collision object added to planning scene.")

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
