"""Save joint states service node.

Provides the /follower/save_joint_states service (soa_interfaces/srv/SaveJointStates)
to capture the current joint states of the follower arm and optionally append them
to a CSV file for later analysis or replay.

The joint state positions are recorded with column headers derived from the joint
names in the JointState message.

Can be run standalone without a namespace argument:
    ros2 run soa_services save_joint_states

Services:
    /follower/save_joint_states (soa_interfaces/srv/SaveJointStates)
        request:  csv_path — path to CSV file; if empty, joint states are not saved
        response: success, joint_states

Subscriptions:
    /follower/joint_states (sensor_msgs/JointState)
"""

import csv
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from soa_interfaces.srv import SaveJointStates


class SaveJointStatesNode(Node):

    def __init__(self):
        super().__init__('save_joint_states')

        self._latest_js: JointState | None = None
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState,
            '/follower/joint_states',
            self._on_joint_states,
            10,
            callback_group=self._cb_group,
        )

        self.create_service(
            SaveJointStates,
            '/follower/save_joint_states',
            self._handle_save_joint_states,
            callback_group=self._cb_group,
        )

        self.get_logger().info('SaveJointStates service ready.')

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_js = msg

    def _handle_save_joint_states(self, req, res):
        if self._latest_js is None:
            self.get_logger().warn('No joint states received yet.')
            res.success = False
            return res

        res.joint_states = self._latest_js
        res.success = True

        if req.csv_path:
            try:
                self._append_to_csv(req.csv_path, self._latest_js)
            except OSError as e:
                self.get_logger().error(f'Failed to write CSV: {e}')
                res.success = False

        return res

    def _append_to_csv(self, path: str, js: JointState) -> None:
        file_exists = os.path.isfile(path)
        headers = list(js.name)
        row = {name: pos for name, pos in zip(js.name, js.position)}
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(f'Joint states appended to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = SaveJointStatesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
