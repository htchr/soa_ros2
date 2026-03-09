"""Save pose service node.

Provides the ~/save_pose service (soa_interfaces/srv/SavePose) to capture
the current end-effector pose of the follower arm and optionally append it
to a CSV file for later replay with the move_to_pose action.

The pose is expressed as geometry_msgs/Pose (position XYZ in metres,
orientation as a unit quaternion XYZW) in the follower/base_link frame —
the same convention used by move_to_pose_server.py.

Launch in the /follower namespace (soa_bringup.launch.py handles this).

Services:
    /follower/save_pose (soa_interfaces/srv/SavePose)
        request:  csv_path — path to CSV file; if empty, pose is not saved
        response: success, pose

Parameters:
    tf_timeout (float, default 1.0)
        Seconds to wait for the TF transform before timing out.
"""

import csv
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Pose
from soa_interfaces.srv import SavePose

_CSV_HEADERS = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']


class SavePoseNode(Node):

    def __init__(self):
        super().__init__('save_pose')
        self.declare_parameter('tf_timeout', 1.0)

        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_service(
            SavePose,
            'save_pose',
            self._handle_save_pose,
            callback_group=self._cb_group,
        )

        self.get_logger().info('SavePose service ready.')

    def _handle_save_pose(self, req, res):
        timeout = self.get_parameter('tf_timeout').get_parameter_value().double_value

        # Look up end-effector pose relative to base frame
        try:
            transform = self._tf_buffer.lookup_transform(
                'follower/base_link',
                'follower/gripper_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            res.success = False
            return res

        # Convert TransformStamped → geometry_msgs/Pose
        t = transform.transform
        pose = Pose()
        pose.position.x = t.translation.x
        pose.position.y = t.translation.y
        pose.position.z = t.translation.z
        pose.orientation.x = t.rotation.x
        pose.orientation.y = t.rotation.y
        pose.orientation.z = t.rotation.z
        pose.orientation.w = t.rotation.w

        res.pose = pose
        res.success = True

        # Optionally save to CSV
        if req.csv_path:
            try:
                self._append_to_csv(req.csv_path, pose)
            except OSError as e:
                self.get_logger().error(f'Failed to write CSV: {e}')
                res.success = False

        return res

    def _append_to_csv(self, path: str, pose: Pose) -> None:
        file_exists = os.path.isfile(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=_CSV_HEADERS)
            if not file_exists:
                writer.writeheader()
            writer.writerow({
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
            })
        self.get_logger().info(f'Pose appended to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = SavePoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
