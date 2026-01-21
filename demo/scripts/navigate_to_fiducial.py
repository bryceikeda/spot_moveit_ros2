#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from spot_behavior_msgs.action import NavigateToFiducial

from synchros2.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    ODOM_FRAME_NAME,
    get_vision_tform_body,
)
from bosdyn.client.math_helpers import Quat, SE3Pose, SE2Pose
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from synchros2.tf_listener_wrapper import TFListenerWrapper


class NavigateToFiducialClient(Node):

    def __init__(self):
        super().__init__("navigate_to_fiducial_client")

        self._client = ActionClient(
            self, NavigateToFiducial, "/navigation/navigate_to_fiducial"
        )

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener_wrapper = TFListenerWrapper(self)

    def send_goal(self, fiducial_name: str):
        self.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        goal = NavigateToFiducial.Goal()
        goal.fiducial_name = fiducial_name

        self.get_logger().info(f"Sending goal: {fiducial_name}")

        # Send goal synchronously
        future_goal = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future_goal)
        goal_handle = future_goal.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return

        self.get_logger().info("Goal accepted.")

        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result()

        return result

    def navigate_to_fiducial(self, fiducial_name):
        fiducial_rt_world = self.tf_listener_wrapper.wait_for_a_tform_b(
            VISION_FRAME_NAME, fiducial_name
        )
        attempts_limit = 5
        current_attempts = 0
        fiducial_rt_world = None
        while current_attempts < attempts_limit:
            fiducial_rt_world = self.tf_listener_wrapper.lookup_a_tform_b(
                VISION_FRAME_NAME, fiducial_name
            )

            if fiducial_rt_world is not None:
                self.get_logger().info(f"Located fiducial {fiducial_rt_world}.")
                self.go_to_tag(fiducial_rt_world)
                return

            current_attempts += 1

        self.get_logger().warning(
            f"Failed to locate fiducial {fiducial_name} after {attempts_limit} attempts."
        )

    def publish_goal_transform(self, goto_xy, heading, frame_id="navigation_goal"):
        """Publish the goal position to the TF tree."""
        t = TransformStamped()

        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = VISION_FRAME_NAME
        t.child_frame_id = frame_id

        # Translation
        t.transform.translation.x = float(goto_xy[0])
        t.transform.translation.y = float(goto_xy[1])
        t.transform.translation.z = 0.0

        # Rotation (heading)
        t.transform.rotation.x = float(heading.x)
        t.transform.rotation.y = float(heading.y)
        t.transform.rotation.z = float(heading.z)
        t.transform.rotation.w = float(heading.w)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published goal transform to TF tree: {frame_id}")

    def go_to_tag(self, fiducial_rt_world):
        """Use the position of the april tag in vision world frame and command the robot."""
        # Compute the go-to point (offset by .5m) and heading at this point.

        fiducial_se3 = SE3Pose(
            fiducial_rt_world.transform.translation.x,
            fiducial_rt_world.transform.translation.y,
            fiducial_rt_world.transform.translation.z,
            Quat(
                fiducial_rt_world.transform.rotation.w,
                fiducial_rt_world.transform.rotation.x,
                fiducial_rt_world.transform.rotation.y,
                fiducial_rt_world.transform.rotation.z,
            ),
        )

        goto_xy, heading = self.offset_tag_pose(fiducial_se3, 1.0)

        # Publish the goal to TF tree
        self.publish_goal_transform(goto_xy, heading, "navigation_goal")

        pose = PoseStamped()
        pose.header.frame_id = VISION_FRAME_NAME  # usually "vision" or "world"

        pose.pose.position.x = float(goto_xy[0])
        pose.pose.position.y = float(goto_xy[1])
        pose.pose.position.z = 0.0

        # heading is a Quat object — convert to xyzw
        pose.pose.orientation.x = float(heading.x)
        pose.pose.orientation.y = float(heading.y)
        pose.pose.orientation.z = float(heading.z)
        pose.pose.orientation.w = float(heading.w)

        return

    def offset_tag_pose(self, object_rt_world: SE3Pose, dist_margin=1.0):
        """
        Compute a navigation goal dist_margin meters along the fiducial's +Z axis
        (projected into the ground plane), and a heading that faces the fiducial.
        """

        # Fiducial +Z axis in world frame (tag normal), projected to XY
        approach_vec = np.array(object_rt_world.rot.transform_point(0, 0, 1))[:2]
        norm = np.linalg.norm(approach_vec)
        if norm < 1e-6:
            approach_vec = np.array([1.0, 0.0])
        else:
            approach_vec /= norm

        # Goal position in XY
        goto_xy = (
            np.array([object_rt_world.x, object_rt_world.y])
            + approach_vec * dist_margin
        )

        # Heading: face the fiducial
        to_object = np.array([object_rt_world.x, object_rt_world.y]) - goto_xy
        norm = np.linalg.norm(to_object)
        heading = (
            Quat.from_yaw(np.arctan2(to_object[1], to_object[0]))
            if norm >= 1e-6
            else Quat()
        )

        return goto_xy, heading


import threading


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToFiducialClient()

    # ---- START SPINNING IMMEDIATELY ----
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # -----------------------------------

    fiducial_name = "filtered_fiducial_1"
    node.send_goal(fiducial_name)

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
