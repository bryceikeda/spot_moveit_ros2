from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Tuple
import logging
import numpy as np
import rclpy
from rclpy.node import Node
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from geometry_msgs.msg import PoseStamped, TransformStamped

from synchros2.tf_listener_wrapper import TFListenerWrapper
from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)

APPROACH_DISTANCE = 1.0


class FiducialHandler(Node):
    """Handles fiducial detection and approach pose calculation."""

    def __init__(
        self,
        tf_listener_wrapper: TFListenerWrapper,
        default_approach_distance: float = 1.0,
    ):
        super().__init__("fiducial_handler")
        self.tf_listener_wrapper = tf_listener_wrapper
        self.tf_broadcaster = TransformBroadcaster(self)
        self.default_approach_distance = default_approach_distance

    def compute_pose_facing_fiducial(self, fiducial_name):
        self.tf_listener_wrapper.wait_for_a_tform_b(VISION_FRAME_NAME, fiducial_name)
        fiducial_pose = self.tf_listener_wrapper.lookup_a_tform_b(
            VISION_FRAME_NAME, fiducial_name
        )
        approach_distance = APPROACH_DISTANCE

        # Convert transform to SE3Pose
        fiducial_se3 = self._transform_to_se3(fiducial_pose)

        # Calculate offset position and heading
        goto_xy, heading = self._offset_tag_pose(fiducial_se3, approach_distance)

        # Build PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = VISION_FRAME_NAME
        pose.header.stamp = self.clock.now().to_msg()

        pose.pose.position.x = float(goto_xy[0])
        pose.pose.position.y = float(goto_xy[1])
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = float(heading.x)
        pose.pose.orientation.y = float(heading.y)
        pose.pose.orientation.z = float(heading.z)
        pose.pose.orientation.w = float(heading.w)

        return pose

    def publish_goal_transform(
        self,
        goal_position: np.ndarray,
        goal_heading: Quat,
        frame_id: str = "navigation_goal",
        parent_frame: str = VISION_FRAME_NAME,
    ):
        """
        Publish a goal pose to the TF tree for visualization.

        Args:
            goal_position: [x, y] position in parent frame
            goal_heading: Quaternion orientation
            frame_id: Name for the goal frame
            parent_frame: Parent frame for the transform
        """
        t = TransformStamped()

        t.header.stamp = self.clock.now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = frame_id

        t.transform.translation.x = float(goal_position[0])
        t.transform.translation.y = float(goal_position[1])
        t.transform.translation.z = 0.0

        t.transform.rotation.x = float(goal_heading.x)
        t.transform.rotation.y = float(goal_heading.y)
        t.transform.rotation.z = float(goal_heading.z)
        t.transform.rotation.w = float(goal_heading.w)

        self.tf_broadcaster.sendTransform(t)
        logger.info(f"Published goal transform to TF tree: {frame_id}")

    # ====================================================================
    #                         PRIVATE METHODS
    # ====================================================================

    def _transform_to_se3(self, transform: TransformStamped) -> SE3Pose:
        """Convert ROS TransformStamped to Boston Dynamics SE3Pose."""
        return SE3Pose(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            Quat(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
            ),
        )

    def _offset_tag_pose(
        self,
        fiducial_se3: SE3Pose,
        dist_margin: float,
    ) -> Tuple[np.ndarray, Quat]:
        """
        Calculate offset pose in front of a fiducial.

        Computes a navigation goal dist_margin meters along the fiducial's
        +Z axis (tag normal) projected onto the ground plane, with heading
        that faces toward the fiducial.

        Args:
            fiducial_se3: Fiducial pose in world frame
            dist_margin: Distance (meters) to offset from fiducial

        Returns:
            Tuple of (goal_xy_position, goal_heading)
        """
        # Fiducial +Z axis in world frame (tag normal), projected to XY plane
        approach_vec = np.array(fiducial_se3.rot.transform_point(0, 0, 1))[:2]
        norm = np.linalg.norm(approach_vec)

        if norm < 1e-6:
            # Degenerate case: tag is horizontal, default to +X approach
            approach_vec = np.array([1.0, 0.0])
            logger.warning(
                "Fiducial +Z axis is nearly vertical, using default approach direction"
            )
        else:
            approach_vec /= norm

        # Goal position: offset along approach vector
        fiducial_xy = np.array([fiducial_se3.x, fiducial_se3.y])
        goto_xy = fiducial_xy + approach_vec * dist_margin

        # Heading: face toward the fiducial from goal position
        to_fiducial = fiducial_xy - goto_xy
        norm = np.linalg.norm(to_fiducial)

        if norm >= 1e-6:
            heading = Quat.from_yaw(np.arctan2(to_fiducial[1], to_fiducial[0]))
        else:
            # Already at fiducial position (shouldn't happen with offset)
            heading = Quat()
            logger.warning("Goal position coincides with fiducial")

        return goto_xy, heading
