from __future__ import annotations
import logging
import time
import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose, SE2Pose
from synchros2.tf_listener_wrapper import TFListenerWrapper


class NavigationGoalChecker:
    def __init__(
        self,
        tf_listener_wrapper: TFListenerWrapper,
        close_to_goal_m=0.5,
        close_to_goal_rad=1.5708,
    ):
        self.close_to_goal_m = close_to_goal_m
        self.close_to_goal_rad = close_to_goal_rad
        self.tf_listener_wrapper = tf_listener_wrapper

    def is_goal_reached(
        self,
        goal: SE2Pose,
        frame_id=VISION_FRAME_NAME,
        distance_m=0.2,
        abs_angle_rad=0.3,
    ):
        curr_pose = self.tf_listener_wrapper.lookup_a_tform_b(frame_id, BODY_FRAME_NAME)
        if curr_pose is None:
            logging.info("Could not lookup transform.")
            return False
        curr_pose_se2 = SE3Pose(
            curr_pose.transform.translation.x,
            curr_pose.transform.translation.y,
            curr_pose.transform.translation.z,
            Quat(
                curr_pose.transform.rotation.w,
                curr_pose.transform.rotation.x,
                curr_pose.transform.rotation.y,
                curr_pose.transform.rotation.z,
            ),
        ).get_closest_se2_transform()

        distance_2d_m = self.euclidean_distance_2d_m(goal, curr_pose_se2)
        angle_error_rad = self.angle_difference_rad(goal.angle, curr_pose_se2.angle)

        distance_reached = distance_2d_m < distance_m
        angle_reached = angle_error_rad < abs_angle_rad
        result = distance_reached and angle_reached

        logging.info(f"Current Euclidean distance to target pose: {distance_2d_m} m")
        logging.info(
            f"Current absolute angular error from target pose: {angle_error_rad} rad"
        )
        logging.info(f"Ending navigation? {result}")

        return result

    def euclidean_distance_2d_m(self, pose_a: SE2Pose, pose_b: SE2Pose) -> float:
        """Compute the Euclidean distance (meters) between two poses on the 2D plane.

        :param pose_a: First 2D pose used to compute the distance
        :param pose_b: Second 2D pose used to compute the distance
        :return: Straight-line distance (meters) between the two 2D poses
        """
        return float(
            np.linalg.norm(np.array([pose_a.x - pose_b.x, pose_a.y - pose_b.y]))
        )

    def angle_difference_rad(self, a_rad: float, b_rad: float) -> float:
        """Compute the absolute difference (in normalized radians) between two angles.

        :param a_rad: First angle (radians) in the difference
        :param b_rad: Second angle (radians) in the difference
        :return: Absolute angle difference (radians, between 0 and pi)
        """
        difference_rad = self.normalize_angle(a_rad - b_rad)
        return abs(difference_rad)

    def normalize_angle(self, angle_rad: float) -> float:
        """Normalize the given angle (in radians) into the range [-pi, pi]."""
        while angle_rad < -np.pi:
            angle_rad += 2 * np.pi
        while angle_rad > np.pi:
            angle_rad -= 2 * np.pi
        return angle_rad
