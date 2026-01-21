from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from spot_behavior_msgs.action import DetectAprilTags
from spot_behavior_msgs.msg import AprilTagDetection

from geometry_msgs.msg import Pose, PoseStamped

if TYPE_CHECKING:
    from spot_core.perception.april_tag_tracker import AprilTagTracker


class PerceptionActionServer(Node):
    """Action server that detects AprilTags and returns their world poses."""

    def __init__(self, april_tag_tracker: AprilTagTracker):
        super().__init__("perception_action_server")

        self._april_tag_tracker = april_tag_tracker

        self.detect_april_tags_server = ActionServer(
            self,
            DetectAprilTags,
            "/perception/detect_april_tags",
            execute_callback=self.detect_april_tags_execute_callback,
            goal_callback=self.detect_april_tags_goal_callback,
            cancel_callback=self.detect_april_tags_cancel_callback,
        )

        self.get_logger().info(
            "PerceptionActionServer started on /perception/detect_april_tags"
        )

    def destroy(self):
        self.detect_april_tags_server.destroy()
        super().destroy_node()

    # ---------------------------------------
    # Goal / Cancel Callbacks
    # ---------------------------------------

    def detect_april_tags_goal_callback(
        self, goal_request: DetectAprilTags.Goal
    ) -> GoalResponse:
        self.get_logger().info("Received DetectAprilTags goal request.")
        return GoalResponse.ACCEPT

    def detect_april_tags_cancel_callback(self, goal_handle):
        self.get_logger().warn("DetectAprilTags cancel request received.")
        return CancelResponse.ACCEPT

    # ---------------------------------------
    # Execute Callback
    # ---------------------------------------

    async def detect_april_tags_execute_callback(self, goal_handle):
        """Run AprilTag detection and return all detections."""

        self.get_logger().info("Detecting AprilTags...")

        try:
            detections = self._april_tag_tracker.detect_fiducials()
        except Exception as e:
            self.get_logger().error(f"AprilTag detection failed: {e}")
            result = DetectAprilTags.Result()
            goal_handle.abort()
            return result

        # Build result
        result = DetectAprilTags.Result()
        result.detections = []

        for det in detections:
            msg = AprilTagDetection()
            msg.id = det["id"]
            msg.header.frame_id = det["frame_id"]
            pose = Pose()
            pose.position.x = det["pose"]["position"][0]
            pose.position.y = det["pose"]["position"][1]
            pose.position.z = det["pose"]["position"][2]
            pose.orientation.x = det["pose"]["orientation"][0]
            pose.orientation.y = det["pose"]["orientation"][1]
            pose.orientation.z = det["pose"]["orientation"][2]
            pose.orientation.w = det["pose"]["orientation"][3]

            msg.pose = pose
            result.detections.append(msg)

        goal_handle.succeed()
        self.get_logger().info(
            f"AprilTag detection complete. Found {len(result.detections)} tags."
        )

        return result
