from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseStamped

if TYPE_CHECKING:
    from spot_core.navigation.spot_navigation_controller import (
        SpotNavigationController,
    )


class NavigationActionServer(Node):
    """Action server for Spot navigation."""

    def __init__(self, spot_navigation_controller: SpotNavigationController):
        super().__init__("navigation_action_server")

        self._spot_navigation_controller = spot_navigation_controller

        self.pose_goal = PoseStamped()

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/navigation/navigate_to_pose",
            execute_callback=self.navigate_to_pose_execute_callback,
            goal_callback=self.navigate_to_pose_goal_callback,
            cancel_callback=self.navigate_to_pose_cancel_callback,
        )

        self.get_logger().info(
            "NavigationActionServer started on /navigation/navigate_to_pose"
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # --------------------------
    # Callbacks
    # --------------------------

    def navigate_to_pose_goal_callback(
        self, goal_request: NavigateToPose.Goal
    ) -> GoalResponse:
        self.get_logger().info("Received NavigateToPose goal request.")
        return GoalResponse.ACCEPT

    def navigate_to_pose_cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel request received (not implemented).")
        return CancelResponse.ACCEPT

    async def navigate_to_pose_execute_callback(self, goal_handle):
        """Execute navigation request."""
        goal_msg = goal_handle.request
        pose: PoseStamped = goal_msg.pose

        self.get_logger().info(
            f"Executing navigation to: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        # Execute navigation (your own Spot SDK logic)
        try:
            outcome = self._spot_navigation_controller.navigate_to_pose(
                pose, True  # Pass just the Pose, not PoseStamped
            )
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            result = NavigateToPose.Result()
            goal_handle.abort()
            return result

        # Construct result object
        result = NavigateToPose.Result()
        if outcome:
            goal_handle.succeed()
            self.get_logger().info("Navigation succeeded.")
        else:
            goal_handle.abort()
            self.get_logger().error("Navigation failed.")

        return result
