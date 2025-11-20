"""ROS 2 Action Server for Spot gripper control."""

from __future__ import annotations
import time
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import GripperCommand

from spot_simple_controllers.manipulation.spot_arm_controller import GripperCommandOutcome

if TYPE_CHECKING:
    from spot_simple_controllers.spot_sdk_client import SpotSDKClient
    from spot_simple_controllers.manipulation.spot_arm_controller import SpotArmController


class GripperActionServer(Node):
    """Action server for gripper control.
    
    Responsibilities:
    - Handle GripperCommand action requests
    - Execute gripper open/close commands
    - Report gripper status
    """
    
    def __init__(self, spot_sdk_client: SpotSDKClient, 
                 arm_controller: SpotArmController):
        super().__init__("gripper_action_server")
        
        self._client = spot_sdk_client
        self._arm_controller = arm_controller
        self._gripper_locked = True
        
        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        
        self.get_logger().info(
            "GripperActionServer started on /gripper_controller/gripper_cmd"
        )
    
    def unlock_gripper(self) -> None:
        """Enable gripper control."""
        self._gripper_locked = False
        self.get_logger().info("Gripper unlocked.")
    
    def lock_gripper(self) -> None:
        """Disable gripper control."""
        self._gripper_locked = True
        self.get_logger().info("Gripper locked.")
    
    def is_gripper_locked(self) -> bool:
        """Check if gripper is locked."""
        return self._gripper_locked
    
    def goal_callback(self, goal_request: GripperCommand.Goal) -> GoalResponse:
        """Validate incoming gripper goals."""
        self.get_logger().info(
            f"Received gripper command: position={goal_request.command.position:.3f}, "
            f"max_effort={goal_request.command.max_effort:.3f}"
        )
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().warn("Cancel request received (currently not implemented)")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute gripper command."""
        goal = goal_handle.request
        result = GripperCommand.Result()
        
        # Check if gripper is locked
        if self._gripper_locked:
            result.reached_goal = False
            self.get_logger().warn("Gripper command rejected: gripper is locked.")
            goal_handle.abort()
            return result
        
        # Ensure we have control
        if not self._client.has_control():
            if not self._client.take_control():
                result.reached_goal = False
                self.get_logger().error("Failed to acquire robot control.")
                goal_handle.abort()
                return result
        
        # Execute gripper command
        goal_position = goal.command.position
        self.get_logger().info(f"Commanding gripper to position: {goal_position:.3f} rad")
        
        outcome = self._arm_controller.command_gripper(goal_position)
        time.sleep(0.25)
        
        if outcome == GripperCommandOutcome.FAILURE:
            result.reached_goal = False
            self.get_logger().error("Gripper command failed.")
            goal_handle.abort()
        else:
            result.reached_goal = (outcome == GripperCommandOutcome.REACHED_SETPOINT)
            result.stalled = (outcome == GripperCommandOutcome.STALLED)
            
            if result.reached_goal:
                self.get_logger().info("Gripper reached goal position.")
            elif result.stalled:
                self.get_logger().info("Gripper stalled (applying force).")
            
            goal_handle.succeed()
        
        return result