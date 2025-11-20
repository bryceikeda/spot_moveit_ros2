"""ROS 2 Action Server for Spot arm trajectory control."""

from __future__ import annotations
import time
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory

from spot_simple_controllers.manipulation.joint_trajectory import JointTrajectory
from spot_simple_controllers.manipulation.spot_arm_controller import ArmCommandOutcome

if TYPE_CHECKING:
    from spot_simple_controllers.spot_sdk_client import SpotSDKClient
    from spot_simple_controllers.spot_arm_controller import SpotArmController


class ArmTrajectoryActionServer(Node):
    """Action server for arm trajectory following.
    
    Responsibilities:
    - Handle FollowJointTrajectory action requests
    - Validate and execute arm trajectories
    - Report trajectory execution status
    """
    
    def __init__(self, spot_sdk_client: SpotSDKClient, 
                 arm_controller: SpotArmController):
        super().__init__("arm_trajectory_action_server")
        
        self._client = spot_sdk_client
        self._arm_controller = arm_controller
        self._arm_locked = True
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        
        self.get_logger().info(
            "ArmTrajectoryActionServer started on /arm_controller/follow_joint_trajectory"
        )
    
    def unlock_arm(self) -> None:
        """Enable arm control."""
        self._arm_locked = False
        self._arm_controller.unlock_arm()
        self.get_logger().info("Arm unlocked for trajectory control.")
    
    def lock_arm(self) -> None:
        """Disable arm control."""
        self._arm_locked = True
        self.get_logger().info("Arm locked.")
    
    def is_arm_locked(self) -> bool:
        """Check if arm is locked."""
        return self._arm_locked
    
    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        """Validate incoming trajectory goals."""
        try:
            num_points = len(goal_request.trajectory.points)
            self.get_logger().info(f"Received arm trajectory with {num_points} points")
            return GoalResponse.ACCEPT
        except AttributeError:
            self.get_logger().error("Received goal of incorrect type")
            return GoalResponse.REJECT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().warn("Cancel request received (currently not implemented)")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute arm trajectory."""
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        result.error_code = -1
        
        # Check if arm is locked
        if self._arm_locked:
            result.error_string = "Arm is locked."
            self.get_logger().warn(result.error_string)
            goal_handle.abort()
            return result
        
        # Ensure we have control
        if not self._client.has_control():
            if not self._client.take_control():
                result.error_string = "Failed to acquire robot control."
                self.get_logger().error(result.error_string)
                goal_handle.abort()
                return result
        
        # Convert ROS trajectory to internal format
        trajectory = JointTrajectory.from_ros_msg(goal.trajectory)
        
        if trajectory.points:
            first_time = trajectory.points[0].time_from_start_s
            last_time = trajectory.points[-1].time_from_start_s
            duration = last_time - first_time
            self.get_logger().info(
                f"Executing trajectory: {len(trajectory.points)} points, {duration:.2f}s duration"
            )
        
        # Execute trajectory
        outcome = self._arm_controller.command_trajectory(trajectory)
        
        if outcome == ArmCommandOutcome.SUCCESS:
            time.sleep(0.25)
            result.error_code = int(outcome)
            result.error_string = "Success!"
            self.get_logger().info("Trajectory execution succeeded.")
            goal_handle.succeed()
        elif outcome == ArmCommandOutcome.ARM_LOCKED:
            result.error_string = "Arm is locked."
            self.get_logger().warn(result.error_string)
            goal_handle.abort()
        elif outcome == ArmCommandOutcome.INVALID_START:
            result.error_string = "Invalid start configuration."
            self.get_logger().error(result.error_string)
            goal_handle.abort()
        elif outcome == ArmCommandOutcome.PREEMPTED:
            result.error_string = "Trajectory was preempted."
            self.get_logger().warn(result.error_string)
            goal_handle.abort()
        else:
            result.error_string = "Trajectory execution failed."
            self.get_logger().error(result.error_string)
            goal_handle.abort()
        
        return result