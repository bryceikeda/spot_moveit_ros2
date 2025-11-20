"""ROS 2 version: Control Spot's arm using the Spot SDK."""

from __future__ import annotations

import time
import logging
from enum import IntEnum
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from bosdyn.api.gripper_command_pb2 import ClawGripperCommand
from bosdyn.client.exceptions import InvalidRequestError
from bosdyn.client.robot_command import RobotCommandBuilder, block_until_arm_arrives
from bosdyn.util import duration_to_seconds
from bosdyn.client.robot_command import block_until_arm_arrives

from spot_simple_controllers.manipulation.arm_configuration import MAP_JOINT_NAMES_SPOT_SDK_TO_URDF
from spot_simple_controllers.time_stamp import TimeStamp

if TYPE_CHECKING:
    from bosdyn.api.arm_command_pb2 import ArmJointTrajectory
    from spot_simple_controllers.manipulation.joint_trajectory import JointTrajectory
    from spot_simple_controllers.manipulation.segment_schedule import SegmentSchedule
    from spot_simple_controllers.spot_sdk_client import SpotSDKClient

MIN_OPEN_ANGLE_RAD: Final[float] = -1.5707  # Fully closed
MAX_OPEN_ANGLE_RAD: Final[float] = 0.0      # Fully open
START_ANGLE_TOLERANCE_RAD =  0.01
class ArmCommandOutcome(IntEnum):
    """Outcome codes for arm trajectory commands."""
    INVALID_START = -1
    SUCCESS = 0
    PREEMPTED = 1
    ARM_LOCKED = 2


class GripperCommandOutcome(IntEnum):
    """Outcome codes for gripper commands."""
    FAILURE = -1
    REACHED_SETPOINT = 0
    STALLED = 1

class SpotArmController():
    """Low-level controller for Spot's arm and gripper.
    
    This class handles:
    - Trajectory segmentation and execution
    - Timing synchronization with the robot
    - Gripper open/close commands
    - Arm lock state management
    
    Note: This is not a ROS node - it's a pure controller that can be used
    by action servers or other higher-level components.
    """
    def __init__(
        self, 
        spot_sdk_client: SpotSDKClient, 
        max_segment_len: int = 250):

        self._client = spot_sdk_client

        if not self.has_arm():
            raise ValueError("Cannot control Spot's arm if Spot has no arm!")

        self.command_id: Optional[int] = None
        self._locked = True
        self.max_segment_len = min(max_segment_len, 250)
        self._start_angle_tolerance_rad = START_ANGLE_TOLERANCE_RAD
        self._future_proof_s = 1.0
        # Create a logger for the module
        self._logger = logging.getLogger("SpotArmController")
        self._logger.setLevel(logging.INFO)  # Change to DEBUG for more verbose output

        # Add a console handler
        ch = logging.StreamHandler()
        formatter = logging.Formatter("[%(levelname)s] %(name)s: %(message)s")
        ch.setFormatter(formatter)
        self._logger.addHandler(ch)

    def _log_info(self, msg: str):
        self._logger.info(msg)

    def _log_warn(self, msg: str):
        self._logger.warning(msg)

    def _log_error(self, msg: str):
        self._logger.error(msg)

    # -------------------------------------------------------------------------
    # Core control methods
    # -------------------------------------------------------------------------
    def has_arm(self) -> None:
        return self._client.has_arm()
    
    def unlock_arm(self) -> None:
        """Enable arm control."""
        self._locked = False
        self._log_info("Arm unlocked.")
    
    def lock_arm(self) -> None:
        """Disable arm control."""
        self._locked = True
        self._log_info("Arm locked.")
    
    def is_locked(self) -> bool:
        """Check if arm is locked."""
        return self._locked

    def _sleep_until(self, local_time_s: float) -> None:
        """Sleep until the specified local time.
        
        Args:
            local_time_s: Target time in seconds (time.time() format)
        """
        sleep_for_s = max(0.0, local_time_s - time.time())
        deadline_s = time.monotonic() + sleep_for_s
        
        while (remainder_s := deadline_s - time.monotonic()) > 0:
            time.sleep(min(remainder_s, 0.01))

    def _send_segment_command(
        self, 
        idx: int, 
        schedule: SegmentSchedule, 
        max_attempts: int
    ) -> None:
        """Send a single trajectory segment to the robot.
        
        Args:
            idx: Index of segment in schedule
            schedule: Complete segment schedule
            max_attempts: Maximum retry attempts
            
        Raises:
            InvalidRequestError: If command fails after all retries
        """
        if self._locked:
            self._log_warn("Arm is locked. Skipping trajectory segment.")
            return

        # Extract trajectory from command
        traj = schedule.commands[idx].synchronized_command.arm_command.arm_joint_move_command.trajectory
        
        # Validate segment
        assert len(traj.points) > 0, "Segment has no points."
        assert len(traj.points) <= self.max_segment_len, "Segment too long!"
        assert traj.HasField("reference_time"), "Segment must have a reference_time."

        # Calculate timing parameters
        max_rtt_s = max(0.0, self._client.time_sync.max_round_trip_s)
        cushion_s = max(0.1, 2.0 * max_rtt_s)
        eps_s = 0.03  # Additional margin for segment-adjusting overhead
        send_early_s = schedule.min_lead_s + cushion_s + eps_s

        # Wait until it's time to send
        send_local_s = schedule.compute_send_local_time_s(idx, send_early_s)
        self._sleep_until(send_local_s)

        # Adjust schedule if we're late
        delta_s = schedule.slide_segment_if_late(idx, send_early_s)
        if delta_s > 0:
            self._log_warn(f"Late by {delta_s:.3f}s; shifted schedule.")

        # Retry loop for timing-related failures
        for attempt in range(1, max_attempts + 1):
            try:
                self.command_id = self._client.send_robot_command(schedule.commands[idx])
            except InvalidRequestError as err:
                self._log_warn(f"Segment attempt {attempt}/{max_attempts} failed: {err}")
                
                # Only retry for timing-related errors
                if "time point before the current robot time" not in str(err):
                    raise err
                
                if attempt == max_attempts:
                    raise err
                
                # Exponential backoff for retries
                bump_s = 0.03 * (2 ** (attempt - 1))
                delta_s = schedule.slide_segment_if_late(idx, send_early_s + bump_s)
                if delta_s > 0:
                    self._log_warn(f"Shifted schedule by {delta_s:.3f}s for retry.")
            else:
                self._log_info("Trajectory segment sent successfully.")
                return

    def _validate_trajectory_start(
        self, 
        trajectory: JointTrajectory
    ) -> bool:
        """Validate that trajectory starts close to current arm position.
        
        Args:
            trajectory: Trajectory to validate
            
        Returns:
            True if trajectory start is valid, False otherwise
        """
        arm_configuration = self._client.get_arm_configuration()
        command_start_angles_rad = trajectory.points[0].positions_rad
        
        for sdk_joint, curr_rad in arm_configuration.items():
            # Convert SDK joint name to URDF name
            urdf_joint = MAP_JOINT_NAMES_SPOT_SDK_TO_URDF[sdk_joint]
            
            # Find corresponding commanded angle
            try:
                joint_idx = trajectory.joint_names.index(urdf_joint)
            except ValueError:
                self._log_warn(f"Joint {urdf_joint} not found in trajectory.")
                return False
            
            cmd_rad = command_start_angles_rad[joint_idx]
            
            # Check if start position is close enough
            if abs(curr_rad - cmd_rad) > self._start_angle_tolerance_rad:
                self._log_warn(
                    f"Start mismatch for {urdf_joint}: "
                    f"current={curr_rad:.3f} rad, commanded={cmd_rad:.3f} rad "
                    f"(diff={abs(curr_rad - cmd_rad):.3f} rad)"
                )
                return False
        
        return True

    def command_trajectory(
        self,
        trajectory: JointTrajectory,
        preempt_check: Optional[Callable[[], bool]] = None,
        max_attempts: int = 5,
    ) -> ArmCommandOutcome:
        """Execute a joint trajectory on Spot's arm.
        
        Args:
            trajectory: Joint trajectory to execute
            preempt_check: Optional function that returns True to cancel execution
            max_attempts: Maximum retry attempts per segment
            
        Returns:
            Outcome code indicating success or failure reason
        """
        # Check if arm is locked
        if self._locked:
            self._log_warn("Trajectory rejected: arm is locked.")
            return ArmCommandOutcome.ARM_LOCKED

        # Resynchronize time with robot
        self._client.time_sync.resync()
        
        # Validate trajectory start position
        if not self._validate_trajectory_start(trajectory):
            return ArmCommandOutcome.INVALID_START

        # Set trajectory timing
        local_start_time_s = time.time() + self._future_proof_s
        trajectory.reference_timestamp = TimeStamp.from_time_s(local_start_time_s)
        
        # Create segmented schedule
        segments_schedule = trajectory.create_segment_schedule(self.max_segment_len)
        
        self._log_info(
            f"Executing trajectory with {len(segments_schedule.commands)} segments."
        )

        # Execute each segment
        preempted = False
        for idx in range(len(segments_schedule.commands)):
            # Check for preemption
            if preempt_check and preempt_check():
                self._log_warn("Trajectory preempted by caller.")
                preempted = True
                break
            
            # Send segment
            self._send_segment_command(idx, segments_schedule, max_attempts)

        # Wait for arm to reach final position
        if self.command_id is not None:
            self._log_info("Waiting for arm to reach final position...")
            self.block_until_arm_reaches_goal()
            self._log_info("Arm reached final position.")

        return ArmCommandOutcome.PREEMPTED if preempted else ArmCommandOutcome.SUCCESS

    def command_gripper(self, target_rad: float) -> GripperCommandOutcome:
        """Command the gripper to move to a specified open angle.
        
        Args:
            target_rad: Target gripper angle in radians
                       (fully closed = -π/2, fully open = 0)
            
        Returns:
            Outcome code indicating success or failure
        """
        # Check if arm is locked
        if self._locked:
            self._log_warn("Gripper command rejected: arm is locked.")
            return GripperCommandOutcome.FAILURE

        # Check if we have control
        if not self._client.has_control():
            self._log_warn("Gripper command rejected: no control of Spot.")
            return GripperCommandOutcome.FAILURE

        # Validate gripper angle using constants
        if not (MIN_OPEN_ANGLE_RAD <= target_rad <= 
                MAX_OPEN_ANGLE_RAD):
            self._log_warn(
                f"Invalid gripper target: {target_rad:.3f} rad. "
                f"Valid range: [{MIN_OPEN_ANGLE_RAD:.3f}, "
                f"{MAX_OPEN_ANGLE_RAD:.3f}]"
            )
            return GripperCommandOutcome.FAILURE

        # Create and send gripper command
        robot_command = RobotCommandBuilder.claw_gripper_open_angle_command(target_rad)
        self.command_id = self._client.send_robot_command(robot_command)
        
        if self.command_id is None:
            self._log_error("No command ID returned from gripper command.")
            return GripperCommandOutcome.FAILURE
        
        self._log_info(f"Gripper command sent: target={target_rad:.3f} rad")

        # Block until gripper completes command
        return self.block_during_gripper_command()


    def deploy_arm(self) -> bool:
        """Deploy Spot's arm to ready position."""
        if not self._client.has_control():
            self._log_warn("Cannot deploy arm: no control.")
            return False
        
        self._log_info("Deploying arm...")
        cmd = RobotCommandBuilder.arm_ready_command()
        self.command_id = self._client.send_robot_command(cmd)
        
        if self.command_id is None:
            return False
        
        self.block_until_arm_reaches_goal()
        self._log_info("Arm deployed.")
        return True
    
    def stow_arm(self) -> bool:
        """Stow Spot's arm."""
        if not self._client.has_control():
            self._log_warn("Cannot stow arm: no control.")
            return False
        
        self._log_info("Stowing arm...")
        cmd = RobotCommandBuilder.arm_stow_command()
        self.command_id = self._client.send_robot_command(cmd)
        
        if self.command_id is None:
            return False
        
        self.block_until_arm_reaches_goal()
        self._log_info("Arm stowed.")
        return True

    def block_until_arm_reaches_goal(self) -> None:
        """Block until Spot's arm arrives at the identified command's goal."""

        self._log_info("Blocking until arm arrives...")
        block_until_arm_arrives(self._client.command_client, self.command_id)
        time.sleep(0.5)
        self._log_info("Done blocking.\n")

    def block_during_gripper_command(
            self,
            timeout_s: float = 5.0,
        ) -> GripperCommandOutcome:
        """Block until Spot's gripper completes the identified command (or time runs out).

        :param timeout_s: Timeout (seconds) after which the command is considered failed
        :return: Enum member indicating the outcome of the gripper command
        """
        end_time = time.time() + timeout_s
        now = time.time()

        while now < end_time:
            response = self._client.command_client.robot_command_feedback(self.command_id)
            if response.feedback.HasField("synchronized_feedback"):
                sync_fb = response.feedback.synchronized_feedback

                if sync_fb.HasField("gripper_command_feedback"):
                    gripper_status = sync_fb.gripper_command_feedback.claw_gripper_feedback.status

                    # If gripper has reached its goal, or entered force control mode, success!
                    if gripper_status == ClawGripperCommand.Feedback.STATUS_AT_GOAL:
                        return GripperCommandOutcome.REACHED_SETPOINT

                    if gripper_status == ClawGripperCommand.Feedback.STATUS_APPLYING_FORCE:
                        return GripperCommandOutcome.STALLED

                    if gripper_status == ClawGripperCommand.Feedback.STATUS_UNKNOWN:
                        return GripperCommandOutcome.FAILURE

            time.sleep(0.1)
            now = time.time()

        return GripperCommandOutcome.FAILURE