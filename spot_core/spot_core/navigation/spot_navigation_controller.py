from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import time
from geometry_msgs.msg import PoseStamped


if TYPE_CHECKING:
    from spot_core.spot_commander import (
        SpotCommander,
    )
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose, SE2Pose

from synchros2.tf_listener_wrapper import TFListenerWrapper
from bosdyn.api import (
    arm_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
)
from spot_core.navigation.navigation_goal_checker import (
    NavigationGoalChecker,
)
from google.protobuf import wrappers_pb2
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
)


class SpotNavigationController:
    """A ROS 2 node providing navigation for Spot."""

    def __init__(
        self,
        spot_commander: SpotCommander,
        state_client,
        tf_listener_wrapper: TFListenerWrapper,
        navigation_goal_checker: NavigationGoalChecker,
        timeout_s=20.0,
    ) -> None:
        self.spot_commander = spot_commander
        self.state_client = state_client
        self.timeout_s = timeout_s

        self.tf_listener_wrapper = tf_listener_wrapper
        self._CMD_VEL_DURATION_S = 1.0
        self.navigation_goal_checker = navigation_goal_checker

    # ======================================================================
    #                          SERVICE HANDLERS
    # ======================================================================
    def stand(self):
        """Stand robot."""
        self.spot_commander.stand_up()

    def sit(self):
        """Sit robot."""
        self.spot_commander.sit_down()

    def build_arm_command(self, arm_traj=None, max_vel=4, max_acc=4):
        """Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory."""

        max_vel = wrappers_pb2.DoubleValue(value=max_vel)
        max_acc = wrappers_pb2.DoubleValue(value=max_acc)

        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(
            points=[arm_traj], maximum_velocity=max_vel, maximum_acceleration=max_acc
        )
        joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(
            trajectory=arm_joint_traj
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_joint_move_command=joint_move_command
        )
        sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        arm_sync_robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=sync_arm
        )
        return arm_sync_robot_cmd

    def compute_goal_in_vision_frame(self, goal: PoseStamped, frame_id: str) -> SE2Pose:
        """
        Compute the goal pose in the vision frame.

        Returns:
            SE2Pose: The goal pose in vision frame coordinates
        """
        # vision frame is world coordinates based on how it views the world from startup
        # Convert from vision to body frame to get the body in world coordinates
        world_t_robot = self.tf_listener_wrapper.lookup_a_tform_b(
            VISION_FRAME_NAME, frame_id
        )
        if world_t_robot is None:
            logging.error("Could not lookup transform.")
            return None

        world_t_robot_se2 = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()

        # Add the goal distance to the current body position in world coordinates
        return world_t_robot_se2 * goal

    def navigate_to_pose(self, waypoint: PoseStamped, use_arm: bool) -> bool:
        """
        Walk forward to a goal in the world frame.
        Arguments:
            waypoint: The waypoint to walk to with respect to the body frame.
                      Type is geometry_msgs.msg.PoseStamped.
                      This is using the vision frame of the spot
            use_arm: Whether to use the arm during navigation

        Returns:
            bool: True if goal was reached, False otherwise
        """
        # Convert goal to 2d coordinates
        goal = SE3Pose(
            waypoint.pose.position.x,
            waypoint.pose.position.y,
            waypoint.pose.position.z,
            Quat(
                waypoint.pose.orientation.w,
                waypoint.pose.orientation.x,
                waypoint.pose.orientation.y,
                waypoint.pose.orientation.z,
            ),
        ).get_closest_se2_transform()

        if waypoint.header.frame_id == "body":
            goal = self.compute_goal_in_vision_frame(goal, BODY_FRAME_NAME)

            if goal is None:
                logging.error("Failed to compute vision frame goal")
                return False

        mobility_command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goal.x,
            goal_y=goal.y,
            goal_heading=goal.angle,
            frame_name=VISION_FRAME_NAME,
        )

        robot_command = mobility_command

        if use_arm:
            traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
                0, -1.77, 2.73, 0, -0.9, 0
            )
            # Build the arm command
            arm_command = self.build_arm_command(traj_point)

            # Combine arm and mobility commands into a synchronized command
            robot_command = RobotCommandBuilder.build_synchro_command(
                arm_command, mobility_command
            )

        return self.send_goal_until_at_pose(robot_command, goal, VISION_FRAME_NAME)

    def send_goal_until_at_pose(self, robot_command, goal, frame_id):
        """
        Send robot command repeatedly until goal is reached or timeout occurs.

        Args:
            robot_command: The robot command to send
            world_t_goal: The target goal pose

        Returns:
            bool: True if goal was reached, False otherwise
        """
        reached_goal = self.navigation_goal_checker.is_goal_reached(goal, frame_id)
        end_time_s = time.time() + self.timeout_s

        while not reached_goal and time.time() < end_time_s:
            command_id = self.spot_commander.send_robot_command(
                robot_command, duration_s=5
            )
            if command_id is None:
                logging.warning(
                    "Navigation attempt returned None instead of a command ID."
                )
                time.sleep(0.2)
                continue

            reached_goal = self.navigation_goal_checker.is_goal_reached(goal, frame_id)
            time.sleep(0.2)

        stop_command = RobotCommandBuilder.stop_command()
        self.spot_commander.send_robot_command(stop_command)

        return self.navigation_goal_checker.is_goal_reached(goal, frame_id)
