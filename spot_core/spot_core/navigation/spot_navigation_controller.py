from __future__ import annotations

from typing import TYPE_CHECKING

import time

if TYPE_CHECKING:
    from spot_core.spot_commander import (
        SpotCommander,
    )
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    get_vision_tform_body,
)
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
from spot_core.navigation.graph_nav_client_wrapper import (
    GraphNavClientWrapper,
)
import math
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, TransformStamped
from spot_core.navigation.fiducial_handler import FiducialHandler
from tf2_ros import TransformBroadcaster
import logging

logger = logging.getLogger(__name__)


class SpotNavigationController:
    def __init__(
        self,
        spot_commander: SpotCommander,
        state_client,
        graph_nav_client_wrapper: GraphNavClientWrapper,
        tf_listener_wrapper: TFListenerWrapper,
        fiducial_handler: FiducialHandler,
        navigation_goal_checker: NavigationGoalChecker,
        timeout_s=20.0,
    ) -> None:
        self.spot_commander = spot_commander
        self.state_client = state_client
        self.timeout_s = timeout_s
        self.graph_nav_client_wrapper = graph_nav_client_wrapper
        self.tf_listener_wrapper = tf_listener_wrapper
        self.fiducial_handler = fiducial_handler
        self._CMD_VEL_DURATION_S = 1.0
        self.navigation_goal_checker = navigation_goal_checker
        self.current_sign = -1
        self.joint_0 = 1.5
        self.joint_3 = 0.97
        self.joint_5 = 0.45

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

    def compute_goal_in_vision_frame(self, goal, frame_id: str) -> SE2Pose:
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
            logger.error("Could not lookup transform.")
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

    def navigate_to_graph_nav_waypoint(self, waypoint: str):
        self.graph_nav_client_wrapper.navigate_to_waypoint_blocking(waypoint)

    def navigate_to_pose(self, waypoint: PoseStamped) -> bool:
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
        logger.info("Adding pose to goal")
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

        mobility_command = self.build_mobility_command(
            goal,
            waypoint.header.frame_id,
        )

        return self.send_goal_until_at_pose(mobility_command, goal, VISION_FRAME_NAME)

    def build_mobility_command(self, goal, frame_id: str):
        if frame_id == "body":
            goal = self.compute_goal_in_vision_frame(goal, BODY_FRAME_NAME)

            if goal is None:
                logger.error("Failed to compute vision frame goal")
                return False

        mobility_command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goal.x,
            goal_y=goal.y,
            goal_heading=goal.angle,
            frame_name=VISION_FRAME_NAME,
            locomotion_hint=spot_command_pb2.HINT_TROT,
        )

        return mobility_command

    def build_initial_arm_command(self):
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            0, -1.77, 2.73, 0, -0.9, 0
        )
        # Build the arm command
        arm_command = self.build_arm_command(traj_point)
        return arm_command

    def build_oscillating_arm_command(
        self,
    ):
        self.joint_0 = self.joint_0 * self.current_sign
        self.joint_3 = self.joint_3 * self.current_sign
        self.joint_5 = self.joint_5 * self.current_sign
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            self.joint_0, -1.77, 2.73, self.joint_3, -0.9, self.joint_5
        )

        return self.build_arm_command(traj_point)

    def send_goal_until_at_pose(self, mobility_command, goal, frame_id):
        """
        Send robot command repeatedly until goal is reached or timeout occurs.

        Args:
            robot_command: The robot command to send
            world_t_goal: The target goal pose

        Returns:
            bool: True if goal was reached, False otherwise
        """
        reached_goal = False
        end_time_s = time.time() + self.timeout_s
        arm_command = self.build_initial_arm_command()
        robot_command = RobotCommandBuilder.build_synchro_command(
            arm_command, mobility_command
        )
        while not reached_goal and time.time() < end_time_s:

            command_id = self.spot_commander.send_robot_command(
                robot_command, duration_s=20
            )
            if command_id is None:
                logger.warning(
                    "Navigation attempt returned None instead of a command ID."
                )
                time.sleep(0.2)
                continue

            reached_goal = self.navigation_goal_checker.is_goal_reached(goal, frame_id)
            time.sleep(0.2)
            # arm_command = self.build_oscillating_arm_command()
            # robot_command = RobotCommandBuilder.build_synchro_command(
            #     arm_command, mobility_command
            # )

        stop_command = RobotCommandBuilder.stop_command()
        self.spot_commander.send_robot_command(stop_command)

        return self.navigation_goal_checker.is_goal_reached(goal, frame_id)

    def navigate_to_fiducial(self, fiducial_name):
        facing_fiducial_rt_world = self.fiducial_handler.compute_pose_facing_fiducial(
            fiducial_name
        )

        if facing_fiducial_rt_world is not None:
            logger.info(
                f"Computed position facing fiducial {facing_fiducial_rt_world}."
            )
            self.navigate_to_pose(facing_fiducial_rt_world)
            return

        logger.warning(f"Failed to locate fiducial {fiducial_name}.")
