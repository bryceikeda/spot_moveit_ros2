#!/usr/bin/env python3
"""Main entry point for Spot ROS 2 control system."""
from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_core.manipulation.arm_trajectory_action_server import (
    ArmTrajectoryActionServer,
)
from spot_core.manipulation.gripper_action_server import (
    GripperActionServer,
)
from spot_core.manipulation.spot_arm_controller import SpotArmController
from spot_core.navigation.spot_navigation_controller import (
    SpotNavigationController,
)
from spot_core.navigation.navigation_action_server import (
    NavigationActionServer,
)
from spot_core.navigation.navigation_goal_checker import (
    NavigationGoalChecker,
)
from spot_core.spot_service_provider import SpotServiceProvider
from synchros2.tf_listener_wrapper import TFListenerWrapper

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.util import setup_logging
from bosdyn.client.graph_nav import GraphNavClient

from spot_core.spot_commander import SpotCommander

MAX_SEGMENT_LENGTH = 30
"""Core interface for Spot robot connection and control."""


class SpotControlNode(Node):
    """Handles initialization of Spot control system components."""

    def __init__(self):
        super().__init__("spot_control_node")

        # Declare parameters
        self.declare_parameter("hostname", "")
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")
        self.declare_parameter("auto_claim", False)
        self.declare_parameter("auto_stand", False)

        # Get parameters
        self.hostname = self.get_parameter("hostname").value
        self.username = self.get_parameter("username").value
        self.password = self.get_parameter("password").value
        self.auto_claim = self.get_parameter("auto_claim").value
        self.auto_stand = self.get_parameter("auto_stand").value

        # Initialize components
        self.nodes = []

        setup_logging(verbose=False)

        # Connect and authenticate
        self._sdk = create_standard_sdk("spot")
        self.robot = self._sdk.create_robot(self.hostname)
        self.robot.authenticate(username=self.username, password=self.password)

        self.state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )
        self.graph_nav_client = self.robot.ensure_client(
            GraphNavClient.default_service_name
        )

        self.spot_commander = None

        # Controllers
        self.arm_trajectory_server = None
        self.gripper_server = None
        self.navigation_action_server = None
        self.tf_listener_wrapper = None
        self.navigation_goal_checker = None

    def initialize_robot(self):
        # Setup robot if auto_claim enabled
        self.spot_commander = SpotCommander(self.robot)
        self.nodes.append(self.spot_commander)

        if not self.spot_commander.take_control_and_power_on(force=True):
            raise RuntimeError("Failed to take control of Spot robot")

        self.tf_listener_wrapper = TFListenerWrapper(self)

        if self.auto_claim:
            self._setup_controllers()

    def _setup_controllers(self):
        """Claim robot and initialize arm controllers."""
        # Initialize arm controller and action servers
        if not self.robot.has_arm():
            raise ValueError("Cannot control Spot's arm if Spot has no arm!")

        arm_controller = SpotArmController(
            self.spot_commander, self.state_client, MAX_SEGMENT_LENGTH
        )
        self.arm_trajectory_server = ArmTrajectoryActionServer(arm_controller)
        self.gripper_server = GripperActionServer(arm_controller)

        self.navigation_goal_checker = NavigationGoalChecker(self.tf_listener_wrapper)
        navigation_controller = SpotNavigationController(
            self.spot_commander,
            self.state_client,
            self.tf_listener_wrapper,
            self.navigation_goal_checker,
        )
        self.navigation_action_server = NavigationActionServer(navigation_controller)

        self.nodes.extend(
            [
                self.arm_trajectory_server,
                self.gripper_server,
                self.navigation_action_server,
            ]
        )

        # Stand robot if configured
        if self.auto_stand:
            self.get_logger().info("Standing robot...")
            if not self.spot_commander.stand_up():
                raise RuntimeError("Failed to stand robot")

        # Create service provider
        self.nodes.append(
            SpotServiceProvider(
                self.spot_commander, arm_controller, navigation_controller
            )
        )

    def shutdown(self):
        self.spot_commander.shutdown()


def main(args=None):
    """Initialize and run Spot control system."""
    rclpy.init(args=args)

    node = None
    executor = MultiThreadedExecutor()

    try:
        node = SpotControlNode()
        executor.add_node(node)

        node.initialize_robot()

        for child_node in node.nodes:
            executor.add_node(child_node)

        executor.spin()

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down...")
    except Exception as e:
        if node:
            node.get_logger().error(f"Error: {e}")
        raise
    finally:
        executor.shutdown()

        if node:
            node.shutdown()

        if node:
            for child_node in node.nodes:
                try:
                    child_node.destroy_node()
                except Exception:
                    pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
