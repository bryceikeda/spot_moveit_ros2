#!/usr/bin/env python3
"""Main entry point for Spot ROS 2 control system."""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_simple_controllers.spot_sdk_client import SpotSDKClient
from spot_simple_controllers.manipulation.arm_trajectory_action_server import (
    ArmTrajectoryActionServer,
)
from spot_simple_controllers.manipulation.gripper_action_server import (
    GripperActionServer,
)
from spot_simple_controllers.manipulation.spot_arm_controller import SpotArmController
from spot_simple_controllers.navigation.spot_navigation_controller import (
    SpotNavigationController,
)
from spot_simple_controllers.navigation.navigation_action_server import (
    NavigationActionServer,
)
from spot_simple_controllers.navigation.navigation_goal_checker import (
    NavigationGoalChecker,
)
from spot_simple_controllers.spot_service_provider import SpotServiceProvider
from synchros2.tf_listener_wrapper import TFListenerWrapper

MAX_SEGMENT_LENGTH = 30


class SpotSimpleControllersNode(Node):
    """Handles initialization of Spot control system components."""

    def __init__(self):
        super().__init__("spot_simple_controllers_node")

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
        self.spot_sdk_client = None

        self.arm_trajectory_server = None
        self.gripper_server = None
        self.navigation_action_server = None
        self.tf_listener_wrapper = None
        self.navigation_goal_checker = None

    def initialize_robot(self):
        # Setup robot if auto_claim enabled
        self.spot_sdk_client = SpotSDKClient(
            self.hostname, self.username, self.password
        )
        self.nodes.append(self.spot_sdk_client)

        if not self.spot_sdk_client.take_control_and_power_on(force=True):
            raise RuntimeError("Failed to take control of Spot robot")

        self.tf_listener_wrapper = TFListenerWrapper(self)

        if self.auto_claim:
            self._setup_controllers()

    def _setup_controllers(self):
        """Claim robot and initialize arm controllers."""
        # Initialize arm controller and action servers
        arm_controller = SpotArmController(self.spot_sdk_client, MAX_SEGMENT_LENGTH)
        self.arm_trajectory_server = ArmTrajectoryActionServer(arm_controller)
        self.gripper_server = GripperActionServer(arm_controller)

        self.navigation_goal_checker = NavigationGoalChecker(self.tf_listener_wrapper)
        navigation_controller = SpotNavigationController(
            self.spot_sdk_client,
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
            if not self.spot_sdk_client.stand_up():
                raise RuntimeError("Failed to stand robot")

        # Create service provider
        self.nodes.append(
            SpotServiceProvider(
                self.spot_sdk_client, arm_controller, navigation_controller
            )
        )


def main(args=None):
    """Initialize and run Spot control system."""
    rclpy.init(args=args)

    node = None
    executor = MultiThreadedExecutor()

    try:
        node = SpotSimpleControllersNode()
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

        if node and node.spot_sdk_client:
            node.spot_sdk_client.shutdown()

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
