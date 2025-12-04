"""ROS 2 service provider for Spot control operations."""

from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

if TYPE_CHECKING:
    from spot_simple_controllers.spot_sdk_client import SpotSDKClient
    from spot_simple_controllers.manipulation.spot_arm_controller import (
        SpotArmController,
    )
    from spot_simple_controllers.navigation.spot_navigation_controller import (
        SpotNavigationController,
    )


class SpotServiceProvider(Node):
    """Provides ROS 2 services for Spot control.

    Responsibilities:
    - Stand/sit services
    - Arm deploy/stow services
    - Shutdown service
    """

    def __init__(
        self,
        spot_sdk_client: SpotSDKClient,
        arm_controller: SpotArmController,
        navigation_controller: SpotNavigationController,
    ):
        super().__init__("spot_service_provider")

        self._client = spot_sdk_client
        self._arm_controller = arm_controller
        self._navigation_controller = navigation_controller

        # Create services
        self._stand_service = self.create_service(
            Trigger, "spot/stand", self.handle_stand
        )
        self._sit_service = self.create_service(Trigger, "spot/sit", self.handle_sit)

        self._deploy_arm_service = self.create_service(
            Trigger, "spot/deploy_arm", self.handle_deploy_arm
        )
        self._stow_arm_service = self.create_service(
            Trigger, "spot/stow_arm", self.handle_stow_arm
        )
        self._shutdown_service = self.create_service(
            Trigger, "spot/shutdown", self.handle_shutdown
        )

        self.get_logger().info("SpotServiceProvider initialized with all services.")

    def handle_stand(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle stand service request."""
        self.get_logger().info("Stand service called.")

        has_control = self._client.has_control() or self._client.take_control()
        stood_up = self._client.stand_up() if has_control else False

        response.success = stood_up
        response.message = (
            "Spot is now standing." if stood_up else "Could not make Spot stand."
        )
        return response

    def handle_sit(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle sit service request."""
        self.get_logger().info("Sit service called.")

        has_control = self._client.has_control() or self._client.take_control()
        sit_success = self._client.sit_down() if has_control else False

        response.success = sit_success
        response.message = (
            "Spot is now sitting." if sit_success else "Spot could not sit."
        )
        return response

    def handle_deploy_arm(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle deploy arm service request."""
        self.get_logger().info("Deploy arm service called.")

        has_control = self._client.has_control() or self._client.take_control()
        deployed = self._arm_controller.deploy_arm() if has_control else False

        response.success = deployed
        response.message = (
            "Spot's arm has been deployed." if deployed else "Could not deploy arm."
        )
        return response

    def handle_stow_arm(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle stow arm service request."""
        self.get_logger().info("Stow arm service called.")

        has_control = self._client.has_control() or self._client.take_control()
        arm_stowed = self._arm_controller.stow_arm() if has_control else False

        response.success = arm_stowed
        response.message = (
            "Spot's arm has been stowed." if arm_stowed else "Could not stow arm."
        )
        return response

    def handle_shutdown(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle shutdown service request."""
        self.get_logger().info("Shutdown service called.")

        # Stow arm if
        self._arm_controller.stow_arm()

        # Shutdown robot interface
        self._client.shutdown()

        # Shutdown ROS
        self.get_logger().info("Shutting down ROS...")
        rclpy.shutdown()

        response.success = True
        response.message = "Spot has been shut down."
        return response
