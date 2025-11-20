"""Core interface for Spot robot connection and control."""

from __future__ import annotations
import time
from typing import Optional
import rclpy
from rclpy.node import Node

from bosdyn.client import create_standard_sdk
from bosdyn.client.estop import EstopClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.util import setup_logging
from bosdyn.api.estop_pb2 import ESTOP_LEVEL_NONE

from spot_simple_controllers.spot_time_sync import SpotTimeSync
from spot_simple_controllers.manipulation.arm_configuration import SPOT_SDK_ARM_JOINT_NAMES
from bosdyn.api.spot.robot_command_pb2 import BodyControlParams, MobilityParams

from bosdyn.client.robot_command import (
    blocking_stand,
    blocking_sit,
)
    
class SpotSDKClient(Node):
    """Manages connection, authentication, and low-level control of Spot robot.
    
    Responsibilities:
    - Connection and authentication
    - Lease management
    - Power control
    - E-stop monitoring
    - Time synchronization
    - State queries
    """
    
    def __init__(self, hostname: str, username: str, password: str):
        super().__init__("spot_sdk_client")
        
        self._created_time_s = time.time()
        setup_logging(verbose=True)
        
        # Connect and authenticate
        self._sdk = create_standard_sdk("spot")
        self._robot = self._sdk.create_robot(hostname)
        self._robot.authenticate(username=username, password=password)
        
        # Initialize time sync
        self.time_sync = SpotTimeSync(self._robot)
        self.get_logger().info("Time sync established with Spot.")
        for _ in range(5):  # Repeatedly re-sync to hopefully better model network variance
            self.resync_and_log()

        # Initialize clients
        self.command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        self._lease_keeper: Optional[LeaseKeepAlive] = None

        # Verify not e-stopped
        if not self.wait_while_estopped():
            raise RuntimeError("Spot is e-stopped and cannot continue.")
        
        self.get_logger().info("SpotSDKClient initialized successfully.")
    
    def resync_and_log(self) -> None:
        """Resync with Spot and log information describing the resulting time sync."""
        self.time_sync.resync()

        round_trip_s = self.time_sync.get_round_trip_s()
        self.get_logger().info(f"Current round trip time: {round_trip_s} seconds.")

        max_round_trip_s = self.time_sync.max_round_trip_s
        self.get_logger().info(f"Maximum observed round trip time: {max_round_trip_s} seconds.")

        clock_skew_s = self.time_sync.get_robot_clock_skew_s()
        self.get_logger().info(f"Current robot clock skew from local: {clock_skew_s} seconds.")

        max_sync_time_s = self.time_sync.max_sync_time_s
        self.get_logger().info(
            f"Maximum duration any time-sync has taken: {max_sync_time_s} seconds.",
        )

        avg_sync_time_s = self.time_sync.get_avg_sync_time_s()
        self.get_logger().info(
            f"Average duration per resync with Spot: {avg_sync_time_s} seconds.",
        )

    def wait_while_estopped(self, timeout_s: int = 30) -> bool:
        """Wait for Spot to be un-estopped."""
        estop_level = self._estop_client.get_status().stop_level
        start_t = time.time()
        
        while (time.time() - start_t) < timeout_s:
            if estop_level == ESTOP_LEVEL_NONE:
                self.get_logger().info("Spot is not e-stopped.")
                return True
            self.get_logger().info("Spot is e-stopped! Waiting...")
            time.sleep(0.5)
            estop_level = self._estop_client.get_status().stop_level
        
        self.get_logger().error("Timeout waiting for Spot to un-estop.")
        return False
    
    def take_control_and_power_on(self, resource: str = "body", force: bool = False) -> bool:
        """Acquire Spot's lease and power on."""
        if self._lease_keeper is None:
            self.get_logger().info(f"Acquiring resource '{resource}'...")
            if force:
                self._lease_client.take(resource=resource)
            else:
                self._lease_client.acquire(resource=resource)
            
            self._lease_keeper = LeaseKeepAlive(
                lease_client=self._lease_client,
                resource=resource,
                must_acquire=True,
                return_at_exit=True,
            )
    
        if not self._robot.is_powered_on():
            self.get_logger().info("Powering on Spot...")
            self._robot.power_on(timeout_sec=20)
        
        power_success = self._robot.is_powered_on()
        self.get_logger().info("Spot powered on." if power_success else "Spot power on failed.")
        return self.has_control()
    
    def has_control(self) -> bool:
        """Check if we have lease and power."""
        lease_alive = self._lease_keeper is not None and self._lease_keeper.is_alive()
        powered_on = self._robot.is_powered_on()
        return lease_alive and powered_on
    
    def release_control(self) -> None:
        """Release control of Spot."""
        if self._lease_keeper is not None:
            self.get_logger().info("Releasing control of Spot...")
            self._lease_keeper.shutdown()
            self._lease_keeper = None
    
    def has_arm(self) -> bool:
        """Check if Spot has an arm."""
        return self._robot.has_arm()
    
    def get_arm_configuration(self) -> dict[str, float]:
        """Get current arm joint positions."""
        robot_state = self._state_client.get_robot_state()
        sdk_joint_states = robot_state.kinematic_state.joint_states
        
        return {
            joint.name: joint.position.value
            for joint in sdk_joint_states
            if joint.name in SPOT_SDK_ARM_JOINT_NAMES
        }
    
    def send_robot_command(self, command, duration_s: Optional[float] = None) -> Optional[int]:
        """Send a robot command and return command ID."""
        if not self.has_control():
            self.get_logger().warn("Cannot send command: no control of robot.")
            return None
        
        if duration_s is None:
            command_id = self.command_client.robot_command(
                command,
                timesync_endpoint=self.time_sync.endpoint,
            )
        else:
            command_id = self.command_client.robot_command(
                command,
                end_time_secs=time.time() + duration_s,
                timesync_endpoint=self.time_sync.endpoint,
            )
        
        self.get_logger().info(f"Issued robot command with ID: {command_id}")
        return command_id
    
    def safely_power_off(self) -> None:
        """Power Spot off safely."""
        self._robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self._robot.is_powered_on(), "Robot power off failed."
        self.get_logger().info("Robot safely powered off.")
    
    def shutdown(self) -> None:
        """Shutdown interface."""
        if self.has_control():
            self.safely_power_off()
            self.release_control()

    def stand_up(self, timeout_s: float = 20.0, 
                 control_params: Optional[BodyControlParams] = None) -> bool:
        """Command Spot to stand."""
        if not self.has_control():
            self.get_logger().warn("Cannot stand: no control.")
            return False
        
        self.get_logger().info("Standing up...")
        try:
            if control_params is None:
                blocking_stand(self.command_client, timeout_sec=timeout_s)
            else:
                blocking_stand(
                    self.command_client,
                    timeout_sec=timeout_s,
                    params=MobilityParams(body_control=control_params),
                )
        except Exception as e:
            self.get_logger().error(f"Failed to stand: {e}")
            return False
        
        self.get_logger().info("Spot is standing.")
        return True

    def sit_down(self, timeout_s: float = 20.0) -> bool:
        """Command Spot to sit."""
        if not self.has_control():
            self.get_logger().warn("Cannot sit: no control.")
            return False
        
        self.get_logger().info("Sitting down...")
        try:
            blocking_sit(self.command_client, timeout_sec=timeout_s)
        except Exception as e:
            self.get_logger().error(f"Failed to sit: {e}")
            return False
        
        self.get_logger().info("Spot is sitting.")
        return True
    
