#!/usr/bin/env python3
"""Main entry point for Spot ROS 2 control system."""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from typing import List, Optional, Tuple

from spot_simple_controllers.spot_sdk_client import SpotSDKClient
from spot_simple_controllers.manipulation.arm_trajectory_action_server import ArmTrajectoryActionServer
from spot_simple_controllers.manipulation.gripper_action_server import GripperActionServer
from spot_simple_controllers.manipulation.spot_arm_controller import SpotArmController
from spot_simple_controllers.spot_service_provider import SpotServiceProvider

MAX_SEGMENT_LENGTH = 30


class SpotConfig:
    """Configuration container for Spot robot parameters."""
    
    def __init__(self, node: Node):
        """Read and store configuration from ROS parameters."""
        node.declare_parameter('hostname', "")
        node.declare_parameter('username', "")
        node.declare_parameter('password', "")
        node.declare_parameter('auto_claim', False)
        node.declare_parameter('auto_stand', False)
        node.declare_parameter('auto_unlock_arm', False)

        self.hostname = node.get_parameter('hostname').value
        self.username = node.get_parameter('username').value
        self.password = node.get_parameter('password').value
        self.auto_claim = node.get_parameter('auto_claim').value
        self.auto_stand = node.get_parameter('auto_stand').value
        self.auto_unlock_arm = node.get_parameter('auto_unlock_arm').value

    def log_config(self, logger):
        """Log configuration details."""
        logger.info(f"Connecting to Spot at {self.hostname} as {self.username}")
        logger.info(f"Auto-claim: {self.auto_claim}, Auto-stand: {self.auto_stand}")


class SpotSystemInitializer:
    """Handles initialization of Spot control system components."""
    
    def __init__(self, config: SpotConfig):
        self.config = config
        self.nodes: List[Node] = []
        self.spot_sdk_client: Optional[SpotSDKClient] = None
        self.arm_trajectory_server: Optional[ArmTrajectoryActionServer] = None
        self.gripper_server: Optional[GripperActionServer] = None
    
    def initialize_sdk_client(self) -> SpotSDKClient:
        """Initialize and return the Spot SDK client."""
        self.spot_sdk_client = SpotSDKClient(
            hostname=self.config.hostname,
            username=self.config.username,
            password=self.config.password,
        )
        self.nodes.append(self.spot_sdk_client)
        return self.spot_sdk_client
    
    def claim_and_setup_robot(self) -> None:
        """Take control of robot and setup arm controllers if auto_claim is enabled."""
        if not self.config.auto_claim:
            return
        
        if not self.spot_sdk_client.take_control_and_power_on(force=True):
            raise RuntimeError("Failed to take control of Spot robot and power it on.")
        
        arm_controller = SpotArmController(
            spot_sdk_client=self.spot_sdk_client,
            max_segment_len=MAX_SEGMENT_LENGTH
        )
        
        self.arm_trajectory_server = ArmTrajectoryActionServer(
            self.spot_sdk_client, 
            arm_controller
        )
        self.gripper_server = GripperActionServer(
            self.spot_sdk_client, 
            arm_controller
        )
        self.nodes.extend([self.arm_trajectory_server, self.gripper_server])
        
        if self.config.auto_unlock_arm:
            self._unlock_arm_and_gripper()
    
    def _unlock_arm_and_gripper(self) -> None:
        """Unlock arm and gripper for operation."""
        self.arm_trajectory_server.unlock_arm()
        self.gripper_server.unlock_gripper()
        self.spot_sdk_client.get_logger().info(
            "Spot's arm and gripper are now unlocked."
        )
    
    def stand_robot(self) -> None:
        """Stand the robot if auto_stand is enabled."""
        if not self.config.auto_stand:
            return
        
        self.spot_sdk_client.get_logger().info("Standing robot...")
        if not self.spot_sdk_client.stand_up():
            raise RuntimeError("Failed to stand Spot robot.")
    
    def create_service_provider(self) -> SpotServiceProvider:
        """Create and return the service provider node."""
        service_provider = SpotServiceProvider(
            spot_sdk_client=self.spot_sdk_client,
            arm_trajectory_action_server=self.arm_trajectory_server,
            gripper_action_server=self.gripper_server,
        )
        self.nodes.append(service_provider)
        return service_provider


def read_configuration() -> SpotConfig:
    """Read configuration parameters from ROS."""
    temp_node = Node('spot_param_reader')
    config = SpotConfig(temp_node)
    config.log_config(temp_node.get_logger())
    temp_node.destroy_node()
    return config


def setup_executor(nodes: List[Node]) -> MultiThreadedExecutor:
    """Create and configure the executor with all nodes."""
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    return executor


def cleanup_system(
    executor: Optional[MultiThreadedExecutor],
    spot_sdk_client: Optional[SpotSDKClient],
    nodes: List[Node]
) -> None:
    """Clean up all system resources."""
    if executor:
        executor.shutdown()
    
    if spot_sdk_client:
        spot_sdk_client.shutdown()
    
    for node in nodes:
        try:
            node.destroy_node()
        except Exception:
            pass
    
    if rclpy.ok():
        rclpy.shutdown()


def log_startup_info(logger_node: Node) -> None:
    """Log startup information about available services and actions."""
    logger = logger_node.get_logger()
    
    services = [
        "/spot/stand", "/spot/sit", "/spot/unlock_arm",
        "/spot/lock_arm", "/spot/deploy_arm", "/spot/stow_arm",
        "/spot/shutdown"
    ]
    
    action_servers = [
        "/arm_controller/follow_joint_trajectory",
        "/gripper_controller/gripper_cmd"
    ]
    
    logger.info("=" * 60)
    logger.info("Spot ROS 2 Control System Ready!")
    logger.info("=" * 60)
    logger.info("Available services:")
    for service in services:
        logger.info(f"  - {service}")
    logger.info("Available action servers:")
    for action in action_servers:
        logger.info(f"  - {action}")
    logger.info("=" * 60)


def main(args=None):
    """Initialize and run Spot control system."""
    rclpy.init(args=args)
    
    initializer = None
    executor = None
    
    try:
        # Read configuration
        config = read_configuration()
        
        # Initialize system components
        initializer = SpotSystemInitializer(config)
        spot_sdk_client = initializer.initialize_sdk_client()
        
        # Claim robot and setup controllers
        initializer.claim_and_setup_robot()
        
        # Stand robot if configured
        initializer.stand_robot()
        
        # Create service provider
        initializer.create_service_provider()
        
        # Setup and run executor
        executor = setup_executor(initializer.nodes)
        log_startup_info(spot_sdk_client)
        executor.spin()
        
    except KeyboardInterrupt:
        if initializer and initializer.spot_sdk_client:
            initializer.spot_sdk_client.get_logger().info(
                "Keyboard interrupt received, shutting down..."
            )
    except Exception as e:
        if initializer and initializer.spot_sdk_client:
            initializer.spot_sdk_client.get_logger().error(f"Error: {e}")
        raise
    finally:
        cleanup_system(
            executor,
            initializer.spot_sdk_client if initializer else None,
            initializer.nodes if initializer else []
        )


if __name__ == '__main__':
    main()