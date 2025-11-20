import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _get_controller_config(hardware_interface: str) -> tuple[dict, list[str]]:
    """
    Determine robot mappings and controllers based on hardware interface type.
    
    Args:
        hardware_interface: Either 'mock' or 'robot'
    
    Returns:
        Tuple of (mappings dict, controller list)
    """
    mappings = {"ros2_control_hardware": "mock_components"}
    controllers = ["arm_controller", "gripper_controller", "joint_state_broadcaster"]
    
    if hardware_interface == 'robot':
        mappings = {}
        controllers = ["joint_state_broadcaster"]

    return mappings, controllers


def _build_moveit_config(mappings: dict) -> MoveItConfigsBuilder:
    """Build MoveIt configuration with specified mappings."""
    return (
        MoveItConfigsBuilder("spot")
        .robot_description(
            file_path="config/spot.urdf.xacro",
            mappings=mappings,
        )
        .robot_description_semantic(file_path="config/spot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )


def _create_move_group_node(moveit_config) -> Node:
    """Create the MoveIt move_group node."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )


def _create_rviz_node(moveit_config) -> Node:
    """Create RViz2 node with MoveIt configuration."""
    pkg_share = get_package_share_directory("spot_moveit_config")
    rviz_config_path = Path(pkg_share) / "config" / "moveit.rviz"
    
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(rviz_config_path)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


def _create_robot_state_publisher(moveit_config) -> Node:
    """Create robot state publisher node."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


def _create_ros2_control_node() -> Node:
    """Create ros2_control node."""
    pkg_share = get_package_share_directory("spot_moveit_config")
    controllers_path = Path(pkg_share) / "config" / "ros2_controllers.yaml"
    
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(controllers_path)],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )


def _create_controller_spawners(controller_names: list[str]) -> list[ExecuteProcess]:
    """Create spawner processes for each controller."""
    return [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", name],
            output="log",
        )
        for name in controller_names
    ]


def launch_setup(context, *args, **kwargs):
    """Launch setup function that configures all nodes based on launch arguments."""
    hardware_interface = context.launch_configurations.get('hardware_interface', 'mock')
    
    # Determine configuration based on hardware interface
    mappings, controllers = _get_controller_config(hardware_interface)
    
    # Build MoveIt configuration
    moveit_config = _build_moveit_config(mappings)
    
    # Create all nodes
    nodes = [
        _create_rviz_node(moveit_config),
        _create_robot_state_publisher(moveit_config),
        _create_move_group_node(moveit_config),
        _create_ros2_control_node(),
    ]
    
    # Add controller spawners
    nodes.extend(_create_controller_spawners(controllers))
    
    return nodes


def generate_launch_description():
    """Generate the launch description for the Spot MoveIt configuration."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_interface',
            default_value='mock',
            description='Hardware interface type: "mock" for simulation, "robot" for real hardware'
        ),
        OpaqueFunction(function=launch_setup)
    ])