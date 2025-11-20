from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Allow the demo to be specified via a launch argument
    demo_arg = DeclareLaunchArgument(
        "demo",
        default_value="move_to_pose",
        description="Name of the demo to run",
    )

    # Build MoveIt config for the Spot robot
    moveit_config = MoveItConfigsBuilder("spot").to_moveit_configs()

    # Node configuration using the provided demo
    move_group_demo = Node(
        name=LaunchConfiguration("demo"),
        package="demo",
        executable=LaunchConfiguration("demo"),
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )          

    return LaunchDescription([
        demo_arg,
        move_group_demo,
    ])
