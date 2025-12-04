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

    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Target X position in map frame"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Target Y position in map frame"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Target yaw in degrees"
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="vision", description="Target frame name"
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
            {"x": LaunchConfiguration("x")},
            {"y": LaunchConfiguration("y")},
            {"yaw": LaunchConfiguration("yaw")},
            {"frame_id": LaunchConfiguration("frame_id")},
        ],
    )

    return LaunchDescription(
        [
            demo_arg,
            x_arg,
            y_arg,
            yaw_arg,
            frame_id_arg,
            move_group_demo,
        ]
    )
