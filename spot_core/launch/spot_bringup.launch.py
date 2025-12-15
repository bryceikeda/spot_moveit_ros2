import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the directory where this launch file is located
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # Path to the config file relative to the launch file
    config_file = os.path.join(current_dir, "..", "config", "spot_parameters.yaml")

    # Include the spot_driver launch file
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("spot_driver"), "launch", "spot_driver.launch.py"]
                )
            ]
        ),
        launch_arguments={"config_file": config_file}.items(),
    )

    # Include the spot_moveit_config launch file (delayed)
    spot_moveit_launch = TimerAction(
        period=5.0,  # Wait 5 seconds after spot_driver starts
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("spot_moveit_config"),
                                "launch",
                                "moveit_spot.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"hardware_interface": "robot"}.items(),
            )
        ],
    )

    # Launch the spot_core node (delayed)
    spot_control_node = TimerAction(
        period=5.0,  # Wait 5 seconds after spot_driver starts
        actions=[
            Node(
                package="spot_core",
                executable="spot_control_node",
                name="spot_control_node",
                output="screen",
                parameters=[config_file],
            )
        ],
    )

    return LaunchDescription(
        [spot_driver_launch, spot_moveit_launch, spot_control_node]
    )
