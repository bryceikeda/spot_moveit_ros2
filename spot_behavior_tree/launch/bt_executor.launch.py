from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Build MoveIt config for the Spot robot
    moveit_config = MoveItConfigsBuilder("spot").to_moveit_configs()

    bt_config = os.path.join(
    get_package_share_directory('spot_behavior_tree'),
    'config',
    'bt_executor.yaml'
    )

    # Node configuration
    spot_behavior_tree_executor = Node(
        package="spot_behavior_tree",
        executable="bt_executor",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            bt_config
        ],
    )

    return LaunchDescription([
        spot_behavior_tree_executor,
    ])
