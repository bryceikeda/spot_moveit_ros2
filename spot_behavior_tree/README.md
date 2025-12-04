# Behaviors

This package demonstrates different spot behaviors using MoveIt with both the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP.git) and [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2.git) libraries. 

## Running on the physical robot
Make sure the robot is in an open space if running on the physical robot.

After this, put your login credentials into a configuration file yaml ([spot_parameters.yaml](../spot_simple_controllers/config/spot_parameters.yaml)). The driver and controllers can be started via the following launchfile:
```bash
ros2 launch spot_simple_controllers spot_bringup.launch.py
```

## Running in simulation (Manipulation only)
```bash
ros2 launch spot_moveit_config moveit_spot.launch.py
```

# TreeExecutionServer

To start the Execution Server that load a list of plugins and BehaviorTrees from the [`bt_executor.yaml`](config/bt_executor.yaml) file:
``` bash
ros2 launch spot_behavior_tree bt_executor.launch.py
```

> *NOTE:* For documentation on the `yaml` parameters please see [bt_executor_parameters.md](config/bt_executor_parameters.md).

As the Server starts up it will print out the name of the ROS Action followed by the plugins and BehaviorTrees it was able to load.
```
[bt_action_server]: Loaded ROS Plugin: create_joint_state_plugin.so
[bt_action_server]: Loaded ROS Plugin: setup_move_group_plugin.so
[bt_action_server]: Loaded ROS Plugin: smash_door_plugin.so
[bt_action_server]: Loaded ROS Plugin: add_pose_stamped_to_vector_plugin.so
[bt_action_server]: Loaded ROS Plugin: get_current_planning_scene_plugin.so
[bt_action_server]: Loaded ROS Plugin: is_door_closed_plugin.so
[bt_action_server]: Loaded ROS Plugin: create_collision_object_from_solid_primitive_plugin.so
[bt_action_server]: Loaded ROS Plugin: plan_moveit_target_plugin.so
[bt_action_server]: Loaded ROS Plugin: execute_moveit_trajectory_plugin.so
[bt_action_server]: Loaded ROS Plugin: create_solid_primitive_box_plugin.so
[bt_action_server]: Loaded ROS Plugin: add_collision_object_to_planning_scene_plugin.so
[bt_action_server]: Loaded ROS Plugin: create_stamped_pose_plugin.so
[bt_action_server]: Loaded ROS Plugin: set_moveit_joint_value_target_plugin.so
[bt_action_server]: Loaded ROS Plugin: visualize_waypoints_plugin.so
```
To call the Action Server from the command line:
``` bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: MoveToPose}"
```

* [`MoveToPose`](behavior_trees/move_to_pose.xml): Move Spot's arm to a target pose. 
* [`MoveToJointSpaceGoal`](behavior_trees/move_to_joint_space_goal.xml): Move Spot's arm to a joint space goal. 
* [`CartesianMovementToWaypoints`](behavior_trees/cartesian_movement_to_waypoints.xml): Move Spot's arm to three waypoints using cartesian movements. 
* [`PathConstrainedMovement`](behavior_trees/path_constrained_movement.xml): Move Spot's arm to a target pose while adhering to specific path constraints.
* [`MoveToPoseAroundObstacle`](behavior_trees/move_to_pose_around_obstacle.xml): Move Spot's arm to a target pose around an obstacle.
* [`MoveObjectOverObstacle`](behavior_trees/move_object_over_obstacle.xml): Grab an object, move Spot's arm to a target pose around an obstacle, then let go of the object. 