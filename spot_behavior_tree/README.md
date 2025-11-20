# Behaviors

This package demonstrates different spot behaviors using MoveIt. Documentation of the derived class methods for each ROS interface type can be found [here](../behaviortree_ros2/ros_behavior_wrappers.md).

# TreeExecutionServer

To start the Execution Server that load a list of plugins and BehaviorTrees from the [`bt_executor.yaml`](config/bt_executor.yaml) file:
``` bash
ros2 launch spot_behavior_tree bt_executor.launch.xml
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
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: bt1}"
```

