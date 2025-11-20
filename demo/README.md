# Demo

This repository provides examples of how to interface with the Spot robot using MoveIt 2. 

Before running any of these nodes, make sure that the robot is in an open space and that you've built and sourced your ROS 2 workspace. 
```bash
cd <ros2_workspace>
colcon build --symlink-install
source install/setup.bash
```
After this, put your login credentials into a configuration file yaml ([spot_parameters.yaml](../spot_simple_controllers/config/spot_parameters.yaml)). The driver and controllers can be started via the following launchfile:
```bash
ros2 launch spot_simple_controllers spot_bringup.launch.py
```

## Examples
Once the driver has been started, different examples demonstrating how to interface with MoveIt can be run with the following command:
```bash
ros2 launch demo demo.launch.py demo:=<example_node>
```

* [`move_to_pose`](src/move_to_pose.cpp): Move Spot's arm to a target pose. 
* [`move_to_joint_space_goal`](src/move_to_joint_space_goal.cpp): Move Spot's arm to a joint space goal. 
* [`cartesian_movement_to_waypoints`](src/cartesian_movement_to_waypoints.cpp): Move Spot's arm to three waypoints using cartesian movements. 
* [`path_constrained_movement`](src/path_constrained_movement.cpp): Move Spot's arm to a target pose while adhering to specific path constraints.
* [`move_to_pose_around_obstacle`](src/move_to_pose_around_obstacle.cpp): Move Spot's arm to a target pose around an obstacle.
* [`move_object_over_obstacle`](src/move_object_over_obstacle.cpp): Grab an object, move Spot's arm to a target pose around an obstacle, then let go of the object. 

## Note
This package provides examples for interfacing with MoveIt via C++ but not Python. This is because [moveit_py](https://github.com/moveit/moveit2/tree/main/moveit_py) is not provided in ROS 2 Humble. However, Python functionality is possible if the [MoveIt 2 source installation](https://moveit.ai/install-moveit2/source/) is utilized.