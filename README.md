# Overview

This repository integrates MoveIt 2 the Boston Dynamics Spot Quadruped robot. 

## Requirements
This repository is supported for use with Ubuntu 22.04 and [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) on both ARM64 and AMD64 platforms.

## Installation
Set up your ROS 2 workspace, and clone the repository in the `src` directory:
```bash
mkdir <ROS workspace> && cd <ROS workspace>
git clone https://github.com/bryceikeda/spot_moveit_ros2.git src
```

Next, run the following script to install the necessary Boston Dynamics packages (both Python and C++) and ROS dependencies.
The install script takes the optional argument ```--arm64```; it otherwise defaults to an AMD64 install.
```bash
./install_spot_ros2.sh
or
./install_spot_ros2.sh --arm64
source install/setup.bash
```

## Packages
* [`spot_core`](spot_core): Core wrapper for interfacing between MoveIt 2 and the [spot_ros2](https://github.com/bdaiinstitute/spot_ros2) package. 
* [`demo`](demo): Examples of how to control Spot via MoveIt. 
* [`spot_moveit_config`](spot_moveit_config): The Spot MoveIt configuration files. 
* [`spot_behavior_tree`](spot_behavior_tree): Control the spot robot via MoveIt and a behavior tree 

## Running these packages
See [`demo`](demo) for examples of how to control Spot via MoveIt. See [`spot_behavior_tree`](spot_behavior_tree) if you would like to control the robot using a behavior tree. 

## Acknowledgments
This repository was adapted from the ROS1 [spot_skills](https://github.com/Benned-H/spot_skills/tree/main) package written by Benned Hedegaard. 