# spot_simple_controllers
This package provides the core wrapper for interfacing between MoveIt 2 and the [spot_ros2](https://github.com/bdaiinstitute/spot_ros2) package. Relevant configuration parameters are located in the [spot_parameters.yaml](config/spot_parameters.yaml). In particular, you should set both the username, password, and hostname within that file. The driver and controllers can be started via the following launchfile:

```bash
ros2 launch spot_simple_controllers spot_bringup.launch.py
```