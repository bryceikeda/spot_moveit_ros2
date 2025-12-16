# spot_core
This package provides the core wrapper for interfacing between MoveIt 2 and the [spot_ros2](https://github.com/bdaiinstitute/spot_ros2) package. Relevant configuration parameters are located in the [spot_parameters_example.yaml](config/spot_parameters_example.yaml). In particular, you should set both the username, password, and hostname within that file, then **change the name to spot_parameters.yaml**. The driver and controllers can be started via the following launchfile:

```bash
ros2 launch spot_core spot_bringup.launch.py
```

# Service calls
Certain functions can also be called via the command line using service calls. These include the following: 

```bash
ros2 service call "spot/stand" std_srvs/srv/Trigger {}
ros2 service call "spot/sit" std_srvs/srv/Trigger {}
ros2 service call "spot/deploy_arm" std_srvs/srv/Trigger {}
ros2 service call "spot/stow_arm" std_srvs/srv/Trigger {}
ros2 service call "spot/shutdown" std_srvs/srv/Trigger {}
```