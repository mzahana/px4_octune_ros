# px4_octune_ros
A ROS package for interfacing [OCTUNE algorithm](https://github.com/mzahana/octune) with PX4 autopilot firmware. This package allows using OCTUNE to automatically tune PX4 PID controllers, using ROS.

# Videos
* Simulations: https://youtu.be/OY9XY9CdGhA
* Real experiments: https://youtu.be/a3mrDvK2b-c
# Installation
* This is tested with Ubuntu 18 and ROS Melodic
* To install, you can use the `scripts/setup.sh` script, which shows also installs the required dependencies including the core [OCTUNE package](https://github.com/mzahana/octune)

# Test
* To quicklu test the package in Gazebo simulation, you can use the `example.launch` file
  ```bash
  roslaunch px4_octune_ros example.launch
  ```
 * Run QGroundControl (QGC) software, and issue takeoff command
 * Use the Parameters Tab QGC in order to change the PID gains. For example, Increase the `MC_ROLLRATE_P` gains to introduce high frequency oscillations.
 * Start the OCTUNE state machine
  ```bash
  rosservice call /px4_tuner_node/px4_octune/start_tune
  ```
 * To stop the tuning process
  ```bash
  rosservice call /px4_tuner_node/px4_octune/stop_tune
  ```
 * To plot figures after tuning (can not be done on a real drone, but you can save figures)
  ```bash
  rosservice call /px4_tuner_node/px4_octune/plot
  ```
 * To save plot to the predefine path in [config/tuning_params.yaml](https://github.com/mzahana/px4_octune_ros/blob/main/config/tuning_params.yaml#L4)
  ```bash
  rosservice call /px4_tuner_node/px4_octune/save_plot
  ```
  
  # Run on hardware
  * You can use the [launch/run_on_hardware.launch](https://github.com/mzahana/px4_octune_ros/blob/main/launch/run_on_hardware.launch) to run on an onboard computer connected to a PX4 autopilot
  * You will need to set the following environment variables (for example in your `~/.bashrc` file)
    * `RUN_MAVROS=True`
    * `RUN_OCTUNE=True`
    * `OCTUNE_PARAMS_YAML=path_to_octune_yaml file` Similar to the [config/tuning_params.yaml](https://github.com/mzahana/px4_octune_ros/blob/main/config/tuning_params.yaml)
