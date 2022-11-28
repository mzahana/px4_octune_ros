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

# Citation
If you use this work in your research, please cite the following reference.
```

@Article{s22239240,
AUTHOR = {Abdelkader, Mohamed and Mabrok, Mohamed and Koubaa, Anis},
TITLE = {OCTUNE: Optimal Control Tuning Using Real-Time Data with Algorithm and Experimental Results},
JOURNAL = {Sensors},
VOLUME = {22},
YEAR = {2022},
NUMBER = {23},
ARTICLE-NUMBER = {9240},
URL = {https://www.mdpi.com/1424-8220/22/23/9240},
ISSN = {1424-8220},
ABSTRACT = {Autonomous robots require control tuning to optimize their performance, such as optimal trajectory tracking. Controllers, such as the Proportional&ndash;Integral&ndash;Derivative (PID) controller, which are commonly used in robots, are usually tuned by a cumbersome manual process or offline data-driven methods. Both approaches must be repeated if the system configuration changes or becomes exposed to new environmental conditions. In this work, we propose a novel algorithm that can perform online optimal control tuning (OCTUNE) of a discrete linear time-invariant (LTI) controller in a classical feedback system without the knowledge of the plant dynamics. The OCTUNE algorithm uses the backpropagation optimization technique to optimize the controller parameters. Furthermore, convergence guarantees are derived using the Lyapunov stability theory to ensure stable iterative tuning using real-time data. We validate the algorithm in realistic simulations of a quadcopter model with PID controllers using the known Gazebo simulator and a real quadcopter platform. Simulations and actual experiment results show that OCTUNE can be effectively used to automatically tune the UAV PID controllers in real-time, with guaranteed convergence. Finally, we provide an open-source implementation of the OCTUNE algorithm, which can be adapted for different applications.},
DOI = {10.3390/s22239240}
}
```
