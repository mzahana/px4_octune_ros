# MAVLINK messages
The following are the MAVLink messages that are used to get the required control/sensor signals for control tuning

 ## Setpoint/target messages
* [`ACTUATOR_CONTROL_TARGET`](https://mavlink.io/en/messages/common.html#ACTUATOR_CONTROL_TARGET), with **MAVLink msg ID 140**
This has vehicle attitude and body angular rates.
    * MAVROS topic `/mavros/target_actuator_control` 

* [`ATTITUDE_TARGET`](https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET), with **MAVLink msg ID 83**
Reports the current commanded attitude of the vehicle as specified by the autopilot
    * MAVROS topic `/mavros/setpoint_raw/target_attitude`

* [`POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED), with **MAVLink msg ID 85**
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot
    * MAVROS topic `/mavros/setpoint_raw/target_local` 

## Feedback messages
* [`HIGHRES_IMU`](https://mavlink.io/en/messages/common.html#HIGHRES_IMU), with **MAVLink msg ID 105**
The IMU readings in SI units (angular velocty & linear acceleration).
    * MAVROS topic `/mavros/imu/data_raw`, this is Front-left-up body frame. Does not have attitude data
* [`ATTITUDE_QUATERNION`](https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION), with **MAVLink msg ID 31**
Has the vehicle attitude (quaternion), and angilar velocity
    * MAVROS topic `/mavros/imu/data`, in front-left-up bofy frame. This topic has attitude data
* [`LOCAL_POSITION_NED`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED), with **MAVLink mg ID 32**
Has the local frame position and velocity measurments
    * Local position MAVROS topic `mavros/local_position/pose`
    * Local velocity MAVROS topic `mavros/local_position/velocity_local`

# Change stream rate
Use `/mavros/set_message_interval` MAVROS service, [check here](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/sys_status.cpp#L1172), to increase mesage stream rates. **NOTE** This is constraiend by the link bandwidth

```
rosservice call /mavros/set_message_interval "message_id: 0
message_rate: 0.0"
```