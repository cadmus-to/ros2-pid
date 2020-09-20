# ROS 2 PID - Params
This document goes over all launch parameters of the ROS 2 PID package.

## Required Params

| Name | Type     | Description                        |
| :--- | :------- | :--------------------------------- |
| kp   | `double` | Proportional gain, default = `0.0` |
| ki   | `double` | Integral gain, default = `0.0`     |
| kd   | `double` | Derivative gain, default = `0.0`   |

## Optional Params

| Name             | Type     | Description                                                                                                              |
| :--------------- | :------- | :----------------------------------------------------------------------------------------------------------------------- |
| plant_topic      | `string` | allows the user to configure an alternative topic name for the plant, default = `state`                                  |
| setpoint_topic   | `string` | allows the user to configure an alternative topic name for the setpoint, default = `setpoint`                            |
| enable_topic     | `string` | allows the user to configure an alternative topic name to turn the system on and of, default = `enable`                  |
| controller_topic | `string` | allows the user to configure an alternative topic name for the control effort, default = `control_effort`                |
| -                | -        | -                                                                                                                        |
| upper_limit      | `double` | maximum control effort value, default = `double_max`                                                                     |
| lower_limit      | `double` | minimum control effort value, default = `double_lowest`                                                                  |
| windup_limit     | `double` | limits the error increase or decrease for the integrator (mainly used for hardware restrictions), default = `double_max` |