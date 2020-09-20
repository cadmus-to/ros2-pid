# ROS 2 PID - Interfaces
This document goes over all interfaces present in the ROS 2 PID package.

## Topics

| Name           | Type               | Description                                                        |
| :------------- | :----------------- | :----------------------------------------------------------------- |
| state          | `std_msgs/Float64` | input data used for the PID system, e.g. current angle             |
| setpoint       | `std_msgs/Float64` | the setpoint of the PID system, e.g. target angle                  |
| enable         | `std_msgs/Bool`    | if true, updates the PID system. If false, disables the PID system |
| control_effort | `std_msgs/Float64` | the control effort, or output of the PID system                    |