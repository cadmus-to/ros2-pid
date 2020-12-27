# ROS 2 PID

PID controller for ROS2

## Installation
To use the library effectivly, we will have to make use of vcs-tool. 
The following steps need to be taken:
```sh
mkdir -p ~/jlb_pid_ws/src
cd ~/jlb_pid_ws
wget https://gitlab.com/Juulbl/ros2-pid/-/raw/ros2/jlb_pid.repos
vcs import src < jlb_pid.repos
colcon build --symlink-install

# Add them to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.):
echo 'source ~/jlb_pid_ws/install/setup.sh' >> ~/.bashrc
```

Once this is set up, we will need to setup the dependencies of the package.

`package.xml`:
```xml
<!-- As a generic dependency -->
<depend>jlb_pid</depend>

<!-- Or, as an exec dependency -->
<exec_depend>jlb_pid</exec_depend>
```

`CMakeLists.txt`:
```cmake
find_package(jlb_pid REQUIRED)
```


## Documentation
This section will go over the interaction with the PID node.

### Topics

| Name           | Type               | Description                                                        |
| :------------- | :----------------- | :----------------------------------------------------------------- |
| state          | `std_msgs/Float64` | input data used for the PID system, e.g. current angle             |
| setpoint       | `std_msgs/Float64` | the setpoint of the PID system, e.g. target angle                  |
| enable         | `std_msgs/Bool`    | if true, updates the PID system. If false, disables the PID system |
| control_effort | `std_msgs/Float64` | the control effort, or output of the PID system                    |


### Parameters
#### Required

| Name | Type     | Description                        |
| :--- | :------- | :--------------------------------- |
| kp   | `double` | Proportional gain, default = `0.0` |
| ki   | `double` | Integral gain, default = `0.0`     |
| kd   | `double` | Derivative gain, default = `0.0`   |

> Note: `kp`, `ki` and `kd` must all be either positive or negative (a mixture of signs results in an error)

#### Optional

| Name             | Type     | Description                                                                                                              |
| :--------------- | :------- | :----------------------------------------------------------------------------------------------------------------------- |
| plant_topic      | `string` | allows the user to configure an alternative topic name for the plant, default = `state`                                  |
| setpoint_topic   | `string` | allows the user to configure an alternative topic name for the setpoint, default = `setpoint`                            |
| enable_topic     | `string` | allows the user to configure an alternative topic name to turn the system on and of, default = `enable`                  |
| controller_topic | `string` | allows the user to configure an alternative topic name for the control effort, default = `control_effort`                |
| -                | -        | -                                                                                                                        |
| update_rate      | `double` | update rate of the node expressed in Hz, default = `1000`                                                                |
| -                | -        | -                                                                                                                        |
| upper_limit      | `double` | maximum control effort value, default = `double_max`                                                                     |
| lower_limit      | `double` | minimum control effort value, default = `double_lowest`                                                                  |
| windup_limit     | `double` | limits the error increase or decrease for the integrator (mainly used for hardware restrictions), default = `double_max` |

## Examples
A set of example projects have been created and can be found [here][example_repo].

## Usage
To setup this PID control library, 
add a new node to the launch file.

The following snippet is an example of a node.
In this example we specify that we want to use the `pid` package.
We will add the node to the namespace `example/distance_controller`, 
such that every topic will be append after it.
We want to use the executable of the control node, using `controller_node`.
Finally, we will specify all launch parameters.

```py
Node(
    package='jlb_pid',
    namespace='example/distance_controller',
    executable='controller_node',
    name='distance_controller',
    parameters=[
        # Required
        {'kp': 2.0},
        {'ki': 0.01},
        {'kd': 0.02},
        # Optional
        {'upper_limit': 0.6},
        {'lower_limit': -0.6},
        {'windup_limit': 0.001}
    ]
)

```


[example_repo]: https://gitlab.com/Larsbl00/ros2-pid-examples "ROS 2 PID | Examples"
