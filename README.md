# ROS 2 PID

PID controller for ROS2

## Documentation
All documentation can be found in the `doc` directory.
For convenience, you can use the following items to navigate:

- [Interfaces](doc/interfaces.md)
- [Launch Parameters](doc/params.md)

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
        {'kp': 2.0},
        {'ki': 0.01},
        {'kd': 0.02},
        {'upper_limit': 0.6},
        {'lower_limit': -0.6},
        {'windup_limit': 0.001}
    ]
)

```


[example_repo]: https://gitlab.com/Larsbl00/ros2-pid-examples "ROS 2 PID | Examples"