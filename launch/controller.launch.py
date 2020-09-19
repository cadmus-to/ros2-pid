from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pid',
            namespace='pid',
            executable='controller_node',
            name='pid_node',
            parameters=[
                {'kp': 1.0},
                {'ki': 0.5},
                {'kd': 0.5}
            ]
        )
    ])
