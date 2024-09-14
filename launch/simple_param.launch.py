from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        ),
        Node(
            package='lecture3_noise_generator',
            namespace='linear',
            executable='noise_generator',
            name='linear_noise',
            parameters=[
                {'rate': 10.0}
            ]
        ),
        Node(
            package='lecture3_noise_generator',
            namespace='angular',
            executable='noise_generator',
            name='angular_noise',
            parameters=[
                {'rate': 20.0}
            ]
        ),
        Node(
            package='lecture3_noise_generator',
            namespace='',
            executable='velocity_mux',
            name='mux',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')
            ],
            parameters=[
                {'rate': 30.0}
            ]
        ),
    ])