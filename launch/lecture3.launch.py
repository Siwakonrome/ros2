from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


EXE_VEL_MUX = 'velocity_mux'
EXE_NOISE = 'noise_generator'
PACKAGE_NAME = 'lecture3_noise_generator'


def generate_launch_description():

    launch_description = LaunchDescription()
    rate_mux = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value='5.0'
    )


    launch_description.add_action(rate_launch_arg)


    turtlesim = Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim')
    launch_description.add_action(turtlesim)

    rates = [10.0, 30.0]
    namespaces = ['linear', 'angular']
    for i, namespace in enumerate(namespaces):
        noise_gen = Node(
            package=PACKAGE_NAME,
            namespace=namespace,
            executable=EXE_NOISE,
            name=f'{namespace}_noise',
            parameters=[
                {'rate': rates[i]}
            ]
        )
        launch_description.add_action(noise_gen)

    vel_mux = Node(
            package=PACKAGE_NAME,
            namespace='',
            executable=EXE_VEL_MUX,
            name='mux',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')
            ],
            parameters=[
                {'rate': rate_mux}
            ])
    launch_description.add_action(vel_mux)
    
    return launch_description