from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable



PACKAGE_NAME = 'lecture3_interfaces'


def generate_launch_description():

    mean_linear, variance_linear = 1.0, 0.1
    mean_angular, variance_angular = 0.0, 3.0

    ld = LaunchDescription()
    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/linear/set_noise ",
                f"{PACKAGE_NAME}/srv/SetNoise ",
                f'"{{mean: {{data: {mean_linear}}}, variance: {{data: {variance_linear}}}}}"',
            ]],
            shell=True
        )
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/angular/set_noise ",
                f"{PACKAGE_NAME}/srv/SetNoise ",
                f'"{{mean: {{data: {mean_angular}}}, variance: {{data: {variance_angular}}}}}"',
            ]],
            shell=True
        )
    )
    return ld









