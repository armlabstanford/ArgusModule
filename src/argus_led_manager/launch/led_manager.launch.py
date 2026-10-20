import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Namespace defaults to /<hostname>, matching the camera, IMU, and OpenVINS nodes.
        DeclareLaunchArgument('namespace', default_value='/' + socket.gethostname()),
        # BCM GPIO numbers. -1 leaves that LED disabled.
        DeclareLaunchArgument('led1_gpio', default_value='24'),
        DeclareLaunchArgument('led2_gpio', default_value='25'),
        DeclareLaunchArgument('timeout_s', default_value='1.0'),
        Node(
            package='argus_led_manager',
            executable='led_manager',
            name='led_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'led1_gpio': ParameterValue(LaunchConfiguration('led1_gpio'), value_type=int),
                'led2_gpio': ParameterValue(LaunchConfiguration('led2_gpio'), value_type=int),
                'timeout_s': ParameterValue(LaunchConfiguration('timeout_s'), value_type=float),
            }],
        ),
    ])
