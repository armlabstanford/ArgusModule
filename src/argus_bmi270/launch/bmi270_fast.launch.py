from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus_path', default_value='/dev/i2c-1'),
        DeclareLaunchArgument('i2c_addr', default_value='104'),
        DeclareLaunchArgument('topic', default_value='/imu0'),
        DeclareLaunchArgument('frame_id', default_value='imu'),
        DeclareLaunchArgument('odr_hz', default_value='200.0'),
        DeclareLaunchArgument('time_offset', default_value='0.0'),
        Node(
            package='argus_bmi270',
            executable='bmi270_fast_node',
            name='bmi270_fast_node',
            output='screen',
            parameters=[{
                'i2c_bus_path': LaunchConfiguration('i2c_bus_path'),
                'i2c_addr': ParameterValue(LaunchConfiguration('i2c_addr'), value_type=int),
                'topic': LaunchConfiguration('topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'odr_hz': ParameterValue(LaunchConfiguration('odr_hz'), value_type=float),
                'time_offset': ParameterValue(LaunchConfiguration('time_offset'), value_type=float),
            }],
        ),
    ])
