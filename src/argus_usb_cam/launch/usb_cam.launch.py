from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fourcc', default_value='MJPG'),
        DeclareLaunchArgument('open_fps', default_value='100.0'),
        DeclareLaunchArgument('target_fps', default_value='20.0'),
        DeclareLaunchArgument('grayscale', default_value='true'),
        DeclareLaunchArgument('camera_frame_id', default_value='cam0'),
        DeclareLaunchArgument('timeshift_cam_imu', default_value='0.0'),
        DeclareLaunchArgument('exposure_seconds', default_value='0.0'),
        DeclareLaunchArgument('use_mid_exposure_stamp', default_value='false'),
        DeclareLaunchArgument('buffers', default_value='1'),
        Node(
            package='argus_usb_cam',
            executable='usb_cam_node',
            name='usb_cam_cpp',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                'fourcc': LaunchConfiguration('fourcc'),
                'open_fps': ParameterValue(LaunchConfiguration('open_fps'), value_type=float),
                'target_fps': ParameterValue(LaunchConfiguration('target_fps'), value_type=float),
                'grayscale': ParameterValue(LaunchConfiguration('grayscale'), value_type=bool),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'timeshift_cam_imu': ParameterValue(
                    LaunchConfiguration('timeshift_cam_imu'), value_type=float),
                'exposure_seconds': ParameterValue(
                    LaunchConfiguration('exposure_seconds'), value_type=float),
                'use_mid_exposure_stamp': ParameterValue(
                    LaunchConfiguration('use_mid_exposure_stamp'), value_type=bool),
                'buffers': ParameterValue(LaunchConfiguration('buffers'), value_type=int),
            }],
        ),
    ])
