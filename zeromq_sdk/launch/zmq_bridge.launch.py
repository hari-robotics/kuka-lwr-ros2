from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pub_address', default_value='tcp://*:5555'),
        DeclareLaunchArgument('sub_address', default_value='tcp://*:5556'),
        Node(
            package='zeromq_sdk',
            executable='zmq_bridge',
            name='zmq_bridge',
            output='screen',
            parameters=[{
                'pub_address': LaunchConfiguration('pub_address'),
                'sub_address': LaunchConfiguration('sub_address'),
            }],
        ),
    ])
