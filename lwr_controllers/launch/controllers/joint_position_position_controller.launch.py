from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_position_position_controller', '-c', '/lwr/controller_manager'])])
