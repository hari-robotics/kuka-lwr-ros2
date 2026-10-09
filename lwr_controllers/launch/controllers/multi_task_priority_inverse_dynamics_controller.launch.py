from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='controller_manager', executable='spawner', output='screen',
        arguments=['multi_task_priority_inverse_dynamics', '-c', '/lwr/controller_manager'])])
