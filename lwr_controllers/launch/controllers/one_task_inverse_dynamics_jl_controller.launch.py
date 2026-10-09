from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='controller_manager', executable='spawner', output='screen',
        arguments=['one_task_inverse_dynamics_JL', '-c', '/lwr/controller_manager'])])
