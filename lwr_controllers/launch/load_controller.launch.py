from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('controller', default_value='joint_trajectory_controller'),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[LaunchConfiguration('controller'), '-c', '/lwr/controller_manager'])])
