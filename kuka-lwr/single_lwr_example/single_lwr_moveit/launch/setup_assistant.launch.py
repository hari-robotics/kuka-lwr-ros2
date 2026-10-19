"""ROS 2 port of setup_assistant.launch.

Re-launch the MoveIt Setup Assistant with this configuration package already loaded.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Debug Info
        DeclareLaunchArgument('debug', default_value='false'),
        # Run
        Node(name='moveit_setup_assistant', package='moveit_setup_assistant', executable='moveit_setup_assistant',
             arguments=['--config_pkg=single_lwr_moveit'], output='screen',
             prefix=PythonExpression(["'gdb --ex run --args' if '", LaunchConfiguration('debug'),
                                      "'.lower() == 'true' else ''"])),
    ])
