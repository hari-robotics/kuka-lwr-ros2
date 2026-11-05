"""cartesian_position_node with the tooltip selected from config/tool_configuration.yaml."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # entry of config/tool_configuration.yaml
        DeclareLaunchArgument('tooltip', default_value='pin_cali_tooltip'),

        # robot model and kinematic chain, given by the launch file of the robot
        DeclareLaunchArgument('robot_description'),
        DeclareLaunchArgument('root'),
        DeclareLaunchArgument('tip'),

        # ROS 1: args="/lwr/joint_states" (subscribed topic); the global /robot_description and
        # /lwr/root, /lwr/tip (hw_interface.yaml) it read are passed as node parameters
        Node(name='cartesian_position_node', package='cartesian_position_node', executable='cartesian_position_node',
             arguments=['/lwr/joint_states'], respawn=False,
             parameters=[{'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str),
                          'lwr': {'root': LaunchConfiguration('root'), 'tip': LaunchConfiguration('tip')},
                          'tooltip': LaunchConfiguration('tooltip')}]),
    ])
