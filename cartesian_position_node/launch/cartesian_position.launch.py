"""Read the existing robot description and joint states; publish Cartesian state."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'joint_states_topic': '/lwr/joint_states',
        'robot_description_topic': '/robot_description',
        'root_link': 'lwr_base_link',
        'end_effector_link': 'lwr_7_link',
        'tooltip_link': 'tcp',
        'velocity_filter_alpha': '0.7',
        'use_sim_time': 'false',
    }
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
                  for key in defaults if key not in ('velocity_filter_alpha', 'use_sim_time')}
    parameters['velocity_filter_alpha'] = ParameterValue(LaunchConfiguration('velocity_filter_alpha'), value_type=float)
    parameters['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()] +
        [Node(package='cartesian_position_node', executable='cartesian_position_node',
              output='screen', parameters=[parameters])])
