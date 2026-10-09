"""Run one selected trajectory node against an already running robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch(context):
    selected = LaunchConfiguration('trajectory_node').perform(context)
    parameters = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    if selected == 'tool_relative_motion_node':
        parameters['monitor_only'] = ParameterValue(LaunchConfiguration('monitor_only'), value_type=bool)
    params_file = LaunchConfiguration('params_file').perform(context)
    return [Node(package='ultrasound_trajectory', executable=selected, output='screen',
                 parameters=([params_file] if params_file else []) + [parameters])]


def generate_launch_description():
    nodes = [
        'ultrasound_traj_node', 'linear_traj_node', 'linear_traj_node_Luca',
        'pose_initialization', 'ultrasound_traj_cartesian_node', 'pose_init',
        'ee_incremental_motion', 'ee_incremental_impedance_node', 'tool_relative_motion_node',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('trajectory_node', default_value='tool_relative_motion_node', choices=nodes),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('monitor_only', default_value='true',
                             description='Applies only to tool_relative_motion_node; suppresses command publication.'),
        DeclareLaunchArgument('params_file', default_value='', description='Optional ROS 2 parameter YAML file.'),
        OpaqueFunction(function=_launch),
    ])
