from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _launch(context):
    from launch.substitutions import LaunchConfiguration
    boolean = lambda key: LaunchConfiguration(key).perform(context).lower() == 'true'
    path = Path(get_package_share_directory('single_lwr_moveit')) / 'launch/moveit_config.py'
    spec = importlib.util.spec_from_file_location('single_lwr_moveit_config', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hardware = 'gazebo' if boolean('use_sim_time') else ('mock' if boolean('use_mock_hardware') else 'real')
    config = module.get_config(hardware, boolean('t1_limits'))
    config.update(use_sim_time=boolean('use_sim_time'), allow_trajectory_execution=boolean('allow_trajectory_execution'))
    return [Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=[config], remappings=[('joint_states', '/lwr/joint_states')])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
                              DeclareLaunchArgument('t1_limits', default_value='false'),
                              DeclareLaunchArgument('use_mock_hardware', default_value='false'),
                              DeclareLaunchArgument('allow_trajectory_execution', default_value='true'),
                              OpaqueFunction(function=_launch)])
