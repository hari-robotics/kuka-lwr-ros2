from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    package = Path(get_package_share_directory('single_lwr_moveit'))
    spec = importlib.util.spec_from_file_location('single_lwr_moveit_config', package / 'launch/moveit_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    boolean = lambda name: LaunchConfiguration(name).perform(context).lower() in ('true', '1')
    simulation = boolean('use_sim_time')
    return [Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(package / 'launch/moveit.rviz')],
             parameters=[module.get_config('gazebo' if simulation else 'mock', boolean('t1_limits')),
                         {'use_sim_time': simulation}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'), OpaqueFunction(function=_launch)])
