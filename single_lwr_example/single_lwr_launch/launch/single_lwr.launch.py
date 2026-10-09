"""Single LWR bringup with optional MoveIt and RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    package = Path(get_package_share_directory('single_lwr_launch'))
    moveit = Path(get_package_share_directory('single_lwr_moveit'))
    source = lambda path: PythonLaunchDescriptionSource(str(path))
    moveit_args = {'use_sim_time': LaunchConfiguration('use_lwr_sim'),
                   't1_limits': LaunchConfiguration('t1_limits')}.items()
    return LaunchDescription([
        DeclareLaunchArgument('load_moveit', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value=LaunchConfiguration('load_moveit')),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('use_cartesian_position', default_value='true'),
        IncludeLaunchDescription(source(robot / 'launch/robot_bringup.launch.py')),
        Node(package='cartesian_position_node', executable='cartesian_position_node',
             output='screen', condition=IfCondition(LaunchConfiguration('use_cartesian_position')),
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_lwr_sim'), value_type=bool)}]),
        IncludeLaunchDescription(source(moveit / 'launch/move_group.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('load_moveit')),
                                 launch_arguments=moveit_args),
        IncludeLaunchDescription(source(moveit / 'launch/moveit_rviz.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('load_moveit')),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_lwr_sim'),
                                     't1_limits': LaunchConfiguration('t1_limits'),
                                     'use_rviz': LaunchConfiguration('use_rviz')}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_rviz'),
                 "'.lower() in ('true', '1') and '", LaunchConfiguration('load_moveit'),
                 "'.lower() not in ('true', '1')"])),
             arguments=['-d', str(package / 'launch/rviz_config.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_lwr_sim'), value_type=bool)}])
    ])
