"""Planning and execution demo with ros2_control's GenericSystem instead of ROS 1 fake controllers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = Path(get_package_share_directory('single_lwr_robot')) / 'launch/robot_bringup.launch.py'
    moveit = Path(get_package_share_directory('single_lwr_moveit')) / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={'use_lwr_sim': 'false', 'lwr_powered': 'false',
                              'stopped_controllers': ''}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'move_group.launch.py')),
            launch_arguments={'use_sim_time': 'false', 't1_limits': LaunchConfiguration('t1_limits')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'moveit_rviz.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'use_rviz': LaunchConfiguration('use_rviz'),
                              't1_limits': LaunchConfiguration('t1_limits')}.items())])
