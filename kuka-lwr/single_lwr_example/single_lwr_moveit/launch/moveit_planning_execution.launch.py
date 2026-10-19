"""ROS 2 port of moveit_planning_execution.launch.

The planning and execution components of MoveIt! configured to run
using the ROS-Industrial interface.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    launch = Path(get_package_share_directory('single_lwr_launch'))
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        # the "sim" argument controls whether we connect to a Simulated or Real robot
        #  - if sim=false, a robot_ip argument is required
        DeclareLaunchArgument('sim', default_value='true', description='If true, the robot will be simulated in Gazebo'),
        DeclareLaunchArgument('ip', default_value='192.168.0.20', description='The IP address of the robot'),
        DeclareLaunchArgument('port', default_value='49939', description='The listening port of the FRI interface'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('fril_init_file', default_value=str(robot / 'config/980241-FRI-Driver.init')),
        DeclareLaunchArgument('controllers', default_value='joint_trajectory_controller'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch / 'launch/single_lwr.launch.py')),
            launch_arguments={
                'use_lwr_sim': sim,
                'lwr_powered': PythonExpression(["'false' if '", sim, "'.lower() == 'true' else 'true'"]),
                'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
                't1_limits': LaunchConfiguration('t1_limits'),
                'file': LaunchConfiguration('fril_init_file'),
                'load_moveit': 'true',
                'controllers': LaunchConfiguration('controllers'),
            }.items()),
    ])
