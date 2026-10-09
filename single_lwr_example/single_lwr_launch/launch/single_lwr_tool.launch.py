"""ROS 2 port of single_lwr_tool.launch, using the shared tool-aware bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    package = Path(get_package_share_directory('single_lwr_launch'))
    # Preserve the ROS 1 tool launch defaults. The shared assembly now includes
    # tool, tool_ee and tcp, mounted directly to world without the old pedestal.
    defaults = {
        'robot_name': 'single_lwr_robot',
        'use_lwr_sim': 'true',
        'lwr_powered': 'false',
        'port': '49938',
        'ip': '192.168.10.2',
        'update_rate': '200',
        'file': str(robot / 'config/980241-FRI-Driver.init'),
        't1_limits': 'false',
        'controllers': 'one_task_inverse_kinematics',
        'stopped_controllers': 'gravity_compensation_controller',
        'load_moveit': 'false',
        'use_rviz': 'true',
        'gui': 'true',
        'use_cartesian_position': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/single_lwr.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
