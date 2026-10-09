"""Launch only the receive-only FRI state node; no hardware/control activation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    profile = Path(get_package_share_directory('lwr_hw')) / 'config/fri_udp_only.xml'
    return LaunchDescription([
        DeclareLaunchArgument('bind_ip', default_value='192.168.10.1'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.10.2'),
        DeclareLaunchArgument('port', default_value='49938'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile)),
        Node(package='lwr_hw', executable='fri_state_receiver.py', output='screen',
             parameters=[{
                 'bind_ip': LaunchConfiguration('bind_ip'),
                 'robot_ip': LaunchConfiguration('robot_ip'),
                 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
             }]),
    ])
