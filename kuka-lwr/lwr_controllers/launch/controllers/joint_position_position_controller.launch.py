"""ROS 2 port of controllers/joint_position_position_controller.launch."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = Path(get_package_share_directory('lwr_controllers'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/load_controller.launch.py')),
            launch_arguments={'controller': 'joint0_position_position_controller joint1_position_position_controller joint2_position_position_controller joint3_position_position_controller joint4_position_position_controller joint5_position_position_controller joint6_position_position_controller'}.items()),
    ])
