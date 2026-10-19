"""ROS 2 port of controllers/multi_task_priority_inverse_dynamics_controller.launch."""
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
            launch_arguments={'controller': 'MultiTaskPriorityInverseDynamics'}.items()),
    ])
