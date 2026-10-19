"""ROS 2 port of controllers/joint_state_controller.launch."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Load joint controller configurations from YAML file to parameter server
        # TODO(ros2-migration): lwr_controllers/config/lwr_controllers.yaml is missing in ROS 1 as well.
        Node(namespace='lwr', name='controller_spawner', package='controller_manager', executable='spawner',
             respawn=False, output='screen', arguments=['joint_state_controller']),
    ])
