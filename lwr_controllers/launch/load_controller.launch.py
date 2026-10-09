"""ROS 2 port of load_controller.launch: joint_state_controller plus one controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    manager = value('controller_manager')
    # As in ROS 1, joint_state_controller always accompanies the selected controller.
    active = list(dict.fromkeys(['joint_state_controller'] + value('controller').split()))
    stopped = [name for name in value('stopped_controllers').split() if name not in active]
    actions = [Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=active + ['-c', manager])]
    if stopped:
        # ROS 1 "spawner --stopped": load and configure without activating.
        actions.append(Node(package='controller_manager', executable='spawner', output='screen',
                            arguments=stopped + ['-c', manager, '--inactive']))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='joint_position_trajectory_controller',
                              description='Controller(s) to load and start'),
        DeclareLaunchArgument('stopped_controllers', default_value='',
                              description='Controllers to load without starting'),
        DeclareLaunchArgument('controller_manager', default_value='/lwr/controller_manager'),
        OpaqueFunction(function=_spawn)])
