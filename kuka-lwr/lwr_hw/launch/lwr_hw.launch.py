"""ROS 2 port of lwr_hw.launch.

The ROS 1 lwr_hw_fri_node (FRI hardware + its own ControllerManager loop) is replaced by
ros2_control_node loading the lwr_hw/LWRSystemHardware plugin. The name/port/ip/file
parameters become ros2_control hardware parameters of the expanded description.
"""
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def update_rate_file(context):
    """Parameter file setting update_rate of <name>/controller_manager.

    A dict in Node(parameters=...) is written under the /** wildcard. rcl merges the files per
    node key in order of first appearance and rclcpp applies the keys in that order; /** first
    appears with robot_description, so the full-name /lwr/controller_manager entry of
    controllers.yaml was applied after it and the launch argument was ignored (1000 Hz).
    The same full-name key, loaded after controllers_file, overrides it.
    """
    name = LaunchConfiguration('name').perform(context)
    rate = int(LaunchConfiguration('update_rate').perform(context))
    with tempfile.NamedTemporaryFile('w', prefix='lwr_update_rate_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump({f'/{name}/controller_manager': {'ros__parameters': {'update_rate': rate}}}, f)
    return f.name


def generate_launch_description():
    # the real hardware interface: expand the description with the FRI parameters
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', LaunchConfiguration('description_file'),
        ' ros2_control_hardware_type:=real',
        ' fri_port:=', LaunchConfiguration('port'),
        ' fri_ip:=', LaunchConfiguration('ip'),
        ' fri_init_file:=', LaunchConfiguration('file_with_path'),
        ' controller_update_rate:=', LaunchConfiguration('update_rate'),
        ' ', LaunchConfiguration('xacro_args')]), value_type=str)
    return LaunchDescription([
        # LAUNCH INTERFACE
        DeclareLaunchArgument('name', default_value='lwr',
                              description='The kuka lwr 4+ name. Ensure this is the same name you give to the arm in the urdf instance.'),
        DeclareLaunchArgument('port', default_value='49939'),
        DeclareLaunchArgument('ip', default_value='192.168.0.20'),
        DeclareLaunchArgument('file_with_path', default_value=''),
        # ROS 2: ROS 1 read robot_description and the controller parameters from the parameter
        # server and ran at the robot sample time; ros2_control_node needs them explicitly.
        DeclareLaunchArgument('description_file', default_value='',
                              description='URDF/xacro of the robot (ROS 1: robot_description parameter)'),
        DeclareLaunchArgument('xacro_args', default_value='',
                              description='Additional xacro arguments for description_file'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='ROS 2 controller_manager parameter file (ROS 1: rosparam load)'),
        DeclareLaunchArgument('update_rate', default_value='500',
                              description='Controller update rate, must match KRL friOpen() (2 ms -> 500 Hz)'),

        # LAUNCH IMPLEMENTATION

        # KUKA FRI
        OpaqueFunction(function=lambda context: [
            Node(package='controller_manager', executable='ros2_control_node', namespace=LaunchConfiguration('name'),
                 respawn=False, output='screen', on_exit=Shutdown(),
                 parameters=[{'robot_description': robot_description},
                             LaunchConfiguration('controllers_file'),
                             update_rate_file(context)])]),

        # STANDFORD FRIL
        # the real hardware interface (lwr_hw/LWRHWFRIL, built with -DLWR_HW_WITH_FRIL=ON) uses the
        # same node; its init file is the "file" hardware parameter (file_with_path).
    ])
