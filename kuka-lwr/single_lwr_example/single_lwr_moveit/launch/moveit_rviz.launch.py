"""ROS 2 port of moveit_rviz.launch."""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AnonName, Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('single_lwr_moveit'))
    robot = Path(get_package_share_directory('single_lwr_robot'))
    spec = importlib.util.spec_from_file_location('single_lwr_moveit_config', package / 'launch/moveit_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # ROS 1 loaded kinematics.yaml only; the MoveIt 2 RViz plugin also needs the robot model
    # parameters that ROS 1 read from the global parameter server.
    config = module.get_config(t1_limits=False, with_description=False)
    parameters = {name: config[name] for name in
                  ('robot_description_semantic', 'robot_description_kinematics', 'robot_description_planning')}
    parameters['robot_description'] = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', str(robot / 'robot') + '/',
        LaunchConfiguration('robot_name', default='single_lwr_robot'), '.urdf.xacro',
        ' ros2_control_hardware_type:=none use_stiffness_joints:=false']), value_type=str)
    launch_prefix = PythonExpression(["'gdb --ex run --args' if '", LaunchConfiguration('debug'),
                                      "'.lower() == 'true' else ''"])
    rviz = dict(name=AnonName('rviz'), package='rviz2', executable='rviz2', respawn=False, output='screen',
                prefix=launch_prefix, parameters=[parameters])
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('config', default_value='false'),
        # command_args: -d moveit.rviz when config is true
        Node(**rviz, condition=UnlessCondition(LaunchConfiguration('config'))),
        Node(**rviz, condition=IfCondition(LaunchConfiguration('config')),
             arguments=['-d', str(package / 'launch/moveit.rviz')]),
    ])
