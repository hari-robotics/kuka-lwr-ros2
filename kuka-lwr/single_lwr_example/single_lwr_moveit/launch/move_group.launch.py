"""ROS 2 port of move_group.launch (planning_context, planning pipeline, trajectory execution)."""
import importlib.util
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _moveit_config():
    path = Path(get_package_share_directory('single_lwr_moveit')) / 'launch/moveit_config.py'
    spec = importlib.util.spec_from_file_location('single_lwr_moveit_config', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    config = _moveit_config()
    # ROS 1 move_group read the global /robot_description of the including launch; expand the
    # same robot (robot_name of the including launch) without a ros2_control block.
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', str(robot / 'robot') + '/',
        LaunchConfiguration('robot_name', default='single_lwr_robot'), '.urdf.xacro',
        ' ros2_control_hardware_type:=none use_stiffness_joints:=false']), value_type=str)
    # ROS 1 single_lwr.launch loaded t1_joint_limits.yaml into robot_description_planning
    t1_limits = LaunchConfiguration('t1_limits', default='false')

    def move_group(t1, condition):
        parameters = config.get_config(t1_limits=t1, with_description=False)
        parameters.update({
            'robot_description': robot_description,
            'allow_trajectory_execution': ParameterValue(LaunchConfiguration('allow_trajectory_execution'), value_type=bool),
            'max_safe_path_cost': ParameterValue(LaunchConfiguration('max_safe_path_cost'), value_type=float),
            'jiggle_fraction': ParameterValue(LaunchConfiguration('jiggle_fraction'), value_type=float),
            # MoveGroup capabilities to load (MoveIt 2 names: MoveGroupExecuteService is now
            # MoveGroupExecuteTrajectoryAction, MoveGroupPickPlaceAction does not exist anymore)
            'capabilities': ' '.join([
                'move_group/MoveGroupCartesianPathService',
                'move_group/MoveGroupExecuteTrajectoryAction',
                'move_group/MoveGroupKinematicsService',
                'move_group/MoveGroupMoveAction',
                'move_group/MoveGroupPlanService',
                'move_group/MoveGroupQueryPlannersService',
                'move_group/MoveGroupStateValidationService',
                'move_group/MoveGroupGetPlanningSceneService',
                'move_group/ClearOctomapService']),
            # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
            'publish_planning_scene': ParameterValue(LaunchConfiguration('publish_monitored_planning_scene'), value_type=bool),
            'publish_geometry_updates': ParameterValue(LaunchConfiguration('publish_monitored_planning_scene'), value_type=bool),
            'publish_state_updates': ParameterValue(LaunchConfiguration('publish_monitored_planning_scene'), value_type=bool),
            'publish_transforms_updates': ParameterValue(LaunchConfiguration('publish_monitored_planning_scene'), value_type=bool),
        })
        # Start the actual move_group node/action server
        return Node(name='move_group', package='moveit_ros_move_group', executable='move_group',
                    respawn=False, output='screen', condition=condition,
                    prefix=PythonExpression(["'gdb --ex run --args' if '", LaunchConfiguration('debug'),
                                             "'.lower() == 'true' else ''"]),
                    arguments=[PythonExpression(["'--debug' if '", LaunchConfiguration('info'),
                                                 "'.lower() == 'true' else ''"])],
                    # Set the display variable, in case OpenGL code is used internally
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
                    parameters=[parameters])

    return LaunchDescription([
        # GDB Debug Option
        DeclareLaunchArgument('debug', default_value='false'),
        # Verbose Mode Option
        DeclareLaunchArgument('info', default_value=LaunchConfiguration('debug')),
        # move_group settings
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true'),
        # TODO(ros2-migration): MoveIt 2 (Humble) has no fake controller manager; the offline
        # demo (demo.launch.py) executes on ros2_control mock hardware instead.
        DeclareLaunchArgument('fake_execution', default_value='false'),
        DeclareLaunchArgument('max_safe_path_cost', default_value='1'),
        DeclareLaunchArgument('jiggle_fraction', default_value='0.05'),
        DeclareLaunchArgument('publish_monitored_planning_scene', default_value='true'),

        move_group(False, UnlessCondition(t1_limits)),
        move_group(True, IfCondition(t1_limits)),
    ])
