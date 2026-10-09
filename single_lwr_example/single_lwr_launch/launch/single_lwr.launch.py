"""ROS 2 port of single_lwr.launch: robot bringup with optional MoveIt and RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    package = Path(get_package_share_directory('single_lwr_launch'))
    moveit = Path(get_package_share_directory('single_lwr_moveit'))
    source = lambda path: PythonLaunchDescriptionSource(str(path))
    use_sim_time = ParameterValue(LaunchConfiguration('use_lwr_sim'), value_type=bool)
    moveit_args = {'use_sim_time': LaunchConfiguration('use_lwr_sim'),
                   't1_limits': LaunchConfiguration('t1_limits'),
                   'use_mock_hardware': LaunchConfiguration('use_mock_hardware')}
    return LaunchDescription([
        # Same defaults as ROS 1; hardware arguments are declared by robot_bringup.
        DeclareLaunchArgument('load_moveit', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        # Workspace addition (not in ROS 1); single_lwr_tool.launch.py enables it.
        DeclareLaunchArgument('use_cartesian_position', default_value='false'),
        IncludeLaunchDescription(source(robot / 'launch/robot_bringup.launch.py')),
        # ROS 1 joint_state_publisher: republish /lwr/joint_states on /joint_states.
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'source_list': ['/lwr/joint_states'], 'use_sim_time': use_sim_time}],
             remappings=[('robot_description', '/robot_description')]),
        Node(package='cartesian_position_node', executable='cartesian_position_node',
             output='screen', condition=IfCondition(LaunchConfiguration('use_cartesian_position')),
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='rviz2', executable='rviz2', name='lwr_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(package / 'launch/rviz_config.rviz')],
             parameters=[{'use_sim_time': use_sim_time}]),
        IncludeLaunchDescription(source(moveit / 'launch/move_group.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('load_moveit')),
                                 launch_arguments=moveit_args.items()),
        # As in ROS 1, MoveIt always brings its own RViz, independent of use_rviz.
        IncludeLaunchDescription(source(moveit / 'launch/moveit_rviz.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('load_moveit')),
                                 launch_arguments={**moveit_args, 'use_rviz': 'true'}.items()),
    ])
