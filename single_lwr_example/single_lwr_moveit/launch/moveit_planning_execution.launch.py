from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    launch = robot / 'launch/robot_bringup.launch.py'
    moveit = Path(get_package_share_directory('single_lwr_moveit')) / 'launch'
    defaults = {'sim': 'true', 'ip': '192.168.0.20', 'port': '49939', 't1_limits': 'false',
                'fril_init_file': str(robot / 'config/980241-FRI-Driver.init'), 'fri_backend': 'fri',
                'update_rate': '500', 'receive_timeout_ms': '100', 'use_rviz': 'true',
                'controllers': 'joint_trajectory_controller'}
    args = {'use_lwr_sim': LaunchConfiguration('sim'),
            'lwr_powered': PythonExpression(["'", LaunchConfiguration('sim'), "' == 'false'"]),
            'ip': LaunchConfiguration('ip'), 'port': LaunchConfiguration('port'),
            't1_limits': LaunchConfiguration('t1_limits'), 'file': LaunchConfiguration('fril_init_file'),
            'controllers': LaunchConfiguration('controllers'),
            'fri_backend': LaunchConfiguration('fri_backend'), 'update_rate': LaunchConfiguration('update_rate'),
            'receive_timeout_ms': LaunchConfiguration('receive_timeout_ms')}
    moveit_args = {'use_sim_time': LaunchConfiguration('sim'), 't1_limits': LaunchConfiguration('t1_limits'),
                   'use_rviz': LaunchConfiguration('use_rviz')}.items()
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments=args.items()),
           IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'move_group.launch.py')), launch_arguments=moveit_args),
           IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'moveit_rviz.launch.py')), launch_arguments=moveit_args)])
