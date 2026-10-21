"""ROS 2 port of demo.launch.

MoveIt 2 (Humble) has no fake controller manager: the ROS 1 fake controller
(fake_full_lwr_controller, joint states on /move_group/fake_controller_joint_states) is a
joint_trajectory_controller with that name on ros2_control mock_components/GenericSystem,
with a controller manager in the /move_group namespace, separate from the robot (/lwr).
"""
from pathlib import Path
import importlib.util
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch(context):
    robot = Path(get_package_share_directory('single_lwr_robot'))
    moveit = Path(get_package_share_directory('single_lwr_moveit'))
    spec = importlib.util.spec_from_file_location('single_lwr_moveit_config', moveit / 'launch/moveit_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = ET.fromstring(module.get_config()['robot_description'])
    # Substitute hardware only in this demo. Production models and controllers
    # keep the same interfaces and behavior regardless of how they are tested.
    control = ET.SubElement(description, 'ros2_control', name='LWR_demo', type='system')
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'mock_components/GenericSystem'
    config = yaml.safe_load((robot / 'config/controllers.yaml').read_text())
    fake = yaml.safe_load((moveit / 'config/fake_controllers.yaml').read_text())['controller_list'][0]
    # demo controller name -> robot controller whose configuration it uses
    names = {'joint_state_controller': 'joint_state_controller', fake['name']: 'joint_trajectory_controller'}
    manager = config['/lwr/controller_manager']['ros__parameters']
    demo_config = {'/move_group/controller_manager': {'ros__parameters': {
        'update_rate': manager['update_rate'], **{name: manager[source] for name, source in names.items()}}}}
    for name, source in names.items():
        demo_config['/move_group/' + name] = config['/lwr/' + source]
    for name in fake['joints']:
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='position')
        position = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(position, 'param', name='initial_value').text = '0.0'
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')
    description = ET.tostring(description, encoding='unicode')
    with tempfile.NamedTemporaryFile(mode='w', prefix='lwr_moveit_demo_', suffix='.yaml', delete=False) as output:
        yaml.safe_dump(demo_config, output, sort_keys=False)
        control_file = output.name
    # Given the published joint states, publish tf for the robot links
    return [Node(name='robot_state_publisher', package='robot_state_publisher', executable='robot_state_publisher',
                 respawn=True, output='screen', parameters=[{'robot_description': description}]),
            # We do not have a robot connected, so publish fake joint states
            Node(name='joint_state_publisher', package='joint_state_publisher', executable='joint_state_publisher',
                 parameters=[{'robot_description': description,
                              'source_list': ['/move_group/fake_controller_joint_states']}]),
            Node(package='controller_manager', executable='ros2_control_node', namespace='move_group',
                 parameters=[{'robot_description': description}, control_file], output='screen',
                 remappings=[('/move_group/joint_states', '/move_group/fake_controller_joint_states')]),
            *[Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=[name, '-c', '/move_group/controller_manager', '--controller-manager-timeout', '60'])
              for name in names],
            RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: Path(control_file).unlink(missing_ok=True)))]


def generate_launch_description():
    moveit = Path(get_package_share_directory('single_lwr_moveit')) / 'launch'
    return LaunchDescription([
        # By default, we do not start a database (it can be large)
        DeclareLaunchArgument('db', default_value='false'),
        # By default, we are not in debug mode
        DeclareLaunchArgument('debug', default_value='false'),
        # If database loading was enabled, start mongodb as well
        # ROS 2: default_warehouse_db.launch is not migrated (MoveIt 2 Humble has no
        # mongo_wrapper_ros.py); db:=true is refused instead of being ignored.
        LogInfo(condition=IfCondition(LaunchConfiguration('db')),
                msg='db:=true is not supported: the MoveIt warehouse database is not migrated to ROS 2'),
        Shutdown(condition=IfCondition(LaunchConfiguration('db')),
                 reason='db:=true is not supported (MoveIt warehouse database not migrated)'),
        OpaqueFunction(function=_launch),
        # Run the main MoveIt executable
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'move_group.launch.py')),
            launch_arguments={'allow_trajectory_execution': 'true', 'fake_execution': 'true',
                              'info': 'true', 'debug': LaunchConfiguration('debug')}.items()),
        # Run Rviz and load the default config to see the state of the move_group node
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'moveit_rviz.launch.py')),
            launch_arguments={'config': 'true', 'debug': LaunchConfiguration('debug')}.items())])
