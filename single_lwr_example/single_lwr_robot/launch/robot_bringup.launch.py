"""Shared robot and hardware bringup; higher-level launch files add MoveIt/RViz."""
from pathlib import Path
import fcntl
import os
import tempfile
from uuid import uuid4
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


class ControllerDumper(yaml.SafeDumper):
    # rcl_yaml_param_parser rejects YAML anchors and aliases.
    def ignore_aliases(self, data):
        return True


def _acquire_instance_lock(domain):
    # Gazebo partitions do not isolate /lwr ROS services, actions or /clock.
    # Keep the inode: unlinking a lock file can let two processes both own it.
    lock = open(Path(tempfile.gettempdir()) / f'single_lwr_{os.getuid()}_domain_{domain}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock.close()
        raise RuntimeError(
            f'An LWR bringup already owns ROS_DOMAIN_ID={domain}. '
            'Stop the previous launch with Ctrl+C before restarting. '
            'For independent simulations, set a different ROS_DOMAIN_ID in each terminal; '
            'gz_partition alone does not isolate ROS controllers.') from error
    return lock


def _existing_controller_manager(domain):
    # Also detect older launches that predate the lock, or standalone managers.
    import rclpy
    from rclpy.context import Context
    from controller_manager_msgs.srv import ListControllers

    ros_context = Context()
    node = None
    try:
        rclpy.init(args=[], context=ros_context, domain_id=domain)
        node = rclpy.create_node('lwr_bringup_check', context=ros_context,
                                enable_rosout=False, start_parameter_services=False)
        client = node.create_client(ListControllers, '/lwr/controller_manager/list_controllers')
        return client.wait_for_service(timeout_sec=1.0)
    finally:
        if node is not None:
            node.destroy_node()
        if ros_context.ok():
            ros_context.shutdown()


def _hardware(context):
    from launch.substitutions import LaunchConfiguration
    boolean = lambda key: LaunchConfiguration(key).perform(context).lower() in ('true', '1')
    selected = [name for name, key in (('gazebo', 'use_lwr_sim'), ('real', 'lwr_powered'),
                                       ('mock', 'use_mock_hardware')) if boolean(key)]
    if len(selected) > 1:
        raise ValueError('use_lwr_sim, lwr_powered and use_mock_hardware are mutually exclusive')
    # None matches ROS 1 with use_lwr_sim:=false lwr_powered:=false: no hardware
    # is started here and the spawners wait for an external /lwr/controller_manager
    # (e.g. lwr_hw.launch.py). Mock hardware is never selected implicitly.
    return selected[0] if selected else None


def _check_single_instance(context):
    domain = int(context.environment.get('ROS_DOMAIN_ID', '0'))
    lock = _acquire_instance_lock(domain)
    try:
        # An external controller manager is expected when this launch starts no hardware.
        if _hardware(context) is not None and _existing_controller_manager(domain):
            raise RuntimeError(
                f'/lwr/controller_manager already exists in ROS_DOMAIN_ID={domain}. '
                'Stop the previous robot/Gazebo launch with Ctrl+C before restarting. '
                'MoveIt could otherwise send trajectories to the previous instance.')
    except Exception:
        lock.close()
        raise
    # Hold the lock until the launch shuts down, including during model creation.
    return [RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: lock.close()))]


def _launch(context):
    from launch.substitutions import LaunchConfiguration
    value = lambda key: LaunchConfiguration(key).perform(context)
    boolean = lambda key: value(key).lower() in ('true', '1')
    robot = Path(get_package_share_directory('single_lwr_robot'))
    hardware = _hardware(context)
    simulation = hardware == 'gazebo'
    if hardware == 'real':
        try:
            get_package_share_directory('lwr_hw')
        except LookupError as error:
            raise RuntimeError('Build and source lwr_hw before selecting real hardware.') from error
    config = yaml.safe_load((robot / 'config/controllers.yaml').read_text())
    # The restored LWR Gazebo adapter adds gravity, matching the original FRI semantics.
    config['/lwr/gravity_compensation_controller']['ros__parameters']['hardware_gravity_compensation'] = hardware != 'mock'
    if hardware == 'real':
        config['/lwr/controller_manager']['ros__parameters']['update_rate'] = int(value('update_rate'))
    hw_config = yaml.safe_load((robot / 'config/hw_interface.yaml').read_text())['lwr']
    # Controllers are ROS 2 nodes with local parameters, not a shared ROS 1 parameter server.
    with tempfile.NamedTemporaryFile(mode='w', prefix='single_lwr_controllers_', suffix='.yaml', delete=False) as output:
        control_file = output.name
        description = xacro.process_file(str(robot / 'robot' / (value('robot_name') + '.urdf.xacro')),
            mappings={'ros2_control_hardware_type': hardware or 'real',
                      'ros2_control_params_file': control_file,
                      'use_stiffness_joints': 'false', 'fri_backend': value('fri_backend'),
                      'fri_port': value('port'), 'fri_ip': value('ip'), 'fri_init_file': value('file'),
                      'fri_receive_timeout_ms': value('receive_timeout_ms'),
                      'hw_root': hw_config['root'], 'hw_tip': hw_config['tip']}).toxml()
        for name, parameters in config.items():
            parameters['ros__parameters']['use_sim_time'] = simulation
            if name != '/lwr/controller_manager':
                parameters['ros__parameters']['robot_description'] = description
        yaml.dump(config, output, Dumper=ControllerDumper, sort_keys=False)
    # gz_ros2_control looks up the description parameter service in its /lwr namespace.
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                    name='robot_state_publisher', output='screen',
                    namespace='/lwr' if simulation else None,
                    parameters=[{'robot_description': description, 'use_sim_time': simulation}],
                    remappings=[('joint_states', '/lwr/joint_states'),
                                ('robot_description', '/robot_description')])]
    if simulation:
        gazebo = Path(get_package_share_directory('ros_gz_sim'))
        args = value('gz_args') + ('' if boolean('gui') else ' -s')
        # Fortress otherwise shares transport with every Gazebo started by this user.
        # Keep the server, GUI, model creation and clock bridge in the same isolated partition.
        actions.append(GroupAction([
                    SetEnvironmentVariable('IGN_PARTITION', value('gz_partition')),
                    SetEnvironmentVariable('GZ_PARTITION', value('gz_partition')),
                    LogInfo(msg='Gazebo transport partition: ' + value('gz_partition')),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gz_sim.launch.py')),
                        launch_arguments={'gz_args': args + ' ' + str(robot / 'worlds/simple_environment.world'),
                                          'on_exit_shutdown': 'true'}.items()),
                    Node(package='ros_gz_sim', executable='create', output='screen',
                         parameters=[{'robot_description': description}],
                         arguments=['-world', 'default', '-name', value('robot_name'), '-param', 'robot_description']),
                    Node(package='ros_gz_bridge', executable='parameter_bridge', name='lwr_clock_bridge',
                         output='screen', parameters=[{'config_file': str(robot / 'config/clock_bridge.yaml')}])]))
    elif hardware is not None:
        actions.append(Node(package='controller_manager', executable='ros2_control_node', namespace='lwr',
                            output='screen', parameters=[{'robot_description': description}, control_file]))
    else:
        actions.append(LogInfo(msg='No LWR hardware selected (use_lwr_sim:=false lwr_powered:=false); '
                                   'waiting for an external /lwr/controller_manager such as lwr_hw.launch.py.'))
    active = list(dict.fromkeys(['joint_state_controller', 'arm_state_controller'] + value('controllers').split()))
    inactive = value('stopped_controllers').split()
    if set(active) & set(inactive):
        raise ValueError('A controller cannot be both active and stopped')
    for name in active + inactive:
        if '/lwr/' + name not in config:
            raise ValueError('Unknown controller: ' + name)
        arguments = [name, '-c', '/lwr/controller_manager', '--controller-manager-timeout', '60']
        if name in inactive:
            arguments.append('--inactive')
        actions.append(Node(package='controller_manager', executable='spawner', output='screen', arguments=arguments))
    return actions


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    defaults = {'robot_name': 'single_lwr_robot', 'use_lwr_sim': 'true', 'lwr_powered': 'false',
                # Development aid only: ros2_control GenericSystem instead of a robot.
                'use_mock_hardware': 'false',
                'port': '49939', 'ip': '192.168.0.10', 'file': str(robot / 'config/980241-FRI-Driver.init'),
                'fri_backend': 'fri', 'update_rate': '500', 'receive_timeout_ms': '100',
                'controllers': 'joint_trajectory_controller',
                'stopped_controllers': 'gravity_compensation_controller one_task_inverse_kinematics',
                'gui': 'true', 'gz_args': '-r -v 3', 'gz_partition': 'single_lwr_' + uuid4().hex}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
                             + [OpaqueFunction(function=_check_single_instance),
                                OpaqueFunction(function=_launch)])
