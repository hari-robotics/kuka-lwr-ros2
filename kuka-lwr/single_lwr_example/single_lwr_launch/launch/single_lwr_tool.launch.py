"""ROS 2 port of single_lwr_tool.launch."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = Path(get_package_share_directory('single_lwr_robot'))
    package = Path(get_package_share_directory('single_lwr_launch'))
    moveit = Path(get_package_share_directory('single_lwr_moveit'))
    tool = Path(get_package_share_directory('lwr_robot_tool'))
    lwr_hw = Path(get_package_share_directory('lwr_hw'))
    ros_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    # robotHW configurations (ROS 1: rosparam load hw_interface.yaml -> /lwr/root, /lwr/tip)
    hw_interface = yaml.safe_load((robot / 'config/hw_interface.yaml').read_text())['lwr']
    # controller configurations (ROS 1: rosparam load controllers.yaml)
    controllers = str(robot / 'config/controllers.yaml')

    use_lwr_sim = LaunchConfiguration('use_lwr_sim')
    lwr_powered = LaunchConfiguration('lwr_powered')
    robot_xacro = [str(robot / 'robot') + '/', LaunchConfiguration('robot_name'), '.urdf.xacro']
    # ros2_control has to know which hardware backs the model; the rest of the model is unchanged
    hardware_type = PythonExpression([
        "'gazebo' if '", use_lwr_sim, "'.lower() in ('true', '1') else "
        "('real' if '", lwr_powered, "'.lower() in ('true', '1') else 'none')"])
    xacro_args = ' use_stiffness_joints:=false hw_root:={} hw_tip:={}'.format(hw_interface['root'], hw_interface['tip'])

    # the urdf/sdf parameter
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', str(tool / 'urdf/lwr_with_tool.urdf'),
        ' ros2_control_hardware_type:=', hardware_type,
        ' ros2_control_params_file:=', controllers,
        ' fri_port:=', LaunchConfiguration('port'),
        ' fri_ip:=', LaunchConfiguration('ip'),
        ' fri_init_file:=', LaunchConfiguration('file'),
        xacro_args]), value_type=str)

    return LaunchDescription([
        # LAUNCH INTERFACE

        # in case you have different robot configurations
        DeclareLaunchArgument('robot_name', default_value='single_lwr_robot'),

        # the default is the simulator
        DeclareLaunchArgument('use_lwr_sim', default_value='true'),

        # set the parameters for the real interface
        DeclareLaunchArgument('lwr_powered', default_value='false'),
        DeclareLaunchArgument('port', default_value='49938'),
        DeclareLaunchArgument('ip', default_value='192.168.10.2'),
        DeclareLaunchArgument('file', default_value=str(robot / 'config/980241-FRI-Driver.init')),

        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('controllers', default_value='one_task_inverse_kinematics'),
        DeclareLaunchArgument('stopped_controllers', default_value='gravity_compensation_controller'),

        # in case you want to load moveit from here, it might be hard with the real HW though
        DeclareLaunchArgument('load_moveit', default_value='false'),

        # set some ros tools
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),

        # ROS 2: ROS 1 ran the real robot at 1/getSampleTime(); ros2_control needs the rate
        # at launch. 500 Hz matches friOpen(2) of the bundled KRL.
        DeclareLaunchArgument('update_rate', default_value='500'),

        # LAUNCH IMPLEMENTATION

        # empty_world.launch set the global /use_sim_time for every node
        SetParameter(name='use_sim_time', value=True, condition=IfCondition(use_lwr_sim)),

        # joint and robot state publishers of the full robot description
        Node(name='joint_state_publisher', package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': robot_description, 'source_list': ['/lwr/joint_states']}]),
        Node(name='robot_state_publisher', package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             condition=UnlessCondition(use_lwr_sim)),
        # gz_ros2_control (Humble) reads the model from robot_state_publisher in its own /lwr
        # namespace only; keep the ROS 1 topics of the root-namespace publisher.
        Node(name='robot_state_publisher', namespace='lwr', package='robot_state_publisher',
             executable='robot_state_publisher', parameters=[{'robot_description': robot_description}],
             remappings=[('joint_states', '/joint_states'), ('robot_description', '/robot_description')],
             condition=IfCondition(use_lwr_sim)),

        Node(name='lwr_rviz', package='rviz2', executable='rviz2', respawn=False, output='screen',
             arguments=['-d', str(package / 'launch/rviz_config.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),

        # robot simulation uses the full robot description
        GroupAction(condition=IfCondition(use_lwr_sim), actions=[
            # spawn the robot in gazebo
            Node(name='spawn_urdf', package='ros_gz_sim', executable='create', respawn=False, output='screen',
                 parameters=[{'robot_description': robot_description}],
                 arguments=['-world', 'default', '-param', 'robot_description',
                            '-name', LaunchConfiguration('robot_name')]),

            # Gazebo Fortress replaces empty_world.launch (paused:=false -> -r, gui:=true)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ros_gz_sim / 'launch/gz_sim.launch.py')),
                launch_arguments={'gz_args': '-r -v 3 ' + str(robot / 'worlds/simple_environment.world')}.items()),
            # Gazebo Fortress does not publish /clock to ROS by itself
            Node(name='lwr_clock_bridge', package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 parameters=[{'config_file': str(robot / 'config/clock_bridge.yaml')}]),
        ]),

        # ROS 1: args="/lwr/joint_states" (subscribed topic); the global /robot_description and
        # /lwr/root, /lwr/tip (hw_interface.yaml) it read are passed as node parameters
        Node(name='cartesian_position_node', package='cartesian_position_node', executable='cartesian_position_node',
             arguments=['/lwr/joint_states'], respawn=False,
             parameters=[{'robot_description': robot_description}, {'lwr': hw_interface}]),

        # load moveit configuration
        GroupAction(condition=IfCondition(LaunchConfiguration('load_moveit')), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit / 'launch/move_group.launch.py')),
                launch_arguments={'allow_trajectory_execution': 'true', 'fake_execution': 'false',
                                  'info': 'true', 'debug': 'false'}.items()),

            # run Rviz and load the default config to see the state of the move_group node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit / 'launch/moveit_rviz.launch.py')),
                launch_arguments={'config': 'true', 'debug': 'false'}.items()),
        ]),

        # Load updated joint limits (override information from single_lwr_moveit) to respect T1 mode limits
        # ROS 2: move_group.launch.py reads t1_limits and loads config/t1_joint_limits.yaml itself

        # real robot and controllers
        GroupAction(condition=IfCondition(lwr_powered), actions=[
            # the real hardware interface expands its own copy of the robot description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lwr_hw / 'launch/lwr_hw.launch.py')),
                launch_arguments={'port': LaunchConfiguration('port'),
                                  'ip': LaunchConfiguration('ip'),
                                  'file_with_path': LaunchConfiguration('file'),
                                  # this name MUST match the name you give to the arm in the URDF instance
                                  'name': 'lwr',
                                  'description_file': robot_xacro,
                                  'xacro_args': xacro_args,
                                  'controllers_file': controllers,
                                  'update_rate': LaunchConfiguration('update_rate')}.items()),
        ]),

        # spawn only desired controllers in current namespace
        # shell=True lets the shell split the space separated controller lists, as roslaunch did
        Node(namespace='lwr', name='controller_spawner', package='controller_manager', executable='spawner',
             respawn=False, output='screen', shell=True,
             arguments=['joint_state_controller', 'arm_state_controller', LaunchConfiguration('controllers'),
                        '--controller-manager-timeout', '30']),

        Node(namespace='lwr', name='controller_stopper', package='controller_manager', executable='spawner',
             shell=True,
             arguments=['--stopped', LaunchConfiguration('stopped_controllers'),
                        '--controller-manager-timeout', '30']),
    ])
