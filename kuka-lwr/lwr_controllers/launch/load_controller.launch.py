"""ROS 2 port of load_controller.launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='joint_position_trajectoy_controller',
                              description='Name of the controller to be loaded (e.g. joint_trajectoy_controller)'),
        DeclareLaunchArgument('stopped_controllers', default_value='OneTaskInverseKinematics',
                              description='Additional controllers to be loaded, but not started '
                                          '(can be started later with controller_manager)'),

        # Load joint controller configurations from YAML file to parameter server
        # TODO(ros2-migration): ROS 1 loaded lwr_controllers/config/lwr_controllers.yaml here, a file
        # that is missing from the ROS 1 package as well. ROS 2 controller parameters belong to the
        # controller_manager (see single_lwr_robot/config/controllers.yaml).

        # shell=True lets the shell split the space separated controller lists, as roslaunch did
        Node(namespace='lwr', name='controller_spawner', package='controller_manager', executable='spawner',
             respawn=False, output='screen', shell=True,
             arguments=['joint_state_controller', LaunchConfiguration('controller')]),

        Node(namespace='lwr', name='controller_spawner_stopped', package='controller_manager', executable='spawner',
             respawn=False, output='screen', shell=True,
             arguments=['--stopped', LaunchConfiguration('stopped_controllers')]),
    ])
