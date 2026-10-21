"""ROS 2 port of kuka_launch/launch/impedance_simulation.launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # LAUNCH INTERFACE
        # Arguments
        DeclareLaunchArgument('dt', default_value='0.001'),
        DeclareLaunchArgument('Fd', default_value='5.0'),
        DeclareLaunchArgument('nb_points', default_value='5'),
        DeclareLaunchArgument('coeff_classification', default_value='1'),
        DeclareLaunchArgument('teleop', default_value='false'),  # can be "true" or "false"

        # Topics
        DeclareLaunchArgument('force_topic_name', default_value='/force_model'),
        DeclareLaunchArgument('teleop_pose_topic_name', default_value='/teleop/pose'),
        DeclareLaunchArgument('teleop_vel_topic_name', default_value='/teleop/vel'),
        DeclareLaunchArgument('ekf_topic_name', default_value='/kalman'),
        DeclareLaunchArgument('x_init_topic_name', default_value='/x_init_measurement'),
        DeclareLaunchArgument('controller_topic_name', default_value='lwr/hierarchical_controller/command'),
        DeclareLaunchArgument('fd_topic_name', default_value='/fd'),

        GroupAction(condition=IfCondition(LaunchConfiguration('teleop')), actions=[
            # node name="sigma_master" pkg="sigma" type="master" output="screen" respawn="false"
            Node(name='kuka_teleop', package='kuka_teleop', executable='kuka_teleop.py',
                 arguments=[LaunchConfiguration('teleop_pose_topic_name'),
                            LaunchConfiguration('teleop_vel_topic_name')],
                 respawn=False),
        ]),

        # Nodes launched
        # TODO(ros2-migration): force_models, kuka_stiffness_estimation and kuka_impedance_controller
        # have no ROS 2 version in this workspace (force_models is not in the ROS 1 workspace
        # either); this launch needs them (see MIGRATION_NOTES).
        Node(name='boltzmann_force_model_diag', package='force_models',
             executable='boltzmann_force_model_diag.py', respawn=False),
        Node(name='stiffness_observer', package='kuka_stiffness_estimation', executable='constrained_observer',
             arguments=[LaunchConfiguration('dt'), LaunchConfiguration('Fd'), LaunchConfiguration('nb_points'),
                        LaunchConfiguration('coeff_classification'), LaunchConfiguration('x_init_topic_name'),
                        LaunchConfiguration('force_topic_name')],
             output='screen', respawn=False),
        Node(name='impedance_controller', package='kuka_impedance_controller', executable='impedance_controller',
             arguments=[LaunchConfiguration('dt'), LaunchConfiguration('Fd'),
                        LaunchConfiguration('teleop_pose_topic_name'), LaunchConfiguration('teleop_vel_topic_name'),
                        LaunchConfiguration('controller_topic_name'), LaunchConfiguration('x_init_topic_name'),
                        LaunchConfiguration('ekf_topic_name'), LaunchConfiguration('force_topic_name'),
                        LaunchConfiguration('teleop'), LaunchConfiguration('fd_topic_name')],
             output='screen', respawn=False),
    ])
