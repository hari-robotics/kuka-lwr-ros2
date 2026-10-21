"""ROS 2 port of probe_motion.launch.

  Starts probe_relative_motion_node. Calibration constants come from
  config/probe_calibration.yaml and can be overridden here via args
  (e.g. after re-measuring the calibration).

  SAFETY: this node publishes to /lwr/one_task_inverse_kinematics/command,
  the same legacy topic used by ultrasound_trajectory's
  ee_incremental_motion, ultrasound_traj_node, and tool_relative_motion_node.
  Do NOT run this launch file at the same time as any of those nodes:
  only one publisher on that topic may be active at a time, or commands
  will race/overwrite each other with no arbitration.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    calibration = Path(get_package_share_directory('ultrasound_probe_motion')) / 'config/probe_calibration.yaml'
    calibration_values = yaml.safe_load(calibration.read_text())['/**']['ros__parameters']

    def calibration_override(name):
        # ROS 1: <param name if="$(eval arg(name) != '')"> overrides the rosparam file only for a
        # non-empty arg; an empty arg keeps the file value
        return ParameterValue(PythonExpression(
            ["'", LaunchConfiguration(name), "' or ", repr(calibration_values[name])]), value_type=float)

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    def text(name):
        return ParameterValue(LaunchConfiguration(name), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('probe_length', default_value=''),
        DeclareLaunchArgument('probe_y_offset', default_value=''),
        DeclareLaunchArgument('probe_z_rotation_deg', default_value=''),

        DeclareLaunchArgument('action_topic', default_value='/probe_action'),
        DeclareLaunchArgument('ee_pose_topic', default_value='/cartesian_position/pose/end_effector'),
        DeclareLaunchArgument('command_topic', default_value='/lwr/one_task_inverse_kinematics/command'),
        DeclareLaunchArgument('step_action_name', default_value='/probe_step'),

        # Step timing. step_duration is the quintic interpolation time for a normal
        # action; recovery_step_duration applies to actions received on
        # recovery_action_topic. Defaults match ultrasound_probe_impedance's
        # probe_incremental_force_node: 3.0 s normal and recovery (commissioned for
        # safety), recovery channel on (/probe_action_recovery). Pass
        # recovery_action_topic:="" to disable it.
        DeclareLaunchArgument('step_duration', default_value='3.0'),
        DeclareLaunchArgument('recovery_step_duration', default_value='3.0'),
        DeclareLaunchArgument('recovery_action_topic', default_value='/probe_action_recovery'),

        Node(name='probe_relative_motion_node', package='ultrasound_probe_motion',
             executable='probe_relative_motion_node', output='screen',
             parameters=[str(calibration),
                         {'probe_length': calibration_override('probe_length'),
                          'probe_y_offset': calibration_override('probe_y_offset'),
                          'probe_z_rotation_deg': calibration_override('probe_z_rotation_deg'),

                          'action_topic': text('action_topic'),
                          'ee_pose_topic': text('ee_pose_topic'),
                          'command_topic': text('command_topic'),
                          'step_action_name': text('step_action_name'),
                          'step_duration': number('step_duration'),
                          'recovery_step_duration': number('recovery_step_duration'),
                          'recovery_action_topic': text('recovery_action_topic')}]),
    ])
