"""ROS 2 port of probe_scan_motion.launch.

  Starts probe_scan_motion_node: the combined primary-scan (constant-velocity
  +Y cruise) plus relative-adjustment motion node. Parallel to probe_motion.launch
  (which runs probe_relative_motion_node). Use this one for the cruise-based
  GPIS and open-loop-test variants.

  SAFETY: publishes to /lwr/one_task_inverse_kinematics/command. Only ONE
  publisher on that topic may be active. Do NOT run this at the same time as
  probe_motion.launch, or ultrasound_trajectory's ee_incremental_motion,
  ultrasound_traj_node, tool_relative_motion_node.

  There is no contact-force loop: one_task_inverse_kinematics is stiff position
  control and contact depth is owned by the operator.
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
        DeclareLaunchArgument('recovery_action_topic', default_value='/probe_action_recovery'),
        DeclareLaunchArgument('cruise_topic', default_value='/probe_cruise_velocity'),
        DeclareLaunchArgument('ee_pose_topic', default_value='/cartesian_position/pose/end_effector'),
        DeclareLaunchArgument('command_topic', default_value='/lwr/one_task_inverse_kinematics/command'),
        DeclareLaunchArgument('step_action_name', default_value='/probe_step'),
        # Excursion / return quintic durations. The acknowledged GPIS cruise
        # profile uses 1.0 s outward and an intentionally faster 0.5 s recovery.
        DeclareLaunchArgument('step_duration', default_value='1.0'),
        DeclareLaunchArgument('recovery_step_duration', default_value='1.0'),
        # Acknowledged action trajectories use distance-based quintic timing.
        # These are true peak Cartesian limits; legacy Twist topics still use the
        # fixed step durations above. Recovery is intentionally faster.
        DeclareLaunchArgument('minimum_action_duration_sec', default_value='0.25'),
        DeclareLaunchArgument('adjustment_max_linear_velocity_mps', default_value='0.060'),
        DeclareLaunchArgument('adjustment_max_linear_acceleration_mps2', default_value='0.54'),
        DeclareLaunchArgument('adjustment_max_angular_velocity_deg_s', default_value='14.0625'),
        DeclareLaunchArgument('adjustment_max_angular_acceleration_deg_s2', default_value='129.904'),
        # Local probe-Z adjustment only: twice the default angular profile.
        DeclareLaunchArgument('local_z_max_angular_velocity_deg_s', default_value='37.5'),
        DeclareLaunchArgument('local_z_max_angular_acceleration_deg_s2', default_value='129.904'),
        DeclareLaunchArgument('recovery_max_linear_velocity_mps', default_value='0.120'),
        DeclareLaunchArgument('recovery_max_linear_acceleration_mps2', default_value='1.08'),
        DeclareLaunchArgument('recovery_max_angular_velocity_deg_s', default_value='37.5'),
        DeclareLaunchArgument('recovery_max_angular_acceleration_deg_s2', default_value='129.904'),
        # Primary +Y cruise. cruise_speed_max is a hard ceiling on the commanded sweep
        # speed; the orchestrator (GPIS / test node) sends the actual speed on
        # cruise_topic. cruise_accel shapes the trapezoidal ramps. cruise_following_gate_m
        # freezes the streamed target if the arm falls this far behind.
        DeclareLaunchArgument('cruise_speed_max', default_value='0.02'),
        DeclareLaunchArgument('cruise_accel', default_value='0.01'),
        DeclareLaunchArgument('cruise_following_gate_m', default_value='0.005'),
        DeclareLaunchArgument('control_rate_hz', default_value='100.0'),

        Node(name='probe_scan_motion_node', package='ultrasound_probe_motion',
             executable='probe_scan_motion_node', output='screen',
             parameters=[str(calibration),
                         {'probe_length': calibration_override('probe_length'),
                          'probe_y_offset': calibration_override('probe_y_offset'),
                          'probe_z_rotation_deg': calibration_override('probe_z_rotation_deg'),

                          'action_topic': text('action_topic'),
                          'recovery_action_topic': text('recovery_action_topic'),
                          'cruise_topic': text('cruise_topic'),
                          'ee_pose_topic': text('ee_pose_topic'),
                          'command_topic': text('command_topic'),
                          'step_action_name': text('step_action_name'),
                          'step_duration': number('step_duration'),
                          'recovery_step_duration': number('recovery_step_duration'),
                          'minimum_action_duration_sec': number('minimum_action_duration_sec'),
                          'adjustment_max_linear_velocity_mps': number('adjustment_max_linear_velocity_mps'),
                          'adjustment_max_linear_acceleration_mps2': number('adjustment_max_linear_acceleration_mps2'),
                          'adjustment_max_angular_velocity_deg_s': number('adjustment_max_angular_velocity_deg_s'),
                          'adjustment_max_angular_acceleration_deg_s2': number('adjustment_max_angular_acceleration_deg_s2'),
                          'local_z_max_angular_velocity_deg_s': number('local_z_max_angular_velocity_deg_s'),
                          'local_z_max_angular_acceleration_deg_s2': number('local_z_max_angular_acceleration_deg_s2'),
                          'recovery_max_linear_velocity_mps': number('recovery_max_linear_velocity_mps'),
                          'recovery_max_linear_acceleration_mps2': number('recovery_max_linear_acceleration_mps2'),
                          'recovery_max_angular_velocity_deg_s': number('recovery_max_angular_velocity_deg_s'),
                          'recovery_max_angular_acceleration_deg_s2': number('recovery_max_angular_acceleration_deg_s2'),
                          'cruise_speed_max': number('cruise_speed_max'),
                          'cruise_accel': number('cruise_accel'),
                          'cruise_following_gate_m': number('cruise_following_gate_m'),
                          'control_rate_hz': number('control_rate_hz')}]),
    ])
