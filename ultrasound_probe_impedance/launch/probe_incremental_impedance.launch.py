"""ROS 2 port of probe_incremental_impedance.launch.

    Starts probe_incremental_impedance_node. Calibration constants are loaded
    from ultrasound_probe_motion/config/probe_calibration.yaml (the single
    source of truth for the flange->probe transform T_E_P) and can be
    overridden here via args (e.g. after re-measuring the calibration).

    SAFETY: this node publishes to /lwr/cartesian_impedance_controller/command,
    the same topic used by ultrasound_trajectory's
    ee_incremental_impedance_node. Do NOT run this launch file at the same
    time as that node: only one publisher on that topic may be active at a
    time, or commands will race/overwrite each other with no arbitration.
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

        DeclareLaunchArgument('pose_topic', default_value='/cartesian_position/pose/end_effector'),
        DeclareLaunchArgument('action_topic', default_value='/probe_action_impedance'),
        DeclareLaunchArgument('command_topic', default_value='/lwr/cartesian_impedance_controller/command'),
        DeclareLaunchArgument('force_topic', default_value='/Force_calibrated'),

        Node(name='probe_incremental_impedance_node', package='ultrasound_probe_impedance',
             executable='probe_incremental_impedance_node', output='screen',
             parameters=[str(calibration),
                         {'probe_length': calibration_override('probe_length'),
                          'probe_y_offset': calibration_override('probe_y_offset'),
                          'probe_z_rotation_deg': calibration_override('probe_z_rotation_deg'),

                          'pose_topic': text('pose_topic'),
                          'action_topic': text('action_topic'),
                          'command_topic': text('command_topic'),
                          'force_topic': text('force_topic')}]),
    ])
