"""Shared ROS 2 node parameters from the original MoveIt configuration files."""
from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def get_config(hardware='real', t1_limits=False):
    package = Path(get_package_share_directory('single_lwr_moveit'))
    robot = Path(get_package_share_directory('single_lwr_robot'))
    load = lambda name: yaml.safe_load((package / 'config' / name).read_text())
    description = xacro.process_file(str(robot / 'robot/single_lwr_robot.urdf.xacro'),
        mappings={'ros2_control_hardware_type': hardware, 'use_stiffness_joints': 'false'}).toxml()
    limits = load('joint_limits.yaml')
    if t1_limits:
        limits = yaml.safe_load((robot / 'config/t1_joint_limits.yaml').read_text())
    ompl = {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join('default_planner_request_adapters/' + name for name in
                ['AddTimeOptimalParameterization', 'ResolveConstraintFrames', 'FixWorkspaceBounds', 'FixStartStateBounds',
                 'FixStartStateCollision', 'FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}
    ompl.update(load('ompl_planning.yaml'))
    result = {'robot_description': description,
              'robot_description_semantic': (package / 'config/single_lwr_robot.srdf').read_text(),
              'robot_description_kinematics': load('kinematics.yaml'),
              'robot_description_planning': limits,
              'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
              'trajectory_execution.allowed_execution_duration_scaling': 1.2,
              'trajectory_execution.allowed_goal_duration_margin': 0.5,
              'trajectory_execution.allowed_start_tolerance': 0.01,
              'moveit_manage_controllers': True,
              'publish_robot_description_semantic': True,
              'publish_planning_scene': True, 'publish_geometry_updates': True,
              'publish_state_updates': True, 'publish_transforms_updates': True}
    result.update(load('controllers.yaml'))
    return result
