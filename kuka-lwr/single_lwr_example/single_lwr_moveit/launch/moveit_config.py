"""Shared ROS 2 node parameters from the original MoveIt configuration files."""
from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def get_config(t1_limits=False, with_description=True, fake_execution=False):
    package = Path(get_package_share_directory('single_lwr_moveit'))
    robot = Path(get_package_share_directory('single_lwr_robot'))
    load = lambda name: yaml.safe_load((package / 'config' / name).read_text())
    limits = load('joint_limits.yaml')
    if t1_limits:
        # rosparam load of t1_joint_limits.yaml on top of robot_description_planning
        def merge(base, override):
            for key, value in override.items():
                if isinstance(value, dict) and isinstance(base.get(key), dict):
                    merge(base[key], value)
                else:
                    base[key] = value
        merge(limits, yaml.safe_load((robot / 'config/t1_joint_limits.yaml').read_text()))
    ompl = {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join('default_planner_request_adapters/' + name for name in
                ['AddTimeOptimalParameterization', 'ResolveConstraintFrames', 'FixWorkspaceBounds', 'FixStartStateBounds',
                 'FixStartStateCollision', 'FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}
    ompl.update(load('ompl_planning.yaml'))
    result = {'robot_description_semantic': (package / 'config/single_lwr_robot.srdf').read_text(),
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
    if fake_execution:
        # ROS 1 fake_moveit_controller_manager.launch.xml: MoveItFakeControllerManager with
        # fake_controllers.yaml. MoveIt 2 (Humble) has no fake controller manager; the same
        # controllers are FollowJointTrajectory controllers under /move_group, served by the
        # ros2_control mock hardware of demo.launch.py and never by the robot controllers.
        fake = load('fake_controllers.yaml')['controller_list']
        names = ['/move_group/' + controller['name'] for controller in fake]
        result['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        result['moveit_simple_controller_manager'] = {'controller_names': names, **{
            name: {'action_ns': 'follow_joint_trajectory', 'type': 'FollowJointTrajectory', 'default': True,
                   'joints': controller['joints']} for name, controller in zip(names, fake)}}
    else:
        result.update(load('controllers.yaml'))
    if with_description:
        # planning_context.launch with load_robot_description:=true
        result['robot_description'] = xacro.process_file(str(robot / 'robot/single_lwr_robot.urdf.xacro'),
            mappings={'ros2_control_hardware_type': 'none', 'use_stiffness_joints': 'false'}).toxml()
    return result
