"""Standalone controller manager for an existing LWR URDF/xacro and controller YAML."""
from pathlib import Path
import tempfile
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

class ControllerDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

def _launch(context):
    from launch.substitutions import LaunchConfiguration
    value = lambda key: LaunchConfiguration(key).perform(context)
    source = Path(value('description_file'))
    config_file = Path(value('controllers_file'))
    if not source.is_file() or not config_file.is_file():
        raise ValueError('description_file and controllers_file must name existing URDF/xacro and ROS 2 controller YAML files')
    description = xacro.process_file(str(source), mappings={
        'ros2_control_hardware_type': 'real', 'use_stiffness_joints': 'false',
        'fri_backend': value('fri_backend'), 'fri_port': value('port'),
        'fri_ip': value('ip'), 'fri_init_file': value('file_with_path'),
        'fri_receive_timeout_ms': value('receive_timeout_ms')}).toxml()
    config = yaml.safe_load(config_file.read_text())
    if '/' + value('name').strip('/') + '/controller_manager' not in config:
        raise ValueError('controllers_file must contain the controller_manager block for the selected namespace')
    for name, block in config.items():
        block['ros__parameters']['use_sim_time'] = False
        if name.endswith('/controller_manager'):
            block['ros__parameters']['update_rate'] = int(value('update_rate'))
        else:
            block['ros__parameters']['robot_description'] = description
    with tempfile.NamedTemporaryFile(mode='w',prefix='lwr_hw_controllers_',suffix='.yaml',delete=False) as output:
        yaml.dump(config, output, Dumper=ControllerDumper, sort_keys=False)
        control_file = output.name
    return [Node(package='controller_manager', executable='ros2_control_node', namespace=value('name'),
        parameters=[{'robot_description': description},control_file],output='screen')]

def generate_launch_description():
    defaults={'name':'lwr','port':'49939','ip':'192.168.0.20','file_with_path':'',
              'fri_backend':'fri','description_file':'','controllers_file':'','update_rate':'500',
              'receive_timeout_ms':'100'}
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value) for key,value in defaults.items()]
                             + [OpaqueFunction(function=_launch)])
