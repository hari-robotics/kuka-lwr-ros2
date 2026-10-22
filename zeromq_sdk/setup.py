from glob import glob

from setuptools import setup

package_name = 'zeromq_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nearlab',
    maintainer_email='nearlab@todo.todo',
    description='Bridges ROS topics to ZeroMQ so the controllers Python environment runs without ROS',
    license='TODO',
    entry_points={
        'console_scripts': [
            'zmq_bridge = zeromq_sdk.zmq_bridge:main',
        ],
    },
)
