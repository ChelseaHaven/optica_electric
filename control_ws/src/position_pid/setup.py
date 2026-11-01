from glob import glob
from setuptools import setup
setup(name='position_pid',version='0.1.0',packages=['position_pid'],
 data_files=[('share/ament_index/resource_index/packages',['resource/position_pid']),
 ('share/position_pid',['package.xml']),('share/position_pid/config',glob('config/*.yaml')),
 ('share/position_pid/launch',glob('launch/*.launch.py'))],install_requires=['setuptools'],
 zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',description='Planar world position and yaw PID controller',
 license='Apache-2.0',entry_points={'console_scripts':['position_pid = position_pid.node:main']})
