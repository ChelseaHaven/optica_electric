from glob import glob
from setuptools import setup
setup(name='control_common',version='0.1.0',packages=['control_common'],
 data_files=[('share/ament_index/resource_index/packages',['resource/control_common']),('share/control_common',['package.xml']),
 ('share/control_common/config',glob('config/*.yaml')),('share/control_common/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',
 description='Common planar control ROS interfaces',license='Apache-2.0',entry_points={'console_scripts':[]})
