from glob import glob
from setuptools import setup
setup(name='control_bringup',version='0.1.0',packages=['control_bringup'],
 data_files=[('share/ament_index/resource_index/packages',['resource/control_bringup']),
 ('share/control_bringup',['package.xml']),('share/control_bringup/config',glob('config/*.yaml')),
 ('share/control_bringup/launch',glob('launch/*.launch.py'))],install_requires=['setuptools'],
 zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',description='Select and launch control algorithms',
 license='Apache-2.0',entry_points={'console_scripts':[]})
