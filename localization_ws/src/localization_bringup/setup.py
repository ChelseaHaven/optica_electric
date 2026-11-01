from glob import glob
from setuptools import setup
setup(name='localization_bringup',version='0.1.0',packages=['localization_bringup'],
 data_files=[('share/ament_index/resource_index/packages',['resource/localization_bringup']),
 ('share/localization_bringup',['package.xml']),('share/localization_bringup/config',glob('config/*.yaml')),
 ('share/localization_bringup/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',
 description='Select and launch parallel localization algorithms',license='Apache-2.0',entry_points={'console_scripts':[]})
