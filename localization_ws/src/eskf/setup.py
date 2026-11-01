from glob import glob
from setuptools import setup
setup(name='eskf',version='0.1.0',packages=['eskf'],
 data_files=[('share/ament_index/resource_index/packages',['resource/eskf']),
 ('share/eskf',['package.xml']),('share/eskf/config',glob('config/*.yaml')),
 ('share/eskf/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',
 description='Planar error-state Kalman filter',license='Apache-2.0',entry_points={'console_scripts':['eskf = eskf.node:main']})
