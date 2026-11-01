from glob import glob
from setuptools import setup
setup(name='mpc',version='0.1.0',packages=['mpc'],
 data_files=[('share/ament_index/resource_index/packages',['resource/mpc']),('share/mpc',['package.xml']),
 ('share/mpc/config',glob('config/*.yaml')),('share/mpc/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',
 description='Constrained holonomic model predictive position controller',license='Apache-2.0',entry_points={'console_scripts':['mpc = mpc.node:main']})
