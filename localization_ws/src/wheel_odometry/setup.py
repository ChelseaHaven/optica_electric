from glob import glob
from setuptools import setup
setup(name='wheel_odometry', version='0.1.0', packages=['wheel_odometry'],
      data_files=[('share/ament_index/resource_index/packages',['resource/wheel_odometry']),
                  ('share/wheel_odometry',['package.xml']),
                  ('share/wheel_odometry/config',glob('config/*.yaml')),
                  ('share/wheel_odometry/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='pi',
      maintainer_email='maintainer@example.com', license='Apache-2.0',
      description='Pure mecanum wheel velocity odometry',
      entry_points={'console_scripts':['wheel_odometry = wheel_odometry.node:main']})
