from glob import glob
from setuptools import setup
setup(name='serial_bridge', version='0.1.0', packages=['serial_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/serial_bridge']),
                  ('share/serial_bridge', ['package.xml']),
                  ('share/serial_bridge/config', glob('config/*.yaml')),
                  ('share/serial_bridge/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='pi',
      maintainer_email='maintainer@example.com', description='Generic UART chassis and sensor bridge',
      license='Apache-2.0', entry_points={'console_scripts': ['serial_bridge = serial_bridge.node:main']})
