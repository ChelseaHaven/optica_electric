from glob import glob
from setuptools import setup
setup(name='localization_common',version='0.1.0',packages=['localization_common'],
 data_files=[('share/ament_index/resource_index/packages',['resource/localization_common']),
 ('share/localization_common',['package.xml']),('share/localization_common/config',glob('config/*.yaml')),
 ('share/localization_common/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='pi',maintainer_email='maintainer@example.com',
 description='Shared planar sensor geometry and covariance helpers',license='Apache-2.0',entry_points={'console_scripts':[]})
