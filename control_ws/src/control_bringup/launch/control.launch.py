from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ALGORITHMS = {'position_pid': ('position_pid', 'position_pid.launch.py'),
              'mpc': ('mpc', 'mpc.launch.py')}


def start(context):
    algorithm = LaunchConfiguration('algorithm').perform(context)
    if algorithm not in ALGORITHMS:
        raise RuntimeError('Unknown control algorithm: '+algorithm)
    package, launch_file = ALGORITHMS[algorithm]
    path = str(Path(get_package_share_directory(package))/'launch'/launch_file)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('algorithm', default_value='position_pid'),
                              OpaqueFunction(function=start)])
