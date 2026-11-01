"""Algorithm registry: add future algorithms as sibling packages and entries."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

ALGORITHMS = {'eskf': ('eskf', 'eskf.launch.py'),
              'wheel_odometry': ('wheel_odometry', 'wheel_odometry.launch.py')}


def start(context):
    algorithm = LaunchConfiguration('algorithm').perform(context)
    if algorithm not in ALGORITHMS:
        raise RuntimeError('Unknown algorithm: ' + algorithm)
    package, launch_file = ALGORITHMS[algorithm]
    path = str(Path(get_package_share_directory(package)) / 'launch' / launch_file)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
        'publish_tf': LaunchConfiguration('publish_tf').perform(context),
        'wheel_topic': LaunchConfiguration('wheel_topic').perform(context),
        'odom_topic': LaunchConfiguration('odom_topic').perform(context)}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('algorithm', default_value='eskf'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('wheel_topic', default_value='/drive/wheel_states'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        OpaqueFunction(function=start)])
