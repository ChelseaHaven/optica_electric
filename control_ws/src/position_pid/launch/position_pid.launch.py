from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package='position_pid', executable='position_pid', name='position_pid', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('position_pid'), 'config', 'position_pid.yaml'])])])
