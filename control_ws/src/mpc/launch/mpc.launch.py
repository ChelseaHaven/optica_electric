from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package='mpc', executable='mpc', name='mpc', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('mpc'), 'config', 'mpc.yaml'])])])
