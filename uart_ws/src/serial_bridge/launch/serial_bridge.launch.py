from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        Node(package='serial_bridge', executable='serial_bridge', name='serial_bridge', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('serial_bridge'), 'config', 'serial_bridge.yaml']),
                         {'port': LaunchConfiguration('port'),
                          'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int)}])])
