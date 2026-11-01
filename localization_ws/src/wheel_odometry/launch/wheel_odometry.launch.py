from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('wheel_topic', default_value='/drive/wheel_states'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(package='wheel_odometry', executable='wheel_odometry', name='wheel_odometry',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('localization_common'), 'config', 'sensors.yaml']),
                         PathJoinSubstitution([FindPackageShare('wheel_odometry'), 'config', 'wheel_odometry.yaml']),
                         {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                          'wheel_topic': LaunchConfiguration('wheel_topic')}],
             remappings=[('/odom', LaunchConfiguration('odom_topic'))])])
