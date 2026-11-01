"""Isolated synthetic sensor test. Never publishes cmd_vel or touches a UART."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from localization_common.geometry import WHEEL_NAMES
from eskf.node import EskfNode


def main():
    prefix = '/eskf_test_' + str(os.getpid())
    rclpy.init(args=['--ros-args', '-r', '__ns:='+prefix,
                     '-r', '/odom:='+prefix+'/odom',
                     '-r', '/localization/status:='+prefix+'/status',
                     '-p', 'wheel_topic:='+prefix+'/wheels', '-p', 'imu_topic:='+prefix+'/imu',
                     '-p', 'calibration_sec:=0.3', '-p', 'publish_tf:=true'])
    filter_node = EskfNode()
    source = Node('synthetic_sensors')
    executor = SingleThreadedExecutor()
    executor.add_node(filter_node); executor.add_node(source)
    wheels = source.create_publisher(JointState, prefix+'/wheels', qos_profile_sensor_data)
    imu = source.create_publisher(Imu, prefix+'/imu', qos_profile_sensor_data)
    received = {'odom': [], 'status': [], 'tf': []}
    subscriptions = [source.create_subscription(Odometry, 'odom', received['odom'].append, 10),
                     source.create_subscription(String, 'status', received['status'].append, 10),
                     source.create_subscription(TFMessage, '/tf', received['tf'].append, 10)]

    def run(duration, velocity=0.0, send_imu=True):
        end = time.monotonic()+duration
        next_sample = 0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_sample:
                stamp = source.get_clock().now().to_msg()
                j = JointState(); j.header.stamp = stamp
                j.name = list(WHEEL_NAMES); j.velocity = [velocity/.0385]*4
                wheels.publish(j)
                if send_imu:
                    m = Imu(); m.header.stamp = stamp
                    m.orientation_covariance[0] = -1
                    m.linear_acceleration.z = 9.80665
                    imu.publish(m)
                next_sample = now+.01
            executor.spin_once(timeout_sec=.002)

    try:
        run(.8)
        assert filter_node.filter.ready and received['odom']
        run(1., .1)
        run(.3)
        pose = received['odom'][-1].pose.pose.position
        assert .07 < pose.x < .13 and abs(pose.y) < .01
        assert received['tf'] and received['tf'][-1].transforms[-1].child_frame_id == 'base_link'
        run(.3, send_imu=False)
        count = len(received['odom'])
        run(.2, send_imu=False)
        assert len(received['odom']) == count and filter_node.state == 'imu_stale'
        assert not filter_node.failed
        print(json.dumps({'result': 'PASS', 'x_after_0.1m_straight': pose.x,
                          'odom_messages': count, 'tf_messages': len(received['tf']),
                          'stale_state': filter_node.state}))
    finally:
        executor.remove_node(filter_node); executor.remove_node(source)
        filter_node.destroy_node(); source.destroy_node()
        executor.shutdown(); rclpy.shutdown()


if __name__ == '__main__':
    main()
