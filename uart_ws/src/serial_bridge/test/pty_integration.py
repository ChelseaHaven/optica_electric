"""Runs the real ROS node and UART transport against a pseudo-terminal, no board."""
import os
import pty
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from serial_bridge.node import SerialBridge
from serial_bridge.protocol import Parser, pack


def main():
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/uart_pty_test', '-p', 'port:=' + os.ttyname(slave)])
    bridge = SerialBridge()
    probe = Node('test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    seen = {'wheel': [], 'imu': [], 'mag': []}
    subscriptions = [probe.create_subscription(JointState, 'drive/wheel_states', seen['wheel'].append, qos_profile_sensor_data),
                     probe.create_subscription(Imu, 'imu/data_raw', seen['imu'].append, qos_profile_sensor_data),
                     probe.create_subscription(Float64MultiArray, 'imu/magnetic_field_raw', seen['mag'].append, qos_profile_sensor_data)]
    publisher = probe.create_publisher(Twist, 'cmd_vel', 1)
    parser = Parser()
    sent = []

    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)
            try:
                data = os.read(master, 4096)
            except BlockingIOError:
                data = b''
            for cmd, payload, _ in parser.feed(data, time.monotonic()):
                if cmd == 0x101 and len(payload) == 12:
                    sent.append(struct.unpack('<3f', payload))

    try:
        pump(.5)
        assert sent and all(v == (0, 0, 0) for v in sent)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = .4, -.2, .8
        for _ in range(5):
            publisher.publish(msg)
            pump(.03)
        assert any(abs(v[0] - .4) < 1e-6 and abs(v[1] + .2) < 1e-6 and abs(v[2] - .8) < 1e-6 for v in sent)
        pump(.5)
        assert sent[-1] == (0, 0, 0), 'stale cmd_vel was not cleared'
        wheel = pack(0x101, struct.pack('<4f', 1, -2, 3, -4), 1)
        imu = pack(0x102, struct.pack('<9f', .1, -.2, .3, 0, 0, 9.80665, 123, -456, 789), 2)
        os.write(master, wheel[:6]); pump(.01)
        os.write(master, wheel[6:] + imu); pump(.2)
        assert list(seen['wheel'][-1].velocity) == [1, -2, 3, -4]
        assert list(seen['wheel'][-1].name) == ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
        assert abs(seen['imu'][-1].linear_acceleration.z - 9.80665) < 1e-5
        assert seen['imu'][-1].orientation_covariance[0] == -1
        assert list(seen['mag'][-1].data) == [123, -456, 789]
        count = len(seen['imu'])
        corrupt = bytearray(imu); corrupt[10] ^= 1
        os.write(master, corrupt); pump(.1)
        assert len(seen['imu']) == count and bridge.parser.crc_errors > 0
        os.close(master); master = None
        until = time.monotonic() + .2
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)
        assert bridge.serial is None
        print('PASS: real ROS2 cmd_vel -> UART; timeout zero; fragmented wheel/IMU -> topics; CRC rejection; disconnect')
    finally:
        executor.remove_node(bridge); executor.remove_node(probe)
        bridge.destroy_node(); probe.destroy_node()
        executor.shutdown(); rclpy.shutdown()
        if master is not None:
            os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
