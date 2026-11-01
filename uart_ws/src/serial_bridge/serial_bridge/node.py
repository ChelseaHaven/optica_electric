import json
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Float64MultiArray, String

from .protocol import Parser, WHEEL_SPEED, IMU_RAW, decode_telemetry, pack_velocity
from .transport import SerialPort


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        defaults = {'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 'baudrate': 460800, 'command_rate_hz': 50.0,
                    'command_timeout_sec': 0.3, 'reconnect_interval_sec': 1.0,
                    'imu_frame_id': 'imu_link', 'wheel_frame_id': 'base_link',
                    'wheel_names': ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'],
                    'wheel_topic': 'drive/wheel_states',
                    'magnetic_scale_t_per_lsb': 0.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name in ('command_rate_hz', 'command_timeout_sec', 'reconnect_interval_sec'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(name + ' must be positive and finite')
        if self.settings['command_rate_hz'] > 500:
            raise ValueError('command_rate_hz must be <= 500')
        if len(self.settings['wheel_names']) != 4:
            raise ValueError('wheel_names must contain four names in firmware A/B/C/D order')
        scale = self.settings['magnetic_scale_t_per_lsb']
        if not math.isfinite(scale) or scale < 0:
            raise ValueError('magnetic_scale_t_per_lsb must be finite and nonnegative')
        self.serial = None
        self.parser = Parser()
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None
        self.next_connect = self.next_transmit = 0.0
        self.pending = bytearray()
        self.sequence = self.tx_frames = self.invalid_frames = 0
        self.last_wheel = self.last_imu = None
        self.wheel_pub = self.create_publisher(JointState, self.settings['wheel_topic'], qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.mag_raw_pub = self.create_publisher(Float64MultiArray, 'imu/magnetic_field_raw', qos_profile_sensor_data)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/magnetic_field', qos_profile_sensor_data) if scale > 0 else None
        self.status_pub = self.create_publisher(String, 'serial/status', 10)
        self.create_subscription(Twist, 'cmd_vel', self.receive_command, 1)
        self.create_timer(0.002, self.service)
        self.create_timer(1.0, self.publish_status)
        if not self.settings['port']:
            self.get_logger().info('No port selected; set port:=/dev/... after connecting the controller')

    def receive_command(self, msg):
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        try:
            pack_velocity(*command)
        except (ValueError, OverflowError):
            self.command_time = None
            self.command = (0.0, 0.0, 0.0)
            self.get_logger().warning('Invalid cmd_vel; sending zero velocity')
            return
        self.command = command
        self.command_time = time.monotonic()

    def disconnect(self, error):
        self.get_logger().warning(f'Serial disconnected: {error}')
        self.serial.close()
        self.serial = None
        self.pending.clear()
        self.parser.reset()
        self.command_time = None
        self.next_connect = time.monotonic() + self.settings['reconnect_interval_sec']

    def service(self):
        now = time.monotonic()
        if self.serial is None:
            if not self.settings['port'] or now < self.next_connect:
                return
            self.next_connect = now + self.settings['reconnect_interval_sec']
            try:
                self.serial = SerialPort(self.settings['port'], self.settings['baudrate'])
            except (OSError, ValueError) as error:
                self.get_logger().warning(f'Cannot open serial port: {error}')
                return
            # A new connection requires a new velocity message; no queued replay.
            self.command_time = None
            self.pending.clear()
            self.parser.reset()
            self.next_transmit = now
            self.get_logger().info(f'Connected: {self.settings["port"]} at {self.settings["baudrate"]}')
        try:
            for command, payload, _ in self.parser.feed(self.serial.read(), now):
                self.publish_telemetry(command, payload, now)
            if not self.pending and now >= self.next_transmit:
                fresh = self.command_time is not None and now - self.command_time < self.settings['command_timeout_sec']
                self.pending.extend(pack_velocity(*(self.command if fresh else (0.0, 0.0, 0.0)), self.sequence))
                self.sequence = (self.sequence + 1) & 255
                self.next_transmit = now + 1.0 / self.settings['command_rate_hz']
            if self.pending:
                sent = self.serial.write(self.pending)
                del self.pending[:sent]
                if not self.pending:
                    self.tx_frames += 1
        except (OSError, ValueError) as error:
            self.disconnect(error)

    def publish_telemetry(self, command, payload, now):
        values = decode_telemetry(command, payload)
        if values is None:
            self.invalid_frames += 1
            return
        stamp = self.get_clock().now().to_msg()
        if command == WHEEL_SPEED:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = self.settings['wheel_frame_id']
            msg.name = self.settings['wheel_names']
            msg.velocity = list(values)
            self.wheel_pub.publish(msg)
            self.last_wheel = now
        elif command == IMU_RAW:
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.settings['imu_frame_id']
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values[:3]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values[3:6]
            self.imu_pub.publish(msg)
            raw = Float64MultiArray()
            raw.data = list(values[6:9])
            self.mag_raw_pub.publish(raw)
            if self.mag_pub is not None:
                magnetic = MagneticField()
                magnetic.header = msg.header
                magnetic.magnetic_field.x, magnetic.magnetic_field.y, magnetic.magnetic_field.z = [v * self.settings['magnetic_scale_t_per_lsb'] for v in values[6:9]]
                self.mag_pub.publish(magnetic)
            self.last_imu = now

    def publish_status(self):
        now = time.monotonic()
        msg = String()
        msg.data = json.dumps({'connected': self.serial is not None, 'tx_frames': self.tx_frames,
                               'rx_frames': self.parser.frames, 'crc_errors': self.parser.crc_errors,
                               'length_errors': self.parser.length_errors, 'invalid_frames': self.invalid_frames,
                               'wheel_age_sec': None if self.last_wheel is None else now-self.last_wheel,
                               'imu_age_sec': None if self.last_imu is None else now-self.last_imu})
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.serial is not None:
            try:
                # Complete any partial packet before appending the stop packet.
                self.pending.extend(pack_velocity(0.0, 0.0, 0.0, self.sequence))
                deadline = time.monotonic() + 0.05
                while self.pending and time.monotonic() < deadline:
                    count = self.serial.write(self.pending)
                    del self.pending[:count]
                    if not count:
                        time.sleep(0.001)
            except OSError:
                pass
            self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch may forward another SIGINT while cleanup is in progress.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
