import json
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from localization_common.geometry import rotation_matrix, wheel_velocity, odometry_covariances
from .filter import PlanarESKF, Noise


class EskfNode(Node):
    def __init__(self):
        super().__init__('eskf')
        defaults = {'wheel_topic': '/drive/wheel_states', 'imu_topic': '/imu/data_raw',
                    'base_frame': 'base_link', 'odom_frame': 'odom', 'publish_tf': False,
                    'rate_hz': 50.0, 'sensor_timeout_sec': .1, 'calibration_sec': 1.0,
                    'stationary_dwell_sec': .5, 'wheel_radius_m': .0385,
                    'track_width_m': .196, 'wheel_base_m': .160,
                    'imu_to_body': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                    'imu_position_body_m': [.08, 0., 0.],
                    'stationary_bias_only': True, 'angular_accel_time_constant_sec': .05,
                    'accel_noise': .6, 'gyro_noise': .035, 'gyro_bias_walk': .0005,
                    'accel_bias_walk': .04, 'wheel_noise_x': .05, 'wheel_noise_y': .05}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        self.rotation = rotation_matrix(self.cfg['imu_to_body'])
        for name in ['rate_hz', 'sensor_timeout_sec', 'calibration_sec', 'stationary_dwell_sec',
                     'wheel_radius_m', 'track_width_m', 'wheel_base_m']:
            if not np.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name+' must be positive and finite')
        if not 10 <= self.cfg['rate_hz'] <= 200:
            raise ValueError('rate_hz must be between 10 and 200')
        noise = Noise(self.cfg['accel_noise'], self.cfg['gyro_noise'], self.cfg['gyro_bias_walk'],
                      self.cfg['accel_bias_walk'], self.cfg['wheel_noise_x'], self.cfg['wheel_noise_y'])
        self.filter = PlanarESKF(noise, self.cfg['imu_position_body_m'],
                                 self.cfg['angular_accel_time_constant_sec'], self.cfg['stationary_bias_only'])
        self.imu = self.wheel = None
        self.last_time = None
        self.last_wheel_stamp = None
        self.calibration = []
        self.stationary_time = 0.0
        self.state = 'waiting_for_sensors'
        self.error = ''
        self.failed = False
        self.rejected_inputs = 0
        self.last_publish = None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/localization/status', 10)
        self.tf = TransformBroadcaster(self) if self.cfg['publish_tf'] else None
        self.create_subscription(Imu, self.cfg['imu_topic'], self.receive_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, self.cfg['wheel_topic'], self.receive_wheel, qos_profile_sensor_data)
        self.create_timer(1/self.cfg['rate_hz'], self.step)
        self.create_timer(1., self.publish_status)
        self.get_logger().info('Planar ESKF: keep stationary for initialization; output namespace ' + self.get_namespace())

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def receive_imu(self, msg):
        if msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
            self.rejected_inputs += 1
            return
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if not np.isfinite([a, w]).all():
            self.rejected_inputs += 1
            return
        self.imu = (self.stamp(msg), self.rotation @ a, self.rotation @ w)

    def receive_wheel(self, msg):
        try:
            v = wheel_velocity(msg.name, msg.velocity, self.cfg['wheel_radius_m'],
                               self.cfg['track_width_m'], self.cfg['wheel_base_m'])
        except (KeyError, ValueError):
            self.rejected_inputs += 1
            return
        self.wheel = (self.stamp(msg), v)

    def fresh(self, sample, now):
        return sample is not None and 0 <= now-sample[0] <= self.cfg['sensor_timeout_sec']

    def step(self):
        if self.failed:
            return
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds*1e-9
        dt = 1/self.cfg['rate_hz'] if self.last_time is None else now-self.last_time
        self.last_time = now
        if not 0 < dt <= .1:
            self.calibration.clear()
            self.stationary_time = 0
            self.filter.previous_omega = None
            self.state = 'timing_gap'
            return
        if not self.fresh(self.imu, now):
            self.calibration.clear()
            self.stationary_time = 0
            self.filter.previous_omega = None
            self.state = 'imu_stale'
            return
        _, a, w = self.imu
        wheel_fresh = self.fresh(self.wheel, now)
        still = wheel_fresh and np.linalg.norm(self.wheel[1][:2]) < .02 and np.linalg.norm(w) < .08
        if self.filter.ready:
            still = still and np.linalg.norm(a[:2]-self.filter.x[6:8]) < .25
        else:
            still = still and 7 < np.linalg.norm(a) < 12 and np.linalg.norm(a[:2]) < 1.5
        if not self.filter.ready:
            self.state = 'calibrating' if still else 'waiting_for_stationary'
            if not still:
                self.calibration.clear()
                return
            self.calibration.append((dt, a.copy(), w.copy()))
            if sum(entry[0] for entry in self.calibration) < self.cfg['calibration_sec']:
                return
            self.filter.initialize(np.mean([x[1] for x in self.calibration], axis=0),
                                   np.mean([x[2] for x in self.calibration], axis=0))
            self.calibration.clear()
            self.get_logger().info('Stationary initialization complete; local x/y/yaw start at zero')
        try:
            self.filter.predict(a, w, dt)
            if wheel_fresh and self.wheel[0] != self.last_wheel_stamp:
                self.filter.wheel_update(self.wheel[1][:2])
                self.last_wheel_stamp = self.wheel[0]
            self.stationary_time = self.stationary_time+dt if still else 0.0
            if self.stationary_time >= self.cfg['stationary_dwell_sec']:
                self.filter.stationary_update(a, w)
            self.state = 'tracking' if wheel_fresh else 'imu_only'
            self.publish_odom(now_ros.to_msg(), w[2]-self.filter.x[5])
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            self.failed = True
            self.state = 'filter_error'
            self.error = str(error)
            # Stop publishing a failed estimator; retain state for diagnosis.
            self.get_logger().error('ESKF update failed: '+str(error))

    def publish_odom(self, stamp, yaw_rate):
        x = self.filter.x
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, self.cfg['odom_frame']
        msg.child_frame_id = self.cfg['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x[0]), float(x[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(x[4]/2), math.cos(x[4]/2)
        c, s = math.cos(x[4]), math.sin(x[4])
        msg.twist.twist.linear.x = float(c*x[2]+s*x[3])
        msg.twist.twist.linear.y = float(-s*x[2]+c*x[3])
        msg.twist.twist.angular.z = float(yaw_rate)
        pose, twist = odometry_covariances(x, self.filter.p, self.filter.noise.gyroscope**2)
        msg.pose.covariance, msg.twist.covariance = pose, twist
        self.odom_pub.publish(msg)
        self.last_publish = stamp.sec+stamp.nanosec*1e-9
        if self.tf is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y = float(x[0]), float(x[1])
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({'state': self.state, 'ready': self.filter.ready,
                               'gyro_bias_rad_s': float(self.filter.x[5]),
                               'accel_bias_m_s2': self.filter.x[6:8].tolist(),
                               'wheel_nis': self.filter.wheel_nis, 'rejected_inputs': self.rejected_inputs,
                               'covariance_diagonal': np.diag(self.filter.p).tolist(),
                               'last_odom_stamp': self.last_publish, 'error': self.error}, allow_nan=False)
        self.status_pub.publish(msg)


def main(args=None):
    import signal
    rclpy.init(args=args)
    node = None
    try:
        node = EskfNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
