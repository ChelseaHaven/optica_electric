import json
import math
import signal
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from localization_common.geometry import wheel_velocity
from .odometry import WheelOdometry, twist_covariance


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        defaults = {'wheel_topic':'/drive/wheel_states','base_frame':'base_link','odom_frame':'odom',
                    'publish_tf':False,'rate_hz':50.0,'sensor_timeout_sec':.1,
                    'wheel_radius_m':.0385,'track_width_m':.196,'wheel_base_m':.160,
                    'wheel_speed_std_rad_s':.1,'initial_position_std_m':.001,'initial_yaw_std_rad':.001}
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        self.cfg = {name:self.get_parameter(name).value for name in defaults}
        if not 10 <= self.cfg['rate_hz'] <= 200:
            raise ValueError('rate_hz must be between 10 and 200')
        if not math.isfinite(self.cfg['sensor_timeout_sec']) or self.cfg['sensor_timeout_sec'] <= 0:
            raise ValueError('sensor_timeout_sec must be positive and finite')
        covariance = twist_covariance(self.cfg['wheel_radius_m'],self.cfg['track_width_m'],
                                      self.cfg['wheel_base_m'],self.cfg['wheel_speed_std_rad_s'])
        self.estimator = WheelOdometry(covariance,self.cfg['initial_position_std_m'],self.cfg['initial_yaw_std_rad'])
        self.sample = None
        self.last_time = None
        self.state = 'waiting_for_wheels'
        self.last_publish = None
        self.invalid_frames = 0
        self.failed = False
        self.error = ''
        self.odom_pub = self.create_publisher(Odometry,'/odom',10)
        self.status_pub = self.create_publisher(String,'/localization/status',10)
        self.tf = TransformBroadcaster(self) if self.cfg['publish_tf'] else None
        self.create_subscription(JointState,self.cfg['wheel_topic'],self.receive,qos_profile_sensor_data)
        self.create_timer(1/self.cfg['rate_hz'],self.step)
        self.create_timer(1.,self.publish_status)
        self.get_logger().info('Pure wheel odometry ready; no IMU subscription')

    def receive(self,msg):
        try:
            velocity = wheel_velocity(msg.name,msg.velocity,self.cfg['wheel_radius_m'],
                                      self.cfg['track_width_m'],self.cfg['wheel_base_m'])
        except (KeyError,ValueError):
            self.invalid_frames += 1
            return
        self.sample = (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,velocity)

    def step(self):
        if self.failed:
            return
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds*1e-9
        if self.sample is None:
            self.state = 'waiting_for_wheels'
            self.last_time = None
            return
        if not 0 <= now-self.sample[0] <= self.cfg['sensor_timeout_sec']:
            self.state = 'wheel_stale'
            self.last_time = None
            return
        dt = None if self.last_time is None else now-self.last_time
        self.last_time = now
        if dt is not None and not 0 < dt <= .1:
            self.state = 'timing_gap'
            self.last_time = None
            return
        try:
            # First fresh sample establishes the origin without integrating a stale gap.
            if dt is not None:
                self.estimator.step(self.sample[1],dt)
            self.publish_odom(now_ros.to_msg(),self.sample[1])
            self.state = 'tracking'
        except (ValueError,FloatingPointError) as error:
            self.error = str(error)
            self.failed = True
            self.state = 'odometry_error'
            self.get_logger().error(self.error)

    def publish_odom(self,stamp,velocity):
        x,y,yaw = self.estimator.pose
        msg = Odometry()
        msg.header.stamp,msg.header.frame_id = stamp,self.cfg['odom_frame']
        msg.child_frame_id = self.cfg['base_frame']
        msg.pose.pose.position.x,msg.pose.pose.position.y = float(x),float(y)
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z = map(float,velocity)
        msg.pose.covariance,msg.twist.covariance = self.estimator.ros_covariances()
        self.odom_pub.publish(msg)
        self.last_publish = stamp.sec+stamp.nanosec*1e-9
        if self.tf is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x,transform.transform.translation.y = float(x),float(y)
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({'algorithm':'wheel_odometry','state':self.state,
                               'ready':self.state=='tracking','invalid_frames':self.invalid_frames,
                               'pose':self.estimator.pose.tolist(),
                               'covariance_diagonal':np.diag(self.estimator.covariance).tolist(),
                               'last_odom_stamp':self.last_publish,'error':self.error})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
