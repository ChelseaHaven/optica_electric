import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String



def yaw_from_quaternion(q):
    values = [q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    x, y, z, w = [v/norm for v in values]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class PlanarControlNode(Node):
    def __init__(self, algorithm, controller_defaults, controller_factory, control_rate_hz=50.0):
        super().__init__(algorithm)
        self.algorithm = algorithm
        defaults = controller_defaults.copy()
        defaults.update({'control_rate_hz': control_rate_hz, 'odom_timeout_sec': .2,
                         'odom_frame': 'odom', 'base_frame': 'base_link'})
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if not 10 <= self.cfg['control_rate_hz'] <= 200:
            raise ValueError('control_rate_hz must be between 10 and 200')
        if not math.isfinite(self.cfg['odom_timeout_sec']) or self.cfg['odom_timeout_sec'] <= 0:
            raise ValueError('odom_timeout_sec must be positive and finite')
        self.controller = controller_factory({name: self.cfg[name] for name in controller_defaults})
        self.goal = self.odom = None
        self.last_time = None
        self.state = 'waiting_for_goal'
        self.rejected_inputs = 0
        self.last_error = ''
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        self.create_subscription(Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/goal_pose', self.receive_goal, 1)
        self.create_timer(1/self.cfg['control_rate_hz'], self.step)
        self.create_timer(.5, self.publish_status)
        self.get_logger().info(algorithm+' publishing cmd_vel at startup; set /goal_pose to move')

    def receive_odom(self, msg):
        try:
            if msg.header.frame_id != self.cfg['odom_frame'] or msg.child_frame_id != self.cfg['base_frame']:
                raise ValueError('odometry frame mismatch')
            p, v = msg.pose.pose.position, msg.twist.twist
            pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
            velocity = (v.linear.x, v.linear.y, v.angular.z)
            if not all(math.isfinite(x) for x in (*pose, *velocity)):
                raise ValueError('invalid odometry')
            self.odom = (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, pose, velocity)
        except ValueError:
            self.rejected_inputs += 1

    def receive_goal(self, msg):
        try:
            if msg.header.frame_id != self.cfg['odom_frame']:
                raise ValueError('goal must be expressed in '+self.cfg['odom_frame'])
            goal = (msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
            if not all(math.isfinite(v) for v in goal):
                raise ValueError('invalid goal')
        except ValueError as error:
            self.rejected_inputs += 1
            self.get_logger().warning('Rejected goal: '+str(error))
            return
        self.goal = goal
        self.controller.reset(keep_command=True)
        self.state = 'tracking'
        self.get_logger().info(f'Goal accepted: x={goal[0]:.3f}, y={goal[1]:.3f}, yaw={goal[2]:.3f}')

    def odom_fresh(self, now):
        return self.odom is not None and 0 <= now-self.odom[0] <= self.cfg['odom_timeout_sec']

    def stop(self):
        self.controller.reset()
        self.cmd_pub.publish(Twist())

    def step(self):
        now = self.get_clock().now().nanoseconds*1e-9
        dt = 1/self.cfg['control_rate_hz'] if self.last_time is None else now-self.last_time
        self.last_time = now
        if not 0 < dt <= .1:
            self.state = 'timing_gap'
            self.stop()
            return
        if not self.odom_fresh(now):
            self.state = 'odom_stale'
            self.stop()
            return
        if self.goal is None:
            self.state = 'waiting_for_goal'
            self.stop()
            return
        try:
            vx, vy, wz = self.controller.step(self.odom[1], self.odom[2], self.goal, dt)
        except (ValueError, RuntimeError) as error:
            self.state = 'control_error'
            self.last_error = str(error)
            diagnostics = dict(getattr(self.controller, 'diagnostics', {}))
            self.stop()
            if diagnostics:
                self.controller.diagnostics = diagnostics
            self.get_logger().error(str(error))
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)
        self.state = 'holding' if self.controller.reached else 'tracking'

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({'algorithm': self.algorithm, 'state': self.state,
                               'goal': self.goal, 'error': self.controller.error,
                               'goal_reached': self.controller.reached, 'command': self.controller.previous,
                               'rejected_inputs': self.rejected_inputs,
                               'solver': getattr(self.controller, 'diagnostics', {}), 'last_error': self.last_error})
        self.status_pub.publish(msg)

    def destroy_node(self):
        # No enable switch: publish zero on shutdown while ROS is available.
        if self.context.ok():
            self.stop()
        return super().destroy_node()
