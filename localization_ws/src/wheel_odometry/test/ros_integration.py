"""Isolated pure-wheel ROS test: deliberately no IMU publisher."""
import os
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from wheel_odometry.node import WheelOdometryNode
from localization_common.geometry import WHEEL_NAMES


def main():
    ns='/wheel_test_'+str(os.getpid())
    rclpy.init(args=['--ros-args','-p','wheel_topic:='+ns+'/wheels',
                     '-r','/odom:='+ns+'/odom','-r','/localization/status:='+ns+'/status'])
    estimator=WheelOdometryNode();source=Node('wheel_test_source')
    executor=SingleThreadedExecutor();executor.add_node(estimator);executor.add_node(source)
    pub=source.create_publisher(JointState,ns+'/wheels',qos_profile_sensor_data)
    outputs=[];sub=source.create_subscription(Odometry,ns+'/odom',outputs.append,10)
    def run(duration,values=None):
        end=time.monotonic()+duration;next_sample=0
        while time.monotonic()<end:
            if values is not None and time.monotonic()>=next_sample:
                msg=JointState();msg.header.stamp=source.get_clock().now().to_msg()
                # Name mapping must also work when the array order is reversed.
                msg.name=list(WHEEL_NAMES[::-1]);msg.velocity=list(values[::-1]);pub.publish(msg)
                next_sample=time.monotonic()+.01
            executor.spin_once(timeout_sec=.002)
    try:
        run(.2);assert not outputs
        run(.3,[0.]*4);assert estimator.state=='tracking'
        run(1.,[.2/.0385]*4);run(.2,[0.]*4)
        assert .17<outputs[-1].pose.pose.position.x<.23
        x=outputs[-1].pose.pose.position.x
        arm=(.196+.160)/2;speed=.4*arm/.0385
        run(.5,[-speed,speed,-speed,speed]);run(.15,[0.]*4)
        assert abs(outputs[-1].pose.pose.position.x-x)<.005
        yaw=2*math.atan2(outputs[-1].pose.pose.orientation.z,outputs[-1].pose.pose.orientation.w)
        assert .16<yaw<.24
        run(.3);count=len(outputs);run(.2)
        assert len(outputs)==count and estimator.state=='wheel_stale'
        old=estimator.estimator.pose.copy()
        run(.2,[0.]*4)
        assert estimator.state=='tracking'
        assert max(abs(estimator.estimator.pose-old))<1e-10
        print(f'PASS: no IMU; named wheel mapping; straight x={x:.4f}; yaw={yaw:.4f}; stale pause and recovery')
    finally:
        executor.remove_node(estimator);executor.remove_node(source)
        estimator.destroy_node();source.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
