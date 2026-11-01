"""Isolated ROS test: startup publishes immediately; no enable service exists."""
import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from position_pid.node import PositionControl


def main():
    namespace='/position_pid_test_'+str(os.getpid())
    args=['--ros-args']
    for name in ['/cmd_vel','/goal_pose','/odom','/control/status']:
        args+=['-r',name+':='+namespace+name]
    rclpy.init(args=args)
    control=PositionControl();source=Node('control_test_source')
    executor=SingleThreadedExecutor();executor.add_node(control);executor.add_node(source)
    odom=source.create_publisher(Odometry,'/odom',10)
    goals=source.create_publisher(PoseStamped,'/goal_pose',1)
    commands=[]
    sub=source.create_subscription(Twist,'/cmd_vel',commands.append,10)
    def pump(duration,publish_odom=True):
        end=time.monotonic()+duration;next_sample=0
        while time.monotonic()<end:
            if publish_odom and time.monotonic()>=next_sample:
                msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link'
                msg.header.stamp=source.get_clock().now().to_msg();msg.pose.pose.orientation.w=1.0
                odom.publish(msg);next_sample=time.monotonic()+.02
            executor.spin_once(timeout_sec=.003)
    try:
        pump(.4)
        assert len(commands)>5 and all(m.linear.x==m.linear.y==m.angular.z==0 for m in commands)
        assert control.state=='waiting_for_goal'
        services=dict(control.get_service_names_and_types())
        assert '/control/enable' not in services
        assert not hasattr(control,'enabled')
        goal=PoseStamped();goal.header.frame_id='odom';goal.pose.position.x=.2;goal.pose.orientation.w=1.0
        goals.publish(goal);pump(.3)
        assert any(m.linear.x>0 for m in commands) and control.state=='tracking'
        assert all(math.hypot(m.linear.x,m.linear.y)<=.200001 for m in commands)
        pump(.4,publish_odom=False)
        assert control.state=='odom_stale' and commands[-1].linear.x==0
        count=len(commands);pump(.1,publish_odom=False)
        assert len(commands)>count and commands[-1].linear.x==0
        pump(.2)
        assert control.state=='tracking' and commands[-1].linear.x>0
        print('PASS: immediate periodic zero; goal starts control; no enable switch; stale zero; automatic recovery')
    finally:
        executor.remove_node(control);executor.remove_node(source)
        control.destroy_node();source.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
