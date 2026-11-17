#!/usr/bin/env python3
"""Real bridge handshake under a fixture clock; no planner/controller/flight."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

parser=argparse.ArgumentParser()
parser.add_argument('--backend',choices=['swarm','v2'],required=True)
args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']='75' if args.backend=='swarm' else '76'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Bool,UInt64
from uav_common_msg.msg import NativeTrajectory,TrajectoryFeedback
if args.backend=='swarm':
    from traj_utils.msg import Bspline as Input
else:
    from traj_utils.msg import PolyTraj as Input

rclpy.init();node=rclpy.create_node('bridge_rejection_fixture')
clock=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
goals=node.create_publisher(PoseStamped,'/autonomy/goal',10)
native=node.create_publisher(Input,'/ego/native_trajectory',10)
feedback=node.create_publisher(TrajectoryFeedback,'/autonomy/trajectory_feedback',10)
received=[];forwarded=[];permission=[];acks=[]
if args.backend=='swarm':
    Goal=PoseStamped
    def goal_callback(m):received.append([m.pose.position.x,m.pose.position.y,m.pose.position.z])
else:
    from quadrotor_msgs.msg import GoalSet as Goal
    def goal_callback(m):received.append(list(m.goal))
node.create_subscription(Goal,'/ego/goal',goal_callback,10)
node.create_subscription(NativeTrajectory,'/autonomy/native_trajectory',lambda m:forwarded.append(m),10)
node.create_subscription(Bool,'/autonomy/execution_allowed',lambda m:permission.append(m.data),
    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(UInt64,'/autonomy/native_accepted',lambda m:acks.append(m.data),10)
tick=0
def step(count=1):
    global tick
    for _ in range(count):
        tick+=1;m=Clock();m.clock.sec,m.clock.nanosec=divmod(1_000_000_000+tick*20_000_000,10**9)
        clock.publish(m)
        for _ in range(5):rclpy.spin_once(node,timeout_sec=0)
        time.sleep(.01)
    return m.clock
def until(predicate,limit=8):
    end=time.monotonic()+limit
    while not predicate():
        if time.monotonic()>end:raise AssertionError('Handshake timeout')
        step()
def input_message(identifier,stamp):
    m=Input();m.start_time=stamp;m.traj_id=identifier;m.order=3 if args.backend=='swarm' else 5
    if args.backend=='swarm':
        m.knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.]
        m.pos_pts=[Point(x=float(i),y=0.,z=2.) for i in range(4)]
    else:
        m.duration=[2.];m.coef_x=[0.]*6;m.coef_y=[0.]*6;m.coef_z=[0.,0.,0.,0.,0.,2.]
    return m
exe=Path.home()/f'.cache/gazebo_ego_{args.backend}/install/ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge'
with tempfile.TemporaryDirectory(prefix='bridge_rejection_') as folder:
    log=Path(folder)/'bridge.log'
    with log.open('w') as output:
        child=subprocess.Popen([str(exe),'--ros-args','-p','use_sim_time:=true'],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            until(lambda:goals.get_subscription_count()>0 and native.get_subscription_count()>0)
            goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=step();goal.pose.position.x=3.;goal.pose.position.z=2.;goal.pose.orientation.w=1.
            goals.publish(goal);until(lambda:len(received)==1)
            first=input_message(10,step());native.publish(first);until(lambda:len(forwarded)==1)
            accepted=TrajectoryFeedback();accepted.trajectory_id=forwarded[0].trajectory_id;accepted.accepted=True;feedback.publish(accepted)
            until(lambda:acks==[10])
            native.publish(input_message(11,step()));until(lambda:len(forwarded)==2)
            rejected=TrajectoryFeedback();rejected.trajectory_id=forwarded[-1].trajectory_id;rejected.reason='Invalid trajectory duration'
            before=len(permission);feedback.publish(rejected)
            # Goal and permission use separate DDS topics; neither topic's
            # callback establishes delivery order for the other. Await both
            # reset/rearm evidence and the retried goal before asserting.
            until(lambda:len(received)==2 and False in permission[before:] and permission[-1])
            assert received[1]==received[0] and False in permission[before:] and permission[-1]
            # A late candidate from before the reset is not executable.
            native.publish(first);step(20);assert len(forwarded)==2
            native.publish(input_message(12,step()));until(lambda:len(forwarded)==3)
            # A real guard stop must not be automatically retried/rearmed.
            stop=TrajectoryFeedback();stop.trajectory_id=forwarded[-1].trajectory_id;stop.braking=True
            feedback.publish(stop);until(lambda:not permission[-1]);count=len(received)
            feedback.publish(rejected);native.publish(input_message(13,step()));step(40)
            assert len(received)==count and len(forwarded)==3 and not permission[-1]
            goal.header.stamp=step();goals.publish(goal);until(lambda:len(received)==count+1)
            native.publish(input_message(14,step()));until(lambda:len(forwarded)==4)
            print(args.backend+': soft rejection retries same goal, rejects late epoch; hard stop stays revoked; new goal rearms: PASS')
        except BaseException:
            print(log.read_text());raise
        finally:
            if child.poll() is None:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
node.destroy_node();rclpy.shutdown()
