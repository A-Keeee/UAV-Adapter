#!/usr/bin/env python3
"""Real EGO FSM must wait for controller acceptance before advancing a candidate."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID']=str(120+os.getpid()%80)
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Float64
from uav_common_msg.msg import NativeTrajectory, TrajectoryFeedback
ROOT=Path(__file__).resolve().parents[3]

def main():
    backend=sys.argv[1];slow='--slow-diagonal' in sys.argv[2:];obstacles='--obstacle-course' in sys.argv[2:];warehouse='--warehouse-envelope' in sys.argv[2:];reduction='--speed-reduction' in sys.argv[2:];requested_speed=.5;rclpy.init();n=Node('planner_ack_regression')
    clock=n.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    odom=n.create_publisher(Odometry,'/ack/odom',10);cloud=n.create_publisher(PointCloud2,'/ack/cloud',qos_profile_sensor_data)
    goals=n.create_publisher(PoseStamped,'/autonomy/goal',10);feedback=n.create_publisher(TrajectoryFeedback,'/autonomy/trajectory_feedback',10)
    limit=n.create_publisher(Float64,'/autonomy/speed_limit',10)
    received=[];n.create_subscription(NativeTrajectory,'/autonomy/native_trajectory',received.append,10)
    ns=1_000_000_000
    def step(seconds):
        nonlocal ns
        end=ns+round(seconds*1e9)
        while ns<end:
            ns+=20_000_000;c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
            o=Odometry();o.header=Header(stamp=c.clock,frame_id='odom');o.pose.pose.position.z=4.17 if slow and backend=='swarm' else 1.8;o.pose.pose.position.x=-1.1 if slow else 0.;o.pose.pose.position.y=-.2 if slow else 0.;o.pose.pose.orientation.w=1.;odom.publish(o)
            if slow:limit.publish(Float64(data=.18 if backend=='v2' else .8*.18/(3.**.5)))
            if reduction:limit.publish(Float64(data=requested_speed))
            points=[[1.5,0.,z] for z in (1.5,1.8,2.1)]+[[3.,1.6,1.8]] if obstacles else [[0.,5.,1.5],[10.,3.,1.5]]
            cloud.publish(create_cloud_xyz32(o.header,points))
            for _ in range(4):rclpy.spin_once(n,timeout_sec=0)
            time.sleep(.012)
    def goal():
        g=PoseStamped();g.header.frame_id='odom';g.header.stamp.sec,g.header.stamp.nanosec=divmod(ns,10**9);g.pose.position.x=0. if slow and backend=='swarm' else 5.;g.pose.position.z=3.72 if warehouse else (1.5 if slow and backend=='swarm' else 1.8);g.pose.orientation.w=1.
        if reduction:g.pose.position.x=20.
        goals.publish(g)
    overlay=Path.home()/f'.cache/gazebo_ego_{backend}/install'
    commands=[[str(overlay/'ego_planner/lib/ego_planner/ego_planner_node'),'--ros-args','--params-file',str(ROOT/'uav_autonomy/config'/f'ego_{backend}.yaml'),
        '-r','odom_world:=/ack/odom','-r','grid_map/odom:=/ack/odom','-r','grid_map/cloud:=/ack/cloud',
        '-r','planning/trajectory:=/ego/native_trajectory','-r','planning/bspline:=/ego/native_trajectory',
        '-r','/goal:=/ego/goal','-r','/move_base_simple/goal:=/ego/goal'],
        [str(overlay/'ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge'),'--ros-args','-p','use_sim_time:=true']]
    ps=[]
    if backend=='swarm' and not warehouse:commands[0] += ['-p','grid_map/virtual_floor_height:=-1.0']
    with tempfile.TemporaryFile() as log:
        try:
            for cmd in commands:ps.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            step(2.);goal()
            if reduction:
                for _ in range(50):
                    step(.1)
                    if received:break
                assert len(received)==1,'Initial non-terminal plan missing'
                requested_speed=.18;step(.2)
                feedback.publish(TrajectoryFeedback(trajectory_id=received[0].trajectory_id,accepted=True))
                for _ in range(50):
                    step(.1)
                    if len(received)>1:break
                assert len(received)==2,'Reduced-speed candidate missing'
                import numpy as np
                def speed(m):
                    largest=0.
                    for duration,piece in zip(m.durations,np.asarray(m.coefficients).reshape(-1,3,6)):
                        times=np.linspace(0,duration,3001)
                        largest=max(largest,float(np.linalg.norm(np.column_stack([np.polyval(np.polyder(axis),times) for axis in piece]),axis=1).max()))
                    return largest
                before,after=map(speed,received)
                print('Native speed before/after limit reduction:',before,after,flush=True)
                assert before>.18,'Fixture did not create a faster retained global seed'
                assert after<=.18*1.05,'Retained global seed forced an over-speed terminal boundary'
                print('Native replan respects reduced speed with a retained global seed: PASS');return
            step(3.)
            assert len(received)==1,f'Unaccepted candidate advanced or first plan missing: {len(received)}'
            if slow:
                import numpy as np
                if backend=='v2':
                    m=received[0];coefficients=np.asarray(m.coefficients).reshape(-1,3,6)
                    bounds=[0.,0.,0.]
                    for duration,piece in zip(m.durations,coefficients):
                        times=np.linspace(0,duration,3001)
                        if obstacles:
                            positions=np.column_stack([np.polyval(axis,times) for axis in piece])
                            for obstacle in [[1.5,0.,z] for z in (1.5,1.8,2.1)]+[[3.,1.6,1.8]]:
                                assert np.linalg.norm(positions-np.asarray(obstacle),axis=1).min()>=.8,'MINCO crossed the obstacle margin'
                        for d in (1,2,3):
                            values=np.column_stack([np.polyval(np.polyder(axis,d),times) for axis in piece])
                            bounds[d-1]=max(bounds[d-1],float(np.linalg.norm(values,axis=1).max()))
                    print('Low-speed native MINCO V/A/J bounds:',bounds,flush=True)
                    assert bounds[0]<=.18 and bounds[1]<=.75 and bounds[2]<=1.5,'Native low-speed MINCO violates execution limits'
                    print('Low-speed original MINCO trajectory: PASS');return
                from scipy.interpolate import BSpline
                m=received[0];spline=BSpline(np.asarray(m.knots),np.asarray(m.control_points).reshape(-1,3),m.degree)
                times=np.linspace(m.knots[m.degree],m.knots[-m.degree-1],3001)
                if warehouse:
                    heights=spline(times)[:,2]
                    assert heights.min()>=3.6 and heights.max()<=5.8,'Native trajectory left the warehouse height envelope'
                bounds=[float(np.linalg.norm(spline.derivative(d)(times),axis=1).max()) for d in (1,2,3)]
                print('Low-speed diagonal native V/A/J bounds:',bounds,flush=True)
                axis_speed=float(np.abs(spline.derivative(1)(times)).max())
                assert axis_speed<=.8*.18/(3.**.5)*1.05+1e-4,'Native refinement exceeded its requested per-axis limit'
                assert bounds[0]<=.18*1.05 and bounds[1]<=.75 and bounds[2]<=1.5,'Native low-speed return violates execution limits'
                print('Low-speed diagonal return with original B-spline: PASS');return
            feedback.publish(TrajectoryFeedback(trajectory_id=received[0].trajectory_id,accepted=True));step(2.)
            assert len(received)==2,f'Accepted candidate did not advance once: {len(received)}'
            feedback.publish(TrajectoryFeedback(trajectory_id=received[1].trajectory_id,accepted=False));step(1.)
            assert len(received)==2,'Rejected candidate continued advancing'
            goal();step(2.)
            assert len(received)==3,'Replacement goal did not start after rejection'
            goal();step(2.)
            assert len(received)==4,'Superseding goal remained blocked by an unaccepted candidate'
            step(1.)
            assert len(received)==4,'Superseding candidate advanced without acknowledgement'
            print(backend+': one outstanding candidate, controller acknowledgement, rejection, restart and superseding goal: PASS')
        except BaseException:
            log.seek(0);print(log.read().decode(),file=sys.stderr);raise
        finally:
            for p in ps:
                if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
            for p in ps:
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
