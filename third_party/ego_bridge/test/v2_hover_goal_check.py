#!/usr/bin/python3
"""Recorded hovering derivatives with a distant B: real v2/bridge, no vehicle.

Inputs from forest_os1_64_2_attempt2 planner odometry at 679.596/692.476 s.
Empty nearby synthetic obstacle space isolates geometric-guide construction;
this checks native boundary conditions and liveness, not flight acceptance.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
os.environ['ROS_DOMAIN_ID']='79'
os.environ['ROS_LOCALHOST_ONLY']='1'
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Float64
from rosgraph_msgs.msg import Clock
from uav_common_msg.msg import NativeTrajectory

ROOT=Path(__file__).resolve().parents[3]
CASES=[
 ([33.664119720458984,1.8813530206680298,2.013519287109375],
  [-.003585340222343803,-.0017276700818911203,-.0172381903976202]),
 ([33.66309356689453,1.8707785606384277,1.99853515625],
  [.00038324488559737444,.016341557726264,.009554454125463963]),
]

def check(planner,position,velocity):
    rclpy.init();node=Node('v2_hover_goal_regression')
    clock=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    odom=node.create_publisher(Odometry,'/probe/odom',10)
    cloud=node.create_publisher(PointCloud2,'/probe/cloud',qos_profile_sensor_data)
    goal=node.create_publisher(PoseStamped,'/autonomy/goal',10)
    limit=node.create_publisher(Float64,'/autonomy/speed_limit',10)
    received=[]
    node.create_subscription(NativeTrajectory,'/autonomy/native_trajectory',received.append,10)
    overlay=Path.home()/'.cache/gazebo_ego_v2/install'
    commands=[
      [str(planner),'--ros-args','--params-file',str(ROOT/'uav_autonomy/config/ego_v2.yaml'),
       '-r','odom_world:=/probe/odom','-r','grid_map/odom:=/probe/odom',
       '-r','grid_map/cloud:=/probe/cloud','-r','planning/trajectory:=/ego/native_trajectory','-r','/goal:=/ego/goal'],
      [str(overlay/'ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge'),'--ros-args','-p','use_sim_time:=true']]
    processes=[]
    with tempfile.TemporaryFile() as logs:
      try:
        for command in commands:processes.append(subprocess.Popen(command,stdout=logs,stderr=subprocess.STDOUT,start_new_session=True))
        ns=1_000_000_000;sent=False;deadline=time.monotonic()+12
        time.sleep(.5)
        while not received and time.monotonic()<deadline:
          ns+=20_000_000;c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
          o=Odometry();o.header=Header(stamp=c.clock,frame_id='odom');o.pose.pose.orientation.w=1.
          o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=position
          o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=velocity
          odom.publish(o)
          if ns%100_000_000==0:
            cloud.publish(create_cloud_xyz32(o.header,[[position[0]+8,position[1]+8,.1],[position[0]-8,position[1]-8,.1]]))
            limit.publish(Float64(data=.18))
          if ns>=3_000_000_000 and not sent:
            g=PoseStamped();g.header=o.header;g.pose.orientation.w=1.;g.pose.position.x=80.;g.pose.position.z=1.8
            goal.publish(g);sent=True
          for _ in range(4):rclpy.spin_once(node,timeout_sec=0)
          time.sleep(.008)
        assert received,'No native MINCO candidate for recorded safe hover and distant B'
        m=received[0];assert m.representation==m.POLYNOMIAL
        first=np.array(m.coefficients[:18]).reshape(3,6)
        assert np.linalg.norm(first[:,5]-position)<1e-4,'Native start position changed'
        assert np.linalg.norm(first[:,4]-velocity)<1e-4,'Geometric guide conditioning changed actual native start velocity'
        assert np.linalg.norm(2*first[:,3])<1e-4,'Native start acceleration changed'
        # Native coefficients/time are used directly; verify the initial flying
        # prefix still stays inside the original hard execution heights.
        t=np.linspace(0,min(1.,m.durations[0]),100)
        z=np.polyval(first[2],t)
        assert z.min()>=1.5 and z.max()<=2.3,'Native initial prefix leaves the hard height envelope'
        print('Recorded hover vz=%+.8f: original MINCO start derivatives retained; distant goal produces native plan: PASS'%velocity[2],flush=True)
      except BaseException:
        logs.seek(0);print(logs.read().decode(),flush=True);raise
      finally:
        for p in processes:
          if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
        for p in processes:
          try:p.wait(timeout=3)
          except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--planner',type=Path,default=Path.home()/'.cache/gazebo_ego_v2/install/ego_planner/lib/ego_planner/ego_planner_node');args=parser.parse_args()
    for position,velocity in CASES:check(args.planner,position,velocity)
