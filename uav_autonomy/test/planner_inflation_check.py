#!/usr/bin/env python3
"""Black-box check of native map inflation and v2 ring-buffer clearing; no control."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID']=str(120+os.getpid()%80)
os.environ['ROS_LOCALHOST_ONLY']='1'
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
ROOT=Path(__file__).resolve().parents[2]

def main():
    backend=sys.argv[1];rclpy.init();n=Node('native_inflation_regression')
    clock=n.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    odom=n.create_publisher(Odometry,'/inflation/odom',10)
    cloud=n.create_publisher(PointCloud2,'/inflation/cloud',qos_profile_sensor_data)
    latest=[];current_ns=5_000_000_000
    def receive(m):
        data=read_points(m,field_names=('x','y','z'),skip_nans=True)
        if len(data):latest.append(np.column_stack([data[a] for a in ('x','y','z')]))
    n.create_subscription(PointCloud2,'/grid_map/occupancy_inflate',receive,qos_profile_sensor_data)
    config=ROOT/'uav_autonomy/config'/f'ego_{backend}.yaml'
    binary=Path.home()/f'.cache/gazebo_ego_{backend}/install/ego_planner/lib/ego_planner/ego_planner_node'
    args=[str(binary),'--ros-args','--params-file',str(config),'-r','odom_world:=/inflation/odom',
          '-r','grid_map/odom:=/inflation/odom','-r','grid_map/cloud:=/inflation/cloud']
    def run_phase(position, endpoint, duration=1.8):
        nonlocal current_ns
        latest.clear();end=time.monotonic()+duration
        while time.monotonic()<end:
            current_ns+=20_000_000;c=Clock();c.clock.sec,c.clock.nanosec=divmod(current_ns,10**9);clock.publish(c)
            o=Odometry();o.header=Header(stamp=c.clock,frame_id='odom');o.pose.pose.position.x=position;o.pose.pose.position.z=1.8;o.pose.pose.orientation.w=1.;odom.publish(o)
            cloud.publish(create_cloud_xyz32(o.header,[endpoint]));rclpy.spin_once(n,timeout_sec=.005);time.sleep(.015)
        assert latest,'Native inflated map was not published'
        return latest[-1]
    def contains(points,p):return float(np.linalg.norm(points-np.array(p),axis=1).min())<.03
    with tempfile.TemporaryFile() as log:
        process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(.5);points=run_phase(0.,[2.,2.,1.5])
            assert contains(points,[2.9,2.1,1.5]),'Cardinal obstacle margin was lost'
            assert not contains(points,[2.9,2.9,1.5]),'Inflation still includes distant cube corners'
            if backend=='v2':
                run_phase(30.,[32.,2.,1.5]);points=run_phase(0.,[4.,2.,1.5])
                assert not contains(points,[2.1,2.1,1.5]),'Out-of-window occupancy resurrected on return'
                assert contains(points,[4.1,2.1,1.5]),'Current observation missing after ring-buffer move'
            print(backend+': spherical inflation'+(' and ring-buffer clearing' if backend=='v2' else '')+': PASS')
        except BaseException:
            log.seek(0);print(log.read().decode(),file=sys.stderr);raise
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
