#!/usr/bin/env python3
"""Real task-node regression under synthetic odometry/TF, without PX4/Gazebo.

Arrive at the first inspection point, then observe an obstacle at the second.
Auto-selected tours must restart as a new four-point generation; explicit
operator points must remain unchanged and report a blocked target.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from uav_common_msg.msg import ExternalReference


def check(executable, mode, domain):
    os.environ['ROS_DOMAIN_ID']=str(domain)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=rclpy.create_node('patrol_revalidation_fixture')
    clock=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    odom=node.create_publisher(Odometry,'/autonomy/odom',10)
    controller=node.create_publisher(String,'/ganyu/status',10)
    cloud=node.create_publisher(PointCloud2,'/revalidation/points',qos_profile_sensor_data)
    mission=node.create_publisher(String,'/autonomy/mission_command',10)
    dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    sensor=TransformStamped();sensor.header.frame_id='base_link';sensor.child_frame_id='unregistered_test_sensor';sensor.transform.rotation.w=1.;static.sendTransform(sensor)
    state=dict(armed=False,position=[0.,0.,1.8],yaw=0.,phase='WAIT_DATA',requested=False)
    node.create_subscription(String,'/ganyu/keyboard',lambda m:state.update(armed=True) if m.data=='t' else None,10)
    node.create_subscription(ExternalReference,'/autonomy/reference',lambda m:state.update(yaw=m.yaw),100)
    node.create_subscription(String,'/autonomy/status',lambda m:state.update(status=json.loads(m.data),phase=json.loads(m.data)['phase']),10)
    explicit=[[2.,0.,2.2],[-2.,0.,2.6],[0.,2.,3.],[0.,-2.,3.4]]
    with tempfile.TemporaryDirectory(prefix='patrol_revalidation_') as temporary:
        folder=Path(temporary);log=(folder/'node.log').open('w')
        def goal(message):
            if 'old_points' in state:return
            state['old_points']=json.loads((folder/'patrol_points.json').read_text())
            state['position']=list(state['old_points'][0])
            state['blocked_target']=list(state['old_points'][1])
            state['first_arrival']=current
            state['saved_map_hash']=hashlib.sha256((folder/'warehouse.bt').read_bytes()).hexdigest()
        node.create_subscription(PoseStamped,'/autonomy/goal',goal,10)
        process=subprocess.Popen([executable,'--ros-args','-p','use_sim_time:=true','-p','auto_start:=true',
            '-p','task:=explore','-p','takeoff_height:=1.8','-p','flight_max_height:=5.8',
            '-p','launch_clearance_radius:=6.0','-p','launch_clearance_height:=6.0',
            '-p','cloud_topic:=/revalidation/points','-p','output_directory:='+temporary],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(.6);deadline=time.monotonic()+40;tick=0
            while time.monotonic()<deadline:
                tick+=1;ns=1_000_000_000+tick*20_000_000;current=ns/1e9
                c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
                p=Odometry();p.header=Header(stamp=c.clock,frame_id='odom');p.child_frame_id='base_link'
                x,y,z=state['position'] if state['armed'] else [0.,0.,0.]
                p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.position.z=z
                yaw=state['yaw'];p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2);odom.publish(p)
                tf=TransformStamped();tf.header=p.header;tf.child_frame_id='base_link';tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.translation.z=z;tf.transform.rotation=p.pose.pose.orientation;dynamic.sendTransform(tf)
                if tick%5==0:
                    controller.publish(String(data=json.dumps(dict(phase='ACTIVE' if state['armed'] else 'IDLE',ready=True,
                        armed=state['armed'],landed=not state['armed'],internal_reference_settled=True,target_ned=[y,x,-z]))))
                    points=[[8*math.cos(i*math.pi/8),8*math.sin(i*math.pi/8),0.] for i in range(16)]
                    if 'blocked_target' in state:
                        dx,dy,dz=[a-b for a,b in zip(state['blocked_target'],[x,y,z])]
                        points.append([math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,dz])
                    cloud.publish(create_cloud_xyz32(Header(stamp=c.clock,frame_id='unregistered_test_sensor'),points))
                for _ in range(4):rclpy.spin_once(node,timeout_sec=0)
                if state['phase']=='OBSERVE' and not state['requested']:
                    command=dict(action='patrol')
                    if mode=='requested':command['points']=explicit
                    mission.publish(String(data=json.dumps(command)));state['requested']=True
                if 'old_points' in state:
                    events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
                    points=json.loads((folder/'patrol_points.json').read_text())
                    if mode=='auto' and any(e['event']=='auto_patrol_replanned' for e in events) and state['status']['patrol_index']==0:
                        assert points!=state['old_points'] and len(points)==4
                        assert state['status']['patrol_index']==0,'Old patrol progress carried into a new generation'
                        assert json.loads((folder/'patrol_points.revision0.json').read_text())==state['old_points']
                        assert hashlib.sha256((folder/'warehouse.bt').read_bytes()).hexdigest()==state['saved_map_hash'],'Patrol repair replaced the saved exploration map'
                        assert any(e['event']=='auto_patrol_target_invalidated' and e['completed_points_discarded']==1 for e in events)
                        print('New obstacle invalidates automatic target: archived tour, new four-point generation, progress reset, saved map preserved: PASS');return
                    if mode=='requested' and state['phase']=='FAILED_HOVER':
                        assert state['status']['reason']=='requested_patrol_target_blocked'
                        assert points==explicit and not list(folder.glob('patrol_points.revision*.json'))
                        print('Blocked operator target reports failure without changing inspection points: PASS');return
                    if current-state['first_arrival']>30:
                        raise AssertionError('Invalidated '+mode+' patrol target kept cycling instead of a bounded response')
                if process.poll() is not None:raise AssertionError('Task node exited')
                time.sleep(.008)
            raise AssertionError('Fixture timed out in '+state['phase'])
        except BaseException:
            log.flush();print((folder/'node.log').read_text());raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('executable');parser.add_argument('--case',choices=['auto','requested']);args=parser.parse_args()
    for index,mode in enumerate([args.case] if args.case else ['auto','requested']):
        check(args.executable,mode,228+index)
