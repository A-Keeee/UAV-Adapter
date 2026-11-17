#!/usr/bin/env python3
"""Run current interface checks in an isolated ROS domain, without a vehicle.

Synthetic timestamped XYZ and TF exercise transport and parser behavior. These
results are interface evidence only and never count as Gazebo flight acceptance.
"""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
from autonomy_provenance import runtime_source_hashes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--os1-snapshot',type=Path,help='Prepared sparse.msg; sensor metadata stays in this test injector.')
    parser.add_argument('--case',help='Run a single named interface case.')
    parser.add_argument('--domain',type=int,choices=range(210,220),help='Explicit isolated domain for transport regressions.')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    executable=ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node'
    binary_hash=hashlib.sha256(executable.read_bytes()).hexdigest()
    runtime_hashes=runtime_source_hashes()
    os.environ['ROS_DOMAIN_ID']=str(args.domain if args.domain is not None else 210+os.getpid()%10)
    # Humble's localhost flag appends another default-sized SHM transport,
    # defeating the large-frame profile. Enforce loopback in XML instead.
    # https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp
    os.environ['ROS_LOCALHOST_ONLY']='0'
    # Scope the large-data transport to this unarmed fixture and its children.
    # Default SHM segments cannot hold a complete resource-limit test frame.
    transport_profile=ROOT/'scripts/interface_fastdds.xml'
    os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(transport_profile)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2,PointField
    from rclpy.serialization import deserialize_message
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

    def message(points):
        cloud=PointCloud2();cloud.header.frame_id='fixture_lidar'
        cloud.height=1;cloud.width=len(points);cloud.point_step=12;cloud.row_step=12*len(points)
        cloud.fields=[PointField(name=name,offset=4*i,datatype=7,count=1) for i,name in enumerate('xyz')]
        cloud.data=np.asarray(points,dtype='<f4').tobytes();cloud.is_dense=False
        return cloud

    rclpy.init();node=Node('isolated_cloud_fixture')
    # Verify that the intended participant transport was actually instantiated;
    # an ignored/invalid XML profile must not silently fall back to tiny buffers.
    shm_sizes=[]
    for line in Path('/proc/self/maps').read_text().splitlines():
        fields=line.split(maxsplit=5)
        if len(fields)==6 and re.fullmatch(r'/dev/shm/fastrtps_[0-9a-f]{16}',fields[5]):
            path=Path(fields[5].removesuffix(' (deleted)'))
            if path.exists():shm_sizes.append(path.stat().st_size)
    if not shm_sizes or min(shm_sizes)<268435456:
        node.destroy_node();rclpy.shutdown()
        raise RuntimeError('The isolated large-frame shared-memory profile is not active')
    clock=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    odom=node.create_publisher(Odometry,'/autonomy/odom',10)
    dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    sensor=TransformStamped();sensor.header.frame_id='base_link';sensor.child_frame_id='fixture_lidar'
    sensor.transform.translation.x=.2;sensor.transform.translation.z=.1;sensor.transform.rotation.w=1.
    static.sendTransform(sensor)
    angles=np.arange(160)*2*np.pi/160
    points=np.stack([6*np.cos(angles),6*np.sin(angles),np.zeros(len(angles))],axis=1)
    nominal=message(points)
    mixed=PointCloud2();mixed.header=nominal.header;mixed.height=1;mixed.width=len(points);mixed.point_step=32;mixed.row_step=32*len(points)
    mixed.fields=[PointField(name='z',offset=0,datatype=7,count=1),PointField(name='reflectance',offset=4,datatype=4,count=1),
        PointField(name='x',offset=8,datatype=7,count=1),PointField(name='y',offset=16,datatype=8,count=1)]
    buffer=bytearray(mixed.row_step)
    for i,(x,y,z) in enumerate(points):
        struct.pack_into('<f',buffer,32*i,z);struct.pack_into('<f',buffer,32*i+8,x);struct.pack_into('<d',buffer,32*i+16,y)
    mixed.data=bytes(buffer)
    invalid=points.copy();invalid[::4,0]=np.nan;nonfinite=message(invalid)
    indices=np.arange(80000);z=1-2*(indices+.5)/80000;a=indices*np.pi*(3-np.sqrt(5));r=np.sqrt(1-z*z)
    dense=message(8*np.stack([r*np.cos(a),r*np.sin(a),z],axis=1))
    oversized=message(np.zeros((4000001,3),dtype=np.float32))
    clouds={'/lidar/points':nominal,'/fixture/mixed':mixed,'/fixture/nonfinite':nonfinite,
        '/fixture/dense':dense,'/fixture/oversized':oversized}
    replay_metadata=None
    if args.os1_snapshot:
        replay_metadata=json.loads(args.os1_snapshot.with_name('channel_selection.json').read_text())
        source_data=args.os1_snapshot.read_bytes()
        if hashlib.sha256(source_data).hexdigest()!=replay_metadata['sparse_cloud_sha256']:raise RuntimeError('Recorded replay bytes changed')
        replay=deserialize_message(source_data,PointCloud2);replay.header.frame_id='fixture_lidar'
        clouds['/fixture/os1_sparse']=replay
    publishers={name:node.create_publisher(PointCloud2,name,qos_profile_sensor_data) for name in clouds}
    begun=time.monotonic();last_cloud={name:0. for name in clouds}
    publications={name:0 for name in clouds}
    def tick():
        now=time.monotonic();ns=int((1+now-begun)*1e9)
        c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
        p=Odometry();p.header.stamp=c.clock;p.header.frame_id='odom';p.child_frame_id='base_link'
        p.pose.pose.position.z=1.8;p.pose.pose.orientation.w=1.;odom.publish(p)
        transform=TransformStamped();transform.header=p.header;transform.child_frame_id='base_link'
        transform.transform.translation.z=1.8;transform.transform.rotation.w=1.;dynamic.sendTransform(transform)
        for name,publisher in publishers.items():
            # A real 48 MB over-budget frame must reach the consumer before
            # its rejection can be tested. Do not flood the best-effort link
            # with a new fragmented frame every half second.
            period=2. if name=='/fixture/oversized' else .5 if name=='/fixture/dense' else .1
            if publisher.get_subscription_count() and now-last_cloud[name]>=period:
                clouds[name].header.stamp=c.clock;publisher.publish(clouds[name]);last_cloud[name]=now
                publications[name]+=1
    node.create_timer(.02,tick)
    child=None
    try:
        with (args.output/'verifier.log').open('w') as log:
            command=[sys.executable,str(ROOT/'scripts/verify_cloud_interface.py'),'--output',str(args.output),'--extended']
            if args.os1_snapshot:command.append('--os1-sparse-replay')
            if args.case:command+=['--case',args.case]
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+240
            while child.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            if child.poll() is None:raise RuntimeError('Isolated interface validation timed out')
            results=json.loads((args.output/'results.json').read_text())
            if replay_metadata is not None:
                replay_metadata=dict(replay_metadata,interface_replay_verified=any(r['case']=='os1_sparse_replay' and r['passed'] for r in results))
            summary=dict(passed=sum(bool(r['passed']) for r in results),total=len(results),
                flight_acceptance=False,source='Synthetic fixtures and optional recorded OS1 scan replay, under fixture odometry and acquisition-time TF',
                ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),isolated_from_live_vehicle=True,
                transport_profile_sha256=hashlib.sha256(transport_profile.read_bytes()).hexdigest(),
                source_shm_segment_sizes=shm_sizes,
                loopback_enforced_by_transport_profile=True,
                publications=publications,
                autonomy_binary_sha256=binary_hash,runtime_sources_sha256=runtime_hashes,
                recorded_os1_channel_selection=replay_metadata,
                recorded_os1_snapshot=str(args.os1_snapshot.resolve()) if args.os1_snapshot else None,
                implementation_constant_during_test=(hashlib.sha256(executable.read_bytes()).hexdigest()==binary_hash and runtime_source_hashes()==runtime_hashes))
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps(summary),flush=True)
            expected=1 if args.case else 18 if args.os1_snapshot else 17
            if child.returncode or summary['passed']!=summary['total'] or summary['total']!=expected or not summary['implementation_constant_during_test']:raise SystemExit(1)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
