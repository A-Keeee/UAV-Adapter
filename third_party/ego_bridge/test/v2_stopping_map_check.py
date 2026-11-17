#!/usr/bin/python3
"""Real native rolling-map query via its published inflated voxels, no flight.

The witness obstacle/brake point comes from forest_mid360_0_attempt7.
Other probes cover subvoxel corners and all three inflation axes.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import math

os.environ['ROS_DOMAIN_ID'] = '77'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

ROOT = Path(__file__).resolve().parents[3]
RES = .2
OBSTACLE = np.array([36.9, 1.3, 1.9])
WITNESS = np.array([37.600273971460155, .3752420403409365, 2.0755426465339415])


def main():
    rclpy.init()
    node = rclpy.create_node('native_stopping_map_fixture')
    clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/probe/odom', 10)
    cloud = node.create_publisher(PointCloud2, '/probe/cloud', qos_profile_sensor_data)
    inflated = set()
    def receive(m):
        for xyz in read_points(m, field_names=['x', 'y', 'z']):
            point = np.array(list(xyz), dtype=float)
            # The lattice check catches upstream silently coarsening the map
            # when inflation exceeds four cells.
            cell = point / RES - .5
            assert np.max(np.abs(cell-np.rint(cell))) < 3e-4, 'Native map silently changed its configured 0.2 m resolution'
            inflated.add(tuple(np.floor(point/RES).astype(int)))
    node.create_subscription(PointCloud2, '/grid_map/occupancy_inflate', receive, 10)
    planner = Path.home()/'.cache/gazebo_ego_v2/install/ego_planner/lib/ego_planner/ego_planner_node'
    command = [str(planner), '--ros-args', '--params-file', str(ROOT/'uav_autonomy/config/ego_v2.yaml'),
        '-p', 'grid_map/enable_virtual_wall:=false',
        '-r', 'odom_world:=/probe/odom', '-r', 'grid_map/odom:=/probe/odom',
        '-r', 'grid_map/cloud:=/probe/cloud']
    with tempfile.TemporaryFile() as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(.5)
            for tick in range(1, 501):
                c = Clock(); ns = 1_000_000_000 + tick*20_000_000
                c.clock.sec, c.clock.nanosec = divmod(ns, 10**9); clock.publish(c)
                o = Odometry(); o.header = Header(stamp=c.clock, frame_id='odom')
                o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = [37.67, .04, 2.06]
                angle = (tick//20)*math.pi/4
                o.pose.pose.orientation.z = math.sin(angle/2); o.pose.pose.orientation.w = math.cos(angle/2)
                odom.publish(o)
                if tick % 5 == 0:
                    cloud.publish(create_cloud_xyz32(o.header, [OBSTACLE.tolist()]))
                for _ in range(4): rclpy.spin_once(node, timeout_sec=0)
                time.sleep(.006)
            witness = tuple(np.floor(WITNESS/RES).astype(int))
            assert inflated, 'Native inflated map never published'
            assert witness in inflated, 'Recorded stopping-unsafe point is native-map free'
            radius = .65 + .15 + RES + .866025403784*RES
            checked = 0
            # Every queried point forbidden by the unchanged independent
            # stopping clearance must also be forbidden to the planner.
            for x in np.arange(-1.3, 1.31, .1):
                for y in np.arange(-1.3, 1.31, .1):
                    for z in np.arange(-1.3, 1.31, .1):
                        offset = np.array([x, y, z])
                        if np.linalg.norm(offset) > radius-1e-6: continue
                        point = OBSTACLE + offset
                        assert tuple(np.floor(point/RES+1e-8).astype(int)) in inflated, 'A stopping-unsafe subvoxel point escaped native inflation'
                        checked += 1
            assert tuple(np.floor((OBSTACLE + [2.5, 0, 0])/RES).astype(int)) not in inflated, 'Native inflation incorrectly blocks distant free space'
            print(f'Recorded stopping witness and {checked} subvoxel/three-axis probes blocked at original 0.2 m resolution: PASS')
        except BaseException:
            log.seek(0); print(log.read().decode()); raise
        finally:
            if child.poll() is None: os.killpg(child.pid, signal.SIGTERM)
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
