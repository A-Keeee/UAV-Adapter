#!/usr/bin/env python3
"""Reject stopped/old-goal trajectories and preserve each native representation."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID'] = str(120+os.getpid()%80)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import PoseStamped, Point
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt64
from uav_common_msg.msg import NativeTrajectory, TrajectoryFeedback
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
if sys.argv[1]=='v2':
    from traj_utils.msg import PolyTraj as Input
else:
    from traj_utils.msg import Bspline as Input


def main():
    backend = sys.argv[1]; rclpy.init(); n = Node('bridge_gate_regression')
    inp = n.create_publisher(Input, '/ego/native_trajectory', 10)
    goal = n.create_publisher(PoseStamped, '/autonomy/goal', 10)
    feedback = n.create_publisher(TrajectoryFeedback, '/autonomy/trajectory_feedback', 10)
    clock = n.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    received = []; acknowledgements = []
    n.create_subscription(UInt64, "/autonomy/native_accepted", lambda m: acknowledgements.append(m.data), 10)
    n.create_subscription(NativeTrajectory, '/autonomy/native_trajectory', received.append, 10)
    def stamp(t):
        out = Clock(); ns=round(t*1e9); out.clock.sec,out.clock.nanosec = divmod(ns,10**9); return out.clock
    def advance(t, duration=.25):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            clock.publish(Clock(clock=stamp(t))); rclpy.spin_once(n,timeout_sec=.01)
    def trajectory(t):
        m = Input(); m.start_time = stamp(t); m.traj_id=round(t*1000); m.order = 5 if backend=='v2' else 3
        if backend=='v2':
            m.duration = [1.]; m.coef_x = [0.,0.,0.,0.,.25,1.]; m.coef_y = [0.]*6; m.coef_z = [0.]*6
        else:
            m.knots = [0.,0.,0.,0.,1.,1.,1.,1.]; m.pos_pts = [Point(x=float(i),y=0.,z=1.) for i in range(4)]
        inp.publish(m)
    def new_goal(t):
        g = PoseStamped(); g.header.frame_id='odom'; g.header.stamp=stamp(t); g.pose.position.x=5.; g.pose.orientation.w=1.; goal.publish(g); advance(t,.2)
    binary = Path.home()/f'.cache/gazebo_ego_{backend}/install/ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge'
    with tempfile.TemporaryFile() as log:
        p=subprocess.Popen([str(binary),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            advance(10.,1.)
            trajectory(9.); advance(10.)
            assert not received, 'Stopped planner trajectory was forwarded'
            new_goal(10.); advance(10.2); trajectory(9.); advance(10.2)
            assert not received, 'Previous goal trajectory was forwarded'
            trajectory(10.2); advance(10.2)
            assert len(received)==1, 'Current goal trajectory was lost'
            m=received[0]
            assert not acknowledgements,'Planner was acknowledged before controller acceptance'
            feedback.publish(TrajectoryFeedback(trajectory_id=m.trajectory_id,accepted=True));advance(10.2)
            assert acknowledgements==[10200], 'Controller acceptance was not mapped to the native trajectory ID'
            if backend=='v2':
                assert list(m.durations)==[1.] and list(m.coefficients[:6])==[0.,0.,0.,0.,.25,1.]
            else:
                assert list(m.knots)==[0.,0.,0.,0.,1.,1.,1.,1.] and list(m.control_points[:6])==[0.,0.,1.,1.,0.,1.]
            feedback.publish(TrajectoryFeedback(accepted=False)); advance(10.3); trajectory(10.3); advance(10.3)
            assert len(received)==1, 'Rejected execution kept forwarding trajectories'
            new_goal(11.); advance(11.); advance(11.2); trajectory(10.5); advance(11.2)
            assert len(received)==1, 'Old trajectory leaked into the replacement goal'
            trajectory(11.2); advance(11.2)
            assert len(received)==2 and received[1].trajectory_id>received[0].trajectory_id
            print(backend+': stopped/old-goal filtering and exact native transport: PASS')
        except BaseException:
            log.seek(0); print(log.read().decode(),file=sys.stderr); raise
        finally:
            if p.poll() is None: os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
