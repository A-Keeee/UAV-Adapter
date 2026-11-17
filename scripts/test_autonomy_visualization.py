#!/usr/bin/env python3
"""Isolated ROS fixtures and curve checks; no Gazebo, PX4 or controller."""
import json
import math
import os
import time
from types import SimpleNamespace

import numpy as np
from visualization_trajectory import sample_native


def spline(ident=42):
    from uav_common_msg.msg import NativeTrajectory
    m = NativeTrajectory()
    m.header.frame_id = 'odom'
    m.trajectory_id = ident
    m.start_time.sec = 1
    m.degree = 3
    m.representation = m.BSPLINE
    m.knots = [0., 0., 0., 0., 3., 3., 3., 3.]
    m.control_points = [0., 0., 2., 1., 2., 2., 2., -1., 3., 3., 0., 4.]
    return m


def polynomial():
    from uav_common_msg.msg import NativeTrajectory
    m = NativeTrajectory()
    m.header.frame_id = 'odom'
    m.trajectory_id = 43
    m.start_time.sec = 1
    m.degree = 5
    m.representation = m.POLYNOMIAL
    m.durations = [1., 2.]
    # Each piece: x = local time + previous duration, y = 0, z = 2.
    m.coefficients = [0.,0.,0.,0.,1.,0., 0.,0.,0.,0.,0.,0., 0.,0.,0.,0.,0.,2.,
                      0.,0.,0.,0.,1.,1., 0.,0.,0.,0.,0.,0., 0.,0.,0.,0.,0.,2.]
    return m


def main():
    os.environ['ROS_DOMAIN_ID'] = '231'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from uav_common_msg.msg import NativeTrajectory, ExternalReference, TrajectoryFeedback
    from visualization_msgs.msg import MarkerArray
    from autonomy_visualization import AutonomyVisualization, PREFIX
    checks = []
    m = spline()
    t, xyz = sample_native(m)
    u = (t/3.)[:,None]
    p = np.asarray(m.control_points).reshape(-1,3)
    expected = (1-u)**3*p[0]+3*(1-u)**2*u*p[1]+3*(1-u)*u*u*p[2]+u**3*p[3]
    assert np.allclose(xyz,expected)
    checks.append('native_bspline_geometry_and_original_time')
    m.knots = [-3.,-2.,-1.,0.,1.,2.,3.,4.]
    m.control_points = [0.,0.,0.,1.,1.,1.,2.,2.,2.,3.,3.,3.]
    t,xyz = sample_native(m)
    assert np.allclose(xyz[:,0],1+t) and np.allclose(xyz[:,1],1+t)
    checks.append('unclamped_negative_original_knots')
    t, xyz = sample_native(polynomial())
    assert np.allclose(xyz[:,0],t) and np.allclose(xyz[:,2],2.) and t[-1]==3.
    checks.append('original_quintic_segments_and_piece_boundary')
    assert len(sample_native(spline(),max_points=12)[0])==12
    bad = spline();bad.control_points[0]=float('nan')
    for m in (bad, spline()):
        if m is not bad:m.header.frame_id='ned'
        try:sample_native(m)
        except ValueError:pass
        else:raise AssertionError('Bad display curve accepted')
    bad = spline();bad.control_points=[0.]*12291
    try:sample_native(bad)
    except ValueError:pass
    else:raise AssertionError('Unbounded display work')
    checks.append('invalid_frames_nonfinite_and_resource_limits_rejected')

    rclpy.init()
    fixture = Node('visualization_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    visual = None
    try:
        start=time.monotonic()
        while time.monotonic()-start<1.:
            executor.spin_once(timeout_sec=.05)
        assert not [(n,s) for n,s in fixture.get_node_names_and_namespaces() if n!=fixture.get_name()], 'Isolated domain is busy'
        visual = AutonomyVisualization();executor.add_node(visual)
        clock = fixture.create_publisher(Clock,'/clock',qos_profile_sensor_data)
        odom = fixture.create_publisher(Odometry,'/autonomy/odom',10)
        native = fixture.create_publisher(NativeTrajectory,'/autonomy/native_trajectory',10)
        feedback = fixture.create_publisher(TrajectoryFeedback,'/autonomy/trajectory_feedback',10)
        control = fixture.create_publisher(ExternalReference,'/ganyu/control_reference',10)
        status = fixture.create_publisher(String,'/autonomy/status',10)
        received = {}
        for name in ('actual_path','reference_path','candidate_path','confirmed_path'):
            fixture.create_subscription(Path,PREFIX+name,lambda m,n=name:received.__setitem__(n,m),10)
        fixture.create_subscription(MarkerArray,PREFIX+'markers',lambda m:received.__setitem__('markers',m),10)
        def pump(seconds=.7, sim=2.):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                c=Clock();c.clock.sec=int(sim);c.clock.nanosec=int((sim-int(sim))*1e9);clock.publish(c)
                executor.spin_once(timeout_sec=.01)
        pump()
        m=Odometry();m.header.frame_id='odom';m.header.stamp.sec=2;m.child_frame_id='base_link'
        m.pose.pose.orientation.w=1.;m.pose.pose.position.z=2.;m.twist.twist.linear.x=.3
        odom.publish(m)
        ref=ExternalReference();ref.header=m.header;ref.position=[.1,0.,2.];control.publish(ref)
        status.publish(String(data=json.dumps(dict(phase='TRANSIT',planning_speed_limit_mps=.5))))
        native.publish(spline());pump()
        assert len(received['actual_path'].poses)==1 and len(received['reference_path'].poses)==1
        assert len(received['candidate_path'].poses)>2 and not received['confirmed_path'].poses
        checks.append('actual_reference_and_candidate_are_distinct_ros_topics')
        f=TrajectoryFeedback();f.trajectory_id=42;f.accepted=True;feedback.publish(f);pump()
        assert received['confirmed_path'].poses
        native.publish(polynomial());f.trajectory_id=43;f.accepted=False;feedback.publish(f);pump()
        assert received['confirmed_path'].poses[-1].pose.position.z==4.
        checks.append('confirmed_curve_requires_feedback_and_survives_soft_rejection')
        f.braking=True;feedback.publish(f);pump()
        assert not received['confirmed_path'].poses and not received['candidate_path'].poses
        f.braking=False;f.accepted=True;f.trajectory_id=42;feedback.publish(f);native.publish(spline());pump()
        assert not received['confirmed_path'].poses
        checks.append('brake_and_late_old_ack_cannot_restore_displayed_confirmation')
        for i in range(5000):
            visual.append_history(visual.actual,3.+i*.11,(i*.03,0.,2.))
        assert len(visual.actual)==3000
        checks.append('flight_history_has_fixed_memory_bound')
        visual.reset();visual.on_odom(m);visual.on_reference(ref)
        visual.on_clock(SimpleNamespace(clock=SimpleNamespace(sec=10,nanosec=0)))
        visual.on_clock(SimpleNamespace(clock=SimpleNamespace(sec=1,nanosec=0)))
        assert not visual.actual and not visual.reference and visual.confirmed is None
        checks.append('clock_rollback_clears_previous_epoch_visuals')
        status.publish(String(data='not json'));pump()
        assert 'markers' in received
        topics={p.topic_name for p in visual.publishers}
        assert all(t.startswith(PREFIX) or t in ('/rosout','/parameter_events') for t in topics),topics
        checks.append('read_only_adapter_has_no_control_goal_or_tf_publishers')
        print(json.dumps(dict(passed=True,checks=checks,ros_domain_id=231,flight_started=False),ensure_ascii=False))
    finally:
        executor.shutdown()
        if visual is not None:visual.destroy_node()
        fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
