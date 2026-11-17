#!/usr/bin/env python3
"""Exercise real task-node takeoff/scan/goal transitions without an EGO reply.

This is a ROS integration regression, not a flight or clearance acceptance test.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

# Linux's default ephemeral UDP ports overlap DDS domains 102--214. Keep
# fixture traffic separate from the live vehicle and outside that port range.
os.environ['ROS_DOMAIN_ID'] = str(220+os.getpid()%8)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String, Empty
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from uav_common_msg.msg import ExternalReference, NativeTrajectory, TrajectoryFeedback


def main():
    rclpy.init()
    node = Node('reference_liveness_regression')
    clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/autonomy/odom', 10)
    status = node.create_publisher(String, '/ganyu/status', 10)
    cloud = node.create_publisher(type(create_cloud_xyz32(Header(), [])), '/regression/points', qos_profile_sensor_data)
    tf = TransformBroadcaster(node); static = StaticTransformBroadcaster(node)
    sensor = TransformStamped(); sensor.header.frame_id = 'base_link'; sensor.child_frame_id = 'anonymous_sensor'
    sensor.transform.rotation.w = 1.; static.sendTransform(sensor)
    state = dict(takeoff_at=None, first_reference=None, goal_at=None, turn_injected_at=None, resume_at=None, brakes=0, injected=0, moved=False, patrol_requested=False, planning_status=[], position=[0.,0.,1.8], viewing_refs=[], yaw=0., refs=[], phase='WAIT_DATA')
    def command(m):
        if m.data=='t' and state['takeoff_at'] is None: state['takeoff_at'] = current
        if m.data=='r': state['resume_at'] = current
        if m.data=='c': state['arming_cancelled'] = True
    def reference(m):
        stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if height_view and state['phase']=='TRANSIT' and not (1.6-1e-6 <= m.position[2] <= 2.2+1e-6):
            raise AssertionError('Native reference consumed the height stopping reserve')
        if state['first_reference'] is None: state['first_reference'] = stamp
        state['yaw'] = m.yaw; state['refs'].append(stamp)
        if tail and state['injected'] and state['phase']=='TRANSIT':
            state['position']=list(m.position);state.setdefault('tail_refs',[]).append((stamp,list(m.position),m.valid_until.sec+m.valid_until.nanosec*1e-9))
        if replacement and state.get('replacement_started') and state['phase']=='TRANSIT':
            state['position']=list(m.position);state.setdefault('replacement_refs',[]).append((stamp,list(m.position),m.trajectory_id))
        if viewing and state['phase']=='OBS_MOVE': state['position']=list(m.position);state['moved']=True;state['viewing_refs'].append(stamp)
        if state['phase']=='TURN' and state['turn_injected_at'] is None: state['turn_injected_at'] = stamp
    node.create_subscription(String, '/ganyu/keyboard', command, 10)
    node.create_subscription(Empty, '/autonomy/brake', lambda m: state.update(brakes=state['brakes']+1), 10)
    node.create_subscription(ExternalReference, '/autonomy/reference', reference, 100)
    def planned_goal(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        state['goal_at']=stamp
        state.setdefault('goals',[]).append((stamp,[m.pose.position.x,m.pose.position.y,m.pose.position.z]))
    node.create_subscription(PoseStamped, '/autonomy/goal', planned_goal, 10)
    def task_status(m):
        data=json.loads(m.data);state['phase']=data['phase'];state['reason']=data['reason'];state['speed_limit']=data['planning_speed_limit_mps']
        if offset and not data['current_safe'] and data.get('controller_hold_safe'):
            state['saw_supported_hold_only']=True
        if indoor and state['patrol_requested']:
            state['planning_status'].append((current,data['data_ready'],data['odom_age']))
    node.create_subscription(String, '/autonomy/status', task_status, 10)
    mission = node.create_publisher(String, '/autonomy/mission_command', 10)
    state_status = node.create_publisher(String, '/autonomy/state_status', 10)
    native = node.create_publisher(NativeTrajectory,'/autonomy/native_trajectory',10)
    current = 1.; process = None; indoor_view='--indoor-viewing' in sys.argv[2:]; patrol_view='--patrol-viewing' in sys.argv[2:]; stalled='--planner-stall' in sys.argv[2:] or patrol_view; offset='--viewing-offset' in sys.argv[2:]; viewing = '--viewing-move' in sys.argv[2:] or offset or stalled or indoor_view; rear = '--rear' in sys.argv[2:]; gap = '--takeoff-gap' in sys.argv[2:]; indoor = '--indoor-route' in sys.argv[2:]; alignment = '--initial-alignment' in sys.argv[2:]; tail = '--tail-envelope' in sys.argv[2:]; replacement='--replacement-rejection' in sys.argv[2:]
    speed_reduction='--replacement-speed-reduction' in sys.argv[2:]
    replacement=replacement or speed_reduction
    height_view='--height-viewing' in sys.argv[2:]
    stopping_view='--stopping-viewing' in sys.argv[2:]
    recover_view='--viewing-move' in sys.argv[2:] or height_view
    viewing = viewing or height_view or stopping_view
    if recover_view:
        node.create_subscription(TrajectoryFeedback,'/autonomy/trajectory_feedback',lambda m: state.setdefault('retry_feedback',[]).append((m.trajectory_id,m.accepted,m.reason)),10)
    if replacement:
        recorded=json.loads((Path(__file__).parent/'data/os1_return_replacements.json').read_text())['trajectories']
        c=recorded[0]['control_points']
        shift=[state['position'][a]-(c[a]+4*c[3+a]+c[6+a])/6 for a in range(3)]
    patrol_points=[[2.,0.,2.2],[-2.,0.,2.6],[0.,2.,3.],[0.,-2.,3.4]]
    clock_reset='--clock-reset' in sys.argv[2:]
    takeoff_stall='--takeoff-stall' in sys.argv[2:]
    partial_stall='--partial-takeoff-stall' in sys.argv[2:]
    arm_stall='--takeoff-arm-stall' in sys.argv[2:]
    takeoff_fault=takeoff_stall or partial_stall or arm_stall
    if offset:state['position'][2]=1.84
    with tempfile.TemporaryDirectory(prefix='autonomy_liveness_') as folder:
        log = open(Path(folder)/'node.log', 'w')
        try:
            command=[sys.argv[1], '--ros-args', '-p', 'use_sim_time:=true', '-p', 'task:='+('explore' if indoor or indoor_view or patrol_view else 'forest'),
                '-p', 'auto_start:=true', '-p', 'cloud_topic:=/regression/points', '-p', 'takeoff_height:='+('1.84' if offset else '1.8'),
                '-p', 'goal:=[0.0,80.0,1.8]', '-p', 'launch_clearance_radius:='+('9.0' if indoor or tail else '1.3'), '-p', 'launch_clearance_height:='+('2.1' if offset else ('8.0' if indoor or tail else '3.0')),
                '-p', 'output_directory:='+folder]
            if patrol_view:command += ['-p','launch_clearance_radius:=4.0','-p','launch_clearance_height:=5.0']
            if tail or height_view:command += ['-p','execution_min_z:=1.5','-p','execution_max_z:=2.3']
            if replacement:command += ['-p','launch_clearance_radius:=9.0','-p','launch_clearance_height:=8.0','-p','max_velocity:='+('0.5' if speed_reduction else '0.18')]
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(.6)
            deadline = time.monotonic()+(50 if takeoff_fault else (65 if stalled else (60 if viewing or indoor else 30)))
            tick = 0;clock_offset=0
            while time.monotonic()<deadline:
                tick += 1; stamp_ns = 1_000_000_000+tick*20_000_000+clock_offset; current = stamp_ns/1e9
                msg = Clock(); msg.clock.sec = stamp_ns//1_000_000_000; msg.clock.nanosec = stamp_ns%1_000_000_000
                clock.publish(msg)
                armed = state['takeoff_at'] is not None and not arm_stall
                z = state['position'][2] if armed else 0.
                if offset and armed:z+=.02
                if takeoff_stall or arm_stall:z=0.
                if partial_stall and armed:z=.8
                heading = state['yaw']
                pose = Odometry(); pose.header.stamp = msg.clock; pose.header.frame_id = 'odom'; pose.child_frame_id = 'base_link'
                pose.pose.pose.position.z = z; pose.pose.pose.orientation.z = math.sin(heading/2); pose.pose.pose.orientation.w = math.cos(heading/2)
                if state['turn_injected_at'] and current-state['turn_injected_at']<.4: pose.twist.twist.linear.x = .10
                if viewing or tail or replacement:
                    pose.pose.pose.position.x=state['position'][0];pose.pose.pose.position.y=state['position'][1]
                odom.publish(pose)
                trans = TransformStamped(); trans.header = pose.header; trans.child_frame_id = 'base_link'
                trans.transform.translation.x=pose.pose.pose.position.x;trans.transform.translation.y=pose.pose.pose.position.y;trans.transform.translation.z = z; trans.transform.rotation = pose.pose.pose.orientation; tf.sendTransform(trans)
                settled = armed and current-(state['resume_at'] or state['takeoff_at'])>=2
                if tick%5==0:
                    status.publish(String(data=json.dumps(dict(phase=('WAIT_ARM' if arm_stall and state['takeoff_at'] is not None else ('ACTIVE' if armed else 'IDLE')), ready=True,
                        armed=armed, landed=not armed, internal_reference_settled=settled, target_ned=[state['position'][1],state['position'][0],-state['position'][2] if armed else 0.]))))
                    points=([[.7,0.,0.]] if state.get('replacement_obstacle') else [[8*math.cos(a),8*math.sin(a),0.] for a in ([math.pi+(i-8)*.04 for i in range(17)] if rear else [i*math.pi/8 for i in range(16)])])
                    if stopping_view and state['goal_at']:
                        # A fixed measured obstacle is outside base clearance
                        # but inside the forest stopping reserve. Convert its
                        # world position to the moving anonymous sensor frame.
                        dx,dy=1.1-state['position'][0],-state['position'][1]
                        points.append([math.cos(heading)*dx+math.sin(heading)*dy,
                                       -math.sin(heading)*dx+math.cos(heading)*dy,1.8-z])
                    if not (gap and armed and .2<=current-state['takeoff_at']<1.8): cloud.publish(create_cloud_xyz32(Header(stamp=msg.clock, frame_id='anonymous_sensor'),points))
                for _ in range(4): rclpy.spin_once(node, timeout_sec=0)
                if takeoff_fault:
                    if state['phase']=='FAILED_HOVER':
                        expected='takeoff_no_liftoff' if takeoff_stall else 'takeoff_did_not_reach_height'
                        assert state['reason']==expected,state['reason']
                        assert state['goal_at'] is None and state['first_reference'] is None,'Failed takeoff reached exploration/EGO execution'
                        elapsed=current-state['takeoff_at'];assert elapsed<48.,'Failed takeoff consumed the exploration time budget'
                        if takeoff_stall:assert 15<elapsed<17. and state['brakes']>0
                        else:assert elapsed>45.
                        if arm_stall:
                            # Task status and cancel are different DDS topics.
                            if not state.get('arming_cancelled'):
                                time.sleep(.008);continue
                        else:assert state['brakes']>0,'Stalled climb did not revoke control references'
                        result=json.loads((Path(folder)/'result.json').read_text())
                        assert result['success'] is False and result['reason']==expected
                        print(('Unarmed request cancelled' if arm_stall else ('Partial lift' if partial_stall else 'Recorded no lift despite active controller'))+': bounded takeoff failure, no exploration/false success, PASS');return
                    time.sleep(.008);continue
                if state.get('clock_jumped_at') is not None:
                    if current-state['clock_jumped_at']>.8:
                        assert state['phase']=='FAILED_HOVER' and state.get('reason')=='simulation_clock_moved_backwards','Task retained an old-clock mission after time rollback'
                        assert state['brakes']>state['brakes_before_reset'],'Task did not revoke execution on time rollback'
                        assert len(state['refs'])<=state['refs_before_reset']+3,'Task continued publishing old-clock references'
                        print('Backward simulation clock terminates the mission and revokes old references: PASS');return
                    time.sleep(.008);continue
                if alignment and armed and current-state['takeoff_at']>.5 and not state.get('alignment_injected'):
                    state_status.publish(String(data=json.dumps(dict(reason='ekf_origin_reset',displacement=.0001,rotation_jump=.15))))
                    state['alignment_injected']=True
                if indoor and state['phase']=='OBSERVE' and state['first_reference'] and not state['patrol_requested']:
                    # Synthetic known free launch volume exercises expensive,
                    # distant three-dimensional connections, without Gazebo truth.
                    mission.publish(String(data=json.dumps(dict(action='patrol',points=[[5.,5.,6.],[-5.,5.,5.5],[-5.,-5.,5.],[5.,-5.,4.5]]))))
                    state['patrol_requested']=True
                if patrol_view and state['phase']=='OBSERVE' and state['first_reference'] and not state['patrol_requested']:
                    mission.publish(String(data=json.dumps(dict(action='patrol',points=patrol_points))))
                    state['patrol_requested']=True
                if replacement and state['goal_at'] and state['phase']=='TRANSIT':
                    index=0 if not state.get('replacement_started') else 1
                    if index==0 or (not state.get('replacement_rejected_at') and current-state['replacement_started']>=.516):
                        source=recorded[index];m=NativeTrajectory();m.header=Header(stamp=msg.clock,frame_id='odom');m.start_time=msg.clock
                        m.trajectory_id=source['id'];m.representation=m.BSPLINE;m.degree=source['degree'];m.knots=source['knots']
                        m.control_points=[v+shift[i%3] for i,v in enumerate(source['control_points'])]
                        native.publish(m)
                        if index==0:state['replacement_started']=current;state['brakes_at_replacement']=state['brakes']
                        else:state['replacement_rejected_at']=current
                if replacement and state.get('replacement_rejected_at') and current-state['replacement_rejected_at']>4 and not state.get('replacement_guard_test'):
                    events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                    assert any(e['event']=='replacement_rejected_current_retained' and e['rejected_id']==4246 for e in events),'Invalid replacement cancelled the accepted recorded trajectory'
                    refs=[r for r in state.get('replacement_refs',[]) if r[0]>state['replacement_rejected_at']]
                    assert len(refs)>20 and max(b[0]-a[0] for a,b in zip(refs,refs[1:]))<.12,'Reference stream stopped on candidate rejection'
                    assert math.dist(refs[0][1],refs[-1][1])>.03,'Retained trajectory did not keep moving'
                    assert len({r[2] for r in refs})==1 and state['brakes']==state['brakes_at_replacement'],'Rejected candidate preempted the current controller reference'
                    if speed_reduction:
                        m=NativeTrajectory();m.header=Header(stamp=msg.clock,frame_id='odom');m.start_time=msg.clock;m.trajectory_id=4247
                        m.representation=m.POLYNOMIAL;m.degree=5;m.durations=[2.];m.coefficients=[0.]*18
                        m.coefficients[4]=2.;m.coefficients[5]=state['position'][0];m.coefficients[11]=state['position'][1];m.coefficients[17]=state['position'][2]
                        native.publish(m)
                    state['replacement_obstacle']=not speed_reduction;state['replacement_guard_test']=True;state['obstacle_at']=current
                if replacement and state.get('replacement_guard_test') and state['brakes']>state['brakes_at_replacement'] and (not speed_reduction or state.get('speed_limit',.5)<.5):
                    print('Recorded invalid replacement preserves safe native references; '+('lowered speed cap' if speed_reduction else 'a new obstacle')+' still brakes: PASS');return
                if replacement and state.get('obstacle_at') and current-state['obstacle_at']>2:
                    raise AssertionError('Preserving a valid trajectory bypassed the live obstacle guard')
                if viewing and not stalled and state['goal_at'] and state['injected']<2 and current-state['goal_at']>.2*(state['injected']+1):
                    m=NativeTrajectory();m.header=Header(stamp=msg.clock,frame_id='odom');m.start_time=msg.clock;m.trajectory_id=state['injected']+1
                    m.representation=m.POLYNOMIAL;m.degree=5;m.durations=[5.];m.coefficients=[0.]*18;m.coefficients[5]=0. if height_view or stopping_view else (50. if recover_view else 5.);m.coefficients[17]=1.59 if height_view else 1.8
                    native.publish(m);state['injected']+=1
                if stopping_view and state['injected']==2:
                    events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                    requests=[e for e in events if e['event']=='observation_requested']
                    if requests:
                        requested=requests[-1]['sim_time']
                        fast=[e for e in events if e['event']=='known_stopping_constraint_reposition']
                        if fast:
                            assert fast[-1]['sim_time']-requested<5.,'Known obstacle recovery waited for a redundant full scan'
                            assert fast[-1]['occupied_cells']>0 and fast[-1]['unknown_swept_body_cells']==0
                            assert not any(e['event']=='observation_completed' and e['sim_time']>requested for e in events),'Known stopping obstacle was treated as missing observations'
                            print('Measured stopping obstacle selects a guarded viewing move after settlement, without a redundant scan: PASS');return
                        if current-requested>6.:
                            raise AssertionError('Known stopping obstacle triggered a redundant full stationary scan')
                if tail and state['goal_at'] and not state['injected'] and current-state['goal_at']>.2:
                    m=NativeTrajectory();m.header=Header(stamp=msg.clock,frame_id='odom');m.start_time=msg.clock;m.trajectory_id=1
                    m.representation=m.POLYNOMIAL;m.degree=5;m.durations=[8.];m.coefficients=[0.]*18
                    m.coefficients[15]=.02;m.coefficients[17]=1.8
                    native.publish(m);state['injected']=1;state['tail_injected_at']=current;state['tail_brakes_before']=state['brakes']
                if tail and state.get('tail_injected_at') and current-state['tail_injected_at']>6:
                    refs=state.get('tail_refs',[])
                    assert refs and max(p[2] for _,p,_ in refs)>1.85,'Usable native prefix was rejected with its future tail'
                    assert all(1.5<=p[2]<=2.3 for _,p,_ in refs),'An invalid native tail was executed'
                    assert state['brakes']>state['tail_brakes_before'],'Native execution did not brake before its height boundary'
                    events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                    horizon=next(e['valid_until'] for e in events if e['event']=='trajectory_execution_horizon')
                    assert all(valid<=horizon+1e-8 for _,_,valid in refs),'Controller validity outlived the checked prefix'
                    print('Exact native prefix executes; invalid tail expires; full brake stays inside planner height bounds: PASS');return
                if viewing and state['moved'] and state['phase']=='WAIT_OBSERVE':
                    if patrol_view:
                        state.setdefault('view_complete_at',current)
                        time.sleep(.008);continue
                    if indoor_view:
                        events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                        assert any(e['event']=='frontier_viewpoint_retired' for e in events),'Unsupported indoor viewpoint was repeatedly reused'
                    if offset:assert state.get('saw_supported_hold_only'),'Tracking-offset fixture never reached the unknown voxel edge'
                    if not stalled and state.get('speed_limit',.5)>=.5: raise AssertionError('Insufficient observation did not lower the planner speed limit')
                    if math.hypot(*state['position'][:2])<.3: raise AssertionError('Viewing manoeuvre did not reach the observed offset')
                    if len(state['viewing_refs'])<20 or max(b-a for a,b in zip(state['viewing_refs'],state['viewing_refs'][1:]))>.12: raise AssertionError('Viewing reference stream interrupted')
                    if stalled:
                        events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                        assert any(e['event']=='observation_requested' and e['reason']=='native_planner_no_executable_trajectory' for e in events),'Missing-output watchdog was not exercised'
                        assert state['injected']==0,'Planner stall fixture accidentally sent a candidate'
                    if recover_view:
                        if height_view:
                            events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                            assert any(e['event']=='observation_requested' and e['reason']=='trajectory_lacks_observed_braking_corridor' and e.get('unsupported_height_safe') and not e.get('unsupported_stopping_safe') for e in events), 'Height-limited brake was retried as map-free instead of requesting recovery'
                        state.setdefault('view_complete_at',current);time.sleep(.008);continue
                    print(('Planner with no output: ' if stalled else ('Supported hold with tracking offset: ' if offset else ''))+'Independently checked viewing manoeuvre: PASS');return
                if recover_view and state.get('view_complete_at') and state['phase']=='TRANSIT' and not state.get('retry_checked_at'):
                    resumed=[(stamp,goal) for stamp,goal in state.get('goals',[]) if stamp>state['view_complete_at']]
                    if resumed:
                        if current-resumed[0][0]<2:
                            time.sleep(.008);continue
                        assert math.dist(resumed[-1][1],[0.,80.,1.8])<.01,'Viewing move lost the original forest goal'
                        events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                        assert any(e['event']=='observation_replan_from_new_view' and not e['previous_failure_supported'] for e in events),'Fixture did not retry with the old failure still unsupported'
                        # The SAME unsupported point must still fail execution
                        # validation after task-level observation recovery.
                        m=NativeTrajectory();m.header=Header(stamp=msg.clock,frame_id='odom');m.start_time=msg.clock;m.trajectory_id=3
                        m.representation=m.POLYNOMIAL;m.degree=5;m.durations=[5.];m.coefficients=[0.]*18;m.coefficients[5]=0. if height_view else (50. if recover_view else 5.);m.coefficients[17]=1.59 if height_view else 1.8
                        native.publish(m);state['retry_checked_at']=current
                if recover_view and state.get('retry_checked_at'):
                    if any(identifier==3 and not accepted and reason=='trajectory_lacks_observed_braking_corridor' for identifier,accepted,reason in state.get('retry_feedback',[])):
                        print('Guarded viewing move and scan retry the original goal; stale unknown evidence stays unknown and unsafe native candidates remain rejected: PASS');return
                    if current-state['retry_checked_at']>2:raise AssertionError('Retrying a fresh plan bypassed the observed-corridor guard')
                if patrol_view and state.get('view_complete_at') and state['phase']=='PATROL':
                    resumed=[goal for stamp,goal in state['goals'] if stamp>state['view_complete_at']]
                    if resumed:
                        assert math.dist(resumed[-1],patrol_points[0])<.01, 'Viewing waypoint replaced the original inspection target: '+repr(resumed[-1])
                        assert len(state['viewing_refs'])>20 and state['injected']==0, 'Patrol recovery did not exercise a complete independent viewing move'
                        print('Original inspection target survives viewing move, scan and native-planner restart: PASS');return
                if offset and state['phase']=='FAILED_HOVER':
                    assert state.get('saw_supported_hold_only'),'Tracking-offset fixture did not retain a supported reference'
                    raise AssertionError('A supported hold was trapped by the tracking-offset unknown-cell fringe')
                if not viewing and not tail and not replacement and state['goal_at'] and current-state['goal_at']>3:
                    if clock_reset:
                        clock_offset-=5_000_000_000;state['clock_jumped_at']=current-5
                        state['refs_before_reset']=len(state['refs']);state['brakes_before_reset']=state['brakes'];continue
                    if indoor:
                        samples=state['planning_status']
                        assert state['patrol_requested'] and len(samples)>10,'Patrol connection not exercised'
                        assert all(ready and age<.3 for _,ready,age in samples),'Path search starved live sensor/state callbacks'
                        assert max(b[0]-a[0] for a,b in zip(samples,samples[1:]))<.5,'Path search blocked status loop'
                        print('Distant indoor patrol validation and connection preserve sensor/state liveness: PASS');return
                    stamps = [t for t in state['refs'] if t>=state['goal_at']]
                    if state['turn_injected_at'] is None: raise AssertionError('Turn velocity disturbance was not exercised')
                    turning = [t for t in state['refs'] if state['turn_injected_at']<=t<=state['goal_at']]
                    if max((b-a for a,b in zip(turning,turning[1:])), default=1.)>.12:
                        raise AssertionError('Turn reference paused after transient velocity feedback')
                    if len(stamps)<20 or max(b-a for a,b in zip(stamps,stamps[1:]))>.12:
                        raise AssertionError('Reference stream stopped while awaiting the first planner trajectory')
                    if state['first_reference']<state['takeoff_at']+2:
                        raise AssertionError('External references took over before the takeoff reference settled')
                    if (gap or alignment) and (state['brakes']==0 or state['resume_at'] is None or state['first_reference']<state['resume_at']+2):
                        raise AssertionError('Takeoff did not brake and await a fresh resumed reference')
                    expected = -math.pi/2 if rear else math.pi/2
                    if abs(math.remainder(state['yaw']-expected,2*math.pi))>.15:
                        raise AssertionError('Observed return direction did not orient the unregistered sensor toward travel')
                    if alignment:
                        if not state.get('mission_reset_at'):
                            events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
                            assert any(e['event']=='initial_alignment_map_rebuilt' for e in events),'Initial observations were not invalidated'
                            state_status.publish(String(data=json.dumps(dict(reason='ekf_origin_reset',displacement=.0001,rotation_jump=.15))))
                            state['mission_reset_at']=current
                        elif state['phase']=='FAILED_HOVER':
                            print('Initial heading reset rebuilds observations and resumes; in-flight reset aborts: PASS');return
                        elif current-state['mission_reset_at']>1:
                            raise AssertionError('In-flight pose reset was silently accepted')
                    else:
                        print(('Rear-facing anonymous sensor: ' if rear else '')+'Takeoff settlement and continuous references while waiting for EGO: PASS')
                        return
                if process.poll() is not None: raise AssertionError('Task node exited')
                time.sleep(.008)
            raise AssertionError('Timed out in '+state['phase'])
        except BaseException:
            log.flush(); print((Path(folder)/'node.log').read_text(), file=sys.stderr)
            print(json.dumps(dict(domain=os.environ['ROS_DOMAIN_ID'], phase=state['phase'],
                clock_subscribers=clock.get_subscription_count(), odom_subscribers=odom.get_subscription_count(),
                cloud_subscribers=cloud.get_subscription_count(), status_publishers=node.count_publishers('/autonomy/status'))),file=sys.stderr)
            raise
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL); process.wait()
            log.close(); node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
    if '--patrol-viewing' in sys.argv[2:]:
        subprocess.run([sys.executable,str(Path(__file__).with_name('patrol_revalidation_check.py')),sys.argv[1]],check=True)
    if '--viewing-move' in sys.argv[2:]:
        subprocess.run([sys.executable,__file__,sys.argv[1],'--height-viewing'],check=True)
        subprocess.run([sys.executable,__file__,sys.argv[1],'--stopping-viewing'],check=True)
    if '--tail-envelope' in sys.argv[2:]:
        for mode in ('--replacement-rejection','--replacement-speed-reduction'):
            subprocess.run([sys.executable,__file__,sys.argv[1],mode],check=True)

    if '--takeoff-gap' in sys.argv[2:]:
        for mode in ('--takeoff-stall','--partial-takeoff-stall','--takeoff-arm-stall'):
            subprocess.run([sys.executable,__file__,sys.argv[1],mode],check=True)
