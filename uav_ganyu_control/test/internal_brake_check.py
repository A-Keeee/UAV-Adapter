#!/usr/bin/env python3
"""Private-DDS regression of internal takeoff braking, duplicate stop and resume."""
import os
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
from std_msgs.msg import String, Empty
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus, VehicleLandDetected, VehicleCommand
from uav_common_msg.msg import ExternalReference, TrajectoryFeedback
import json


def main():
    staged = '--staged-takeoff' in sys.argv[2:]
    staged_brake = '--staged-brake' in sys.argv[2:]
    clock_reset = '--clock-reset' in sys.argv[2:]
    rclpy.init();n=Node('ganyu_internal_brake_regression');prefix='/px4_1/fmu/out/'
    clock=n.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    pubs={cls:n.create_publisher(cls,prefix+name+(f'_v{VehicleStatus.MESSAGE_VERSION}' if cls is VehicleStatus and VehicleStatus.MESSAGE_VERSION else ''),qos_profile_sensor_data) for cls,name in (
        (VehicleOdometry,'vehicle_odometry'),(VehicleLocalPosition,'vehicle_local_position'),
        (VehicleStatus,'vehicle_status'),(VehicleLandDetected,'vehicle_land_detected'))}
    alive=n.create_publisher(Empty,'/ganyu/keyboard_alive',10);keys=n.create_publisher(String,'/ganyu/keyboard',10)
    brake=n.create_publisher(Empty,'/autonomy/brake',10)
    state=dict(armed=False,status={},reference=None, feedback=[],references=[]);stage='ready';current=1.;at=0.;first_goal=None
    def command(m):
        if m.command==VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:state['armed']=m.param1>.5
    n.create_subscription(VehicleCommand,'/px4_1/fmu/in/vehicle_command',command,10)
    n.create_subscription(String,'/ganyu/status',lambda m:state.update(status=json.loads(m.data)),10)
    n.create_subscription(TrajectoryFeedback,'/autonomy/controller_feedback',lambda m:state['feedback'].append(m.reason),10)
    def reference(m):
        state['reference']=m
        if staged:state['references'].append(m)
    n.create_subscription(ExternalReference,'/ganyu/control_reference',reference,10)
    with tempfile.TemporaryFile() as log:
        command=[sys.argv[1],'--ros-args','-p','reference_source:=external']
        if staged or staged_brake:command+=['-p','takeoff_height:=3.8','-p','external_takeoff_duration:=18.1']
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        tick=0;clock_offset=0;deadline=time.monotonic()+30
        try:
            while time.monotonic()<deadline:
                tick+=1;ns=1_000_000_000+tick*20_000_000+clock_offset;current=ns/1e9
                c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
                o=VehicleOdometry();o.pose_frame=VehicleOdometry.POSE_FRAME_NED;o.velocity_frame=VehicleOdometry.VELOCITY_FRAME_NED
                o.q=[1.,0.,0.,0.];o.position=[0.,0.,0.];o.velocity=[0.,0.,0.]
                if staged and state['reference'] is not None:
                    r=state['reference'];o.position=[r.position[1],r.position[0],-r.position[2]];o.velocity=[r.velocity[1],r.velocity[0],-r.velocity[2]]
                pubs[VehicleOdometry].publish(o)
                p=VehicleLocalPosition();p.xy_valid=True;p.z_valid=True;p.v_xy_valid=True;p.v_z_valid=True;p.heading_good_for_control=True;pubs[VehicleLocalPosition].publish(p)
                v=VehicleStatus();v.pre_flight_checks_pass=True;v.arming_state=VehicleStatus.ARMING_STATE_ARMED if state['armed'] else VehicleStatus.ARMING_STATE_DISARMED
                v.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD;pubs[VehicleStatus].publish(v)
                l=VehicleLandDetected();l.landed=not state['armed'];pubs[VehicleLandDetected].publish(l);alive.publish(Empty())
                for _ in range(4):rclpy.spin_once(n,timeout_sec=0)
                status=state['status']
                if staged:
                    if stage=='ready' and status.get('ready'):
                        keys.publish(String(data='t'));stage='takeoff'
                    if status.get('phase')=='ACTIVE':
                        goal=status['target_ned'][2]
                        if abs(goal+.8)<1e-5:state['initial_lift_seen']=True
                        if abs(goal+3.8)<1e-5 and not state.get('remaining_climb_seen'):
                            assert state.get('initial_lift_seen'),'Tall takeoff skipped its initial lift'
                            assert abs(o.position[2]+.8)<.12,'Remaining climb started before measured lift settled'
                            state['remaining_climb_seen']=True
                        if state.get('remaining_climb_seen') and status.get('internal_reference_settled'):
                            refs=state['references'];assert refs and abs(refs[-1].position[2]-3.8)<.01
                            for field,limit in [('velocity',.5),('acceleration',.7),('jerk',1.5)]:
                                assert max(sum(x*x for x in getattr(r,field))**.5 for r in refs)<=limit+1e-6,field
                            print('Tall takeoff: measured 0.8 m lift before remaining climb; V/A/J within limits, PASS');return
                    time.sleep(.008);continue
                if stage=='ready' and status.get('ready'):
                    keys.publish(String(data='t'));stage='takeoff'
                elif stage=='takeoff' and status.get('phase')=='ACTIVE':stage='climb';at=current
                elif stage=='climb' and current-at>(1.5 if staged_brake else 3):
                    brake.publish(Empty());stage='braking';at=current
                elif stage=='braking' and current-at>.8:
                    first_goal=status.get('target_ned');assert first_goal and abs(first_goal[2])<1., 'Internal takeoff ignored stop request'
                    brake.publish(Empty());stage='duplicate';at=current
                elif stage=='duplicate' and current-at>3 and status.get('internal_reference_settled'):
                    assert max(abs(a-b) for a,b in zip(first_goal,status['target_ned']))<1e-5,'Duplicate stop changed the checked braking goal'
                    reference=state['reference'];assert reference is not None and max(abs(x) for x in reference.velocity)<1e-5
                    held=ExternalReference();held.header.frame_id='odom';held.header.stamp=c.clock;held.trajectory_id=1
                    future=ns+120_000_000;held.valid_until.sec,held.valid_until.nanosec=divmod(future,10**9)
                    held.position=[float(first_goal[1]),float(first_goal[0]),float(-first_goal[2])];held.yaw=1.5707963267948966
                    if clock_reset:held.velocity=[.1,0.,0.]
                    pub=n.create_publisher(ExternalReference,'/autonomy/reference',10);pub.publish(held)
                    stage='hold';at=current
                elif stage=='hold':
                    future=ns+120_000_000;held.header.stamp=c.clock;held.valid_until.sec,held.valid_until.nanosec=divmod(future,10**9);pub.publish(held)
                    if current-at>.8 and status.get('trajectory_active'):
                        if clock_reset:
                            clock_offset-=5_000_000_000;stage='clock_reset';at=current-5
                            state['reset_goal_before']=status['target_ned'];continue
                        time.sleep(1.)
                        current_ns=ns+20_000_000;resume_clock=Clock();resume_clock.clock.sec,resume_clock.clock.nanosec=divmod(current_ns,10**9);clock.publish(resume_clock)
                        time.sleep(.05)
                        for _ in range(15):rclpy.spin_once(n,timeout_sec=.005)
                        assert 'reference_or_state_timeout' not in state['feedback'],'Simulation pause consumed state liveness timeout'
                        brake.publish(Empty());stage='after_pause';at=current
                elif stage=='clock_reset':
                    if current-at>.5:
                        assert 'simulation_clock_moved_backwards' in state['feedback'],'Backward clock left the old external lease valid'
                        assert not status.get('trajectory_active'),'Old external reference survived clock rollback'
                    if current-at>3 and status.get('internal_reference_settled'):
                        state['reset_hold']=status['target_ned'];stage='clock_latched';at=current
                elif stage=='clock_latched':
                    # Continue past the old lease's numeric timestamp and send
                    # otherwise valid fresh references. Restart is required.
                    future=ns+120_000_000;held.header.stamp=c.clock;held.valid_until.sec,held.valid_until.nanosec=divmod(future,10**9)
                    held.trajectory_id=2;held.velocity=[0.,0.,0.];held.position=list(map(float,[state['reset_hold'][1],state['reset_hold'][0],-state['reset_hold'][2]]))
                    pub.publish(held)
                    if current-at>3:
                        assert not status.get('trajectory_active'),'Clock fault silently accepted a new trajectory'
                        assert status.get('clock_fault_latched'),'Clock fault latch was lost'
                        assert max(abs(a-b) for a,b in zip(status['target_ned'],state['reset_hold']))<1e-5,'Clock fault changed the completed braking hold'
                        print('Clock rollback revokes moving reference, completes brake and blocks implicit restart: PASS');return
                elif stage=='after_pause' and current-at>3 and status.get('internal_reference_settled'):
                    keys.publish(String(data='r'));stage='resume';at=current
                elif stage=='resume' and current-at>2:
                    expected=(.8 if -first_goal[2]<.6 else 3.8) if staged_brake else 1.5
                    assert abs(status['target_ned'][2]+expected)<1e-5,'Resume lost the takeoff target'
                    assert state['reference'].position[2]>-first_goal[2]+.005,'Resumed takeoff reference did not advance'
                    print('Internal takeoff stop, duplicate-stop invariance, stamped reference and resume: PASS');return
                if process.poll() is not None:raise AssertionError('Controller exited')
                time.sleep(.008)
            raise AssertionError('Controller regression timed out in '+stage)
        except BaseException:
            log.seek(0);print(log.read().decode(),file=sys.stderr);raise
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
