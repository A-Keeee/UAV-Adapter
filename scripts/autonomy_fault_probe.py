#!/usr/bin/env python3
"""Owned-SITL fault injection and response observation; no truth control input."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleCommand, VehicleRatesSetpoint
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from uav_common_msg.msg import TrajectoryFeedback

MOVING_SPEED = .12  # Native low-speed replans target 0.144 m/s.


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--kind',required=True,choices=['cloud_pause','state_pause','manual_exit','blocked'])
    p.add_argument('--scene-pid',required=True,type=int)
    p.add_argument('--proxy-pid',required=True,type=int)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--wall-timeout',type=float,default=330)
    args=p.parse_args(); rclpy.init(); n=Node('autonomy_fault_probe')
    state=dict(sim=0.,task={},controller={},rates=[],brakes=[])
    n.create_subscription(Clock,'/clock',lambda m:state.update(sim=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(String,'/autonomy/status',lambda m:state.update(task=json.loads(m.data)),10)
    n.create_subscription(String,'/ganyu/status',lambda m:state.update(controller=json.loads(m.data)),10)
    n.create_subscription(VehicleRatesSetpoint,'/px4_1/fmu/in/vehicle_rates_setpoint',lambda m:state['rates'].append(state['sim']),qos_profile_sensor_data)
    n.create_subscription(TrajectoryFeedback,'/autonomy/controller_feedback',lambda m:state['brakes'].append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9) if m.braking else None,10)
    commands=n.create_publisher(VehicleCommand,'/px4_1/fmu/in/vehicle_command',10)
    paused=None; triggered=None; resumed=None; yielded=None; last_command=-100.; stable=None
    brake_seen=False; stopped_seen=False; timeout_seen=False
    result=dict(kind=args.kind,passed=False,navigation_received_truth=False,
        movement_detection_threshold_mps=MOVING_SPEED)
    end=time.monotonic()+args.wall_timeout
    try:
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            t=state['sim']; task=state['task']; controller=state['controller']
            if not t or not task or not controller:continue
            speed=math.sqrt(sum(float(v)**2 for v in controller.get('velocity_ned',[0,0,0])))
            if args.kind=='blocked':
                expected=task.get('phase')=='FAILED_HOVER' and task.get('reason') in (
                    'no_progress_observation_or_execution_blocked','insufficient_observation_after_scanning',
                    'no_observed_safe_observation_viewpoint','mission_timeout')
                if expected and controller.get('phase')=='ACTIVE' and controller.get('armed') and speed<.10:
                    if stable is None:stable=t
                    if t-stable>=2:
                        result.update(passed=True,sim_time=t,reason=task['reason'],controlled_hover=True);break
                else:stable=None
                if task.get('phase')=='COMPLETE':raise RuntimeError('Blocked trial unexpectedly completed')
                continue
            if args.kind=='manual_exit':
                if triggered is None and task.get('phase')=='OBSERVE' and controller.get('phase')=='ACTIVE' and speed<.08:
                    triggered=t;result['requested_at_sim']=t
                if triggered is not None and controller.get('phase')!='YIELDED' and t-last_command>1:
                    # Human requests PX4 Hold (AUTO/LOITER), available without RC.
                    m=VehicleCommand();m.timestamp=n.get_clock().now().nanoseconds//1000
                    m.command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE;m.param1=1.;m.param2=4.;m.param3=3.
                    m.target_system=2;m.target_component=1;m.source_system=246;m.source_component=1;m.from_external=True
                    commands.publish(m);last_command=t
                if controller.get('phase')=='YIELDED' and yielded is None:yielded=t;result['yielded_at_sim']=t
                if yielded is not None and t-yielded>=3:
                    late=[stamp for stamp in state['rates'] if stamp>yielded+.15]
                    if late:raise RuntimeError('GanYu published rates after yielding')
                    if controller.get('phase')!='YIELDED' or controller.get('nav_state')==14:
                        raise RuntimeError('Offboard was reacquired after human exit')
                    if task.get('reason')!='operator_left_offboard':raise RuntimeError('Task did not report human takeover')
                    result.update(passed=True,rate_messages_after_yield=len(late),reason=task['reason'],sim_time=t);break
                if triggered is not None and yielded is None and t-triggered>12:raise RuntimeError('PX4 Hold did not yield control')
                continue
            if triggered is None and task.get('phase')=='TRANSIT' and speed>MOVING_SPEED:
                if args.kind=='cloud_pause':
                    process=psutil.Process(args.proxy_pid)
                    if 'cloud_proxy' not in ' '.join(process.cmdline()):raise RuntimeError('Proxy ownership check failed')
                    paused=os.getpgid(process.pid)
                else:
                    owned=psutil.Process(args.scene_pid).children(recursive=True)
                    groups={os.getpgid(c.pid) for c in owned if c.is_running() and 'px4_state_bridge' in ' '.join(c.cmdline())}
                    if len(groups)!=1:raise RuntimeError('State bridge ownership is ambiguous')
                    paused=groups.pop()
                if paused==os.getpgrp():raise RuntimeError('Refusing to pause the probe group')
                os.killpg(paused,signal.SIGSTOP);triggered=t;result['pause_start_sim']=t
            if triggered is not None:
                brake_seen=brake_seen or any(stamp>=triggered for stamp in state['brakes'])
                timeout_seen=timeout_seen or not task.get('data_ready',True)
                if not task.get('data_ready',True) and speed<.10:stopped_seen=True
                if resumed is None and t-triggered>=5:
                    os.killpg(paused,signal.SIGCONT);paused=None;resumed=t;result['pause_end_sim']=t
                if resumed is not None and task.get('data_ready') and task.get('phase')=='TRANSIT' and speed>MOVING_SPEED:
                    if not (brake_seen and stopped_seen and timeout_seen):raise RuntimeError('Outage did not produce a verified stop')
                    result.update(passed=True,recovered_at_sim=t,brake_seen=brake_seen,stopped_seen=stopped_seen,timeout_seen=timeout_seen);break
                if task.get('phase')=='FAILED_HOVER':raise RuntimeError('Outage recovery failed: '+task.get('reason',''))
        else:raise RuntimeError('Fault probe wall timeout')
    except Exception as error:result['reason']=str(error)
    finally:
        if paused is not None:
            try:os.killpg(paused,signal.SIGCONT)
            except ProcessLookupError:pass
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
