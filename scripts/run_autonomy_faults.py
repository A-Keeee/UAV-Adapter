#!/usr/bin/env python3
"""Separate owned-SITL fault trials. Never counted as normal matrix flights."""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import psutil
from trial_index import latest_fault_configurations

ROOT=Path(__file__).resolve().parents[1]
KINDS=['cloud_pause','state_pause','manual_exit','blocked']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--kind',choices=KINDS)
    p.add_argument('--lidar',choices=['mid360','avia','os1_64'],default='mid360')
    p.add_argument('--output',type=Path,default=ROOT/'log/fault_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    p.add_argument('--wall-timeout',type=float,default=1900)
    p.add_argument('--resume',action='store_true',help='Skip passed fault kinds and preserve failed attempts when retrying.')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    owner=['--owner-pid',str(os.getpid()),'--owner-start-time',str(psutil.Process().create_time())]
    results=[json.loads(p.read_text()) for p in sorted(args.output.glob('*/fault_record.json'))]
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    for kind in ([args.kind] if args.kind else KINDS):
        if args.resume and latest_fault_configurations(results).get((kind,args.lidar),{}).get('fault_check_pass'):continue
        base=args.output/(kind+'_'+args.lidar);folder=base;attempt=1
        while folder.exists() and args.resume:
            attempt+=1;folder=base.with_name(base.name+f'_attempt{attempt}')
        folder.mkdir(exist_ok=False)
        processes=[];files=[];mission=None;probe=None;reason='';probe_data={};mission_data={};evaluation={}
        def start(name,command):
            f=open(folder/(name+'.log'),'w');files.append(f)
            child=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
            processes.append((name,child));return child
        try:
            scene=start('scene',[str(ROOT/'run_autonomy_scene.sh'),'--world','forest_blocked' if kind=='blocked' else 'forest_long',
                '--lidar',args.lidar,'--headless','--wall-timeout',str(args.wall_timeout+300),*owner])
            end=time.monotonic()+150
            while time.monotonic()<end and scene.poll() is None:
                if 'verified; vehicle remains unarmed.' in (folder/'scene.log').read_text():break
                time.sleep(.25)
            else:raise RuntimeError('Scene did not become ready')
            proxy=start('anonymous_cloud',['ros2','run','uav_autonomy','cloud_proxy','--output','/fault/points','--frame','fault_unregistered_input',*owner])
            mission=start('mission',[str(ROOT/'run_autonomy.sh'),'--task','forest','--start','--cloud-topic','/fault/points',
                '--output',str(folder/'mission'),'--wall-timeout',str(args.wall_timeout),*owner])
            probe=start('probe',[sys.executable,str(ROOT/'scripts/autonomy_fault_probe.py'),'--kind',kind,
                '--scene-pid',str(scene.pid),'--proxy-pid',str(proxy.pid),'--output',str(folder/'probe.json'),
                # A blocked mission can legitimately approach the barrier and
                # inspect a detour before its 120 s no-progress watchdog fires.
                # Share the enclosing trial budget instead of silently using
                # the probe's shorter standalone default (330 s).
                '--wall-timeout',str(args.wall_timeout)])
            end=time.monotonic()+args.wall_timeout
            while time.monotonic()<end:
                if scene.poll() is not None:raise RuntimeError('Scene exited during fault trial')
                if (folder/'probe.json').exists():
                    probe_data=json.loads((folder/'probe.json').read_text())
                    if not probe_data.get('passed'):raise RuntimeError(probe_data.get('reason','Fault probe failed'))
                if (folder/'mission/result.json').exists():
                    mission_data=json.loads((folder/'mission/result.json').read_text())
                    if probe_data or kind in ('cloud_pause','state_pause') and not mission_data.get('success'):break
                if probe.poll() is not None and not probe_data:raise RuntimeError('Probe exited without a result')
                if mission.poll() is not None and not (folder/'mission/result.json').exists():raise RuntimeError('Mission process exited without a result')
                time.sleep(.25)
            else:raise RuntimeError('Fault trial wall timeout')
        except Exception as error:reason=str(error)
        finally:
            signal.signal(signal.SIGTERM,signal.SIG_IGN)
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            # A stopped bridge/proxy must be resumed before its owning session
            # is terminated, including when the probe itself was interrupted.
            for _,process in processes:
                try:
                    owned=[psutil.Process(process.pid),*psutil.Process(process.pid).children(recursive=True)]
                    groups={os.getpgid(c.pid) for c in owned if c.is_running()}
                    for group in groups:
                        if group!=os.getpgrp():os.killpg(group,signal.SIGCONT)
                except (psutil.Error,ProcessLookupError):pass
            if probe is not None and probe.poll() is None:
                os.killpg(probe.pid,signal.SIGTERM)
                try:probe.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(probe.pid,signal.SIGKILL);probe.wait()
            if mission is not None and mission.poll() is None:
                os.killpg(mission.pid,signal.SIGTERM)
                try:mission.wait(timeout=540)
                except subprocess.TimeoutExpired:os.killpg(mission.pid,signal.SIGKILL);mission.wait()
            for _,process in reversed(processes):
                if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
            for _,process in processes:
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            for f in files:f.close()
            score=folder/'mission/evaluation/evaluation.json'
            if score.exists():evaluation=json.loads(score.read_text())
            if (folder/'probe.json').exists():probe_data=json.loads((folder/'probe.json').read_text())
            independent_safety=bool(evaluation.get('contact_frames',0)>0 and evaluation.get('contact_pair_count')==0
                and evaluation.get('minimum_enroute_clearance_m') is not None and evaluation['minimum_enroute_clearance_m']>=.65)
            stopped_truth_speed=None
            if probe_data.get('pause_end_sim') and (folder/'mission/evaluation/truth.csv').exists():
                end=probe_data['pause_end_sim']
                rows=[r for r in csv.DictReader((folder/'mission/evaluation/truth.csv').open()) if end-1<=float(r['simulation_time'])<=end-.1]
                velocities=[]
                for a,b in zip(rows,rows[1:]):
                    dt=float(b['simulation_time'])-float(a['simulation_time'])
                    if 0<dt<.2:velocities.append(float(np.linalg.norm([float(b[k])-float(a[k]) for k in ('x','y','z')])/dt))
                if velocities:stopped_truth_speed=max(velocities)
            expected=bool(evaluation.get('acceptance_pass')) if kind in ('cloud_pause','state_pause') else bool(mission_data and not mission_data.get('success'))
            stop_verified=kind not in ('cloud_pause','state_pause') or stopped_truth_speed is not None and stopped_truth_speed<.10
            record=dict(fault_kind=kind,simulated_payload=args.lidar,normal_matrix_trial=False,
                attempt=attempt,trial_directory=str(folder.resolve()),
                fault_check_pass=bool(not reason and probe_data.get('passed') and independent_safety and expected and stop_verified),
                reason=reason or probe_data.get('reason',''),probe=probe_data,mission=mission_data,evaluation=evaluation,
                maximum_truth_speed_during_final_hold_mps=stopped_truth_speed,
                cleanup='Owned SITL reset after controlled hold; fault cleanup does not claim a mission landing')
            (folder/'fault_record.json').write_text(json.dumps(record,indent=2)+'\n');results.append(record)
            (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps({k:record[k] for k in ('fault_kind','simulated_payload','fault_check_pass','reason')}),flush=True)
            signal.signal(signal.SIGTERM,interrupted)
            signal.signal(signal.SIGINT,interrupted)
    expected={(kind,args.lidar) for kind in ([args.kind] if args.kind else KINDS)}
    (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    passed={key for key,r in latest_fault_configurations(results).items() if r.get('fault_check_pass')}
    if not expected <= passed:raise SystemExit(1)


if __name__=='__main__':main()
