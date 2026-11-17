#!/usr/bin/env python3
"""Run the 2 tasks x 3 anonymous lidars x 3 fixed initial conditions matrix."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil
from trial_index import latest_configurations

ROOT = Path(__file__).resolve().parents[1]


def main():
    def interrupted(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT/'log/matrix'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    p.add_argument('--task', choices=['explore', 'forest'])
    p.add_argument('--lidar', choices=['mid360', 'avia', 'os1_64'])
    condition_options = p.add_mutually_exclusive_group()
    condition_options.add_argument('--conditions', type=int, choices=[1, 3], default=3)
    condition_options.add_argument('--condition-index', type=int, choices=[0, 1, 2], help='Run just one fixed initial condition for a reproducible retry.')
    p.add_argument('--resume', action='store_true', help='Retain previous records and retry only configurations without an accepted flight.')
    p.add_argument('--recheck-passed', action='store_true', help='Revalidate selected passed configurations with current software; latest attempts determine acceptance.')
    p.add_argument('--wall-timeout', type=float, default=4000)
    p.add_argument('--scene-only', action='store_true', help='Unarmed sensor/world smoke test; never counts as a full flight.')
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    owner = ['--owner-pid',str(os.getpid()),'--owner-start-time',str(psutil.Process().create_time())]
    if args.resume and any(args.output.glob('*/record.json')):
        # Rebuild from immutable per-trial evidence before selecting retries;
        # a previous interrupted batch may have left an older verified index.
        subprocess.run([sys.executable,str(ROOT/'scripts/rescore_autonomy_matrix.py'),str(args.output)],
            check=True,stdout=subprocess.DEVNULL)
    previous = args.output/'verified_results.json'
    if not previous.exists(): previous = args.output/'results.json'
    results = json.loads(previous.read_text()) if args.resume and previous.exists() else []
    latest = latest_configurations(results)
    if previous.exists() and not args.resume:
        raise RuntimeError('Output contains trials; use --resume or a new output directory.')
    tasks = [args.task] if args.task else ['explore', 'forest']
    lidars = [args.lidar] if args.lidar else ['mid360', 'avia', 'os1_64']
    for task in tasks:
        for lidar in lidars:
            for condition, yaw in enumerate([0., .7853981633974483, -.7853981633974483][:args.conditions]):
                if args.condition_index is not None and condition!=args.condition_index: continue
                key=(task,lidar,condition)
                if not args.recheck_passed and latest.get(key,{}).get('acceptance_pass'): continue
                base=args.output/f'{task}_{lidar}_{condition}'; folder=base; attempt=1
                while folder.exists():
                    attempt+=1; folder=base.with_name(base.name+f'_attempt{attempt}')
                folder.mkdir(exist_ok=False)
                processes = []; files = []; success = False; reason = ''; data = {}; flight_attempted = False
                def start(name, command):
                    file = open(folder/f'{name}.log', 'w'); files.append(file)
                    process = subprocess.Popen(command, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
                    processes.append((name, process)); return process
                try:
                    scene = start('scene', [str(ROOT/'run_autonomy_scene.sh'), '--world', 'warehouse' if task=='explore' else 'forest_long',
                        '--lidar', lidar, '--headless', '--spawn-yaw', str(yaw), '--wall-timeout',str(args.wall_timeout+300),*owner])
                    end = time.monotonic()+150
                    ready = False
                    while time.monotonic()<end and scene.poll() is None:
                        if 'verified; vehicle remains unarmed.' in (folder/'scene.log').read_text(): ready=True; break
                        time.sleep(.25)
                    if not ready: raise RuntimeError('scene_readiness_failed')
                    evaluation_python = Path.home()/'.cache/gazebo_autonomy/eval_venv/bin/python'
                    checker = start('scene_returns', [str(evaluation_python), str(ROOT/'scripts/check_autonomy_scene_returns.py'),
                        '--world-file', str(ROOT/'assets/gazebo_world'/('warehouse.sdf' if task=='explore' else 'forest_long.sdf')),
                        '--lidar', lidar, '--output', str(folder/'scene_returns.json'), '--timeout', '20'])
                    if checker.wait(timeout=40) != 0: raise RuntimeError('physics_surface_echo_check_failed')
                    if args.scene_only:
                        reason='unarmed_scene_check_only'; success=True
                    else:
                        proxy = start('anonymous_cloud', ['bash', '-c', 'source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && exec ros2 run uav_autonomy cloud_proxy --output /anonymous/points --frame unregistered_input "${@:2}"',
                            'matrix_proxy', str(ROOT),*owner])
                        session = start('mission', [str(ROOT/'run_autonomy.sh'), '--task', task, '--start', '--cloud-topic', '/anonymous/points', '--output', str(folder/'mission'),
                            '--wall-timeout',str(args.wall_timeout),'--exit-on-failure',*owner])
                        flight_attempted = True
                        end = time.monotonic()+args.wall_timeout
                        while time.monotonic()<end and scene.poll() is None and session.poll() is None:
                            result = folder/'mission/result.json'
                            if result.exists():
                                data = json.loads(result.read_text()); reason=data.get('reason', ''); break
                            time.sleep(.5)
                        if not data and (folder/'mission/result.json').exists():
                            data=json.loads((folder/'mission/result.json').read_text());reason=data.get('reason','')
                        if not data: reason='wall_timeout_or_process_exit'
                        # Ending an owned SITL instance is a simulator reset. A
                        # failed mission is never converted into a blind landing.
                        if session.poll() is None: os.killpg(session.pid, signal.SIGTERM)
                        try: session.wait(timeout=540)
                        except subprocess.TimeoutExpired: os.killpg(session.pid, signal.SIGKILL); session.wait()
                        evaluation = folder/'mission/evaluation/evaluation.json'
                        if evaluation.exists(): data['evaluation']=json.loads(evaluation.read_text())
                        success=bool(data.get('evaluation', {}).get('acceptance_pass', False))
                except Exception as error: reason=str(error)
                finally:
                    for _, process in reversed(processes):
                        if process.poll() is None: os.killpg(process.pid, signal.SIGTERM)
                    for name, process in processes:
                        try: process.wait(timeout=540 if name=='mission' else 10)
                        except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL); process.wait()
                    for file in files: file.close()
                    result=dict(task=task, simulated_payload=lidar, condition=condition, initial_yaw=yaw,
                        attempt=attempt, trial_directory=str(folder.resolve()),
                        navigation_sensor_metadata_provided=False, flight_attempted=flight_attempted,
                        acceptance_pass=success and not args.scene_only, scene_check_pass=success if args.scene_only else None,
                        reason=reason, data=data)
                    results.append(result);(folder/'record.json').write_text(json.dumps(result, indent=2)+'\n')
                    (args.output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
                    print(json.dumps({key:value for key, value in result.items() if key!='data'}), flush=True)
                    subprocess.run([sys.executable,str(ROOT/'scripts/rescore_autonomy_matrix.py'),str(args.output)],
                        check=True,stdout=subprocess.DEVNULL)
    selected_conditions = [args.condition_index] if args.condition_index is not None else range(args.conditions)
    expected = {(task,lidar,condition) for task in tasks for lidar in lidars for condition in selected_conditions}
    passed = {key for key,r in latest_configurations(results).items()
              if r.get('scene_check_pass' if args.scene_only else 'acceptance_pass')}
    if not expected <= passed:
        raise SystemExit(1)


if __name__ == '__main__': main()
