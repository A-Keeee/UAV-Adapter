#!/usr/bin/env python3
"""Start one EGO backend, generic perception/task node and GanYu controller."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import shutil
import subprocess
import sys
import time
import yaml
from autonomy_provenance import fingerprint, active_scene
from ego_environment import without_ego_overlays
from owned_processes import owner_alive, signal_group, stop_group

ROOT = Path(__file__).resolve().parents[1]


def execution_height_bounds(task, planner, backend):
    if backend == 'v2':
        derived = [planner['grid_map/virtual_ground']+.1,
                   planner['grid_map/virtual_ceil']-.1]
    else:
        derived = [planner['grid_map/virtual_floor_height']+.1,
                   planner['grid_map/virtual_ceil_height']-planner['grid_map/resolution']-.1]
    # Explicit task safety limits do not follow optimizer wall tuning.
    return [task.get('execution_min_z', derived[0]), task.get('execution_max_z', derived[1])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['explore', 'forest'], required=True)
    parser.add_argument('--start', action='store_true', help='Authorize automatic takeoff, mission and landing.')
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument('--record', dest='record', action='store_true', help='Record ROS topics for replay (default).')
    recording.add_argument('--no-record', dest='record', action='store_false', help='Skip ROS bag recording, compression and replay verification; retain text logs and task results.')
    parser.set_defaults(record=True)
    parser.add_argument('--cloud-topic', default='/lidar/points')
    parser.add_argument('--sensor-origin-frame', default='')
    parser.add_argument('--mass', type=float, help='Total simulated vehicle mass, kg; no lidar capability parameters.')
    parser.add_argument('--goal', nargs=3, type=float)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--wall-timeout', type=float, default=0)
    parser.add_argument('--owner-pid', type=int, default=0)
    parser.add_argument('--owner-start-time', type=float, default=0)
    parser.add_argument('--exit-on-failure', action='store_true', help='Finite batch trials stop after recording a task failure.')
    args = parser.parse_args()
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')
    lock = open(runtime/f'gazebo_sim_ganyu_{os.getuid()}.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError(
            '已有自主任务或 GanYu 控制会话正在运行，无法重复启动。'
            '请先在原任务终端按 Ctrl+C，等待其退出后再重试。'
            f'（控制器锁：{lock.name}）'
        ) from error
    # Only the physical vehicle mass is imported. Brand, scanning pattern, FOV,
    # range, line count and blind zone never enter any navigation process.
    scene_info = ROOT/'log/autonomy_scene/latest/vehicle.json'
    scene_record = active_scene(scene_info)
    mass_source = 'explicit' if args.mass is not None else 'active_simulation_vehicle'
    if args.mass is None:
        if scene_record is None:
            raise RuntimeError('Provide --mass for the vehicle, or start run_autonomy_scene.sh first.')
        args.mass = scene_record['mass']
    if args.mass <= 0 or args.mass*9.81 > 30:
        raise ValueError('Mass exceeds the configured GanYu thrust authority.')
    backend = 'swarm' if args.task == 'explore' else 'v2'
    overlay = Path.home()/f'.cache/gazebo_ego_{backend}/install'
    if not (overlay/'setup.bash').exists():
        raise RuntimeError('Build EGO backends with scripts/build_ego_backends.sh first.')
    folder = args.output or ROOT/'log/autonomy'/f'{datetime.datetime.now():%Y%m%d_%H%M%S}_{args.task}_{os.getpid()}'
    folder = folder.resolve(); folder.mkdir(parents=True, exist_ok=True)
    if any((folder/name).exists() for name in ('run.json', 'result.json', 'bag')):
        raise RuntimeError('Output already contains a trial; choose a new directory to preserve its evidence.')
    processes = []; files = []
    def start(name, command, ego=False):
        log = open(folder/(name+'.log'), 'w'); files.append(log)
        if ego:
            # A fresh child shell loads exactly one overlay, avoiding duplicate
            # traj_utils/quadrotor_msgs type libraries from different backends.
            # local_setup avoids replaying underlay chains captured by an old
            # build in a user's previously sourced terminal.
            setup = ['source /opt/ros/humble/setup.bash',
                     'source '+shlex.quote(str(ROOT/'install/local_setup.bash')),
                     'source '+shlex.quote(str(overlay/'local_setup.bash'))]
            command = ['bash', '-c', ' && '.join(setup)+' && exec '+shlex.join(command)]
        p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             start_new_session=True, env=without_ego_overlays(os.environ))
        processes.append((name, p))
    def interrupted(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted); signal.signal(signal.SIGTERM, interrupted)
    config = folder/'configuration'
    config.mkdir()
    for source in (ROOT/'uav_autonomy/config').glob('*.yaml'):
        shutil.copy2(source, config/source.name)
    task_config = config/('warehouse.yaml' if args.task=='explore' else 'forest.yaml')
    task_parameters = yaml.safe_load(task_config.read_text())['autonomy']['ros__parameters']
    planner_parameters = yaml.safe_load((config/f'ego_{backend}.yaml').read_text())['ego_planner_node']['ros__parameters']
    execution_bounds = execution_height_bounds(task_parameters, planner_parameters, backend)
    task_parameters.update(execution_min_z=execution_bounds[0],execution_max_z=execution_bounds[1])
    task_config.write_text(yaml.safe_dump({'autonomy':{'ros__parameters':task_parameters}},sort_keys=False))
    takeoff_height = float(task_parameters['takeoff_height'])
    takeoff_duration = max(12., 4.5*takeoff_height+1.)
    try:
        start('ganyu', ['ros2', 'run', 'uav_ganyu_control', 'ganyu_controller', '--ros-args', '-p', 'reference_source:=external',
            '-p', f'mass:={args.mass}', '-p', f'takeoff_height:={takeoff_height}',
            '-p', f'external_takeoff_duration:={takeoff_duration}'])
        if args.start and args.record:
            # The installed Humble zstd plugin fails to finish frames for an
            # input exactly divisible by its 128 KiB read buffer, then reuses
            # that unfinished context for another fragment. Keep SQLite until
            # recording has closed; our separate compressor verifies every
            # decompressed byte before removing an original fragment.
            start('recording', ['ros2', 'bag', 'record', '--storage-preset-profile', 'resilient', '-o', str(folder/'bag'),
                '--max-bag-size', '536870912',
                '/autonomy/raw_cloud', '/autonomy/cloud', '/autonomy/occupied',
                '/autonomy/odom', '/autonomy/native_trajectory', '/autonomy/reference',
                '/autonomy/trajectory_feedback', '/autonomy/controller_feedback',
                '/autonomy/goal', '/autonomy/speed_limit', '/autonomy/planner_odom',
                '/autonomy/native_accepted', '/autonomy/execution_allowed',
                '/autonomy/status', '/autonomy/planning_metrics', '/ganyu/status', '/ganyu/control_debug', '/ganyu/control_reference', '/clock', '/tf', '/tf_static'])
        if args.start:
            evaluation_python = Path.home()/'.cache/gazebo_autonomy/eval_venv/bin/python'
            if evaluation_python.exists() and scene_record is not None:
                scene = scene_record
                start('evaluation', [str(evaluation_python), str(ROOT/'scripts/autonomy_evaluator.py'),
                    '--world', scene['world'], '--world-file', scene.get('world_file',str(ROOT/'assets/gazebo_world'/f'{scene["world"]}.sdf')),
                    '--vehicle', scene['model']+'_1', '--task', args.task, '--output', str(folder/'evaluation'),
                    '--contacts-manifest', str(Path(scene['scene_directory'])/'contacts.json'), '--mission-result', str(folder/'result.json')])
        planner = ['ros2', 'run', 'ego_planner', 'ego_planner_node', '--ros-args', '--params-file', str(config/f'ego_{backend}.yaml'),
                   '-r', 'odom_world:=/autonomy/planner_odom', '-r', 'grid_map/odom:=/autonomy/planner_odom',
                   '-r', 'grid_map/cloud:='+('/autonomy/occupied' if backend=='swarm' else '/autonomy/cloud'), '-r', 'planning/trajectory:=/ego/native_trajectory',
                   '-r', 'planning/bspline:=/ego/native_trajectory', '-r', '/goal:=/ego/goal', '-r', '/move_base_simple/goal:=/ego/goal']
        start('ego', planner, ego=True)
        start('trajectory_bridge', ['ros2', 'run', 'ego_ganyu_bridge', 'ego_ganyu_bridge', '--ros-args', '-p', 'use_sim_time:=true'], ego=True)
        command = ['ros2', 'run', 'uav_autonomy', 'autonomy_node', '--ros-args', '--params-file', str(task_config),
                   '-p', f'auto_start:={str(args.start).lower()}', '-p', 'cloud_topic:='+args.cloud_topic,
                   '-p', 'output_directory:='+str(folder)]
        if args.sensor_origin_frame:
            command += ['-p', 'sensor_origin_frame:='+args.sensor_origin_frame]
        if args.goal:
            command += ['-p', 'goal:='+str(args.goal)]
        start('autonomy', command)
        (folder/'run.json').write_text(json.dumps(dict(task=args.task, backend=backend, automatic_start=args.start, recording_enabled=args.start and args.record, mass=args.mass, mass_source=mass_source,
            cloud_topic=args.cloud_topic, sensor_origin_frame=args.sensor_origin_frame,
            configuration_directory=str(config), takeoff_height=takeoff_height, takeoff_duration=takeoff_duration,
            execution_z_bounds=execution_bounds,
            scene=scene_record, software=fingerprint(backend)), indent=2)+'\n')
        print(f'{"EXECUTION" if args.start else "PREVIEW: no arming"}, task={args.task}; logs: {folder}', flush=True)
        print('ROS bag recording: '+('ON' if args.start and args.record else 'OFF (text logs and task results retained)'), flush=True)
        deadline = time.monotonic()+args.wall_timeout if args.wall_timeout else float('inf')
        while time.monotonic() < deadline:
            time.sleep(.25)
            if not owner_alive(args.owner_pid,args.owner_start_time):
                raise RuntimeError('Owning batch exited; close this simulation trial.')
            dead = [name for name, process in processes if process.poll() is not None]
            if dead:
                result = folder/'result.json'
                if not result.exists():
                    result.write_text(json.dumps(dict(success=False, task=args.task, phase='FAILED_PROCESS', reason='nodes_exited:'+','.join(dead)), indent=2)+'\n')
                raise RuntimeError('Nodes exited: '+', '.join(dead))
            result = folder/'result.json'
            if result.exists():
                data = json.loads(result.read_text())
                if data.get('success'):
                    print('Mission landed. Evaluation must still verify coverage, collisions and tracking.', flush=True)
                    return
                if data.get('phase')=='COMPLETE_WITH_INCOMPLETE_MAP':
                    raise RuntimeError('Mission landed, but exploration exceeded its budget; task acceptance failed.')
                if args.exit_on_failure:
                    raise RuntimeError('Task failed: '+data.get('reason','unknown'))
        raise RuntimeError('Session wall timeout; this is not task success.')
    finally:
        # The measured 25.8 GB OS1 warehouse bag already needs about three
        # minutes for byte-verified compression and replay checks. Leave a
        # bounded eight-minute window for the longer 3600 s task configuration.
        cleanup_deadline = time.monotonic()+480
        # A parent may stop us immediately after noticing a failed result.
        # Cleanup must finish even when that signal races with our own exit.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # Revoke the reference producers before asking the still-live
        # controller to brake; they cannot resume a goal during cleanup.
        for name, process in processes:
            if name in ('autonomy','ego','trajectory_bridge'):
                signal_group(process, signal.SIGTERM)
        # Ask for a brake while the controller is alive. Never initiate a blind
        # landing over an unknown surface as an error-cleanup side effect.
        if args.start:
            subprocess.run(['timeout', '4', 'ros2', 'topic', 'pub', '--once', '/autonomy/brake', 'std_msgs/msg/Empty', '{}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2)
        for _, process in reversed(processes):
            signal_group(process, signal.SIGTERM)
        for name, process in processes:
            stop_group(process, timeout=180 if name=='evaluation' else 30 if name=='recording' else 4,
                       already_signalled=True)
        if (folder/'bag/metadata.yaml').is_file():
            finalization = dict(passed=False)
            try:
                for script, extra in [('compress_closed_recording.py', []),
                                      ('verify_closed_recording.py', ['--reuse-verified'])]:
                    remaining = cleanup_deadline-time.monotonic()
                    if remaining <= 1:
                        raise TimeoutError('Recording finalization deadline exceeded; originals retained')
                    finalizer = subprocess.Popen([sys.executable, str(ROOT/'scripts'/script), str(folder/'bag'), *extra],
                                                 start_new_session=True)
                    try:
                        if finalizer.wait(timeout=remaining):
                            raise RuntimeError('Recording tool failed: '+script)
                    finally:
                        stop_group(finalizer,timeout=3)
                finalization = dict(passed=True, compression='FILE zstd', byte_identity_verified=True,
                                    replay_verification=str(folder/'bag/replay_verification.json'))
            except Exception as error:
                # Preserve flight scoring and the original fragments. A failed
                # artifact gate cannot be called a replayable delivery.
                finalization['error'] = str(error)
                print('Recording finalization failed: '+str(error), file=sys.stderr, flush=True)
            (folder/'recording_finalization.json').write_text(json.dumps(finalization,indent=2)+'\n')
        for file in files: file.close()
        print('Logs retained: '+str(folder), flush=True)


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: sys.exit(130)
    except Exception as exc: print(str(exc), file=sys.stderr); sys.exit(1)
