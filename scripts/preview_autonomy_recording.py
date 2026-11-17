#!/usr/bin/env python3
"""Replay a closed task bag into an isolated, unarmed perception preview.

Uses the recorded /clock, not a generated clock based on storage timestamps.
Compressed fragments are hard-linked into a temporary sibling directory so
the ROS player cannot expand databases in the original recording directory.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml

from autonomy_provenance import ROOT, runtime_source_hashes
from owned_processes import stop_group
from verify_closed_recording import ensure_closed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--task', choices=['explore', 'forest'], default='explore')
    parser.add_argument('--cloud-topic', default='/autonomy/raw_cloud')
    parser.add_argument('--sensor-origin-frame', default='')
    parser.add_argument('--domain', type=int, default=230)
    parser.add_argument('--seconds', type=float, default=5., help='Minimum recorded simulation time to observe')
    parser.add_argument('--wall-timeout', type=float, default=120.)
    args = parser.parse_args()
    if not 220 <= args.domain <= 232 or args.domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        parser.error('Use a separate replay domain between 220 and 232')
    if args.seconds <= 0 or args.wall_timeout <= 0:
        parser.error('Durations must be positive')
    args.bag = args.bag.resolve()
    ensure_closed(args.bag)
    metadata = (args.bag/'metadata.yaml').read_bytes()
    info = yaml.safe_load(metadata)['rosbag2_bagfile_information']
    required = {args.cloud_topic, '/autonomy/odom', '/clock', '/tf', '/tf_static'}
    counts = {entry['topic_metadata']['name']:entry['message_count'] for entry in info['topics_with_message_count']}
    if any(counts.get(name, 0) <= 0 for name in required):
        parser.error('Replay requires measured cloud, odometry, /clock, /tf and /tf_static')
    paths = info['relative_file_paths']
    if any(Path(name).name != name or not (args.bag/name).is_file() for name in paths):
        parser.error('Invalid recording fragment paths')
    source_stats = {name:((args.bag/name).stat().st_size, (args.bag/name).stat().st_mtime_ns) for name in paths}
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    rclpy.init()
    observer = Node('closed_recording_preview_observer')
    statuses = []
    clocks = []
    commands = []
    observer.create_subscription(String, '/autonomy/status', lambda m: statuses.append(json.loads(m.data)), 10)
    observer.create_subscription(Clock, '/clock', lambda m: clocks.append(m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)
    observer.create_subscription(String, '/ganyu/keyboard', lambda m: commands.append(m.data), 10)
    children = []
    logs = []
    report = dict(passed=False, bag=str(args.bag), ros_domain_id=args.domain,
                  automatic_start=False, controller_launched=False, recorded_clock_used=True)
    executable = ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node'
    binary_hash = hashlib.sha256(executable.read_bytes()).hexdigest()
    sources = runtime_source_hashes()
    def interrupted(number, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    def start(name, command):
        log = (args.output/(name+'.log')).open('w')
        logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        children.append(child)
        return child
    try:
        discovery = time.monotonic()+2
        while time.monotonic() < discovery:
            rclpy.spin_once(observer, timeout_sec=.05)
        peers = [(name, namespace) for name, namespace in observer.get_node_names_and_namespaces()
                 if name != observer.get_name()]
        if peers:
            raise RuntimeError('Replay domain already has nodes; choose another --domain')
        with tempfile.TemporaryDirectory(prefix='autonomy_replay_', dir=args.bag.parent) as temporary:
            copy = Path(temporary)/'bag'
            copy.mkdir()
            (copy/'metadata.yaml').write_bytes(metadata)
            for name in paths:
                os.link(args.bag/name, copy/name)
            # A bag may contain static transforms from several original
            # publishers. Preserve them all when one player republishes them.
            qos = Path(temporary)/'qos.yaml'
            qos.write_text(yaml.safe_dump({'/tf_static':dict(history='keep_last', depth=100,
                reliability='reliable', durability='transient_local')}))
            command = [str(executable), '--ros-args', '-p', 'use_sim_time:=true', '-p', 'auto_start:=false',
                       '-p', 'task:='+args.task, '-p', 'cloud_topic:=/replay/raw_cloud',
                       '-p', 'output_directory:='+str(args.output.resolve()/'preview')]
            if args.sensor_origin_frame:
                command += ['-p', 'sensor_origin_frame:='+args.sensor_origin_frame]
            preview = start('preview', command)
            player = start('player', ['ros2', 'bag', 'play', str(copy), '--disable-keyboard-controls',
                '--delay', '1', '--qos-profile-overrides-path', str(qos),
                '--topics', *sorted(required), '--remap', args.cloud_topic+':=/replay/raw_cloud'])
            deadline = time.monotonic()+args.wall_timeout
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(observer, timeout_sec=.05)
                    ready = [s for s in statuses if s.get('data_ready')]
                    if clocks and clocks[-1]-clocks[0] >= args.seconds and len(ready) >= 5:
                        break
                    if preview.poll() is not None:
                        raise RuntimeError('Perception preview exited')
                    if player.poll() is not None:
                        raise RuntimeError('Playback ended before valid input was observed')
                else:
                    raise TimeoutError('Replay did not provide usable clock/TF/cloud/state within the deadline')
            finally:
                # Close readers before removing their temporary databases.
                for child in reversed(children):
                    stop_group(child, timeout=5)
                children.clear()
            unchanged = source_stats == {name:((args.bag/name).stat().st_size, (args.bag/name).stat().st_mtime_ns) for name in paths}
            unchanged = unchanged and (args.bag/'metadata.yaml').read_bytes() == metadata
            constant = binary_hash == hashlib.sha256(executable.read_bytes()).hexdigest() and sources == runtime_source_hashes()
            if not unchanged or not constant or commands:
                raise RuntimeError('Source changed, implementation changed, or preview emitted a flight command')
            report.update(passed=True, observed_simulation_seconds=clocks[-1]-clocks[0],
                recorded_clock_start=clocks[0], recorded_clock_end=clocks[-1],
                storage_start_nanoseconds=info['starting_time']['nanoseconds_since_epoch'],
                ready_status_samples=len(ready), maximum_known_cells=max(s['known_cells'] for s in ready),
                input_frames=sorted({s['input_frame'] for s in ready}),
                maximum_ready_cloud_age=max(s['cloud_age'] for s in ready),
                maximum_ready_odom_age=max(s['odom_age'] for s in ready), flight_commands_observed=commands,
                source_files_unchanged=unchanged, implementation_constant=constant,
                autonomy_binary_sha256=binary_hash, runtime_sources_sha256=sources,
                metadata_sha256=hashlib.sha256(metadata).hexdigest(),
                scope='Recorded cloud, acquisition-time TF, odometry and clock accepted by current perception preview; not a flight acceptance test')
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        raise
    finally:
        for child in reversed(children):
            stop_group(child, timeout=5)
        for log in logs:
            log.close()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
