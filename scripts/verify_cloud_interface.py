#!/usr/bin/env python3
"""Exercise the live generic interface without arming, with anonymous frames."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT/'log/interface_checks')
    p.add_argument('--case', help='Run one named case, retaining other results.')
    p.add_argument('--extended', action='store_true', help='Additional deterministic source topics supplied by cloud_interface_fixture.py.')
    p.add_argument('--os1-sparse-replay',action='store_true',help='Additional actual recorded 16-of-64-channel replay source.')
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = rclpy.create_node('generic_interface_check'); latest = {}
    node.create_subscription(String, '/autonomy/status', lambda m: latest.update(json.loads(m.data)), 10)
    cloud_bounds = {}
    def normalized(message):
        if message.point_step!=12 or not message.width: return
        values=np.frombuffer(message.data,dtype='>f4' if message.is_bigendian else '<f4').reshape(-1,3)
        cloud_bounds.update(minimum=values.min(axis=0).tolist(),maximum=values.max(axis=0).tolist())
    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(PointCloud2, '/autonomy/cloud', normalized, qos_profile_sensor_data)
    cases = [(name, ['--layout', name], True) for name in ['xyz', 'reordered', 'float64', 'padded', 'big_endian']]
    cases += [('sparse', ['--every', '8'], True), ('low_frequency', ['--frequency', '2'], True),
              ('cropped_forward', ['--forward-cosine', '.8'], True),
              ('world_with_origin', ['--world-frame', 'odom'], True),
              ('world_without_origin', ['--world-frame', 'odom'], False),
              ('empty', ['--empty-every', '1'], False), ('missing_tf', ['--missing-tf'], False),
              ('skewed_clock', ['--time-offset', '3'], False)]
    if args.extended:
        cases += [('mixed_xyz_types',['--input','/fixture/mixed'],True),
                  ('nonfinite',['--input','/fixture/nonfinite'],True),
                  ('spatial_overload',['--input','/fixture/dense'],True),
                  ('resource_limit',['--input','/fixture/oversized'],False)]
    if args.os1_sparse_replay:cases += [('os1_sparse_replay',['--input','/fixture/os1_sparse'],True)]
    if args.case: cases = [case for case in cases if case[0]==args.case]
    results = []
    try:
        for index, (name, options, valid) in enumerate(cases):
            processes = []; files = []; latest.clear(); cloud_bounds.clear()
            folder = args.output/name; folder.mkdir(exist_ok=True)
            try:
                commands = [
                    ['ros2', 'run', 'uav_autonomy', 'cloud_proxy', '--output', '/unknown_interface/points', '--frame', f'anonymous_frame_{index}', *options],
                    ['ros2', 'run', 'uav_autonomy', 'autonomy_node', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'auto_start:=false',
                     '-p', 'task:=forest', '-p', 'cloud_topic:=/unknown_interface/points', '-p', 'output_directory:='+str(folder)],
                ]
                if name=='resource_limit':
                    # Test the production subscriber with the full valid-layout
                    # 4,000,001-point frame. A second 48 MB best-effort hop via
                    # the format proxy can drop it before the node sees it;
                    # missing delivery is not evidence of resource rejection.
                    # Other cases continue to exercise the anonymous proxy.
                    commands = [commands[1]]
                    commands[0][commands[0].index('cloud_topic:=/unknown_interface/points')] = 'cloud_topic:=/fixture/oversized'
                if name=='world_with_origin': commands[1] += ['-p',f'sensor_origin_frame:=anonymous_frame_{index}']
                expected_frame='fixture_lidar' if name=='resource_limit' else 'odom' if name.startswith('world_') else f'anonymous_frame_{index}'
                for i, command in enumerate(commands):
                    file = open(folder/f'process_{i}.log', 'w'); files.append(file)
                    processes.append(subprocess.Popen(command, stdout=file, stderr=subprocess.STDOUT, start_new_session=True))
                deadline = time.monotonic()+(15 if name in ('spatial_overload','resource_limit') else 8 if valid else 5)
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                    if any(process.poll() is not None for process in processes): break
                    # A late status from the previous consumer is not evidence
                    # about this case, even when it reports data_ready=true.
                    if latest.get('input_frame')!=expected_frame:continue
                    if valid and latest.get('data_ready') and not latest.get('cloud_error') and (name!='low_frequency' or latest.get('cloud_period', 0)>.4):
                        if name!='spatial_overload' or cloud_bounds: break
                    if name=='resource_limit' and latest.get('cloud_error')=='Malformed or resource-limit-exceeding PointCloud2': break
                passed = latest.get('phase')=='WAIT_DATA' and ((latest.get('data_ready') and not latest.get('cloud_error')) if valid else latest.get('points', 0)==0)
                expected_error={'empty':'empty_measurement','missing_tf':'waiting_for_sample_time_tf',
                    'skewed_clock':'cloud_clock_mismatch','world_without_origin':'World cloud requires sensor_origin_frame',
                    'resource_limit':'Malformed or resource-limit-exceeding PointCloud2'}
                if not valid: passed = passed and latest.get('cloud_error')==expected_error[name]
                passed = passed and latest.get('input_frame')==expected_frame
                if name=='low_frequency': passed = passed and latest.get('cloud_period', 0)>.4
                if name=='nonfinite': passed = passed and 0<latest.get('valid_ratio',1)<1
                if name=='spatial_overload':
                    # Input is ordered from the sphere's north pole to south.
                    # Truncating a prefix loses its southern hemisphere.
                    passed = passed and latest.get('input_points')==80000 and 0<latest.get('points',0)<=15000 and latest.get('spatial_voxel',0)>.15
                    passed = passed and cloud_bounds.get('minimum',[0,0,0])[2]<-4 and cloud_bounds.get('maximum',[0,0,0])[2]>7
                if name=='os1_sparse_replay':
                    passed = passed and latest.get('input_points',0)>0 and latest.get('clearance_evidence',{}).get('unknown_swept_body',0)>0 and latest.get('current_safe') is False
                observed=latest.get('input_frame')==expected_frame
                results.append(dict(case=name, expected_measurements=valid, passed=bool(passed),
                    input_observed=observed,
                    failure_kind=None if passed else 'assertion_failed' if observed else 'input_not_delivered',
                    diagnostics=dict(latest),normalized_bounds=dict(cloud_bounds)))
                print(json.dumps(results[-1]), flush=True)
            finally:
                for process in processes:
                    if process.poll() is None: os.killpg(process.pid, signal.SIGTERM)
                for process in processes:
                    try: process.wait(timeout=4)
                    except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL); process.wait()
                for file in files: file.close()
        if args.case and (args.output/'results.json').exists():
            previous = json.loads((args.output/'results.json').read_text())
            (args.output/f'{args.case}_retry.json').write_text(json.dumps(results, indent=2)+'\n')
            by_case = {item['case']:item for item in previous}
            by_case.update({item['case']:item for item in results}); results = list(by_case.values())
        (args.output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    finally:
        node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if all(result['passed'] for result in results) else 1)


if __name__ == '__main__': main()
