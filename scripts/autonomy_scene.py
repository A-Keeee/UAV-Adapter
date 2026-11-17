#!/usr/bin/env python3
"""Own exactly the processes of one unarmed autonomy simulation session."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
import psutil
from owned_processes import owner_alive, signal_group, stop_group

ROOT = Path(__file__).resolve().parents[1]
PAYLOADS = {'mid360': (2.390307692307692, (0, 0, .12), 'LivoxMid360.mat3x4f'),
            'avia': (2.623307692307692, (.12, 0, .12), 'LivoxAvia.mat3x4f'),
            'os1_64': (2.550307692307692, (0, 0, .15), 'OusterOS1_64.mat3x4f')}


def spawn_pose(world, yaw):
    # The vendor warehouse floor top is at z=0.034225 m. The merged X500
    # model has its lowest skid at z=0.013 m for a zero model pose: spawning
    # there embeds it in the floor. Drop the unarmed vehicle from above the
    # surface and let physics settle it before normal readiness/arming.
    # This scene setup pose is never supplied as a navigation obstacle map.
    return (0., 0., .10 if world == 'warehouse' else 0., 0., 0., yaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=['warehouse', 'forest_original', 'forest_long', 'forest_blocked'], default='warehouse')
    parser.add_argument('--lidar', choices=list(PAYLOADS), default='mid360')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--check', action='store_true', help='Verify measurements and PX4, then stop without arming.')
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--spawn-yaw', type=float, default=0, help='Fixed test initial yaw in radians.')
    parser.add_argument('--wall-timeout', type=float, default=0)
    parser.add_argument('--owner-pid', type=int, default=0)
    parser.add_argument('--owner-start-time', type=float, default=0)
    args = parser.parse_args()
    end = time.monotonic()+args.wall_timeout if args.wall_timeout else float('inf')
    if not owner_alive(args.owner_pid,args.owner_start_time):
        raise RuntimeError('Owning batch is no longer running')
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')
    lock = open(runtime/f'gazebo_sim_obstacle_{os.getuid()}.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if subprocess.run(['pgrep', '-u', str(os.getuid()), '-x', 'px4'], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('PX4 is already running; stop its owning session first.')
    existing = subprocess.run(['timeout', '5', 'gz', 'topic', '-l'], capture_output=True, text=True)
    if '/world/' in existing.stdout:
        raise RuntimeError('A Gazebo world is already running.')
    px4 = Path(os.environ['PX4_DIR'])
    vehicle = 'x500_'+args.lidar
    mass, pose, pattern = PAYLOADS[args.lidar]
    initial_pose = spawn_pose(args.world, args.spawn_yaw)
    if not (Path(os.environ['RGL_PATTERNS_DIR'])/pattern).exists():
        raise RuntimeError('Run scripts/setup_autonomy_lidars.sh first.')
    world = ROOT/'assets/gazebo_world'/f'{args.world}.sdf'
    checked = subprocess.run(['gz', 'sdf', '-k', str(world)], capture_output=True, text=True)
    if checked.returncode:
        raise RuntimeError(checked.stderr)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder = ROOT/'log/autonomy_scene'/f'{stamp}_{os.getpid()}'
    folder.mkdir(parents=True); (folder/'px4').mkdir()
    snapshot = folder/'world.sdf'
    snapshot.write_bytes(world.read_bytes())
    world = snapshot
    latest = folder.parent/'latest'
    if latest.is_symlink(): latest.unlink()
    latest.symlink_to(folder)
    ray_surfaces = ROOT/'assets/vendor/RGL_COLLISION_SURFACES.json'
    (folder/'vehicle.json').write_text(json.dumps(dict(mass=mass, sensor_frame=args.lidar+'_link', sensor_translation=pose, world=args.world, model=vehicle, spawn_pose=list(initial_pose), owner_pid=os.getpid(), owner_start_time=psutil.Process().create_time(), world_file=str(world.resolve()), world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(), collision_ray_surfaces_sha256=hashlib.sha256(ray_surfaces.read_bytes()).hexdigest() if ray_surfaces.exists() else None), indent=2)+'\n')
    # Expand the unchanged airframe and add contact sensors to its existing
    # collision shapes. Sensors add no mass, geometry or controller authority.
    expanded = subprocess.run(['gz', 'sdf', '-p', str(ROOT/'assets/gazebo_models'/vehicle/'model.sdf')],
                              capture_output=True, text=True, check=True)
    model_tree = ET.fromstring(expanded.stdout)
    contact_topics = []
    for link in model_tree.find('model').findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            topic = f'/evaluation/contact/{link.attrib["name"]}/{index}'
            sensor = ET.SubElement(link, 'sensor', name=f'evaluation_contact_{index}', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '50'
            ET.SubElement(sensor, 'topic').text = topic
            ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = collision.attrib['name']
            contact_topics.append(f'/world/{args.world}/model/{vehicle}_1/link/{link.attrib["name"]}/sensor/evaluation_contact_{index}/contact')
    generated = folder/'models'/vehicle; generated.mkdir(parents=True)
    ET.ElementTree(model_tree).write(generated/'model.sdf', encoding='utf-8', xml_declaration=True)
    (generated/'model.config').write_text(f'<model><name>{vehicle}</name><version>1</version><sdf version="1.11">model.sdf</sdf><description>Instrumented simulation vehicle</description></model>')
    (folder/'contacts.json').write_text(json.dumps(contact_topics, indent=2)+'\n')
    (folder/'source.sha256').write_text(hashlib.sha256(world.read_bytes()).hexdigest()+'  '+str(world)+'\n')
    bridges = [dict(ros_topic_name='/clock', gz_topic_name=f'/world/{args.world}/clock', ros_type_name='rosgraph_msgs/msg/Clock', gz_type_name='gz.msgs.Clock', direction='GZ_TO_ROS'),
               dict(ros_topic_name='/lidar/points', gz_topic_name='/'+args.lidar+'/points', ros_type_name='sensor_msgs/msg/PointCloud2', gz_type_name='gz.msgs.PointCloudPacked', direction='GZ_TO_ROS')]
    (folder/'bridge.yaml').write_text(yaml.safe_dump(bridges))
    processes = []
    files = []
    def start(name, command, env=None):
        log = open(folder/(name+'.log'), 'w'); files.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                   start_new_session=True, env=env)
        processes.append((name, process)); return process
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        start('agent', ['MicroXRCEAgent', 'udp4', '-p', '8888'])
        gz = ['gz', 'sim', '-r', '-s', '-v', '3', str(world)]
        if args.headless: gz.append('--headless-rendering')
        start('gazebo', gz)
        if not args.headless: start('gui', ['gz', 'sim', '-g'])
        env = os.environ.copy()
        env.update(PX4_GZ_STANDALONE='1', PX4_GZ_WORLD=args.world, PX4_SYS_AUTOSTART='4002',
                   PX4_SIM_MODEL='gz_'+vehicle, PX4_GZ_MODEL_POSE=','.join(map(str, initial_pose)), PX4_PARAM_EKF2_HGT_REF='1',
                   PX4_PARAM_EKF2_MAG_TYPE='1', PX4_GZ_MODELS=str(folder/'models'))
        start('px4', [str(px4/'build/px4_sitl_default/bin/px4'), '-d', '-i', '1', '-w', str(folder/'px4'), str(px4/'build/px4_sitl_default/etc')], env)
        start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args', '-p', 'config_file:='+str(folder/'bridge.yaml')])
        start('state', ['ros2', 'run', 'uav_autonomy', 'px4_state_bridge', '--ros-args', '-p', 'use_sim_time:=true'])
        start('sensor_tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]), '--frame-id', 'base_link', '--child-frame-id', args.lidar+'_link'])
        print(f'Unarmed {args.world}, {args.lidar}; logs: {folder}', flush=True)
        # Readiness requires real clouds and timestamped converted odometry.
        import rclpy
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init(); node = rclpy.create_node('autonomy_scene_check')
        counts = {'cloud': 0, 'odom': 0}
        def cloud(m):
            if m.width*m.height and {f.name for f in m.fields} >= {'x', 'y', 'z'}: counts['cloud'] += 1
        node.create_subscription(PointCloud2, '/lidar/points', cloud, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/autonomy/odom', lambda m: counts.update(odom=counts['odom']+1), 10)
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline and (counts['cloud'] < 3 or counts['odom'] < 10):
            if not owner_alive(args.owner_pid,args.owner_start_time):
                raise RuntimeError('Owning batch exited during scene startup')
            rclpy.spin_once(node, timeout_sec=.1)
            dead = [name for name, process in processes if process.poll() is not None]
            if dead: raise RuntimeError('Processes exited: '+', '.join(dead))
        (folder/'readiness.json').write_text(json.dumps(counts, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
        if counts['cloud'] < 3 or counts['odom'] < 10:
            raise RuntimeError('Readiness timeout: '+str(counts))
        print('PointCloud2 and sample-time PX4 odometry verified; vehicle remains unarmed.', flush=True)
        if not args.check:
            while True:
                time.sleep(.5)
                if time.monotonic()>end or not owner_alive(args.owner_pid,args.owner_start_time):
                    raise RuntimeError('Scene lifetime or owning batch ended')
                dead = [name for name, process in processes if process.poll() is not None]
                if dead: raise RuntimeError('Processes exited: '+', '.join(dead))
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for _, process in reversed(processes):
            signal_group(process, signal.SIGTERM)
        for _, process in processes:
            stop_group(process, timeout=4, already_signalled=True)
        for file in files: file.close()
        print('Session stopped; logs retained: '+str(folder), flush=True)


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: sys.exit(130)
    except Exception as exc:
        print(str(exc), file=sys.stderr); sys.exit(1)
