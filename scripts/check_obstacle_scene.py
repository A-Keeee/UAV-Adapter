#!/usr/bin/env python3
"""Read-only readiness check for Walls + x500_depth; never arms the vehicle."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import time

from PIL import Image as PillowImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


class SceneCheck(Node):
    def __init__(self):
        super().__init__('obstacle_scene_check')
        self.count = {}
        self.last = {}
        self.last_received = {}
        self.first_clock = None
        status_version = VehicleStatus.MESSAGE_VERSION
        status_suffix = f'_v{status_version}' if status_version else ''
        topics = [
            ('position', VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position'),
            ('odometry', VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry'),
            ('status', VehicleStatus, f'/px4_1/fmu/out/vehicle_status{status_suffix}'),
            ('image', Image, '/camera/image'),
            ('camera_info', CameraInfo, '/camera/camera_info'),
            ('clock', Clock, '/clock'),
        ]
        self.subscriptions_ = [self.create_subscription(
            kind, topic, lambda msg, key=key: self.received(key, msg),
            qos_profile_sensor_data) for key, kind, topic in topics]

    def received(self, key, msg):
        self.count[key] = self.count.get(key, 0) + 1
        self.last[key] = msg
        self.last_received[key] = time.monotonic()
        if key == 'clock' and self.first_clock is None:
            self.first_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def ready(self):
        if not all(self.count.get(key, 0) >= 3 for key in
                   ('position', 'odometry', 'status', 'image', 'camera_info', 'clock')):
            return False
        if any(time.monotonic() - stamp > 3 for stamp in self.last_received.values()):
            return False
        pos, odom = self.last['position'], self.last['odometry']
        img, info = self.last['image'], self.last['camera_info']
        clock = self.last['clock'].clock
        finite = all(math.isfinite(v) for v in
                     [pos.x, pos.y, pos.z, *odom.position, *odom.velocity, *odom.q])
        return (finite and pos.xy_valid and pos.z_valid
                and abs(sum(v*v for v in odom.q) - 1) < 0.05
                and img.width > 0 and img.height > 0 and len(img.data) == img.step * img.height
                and info.width == img.width and info.height == img.height and info.k[0] > 0
                and clock.sec + clock.nanosec * 1e-9 > self.first_clock + 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SceneCheck()
    result = {'passed': False, 'world': 'walls_demo', 'vehicle': 'x500_depth_1'}
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
        result['message_counts'] = node.count
        if not node.ready():
            raise RuntimeError(f'ROS data missing, stale or invalid: {node.count}')
        scene = subprocess.run([
            'gz', 'service', '-s', '/world/walls_demo/scene/info',
            '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
            '--timeout', '4000', '--req', '',
        ], capture_output=True, text=True, timeout=8, check=True).stdout
        expected = ['box1', 'box2', 'box3', 'box4', 'takeoff_pad', 'x500_depth_1']
        missing = [name for name in expected if not re.search(r'name:\s*"' + re.escape(name) + r'"', scene)]
        if missing:
            raise RuntimeError(f'Gazebo scene entities missing: {missing}')
        result['scene_entities_verified'] = expected
        img = node.last['image']
        encodings = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'),
                     'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA'), 'mono8': ('L', 'L')}
        if img.encoding not in encodings:
            raise RuntimeError(f'Unsupported snapshot encoding: {img.encoding}')
        mode, raw_mode = encodings[img.encoding]
        PillowImage.frombytes(mode, (img.width, img.height), bytes(img.data),
                             'raw', raw_mode, img.step, 1).save(args.output / 'camera.png')
        pos, status = node.last['position'], node.last['status']
        result.update(passed=True, local_position_ned_m=[pos.x, pos.y, pos.z],
                      position_valid={'xy': pos.xy_valid, 'z': pos.z_valid},
                      arming_state=status.arming_state, nav_state=status.nav_state,
                      preflight_checks_pass=status.pre_flight_checks_pass,
                      image={'width': img.width, 'height': img.height, 'encoding': img.encoding},
                      simulation_time_s=node.last['clock'].clock.sec,
                      checks='ROS data, simulation clock advancement, and Gazebo scene entities; no flight')
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        result['error'] = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (args.output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
