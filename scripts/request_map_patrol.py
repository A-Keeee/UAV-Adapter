#!/usr/bin/env python3
"""Use the running exploration session's map and coordinate origin for patrol."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--points', type=Path, help='JSON array of at least four ENU XYZ positions; otherwise choose from the current map.')
args = p.parse_args()
command = dict(action='patrol')
if args.points:
    command['points'] = json.loads(args.points.read_text())
    if len(command['points']) < 4: p.error('At least four points required')
rclpy.init(); node = rclpy.create_node('request_map_patrol')
publisher = node.create_publisher(String, '/autonomy/mission_command', 10)
deadline = time.monotonic()+5
while not publisher.get_subscription_count() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
if not publisher.get_subscription_count():
    raise SystemExit('No exploration session is running; start run_autonomy.sh --task explore first.')
publisher.publish(String(data=json.dumps(command)))
rclpy.spin_once(node, timeout_sec=.5)
node.destroy_node(); rclpy.shutdown()
print('Patrol requested. The running task validates observation, clearance and reachability.')
