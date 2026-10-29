#!/usr/bin/python3
"""Independent read-only observer. Start before the waypoint mission.

Optional /verification/gazebo_poses is a Pose_V -> TFMessage bridge of the
single-vehicle world's dynamic_pose/info. Its first pose is the vehicle model's
world pose; subsequent link poses are relative and deliberately not used.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from uav_path_planner.core import World


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world', type=Path, default=Path(__file__).resolve().parents[1]/'assets/gazebo_world/walls_demo.sdf')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--require-truth', action='store_true')
    args = parser.parse_args()
    world = World.from_sdf(args.world, [[-15, -16, .8], [25, 18, 4]])
    rclpy.init()
    node = Node('planner_flight_observer')
    counts = {'rates': 0, 'body_rate_modes': 0, 'other_modes': 0,
              'px4_position_setpoints': 0, 'truth_poses': 0}
    state, clearances, phases = {}, [], set()
    def rates(_):
        counts['rates'] += 1
    def position(_):
        counts['px4_position_setpoints'] += 1
    def mode(msg):
        body_rate = msg.body_rate and not (msg.position or msg.velocity or msg.acceleration or msg.attitude)
        counts['body_rate_modes' if body_rate else 'other_modes'] += 1
    def truth(msg):
        if msg.transforms:
            p = msg.transforms[0].transform.translation
            clearances.append(world.clearance([p.x, p.y, p.z]))
            counts['truth_poses'] += 1
    def status(msg):
        state.update(json.loads(msg.data))
        phases.add(state['state'])
    subscriptions = [
        node.create_subscription(VehicleRatesSetpoint, '/px4_1/fmu/in/vehicle_rates_setpoint', rates, 10),
        node.create_subscription(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', position, 10),
        node.create_subscription(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', mode, 10),
        node.create_subscription(TFMessage, '/verification/gazebo_poses', truth, 10),
        node.create_subscription(String, '/planning/status', status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if state.get('state') in ('COMPLETE', 'FAILED'):
                break
        result = {'passed': bool(state.get('succeeded') and counts['rates'] > 100
                                 and counts['body_rate_modes'] > 100
                                 and not counts['other_modes'] and not counts['px4_position_setpoints']
                                 and (not args.require_truth or (len(clearances) > 100 and min(clearances) >= world.radius))),
                  'observed_takeoff': 'TAKEOFF' in phases, 'counts': counts,
                  'minimum_gazebo_truth_wall_clearance_m': min(clearances) if clearances else None,
                  'mission_state': state.get('state'), 'observed_states': sorted(phases)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
