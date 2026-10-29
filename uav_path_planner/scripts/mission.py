#!/usr/bin/python3
"""Waypoint mission: plan before arming, send entire splines, record tracking."""
import csv
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, Float64MultiArray, String
from uav_common_msg.msg import BSplineTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from uav_path_planner.core import World, plan


def enu(ned):
    return np.asarray([ned[1], ned[0], -ned[2]], dtype=float)


def ned(point):
    return enu(point)  # This change of basis is its own inverse.


class Mission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        def param(name, default):
            return self.declare_parameter(name, default).value
        default_world = str(Path(get_package_share_directory('uav_path_planner'))/'worlds/walls_demo.sdf')
        self.world = World.from_sdf(param('world_file', default_world),
                                    [param('map_min', [-15.0, -16.0, 0.8]), param('map_max', [25.0, 18.0, 4.0])],
                                    param('safety_radius', 0.65), param('resolution', 0.4))
        flat = np.asarray(param('waypoints', [0.0, 8.0, 2.5, 8.0, 8.0, 2.5, 8.0, 0.0, 2.5]))
        if not flat.size or flat.size % 3 or not np.isfinite(flat).all():
            raise ValueError('waypoints must be finite ENU triples')
        self.waypoints = flat.reshape(-1, 3)
        self.height = float(param('takeoff_height', 2.5))
        self.auto_start = param('auto_start', False)
        self.land_at_end = param('land_at_end', True)
        self.result_file = param('result_file', '')
        self.limits = [param('max_velocity', 1.0), param('max_acceleration', 0.7), param('max_jerk', 1.5)]
        if not math.isfinite(self.height) or not 0.8 <= self.height <= 3:
            raise ValueError('takeoff_height must be between 0.8 and 3 m')
        self.state, self.error = 'PLANNING', ''
        self.status, self.status_at, self.sim_time, self.clock_at = None, 0, 0.0, 0.0
        self.home = np.zeros(3)
        self.started_at = time.monotonic()
        self.last_action, self.last_visual, self.last_report = 0.0, 0.0, 0.0
        self.stable_since = None
        self.leg, self.trajectory_id, self.sent_at, self.spline_end = 0, 0, 0.0, 0.0
        self.done, self.succeeded, self.interrupted = False, False, False
        self.errors, self.clearances = [], []
        self.passed_waypoints = []
        self.actual, self.references = [], []
        self.plans = []
        self.trace = self.trace_file = None
        self.gcs = None
        self.last_gcs = 0
        self.last_trace = -1.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.key_pub = self.create_publisher(String, '/ganyu/keyboard', 10)
        self.alive_pub = self.create_publisher(Empty, '/ganyu/keyboard_alive', 10)
        self.trajectory_pub = self.create_publisher(BSplineTrajectory, '/planning/trajectory', 10)
        self.status_pub = self.create_publisher(String, '/planning/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/planning/markers', latched)
        self.astar_pub = self.create_publisher(PathMsg, '/planning/astar_path', latched)
        self.spline_pub = self.create_publisher(PathMsg, '/planning/bspline_path', latched)
        self.actual_pub = self.create_publisher(PathMsg, '/planning/actual_path', 1)
        self.reference_pub = self.create_publisher(PathMsg, '/planning/tracking_reference', 1)
        self.create_subscription(String, '/ganyu/status', self.on_status, 10)
        self.create_subscription(Float64MultiArray, '/ganyu/control_debug', self.on_debug, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        if param('provide_gcs_heartbeat', True):
            from pymavlink import mavutil
            try:
                self.gcs = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255, source_component=190)
                self.mavutil = mavutil
            except OSError as exc:
                self.get_logger().warning(f'UDP 14550 already occupied; using external GCS: {exc}')
        if self.result_file:
            output = Path(self.result_file)
            output.parent.mkdir(parents=True, exist_ok=True)
            self.trace_file = output.with_suffix('.csv').open('w', newline='')
            self.trace = csv.writer(self.trace_file)
            self.trace.writerow(['sim_time', 'leg', 'east', 'north', 'up', 'ref_east', 'ref_north', 'ref_up',
                                 'tracking_error_m', 'wall_clearance_m'])
        start = np.array([0.0, 0.0, self.height])
        try:
            # Check takeoff/landing columns as well as the cruise paths.
            if not self.world.free(start):
                raise ValueError('Takeoff point is blocked')
            for goal in self.waypoints:
                raw, spline, info = plan(self.world, start, goal, *self.limits)
                self.plans.append((raw, spline, info))
                self.get_logger().info(f'Leg {len(self.plans)}: {info}')
                start = goal
            for point in (np.array([0.0, 0.0, self.height]), self.waypoints[-1]):
                for box in self.world.boxes:
                    if (np.all(point[:2] >= box.low[:2]-self.world.radius)
                            and np.all(point[:2] <= box.high[:2]+self.world.radius)
                            and box.low[2] <= point[2]+self.world.radius and box.high[2] >= 0):
                        raise ValueError('Takeoff/landing column intersects obstacle')
            self.set_state('WAIT_READY' if self.auto_start else 'PREVIEW')
            self.publish_plan()
        except ValueError as exc:
            self.error = str(exc)
            self.set_state('FAILED')
            self.done = True
        self.timer = self.create_timer(0.05, self.tick)

    def stamp(self):
        nanoseconds = int(self.sim_time*1e9)
        from builtin_interfaces.msg import Time
        return Time(sec=nanoseconds//1_000_000_000, nanosec=nanoseconds % 1_000_000_000)

    def set_state(self, state):
        self.state = state
        self.last_action = 0
        self.stable_since = None
        self.get_logger().info(f'Mission -> {state} (leg {min(self.leg+1, len(self.waypoints))}/{len(self.waypoints)})')

    def on_clock(self, msg):
        value = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.sim_time and value < self.sim_time-0.01:
            self.fail('Simulation clock moved backwards')
        self.sim_time, self.clock_at = value, time.monotonic()

    def on_status(self, msg):
        self.status, self.status_at = json.loads(msg.data), time.monotonic()
        if self.status['phase'] == 'ACTIVE':
            self.home = np.asarray(self.status['home_ned'])
        elif self.status['phase'] == 'LANDING':
            self.actual.append(enu(np.asarray(self.status['position_ned'])-self.home))

    def on_debug(self, msg):
        if len(msg.data) != 26 or self.sim_time == self.last_trace:
            return
        self.last_trace = self.sim_time
        reference = enu(np.array(msg.data[5:8])-self.home)
        actual = enu(np.array(msg.data[20:23])-self.home)
        error = float(np.linalg.norm(actual-reference))
        clearance = self.world.clearance(actual)
        if not np.isfinite([*actual, *reference, error, clearance]).all():
            self.fail('Non-finite tracking feedback')
            return
        self.errors.append(error)
        self.clearances.append(clearance)
        self.actual.append(actual)
        self.references.append(reference)
        if self.trace:
            self.trace.writerow([self.sim_time, self.leg, *actual, *reference, error, clearance])
        if self.state in ('TRACKING', 'SETTLING') and (error > 0.6 or clearance < self.world.radius):
            self.fail(f'Tracking envelope exceeded: error={error:.3f}, clearance={clearance:.3f}')

    def key(self, key):
        self.key_pub.publish(String(data=key))
        self.last_action = time.monotonic()

    def path(self, points):
        msg = PathMsg()
        msg.header.frame_id, msg.header.stamp = 'odom', self.stamp()
        offset = enu(self.home)
        for point in points:
            pose = PoseStamped()
            pose.header = msg.header
            p = point+offset
            pose.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_plan(self):
        raw = [p for path, _, _ in self.plans for p in path]
        smooth = [s.sample(t) for _, s, _ in self.plans for t in np.linspace(0, s.duration, 160)]
        self.astar_pub.publish(self.path(raw))
        self.spline_pub.publish(self.path(smooth))
        markers = MarkerArray()
        def marker(namespace, identity, kind, position, scale, color):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'odom', self.stamp()
            m.ns, m.id, m.type, m.action = namespace, identity, kind, Marker.ADD
            p = np.asarray(position)+enu(self.home)
            m.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = map(float, scale)
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.markers.append(m)
            return m
        for i, box in enumerate(self.world.boxes):
            center, size = (box.low+box.high)/2, box.high-box.low
            marker('walls', i, Marker.CUBE, center, size, (0.65, 0.68, 0.72, 0.22))
            marker('inflation', i, Marker.CUBE, center, size+2*self.world.radius, (1.0, 0.2, 0.15, 0.08))
        for i, point in enumerate(self.waypoints):
            marker('waypoints', i, Marker.SPHERE, point, [.3]*3, (1.0, 0.2, 0.8, 1.0))
            m = marker('labels', i, Marker.TEXT_VIEW_FACING, point+[0, 0, .6], [0, 0, .4], (1.0, .9, .9, 1.0))
            m.text = f'WP{i+1}'
        self.marker_pub.publish(markers)

    def settled(self, target):
        s = self.status
        actual = enu(np.array(s['position_ned'])-self.home)
        stable = np.linalg.norm(actual-target) < 0.15 and np.linalg.norm(s['velocity_ned']) < 0.15
        if not stable:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = self.sim_time
        return self.stable_since is not None and self.sim_time-self.stable_since > 1.0

    def send_spline(self):
        _, spline, _ = self.plans[self.leg]
        msg = BSplineTrajectory()
        msg.header.frame_id, msg.header.stamp = 'px4_ned', self.stamp()
        self.trajectory_id = max(time.time_ns(), self.trajectory_id+1)
        msg.trajectory_id, msg.degree = self.trajectory_id, spline.degree
        start = self.sim_time+0.5
        msg.start_time.sec = int(start)
        msg.start_time.nanosec = int((start-int(start))*1e9)
        msg.knots = spline.knots.tolist()
        msg.control_points = np.array([ned(p)+self.home for p in spline.points]).ravel().tolist()
        self.trajectory_pub.publish(msg)
        self.spline_end, self.sent_at = start+spline.duration, time.monotonic()
        self.set_state('TRACKING')

    def fail(self, reason):
        if self.state in ('FAILED', 'ABORTING') or self.done:
            return
        self.error = reason
        self.get_logger().error(reason)
        if self.status and (self.status['armed'] or self.status['phase'] in ('PRESTREAM', 'WAIT_ARM', 'WAIT_OFFBOARD')):
            self.set_state('ABORTING')
            self.abort_at = time.monotonic()
            self.key('l')
        else:
            self.set_state('FAILED')
            self.done = True

    def report(self):
        return {'state': self.state, 'succeeded': self.succeeded, 'error': self.error,
                'map_source': 'static SDF boxes (not lidar mapping or dynamic obstacle avoidance)',
                'waypoints_enu': self.waypoints.tolist(), 'safety_radius_m': self.world.radius,
                'plans': [info for _, _, info in self.plans], 'completed_waypoints': self.passed_waypoints,
                'tracking_samples': len(self.errors),
                'tracking_rmse_m': float(np.sqrt(np.mean(np.square(self.errors)))) if self.errors else None,
                'tracking_max_error_m': max(self.errors) if self.errors else None,
                'minimum_measured_wall_clearance_m': min(self.clearances) if self.clearances else None,
                'landed_disarmed': bool(self.status and self.status['landed'] and not self.status['armed'])}

    def write_report(self):
        result = self.report()
        self.status_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if self.result_file:
            output = Path(self.result_file)
            temp = output.with_suffix('.tmp')
            temp.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
            temp.replace(output)
            self.trace_file.flush()

    def tick(self):
        now = time.monotonic()
        self.alive_pub.publish(Empty())
        if self.gcs:
            for _ in range(40):
                if self.gcs.recv_match(blocking=False) is None:
                    break
            if now-self.last_gcs > 1:
                m = self.mavutil.mavlink
                self.gcs.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, m.MAV_STATE_ACTIVE)
                self.last_gcs = now
        if now-self.last_visual > 1:
            self.publish_plan()
            self.actual_pub.publish(self.path(self.actual[-12000:]))
            self.reference_pub.publish(self.path(self.references[-12000:]))
            self.last_visual = now
        if now-self.last_report > 1 or self.done:
            self.write_report()
            self.last_report = now
        if self.interrupted:
            self.interrupted = False
            if self.state == 'PREVIEW':
                self.done = True
                return
            if self.state == 'HOLDING':
                self.key('l')
                self.set_state('LANDING')
                self.landing_at = now
            else:
                self.fail('Operator requested stop')
        if self.done or self.state == 'PREVIEW':
            return
        s = self.status
        if self.state == 'ABORTING':
            if s and now-self.status_at < 1 and s['phase'] == 'IDLE' and not s['armed']:
                self.set_state('FAILED')
                self.done = True
            elif now-self.abort_at > 45:
                self.error += '; landing acknowledgment timed out'
                self.done = True
            elif now-self.last_action > 1:
                self.key('l')
            return
        if self.state == 'WAIT_READY':
            if now-self.started_at > 90:
                self.fail('Controller/PX4 not ready within 90 s')
            elif s and now-self.status_at < 1 and s['reference_source'] != 'planner':
                self.fail('Controller must run with reference_source:=planner')
            elif s and s['ready'] and s['preflight'] and not s['armed'] and s['phase'] == 'IDLE':
                if abs(float(s['target_ned'][2])-float(s['home_ned'][2])) > .1:
                    self.fail('Controller has an unexpected existing reference')
                    return
                self.key('t')
                self.set_state('TAKEOFF')
                self.takeoff_at = now
            return
        if not s or now-self.status_at > 2 or now-self.clock_at > 2:
            self.fail('Controller feedback or simulation clock timed out')
            return
        if s['phase'] == 'YIELDED':
            self.error = 'PX4 left Offboard; mission yielded control'
            self.set_state('FAILED')
            self.done = True
            return
        if self.state == 'TAKEOFF':
            if now-self.takeoff_at > 60:
                self.fail('Takeoff did not settle within 60 s')
            elif s['phase'] == 'ACTIVE' and self.settled(np.array([0, 0, self.height])):
                self.send_spline()
        elif self.state == 'TRACKING':
            if s['phase'] != 'ACTIVE':
                self.fail('Controller left ACTIVE during tracking')
            elif s['trajectory_id'] != self.trajectory_id:
                if now-self.sent_at > 2:
                    self.fail('Controller did not accept spline within 2 s')
            elif self.sim_time >= self.spline_end:
                self.set_state('SETTLING')
                self.settle_at = now
            elif not s['trajectory_active'] and self.sim_time < self.spline_end-.25:
                self.fail('Trajectory was canceled before completion')
        elif self.state == 'SETTLING':
            if self.settled(self.waypoints[self.leg]):
                self.passed_waypoints.append(self.leg+1)
                self.leg += 1
                if self.leg < len(self.plans):
                    self.send_spline()
                elif self.land_at_end:
                    self.key('l')
                    self.set_state('LANDING')
                    self.landing_at = now
                else:
                    self.set_state('HOLDING')
            elif now-self.settle_at > 20:
                self.fail('Waypoint did not settle within 20 s')
        elif self.state == 'LANDING':
            if s['phase'] == 'IDLE' and not s['armed'] and s['landed']:
                self.succeeded = True
                self.set_state('COMPLETE')
                self.done = True
            elif now-self.landing_at > 45:
                self.fail('Landing did not complete within 45 s')
            elif now-self.last_action > 1:
                self.key('l')

    def close(self):
        self.write_report()
        if self.trace_file:
            self.trace_file.close()
        if self.gcs:
            self.gcs.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Mission()
        signal.signal(signal.SIGINT, lambda *_: setattr(node, 'interrupted', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'interrupted', True))
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if node.succeeded or (not node.auto_start and not node.error) else 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
