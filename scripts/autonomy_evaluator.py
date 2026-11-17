#!/usr/bin/env python3
"""Independent Gazebo collision-mesh clearance, truth trajectory and task scoring."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from uav_common_msg.msg import ExternalReference
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from autonomy_geometry import world_collision_mesh
from waypoint_context import patrol_truth_errors
from evaluation_finalization import atomic_json, protected_finalization
from evaluation_phases import acceptance_pass, enroute_metrics, enroute_samples, ground_contact_authorized
import trimesh
import psutil


class Evaluator(Node):
    def __init__(self, args):
        super().__init__('autonomy_independent_evaluator')
        self.args = args; args.output.mkdir(parents=True, exist_ok=True)
        self.mesh, self.planes = world_collision_mesh(args.world_file)
        self.proximity = trimesh.proximity.ProximityQuery(self.mesh)
        self.samples = []; self.errors = []; self.latest = None; self.lock = threading.Lock()
        self.phase = ''; self.task_phase = ''; self.armed = False; self.last_sample = -1; self.known = set()
        self.patrol_index = 0
        self.patrol_generation = 0; self.patrol_marker = None
        self.reason = ''; self.task_success = False; self.max_capacity = 0; self.capacities = set()
        self.contact_frames = 0; self.contact_pairs = set(); self.ground_contacts = 0
        self.offset = None; self.offset_samples = []; self.clock = 0.; self.references = []
        self.odom = None; self.contact_start_time = None; self.stamped_reference_samples = 0
        self.finishing = False; self.planning = []; self.mapping = []; self.native_capacities = set()
        self.saved_map_verified = False; self.coverage_source = None
        self.resource_processes = {}; self.resource_samples = []
        self.session_process = psutil.Process(args.session_pid or os.getppid())
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', self.on_truth)
        if args.contacts_manifest:
            for topic in json.loads(args.contacts_manifest.read_text()):
                self.gz.subscribe(Contacts, topic, self.on_contacts)
        self.create_subscription(String, '/ganyu/status', self.on_controller, 10)
        self.create_subscription(String, '/autonomy/status', self.on_status, 10)
        self.create_subscription(Float64MultiArray, '/ganyu/control_debug', self.on_debug, 10)
        self.create_subscription(ExternalReference, '/ganyu/control_reference', self.on_reference, 10)
        self.create_subscription(Float64MultiArray, '/autonomy/planning_metrics', self.on_planning, 10)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/autonomy/odom', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/autonomy/observed_free', self.on_free, qos_profile_sensor_data)
        self.create_timer(.05, self.sample)
        self.create_timer(.5, self.resources)
        self.file = open(args.output/'truth.csv', 'w'); self.csv = csv.writer(self.file)
        self.csv.writerow(['simulation_time', 'x', 'y', 'z', 'clearance', 'controller_phase', 'task_phase', 'patrol_index', 'patrol_generation'])
        self.create_timer(1., self.checkpoint)

    def checkpoint(self, finalized=False):
        # Contact events are independent Gazebo measurements, not inferred from
        # sampled clearance. Persist them before potentially expensive scoring.
        with self.lock:
            data = dict(finalized=finalized, simulation_time=self.clock,
                truth_samples=len(self.samples), contact_frames=self.contact_frames,
                colliding_pairs=sorted(self.contact_pairs),
                authorized_ground_contact_frames=self.ground_contacts,
                truth_to_odom_translation=self.offset.tolist() if self.offset is not None else None,
                stamped_reference_samples=self.stamped_reference_samples)
        atomic_json(self.args.output/'evaluation_checkpoint.json', data)

    def on_contacts(self, message):
        with self.lock:
            if self.finishing: return
            self.contact_frames += 1
            for item in message.contact:
                names = sorted([item.collision1.name, item.collision2.name])
                if all(self.args.vehicle in name for name in names): continue
                ground = any('GroundB' in name or 'ground' in name.lower() or 'grasspatch' in name for name in names)
                if ground_contact_authorized(ground, self.armed, self.phase, self.task_phase):
                    self.ground_contacts += 1; continue
                if any(depth>0 for depth in item.depth): self.contact_pairs.add(tuple(names))

    def on_truth(self, message):
        model = next((p for p in message.pose if p.name == self.args.vehicle), None)
        body = next((p for p in message.pose if p.name == 'base_link'), None)
        if model is None: return
        q = model.orientation
        rotation = trimesh.transformations.quaternion_matrix([q.w, q.x, q.y, q.z])[:3, :3]
        p = np.array([model.position.x, model.position.y, model.position.z])
        if body is not None:
            p += rotation@np.array([body.position.x, body.position.y, body.position.z])
        t = message.header.stamp.sec+message.header.stamp.nsec*1e-9
        with self.lock:
            if not self.finishing: self.latest = (t, p)

    def on_odom(self, message):
        p = message.pose.pose.position
        self.odom = np.array([p.x, p.y, p.z])
        with self.lock: latest = self.latest
        if latest is not None and not self.armed:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if abs(stamp-latest[0]) < .1 and self.offset is None:
                self.offset_samples.append(latest[1]-self.odom)
                if len(self.offset_samples) >= 10:
                    self.offset = np.median(self.offset_samples, axis=0)

    def on_controller(self, m):
        data = json.loads(m.data); self.phase = data['phase']; self.armed = data['armed']

    def on_status(self, m):
        data = json.loads(m.data); self.task_phase = data['phase']; self.patrol_index = int(data.get('patrol_index',0)); self.reason = data['reason']
        if self.args.task=='explore' and self.args.mission_result:
            marker = self.args.mission_result.parent/'patrol_points.json'
            if marker.exists() and marker.stat().st_mtime_ns!=self.patrol_marker:
                self.patrol_marker = marker.stat().st_mtime_ns
                self.patrol_generation += 1
        self.capacities.add(data['map_capacity']); self.max_capacity = max(self.max_capacity, data['map_capacity'])
        if data.get('mapping_ms', 0)>0: self.mapping.append(float(data['mapping_ms']))

    def on_planning(self, m):
        if len(m.data)>=2 and np.isfinite(m.data).all(): self.planning.append(tuple(m.data[:2]))
        if len(m.data)>=4: self.native_capacities.add(tuple(int(v) for v in m.data[2:4]))

    def resources(self):
        try: processes = [self.session_process, *self.session_process.children(recursive=True)]
        except psutil.Error: return
        cpu = 0.; rss = 0
        for process in processes:
            if process.pid == os.getpid(): continue
            try:
                if process.pid not in self.resource_processes:
                    self.resource_processes[process.pid] = process
                    process.cpu_percent(None)
                else: cpu += self.resource_processes[process.pid].cpu_percent(None)
                rss += process.memory_info().rss
            except psutil.Error: continue
        self.resource_samples.append((cpu, rss/2**20))

    def on_debug(self, m):
        if len(m.data) >= 26 and self.armed:
            # Debug layout: thrust/input/rates then p,v,a,j,integral,state p,v.
            self.errors.append(float(np.linalg.norm(np.array(m.data[5:8])-np.array(m.data[20:23]))))
    def on_reference(self, m):
        if self.armed and m.header.frame_id=='odom':
            self.references.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,np.asarray(m.position)))
            self.stamped_reference_samples += 1

    def on_free(self, m):
        # Only the evaluation process retains global observed cells. Forest
        # navigation itself has a bounded local cache and never reads this set.
        if self.args.task != 'explore': return
        if m.point_step != 12 or m.is_bigendian: return
        points = np.frombuffer(m.data, dtype='<f4').reshape(-1, 3)
        self.known.update(map(tuple, np.floor(points/.2).astype(int)))

    def sample(self):
        with self.lock: latest = self.latest
        if latest is None or latest[0] <= self.last_sample: return
        t, p = latest; self.last_sample = t
        _, distances, _ = self.proximity.on_surface(p.reshape(1, 3))
        clearance = float(distances[0])
        for z in self.planes: clearance = min(clearance, abs(p[2]-z))
        clearance = float(clearance)
        self.samples.append((t, p, clearance, self.phase, self.task_phase, self.patrol_index, self.patrol_generation))
        self.csv.writerow([t, *p, clearance, self.phase, self.task_phase, self.patrol_index, self.patrol_generation]); self.file.flush()

    def finish(self):
        with self.lock: self.finishing = True
        self.file.close()
        self.checkpoint(finalized=True)
        airborne = enroute_samples(self.samples)
        flight_metrics = enroute_metrics(airborne)
        # A sampled center clearance above the airframe's bounding radius proves
        # separation at samples; it is not an uninstrumented contact-event count.
        truth_errors = []
        if self.samples and self.offset is not None:
            times = np.array([s[0] for s in self.samples]); positions = np.array([s[1] for s in self.samples])
            for stamp, reference in self.references:
                if times[0] <= stamp <= times[-1]:
                    actual = np.array([np.interp(stamp, times, positions[:, axis]) for axis in range(3)])
                    truth_errors.append(float(np.linalg.norm(reference+self.offset-actual)))
        coverage = None
        if self.args.task == 'explore' and self.offset is not None and self.known:
            coverage = self.coverage()
        mission = {}
        if self.args.mission_result and self.args.mission_result.exists():
            mission = json.loads(self.args.mission_result.read_text())
        truth_rmse = float(np.sqrt(np.mean(np.square(truth_errors)))) if truth_errors else None
        waypoint_errors = []
        if self.offset is not None and airborne:
            targets = mission.get('patrol_points_enu', []) if self.args.task=='explore' else [mission.get('goal_enu')]
            if self.args.task=='explore':
                waypoint_errors = patrol_truth_errors([s[1] for s in airborne], [s[5] for s in airborne],
                    [s[6] for s in airborne], targets, self.offset)
            else:
                positions = np.array([s[1] for s in airborne if s[4]=='TRANSIT'])
                for target in targets:
                    waypoint_errors.append(float(np.linalg.norm(positions-(np.asarray(target)+self.offset), axis=1).min()) if target is not None and len(positions) else None)
        expected_targets = 4 if self.args.task=='explore' else 1
        waypoints_pass = len(waypoint_errors)>=expected_targets and all(error is not None and error<=.25 for error in waypoint_errors)
        def distribution(values):
            return dict(samples=len(values), median=float(np.median(values)), p95=float(np.percentile(values,95)),
                        maximum=float(max(values))) if values else None
        result = dict(truth_samples=len(self.samples), mission_reason=mission.get('reason', self.reason),
                      waypoint_minimum_truth_errors_m=waypoint_errors, waypoints_pass=waypoints_pass,
                      waypoint_context='latest_patrol_generation_and_target_index_including_turn_and_replan',
                      planning_wall_ms=distribution([p[0] for p in self.planning]),
                      planning_successes=sum(p[1]>.5 for p in self.planning),
                      mapping_wall_ms=distribution(self.mapping),
                      navigation_cpu_percent=distribution([s[0] for s in self.resource_samples]),
                      navigation_rss_mib=distribution([s[1] for s in self.resource_samples]),
                      resource_scope='navigation session descendants, excluding evaluator; CPU 100% is one core; excludes Gazebo/PX4',
                      tracking_rmse_m=truth_rmse, control_reference_timestamp='controller_simulation_sample_time',
                      stamped_reference_samples=self.stamped_reference_samples,
                      feedback_tracking_rmse_m=float(np.sqrt(np.mean(np.square(self.errors)))) if self.errors else None,
                      truth_to_odom_translation=self.offset.tolist() if self.offset is not None else None,
                      native_map_capacities=[list(v) for v in sorted(self.native_capacities)], native_capacity_constant=len(self.native_capacities)==1,
                      observation_cache_capacities=sorted(self.capacities), cache_capacity_constant=len(self.capacities)==1,
                      colliding_pairs=sorted(self.contact_pairs), contact_pair_count=len(self.contact_pairs) if self.contact_frames else None,
                      contact_frames=self.contact_frames, authorized_ground_contact_frames=self.ground_contacts,
                      contact_instrumentation='connected' if self.contact_frames else 'no_contact_messages_received',
                      coverage=coverage, coverage_verified=coverage is not None,
                      coverage_source=self.coverage_source, saved_map_verified=self.saved_map_verified,
                      mission_completed=mission.get('success', False),
                      acceptance_pass=False)
        result.update(flight_metrics)
        result['acceptance_pass'] = acceptance_pass(mission, result)
        atomic_json(self.args.output/'evaluation.json', result)

    def coverage(self):
        # Explicit benchmark ROI: physically reachable, inflated free centers
        # inside the warehouse, at 0.4 m spacing. It is generated only here from
        # collision geometry and is never delivered to either planner.
        xs = np.arange(-6.6, 6.61, .4); ys = np.arange(-10., 10.01, .4); zs = np.arange(.9, 8.11, .4)
        points = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1).reshape(-1, 3)
        distances = []
        for start in range(0, len(points), 4000):
            distances.extend(self.proximity.on_surface(points[start:start+4000])[1])
        free = (np.array(distances)>=.65+.2*np.sqrt(3)).reshape(len(xs), len(ys), len(zs))
        origin = self.offset + np.array([0., 0, 1.5])
        candidates = np.argwhere(free)
        if not len(candidates): return None
        source = candidates[np.argmin(np.linalg.norm(points[free.ravel()]-origin, axis=1))]
        todo = deque([tuple(source)]); reachable = {tuple(source)}
        while todo:
            current = todo.popleft()
            for axis in range(3):
                for sign in (-1, 1):
                    adjacent = list(current); adjacent[axis] += sign; adjacent = tuple(adjacent)
                    if any(v<0 or v>=free.shape[i] for i, v in enumerate(adjacent)): continue
                    if free[adjacent] and adjacent not in reachable:
                        reachable.add(adjacent); todo.append(adjacent)
        queries = np.asarray([[xs[i],ys[j],zs[k]] for i,j,k in sorted(reachable)])-self.offset
        covered = sum(tuple(np.floor(point/.2).astype(int)) in self.known for point in queries)
        self.coverage_source = 'observed_free_union_diagnostic_only'
        artifact = self.args.mission_result.parent/'warehouse.bt' if self.args.mission_result else None
        query_tool = Path(__file__).resolve().parents[1]/'install/uav_autonomy/lib/uav_autonomy/map_query'
        if artifact and artifact.exists() and query_tool.exists():
            query_text = ''.join(' '.join(map(str,p))+'\n' for p in queries)
            inspected = subprocess.run([str(query_tool),str(artifact)],input=query_text,text=True,
                                       capture_output=True,check=True,timeout=20)
            states = np.fromstring(inspected.stdout,dtype=int,sep=' ')
            if len(states)!=len(queries): raise RuntimeError('Saved OctoMap query count mismatch')
            covered = int(np.sum(states==0))
            self.coverage_source = 'saved_exploration_octomap'
            self.saved_map_verified = True
        np.savetxt(self.args.output/'coverage_queries_odom.xyz',queries,fmt='%.9f')
        (self.args.output/'coverage_roi.json').write_text(json.dumps(dict(definition='reachable inflated warehouse free centers',
            resolution=.4, safety_inflation=.65+.2*np.sqrt(3), cells=len(reachable), covered=covered,
            source=self.coverage_source), indent=2)+'\n')
        return covered/len(reachable) if reachable else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world', required=True); p.add_argument('--world-file', required=True, type=Path)
    p.add_argument('--vehicle', required=True); p.add_argument('--task', choices=['forest', 'explore'], required=True)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--duration', type=float, default=0)
    p.add_argument('--contacts-manifest', type=Path)
    p.add_argument('--mission-result', type=Path)
    p.add_argument('--session-pid', type=int)
    args = p.parse_args(); rclpy.init(); node = Evaluator(args)
    def stop(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    end = time.monotonic()+args.duration if args.duration else float('inf')
    try:
        while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        with protected_finalization():
            try:
                node.finish()
            except Exception as error:
                atomic_json(args.output/'evaluation_error.json',
                    dict(acceptance_pass=False, error_type=type(error).__name__, error=str(error)))
                raise
            finally:
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
