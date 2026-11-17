"""Read-only display adapter. Output is confined to /autonomy/visualization/.

It never publishes goals, references, TF, planner feedback or flight commands.
Native curves are position samples for RViz, not an execution authorization.
"""
from collections import OrderedDict, deque
import json
import math
import time

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from uav_common_msg.msg import ExternalReference, NativeTrajectory, TrajectoryFeedback
from visualization_msgs.msg import Marker, MarkerArray

from visualization_trajectory import sample_native


PREFIX = '/autonomy/visualization/'


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def stamp_at(value):
    from builtin_interfaces.msg import Time
    ns = max(0, round(value*1e9))
    return Time(sec=ns//10**9, nanosec=ns%10**9)


def point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class AutonomyVisualization(Node):
    def __init__(self):
        super().__init__('autonomy_visualization')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        limit = self.declare_parameter('max_history_points', 3000).value
        if not isinstance(limit, int) or not 100 <= limit <= 6000:
            raise ValueError('max_history_points must be between 100 and 6000')
        self.actual = deque(maxlen=limit)
        self.reference = deque(maxlen=limit)
        self.odom = None
        self.goal = None
        self.status = {}
        self.status_at = None
        self.clock_value = None
        self.clock_changed_at = None
        self.candidates = OrderedDict()
        self.candidate = None
        self.confirmed = None
        self.pending_accept = None
        self.revoked_sequence = 0
        self.revoked_at = -math.inf
        self.last_feedback = ''
        self.path_publishers = {name:self.create_publisher(Path, PREFIX+name, 1)
            for name in ('actual_path', 'reference_path', 'candidate_path', 'confirmed_path')}
        self.markers = self.create_publisher(MarkerArray, PREFIX+'markers', 1)
        self.create_subscription(Odometry, '/autonomy/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ExternalReference, '/ganyu/control_reference', self.on_reference, qos_profile_sensor_data)
        self.create_subscription(NativeTrajectory, '/autonomy/native_trajectory', self.on_native, 10)
        # This feedback carries the adapter's native ID. /native_accepted
        # instead carries the upstream planner ID, which is a different ID.
        self.create_subscription(TrajectoryFeedback, '/autonomy/trajectory_feedback', self.on_feedback, 10)
        self.create_subscription(PoseStamped, '/autonomy/goal', self.on_goal, 10)
        self.create_subscription(String, '/autonomy/status', self.on_status, 10)
        self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)
        # Refresh diagnostics while simulation is paused, without changing its time.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.2, self.publish, clock=self.wall_clock)
        self.get_logger().info('Read-only RViz adapter ready; no control or TF publishers.')

    def reset(self):
        self.actual.clear()
        self.reference.clear()
        self.odom = None
        self.goal = None
        self.candidates.clear()
        self.candidate = self.confirmed = self.pending_accept = None
        self.status = {}
        self.status_at = None
        self.revoked_sequence = 0
        self.revoked_at = -math.inf

    def on_clock(self, message):
        value = seconds(message.clock)
        if self.clock_value is not None and value < self.clock_value:
            self.reset()
        if value != self.clock_value:
            self.clock_changed_at = time.monotonic()
        self.clock_value = value

    def append_history(self, history, stamp, xyz):
        if not all(math.isfinite(v) for v in xyz) or stamp <= 0:
            return
        if history and stamp <= history[-1][0]:
            return
        if history and (stamp-history[-1][0] < .1 or math.dist(xyz, history[-1][1]) < .02):
            return
        history.append((stamp, tuple(xyz)))

    def on_odom(self, message):
        if message.header.frame_id != 'odom':
            return
        p = message.pose.pose.position
        xyz = (p.x, p.y, p.z)
        stamp = seconds(message.header.stamp)
        if not all(math.isfinite(v) for v in xyz):
            return
        if self.actual and (stamp < self.actual[-1][0] or math.dist(xyz, self.actual[-1][1]) > 3):
            self.reset()
        self.odom = message
        self.append_history(self.actual, stamp, xyz)

    def on_reference(self, message):
        if message.header.frame_id == 'odom':
            self.append_history(self.reference, seconds(message.header.stamp), message.position)

    def on_goal(self, message):
        if message.header.frame_id == 'odom' and all(math.isfinite(v) for v in
                (message.pose.position.x, message.pose.position.y, message.pose.position.z)):
            self.goal = message

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict):
                self.status = data
                self.status_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_native(self, message):
        if message.trajectory_id <= self.revoked_sequence or seconds(message.start_time) < self.revoked_at:
            return
        try:
            times, positions = sample_native(message)
        except (ValueError, TypeError, IndexError) as error:
            self.get_logger().warning('Invalid display trajectory: '+str(error), throttle_duration_sec=3.)
            return
        item = (message, times, positions)
        self.candidates[message.trajectory_id] = item
        self.candidate = item
        while len(self.candidates) > 8:
            self.candidates.popitem(last=False)
        if self.pending_accept == message.trajectory_id:
            self.confirmed = item
            self.pending_accept = None

    def on_feedback(self, message):
        self.last_feedback = message.reason
        if message.braking:
            self.revoked_sequence = max([self.revoked_sequence, *self.candidates.keys()])
            self.revoked_at = self.get_clock().now().nanoseconds*1e-9
            self.confirmed = self.candidate = self.pending_accept = None
            self.candidates.clear()
        elif message.accepted and message.trajectory_id > self.revoked_sequence:
            self.confirmed = self.candidates.get(message.trajectory_id)
            self.pending_accept = message.trajectory_id if self.confirmed is None else None
        # Rejection of a replacement does not revoke the prior confirmed curve.

    def make_path(self, samples):
        result = Path()
        result.header.frame_id = 'odom'
        result.header.stamp = self.get_clock().now().to_msg()
        for stamp, xyz in samples:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = stamp_at(stamp)
            pose.pose.position = point(xyz)
            pose.pose.orientation.w = 1.
            result.poses.append(pose)
        return result

    def native_path(self, item):
        if item is None:
            return self.make_path(())
        message, times, positions = item
        start = seconds(message.start_time)
        return self.make_path((start+float(t), xyz) for t, xyz in zip(times, positions))

    def marker(self, ident, kind, xyz, color, scale):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'autonomy_view'
        m.id = ident
        m.type = kind
        m.action = Marker.ADD
        m.pose.position = point(xyz)
        m.pose.orientation.w = 1.
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def publish(self):
        now = self.get_clock().now().nanoseconds*1e-9
        for name in ('candidate', 'confirmed'):
            item = getattr(self, name)
            if item is not None and now > seconds(item[0].start_time)+float(item[1][-1]):
                setattr(self, name, None)
        self.path_publishers['actual_path'].publish(self.make_path(self.actual))
        self.path_publishers['reference_path'].publish(self.make_path(self.reference))
        self.path_publishers['candidate_path'].publish(self.native_path(self.candidate))
        self.path_publishers['confirmed_path'].publish(self.native_path(self.confirmed))
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        xyz = (0., 0., 1.)
        speed = 0.
        if self.odom is not None:
            p = self.odom.pose.pose.position
            xyz = (p.x, p.y, p.z)
            v = self.odom.twist.twist.linear
            speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            body = self.marker(0, Marker.ARROW, xyz, (.2, .7, 1., 1.), (.8, .16, .16))
            body.pose.orientation = self.odom.pose.pose.orientation
            markers.markers.append(body)
        if self.goal is not None:
            p = self.goal.pose.position
            markers.markers.append(self.marker(1, Marker.SPHERE, (p.x, p.y, p.z), (1., .25, .1, .9), (.3, .3, .3)))
        label = self.marker(2, Marker.TEXT_VIEW_FACING, (xyz[0], xyz[1], xyz[2]+1.2), (1., 1., 1., 1.), (0., 0., .24))
        stale = self.status_at is None or time.monotonic()-self.status_at > 2.
        phase = str(self.status.get('phase', 'WAITING FOR DATA'))[:50]
        limit = self.status.get('planning_speed_limit_mps')
        limit_text = f'{limit:.2f}' if isinstance(limit, (int, float)) else '--'
        clock_stopped = self.clock_changed_at is None or time.monotonic()-self.clock_changed_at > 1.5
        label.text = (f'{phase}'+(' | STATUS STALE' if stale else '')+
                      f'\nSpeed {speed:.2f} m/s | Limit {limit_text} m/s'+
                      ('\nSIMULATION PAUSED / WAITING FOR CLOCK' if clock_stopped else '')+
                      '\nBlue: actual | Magenta: reference\nGreen: EGO curve preview')
        markers.markers.append(label)
        self.markers.publish(markers)
