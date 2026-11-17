#!/usr/bin/env python3
"""PX4 NED/FRD -> timestamped ENU/FLU odometry and the sole dynamic body TF."""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry, TimesyncStatus
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class StateBridge(Node):
    def __init__(self):
        super().__init__('autonomy_state_bridge')
        prefix = self.declare_parameter('px4_namespace', '/px4_1').value
        self.domain = self.declare_parameter('timestamp_domain', 'dds').value
        if self.domain not in ('dds', 'simulation'):
            raise ValueError('timestamp_domain must be dds or simulation')
        self.sim_ns = None
        self.offset_us = None
        self.previous_ns = 0
        self.reset_counter = None
        self.previous_position = None
        self.previous_velocity = None
        self.previous_quaternion = None
        self.odom = self.create_publisher(Odometry, '/autonomy/odom', 10)
        self.status = self.create_publisher(String, '/autonomy/state_status', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, prefix+'/fmu/out/timesync_status', self.on_sync, qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, prefix+'/fmu/out/vehicle_odometry', self.on_odom, qos_profile_sensor_data)

    def on_clock(self, m):
        stamp = m.clock.sec*10**9+m.clock.nanosec
        if self.sim_ns is not None and stamp < self.sim_ns:
            self.previous_ns = 0
            self.report(False, 'simulation_clock_reset')
        self.sim_ns = stamp

    def on_sync(self, m):
        # PX4 uXRCE serialization adds the DDS session offset. TimesyncStatus
        # estimated_offset is local minus remote, in microseconds (DDS protocol=2).
        if m.source_protocol == 2:
            self.offset_us = int(m.estimated_offset)

    def report(self, valid, reason, **kwargs):
        if reason.startswith('ekf_'):
            self.get_logger().info(json.dumps(dict(valid=valid, reason=reason, **kwargs)))
        self.status.publish(String(data=json.dumps(dict(valid=valid, reason=reason, **kwargs))))

    def on_odom(self, m):
        if self.sim_ns is None or (self.domain == 'dds' and self.offset_us is None):
            return
        stamp = (int(m.timestamp_sample)+(self.offset_us if self.domain == 'dds' else 0))*1000
        age = (self.sim_ns-stamp)*1e-9
        if stamp <= 0 or stamp <= self.previous_ns or age < -.1 or age > .5:
            self.report(False, 'sample_clock_mismatch', age=age)
            return
        if m.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.report(False, 'unsupported_pose_frame')
            return
        p = np.asarray(m.position, dtype=float)
        v = np.asarray(m.velocity, dtype=float)
        q = np.asarray(m.q, dtype=float)  # wxyz, body FRD to NED
        if not np.all(np.isfinite(np.r_[p, v, q])) or abs(np.linalg.norm(q)-1) > .05:
            return
        q /= np.linalg.norm(q)
        w, x, y, z = q
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                             [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                             [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
        if m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v = rotation @ v
        elif m.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            return
        enu_ned = np.array([[0., 1, 0], [1, 0, 0], [0, 0, -1]])
        # Quaternion composition q(ENU<-NED) * q(NED<-FRD) * q(FRD<-FLU).
        def multiply(a, b):
            aw, ax, ay, az = a; bw, bx, by, bz = b
            return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                             aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])
        out_q = multiply(multiply([0, 1/math.sqrt(2), 1/math.sqrt(2), 0], q), [0, 1, 0, 0])
        p = enu_ned @ p; v = enu_ned @ v
        out = Odometry()
        out.header.frame_id = 'odom'; out.child_frame_id = 'base_link'
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp, 10**9)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, p)
        out.pose.pose.orientation.w, out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z = map(float, out_q)
        # nav_msgs twist is expressed in the child frame (FLU).
        body_v = (enu_ned @ rotation @ np.diag([1., -1, -1])).T @ v
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, body_v)
        rate = np.asarray(m.angular_velocity)*[1., -1, -1]
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, rate)
        tf = TransformStamped(); tf.header = out.header; tf.child_frame_id = out.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
        tf.transform.rotation = out.pose.pose.orientation
        discontinuous = False
        if self.reset_counter is not None and m.reset_counter != self.reset_counter:
            dt = (stamp-self.previous_ns)*1e-9
            displacement = np.linalg.norm(p-self.previous_position-self.previous_velocity*dt)
            rotation_jump = 2*math.acos(float(np.clip(abs(out_q @ self.previous_quaternion), 0, 1)))
            # VehicleOdometry reset_counter also changes for velocity-only EKF
            # resets. Abort only if the measured pose loses continuity.
            discontinuous = displacement > .10 or rotation_jump > .10
            self.report(not discontinuous, 'ekf_origin_reset' if discontinuous else 'ekf_state_reset_continuous',
                        displacement=float(displacement), rotation_jump=rotation_jump)
        if not discontinuous:
            self.odom.publish(out); self.tf.sendTransform(tf)
        self.previous_position = p; self.previous_velocity = v; self.previous_quaternion = out_q
        self.reset_counter = m.reset_counter; self.previous_ns = stamp


def main():
    rclpy.init()
    node = StateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
