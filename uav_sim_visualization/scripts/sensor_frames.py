#!/usr/bin/python3
"""Read PX4 NED/FRD odometry and publish an ENU/FLU visualization TF tree."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw-ax*bx-ay*by-az*bz,
            aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw)


def ned_frd_to_enu_flu(position, quaternion):
    values = [float(value) for value in (*position, *quaternion)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Non-finite odometry')
    norm = math.sqrt(sum(float(value)**2 for value in quaternion))
    if norm < 1e-6:
        raise ValueError('Zero quaternion')
    q = [float(value)/norm for value in quaternion]
    # ENU<-NED is a pi rotation about (1,1,0); FRD<-FLU is a pi rotation about X.
    s = math.sqrt(0.5)
    enu_flu = multiply(multiply((0.0, s, s, 0.0), q), (0.0, 1.0, 0.0, 0.0))
    return (float(position[1]), float(position[0]), -float(position[2])), enu_flu


class SensorFrames(Node):
    def __init__(self):
        super().__init__('sim_sensor_frames')
        namespace = self.declare_parameter('px4_namespace', '/px4_1').value
        self.world_frame = self.declare_parameter('world_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        lidar = TransformStamped()
        lidar.header.stamp = self.get_clock().now().to_msg()
        lidar.header.frame_id = self.base_frame
        lidar.child_frame_id = 'mid360_link'
        lidar.transform.translation.z = 0.12
        lidar.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(lidar)
        self.subscription = self.create_subscription(
            VehicleOdometry, namespace + '/fmu/out/vehicle_odometry', self.on_odometry,
            qos_profile_sensor_data)
        self.get_logger().info('Visualization TF only: odom -> base_link -> mid360_link; no control publishers')

    def on_odometry(self, msg):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        try:
            position, quaternion = ned_frd_to_enu_flu(msg.position, msg.q)
        except ValueError:
            return
        stamp = self.get_clock().now()
        if stamp.nanoseconds == 0:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        t = transform.transform.translation
        t.x, t.y, t.z = position
        q = transform.transform.rotation
        q.w, q.x, q.y, q.z = quaternion
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = SensorFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
