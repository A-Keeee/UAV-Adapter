#!/usr/bin/env python3
"""Anonymous external PointCloud2 publisher and reproducible input fault injector."""
import argparse
import math
import struct
import time
import copy
import sys
import psutil
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


class Proxy(Node):
    def __init__(self, args):
        super().__init__('anonymous_cloud_proxy')
        self.args = args; self.last = -math.inf; self.counter = 0; self.tf_sent = False
        self.first_stamp = None
        self.tf = StaticTransformBroadcaster(self)
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.create_timer(.02, self.retry_pending)
        self.publisher = self.create_publisher(PointCloud2, args.output, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.input, self.receive, qos_profile_sensor_data)
        if args.owner_pid:
            self.create_timer(.5,self.check_owner)

    def check_owner(self):
        try:
            process=psutil.Process(self.args.owner_pid)
            alive=process.is_running() and process.status()!=psutil.STATUS_ZOMBIE and abs(process.create_time()-self.args.owner_start_time)<.01
        except psutil.Error:
            alive=False
        if not alive:
            raise SystemExit(0)

    def retry_pending(self):
        if self.pending is not None: self.receive(self.pending)

    def receive(self, msg):
        world_transform = None
        if self.args.world_frame:
            try: world_transform = self.buffer.lookup_transform(self.args.world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
            except TransformException:
                self.pending = msg
                return
            self.pending = None
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.first_stamp is None: self.first_stamp = t
        elapsed = t-self.first_stamp
        if self.args.drop_after is not None and elapsed>=self.args.drop_after: return
        if self.args.pause_at is not None and self.args.pause_at<=elapsed<self.args.pause_at+self.args.pause_for: return
        if self.args.frequency and t-self.last < 1/self.args.frequency: return
        self.last = t; self.counter += 1
        if not self.tf_sent and not self.args.missing_tf:
            transform = TransformStamped(); transform.header = msg.header
            transform.child_frame_id = self.args.frame; transform.transform.rotation.w = 1.
            self.tf.sendTransform(transform); self.tf_sent = True
        if self.args.layout == 'passthrough' and self.args.every == 1 and self.args.forward_cosine is None and not self.args.world_frame:
            out = copy.copy(msg)
            out.header = copy.deepcopy(msg.header)
        else:
            fields = {field.name: field for field in msg.fields}
            for name in ('x', 'y', 'z'):
                if name not in fields or fields[name].datatype not in (7, 8): raise ValueError('Expected float XYZ')
            points = []; endian = '>' if msg.is_bigendian else '<'
            for row in range(msg.height):
                for col in range(msg.width):
                    if (row*msg.width+col) % self.args.every: continue
                    point = tuple(struct.unpack_from(endian+('f' if fields[name].datatype == 7 else 'd'),
                        msg.data, row*msg.row_step+col*msg.point_step+fields[name].offset)[0] for name in ('x', 'y', 'z'))
                    if self.args.forward_cosine is not None:
                        length = math.sqrt(sum(v*v for v in point))
                        if not length or point[0]/length < self.args.forward_cosine: continue
                    points.append(point)
            if self.args.world_frame:
                transform = world_transform
                q = transform.rotation; x,y,z,w = q.x,q.y,q.z,q.w
                rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                translation = transform.translation
                if points: points = (np.asarray(points)@rotation.T+np.array([translation.x,translation.y,translation.z])).tolist()
            out = PointCloud2(); out.header = copy.deepcopy(msg.header)
            is64 = self.args.layout in ('float64', 'big_endian')
            size = 8 if is64 else 4; datatype = 8 if is64 else 7
            names = ('z', 'x', 'y') if self.args.layout != 'xyz' else ('x', 'y', 'z')
            out.fields = [PointField(name=name, offset=i*size, datatype=datatype, count=1) for i, name in enumerate(names)]
            out.point_step = 3*size; out.height = 2 if self.args.layout == 'padded' and points else 1
            out.width = (len(points)+out.height-1)//out.height
            out.row_step = out.width*out.point_step+(7 if self.args.layout == 'padded' else 0)
            out.is_bigendian = self.args.layout == 'big_endian'; out.is_dense = False
            data = bytearray(out.row_step*out.height)
            fmt = ('>' if out.is_bigendian else '<')+('d' if is64 else 'f')*3
            for i in range(out.width*out.height):
                point = points[i] if i < len(points) else (math.nan,)*3
                by_name = dict(zip(('x', 'y', 'z'), point))
                struct.pack_into(fmt, data, (i//out.width)*out.row_step+(i%out.width)*out.point_step, *(by_name[name] for name in names))
            out.data = bytes(data)
        out.header.frame_id = self.args.world_frame or self.args.frame
        if self.args.time_offset:
            stamp = int((t+self.args.time_offset)*1e9)
            out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp, 10**9)
        if self.args.empty_every and self.counter % self.args.empty_every == 0:
            out.width = 0; out.height = 1; out.row_step = 0; out.data = b''
        self.publisher.publish(out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', default='/lidar/points'); p.add_argument('--output', default='/external/points')
    p.add_argument('--frame', default='unregistered_sensor_37')
    p.add_argument('--layout', choices=['passthrough', 'xyz', 'reordered', 'float64', 'padded', 'big_endian'], default='passthrough')
    p.add_argument('--every', type=int, default=1); p.add_argument('--frequency', type=float, default=0)
    p.add_argument('--forward-cosine', type=float); p.add_argument('--empty-every', type=int, default=0)
    p.add_argument('--time-offset', type=float, default=0); p.add_argument('--missing-tf', action='store_true')
    p.add_argument('--world-frame', help='Transform XYZ at acquisition time; --frame remains the true measurement origin TF.')
    p.add_argument('--drop-after', type=float, help='Stop publishing after this many sensor seconds.')
    p.add_argument('--pause-at', type=float, help='Begin a recoverable interruption after this many sensor seconds.')
    p.add_argument('--pause-for', type=float, default=3)
    p.add_argument('--owner-pid', type=int, default=0)
    p.add_argument('--owner-start-time', type=float, default=0)
    from rclpy.utilities import remove_ros_args
    args = p.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.every < 1 or args.frequency < 0 or args.pause_for<0 or any(v is not None and v<0 for v in (args.drop_after,args.pause_at)): p.error('Invalid density/frequency/interruption')
    rclpy.init(args=sys.argv); node = Proxy(args)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
