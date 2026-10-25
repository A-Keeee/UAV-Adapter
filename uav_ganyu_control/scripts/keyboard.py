#!/usr/bin/python3
"""Interactive keyboard for the GanYu ROS 2 controller; movement is in world N/E/up."""
import argparse
import json
import os
import select
import signal
import socket
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String


class Keyboard(Node):
    def __init__(self, use_gcs):
        super().__init__('ganyu_keyboard')
        self.key_pub = self.create_publisher(String, '/ganyu/keyboard', 10)
        self.alive_pub = self.create_publisher(Empty, '/ganyu/keyboard_alive', 10)
        self.status = None
        self.status_at = 0
        self.subscription = self.create_subscription(String, '/ganyu/status', self.on_status, 10)
        self.gcs = None
        if use_gcs:
            try:
                from pymavlink import mavutil
                self.gcs = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255, source_component=190)
                self.mavutil = mavutil
                print('已启用本机仿真地面站心跳。', flush=True)
            except (OSError, socket.error) as exc:
                print(f'UDP 14550 已占用，未启动内置地面站心跳：{exc}\n请确认已有地面站连接。', flush=True)
        self.last_heartbeat = 0
        self.timer = self.create_timer(0.1, self.heartbeat)

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_at = time.monotonic()

    def heartbeat(self):
        self.alive_pub.publish(Empty())
        if self.gcs is not None:
            for _ in range(40):
                if self.gcs.recv_match(blocking=False) is None:
                    break
            if time.monotonic() - self.last_heartbeat >= 1:
                mav = self.mavutil.mavlink
                self.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
                self.last_heartbeat = time.monotonic()

    def send(self, key):
        msg = String()
        msg.data = key
        self.key_pub.publish(msg)

    def land_before_exit(self):
        print('\n请求降落，等待落地并上锁后退出……', flush=True)
        deadline = time.monotonic() + 45
        requested_at = time.monotonic()
        last_send = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - last_send >= 1:
                self.send('x')
                last_send = time.monotonic()
            if self.status and time.monotonic() - requested_at > 0.5 and self.status_at > requested_at and time.monotonic() - self.status_at < 1 and not self.status['armed']:
                # A fresh IDLE acknowledgment also cancels any pending prestream / arm request.
                if self.status['phase'] == 'IDLE':
                    print('已确认未解锁，键盘控制退出。', flush=True)
                    return True
        print('未能在 45 秒内确认落地上锁，请检查 Gazebo / 地面站。', flush=True)
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--external-gcs', action='store_true')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        raise SystemExit('请在可交互终端运行键盘控制，不能重定向标准输入。')
    # Keep ROS alive while Ctrl+C performs the landing handshake.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Keyboard(not args.external_gcs)
    print('''
GanYu 单机键盘控制（固定世界方向，无自动避障）
  t       起飞到 1.5 m（默认值；等待“就绪”后按）
  w / s   北 / 南，每次改变目标 0.25 m
  a / d   西 / 东，每次改变目标 0.25 m
  e / q   上升 / 下降，每次改变目标 0.25 m
  空格    将当前位置设为悬停目标
  l       PX4 受控降落并上锁
  x       降落后退出；Ctrl+C 同样先请求降落
松开移动键后保持最后目标。水平目标限在起飞点 ±3 m，高度 0.5～3 m。
操作此终端时保持键盘焦点；查看画面后需点击此终端继续控制。
''', flush=True)
    settings = termios.tcgetattr(sys.stdin.fileno())
    last_display = 0
    result = 0
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0.03)[0]:
                # Read the tty fd directly: TextIO buffering can hide queued keys from select().
                raw_key = os.read(sys.stdin.fileno(), 1)
                if not raw_key:
                    raise KeyboardInterrupt()
                key = raw_key.decode('ascii', errors='ignore').lower()
                if key in ('x', '\x03'):
                    result = 0 if node.land_before_exit() else 1
                    break
                if key and key in 'twasdeql ':
                    node.send(key)
                    print(f'\n按键 {"空格" if key == " " else key}', flush=True)
            if time.monotonic()-last_display > 1:
                last_display = time.monotonic()
                if node.status and time.monotonic()-node.status_at < 2:
                    s = node.status
                    n, e, h = s['offset_neu']
                    ready = '就绪' if s['ready'] else '等待预检/数据'
                    print(f'\r\033[K{ready} | {s["phase"]} | 北 {n:+.2f} 东 {e:+.2f} 高 {h:.2f} m | '
                          f'{"已解锁" if s["armed"] else "未解锁"}', end='', flush=True)
                else:
                    print('\r\033[K等待 GanYu 控制器及仿真数据……', end='', flush=True)
    except KeyboardInterrupt:
        result = 0 if node.land_before_exit() else 1
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        if node.gcs:
            node.gcs.close()
        node.destroy_node()
        rclpy.shutdown()
        print()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
