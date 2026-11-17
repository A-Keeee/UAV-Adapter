#!/usr/bin/env python3
"""Open RViz and its read-only display adapter without starting a mission."""
import argparse
import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

from ego_environment import without_ego_overlays
from owned_processes import stop_group

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-rviz', action='store_true', help='Run the display adapter only.')
    parser.add_argument('--config', type=Path, default=ROOT/'uav_autonomy/config/autonomy.rviz')
    parser.add_argument('--duration', type=float, default=0., help='Close after wall seconds; 0 keeps the viewer open.')
    args = parser.parse_args()
    if args.duration < 0:
        parser.error('--duration cannot be negative')
    if not args.no_rviz and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No desktop display; start this on the graphical/remote desktop, or use --no-rviz.')
    if not args.config.is_file():
        parser.error('RViz configuration is missing: '+str(args.config))
    # This adapter has no EGO message-library dependencies beyond the common
    # transport messages. Keep inherited duplicate planner overlays out of Qt.
    environment = without_ego_overlays(os.environ)
    os.environ.update(environment)
    import rclpy
    from autonomy_visualization import AutonomyVisualization
    rclpy.init()
    node = AutonomyVisualization()
    rviz = None
    log = None
    folder = ROOT/'log/autonomy_visualization'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    try:
        if not args.no_rviz:
            log = (folder/'rviz.log').open('w')
            rviz = subprocess.Popen(['rviz2', '-d', str(args.config.resolve()), '--ros-args',
                '-p', 'use_sim_time:=true', '-r', '__node:=autonomy_rviz'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        print('Read-only autonomy viewer; no arming or control commands. Logs: '+str(folder), flush=True)
        deadline = time.monotonic()+args.duration if args.duration else float('inf')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError('RViz failed; see '+str(folder/'rviz.log'))
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rviz is not None:
            stop_group(rviz, timeout=3)
        if log is not None:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
