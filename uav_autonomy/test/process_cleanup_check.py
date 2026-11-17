#!/usr/bin/env python3
"""A terminated ros2 launcher must not leave a native planner alive."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import psutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from owned_processes import live_group, signal_group, stop_group


def wait_for(path):
    end = time.monotonic()+3
    while not path.exists() and time.monotonic()<end:
        time.sleep(.01)
    assert path.exists(), 'Graceful shutdown fixture did not reach '+path.name


with tempfile.TemporaryDirectory() as folder:
    folder = Path(folder)
    fixture = folder/'graceful.py'
    fixture.write_text('''import json, signal, sys, time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from evaluation_finalization import atomic_json, protected_finalization
folder = Path(sys.argv[2])
protected = sys.argv[3] == "protected"
def stop(number, frame): raise KeyboardInterrupt
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
(folder/"ready").touch()
def finish():
    (folder/"finishing").touch()
    time.sleep(.5)
    atomic_json(folder/"evaluation.json", {"contact_frames": 12, "acceptance_pass": False})
try:
    while True: time.sleep(.1)
except KeyboardInterrupt: pass
finally:
    if protected:
        with protected_finalization(): finish()
    else: finish()
''')
    for kind in ('single_signal', 'protected'):
        output = folder/kind; output.mkdir()
        process = subprocess.Popen([sys.executable, str(fixture), str(ROOT/'scripts'), str(output), kind],
                                   start_new_session=True)
        try:
            wait_for(output/'ready')
            signal_group(process, signal.SIGTERM)
            wait_for(output/'finishing')
            if kind=='protected':
                signal_group(process, signal.SIGTERM)
                signal_group(process, signal.SIGINT)
            stop_group(process, timeout=3, already_signalled=True)
            assert process.returncode==0, 'Scoring finalization was interrupted'
            assert (output/'evaluation.json').exists(), 'Independent evidence was lost'
        finally:
            stop_group(process, timeout=.1)
print('Single shutdown signal and protected scoring under repeated TERM/INT: final evidence retained, PASS')

with tempfile.TemporaryDirectory() as folder:
    marker = Path(folder)/'child'
    child = 'import os,signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);open('+repr(str(marker))+',"w").write(str(os.getpid()));time.sleep(60)'
    launcher = 'import subprocess;subprocess.Popen(["/usr/bin/python3","-c",'+repr(child)+'])'
    process = subprocess.Popen(['/usr/bin/python3', '-c', launcher], start_new_session=True)
    try:
        process.wait(timeout=3)
        end = time.monotonic()+3
        while not marker.exists() and time.monotonic()<end:
            time.sleep(.01)
        assert marker.exists() and live_group(process.pid), 'Orphan fixture was not started'
        stop_group(process, timeout=.15)
        end = time.monotonic()+2
        while live_group(process.pid) and time.monotonic()<end:
            time.sleep(.02)
        assert not live_group(process.pid), 'Native descendant survived launcher cleanup'
    finally:
        try: os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError: pass
print('Exited launcher and SIGTERM-resistant native descendant: owned group fully stopped, PASS')

# Anonymous publishers must also exit when their batch owner disappears.
owner = subprocess.Popen(['/usr/bin/python3','-c','import time;time.sleep(60)'])
environment = dict(os.environ,ROS_DOMAIN_ID=str(170+os.getpid()%40),ROS_LOCALHOST_ONLY='1')
proxy = subprocess.Popen(['/usr/bin/python3',str(ROOT/'install/uav_autonomy/lib/uav_autonomy/cloud_proxy'),
    '--owner-pid',str(owner.pid),'--owner-start-time',str(psutil.Process(owner.pid).create_time())],
    env=environment,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
try:
    time.sleep(1)
    assert proxy.poll() is None,'Proxy owner fixture failed to start'
    owner.terminate()
    assert proxy.wait(timeout=5)==0,'Anonymous publisher survived its batch owner'
    owner.wait(timeout=2)
finally:
    if owner.poll() is None:owner.terminate();owner.wait(timeout=2)
    stop_group(proxy,timeout=.2)
print('Anonymous PointCloud2 publisher stops after its owning batch exits: PASS')
