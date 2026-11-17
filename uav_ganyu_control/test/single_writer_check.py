#!/usr/bin/env python3
"""Real controller processes must reject duplicate writers, then release on exit."""
import os
import signal
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID'] = str(210+os.getpid()%20)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from std_msgs.msg import String

rclpy.init();node=rclpy.create_node('single_writer_regression');statuses=[]
node.create_subscription(String,'/ganyu/status',statuses.append,10)
processes=[]
prefix='/writer_regression_'+str(os.getpid())
with tempfile.TemporaryFile() as log:
    def start(namespace,extra=()):
        p=subprocess.Popen([sys.argv[1],'--ros-args','-p','px4_namespace:='+namespace,*extra],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def alive(p):
        end=time.monotonic()+1.5
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            assert p.poll() is None,'Controller unexpectedly exited'
    try:
        first=start(prefix);alive(first)
        assert statuses,'First controller did not initialize'
        second=start(prefix);assert second.wait(timeout=4)==1,'Duplicate writer was accepted'
        log.seek(0);assert b'Another GanYu controller already owns' in log.read(),'Duplicate was not rejected by the ownership lock'
        alias=start(prefix.lstrip('/'),['-p','target_system:=3'])
        assert alias.wait(timeout=4)==1,'Relative namespace or target-system alias bypassed ownership'
        remapped=start(prefix+'_remapped',['-r',prefix+'_remapped/fmu/in/vehicle_rates_setpoint:='+prefix+'/fmu/in/vehicle_rates_setpoint'])
        assert remapped.wait(timeout=4)==1,'A remapped duplicate output bypassed ownership'
        other=start(prefix+'_other');alive(other)
        os.killpg(first.pid,signal.SIGTERM);first.wait(timeout=4)
        replacement=start(prefix);alive(replacement)
        print('Duplicate output rejected; separate namespace allowed; ownership released on exit: PASS')
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
        for p in processes:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        node.destroy_node();rclpy.shutdown()
