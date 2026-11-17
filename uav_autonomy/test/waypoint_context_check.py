#!/usr/bin/env python3
"""A patrol turn counts as arrival; exploration and earlier patrols do not."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import numpy as np
from waypoint_context import patrol_truth_errors
from trial_index import latest_configurations, latest_fault_configurations

target=[[0.,0.,1.]]
positions=[[0.,0.,1.],[.4,0.,1.],[.1,0.,1.]]
assert patrol_truth_errors(positions,[0,0,0],[0,1,1],target,np.zeros(3))==[.1]
assert patrol_truth_errors(positions[:2],[0,0],[0,1],target,np.zeros(3))==[.4]
assert patrol_truth_errors(positions,[0,0,0],[1,2,1],target,np.zeros(3))==[.4]
assert patrol_truth_errors(positions,[0,0,1],[0,1,1],target,np.zeros(3))==[.4]
print('Active patrol target includes turns; exploration, other targets and prior patrols excluded: PASS')

older=dict(task='forest',simulated_payload='mid360',condition=0,attempt=1,acceptance_pass=True)
newer=dict(older,attempt=2,acceptance_pass=False)
assert not latest_configurations([newer,older])[('forest','mid360',0)]['acceptance_pass']
good=dict(fault_kind='cloud_pause',simulated_payload='mid360',attempt=1,fault_check_pass=True)
bad=dict(good,attempt=2,fault_check_pass=False)
other=dict(good,simulated_payload='avia',attempt=3)
selected=latest_fault_configurations([bad,good,other])
assert not selected[('cloud_pause','mid360')]['fault_check_pass']
assert selected[('cloud_pause','avia')]['fault_check_pass']
assert latest_fault_configurations([good,bad,dict(good,attempt=3)])[('cloud_pause','mid360')]['fault_check_pass']
print('Normal/fault trial resumption and acceptance use latest attempts, separately per payload: PASS')
