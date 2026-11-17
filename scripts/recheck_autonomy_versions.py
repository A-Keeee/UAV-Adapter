#!/usr/bin/env python3
"""Re-fly accepted configurations whose evidence predates delivered software.

One finite pass, no automatic retry: a failed new flight stops the validation.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

from autonomy_provenance import ROOT
from owned_processes import stop_group
from trial_index import latest_configurations
from audit_autonomy_release import current_os1_replay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix',required=True,type=Path)
    parser.add_argument('--faults',required=True,type=Path)
    parser.add_argument('--interfaces',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    child=None
    def interrupted(number,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    def run(command):
        nonlocal child
        child=subprocess.Popen(command,cwd=ROOT,start_new_session=True)
        code=child.wait();child=None
        return code
    audit=[sys.executable,str(ROOT/'scripts/audit_autonomy_release.py'),'--matrix',str(args.matrix),
        '--faults',str(args.faults),'--interfaces',str(args.interfaces),'--output',str(args.output/'audit.json')]
    try:
        if run([sys.executable,str(ROOT/'scripts/rescore_autonomy_matrix.py'),str(args.matrix),'--require-complete']):
            raise RuntimeError('Finish the normal matrix before revalidating software versions')
        recording_code=run([sys.executable,str(ROOT/'scripts/verify_closed_recording.py'),'--matrix',str(args.matrix),
                '--faults',str(args.faults),'--reuse-verified','--keep-going','--output',str(args.output/'recordings.json')])
        if recording_code not in (0,1):raise RuntimeError('Recording verifier did not finish')
        replay_root=ROOT/'log/os1_channel_replay_results';replay_root.mkdir(exist_ok=True)
        if not current_os1_replay(replay_root/'latest'):
            output=replay_root/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
            command=[sys.executable,str(ROOT/'scripts/cloud_interface_fixture.py'),'--output',str(output),
                '--case','os1_sparse_replay','--os1-snapshot',str(ROOT/'log/os1_channel_replay_20261112_verified/sparse.msg')]
            if run(command):raise RuntimeError('Current-version OS1 channel replay failed')
            temporary=replay_root/('.latest_'+str(os.getpid()));temporary.symlink_to(output.resolve());os.replace(temporary,replay_root/'latest')
        code=run(audit)
        if code not in (0,1) or not (args.output/'audit.json').exists():raise RuntimeError('Release audit failed to produce evidence')
        report=json.loads((args.output/'audit.json').read_text())
        records=latest_configurations(json.loads((args.matrix/'verified_results.json').read_text()))
        rechecked=[]
        for key in sorted(records):
            label='/'.join(map(str,key))
            reasons=[issue for issue in report['issues'] if issue.startswith(label+': ')]
            if not reasons:continue
            if shutil.disk_usage(ROOT).free<30*2**30:raise RuntimeError('Less than 30 GiB remains for a new complete recording')
            print(json.dumps(dict(rechecking=label,reasons=reasons)),flush=True)
            task,lidar,condition=key
            command=[sys.executable,str(ROOT/'scripts/run_autonomy_matrix.py'),'--output',str(args.matrix),
                '--resume','--recheck-passed','--task',task,'--lidar',lidar,'--condition-index',str(condition),'--wall-timeout','4000']
            if run(command):raise RuntimeError('Current-version physical recheck failed: '+label)
            updated=latest_configurations(json.loads((args.matrix/'verified_results.json').read_text()))[key]
            bag=Path(updated['trial_directory'])/'mission/bag'
            if run([sys.executable,str(ROOT/'scripts/verify_closed_recording.py'),str(bag),'--require-flight','--reuse-verified']):
                raise RuntimeError('New physical recheck recording is not replayable: '+label)
            rechecked.append(label)
            (args.output/'rechecked.json').write_text(json.dumps(rechecked,indent=2)+'\n')
        if run([sys.executable,str(ROOT/'scripts/verify_closed_recording.py'),'--matrix',str(args.matrix),
                '--faults',str(args.faults),'--reuse-verified','--output',str(args.output/'recordings.json')]):
            raise RuntimeError('Final current recordings are not replayable')
        if run(audit):raise RuntimeError('Delivered implementation still lacks complete acceptance evidence')
    finally:
        if child is not None:stop_group(child,timeout=300)


if __name__=='__main__':main()
