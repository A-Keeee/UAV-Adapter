#!/usr/bin/env python3
"""Wait for one verified live finite queue; a stopped queue is never success."""
import argparse
import json
from pathlib import Path
import time

import psutil
from owned_processes import owner_alive


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state',required=True,type=Path)
    parser.add_argument('--wall-timeout',type=float,default=43200)
    args=parser.parse_args()
    state=json.loads(args.state.read_text())
    if state.get('status')=='complete':return
    pid=state['pid'];started=psutil.Process(pid).create_time()
    deadline=time.monotonic()+args.wall_timeout
    while time.monotonic()<deadline:
        state=json.loads(args.state.read_text())
        if state.get('status')=='complete':return
        if state.get('pid')!=pid or state.get('status') in ('failed','stopped','stopped_after_current','disk_limit'):
            raise RuntimeError('Preceding validation did not complete: '+json.dumps(state))
        if not owner_alive(pid,started):raise RuntimeError('Preceding queue owner exited without a completed result')
        time.sleep(.5)
    raise RuntimeError('Preceding validation exceeded the finite wait deadline')


if __name__=='__main__':main()
