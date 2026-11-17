#!/usr/bin/env python3
"""Run a finite, durable simulation queue; stop on failure, retain every trial."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from owned_processes import stop_group

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs',required=True,type=Path)
    parser.add_argument('--detach',action='store_true')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--stop-before',type=int,help='Stop before this zero-based job index; never start that job.')
    args=parser.parse_args();jobs_path=args.jobs.resolve();folder=jobs_path.parent
    state_path=folder/'state.json';log_path=folder/'runner.log'
    if args.detach:
        command=[sys.executable,str(Path(__file__).resolve()),'--jobs',str(jobs_path)]
        if args.resume:command.append('--resume')
        if args.stop_before is not None:command+=['--stop-before',str(args.stop_before)]
        with log_path.open('a') as log:
            child=subprocess.Popen(command,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,
                stderr=subprocess.STDOUT,start_new_session=True)
        print(json.dumps(dict(pid=child.pid,state_file=str(state_path),log_file=str(log_path))))
        return
    lock=(folder/'runner.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    content=jobs_path.read_bytes();jobs=json.loads(content);digest=hashlib.sha256(content).hexdigest()
    if not isinstance(jobs,list) or not 0<len(jobs)<=50:raise RuntimeError('Expected a finite queue of 1–50 jobs')
    if args.stop_before is not None and not 0<=args.stop_before<=len(jobs):
        parser.error('--stop-before must be between zero and the job count')
    old=json.loads(state_path.read_text()) if state_path.exists() else {}
    if old and not args.resume:raise RuntimeError('Queue already has state; use --resume')
    if old and old.get('jobs_sha256')!=digest:raise RuntimeError('Queue changed; use a separate output folder')
    start=old.get('completed_jobs',0);child=None
    state=dict(pid=os.getpid(),jobs_sha256=digest,total_jobs=len(jobs),completed_jobs=start,status='starting')
    def save(**values):
        state.update(values);temporary=state_path.with_suffix('.json.partial')
        temporary.write_text(json.dumps(state,indent=2)+'\n');os.replace(temporary,state_path)
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGHUP,signal.SIG_IGN)
    try:
        for index in range(start,len(jobs)):
            if args.stop_before is not None and index>=args.stop_before:
                save(status='stopped_at_limit',child_pid=None)
                return 0
            if (folder/'stop_after_current').exists():
                save(status='stopped_after_current',child_pid=None)
                return 0
            job=jobs[index];command=job['command']
            minimum=float(job.get('min_free_gib',30))
            if shutil.disk_usage(ROOT).free<minimum*2**30:
                save(status='disk_limit',job_index=index,job_name=job['name'],minimum_free_gib=minimum);return 1
            print('START '+json.dumps(job,ensure_ascii=False),flush=True)
            if job.get('log_file'):
                path=Path(job['log_file'])
                if path.exists():
                    attempt=1
                    while path.with_name(path.name+f'.attempt{attempt}').exists():attempt+=1
                    path.rename(path.with_name(path.name+f'.attempt{attempt}'))
                output=path.open('w')
            else:output=None
            try:
                child=subprocess.Popen(command,cwd=ROOT,start_new_session=True,stdin=subprocess.DEVNULL,
                    stdout=output,stderr=subprocess.STDOUT if output else None)
                save(status='running',job_index=index,job_name=job['name'],command=command,child_pid=child.pid,
                    job_log=job.get('log_file'))
                code=child.wait();child=None
            finally:
                if output:output.close()
            print('FINISH '+str(code),flush=True)
            if code:
                save(status='failed',return_code=code,child_pid=None);return 1
            save(completed_jobs=index+1,return_code=0,child_pid=None)
        save(status='complete')
        return 0
    except KeyboardInterrupt:
        save(status='stopping')
        return 130
    except BaseException as error:
        save(status='failed',error=str(error))
        raise
    finally:
        if child is not None:
            stop_group(child,timeout=300)
            save(status='stopped',child_pid=None)


if __name__=='__main__':
    sys.exit(main() or 0)
