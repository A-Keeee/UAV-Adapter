#!/usr/bin/env python3
"""Finish an explicitly requested scheduler handoff without stopping its flight."""
import argparse
import json
from pathlib import Path
import signal
import time

import psutil
from evaluation_finalization import atomic_json
from owned_processes import owner_alive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('handoff', type=Path)
    args = parser.parse_args()
    record = json.loads(args.handoff.read_text())
    while any(owner_alive(child['pid'], child['started']) for child in record['children']):
        time.sleep(.5)
    if owner_alive(record['queue_pid'], record['queue_started']):
        process = psutil.Process(record['queue_pid'])
        if process.status() != psutil.STATUS_STOPPED:
            raise RuntimeError('Scheduler is no longer stopped; refuse to touch a changed owner')
        # TERM remains pending while stopped; CONT delivers it before another
        # job can start. The completed trial's owned cleanup already finished.
        process.send_signal(signal.SIGTERM)
        process.send_signal(signal.SIGCONT)
        deadline = time.monotonic()+30
        while owner_alive(record['queue_pid'], record['queue_started']) and time.monotonic() < deadline:
            time.sleep(.1)
        if owner_alive(record['queue_pid'], record['queue_started']):
            raise RuntimeError('Scheduler has not exited; no further signals sent')
    record.update(status='complete', current_trial_finished=True, automatic_successors_disabled=True)
    atomic_json(args.handoff, record)
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
