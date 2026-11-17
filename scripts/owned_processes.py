"""Stop an owned subprocess group even if its ros2 launcher already exited."""
import os
import signal
import subprocess
import time

import psutil


def owner_alive(pid, started):
    if not pid:
        return True
    try:
        process = psutil.Process(pid)
        return process.is_running() and process.status()!=psutil.STATUS_ZOMBIE and abs(process.create_time()-started)<.01
    except psutil.Error:
        return False


def signal_group(process, signum):
    # Every caller creates this child with start_new_session=True.
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def live_group(group):
    members = []
    for process in psutil.process_iter(['pid', 'status']):
        try:
            if process.info['status'] != psutil.STATUS_ZOMBIE and os.getpgid(process.pid) == group:
                members.append(process.pid)
        except (psutil.Error, ProcessLookupError):
            pass
    return members


def stop_group(process, timeout=4, *, already_signalled=False):
    # A second signal can interrupt a child's graceful data finalization.
    if not already_signalled:
        signal_group(process, signal.SIGTERM)
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        process.poll()  # reap a completed launcher, independently of its child
        if not live_group(process.pid):
            break
        time.sleep(.1)
    if live_group(process.pid):
        signal_group(process, signal.SIGKILL)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('Owned process did not exit after SIGKILL') from error
