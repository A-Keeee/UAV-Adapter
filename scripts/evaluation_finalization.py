"""Protect independent scoring artifacts during graceful process shutdown."""
from contextlib import contextmanager
import json
import os
import signal


def atomic_json(path, value):
    temporary = path.with_name(path.name+'.partial')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


@contextmanager
def protected_finalization():
    # The supervisor retains a bounded SIGKILL deadline for a stuck evaluator.
    previous = {number: signal.signal(number, signal.SIG_IGN)
                for number in (signal.SIGTERM, signal.SIGINT)}
    try:
        yield
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)
