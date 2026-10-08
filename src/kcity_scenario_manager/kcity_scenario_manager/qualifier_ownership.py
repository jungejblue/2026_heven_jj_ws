"""Local cooperative ownership leases; ROS/CARLA audits remain mandatory.

Locks are per host user and simulator endpoint, independent of ROS_DOMAIN_ID:
two DDS domains must not independently own the same CARLA world.
"""
import fcntl
import os
from pathlib import Path


def acquire(role):
    directory = Path(f"/tmp/heven-qualifier-{os.getuid()}")
    directory.mkdir(mode=0o700, exist_ok=True)
    stream = (directory / f"localhost-2000-{role}.lock").open("a+")
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        raise RuntimeError(f"Qualifier {role} owner already running; stop that terminal first")
    return stream
