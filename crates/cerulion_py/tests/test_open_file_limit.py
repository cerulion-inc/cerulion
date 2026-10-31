"""``cerulion.connect()`` keeps the soft open-file limit at or below 1024.

Each test starts a child interpreter whose soft ``RLIMIT_NOFILE`` is set
before ``exec``, connects there, and reads the limits back. The expected
soft limit is derived here from the hard limit: at most 1024
(``FD_SETSIZE``), the number ``select.select`` refuses, on every platform.
"""

import os
import resource
import subprocess
import sys

import pytest

FD_SETSIZE = 1024

CHILD = """
import resource
import cerulion
start = resource.getrlimit(resource.RLIMIT_NOFILE)[0]
cerulion.connect()
soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
print(f"NOFILE start={start} after={soft} hard={hard}")
"""


def _hard():
    return resource.getrlimit(resource.RLIMIT_NOFILE)[1]


def _run_child(soft):
    hard = _hard()

    def lower():
        resource.setrlimit(resource.RLIMIT_NOFILE, (soft, hard))

    out = subprocess.run(
        [sys.executable, "-c", CHILD],
        preexec_fn=lower,
        capture_output=True,
        text=True,
        timeout=60,
        env=dict(os.environ),
    )
    assert out.returncode == 0, f"child failed:\n{out.stdout}\n{out.stderr}"
    line = next(l for l in out.stdout.splitlines() if l.startswith("NOFILE "))
    fields = dict(kv.split("=") for kv in line.split()[1:])
    return int(fields["start"]), int(fields["after"]), int(fields["hard"])


def _finite(limit):
    return sys.maxsize if limit == resource.RLIM_INFINITY else limit


def test_connect_raises_a_low_soft_limit_no_higher_than_fd_setsize():
    hard = _finite(_hard())
    if hard <= 256:
        pytest.skip(f"the hard limit {hard} leaves nothing to raise")
    start, after, child_hard = _run_child(256)
    assert start == 256
    assert _finite(child_hard) == hard
    assert after == min(hard, FD_SETSIZE)


def test_connect_keeps_a_soft_limit_already_above_fd_setsize():
    hard = _finite(_hard())
    if hard < 4096:
        pytest.skip(f"the hard limit {hard} is below 4096")
    start, after, _ = _run_child(4096)
    assert (start, after) == (4096, 4096)
