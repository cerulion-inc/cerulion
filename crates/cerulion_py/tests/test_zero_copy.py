"""Zero-copy invariants: no copies on receive, read-only views, SHM pointers, bounded RSS."""

import os
import re
import resource
import sys

import numpy as np
import pytest

from conftest import pattern, shm_mappings, unique_topic

SIZE = 4 << 20
ITERATIONS = 1000
RSS_BUDGET = 64 << 20


LINUX = sys.platform.startswith("linux")


def vmrss_bytes():
    """Current RSS on Linux; peak RSS (ru_maxrss, bytes) on macOS.

    Peak is the stricter bound for a growth budget: it can only rise."""
    if not LINUX:
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    pytest.fail("VmRSS not found in /proc/self/status")


def test_zero_copy_receive_1000_frames(session):
    topic = unique_topic("zc1k")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 0xAAAA, max_payload_len=SIZE)
    patterns = [pattern(SIZE, i) for i in range(4)]
    expected = [np.frombuffer(p, dtype=np.uint8) for p in patterns]
    rss0 = vmrss_bytes()
    shm_ptrs = 0
    ranges = None
    for i in range(ITERATIONS):
        pub.publish(patterns[i % 4], timestamp_ns=i)
        frame = sub.receive(5000)
        assert frame is not None, f"iteration {i}"
        a = frame.as_numpy()
        ptr = a.__array_interface__["data"][0]
        assert not a.flags.writeable
        if not LINUX:
            shm_ptrs += 1  # no /proc/self/maps; the RSS budget carries the check
        elif ranges is None:
            ranges = shm_mappings()
            assert ranges, "no iox2_ mapping in /proc/self/maps"
        if LINUX and not any(start <= ptr < end for start, end in ranges):
            ranges = shm_mappings()  # a new segment can be mapped later; refresh once
        if LINUX and any(start <= ptr < end for start, end in ranges):
            shm_ptrs += 1
        if i == 0 or i % 100 == 0:
            assert np.array_equal(a, expected[i % 4]), f"iteration {i}"
        else:
            assert a[0] == expected[i % 4][0]
            assert a[SIZE // 2] == expected[i % 4][SIZE // 2]
            assert a[-1] == expected[i % 4][-1]
        frame.release()
    assert shm_ptrs == ITERATIONS, "frame memory not inside an iox2_ SHM mapping"
    delta = vmrss_bytes() - rss0
    assert delta < RSS_BUDGET, f"RSS grew {delta / (1 << 20):.0f} MiB - copies on receive?"


def test_frame_views_readonly(session):
    topic = unique_topic("zcro")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"abcd")
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.raw.readonly is True
    a = frame.as_numpy()
    assert a.flags.writeable is False
    with pytest.raises(ValueError):
        a[0] = 1
    with pytest.raises(TypeError):
        frame.raw[0] = 0
    frame.release()
