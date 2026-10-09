import platform
import subprocess
import time

import numpy as np
import pytest

import cerulion

from conftest import finish_proc, pattern, shm_mappings, spawn_fixture, unique_topic, wait_ready


SCHEMA_HASH = "5798738998627362816"

# The two /proc probes read Linux-only files; the borrow-floor probe needs
# only the fixture and a subscriber, so it runs on every platform.
linux_only = pytest.mark.skipif(platform.system() != "Linux", reason="requires Linux SHM mappings")


def _du_bytes(*options):
    """Best-effort `du` total over /dev/shm, or None when du cannot report one.

    The value is printed, never asserted, so a missing du or an entry it
    cannot read must not fail the probe.
    """
    try:
        result = subprocess.run(
            ["du", "-s", "--block-size=1", *options, "/dev/shm"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
    except OSError:
        return None
    fields = result.stdout.split()
    return int(fields[0]) if fields and fields[0].isdigit() else None


def _metric_snapshot():
    values = {}
    with open("/proc/self/smaps_rollup", encoding="ascii") as stream:
        for line in stream:
            key, _, value = line.partition(":")
            if key in {"Rss", "Pss", "Shared_Clean", "Shared_Dirty",
                       "Private_Clean", "Private_Dirty"}:
                values[key] = int(value.strip().split()[0]) * 1024
    with open("/proc/self/status", encoding="ascii") as stream:
        for line in stream:
            key, _, value = line.partition(":")
            if key in {"VmRSS", "VmHWM"}:
                values[key] = int(value.strip().split()[0]) * 1024
    values["shm_apparent"] = _du_bytes("--apparent-size")
    values["shm_resident"] = _du_bytes()
    return values


def _spawn_publish_hold(fixture_bin, topic, count, size, *extra):
    """Start `publish-hold`: it publishes nothing until a batch is requested."""
    return spawn_fixture(
        fixture_bin,
        ["publish-hold", "--topic", topic, "--schema-hash", SCHEMA_HASH,
         "--count", str(count), "--size", str(size), "--linger-ms", "500", *extra],
        stdin=subprocess.PIPE,
    )


def _request_frames(proc, count):
    """Ask the fixture for `count` more frames.

    The caller keeps `count` within the room left in its queue, so the batch
    can never overflow it however slowly the frames are received.
    """
    proc.stdin.write(f"{count}\n")
    proc.stdin.flush()


def _finish_publish_hold(proc):
    """End the fixture's input and require a clean exit."""
    proc.stdin.close()
    finish_proc(proc)


def _receive_frames(subscriber, count):
    frames = []
    deadline = time.monotonic() + 15
    while len(frames) < count and time.monotonic() < deadline:
        frame = subscriber.receive(100)
        if frame is not None:
            frames.append(frame)
    assert len(frames) == count
    return frames


def _print_probe(mode, n, size, metrics):
    keys = (
        "Rss", "Pss", "Shared_Clean", "Shared_Dirty", "Private_Clean",
        "Private_Dirty", "VmRSS", "VmHWM", "shm_apparent", "shm_resident",
    )
    print(
        f"MEMORY_PROBE mode={mode} N={n} P={size} "
        + " ".join(f"{key}={metrics[key]}" for key in keys),
        flush=True,
    )


@linux_only
def test_to_bytes_private_copy_positive_control(fixture_bin, session):
    n, size = 16, 1 << 20
    topic = unique_topic("memory-hold")
    proc = _spawn_publish_hold(fixture_bin, topic, n + 2, size, "--borrow-floor", str(n + 1))
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=n)
        before = _metric_snapshot()
        _request_frames(proc, n)
        copies = []
        for frame in _receive_frames(subscriber, n):
            copies.append(frame.to_bytes())
            frame.release()
        after = _metric_snapshot()
        assert all(value == pattern(size, sequence)
                   for sequence, value in enumerate(copies))
        private_growth = (
            after["Private_Clean"] + after["Private_Dirty"]
            - before["Private_Clean"] - before["Private_Dirty"]
        )
        assert private_growth >= int(0.9 * n * size)
        _print_probe("bytes", n, size, after)
        _request_frames(proc, 2)
        remaining = _receive_frames(subscriber, 2)
        assert [frame.sequence for frame in remaining] == [n, n + 1]
        for frame in remaining:
            frame.release()
    finally:
        _finish_publish_hold(proc)


@linux_only
def test_held_frames_stay_in_publisher_shm(fixture_bin, session):
    n, size = 16, 1 << 20
    topic = unique_topic("memory-view")
    proc = _spawn_publish_hold(fixture_bin, topic, n + 2, size, "--borrow-floor", str(n + 1))
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=n)
        before = _metric_snapshot()
        _request_frames(proc, n)
        frames = _receive_frames(subscriber, n)
        arrays = [frame.as_numpy() for frame in frames]
        addresses = [int(array.__array_interface__["data"][0]) for array in arrays]
        ranges = shm_mappings()
        assert all(any(start <= address < end for start, end in ranges)
                   for address in addresses)
        assert len(set(addresses)) == n
        assert min(abs(left - right) for left in addresses for right in addresses
                   if left != right) >= size + 32
        after = _metric_snapshot()
        private_growth = (
            after["Private_Clean"] + after["Private_Dirty"]
            - before["Private_Clean"] - before["Private_Dirty"]
        )
        assert private_growth < n * size // 8
        _print_probe("view", n, size, after)
        for sequence, (frame, array) in enumerate(zip(frames, arrays)):
            assert frame.sequence == sequence
            np.testing.assert_array_equal(
                array, np.frombuffer(pattern(size, sequence), dtype=np.uint8)
            )
        del array
        del arrays
        for frame in frames:
            frame.release()
        del frame
        del frames
        _request_frames(proc, 2)
        remaining = _receive_frames(subscriber, 2)
        assert [frame.sequence for frame in remaining] == [n, n + 1]
        for frame in remaining:
            frame.release()
    finally:
        _finish_publish_hold(proc)


def test_default_borrow_floor_rejects_third_held_frame(fixture_bin, session):
    topic = unique_topic("memory-floor")
    # No --borrow-floor: the topic is created at the transport default.
    proc = _spawn_publish_hold(fixture_bin, topic, 4, 256)
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=4)
        assert subscriber.max_borrowed_samples == 2
        _request_frames(proc, 4)
        frames = _receive_frames(subscriber, 2)
        with pytest.raises(cerulion.BorrowLimitExceeded):
            _receive_frames(subscriber, 1)
        for frame in frames:
            frame.release()
        remaining = _receive_frames(subscriber, 2)
        assert [frame.sequence for frame in remaining] == [2, 3]
        for frame in remaining:
            frame.release()
    finally:
        _finish_publish_hold(proc)
