import os
import platform
import re
import subprocess
import time

import numpy as np
import pytest

import cerulion

from conftest import pattern, spawn_fixture, unique_topic, wait_ready


pytestmark = pytest.mark.skipif(platform.system() != "Linux", reason="requires Linux SHM mappings")


def _shm_ranges():
    ranges = []
    with open("/proc/self/maps", encoding="ascii") as maps:
        for line in maps:
            if "iox2_" not in line:
                continue
            match = re.match(r"([0-9a-f]+)-([0-9a-f]+)", line)
            if match:
                ranges.append((int(match.group(1), 16), int(match.group(2), 16)))
    return ranges


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
    apparent = subprocess.check_output(
        ["du", "-s", "--block-size=1", "--apparent-size", "/dev/shm"],
        text=True,
    )
    resident = subprocess.check_output(
        ["du", "-s", "--block-size=1", "/dev/shm"], text=True
    )
    values["shm_apparent"] = int(apparent.split()[0])
    values["shm_resident"] = int(resident.split()[0])
    return values


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


def test_to_bytes_private_copy_positive_control(fixture_bin, session):
    n, size = 16, 1 << 20
    topic = unique_topic("memory-hold")
    proc = spawn_fixture(
        fixture_bin,
        ["publish-hold", "--topic", topic, "--schema-hash", "5798738998627362816",
         "--count", str(n + 2), "--size", str(size), "--borrow-floor", str(n + 1),
         "--linger-ms", "500"],
    )
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=n)
        before = _metric_snapshot()
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
        remaining = _receive_frames(subscriber, 2)
        assert [frame.sequence for frame in remaining] == [n, n + 1]
        for frame in remaining:
            frame.release()
    finally:
        proc.kill()
        proc.wait()
        if proc.returncode not in (0, -9):
            pytest.fail(proc.stderr.read())


def test_held_frames_stay_in_publisher_shm(fixture_bin, session):
    n, size = 16, 1 << 20
    topic = unique_topic("memory-view")
    proc = spawn_fixture(
        fixture_bin,
        ["publish-hold", "--topic", topic, "--schema-hash", "5798738998627362816",
         "--count", str(n + 2), "--size", str(size), "--borrow-floor", str(n + 1),
         "--linger-ms", "500"],
    )
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=n)
        before = _metric_snapshot()
        frames = _receive_frames(subscriber, n)
        arrays = [frame.as_numpy() for frame in frames]
        addresses = [int(array.__array_interface__["data"][0]) for array in arrays]
        ranges = _shm_ranges()
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
        remaining = _receive_frames(subscriber, 2)
        assert [frame.sequence for frame in remaining] == [n, n + 1]
        for frame in remaining:
            frame.release()
    finally:
        proc.kill()
        proc.wait()
        if proc.returncode not in (0, -9):
            pytest.fail(proc.stderr.read())


def test_default_borrow_floor_rejects_third_held_frame(fixture_bin, session):
    topic = unique_topic("memory-floor")
    proc = spawn_fixture(
        fixture_bin,
        ["publish-hold", "--topic", topic, "--schema-hash", "5798738998627362816",
         "--count", "4", "--size", "256", "--borrow-floor", "2", "--linger-ms", "500"],
    )
    try:
        wait_ready(proc)
        subscriber = session.subscriber(topic, depth=4)
        frames = _receive_frames(subscriber, 2)
        with pytest.raises(cerulion.BorrowLimitExceeded):
            _receive_frames(subscriber, 1)
        for frame in frames:
            frame.release()
    finally:
        proc.kill()
        proc.wait()
        if proc.returncode not in (0, -9):
            pytest.fail(proc.stderr.read())
