import os
import re
import struct
import subprocess

import pytest

import cerulion

FIXTURE_ENV = "CERULION_PY_FIXTURE"
DEFAULT_FIXTURE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "target", "release", "cerulion_py_fixture")
)


def unique_topic(name):
    return f"/cerulion_py/{name}/{os.getpid()}"


def pattern(size, i):
    """Body byte formula shared with the Rust fixture: byte k = (k*7 + 3 + i*11) & 0xFF."""
    return bytes(((k * 7 + 3 + i * 11) & 0xFF) for k in range(size))


def fnv1a64(data):
    h = 0xCBF29CE484222325
    for b in data:
        h = ((h ^ b) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def shm_mappings():
    """[start, end) ranges of /dev/shm/iox2_ mappings (data segments, not [anon shmem])."""
    ranges = []
    with open("/proc/self/maps") as f:
        for line in f:
            if "iox2_" not in line:
                continue
            match = re.match(r"([0-9a-f]+)-([0-9a-f]+)", line)
            if match:
                ranges.append((int(match.group(1), 16), int(match.group(2), 16)))
    return ranges


def expected_header_bytes(schema_hash, total_size, sequence, timestamp_ns):
    """32-byte wire header: <QIIIIQ. Offset table offset/count are zero for raw frames."""
    return struct.pack("<QIIIIQ", schema_hash, total_size, 0, 0, sequence, timestamp_ns)


def spawn_fixture(fixture_bin, args):
    return subprocess.Popen(
        [fixture_bin, *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def wait_ready(proc, timeout=15):
    """Read the fixture's READY line. Fails the test on death or timeout instead of hanging."""
    import selectors

    sel = selectors.DefaultSelector()
    sel.register(proc.stdout, selectors.EVENT_READ)
    try:
        if not sel.select(timeout):
            proc.kill()
            pytest.fail(f"fixture did not print READY within {timeout}s: {proc.stderr.read()}")
        line = proc.stdout.readline().strip()
        if line != "READY":
            proc.kill()
            pytest.fail(f"expected READY, got {line!r}: {proc.stderr.read()}")
    finally:
        sel.close()


def finish_proc(proc, timeout=30):
    """Wait for the fixture to exit cleanly; forward stderr on failure."""
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        pytest.fail(f"fixture did not exit within {timeout}s: {proc.stderr.read()}")
    if proc.returncode != 0:
        pytest.fail(
            f"fixture exited with {proc.returncode}:\nstderr: {proc.stderr.read()}\n"
            f"stdout: {proc.stdout.read()}"
        )


@pytest.fixture(scope="session")
def session():
    return cerulion.connect()


@pytest.fixture(scope="session")
def fixture_bin():
    env = os.environ.get(FIXTURE_ENV)
    if env:
        if not os.path.isfile(env):
            pytest.fail(
                f"{FIXTURE_ENV}={env} does not exist. Build it with: "
                "cargo build --release -p cerulion_py_fixtures (inside crates/cerulion_py/)"
            )
        return os.path.abspath(env)
    if os.path.isfile(DEFAULT_FIXTURE):
        return DEFAULT_FIXTURE
    pytest.skip(
        f"cerulion_py_fixture not found at {DEFAULT_FIXTURE} and {FIXTURE_ENV} unset. "
        "Build it with: cargo build --release -p cerulion_py_fixtures"
    )
