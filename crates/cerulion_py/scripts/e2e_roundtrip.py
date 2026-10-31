"""e2e oracle for a built wheel: Rust fixture -> Python, then Python -> Rust fixture.

Prints one deterministic line per frame; the caller diffs stdout against a
literal expected block. Run from a clean cwd (e.g. /tmp) so `cerulion`
resolves to the INSTALLED wheel, not the source tree.
"""

import os
import selectors
import subprocess

import cerulion

FIXTURE = os.environ["CERULION_PY_FIXTURE"]
SCHEMA_HASH = 305419896  # 0x12345678 - decimal, matches the fixture's output format


def pattern(size, i):
    return bytes(((k * 7 + 3 + i * 11) & 0xFF) for k in range(size))


def fnv1a64(data):
    h = 0xCBF29CE484222325
    for b in data:
        h = ((h ^ b) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def _stderr(proc):
    try:
        return proc.stderr.read()
    except Exception:
        return "<unreadable stderr>"


def kill_reap(proc):
    """Kill the child and reap it so no zombie survives a failure path."""
    try:
        proc.kill()
    except ProcessLookupError:
        pass
    proc.wait()


def wait_ready(proc, timeout=15):
    """Read the fixture's READY line with a deadline; die loudly on timeout/EOF."""
    sel = selectors.DefaultSelector()
    sel.register(proc.stdout, selectors.EVENT_READ)
    try:
        if not sel.select(timeout):
            kill_reap(proc)
            raise RuntimeError(
                f"fixture did not print READY within {timeout}s: {_stderr(proc)}"
            )
        line = proc.stdout.readline().strip()
        if line != "READY":
            kill_reap(proc)
            raise RuntimeError(f"expected READY, got {line!r}: {_stderr(proc)}")
    finally:
        sel.close()


def finish(proc, timeout=30):
    """Wait for the fixture to exit; nonzero exit or timeout fails with stderr."""
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        kill_reap(proc)
        raise RuntimeError(
            f"fixture did not exit within {timeout}s: {_stderr(proc)}"
        ) from None
    if proc.returncode != 0:
        raise RuntimeError(
            f"fixture exited with {proc.returncode}:\n"
            f"stderr: {_stderr(proc)}\nstdout: {proc.stdout.read()}"
        )


def main():
    topic = f"/cerulion_py/e2e/{os.getpid()}"
    session = cerulion.connect()
    procs = []  # still-live children, killed+reaped on any exit path
    spawned = []  # every child ever started - pipes closed in `finally`
    try:
        # Rust -> Python: fixture publishes 3 x 64 B frames.
        sub = session.subscriber(topic, depth=4)
        proc = subprocess.Popen(
            [
                FIXTURE,
                "publish",
                "--topic",
                topic,
                "--count",
                "3",
                "--size",
                "64",
                "--schema-hash",
                str(SCHEMA_HASH),
                "--timestamp-ns",
                "1000",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        procs.append(proc)
        spawned.append(proc)
        for _ in range(3):
            frame = sub.receive(10000)
            if frame is None:
                # Kill+reap BEFORE reading stderr: `stderr.read()` on a
                # live child blocks for EOF and would outlive the timeout.
                kill_reap(proc)
                raise RuntimeError(
                    f"timed out waiting for fixture frame: {_stderr(proc)}"
                )
            body = frame.to_bytes()
            print(
                f"frame seq={frame.sequence} schema_hash={frame.schema_hash} "
                f"timestamp_ns={frame.timestamp_ns} total_size={frame.total_size} "
                f"payload_len={len(body)} fnv1a64={fnv1a64(body):016x}"
            )
            frame.release()
        finish(proc)
        procs.remove(proc)

        # Python -> Rust: fixture subscribes to one 64 B frame.
        proc = subprocess.Popen(
            [FIXTURE, "subscribe", "--topic", topic, "--count", "1", "--timeout-ms", "30000"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        procs.append(proc)
        spawned.append(proc)
        wait_ready(proc)
        pub = session.publisher(topic, SCHEMA_HASH, max_payload_len=64)
        pub.publish(pattern(64, 0), timestamp_ns=77)
        sel = selectors.DefaultSelector()
        sel.register(proc.stdout, selectors.EVENT_READ)
        try:
            if not sel.select(30):
                kill_reap(proc)
                raise RuntimeError(
                    f"fixture printed no frame line within 30s: {_stderr(proc)}"
                )
            line = proc.stdout.readline().strip()
        finally:
            sel.close()
        finish(proc)
        procs.remove(proc)
        print("fixture: " + line)
    finally:
        for proc in procs:
            kill_reap(proc)
        for proc in spawned:
            for pipe in (proc.stdout, proc.stderr):
                if pipe is not None:
                    pipe.close()


if __name__ == "__main__":
    main()
