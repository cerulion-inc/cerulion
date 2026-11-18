# SPDX-License-Identifier: AGPL-3.0-only
"""Sensing off the answer path: a robot-side check that only gathers evidence runs on
a worker thread while the operator thinks, and its finding is read when the next request arrives.

    start(job, fn)      returns at once. job is a plain dict {call, what, ...}; fn(job) -> a verdict word. One worker
                        thread per job; a job started while another runs waits behind it (one request at a time).
    join(max_s=3.0)     [{call, what, verdict ("in" | "out" | "not known"), seconds}] of every job started since the
                        last join, in the order they were started, waiting at most max_s IN ALL. A job that is still
                        running then is returned as "not known" with timeout true and its late answer is dropped: it
                        never appears in a later join. A job whose fn raised is "not known" with error true.
    stats()             {checks, seconds_total, seconds_max, timeouts, errors}

The worker never touches an executor, a session, a simulator or a journal: fn gets the job's plain values and
returns a word. Findings are only ever read at a join, so a session sees them at the start of a request and
nowhere else: two runs with the same answers from fn give the same findings on the same calls.

Standard library only (threading, time). Sensor is one session's sensing; the module functions use one default
Sensor for a caller that has a single session.
"""
import threading
import time

VERDICTS = ("in", "out", "not known")
UNKNOWN = "not known"
MAX_S = 3.0


class _Job:
    def __init__(self, job, fn, behind):
        self.job, self.fn, self.behind = dict(job or {}), fn, behind
        self.done = threading.Event()        # fn returned or raised
        self.released = threading.Event()    # the next job may run: this one is done, or was given up at a join
        self.verdict, self.error, self.given_up = None, False, False
        self.t0 = self.t1 = None             # when fn started and ended (monotonic seconds)
        self.asked = time.monotonic()
        self.thread = None

    def run(self):
        if self.behind is not None:
            self.behind.released.wait()
        self.t0 = time.monotonic()
        try:
            verdict = self.fn(self.job)
            self.verdict = verdict if verdict in VERDICTS else UNKNOWN
        except Exception:                    # a job that fails is a finding that says nothing, never an error out
            self.verdict, self.error = UNKNOWN, True
        self.t1 = time.monotonic()
        self.done.set()
        self.released.set()


class Sensor:
    """One session's off-path sensing (module text)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._open = []                      # started, not yet returned by a join
        self._last = None
        self._stats = {"checks": 0, "seconds_total": 0.0, "seconds_max": 0.0, "timeouts": 0, "errors": 0}

    def start(self, job, fn):
        """Start one job; returns at once (the job's number among the open ones)."""
        with self._lock:
            j = _Job(job, fn, self._last)
            self._last = j
            self._open.append(j)
            n = len(self._open)
        j.thread = threading.Thread(target=j.run, name="assist_sense", daemon=True)
        j.thread.start()
        return n

    def pending(self):
        with self._lock:
            return len(self._open)

    def join(self, max_s=MAX_S):
        """The findings of the jobs started since the last join, in start order (module text)."""
        with self._lock:
            jobs, self._open = self._open, []
        if not jobs:
            return []
        deadline = time.monotonic() + max(0.0, float(max_s))
        out = []
        for j in jobs:
            left = deadline - time.monotonic()
            finished = j.done.wait(left) if left > 0 else j.done.is_set()
            now = time.monotonic()
            row = {"call": j.job.get("call"), "what": j.job.get("what")}
            if finished:
                row["verdict"] = j.verdict
                seconds = max(0.0, (j.t1 or now) - (j.t0 or now))
                if j.error:
                    row["error"] = True
            else:
                j.given_up = True            # its late answer is dropped: this job is in no later join
                j.released.set()             # a job behind it need not wait for it any more
                row["verdict"], row["timeout"] = UNKNOWN, True
                seconds = max(0.0, now - (j.t0 if j.t0 is not None else j.asked))
            row["seconds"] = round(seconds, 3)
            out.append(row)
            st = self._stats
            st["checks"] += 1
            st["seconds_total"] += seconds
            st["seconds_max"] = max(st["seconds_max"], seconds)
            st["timeouts"] += int(not finished)
            st["errors"] += int(bool(j.error) and finished)
        return out

    def stats(self):
        st = self._stats
        return {"checks": st["checks"], "seconds_total": round(st["seconds_total"], 3),
                "seconds_max": round(st["seconds_max"], 3), "timeouts": st["timeouts"], "errors": st["errors"]}


_DEFAULT = Sensor()


def start(job, fn):
    return _DEFAULT.start(job, fn)


def join(max_s=MAX_S):
    return _DEFAULT.join(max_s)


def stats():
    return _DEFAULT.stats()


def reset():
    """A fresh default Sensor (tests; a process that runs one session after another)."""
    global _DEFAULT
    _DEFAULT = Sensor()
    return _DEFAULT


__all__ = ["MAX_S", "Sensor", "UNKNOWN", "VERDICTS", "join", "reset", "start", "stats"]
