# SPDX-License-Identifier: AGPL-3.0-only
"""The unix socket loop of one episode's RCP server (rcpx/server.py re-exports serve and the names here), and the
file names of an episode dir. Kept free of the session's imports so a client (rcpx/cli.py) can load it alone.

Single threaded, newline-delimited JSON: one request line in, one response line out. The loop exits when the
episode ends or on SIGTERM.
"""
import json
import os
import signal
import socket
import stat
import threading
import time

SOCK_NAME = "rcp.sock"            # EPDIR/rcp.sock
END_NAME = "episode_end.json"     # written when the episode ends: reason, claim, budget used
JOURNAL_NAME = "journal.jsonl"
FRAMES_DIR = "frames"
ENDED = {"episode": "ended", "message": "episode ended"}
MAX_SOCK_PATH = 103               # macOS sun_path is 104 bytes with the terminator
MAX_REQUEST_BYTES = 4 * 1024 * 1024
IDLE_TIMEOUT_S = 3 * 3600.0       # serve: no request for this long (wall clock) ends the episode as "abandoned"
IDLE_ENV = "RCPX_SERVE_IDLE_S"    # the harness option: seconds, or 0 for no bound


def _clear_stale(sock_path):
    try:
        mode = os.stat(sock_path).st_mode
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(mode):
        raise FileExistsError(f"{sock_path} exists and is not a socket")
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        probe.connect(sock_path)
    except OSError:
        os.unlink(sock_path)  # nobody answers: a leftover from a dead server
        return
    finally:
        probe.close()
    raise RuntimeError(f"a server already answers on {sock_path}")


def _send(conn, response, timeout_s):
    try:
        conn.settimeout(timeout_s)
        conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
        return True
    except OSError:
        return False  # the client went away; the journal already holds the response


def _serve_conn(session, conn, conn_timeout_s, poll_s, stop):
    buf, idle_s = b"", 0.0
    while True:
        try:
            conn.settimeout(poll_s)  # short waits so a SIGTERM is seen while a client is idle
            chunk = conn.recv(65536)
        except socket.timeout:
            idle_s += poll_s
            if stop["signal"] is not None or idle_s >= conn_timeout_s:
                return
            continue
        except OSError:  # reset: drop this client
            return
        idle_s = 0.0
        eof = not chunk
        buf += chunk
        if len(buf) > MAX_REQUEST_BYTES:
            _send(conn, session.refuse_request(f"the request is larger than {MAX_REQUEST_BYTES} bytes"),
                  conn_timeout_s)
            return
        if eof:
            lines, buf = buf.split(b"\n"), b""
        else:
            *lines, buf = buf.split(b"\n")
        for line in lines:
            if line.strip():
                stop["last"] = stop["clock"]()   # a request: the episode is not abandoned
            response = session.handle_text(line.decode("utf-8", "replace")) if line.strip() else None
            if response is not None and not _send(conn, response, conn_timeout_s):
                return
        if eof or session.ended or stop["signal"] is not None:
            return


def idle_bound_s(idle_timeout_s=None):
    """The idle bound in seconds, or None for no bound: the argument, else the env RCPX_SERVE_IDLE_S, else
    IDLE_TIMEOUT_S (3 hours). 0 or less means no bound."""
    if idle_timeout_s is None:
        try:
            idle_timeout_s = float(os.environ.get(IDLE_ENV, IDLE_TIMEOUT_S))
        except ValueError:
            idle_timeout_s = IDLE_TIMEOUT_S
    return float(idle_timeout_s) if float(idle_timeout_s) > 0 else None


def serve(session, sock_path, *, on_ready=None, conn_timeout_s=120.0, poll_s=0.2, idle_timeout_s=None,
          clock=time.monotonic):
    """Serve session on a unix socket until the episode ends, SIGTERM arrives or no request came for the idle
    bound (a harness option, wall clock by default: an abandoned episode must not keep its server forever; the
    episode then ends with reason "abandoned"); returns the end reason. clock: the idle bound's clock (tests)."""
    idle_s = idle_bound_s(idle_timeout_s)
    sock_path = os.path.abspath(sock_path)
    if len(os.fsencode(sock_path)) > MAX_SOCK_PATH:
        raise ValueError(f"socket path is {len(os.fsencode(sock_path))} bytes; unix sockets allow "
                         f"{MAX_SOCK_PATH}: use a shorter episode dir")
    _clear_stale(sock_path)
    stop = {"signal": None, "clock": clock, "last": clock()}
    abandoned = False
    installed, previous = False, None
    if threading.current_thread() is threading.main_thread():
        def on_term(signum, _frame):
            stop["signal"] = signum
        previous = signal.signal(signal.SIGTERM, on_term)
        installed = True
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        server.bind(sock_path)  # a failure here leaves the episode open: nothing was served
        server.listen(16)
    except OSError:
        server.close()
        if installed:
            signal.signal(signal.SIGTERM, previous if previous is not None else signal.SIG_DFL)
        raise
    try:
        server.settimeout(poll_s)  # the accept wait is how a SIGTERM is noticed between clients
        if on_ready is not None:
            on_ready()
        while not session.ended and stop["signal"] is None:
            if idle_s is not None and clock() - stop["last"] >= idle_s:
                abandoned = True
                break
            try:
                conn, _ = server.accept()
            except (socket.timeout, InterruptedError):
                continue
            with conn:
                _serve_conn(session, conn, conn_timeout_s, poll_s, stop)
    finally:
        server.close()
        try:
            if stat.S_ISSOCK(os.stat(sock_path).st_mode):
                os.unlink(sock_path)
        except FileNotFoundError:
            pass
        if abandoned and callable(getattr(session, "park_arm", None)):
            session.park_arm("abandoned")
        session.close("abandoned" if abandoned else "sigterm" if stop["signal"] is not None else "server stopped")
        if installed:
            signal.signal(signal.SIGTERM, previous if previous is not None else signal.SIG_DFL)
    return session.end_reason
