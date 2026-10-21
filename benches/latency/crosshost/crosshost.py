#!/usr/bin/env python3
"""crosshost.py: wall-clock round-trip latency between two machines.

One machine LISTENS and echoes (`serve`); the other DIALS, sends the pings
and measures (`ping`). Both stamps of a round trip are read on the dialing
machine's CLOCK_MONOTONIC, so the clock offset between the two machines never
enters a sample.

Stacks (`--stack`):

  cerulion-tcp   `cerulion graph run` on each side; the dialer's graph has
  cerulion-quic  ping + latency, the listener's graph has pong, joined by a
                 Strict `network:` block over `tcp/` or `quic/`.
  fastdds-tcp    the ROS 2 ping/pong/latency nodes of ../ros2 on rmw_fastrtps,
                 with Fast DDS profiles that put every inter-machine byte on
                 TCPv4 connections the dialer's participants open.

The dialer opens every inter-machine connection, so the pair works on a
network that admits connections from one side only.

Subcommands:

  build      build the binaries a stack needs on this machine
  cert       mint a self-signed end-entity certificate for a quic/ listener
  serve      run the echo side for one payload size until stdin closes
  ping       run the measuring side for one payload size; writes a .bin
  pair       drive serve + ping on two machines over ssh for a size sweep
  summarize  print the min/p50/p99/max table for a directory of .bin files

Every .bin holds little-endian u64 round-trip samples in nanoseconds, the
same format as the rest of the latency suite (../compile_csv.py).
Python 3.8+, standard library only.
"""

import argparse
import fcntl
import json
import os
import re
import shlex
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LATENCY_DIR = HERE.parent
REPO_ROOT = LATENCY_DIR.parent.parent
WORKSPACE_DIR = LATENCY_DIR / "workspace"
ROS2_DIR = LATENCY_DIR / "ros2"

sys.path.insert(0, str(LATENCY_DIR))
from compile_csv import percentile  # noqa: E402  (the suite's one quantile)

STACKS = ("cerulion-tcp", "cerulion-quic", "fastdds-tcp")
CERULION_CARRIER = {"cerulion-tcp": "tcp", "cerulion-quic": "quic"}

# Topic names the two Cerulion graphs share. Both graphs use one prefix, so
# the ping graph produces /xhost/ping/ping_out and the pong graph produces
# /xhost/pong/echo_out.
PREFIX = "xhost"
PING_TOPIC = f"/{PREFIX}/ping/ping_out"
ECHO_TOPIC = f"/{PREFIX}/pong/echo_out"

# Rendered graphs land in the workspace's graphs/ directory, where
# `cerulion graph run <name>` looks them up. workspace/.gitignore ignores
# this file-name prefix.
GRAPH_STEM = "xhost_"

# The ROS 2 domain both machines join. The listener's Fast DDS profile has
# no UDP transport and the dialer's UDP transport is bound to loopback, so
# the domain only separates this pair from other participants on the
# dialer's own loopback.
ROS_DOMAIN_ID = 73

DEFAULT_RAW_DIR = HERE / "results"


# --------------------------------------------------------------------------
# rendering (pure functions, unit tested)
# --------------------------------------------------------------------------


def slice_len(size: int) -> int:
    """`max_slice_len` for the Image `data` field at one payload size."""
    return size + 65536


def render_ping_graph(carrier: str, peer: str, port: int, size: int,
                      root_ca: str = "") -> str:
    """The dialer's graph: ping + latency, egress ping, ingress echo."""
    tls = ""
    if carrier == "quic":
        if not root_ca:
            raise ValueError("a quic/ connect needs --root-ca")
        tls = f"  tls:\n    root_ca: {root_ca}\n"
    return (
        "# Rendered by benches/latency/crosshost/crosshost.py (dialing side).\n"
        f"prefix: {PREFIX}\n"
        "nodes:\n"
        "- id: ping\n"
        "  type: ping_node\n"
        "  inputs: []\n"
        "  outputs:\n"
        "  - name: ping_out\n"
        "    schema: sensor_msgs/Image\n"
        f"    max_slice_len: {slice_len(size)}\n"
        "    history_size: 0\n"
        "- id: latency\n"
        "  type: latency_node\n"
        "  inputs:\n"
        "  - name: echo_in\n"
        f"    source: {ECHO_TOPIC}\n"
        "  outputs: []\n"
        "network:\n"
        "  mode: peer\n"
        "  connect:\n"
        f"  - {carrier}/{peer}:{port}\n"
        "  egress:\n"
        f"  - {PING_TOPIC}\n"
        "  ingress:\n"
        f"  - {ECHO_TOPIC}\n"
        f"{tls}"
    )


def render_pong_graph(carrier: str, listen_ip: str, port: int, size: int,
                      cert: str = "", key: str = "") -> str:
    """The listener's graph: pong, ingress ping, egress echo."""
    tls = ""
    if carrier == "quic":
        if not (cert and key):
            raise ValueError("a quic/ listener needs a certificate and key")
        tls = f"  tls:\n    certificate: {cert}\n    private_key: {key}\n"
    return (
        "# Rendered by benches/latency/crosshost/crosshost.py (listening side).\n"
        f"prefix: {PREFIX}\n"
        "nodes:\n"
        "- id: pong\n"
        "  type: pong_node\n"
        "  inputs:\n"
        "  - name: ping_in\n"
        f"    source: {PING_TOPIC}\n"
        "  outputs:\n"
        "  - name: echo_out\n"
        "    schema: sensor_msgs/Image\n"
        f"    max_slice_len: {slice_len(size)}\n"
        "    history_size: 0\n"
        "network:\n"
        "  mode: peer\n"
        "  listen:\n"
        f"  - {carrier}/{listen_ip}:{port}\n"
        "  egress:\n"
        f"  - {ECHO_TOPIC}\n"
        "  ingress:\n"
        f"  - {PING_TOPIC}\n"
        f"{tls}"
    )


_FASTDDS_HEAD = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<!-- Rendered by benches/latency/crosshost/crosshost.py. -->\n'
    '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n'
)


def render_fastdds_listener(listen_ip: str, port: int) -> str:
    """Fast DDS profile for the listening machine.

    One TCPv4 transport that accepts on `listen_ip:port`, no built-in
    transports: the participant has no UDP or shared-memory locator, so a
    remote participant can reach it only through that TCP port, and the
    interface whitelist keeps every advertised locator on `listen_ip`.
    `enable_tcp_nodelay` sets TCP_NODELAY on the socket, so a small sample
    is sent at once instead of waiting for the previous segment's ACK.
    """
    return (
        _FASTDDS_HEAD
        + "  <transport_descriptors>\n"
        "    <transport_descriptor>\n"
        "      <transport_id>xhost_tcp</transport_id>\n"
        "      <type>TCPv4</type>\n"
        f"      <listening_ports><port>{port}</port></listening_ports>\n"
        "      <enable_tcp_nodelay>true</enable_tcp_nodelay>\n"
        f"      <interfaceWhiteList><address>{listen_ip}</address></interfaceWhiteList>\n"
        "    </transport_descriptor>\n"
        "  </transport_descriptors>\n"
        '  <participant profile_name="xhost_listener" is_default_profile="true">\n'
        "    <rtps>\n"
        "      <userTransports><transport_id>xhost_tcp</transport_id></userTransports>\n"
        "      <useBuiltinTransports>false</useBuiltinTransports>\n"
        "    </rtps>\n"
        "  </participant>\n"
        "</profiles>\n"
    )


def render_fastdds_dialer(peer: str, port: int) -> str:
    """Fast DDS profile for the dialing machine.

    A TCPv4 transport with no listening port, so this participant only
    opens connections, plus a UDPv4 transport bound to loopback for the
    ping and latency processes on this machine to find each other. The
    initial peers are the listener's TCP port and this machine's loopback.
    The listener has no UDP transport, so it ignores the loopback locators
    and answers on the TCP connection this side opened. The TCP socket
    sets TCP_NODELAY, as on the listener.
    """
    return (
        _FASTDDS_HEAD
        + "  <transport_descriptors>\n"
        "    <transport_descriptor>\n"
        "      <transport_id>xhost_tcp</transport_id>\n"
        "      <type>TCPv4</type>\n"
        "      <enable_tcp_nodelay>true</enable_tcp_nodelay>\n"
        "    </transport_descriptor>\n"
        "    <transport_descriptor>\n"
        "      <transport_id>xhost_loopback_udp</transport_id>\n"
        "      <type>UDPv4</type>\n"
        "      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>\n"
        "    </transport_descriptor>\n"
        "  </transport_descriptors>\n"
        '  <participant profile_name="xhost_dialer" is_default_profile="true">\n'
        "    <rtps>\n"
        "      <builtin>\n"
        "        <initialPeersList>\n"
        "          <locator>\n"
        "            <tcpv4>\n"
        f"              <address>{peer}</address>\n"
        f"              <physical_port>{port}</physical_port>\n"
        "            </tcpv4>\n"
        "          </locator>\n"
        "          <locator>\n"
        "            <udpv4><address>127.0.0.1</address></udpv4>\n"
        "          </locator>\n"
        "        </initialPeersList>\n"
        "      </builtin>\n"
        "      <userTransports>\n"
        "        <transport_id>xhost_tcp</transport_id>\n"
        "        <transport_id>xhost_loopback_udp</transport_id>\n"
        "      </userTransports>\n"
        "      <useBuiltinTransports>false</useBuiltinTransports>\n"
        "    </rtps>\n"
        "  </participant>\n"
        "</profiles>\n"
    )


def raw_name(stack: str) -> str:
    """The .bin file stem for a stack: `xhost_<stack>`."""
    return "xhost_" + stack.replace("-", "_")


def read_bin(path: Path) -> list:
    data = path.read_bytes()
    if len(data) % 8:
        raise ValueError(f"{path}: {len(data)} bytes is not a whole number of u64 samples")
    return list(struct.unpack(f"<{len(data) // 8}Q", data))


def summarize_samples(samples: list) -> dict:
    """min / p50 / p99 / max in nanoseconds, the suite's quantile rule."""
    s = sorted(samples)
    return {
        "n": len(s),
        "min": s[0],
        "p50": percentile(s, 0.50),
        "p99": percentile(s, 0.99),
        "max": s[-1],
    }


def parse_bin_name(name: str):
    """`xhost_cerulion_tcp_1024.bin` -> ("xhost_cerulion_tcp", 1024)."""
    if not name.endswith(".bin"):
        return None
    stem = name[:-4]
    head, sep, tail = stem.rpartition("_")
    if not sep or not tail.isdigit():
        return None
    return head, int(tail)


def fmt_us(ns: int) -> str:
    return f"{ns / 1000:.1f}"


ACCEPTED = "ACCEPTED"


def accepted_names(raw_dir: Path) -> list:
    f = raw_dir / ACCEPTED
    return [n for n in f.read_text().split() if n] if f.exists() else []


def set_accepted(raw_dir: Path, name: str, accepted: bool):
    """Add `name` to, or drop it from, the directory's ACCEPTED list."""
    names = [n for n in accepted_names(raw_dir) if n != name]
    if accepted:
        names.append(name)
    tmp = raw_dir / (ACCEPTED + ".tmp")
    tmp.write_text("".join(n + "\n" for n in names))
    tmp.replace(raw_dir / ACCEPTED)


def summary_table(raw_dir: Path) -> str:
    """Markdown table: one row per (stack, size), times in microseconds,
    over the .bin files named in the directory's ACCEPTED list only."""
    rows = []
    for name in sorted(set(accepted_names(raw_dir))):
        p = raw_dir / name
        parsed = parse_bin_name(p.name)
        if parsed is None:
            continue
        stem, size = parsed
        rows.append((stem, size, summarize_samples(read_bin(p))))
    rows.sort(key=lambda r: (r[1], r[0]))
    out = [
        "| Payload (B) | Stack | Samples | min (us) | p50 (us) | p99 (us) | max (us) |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ]
    for stem, size, s in rows:
        stack = stem[len("xhost_"):].replace("_", "-") if stem.startswith("xhost_") else stem
        out.append(
            f"| {size} | {stack} | {s['n']} | {fmt_us(s['min'])} | "
            f"{fmt_us(s['p50'])} | {fmt_us(s['p99'])} | {fmt_us(s['max'])} |"
        )
    return "\n".join(out) + "\n"


def parse_receipts(text: str) -> list:
    """Every `DELIVERY role=...` / `RTT_DELIVERY role=...` line as a dict."""
    out = []
    for line in text.splitlines():
        for tag in ("RTT_DELIVERY ", "DELIVERY "):
            i = line.find(tag)
            if i < 0:
                continue
            fields = {}
            for tok in line[i + len(tag):].split():
                k, eq, v = tok.partition("=")
                if eq:
                    fields[k] = v
            if "role" in fields:
                out.append(fields)
            break
    return out


def port_is_bound(port: int, proc_net: Path = Path("/proc/net")) -> bool:
    """True when a local TCP socket LISTENs on, or a UDP socket is bound to,
    `port` (read from /proc/net, so Linux only)."""
    want = f"{port:04X}"
    for name, listen_only in (("tcp", True), ("tcp6", True), ("udp", False), ("udp6", False)):
        f = proc_net / name
        if not f.exists():
            continue
        for line in f.read_text().splitlines()[1:]:
            cols = line.split()
            if len(cols) < 4:
                continue
            local, state = cols[1], cols[3]
            if local.rsplit(":", 1)[-1] != want:
                continue
            if listen_only and state != "0A":
                continue
            return True
    return False


def _hex_addr(field: str) -> str:
    """`0100007F:1E0A` (/proc/net/tcp, IPv4) -> `127.0.0.1:7690`."""
    host, port = field.split(":")
    if len(host) == 8:
        ip = ".".join(str(int(host[i:i + 2], 16)) for i in (6, 4, 2, 0))
    else:
        ip = host
    return f"{ip}:{int(port, 16)}"


def connected_peers(port: int, proc_net: Path = Path("/proc/net")) -> list:
    """Remote ends of the connected TCP and UDP sockets whose local port is
    `port` (TCP state 01 ESTABLISHED, UDP state 01 connected)."""
    want = f"{port:04X}"
    peers = set()
    for name in ("tcp", "tcp6", "udp", "udp6"):
        f = proc_net / name
        if not f.exists():
            continue
        for line in f.read_text().splitlines()[1:]:
            cols = line.split()
            if len(cols) < 4 or cols[3] != "01":
                continue
            if cols[1].rsplit(":", 1)[-1] == want:
                peers.add(f"{name[:3]}:{_hex_addr(cols[2])}")
    return sorted(peers)


# --------------------------------------------------------------------------
# process helpers
# --------------------------------------------------------------------------


def say(*parts):
    print(" ".join(str(p) for p in parts), flush=True)


def hold_box_lock(path: str):
    """flock `path` for the life of this process and return the open file
    (keep the reference). An existing file is opened read-only without
    O_CREAT, which works for a lock file another user owns in a sticky
    directory such as /tmp; a missing one is created mode 0666."""
    if not path:
        return None
    try:
        fd = os.open(path, os.O_RDONLY)
    except FileNotFoundError:
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    f = os.fdopen(fd, "r")
    say(f"LOCK waiting {path}")
    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
    say(f"LOCK held {path} pid={os.getpid()}")
    return f


class Stopped(Exception):
    """Raised in the main thread by SIGINT, SIGTERM or SIGHUP."""


def raise_on_signals():
    """Turn SIGINT/SIGTERM/SIGHUP into `Stopped`, so every `finally` that
    tears a run down executes before this process (and its box lock) goes.
    A dropped ssh session delivers SIGHUP."""
    def on_signal(sig, _frame):
        raise Stopped(signal.Signals(sig).name)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)


def ignore_signals():
    """Ignore SIGINT/SIGTERM/SIGHUP for the rest of this process, so a
    second signal cannot interrupt teardown. The node processes run in
    their own sessions and are stopped by `stop()`, not by these."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)


def start(cmd, log: Path, env=None, cwd=None):
    """Start `cmd` in its own session (so its process group id is its pid)
    with stdout+stderr to `log`."""
    log.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log, "w")
    say("START", shlex.join(cmd))
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            env=env, cwd=cwd, start_new_session=True)


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def stop(proc, sig=signal.SIGINT, grace=20.0) -> int:
    """Send `sig` to the process group, wait up to `grace` s for the whole
    group (leader and descendants) to exit, then SIGKILL what is left and
    wait for the group to empty. Works whether or not the leader has
    already exited. Returns the leader's exit status."""
    pgid = proc.pid
    if _group_alive(pgid):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        proc.poll()
        if proc.returncode is not None and not _group_alive(pgid):
            break
        time.sleep(0.1)
    if _group_alive(pgid):
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    proc.wait()
    deadline = time.monotonic() + 5.0
    while _group_alive(pgid) and time.monotonic() < deadline:
        time.sleep(0.05)
    if _group_alive(pgid):
        say(f"WARNING process group {pgid} still has members after SIGKILL")
    return proc.returncode


def remove_container(name: str):
    """Kill and remove a named container, then wait up to 30 s for
    `docker inspect` to stop finding it; a no-op when it is already gone.
    Prints a WARNING if it is still there, before the caller releases its
    lock."""
    deadline = time.monotonic() + 30.0
    while True:
        subprocess.call(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
        gone = subprocess.call(["docker", "inspect", name], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) != 0
        if gone:
            return
        if time.monotonic() > deadline:
            say(f"WARNING container {name} still exists after docker rm -f")
            return
        time.sleep(1.0)


def wait_port(port: int, proc, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return False
        if port_is_bound(port):
            return True
        time.sleep(0.2)
    return False


def write_text(path: Path, text: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


# --------------------------------------------------------------------------
# Cerulion
# --------------------------------------------------------------------------


def cerulion_bin(args) -> str:
    return args.cerulion or str(REPO_ROOT / "target" / "release" / "cerulion")


def cerulion_env(size: int, leg: str, samples: int, warmup: int, rate: int, out_dir: Path,
                 name: str) -> dict:
    env = dict(os.environ)
    env.update({
        "CERULION_LOGIN_GATE": "off",
        "CERULION_CPU_DMA_LOCK": "1",
        "IOX2_LOG_LEVEL": "error",
        # The live loop's per-input delivery telemetry (`drop_oldest=`) is an
        # info line; the delivery verdict requires it.
        "RUST_LOG": "info",
        "CER_BENCH_PAYLOAD_SIZE": str(size),
        "CER_BENCH_TARGET_SAMPLES": str(samples),
        "CER_BENCH_WARMUP": str(warmup),
        "CER_BENCH_TARGET_RATE_HZ": str(rate),
        "CER_BENCH_LEG": leg,
        "CER_BENCH_RAW_DUMP_DIR": str(out_dir),
        "CER_BENCH_RAW_NAME": name,
    })
    return env


def iox2_prefix(role: str) -> str:
    return f"xh{role[0]}{os.getpid()}_"


class Iox2Config:
    """The workspace's config/iceoryx2.toml with this side's private prefix
    for the duration of a `with` block; the file's earlier content (or its
    absence) is restored on exit. `graph run` and the gateway child it
    spawns both start in the workspace root, and iceoryx2 reads
    config/iceoryx2.toml relative to the working directory, so the graph
    and its gateway share the prefix. One run per checkout at a time."""

    def __init__(self, role: str):
        self.role = role
        self.path = WORKSPACE_DIR / "config" / "iceoryx2.toml"
        self.saved = None
        self.entered = False

    def __enter__(self):
        if self.path.exists():
            self.saved = self.path.read_text()
        self.entered = True
        write_text(self.path, f'[global]\nprefix = "{iox2_prefix(self.role)}"\n')
        return WORKSPACE_DIR

    def __exit__(self, *_exc):
        if not self.entered:
            return False
        self.entered = False
        if self.saved is None:
            self.path.unlink(missing_ok=True)
            try:
                self.path.parent.rmdir()
            except OSError:
                pass
        else:
            self.path.write_text(self.saved)
        remove_iox2_files(self.role)
        return False


def remove_iox2_files(role: str):
    """Delete the /dev/shm files left under this run's private prefix."""
    shm = Path("/dev/shm")
    if shm.is_dir():
        for f in shm.glob(iox2_prefix(role) + "*"):
            try:
                f.unlink()
            except OSError:
                pass


def build_cerulion():
    for cwd, cmd in ((REPO_ROOT, ["cargo", "build", "--release", "--locked", "-p", "cerulion_cli"]),
                     (WORKSPACE_DIR, ["cargo", "build", "--release", "--locked",
                                      "-p", "ping_node", "-p", "pong_node", "-p", "latency_node"])):
        say("BUILD", cwd, shlex.join(cmd))
        rc = subprocess.call(cmd, cwd=cwd)
        if rc:
            sys.exit(f"build failed ({rc}): {shlex.join(cmd)}")


# --------------------------------------------------------------------------
# ROS 2 / Fast DDS
# --------------------------------------------------------------------------


def ros_ws_dir(tag: str) -> Path:
    return HERE / ".ros_ws" / tag


def ros_tag(docker_image: str) -> str:
    base = "docker-" + docker_image.replace(":", "-").replace("/", "-") if docker_image else "native"
    return f"{base}-{os.uname().machine}"


def ros_shell(setup: str, ws: Path, body: str) -> str:
    return (f"source {shlex.quote(setup)}; "
            f"if [ -f {shlex.quote(str(ws / 'install' / 'setup.bash'))} ]; then "
            f"source {shlex.quote(str(ws / 'install' / 'setup.bash'))}; fi; {body}")


def ros_command(args, body: str, name: str) -> list:
    """bash -c `body` with ROS sourced, natively or in a container that
    shares the host network and bind-mounts the repository at its path.
    `--init` puts a signal-forwarding init process at PID 1 in the
    container, so a signal to the container reaches bash."""
    ws = ros_ws_dir(ros_tag(args.ros_docker))
    script = ros_shell(args.ros_setup, ws, body)
    if not args.ros_docker:
        return ["bash", "-c", script]
    cmd = ["docker", "run", "--rm", "--init", "--name", name, "--network", "host", "--ipc", "host",
           "-v", f"{REPO_ROOT}:{REPO_ROOT}", "-w", str(HERE)]
    if os.path.exists("/dev/cpu_dma_latency"):
        cmd += ["--device", "/dev/cpu_dma_latency"]
    for k in sorted(ros_env_keys()):
        if k in os.environ:
            cmd += ["-e", k]
    return cmd + [args.ros_docker, "bash", "-c", script]


ROS_BIN = "B=$(ros2 pkg prefix ros2_rtt_bench)/lib/ros2_rtt_bench; "


def ros_env_keys():
    return {"ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "ROS_LOCALHOST_ONLY", "CER_BENCH_PAYLOAD_SIZE", "CER_BENCH_TARGET_SAMPLES",
            "CER_BENCH_WARMUP", "CER_BENCH_TARGET_RATE_HZ", "CER_BENCH_PACING", "CER_BENCH_QOS",
            "CER_BENCH_MSG", "CER_BENCH_RAW_DUMP_DIR", "CER_BENCH_RAW_NAME"}


def set_ros_env(profile: Path, size: int, samples=0, warmup=0, rate=0, out_dir=None, name=""):
    os.environ.update({
        "ROS_DOMAIN_ID": str(ROS_DOMAIN_ID),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
        "ROS_LOCALHOST_ONLY": "0",
        "CER_BENCH_PAYLOAD_SIZE": str(size),
        "CER_BENCH_PACING": "fixed100",
        "CER_BENCH_QOS": "rel10",
        "CER_BENCH_MSG": "image",
    })
    if out_dir is not None:
        os.environ.update({
            "CER_BENCH_TARGET_SAMPLES": str(samples),
            "CER_BENCH_WARMUP": str(warmup),
            "CER_BENCH_TARGET_RATE_HZ": str(rate),
            "CER_BENCH_RAW_DUMP_DIR": str(out_dir),
            "CER_BENCH_RAW_NAME": name,
        })


def build_ros(args):
    ws = ros_ws_dir(ros_tag(args.ros_docker))
    (ws / "src").mkdir(parents=True, exist_ok=True)
    for pkg in ("ros2_rtt_msgs", "ros2_rtt_bench"):
        link = ws / "src" / pkg
        if not link.is_symlink():
            link.symlink_to(ROS2_DIR / pkg)
    body = (f"cd {shlex.quote(str(ws))} && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release "
            "&& source install/setup.bash && ros2 pkg prefix ros2_rtt_bench "
            "&& { dpkg-query -W -f '${Package} ${Version}\\n' "
            "'ros-*-fastrtps' 'ros-*-rmw-fastrtps-cpp' 'ros-*-rclcpp' 2>/dev/null || true; }")
    rc = subprocess.call(ros_command(args, body, f"xhost-build-{os.getpid()}"))
    if rc:
        sys.exit(f"ROS 2 build failed ({rc})")


# --------------------------------------------------------------------------
# subcommands
# --------------------------------------------------------------------------


def cmd_build(args):
    if args.stack.startswith("cerulion"):
        build_cerulion()
    else:
        build_ros(args)


def cmd_cert(args):
    _lock = hold_box_lock(args.box_lock)
    d = Path(args.dir)
    d.mkdir(parents=True, exist_ok=True)
    pem, key = d / "listener.pem", d / "listener.key"
    subprocess.run(
        ["openssl", "req", "-x509", "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:P-256",
         "-nodes", "-keyout", str(key), "-out", str(pem), "-days", "2", "-subj", "/CN=xhost",
         "-addext", f"subjectAltName=IP:{args.ip}",
         "-addext", "basicConstraints=critical,CA:FALSE"],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    key.chmod(0o600)
    sys.stdout.write(pem.read_text())


def cmd_trust(args):
    _lock = hold_box_lock(args.box_lock)
    pem = sys.stdin.read()
    if "-----BEGIN CERTIFICATE-----" not in pem:
        sys.exit("trust: stdin holds no PEM certificate")
    write_text(Path(args.out), pem)
    say(f"TRUSTED {args.out}")


def stdin_closed(timeout: float) -> bool:
    """True once stdin is at EOF; False after `timeout` s without EOF. The
    pair driver closes the ssh channel's stdin to stop a serve."""
    fd = sys.stdin.fileno()
    os.set_blocking(fd, False)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if os.read(fd, 4096) == b"":
                return True
        except BlockingIOError:
            pass
        time.sleep(0.2)
    return False


def refuse_bound_port(port: int):
    if port_is_bound(port):
        sys.exit(f"port {port} is already bound on this machine; pick another --port")


# Exit statuses a stop signal leaves on a node process: 0 for a clean
# handler exit, -N when the signal itself ended it, 128+N from a shell.
_STOP_STATUSES = {0, -signal.SIGINT, -signal.SIGTERM, 128 + signal.SIGINT, 128 + signal.SIGTERM}


def cmd_serve(args):
    raise_on_signals()
    _lock = hold_box_lock(args.box_lock)
    refuse_bound_port(args.port)
    out = Path(args.log_dir).resolve()
    log = out / f"serve_{raw_name(args.stack)}_{args.size}.log"
    container = f"xhost-pong-{os.getpid()}" if args.ros_docker else ""
    cerulion = args.stack in CERULION_CARRIER
    iox = Iox2Config("pong") if cerulion else None
    proc = None
    rc = None
    try:
        if cerulion:
            carrier = CERULION_CARRIER[args.stack]
            cert = key = ""
            if carrier == "quic":
                cert = str(Path(args.cert_dir).resolve() / "listener.pem")
                key = str(Path(args.cert_dir).resolve() / "listener.key")
            graph = f"{GRAPH_STEM}pong_{carrier}"
            write_text(WORKSPACE_DIR / "graphs" / f"{graph}.yaml",
                       render_pong_graph(carrier, args.listen_ip, args.port, args.size, cert, key))
            env = cerulion_env(args.size, "xhost-pong", 0, 0, 0, out, raw_name(args.stack))
            cwd = iox.__enter__()
            proc = start([cerulion_bin(args), "graph", "run", graph, "--release",
                          "--single-process"], log, env=env, cwd=cwd)
            sig = signal.SIGINT
        else:
            profile = out / "fastdds_listener.xml"
            write_text(profile, render_fastdds_listener(args.listen_ip, args.port))
            set_ros_env(profile, args.size)
            proc = start(ros_command(args, ROS_BIN + "exec $B/pong_node", container), log)
            sig = signal.SIGTERM
        if not wait_port(args.port, proc, 90):
            say(f"SERVE_FAILED stack={args.stack} size={args.size} log={log}")
            sys.stdout.write(log.read_text(errors="replace")[-4000:])
            return 3
        say(f"SERVE_READY stack={args.stack} size={args.size} port={args.port}")
        peers = set()
        while proc.poll() is None:
            peers.update(connected_peers(args.port))
            if stdin_closed(1.0):
                break
        say("PEERS " + (" ".join(sorted(peers)) or "none"))
        if proc.poll() is not None:
            say(f"SERVE_FAILED stack={args.stack} size={args.size}: the node exited "
                f"({proc.returncode}) before the ping side finished")
            return 3
        rc = stop(proc, sig)
    except Stopped as e:
        say(f"SERVE_STOPPED by {e}")
        return 130
    finally:
        ignore_signals()
        if proc is not None:
            stop(proc, signal.SIGTERM if not cerulion else signal.SIGINT)
        if container:
            remove_container(container)
        if iox is not None:
            iox.__exit__(None, None, None)
    text = log.read_text(errors="replace")
    receipts = parse_receipts(text)
    for r in receipts:
        say("RECEIPT", json.dumps(r, sort_keys=True))
    if rc not in _STOP_STATUSES:
        say(f"SERVE_FAILED stack={args.stack} size={args.size}: exit status {rc}")
        return 6
    why = serve_verdict(args.stack, receipts, args.expect, text)
    if why:
        say(f"SERVE_FAILED stack={args.stack} size={args.size}: {why}")
        return 7
    say(f"SERVE_END stack={args.stack} size={args.size} rc={rc}")
    return 0


# The Cerulion ping node ticks every 1 ms (`period_ms = 1`) and publishes on
# a wall grid at the requested rate, so the grid can be no finer than that.
CERULION_MAX_RATE_HZ = 1000
# The suite's fixed100 sustain rule: a cell whose ping skipped more than 1%
# of its grid slots did not run at the requested rate.
MAX_SKIPPED_FRACTION = 0.01


# Round trips a cell may lose without failing: the Cerulion ping publishes
# from its first tick, before the two gateways have connected, and frames
# are in flight when the latency node stops. The smoke runs lost 5 to 8.
MAX_UNANSWERED = 50


def _count(receipt: dict, key: str):
    try:
        return int(receipt[key])
    except (KeyError, ValueError):
        return None


def eviction_verdict(log_text: str) -> str:
    """'' when the Cerulion run log carries delivery telemetry and every
    `drop_oldest=` counter in it is zero, else the reason."""
    plain = re.sub(r"\x1b\[[0-9;]*m", "", log_text)
    evicted = [int(n) for n in re.findall(r"\bdrop_oldest=(\d+)", plain)]
    if not evicted:
        return "no drop_oldest telemetry in the run log"
    if any(evicted):
        return f"input evictions counted: drop_oldest={max(evicted)}"
    return ""


def serve_verdict(stack: str, receipts: list, expect: int, log_text: str) -> str:
    """'' when the listening side's receipt shows at least `expect` echoes
    (0 skips the count) and, for Cerulion, no input eviction."""
    pong = {r.get("role"): r for r in receipts}.get("pong")
    key = "forwarded" if stack in CERULION_CARRIER else "echoed"
    if pong is None or _count(pong, key) is None:
        return f"pong receipt has no {key}"
    if _count(pong, key) < expect:
        return f"pong {key} {pong[key]} of {expect}"
    if stack in CERULION_CARRIER:
        return eviction_verdict(log_text)
    return ""


def delivery_verdict(stack: str, receipts: list, total: int, log_text: str = "") -> str:
    """'' when the receipts show the cell ran as requested, else the reason.

    Both stacks: every counter named below must be present, the latency
    node must have received at least `total` (warmup + measured) echoes,
    and the ping must not exceed that by more than MAX_UNANSWERED.
    Cerulion: the ping receipt's `slots_skipped` must stay within 1% of
    `total` (the suite's fixed100 sustain rule), and the run log must carry
    `drop_oldest` telemetry with every count zero. Fast DDS: the kicks the
    latency node sent may exceed the pings published by at most
    MAX_UNANSWERED, and its `unstamped` and `nonpositive_rtt` must be
    zero."""
    by_role = {r.get("role"): r for r in receipts}
    ping, lat = by_role.get("ping"), by_role.get("latency")
    if ping is None or lat is None:
        return "missing ping or latency receipt"
    if stack in CERULION_CARRIER:
        need = {"ping": ("published", "slots_skipped"), "latency": ("received", "measured")}
    else:
        need = {"ping": ("published",),
                "latency": ("received", "kicks_sent", "unstamped", "nonpositive_rtt")}
    for role, keys in need.items():
        for k in keys:
            if _count(by_role[role], k) is None:
                return f"{role} receipt has no {k}"
    sent, got = _count(ping, "published"), _count(lat, "received")
    if got < total:
        return f"latency received {got} of {total}"
    if sent - got > MAX_UNANSWERED:
        return f"ping sent {sent}, latency received {got}"
    if stack in CERULION_CARRIER:
        skipped = _count(ping, "slots_skipped")
        if skipped > MAX_SKIPPED_FRACTION * total:
            return f"ping skipped {skipped} grid slots of {total}"
        return eviction_verdict(log_text)
    kicks = _count(lat, "kicks_sent")
    if kicks - sent > MAX_UNANSWERED:
        return f"latency sent {kicks} kicks, ping published {sent}"
    for k in ("unstamped", "nonpositive_rtt"):
        if _count(lat, k) != 0:
            return f"latency receipt {k}={lat[k]}"
    return ""


def cmd_ping(args):
    if args.stack in CERULION_CARRIER and not 0 < args.rate <= CERULION_MAX_RATE_HZ:
        sys.exit(f"--rate {args.rate}: the Cerulion ping node publishes on a 1 ms tick, "
                 f"so the rate must be 1..{CERULION_MAX_RATE_HZ} Hz")
    raise_on_signals()
    _lock = hold_box_lock(args.box_lock)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    name = raw_name(args.stack)
    bin_path = out / f"{name}_{args.size}.bin"
    rejected = bin_path.with_suffix(".rejected")
    for stale in (bin_path, rejected):
        stale.unlink(missing_ok=True)
    set_accepted(out, bin_path.name, False)
    log = out / "_logs" / f"{name}_{args.size}.log"
    budget = (args.samples + args.warmup) / args.rate + args.slack
    container = f"xhost-ping-{os.getpid()}" if args.ros_docker else ""
    cerulion = args.stack in CERULION_CARRIER
    iox = Iox2Config("ping") if cerulion else None
    proc = None
    rc = None
    try:
        if cerulion:
            carrier = CERULION_CARRIER[args.stack]
            graph = f"{GRAPH_STEM}ping_{carrier}"
            root_ca = str(Path(args.root_ca).resolve()) if args.root_ca else ""
            write_text(WORKSPACE_DIR / "graphs" / f"{graph}.yaml",
                       render_ping_graph(carrier, args.peer, args.port, args.size, root_ca))
            env = cerulion_env(args.size, "xhost-ping", args.samples, args.warmup, args.rate,
                               out, name)
            cwd = iox.__enter__()
            proc = start([cerulion_bin(args), "graph", "run", graph, "--release",
                          "--single-process"], log, env=env, cwd=cwd)
            sig = signal.SIGINT
        else:
            profile = out / "_logs" / "fastdds_dialer.xml"
            write_text(profile, render_fastdds_dialer(args.peer, args.port))
            set_ros_env(profile, args.size, args.samples, args.warmup, args.rate, out, name)
            body = (ROS_BIN + "$B/ping_node & P=$!; "
                    "trap 'kill -TERM $P 2>/dev/null' EXIT; "
                    "$B/latency_node; rc=$?; "
                    "kill -TERM $P; wait $P; trap - EXIT; exit $rc")
            proc = start(ros_command(args, body, container), log)
            sig = signal.SIGTERM
        try:
            rc = proc.wait(budget)
        except subprocess.TimeoutExpired:
            say(f"PING_TIMEOUT after {budget:.0f}s")
    except Stopped as e:
        say(f"PING_STOPPED by {e}")
    finally:
        ignore_signals()
        if proc is not None:
            stop(proc, signal.SIGINT if cerulion else signal.SIGTERM)
        if container:
            remove_container(container)
        if iox is not None:
            iox.__exit__(None, None, None)
    text = log.read_text(errors="replace") if log.exists() else ""
    receipts = parse_receipts(text)
    for r in receipts:
        say("RECEIPT", json.dumps(r, sort_keys=True))

    def reject(why: str, code: int):
        if bin_path.exists():
            bin_path.rename(rejected)
        say(f"PING_FAILED stack={args.stack} size={args.size} rc={rc}: {why}")
        sys.stdout.write(text[-4000:])
        return code

    if rc != 0:
        return reject("the run did not exit cleanly", 4)
    if not bin_path.exists():
        return reject(f"no {bin_path.name}", 4)
    try:
        samples = read_bin(bin_path)
    except (ValueError, OSError) as e:
        return reject(str(e), 5)
    if len(samples) != args.samples:
        return reject(f"{len(samples)} samples, expected {args.samples}", 5)
    why = delivery_verdict(args.stack, receipts, args.samples + args.warmup, text)
    if why:
        return reject(why, 5)
    s = summarize_samples(samples)
    say(f"PING_DONE stack={args.stack} size={args.size} n={s['n']} "
        f"min_us={fmt_us(s['min'])} p50_us={fmt_us(s['p50'])} p99_us={fmt_us(s['p99'])} "
        f"max_us={fmt_us(s['max'])}")
    return 0


def cmd_reject(args):
    """Drop a cell from ACCEPTED and rename its .bin to .rejected (the pair
    driver calls this for every failed cell)."""
    _lock = hold_box_lock(args.box_lock)
    out = Path(args.out)
    b = out / f"{raw_name(args.stack)}_{args.size}.bin"
    if out.is_dir():
        set_accepted(out, b.name, False)
    if b.exists():
        b.rename(b.with_suffix(".rejected"))
        say(f"REJECTED {b.name}")
    return 0


def cmd_accept(args):
    """Add a cell that `ping` left pending to ACCEPTED (the pair driver
    calls this once both sides of the cell passed)."""
    _lock = hold_box_lock(args.box_lock)
    out = Path(args.out)
    b = out / f"{raw_name(args.stack)}_{args.size}.bin"
    if not b.exists():
        sys.exit(f"accept: no {b}")
    set_accepted(out, b.name, True)
    say(f"ACCEPTED {b.name}")
    return 0


def remote(ssh: str, repo: str, sub: str) -> list:
    script = f"cd {shlex.quote(repo)}/benches/latency/crosshost && exec python3 crosshost.py {sub}"
    return shlex.split(ssh) + [script]


def cmd_pair(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    journal = open(out / "pair.log", "a")

    def note(line):
        say(line)
        journal.write(line + "\n")
        journal.flush()

    lock = f" --box-lock {shlex.quote(args.box_lock)}" if args.box_lock else ""
    remote_cert_dir = f"{args.listen_repo}/benches/latency/crosshost/.certs"
    remote_ca = f"{args.dial_repo}/benches/latency/crosshost/.certs/peer.pem"
    dial_out = f"{args.dial_repo}/benches/latency/crosshost/results/{args.run}"
    q = shlex.quote(dial_out)
    if subprocess.call(shlex.split(args.dial_ssh) + [f"mkdir -p $(dirname {q}) && mkdir {q}"]) != 0:
        sys.exit(f"could not reserve {dial_out} on the dialer (it exists?); pick a new --run")
    failed = []
    for stack in args.stacks:
        ros_l = ros_d = ""
        if stack == "fastdds-tcp":
            if args.listen_ros_docker:
                ros_l = f" --ros-docker {shlex.quote(args.listen_ros_docker)}"
            if args.dial_ros_docker:
                ros_d = f" --ros-docker {shlex.quote(args.dial_ros_docker)}"
        root_ca = ""
        if stack == "cerulion-quic":
            got = subprocess.run(remote(args.listen_ssh, args.listen_repo,
                                        f"cert --ip {args.listen_ip} --dir {remote_cert_dir}{lock}"),
                                 check=True, capture_output=True, text=True).stdout
            pem = got[got.find("-----BEGIN CERTIFICATE-----"):]
            subprocess.run(remote(args.dial_ssh, args.dial_repo,
                                  f"trust --out {remote_ca}{lock}"),
                           input=pem, text=True, check=True)
            root_ca = f" --root-ca {remote_ca}"
            note(f"CERT stack={stack} listener certificate copied to the dialer")
        for size in args.sizes:
            remote_log = f"{args.listen_repo}/benches/latency/crosshost/results/_serve"
            serve = subprocess.Popen(
                remote(args.listen_ssh, args.listen_repo,
                       f"serve --stack {stack} --size {size} --listen-ip {args.listen_ip} "
                       f"--port {args.port} --cert-dir {remote_cert_dir} "
                       f"--log-dir {remote_log} --expect {args.samples + args.warmup}"
                       f"{ros_l}{lock}"),
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            ready = False
            for line in serve.stdout:
                note(f"listen| {line.rstrip()}")
                if line.startswith("SERVE_READY"):
                    ready = True
                    break
                if line.startswith("SERVE_FAILED"):
                    break
            ping_rc = None
            if ready:
                time.sleep(args.settle)
                ping = subprocess.run(
                    remote(args.dial_ssh, args.dial_repo,
                           f"ping --stack {stack} --size {size} --peer {args.listen_ip} "
                           f"--port {args.port} --out {dial_out} --samples {args.samples} "
                           f"--warmup {args.warmup} --rate {args.rate}"
                           f"{root_ca}{ros_d}{lock}"),
                    capture_output=True, text=True)
                for line in (ping.stdout + ping.stderr).splitlines():
                    note(f"dial| {line}")
                ping_rc = ping.returncode
            serve.stdin.close()
            for line in serve.stdout:
                note(f"listen| {line.rstrip()}")
            serve.wait()
            ok = ready and ping_rc == 0 and serve.returncode == 0
            if ok:
                r = subprocess.run(remote(args.dial_ssh, args.dial_repo,
                                          f"accept --stack {stack} --size {size} "
                                          f"--out {dial_out}{lock}"),
                                   capture_output=True, text=True)
                for line in (r.stdout + r.stderr).splitlines():
                    note(f"dial| {line}")
                ok = r.returncode == 0
            if not ok:
                failed.append(f"{stack}/{size}")
                r = subprocess.run(remote(args.dial_ssh, args.dial_repo,
                                          f"reject --stack {stack} --size {size} "
                                          f"--out {dial_out}{lock}"),
                                   capture_output=True, text=True)
                for line in (r.stdout + r.stderr).splitlines():
                    note(f"dial| {line}")
                if r.returncode != 0:
                    note(f"REJECT_FAILED stack={stack} size={size} rc={r.returncode}: "
                         f"{dial_out} may still hold this cell's .bin")
            note(f"CELL_{'END' if ok else 'FAILED'} stack={stack} size={size} "
                 f"ping_rc={ping_rc} serve_rc={serve.returncode}")
    note(f"PAIR_END cells={len(args.stacks) * len(args.sizes)} failed={len(failed)} "
         + " ".join(failed))
    return 1 if failed else 0


def cmd_summarize(args):
    sys.stdout.write(summary_table(Path(args.dir)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    def ros_opts(p):
        p.add_argument("--ros-setup", default="/opt/ros/humble/setup.bash",
                       help="ROS 2 setup script to source (fastdds-tcp)")
        p.add_argument("--ros-docker", default="",
                       help="run the ROS 2 nodes in this image instead of natively")

    p = sub.add_parser("build", help="build the binaries a stack needs on this machine")
    p.add_argument("--stack", choices=STACKS, required=True)
    ros_opts(p)
    p.set_defaults(fn=cmd_build)

    p = sub.add_parser("cert", help="mint a quic/ listener certificate; prints the PEM")
    p.add_argument("--ip", required=True, help="the IP the dialer connects to")
    p.add_argument("--dir", required=True)
    p.add_argument("--box-lock", default="")
    p.set_defaults(fn=cmd_cert)

    p = sub.add_parser("trust", help="write the listener certificate read from stdin")
    p.add_argument("--out", required=True)
    p.add_argument("--box-lock", default="")
    p.set_defaults(fn=cmd_trust)

    for name, fn in (("serve", cmd_serve), ("ping", cmd_ping)):
        p = sub.add_parser(name)
        p.add_argument("--stack", choices=STACKS, required=True)
        p.add_argument("--size", type=int, required=True, help="payload bytes")
        p.add_argument("--port", type=int, default=7690)
        p.add_argument("--cerulion", default="", help="cerulion binary (default: target/release)")
        p.add_argument("--box-lock", default="", help="flock this file for the life of the run")
        ros_opts(p)
        p.set_defaults(fn=fn)
        if name == "serve":
            p.add_argument("--listen-ip", required=True)
            p.add_argument("--cert-dir", default=str(HERE / ".certs"))
            p.add_argument("--log-dir", default=str(DEFAULT_RAW_DIR / "_serve"))
            p.add_argument("--expect", type=int, default=0,
                           help="fail unless pong echoed at least this many frames")
        else:
            p.add_argument("--peer", required=True, help="the listener's IP")
            p.add_argument("--out", default=str(DEFAULT_RAW_DIR / "latest"))
            p.add_argument("--root-ca", default="")
            p.add_argument("--samples", type=int, default=2000, help="measured round trips")
            p.add_argument("--warmup", type=int, default=100)
            p.add_argument("--rate", type=int, default=100, help="pings per second")
            p.add_argument("--slack", type=float, default=90.0,
                           help="seconds allowed beyond (samples+warmup)/rate")

    p = sub.add_parser("pair", help="run serve + ping on two machines over ssh")
    p.add_argument("--listen-ssh", required=True, help="command prefix that runs a command on the listener")
    p.add_argument("--dial-ssh", required=True, help="command prefix that runs a command on the dialer")
    p.add_argument("--listen-ip", required=True)
    p.add_argument("--listen-repo", required=True)
    p.add_argument("--dial-repo", required=True)
    p.add_argument("--listen-ros-docker", default="")
    p.add_argument("--dial-ros-docker", default="")
    p.add_argument("--stacks", nargs="+", choices=STACKS, default=list(STACKS))
    p.add_argument("--sizes", nargs="+", type=int, default=[64, 1024, 16384, 65536])
    p.add_argument("--port", type=int, default=7690)
    p.add_argument("--samples", type=int, default=2000)
    p.add_argument("--warmup", type=int, default=100)
    p.add_argument("--rate", type=int, default=100)
    p.add_argument("--settle", type=float, default=2.0, help="seconds between serve ready and ping")
    p.add_argument("--run", default="latest", help="results subdirectory on the dialer")
    p.add_argument("--box-lock", default="", help="lock file both machines flock per command")
    p.add_argument("--out", default=str(DEFAULT_RAW_DIR / "_pair"))
    p.set_defaults(fn=cmd_pair)

    for name, fn, text in (("reject", cmd_reject, "mark one cell's .bin as rejected"),
                           ("accept", cmd_accept, "list a pending cell as accepted")):
        p = sub.add_parser(name, help=text)
        p.add_argument("--stack", choices=STACKS, required=True)
        p.add_argument("--size", type=int, required=True)
        p.add_argument("--out", required=True)
        p.add_argument("--box-lock", default="")
        p.set_defaults(fn=fn)

    p = sub.add_parser("summarize", help="min/p50/p99/max table for a results directory")
    p.add_argument("dir")
    p.set_defaults(fn=cmd_summarize)

    args = ap.parse_args(argv)
    return args.fn(args) or 0


if __name__ == "__main__":
    sys.exit(main())
