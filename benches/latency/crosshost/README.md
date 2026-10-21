# Cross-host round-trip latency

`crosshost.py` measures the wall-clock round trip between two machines: a
ping leaves the dialing machine, the listening machine echoes it, and the echo
lands back on the dialing machine. Both stamps of a sample are read on the
dialing machine's `CLOCK_MONOTONIC`, so the offset between the two clocks
never enters a number. Half a round trip is not a one-way latency unless the
two directions are symmetric; the table reports round trips only.

## Topology

```
dialing machine                               listening machine
ping -> egress /xhost/ping/ping_out  ------>  ingress -> pong
latency <- ingress /xhost/pong/echo_out <---  egress  <- pong
```

The dialing machine opens every connection between the two. The pair
therefore works on a network that admits connections from one side only.

## Stacks

| `--stack` | Dialing side | Listening side | Link |
|---|---|---|---|
| `cerulion-tcp` | `cerulion graph run` with ping + latency | `cerulion graph run` with pong | Strict `network:` block, `tcp/<listener-ip>:<port>` |
| `cerulion-quic` | the same | the same | `quic/<listener-ip>:<port>`, TLS 1.3, the dialer verifies the listener's certificate |
| `fastdds-tcp` | ROS 2 `ping_node` + `latency_node` on `rmw_fastrtps_cpp` | ROS 2 `pong_node` | Fast DDS TCPv4, one connection per dialing process |

- **Cerulion.** The nodes are the workspace bench nodes in `../workspace/nodes`
  (`ping_node`, `pong_node`, `latency_node`), so the message is
  `sensor_msgs/Image` with the payload in its `data` field, and the ping node
  publishes on a 100 Hz wall grid. Each side runs `--single-process`. The
  Strict block makes each side spawn its own gateway child, which carries the
  frames between shared memory and the link. Every topic is reliable.
- **Fast DDS.** The nodes are the ROS 2 bench nodes in `../ros2` with
  `CER_BENCH_MSG=image` (`sensor_msgs/msg/Image`) and `CER_BENCH_QOS=rel10`
  (RELIABLE, KEEP_LAST 10), paced at 100 Hz by the latency node's kick timer.
  The listener's profile has one TCPv4 transport, whitelisted to the listen
  address, and no built-in transports. The dialer's profile has a TCPv4
  transport with no listening port, so it only opens connections, plus a
  UDPv4 transport bound to loopback that carries discovery and the kick
  between the two local processes. Each dialing process is its own
  participant and opens its own connection: the ping travels on the ping
  process's connection and the echo returns on the latency process's. The
  listener has no UDP transport, so all of its traffic to the dialer rides
  those connections. Both profiles set `enable_tcp_nodelay`, so a small
  sample is sent at once rather than held for the previous segment's ACK.
  Fast DDS over UDP needs the listener to open flows toward the dialer, which
  a one-sided network refuses.

Each stack's samples go into `<out>/xhost_<stack>_<payload>.bin`
(little-endian u64 nanoseconds, the suite's format). The default run is 100
warmup round trips, then 2000 measured round trips at 100 Hz per payload
size. With 2000 samples the p99 rests on 20 samples above it, the suite's
citable minimum (`../compile_csv.py`).

## Running

Both machines need a checkout of this repository. On each:

```sh
cd benches/latency/crosshost
python3 crosshost.py build --stack cerulion-tcp     # cerulion CLI + the three node cdylibs
python3 crosshost.py build --stack fastdds-tcp      # colcon build of ../ros2 (ROS 2 sourced)
python3 crosshost.py build --stack fastdds-tcp --ros-docker ros:humble-ros-base   # or in a container
```

One cell by hand, listener first. Both sides must take the same `--size`:
each pong builds its echo at its own configured size and nothing checks the
received length, so a mismatch carries a different payload back than out
under the ping side's label. `pair` passes one size to both.

```sh
# listening machine (stays up until stdin closes or Ctrl-C)
python3 crosshost.py serve --stack cerulion-tcp --size 1024 --listen-ip <listener-ip> --expect 2100
# dialing machine
python3 crosshost.py ping --stack cerulion-tcp --size 1024 --peer <listener-ip> --out results/run1
# close serve's stdin (Ctrl-D); once it printed SERVE_END, on the dialing machine:
python3 crosshost.py accept --stack cerulion-tcp --size 1024 --out results/run1
python3 crosshost.py summarize results/run1
```

`ping` leaves a passing cell pending: only the listener's log shows its
evictions and echo count, so a cell enters `ACCEPTED` through `accept` after
`serve` printed `SERVE_END` (or `reject` after `SERVE_FAILED`). Ctrl-C stops
`serve` without checking its side.

For `cerulion-quic`, mint the listener's certificate with
`crosshost.py cert --ip <listener-ip> --dir .certs` on the listening machine,
copy the printed PEM to the dialing machine, and pass it as `--root-ca`.

`pair` drives a whole sweep from a third machine over ssh, starting `serve`
on the listener and `ping` on the dialer for each stack and size, and copying
the QUIC certificate across:

```sh
python3 crosshost.py pair \
  --listen-ssh 'ssh listener-host' --listen-ip <listener-ip> --listen-repo /path/on/listener \
  --dial-ssh 'ssh dialer-host' --dial-repo /path/on/dialer \
  --stacks cerulion-tcp cerulion-quic fastdds-tcp --sizes 64 1024 16384 65536 \
  --run run1 --box-lock /tmp/bench.lock
```

`--box-lock` makes every command `flock` that file for its own lifetime on
the machine it runs on (an existing file is opened read-only; a missing one
is created). `pair` refuses a `--run` directory that already exists on the
dialer and exits nonzero when any cell fails. `--listen-ros-docker` / `--dial-ros-docker` run the
ROS 2 side of either machine in a container.

## What each line says

- `SERVE_READY` prints once the listener's port is bound (TCP LISTEN or a
  bound UDP socket for QUIC), read from `/proc/net`, so `serve` needs Linux.
- `RECEIPT` lines repeat each node's delivery receipt: `published` by ping,
  `forwarded` (Cerulion) or `echoed` (Fast DDS) by pong, `received` by
  latency. The Cerulion ping publishes from its first tick, before the
  gateways have connected, and the latency node stops at its sample count
  while pings are still in flight, so its `published` exceeds `received` by
  a few frames. The Fast DDS ping publishes only when the latency node
  kicks it. The latency node starts kicking once its kick writer has
  matched the ping's reader, its echo reader has matched the pong's writer,
  and the graph shows a reader on the ping topic; the ping writer's own
  match to the pong is inferred from those, so an early ping can still be
  lost before it, which the warmup absorbs.
- `PING_DONE` prints min/p50/p99/max for the cell and leaves its `.bin`
  pending; `accept` adds it to the run directory's `ACCEPTED` list, which is
  all `summarize` reads (`pair` runs `accept` only after both sides passed). A cell fails instead, and its `.bin` is renamed `.rejected`, when the run
  does not exit cleanly, the sample count is not `--samples`, a receipt or
  one of its counters is missing, the latency node received fewer than
  warmup + measured echoes, the ping sent more than 50 beyond what came
  back, the Cerulion ping skipped more than 1% of its grid slots or the run
  log has no `drop_oldest` telemetry or counts an eviction, or the Fast DDS
  latency node sent more than 50 kicks beyond the pings published or counted
  an unstamped or non-positive echo. `serve --expect N` (which `pair` passes
  as warmup + measured) fails the listening side when pong echoed fewer
  than N frames or, for Cerulion, when its log has no `drop_oldest`
  telemetry or counts an eviction; `pair` then rejects the cell's `.bin` and
  says so if it cannot.
- `PEERS` lists the remote addresses connected to the listener's port during
  the cell (TCP; a QUIC listener's UDP socket is unconnected and lists none).

## Limits

- One run per cell. Repeat runs to see run-to-run spread.
- Fast DDS pacing comes from the latency node's 100 Hz wall timer. A timer
  that fires late sends fewer kicks over a longer window; no receipt counts
  that, unlike the Cerulion ping's skipped-slot count.
- The numbers include both machines' scheduling and the network between them.
  Report the path (wired, Wi-Fi, an overlay and its MTU) next to any table.
- The stacks differ in more than the transport: Cerulion goes through a
  gateway process on each machine, while the Fast DDS participants send from
  the node processes themselves.
