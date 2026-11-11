# Remote access safety: the deadman, degraded links, and offline operation

When you drive a robot from another network, three things decide whether a bad
link is safe: what happens when your control link goes quiet, what happens when
it is merely slow, and what still works when the internet does not. This page
answers those three for the remote plane, the internet path to a robot: one
iroh endpoint per robot, dial-by-key, the `cerulion/ops/1` and
`cerulion/wire/2` protocols, designed to be reachable from boot, gated by the
offline pairing access list.

A serving machine can also read paired robots through that same device
endpoint. Outbound topic mirrors use the existing pull protocol; releasing a
mirror preserves the machine's incoming remote service.

It is separate from [`docs/networking.md`](networking.md), which covers the LAN
zenoh gateway (`network:` blocks, `topic list`, mDNS).

**What ships today.** The lease, deadman and e-stop state machine, and the
remote `engage-estop` verb. Engaging the e-stop records the engaged state and
its engager and sends an `estop` to the robot's `cerulion-rcpd`, again until
rcpd answers, which closes rcpd's own latch and publishes an e-stop frame to
the graph; neither stops motion by itself. The engaged state is held in the daemon's memory, so a
daemon restart clears it, and nothing else does; rcpd's latch, once that
`estop` closed it, clears only through its `estop_clear` or by deleting its
latch file and restarting rcpd.
No verb returns that state yet; a verb or rcp tool call that it refuses says
so in its error. An `engage-estop` engages the latch, sends that `estop`,
then writes a warning to the log, asks Flashback to save a recording
and returns a reply to the caller; an `estop` over the rcp plane engages
the latch without that ask.
It is the one e-stop latch the ops act verbs, `restart` and the
`cerulion/rcp/1` plane obey: while it is engaged, the ops act verbs `topic-publish` and
`service-call`, the ops verb `restart` and every tool call over the
`cerulion/rcp/1` plane except `estop` are refused, and an `estop` over that
plane engages it too (see
[`docs/rcp.md`](rcp.md#one-e-stop-for-the-ops-and-rcp-planes)). `restart`
reads it before it stops the graph and again before it starts it: an engage
during the stop means the start command does not run, and an engage after
the second read does not stop the graph it starts. LAN ingress does not read
it. An `engage-estop`
refuses each rcp tool call other than `estop` that the plane has not yet sent
to the robot's `cerulion-rcpd` (it reads the latch on its open connection to
rcpd just before each send), then sends an `estop` to rcpd without waiting
for it, and sends it again while rcpd is not reachable or answers with an
error, until rcpd answers that its latch closed. It logs the first failure and
the 10th, 100th and so on at warn and the others at debug, and a delivery that
ends before rcpd answers, as at a stop by Ctrl-C or SIGTERM, at error; a kill,
a crash, the hard-exit deadline or a second stop signal may end it with no
line. That latch then refuses the mutating calls rcpd holds when the `estop`
reaches it and has not yet published;
an `estop` over the rcp plane closes rcpd's own latch too. A robot still needs its own stop path (a hardwired e-stop) to
stop motion. Only the `cerulion-remoted` daemon serves
`engage-estop`, `restart`, the act verbs (`topic-publish`, `service-call`,
`topic-snapshot`, `topic-echo-once`) and the `cerulion/rcp/1` plane: a process
that hosts the remote plane as a library refuses each of those verbs with an
`unsupported_host` error and serves no rcp plane. An Android app can
host the plane in its own process (see
[Host the paired remote plane](android.md#host-the-paired-remote-plane)); a
paired desk reads its catalog, schemas and topics there and cannot publish
into the app. The public iroh relays an app or a robot uses by default are for
development only: run your own relays for a robot in service. The `teleop` verb is classified
(it needs `CAP_TELEOP` and at least the operator role) but has no handler yet,
and the daemon does not run the deadman yet. The deadman and renewal rules below
are the contract a teleop handler runs under.

## What a desk can do today

The `cerulion/wire/2` protocol carries topic data from the robot to the desk
only; the desk sends control requests, nothing else. A paired desk with observe
access can:

- read the robot's topic catalog;
- start and stop a stream of any topic it may observe, and read each frame as
  the robot publishes it (`cerulion connect` brings these into the desk's own
  shared memory);
- fetch a topic's schema, so a desk that has never seen the type can decode it;
- read the per-topic counters, and the count of log lines the robot's
  `cerulion-remoted` dropped (`status`);
- deliver a revocation epoch it carries, which the robot checks on its own.

The topic CLI and `viz --robot` also use this plane automatically for account-owned
and explicitly paired robots. `topic list` shows the robot's reachability and its
catalog without a manual connect command. Desk observers use private channels
under `/__cerulion/observers/`, keeping a robot's topic separate from a local topic
with the same canonical name; wire/2 still names the original topic. Observers of
one source share a demand, and the last observer releases the channel. Graph
ingress keeps canonical names and the single-writer contract.

A desk cannot write a topic on the robot or send it teleop commands over
`cerulion/wire/2`. No verb on it takes a frame from the desk, and the robot
never reads a data stream from the desk. The `teleop` and `deploy` verbs
are classified but have no handler. The ops protocol (`cerulion/ops/1`) does run
the verbs the robot has registered, such as `inventory`, `log-tail`, `restart`
and `engage-estop`, for a desk with the access each one needs. Four more, the
act verbs, work on the robot's own shared memory; with `--robot`,
`cerulion topic pub`, `service call` and `topic snapshot` call `topic-publish`,
`service-call` and `topic-snapshot`:

- With the teleoperate capability (`CAP_TELEOP`) and at least the operator
  role, `topic-publish` publishes a message on a robot topic and
  `service-call` calls a robot service once. Both are refused while the
  robot's e-stop is engaged, and one already running stops when it engages.
  They check the e-stop, not the control lease, so the deadman does not cover
  them. `topic-publish` refuses a topic under the reserved `/__cerulion`
  namespace.
- With observe access, `topic-snapshot` reads the newest frame of a camera
  topic as a JPEG and `topic-echo-once` reads the newest message of a topic,
  decoded.

While the desk's network kill-switch is engaged (`CERULION_NETWORK=off`),
`--robot` is refused on the desk before anything is sent to the robot.
`cerulion rcp` is refused the same way.

A graph with declared `ingress:` can pull an exported topic from a unique
same-account or explicitly paired producer over the authenticated remote plane.
The receiving graph checks the declared topic, schema, QoS and single-writer
rules. Declared imports require a native service provisioned for one publisher;
a multi-publisher service is refused even with only one live writer. The
producer serves its stream; the consumer does not send arbitrary data
frames to the producer. A machine can serve and consume through the same owned
endpoint. This is distinct from the ops `topic-publish` verb and its capability
and e-stop checks.

Each robot exports its own local producers. Imported graph topics and private
observer channels keep their source robot's access policy: read them on that
source robot. A refused outbound demand or an unavailable robot during metadata
discovery preserves the machine's other active readers. Ops snapshot, one-shot
echo, publish and service calls enforce the same source restriction. Outbound
readers retain the owned server against idle shutdown; new admission returns a retryable
refusal while shutdown is already committed. Account ownership proofs survive
the switch between consuming and serving. Reader restoration shares one bounded
budget; unresolved readers receive a named loss notice and can reconnect.

The zenoh LAN ingress compatibility path proves no identity per peer and is
closed by the default `CERULION_LAN_ACCESS=paired`. Enabling that path requires
explicit `CERULION_LAN_ACCESS=open` and warns at startup. Authenticated discovery is described in [networking](networking.md#making-a-robot-discoverable).

## The deadman: a quiet operator loses the lease

Actuation through a teleop handler is held by a control lease, and the holder
must renew it inside the deadman window. Miss the window and the deadman fires,
putting the robot into its safe frame, so an operator who goes silent, for any
reason, is designed not to leave a robot moving.

The window ships at **500 ms**
(`LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM`, `crates/cerud/src/constants.rs`).

**Size it from the actuator, not from the network.** The right window is decided
by the actuator's control-loop rate and its physical stopping distance, so a
deployment sets it on the robot and confirms it there. The same applies to what
the safe frame does: which actuators to zero, which brakes to engage, the ramp
profile. The lease layer owns the permission floor only, who may stop; it emits
no actuation of its own. Do not treat 500 ms as an actuation contract until it
has been confirmed against your own hardware.

## A slow link eats the window

The deadman measures wall time between renewals **on the robot**. A remote
operator's renewals travel the control path, so the cadence the robot sees is
your send cadence plus the one-way latency and its jitter. On a path with
round-trip time `R` and jitter `J`, renewing every `T` keeps actuation live only
while:

```
T + (R/2) + J  <  deadman_window
```

| Path | Typical round trip | Against a 500 ms window |
|---|---|---|
| LAN or direct hole punch | under 5 ms | Ample. Renew every 100 ms and normal jitter never fires the deadman. |
| Relayed wide-area | 100 to 300 ms plus jitter | Tight. One stall or a jitter burst can push a renewal past the window and fire the safe frame. |

That is the intended failure: a degraded control link produces a stop, not a
runaway. Size the window for the worst latency the paths you allow teleop over
can produce.

**Relayed paths are not refused.** Authorization does not look at the path, so a
teleop handler will not refuse a relayed connection, including relay fallback:
degraded latency is designed to drive a safe frame through the deadman rather
than a silent runaway. A teleop client should show the operator the live
control-path round trip and warn as it approaches the window, rather than
quietly downgrading.

## E-stop does not wait behind another session

E-stop is the permission floor. Any paired session may engage it, it is never
lease-gated, and a teleop handler must not actuate while it is engaged. That has
to hold at the transport layer too, so ops sessions are served **concurrently**:
one shared ops server handles every session at once, with the receipt log behind
a brief append lock. A stalled or hostile session parks on a blocking read
holding no lock, so it cannot delay anyone else, and a reconnect loop cannot
hold a permit to keep a legitimate e-stop waiting. The safety effect takes only
the control-lease lock and runs before its audit receipt, so it never waits on
the audit log. Its reply then waits for the receipt append, a brief lock held
across one write and sync of the receipt file. Hash-chain integrity survives:
each append is one atomic critical section, so concurrent appends form one valid
tamper-evident chain.

**A blocked log does not hold an e-stop.** `cerulion-remoted` writes its log
lines to stderr from a thread of its own, through a queue of at most 4096
lines and 1 MiB. While stderr takes no more lines, as with a full pipe or a
stalled journald, no part of the daemon waits on one of those lines: a line
that does not fit in the queue, or whose write fails, is dropped and counted.
So an `estop` over `cerulion/rcp/1` and an ops `engage-estop` still reach the
robot's `cerulion-rcpd`, and a graceful stop still ends. Two writers bypass
the queue and still write to stderr directly: the iceoryx2 console logger and
the message of a panic. Once stderr takes lines again, a warning names how
many were dropped and, when more than one was, an info line says the log
caught up. The count since the daemon started is `log_lines_dropped` in the
`cerulion/wire/2` `status` reply; it leaves out the writer's own report lines
and the lines an exit discards. A host that embeds the remote plane logs
through its own subscriber and sends no such count. Each exit waits at most
100 ms for the lines still queued; past four fifths of that, all but the
newest 8 are discarded, and one error line counts them if stderr takes it in
time.

**What this rules out is serialization, not unavailability.** Resource
exhaustion and network delivery still decide whether a remote `engage-estop`
arrives. Each session is bounded by a 120 s deadline that closes an
authenticated-then-stalled connection, which is a per-session resource guard
rather than a serialization point. Two caps that would bound the total footprint
under a distributed flood, a global concurrency cap with a reserved safety-floor
slot and the daemon's accept-loop cap, are **not implemented**.

## A dropped connection does not drop the lease

The lease names its holder, and the e-stop its engager, by a session token,
and the remote plane uses the TLS-authenticated device key as that token
(`engage-estop` does today). The key is stable per pairing, unlike the
connection id. A link that drops and reopens inside the deadman window
re-presents the same identity, so a teleop handler renews the same lease: no
fresh grant, no re-arbitration, no gap in actuation. An engaged e-stop persists
across reconnects (the remote plane has no verb that clears it), and a
different paired device cannot take the floor from the one that engaged it. A
drop **longer** than the window lapses the lease under the deadman contract,
which is correct: the operator is gone.

## It never bricks offline

On the LAN, an established pairing reaches the robot with no cloud anywhere in
the connect path: a direct iroh dial to the robot address you give
(`--addr ip:port`), with the relay disabled (`--relay-disabled`), and offline
verification against the durable access list. mDNS supplies the robot's
endpoint id to `cerulion pair`; it does not supply a dial address, so an
offline connect needs `--addr`. Established pairings do not expire (a
time-boxed guest grant from the owner is the exception), last until revoked
(see [`docs/revocation.md`](revocation.md)), and never re-run the certificate
chain per connect; the access row is the truth.

The issuer, which mints certificates for **new** pairings, and the relay, which
provides reach from off the LAN, are conveniences rather than dependencies. A
robot with the issuer down and the relay unreachable still serves an established
pairing on the LAN.

## See also

- [`docs/revocation.md`](revocation.md): taking an account's or one device's
  access back.
- [`docs/networking.md`](networking.md): the LAN plane, discovery and what a run
  exposes by default.

## Graph ingress queue policy

A graph producer declares its reliable queue policy with its served topic
metadata. Wire forwarding sizes that queue to `send_queue`, or the actual tap
depth when omitted. At capacity it refuses the new frame, retains queued
frames in order, and exposes `wan_refused` separately from `wan_dropped`.
A source without policy metadata is unverified for graph ingress.
Best-effort datagram graph ingress is refused on this wire path.
Changing a retained topic's queue policy requires restarting `cerulion-netd`.

Observer reads use their existing bounded best-effort queue, including exports
with BestEffort policy and multiple publishers. The reliable-policy and
single-writer refusals apply to canonical graph imports.
