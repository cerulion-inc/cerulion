# Remote access safety: the deadman, degraded links, and offline operation

When you drive a robot from another network, three things decide whether a bad
link is safe: what happens when your control link goes quiet, what happens when
it is merely slow, and what still works when the internet does not. This page
answers those three for the remote plane, the internet path to a robot: one
iroh endpoint per robot, dial-by-key, the `cerulion/ops/1` and
`cerulion/wire/2` protocols, designed to be reachable from boot, gated by the
offline pairing access list.

It is separate from [`docs/networking.md`](networking.md), which covers the LAN
zenoh gateway (`network:` blocks, `topic list`, mDNS).

**What ships today.** The lease, deadman and e-stop state machine, and the
remote `engage-estop` verb. Engaging the e-stop records the engaged state and
its engager; it does not stop motion by itself, and the state is held in the
daemon's memory, so a daemon restart clears it. Nothing outside the daemon can
read that state yet: an engage returns a reply to the caller, writes a warning
to the log, and asks Flashback to save a recording. A robot still needs its own
stop path (a hardwired e-stop) to stop motion. The `teleop` verb is classified
(it needs `CAP_TELEOP` and at least the operator role) but has no handler yet,
and the daemon does not run the deadman yet. The deadman and renewal rules below
are the contract a teleop handler runs under.

## What a desk can do today

The `cerulion/wire/2` protocol carries data from the robot to the desk only. A
paired desk with observe access can:

- read the robot's topic catalog;
- start and stop a stream of any topic it may observe, and read each frame as
  the robot publishes it (`cerulion connect` brings these into the desk's own
  shared memory);
- fetch a topic's schema, so a desk that has never seen the type can decode it;
- read the per-topic counters (`status`);
- deliver a revocation epoch it carries, which the robot checks on its own.

A desk cannot write a topic on the robot or send it a command over the remote
plane. No verb on `cerulion/wire/2` takes a frame from the desk, and the robot
never reads a data stream from the desk. The `teleop` and `deploy` verbs are
classified but have no handler. The ops protocol (`cerulion/ops/1`) runs the
verbs the robot has registered, such as `inventory`, `log-tail`, `restart` and
`engage-estop`; none of them carries topic data. The only desk to robot topic
path today is LAN ingress (an `ingress:` block in a graph), which proves no
identity per peer and is closed when `CERULION_LAN_ACCESS=paired`.

## The deadman: a quiet operator loses the lease

Actuation is held by a control lease, and the holder must renew it inside the
deadman window. Miss the window and the deadman fires, putting the robot into
its safe frame, so an operator who goes silent, for any reason, is designed not
to leave a robot moving.

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
