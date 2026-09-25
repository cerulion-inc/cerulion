# Remote access safety: the deadman, degraded links, and offline

When you drive a robot from another network, three things decide whether a bad
link is safe: what happens when your control link goes quiet, what happens when
it is merely slow, and what still works when the internet does not. This page
answers those three for the remote plane, the internet path to a robot: one
iroh endpoint per robot, dial-by-key, reachable from boot, gated by the offline
pairing access list.

It is separate from [`docs/networking.md`](networking.md), which covers the LAN
zenoh gateway (`network:` blocks, `topic list`, mDNS).

## The deadman: a quiet operator stops the robot

Actuation is held by a control lease, and the holder must renew it inside the
deadman window. Miss the window and the robot enters its safe frame. An operator
who goes silent, for any reason, does not leave a robot moving.

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
| LAN or direct hole punch | under 1 to 5 ms | Ample. Renew every 100 ms and normal jitter never fires the deadman. |
| Relayed wide-area | 100 to 300 ms plus jitter | Tight. One stall or a jitter burst can push a renewal past the window and fire the safe frame. |

That is the intended failure: a degraded control link produces a stop, not a
runaway. Size the window for the worst latency the paths you allow teleop over
can produce.

**Teleop is allowed over relayed paths**, including relay fallback, rather than
refused. There is no relayed-path refusal: degraded latency drives a safe frame
through the deadman, never a silent runaway, and the operator is shown the live
control-path round trip and warned as it approaches the window rather than being
quietly downgraded.

## E-stop does not wait behind another session

E-stop is the permission floor. Any paired session may engage it, it is never
lease-gated, and nothing actuates while it is engaged. That has to hold at the
transport layer too, so ops sessions are served **concurrently**: one shared ops
server handles every session at once, with the receipt log behind a brief append
lock. A stalled or hostile session parks on a blocking read holding no lock, so
it cannot delay anyone else, and a reconnect loop cannot hold a permit to keep a
legitimate e-stop waiting. The safety effect runs before its audit receipt, and
the only contention inside the daemon is that brief append. Hash-chain integrity
survives: each append is one atomic critical section, so concurrent appends form
one valid tamper-evident chain.

**What this rules out is serialization, not unavailability.** Resource
exhaustion and network delivery still decide whether a remote engage-estop
arrives. Each session is bounded by a 120 s deadline that closes an
authenticated-then-stalled connection, which is a per-session resource guard
rather than a serialization point. Two caps that would bound the total footprint
under a distributed flood, a global concurrency cap with a reserved safety-floor
slot and the daemon's accept-loop cap, are **not implemented**.

## A dropped connection does not drop the lease

The lease holder is the TLS-authenticated device key, stable per pairing, not
the connection id. A link that drops and reopens inside the deadman window
re-presents the same identity and renews the same lease: no fresh grant, no
re-arbitration, no gap in actuation. An engaged e-stop is robot state that
persists across reconnects until it is explicitly cleared, and a different paired
device cannot take the floor from the one that engaged it. A drop **longer** than
the window fires the safe frame, which is correct: the operator is gone.

## It never bricks offline

On the LAN, an established pairing reaches the robot with no cloud anywhere in
the connect path: mDNS or a direct dial for discovery, a direct iroh dial with
the relay disabled for transport, and offline verification against the local
root set and the durable access list. Established pairings never expire and
never re-run the certificate chain per connect; the access row is the truth.

The issuer, which mints certificates for **new** pairings, and the relay, which
provides reach from off the LAN, are conveniences rather than dependencies. A
robot with the issuer down and the relay unreachable still serves an established
pairing on the LAN.

## See also

- [`docs/revocation.md`](revocation.md): taking an account's or one device's
  access back.
- [`docs/networking.md`](networking.md): the LAN plane, discovery and what a run
  exposes by default.
