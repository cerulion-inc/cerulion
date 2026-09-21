# The Remote Plane: Deadman, WAN Latency, and Offline Safety

This document covers the SAFETY posture of the iroh remote plane
(`cerulion_remoted` + `cerud`): the control-lease deadman window, how it
interacts with WAN round-trip latency, why remote teleop is allowed over relayed
paths, and why the e-stop channel is not serialized behind other sessions and
never bricks offline.

It is distinct from `docs/networking.md`, which covers the LAN **zenoh gateway**
(`network:` block, `topic list`, mDNS). The remote plane is the internet path:
one iroh endpoint per robot, dial-by-key, the `cerulion/ops/1` (cerud) and
`cerulion/wire/1` planes, reachable 24/7 from boot by design, auth-gated
by the offline pairing access list.

## The control-lease deadman

The `cerud` control lease (`crates/cerud/src/lease.rs`) is the single actuation-lease /
deadman / e-stop state machine. A lease holder must **renew within the deadman
window** or the deadman fires and the robot enters the safe frame (a live
operator that goes silent must not leave the robot actuating).

The deadman window is a constant:

| Constant | Value | Where |
|---|---|---|
| `LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM` | **500 ms** | `crates/cerud/src/constants.rs` |

**ROBOT_CONFIRM.** 500 ms is a **placeholder**. The final window is confirmed
on-robot at integration time: it depends on the actuator control-loop rate and
the physical stopping distance, not on the network. `cerud` owns the *permission*
floor only (who may stop); it does not emit any actuation, and the safe-frame
CONTENTS (which actuators to zero, which brakes to engage, the ramp profile) are
themselves ROBOT_CONFIRM. Do not bake 500 ms into any actuation contract without
the on-robot confirmation. (The 500 ms figure this doc cites is pinned against the
constant by `crates/cerud/tests/constants_test.rs`, so re-tuning the constant fails that
test, forcing this doc to be updated in the same change.)

## Deadman vs WAN round-trip latency

The deadman measures wall time between renewals **on the robot** (it takes an
explicit `now_ns`, so it is deterministic and replay-safe). A remote operator's
renewals travel the control path, so the effective renewal cadence the robot sees
is the operator's send cadence **plus the one-way network latency and its
jitter**. On a path with round-trip time `R` and jitter `J`, an operator sending
renewals every `T` must satisfy, at the robot:

```
T + (R/2) + J  <  deadman_window
```

for actuation to stay live. Concretely:

| Path | Typical RTT | Headroom against a 500 ms window |
|---|---|---|
| LAN / direct holepunch | < 1 to 5 ms | Ample: renew every ~100 ms and the deadman never fires under normal jitter. |
| Relayed WAN (off-LAN) | 100 to 300 ms + jitter | **Tight.** A single stall or a burst of jitter can push a renewal past the window and fire the safe frame. |

This is by design: on a relayed WAN path, one stalled renewal is close to one safe
frame away, which is the SAFE failure (an operator whose control link degraded
gets a stop, not a runaway). The window must be sized for the **worst admissible
control-path latency** of the paths teleop is allowed over.

## Teleop over relayed paths is allowed

**`CAP_TELEOP` is allowed over relayed paths**, with deadman tuning. Teleop works everywhere, including
relay-fallback connections. Safety rides:

1. the cerud deadman (tuned / widened for WAN as needed, sized for the worst
   admissible control-path latency), and
2. **loud latency indicators**: the operator sees the live control-path RTT and
   is warned as it approaches the deadman window.

There is **no relayed-path teleop refusal arm**. The remote plane does NOT reject
`CAP_TELEOP` on a relayed connection; degraded relay latency drives a safe frame
via the deadman, never a silent runaway, and the operator is shown the degraded
latency rather than being silently downgraded.

## E-stop is not serialized behind other sessions

E-stop is the **permission floor**: any paired session may engage it, it is never
lease-gated, and it always wins (no actuation while engaged). That guarantee must
hold at the transport layer too: a legitimate remote `engage-estop` must not
**queue behind other sessions, hostile or not, that are in flight**.

Serving ops sessions SERIALLY behind one permit (the receipt log as a
single hash-chained sink) would let a reconnect-loop attacker hold the permit up
to the per-session deadline and delay a legitimate e-stop. **So ops
serving is concurrent**: `cerud`'s `OpsServer` is `Send + Sync` with the receipt log
behind a brief-append `Mutex`, so one `Arc<OpsServer>` serves every session
CONCURRENTLY. A stalled/hostile session is parked on a blocking read holding no
lock, so it never delays another session. A paired `engage-estop` runs on its own
concurrent session and reaches the lease without waiting for another session: the
safety EFFECT runs BEFORE its audit receipt (e-stop is non-mutating, so no intent
receipt precedes it), and its only contention inside `cerud` is the brief receipt
append.

Serialization is what this section rules out, not unavailability. Resource
exhaustion and network delivery still affect whether a remote `engage-estop`
arrives, and the caps that would bound the first of those are listed below as
not implemented.

Hash-chain integrity is preserved: the receipt sink's `Mutex` serializes each
append as one atomic critical section, so concurrent appends still form one valid
tamper-evident chain.

Each session is still bounded by a per-session deadline (`OPS_SESSION_DEADLINE`,
120 s) that closes an authed-then-stalled connection so its blocking task ends
(a per-session resource guard, not a serialization point). A global
concurrency cap with a reserved safety-floor slot (to bound the total blocking-
task footprint under a distributed flood while keeping `engage-estop` always
admissible) is not implemented, and neither is the daemon's accept-loop concurrency cap.

Pinned by `crates/cerulion_remoted/tests/estop_starvation_test.rs`.

## Lease over reconnects

The lease holder / engager is a **stable per-pairing session token**, the
TLS-authenticated device key (`caller.id`), which is stable per pairing, **NOT
the ephemeral QUIC connection id**. So a WAN drop + reopen within the deadman
window re-presents the SAME token and RENEWS the same lease (no fresh grant, no
re-arbitration, no gap in actuation). E-stop is robot state (`EstopState::Engaged`)
that persists across reconnects until explicitly cleared: a transient reconnect
never drops the floor, and a different paired token cannot take the floor from the
first engager. A WAN drop LONGER than the deadman fires the safe frame: correct,
the operator is gone.

Pinned by `crates/cerud/tests/lease_test.rs` (pure state machine) +
`crates/cerulion_remoted/tests/estop_starvation_test.rs` (e2e over iroh).

## Never bricks offline

On the LAN, an ESTABLISHED pairing reaches the robot with **zero cloud in the
connect path**:

1. **discovery**: mDNS multicast / direct-dial (no cloud);
2. **transport**: iroh DIRECT dial via `direct_addr` + `RelayConfig::Disabled`
   (no relay);
3. **verify**: the offline `cerulion_pairing` `TrustStore` against the local root
   set + the durable access list (I/O-free; established pairings never expire and
   never re-run the chain per connect: the access row is truth).

The issuer (which mints fresh certs for NEW pairings) and the relay (off-LAN
reach) are conveniences, not dependencies: a robot with the issuer down and the
relay unreachable still serves an established pairing on the LAN. Pinned by
`crates/cerulion_remoted/tests/offline_connect_test.rs` (an established pairing connects
and runs a verb with an UNREACHABLE relay and no issuer in the path, proving the
connect path opened no cloud/issuer I/O).
