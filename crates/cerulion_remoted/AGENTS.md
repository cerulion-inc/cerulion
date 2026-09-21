# cerulion_remoted - agent notes (the remote-access family's scoped file)

The robot-side remote-plane daemon: ONE iroh endpoint per robot (the device
key's PUBLIC half IS the `EndpointId`; its 32-byte secret also seeds the pairing
identity), multiplexing `cerulion/wire/1` (topic frames) and `cerulion/ops/1`
(cerud verbs). This file also carries the family-wide rules
for `cerulion_pairing`, `cerulion_link`, `cerulion_wireclient`,
`cerulion_connectd`, `cerud`, `cerulion_accountd`, `cerulion-wire` (none has its
own scoped file - the root map points here).

## Invariants

- Deny-by-default: every accept is classified by `PairingAuthorizer`; the verb →
  capability table in `authorizer.rs` is the sacred security surface - an
  unclassified verb is DENIED, and a new (especially mutating) verb must be
  classified there. Pinned cell-by-cell by `authorizer_matrix_test.rs`.
- Bootstrap verbs self-gate on their own proof (chassis secret / cert chain /
  owner-signed grant / CPace witness); the pairing subject is ALWAYS the
  TLS-authenticated device key, never a client arg; validity/TTL/rollback
  checks use the robot's `RemotedClock`, never a client-supplied time.
- E-stop is the permission floor: any paired account, never lease-gated, served
  concurrently so a hostile session cannot starve it
  (`estop_starvation_test.rs`).
- Trust persistence is fail-closed: store-BEFORE-index, the live binding
  published only after the durable write succeeds. Never reorder (`trust.rs`).
- `cerulion_pairing`'s canonical signing bytes are pinned by INDEPENDENT
  hand-serializer vectors (`format_vectors_test.rs`) - a format change updates
  encoder AND oracle, deliberately. A store-format bump REFUSES old files, never
  migrates. Epoch apply is monotonic + wholesale-replace (the tombstone).
- `cerulion_link` framing (`read_frame`/`write_frame`) is NOT cancel-safe: never
  `select!` on it - one dedicated task per stream, everywhere in the family.
- connectd MIRRORS the robot protocol (no remoted dep in production builds);
  `protocol_parity_test.rs` is the no-divergence gate - change both sides.
- Peer-supplied text (robot names, error prose) is sanitized + length-bounded
  where it ENTERS an error/log (`sanitize_peer_text`), never only at render.
- Desk key files: exactly 32 raw bytes, created 0600 with `create_new`, NEVER
  overwritten. `~/.cerulion` secret writes go through
  `cerulion_cli_engine::auth::atomic_write_secret`.
- Epoch-cache resolution (file name / dir / env) is homed in
  `cerulion_pairing::verify` - ONE resolver shared by the writer and both desk
  readers; never hand-roll the path (a divergence silently stops revocations).

## Testing

- The whole family is parallel-safe: `cargo test -p <crate>` for pairing /
  link / wireclient / connectd / remoted / cerud / accountd / cerulion-wire -
  zero `#[serial]`, zero `#[ignore]`. The e2e suites are LOOPBACK iroh only
  (relay disabled, direct dial, every await bounded) - no relay, WAN, or
  multicast is ever touched.
- remoted / link cold-compile the ~390-crate iroh tree - long builds are normal.
  remoted, connectd, wireclient, link, accountd are OUT of `default-members`;
  build them with `-p` (a plain `cargo build` stays iroh-free).
- netd's WAN plane (`wan`, default-on) is covered by `crates/cerulion_netd/AGENTS.md`.

Deep reference: docs/internals/remote-access.md - read before touching the verb
table, trust persistence, the wire plane, pairing formats, or epoch delivery.
