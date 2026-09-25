# cerulion_remoted - agent notes (the remote-access family's scoped file)

The robot-side remote-plane daemon: ONE iroh endpoint per robot (the device key's
PUBLIC half IS the `EndpointId`, its 32-byte secret also seeds the pairing identity)
multiplexing `cerulion/wire/1` and `cerulion/ops/1`. It also carries the family-wide
rules for `cerulion_pairing`, `cerulion_link`, `cerulion_wireclient`,
`cerulion_connectd`, `cerud`, `cerulion_accountd` and `cerulion-wire`.


## Invariants
- Deny-by-default: every accept is classified by `PairingAuthorizer`, and the verb
  to capability table in `authorizer.rs` is the sacred security surface. An
  unclassified verb is DENIED; a new one must be classified there
  (`authorizer_matrix_test.rs` pins it cell by cell).
- Bootstrap verbs self-gate on their own proof (chassis secret, cert chain, owner
  grant, CPace witness). The pairing subject is ALWAYS the TLS-authenticated device
  key, never a client arg, and validity, TTL and rollback checks use the robot's
  `RemotedClock`.
- `pair` takes exactly ONE proof (`owner_certificate_postcard` or
  `presentation_postcard`); the versioned envelope rejects trailing bytes. Verify TLS
  key, owner, chain, scope, clocks and revocation, preserve the owner row, persist
  before binding, and refuse a narrower cert: permissions are per-account.
- `code-pair-finish` binds the SELF-account derived from the authenticated transport
  key and refuses any other value; a cloud account needs the roots-verified chain
  whose device key equals the peer key. The owner row is never preserved for a guest.
- E-stop is the permission floor: any paired account, never lease-gated, served
  concurrently so a hostile session cannot starve it (`estop_starvation_test.rs`).
  Trust persistence is fail-closed: store-BEFORE-index, the live binding published
  only after the durable write succeeds. Never reorder (`trust.rs`).
- `cerulion_pairing`'s canonical signing bytes are pinned by INDEPENDENT
  hand-serializer vectors (`format_vectors_test.rs`): a format change updates encoder
  AND oracle. A store-format bump REFUSES old files, and epoch apply is monotonic
  wholesale-replace.
- `cerulion_link` framing is NOT cancel-safe: never `select!` on it, one dedicated
  task per stream. connectd MIRRORS the robot protocol (no remoted dep in production
  builds), and `protocol_parity_test.rs` is the no-divergence gate: change both.
- Peer-supplied text is sanitized and length-bounded where it ENTERS an error or log
  (`sanitize_peer_text`), never only at render.
- Desk key files are exactly 32 raw bytes, created 0600 with `create_new`, NEVER
  overwritten; `~/.cerulion` secret writes go through `auth::atomic_write_secret`.
  Epoch-cache resolution is homed in `cerulion_pairing::verify`: ONE resolver for the
  writer and both desk readers, never hand-rolled (divergence stops revocations).
- Metadata reads the EXISTING local netd serving snapshot, its SHM namespace checked
  first; it never starts netd or queries discovery. Catalog and Schema reauthorize
  CAP_OBSERVE after async provider work (`local_schema_provider_test`).

## Testing
- Run `wire_plane_test` with `-- --test-threads=1`: under the macOS 256-file soft
  limit a parallel run hit iceoryx service/port errors. Keep this real-SHM binary
  serial in local focused gates.
- The whole family is parallel-safe: `cargo test -p <crate>` for pairing / link /
  wireclient / connectd / remoted / cerud / accountd / cerulion-wire, zero
  `#[serial]`, zero `#[ignore]`. The e2e suites are LOOPBACK iroh only (relay
  disabled, direct dial, every await bounded): no relay, WAN or multicast.
- remoted and link cold-compile the ~390-crate iroh tree, so long builds are normal.
  remoted, connectd, wireclient, link and accountd are OUT of `default-members`:
  build them with `-p`. netd's WAN plane has its own AGENTS.md.

Deep reference: docs/internals/remote-access.md, before touching the verb table,
trust persistence, the wire plane, pairing formats or epoch delivery.
