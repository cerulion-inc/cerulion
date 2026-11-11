# cerulion_remoted - agent notes (the remote-access family's scoped file)

Robot plane: ONE iroh endpoint, device public key = `EndpointId`, same secret
for pairing; multiplexes wire/2 and ops/1. This guide also covers pairing, link,
wireclient, connectd, cerud, accountd and cerulion-wire.

## Invariants
- Deny-by-default: every accept is classified by `PairingAuthorizer`, and the verb
  to capability table in `authorizer.rs` is the sacred security surface. An
  unclassified verb is DENIED (`authorizer_matrix_test.rs` pins every cell).
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
  AND oracle. A store-format bump migrates or refuses old files loudly (v3 -> v4
  migrates); epoch apply is monotonic wholesale-replace.
- Rows keyed (account, device); one cloud predicate `TrustStore::cloud_allowed` ->
  `KeyAccess::CloudRefused` -> `authorizer::cloud_refusal` on every plane and verb.
- `cerulion_link` framing is NOT cancel-safe: never `select!` on it, one dedicated
  task per stream. connectd MIRRORS the robot protocol (no remoted dep in production
  builds), and `protocol_parity_test.rs` is the no-divergence gate: change both.
- Peer text is sanitized and bounded where it ENTERS an error or log (`sanitize_peer_text`).
- Desk key files are exactly 32 raw bytes, created 0600 with `create_new`, NEVER
  overwritten; `~/.cerulion` secret writes go through `auth::atomic_write_secret`.
  Epoch-cache resolution is homed in `cerulion_pairing::verify`: ONE resolver for the
  writer and both desk readers, never hand-rolled (divergence stops revocations).
- The revocation sweep runs on its OWN task and timer: assert the revocation stands
  and the peer ends evicted, never response ordering (`epoch_push_e2e_test.rs`).
- Metadata reads namespace-checked netd snapshots (no startup) or in-process
  `SchemaSource`; reauthorize CAP_OBSERVE after provider awaits.
- Wire graph demands honor declared Reliable/send_queue; unknown policy stays
  unknown, best-effort refuses. Count refusals separately; checked producer counts.
- Same-user outbound control reuses the endpoint, runtime and transport node.
  Refuse key/namespace mismatch before dialing; retire outbound sessions only.

## Testing
- Run `wire_plane_test` with `-- --test-threads=1`: under the macOS 256-file soft
  limit a parallel run hits iceoryx service/port errors.
- The family is parallel-safe (`cargo test -p <crate>`, no `#[serial]`/`#[ignore]`);
  e2e suites are LOOPBACK iroh only (relay off, direct dial, bounded awaits).
- remoted, connectd, wireclient, link and accountd are OUT of default members;
  use `-p`. remoted/link cold-compile the iroh tree.

Deep reference: docs/internals/remote-access.md, before touching the verb table,
trust persistence, the wire plane, pairing formats or epoch delivery.
