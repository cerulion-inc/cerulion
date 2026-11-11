# cerulion_remoted - agent notes (the remote-access family's scoped file)

ONE robot endpoint: device-key public half is `EndpointId`; its 32-byte secret
seeds pairing. Daemon/`RemotePlane` multiplexes wire/2 and ops/1. Family rules cover
`cerulion_{pairing,link,wireclient,connectd,accountd}`, `cerud` and `cerulion-wire`.

## Invariants
- Catalog/schema/demand/acts obey producer scope before ports; taps/mirrors retain leases. Capabilities never override privacy; safety ops are independent.
- Daemon `OpsServing::serving_holds` is required: acquire before shell/exec through
  final drain/witness. Ops/RCP/latch/owed work hold the shared fence; parent exclusive
  fences idle. Never unlink; explicit SIGTERM still runs shutdown participants.
- `PairingAuthorizer` classifies every accept; extend authorizer.rs for each new
  verb/capability. Unknown verbs are DENIED; `authorizer_matrix_test.rs` pins cells.
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
- Pairing signing bytes use independent hand-serializer vectors (`format_vectors_test.rs`): change encoder AND oracle. Store bumps migrate or refuse loudly (v3 -> v4 migrates); epoch apply is monotonic wholesale-replace.
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
- Metadata reads the EXISTING netd snapshot (namespace checked; never starts netd)
  or a host's in-process `SchemaSource`. Catalog and Schema reauthorize
  CAP_OBSERVE after async provider work (`local_schema_provider_test`).

## Testing
- Run `wire_plane_test` with `-- --test-threads=1`: under the macOS 256-file soft
  limit a parallel run hits iceoryx service/port errors.
- The family is parallel-safe (`cargo test -p <crate>`, no `#[serial]`/`#[ignore]`);
  e2e suites are LOOPBACK iroh only (relay off, direct dial, bounded awaits).
- remoted/link cold-compile iroh. remoted/connectd/wireclient/link/accountd are OUT of default-members: build with `-p`.

Deep reference: docs/internals/remote-access.md, before touching the verb table,
trust persistence, the wire plane, pairing formats or epoch delivery.
