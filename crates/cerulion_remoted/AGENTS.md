# cerulion_remoted

One robot endpoint: public key = EndpointId, 32-byte secret seeds pairing; wire/2 + ops/1. Family: pairing/link/wireclient/connectd/accountd/cerud/cerulion-wire.

## Invariants
- Catalog/schema/demand/acts gate producer scope/origin before ports; taps/mirrors retain leases. Capabilities never override privacy; safety ops independent.
- Daemon `OpsServing::serving_holds` is required: acquire before shell/exec through
  final drain/witness. Ops/RCP/latch/owed work hold the shared fence; parent exclusive
  fences idle. Never unlink; explicit SIGTERM still runs shutdown participants.
- `PairingAuthorizer` classifies every accept; extend authorizer.rs for each new
  verb/capability. Unknown verbs are DENIED; `authorizer_matrix_test.rs` pins cells.
- Bootstrap verifies its proof (chassis secret/cert chain/owner grant/CPace witness). Pairing subject is TLS device key, never client arg; validity/TTL/rollback use RemotedClock.
- `pair` takes ONE proof (`owner_certificate_postcard` or
  `presentation_postcard`); the versioned envelope rejects trailing bytes. Verify TLS
  key, owner, chain, scope, clocks and revocation, preserve the owner row, persist
  before binding, and refuse a narrower cert: permissions are per-account.
- `code-pair-finish` binds the SELF-account derived from the authenticated transport
  key and refuses any other value; a cloud account needs the roots-verified chain
  whose device key equals the peer key. The owner row is never preserved for a guest.
- E-stop is the permission floor: any paired account, never lease-gated, served
  concurrently so no session starves it (`estop_starvation_test.rs`).
  TLS: docs/remote_plane.md; paired keys Retry first.
  Trust persistence is fail-closed: store-BEFORE-index, the live binding published
  only after the durable write succeeds. Never reorder (`trust.rs`).
- Pairing signing bytes use independent hand-serializer vectors (`format_vectors_test.rs`): change encoder AND oracle. Store bumps migrate or refuse loudly (v3 -> v4 migrates); epoch apply is monotonic wholesale-replace.
- Rows keyed (account, device); one cloud predicate `TrustStore::cloud_allowed` ->
  `KeyAccess::CloudRefused` -> `authorizer::cloud_refusal` on every plane and verb.
- `cerulion_link` framing is NOT cancel-safe: never `select!` on it, one dedicated
  task per stream. connectd mirrors the robot protocol (no production remoted dep);
  `protocol_parity_test.rs` gates parity: change both.
- Sanitize/bound peer text at entry (`sanitize_peer_text`).
- Desk key files are 32 raw bytes, created 0600 with `create_new`, NEVER
  overwritten; `~/.cerulion` secret writes go through `auth::atomic_write_secret`.
  Epoch-cache resolution is homed in `cerulion_pairing::verify`: ONE resolver for the
  writer and both desk readers, never hand-rolled (divergence stops revocations).
- The revocation sweep runs on its OWN task and timer: assert the revocation stands
  and the peer ends evicted, never response ordering (`epoch_push_e2e_test.rs`).
- Metadata: namespace-checked netd snapshots or `SchemaSource`; no startup.
  Recheck CAP_OBSERVE/privacy after awaits; schema-only reads hold no leases.
- Declared imports need Reliable/send_queue, native capacity one and one live writer; observers keep best-effort/all writers; proxies carry the flag. Never re-export mirrors.
- Same-user outbound shares endpoint/runtime/node; pre-dial key/namespace gate. Metadata per-call; validation refusals keep readers/idle fence; cleanup errors fence control. Retire only outbound.

## Tests
- `local_schema_provider_test` uses `-- --test-threads=1` and one private iceoryx2
  root per process via `cerulion_core::testing::IsolatedRoot`. Refcounted holders
  remove the directory after transport teardown and recreate it for later tests.
  Short per-manager prefixes keep namespaces distinct within the 108-byte
  listener socket-path budget. The library-host tests share this fixture.
- `wire_plane_test` uses `-- --test-threads=1`: parallel tests at the macOS
  256-file soft limit hit iceoryx service/port errors.
- Family: parallel (`cargo test -p <crate>`, no `#[ignore]`); direct
  `TransportManager::get_or_init()` callers are `#[serial]`;
  e2e: loopback iroh and relays only, bounded awaits.
- remoted/link cold-compile iroh; remoted/connectd/wireclient/link/accountd need `-p`.

Read docs/internals/remote-access.md before verb, trust, wire, pairing and epoch edits.
