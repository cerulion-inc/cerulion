# cerulion_remoted

One endpoint: public key=EndpointId, 32-byte pairing seed; wire/2+ops/1; pairing/link/wireclient/connectd/accountd/cerud/cerulion-wire.

## Invariants
- SessionWork: tickets outlive dropped waits; drain after driver join; settled includes failures.
- Catalog/schema/demand/acts gate producer scope/origin before ports; taps/mirrors retain leases. Capabilities never override privacy; safety ops independent.
- Daemon `OpsServing::serving_holds` is required: acquire before shell/exec through
  final drain/witness. Ops/RCP/latch/owed work hold the shared fence; parent exclusive
  fences idle. Never unlink; explicit SIGTERM still runs shutdown participants.
- `PairingAuthorizer` classifies accepts/verbs; unknown denies (`authorizer_matrix_test.rs`).
  Shell/exec: owner + policy; missing file ON, invalid OFF; absent handle denies, immutable hard-off wins.
  Shared limits; bounds before work; either OFF wins.
- Bootstrap verifies chassis secret/cert chain/owner grant/CPace witness. Subject is TLS device key, never client arg; validity/TTL/rollback use RemotedClock.
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
  Trust: fail closed, store-BEFORE-index; publish binding only after durable success (`trust.rs`).
- Pairing bytes: independent hand-serializer vectors (`format_vectors_test.rs`); update encoder AND oracle. Store bumps migrate or refuse loudly (v3 -> v4 migrates); epochs monotonic wholesale-replace.
- Rows keyed (account, device); one cloud predicate `TrustStore::cloud_allowed` ->
  `KeyAccess::CloudRefused` -> `authorizer::cloud_refusal` on every plane and verb.
- link/session_io: one stream/task; never select!; cancel drops stream; bound I/O.
  connectd mirrors the robot protocol (no production remoted dep): update both,
  pinned by `protocol_parity_test.rs`.
- Sanitize/bound peer text at entry (`sanitize_peer_text`).
- Audit: private files; append before send; reserve Exit; one read/write line cap; sync checkpoints; external witness failure=Error.
- Desk keys: 32 raw bytes; `create_new`, 0600, NEVER overwrite.
  `~/.cerulion` secrets: `auth::atomic_write_secret`.
  Epoch cache: ONE `cerulion_pairing::verify` resolver for writer and both desk readers;
  never hand-roll (divergence stops revocations).
- Revocation has its OWN task/timer; assert standing revoke and eviction, never response order (`epoch_push_e2e_test.rs`).
- Metadata: namespace-checked netd snapshots or `SchemaSource`; no startup.
  Recheck CAP_OBSERVE/privacy after awaits; schema-only reads hold no leases.
- Declared imports need Reliable/send_queue, native capacity one and one live writer; observers keep best-effort/all writers; proxies carry the flag. Never re-export mirrors.
- Same-user outbound shares endpoint/runtime/node; pre-dial key/namespace gate. Metadata per-call; validation refusals keep readers/idle fence; cleanup errors fence control. Retire only outbound.

## Tests
- `wire_plane_test`: `-- --test-threads=1`; parallel tests hit macOS's 256-fd limit and iceoryx service/port errors.
- Family: parallel (`cargo test -p <crate>`, no `#[ignore]`); direct
  `TransportManager::get_or_init()` callers are `#[serial]`;
  e2e: loopback iroh and relays only, bounded awaits.
- remoted/link cold-compile iroh; remoted/connectd/wireclient/link/accountd need `-p`.

Before verb/trust/wire/pairing/epoch edits: docs/internals/remote-access.md.
