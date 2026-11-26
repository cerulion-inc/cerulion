# cerulion_remoted

One endpoint: public key=EndpointId, 32-byte pairing seed; wire/2+ops/1.

## Invariants
- SessionWork: tickets outlive dropped waits; drain after driver join; settled includes failures.
- Linux: unreaped leader pins SID; capture identity before SID. Use pidfds. Fallback rechecks SID/birth nonatomically. No containment. Probe waits/readiness/signals before spawn.
- Catalog/schema/demand/acts gate producer scope/origin before ports; acts order: e-stop, args, scope. Taps/mirrors keep leases; privacy wins over caps; safety ops independent.
- Daemon `OpsServing::serving_holds` is required: acquire before shell/exec through final drain/witness. Ops/RCP/latch/owed work hold the shared fence; parent exclusive fences idle. Never unlink; explicit SIGTERM still runs shutdown participants.
- `PairingAuthorizer` gates accepts/verbs; unknown denies (`authorizer_matrix_test.rs`). Shell/exec: owner+policy; no file ON/invalid OFF; no handle: deny; immutable hard-off wins. Settings: validate first; either OFF wins.
- Bootstrap verifies chassis secret/cert chain/owner grant/CPace witness. Subject: TLS device key, never client arg; validity/TTL/rollback use RemotedClock.
- `pair` takes ONE proof (`owner_certificate_postcard` or `presentation_postcard`); the versioned envelope rejects trailing bytes. Verify TLS key, owner, chain, scope, clocks and revocation, preserve the owner row, persist before binding, and refuse a narrower cert: permissions per-account.
- `code-pair-finish` binds the SELF-account derived from the authenticated transport key and refuses any other value; a cloud account needs the roots-verified chain whose device key equals the peer key. The owner row is never preserved for a guest.
- E-stop floor: any paired account, unleased, concurrent; unpaired cap 64 (paired exempt), 5s first frame (`estop_starvation_test.rs`). TLS: docs/remote_plane.md; paired keys Retry first. Trust: fail closed; durable store before index/binding (`trust.rs`).
- Pairing bytes: independent hand-serializer vectors (`format_vectors_test.rs`); change encoder AND oracle. Store bumps migrate or refuse loudly (v3 -> v4 migrates); epochs monotonic wholesale-replace.
- Rows keyed (account, device); one cloud predicate `TrustStore::cloud_allowed` -> `KeyAccess::CloudRefused` -> `authorizer::cloud_refusal` on every plane and verb.
- `cerulion_link` framing is NOT cancel-safe: never `select!`; one task per stream. connectd mirrors robot protocol (no production remoted dep); `protocol_parity_test.rs`: change both.
- Sanitize/bound peer text at entry (`sanitize_peer_text`).
- Audit: private regular files; append then forward; Exit reserve; shared write/read line cap; external witness.
- Desk keys: 32 raw bytes, `create_new`, 0600; NEVER overwrite. `~/.cerulion` secrets: `auth::atomic_write_secret`. Epoch-cache resolver: `cerulion_pairing::verify`, ONE for writer and both desk readers; never hand-roll (divergence stops revocations).
- Revocation sweep: OWN task/timer; assert standing revocation and eviction, never response order (`epoch_push_e2e_test.rs`).
- Metadata: namespace-checked netd snapshots or `SchemaSource`; no startup. Recheck CAP_OBSERVE/privacy after awaits; schema-only reads hold no leases.
- Declared imports need Reliable/send_queue, native capacity one and one live writer; observers keep best-effort/all writers; proxies carry the flag. Never re-export mirrors.
- Same-user outbound shares endpoint/runtime/node; pre-dial key/namespace gate. Metadata per-call; validation refusals keep readers/idle fence; cleanup errors fence control. Retire only outbound.

## Tests
- `wire_plane_test`: `-- --test-threads=1`; parallel tests hit macOS's 256-fd soft limit and iceoryx service/port errors.
- Family: parallel (`cargo test -p <crate>`, no `#[ignore]`); direct `TransportManager::get_or_init()` callers are `#[serial]`; e2e: loopback iroh and relays only, bounded awaits.
- remoted/link cold-compile iroh; remoted/connectd/wireclient/link/accountd need `-p`.

Before verb/trust/wire/pairing/epoch edits: docs/internals/remote-access.md.
