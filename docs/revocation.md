<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Access revocation & multi-device

A robot's owner can **revoke access**, a whole granted account OR one
specific device (desk), and have that decision reach the robot so it refuses new
connections and evicts live ones. It builds on the owner-signed grants + the
per-demand authorization and mid-session eviction sweep.

## The two revocation granularities

| Granularity | What it cuts | Robot enforcement primitive |
|---|---|---|
| **Account** | An account's entire access to a robot (all its devices) | `AccessListEpoch.revoked_accounts` → `TrustStore::is_allowed` returns `None` |
| **Device** | ONE device (transport) key; the account's OTHER devices keep access | `AccessListEpoch.revoked_devices` → `TrustStore::is_device_revoked` denies at the accept gate |

Both ride the **same signed epoch** the robot applies via
`TrustStore::apply_epoch`. Device revocation is the multi-device story: an owner can
cut a compromised laptop without locking the account's phone out of the robot.

## The lifecycle end-to-end

1. **Owner revokes** (account service, owner-only):
   `POST /v1/robots/{robot_id}/revoke` with exactly one of `{account_id}` /
   `{device_key}`. The service adds the target to the robot's revocation set, bumps
   the monotonic per-robot **epoch**, and returns a fresh `SignedEpoch` (signed by
   the intermediate). `GET /v1/robots/{robot_id}/access` inspects the current state +
   re-fetches the signed epoch to sync.
2. **Sync to the robot: DESKS PUSH IT ON CONNECT** (by design):
   the signed epoch reaches the robot because every desk that dials it **delivers the
   epoch it has cached**. See [Epoch delivery](#epoch-delivery) below. The
   robot then calls `SharedTrust::apply_epoch`, which applies + persists it.
   `apply_epoch` is **monotonic**: it only accepts a strictly-newer epoch, so a
   rolled-back clock or a replayed old epoch changes nothing.
3. **Enforcement**:
   - **New connections**: the accept gate (`PairingAuthorizer::classify_accept` →
     `SharedTrust::snapshot_for_key`) reads the revocation and refuses: a revoked
     device reads `KeyAccess::DeviceRevoked`, a revoked account `NotAllowed`.
   - **Live sessions**: the revocation **sweep** re-checks the demander every
     interval; when the epoch flips the verdict it **closes the connection**,
     resetting every stream (eviction).

### The tombstone

A revoked account/device **cannot resurrect itself** by re-presenting a still-valid
old grant: the pairing establishment paths (`verify_chain`,
`verify_owner_grant`) refuse a revoked device (`RevokedDeviceByEpoch`) / account
(`RevokedByEpoch`). Only a **newer epoch that drops it** re-admits it (an explicit
owner re-admit); the revocation set is authoritative and `apply_epoch` replaces it
wholesale.

**A revoked device also loses its SIGNING authority**, not just its dialing access: a
grant signed by a revoked OWNER device (`verify_owner_grant`) or a revoked DELEGATOR
device (`verify_chain`'s delegation path) is refused; a stolen laptop whose device key
is revoked cannot keep minting grants that verify. The account stays valid (its other
devices still sign); only that device's authority dies.

## Owner self-lockout guard

`POST /v1/robots/{id}/revoke` **REFUSES** (400) revoking the robot's OWN owner
account: that would apply wholesale (`is_allowed` short-circuits to `None`) and
permanently lock the robot out, recoverable only by a physical chassis-secret factory
reset (the epoch sets are grow-only; there is no un-revoke endpoint). The owner always
retains access; to hand a robot over, use the ownership-transfer flow, not a
self-revoke. (Revoking one of the owner's own DEVICE keys is still allowed; it cuts
one desk, the account keeps access, and it is recoverable by registering a new device.)

The robot-side `apply_epoch` additionally WARNS (observability, not enforcement) if a
synced epoch ever names the robot's own claimed owner; it applies the epoch as-is (a
robot-side carve-out could silently break a legitimate future owner change; the guard
belongs at the mint).

## Epoch delivery

**Desks push; robots never poll.** A robot has no outbound cloud connection of its own,
so the parties that already talk to it are the delivery mechanism: when a desk dials a
robot, it hands over the latest epoch it has cached.

**It is UNCONDITIONAL: there is no flag, env gate or config knob to withhold it**
(by design). Revocation propagation is not something a user opts into: a
knob would let someone dial a robot while silently withholding a revocation they are
holding, which would be a footgun. With nothing cached the push is a
silent no-op, and a missing or unusable cache never denies access.

| Step | Where |
|---|---|
| An online sync fetches `GET /v1/robots/{id}/access` and caches `{ intermediate, signed_epoch }` at `~/.cerulion/epochs/<robot>.epoch` (`base64url(postcard(EpochSyncWire))`) | Studio / the account page / a CLI sync (`cerulion_cli_engine::account_cmd::sync_robot_epoch_to_cache`) |
| The desk reads that cache at dial time and hex-encodes it | `cerulion_wireclient::config::resolve_epoch_sync` |
| The desk decides what to send + what the answer meant | `cerulion_wireclient::epoch` (`prepare_epoch_push` / `classify_epoch_reply`); ONE implementation, shared |
| The desk sends it on the connection it already has, right after the catalog admission gate | **BOTH** desk paths: `cerulion-netd`'s iroh WAN plane (every dial of a WAN robot) **and** the `cerulion connect` verb (`cerulion_connectd`'s session driver) |
| The robot verifies + applies it | `cerulion_remoted`'s `sync_epoch` wire verb → `SharedTrust::apply_epoch` |

Both desk paths dial the same `cerulion/wire/1` control plane and run the same shared
substrate, so they cannot diverge in what a push does or reports. The outcome is
observable state, not just a log line:
`IrohMirrorPlane::last_push_outcome(robot)` on the WAN plane and
`ConnectSummary::epoch_push` for a `cerulion connect` session, both
`cerulion_wireclient::epoch::EpochPushOutcome`. Both are SURFACED on their production
path: netd logs the robot's carriage state on every demand (including a demand that
reuses an existing connection, where no push happens), and `cerulion connect` prints a
one-line summary at session end, so an operator learns a revocation did not land
without grepping the dial's logs.

Those two lines are levelled by `EpochPushOutcome::severity()`: **three** classes, not
two, because "the robot is current" and "this desk failed to carry a revocation" do not
partition the space:

| Severity | Level | Which outcomes | Why |
|---|---|---|---|
| `Current` | `info` | `Applied`, `AlreadyCurrent` | The robot took what this desk carried, or already had it. |
| `NothingToCarry` | `debug` | `NoCacheDir`, `NoCachedEpoch` | This desk holds nothing for this robot: a never-synced desk, or a guest desk (the account service's access endpoint is owner-only). There is no revocation being withheld, and this is a PERMANENT steady state: warning about it on every connect and every demand would be a false alarm that trains operators to ignore the real one. The shared substrate still names the exact path it searched, once, at `info`. |
| `NotCarried` | `warn` | `UnverifiedRobotIdentity`, `CacheUnreadable`, `CacheTooLarge`, `NoSink`, `Rejected`, `NotDelivered`, `TransportFailed` | This desk was (or may well have been) holding a revocation that did NOT reach the robot. The one class an operator must act on. |

### One cache, resolved identically by every party

Three independent places resolve the cache: the WRITER, the `cerulion connect` session,
and `cerulion-netd`'s WAN plane. All three go through the SAME functions in
`cerulion_pairing::verify`, because a divergence is invisible at both ends; the writer
reports "cached", the reader reports "nothing cached", and revocations silently never
travel:

| Which part | The one function |
|---|---|
| The DIRECTORY | `resolve_epoch_dir`: `CERULION_EPOCH_DIR` (a DESK-wide var, honored by every path) if set, else the `epochs/` dir NEXT TO the desk key file |
| The FILE NAME | `epoch_cache_file_name`: `<robot>.epoch`, path separators neutered |
| The two joined | `epoch_cache_path_in` |

The CLI and the network daemon resolve one epoch cache path. Each party's entry
point, and the test that pins it against the same literal path:

| Party | Production resolution | Its pin |
|---|---|---|
| The cache WRITER | `account_cmd::resolve_epoch_cache_path` (env + the well-known `~/.cerulion/desk.key`), which `sync_robot_epoch_to_cache` calls | `account_cmd::the_writer_resolves_the_shared_epoch_cache_path` |
| `cerulion connect` | `epoch::resolve_epoch_dir_from_env` + `epoch_cache_path`, from `ConnectCli::into_config` | `cerulion_wireclient::epoch::both_desk_paths_resolve_one_cache_path` |
| `cerulion-netd`'s WAN plane | `WanRegistry::from_env` → `WanRegistry::epoch_cache_path` | `cerulion_netd::wan::netd_from_env_resolves_the_shared_epoch_cache_path` |

The writer anchors on the WELL-KNOWN desk key `~/.cerulion/desk.key` (the same file
`cerulion connect` defaults `--key-file` to, and the conventional target of
`CERULION_NETD_DESK_KEY`). A deployment that keeps its desk key elsewhere must set
`CERULION_EPOCH_DIR`, which is exactly why that var is DESK-WIDE: it is the ONE knob that
keeps all three in agreement.

**Which `<robot>` keys the cache: the name the DESK knows the robot by**, the
`~/.cerulion/robots.toml` name `cerulion pair` pins (and whose name→eid binding the dial
then authenticates cryptographically), never the name the robot reports about itself in
its catalog. A peer-reported key would let the dialed robot choose which cached artifact
it is handed, e.g. another robot's signed epoch, whose revoked account/device ids are
none of its business. A desk with no pinned name for the robot it dialed (a raw `--eid`
dial, or two names pinned to one eid) pushes NOTHING and says so
(`UnverifiedRobotIdentity`); it never falls back to the peer's word to select a file.
netd is unaffected: its demand key is already the operator-configured WAN-registry name.

The cache-LOCATION check runs FIRST, so a desk with no epochs directory at all reports
`NoCacheDir` ("this desk holds nothing") rather than `UnverifiedRobotIdentity` (whose
remediation is "pair the robot"); the reason must match the situation the operator is
actually in. Two further `robots.toml` shapes also yield no desk-verified name, because
both are worse than pushing nothing: a name starting with `-` (which the sibling binary's
argument parser would read as a flag and abort the whole dial) and two names for DIFFERENT
robots whose `<robot>.epoch` files collide (by case on a case-insensitive filesystem, or
by the separator-neutering rule on any filesystem), which would hand one robot's signed
epoch to the other.

The cache is written atomically (temp + rename), so a reader only ever sees the old
artifact or the new one, never a truncated (and therefore unreadable) one.

This mirrors how owner-signed **grants** travel (one canonical wire shape in
`cerulion_pairing`, a desk-side resolver next to `resolve_owner_grant`, a loud
robot-side decode), on the plane the desk is already on: the WAN dial uses
`cerulion/wire/1`, so the push is a control verb there rather than a second
`cerulion/ops/1` connection.

**The desk is a courier, not a trust anchor.** There is no desk-side signature check.
The robot re-verifies the intermediate against its OWN root set plus the epoch's
signature, issuer, robot id, and the monotonic floor. A desk can therefore push
nothing the robot's own CA did not sign, and a **stale** push is a harmless reported
no-op (`applied: false`); an out-of-date desk can never roll a robot back.

**A revoked desk pushing its own revocation is correct and expected.** The robot has
not synced yet, so the desk is admitted; it delivers the epoch intact; the
sweep then evicts its **demanded streams** on the next tick (a connection holding no
demands is not swept; it lingers harmlessly, with every later demand denied at the
gate). Delivery does not depend on the courier still being welcome.

**Freshness does not deny access.** No cache (the normal state for a never-synced
desk, and for every guest: the access endpoint is owner-only), a corrupt cache, an
oversized cache, an older robot that does not know the verb, an unreachable account
service, or a robot that refuses the epoch: each is LOGGED, recorded as an outcome,
and the dial proceeds. The only fatal case is a control-stream transport failure
mid-push, which leaves the framing desynced and would break every later demand anyway
(recorded as `TransportFailed`; a re-dial recovers).

Two refusal CLASSES are kept DISTINCT because they call for different operator actions:

- **`NoSink`**: the robot has no usable epoch sink for this artifact, in three shapes:
  no sync sink is wired; it PREDATES the verb entirely (it cannot even decode the
  request, and answers the shared `UNDECODABLE_REQUEST_NEEDLE` marker); or it refuses
  the artifact's ENVELOPE VERSION (`EPOCH_VERSION_UNSUPPORTED_NEEDLE`; the two builds
  disagree on the epoch-sync wire shape). *Fix: upgrade/configure a build. The epoch
  itself is not implicated.*
- **`Rejected`**: the robot verified the epoch and refused it. *Fix: investigate the
  epoch: forged, issued for a different robot, or a skewed robot clock.*

Each shape keeps its OWN remediation text (pinned by test, so they cannot collapse into
one message), and every peer-supplied string is sanitized + length-bounded before it
reaches a desk log line; a robot's error text is fully peer-controlled and the desk is
where it becomes operator-visible output.

An **oversized** cached artifact is refused desk-side (`CacheTooLarge`) rather than
sent: a control frame over the peer's 16 MiB cap is not merely rejected by the robot's
reader, it consumes the length prefix and permanently desyncs its control stream, so
sending one would turn a cache problem into a dial that fails forever.

**Scope:** the push rides the **WAN (iroh) plane**, where pairing and the accept gate
live. The zenoh LAN plane has no trust store, so there is no epoch to apply there.

## Device self-service (the CLI surface)

A user manages their OWN devices from the CLI (account self-service, like
`cerulion login`):

- `cerulion account devices list`: enumerate your devices (id, kind, revoked state).
- `cerulion account devices revoke <device_id>`: revoke a lost/decommissioned desk.

Self-revoke is **account-scoped**: the account service refuses (404) a device the
caller does not own. Revoking has two effects:

1. It flips the cloud `devices.revoked` record, which **refuses any future
   re-registration** of the key: a revoked device can no longer obtain a fresh device
   cert (`register_device` → 409); the key is dead until you provision a NEW one.
2. It **fans the device into the revocation epoch of every robot you OWN** (reusing the
   `robot_revoke` epoch machinery), so those robots refuse it at their accept gate once
   they sync. The CLI reports how many owned robots were updated (`robots_updated`).

**Scope (what self-revoke does NOT reach):**

- A robot the account is only a **guest** on (granted-but-not-owned) is NOT cut by
  self-revoke; that robot's OWNER must revoke the device via
  `POST /v1/robots/{id}/revoke`. (accountd cannot enumerate robots the account merely
  has a desk-carried grant on; those grants live off-cloud.)
- Per-robot enforcement lands only once each robot **syncs** the new epoch (the offline
  gap below). The CLI output says so.

**Robot ACL management has no robot CLI verb** (by design:
robot-management flows live in Studio / the web account page, not the CLI). The
account-service endpoints + the `cerulion_cli_engine::account_cmd::{revoke_robot_access,
fetch_robot_access}` engine functions are the tested, ready seam those surfaces call.

## The Team & access page

The owner-facing surface for all of the above is the **Team & access page**, a
**hosted** web page the account service serves at `GET /team`, rendered **inside
Cerulion Studio** in an in-app webview. An owner sees their robots, each robot's
revocation state, and their own devices, and can revoke from the page.

It is hosted rather than bundled into Studio for two reasons: Studio only has to
open a webview, and access logic lives with the service that **owns** access, so
a change reaches every desk with no Studio release.

### What the page shows (and what it deliberately does not)

| Section | Source | Actions |
|---|---|---|
| **Robots you own** | `GET /v1/robots` (owner-scoped) + `GET /v1/robots/{id}/access` | revoke an account / a device key from that robot |
| **Your devices** | `GET /v1/devices` | self-revoke a desk (fans into every owned robot) |

**Grants are not listed**, and cannot be. They are **desk-carried and
signed offline**, so the account service has no grant table to enumerate (see the
scope note above). What the cloud owns, and what this page edits, is each
robot's **revocation set**. The page says so in an always-visible footer rather
than implying a completeness it does not have.

### No confirmation prompts

Revoking a device, **including the last remaining device**, carries **no
confirmation prompt** (by design). Account-keyed recovery means a
wrongly-revoked desk is re-provisioned by signing in again, so the page does not
gate the action behind a dialog. Irreversibility is stated inline (the epoch
sets are grow-only). The page's test
`team::tests::revokes_have_no_confirmation_prompt` keeps it that way.

### The credential never rides a URL

`GET /team` is **not** session-gated: a browser navigation cannot carry an
`Authorization` header, and the document itself holds **no account data**; it is
inert chrome plus the script that fetches it. Everything of value comes from the
session-authed endpoints above, each of which authenticates the bearer and enforces
owner-only on its own.

The token reaches the page over a **host handshake**:

1. The page loads and posts `{"cmd":"team_ready"}` over the host's IPC bridge.
2. Studio resolves the desk's **current** tokens from `~/.cerulion/auth.json` and
   calls `window.__cerulion_session({session_token, refresh_token, …})`.
3. The page boots. On a 401 it refreshes via `POST /v1/auth/refresh` and posts the
   **rotated** pair back (`team_session_update`) so Studio persists it: accountd's
   refresh is single-use, so the desk's stored pair must be replaced or the next
   `cerulion` cloud call would present a spent token. A failed persist is loud: the
   host acks the reason and the page raises it in a **persistent** alert bar (not a
   self-clearing toast) carrying the `cerulion login` repair. It is reported *in the
   live page* on purpose: after a successful refresh the rotated pair exists nowhere
   but that page's memory, so replacing the document to report the failure would
   destroy the desk's only live credential.

No credential is ever placed in a URL, a query string, a redirect, or an access
log. Opened in a plain browser (no host) the page renders a loud "open this from
Cerulion Studio" state, never a blank page, and never a token-paste box.

Self-containment is **enforced**, not just documented: the page inlines all CSS/JS
and is served with `Content-Security-Policy: default-src 'none'; … connect-src
'self'`, so a future edit that adds a CDN font or script is blocked at runtime
rather than silently phoning out.

## ⚠️ The OFFLINE gap (read this)

Revocation is **ONLINE-enforced**. The robot only learns of a revocation once it
**syncs the newer epoch**. Until then:

- Desk-carried, owner-signed grants are verified **offline** against the robot's
  own owner chain; a robot that is offline indefinitely will keep honoring a grant
  whose cloud-side revocation it has never seen.
- The defenses that DO hold offline are (a) the grant's own **finite expiry**
  (`not_after_ns`, honored at every access check) and (b) the **anti-rollback
  high-water floor** (a rolled-back clock cannot resurrect an expired credential).

The delivery rule settles **how** an epoch travels (desks push it on connect; see
[Epoch delivery](#epoch-delivery)), which narrows the gap considerably: a robot
syncs the moment ANY desk carrying a newer epoch dials it, including the revoked desk
itself. It does not eliminate the gap, because a robot nobody dials (or that only
never-synced/guest desks dial) still has not heard. For that residue the defenses
remain (a) the grant's own finite **expiry** and (b) the anti-rollback **high-water
floor**.

So: treat device/account revocation as **effective once the robot next syncs**, and
lean on short grant expiries for guests that must be revocable while the robot is
offline.

## Where the code lives

| Concern | Location |
|---|---|
| Epoch format (`revoked_devices`) | `cerulion_pairing::format::AccessListEpoch` |
| Trust-store apply + tombstone | `cerulion_pairing::verify::TrustStore` (`apply_epoch`, `is_device_revoked`, `verify_chain`/`verify_owner_grant`) |
| Robot-side sync seam | `cerulion_remoted::SharedTrust::apply_epoch` |
| Epoch-sync wire shape | `cerulion_pairing::verify::EpochSyncWire` (shared by both sides; versioned + strictly decoded) |
| Cache file name | `cerulion_pairing::verify::epoch_cache_file_name` (ONE convention the writer + every reader resolve), keyed by the DESK's name for the robot |
| Desk carriage | `cerulion_wireclient::config` (`resolve_epoch_sync`, `encode_epoch_cache`) |
| Cache DIRECTORY + key | `cerulion_pairing::verify` (`resolve_epoch_dir` + `EPOCH_DIR_ENV`, `epoch_cache_file_name`, `epoch_cache_path_in`): ONE resolver every desk path AND the writer share; `cerulion_wireclient::epoch` re-exports them + the single env read `resolve_epoch_dir_from_env` |
| Desk push DECISIONS | `cerulion_wireclient::epoch` (`prepare_epoch_push`, `classify_epoch_reply`, `EpochPushOutcome`), shared by both desk paths |
| Desk push on dial | `cerulion-netd`'s iroh WAN plane (`push_epoch`) **and** `cerulion connect` (`cerulion_connectd::worker::push_epoch`); both bound BOTH halves of the control round-trip, so a stalled peer can never hang the dial |
| Cache WRITER | `cerulion_cli_engine::account_cmd` (`sync_robot_epoch_to_cache` → `resolve_epoch_cache_path` → the shared `resolve_epoch_dir` + `epoch_cache_path_in`; `sync_robot_epoch_to_cache_at` is the path-injectable core), an engine seam for Studio / the account page, not a CLI verb |
| Robot-side `sync_epoch` verb | `cerulion_remoted::wire` (`WireRequest::SyncEpoch`, `WirePlane::with_epoch_sink`) |
| Accept-gate + demand enforcement | `cerulion_remoted::authorizer::PairingAuthorizer` (`KeyAccess::DeviceRevoked`) |
| Mid-session eviction sweep | `cerulion_remoted::wire::run_revocation_sweep` |
| Account service endpoints | `cerulion_accountd::api` (`/v1/robots` list, `/v1/robots/{id}/{revoke,access}`, `/v1/devices/{id}/revoke`) + `Ca::issue_epoch` |
| Team & access page (hosted) | `cerulion_accountd::team` (`GET /team`) + `crates/cerulion_accountd/assets/team.html` |
| Studio host (entry point + handshake) | `cerulion-studio` → `native/studio-shell/src/team_panel.rs` + `src/bin/shell.rs` (the third child webview) |
| CLI + engine | `cerulion_cli` (`account devices`) + `cerulion_cli_engine::account_cmd` |
