// SPDX-License-Identifier: AGPL-3.0-only
//! Verb-name constants the [`crate::authorizer::PairingAuthorizer`] classifies.
//!
//! The ops verbs (`inventory` / `log-tail` / `restart`) MIRROR cerud's
//! registered handler names verbatim — pinned by an anti-drift test against
//! cerud's real handlers (`authorizer_matrix_test.rs`). The bootstrap (`claim`
//! / `pair` / `code-pair-start` / `code-pair-finish`) and e-stop
//! (`engage-estop`) verbs are the remote-plane handlers registered
//! in `crate::pairing_verbs`; the authorizer classifies them as the pre-authz
//! bootstrap + safety-floor set. `deploy` / `teleop` are FORWARD classifications
//! (not yet registered in cerud) so a future mutating/actuation verb cannot ship
//! unclassified (deny-by-default).

// ── Ops verbs (mirror cerud's registered handler `name()` strings) ──────────

/// Read-only platform inventory probe (cerud `InventoryVerb`).
pub const INVENTORY: &str = "inventory";
/// Read-only tail of an allow-listed log (cerud `LogTailVerb`).
pub const LOG_TAIL: &str = "log-tail";
/// Restart a systemd unit — a mutating lifecycle op (cerud `RestartVerb`).
pub const RESTART: &str = "restart";
/// Apply a deploy bundle — a mutating lifecycle op. FORWARD (not yet a cerud
/// handler); classified now so it can never ship unclassified.
pub const DEPLOY: &str = "deploy";
/// Teleoperate / command the robot — actuation. FORWARD (not yet a cerud
/// handler); classified now.
pub const TELEOP: &str = "teleop";

// ── E-stop floor ────────────────────────────────────────────────────────────

/// Engage the e-stop permission floor. Allowed for ANY paired account
/// regardless of scope (`cerud::lease` — any paired session, always wins).
pub const ENGAGE_ESTOP: &str = "engage-estop";

// ── Bootstrap verbs (self-gating) ────────────────────

/// Physical-possession claim (chassis secret). The ONLY verb admissible on an
/// unclaimed robot.
pub const CLAIM: &str = "claim";
/// Strong-path new pairing (a signed certificate chain, verified offline).
pub const PAIR: &str = "pair";
/// A5: establish access from a DESK-CARRIED, OWNER-SIGNED access grant,
/// verified offline against the robot's own owner. Self-gating (its proof is the
/// owner's signature over the presented grant) — classified bootstrap so a not-yet-
/// paired but TLS-authed desk can present the grant its owner signed to get onto the
/// access list, exactly like `pair`. Requires a CLAIMED robot (there must be an owner
/// to verify against — an unclaimed robot admits only `claim`).
pub const PRESENT_GRANT: &str = "present-grant";
/// CPace fallback pairing — STEP 1: the guest's `msg1` → the robot's `msg2` +
/// responder confirmation tag (consumes one bounded attempt). No durable effect.
pub const CODE_PAIR_START: &str = "code-pair-start";
/// CPace fallback pairing — STEP 2: verify the initiator confirmation tag and
/// persist the code-paired access row (the durable, mutating effect).
pub const CODE_PAIR_FINISH: &str = "code-pair-finish";
