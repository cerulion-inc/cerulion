// SPDX-License-Identifier: AGPL-3.0-only
//! Control-lease / deadman / e-stop state-machine matrix.

use cerud::constants::LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM;
use cerud::lease::{AcquireOutcome, ControlLease, EstopState, LeaseRole, LeaseViolation};

const WINDOW: u64 = 1_000; // 1000 ns window for deterministic tests

#[test]
fn acquire_free_grants_and_holder_is_visible() {
    let mut lease = ControlLease::new(WINDOW);
    assert_eq!(
        lease.acquire("op-1", LeaseRole::Operator, 0),
        AcquireOutcome::Granted
    );
    let holder = lease.holder().expect("holder is visible");
    assert_eq!(holder.session, "op-1");
    assert_eq!(holder.role, LeaseRole::Operator);
    // Snapshot exposes the same, independent of execution (Principle #3).
    let snap = lease.snapshot();
    assert_eq!(snap.holder.unwrap().session, "op-1");
    assert_eq!(snap.estop, EstopState::Clear);
}

#[test]
fn same_session_reacquire_renews() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);
    assert_eq!(
        lease.acquire("op-1", LeaseRole::Operator, 500),
        AcquireOutcome::Renewed
    );
    assert_eq!(lease.holder().unwrap().last_renew_ns, 500);
}

#[test]
fn same_session_reacquire_cannot_change_role() {
    // SECURITY: a same-session re-acquire RENEWS but must NEVER change the
    // holder's stored role — a session that already holds the lease cannot
    // silently escalate its own steal-precedence / role-gated outcomes by
    // re-acquiring with a higher role, nor de-escalate by re-acquiring lower.
    // (Reverting `acquire` to write the call's `role` on the same-session arm
    // makes both asserts below fail.)
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);

    // Re-acquire carrying a HIGHER role → Renewed, but role stays Operator.
    assert_eq!(
        lease.acquire("op-1", LeaseRole::Safety, 100),
        AcquireOutcome::Renewed
    );
    assert_eq!(
        lease.holder().unwrap().role,
        LeaseRole::Operator,
        "a same-session re-acquire must NOT escalate the stored role"
    );
    // The renew still bumped the deadman clock (the legitimate effect).
    assert_eq!(lease.holder().unwrap().last_renew_ns, 100);

    // Now hold as Supervisor (via a clean release + fresh grant) and re-acquire
    // with a LOWER role → role stays Supervisor (no de-escalation either).
    lease.release("op-1").unwrap();
    lease.acquire("sup-1", LeaseRole::Supervisor, 200);
    assert_eq!(
        lease.acquire("sup-1", LeaseRole::Operator, 300),
        AcquireOutcome::Renewed
    );
    assert_eq!(
        lease.holder().unwrap().role,
        LeaseRole::Supervisor,
        "a same-session re-acquire must NOT lower the stored role"
    );

    // The steal path (strictly-higher DIFFERENT session) is unaffected: a Safety
    // challenger still steals from the Supervisor holder.
    assert_eq!(
        lease.acquire("safety-1", LeaseRole::Safety, 400),
        AcquireOutcome::Stolen {
            from: "sup-1".to_string()
        }
    );
    assert_eq!(lease.holder().unwrap().role, LeaseRole::Safety);
}

#[test]
fn steal_by_role_matrix() {
    // A strictly HIGHER role steals.
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);
    assert_eq!(
        lease.acquire("sup-1", LeaseRole::Supervisor, 10),
        AcquireOutcome::Stolen {
            from: "op-1".to_string()
        }
    );
    assert_eq!(lease.holder().unwrap().session, "sup-1");

    // Safety steals from a Supervisor.
    assert_eq!(
        lease.acquire("safety-1", LeaseRole::Safety, 20),
        AcquireOutcome::Stolen {
            from: "sup-1".to_string()
        }
    );

    // An EQUAL role cannot steal → Busy, holder unchanged.
    assert_eq!(
        lease.acquire("safety-2", LeaseRole::Safety, 30),
        AcquireOutcome::Busy {
            held_by: "safety-1".to_string()
        }
    );
    assert_eq!(lease.holder().unwrap().session, "safety-1");

    // A LOWER role cannot steal → Busy.
    assert_eq!(
        lease.acquire("op-2", LeaseRole::Operator, 40),
        AcquireOutcome::Busy {
            held_by: "safety-1".to_string()
        }
    );
}

#[test]
fn renew_and_release_require_holding() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);

    // A non-holder cannot renew.
    assert_eq!(
        lease.renew("intruder", 100),
        Err(LeaseViolation::NotHolder {
            caller: "intruder".to_string(),
            held_by: "op-1".to_string()
        })
    );
    // The holder can.
    lease.renew("op-1", 100).unwrap();

    // A non-holder cannot release.
    assert!(matches!(
        lease.release("intruder"),
        Err(LeaseViolation::NotHolder { .. })
    ));
    // The holder can.
    lease.release("op-1").unwrap();
    assert!(lease.holder().is_none());

    // Operating on an empty lease is a NoHolder violation, never a silent no-op.
    assert_eq!(
        lease.renew("op-1", 200),
        Err(LeaseViolation::NoHolder {
            caller: "op-1".to_string()
        })
    );
}

#[test]
fn deadman_fires_once_when_the_holder_lapses() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);

    // Within the window: no deadman.
    assert!(lease.poll_deadman(WINDOW).is_none());
    assert!(lease.holder().is_some());

    // Past the window: the deadman fires, drops the holder, enters safe frame.
    let fired = lease.poll_deadman(WINDOW + 1).expect("deadman fires");
    assert_eq!(fired.lapsed_session, "op-1");
    assert!(!fired.safe_frame.description.is_empty());
    assert!(lease.holder().is_none(), "lease dropped after deadman");

    // A second poll does NOT re-fire (already fired).
    assert!(lease.poll_deadman(WINDOW + 100).is_none());
}

#[test]
fn renewing_keeps_the_deadman_at_bay() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);
    // Renew just before expiry resets the deadman clock.
    lease.renew("op-1", WINDOW).unwrap();
    assert!(lease.poll_deadman(WINDOW + WINDOW).is_none());
    // Only after a fresh window of silence does it fire.
    assert!(lease.poll_deadman(WINDOW + WINDOW + 1).is_some());
}

#[test]
fn may_actuate_requires_live_holder_and_clear_estop() {
    let mut lease = ControlLease::new(WINDOW);
    // No holder → no actuation.
    assert!(!lease.may_actuate(0));

    lease.acquire("op-1", LeaseRole::Operator, 0);
    assert!(lease.may_actuate(WINDOW)); // live holder, e-stop clear
    assert!(!lease.may_actuate(WINDOW + 1)); // holder lapsed → no actuation
}

#[test]
fn estop_always_wins_and_any_session_may_engage() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);
    assert!(lease.may_actuate(0));

    // ANY session — not the lease holder, no role gate — may engage e-stop.
    let safe = lease.engage_estop("bystander-42");
    assert!(!safe.description.is_empty());
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: "bystander-42".to_string()
        }
    );

    // E-stop dominates: no actuation even with a perfectly live lease.
    assert!(!lease.may_actuate(0));

    // Re-engaging is idempotent (keeps the original engager).
    lease.engage_estop("someone-else");
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: "bystander-42".to_string()
        }
    );

    // Clearing returns to normal — the lease is still held, so actuation
    // resumes (within the deadman window).
    lease.clear_estop();
    assert_eq!(lease.estop(), &EstopState::Clear);
    assert!(lease.may_actuate(0));
}

#[test]
fn default_window_uses_the_robot_confirm_placeholder() {
    let lease = ControlLease::with_default_window();
    assert_eq!(
        lease.snapshot().deadman_window_ns,
        LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM * 1_000_000
    );
}

#[test]
fn lease_role_rank_ordering() {
    assert!(LeaseRole::Supervisor.rank() > LeaseRole::Operator.rank());
    assert!(LeaseRole::Safety.rank() > LeaseRole::Supervisor.rank());
}

#[test]
fn lease_is_reacquirable_after_a_deadman_drop() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 0);
    // The holder lapses → deadman drops the lease.
    assert!(lease.poll_deadman(WINDOW + 1).is_some());
    assert!(lease.holder().is_none());
    // A fresh acquire after the drop is a clean grant (not Busy).
    assert_eq!(
        lease.acquire("op-2", LeaseRole::Operator, WINDOW + 2),
        AcquireOutcome::Granted
    );
    assert_eq!(lease.holder().unwrap().session, "op-2");
    assert!(lease.may_actuate(WINDOW + 2));
}

#[test]
fn lease_ops_still_function_while_estop_engaged_but_actuation_stays_blocked() {
    let mut lease = ControlLease::new(WINDOW);
    lease.engage_estop("safety-op");
    // take/steal/renew/release all continue to work under an engaged e-stop —
    // the lease bookkeeping is orthogonal to the permission floor.
    assert_eq!(
        lease.acquire("op-1", LeaseRole::Operator, 0),
        AcquireOutcome::Granted
    );
    assert_eq!(
        lease.acquire("sup-1", LeaseRole::Supervisor, 1),
        AcquireOutcome::Stolen {
            from: "op-1".to_string()
        }
    );
    lease.renew("sup-1", 2).unwrap();
    // ...but e-stop ALWAYS wins: no actuation regardless of the live lease.
    assert!(!lease.may_actuate(2));
    lease.release("sup-1").unwrap();
    assert!(lease.holder().is_none());
    // Still no actuation (e-stop engaged AND no holder).
    assert!(!lease.may_actuate(3));

    // Clearing e-stop with no holder still yields no actuation.
    lease.clear_estop();
    assert!(!lease.may_actuate(3));
}

#[test]
fn session_token_reconnect_within_the_deadman_window_renews_the_same_lease() {
    // The lease holder is a stable SESSION TOKEN, NOT the
    // ephemeral QUIC connection id. A remote operator whose WAN connection blips
    // reconnects (a brand-new QUIC connection) and re-presents the SAME per-pairing
    // session token; within the deadman window that RENEWS the same lease — no
    // fresh grant, no Busy, no re-arbitration — so a transient reconnect never
    // hands actuation to someone else or drops it.
    const TOKEN: &str = "pairing-session-7f3a"; // stable per-pairing token
    let mut lease = ControlLease::new(WINDOW);

    // Connection 1 acquires the actuation lease at t=0.
    assert_eq!(
        lease.acquire(TOKEN, LeaseRole::Operator, 0),
        AcquireOutcome::Granted
    );

    // A WAN blip drops connection 1; connection 2 (a NEW QUIC connection, the SAME
    // session token) re-acquires while still inside the deadman window → Renewed.
    let within = WINDOW; // t == WINDOW is still within the window (strictly-greater fires)
    assert_eq!(
        lease.acquire(TOKEN, LeaseRole::Operator, within),
        AcquireOutcome::Renewed
    );
    let holder = lease.holder().expect("holder survives the reconnect");
    assert_eq!(holder.session, TOKEN);
    assert_eq!(
        holder.acquired_at_ns, 0,
        "a reconnect RENEWS in place — it keeps the original acquire time"
    );
    assert_eq!(
        holder.last_renew_ns, within,
        "the reconnect reset the deadman clock"
    );
    // Actuation resumes immediately after the reconnect (live holder, e-stop clear).
    assert!(lease.may_actuate(within));

    // A reconnect AFTER the window is a FRESH grant (the old lease had lapsed): the
    // holder fell silent past the window, the deadman dropped it, so the same token
    // reconnecting later is a clean re-grant — not a renew of a lapsed lease.
    let mut lapsed = ControlLease::new(WINDOW);
    lapsed.acquire(TOKEN, LeaseRole::Operator, 0);
    assert!(
        lapsed.poll_deadman(WINDOW + 1).is_some(),
        "past the window the holder lapses and the deadman drops the lease"
    );
    assert!(lapsed.holder().is_none());
    assert_eq!(
        lapsed.acquire(TOKEN, LeaseRole::Operator, WINDOW + 2),
        AcquireOutcome::Granted,
        "a post-window reconnect is a fresh grant, not a renew"
    );
}

#[test]
fn a_different_session_token_cannot_renew_anothers_lease_on_reconnect() {
    // Anti-tautology for the reconnect-renew: the RENEW is gated on the session
    // TOKEN identity, so a DIFFERENT token (equal role) within the window is Busy —
    // it neither renews nor steals — while the original token's own reconnect still
    // renews. (Reverting `acquire`'s same-session arm to match any session would
    // make the Busy assert fail.)
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("token-A", LeaseRole::Operator, 0);
    assert_eq!(
        lease.acquire("token-B", LeaseRole::Operator, 10),
        AcquireOutcome::Busy {
            held_by: "token-A".to_string()
        }
    );
    assert_eq!(
        lease.acquire("token-A", LeaseRole::Operator, 20),
        AcquireOutcome::Renewed
    );
    assert_eq!(lease.holder().unwrap().session, "token-A");
}

#[test]
fn estop_state_persists_across_a_reconnect_and_a_different_token_cannot_take_the_floor() {
    // Lease over reconnects: e-stop is ROBOT STATE keyed on the
    // stable per-pairing session token (the paired device key), NOT the ephemeral
    // connection. A reconnect from the SAME token re-engages idempotently (the
    // floor persists, same engager); a DIFFERENT token's engage NEVER takes the
    // floor from the first engager. The floor only lifts on an explicit clear.
    const TOKEN_A: &str = "pairing-session-A"; // the first engager's stable token
    const TOKEN_B: &str = "pairing-session-B"; // a different paired session
    let mut lease = ControlLease::new(WINDOW);

    // Connection 1 (token A) engages the floor.
    let safe = lease.engage_estop(TOKEN_A);
    assert!(!safe.description.is_empty());
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: TOKEN_A.to_string()
        }
    );

    // Connection 1 drops; connection 2 (a NEW connection, the SAME token A)
    // re-engages → idempotent, the floor persists with the ORIGINAL engager.
    lease.engage_estop(TOKEN_A);
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: TOKEN_A.to_string()
        },
        "a reconnect from the same token keeps the floor engaged by that token"
    );

    // A DIFFERENT paired token B engaging while the floor is up does NOT change
    // the engager (idempotent-keep-first) — a different token cannot seize the
    // floor's ownership from the first engager.
    lease.engage_estop(TOKEN_B);
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: TOKEN_A.to_string()
        },
        "a different token cannot take the e-stop floor from the first engager"
    );

    // The floor persists (no reconnect clears it); only an explicit clear lifts it.
    assert!(
        !lease.may_actuate(0),
        "e-stop still dominates across reconnects"
    );
    lease.clear_estop();
    assert_eq!(lease.estop(), &EstopState::Clear);
}

#[test]
fn estop_survives_actuation_lease_churn_until_explicitly_cleared() {
    // e-stop is orthogonal to the actuation lease's reconnect/steal/deadman churn:
    // once engaged it dominates through a grant, a reconnect-renew, a steal, and a
    // deadman drop — it lifts ONLY on an explicit clear (never on a lease event).
    const TOKEN: &str = "op-token";
    let mut lease = ControlLease::new(WINDOW);

    lease.engage_estop("safety-observer");
    // A fresh actuation grant does not lift e-stop.
    assert_eq!(
        lease.acquire(TOKEN, LeaseRole::Operator, 0),
        AcquireOutcome::Granted
    );
    assert!(!lease.may_actuate(0), "e-stop dominates a live grant");
    // A reconnect-renew of the actuation lease does not lift e-stop.
    assert_eq!(
        lease.acquire(TOKEN, LeaseRole::Operator, WINDOW),
        AcquireOutcome::Renewed
    );
    assert!(
        !lease.may_actuate(WINDOW),
        "e-stop dominates a reconnect-renew"
    );
    // A higher-role steal does not lift e-stop.
    assert_eq!(
        lease.acquire("sup", LeaseRole::Supervisor, WINDOW + 1),
        AcquireOutcome::Stolen {
            from: TOKEN.to_string()
        }
    );
    assert!(!lease.may_actuate(WINDOW + 1));
    // A deadman drop of the actuation lease does not lift e-stop.
    assert!(lease.poll_deadman(WINDOW + 1 + WINDOW + 1).is_some());
    assert!(lease.holder().is_none());
    assert_eq!(
        lease.estop(),
        &EstopState::Engaged {
            by: "safety-observer".to_string()
        },
        "e-stop persists through a deadman drop of the actuation lease"
    );
    // Only an explicit clear lifts it.
    lease.clear_estop();
    assert_eq!(lease.estop(), &EstopState::Clear);
}

#[test]
fn reconnect_renew_keeps_actuation_gapless_within_the_window() {
    // The reconnect-renew (same stable token, new connection, within the deadman
    // window) keeps actuation LIVE with no gap: may_actuate holds true right up to
    // the window edge before the reconnect, the reconnect resets the deadman clock,
    // and may_actuate holds again after — the deadman never fires across the seam.
    const TOKEN: &str = "pairing-session-gapless";
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire(TOKEN, LeaseRole::Operator, 0);

    // Just before the window edge: still actuating, deadman not firing.
    assert!(lease.may_actuate(WINDOW), "live at the window edge");
    assert!(
        lease.poll_deadman(WINDOW).is_none(),
        "deadman not yet firing"
    );

    // The connection blips; a NEW connection re-presents the SAME token at the edge
    // → Renewed, deadman clock reset to `WINDOW`.
    assert_eq!(
        lease.acquire(TOKEN, LeaseRole::Operator, WINDOW),
        AcquireOutcome::Renewed
    );
    assert_eq!(lease.holder().unwrap().last_renew_ns, WINDOW);

    // After the reconnect: actuation is live through a full FRESH window, no gap.
    assert!(
        lease.may_actuate(WINDOW + WINDOW),
        "live a full window past the renew"
    );
    assert!(
        lease.poll_deadman(WINDOW + WINDOW).is_none(),
        "the deadman never fired across the reconnect seam"
    );
    // Only a full window of silence AFTER the reconnect fires it.
    assert!(lease.poll_deadman(WINDOW + WINDOW + 1).is_some());
}

#[test]
fn deadman_does_not_underflow_or_fire_on_a_backwards_clock() {
    let mut lease = ControlLease::new(WINDOW);
    lease.acquire("op-1", LeaseRole::Operator, 1_000);
    // A clock that moves BACKWARDS (now < last_renew) must not panic/underflow
    // and must NOT fire the deadman (elapsed saturates to 0, well within window).
    assert!(lease.poll_deadman(500).is_none());
    assert!(
        lease.holder().is_some(),
        "holder survives a backwards clock"
    );
    // Actuation is still permitted (0 elapsed, e-stop clear).
    assert!(lease.may_actuate(500));
    // Forward past the window from the real last_renew still fires normally.
    assert!(lease.poll_deadman(1_000 + WINDOW + 1).is_some());
}
