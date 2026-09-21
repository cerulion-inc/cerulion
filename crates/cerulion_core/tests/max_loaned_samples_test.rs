// SPDX-License-Identifier: AGPL-3.0-only
//! The publisher `max_loaned_samples`
//! knob — `TopicServiceConfig::publisher_max_loaned_samples` — pinned on BOTH
//! sides of the default over real iceoryx2.
//!
//! iceoryx2's `publisher_max_loaned_samples` defaults to 2, and the
//! windowed-loan publish path needs ~4
//! outstanding loans per adoption-enabled publisher. The pins:
//!
//! * at the DEFAULT (`None`), a 3rd simultaneous loan fails
//!   `LoanError::ExceedsMaxLoans` (and `loan_proxy` classifies that as
//!   `TransportError::LoanCapacity` — the classification matters: a
//!   match on the older `ExceedsMaxLoanedSamples` spelling,
//!   which iceoryx2 0.9.1 never renders, would map budget exhaustion to the
//!   generic `Loan` variant);
//! * with the knob at `Some(4)`, four loans succeed and the 5th fails;
//! * a released loan returns its budget slot (drop one, loan again);
//! * the choke-point guards reject `Some(0)` (a publisher whose every loan
//!   fails) and an over-[`MAX_REASONABLE_PORTS`] value loudly at creation.
//!
//! What breaks these tests:
//! * deleting the `.max_loaned_samples(..)` plumb in `finish_publisher`
//!   fails `loan_budget_of_four_admits_four_and_refuses_the_fifth` (the 3rd
//!   loan already refuses);
//! * reverting the `loan_proxy` match to `ExceedsMaxLoanedSamples` fails
//!   `default_loan_budget_admits_two_and_refuses_the_third` (the exhausted
//!   `loan_proxy` comes back as generic `Loan`, not `LoanCapacity`).
//!
//! Parallel-safe: every test owns an isolated per-instance SHM root
//! ([`TestTransport`]).

use cerulion_core::error::TransportError;
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::MAX_REASONABLE_PORTS;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

const MSL: MaxSliceLen = MaxSliceLen::const_new(256);

/// Expect a raw-loan failure and return its reason string.
fn expect_loan_refusal(
    result: cerulion_core::error::TransportResult<
        cerulion_core::transport::publisher::RawShmLoanUninit,
    >,
    what: &str,
) -> String {
    match result {
        Err(TransportError::Loan { reason, .. }) => reason,
        Err(other) => panic!("{what}: expected TransportError::Loan, got {other:?}"),
        Ok(_) => panic!("{what}: loan unexpectedly succeeded"),
    }
}

/// At the iceoryx2 default (2), two simultaneous loans hold and the third
/// refuses with `ExceedsMaxLoans`; `loan_proxy` at exhaustion classifies as
/// `LoanCapacity`; dropping a held loan frees its budget slot.
#[test]
fn default_loan_budget_admits_two_and_refuses_the_third() {
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("mls/default", MSL, 0);

    // Two outstanding raw loans (owned handles — they do not borrow the
    // publisher, so both can be held while a third is attempted).
    let mut held = Vec::new();
    for i in 0..2 {
        held.push(
            publisher
                .loan_raw_uninit(64)
                .unwrap_or_else(|e| panic!("loan {i} within the default budget must succeed: {e}")),
        );
    }

    // The 3rd simultaneous loan exceeds the default budget (2).
    let reason = expect_loan_refusal(publisher.loan_raw_uninit(64), "3rd loan at default budget");
    assert!(
        reason.contains("ExceedsMaxLoans"),
        "the refusal must name the loan-budget condition, got: {reason}"
    );

    // The classification pin: `loan_proxy` at loan-budget exhaustion
    // maps to `LoanCapacity` (a dead string match would leave it on the
    // generic `Loan` variant).
    match publisher.loan_proxy::<Vector3>() {
        Err(TransportError::LoanCapacity { .. }) => {}
        Err(other) => panic!("exhausted loan_proxy must classify as LoanCapacity, got {other:?}"),
        Ok(_) => panic!("exhausted loan_proxy unexpectedly succeeded"),
    }

    // A released loan returns its slot: drop one held loan, loan again.
    drop(held.pop());
    if let Err(e) = publisher.loan_raw_uninit(64) {
        panic!("a dropped loan must free its budget slot: {e}");
    }
}

/// With the knob at `Some(4)`, FOUR simultaneous loans hold and the fifth
/// refuses — the plumb-is-live pin (deleting the `.max_loaned_samples(..)`
/// call in `finish_publisher` fails this at the 3rd loan).
#[test]
fn loan_budget_of_four_admits_four_and_refuses_the_fifth() {
    let tt = TestTransport::new();
    let mut cfg = tt.default_topic_config();
    cfg.publisher_max_loaned_samples = Some(4);
    let mut publisher = tt
        .publisher_with_topic_config("mls/four", MSL, 0, cfg)
        .expect("publisher with loan budget 4 must create");

    let mut held = Vec::new();
    for i in 0..4 {
        held.push(
            publisher
                .loan_raw_uninit(64)
                .unwrap_or_else(|e| panic!("loan {i} within budget 4 must succeed: {e}")),
        );
    }
    let reason = expect_loan_refusal(publisher.loan_raw_uninit(64), "5th loan at budget 4");
    assert!(
        reason.contains("ExceedsMaxLoans"),
        "the refusal must name the loan-budget condition, got: {reason}"
    );
}

/// `Some(0)` is refused LOUDLY at publisher creation — a zero loan budget
/// would make every loan fail at runtime (the silent-broken class the
/// choke-point guards exist for).
#[test]
fn zero_loan_budget_is_refused_loudly_at_publisher_creation() {
    let tt = TestTransport::new();
    let mut cfg = tt.default_topic_config();
    cfg.publisher_max_loaned_samples = Some(0);
    let msg = match tt.publisher_with_topic_config("mls/zero", MSL, 0, cfg) {
        Err(e) => format!("{e}"),
        Ok(_) => panic!("a zero loan budget must be refused at creation"),
    };
    assert!(
        msg.contains("publisher_max_loaned_samples") && msg.contains(">= 1"),
        "the refusal must name the knob and the floor: {msg}"
    );
}

/// A value past [`MAX_REASONABLE_PORTS`] is refused LOUDLY at creation —
/// each unit is a slice-ceiling-sized slot in the publisher's data segment,
/// so a garbage value must not become a giant reservation failing as a bare
/// create error.
#[test]
fn oversized_loan_budget_is_refused_loudly_at_publisher_creation() {
    let tt = TestTransport::new();
    let mut cfg = tt.default_topic_config();
    cfg.publisher_max_loaned_samples = Some(MAX_REASONABLE_PORTS + 1);
    let msg = match tt.publisher_with_topic_config("mls/oversized", MSL, 0, cfg) {
        Err(e) => format!("{e}"),
        Ok(_) => panic!("an over-cap loan budget must be refused at creation"),
    };
    assert!(
        msg.contains("publisher_max_loaned_samples")
            && msg.contains(&MAX_REASONABLE_PORTS.to_string()),
        "the refusal must name the knob and the cap: {msg}"
    );
}
