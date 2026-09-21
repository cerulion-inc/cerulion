// SPDX-License-Identifier: AGPL-3.0-only
//! An unrepresentable graph identity REFUSES the transport, it does
//! not abort the process — proven at the constructors that really build an
//! iceoryx2 node.
//!
//! # What this pins
//!
//! `TransportManager` has three doors that turn a configured identity into an
//! iceoryx2 `NodeName`, and two of them used to `.expect()` it: a graph named
//! `café`, or one whose name overruns iceoryx2's byte cap, ABORTED the process
//! with no exit code and no diagnostic. They now share ONE fallible conversion.
//!
//! This file covers the two doors that MINT A MANAGER on success —
//! `init_for_test` and the production `detached_with_config` (the multi-process
//! supervisor's data-plane node) — so each refusal is paired, in the same body,
//! with the same shape SUCCEEDING under an ASCII name. Without that pairing
//! "refuses a hostile name" is satisfied by a constructor that refuses
//! everything.
//!
//! The SINGLETON door (`TransportManager::init` — what `cerulion graph run`
//! calls) is pinned in `cerulion_core`'s own
//! `transport::node_name_refusal_tests`, together with the classifier
//! and the message. It lives there because every call it makes is a refusal that
//! returns before `get_or_init` installs anything, so it touches no iceoryx2
//! namespace — a claim that module asserts directly rather than assuming.
//!
//! Parallel-safe: every manager built here gets its own isolated iceoryx2 config
//! (per-test SHM root), and no test touches the process-global singleton. No
//! `#[serial]`.
//!
//! ```bash
//! cargo test -p cerulion_core --test node_name_refusal_iox2_test
//! ```

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::{NodeNameRefusal, TransportError};

fn config(node_name: &str) -> TransportConfig {
    TransportConfig {
        node_name: node_name.to_string(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 8,
        network: None,
    }
}

/// The identities a real graph can produce that iceoryx2 cannot name, paired
/// with the constraint each one breaks.
///
/// Every entry is a shape a YAML `name:` (or a bag declaring one) really yields:
/// `graph run` mints `cerulion_{graph}` verbatim — it does not sanitize, unlike
/// the multi-process worker path — so the user's bytes arrive here unchanged.
fn hostile_identities() -> Vec<(String, NodeNameRefusal)> {
    let max = iceoryx2::prelude::NodeName::max_len();
    let prefix = "cerulion_";
    vec![
        (
            format!("{prefix}{}", "g".repeat(max)),
            NodeNameRefusal::TooLong {
                len: prefix.len() + max,
                max,
            },
        ),
        (format!("{prefix}café"), NodeNameRefusal::Charset),
        (format!("{prefix}日本語"), NodeNameRefusal::Charset),
        (format!("{prefix}run✅"), NodeNameRefusal::Charset),
    ]
}

/// Assert one refusal completely: the typed variant, the constraint, the name it
/// quotes, and — on the length arm — the two numbers.
fn assert_refused(err: &TransportError, name: &str, expected: &NodeNameRefusal) {
    match err {
        TransportError::UnrepresentableNodeName { node_name, cause } => {
            assert_eq!(node_name, name, "the refusal must quote the offending name");
            assert_eq!(cause, expected, "wrong constraint reported for `{name}`");
        }
        other => panic!("`{name}` produced the wrong error: {other:?}"),
    }
    let msg = err.to_string();
    assert!(
        msg.contains(name),
        "the message must name the identity so an operator knows what to rename: {msg}"
    );
    assert!(
        msg.contains("rename the graph"),
        "the message must carry the remedy (project rule): {msg}"
    );
    if let NodeNameRefusal::TooLong { len, max } = expected {
        assert!(
            msg.contains(&len.to_string()) && msg.contains(&max.to_string()),
            "the length arm must carry BOTH numbers ({len} and {max}): {msg}"
        );
    }
}

/// `init_for_test` refuses every hostile identity — and still builds under an
/// ASCII one.
#[test]
fn init_for_test_refuses_an_unrepresentable_identity() {
    for (name, expected) in hostile_identities() {
        let err = TransportManager::init_for_test(
            config(&name),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .err()
        .unwrap_or_else(|| panic!("`{name}` must be REFUSED, never expected away"));
        assert_refused(&err, &name, &expected);
    }

    // ANTI-TAUTOLOGY, same constructor, same call shape: an ordinary identity
    // still yields a working manager.
    let ok = TransportManager::init_for_test(
        config("cerulion_perception"),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("an ASCII identity must still build a manager");
    assert_eq!(ok.subscriber_buffer_size(), 8);
}

/// `detached_with_config` — the PRODUCTION non-singleton door the multi-process
/// supervisor mints its data-plane node through — refuses the same shapes with
/// the same typed error.
///
/// It was already fallible, but served a bare `{e:?}` behind a generic
/// `InvalidTransportConfig`; it now converges so a supervisor and a resim
/// diagnose an unrepresentable identity identically.
#[test]
fn detached_with_config_refuses_an_unrepresentable_identity() {
    for (name, expected) in hostile_identities() {
        let err = TransportManager::detached_with_config(
            config(&name),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .err()
        .unwrap_or_else(|| panic!("`{name}` must be REFUSED, never expected away"));
        assert_refused(&err, &name, &expected);
    }

    // ANTI-TAUTOLOGY: the same call under an ASCII identity builds.
    let ok = TransportManager::detached_with_config(
        config("cerulion_supervisor_data"),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("an ASCII identity must still build a detached manager");
    assert_eq!(ok.subscriber_buffer_size(), 8);
}

/// A refusal costs NOTHING: the same isolated namespace still accepts a manager
/// afterwards.
///
/// The conversion happens before any iceoryx2 node is built, so a refused
/// constructor must not leave a half-built node, a claimed name, or any other
/// residue on the namespace it was pointed at. Reusing the SAME `Config` for the
/// refusal and the success is what makes that observable.
#[test]
fn a_refused_constructor_leaves_the_namespace_usable() {
    let ix = cerulion_core::testing::iceoryx_test_config();

    let err = TransportManager::init_for_test(config("cerulion_café"), ix.clone())
        .err()
        .expect("hostile identity must be refused");
    assert!(matches!(
        err,
        TransportError::UnrepresentableNodeName {
            cause: NodeNameRefusal::Charset,
            ..
        }
    ));

    let ok = TransportManager::init_for_test(config("cerulion_cafe"), ix)
        .expect("the same namespace must still accept a representable identity");
    assert_eq!(ok.subscriber_buffer_size(), 8);
}
