// SPDX-License-Identifier: AGPL-3.0-only
//! The SHAPE-PARITY pin between `cerulion_connectd`'s MIRRORED
//! `cerulion/wire/1` control types and the ROBOT's serve types
//! (`cerulion_remoted::wire`).
//!
//! `cerulion_connectd` MIRRORS the wire protocol instead of importing
//! `cerulion_remoted` in its PRODUCTION build (which would drag `cerud` — the
//! robot ops server — into the desk client). This dev-dependency test is the
//! no-divergence guarantee: every request/response/preamble/status/decision the
//! desk emits deserializes into the robot's type BYTE-FOR-BYTE, and vice versa.
//! A drift on EITHER side fails here loudly (never a silent wire break at runtime).
//!
//! The comparison is over `serde_json::Value` (order-independent object equality),
//! so the pin is on the JSON wire SHAPE, not on struct field declaration order.

use cerulion_connectd::protocol as desk;
use cerulion_core::transport::cerulion_q::{
    CatalogEntry, CatalogProvenance, CatalogReply, SchemaDoc, SchemaEncoding, SchemaReply,
    CATALOG_WIRE_VERSION,
};
use cerulion_remoted as robot;
use serde::{de::DeserializeOwned, Serialize};

/// Assert two serializable values produce byte-EQUIVALENT JSON (order-independent).
fn same_json<A: Serialize, B: Serialize>(a: &A, b: &B) {
    let va: serde_json::Value = serde_json::to_value(a).expect("serialize a");
    let vb: serde_json::Value = serde_json::to_value(b).expect("serialize b");
    assert_eq!(va, vb, "JSON wire shapes diverged");
}

/// Round-trip `desk_value` INTO the robot type `R` and back, asserting the JSON is
/// identical at every hop (desk→robot AND the re-serialized robot == the desk).
fn cross<D, R>(desk_value: &D)
where
    D: Serialize,
    R: Serialize + DeserializeOwned,
{
    let bytes = serde_json::to_vec(desk_value).expect("serialize desk value");
    let robot_value: R =
        serde_json::from_slice(&bytes).expect("robot type must accept the desk's bytes");
    same_json(desk_value, &robot_value);
}

/// A catalog reply carrying both a hashed and a silent (None) topic.
fn a_catalog() -> CatalogReply {
    CatalogReply {
        version: CATALOG_WIRE_VERSION,
        robot: "go2".to_string(),
        entries: vec![
            CatalogEntry {
                topic: "/imu".to_string(),
                schema_hash: Some(0xDEAD_BEEF_CAFE_F00D),
                schema_name: Some("sensor_msgs/Imu".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            },
            CatalogEntry {
                topic: "/silent".to_string(),
                schema_hash: None,
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            },
        ],
        error: None,
    }
}

/// A found schema reply (root + nested dep closure).
fn a_schema_reply() -> SchemaReply {
    SchemaReply::found(
        "go2",
        "/state",
        vec![
            SchemaDoc {
                qualified: "acme/State".to_string(),
                encoding: SchemaEncoding::Msg,
                text: "acme/Sub sub\n".to_string(),
                deps: vec!["acme/Sub".to_string()],
            },
            SchemaDoc {
                qualified: "acme/Sub".to_string(),
                encoding: SchemaEncoding::Yaml,
                text: "name: Sub\n".to_string(),
                deps: vec![],
            },
        ],
    )
}

#[test]
fn wire_request_parity() {
    // Every desk request deserializes into the robot request identically.
    cross::<_, robot::WireRequest>(&desk::WireRequest::Catalog);
    cross::<_, robot::WireRequest>(&desk::WireRequest::Demand {
        topic: "/imu".to_string(),
    });
    cross::<_, robot::WireRequest>(&desk::WireRequest::Undemand {
        topic: "/imu".to_string(),
    });
    cross::<_, robot::WireRequest>(&desk::WireRequest::Schema {
        topic: "/imu".to_string(),
    });
    cross::<_, robot::WireRequest>(&desk::WireRequest::Status);
    // The epoch push. This is the arm that MATTERS most for drift — a
    // desk whose `sync_epoch` bytes the robot cannot parse would silently stop
    // delivering revocations while every other verb kept working.
    cross::<_, robot::WireRequest>(&desk::WireRequest::SyncEpoch {
        epoch_postcard: "00ff10".to_string(),
    });

    // Reverse: a robot request deserializes into the desk request identically.
    cross::<_, desk::WireRequest>(&robot::WireRequest::Demand {
        topic: "/z".to_string(),
    });
    cross::<_, desk::WireRequest>(&robot::WireRequest::SyncEpoch {
        epoch_postcard: "abcdef".to_string(),
    });
}

#[test]
fn wire_response_parity() {
    // Catalog + Schema arms (SHARED cerulion_q payloads — cannot diverge, but
    // pinned anyway through the full WireResponse wrapper).
    cross::<_, robot::WireResponse>(&desk::WireResponse::Catalog(a_catalog()));
    cross::<_, robot::WireResponse>(&desk::WireResponse::Schema(a_schema_reply()));
    // The desk-mirrored arms.
    cross::<_, robot::WireResponse>(&desk::WireResponse::DemandAccepted {
        topic: "/imu".to_string(),
    });
    cross::<_, robot::WireResponse>(&desk::WireResponse::Undemanded {
        topic: "/imu".to_string(),
        was_demanded: true,
    });
    cross::<_, robot::WireResponse>(&desk::WireResponse::Status(desk::StatusReply {
        topics: vec![desk::TopicStatus {
            topic: "/imu".to_string(),
            wan_forwarded: 100,
            wan_dropped: 3,
            frames_seen: 103,
            hz: 99.5,
            stream_alive: true,
        }],
    }));
    cross::<_, robot::WireResponse>(&desk::WireResponse::Error {
        topic: Some("/imu".to_string()),
        message: "boom".to_string(),
    });
    cross::<_, robot::WireResponse>(&desk::WireResponse::Error {
        topic: None,
        message: "malformed".to_string(),
    });
    // BOTH healthy epoch-sync outcomes (applied + the stale no-op) must
    // cross intact — the desk distinguishes them in its breadcrumb.
    cross::<_, robot::WireResponse>(&desk::WireResponse::EpochSynced {
        epoch: 7,
        applied: true,
    });
    cross::<_, robot::WireResponse>(&desk::WireResponse::EpochSynced {
        epoch: 7,
        applied: false,
    });

    // Reverse direction: a robot response the desk must decode.
    cross::<_, desk::WireResponse>(&robot::WireResponse::DemandAccepted {
        topic: "/z".to_string(),
    });
    cross::<_, desk::WireResponse>(&robot::WireResponse::Catalog(a_catalog()));
    cross::<_, desk::WireResponse>(&robot::WireResponse::EpochSynced {
        epoch: 42,
        applied: true,
    });
}

/// BACK-COMPAT: an OLDER peer that predates a verb must FAIL to deserialize
/// it, so the robot's `handle_request` malformed-request arm answers a
/// `WireResponse::Error` — a POLICY failure the desk logs and moves past. It must NOT
/// silently deserialize into some OTHER variant (which would run the wrong verb) and
/// must NOT be an untagged/ignored no-op (which would look like a delivered epoch).
///
/// `OldWireRequest` models the pre-epoch-sync enum. If someone ever adds
/// `#[serde(other)]` or an untagged fallback to the real enum, this pin fails — which
/// is exactly the regression that would silently break revocation delivery.
#[test]
fn an_older_peer_rejects_the_sync_epoch_verb_rather_than_mis_decoding_it() {
    #[derive(serde::Deserialize)]
    #[serde(tag = "verb", rename_all = "snake_case")]
    #[allow(dead_code)] // deserialize-only fixture: fields exist to match the shape
    enum OldWireRequest {
        Catalog,
        Demand { topic: String },
        Undemand { topic: String },
        Schema { topic: String },
        Status,
    }

    // The exact bytes an epoch-sync-aware desk puts on the wire.
    let new_frame = serde_json::to_vec(&desk::WireRequest::SyncEpoch {
        epoch_postcard: "00ff".to_string(),
    })
    .unwrap();
    assert!(
        serde_json::from_slice::<OldWireRequest>(&new_frame).is_err(),
        "an older peer MUST reject the unknown verb (the robot then answers \
         WireResponse::Error), never mis-decode it into another variant"
    );

    // ANTI-TAUTOLOGY: the old enum is otherwise perfectly functional, so the failure
    // above is about the NEW verb specifically — not a broken fixture.
    assert!(serde_json::from_slice::<OldWireRequest>(
        &serde_json::to_vec(&desk::WireRequest::Status).unwrap()
    )
    .is_ok());
    assert!(serde_json::from_slice::<OldWireRequest>(
        &serde_json::to_vec(&desk::WireRequest::Demand {
            topic: "/imu".to_string()
        })
        .unwrap()
    )
    .is_ok());

    // The same holds in the RESPONSE direction: an older DESK must reject the new
    // reply rather than mistake it for another one.
    #[derive(serde::Deserialize)]
    #[serde(tag = "reply", rename_all = "snake_case")]
    #[allow(dead_code)] // deserialize-only fixture
    enum OldWireResponseTag {
        DemandAccepted { topic: String },
    }
    let new_reply = serde_json::to_vec(&robot::WireResponse::EpochSynced {
        epoch: 3,
        applied: true,
    })
    .unwrap();
    assert!(
        serde_json::from_slice::<OldWireResponseTag>(&new_reply).is_err(),
        "an older desk MUST reject the epoch_synced reply, never mis-decode it"
    );

    // ANTI-TAUTOLOGY: the old response enum is otherwise perfectly functional, so
    // the failure above is about the NEW reply specifically — not a fixture whose
    // `tag` key is misspelled (which would make the `is_err` pass vacuously).
    assert!(serde_json::from_slice::<OldWireResponseTag>(
        &serde_json::to_vec(&robot::WireResponse::DemandAccepted {
            topic: "/z".to_string()
        })
        .unwrap()
    )
    .is_ok());
}

/// An older robot's REAL refusal must classify as
/// [`EpochPushOutcome::NoSink`] ("upgrade the robot"), NEVER as `Rejected`
/// ("investigate the epoch — forged, wrong robot, skewed clock").
///
/// This is the end of the chain the fix rests on, and it uses the REAL serde error —
/// no hand-written stand-in:
///
/// 1. an older peer (`OldWireRequest`, the same fixture the test above uses)
///    fails to deserialize the EXACT bytes an epoch-sync-aware desk sends;
/// 2. the robot's `handle_request` malformed-request arm formats that error as
///    `"{UNDECODABLE_REQUEST_NEEDLE}: {e}"` — reproduced here VERBATIM;
/// 3. the desk's shared classifier must read it as `NoSink`.
///
/// The needle is OUR OWN string (a shared `cerulion_pairing` const), not serde_json's
/// prose, so this pin does not couple the desk to a third-party crate's error
/// formatting — the printed serde text is asserted only to show what a real older
/// peer actually says.
#[test]
fn an_older_robots_real_refusal_classifies_as_no_sink_not_rejected() {
    use cerulion_pairing::verify::{NO_EPOCH_SINK_NEEDLE, UNDECODABLE_REQUEST_NEEDLE};
    use cerulion_wireclient::epoch::{classify_epoch_reply, EpochPushOutcome};

    #[derive(serde::Deserialize, Debug)]
    #[serde(tag = "verb", rename_all = "snake_case")]
    #[allow(dead_code)] // deserialize-only fixture: fields exist to match the shape
    enum OldWireRequest {
        Catalog,
        Demand { topic: String },
        Undemand { topic: String },
        Schema { topic: String },
        Status,
    }

    // (1) The EXACT bytes an epoch-sync-aware desk puts on the wire, against an older peer.
    let pushed = serde_json::to_vec(&desk::WireRequest::SyncEpoch {
        epoch_postcard: "00ff10".to_string(),
    })
    .unwrap();
    let serde_err = serde_json::from_slice::<OldWireRequest>(&pushed)
        .expect_err("an older peer cannot decode the new verb");

    // What a real older robot actually says (printed so the coupling question is
    // answerable from the test output, and asserted loosely — the phrasing belongs to
    // serde_json, which is exactly why the classifier does not match on it).
    let serde_text = serde_err.to_string();
    println!("real serde_json error an older robot reports: {serde_text}");
    assert!(
        serde_text.contains("sync_epoch"),
        "sanity: the real serde error names the unknown verb: {serde_text}"
    );
    assert!(
        !serde_text.contains(NO_EPOCH_SINK_NEEDLE),
        "an older robot CANNOT emit the no-sink needle — only an epoch-sync-aware build can. \
         That is precisely why gating NoSink on it alone misclassified older robots."
    );

    // (2) The robot's malformed-request arm, reproduced verbatim.
    let reply = serde_json::to_vec(&robot::WireResponse::Error {
        topic: None,
        message: format!("{UNDECODABLE_REQUEST_NEEDLE}: {serde_err}"),
    })
    .unwrap();

    // (3) The desk's shared classifier.
    assert_eq!(
        classify_epoch_reply("older-robot", &reply),
        EpochPushOutcome::NoSink,
        "a robot that cannot DECODE the verb must be reported as NoSink (upgrade it), \
         never as a rejected epoch (investigate it)"
    );

    // ANTI-TAUTOLOGY: a genuine epoch rejection from a MODERN robot still classifies
    // as `Rejected`, so the arm above is not swallowing every error reply.
    let rejection = serde_json::to_vec(&robot::WireResponse::Error {
        topic: None,
        message: "the pushed access-list epoch was REJECTED: invalid signature on epoch".into(),
    })
    .unwrap();
    assert_eq!(
        classify_epoch_reply("modern-robot", &rejection),
        EpochPushOutcome::Rejected
    );
}

#[test]
fn stream_preamble_parity() {
    cross::<_, robot::StreamPreamble>(&desk::StreamPreamble {
        topic: "/utlidar/cloud".to_string(),
    });
    cross::<_, desk::StreamPreamble>(&robot::StreamPreamble {
        topic: "/tf".to_string(),
    });
}

#[test]
fn accept_decision_refusal_parity() {
    // The desk recognizes the robot's refusal AcceptDecision — the exact JSON the
    // daemon's skeleton path writes on an unpaired wire dial must decode on the
    // desk's mirror.
    let robot_refuse = robot::AcceptDecision::Refuse {
        reason: "wire plane refused: unpaired device key".to_string(),
    };
    cross::<_, desk::AcceptDecision>(&robot_refuse);
    // And every other decision arm the desk mirrors.
    for d in [
        robot::AcceptDecision::OpsAdmit,
        robot::AcceptDecision::OpsBootstrapOnly,
        robot::AcceptDecision::WireAdmit,
        robot::AcceptDecision::UnknownAlpn {
            alpn: b"cerulion/wire/1".to_vec(),
        },
    ] {
        cross::<_, desk::AcceptDecision>(&d);
    }

    // The headline: the robot's real refusal bytes flow through the desk's
    // `decode_first_reply` as an explicit refusal (NOT a wire response).
    let bytes = serde_json::to_vec(&robot_refuse).unwrap();
    assert!(
        serde_json::from_slice::<desk::WireResponse>(&bytes).is_err(),
        "a refusal must NOT decode as a WireResponse"
    );
    match desk::decode_first_reply(&bytes) {
        Err(Some(reason)) => assert!(reason.contains("unpaired"), "reason: {reason}"),
        other => panic!("expected an explicit refusal, got {other:?}"),
    }
}
