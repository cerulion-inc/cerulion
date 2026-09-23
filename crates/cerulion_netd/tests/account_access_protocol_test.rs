// SPDX-License-Identifier: AGPL-3.0-only
//! Public account IPC boundaries. The spy checks dispatch, not robot reachability.

use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::net::UnixListener;
use std::sync::Arc;
use std::time::Duration;

use cerulion_netd::account_access::{
    AccountAccessReply, AccountAccessRequest, AccountAccessResponse, AccountRobot, AccountSnapshot,
    RobotPresence, MAX_PROBE_BUDGET_MS, MAX_ROBOTS,
};
use cerulion_netd::protocol::{parse_request, Hello, Request, Response};
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdClient, NetdConfig, TopicKey};

fn probe() -> AccountAccessRequest {
    AccountAccessRequest::Probe {
        robot_id: [7; 32],
        budget_ms: 1000,
    }
}

fn snapshot() -> AccountSnapshot {
    AccountSnapshot {
        auth_account_id: "00112233-4455-6677-8899-aabbccddeeff".into(),
        pairing_account_id: [3; 32],
        device_key: [5; 32],
        owner_chain: None,
        robots: vec![AccountRobot {
            robot_id: [7; 32],
            hostname: "robot".into(),
            endpoint_key: Some([11; 32]),
        }],
    }
}

#[test]
fn snapshot_bounds_and_identity_collisions_refuse_without_banning_duplicate_labels() {
    let original = snapshot();
    assert!(original.validate().is_ok());
    let mut changed = original.clone();
    changed.robots.push(changed.robots[0].clone());
    assert_eq!(
        changed.validate().unwrap_err(),
        "account robot snapshot repeats a robot identifier"
    );
    changed.robots[1].robot_id = [13; 32];
    assert_eq!(
        changed.validate().unwrap_err(),
        "account robot snapshot assigns one endpoint to multiple robots"
    );
    changed.robots[1].endpoint_key = None;
    assert!(
        changed.validate().is_ok(),
        "the selector, not the catalog, refuses an ambiguous label"
    );
    for bad in ["", "  ", "robot\nsecond", &"r".repeat(254)] {
        changed.robots[0].hostname = bad.into();
        assert!(changed.validate().is_err());
    }
    let mut changed = original.clone();
    changed.robots = vec![original.robots[0].clone(); MAX_ROBOTS + 1];
    assert_eq!(
        changed.validate().unwrap_err(),
        "account robot snapshot exceeds the robot limit"
    );
    for chain in [vec![], vec![0; 32 * 1024 + 1]] {
        let mut changed = original.clone();
        changed.owner_chain = Some(chain);
        assert!(changed.validate().is_err());
    }
    for bad in ["", "  ", "account\0", &"a".repeat(257)] {
        let mut changed = original.clone();
        changed.auth_account_id = bad.into();
        assert!(changed.validate().is_err());
    }
}

#[test]
fn request_limits_and_version_do_not_turn_unknown_into_online() {
    for budget_ms in [0, MAX_PROBE_BUDGET_MS + 1, u64::MAX] {
        assert!(AccountAccessRequest::Probe {
            robot_id: [7; 32],
            budget_ms
        }
        .validate()
        .is_err());
    }
    for budget_ms in [1, MAX_PROBE_BUDGET_MS] {
        assert!(AccountAccessRequest::Probe {
            robot_id: [7; 32],
            budget_ms
        }
        .validate()
        .is_ok());
    }
    for topic in [
        "relative",
        "",
        "/camera\n",
        &format!("/{}", "x".repeat(1024)),
    ] {
        assert!(AccountAccessRequest::Schema {
            robot_id: [7; 32],
            topic: topic.into()
        }
        .validate()
        .is_err());
    }
    assert!(AccountAccessRequest::Schema {
        robot_id: [7; 32],
        topic: "/camera".into()
    }
    .validate()
    .is_ok());
    let request = Request::AccountAccess {
        id: 42,
        action: probe(),
    };
    assert_eq!(request.min_daemon_version(), 8);
    assert_eq!(request.id(), 42);
    assert_eq!(request.method_name(), "account_access");
    assert_eq!(parse_request(&request.to_json_line()).unwrap(), request);
    assert!(serde_json::from_str::<RobotPresence>(r#"{}"#).is_err());
    assert!(serde_json::from_str::<RobotPresence>(r#"{"state":"unknown"}"#).is_err());
    let mut encoded = serde_json::to_value(snapshot()).unwrap();
    encoded["private_key"] = serde_json::json!([1, 2, 3]);
    assert!(serde_json::from_value::<AccountSnapshot>(encoded).is_err());
}

#[test]
fn unique_response_key_and_identity_pins_reject_crossed_replies() {
    let presence = AccountAccessReply::Presence {
        robot_id: [7; 32],
        presence: RobotPresence::Online,
    };
    assert!(probe().accepts(&presence));
    assert!(!probe().accepts(&AccountAccessReply::Presence {
        robot_id: [8; 32],
        presence: RobotPresence::Online,
    }));
    assert!(!probe().accepts(&AccountAccessReply::Installed { robot_count: 0 }));
    let request = AccountAccessRequest::Install {
        snapshot: Box::new(snapshot()),
    };
    assert!(request.accepts(&AccountAccessReply::Installed { robot_count: 1 }));
    assert!(!request.accepts(&AccountAccessReply::Installed { robot_count: 0 }));
    let response = Response::AccountAccess(AccountAccessResponse {
        id: 42,
        account_access: presence,
    });
    assert_eq!(
        serde_json::from_str::<Response>(&response.to_json_line()).unwrap(),
        response
    );
    assert!(serde_json::from_str::<Response>(r#"{"id":42,"account_access":{}}"#).is_err());
}

#[test]
fn schema_response_must_echo_the_requested_topic_and_robot() {
    let request = AccountAccessRequest::Schema {
        robot_id: [7; 32],
        topic: "/camera".into(),
    };
    let reply = |robot_id, requested: &str| AccountAccessReply::Schema {
        robot_id,
        schema: cerulion_core::SchemaReply::not_found("robot", requested, "unknown schema"),
    };
    assert!(request.accepts(&reply([7; 32], "/camera")));
    assert!(!request.accepts(&reply([7; 32], "/other")));
    assert!(!request.accepts(&reply([8; 32], "/camera")));
}

#[test]
fn account_routes_are_exact_identities_and_old_daemons_must_not_treat_them_as_lan_names() {
    use cerulion_netd::account_access::{parse_robot_route, robot_route};
    let expected = "account:abababababababababababababababababababababababababababababababab";
    assert_eq!(robot_route(&[0xab; 32]), expected);
    assert_eq!(parse_robot_route(expected).unwrap(), Some([0xab; 32]));
    assert_eq!(parse_robot_route("robot").unwrap(), None);
    for invalid in [
        "account:",
        "account:ab",
        " account:ab",
        "\taccount:abababababababababababababababababababababababababababababababab",
        "account:abababababababababababababababababababababababababababababababab ",
        "account:ABABABABABABABABABABABABABABABABABABABABABABABABABABABABABABABAB",
        "account:abababababababababababababababababababababababababababababababag",
    ] {
        assert!(parse_robot_route(invalid).is_err());
    }
    for route in [
        expected,
        "account:malformed",
        " account:malformed ",
        " account:abababababababababababababababababababababababababababababababab ",
    ] {
        for request in [
            Request::Demand {
                id: 1,
                robot: route.into(),
                topic: "/data".into(),
                schema_hash: 7,
            },
            Request::Release {
                id: 1,
                robot: route.into(),
                topic: "/data".into(),
            },
            Request::QueryCatalog {
                id: 1,
                robot: Some(route.into()),
            },
            Request::QuerySchema {
                id: 1,
                robot: Some(route.into()),
                requested: "test/Reading".into(),
            },
            Request::QueryRuns {
                id: 1,
                robot: Some(route.into()),
            },
        ] {
            assert_eq!(request.min_daemon_version(), 8);
        }
    }
    assert_eq!(
        Request::Demand {
            id: 1,
            robot: "robot".into(),
            topic: "/data".into(),
            schema_hash: 7
        }
        .min_daemon_version(),
        1
    );
    assert_eq!(
        Request::QuerySchema {
            id: 1,
            robot: Some("robot".into()),
            requested: "test/Reading".into()
        }
        .min_daemon_version(),
        3
    );
}

fn schema_catalog(entries: &[(&str, &str, Option<u64>)]) -> cerulion_core::CatalogReply {
    cerulion_core::CatalogReply {
        version: 1,
        robot: "robot".into(),
        entries: entries
            .iter()
            .map(|(topic, name, hash)| cerulion_core::CatalogEntry {
                topic: (*topic).into(),
                schema_hash: *hash,
                schema_name: Some((*name).into()),
                provenance: cerulion_core::CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            })
            .collect(),
        error: None,
    }
}

#[test]
fn schema_type_selection_is_exact_and_order_independent_with_known_equal_hashes() {
    use cerulion_netd::account_access::schema_topic_for_type;
    let mut catalog = schema_catalog(&[
        ("/z", "test/Reading", Some(7)),
        ("/a", "test/Reading", Some(7)),
        ("/other", "other/Reading", Some(9)),
    ]);
    assert_eq!(
        schema_topic_for_type(&catalog, "test/Reading").unwrap(),
        Some("/a".into())
    );
    catalog.entries.reverse();
    assert_eq!(
        schema_topic_for_type(&catalog, "test/Reading").unwrap(),
        Some("/a".into())
    );
    assert_eq!(schema_topic_for_type(&catalog, "Reading").unwrap(), None);
    assert_eq!(
        schema_topic_for_type(&catalog, "missing/Reading").unwrap(),
        None
    );
    assert_eq!(
        schema_topic_for_type(
            &schema_catalog(&[("/one", "test/Reading", None)]),
            "test/Reading"
        )
        .unwrap(),
        Some("/one".into())
    );
}

#[test]
fn conflicting_or_unknown_multiple_schema_hashes_refuse_with_the_topic_list() {
    use cerulion_netd::account_access::schema_topic_for_type;
    for hashes in [(Some(7), Some(9)), (None, None), (Some(7), None)] {
        let catalog = schema_catalog(&[
            ("/z", "test/Reading", hashes.0),
            ("/a", "test/Reading", hashes.1),
        ]);
        assert_eq!(
            schema_topic_for_type(&catalog, "test/Reading").unwrap_err(),
            "schema 'test/Reading' is ambiguous across topics: /a, /z"
        );
    }
    let mut catalog = schema_catalog(&[("/one", "test/Reading", Some(7))]);
    catalog.error = Some("access revoked\n\u{1b}[31mpeer-controlled text".into());
    assert_eq!(
        schema_topic_for_type(&catalog, "test/Reading").unwrap_err(),
        "robot catalog refused schema resolution"
    );
    catalog.error = None;
    catalog.version = 999;
    assert_eq!(
        schema_topic_for_type(&catalog, "test/Reading").unwrap_err(),
        "robot catalog has an unsupported version"
    );
    assert!(schema_topic_for_type(
        &schema_catalog(&[("relative", "test/Reading", Some(7))]),
        "test/Reading"
    )
    .is_err());
    assert!(schema_topic_for_type(&schema_catalog(&[]), "").is_err());
    for invalid in ["/z\nforged", &format!("/{}", "x".repeat(1024))] {
        let catalog = schema_catalog(&[
            ("/a", "test/Reading", Some(7)),
            (invalid, "test/Reading", Some(9)),
        ]);
        assert_eq!(
            schema_topic_for_type(&catalog, "test/Reading").unwrap_err(),
            "robot catalog contains an invalid schema topic"
        );
    }
    let catalog = schema_catalog(&[
        ("/a", "test/Reading", None),
        ("/b", "test/Reading", None),
        ("/c", "test/Reading", None),
        ("/d", "test/Reading", None),
        ("/e", "test/Reading", None),
        ("/f", "test/Reading", None),
        ("/g", "test/Reading", None),
        ("/h", "test/Reading", None),
        ("/i", "test/Reading", None),
        ("/j", "test/Reading", None),
    ]);
    assert_eq!(
        schema_topic_for_type(&catalog, "test/Reading").unwrap_err(),
        "schema 'test/Reading' is ambiguous across topics: /a, /b, /c, /d, /e, /f, /g, /h (and 2 more topics)"
    );
}

struct ProbeSpy;
impl MirrorPlane for ProbeSpy {
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        panic!("a presence query must never demand a topic")
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        panic!("a presence query must not own a mirror")
    }
    fn account_access(&self, action: &AccountAccessRequest) -> Result<AccountAccessReply, String> {
        assert_eq!(action, &probe());
        Ok(AccountAccessReply::Presence {
            robot_id: [7; 32],
            presence: RobotPresence::Unknown {
                reason: "no endpoint supplied".into(),
            },
        })
    }
}

#[test]
fn account_operation_crosses_real_daemon_socket_without_a_demand() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("netd.sock");
    let mut daemon =
        cerulion_netd::start(path.clone(), Arc::new(ProbeSpy), NetdConfig::default()).unwrap();
    let mut client = NetdClient::connect_existing_at(path).unwrap();
    assert_eq!(
        client.account_access_once(probe()).unwrap(),
        AccountAccessReply::Presence {
            robot_id: [7; 32],
            presence: RobotPresence::Unknown {
                reason: "no endpoint supplied".into()
            },
        }
    );
    drop(client);
    daemon.shutdown();
}

#[test]
fn older_daemon_refusal_is_local_and_names_restart() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("old.sock");
    let listener = UnixListener::bind(&path).unwrap();
    let peer = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let mut hello = Hello::new();
        hello.protocol = 7;
        writeln!(stream, "{}", hello.to_json_line()).unwrap();
        let mut unexpected = Vec::new();
        stream.read_to_end(&mut unexpected).unwrap();
        assert!(
            unexpected.is_empty(),
            "unsupported requests must not cross the socket"
        );
    });
    let mut client = NetdClient::connect_existing_at(path).unwrap();
    let error = client.account_access_once(probe()).unwrap_err().to_string();
    assert!(
        error.contains("v7") && error.contains("v8") && error.contains("restart"),
        "{error}"
    );
    drop(client);
    peer.join().unwrap();
}

#[test]
fn client_refuses_wrong_correlation_instead_of_accepting_presence() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let path = dir.path().join("peer.sock");
    let listener = UnixListener::bind(&path).unwrap();
    let peer = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        writeln!(stream, "{}", Hello::new().to_json_line()).unwrap();
        let mut line = String::new();
        BufReader::new(stream.try_clone().unwrap())
            .read_line(&mut line)
            .unwrap();
        assert_eq!(parse_request(&line).unwrap().id(), 1);
        let reply = Response::AccountAccess(AccountAccessResponse {
            id: 2,
            account_access: AccountAccessReply::Presence {
                robot_id: [7; 32],
                presence: RobotPresence::Online,
            },
        });
        writeln!(stream, "{}", reply.to_json_line()).unwrap();
    });
    let mut client = NetdClient::connect_existing_at(path).unwrap();
    assert!(client
        .account_access_once(probe())
        .unwrap_err()
        .to_string()
        .contains("correlated"));
    peer.join().unwrap();
}
