// SPDX-License-Identifier: AGPL-3.0-only
//! Pure output oracles: account-directory evidence does not replace LAN evidence.
#![cfg(unix)]

use cerulion_cli_engine::account_robot_access::{render_rows, AccountRobotStatus};
use cerulion_cli_engine::account_robots::{AccountRobot, RobotEndpoint};
use cerulion_cli_engine::topic_cmd::{
    render_remote_evidence_without_discovery, render_remote_topics_section,
    render_remote_topics_section_with_mirrors,
    render_remote_topics_section_with_mirrors_and_robot_rows, MirrorStreamRow, RemoteDiscovery,
    RobotProvenance, RobotRow,
};
use cerulion_netd::account_access::RobotPresence;
use cerulion_pairing::format::RobotId;

const ACCOUNT_ROW: &str = concat!(
    "  shared  (account directory)  unknown: not checked\u{fffd}yet  ",
    "account:0101010101010101010101010101010101010101010101010101010101010101  ",
    "endpoint=unknown\n"
);
const EMPTY_LAN: &str = concat!(
    "\nREMOTE TOPICS\n",
    "none discovered within the 500 ms gather window — no peer answered ",
    "in time. Robots may still exist: LAN scouting is on by default but a slow ",
    "or off-subnet peer can miss the window; retry, or pass ",
    "--connect tcp/<host>:7683 to reach a peer scouting cannot find\n"
);
const EMPTY_REACHABLE: &str = concat!(
    "\nREMOTE TOPICS\n",
    "none discovered within the 500 ms gather window — a reachable peer ",
    "(a given locator or a discovered robot) did not advertise a topic in time. ",
    "Robots may still exist: a slow or busy peer can miss the window (retry), and ",
    "liveliness tokens exist only while a networked publisher is alive\n"
);

fn lan_discovery() -> RemoteDiscovery {
    RemoteDiscovery {
        topics: vec!["/idle".into(), "/state".into()],
        robots: vec![RobotRow {
            robot: "shared".into(),
            locator: None,
            provenance: RobotProvenance::Announce,
            topic_count: 2,
        }],
        peers: Vec::new(),
        announce_entries: Vec::new(),
    }
}

fn account_rows() -> String {
    render_rows(&[AccountRobotStatus {
        robot: AccountRobot {
            robot_id: RobotId([1; 32]),
            hostname: "shared".into(),
            endpoint: RobotEndpoint::Unknown {
                reason: "no endpoint key",
            },
        },
        presence: RobotPresence::Unknown {
            reason: "not checked\nyet".into(),
        },
    }])
}

#[test]
fn empty_account_rows_preserve_authored_legacy_rendering_bytes() {
    let empty = RemoteDiscovery::empty();
    for (reachable, expected) in [(false, EMPTY_LAN), (true, EMPTY_REACHABLE)] {
        assert_eq!(
            render_remote_topics_section_with_mirrors_and_robot_rows(&empty, reachable, &[], ""),
            expected
        );
        assert_eq!(render_remote_topics_section(&empty, reachable), expected);
        assert_eq!(
            render_remote_topics_section_with_mirrors(&empty, reachable, &[]),
            expected
        );
    }
    let discovery = lan_discovery();
    let plain = "\nROBOTS\n  shared  2 topics  (announce)\n\nREMOTE TOPICS\n/idle\n/state\n";
    assert_eq!(render_remote_topics_section(&discovery, false), plain);
    assert_eq!(
        render_remote_topics_section_with_mirrors_and_robot_rows(&discovery, false, &[], ""),
        plain
    );
    let mirrors = [MirrorStreamRow {
        topic: "/state".into(),
        robot: "shared".into(),
    }];
    let streaming = concat!(
        "\nROBOTS\n  shared  2 topics  (announce)\n\nREMOTE TOPICS\n",
        "/idle\n/state  ● streaming  shared\n"
    );
    assert_eq!(
        render_remote_topics_section_with_mirrors_and_robot_rows(&discovery, false, &mirrors, ""),
        streaming
    );
    assert_eq!(
        render_remote_topics_section_with_mirrors(&discovery, false, &mirrors),
        streaming
    );
}

#[test]
fn mixed_account_and_lan_rows_keep_both_identities_under_one_heading() {
    let discovery = lan_discovery();
    let account = account_rows();
    assert_eq!(account, ACCOUNT_ROW);
    let mirrors = [MirrorStreamRow {
        topic: "/state".into(),
        robot: "shared".into(),
    }];
    for streaming in [&[][..], &mirrors[..]] {
        let rendered = render_remote_topics_section_with_mirrors_and_robot_rows(
            &discovery, false, streaming, &account,
        );
        let topics = if streaming.is_empty() {
            "\nREMOTE TOPICS\n/idle\n/state\n"
        } else {
            "\nREMOTE TOPICS\n/idle\n/state  ● streaming  shared\n"
        };
        let expected = [
            "\nROBOTS\n  shared  2 topics  (announce)\n",
            ACCOUNT_ROW,
            topics,
        ]
        .concat();
        assert_eq!(rendered, expected);
        assert_eq!(rendered.matches("\nROBOTS\n").count(), 1);
        assert_eq!(rendered.matches("\nREMOTE TOPICS\n").count(), 1);
    }
}

#[test]
fn account_only_rows_add_one_heading_without_claiming_lan_reachability() {
    let rendered = render_remote_topics_section_with_mirrors_and_robot_rows(
        &RemoteDiscovery::empty(),
        false,
        &[],
        &account_rows(),
    );
    assert_eq!(rendered, ["\nROBOTS\n", ACCOUNT_ROW, EMPTY_LAN].concat());
    assert_eq!(rendered.matches("\nROBOTS\n").count(), 1);
    assert!(!rendered.contains("  online  "));
}

#[test]
fn unavailable_or_skipped_discovery_keeps_evidence_without_an_empty_gather_claim() {
    let account = account_rows();
    assert_eq!(render_remote_evidence_without_discovery(&[], ""), "");
    assert_eq!(
        render_remote_evidence_without_discovery(&[], &account),
        ["\nROBOTS\n", ACCOUNT_ROW].concat()
    );
    let mirrors = [MirrorStreamRow {
        topic: "/state".into(),
        robot: "shared".into(),
    }];
    assert_eq!(
        render_remote_evidence_without_discovery(&mirrors, &account),
        [
            "\nROBOTS\n",
            ACCOUNT_ROW,
            "\nREMOTE TOPICS\n/state  ● streaming  shared\n"
        ]
        .concat()
    );
    assert_eq!(
        render_remote_evidence_without_discovery(&mirrors, ""),
        "\nREMOTE TOPICS\n/state  ● streaming  shared\n"
    );
}
