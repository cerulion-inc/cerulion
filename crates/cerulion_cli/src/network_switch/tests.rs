// SPDX-License-Identifier: AGPL-3.0-only
//! The kill-switch decision for every verb, in process, against the real
//! command tree. `tests/network_switch_sweep_test.rs` runs the same table over
//! the binary.

use std::collections::BTreeSet;

use clap::Command;

use super::cases::{By, Case, Expect, Home, CASES};
use super::*;
use crate::families;

/// Every invocable path of `cmd`'s subcommands: a verb with no subcommand of
/// its own, and a verb whose subcommand is optional (it runs bare too).
fn leaves(cmd: &Command, path: &mut Vec<String>, out: &mut BTreeSet<Vec<String>>) {
    for sub in cmd.get_subcommands() {
        path.push(sub.get_name().to_string());
        if sub.get_subcommands().next().is_none() || !sub.is_subcommand_required_set() {
            out.insert(path.clone());
        }
        leaves(sub, path, out);
        path.pop();
    }
}

/// The subcommand `path` names under `cmd`.
fn find<'a>(cmd: &'a Command, path: &[&str]) -> &'a Command {
    path.iter().fold(cmd, |at, name| {
        at.find_subcommand(name)
            .unwrap_or_else(|| panic!("{path:?}: no verb {name}"))
    })
}

/// `case.argv` parsed by the binary's own command.
fn parse(case: &Case) -> clap::ArgMatches {
    families::cli_command()
        .try_get_matches_from(std::iter::once("cerulion").chain(case.argv.iter().copied()))
        .unwrap_or_else(|e| panic!("{:?} does not parse: {e}", case.argv))
}

/// The verb path `matches` selected.
fn selected(matches: &clap::ArgMatches) -> Vec<String> {
    let mut path = Vec::new();
    let mut at = matches;
    while let Some((name, sub)) = at.subcommand() {
        path.push(name.to_string());
        at = sub;
    }
    path
}

#[test]
fn the_table_names_every_verb_the_command_tree_defines() {
    let mut tree = BTreeSet::new();
    leaves(&families::cli_command(), &mut Vec::new(), &mut tree);
    let table: BTreeSet<Vec<String>> = CASES
        .iter()
        .map(|c| c.verb.iter().map(|s| s.to_string()).collect())
        .collect();
    let missing: Vec<_> = tree.difference(&table).collect();
    let extra: Vec<_> = table.difference(&tree).collect();
    assert!(
        missing.is_empty() && extra.is_empty(),
        "src/network_switch/cases.rs must name every verb, with what the network kill \
         switch does to it: verbs with no row {missing:?}, rows for no verb {extra:?}"
    );
    // Hidden verbs are verbs too: the walk saw them.
    assert!(tree.contains(&vec!["graph".to_string(), "run-worker".to_string()]));
}

#[test]
fn every_row_parses_as_its_own_verb() {
    for case in CASES {
        let matches = parse(case);
        assert_eq!(
            selected(&matches),
            case.verb,
            "{:?} selects another verb",
            case.argv
        );
    }
}

#[test]
fn every_row_states_its_outcome() {
    for case in CASES {
        match case.expect {
            Expect::Refused { says, .. } => {
                assert!(!says.is_empty(), "{:?}: no refusal text", case.argv);
            }
            // An ungated verb never answers with the gate's exit code.
            Expect::Runs { code, says } => {
                assert!(code != 7 && !says.is_empty(), "{:?}", case.argv);
            }
            Expect::NotSpawned(why) => {
                assert!(!why.is_empty(), "{:?}: no reason", case.argv);
            }
            Expect::SignIn { request, .. } => {
                assert!(
                    request.starts_with("GET /v1/") || request.starts_with("POST /v1/"),
                    "{:?}: {request}",
                    case.argv
                );
            }
            Expect::Gated => {}
        }
    }
}

#[test]
fn every_robot_flag_has_a_refused_row_and_a_local_row() {
    // A verb that gains `--robot` reaches a robot: the table must say what the
    // switch does to that form, and to the form without it.
    let cmd = families::cli_command();
    let mut tree = BTreeSet::new();
    leaves(&cmd, &mut Vec::new(), &mut tree);
    let mut robot_verbs = 0;
    for path in tree {
        let path: Vec<&str> = path.iter().map(String::as_str).collect();
        if !find(&cmd, &path)
            .get_arguments()
            .any(|a| a.get_long() == Some("robot"))
        {
            continue;
        }
        robot_verbs += 1;
        let rows: Vec<&Case> = CASES.iter().filter(|c| c.verb == path.as_slice()).collect();
        assert!(
            rows.iter()
                .any(|c| c.argv.contains(&"--robot") && matches!(c.expect, Expect::Refused { .. })),
            "{path:?} takes --robot: it needs a refused row with --robot"
        );
        assert!(
            rows.iter().any(
                |c| !c.argv.contains(&"--robot") && !matches!(c.expect, Expect::Refused { .. })
            ),
            "{path:?} takes --robot: it needs a row without --robot, which the switch does not refuse"
        );
    }
    assert!(
        robot_verbs >= 4,
        "the walk found {robot_verbs} verbs with --robot"
    );
}

/// The `CERULION_NETWORK` values a row is decided under, and whether each one
/// refuses a row refused `by`. A row its own `--network off` refuses is
/// refused under every value.
fn spellings(by: By) -> Vec<(Option<&'static str>, bool)> {
    let run_and_query = by == By::Network;
    let flag = by == By::Flag;
    vec![
        (Some("off"), true),
        (Some(" OFF "), run_and_query || flag),
        (Some("Off"), run_and_query || flag),
        // An unrecognized value fails closed under the run and query parse.
        (Some("on"), run_and_query || flag),
        (Some("typo"), run_and_query || flag),
        (None, flag),
        (Some(""), flag),
        (Some("   "), flag),
    ]
}

#[test]
fn the_decision_agrees_with_every_row_under_every_spelling() {
    let mut refused = 0;
    for case in CASES {
        let matches = parse(case);
        match case.expect {
            Expect::Refused { code, says, by } => {
                refused += 1;
                for (value, refuses) in spellings(by) {
                    let got = families::network_refusal(&matches, &SwitchEnv::network(value));
                    let what = format!("{:?} under CERULION_NETWORK={value:?}", case.argv);
                    if !refuses {
                        assert_eq!(got, None, "{what}");
                        continue;
                    }
                    let got = got.unwrap_or_else(|| panic!("{what}: not refused"));
                    assert_eq!(got.code(), code, "{what}: {}", got.line());
                    // The first fragment opens the line; the label may differ
                    // between spellings, the rest does not.
                    assert!(got.line().starts_with(says[0]), "{what}: {}", got.line());
                    for fragment in says {
                        assert!(got.line().contains(fragment), "{what}: {}", got.line());
                    }
                }
            }
            Expect::Gated | Expect::Runs { .. } | Expect::NotSpawned(_) | Expect::SignIn { .. } => {
                for (value, _) in spellings(By::Network) {
                    assert_eq!(
                        families::network_refusal(&matches, &SwitchEnv::network(value)),
                        None,
                        "{:?} is not refused under CERULION_NETWORK={value:?}",
                        case.argv
                    );
                }
                for daemon in [
                    SwitchEnv {
                        netd: Some("off".into()),
                        ..SwitchEnv::default()
                    },
                    SwitchEnv {
                        vizd: Some("off".into()),
                        ..SwitchEnv::default()
                    },
                ] {
                    assert_eq!(
                        families::network_refusal(&matches, &daemon),
                        None,
                        "{:?} under {daemon:?}",
                        case.argv
                    );
                }
            }
        }
        if matches!(case.expect, Expect::Gated) {
            assert!(
                families::needs_identity(&matches),
                "{:?} is listed as gated but the gate does not run for it",
                case.argv
            );
        }
    }
    assert!(refused >= 7, "the table lost its refused rows: {refused}");
}

#[test]
fn viz_robot_also_refuses_under_the_daemon_switches() {
    let matches = parse(
        CASES
            .iter()
            .find(|c| c.verb == ["viz"] && c.argv.contains(&"--robot"))
            .expect("a viz --robot row"),
    );
    for (env, label) in [
        (
            SwitchEnv {
                netd: Some("off".into()),
                ..SwitchEnv::default()
            },
            "disabled by CERULION_NETD_NETWORK=off; unset CERULION_NETD_NETWORK",
        ),
        (
            SwitchEnv {
                vizd: Some("off".into()),
                ..SwitchEnv::default()
            },
            "disabled by CERULION_VIZD_NETWORK=off; unset CERULION_VIZD_NETWORK",
        ),
    ] {
        let got = families::network_refusal(&matches, &env).expect("refused");
        assert_eq!(got.code(), 1);
        assert!(got.line().contains(label), "{}", got.line());
    }
    // The daemons' own parse: only the exact value `off`.
    let not_off = SwitchEnv {
        netd: Some("OFF".into()),
        vizd: Some(" off".into()),
        ..SwitchEnv::default()
    };
    assert_eq!(families::network_refusal(&matches, &not_off), None);
}

#[test]
fn a_refusal_comes_before_the_gate_and_the_gate_runs_without_one() {
    let refusal = || Some(NetworkRefusal::error("reaches the network", 2));
    let mut gate_runs = 0;
    let code = refuse_or_gate(refusal(), true, || {
        gate_runs += 1;
        None
    });
    assert_eq!(
        (code.map(crate::telemetry::exit_code_number), gate_runs),
        (Some(2), 0)
    );
    let code = refuse_or_gate(refusal(), false, || {
        gate_runs += 1;
        None
    });
    assert_eq!(
        (code.map(crate::telemetry::exit_code_number), gate_runs),
        (Some(2), 0)
    );
    let code = refuse_or_gate(None, true, || {
        gate_runs += 1;
        Some(ExitCode::from(7))
    });
    assert_eq!(
        (code.map(crate::telemetry::exit_code_number), gate_runs),
        (Some(7), 1)
    );
    let code = refuse_or_gate(None, false, || {
        gate_runs += 1;
        Some(ExitCode::from(7))
    });
    assert_eq!((code.is_none(), gate_runs), (true, 1));
}

#[test]
fn telemetry_sees_the_switch_in_the_variable_or_the_verbs_own_flag() {
    let argv = |words: &[&str]| {
        families::cli_command()
            .try_get_matches_from(std::iter::once("cerulion").chain(words.iter().copied()))
            .expect("parse")
    };
    let local = argv(&["graph", "list"]);
    for (value, on) in [
        (Some("off"), true),
        (Some(" OFF "), true),
        (Some("typo"), true),
        (None, false),
        (Some(""), false),
    ] {
        assert_eq!(
            engaged(&local, &SwitchEnv::network(value)),
            on,
            "graph list under CERULION_NETWORK={value:?}"
        );
    }
    let unset = SwitchEnv::default();
    // `--local` on a run is the same switch as its `--network off`.
    for words in [
        &["connect", "bench-arm", "--network", "off"][..],
        &["rcp", "bench-arm", "--network", "off"][..],
        &["graph", "run", "demo", "--network", "off"][..],
        &["node", "run", "Demo", "--network", "off"][..],
        &["graph", "run", "demo", "--local"][..],
        &["node", "run", "Demo", "--local"][..],
    ] {
        assert!(engaged(&argv(words), &unset), "{words:?}");
    }
    // Any other value fails closed, as `CERULION_NETWORK` does: `connect`
    // still connects on it and `rcp` refuses it, and neither sends a usage
    // event.
    for words in [
        &["connect", "bench-arm", "--network", "on"][..],
        &["rcp", "bench-arm", "--network", "on"][..],
    ] {
        assert!(engaged(&argv(words), &unset), "{words:?}");
    }
    for words in [
        &["connect", "bench-arm"][..],
        &["connect", "bench-arm", "--network", " "][..],
        &["graph", "run", "demo"][..],
        &["topic", "list", "--local"][..],
    ] {
        assert!(!engaged(&argv(words), &unset), "{words:?}");
    }
    // `--local` on any other verb selects that verb's own local scope and is
    // not the switch: the verb sends its usage event as without the flag, in
    // every build profile.
    for words in LOCAL_IS_NOT_THE_SWITCH {
        assert!(!engaged(&argv(words), &unset), "{words:?}");
    }
}

/// An invocation of each verb whose `--local` is not the run's switch.
const LOCAL_IS_NOT_THE_SWITCH: &[&[&str]] = &[
    &["topic", "echo", "/camera", "--local"],
    &["topic", "hz", "/camera", "--local"],
    &["topic", "info", "/camera", "--local"],
    &["topic", "type", "/camera", "--local"],
    &["topic", "find", "std_msgs/String", "--local"],
    &["topic", "bw", "/camera", "--local"],
    &[
        "topic",
        "pub",
        "/cmd_vel",
        "std_msgs/String",
        "{data: hi}",
        "--local",
    ],
    &["topic", "snapshot", "/camera", "--local"],
    &["doctor", "--local"],
];

#[test]
fn every_local_flag_is_the_switch_on_a_run_or_named_as_not_the_switch() {
    // A verb that gains `--local` must say whether telemetry reads it as the
    // switch: only the two runs do.
    let cmd = families::cli_command();
    let mut tree = BTreeSet::new();
    leaves(&cmd, &mut Vec::new(), &mut tree);
    let mut local_verbs = 0;
    for path in tree {
        let path: Vec<&str> = path.iter().map(String::as_str).collect();
        if !find(&cmd, &path)
            .get_arguments()
            .any(|a| a.get_id().as_str() == "local")
        {
            continue;
        }
        local_verbs += 1;
        let run = matches!(path.as_slice(), ["graph", "run"] | ["node", "run"]);
        let named = LOCAL_IS_NOT_THE_SWITCH
            .iter()
            .any(|argv| argv.len() > path.len() && argv.starts_with(&path));
        assert!(
            run || named,
            "{path:?} takes --local: add it to LOCAL_IS_NOT_THE_SWITCH, or make it a run"
        );
        assert!(!(run && named), "{path:?} is a run and named as not a run");
    }
    assert_eq!(
        local_verbs,
        LOCAL_IS_NOT_THE_SWITCH.len() + 2,
        "every named verb, and the two runs, take --local"
    );
}

#[test]
fn account_sign_in_is_listed_as_allowed_under_the_switch() {
    // The switch covers Cerulion service traffic, not signing in: each verb
    // that signs in, and the login gate's own sign-in before a local verb,
    // has a row that keeps it allowed, so refusing one fails here and in the
    // sweep.
    let signs_in = |verb: &[&str]| {
        CASES
            .iter()
            .any(|c| c.verb == verb && matches!(c.expect, Expect::SignIn { .. }))
    };
    for verb in [
        &["login"][..],
        &["logout"][..],
        &["account", "devices", "list"][..],
        &["account", "devices", "revoke"][..],
    ] {
        assert!(signs_in(verb), "{verb:?} has no sign-in row");
    }
    let gate = CASES
        .iter()
        .find(|c| {
            matches!(
                c.expect,
                Expect::SignIn {
                    home: Home::Fresh,
                    terminal: true,
                    ..
                }
            )
        })
        .expect("a local verb at a terminal on a machine that never signed in");
    assert!(
        families::needs_identity(&parse(gate)),
        "{:?}: the login gate runs for it",
        gate.argv
    );
    for case in CASES {
        if let Expect::SignIn { .. } = case.expect {
            let matches = parse(case);
            for (value, _) in spellings(By::Network) {
                assert_eq!(
                    families::network_refusal(&matches, &SwitchEnv::network(value)),
                    None,
                    "{:?} signs in, which the switch does not refuse (CERULION_NETWORK={value:?})",
                    case.argv
                );
            }
        }
    }
}

#[test]
fn telemetry_and_the_cli_read_the_switch_the_same_way() {
    for value in [
        None,
        Some(""),
        Some("   "),
        Some("off"),
        Some(" OFF "),
        Some("Off"),
        Some("on"),
        Some("typo"),
        Some("0"),
    ] {
        assert_eq!(
            cerulion_telemetry::kill_switch::engaged(value),
            cerulion_cli_engine::graph_cmd::network_env_kill_from(value.map(str::to_string))
                .is_off(),
            "CERULION_NETWORK={value:?}"
        );
    }
}

#[test]
fn every_switch_flag_says_what_the_switch_covers() {
    // `--network` on every verb that takes it, and `--local` on the two runs,
    // where it is the same switch.
    let cmd = families::cli_command();
    let mut tree = BTreeSet::new();
    leaves(&cmd, &mut Vec::new(), &mut tree);
    let mut flags = 0;
    for path in tree {
        let path: Vec<&str> = path.iter().map(String::as_str).collect();
        let run = matches!(path.as_slice(), ["graph", "run"] | ["node", "run"]);
        for arg in find(&cmd, &path).get_arguments() {
            let id = arg.get_id().as_str();
            if id != "network" && !(run && id == "local") {
                continue;
            }
            flags += 1;
            let help = arg.get_help().map(ToString::to_string).unwrap_or_default();
            let help = help.split_whitespace().collect::<Vec<_>>().join(" ");
            assert!(
                help.contains(
                    "keeps Cerulion service traffic off (robots, relays, hosted Cerulion \
                     services and telemetry); signing in to your account still works"
                ),
                "{path:?} --{id}: {help}"
            );
        }
    }
    assert_eq!(
        flags, 6,
        "connect, rcp and the two runs' --network and --local"
    );
}
