// SPDX-License-Identifier: AGPL-3.0-only
//! Every `cerulion` verb as one invocation (two for a verb with `--robot`),
//! with what the network kill switch does to it.
//!
//! Read by two tests: `network_switch::tests` holds the table against the
//! command tree (a verb or a `--robot` flag with no row fails) and checks the
//! in-process decision for each row, and `tests/network_switch_sweep_test.rs`
//! runs each row over the real binary. Plain data, so both can include it.
//!
//! The tree check forces a row for every verb, but which [`Expect`] a row
//! carries is the author's call: a verb that reaches the network some other
//! way than `--robot` and is listed as [`Expect::Gated`] passes both tests.

/// One invocation.
pub(crate) struct Case {
    /// The verb path the invocation selects.
    pub(crate) verb: &'static [&'static str],
    /// Everything after `cerulion`.
    pub(crate) argv: &'static [&'static str],
    /// What the switch does to it.
    pub(crate) expect: Expect,
}

/// What the network kill switch does to an invocation.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Expect {
    /// It reaches a robot or a daemon over the network, so the switch refuses
    /// it before the login gate: exit `code`, a stderr line that opens with
    /// `says[0]` and holds every fragment of `says`.
    Refused {
        code: u8,
        says: &'static [&'static str],
        by: By,
    },
    /// The switch does not refuse it: it meets the login gate as without the
    /// switch.
    Gated,
    /// The switch does not refuse it, and it runs without the gate: exit
    /// `code`, with `says` in its output.
    Runs { code: i32, says: &'static str },
    /// The switch does not refuse it (or it never meets the switch's check),
    /// and the sweep does not spawn it, for the reason given.
    NotSpawned(&'static str),
}

/// Which setting engages the refusal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum By {
    /// `CERULION_NETWORK` under the run and query parse: `off` after trimming,
    /// in any case, and any other non-empty value, which fails closed.
    Network,
    /// `CERULION_NETWORK` at exactly `off`.
    ExactOff,
    /// The invocation's own `--network off`, with `CERULION_NETWORK` unset.
    Flag,
}

const ROBOT_PUB: &[&str] = &[
    "Error: --robot bench-arm reaches the robot over the network, which ",
    " disables; unset CERULION_NETWORK to call its topic-publish verb (without --robot the \
     command runs on this machine)",
];
const ROBOT_SNAPSHOT: &[&str] = &[
    "Error: --robot bench-arm reaches the robot over the network, which ",
    " disables; unset CERULION_NETWORK to call its topic-snapshot verb (without --robot the \
     command runs on this machine)",
];
const ROBOT_SERVICE: &[&str] = &[
    "Error: --robot bench-arm reaches the robot over the network, which ",
    " disables; unset CERULION_NETWORK to call its service-call verb (without --robot the \
     command runs on this machine)",
];
const VIZ_ROBOT: &[&str] = &[
    "Error: cerulion viz --robot bench-arm: remote visualization is disabled by ",
    "; unset CERULION_NETWORK to attach a robot's topics (local topics need no --robot)",
];
const PAIR: &[&str] = &[
    "Error: cerulion pair reaches the robot over the network, which ",
    " disables; unset CERULION_NETWORK to pair",
];
const CONNECT: &[&str] = &["warning: cerulion connect: network kill-switch engaged \
                            (--network off / CERULION_NETWORK=off), refusing to connect; \
                            exiting cleanly"];
const RCP: &[&str] = &[
    "Error: cerulion rcp reaches the robot over the network, which ",
    " disables; unset CERULION_NETWORK to reach the robot's rcpd",
];
const RCP_FLAG: &[&str] = &[
    "Error: cerulion rcp reaches the robot over the network, which --network off disables; \
     drop --network to reach the robot's rcpd",
];

const fn gated(verb: &'static [&'static str], argv: &'static [&'static str]) -> Case {
    Case {
        verb,
        argv,
        expect: Expect::Gated,
    }
}

/// Every row. A verb the CLI gains needs one here.
pub(crate) static CASES: &[Case] = &[
    // ---- refused: they reach a robot or a daemon over the network ----
    Case {
        verb: &["topic", "pub"],
        argv: &[
            "topic",
            "pub",
            "/cmd_vel",
            "std_msgs/String",
            "{data: hi}",
            "--robot",
            "bench-arm",
        ],
        expect: Expect::Refused {
            code: 2,
            says: ROBOT_PUB,
            by: By::Network,
        },
    },
    Case {
        verb: &["topic", "snapshot"],
        argv: &["topic", "snapshot", "/camera", "--robot", "bench-arm"],
        expect: Expect::Refused {
            code: 2,
            says: ROBOT_SNAPSHOT,
            by: By::Network,
        },
    },
    Case {
        verb: &["service", "call"],
        argv: &[
            "service",
            "call",
            "/reset",
            "std_srvs/srv/Trigger",
            "--robot",
            "bench-arm",
        ],
        expect: Expect::Refused {
            code: 2,
            says: ROBOT_SERVICE,
            by: By::Network,
        },
    },
    Case {
        verb: &["viz"],
        argv: &["viz", "/camera", "--robot", "bench-arm"],
        expect: Expect::Refused {
            code: 1,
            says: VIZ_ROBOT,
            by: By::Network,
        },
    },
    Case {
        verb: &["pair"],
        argv: &["pair", "bench-arm"],
        expect: Expect::Refused {
            code: 1,
            says: PAIR,
            by: By::Network,
        },
    },
    Case {
        verb: &["connect"],
        argv: &["connect", "bench-arm"],
        expect: Expect::Refused {
            code: 0,
            says: CONNECT,
            by: By::ExactOff,
        },
    },
    Case {
        verb: &["connect"],
        argv: &["connect", "bench-arm", "--network", "off"],
        expect: Expect::Refused {
            code: 0,
            says: CONNECT,
            by: By::Flag,
        },
    },
    Case {
        verb: &["rcp"],
        argv: &["rcp", "bench-arm"],
        expect: Expect::Refused {
            code: 2,
            says: RCP,
            by: By::Network,
        },
    },
    Case {
        verb: &["rcp"],
        argv: &["rcp", "bench-arm", "--network", "off"],
        expect: Expect::Refused {
            code: 2,
            says: RCP_FLAG,
            by: By::Flag,
        },
    },
    // ---- the same verbs without --robot are not refused ----
    gated(
        &["topic", "pub"],
        &["topic", "pub", "/cmd_vel", "std_msgs/String", "{data: hi}"],
    ),
    gated(&["topic", "snapshot"], &["topic", "snapshot", "/camera"]),
    gated(
        &["service", "call"],
        &["service", "call", "/reset", "std_srvs/srv/Trigger"],
    ),
    gated(&["viz"], &["viz", "/camera"]),
    // ---- not refused under the switch, behind the login gate ----
    gated(&["workspace", "create"], &["workspace", "create", "demo"]),
    gated(&["workspace", "init"], &["workspace", "init"]),
    gated(&["node", "create"], &["node", "create", "Demo"]),
    gated(&["node", "delete"], &["node", "delete", "Demo"]),
    gated(&["node", "modify"], &["node", "modify", "Demo"]),
    gated(&["node", "build"], &["node", "build", "Demo"]),
    gated(&["node", "stage"], &["node", "stage", "Demo"]),
    gated(&["node", "run"], &["node", "run", "Demo"]),
    gated(&["node", "list"], &["node", "list"]),
    gated(&["node", "info"], &["node", "info", "Demo"]),
    gated(&["graph", "create"], &["graph", "create", "demo"]),
    gated(&["graph", "run"], &["graph", "run", "demo"]),
    gated(&["graph", "validate"], &["graph", "validate", "demo"]),
    gated(&["graph", "list"], &["graph", "list"]),
    gated(&["graph", "levels"], &["graph", "levels", "demo"]),
    gated(&["graph", "profile"], &["graph", "profile", "demo"]),
    gated(&["graph", "partition"], &["graph", "partition", "demo"]),
    gated(&["graph", "plan"], &["graph", "plan", "demo"]),
    gated(&["graph", "pause"], &["graph", "pause", "run-1"]),
    gated(&["graph", "resume"], &["graph", "resume", "run-1"]),
    gated(&["topic", "list"], &["topic", "list"]),
    gated(&["topic", "info"], &["topic", "info", "/camera"]),
    gated(&["topic", "echo"], &["topic", "echo", "/camera"]),
    gated(&["topic", "hz"], &["topic", "hz", "/camera"]),
    gated(&["topic", "type"], &["topic", "type", "/camera"]),
    gated(&["topic", "find"], &["topic", "find", "std_msgs/String"]),
    gated(&["topic", "bw"], &["topic", "bw", "/camera"]),
    gated(
        &["ros2", "migrate"],
        &["ros2", "migrate", "--workspace", "ws"],
    ),
    gated(
        &["ros2", "attach"],
        &["ros2", "attach", "--iface", "127.0.0.1"],
    ),
    gated(
        &["account", "devices", "list"],
        &["account", "devices", "list"],
    ),
    gated(
        &["account", "devices", "revoke"],
        &["account", "devices", "revoke", "device-1"],
    ),
    gated(&["tui"], &["tui"]),
    gated(&["trace", "inspect"], &["trace", "inspect", "traces"]),
    gated(&["bag", "play"], &["bag", "play", "run.mcap"]),
    gated(&["bag", "info"], &["bag", "info", "run.mcap"]),
    gated(&["bag", "migrate"], &["bag", "migrate", "run.mcap"]),
    gated(&["bag", "record"], &["bag", "record", "/camera"]),
    gated(&["flashback"], &["flashback"]),
    gated(&["schema", "create"], &["schema", "create", "Demo"]),
    gated(&["schema", "delete"], &["schema", "delete", "Demo"]),
    gated(&["schema", "info"], &["schema", "info", "std_msgs/String"]),
    gated(&["schema", "list"], &["schema", "list"]),
    // ---- not refused under the switch, and no login gate ----
    Case {
        verb: &["logout"],
        argv: &["logout"],
        expect: Expect::Runs {
            code: 0,
            says: "not_signed_in",
        },
    },
    Case {
        verb: &["telemetry", "status"],
        argv: &["telemetry", "status"],
        expect: Expect::Runs {
            code: 0,
            says: "telemetry",
        },
    },
    Case {
        verb: &["telemetry", "on"],
        argv: &["telemetry", "on"],
        expect: Expect::Runs {
            code: 0,
            says: "telemetry",
        },
    },
    Case {
        verb: &["telemetry", "off"],
        argv: &["telemetry", "off"],
        expect: Expect::Runs {
            code: 0,
            says: "telemetry",
        },
    },
    Case {
        verb: &["completions"],
        argv: &["completions", "zsh"],
        expect: Expect::Runs {
            code: 0,
            says: "cerulion",
        },
    },
    Case {
        verb: &["clean"],
        argv: &["clean", "--report-only"],
        expect: Expect::Runs {
            code: 0,
            says: "No dead iceoryx2 nodes found",
        },
    },
    Case {
        verb: &["doctor"],
        argv: &["doctor", "--local", "--check", "net.mdns"],
        expect: Expect::Runs {
            code: 0,
            says: "net.mdns",
        },
    },
    Case {
        verb: &["ros"],
        argv: &["ros", "attach"],
        expect: Expect::Runs {
            code: 2,
            says: "`cerulion ros attach` has moved",
        },
    },
    // ---- not spawned by the sweep ----
    Case {
        verb: &["login"],
        argv: &["login"],
        expect: Expect::NotSpawned(
            "the sign-in itself: it reaches the account service, which the switch does not cover",
        ),
    },
    Case {
        verb: &["ros2", "run"],
        argv: &["ros2", "run", "demo_nodes_cpp", "talker"],
        expect: Expect::NotSpawned(
            "hands the whole command line to the user's own `ros2` before the parse",
        ),
    },
    Case {
        verb: &["ros2", "launch"],
        argv: &[
            "ros2",
            "launch",
            "demo_nodes_cpp",
            "talker_listener.launch.py",
        ],
        expect: Expect::NotSpawned(
            "hands the whole command line to the user's own `ros2` before the parse",
        ),
    },
    Case {
        verb: &["graph", "run-worker"],
        argv: &["graph", "run-worker", "--plan", "plan.json"],
        expect: Expect::NotSpawned("an internal process a gated `graph run` starts"),
    },
    Case {
        verb: &["graph", "run-gateway"],
        argv: &["graph", "run-gateway", "--handoff", "handoff.json"],
        expect: Expect::NotSpawned("an internal process a gated `graph run` starts"),
    },
    #[cfg(unix)]
    Case {
        verb: &["bagd"],
        argv: &["bagd", "--out", "run.mcap", "--topic", "/camera"],
        expect: Expect::NotSpawned("the recorder daemon, dispatched before the login gate"),
    },
    #[cfg(not(unix))]
    Case {
        verb: &["bagd"],
        argv: &["bagd"],
        expect: Expect::NotSpawned("the recorder daemon, which refuses on this platform"),
    },
    #[cfg(unix)]
    Case {
        verb: &["bootstrap-robot"],
        argv: &["bootstrap-robot", "--state-root", "state"],
        expect: Expect::NotSpawned(
            "an internal worker `cerulion-netd` runs on a robot, before the login gate",
        ),
    },
    #[cfg(unix)]
    Case {
        verb: &["robot-sync-epoch"],
        argv: &["robot-sync-epoch", "--state-root", "state"],
        expect: Expect::NotSpawned(
            "an internal worker `cerulion-netd` runs on a robot, before the login gate",
        ),
    },
];
