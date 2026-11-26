// SPDX-License-Identifier: AGPL-3.0-only
//! Every `cerulion` verb as one invocation, with what the network kill switch
//! does to it: a second row for a verb with `--robot`, and one for each form
//! that signs in, which the switch allows.
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
    /// Local/offline verb: not refused by the switch and does not enter the
    /// login gate, and the sweep does not spawn it for the reason given
    /// (blocks waiting for a stream, TTY, daemon, or harness).
    LocalNoGate(&'static str),
    /// The switch does not refuse it (or it never meets the switch's check),
    /// and the sweep does not spawn it, for the reason given.
    NotSpawned(&'static str),
    /// Account sign-in, which the switch does not cover: the switch covers
    /// Cerulion service traffic (robots, relays, hosted Cerulion services and
    /// telemetry), and signing in is the user's own request. Under the switch
    /// the invocation still reaches the account service: run on `home`, on a
    /// pseudo-terminal when `terminal`, its first request line opens with
    /// `request`.
    SignIn {
        home: Home,
        terminal: bool,
        request: &'static str,
    },
}

/// The home a [`Expect::SignIn`] row runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Home {
    /// A machine that never signed in.
    Fresh,
    /// A machine with a stored session.
    SignedIn,
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
const GRAPH_LIVE: &[&str] = &[
    "Error: cerulion graph live: asking robots is disabled by ",
    ", so this command stays local-only and contacts no robot; unset CERULION_NETWORK to ask \
     robots through cerulion-netd or --connect",
];
const GRAPH_FIRES: &[&str] = &[
    "Error: cerulion graph fires: asking robots is disabled by ",
    ", so this command stays local-only and contacts no robot; unset CERULION_NETWORK to ask \
     robots through cerulion-netd or --connect",
];

const fn refused_query(
    verb: &'static [&'static str],
    argv: &'static [&'static str],
    says: &'static [&'static str],
) -> Case {
    Case {
        verb,
        argv,
        expect: Expect::Refused {
            code: 1,
            says,
            by: By::Network,
        },
    }
}

const fn gated(verb: &'static [&'static str], argv: &'static [&'static str]) -> Case {
    Case {
        verb,
        argv,
        expect: Expect::Gated,
    }
}

const fn local_runs(
    verb: &'static [&'static str],
    argv: &'static [&'static str],
    code: i32,
    says: &'static str,
) -> Case {
    Case {
        verb,
        argv,
        expect: Expect::Runs { code, says },
    }
}

const fn local_no_gate(
    verb: &'static [&'static str],
    argv: &'static [&'static str],
    why: &'static str,
) -> Case {
    Case {
        verb,
        argv,
        expect: Expect::LocalNoGate(why),
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
    // ---- refused in every form: they always ask robots, through
    // cerulion-netd or a session of their own (--connect) ----
    refused_query(&["graph", "live"], &["graph", "live"], GRAPH_LIVE),
    refused_query(
        &["graph", "live"],
        &["graph", "live", "--robot", "bench-arm"],
        GRAPH_LIVE,
    ),
    refused_query(
        &["graph", "live"],
        &["graph", "live", "--connect", "tcp/127.0.0.1:7683"],
        GRAPH_LIVE,
    ),
    refused_query(&["graph", "fires"], &["graph", "fires"], GRAPH_FIRES),
    refused_query(
        &["graph", "fires"],
        &["graph", "fires", "--robot", "bench-arm"],
        GRAPH_FIRES,
    ),
    refused_query(
        &["graph", "fires"],
        &["graph", "fires", "--connect", "tcp/127.0.0.1:7683"],
        GRAPH_FIRES,
    ),
    Case {
        verb: &["topic", "echo"],
        argv: &["topic", "echo", "/camera", "--robot", "bench-arm"],
        expect: Expect::Refused {
            code: 2,
            says: &["Error: --robot bench-arm reaches the robot over the network, which ", " disables; unset CERULION_NETWORK to call its topic-echo verb (without --robot the command runs on this machine)"],
            by: By::Network,
        },
    },
    Case {
        verb: &["topic", "hz"],
        argv: &["topic", "hz", "/camera", "--robot", "bench-arm"],
        expect: Expect::Refused {
            code: 2,
            says: &["Error: --robot bench-arm reaches the robot over the network, which ", " disables; unset CERULION_NETWORK to call its topic-hz verb (without --robot the command runs on this machine)"],
            by: By::Network,
        },
    },
    Case {
        verb: &["topic", "info"],
        argv: &["topic", "info", "/camera", "--robot", "bench-arm"],
        expect: Expect::Refused {
            code: 2,
            says: &["Error: --robot bench-arm reaches the robot over the network, which ", " disables; unset CERULION_NETWORK to call its topic-info verb (without --robot the command runs on this machine)"],
            by: By::Network,
        },
    },
    // ---- the same verbs without --robot are local and ungated ----
    local_runs(
        &["topic", "pub"],
        &[
            "topic",
            "pub",
            "/cmd_vel",
            "std_msgs/String",
            "{data: hi}",
            "--wait-subscribers",
            "0",
        ],
        0,
        "published: topic=/cmd_vel type=std_msgs/String frames=1",
    ),
    local_runs(
        &["topic", "snapshot"],
        &["topic", "snapshot", "/camera", "--timeout", "0"],
        3,
        "/camera",
    ),
    local_no_gate(
        &["service", "call"],
        &["service", "call", "/reset", "std_srvs/srv/Trigger"],
        "waits for a local service server or workspace .srv definition (covered directly in sweep)",
    ),
    local_no_gate(
        &["viz"],
        &["viz", "/camera"],
        "spawns or attaches to cerulion-vizd and stays attached until Ctrl+C",
    ),
    // ---- the simulated robot and the benchmark are local and ungated ----
    local_no_gate(
        &["rcp"],
        &["rcp", "--sim"],
        "launches the local harness simulator (covered in login_gate_e2e_test)",
    ),
    local_no_gate(
        &["rcp"],
        &["rcp", "--sim", "--http", "127.0.0.1:8765"],
        "launches the local harness HTTP server (covered in login_gate_e2e_test)",
    ),
    local_no_gate(
        &["rcp", "bench"],
        &["rcp", "bench", "--model", "m"],
        "launches the local harness benchmark (covered in login_gate_e2e_test)",
    ),
    local_no_gate(
        &["rcp", "bench"],
        &["rcp", "bench", "--verify"],
        "launches the local harness verifier (covered in login_gate_e2e_test)",
    ),
    // ---- local/offline verbs: not refused under the switch and ungated ----
    local_runs(
        &["workspace", "create"],
        &["workspace", "create", "demo"],
        0,
        "Created workspace",
    ),
    local_runs(
        &["workspace", "init"],
        &["workspace", "init"],
        0,
        "Initialized workspace",
    ),
    local_runs(
        &["node", "create"],
        &["node", "create", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "delete"],
        &["node", "delete", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "modify"],
        &["node", "modify", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "build"],
        &["node", "build", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "stage"],
        &["node", "stage", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "run"],
        &["node", "run", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "list"],
        &["node", "list"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["node", "info"],
        &["node", "info", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "create"],
        &["graph", "create", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "run"],
        &["graph", "run", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "validate"],
        &["graph", "validate", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "list"],
        &["graph", "list"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "levels"],
        &["graph", "levels", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "profile"],
        &["graph", "profile", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "partition"],
        &["graph", "partition", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "plan"],
        &["graph", "plan", "demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["graph", "pause"],
        &["graph", "pause", "run-1"],
        1,
        "run-1",
    ),
    local_runs(
        &["graph", "resume"],
        &["graph", "resume", "run-1"],
        1,
        "run-1",
    ),
    local_runs(&["topic", "list"], &["topic", "list"], 0, "LOCAL"),
    local_runs(
        &["topic", "info"],
        &["topic", "info", "/camera"],
        1,
        "/camera",
    ),
    local_no_gate(
        &["topic", "echo"],
        &["topic", "echo", "/camera"],
        "streams local topic messages until Ctrl+C",
    ),
    local_no_gate(
        &["topic", "hz"],
        &["topic", "hz", "/camera"],
        "measures local topic rate until Ctrl+C",
    ),
    local_runs(
        &["topic", "type"],
        &["topic", "type", "/camera"],
        1,
        "/camera",
    ),
    local_no_gate(
        &["topic", "find"],
        &["topic", "find", "std_msgs/String"],
        "exits 0 with empty stdout when no local topic matches the type",
    ),
    local_no_gate(
        &["topic", "bw"],
        &["topic", "bw", "/camera"],
        "measures local topic bandwidth until Ctrl+C",
    ),
    local_no_gate(
        &["ros2", "migrate"],
        &["ros2", "migrate", "--workspace", "ws"],
        "depends on optional clang migration binary availability",
    ),
    local_no_gate(
        &["ros2", "attach"],
        &["ros2", "attach", "--iface", "127.0.0.1"],
        "opens a DDS discovery window (serving gate covered in serving_login_e2e_test)",
    ),
    // ---- account-service verbs: not refused under the switch, behind the login gate ----
    gated(
        &["account", "devices", "list"],
        &["account", "devices", "list"],
    ),
    gated(
        &["account", "devices", "revoke"],
        &["account", "devices", "revoke", "device-1"],
    ),
    local_no_gate(&["tui"], &["tui"], "requires an interactive terminal UI"),
    local_runs(
        &["trace", "inspect"],
        &["trace", "inspect", "traces"],
        1,
        "traces",
    ),
    local_runs(
        &["bag", "play"],
        &["bag", "play", "run.mcap"],
        1,
        "run.mcap",
    ),
    local_runs(
        &["bag", "info"],
        &["bag", "info", "run.mcap"],
        1,
        "run.mcap",
    ),
    local_runs(
        &["bag", "migrate"],
        &["bag", "migrate", "run.mcap"],
        1,
        "run.mcap",
    ),
    local_no_gate(
        &["bag", "record"],
        &["bag", "record", "/camera"],
        "records local topics until Ctrl+C",
    ),
    local_runs(&["flashback"], &["flashback"], 1, "Workspace not found"),
    local_runs(
        &["schema", "create"],
        &["schema", "create", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["schema", "delete"],
        &["schema", "delete", "Demo"],
        1,
        "Workspace not found",
    ),
    local_runs(
        &["schema", "info"],
        &["schema", "info", "std_msgs/String"],
        0,
        "std_msgs/String",
    ),
    local_runs(
        &["schema", "list"],
        &["schema", "list"],
        0,
        "Built-in ROS 2 messages:",
    ),
    // ---- account sign-in: not covered by the switch, so allowed under it ----
    Case {
        verb: &["account", "devices", "list"],
        argv: &["account", "devices", "list"],
        expect: Expect::SignIn {
            home: Home::Fresh,
            terminal: true,
            request: "POST /v1/auth/device/start",
        },
    },
    Case {
        verb: &["login"],
        argv: &["login"],
        expect: Expect::SignIn {
            home: Home::Fresh,
            terminal: false,
            request: "POST /v1/auth/device/start",
        },
    },
    Case {
        verb: &["logout"],
        argv: &["logout"],
        expect: Expect::SignIn {
            home: Home::SignedIn,
            terminal: false,
            request: "POST /v1/auth/revoke",
        },
    },
    Case {
        verb: &["account", "devices", "list"],
        argv: &["account", "devices", "list"],
        expect: Expect::SignIn {
            home: Home::SignedIn,
            terminal: false,
            request: "GET /v1/devices",
        },
    },
    Case {
        verb: &["account", "devices", "revoke"],
        argv: &["account", "devices", "revoke", "device-1"],
        expect: Expect::SignIn {
            home: Home::SignedIn,
            terminal: false,
            request: "POST /v1/devices/device-1/revoke",
        },
    },
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
    // The hidden `pair` stub reaches no robot: it refuses every spelling
    // with the same text, switch or no switch, and never meets the gate.
    Case {
        verb: &["pair"],
        argv: &["pair", "bench-arm"],
        expect: Expect::Runs {
            code: 2,
            says: "Error: code pairing is not available in this release; robots on your \
                   account are reached by signing in with `cerulion login`",
        },
    },
    // ---- not spawned by the sweep ----
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
