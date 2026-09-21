// SPDX-License-Identifier: AGPL-3.0-only
//! Hermetic pins for the discovery-ladder engine
//! (`discovery_ladder::gather_rungs` + `dedupe_by_resolved_addr`).
//!
//! These exercise the injectable core with in-memory rung closures ONLY — no
//! mDNS, no multicast, no zenoh sessions. The parallelism / ceiling / panic-
//! isolation / dedupe / determinism contracts are pinned against HAND ORACLES
//! (never a self-compare). The real-rung wiring (`discover_peers`) and the
//! live announce-presence round-trip live in `topic_network_live_test.rs`.

use cerulion_cli_engine::discovery_ladder::{
    dedupe_by_resolved_addr, gather_rungs, ladder_ceiling, DiscoveredPeer, DiscoveryRung,
    NamedRung, LADDER_TOTAL_CEILING, SCAN_LADDER_CEILING,
};
use std::thread;
use std::time::{Duration, Instant};

/// Build a `DiscoveredPeer` tersely.
fn peer(robot: &str, locator: &str, rung: DiscoveryRung) -> DiscoveredPeer {
    DiscoveredPeer {
        robot: robot.to_string(),
        locator: locator.to_string(),
        rung,
    }
}

/// Build a `NamedRung` from a name + an infallible closure (avoids repeating
/// the `Box<dyn ...>` type annotation at every call site).
fn named(
    name: &'static str,
    f: impl FnOnce() -> Vec<DiscoveredPeer> + Send + 'static,
) -> NamedRung {
    (name, Box::new(f))
}

/// Gather-all: two robots from two rungs both survive, merged then sorted by
/// `(robot, locator)` — hand oracle.
#[test]
fn gather_all_merges_two_rungs() {
    let rungs = vec![
        named("mdns", || {
            vec![peer("robot-a", "tcp/192.168.1.7:7683", DiscoveryRung::Mdns)]
        }),
        named("cache", || {
            vec![peer(
                "robot-b",
                "tcp/192.168.1.8:7683",
                DiscoveryRung::Cache,
            )]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    assert_eq!(
        got,
        vec![
            peer("robot-a", "tcp/192.168.1.7:7683", DiscoveryRung::Mdns),
            peer("robot-b", "tcp/192.168.1.8:7683", DiscoveryRung::Cache),
        ]
    );
}

/// Parallelism proof: two rungs each sleeping ~200 ms inside a 1 s ceiling both
/// return AND the total wall is well under the 400 ms they would take serially
/// (generous 800 ms margin — no flaky tight bound).
#[test]
fn rungs_run_in_parallel_within_budget() {
    let start = Instant::now();
    let rungs = vec![
        named("mdns", || {
            thread::sleep(Duration::from_millis(200));
            vec![peer("a", "tcp/10.0.0.1:7683", DiscoveryRung::Mdns)]
        }),
        named("cache", || {
            thread::sleep(Duration::from_millis(200));
            vec![peer("b", "tcp/10.0.0.2:7683", DiscoveryRung::Cache)]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    let wall = start.elapsed();

    assert_eq!(got.len(), 2, "both rung results must be present");
    assert!(
        wall < Duration::from_millis(800),
        "parallel gather wall {wall:?} must be well under 2×200ms serial — proves concurrency"
    );
}

/// Ceiling honored: a rung that sleeps 5 s under a 300 ms ceiling has its
/// results DROPPED (its thread detached), while the fast rung's results are
/// kept — and the total wall is bounded near the ceiling, not the 5 s.
#[test]
fn ceiling_drops_slow_rung_keeps_fast() {
    let start = Instant::now();
    let rungs = vec![
        named("mdns", || {
            thread::sleep(Duration::from_secs(5));
            vec![peer("slow", "tcp/10.0.0.9:7683", DiscoveryRung::Mdns)]
        }),
        named("cache", || {
            vec![peer("fast", "tcp/10.0.0.1:7683", DiscoveryRung::Cache)]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_millis(300));
    let wall = start.elapsed();

    assert_eq!(
        got,
        vec![peer("fast", "tcp/10.0.0.1:7683", DiscoveryRung::Cache)],
        "the slow rung's results are dropped; the fast rung's are kept"
    );
    assert!(
        wall < Duration::from_millis(1500),
        "ceiling honored: wall {wall:?} bounded near 300ms, never the 5s slow rung"
    );
}

/// Error isolation: a panicking rung is contained — the sibling rung's results
/// are intact.
#[test]
fn panicking_rung_does_not_break_siblings() {
    let rungs = vec![
        named("mdns", || {
            panic!("simulated rung panic — contained by the gather")
        }),
        named("cache", || {
            vec![peer("ok", "tcp/10.0.0.1:7683", DiscoveryRung::Cache)]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    assert_eq!(
        got,
        vec![peer("ok", "tcp/10.0.0.1:7683", DiscoveryRung::Cache)],
        "a panic in one rung must not lose the other's results"
    );
}

/// Dedupe: locators that map to the SAME PURE key collapse to ONE peer,
/// first-in-order (mDNS) winning the tie — with NO resolver involvement. Three
/// collapse shapes, one input:
/// - exact literal (`127.0.0.1` == `127.0.0.1`),
/// - case-folded host (`ROBOT.local` == `robot.local`),
/// - `IpAddr`-normalized v6 (`[::1]` == `[0:0:0:0:0:0:0:1]`).
#[test]
fn dedupe_collapses_same_resolved_addr_first_wins() {
    let input = vec![
        peer("mdns-exact", "tcp/127.0.0.1:7683", DiscoveryRung::Mdns),
        // exact-literal duplicate → collapses into mdns-exact.
        peer("host-exact", "tcp/127.0.0.1:7683", DiscoveryRung::Hostname),
        peer("mdns-case", "tcp/ROBOT.local:7683", DiscoveryRung::Mdns),
        // case-folded host duplicate → collapses into mdns-case.
        peer("host-case", "tcp/robot.local:7683", DiscoveryRung::Hostname),
        peer("mdns-v6", "tcp/[::1]:7683", DiscoveryRung::Mdns),
        // IpAddr-normalized v6 duplicate → collapses into mdns-v6.
        peer(
            "host-v6",
            "tcp/[0:0:0:0:0:0:0:1]:7683",
            DiscoveryRung::Hostname,
        ),
    ];
    assert_eq!(
        dedupe_by_resolved_addr(input),
        vec![
            peer("mdns-exact", "tcp/127.0.0.1:7683", DiscoveryRung::Mdns),
            peer("mdns-case", "tcp/ROBOT.local:7683", DiscoveryRung::Mdns),
            peer("mdns-v6", "tcp/[::1]:7683", DiscoveryRung::Mdns),
        ],
        "each same-key pair collapses purely; the first occurrence (mDNS) wins the tie"
    );
}

/// Dedupe: the same host on DIFFERENT ports are distinct peers.
#[test]
fn dedupe_keeps_distinct_ports() {
    let input = vec![
        peer("a", "tcp/127.0.0.1:7683", DiscoveryRung::Mdns),
        peer("b", "tcp/127.0.0.1:7684", DiscoveryRung::Cache),
    ];
    assert_eq!(
        dedupe_by_resolved_addr(input.clone()),
        input,
        "different ports on one host are distinct (ip,port) keys"
    );
}

/// Dedupe: hostnames key PURELY on the case-folded `(host, port)` — NO
/// resolution ever. Distinct hostnames stay distinct; the same hostname in a
/// different case collapses (mDNS/DNS names are case-insensitive), first wins.
#[test]
fn dedupe_hostnames_key_purely_case_folded() {
    let input = vec![
        peer("a", "tcp/robot-alpha.local:7683", DiscoveryRung::Mdns),
        // Same host, different case → collapses into `a`.
        peer("b", "tcp/ROBOT-ALPHA.local:7683", DiscoveryRung::Cache),
        // A genuinely different hostname → stays distinct.
        peer("c", "tcp/robot-beta.local:7683", DiscoveryRung::Hostname),
    ];
    assert_eq!(
        dedupe_by_resolved_addr(input),
        vec![
            peer("a", "tcp/robot-alpha.local:7683", DiscoveryRung::Mdns),
            peer("c", "tcp/robot-beta.local:7683", DiscoveryRung::Hostname),
        ],
        "hostnames dedupe on the case-folded (host,port), no resolver involved"
    );
}

/// Determinism: identical injected inputs produce byte-identical (sorted)
/// output across two runs — AND that output matches the hand oracle (so it is
/// not a self-compare). The sort key is `(robot, locator)`.
#[test]
fn same_inputs_yield_identical_output() {
    let build = || {
        vec![
            named("mdns", || {
                vec![
                    peer("zeta", "tcp/10.0.0.9:7683", DiscoveryRung::Mdns),
                    peer("alpha", "tcp/10.0.0.1:7683", DiscoveryRung::Mdns),
                ]
            }),
            named("cache", || {
                vec![peer("mid", "tcp/10.0.0.5:7683", DiscoveryRung::Cache)]
            }),
        ]
    };
    let run1 = gather_rungs(build(), Duration::from_secs(1));
    let run2 = gather_rungs(build(), Duration::from_secs(1));
    assert_eq!(
        run1, run2,
        "same injected inputs must produce identical output"
    );
    assert_eq!(
        run1,
        vec![
            peer("alpha", "tcp/10.0.0.1:7683", DiscoveryRung::Mdns),
            peer("mid", "tcp/10.0.0.5:7683", DiscoveryRung::Cache),
            peer("zeta", "tcp/10.0.0.9:7683", DiscoveryRung::Mdns),
        ],
        "output sorted by (robot, locator) — hand oracle, not a self-compare"
    );
}

/// First-rung-wins on the CONCURRENT path: two rungs emit peers that share the
/// SAME dedupe key (identical IP-literal locator) but carry different robot/rung
/// tags. The FIRST rung is SLOW (~150 ms) and the SECOND fast, so their threads
/// COMPLETE second-then-first — yet the ordered-slots merge dedupes in RUNG-LIST
/// order, not completion order, so the FIRST rung's peer is the sole survivor.
#[test]
fn concurrent_dedupe_keeps_first_rung_not_first_completer() {
    let rungs = vec![
        named("mdns", || {
            thread::sleep(Duration::from_millis(150));
            vec![peer("first-rung", "tcp/10.9.9.9:7683", DiscoveryRung::Mdns)]
        }),
        named("cache", || {
            vec![peer(
                "second-rung",
                "tcp/10.9.9.9:7683",
                DiscoveryRung::Cache,
            )]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    assert_eq!(
        got,
        vec![peer("first-rung", "tcp/10.9.9.9:7683", DiscoveryRung::Mdns)],
        "same-key peers collapse to the FIRST rung's peer — rung-list order, not completion order"
    );
}

// ─── Ladder rung 4: the opt-in `--scan` subnet sweep (public surface) ─────────

/// The `Scan` rung stringifies as `scan` (the ROBOTS-section label an operator
/// sees for a swept hit).
#[test]
fn scan_rung_display_is_scan() {
    assert_eq!(DiscoveryRung::Scan.to_string(), "scan");
}

/// `ladder_ceiling` widens ONLY for a scan run: the default run keeps the
/// snappy sub-1.5 s ceiling, a `--scan` run gets the wider one (the sweep is
/// inherently slower). Hand oracle.
#[test]
fn scan_widens_the_ladder_ceiling() {
    assert_eq!(ladder_ceiling(false), LADDER_TOTAL_CEILING);
    assert_eq!(ladder_ceiling(true), SCAN_LADDER_CEILING);
    assert!(
        SCAN_LADDER_CEILING > LADDER_TOTAL_CEILING,
        "the scan ceiling must be wider so the slower sweep is not dropped"
    );
}

/// A scan-tagged peer is LAST in the ladder's rung list (appended after mDNS /
/// cache / hostname), so it LOSES a dedupe tie to an already-verified twin for
/// the same resolved `(ip, port)`. Model that ordering here: mDNS first, scan
/// second, identical locator ⇒ the mDNS peer is the sole survivor.
#[test]
fn scan_hit_loses_dedupe_tie_to_earlier_rung() {
    let rungs = vec![
        named("mdns", || {
            vec![peer(
                "robot-via-mdns",
                "tcp/192.168.1.20:7683",
                DiscoveryRung::Mdns,
            )]
        }),
        named("scan", || {
            vec![peer(
                "192.168.1.20",
                "tcp/192.168.1.20:7683",
                DiscoveryRung::Scan,
            )]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    assert_eq!(
        got,
        vec![peer(
            "robot-via-mdns",
            "tcp/192.168.1.20:7683",
            DiscoveryRung::Mdns
        )],
        "an mDNS hit wins the tie over a same-address scan hit (scan is the last rung)"
    );
}

/// A scan-only hit (no other rung found this address) survives and renders with
/// the `Scan` rung tag — hand oracle.
#[test]
fn scan_only_hit_survives_and_is_tagged() {
    let rungs = vec![
        named("mdns", || {
            vec![peer("known", "tcp/192.168.1.7:7683", DiscoveryRung::Mdns)]
        }),
        named("scan", || {
            vec![peer("swept", "tcp/192.168.1.42:7683", DiscoveryRung::Scan)]
        }),
    ];
    let got = gather_rungs(rungs, Duration::from_secs(1));
    assert_eq!(
        got,
        vec![
            // sorted by (robot, locator): "known" < "swept".
            peer("known", "tcp/192.168.1.7:7683", DiscoveryRung::Mdns),
            peer("swept", "tcp/192.168.1.42:7683", DiscoveryRung::Scan),
        ],
        "a scan-only hit survives the merge, tagged with the Scan rung"
    );
}
