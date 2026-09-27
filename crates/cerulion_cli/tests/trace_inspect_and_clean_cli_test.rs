// SPDX-License-Identifier: AGPL-3.0-only
//! The `cli-tui-clean-trace` coverage gap: the "trace inspect" and "clean"
//! halves (NOT the TUI itself, which already has 8 behavioral `TestBackend`
//! render tests per the repository's test table).
//!
//! (a) `cerulion trace inspect <dir>` reads `trace_*.jsonl` bag files and
//! prints a human-readable timeline (`cerulion_cli/src/main.rs`'s
//! `trace_inspect` + `parse_jsonl_record`). Neither function had ANY test
//! coverage before this file. Reading `parse_jsonl_record` closely surfaced
//! a real, pre-existing bug — fixed in the same commit as this test file
//! (see the comment on `parse_jsonl_record` in `main.rs`): the
//! numeric-field extraction used an inclusive slice (`rest[..=end]` with
//! `end` the delimiter's OWN index) that captured the trailing `,`/`}`
//! delimiter INTO the substring handed to `.parse::<u64>()`, which always
//! rejects trailing punctuation. Since `seq`/`ts_ns` are never the LAST
//! field in the documented `{"topic":..,"seq":..,"ts_ns":..,"schema_hash":..}`
//! shape, this meant `parse_jsonl_record` returned `None` for EVERY
//! conforming line, and `trace_inspect` printed every well-formed record
//! through the "(malformed)" fallback — the documented pretty-print line
//! (`<topic> seq=.. t=..ns schema=..`) was unreachable dead code. Fixed to
//! slice up to (not including) the delimiter for both branches. This file's
//! happy-path test is the regression pin for that fix, driven through the
//! REAL subprocess — the only way to exercise `parse_jsonl_record` at all,
//! since it is a private fn in a binary crate with no unit-test seam.
//!
//! (b) `cerulion clean` (`main.rs`'s `clean_iceoryx2_state`) sweeps DEAD
//! iceoryx2 nodes, and every arm here runs it against a registry this file
//! OWNS: [`write_isolated_iceoryx2_config`] drops the project-local config
//! file iceoryx2 reads first into the spawn's own temporary cwd, so the
//! sweep, the registry count and the report all resolve to a directory under
//! that cwd, and every arm requires the report to name that directory back.
//!
//! ## What the isolation buys, and what it costs
//!
//! A sibling process cannot perturb these arms, because it cannot see the
//! registry they read. The arms cannot perturb a sibling, because they reach
//! nothing outside their own temporary directories. Their runtime is constant
//! in the machine's dead-node population rather than linear in it. The state
//! to be cleaned is minted the same way: the fixture child
//! ([`subprocess_register_an_iceoryx2_node_then_die`]) is spawned from the
//! SAME cwd and so builds its node under the SAME private root, and it
//! refuses to run unless its own global config proves that took.
//!
//! Every arm passes `--report-only`, and the boundary arm DEPENDS on a product
//! defect while it stands: `clean_dead_nodes` calls
//! `cleanup_dead_iceoryx2_nodes_with_diagnostics` before it reads the flag at
//! all, so the destructive dead-node sweep runs whether or not
//! `--report-only` was given, which is what lets the arm below watch a planted
//! node be REMOVED under the flag. That is issue #116, filed rather than fixed
//! here (a flag that promises to report and then deletes is a product change
//! and gets its own pass). WHOEVER FIXES #116 must revisit the boundary arm:
//! the sweep observation has to move to whatever path then performs the sweep,
//! or the arm goes inert.
//!
//! The flag is still what every arm passes, because the one population no
//! config can move is the `/tmp/*.shm_state` files, and `--report-only` keeps
//! that half a bounded READ rather than a machine-wide reclamation a test has
//! no business performing. Not confinable, and not for want of trying:
//! `shm_state::SHM_STATE_DIRECTORY` is the compile-time constant `"/tmp/"`
//! mirroring `iceoryx2_pal_configuration::TEMP_DIRECTORY`, iceoryx2 honours no
//! `TMPDIR`, and the binary reads no environment variable for it.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

/// Set to "1" ONLY on the spawned fixture child, so a bare `-- --ignored` run
/// of this binary no-ops instead of minting a node somewhere unexpected.
const ENV_CHILD: &str = "CER_CLEAN_CLI_CHILD";
/// The private iceoryx2 root the child must find itself already pointed at.
const ENV_ROOT: &str = "CER_CLEAN_CLI_IOX2_ROOT";
/// The fixture child's own test name, spawned with `--exact --ignored`.
const CHILD_TEST: &str = "subprocess_register_an_iceoryx2_node_then_die";

/// Spawn `cerulion <args>` in `cwd`, wait for it to exit (these are all
/// short-lived one-shot commands — no signals needed), and return
/// `(exit_success, stdout, stderr)`.
fn run_cerulion(args: &[&str], cwd: &Path) -> (bool, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(cwd)
        .output()
        .unwrap_or_else(|e| panic!("spawn cerulion {args:?}: {e}"));
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// Point every process spawned from `cwd` at a private iceoryx2 registry
/// under `root`, by writing the PROJECT-LOCAL config file into `cwd`.
///
/// `iceoryx2::config::Config` resolves its file in three places and takes the
/// first that loads: `config/iceoryx2.toml` relative to the process cwd, then
/// the user config directory, then the compiled-in global path. Only the first
/// is reachable from a test, and only because every spawn here already runs
/// from a temporary directory. `Global` is `#[serde(default)]`, so `root-path`
/// is the one key that has to be written and everything else keeps its
/// default.
///
/// The seam has no in-process equivalent: `Config::set_root_path` acts on the
/// caller's own singleton, and neither the binary nor the fixture child reads
/// an environment variable for the root, so a file in the child's cwd is the
/// only way to reach the child's singleton without adding a production seam.
///
/// `root` is written as a TOML basic string with `\` and `"` escaped. A
/// temporary directory is unlikely to carry either, and a path that broke the
/// file would be caught rather than silently ignored (see below), but the cost
/// of getting it right is two `replace` calls and the cost of getting it wrong
/// is the whole point of this file: a child reading no config falls back to the
/// machine's registry. The residual is a control character in a path, which no
/// escaping of these two characters covers and which the loud failure below
/// still catches.
///
/// Nothing asserts here. A config that failed to load would leave the child on
/// `/tmp/iceoryx2`, which every caller catches: the `cerulion` arms require
/// the report to name `root` back, and the fixture child refuses outright.
fn write_isolated_iceoryx2_config(cwd: &Path, root: &Path) {
    let dir = cwd.join("config");
    std::fs::create_dir_all(&dir).expect("create the project-local config dir");
    let escaped = root
        .display()
        .to_string()
        .replace('\\', "\\\\")
        .replace('"', "\\\"");
    std::fs::write(
        dir.join("iceoryx2.toml"),
        format!("[global]\nroot-path = \"{escaped}\"\n"),
    )
    .expect("write the project-local iceoryx2 config");
}

/// A temporary cwd wired to its own private iceoryx2 registry: the cwd every
/// spawn runs from, the config file that redirects them, and the root itself.
struct PrivateRegistry {
    cwd: tempfile::TempDir,
    root: PathBuf,
}

impl PrivateRegistry {
    fn new() -> Self {
        let cwd = tempfile::tempdir().expect("temporary cwd");
        let root = cwd.path().join("iox2-root");
        std::fs::create_dir_all(&root).expect("create the private iceoryx2 root");
        write_isolated_iceoryx2_config(cwd.path(), &root);
        Self { cwd, root }
    }

    fn cwd(&self) -> &Path {
        self.cwd.path()
    }

    /// iceoryx2's node-registry directory under this root, which is what
    /// `cerulion clean`'s report names back.
    fn nodes_dir(&self) -> PathBuf {
        self.root.join("nodes")
    }

    /// Hand oracle for the registry line, transcribed from
    /// `shm_state::render_lines` rather than read back off it: the absent
    /// form, printed when no iceoryx2 process has left state under this root.
    fn registry_line_when_absent(&self) -> String {
        format!(
            "iceoryx2 node registry: none at {} (no iceoryx2 process has left state here)",
            self.nodes_dir().display()
        )
    }

    /// The same oracle for a registry directory that EXISTS and holds
    /// `entries` of them.
    fn registry_line_with(&self, entries: u64) -> String {
        format!(
            "iceoryx2 node registry: {entries} entr{} under {}",
            if entries == 1 { "y" } else { "ies" },
            self.nodes_dir().display()
        )
    }

    /// The registered nodes: the top-level DIRECTORIES of the node-registry
    /// directory, one per node. Its sidecars (`<prefix><id>.node_monitor` and
    /// friends) are files beside it and are not nodes, so counting every
    /// top-level entry would call one node four.
    fn registered_nodes(&self) -> BTreeSet<PathBuf> {
        let mut out = BTreeSet::new();
        let Ok(entries) = std::fs::read_dir(self.nodes_dir()) else {
            return out;
        };
        for entry in entries.flatten() {
            if entry.path().is_dir() {
                out.insert(entry.file_name().into());
            }
        }
        out
    }

    /// Every path under the node-registry directory, relative to it, sorted,
    /// each carrying its CONTENT: the bytes of a file, or `None` for a
    /// directory. That whole value is what an "untouched" assertion compares.
    ///
    /// Paths alone would not be enough. A sweep that truncated or rewrote a
    /// node's `iox2_node.details` in place, leaving every name where it was,
    /// would compare equal to an untouched registry and the assertion would
    /// pass on a registry that had in fact been reached. These files are a few
    /// hundred bytes each and there is one node per root here, so reading them
    /// costs nothing worth trading a hole in the proof for.
    ///
    /// A file that cannot be read is recorded as its error rather than skipped,
    /// so a permission change is a difference too.
    fn registry_contents(&self) -> BTreeMap<PathBuf, Option<Result<Vec<u8>, String>>> {
        let mut out = BTreeMap::new();
        let dir = self.nodes_dir();
        let mut stack = vec![dir.clone()];
        while let Some(next) = stack.pop() {
            let Ok(entries) = std::fs::read_dir(&next) else {
                continue;
            };
            for entry in entries.flatten() {
                let path = entry.path();
                let content = if path.is_dir() {
                    stack.push(path.clone());
                    None
                } else {
                    Some(std::fs::read(&path).map_err(|e| e.to_string()))
                };
                out.insert(
                    path.strip_prefix(&dir)
                        .expect("a walked path is under the directory walked")
                        .to_path_buf(),
                    content,
                );
            }
        }
        out
    }

    /// Mint one DEAD iceoryx2 node under this root: spawn the fixture child
    /// from this cwd, so the project-local config points it at the same
    /// private root, and let it register a node and exit without tearing it
    /// down.
    fn plant_a_dead_node(&self) {
        let exe = std::env::current_exe().expect("current_exe");
        let out = Command::new(exe)
            .args(["--exact", CHILD_TEST, "--ignored", "--nocapture"])
            .current_dir(self.cwd())
            .env(ENV_CHILD, "1")
            .env(ENV_ROOT, &self.root)
            .stdin(std::process::Stdio::null())
            .output()
            .expect("spawn the fixture child");
        assert!(
            out.status.success(),
            "the fixture child must register a node and exit 0; status={:?}\n\
             stdout:\n{}\nstderr:\n{}",
            out.status,
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        assert_eq!(
            self.registered_nodes().len(),
            1,
            "the fixture child must leave exactly one node registered under {}",
            self.nodes_dir().display()
        );
    }
}

/// The fixture: register an iceoryx2 node under the private root this process
/// was spawned into, then die without tearing it down, leaving the registry
/// entry a dead-node sweep is meant to reclaim.
///
/// `#[ignore]` plus the [`ENV_CHILD`] guard: libtest never runs it, and a bare
/// `-- --ignored` run of this binary returns immediately rather than minting a
/// node.
///
/// The global config is READ, never built here, which is what makes this
/// child's node land in the same registry the `cerulion` spawn beside it
/// sweeps, from the same file in the same cwd. It is asserted before anything
/// touches iceoryx2, because a child that had fallen back to the machine's
/// default registry would plant a node on a shared machine and the arm driving
/// it would then be measuring nothing.
// P12 exemption, scoped to this fn (the `clean_orphan_port_tag_test`
// `subprocess_child_mint_orphan_port_tag` precedent): this is the body of a
// SELF-RE-EXEC CHILD process, a process entrypoint by construction, and dying
// without unwinding is the whole point. A libtest teardown that ran here could
// only make the on-disk shape less certain. The ban stays armed for every
// other line in this binary.
#[allow(clippy::disallowed_methods)]
#[test]
#[ignore = "fixture child, spawned by plant_a_dead_node with CER_CLEAN_CLI_CHILD=1"]
fn subprocess_register_an_iceoryx2_node_then_die() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let root = std::env::var(ENV_ROOT).expect("the parent sets the private root");
    let expected = PathBuf::from(root).join("nodes");
    let config = iceoryx2::config::Config::global_config();
    let node_dir = PathBuf::from(String::from(&config.global.node_dir()));
    assert_eq!(
        node_dir, expected,
        "the project-local config must have pointed this child at the private registry; \
         a child on the machine's default registry must plant nothing"
    );

    let manager = cerulion_core::TransportManager::init_for_test(
        cerulion_core::TransportConfig {
            node_name: format!("clean_cli_fixture_{}", std::process::id()),
            ..Default::default()
        },
        config.clone(),
    )
    .expect("the fixture registers a node on the private root");

    // Never dropped: a graceful teardown deregisters the node, and a
    // deregistered node is not the state `cerulion clean` exists to reclaim.
    std::mem::forget(manager);
    std::process::exit(0);
}

/// Hand-build 2 `trace_*.jsonl` files: 4 well-formed records (2 topics × 2
/// records each, with distinct seq/ts_ns/schema_hash so filter/limit/
/// reverse are each observably distinguishable) + 1 line that doesn't parse
/// at all. Read in lexicographic FILE order, then within-file line order:
///
/// 1. seq=1 /a/b   (file 0000, line 1)
/// 2. malformed    (file 0000, line 2)
/// 3. seq=2 /c/d   (file 0000, line 3)
/// 4. seq=3 /a/b   (file 0001, line 1)
/// 5. seq=4 /c/d   (file 0001, line 2)
fn build_trace_dir(dir: &Path) {
    std::fs::create_dir_all(dir).unwrap();
    std::fs::write(
        dir.join("trace_0000.jsonl"),
        r#"{"topic":"/a/b","seq":1,"ts_ns":1111,"schema_hash":"0x1A"}
not a json line at all
{"topic":"/c/d","seq":2,"ts_ns":2222,"schema_hash":"0x2B"}
"#,
    )
    .unwrap();
    std::fs::write(
        dir.join("trace_0001.jsonl"),
        r#"{"topic":"/a/b","seq":3,"ts_ns":3333,"schema_hash":"0x3C"}
{"topic":"/c/d","seq":4,"ts_ns":4444,"schema_hash":"0x4D"}
"#,
    )
    .unwrap();
}

// Exact `println!("{} seq={} t={}ns schema=0x{:016X}", topic, seq, ts,
// schema)` renderings from `trace_inspect` (uppercase, zero-padded-16 hex —
// distinct from `topic_cmd::topic_echo`'s lowercase `{:016x}`).
const SEQ1: &str = "/a/b seq=1 t=1111ns schema=0x000000000000001A";
const SEQ2: &str = "/c/d seq=2 t=2222ns schema=0x000000000000002B";
const SEQ3: &str = "/a/b seq=3 t=3333ns schema=0x000000000000003C";
const SEQ4: &str = "/c/d seq=4 t=4444ns schema=0x000000000000004D";
const MALFORMED: &str = "(malformed) not a json line at all";

/// The whole summary line `report_sweep` prints when the registry holds no
/// dead node, transcribed by hand rather than read back off the source.
const NOTHING_TO_CLEAN: &str = "No dead iceoryx2 nodes found — nothing to clean.";
/// The whole summary line the same renderer prints for one reclaimed node and
/// no refusals.
const ONE_NODE_CLEANED: &str = "Cleaned 1 dead iceoryx2 node(s); 0 cleanup(s) failed.";

/// Assert `stdout` carries `line` as a WHOLE line, not as a substring of a
/// longer one: the counts and the paths in these reports are the content, and
/// a substring match would accept a line that changed either.
fn assert_reports_line(stdout: &str, line: &str, what: &str) {
    assert!(
        stdout.lines().any(|printed| printed == line),
        "{what}: the report must carry the whole line\n  {line}\nstdout:\n{stdout}"
    );
}

/// Assert no line of `stdout` names the machine's own registry, the one place
/// a fallen-back child would report on.
fn assert_never_names_the_machines_registry(stdout: &str) {
    assert!(
        !stdout.contains("/tmp/iceoryx2/nodes"),
        "no arm may report on the machine's own iceoryx2 registry; stdout:\n{stdout}"
    );
}

/// No flags: every line renders in FILE order (both files, lexicographic),
/// well-formed lines as the pretty `<topic> seq=.. t=..ns schema=..` format,
/// the unparseable line passed through as `(malformed) <line>`, and the
/// trailing summary names the total record + file counts.
#[test]
fn trace_inspect_no_flags_renders_all_records_in_file_order() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &["trace", "inspect", tmp.path().to_str().unwrap()],
        tmp.path(),
    );
    assert!(ok, "trace inspect must exit 0; stderr:\n{stderr}");

    for (needle, next) in [
        (SEQ1, MALFORMED),
        (MALFORMED, SEQ2),
        (SEQ2, SEQ3),
        (SEQ3, SEQ4),
    ] {
        let a = stdout
            .find(needle)
            .unwrap_or_else(|| panic!("missing {needle:?}; stdout:\n{stdout}"));
        let b = stdout
            .find(next)
            .unwrap_or_else(|| panic!("missing {next:?}; stdout:\n{stdout}"));
        assert!(
            a < b,
            "{needle:?} must render BEFORE {next:?} (file/line order); stdout:\n{stdout}"
        );
    }
    assert!(
        stdout.contains("[5 record(s) across 2 file(s)]"),
        "summary line missing or wrong; stdout:\n{stdout}"
    );
}

/// `--filter <topic>` restricts to exact-topic-substring matches only, and
/// the summary names the filter.
#[test]
fn trace_inspect_filter_restricts_to_matching_topic() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &[
            "trace",
            "inspect",
            tmp.path().to_str().unwrap(),
            "--filter",
            "/a/b",
        ],
        tmp.path(),
    );
    assert!(ok, "trace inspect --filter must exit 0; stderr:\n{stderr}");

    assert!(stdout.contains(SEQ1), "stdout:\n{stdout}");
    assert!(stdout.contains(SEQ3), "stdout:\n{stdout}");
    assert!(
        !stdout.contains(SEQ2) && !stdout.contains(SEQ4) && !stdout.contains("(malformed)"),
        "filter must exclude non-matching topics and the malformed line; stdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[2 record(s) across 2 file(s) filtered by topic=\"/a/b\"]"),
        "summary must name the filter; stdout:\n{stdout}"
    );
}

/// `--limit N --reverse`: the source reverses FIRST, then truncates — so the
/// surviving records are the LAST N in file order (most-recent-first), not
/// the first N then reversed. This is the order-of-operations discriminator:
/// a truncate-then-reverse bug would instead show `[malformed, seq=1]`.
#[test]
fn trace_inspect_limit_and_reverse_truncates_after_reversing() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &[
            "trace",
            "inspect",
            tmp.path().to_str().unwrap(),
            "--limit",
            "2",
            "--reverse",
        ],
        tmp.path(),
    );
    assert!(
        ok,
        "trace inspect --limit --reverse must exit 0; stderr:\n{stderr}"
    );

    let seq4_at = stdout
        .find(SEQ4)
        .unwrap_or_else(|| panic!("missing {SEQ4:?}; stdout:\n{stdout}"));
    let seq3_at = stdout
        .find(SEQ3)
        .unwrap_or_else(|| panic!("missing {SEQ3:?}; stdout:\n{stdout}"));
    assert!(
        seq4_at < seq3_at,
        "reverse-then-truncate must show seq=4 before seq=3; stdout:\n{stdout}"
    );
    assert!(
        !stdout.contains(SEQ1) && !stdout.contains(SEQ2) && !stdout.contains("(malformed)"),
        "truncation to 2 (after reversing) must drop everything but seq=4/seq=3; \
         stdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[2 record(s) across 2 file(s)]"),
        "summary count must reflect the post-truncation total; stdout:\n{stdout}"
    );
}

/// `cerulion clean` on an EMPTY private registry: nothing to clean, said as
/// such, and the report names the private registry it looked in.
///
/// The naming is the load-bearing half. This arm is worth nothing if the
/// binary swept the machine's own registry instead, and on a shared machine
/// that sweep would report whatever the machine happened to be carrying. So
/// the registry line is required to name this test's own root, which a
/// fallen-back child could not print.
#[test]
fn clean_happy_path_reports_nothing_to_clean() {
    let registry = PrivateRegistry::new();

    let (ok, stdout, stderr) = run_cerulion(&["clean", "--report-only"], registry.cwd());
    assert!(ok, "cerulion clean must exit 0; stderr:\n{stderr}");
    assert_reports_line(&stdout, NOTHING_TO_CLEAN, "an empty private registry");
    assert_reports_line(
        &stdout,
        &registry.registry_line_when_absent(),
        "the registry the sweep looked in",
    );
    assert_never_names_the_machines_registry(&stdout);
}

/// The boundary the isolation is worth anything at: a dead node planted under
/// THIS arm's private root is swept, and a dead node planted under a SECOND
/// private root is left exactly as it was.
///
/// The second root is the anti-tautology control, and it is a control in both
/// directions. Its node is provably sweepable, because the last leg runs the
/// same verb from the second cwd and watches it go, so "untouched" can only
/// mean the first run never reached it, never that there was nothing there to
/// reach. Without that leg a `clean` that swept nothing anywhere would pass.
///
/// The convergence leg in the middle is what pins the reclaim ordering: a
/// sweep that removed a node's name mapping before its details would leave the
/// node permanently unreclaimable, and the second `clean` would find it still
/// registered instead of finding nothing.
///
/// Reads under `--report-only` and still sees nodes REMOVED, because the sweep
/// runs before the flag is read (issue #116, see the module doc). This arm is
/// the one that has to move when that is fixed.
#[test]
fn clean_sweeps_its_own_registry_and_leaves_another_root_untouched() {
    let swept = PrivateRegistry::new();
    let spared = PrivateRegistry::new();
    swept.plant_a_dead_node();
    spared.plant_a_dead_node();
    let spared_before = spared.registry_contents();

    let (ok, stdout, stderr) = run_cerulion(&["clean", "--report-only"], swept.cwd());
    assert!(ok, "cerulion clean must exit 0; stderr:\n{stderr}");
    assert_reports_line(&stdout, ONE_NODE_CLEANED, "the planted dead node");
    assert_reports_line(
        &stdout,
        &swept.registry_line_with(0),
        "the swept registry, now empty",
    );
    assert_never_names_the_machines_registry(&stdout);
    assert_eq!(
        swept.registry_contents(),
        BTreeMap::new(),
        "the swept registry must hold nothing afterwards"
    );
    assert_eq!(
        spared.registry_contents(),
        spared_before,
        "a sweep of one private registry must leave every other root exactly as it was"
    );

    // Convergence: nothing is left, and the verb says so rather than
    // re-finding a node whose mapping an out-of-order reclaim had stranded.
    let (ok, stdout, stderr) = run_cerulion(&["clean", "--report-only"], swept.cwd());
    assert!(
        ok,
        "the second cerulion clean must exit 0; stderr:\n{stderr}"
    );
    assert_reports_line(&stdout, NOTHING_TO_CLEAN, "the second sweep");
    assert_reports_line(
        &stdout,
        &swept.registry_line_with(0),
        "the swept registry on the second sweep",
    );

    // The other direction of the control: the spared node was sweepable all
    // along, and the verb run from ITS cwd sweeps it.
    let (ok, stdout, stderr) = run_cerulion(&["clean", "--report-only"], spared.cwd());
    assert!(
        ok,
        "cerulion clean must exit 0 on the second root; stderr:\n{stderr}"
    );
    assert_reports_line(&stdout, ONE_NODE_CLEANED, "the spared node, once reached");
    assert_reports_line(
        &stdout,
        &spared.registry_line_with(0),
        "the second registry, now empty",
    );
    assert_never_names_the_machines_registry(&stdout);
    assert_eq!(
        spared.registry_contents(),
        BTreeMap::new(),
        "the second registry must hold nothing once its own cwd is swept"
    );
}
