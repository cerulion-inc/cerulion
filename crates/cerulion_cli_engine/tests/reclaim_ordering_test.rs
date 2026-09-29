#![cfg(all(unix, any(target_os = "macos", target_os = "freebsd")))]
//! A `.shm_state` file must not be reclaimed while a dead iceoryx2
//! node that needs it is still registered.
//!
//! # What this pins, and why it needed measuring
//!
//! On the platforms `iceoryx2-pal-posix` gives the state-file indirection
//! (macOS, FreeBSD), `/tmp/<name>.shm_state` is the ONLY mapping from an
//! iceoryx2 resource name to the real POSIX object behind it —
//! `get_real_shm_name` reads it on every `shm_open` and `shm_unlink`
//! (`macos/mman.rs`). `shm_state`'s own module doc already knew half of this:
//! removing the file "would strand [the object] under an unresolvable name".
//!
//! The half nobody had measured is what it costs the REGISTRY. A dead node's
//! details, and the services it must be deregistered from, live in exactly such
//! objects, so once the mapping is gone nothing can name them again.
//!
//! iceoryx2 0.10 changed what that failure LOOKS like, and not for the better.
//! Up to 0.9.1 the unresolvable name propagated: `try_cleanup_dead_nodes` could
//! never reap that node again, on this run or any future one, and the permanent
//! node directory was the visible symptom — which is what
//! `cerulion_cli/tests/trace_inspect_and_clean_cli_test.rs::
//! clean_sweeps_its_own_registry_and_leaves_another_root_untouched` asserts
//! against when it requires a second `clean` over the node it just swept to
//! find nothing. 0.10 removes the node entry anyway and reports the sweep as a
//! success, so the registry comes out clean while the segments stay in the
//! kernel under a name no process can reconstruct: the `.shm_state` file was
//! their only record, `shm_unlink` of the logical name answers ENOENT without
//! it, and `shm_list` is a readdir of those very files. A visible strand became
//! a SILENT orphan, which is why the ordering gate matters more under 0.10 and
//! not less. This test pins the new shape so the next reader measures it rather
//! than inferring it from a version number.
//!
//! # Shape
//!
//! One body, two legs against the same machine state, each with its own freshly
//! SIGKILLed child holding a real node + service + publisher:
//!
//! * **LEG A (control)** — state files left alone: one sweep reaps the node.
//! * **LEG B** — the state files removed FIRST, exactly as an ungated exit pass
//!   would: the sweep reports success, the entry comes off, and the segments
//!   behind it are still in the kernel with nothing left to name them.
//!
//! Without leg A, leg B proves nothing — a node that is simply not reapable on
//! this host would produce the same result.
//!
//! # Isolation
//!
//! Hermetic by PREFIX. Every resource lives under an isolated iceoryx2 config
//! whose `global.prefix` is unique per call, so the `/tmp` files this test
//! removes are provably its own and never a concurrent lane's. The node
//! REGISTRY directory is shared by every isolated config, so the oracle is the
//! identity of the entries this test's own child minted, never a count.

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use cerulion_cli_engine::shm_state;
use cerulion_core::transport::dead_node_sweep::{cleanup_dead_nodes_bounded, BoundedCleanup};
use iceoryx2::prelude::*;
use iceoryx2::service::ipc_threadsafe::Service as CerService;

const ENV_ROLE: &str = "PROBE_ROLE";
const ENV_CONFIG: &str = "PROBE_CONFIG";
const ENV_READY: &str = "PROBE_READY";

/// The real POSIX object name a state file records, trimmed at its NUL.
///
/// The first 33 bytes are the NUL-padded name `generate_real_shm_name` minted
/// (`<pid>_<sec>_<usec>_<counter>`); 0.10 writes four mode bytes after them,
/// which are not part of the name.
fn real_object_name(content: &[u8]) -> Option<std::ffi::CString> {
    let name: Vec<u8> = content
        .iter()
        .copied()
        .take(33)
        .take_while(|b| *b != 0)
        .collect();
    if name.is_empty() {
        return None;
    }
    std::ffi::CString::new(name).ok()
}

/// Does the kernel still hold this object?
///
/// Raw `libc` on purpose: the iceoryx2 call would resolve the LOGICAL name
/// through the state file, which is the very thing this leg removed. Asking the
/// kernel for the REAL name is the only question left that has an answer.
fn shm_object_exists(name: &std::ffi::CStr) -> bool {
    let fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDONLY) };
    if fd < 0 {
        return false;
    }
    unsafe { libc::close(fd) };
    true
}

/// Generous: this is a liveness bound on a child process, not a measurement.
const READY_TIMEOUT: Duration = Duration::from_secs(30);
/// Likewise — the sweep is given far more than it needs so a slow desk cannot
/// turn "reaped" into "deferred" and invert leg A.
const SWEEP_BUDGET: Duration = Duration::from_secs(60);

/// Child role: create a node + service + publisher on the handed config,
/// announce readiness, then park so the parent can SIGKILL it and leave a DEAD
/// node behind. A no-op pass when run without the role env, so the ordinary
/// suite is unaffected.
#[test]
#[ignore = "child entry point: driven by the parent test via self-re-exec"]
fn probe_child_entrypoint() {
    if std::env::var(ENV_ROLE).as_deref() != Ok("child") {
        return;
    }
    let cfg: iceoryx2::config::Config =
        serde_json::from_str(&std::env::var(ENV_CONFIG).expect("config env"))
            .expect("deserialize config");
    let node = NodeBuilder::new()
        .config(&cfg)
        .create::<CerService>()
        .expect("child node");
    let name: iceoryx2::service::service_name::ServiceName =
        "/probe".try_into().expect("service name");
    let service = node
        .service_builder(&name)
        .publish_subscribe::<[u8]>()
        .open_or_create()
        .expect("child service");
    let _publisher = service
        .publisher_builder()
        .create()
        .expect("child publisher");

    std::fs::write(std::env::var(ENV_READY).expect("ready env"), b"ready").expect("ready write");
    loop {
        std::thread::sleep(Duration::from_secs(60));
    }
}

fn prefix_of(cfg: &iceoryx2::config::Config) -> String {
    String::from_utf8(cfg.global.prefix.as_bytes().to_vec()).expect("prefix is utf8")
}

/// The `/tmp/*.shm_state` files minted under this config's unique prefix.
fn state_files(prefix: &str) -> Vec<PathBuf> {
    let mut out = vec![];
    let Ok(entries) = std::fs::read_dir(shm_state::SHM_STATE_DIRECTORY) else {
        return out;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if name.starts_with(prefix) && name.ends_with(".shm_state") {
            out.push(entry.path());
        }
    }
    out.sort();
    out
}

/// The node-registry directory is SHARED by every isolated config (only the
/// resource NAMES carry the prefix), so an entry COUNT is ambient noise from
/// other lanes. The oracle is the IDENTITY of the entries this test minted.
fn node_entry_names(cfg: &iceoryx2::config::Config) -> BTreeSet<String> {
    let dir = cfg.global.node_dir();
    let path = std::path::Path::new(core::str::from_utf8(dir.as_bytes()).expect("node dir utf8"));
    std::fs::read_dir(path)
        .map(|entries| {
            entries
                .flatten()
                .map(|e| e.file_name().to_string_lossy().to_string())
                .collect()
        })
        .unwrap_or_default()
}

/// SIGKILL + reap on drop, so a panicking assertion cannot leave a parked child.
struct ChildGuard(std::process::Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// A dead node planted for one leg: the state files it minted, and the registry
/// entries it owns.
struct DeadNode {
    files: Vec<PathBuf>,
    entries: BTreeSet<String>,
}

fn plant_dead_node(cfg: &iceoryx2::config::Config, tag: &str) -> DeadNode {
    let before = node_entry_names(cfg);
    let ready = std::env::temp_dir().join(format!("ready_{}_{tag}", std::process::id()));
    let _ = std::fs::remove_file(&ready);

    let exe = std::env::current_exe().expect("current exe");
    let mut child = ChildGuard(
        std::process::Command::new(exe)
            .args([
                "--exact",
                "probe_child_entrypoint",
                "--ignored",
                "--nocapture",
            ])
            .env(ENV_ROLE, "child")
            .env(ENV_CONFIG, serde_json::to_string(cfg).expect("serialize"))
            .env(ENV_READY, &ready)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .expect("spawn child"),
    );

    let deadline = Instant::now() + READY_TIMEOUT;
    while Instant::now() < deadline && !ready.exists() {
        std::thread::sleep(Duration::from_millis(25));
    }
    assert!(
        ready.exists(),
        "the child never announced readiness within {READY_TIMEOUT:?}"
    );

    let files = state_files(&prefix_of(cfg));
    let entries: BTreeSet<String> = node_entry_names(cfg).difference(&before).cloned().collect();
    assert!(
        !files.is_empty(),
        "precondition: the child must have minted state files under its own prefix"
    );
    assert!(
        !entries.is_empty(),
        "precondition: the child must have minted a node-registry entry"
    );

    // SIGKILL and reap: the node is now DEAD in the registry, and every state
    // file above records a pid that no longer exists.
    let pid = child.0.id() as i32;
    // SAFETY: `pid` is this test's own just-spawned child, still unreaped.
    unsafe { libc::kill(pid, libc::SIGKILL) };
    let _ = child.0.wait();
    let _ = std::fs::remove_file(&ready);

    DeadNode { files, entries }
}

/// How many of THIS test's own registry entries still exist.
fn surviving(cfg: &iceoryx2::config::Config, planted: &DeadNode) -> usize {
    let live = node_entry_names(cfg);
    planted.entries.iter().filter(|e| live.contains(*e)).count()
}

fn sweep(cfg: &iceoryx2::config::Config) -> BoundedCleanup {
    cleanup_dead_nodes_bounded(cfg, SWEEP_BUDGET)
}

#[test]
fn state_files_removed_before_the_reap_orphan_the_segments_silently() {
    // ---------------- LEG A: the control ----------------
    let cfg_a = cerulion_core::testing::iceoryx_test_config();
    let a = plant_dead_node(&cfg_a, "lega");
    let swept_a = sweep(&cfg_a);
    let left_a = surviving(&cfg_a, &a);
    println!(
        "LEG A (state files intact): cleanups={} failed={} deferred={} own entries left={left_a}",
        swept_a.cleanups, swept_a.failed_cleanups, swept_a.deferred
    );
    // NOT asserted: `swept_a.failed_cleanups == 0`. The isolated config shares
    // its registry ROOT with every other test on the machine, so that counter
    // also reports THEIR residue — an ambient-dependent oracle that would read
    // a dirty runner as this change regressing (the class). The control
    // that is genuinely about this test is its own entry being reaped.
    assert_eq!(
        left_a, 0,
        "control: with its state files intact the dead node must be reaped in ONE sweep — \
         without this, leg B below proves nothing"
    );

    // ---------------- LEG B: reclaim first ----------------
    let cfg_b = cerulion_core::testing::iceoryx_test_config();
    let b = plant_dead_node(&cfg_b, "legb");

    // The production evidence gate really does target these files: the creator
    // is gone, so each parses to `ProvenDead` and an ungated pass would remove
    // every one of them. Asserted through the SHIPPED classifier rather than
    // assumed, so this leg cannot drift away from what the reclaimer does.
    let mut proven_dead = 0usize;
    // The real object names, read BEFORE the files go: after that they cannot be
    // recovered from anything, which is the whole point of the leg.
    let mut real_names: Vec<std::ffi::CString> = Vec::new();
    for file in &b.files {
        let read = shm_state::read_state_file(file);
        if let Ok(content) = read.as_ref().map(|r| r.content.clone()) {
            if let Some(name) = real_object_name(&content) {
                real_names.push(name);
            }
        }
        if matches!(
            shm_state::classify(read.as_ref().map_err(|e| e.clone()), &shm_state::LibcProbe),
            shm_state::StateFileVerdict::ProvenDead(_)
        ) {
            proven_dead += 1;
        }
    }
    assert!(
        proven_dead > 0,
        "precondition: the shipped classifier must call at least one of the dead child's \
         {} state file(s) ProvenDead — otherwise this leg is not modelling the reclaimer",
        b.files.len()
    );

    for file in &b.files {
        let _ = std::fs::remove_file(file);
    }
    println!(
        "LEG B: removed {} state file(s), {proven_dead} of them ProvenDead",
        b.files.len()
    );

    let swept_b = sweep(&cfg_b);
    println!(
        "LEG B sweep 1: cleanups={} failed={} deferred={} own entries left={}",
        swept_b.cleanups,
        swept_b.failed_cleanups,
        swept_b.deferred,
        surviving(&cfg_b, &b)
    );

    // PERMANENCE is the point: not "deferred to next time" but "no future sweep
    // can ever fix it", which is exactly the convergence the `cerulion clean`
    // CLI test requires.
    for round in 2..=3 {
        let s = sweep(&cfg_b);
        println!(
            "LEG B sweep {round}: cleanups={} failed={} deferred={} own entries left={}",
            s.cleanups,
            s.failed_cleanups,
            s.deferred,
            surviving(&cfg_b, &b)
        );
    }
    let stranded = surviving(&cfg_b, &b);
    // Leg B's residue is permanently unreapable BY CONSTRUCTION, so this test
    // must not leave it behind — that ratchet is the very thing being fixed.
    // Removed by hand, scoped to the entries this test minted.
    let dir = cfg_b.global.node_dir();
    let root = std::path::Path::new(core::str::from_utf8(dir.as_bytes()).expect("node dir utf8"));
    for entry in &b.entries {
        let _ = std::fs::remove_dir_all(root.join(entry));
    }

    // THE 0.10 SHAPE, in two halves. The registry comes clean...
    assert_eq!(
        stranded, 0,
        "0.10 removes a dead node's registry entry even when its state files are \
         already gone, so nothing survives here; if THIS starts failing with the \
         entries back, the platform went back to propagating the unresolvable name \
         and the 0.9.1 shape (a visible strand) is what to pin again"
    );

    // ...and the segments do not, which is the defect the ordering + convergence
    // gate exists to avoid. Removing the file does NOT unlink the object: the
    // shipped reclaimer unlinks the OBJECT FIRST and only then the file, and this
    // leg models the wrong order on purpose to price it.
    assert!(
        !real_names.is_empty(),
        "precondition: at least one state file must have yielded a real object name, \
         or the leak half of this oracle is vacuous"
    );
    // Probe and unlink in ONE pass, before anything can panic. These orphans are
    // reachable only through names that live nowhere but this function's stack, so
    // an assertion that fired first would leave them for the life of the machine
    // on exactly the run that found a problem.
    let mut present_before_unlink = 0usize;
    let mut present_after_unlink = 0usize;
    for name in &real_names {
        if shm_object_exists(name) {
            present_before_unlink += 1;
        }
        unsafe { libc::shm_unlink(name.as_ptr()) };
        if shm_object_exists(name) {
            present_after_unlink += 1;
        }
    }

    assert_eq!(
        present_before_unlink,
        real_names.len(),
        "every segment whose mapping this leg removed must still be in the kernel, \
         unreachable under any name a later sweep could form — that silent orphan is \
         what the gate prevents, and it is strictly worse than 0.9.1's strand because \
         no sweep reports it"
    );
    // The CONTROL: without it, a probe that answered `true` for everything would
    // satisfy the assertion above while measuring nothing.
    assert_eq!(
        present_after_unlink, 0,
        "control: the probe must distinguish present from absent, or the leak \
         assertion above proves nothing"
    );
}
