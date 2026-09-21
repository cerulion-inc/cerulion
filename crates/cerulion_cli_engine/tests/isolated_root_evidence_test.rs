#![cfg(all(unix, any(target_os = "macos", target_os = "freebsd")))]
//! A `.shm_state` mapping belonging to an ISOLATED-ROOT iceoryx2
//! namespace must not be reclaimed while that namespace still has a node
//! registered.
//!
//! # What this pins, and why it needed measuring
//!
//! The evidence a reclaimer must have is already established: a `.shm_state` file
//! is the whole NAMESPACE's shared management mapping, so "the creating process
//! is gone" says nothing about whether the segment is still needed — a namespace
//! with any registered node still needs its name mapping, and removing it splits
//! the namespace in two.
//!
//! What first shipped read only the GLOBAL config's registry. The two halves
//! of the layout are not rooted the same way:
//!
//! * a `.shm_state` file is written to `iceoryx2_pal_configuration::
//!   TEMP_DIRECTORY` — `/tmp/` — **whatever `global.root_path` says**
//!   (`iceoryx2-pal-posix-0.9.1/src/macos/settings.rs:15`, used by
//!   `macos/mman.rs`'s `shm_file_path`);
//! * a node registry is `<global.root_path>/<global.node.directory>`
//!   (`iceoryx2-0.9.1/src/config.rs:254`).
//!
//! So `iceoryx2::testing::generate_isolated_config`, which points `root_path` at
//! `TEST_DIRECTORY` (`/tmp/iceoryx2/tests/`, one entry below the compiled-in
//! root), registers its nodes somewhere the global-root-only evidence never
//! looks while dropping its mappings into the very `/tmp/` that `cerulion clean`
//! and the graceful-exit sweep walk.
//!
//! That is not a hypothetical: on the desk this was written, `/tmp` held 1412
//! `test_prefix_*.shm_state` files and `/tmp/iceoryx2/tests/nodes/` held live
//! registrations from concurrently running suites.
//!
//! # Shape
//!
//! A REAL node on a REAL isolated config, and the SHIPPED probe's evidence read
//! against the REAL `.shm_state` files that node minted. Two legs in one body,
//! against the same machine state:
//!
//! * **ALIVE** — the node is up, so its mappings must be covered (refused).
//!   Asserted beside the global-root-only walk, which must MISS them: that
//!   contrast is the defect, and without it the arm could pass on a fix that
//!   simply refused everything.
//! * **DEAD** — the node is dropped and deregistered, so whatever mapping it
//!   leaked must stop being covered. This is the anti-tautology half: covering
//!   the isolated root must not make the reclaimer inert, which is the failure
//!   mode the first evidence walk itself hit when one monitor file blinded it.
//!
//! # What it never does
//!
//! It reads evidence and it removes ITS OWN leaked state files. It never runs a
//! reclaim pass over `/tmp` and never unlinks a shared-memory object, so it is
//! safe beside other lanes running real-iceoryx2 suites on the same desk.

use std::path::{Path, PathBuf};
use std::time::Duration;

use cerulion_cli_engine::shm_state::{self, NamespacesInUse, SystemProbe};
use iceoryx2::prelude::*;
use iceoryx2::service::ipc_threadsafe::Service as CerService;

/// Generous: a bound on a filesystem walk, not a measurement.
const EVIDENCE_BUDGET: Duration = Duration::from_secs(10);

/// How many times an `Unknown` reading is re-taken before it is believed.
///
/// `namespaces_in_use_at` answers `Unknown` when a registry entry cannot be
/// listed, and a node directory being CREATED or REAPED by a concurrent lane
/// can lose that race — the walk lists the entry, the entry is gone by the time
/// its contents are read. That is a transient property of a busy desk, not of
/// this change; it is fail-CLOSED (a refusal, never a wrong reclamation), and
/// on a quiet desk it never happens at all.
///
/// Re-taking it does not weaken the oracle: an implementation that answers
/// `Unknown` because it stopped searching answers `Unknown` every time, so an
/// "always Unknown" variant exhausts these retries and fails against this
/// file.
const UNKNOWN_RETRIES: usize = 5;

fn prefix_of(cfg: &iceoryx2::config::Config) -> String {
    String::from_utf8(cfg.global.prefix.as_bytes().to_vec()).expect("prefix is utf8")
}

/// The `/tmp/*.shm_state` files minted under this config's unique prefix.
///
/// The prefix is per-call unique (`UniqueSystemId`), so this is provably this
/// test's own population and never a concurrent lane's.
fn state_file_names(prefix: &str) -> Vec<String> {
    let mut out = vec![];
    let Ok(entries) = std::fs::read_dir(shm_state::SHM_STATE_DIRECTORY) else {
        return out;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if name.starts_with(prefix) && name.ends_with(shm_state::SHM_STATE_SUFFIX) {
            out.push(name);
        }
    }
    out.sort();
    out
}

fn remove_state_files(names: &[String]) {
    for name in names {
        let _ = std::fs::remove_file(Path::new(shm_state::SHM_STATE_DIRECTORY).join(name));
    }
}

/// The registry directory the isolated config declares, as a `PathBuf`.
fn isolated_registry(cfg: &iceoryx2::config::Config) -> PathBuf {
    PathBuf::from(String::from(&cfg.global.node_dir()))
}

/// Re-take a reading past a transient `Unknown` — see [`UNKNOWN_RETRIES`].
fn past_transient_unknown(mut take: impl FnMut() -> NamespacesInUse) -> NamespacesInUse {
    let mut last = NamespacesInUse::Unknown;
    for _ in 0..=UNKNOWN_RETRIES {
        last = take();
        if matches!(last, NamespacesInUse::Known(_)) {
            return last;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    last
}

/// The SHIPPED probe's evidence — the union this change produces.
fn evidence() -> NamespacesInUse {
    past_transient_unknown(|| shm_state::LibcProbe.namespaces_in_use(EVIDENCE_BUDGET))
}

/// The global-root-only evidence: one registry, no search.
///
/// Also re-taken, and for a reason that matters more here than above:
/// `Unknown::covers` answers TRUE for every name, so a lost race would satisfy
/// a "must MISS this" precondition for the wrong reason.
fn evidence_at(dir: &Path) -> NamespacesInUse {
    past_transient_unknown(|| shm_state::namespaces_in_use_at(dir, EVIDENCE_BUDGET))
}

#[test]
fn a_live_isolated_root_namespaces_mapping_is_covered_by_the_shipped_evidence() {
    let cfg = cerulion_core::testing::iceoryx_test_config();
    let prefix = prefix_of(&cfg);
    let registry = isolated_registry(&cfg);

    // Precondition, asserted rather than assumed: the isolated config really
    // does root its registry somewhere OTHER than the global one. If iceoryx2
    // ever stopped doing that, every assertion below would be vacuous.
    let global_registry = shm_state::iceoryx2_node_dir();
    assert_ne!(
        registry, global_registry,
        "precondition: `iceoryx_test_config` must root its registry away from the global one \
         — otherwise this test is not about isolated roots at all"
    );

    // ------------------------------- ALIVE -------------------------------
    let node = NodeBuilder::new()
        .config(&cfg)
        .create::<CerService>()
        .expect("node on the isolated config");
    let service_name: iceoryx2::service::service_name::ServiceName =
        "/probe".try_into().expect("service name");
    let service = node
        .service_builder(&service_name)
        .publish_subscribe::<[u8]>()
        .open_or_create()
        .expect("service");
    let publisher = service.publisher_builder().create().expect("publisher");

    let alive_files = state_file_names(&prefix);
    assert!(
        !alive_files.is_empty(),
        "precondition: a live node must have minted `.shm_state` files under its own prefix in \
         {} — this test has nothing to say otherwise",
        shm_state::SHM_STATE_DIRECTORY
    );

    let alive = evidence();
    let NamespacesInUse::Known(prefixes) = &alive else {
        panic!(
            "the shipped probe answered Unknown {} times running — that is fail-closed and \
             therefore safe, but it makes this arm unable to prove anything, and a PERSISTENT \
             Unknown means the search stopped searching rather than losing a race",
            UNKNOWN_RETRIES + 1
        );
    };
    assert!(
        prefixes.contains(&prefix),
        "the live isolated-root namespace `{prefix}` is registered under {} but is absent from \
         the evidence — its {} mapping(s) would be reclaimed under a running suite. \
         evidence carried {} prefix(es)",
        registry.display(),
        alive_files.len(),
        prefixes.len()
    );
    for name in &alive_files {
        assert!(
            alive.covers(name),
            "`{name}` belongs to a namespace with a registered node and must be refused"
        );
    }

    // The defect, as an in-body assertion: the earlier evidence — the
    // global registry alone — misses every one of these. Without this the arm
    // above could be satisfied by a walk that never had a blind spot.
    let global_only = evidence_at(&global_registry);
    for name in &alive_files {
        assert!(
            !global_only.covers(name),
            "precondition: the global-root-only walk must MISS `{name}` — if it covers it, the \
             isolated root is no longer separate and this test proves nothing"
        );
    }

    // ------------------------------- DEAD --------------------------------
    // Dropping the node deregisters it. Whatever mapping it leaves behind is
    // then a genuine orphan and must become reclaimable again.
    drop(publisher);
    drop(service);
    drop(node);

    let dead_files = state_file_names(&prefix);
    let after = evidence();
    let NamespacesInUse::Known(prefixes_after) = &after else {
        remove_state_files(&dead_files);
        panic!(
            "the shipped probe answered Unknown {} times running after the node was dropped",
            UNKNOWN_RETRIES + 1
        );
    };
    assert!(
        !prefixes_after.contains(&prefix),
        "anti-tautology: once the node is gone its namespace must leave the evidence — a fix \
         that refused every prefix forever would make `cerulion clean` inert, which is the \
         blinded-walk failure mode this must not reintroduce"
    );
    for name in &dead_files {
        assert!(
            !after.covers(name),
            "`{name}` outlived its namespace and must be reclaimable again"
        );
    }

    // This test's own leak, removed by hand. Nothing else in `/tmp` is touched.
    remove_state_files(&dead_files);
    remove_state_files(&alive_files);
}
