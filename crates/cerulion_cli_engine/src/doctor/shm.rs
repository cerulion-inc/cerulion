// SPDX-License-Identifier: AGPL-3.0-only
//! `shm.dev-shm` and `shm.stale-nodes`: the shared memory local topics live in.
//!
//! `shm.dev-shm` reads `statvfs("/dev/shm")` on Linux, where iceoryx2 creates
//! every topic's shared memory, and compares the space an unprivileged
//! process may still use with one scheduler-trace ring
//! ([`cerulion_core::trace_ring::apparent_ring_bytes`] at the default
//! capacity), the reservation below which `graph run` starts without rings.
//! On macOS POSIX shared memory lives in a kernel namespace with no mount
//! behind it, so there is nothing to read.
//!
//! `shm.stale-nodes` classifies iceoryx2's node registry with
//! [`crate::ipc_cleanup::sweep_dead_nodes`] in [`SweepMode::ReportOnly`], the
//! same walk `cerulion clean --report-only` makes, and on the platforms that
//! keep `.shm_state` files it scans them with [`crate::shm_state::scan`] in
//! report-only mode. Nothing is removed.

// `decide_size` compiles and is tested on every host; only a Linux build
// calls it from `run_size`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use super::{Check, Context, Outcome, Reading, Status};
use crate::ipc_cleanup::SweepMode;

pub(super) const SIZE: Check = Check {
    id: "shm.dev-shm",
    why: "iceoryx2 creates every local topic's shared memory under /dev/shm on Linux; space \
          there is charged when a page is first written, so a full /dev/shm shows up as a \
          crash at first write rather than an error at start, and `graph run` starts without \
          scheduler-trace rings when one ring does not fit.",
    network: false,
    run: run_size,
};

pub(super) const STALE_NODES: Check = Check {
    id: "shm.stale-nodes",
    why: "a process that exits without shutting down (kill -9, a crash) leaves its iceoryx2 \
          node registered with its shared memory, which keeps using space and files until \
          something removes it.",
    network: false,
    run: run_stale,
};

/// What `statvfs("/dev/shm")` returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ShmSpace {
    /// The mount was read.
    Read {
        /// Total size in bytes.
        total: u64,
        /// Bytes an unprivileged process may still use.
        available: u64,
        /// Mounted read-only.
        read_only: bool,
    },
    /// No `/dev/shm`.
    Missing,
    /// The read failed for another reason.
    Unreadable(String),
}

/// The decision for `shm.dev-shm` over the reading and the size of one
/// scheduler-trace ring.
pub(super) fn decide_size(space: &ShmSpace, ring_bytes: u64) -> Outcome {
    match *space {
        ShmSpace::Missing => Outcome::new(
            Status::Fail,
            "/dev/shm does not exist, so iceoryx2 cannot create a local topic",
        )
        .with_fix("mount a tmpfs there: `sudo mount -t tmpfs tmpfs /dev/shm`"),
        ShmSpace::Unreadable(ref err) => {
            Outcome::new(Status::Warn, format!("could not read /dev/shm: {err}"))
                .with_fix("run `df -h /dev/shm` to see the mount")
        }
        ShmSpace::Read {
            total,
            available,
            read_only,
        } => {
            let base = |status, summary: String| {
                Outcome::new(status, summary)
                    .reading("total_bytes", Reading::Int(total))
                    .reading("available_bytes", Reading::Int(available))
                    .reading("trace_ring_bytes", Reading::Int(ring_bytes))
                    .reading("read_only", Reading::Bool(read_only))
            };
            if read_only {
                base(
                    Status::Fail,
                    "/dev/shm is mounted read-only, so iceoryx2 cannot create a local topic"
                        .to_string(),
                )
                .with_fix("remount it writable: `sudo mount -o remount,rw /dev/shm`")
            } else if available < ring_bytes {
                base(
                    Status::Warn,
                    format!(
                        "/dev/shm has {available} bytes available of {total}, less than the \
                         {ring_bytes} bytes one scheduler-trace ring reserves"
                    ),
                )
                .with_fix(format!(
                    "free space there (`cerulion clean` removes what dead nodes left), or \
                     enlarge it until at least {ring_bytes} bytes are available: `--shm-size` \
                     for a Docker container, `sudo mount -o remount,size=<bytes> /dev/shm` on \
                     a host"
                ))
            } else {
                base(
                    Status::Pass,
                    format!("/dev/shm has {available} bytes available of {total}"),
                )
            }
        }
    }
}

/// The readings `shm.stale-nodes` decides over.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct StaleReading {
    /// The node registry directory walked.
    pub(super) node_dir: String,
    /// Dead nodes the walk classified.
    pub(super) dead_nodes: usize,
    /// Lines about the walk itself (an unreadable registry); non-zero means
    /// the count is a lower bound.
    pub(super) registry_errors: usize,
    /// The `.shm_state` scan, on the platforms that keep those files; `None`
    /// elsewhere.
    pub(super) state_files: Option<StateFileScan>,
}

/// What the report-only `.shm_state` scan found.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct StateFileScan {
    /// The directory scanned.
    pub(super) dir: String,
    /// Files whose creator is provably gone, plus empty orphans.
    pub(super) dead: u64,
    /// Why the scan did not see every file: refused, out of budget, or
    /// entries skipped. `None` when it saw them all.
    pub(super) incomplete: Option<String>,
}

fn plural(n: u64, one: &str, many: &str) -> String {
    format!("{n} {}", if n == 1 { one } else { many })
}

/// The decision for `shm.stale-nodes`.
pub(super) fn decide_stale(r: &StaleReading) -> Outcome {
    let mut found = Vec::new();
    if r.dead_nodes > 0 {
        found.push(format!(
            "{} in {}",
            plural(
                len(r.dead_nodes),
                "dead iceoryx2 node",
                "dead iceoryx2 nodes"
            ),
            r.node_dir
        ));
    }
    if let Some(scan) = r.state_files.as_ref().filter(|s| s.dead > 0) {
        found.push(format!(
            "{} whose creator is gone in {}",
            plural(scan.dead, ".shm_state file", ".shm_state files"),
            scan.dir
        ));
    }
    let mut incomplete = Vec::new();
    if r.registry_errors > 0 {
        incomplete.push(format!(
            "the node registry {} could not be read in full ({})",
            r.node_dir,
            plural(len(r.registry_errors), "error", "errors")
        ));
    }
    if let Some(why) = r.state_files.as_ref().and_then(|s| s.incomplete.as_ref()) {
        incomplete.push(why.clone());
    }
    let mut outcome = if !incomplete.is_empty() {
        let so_far = if found.is_empty() {
            String::new()
        } else {
            format!("; found so far: {}", found.join(", "))
        };
        Outcome::new(Status::Warn, format!("{}{so_far}", incomplete.join("; ")))
            .with_fix("run `cerulion clean --report-only` to see what could not be read")
    } else if found.is_empty() {
        Outcome::new(
            Status::Pass,
            format!("no dead iceoryx2 node in {}", r.node_dir),
        )
    } else {
        Outcome::new(Status::Warn, format!("{} left", found.join(" and ")))
            .with_fix("run `cerulion clean`; it removes only what processes that are gone left")
    };
    outcome = outcome
        .reading("node_dir", Reading::Text(r.node_dir.clone()))
        .reading("dead_nodes", Reading::Int(len(r.dead_nodes)))
        .reading("registry_errors", Reading::Int(len(r.registry_errors)));
    if let Some(scan) = &r.state_files {
        outcome = outcome
            .reading("state_file_dir", Reading::Text(scan.dir.clone()))
            .reading("dead_state_files", Reading::Int(scan.dead))
            .reading(
                "state_scan_complete",
                Reading::Bool(scan.incomplete.is_none()),
            );
    }
    outcome
}

/// Why a report-only scan did not see every state file, or `None`.
#[cfg(unix)]
fn scan_incomplete(scan: &crate::shm_state::ShmStateReport) -> Option<String> {
    let dir = scan.dir.display();
    if let Some(refused) = &scan.refused {
        return Some(format!(
            "the state files in {dir} could not be scanned ({refused})"
        ));
    }
    if scan.truncated || scan.classification_truncated {
        return Some(format!(
            "the scan of {dir} ran out of its time budget after {} files",
            scan.counted
        ));
    }
    (scan.skipped_entries > 0).then(|| {
        format!(
            "the scan of {dir} skipped {}",
            plural(
                scan.skipped_entries,
                "entry it could not read",
                "entries it could not read"
            )
        )
    })
}

fn len(n: usize) -> u64 {
    u64::try_from(n).unwrap_or(u64::MAX)
}

#[cfg(target_os = "linux")]
fn read_space() -> ShmSpace {
    match rustix::fs::statvfs("/dev/shm") {
        Ok(stat) => ShmSpace::Read {
            total: stat.f_blocks.saturating_mul(stat.f_frsize),
            available: stat.f_bavail.saturating_mul(stat.f_frsize),
            read_only: stat.f_flag.contains(rustix::fs::StatVfsMountFlags::RDONLY),
        },
        Err(rustix::io::Errno::NOENT) => ShmSpace::Missing,
        Err(err) => ShmSpace::Unreadable(err.to_string()),
    }
}

#[cfg(target_os = "linux")]
fn run_size(_ctx: &Context) -> Outcome {
    let ring = cerulion_core::trace_ring::apparent_ring_bytes(
        cerulion_core::trace_ring::default_capacity_records(),
    );
    decide_size(&read_space(), ring)
}

#[cfg(not(target_os = "linux"))]
fn run_size(_ctx: &Context) -> Outcome {
    Outcome::not_applicable(
        "POSIX shared memory on this platform lives in the kernel, with no /dev/shm mount to \
         read",
    )
}

fn run_stale(_ctx: &Context) -> Outcome {
    let report = crate::ipc_cleanup::sweep_dead_nodes(SweepMode::ReportOnly);
    #[cfg(unix)]
    let state_files = crate::shm_state::platform_uses_shm_state_files().then(|| {
        let budget = crate::shm_state::SHM_STATE_REPORT_BUDGET;
        let scan = crate::shm_state::scan(
            std::path::Path::new(crate::shm_state::SHM_STATE_DIRECTORY),
            budget,
            crate::shm_state::classify_budget(budget),
            crate::shm_state::ScanMode::ReportOnly,
            &crate::shm_state::LibcProbe,
        );
        StateFileScan {
            dir: scan.dir.display().to_string(),
            dead: scan.proven_dead.saturating_add(scan.empty_orphans),
            incomplete: scan_incomplete(&scan),
        }
    });
    #[cfg(not(unix))]
    let state_files = None;
    decide_stale(&StaleReading {
        node_dir: crate::shm_state::iceoryx2_node_dir().display().to_string(),
        dead_nodes: report.dead_nodes.len(),
        registry_errors: report.registry_errors.len(),
        state_files,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const RING: u64 = 42_008_640;

    #[test]
    fn dev_shm_passes_with_room_for_a_ring() {
        let o = decide_size(
            &ShmSpace::Read {
                total: 1 << 30,
                available: RING,
                read_only: false,
            },
            RING,
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "/dev/shm has 42008640 bytes available of 1073741824"
        );
    }

    #[test]
    fn dev_shm_warns_one_byte_short_of_a_ring() {
        let o = decide_size(
            &ShmSpace::Read {
                total: 64 << 20,
                available: RING - 1,
                read_only: false,
            },
            RING,
        );
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "/dev/shm has 42008639 bytes available of 67108864, less than the 42008640 bytes \
             one scheduler-trace ring reserves"
        );
        assert_eq!(
            o.fix.as_deref(),
            Some(
                "free space there (`cerulion clean` removes what dead nodes left), or enlarge \
                 it until at least 42008640 bytes are available: `--shm-size` for a Docker \
                 container, `sudo mount -o remount,size=<bytes> /dev/shm` on a host"
            )
        );
        assert_eq!(
            o.readings.get("trace_ring_bytes"),
            Some(&Reading::Int(RING))
        );
    }

    #[test]
    fn dev_shm_fails_missing_or_read_only() {
        assert_eq!(decide_size(&ShmSpace::Missing, RING).status, Status::Fail);
        let ro = decide_size(
            &ShmSpace::Read {
                total: 1 << 30,
                available: 1 << 30,
                read_only: true,
            },
            RING,
        );
        assert_eq!(ro.status, Status::Fail);
        assert!(ro.fix.unwrap().contains("remount,rw"));
    }

    #[test]
    fn dev_shm_unreadable_warns() {
        let o = decide_size(&ShmSpace::Unreadable("EACCES".into()), RING);
        assert_eq!(o.status, Status::Warn);
        assert!(o.fix.is_some());
    }

    fn reading(dead: usize, errors: usize, files: Option<u64>) -> StaleReading {
        StaleReading {
            node_dir: "/tmp/iceoryx2/nodes".to_string(),
            dead_nodes: dead,
            registry_errors: errors,
            state_files: files.map(|dead| StateFileScan {
                dir: "/tmp".to_string(),
                dead,
                incomplete: None,
            }),
        }
    }

    #[test]
    fn stale_passes_on_a_clean_registry() {
        let o = decide_stale(&reading(0, 0, Some(0)));
        assert_eq!(o.status, Status::Pass);
        assert_eq!(o.summary, "no dead iceoryx2 node in /tmp/iceoryx2/nodes");
        assert_eq!(o.fix, None);
        assert_eq!(
            o.readings.get("state_scan_complete"),
            Some(&Reading::Bool(true))
        );
    }

    #[test]
    fn stale_warns_on_dead_nodes_and_names_clean() {
        let o = decide_stale(&reading(3, 0, None));
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "3 dead iceoryx2 nodes in /tmp/iceoryx2/nodes left"
        );
        assert_eq!(
            o.fix.as_deref(),
            Some("run `cerulion clean`; it removes only what processes that are gone left")
        );
        assert_eq!(o.readings.get("dead_state_files"), None);
    }

    #[test]
    fn stale_names_each_place_and_counts_in_the_singular() {
        let o = decide_stale(&reading(1, 0, Some(1)));
        assert_eq!(
            o.summary,
            "1 dead iceoryx2 node in /tmp/iceoryx2/nodes and 1 .shm_state file whose creator is \
             gone in /tmp left"
        );
    }

    #[test]
    fn stale_registry_errors_warn_and_point_at_the_report() {
        let o = decide_stale(&reading(1, 2, None));
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "the node registry /tmp/iceoryx2/nodes could not be read in full (2 errors); found \
             so far: 1 dead iceoryx2 node in /tmp/iceoryx2/nodes"
        );
        assert!(o.fix.unwrap().contains("--report-only"));
    }

    #[test]
    fn an_incomplete_state_file_scan_warns_instead_of_passing() {
        let mut r = reading(0, 0, Some(0));
        if let Some(scan) = &mut r.state_files {
            scan.incomplete =
                Some("the scan of /tmp ran out of its time budget after 9 files".into());
        }
        let o = decide_stale(&r);
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "the scan of /tmp ran out of its time budget after 9 files"
        );
        assert_eq!(
            o.readings.get("state_scan_complete"),
            Some(&Reading::Bool(false))
        );
    }
}
