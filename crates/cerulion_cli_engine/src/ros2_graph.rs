// SPDX-License-Identifier: AGPL-3.0-only
//! `ros2:` graph entries — spawn + supervise stock ROS 2 processes beside a
//! native graph under `cerulion graph run`.
//!
//! The mixed-stack shape: ONE graph file describes the whole robot — native
//! Cerulion nodes (`type:`) and stock ROS 2 nodes (`ros2:`) — and `graph run`
//! brings all of it up on one transport. This module is the ROS 2 half.
//!
//! **v1 is spawn + supervise ONLY.** A ros2 entry is an opaque child process
//! on the staged transport env (`RMW_IMPLEMENTATION=rmw_cerulion` — the SAME
//! env `cerulion ros2 run` stages, via [`crate::ros2_cmd::stage_base_child_env`]).
//! It is not scheduled: no trigger policy, no DAG level, no barrier seat, no
//! `WorkerPlan`, and no determinism claim — its topics meet native nodes on
//! the shared transport by NAME. `GraphConfig::take_ros2_nodes` lifts the
//! entries out BEFORE the native half is validated-for-build, partitioned,
//! recorded or built, so every downstream seam sees a plain native graph.
//!
//! Supervision (the multi-process supervisor's vocabulary, applied to a
//! non-DAG child): each entry spawns in its OWN process group (a terminal
//! Ctrl-C reaches the graph process, which drives the children's teardown
//! explicitly — the `bagd` precedent), a death-watch thread polls
//! `try_wait`, and a child's exit is judged by [`classify_child_exit`]
//! against the run's `--peer-loss` policy. Teardown = a per-child
//! freeze → peek → SIGINT (so a child's own death is judged and its response
//! to our signal never is — no window between the two), a bounded grace for
//! `ros2 launch` to wind its nodes down, then a SIGKILL backstop to the
//! child's whole PROCESS GROUP (each entry is spawned as a group leader, so
//! the launched nodes go with it); a leader that dies on its OWN takes its
//! group with it before it is reaped, on every path that reaps one — with
//! two stated residuals: a group kill the kernel refuses is logged loudly
//! and the leader alone is reaped, and a leader whose `waitid` PEEK the
//! kernel refuses is judged from `try_wait` alone with its group untouched;
//! `Drop` is the never-orphan floor on error paths (no grace, no SIGINT —
//! the group is SIGKILLed at once) and takes the same group.

use std::path::{Path, PathBuf};
use std::process::ExitStatus;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::graph::config::Ros2NodeDef;
use cerulion_core::graph::GraphConfig;

use crate::error::{CliError, CliResult};
use crate::graph_cmd::{ChildGuard, ChildPoll, PeerLossPolicy};
use crate::ros2_cmd;

/// Death-watch poll cadence — how quickly a child's exit is noticed.
pub const ROS2_CHILD_WATCH_POLL: Duration = Duration::from_millis(200);

/// Grace after the SIGINT fan-out before the SIGKILL backstop. Longer than
/// the gateway's 2 s: `ros2 launch` escalates SIGINT → SIGTERM → SIGKILL to
/// ITS nodes on ~5 s windows, and cutting that short would orphan them.
pub const ROS2_CHILD_SHUTDOWN_GRACE: Duration = Duration::from_secs(10);

/// One `ros2:` entry lifted out of the graph.
#[derive(Debug, Clone)]
pub struct Ros2Entry {
    /// The graph node id (the `id:` of the entry).
    pub id: String,
    /// The `ros2:` block.
    pub def: Ros2NodeDef,
}

/// Lift every `ros2:` entry out of `config` (declaration order), leaving a
/// plain native graph behind. The ONE split seam every run/levels/validate
/// path uses.
pub fn take_ros2_entries(config: &mut GraphConfig) -> Vec<Ros2Entry> {
    config
        .take_ros2_nodes()
        .into_iter()
        .map(|node| Ros2Entry {
            id: node.id,
            def: node.ros2.unwrap_or_default(),
        })
        .collect()
}

/// Human-readable one-line rendering of what an entry runs (`ros2 run pkg
/// exe` / `ros2 launch file`) — for reports and log lines.
pub fn describe(entry: &Ros2Entry) -> String {
    format!("ros2 {}", entry.def.argv(None).join(" "))
}

/// The spawn argv (everything after `ros2`), with a RELATIVE `launch:` /
/// `params_file:` resolved against `workspace_root` — and both checked to
/// EXIST, loudly, before anything is spawned (a missing launch file would
/// otherwise be reinterpreted by `ros2 launch` as a PACKAGE name).
pub fn spawn_argv(entry: &Ros2Entry, workspace_root: &Path) -> CliResult<Vec<String>> {
    let resolve = |raw: &str| -> PathBuf {
        let p = Path::new(raw);
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            workspace_root.join(p)
        }
    };
    // `ros2` is handed a STRING, so a resolved path that is not UTF-8 would
    // reach it lossily rendered — and `ros2 launch` reads a mangled path as a
    // PACKAGE name, the exact misread the preflight exists to prevent. Refuse
    // it by name instead.
    let utf8 = |entry_id: &str, what: &str, path: &Path| -> CliResult<String> {
        path.to_str().map(str::to_owned).ok_or_else(|| {
            CliError::Validation(format!(
                "ros2 entry '{entry_id}': the resolved {what} path is not valid UTF-8 and cannot \
                 be handed to `ros2` as an argument: {}",
                path.display()
            ))
        })
    };
    if let Some(launch) = &entry.def.launch {
        let path = resolve(launch);
        if !path.is_file() {
            return Err(CliError::Validation(format!(
                "ros2 entry '{}': launch file `{}` not found (resolved to `{}`) — `launch:` \
                 must name a launch FILE (relative paths resolve against the workspace root)",
                entry.id,
                launch,
                path.display()
            )));
        }
        return Ok(entry
            .def
            .argv(Some(&utf8(&entry.id, "launch file", &path)?)));
    }
    if let Some(file) = &entry.def.params_file {
        let path = resolve(file);
        if !path.is_file() {
            return Err(CliError::Validation(format!(
                "ros2 entry '{}': params_file `{}` not found (resolved to `{}`) — relative \
                 paths resolve against the workspace root",
                entry.id,
                file,
                path.display()
            )));
        }
        let mut def = entry.def.clone();
        def.params_file = Some(utf8(&entry.id, "params_file", &path)?);
        return Ok(def.argv(None));
    }
    Ok(entry.def.argv(None))
}

/// Stage the env every ros2 child inherits: resolve the Cerulion lib dir,
/// stage the minimal ament prefix, and build the base pairs — the identical
/// path `cerulion ros2 run` takes, so a graph's ROS 2 half and a bare launch
/// file see the same transport by construction. A missing
/// `librmw_cerulion.so` is a loud preflight error (the deployment
/// precondition, named with its remediation).
pub fn stage_ros2_child_env() -> CliResult<Vec<(String, String)>> {
    let lib_dir = ros2_cmd::resolve_lib_dir()?;
    let ament_prefix = ros2_cmd::stage_ament_prefix(&lib_dir)?;
    // A graph entry has no `--adopt-take`
    // flag to pass, so no launch gate has run for this child — the stager
    // refuses an ambient `CERULION_RMW_ADOPT_TAKE` rather than letting the
    // child adopt without the host, hook and preload-order checks.
    ros2_cmd::stage_base_child_env(&lib_dir, &ament_prefix, ros2_cmd::AdoptTakeGate::NotRun)
}

/// What a ros2 child's mid-run exit means for the run. Pure — the
/// death-watch's ONE decision, oracle-tested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildExitVerdict {
    /// Exit status 0: the process finished on its own terms. The graph keeps
    /// running under EITHER policy — a ros2 entry is a sidecar, not a DAG
    /// worker, and a one-shot tool finishing must not stop the robot.
    CleanContinue,
    /// Non-zero / lost under `--peer-loss continue`: warn, keep running.
    DegradedContinue,
    /// Non-zero / lost under `--peer-loss fail`: stop the run and report the
    /// failure (the run exits non-zero).
    StopRun,
}

/// `status = None` is a LOST child (`try_wait` itself errored) — treated as a
/// failure, never as alive.
pub fn classify_child_exit(status: Option<ExitStatus>, policy: PeerLossPolicy) -> ChildExitVerdict {
    match (status.map(|s| s.success()), policy) {
        (Some(true), _) => ChildExitVerdict::CleanContinue,
        (_, PeerLossPolicy::Continue) => ChildExitVerdict::DegradedContinue,
        (_, PeerLossPolicy::Fail) => ChildExitVerdict::StopRun,
    }
}

/// One spawned ros2 child.
struct Ros2Child {
    id: String,
    guard: ChildGuard,
}

impl Drop for Ros2Child {
    /// The never-orphan floor for THIS child: an un-reaped child dropped on
    /// any path takes its whole process group with it (the `ros2 launch`
    /// leader AND the nodes it started), not the leader alone. Runs before
    /// the guard's own `Drop`, which then finds the child reaped.
    fn drop(&mut self) {
        if !self.guard.reaped() {
            tracing::warn!(
                node_id = %self.id,
                "ros2 entry dropped un-reaped (an error path skipped the graceful teardown) — \
                 SIGKILL to its whole process group, no grace"
            );
        }
        self.guard.kill_group_and_reap();
    }
}

/// The supervised set of ros2 children for one run. Empty when the graph has
/// no `ros2:` entries (every method is then a no-op).
pub struct Ros2Children {
    /// Shared with the death-watch thread. The watcher's hold times are
    /// O(syscall); teardown holds it across its whole grace loop, sleeps
    /// included — deliberately, since the watcher checks `torn_down` before
    /// locking and so never judges a child the teardown is stopping.
    children: Arc<Mutex<Vec<Ros2Child>>>,
    /// Set when a child's exit demanded the run STOP (`--peer-loss fail`) —
    /// by the death-watch when it notices the death, or by teardown's own
    /// final pass (see [`judge_exits`]) for a death the watcher never saw;
    /// carries the message the run's `Err` names.
    failure: Arc<Mutex<Option<String>>>,
    /// Teardown handshake: flipped by `finish` / `Drop` so the watcher exits
    /// without judging our own kills.
    torn_down: Arc<AtomicBool>,
    /// The run's `--peer-loss` policy — the watcher's AND teardown's verdict
    /// rule, so the two cannot disagree.
    policy: PeerLossPolicy,
    /// The run's shutdown flag: a `StopRun` verdict flips it.
    running: Arc<AtomicBool>,
    graph: String,
}

fn lock_children(children: &Mutex<Vec<Ros2Child>>) -> std::sync::MutexGuard<'_, Vec<Ros2Child>> {
    children.lock().unwrap_or_else(|e| e.into_inner())
}

fn lock_failure(failure: &Mutex<Option<String>>) -> std::sync::MutexGuard<'_, Option<String>> {
    failure.lock().unwrap_or_else(|e| e.into_inner())
}

impl Ros2Children {
    /// The ONE constructor: an empty set carrying the run's policy and
    /// shutdown flag. (Struct-update syntax cannot be used on a `Drop` type,
    /// so every shape is built here.)
    fn with_policy(graph: &str, policy: PeerLossPolicy, running: Arc<AtomicBool>) -> Self {
        Self {
            children: Arc::new(Mutex::new(Vec::new())),
            failure: Arc::new(Mutex::new(None)),
            torn_down: Arc::new(AtomicBool::new(false)),
            policy,
            running,
            graph: graph.to_string(),
        }
    }

    /// A set over ALREADY-SPAWNED children with NO death-watch — the shape a
    /// failed watcher spawn leaves behind, built directly so the teardown
    /// verdict can be tested against real children without `ros2` or a
    /// staged `librmw_cerulion` on the machine.
    #[cfg(test)]
    fn for_test(
        graph: &str,
        policy: PeerLossPolicy,
        children: Vec<(&str, std::process::Child)>,
    ) -> Self {
        let set = Self::with_policy(graph, policy, Arc::new(AtomicBool::new(true)));
        let mut guarded = lock_children(&set.children);
        for (id, child) in children {
            guarded.push(Ros2Child {
                id: id.to_string(),
                guard: ChildGuard::new_group_leader(child, format!("ros2:{id}")),
            });
        }
        drop(guarded);
        set
    }

    /// Spawn every entry and start the death-watch. Fails LOUDLY — and tears
    /// down whatever it already spawned — on a missing `librmw_cerulion.so`
    /// (the env preflight), a missing launch / params file, `ros2` absent
    /// from `PATH`, or (under `--peer-loss fail`) a death-watch thread that
    /// cannot start. `running` is the run's shutdown flag: a `StopRun`
    /// verdict flips it so the graph drains exactly as on Ctrl-C, and
    /// [`Self::finish`] then turns the drained run into the `Err` the
    /// failure deserves.
    pub fn spawn(
        entries: &[Ros2Entry],
        workspace_root: &Path,
        graph: &str,
        running: Arc<AtomicBool>,
        policy: PeerLossPolicy,
    ) -> CliResult<Self> {
        let set = Self::with_policy(graph, policy, running);
        if entries.is_empty() {
            return Ok(set);
        }
        let env = stage_ros2_child_env()?;
        for entry in entries {
            let argv = spawn_argv(entry, workspace_root)?;
            let mut cmd = std::process::Command::new("ros2");
            cmd.args(&argv).current_dir(workspace_root);
            for (k, v) in &env {
                cmd.env(k, v);
            }
            #[cfg(unix)]
            {
                use std::os::unix::process::CommandExt;
                // Own process group: a terminal Ctrl-C reaches the graph
                // process, which drives this child's teardown explicitly
                // (`finish`) — one owner, one ordering, no double-SIGINT.
                cmd.process_group(0);
            }
            // ...and because WE are the one owner, our SIGINT must be
            // DELIVERABLE. An ignored disposition and a blocked signal BOTH
            // survive `execve`, so a launcher that ignores or blocks SIGINT
            // (a shell script's `cmd &`, `nohup` — `crate::child_signals`
            // lists the shapes) would hand this child a SIGINT it can never
            // receive — and
            // [`Self::teardown_gracefully`]'s per-PID
            // `signal_int_unless_exited` would then be a no-op, every stop
            // expiring [`ROS2_CHILD_SHUTDOWN_GRACE`] and ending in the
            // SIGKILL backstop that takes `ros2 launch`'s nodes down without
            // their own wind-down. `ros2` is a Python console script and
            // CPython PRESERVES an inherited `SIG_IGN`, so it cannot be
            // relied on to fix this for us. Same rule, same helper, same
            // reason as `connect_cmd::spawn_and_wait`.
            crate::child_signals::make_sigint_deliverable(&mut cmd);
            let child = cmd.spawn().map_err(|e| {
                if e.kind() == std::io::ErrorKind::NotFound {
                    CliError::Validation(format!(
                        "ros2 entry '{}': `ros2` not found on PATH — install ROS 2 (or source \
                         its setup script) so `graph run` can spawn `{}`",
                        entry.id,
                        describe(entry)
                    ))
                } else {
                    CliError::Validation(format!(
                        "ros2 entry '{}': failed to spawn `{}`: {e}",
                        entry.id,
                        describe(entry)
                    ))
                }
            })?;
            tracing::info!(
                graph = %graph,
                node_id = %entry.id,
                pid = child.id(),
                command = %describe(entry),
                "ros2 entry spawned on Cerulion transport (RMW_IMPLEMENTATION=rmw_cerulion)"
            );
            lock_children(&set.children).push(Ros2Child {
                id: entry.id.clone(),
                guard: ChildGuard::new_group_leader(child, format!("ros2:{}", entry.id)),
            });
        }
        // A refusal here drops `set`, and `Drop` reaps every child already
        // spawned — the "tears down whatever it already spawned" promise.
        set.spawn_death_watch()?;
        Ok(set)
    }

    /// True when no child was spawned.
    pub fn is_empty(&self) -> bool {
        lock_children(&self.children).is_empty()
    }

    /// Start the death-watch thread. Under `--peer-loss fail` a watcher that
    /// cannot start is a REFUSAL: that policy promises a death stops the run
    /// the moment it happens, and without the watcher a death is only judged
    /// at teardown (by [`judge_exits`]) — a run that kept going would carry
    /// a promise nothing enforces. Under `continue` a death never fails the
    /// run either way, so the loss is only promptness of the warn; say so
    /// and carry on.
    fn spawn_death_watch(&self) -> CliResult<()> {
        let children = Arc::clone(&self.children);
        let failure = Arc::clone(&self.failure);
        let torn_down = Arc::clone(&self.torn_down);
        let running = Arc::clone(&self.running);
        let policy = self.policy;
        let graph = self.graph.clone();
        let watch = move || loop {
            if torn_down.load(Ordering::Acquire) {
                break;
            }
            judge_exits(
                &mut lock_children(&children),
                policy,
                &failure,
                &running,
                &graph,
            );
            std::thread::sleep(ROS2_CHILD_WATCH_POLL);
        };
        let builder = std::thread::Builder::new().name("cer-ros2-watch".to_string());
        #[cfg(test)]
        let spawned = if FAIL_WATCHER_SPAWN_FOR_TEST.with(|f| f.replace(false)) {
            Err(std::io::Error::other("injected watcher spawn failure"))
        } else {
            builder.spawn(watch)
        };
        #[cfg(not(test))]
        let spawned = builder.spawn(watch);
        match (spawned, self.policy) {
            (Ok(_), _) => Ok(()),
            (Err(e), PeerLossPolicy::Fail) => Err(CliError::Validation(format!(
                "graph '{}': could not start the ros2 death-watch thread ({e}) — under \
                 --peer-loss fail a ros2 entry's death must stop the run the moment it \
                 happens, and without the watcher it would only be judged at teardown, so \
                 the run is refused rather than started with a promise nothing enforces. \
                 Raise the process/thread limit, or run with --peer-loss continue to \
                 supervise the ros2 entries at teardown only",
                self.graph
            ))),
            (Err(e), PeerLossPolicy::Continue) => {
                tracing::warn!(
                    graph = %self.graph,
                    error = %e,
                    "could not start the ros2 death-watch thread — a ros2 entry that exits \
                     mid-run is judged at teardown, not when it happens (the run keeps going \
                     either way under --peer-loss continue)"
                );
                Ok(())
            }
        }
    }

    /// End-of-run teardown: stop the watcher, SIGINT every live child, wait
    /// up to [`ROS2_CHILD_SHUTDOWN_GRACE`] for graceful exits, SIGKILL the
    /// stragglers — then fold a `StopRun` failure into the run's result (an
    /// `Ok` run whose ros2 child died under `--peer-loss fail` becomes the
    /// `Err` that names it; an existing `Err` is kept, the ros2 failure
    /// logged beside it).
    pub fn finish(&self, result: CliResult<()>) -> CliResult<()> {
        self.teardown_gracefully();
        let failure = lock_failure(&self.failure).take();
        match (result, failure) {
            (Ok(()), Some(msg)) => Err(CliError::Validation(msg)),
            (Err(e), Some(msg)) => {
                tracing::error!(graph = %self.graph, failure = %msg, "ros2 entry failure (run also failed)");
                Err(e)
            }
            (result, None) => result,
        }
    }

    fn teardown_gracefully(&self) {
        // Win the handshake first so the watcher never judges our own kills.
        if self.torn_down.swap(true, Ordering::AcqRel) {
            return;
        }
        let mut set = lock_children(&self.children);
        // Per child, freeze → peek → signal (`signal_int_unless_exited`): a
        // child that has ALREADY exited died on its own — the watcher may
        // have missed it (its 200 ms poll window, or a watcher that never
        // started under `--peer-loss continue`) — and is judged here exactly
        // as the watcher would have; a child still alive is frozen while we
        // look, so nothing can exit between the look and our SIGINT, and
        // every exit after it is ours and never judged. A judge-then-signal
        // pair has that window: an exit landing between the
        // two calls is reaped by the grace loop below with no failure
        // recorded, and `--peer-loss fail` reports success.
        for child in set.iter_mut().filter(|c| !c.guard.reaped()) {
            #[cfg(test)]
            run_before_signal_hook_for_test();
            let status = match child.guard.signal_int_unless_exited() {
                ChildPoll::Running => continue,
                ChildPoll::Exited(status) => Some(status),
                ChildPoll::Lost => None,
            };
            judge_exit(
                &child.id,
                status,
                self.policy,
                &self.failure,
                &self.running,
                &self.graph,
            );
        }
        if set.iter().all(|c| c.guard.reaped()) {
            return;
        }
        let deadline = Instant::now() + ROS2_CHILD_SHUTDOWN_GRACE;
        loop {
            let mut live = 0usize;
            for child in set.iter_mut().filter(|c| !c.guard.reaped()) {
                // Same group discipline on the way out: a leader that exits
                // after our SIGINT with a straggler node still up does not
                // strand it.
                if matches!(child.guard.try_status_reaping_group(), ChildPoll::Running) {
                    live += 1;
                }
            }
            if live == 0 {
                break;
            }
            if Instant::now() >= deadline {
                for child in set.iter_mut().filter(|c| !c.guard.reaped()) {
                    tracing::warn!(
                        graph = %self.graph,
                        node_id = %child.id,
                        "ros2 entry did not exit within the shutdown grace window — SIGKILL to \
                         its whole process group (the launched nodes go with it)"
                    );
                    child.guard.kill_group_and_reap();
                }
                break;
            }
            std::thread::sleep(ROS2_CHILD_WATCH_POLL);
        }
        tracing::info!(graph = %self.graph, "ros2 entries stopped");
    }
}

/// One supervision pass over every un-reaped child: reap the ones that have
/// exited and judge each exit with [`judge_exit`]. The death-watch runs it
/// every [`ROS2_CHILD_WATCH_POLL`]; teardown judges through the same
/// function, per child, from its freeze → peek → signal step, so the run's
/// verdict does not depend on the watcher having been alive, or awake, at
/// the moment a child died.
fn judge_exits(
    set: &mut [Ros2Child],
    policy: PeerLossPolicy,
    failure: &Mutex<Option<String>>,
    running: &AtomicBool,
    graph: &str,
) {
    for child in set.iter_mut().filter(|c| !c.guard.reaped()) {
        // A leader that died on its own takes its group with it before it
        // is reaped (a crashed `ros2 launch` must not strand its nodes).
        let status = match child.guard.try_status_reaping_group() {
            ChildPoll::Running => continue,
            ChildPoll::Exited(status) => Some(status),
            ChildPoll::Lost => None,
        };
        judge_exit(&child.id, status, policy, failure, running, graph);
    }
}

/// Judge ONE child's own exit by [`classify_child_exit`] — the single verdict
/// site for the watcher and for teardown. A `StopRun` records the FIRST
/// failure and flips `running`.
fn judge_exit(
    child_id: &str,
    status: Option<ExitStatus>,
    policy: PeerLossPolicy,
    failure: &Mutex<Option<String>>,
    running: &AtomicBool,
    graph: &str,
) {
    match classify_child_exit(status, policy) {
        ChildExitVerdict::CleanContinue => tracing::warn!(
            graph = %graph,
            node_id = %child_id,
            "ros2 entry exited cleanly mid-run — the graph keeps running \
             without it (a ros2 entry is a sidecar, not a scheduled node)"
        ),
        ChildExitVerdict::DegradedContinue => tracing::warn!(
            graph = %graph,
            node_id = %child_id,
            status = ?status,
            "ros2 entry DIED mid-run — the graph keeps running DEGRADED \
             (--peer-loss continue); it is not restarted in v1"
        ),
        ChildExitVerdict::StopRun => {
            tracing::error!(
                graph = %graph,
                node_id = %child_id,
                status = ?status,
                "ros2 entry DIED mid-run — stopping the run (--peer-loss fail)"
            );
            let mut failed = lock_failure(failure);
            if failed.is_none() {
                *failed = Some(format!(
                    "ros2 entry '{}' died mid-run ({}) under --peer-loss fail",
                    child_id,
                    status.map_or("lost".to_string(), |s| s.to_string())
                ));
            }
            running.store(false, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
thread_local! {
    /// Test seam: make the NEXT death-watch spawn on this thread fail, so the
    /// refusal arm is reachable without exhausting the process's threads.
    /// Consumed by the spawn that reads it (`replace(false)`).
    static FAIL_WATCHER_SPAWN_FOR_TEST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Test seam: a closure teardown runs ONCE, on this thread, in the
    /// instant before it freezes and signals its first child — the window a
    /// judge-then-signal teardown had between judging and signalling. A test
    /// uses it to make the child exit right there, deterministically.
    static BEFORE_SIGNAL_HOOK_FOR_TEST: std::cell::RefCell<Option<Box<dyn FnOnce()>>> =
        const { std::cell::RefCell::new(None) };
    /// Test seam: give the NEXT fixture child spawned on this thread the
    /// PRE-exec SIGINT disposition an ambient `SIG_IGN` would have left it,
    /// without changing this process's own. Consumed by the spawn that reads
    /// it (`replace(false)`). See `tests::poison_child_sigint_if_armed`.
    static POISON_CHILD_SIGINT_FOR_TEST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

#[cfg(test)]
fn run_before_signal_hook_for_test() {
    if let Some(hook) = BEFORE_SIGNAL_HOOK_FOR_TEST.with(|h| h.borrow_mut().take()) {
        hook();
    }
}

impl Drop for Ros2Children {
    /// The never-orphan floor: an error path that skipped [`Ros2Children::finish`] still
    /// reaps every child (each `Ros2Child`'s `Drop` SIGKILLs its whole process
    /// group + waits for the leader).
    fn drop(&mut self) {
        self.torn_down.store(true, Ordering::Release);
        lock_children(&self.children).clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an `ExitStatus` from a raw wait status (Unix): 0 = success,
    /// `code << 8` = a non-zero exit.
    #[cfg(unix)]
    fn status(code: i32) -> ExitStatus {
        use std::os::unix::process::ExitStatusExt;
        ExitStatus::from_raw(code << 8)
    }

    /// The death-watch's one decision, against a hand oracle: clean exit
    /// continues under BOTH policies; a non-zero exit and a LOST child
    /// follow the policy.
    #[cfg(unix)]
    #[test]
    fn classify_child_exit_follows_the_hand_oracle() {
        use ChildExitVerdict::*;
        let cases = [
            (Some(status(0)), PeerLossPolicy::Continue, CleanContinue),
            (Some(status(0)), PeerLossPolicy::Fail, CleanContinue),
            (Some(status(3)), PeerLossPolicy::Continue, DegradedContinue),
            (Some(status(3)), PeerLossPolicy::Fail, StopRun),
            (None, PeerLossPolicy::Continue, DegradedContinue),
            (None, PeerLossPolicy::Fail, StopRun),
        ];
        for (st, policy, expected) in cases {
            assert_eq!(
                classify_child_exit(st, policy),
                expected,
                "status={st:?} policy={policy:?}"
            );
        }
    }

    /// `spawn_argv` resolves a relative launch file against the workspace
    /// root, refuses a missing one by name, and resolves a relative
    /// `params_file` the same way.
    #[test]
    fn spawn_argv_resolves_relative_paths_and_refuses_missing_files() {
        let root = tempfile::tempdir().expect("tempdir");
        std::fs::create_dir_all(root.path().join("launch")).expect("mkdir");
        std::fs::write(root.path().join("launch/robot.launch.py"), b"#").expect("write");
        std::fs::write(root.path().join("params.yaml"), b"{}").expect("write");

        let launch = Ros2Entry {
            id: "bringup".to_string(),
            def: Ros2NodeDef {
                launch: Some("launch/robot.launch.py".to_string()),
                args: vec!["a:=1".to_string()],
                ..Default::default()
            },
        };
        let resolved = root.path().join("launch/robot.launch.py");
        assert_eq!(
            spawn_argv(&launch, root.path()).expect("resolves"),
            ["launch", &resolved.display().to_string(), "a:=1"]
        );

        let missing = Ros2Entry {
            id: "bringup".to_string(),
            def: Ros2NodeDef {
                launch: Some("launch/nope.launch.py".to_string()),
                ..Default::default()
            },
        };
        let err = spawn_argv(&missing, root.path()).expect_err("missing launch file");
        let msg = err.to_string();
        assert!(
            msg.contains("'bringup'") && msg.contains("nope.launch.py"),
            "{msg}"
        );

        let run = Ros2Entry {
            id: "mg".to_string(),
            def: Ros2NodeDef {
                package: Some("pkg".to_string()),
                executable: Some("exe".to_string()),
                params_file: Some("params.yaml".to_string()),
                ..Default::default()
            },
        };
        let params = root.path().join("params.yaml");
        assert_eq!(
            spawn_argv(&run, root.path()).expect("resolves"),
            [
                "run",
                "pkg",
                "exe",
                "--ros-args",
                "--params-file",
                &params.display().to_string()
            ]
        );
    }

    /// An empty set is a no-op for `finish` and reports empty.
    #[test]
    fn empty_set_is_a_no_op() {
        let set = Ros2Children::with_policy(
            "g",
            PeerLossPolicy::Continue,
            Arc::new(AtomicBool::new(true)),
        );
        assert!(set.is_empty());
        set.finish(Ok(())).expect("nothing to fold");
    }
    /// Arm the no-freeze seam for ONE teardown, and disarm it when the
    /// guard drops — whether or not the teardown consumed it. A seam armed
    /// for a child that is already dead is never consumed (the shared look
    /// returns before any per-platform code runs), and a thread-local left
    /// `true` would make a later live-child test on this thread skip its
    /// freeze.
    #[cfg(unix)]
    struct NoFreeze;

    #[cfg(unix)]
    impl NoFreeze {
        fn arm() -> Self {
            crate::graph_cmd::FAIL_FREEZE_FOR_TEST.with(|f| f.set(true));
            Self
        }
    }

    #[cfg(unix)]
    impl Drop for NoFreeze {
        fn drop(&mut self) {
            crate::graph_cmd::FAIL_FREEZE_FOR_TEST.with(|f| f.set(false));
        }
    }

    /// Spawn a shell child whose exit the test controls.
    #[cfg(unix)]
    fn sh(script: &str) -> std::process::Child {
        sh_with(
            script,
            std::process::Stdio::null(),
            std::process::Stdio::null(),
        )
    }

    /// As [`sh`], with the caller's stdin/stdout — and, like production, in
    /// its OWN process group (`process_group(0)`), so a group kill reaches
    /// what the shell started.
    #[cfg(unix)]
    fn sh_with(
        script: &str,
        stdin: std::process::Stdio,
        stdout: std::process::Stdio,
    ) -> std::process::Child {
        use std::os::unix::process::CommandExt;
        let mut cmd = std::process::Command::new("sh");
        cmd.arg("-c")
            .arg(script)
            .stdin(stdin)
            .stdout(stdout)
            .stderr(std::process::Stdio::null())
            .process_group(0);
        poison_child_sigint_if_armed(&mut cmd);
        reset_sigint_before_exec(&mut cmd);
        cmd.spawn().expect("spawn sh")
    }

    /// Give the child a DEFAULT SIGINT disposition before it execs.
    ///
    /// A fixture that must receive teardown's graceful SIGINT cannot be left at
    /// the mercy of the ambient disposition of the process that spawns it.
    /// `SIG_IGN` survives BOTH `fork` and `execve` (POSIX), and this binary
    /// really does have a window where SIGINT is ignored process-wide:
    /// `connect_cmd::tests::spawn_and_wait_forwards_sigint_even_when_parent_ignores_it`
    /// sets it to model a detached launcher. That test is `#[serial]`, but
    /// `#[serial]` only orders it against OTHER `#[serial]` tests, and the two
    /// thousand tests in this binary are not — so any fixture child spawned in
    /// that window inherits the ignored disposition, sits out the whole
    /// [`ROS2_CHILD_SHUTDOWN_GRACE`], and is reaped by the SIGKILL backstop.
    /// That is a SECOND, independent route to the same failure the
    /// [`sleeping_child`] leader shape fixed, and it defeats a shell fixture
    /// even harder: POSIX forbids a non-interactive shell from trapping a
    /// signal that was ignored on entry, so a `trap … INT` fixture cannot even
    /// INSTALL its handler.
    ///
    /// Production does exactly this, for exactly this reason — see
    /// `connect_cmd::spawn_and_wait`'s `pre_exec` reset.
    /// Model, for ONE spawn on THIS thread, the child disposition an ambient
    /// ignored SIGINT would have produced — without touching this process's
    /// own disposition.
    ///
    /// The property [`reset_sigint_before_exec`] defends is "the child's SIGINT
    /// disposition is `SIG_DFL` at `execve`, whatever it would otherwise have
    /// been". Inheritance is how that state arises in production, but it is not
    /// the property: a `signal(SIGINT, SIG_IGN)` in the child after `fork` and
    /// before `exec` leaves it in a state indistinguishable from having
    /// inherited one, and it is the state at `exec` that survives.
    ///
    /// So the pin models the cause rather than performing it, DELIBERATELY.
    /// Performing it means holding `SIG_IGN` process-wide across a spawn, and
    /// `#[serial]` orders a test only against other `#[serial]` tests — the
    /// ~2050 others in this binary are not, so any of them forking in that
    /// window inherits the ignored disposition and loses its own graceful
    /// teardown. That is the very hazard the reset exists to close, and a pin
    /// for it must not re-open it for everyone else.
    /// A thread-local seam has no reach beyond the thread that arms it.
    ///
    /// Armed by the test, consumed by the spawn that reads it, and registered
    /// SEPARATELY from the reset it poisons — so deleting the reset
    /// leaves the poison in place and the arm still fails.
    #[cfg(unix)]
    fn poison_child_sigint_if_armed(cmd: &mut std::process::Command) {
        use std::os::unix::process::CommandExt;
        if !POISON_CHILD_SIGINT_FOR_TEST.with(|p| p.replace(false)) {
            return;
        }
        // SAFETY: runs in the forked child before exec; `signal(2)` only.
        unsafe {
            cmd.pre_exec(|| {
                libc::signal(libc::SIGINT, libc::SIG_IGN);
                Ok(())
            });
        }
    }

    #[cfg(unix)]
    fn reset_sigint_before_exec(cmd: &mut std::process::Command) {
        use std::os::unix::process::CommandExt;
        // SAFETY: the closure runs in the forked child before exec and calls
        // only the async-signal-safe `signal(2)` — no allocation, no locks, no
        // shared state touched. Mirrors `connect_cmd::spawn_and_wait`.
        unsafe {
            cmd.pre_exec(|| {
                libc::signal(libc::SIGINT, libc::SIG_DFL);
                Ok(())
            });
        }
    }

    /// A LIVE child for the teardown arms: the sleeping process ITSELF is the
    /// leader, with no shell in between — and, like production and [`sh_with`],
    /// in its OWN process group.
    ///
    /// NOT a style preference. A `sh -c "sleep 30"` leader is a shell that
    /// FORKS `sleep` and blocks in `wait`, and a non-interactive shell waiting
    /// on a foreground job DEFERS SIGINT until that job exits — so teardown's
    /// per-PID SIGINT (`ChildGuard::signal_int`, which targets the LEADER, not
    /// the group) leaves the leader alive for the whole
    /// [`ROS2_CHILD_SHUTDOWN_GRACE`] and the SIGKILL backstop fires.
    ///
    /// Whether the shell forks at all varies by shell BUILD, so a `sh -c` leader
    /// puts the arms below at the mercy of whichever `/bin/sh` the host has:
    /// measured, macOS's bash-as-`sh` and its `/bin/dash` both exec (the leader
    /// IS `sleep`), while a dash 0.5.11 build on Linux forks (the
    /// leader stays `sh`). Upstream dash takes the exec path only when `EV_EXIT`
    /// survives `evalstring` and `have_traps()` is false — the candidate
    /// explanation for that split, not a measured one. Where the shell does
    /// fork there is a further race, between our SIGINT and the shell
    /// reaching `wait`: signal first and the leader dies of the default
    /// disposition (the arm passes), let the shell get there first and the
    /// signal is deferred (the arm fails) — a failure that is
    /// intermittent on an idle machine and deterministic on a loaded one.
    /// `the_freeze_fallback_still_judges_an_exit_it_can_see` asserts the
    /// backstop did NOT run, so it fails on that race; the three other arms
    /// that reach `finish` only assert the child was reaped, so they would pass a
    /// full grace window later. (The watcher-refusal `fail` arm never enters
    /// the grace loop at all — it tears down through `Drop`'s SIGKILL — and
    /// takes the helper for consistency.)
    ///
    /// A leader that is the sleeper has no such window on any shell: it dies of
    /// the SIGINT it is sent. It is also the shape production actually spawns —
    /// `Ros2Children::spawn` runs `Command::new("ros2")` directly, with the ROS
    /// env staged via `cmd.env`, never through a `sh -c` wrapper.
    #[cfg(unix)]
    fn sleeping_child() -> std::process::Child {
        use std::os::unix::process::CommandExt;
        let mut cmd = std::process::Command::new("sleep");
        cmd.arg("30")
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .process_group(0);
        // Removing the shell leaves ONE way for the ambient process state to
        // still defeat the graceful signal — an inherited `SIG_IGN`. See
        // [`reset_sigint_before_exec`].
        poison_child_sigint_if_armed(&mut cmd);
        reset_sigint_before_exec(&mut cmd);
        cmd.spawn().expect("spawn sleep")
    }

    /// Read a pipe to EOF on a helper thread, bounded: `true` = EOF within
    /// the budget (every holder of the write end is gone), `false` = still
    /// open at the deadline. A pipe every process in a tree inherits is the
    /// probe-free liveness oracle: it closes only when the LAST of them dies.
    #[cfg(unix)]
    fn pipe_reaches_eof_within(
        mut pipe: impl std::io::Read + Send + 'static,
        budget: Duration,
    ) -> bool {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut sink = Vec::new();
            let _ = pipe.read_to_end(&mut sink);
            let _ = tx.send(());
        });
        rx.recv_timeout(budget).is_ok()
    }

    /// A leader + grandchild tree sharing one pipe: `sh` backgrounds a
    /// `sleep`, then ANNOUNCES it on the pipe, and the test does not return
    /// until it has read the announcement — so the grandchild exists (and
    /// holds the pipe) before anything is killed. Without the handshake the
    /// leader can be killed before `sh` has forked the job, and BOTH arms of
    /// a group-kill test pass vacuously on a tree of one (measured: the
    /// control's pipe closed with the leader).
    #[cfg(unix)]
    fn spawn_tree() -> (
        std::process::Child,
        std::io::BufReader<std::process::ChildStdout>,
    ) {
        use std::io::BufRead;
        let mut child = sh_with(
            "sleep 60 & echo ready; wait",
            std::process::Stdio::null(),
            std::process::Stdio::piped(),
        );
        let mut pipe = std::io::BufReader::new(child.stdout.take().expect("piped stdout"));
        let mut line = String::new();
        pipe.read_line(&mut line).expect("read the announcement");
        assert_eq!(line, "ready\n", "the tree must announce its grandchild");
        (child, pipe)
    }

    /// Block until `pid` has exited WITHOUT reaping it — the child stays a
    /// zombie for the guard to reap, exactly the state a child that died
    /// while nobody was watching is in when teardown finds it. `waitid` +
    /// `WNOWAIT` is the deterministic primitive for that; a sleep would be a
    /// guess.
    #[cfg(unix)]
    fn wait_exited_without_reaping(pid: u32) {
        let mut info: libc::siginfo_t = unsafe { std::mem::zeroed() };
        // SAFETY: `waitid` on this process's OWN child pid, with a zeroed
        // siginfo_t out-param the kernel fills; WNOWAIT leaves the child
        // un-reaped so the owning `ChildGuard` still collects it. Through the
        // production retry helper, so a signal landing on the test process
        // reads as a retry, not a kernel refusal.
        unsafe {
            crate::graph_cmd::waitid_retrying_eintr(
                pid as libc::pid_t,
                &mut info,
                libc::WEXITED | libc::WNOWAIT,
            )
        }
        .unwrap_or_else(|e| panic!("waitid: {e}"));
    }

    /// True once the guard has REAPED `pid`: a non-blocking `waitid` on it
    /// answers `ECHILD` (no such un-waited child of ours) only after the
    /// child's status was collected. Deliberately not a `kill(pid, 0)`
    /// liveness probe — that is the `.shm_state` evidence predicate, which
    /// lives in one module — and stronger than one anyway: a zombie still
    /// answers `kill(0)`, and a reused pid would answer it for a stranger.
    #[cfg(unix)]
    fn is_reaped(pid: u32) -> bool {
        let mut info: libc::siginfo_t = unsafe { std::mem::zeroed() };
        // SAFETY: non-blocking `waitid` on a pid this process spawned, with
        // a zeroed siginfo_t out-param; WNOWAIT so a still-un-reaped child
        // (the failure case) is left for its guard.
        let outcome = unsafe {
            crate::graph_cmd::waitid_retrying_eintr(
                pid as libc::pid_t,
                &mut info,
                libc::WEXITED | libc::WNOHANG | libc::WNOWAIT,
            )
        };
        matches!(outcome, Err(e) if e.raw_os_error() == Some(libc::ECHILD))
    }

    /// The verdict does NOT depend on the watcher: a child that died before
    /// teardown — with NO watcher running, the shape a failed watcher spawn
    /// leaves under `--peer-loss continue`, and the same shape a live watcher
    /// leaves for a death inside its 200 ms poll window — is judged by
    /// teardown's own pass. Hand oracle per row: a non-zero exit under
    /// `fail` is the run's `Err` naming the entry and its status; under
    /// `continue` the run stays `Ok`; a clean exit stays `Ok` under `fail`.
    /// Without teardown's own look, every row reads `Ok` — teardown reaps the
    /// zombie and records nothing. Also the quiet-natural-exit pin: a leader whose
    /// nodes are already gone is reaped WITHOUT a "group kill FAILED" alarm
    /// — on macOS `kill(-pgid, SIGKILL)` answers EPERM, not ESRCH, for a
    /// group holding only the leader's zombie (measured: a classification
    /// that treats EPERM as a failure warns on every such exit).
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn finish_judges_a_child_that_died_before_teardown_even_with_no_watcher() {
        let cases = [
            ("exit 3", PeerLossPolicy::Fail, Some("exit status: 3")),
            ("exit 3", PeerLossPolicy::Continue, None),
            ("exit 0", PeerLossPolicy::Fail, None),
        ];
        for (script, policy, expect_err) in cases {
            let child = sh(script);
            let pid = child.id();
            wait_exited_without_reaping(pid);
            let set = Ros2Children::for_test("g", policy, vec![("sidecar", child)]);
            let result = set.finish(Ok(()));
            match expect_err {
                Some(needle) => {
                    let msg = result
                        .expect_err(&format!("{script} under {policy:?} must fail the run"))
                        .to_string();
                    assert!(
                        msg.contains("'sidecar'")
                            && msg.contains(needle)
                            && msg.contains("--peer-loss fail"),
                        "the Err must name the entry, its status and the policy: {msg}"
                    );
                }
                None => result.unwrap_or_else(|e| {
                    panic!("{script} under {policy:?} must keep the run Ok, got: {e}")
                }),
            }
            assert!(is_reaped(pid), "teardown must have reaped the child");
        }
        assert!(
            !logs_contain("kill(-pgid, SIGKILL) FAILED"),
            "a natural exit with no descendants left is not a group-kill failure"
        );
        // The Drop floor is QUIET on the path that reaped everything: its
        // warn belongs to error paths only (the control for the watcher-
        // refusal arm, which asserts the warn).
        assert!(
            !logs_contain("ros2 entry dropped un-reaped"),
            "a set `finish` reaped is not dropped un-reaped"
        );
    }

    /// The other edge of the same rule: a child still RUNNING at teardown is
    /// ours to stop, and the non-zero status it exits with after OUR SIGINT
    /// is never judged — under `--peer-loss fail` the run stays `Ok`. This is
    /// what pins the pass to run BEFORE the signal fan-out: run it after, and
    /// every graceful shutdown of a healthy sidecar becomes a "death".
    #[cfg(unix)]
    #[test]
    fn finish_does_not_mistake_its_own_shutdown_signal_for_a_death() {
        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        set.finish(Ok(()))
            .unwrap_or_else(|e| panic!("a child we stopped ourselves is not a death: {e}"));
        assert!(
            is_reaped(pid),
            "teardown must have stopped and reaped the child"
        );
    }

    /// The look-to-signal window: the child exits on its own in the
    /// instant AFTER teardown has looked at it and BEFORE it signals. A
    /// judge-then-signal teardown reaps that exit in its grace loop with no
    /// failure recorded. Scope: the seam fires just BEFORE the
    /// per-child step (nothing can fire between a freeze and its signal —
    /// that is what the freeze is for), so with the freeze the child is a
    /// zombie by the time the shared look runs; what this arm pins is that
    /// an exit landing there is judged (`Err` under `fail`, `Ok` under
    /// `continue`) and that a look-then-signal
    /// ordering without the freeze is what lets it slip.
    #[cfg(unix)]
    #[test]
    fn a_death_in_the_look_to_signal_window_is_still_judged() {
        for (policy, expect_err) in [
            (PeerLossPolicy::Fail, true),
            (PeerLossPolicy::Continue, false),
        ] {
            let mut child = sh_with(
                "read x; exit 3",
                std::process::Stdio::piped(),
                std::process::Stdio::null(),
            );
            let pid = child.id();
            let stdin = child.stdin.take().expect("piped stdin");
            let set = Ros2Children::for_test("g", policy, vec![("sidecar", child)]);
            BEFORE_SIGNAL_HOOK_FOR_TEST.with(|h| {
                *h.borrow_mut() = Some(Box::new(move || {
                    drop(stdin); // EOF ⇒ `read` returns ⇒ `exit 3`
                    wait_exited_without_reaping(pid);
                }));
            });
            let result = set.finish(Ok(()));
            assert!(
                BEFORE_SIGNAL_HOOK_FOR_TEST.with(|h| h.borrow().is_none()),
                "the seam must have fired — else this test drove nothing"
            );
            if expect_err {
                let msg = result
                    .expect_err("a death in the window under --peer-loss fail must fail the run")
                    .to_string();
                assert!(
                    msg.contains("'sidecar'") && msg.contains("exit status: 3"),
                    "the Err must name the entry and its status: {msg}"
                );
            } else {
                result
                    .unwrap_or_else(|e| panic!("under --peer-loss continue the run stays Ok: {e}"));
            }
            assert!(is_reaped(pid));
        }
    }

    /// The SIGKILL backstop takes the child's WHOLE process group — a
    /// `ros2 launch` and the nodes it started — not the leader alone. The
    /// tree is `sh` with a backgrounded `sleep` grandchild, both holding one
    /// pipe: EOF within the budget proves the grandchild died too. The
    /// control runs the leader-only reap on an identical tree and shows the
    /// pipe STAYING open (the grandchild survived), so the oracle is proven
    /// to discriminate before it is trusted; the control then kills its own
    /// group so nothing leaks.
    #[cfg(unix)]
    #[test]
    fn the_sigkill_backstop_takes_the_launch_descendants_with_the_leader() {
        // The backstop: group kill ⇒ the grandchild's copy of the pipe closes.
        let (child, pipe) = spawn_tree();
        let pid = child.id();
        let mut guard = ChildGuard::new_group_leader(child, "ros2:tree".to_string());
        guard.kill_group_and_reap();
        assert!(
            guard.reaped() && is_reaped(pid),
            "the leader must be reaped"
        );
        assert!(
            pipe_reaches_eof_within(pipe, Duration::from_secs(10)),
            "the grandchild must die with its leader — the pipe never closed"
        );

        // Control: leader-only reap on the same tree leaves the grandchild
        // alive, so the pipe stays open — the discriminator the arm above
        // relies on. Cleaned up by killing the group directly.
        let (child, pipe) = spawn_tree();
        let pid = child.id();
        let mut guard = ChildGuard::new_group_leader(child, "ros2:tree".to_string());
        guard.kill_and_reap();
        assert!(is_reaped(pid));
        let eof = pipe_reaches_eof_within(pipe, Duration::from_secs(2));
        // SAFETY: the group our own (now reaped) leader led; its surviving
        // member keeps the group id alive, and it is this test's to kill.
        unsafe { libc::kill(-(pid as libc::pid_t), libc::SIGKILL) };
        assert!(
            !eof,
            "control: a leader-only kill must leave the grandchild holding the pipe"
        );
    }

    /// A `ros2 launch` leader that dies on its OWN must not strand the nodes
    /// it started: the leader announces its grandchild on
    /// the shared pipe and then exits 3 by itself; the grandchild `sleep`
    /// would survive a plain reap (it is re-parented to init). Both reap
    /// paths are driven — the death-watch pass, and teardown's freeze step
    /// finding the leader already dead — and each must leave the pipe at
    /// EOF (the grandchild died with the group). Under `continue` the run
    /// stays `Ok` (the leader's non-zero exit is a degraded-continue), so the
    /// oracle is the pipe, not the verdict.
    #[cfg(unix)]
    #[test]
    fn a_leaders_natural_exit_takes_its_descendants_with_it() {
        use std::io::BufRead;
        for path in ["watcher", "teardown"] {
            let mut child = sh_with(
                "sleep 60 & echo ready; exit 3",
                std::process::Stdio::null(),
                std::process::Stdio::piped(),
            );
            let pid = child.id();
            let mut pipe = std::io::BufReader::new(child.stdout.take().expect("piped stdout"));
            let mut line = String::new();
            pipe.read_line(&mut line).expect("announcement");
            assert_eq!(line, "ready\n");
            wait_exited_without_reaping(pid);
            let set =
                Ros2Children::for_test("g", PeerLossPolicy::Continue, vec![("launch", child)]);
            if path == "watcher" {
                judge_exits(
                    &mut lock_children(&set.children),
                    set.policy,
                    &set.failure,
                    &set.running,
                    &set.graph,
                );
                assert!(
                    is_reaped(pid),
                    "{path}: the watcher pass must reap the leader"
                );
            }
            set.finish(Ok(()))
                .unwrap_or_else(|e| panic!("{path}: a degraded-continue keeps the run Ok: {e}"));
            assert!(is_reaped(pid));
            assert!(
                pipe_reaches_eof_within(pipe, Duration::from_secs(10)),
                "{path}: the grandchild must die with its leader — the pipe never closed"
            );
        }
    }

    /// A death BEFORE teardown is judged on EVERY target,
    /// with the death-watch deliberately absent. The platform-independent
    /// first look in `signal_int_unless_exited` is the whole mechanism: it
    /// returns `Exited` before any per-platform code runs, so this arm
    /// exercises exactly what a target without job control exercises. The
    /// no-freeze seam is armed for the BROKEN CASE, not the baseline:
    /// on the baseline the look returns first and the seam is
    /// never consumed — its guard disarms it — while under a variant that
    /// deletes the look, the seam forces a signal-without-looking sequence
    /// on this host and the arm fails. To be exact:
    /// this test covers the Unix arm; the non-Unix `stop_running_child` is
    /// compile-checked only as far as the build host's cfg allows.
    #[cfg(unix)]
    #[test]
    fn a_death_before_teardown_is_judged_on_every_target_watcher_or_not() {
        for (policy, expect_err) in [
            (PeerLossPolicy::Fail, true),
            (PeerLossPolicy::Continue, false),
        ] {
            let child = sh("exit 3");
            let pid = child.id();
            wait_exited_without_reaping(pid);
            let set = Ros2Children::for_test("g", policy, vec![("sidecar", child)]);
            let _seam = NoFreeze::arm();
            let result = set.finish(Ok(()));
            if expect_err {
                let msg = result
                    .expect_err(
                        "a death before teardown must fail a --peer-loss fail run on every target",
                    )
                    .to_string();
                assert!(
                    msg.contains("'sidecar'") && msg.contains("exit status: 3"),
                    "{msg}"
                );
            } else {
                result.unwrap_or_else(|e| panic!("under continue the run stays Ok: {e}"));
            }
            assert!(is_reaped(pid));
        }
    }

    /// When the freeze itself is refused (a kernel refusal no live child of
    /// ours produces — forced through a seam), teardown degrades to
    /// look-then-signal rather than signal-blind: a child ALREADY dead is
    /// still judged (`Err` under `fail`) — by the shared look, which returns
    /// before the freeze is even attempted, so that half never consumes the
    /// seam and is the same shape as the no-watcher arm — and a child still
    /// running is signalled and never judged (`Ok`). The live half is the
    /// one that reaches the fallback, and it PROVES it did: the seam is
    /// consumed (read before its guard disarms it) and the fallback's warn
    /// is in the log — otherwise the freeze path, or the backstop, would
    /// pass this arm just as well.
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn the_freeze_fallback_still_judges_an_exit_it_can_see() {
        let child = sh("exit 3");
        let pid = child.id();
        wait_exited_without_reaping(pid);
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        let _seam_dead = NoFreeze::arm();
        let msg = set
            .finish(Ok(()))
            .expect_err("without the freeze, an exit the look can see is still judged")
            .to_string();
        assert!(
            msg.contains("'sidecar'") && msg.contains("exit status: 3"),
            "{msg}"
        );
        assert!(is_reaped(pid));

        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        let seam_live = NoFreeze::arm();
        let t0 = Instant::now();
        set.finish(Ok(())).unwrap_or_else(|e| {
            panic!("a child we stopped is not a death, freeze or no freeze: {e}")
        });
        assert!(
            crate::graph_cmd::FAIL_FREEZE_FOR_TEST.with(|f| !f.get()),
            "the live child must have reached the freeze step and consumed the seam"
        );
        drop(seam_live);
        assert!(
            logs_contain("signalling without the freeze"),
            "the fallback must have run and said so"
        );
        // Asserting the wall alone
        // (`< ROS2_CHILD_SHUTDOWN_GRACE / 2`) is the
        // load-inversion class — on a loaded machine a graceful stop can
        // exceed any tight bound and the test then reports a backstop that
        // never happened.
        //
        // The discriminator is the BACKSTOP ITSELF, and it announces itself,
        // so the property is asserted as a CONDITION load cannot fake: the
        // backstop marker must be ABSENT. It pairs with the positive control
        // just above (the fallback DID run and said so), so neither half is
        // vacuous. The wall survives only at catastrophe scale — a stop that
        // outlasts the whole grace window did hang.
        assert!(
            !logs_contain("did not exit within the shutdown grace window"),
            "the stop must be graceful — the SIGKILL backstop must not have run"
        );
        assert!(
            t0.elapsed() < ROS2_CHILD_SHUTDOWN_GRACE * 2,
            "the stop hung past the entire grace window"
        );
        assert!(is_reaped(pid));
    }

    /// A group method asked of a guard that
    /// does NOT lead a process group — the plain constructor, the shape of
    /// a multi-process worker — says so loudly and falls back to the leader
    /// alone: the exited child's status is still returned (and reaped), a
    /// running one is still killed and reaped, and no `kill(-pid)` is
    /// attempted against a group that does not exist.
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn a_group_method_on_a_non_leader_falls_back_loudly_to_the_leader() {
        let plain = |script: &str| {
            std::process::Command::new("sh")
                .arg("-c")
                .arg(script)
                .stdin(std::process::Stdio::null())
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .spawn()
                .expect("spawn sh in the parent's group")
        };
        let child = plain("exit 3");
        let pid = child.id();
        wait_exited_without_reaping(pid);
        let mut guard = ChildGuard::new(child, "worker".to_string());
        match guard.try_status_reaping_group() {
            ChildPoll::Exited(status) => assert_eq!(status.code(), Some(3)),
            ChildPoll::Running => panic!("the exit must still be reported, not `Running`"),
            ChildPoll::Lost => panic!("the exit must still be reported, not `Lost`"),
        }
        assert!(guard.reaped() && is_reaped(pid));
        assert!(
            logs_contain("does not lead a process group"),
            "the misuse must be said, not hidden behind ESRCH"
        );

        let child = plain("sleep 30");
        let pid = child.id();
        let mut guard = ChildGuard::new(child, "worker".to_string());
        guard.kill_group_and_reap();
        assert!(
            guard.reaped() && is_reaped(pid),
            "the leader is still killed and reaped"
        );
    }

    /// The GRACEFUL half of teardown is
    /// pinned by the child itself — a `sh` whose INT trap prints `INT` and
    /// exits 0 can only do so if it was continued after the freeze AND
    /// received the SIGINT; the backstop would SIGKILL it silently (and
    /// only after the whole grace). So the trap line on the pipe, a wall
    /// under half the grace, and `Ok` (our own stop) together pin freeze →
    /// SIGINT → SIGCONT. The backgrounded `sleep` ignores SIGINT (an
    /// asynchronous list in a non-interactive `sh` does), so the pipe
    /// reaching EOF afterwards pins that a leader's NATURAL exit takes its
    /// group with it on the graceful path too. Dropping the SIGCONT,
    /// dropping the SIGINT, or making `stop_running_child` a no-op — each leaves
    /// the child to the backstop and fails the trap line or the wall.
    #[cfg(unix)]
    #[test]
    fn a_running_child_is_resumed_and_receives_the_graceful_signal() {
        use std::io::BufRead;
        // `ready` is echoed AFTER the trap is installed and the sleep started:
        // without the handshake the child can be frozen and signalled while
        // `sh` is still parsing, die of the default SIGINT with nothing to
        // show, and the pipe closes at once — measured, first run.
        let mut child = sh_with(
            "trap 'echo INT; exit 0' INT; sleep 60 & echo ready; wait",
            std::process::Stdio::null(),
            std::process::Stdio::piped(),
        );
        let pid = child.id();
        let mut pipe = std::io::BufReader::new(child.stdout.take().expect("piped stdout"));
        let mut line = String::new();
        pipe.read_line(&mut line).expect("the readiness line");
        assert_eq!(
            line, "ready\n",
            "the child must have installed its trap first"
        );
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        let t0 = Instant::now();
        set.finish(Ok(())).expect("our own stop is not a death");
        let elapsed = t0.elapsed();
        line.clear();
        pipe.read_line(&mut line).expect("the trap line");
        assert_eq!(
            line, "INT\n",
            "the child must have RUN its INT trap: SIGINT delivered and SIGCONT resumed it"
        );
        assert!(
            elapsed < ROS2_CHILD_SHUTDOWN_GRACE / 2,
            "a graceful stop, not the SIGKILL backstop: took {elapsed:?}"
        );
        assert!(is_reaped(pid));
        assert!(
            pipe_reaches_eof_within(pipe, Duration::from_secs(10)),
            "the leader's exit must take its group with it on the graceful path"
        );
    }

    /// The UNGRACEFUL twin of the arm above: a leader that will not take the
    /// graceful signal is escalated to, through [`Ros2Children::finish`]'s own
    /// grace loop, and the escalation is said.
    ///
    /// This arm exists because the live fixtures are
    /// [`sleeping_child`], which never reaches the deadline branch. A `sh -c`
    /// leader reaches it only by ACCIDENT — on a `/bin/sh` that forks, the stop
    /// stalls the full grace and escalates — which covers it on
    /// some hosts and not at all on others, so a variant that neuters the
    /// escalation would pass on the rest. A leader that IGNORES SIGINT
    /// reaches it on every platform instead: `trap '' INT` sets SIG_IGN, and an
    /// ignored signal is DISCARDED rather than queued, so neither the freeze's
    /// SIGINT nor the SIGCONT that follows can move it — the leader sits in
    /// `wait` for a 30 s `sleep` and the 10 s grace expires first.
    ///
    /// The readiness handshake is load-bearing, for the same reason the
    /// fixtures avoid a `sh -c` leader: without it the SIGINT can land before the
    /// trap is installed, the leader dies of the default disposition, and the
    /// arm passes VACUOUSLY having proved nothing. The wall is asserted as a
    /// LOWER bound — the direction load cannot fake (contention can only make
    /// a stop later, never earlier), so it can never invert on a busy machine.
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn a_leader_that_ignores_the_graceful_signal_is_killed_by_the_backstop() {
        use std::io::BufRead;
        let mut child = sh_with(
            "trap '' INT; echo ready; sleep 30",
            std::process::Stdio::null(),
            std::process::Stdio::piped(),
        );
        let pid = child.id();
        let mut pipe = std::io::BufReader::new(child.stdout.take().expect("piped stdout"));
        let mut line = String::new();
        pipe.read_line(&mut line).expect("the readiness line");
        assert_eq!(
            line, "ready\n",
            "the child must have installed its INT trap before anything is signalled"
        );
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        let t0 = Instant::now();
        set.finish(Ok(()))
            .expect("the backstop is the run's OWN stop and is never judged as a death");
        let elapsed = t0.elapsed();
        assert!(
            logs_contain("did not exit within the shutdown grace window"),
            "a leader that ignores the graceful signal must be escalated to, and said"
        );
        assert!(
            elapsed >= ROS2_CHILD_SHUTDOWN_GRACE,
            "the backstop must fire AFTER the whole grace, never early: took {elapsed:?}"
        );
        assert!(is_reaped(pid));
    }

    /// A fixture child must be immune to the ambient SIGINT disposition of the
    /// process that spawns it — the SECOND route to a defeated graceful stop,
    /// and the one a bare `sleep` leader does NOT fix on its own.
    ///
    /// `SIG_IGN` survives `fork` AND `execve`, and this binary has a real
    /// window where SIGINT is ignored process-wide (see
    /// [`reset_sigint_before_exec`]). Both arms below spawn under that
    /// disposition and require the graceful stop anyway:
    ///
    /// * the direct-`sleep` leader must still die of teardown's SIGINT rather
    ///   than sit out the grace and be reaped by the backstop;
    /// * the SHELL leader must be able to INSTALL its `INT` trap at all — POSIX
    ///   forbids a non-interactive shell from trapping a signal ignored on
    ///   entry, so the trap line on the pipe is direct evidence the child's
    ///   disposition was `SIG_DFL` when it exec'd, not merely that it died.
    ///
    /// The ignored disposition is applied to the CHILD, through the
    /// [`poison_child_sigint_if_armed`] thread-local seam, and this process's
    /// own disposition is never changed. Holding `SIG_IGN`
    /// process-wide across each spawn would be narrow but not empty:
    /// `#[serial]` does not order this test against the ~2050 non-`#[serial]`
    /// tests in the binary — so a sibling forking inside either window would
    /// inherit the ignored disposition and lose its own graceful
    /// teardown. A pin for this hazard must not re-open it for everyone else,
    /// and the seam reaches no thread but this one.
    /// No `#[serial]`, and nothing to restore, because nothing global moves.
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn a_fixture_child_is_immune_to_an_ambient_ignored_sigint() {
        use std::io::BufRead;
        let poison_next_spawn = || POISON_CHILD_SIGINT_FOR_TEST.with(|p| p.set(true));

        // Arm 1 — the direct `sleep` leader.
        poison_next_spawn();
        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        let t0 = Instant::now();
        set.finish(Ok(())).expect("our own stop is not a death");
        assert!(
            !logs_contain("did not exit within the shutdown grace window"),
            "an inherited SIG_IGN must not survive into the fixture child — the stop must be \
             graceful, not the SIGKILL backstop"
        );
        assert!(
            t0.elapsed() < ROS2_CHILD_SHUTDOWN_GRACE,
            "a graceful stop cannot take a whole grace window: {:?}",
            t0.elapsed()
        );
        assert!(is_reaped(pid));

        // Arm 2 — the SHELL leader, whose trap cannot even be installed if the
        // ignored disposition reaches it.
        poison_next_spawn();
        let mut child = sh_with(
            "trap 'echo INT; exit 0' INT; sleep 60 & echo ready; wait",
            std::process::Stdio::null(),
            std::process::Stdio::piped(),
        );
        let pid = child.id();
        let mut pipe = std::io::BufReader::new(child.stdout.take().expect("piped stdout"));
        let mut line = String::new();
        pipe.read_line(&mut line).expect("the readiness line");
        assert_eq!(line, "ready\n", "the child must have installed its trap");
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        set.finish(Ok(())).expect("our own stop is not a death");
        line.clear();
        pipe.read_line(&mut line).expect("the trap line");
        assert_eq!(
            line, "INT\n",
            "a shell spawned under an ignored SIGINT must still have been able to TRAP it — POSIX \
             forbids trapping a signal ignored on entry, so this line is the proof the child's \
             disposition was reset to SIG_DFL before exec"
        );
        assert!(is_reaped(pid));
    }

    /// A death-watch that cannot start is a REFUSAL under `--peer-loss fail`
    /// (the promise "a death stops the run when it happens" would otherwise
    /// be carried by nothing) and tears down what was already spawned; under
    /// `continue` it is a warn and the run proceeds. The anti-tautology arm
    /// proves the refusal comes from the injected failure, not from the arm
    /// being unconditional: with nothing injected the watcher starts.
    #[cfg(unix)]
    #[tracing_test::traced_test]
    #[test]
    fn a_watcher_that_cannot_start_refuses_the_run_under_peer_loss_fail_only() {
        // fail: refused, loudly, and the child is reaped by the drop.
        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        FAIL_WATCHER_SPAWN_FOR_TEST.with(|f| f.set(true));
        let msg = set
            .spawn_death_watch()
            .expect_err("no watcher under --peer-loss fail must refuse the run")
            .to_string();
        assert!(
            msg.contains("death-watch")
                && msg.contains("--peer-loss fail")
                && msg.contains("--peer-loss continue"),
            "the refusal must name the cause, the policy it protects and the way out: {msg}"
        );
        drop(set);
        assert!(
            is_reaped(pid),
            "the refusal must tear down the already-spawned child"
        );
        // At WARN — a level-blind `logs_contain` let a `debug!` demotion pass.
        logs_assert(|lines: &[&str]| {
            let n = lines
                .iter()
                .filter(|l| l.contains("WARN") && l.contains("ros2 entry dropped un-reaped"))
                .count();
            if n == 1 {
                Ok(())
            } else {
                Err(format!(
                    "the refusal path is the un-torn-down shape the Drop floor exists for and \
                     must say so once, at WARN — saw {n}"
                ))
            }
        });

        // continue: a warn, and the run goes on.
        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Continue, vec![("sidecar", child)]);
        FAIL_WATCHER_SPAWN_FOR_TEST.with(|f| f.set(true));
        set.spawn_death_watch().unwrap_or_else(|e| {
            panic!("under --peer-loss continue a missing watcher is a warn: {e}")
        });
        assert!(
            logs_contain("could not start the ros2 death-watch thread"),
            "under --peer-loss continue the missing watcher is SAID, not silent"
        );
        set.finish(Ok(())).expect("the run itself is untouched");
        assert!(is_reaped(pid));

        // anti-tautology: nothing injected, the watcher starts under fail.
        let child = sleeping_child();
        let pid = child.id();
        let set = Ros2Children::for_test("g", PeerLossPolicy::Fail, vec![("sidecar", child)]);
        set.spawn_death_watch()
            .unwrap_or_else(|e| panic!("with a real thread available nothing is refused: {e}"));
        set.finish(Ok(())).expect("healthy sidecar, stopped by us");
        assert!(is_reaped(pid));
    }

    // ---------------------------------------------------------------------
    // Can the teardown SIGINT REACH the spawned entry?
    //
    // Teardown's graceful step is `ChildGuard::signal_int_unless_exited`,
    // whose SIGINT is a PER-PID `kill(leader, SIGINT)` inside a freeze ->
    // peek -> signal -> resume sequence (the multi-process supervisor's
    // fan-out is the OTHER entry point, `signal_int`). A child inherits both
    // its parent's SIGINT DISPOSITION and its parent's signal MASK, and both
    // survive `execve`, so a launcher that ignores or blocks SIGINT (the
    // shapes are listed once, in `crate::child_signals`) hands that state to
    // the `ros2` child and the graceful step becomes a no-op: every stop expires
    // `ROS2_CHILD_SHUTDOWN_GRACE` and ends in the SIGKILL backstop, taking
    // `ros2 launch`'s nodes down without their own wind-down.
    //
    // That state is PROCESS-GLOBAL. The sibling pin in `connect_cmd` flips it
    // in place under `#[serial]` + an RAII restore, which works but leaves a
    // window: `#[serial]` serialises only against other `#[serial]` tests,
    // and ~2000 non-serial siblings run concurrently in this binary. This one
    // runs in a dedicated SUBPROCESS instead, so nothing outside it can see
    // the mutation at all — the self-re-exec pattern
    // (`crates/cerulion_core/tests/cdylib_tracing_stopgap_test.rs`), whose
    // bounded-wait + drain discipline the parent below also borrows.
    // ---------------------------------------------------------------------

    /// Env switch that turns the `#[ignore]`d child body on. Absent ⇒ the
    /// child test is inert, so a bare `cargo test -- --ignored` never runs it
    /// with an ambient state it would then mutate.
    #[cfg(unix)]
    const SIGINT_CHILD_ENV: &str = "CER_ROS2_SIGINT_DISPOSITION_CHILD";

    /// Where the `ros2` stand-in reports its own pid — the leader's pid,
    /// which `Ros2Children` keeps to itself.
    #[cfg(unix)]
    const SIGINT_PIDFILE_ENV: &str = "CER_ROS2_SIGINT_DISPOSITION_PIDFILE";

    /// The libtest path of the child body, as `--exact` wants it.
    #[cfg(unix)]
    const SIGINT_CHILD_TEST: &str = "ros2_graph::tests::ros2_sigint_disposition_child";

    /// Hard cap on the child's runtime, so a future unbounded wait fails as
    /// an attributable HARNESS timeout rather than hanging until the CI job
    /// cancel (`cdylib_tracing_stopgap_test.rs`'s `CHILD_TIMEOUT` precedent).
    ///
    /// Sized against the SUM of the child's own worst cases, not each one:
    /// eight probes at up to (2 x 10 s `await_stand_in` + 5 s delivery
    /// ceiling) plus two `finish` arms at up to one whole
    /// `ROS2_CHILD_SHUTDOWN_GRACE` each is ~4 minutes of pathological
    /// slowness. A cap below that would fire on a correct run on a starved
    /// machine and be reported as a harness failure — the opposite of what this
    /// constant promises. It is a catastrophe bound; the healthy run is ~16 s.
    #[cfg(unix)]
    const SIGINT_CHILD_TIMEOUT: Duration = Duration::from_secs(420);

    /// How long to wait for a capture thread to hand over what it read, once
    /// the child is gone (or has been killed). Generous: the threads are
    /// blocked only on EOF of pipes nothing else holds.
    #[cfg(unix)]
    const CAPTURE_REPORT_TIMEOUT: Duration = Duration::from_secs(60);

    /// How long a probe waits for a DELIVERED signal to land. Generous and
    /// exited early: load can only DELAY a delivered signal, never prevent
    /// it, so a ceiling cannot invert this arm.
    #[cfg(unix)]
    const PROBE_DELIVERY_CEILING: Duration = Duration::from_secs(5);

    /// How long a probe watches a child it expects to SURVIVE. Short on
    /// purpose: load makes a survivor MORE likely to still be alive, which is
    /// the asserted direction, so this cannot invert either.
    #[cfg(unix)]
    const PROBE_SURVIVAL_WINDOW: Duration = Duration::from_millis(750);

    /// Markers the child prints; the parent requires ALL of them, so a child
    /// that exited 0 having skipped a phase still fails.
    #[cfg(unix)]
    const MARK_IGNORED: &str =
        "item-30.2 arms: an IGNORED SIGINT is defeated, and the helper fixes it";
    #[cfg(unix)]
    const MARK_BLOCKED: &str =
        "item-30.2 arms: a BLOCKED SIGINT is defeated, and the helper fixes it";
    #[cfg(unix)]
    const MARK_SCOPE: &str =
        "item-30.2 arms: the disposition half is SIGINT-only, the mask half is total";
    #[cfg(unix)]
    const MARK_PRODUCTION: &str = "item-30.2 PRODUCTION: the real spawn stopped gracefully";
    #[cfg(unix)]
    const MARK_BACKSTOP: &str = "item-30.2 CONTROL: the backstop announces itself, in those words";

    /// A `ros2` stand-in: a live leader that installs NO signal handler of
    /// its own, so its signal state is EXACTLY what the spawner handed it.
    ///
    /// It `exec`s, never forks — a shell that forks and `wait`s DEFERS
    /// SIGINT until the job exits, which would make a graceful stop look like
    /// a defeated one. `sleep` is named by
    /// ABSOLUTE path because `PATH` is narrowed to the stand-in's own
    /// directory. `$$` is written BEFORE the exec and survives it (an exec
    /// keeps the pid), through a temp file renamed into place so a reader can
    /// never see a torn pid.
    #[cfg(unix)]
    fn write_ros2_stand_in(bin_dir: &Path) -> PathBuf {
        // ABSOLUTE paths — see `resolve_tool`. (`echo` is a shell builtin, so
        // it needs no path.)
        let (sleep, mv) = (resolve_tool("sleep"), resolve_tool("mv"));
        let script = bin_dir.join("ros2");
        std::fs::write(
            &script,
            format!(
                "#!/bin/sh\necho \"$$\" > \"${SIGINT_PIDFILE_ENV}.tmp\"\n{} \"${SIGINT_PIDFILE_ENV}.tmp\" \"${SIGINT_PIDFILE_ENV}\"\nexec {} 30\n",
                mv.display(),
                sleep.display()
            ),
        )
        .expect("write the ros2 stand-in");
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755))
            .expect("chmod the ros2 stand-in");
        script
    }

    /// Owns a spawned stand-in so an UNWIND cannot strand it.
    ///
    /// `std::process::Child`'s `Drop` deliberately does NOT kill the process,
    /// so every fallible check between a spawn and the arm's own cleanup —
    /// `await_exec`'s deadline, the `/proc` read, the "signal was sent"
    /// assert, a pid cross-check — would otherwise leave a `sleep 30` running
    /// in its own process group. That is worst exactly when it matters most:
    /// on a FAILING run, which is the run whose output someone is reading.
    ///
    /// Arm it on the line after the spawn, before anything that can panic.
    /// Every stand-in here leads its own group (`process_group(0)`), so the
    /// teardown takes the group and not merely the leader — which matters for
    /// the re-exec harness, whose descendants would otherwise survive it
    /// holding the capture pipes. The teardown verifies leadership rather than
    /// assuming it, so a caller that forgets can never make it signal a group
    /// belonging to somebody else.
    #[cfg(unix)]
    struct StandInGuard(Option<std::process::Child>);

    #[cfg(unix)]
    impl StandInGuard {
        fn new(child: std::process::Child) -> Self {
            Self(Some(child))
        }
        fn held(&mut self) -> &mut std::process::Child {
            self.0
                .as_mut()
                .expect("harness: the stand-in was already released")
        }
        fn pid(&mut self) -> u32 {
            self.held().id()
        }
        /// Hand the child on to an owner that will reap it (e.g.
        /// `Ros2Children::for_test`), disarming this guard.
        fn release(mut self) -> std::process::Child {
            self.0
                .take()
                .expect("harness: the stand-in was already released")
        }
        /// Reap it here, disarming this guard.
        fn reap(&mut self) {
            if let Some(mut child) = self.0.take() {
                child.wait().expect("harness: reap the stand-in");
            }
        }
    }

    #[cfg(unix)]
    impl Drop for StandInGuard {
        fn drop(&mut self) {
            let Some(mut child) = self.0.take() else {
                return;
            };
            let pid = child.id() as libc::pid_t;
            // Take the whole GROUP when — and only when — this child really
            // leads one. `kill(-pid)` names the group whose id is `pid`, so
            // firing it at a non-leader would at best miss and at worst signal
            // a group that belongs to somebody else. Checking leadership keeps
            // the guard correct for any caller rather than resting on every
            // future one remembering `process_group(0)`.
            //
            // SAFETY: `getpgid`/`kill` on our own un-reaped child's pid, which
            // the kernel cannot have recycled while we hold it. Failures are
            // ignorable — this runs while unwinding, and the per-process kill
            // below is the belt to its braces.
            unsafe {
                if libc::getpgid(pid) == pid {
                    libc::kill(-pid, libc::SIGKILL);
                }
            }
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    /// An ABSOLUTE path to a system tool. The child body narrows `PATH` to
    /// the stand-in's own directory so `Command::new("ros2")` cannot resolve
    /// to a real `ros2` — which also means nothing the harness runs may look
    /// anything up by name.
    #[cfg(unix)]
    fn resolve_tool(name: &str) -> PathBuf {
        [format!("/bin/{name}"), format!("/usr/bin/{name}")]
            .into_iter()
            .map(PathBuf::from)
            .find(|p| p.is_file())
            .unwrap_or_else(|| panic!("no `{name}` at /bin or /usr/bin — the harness needs one"))
    }

    /// This process's CURRENT SIGINT disposition, read without changing it.
    #[cfg(unix)]
    fn sigint_disposition() -> libc::sighandler_t {
        let mut old: libc::sigaction = unsafe { std::mem::zeroed() };
        // SAFETY: a pure QUERY — a null `act` reads the disposition and
        // writes only the `old` out-param we own.
        let rc = unsafe { libc::sigaction(libc::SIGINT, std::ptr::null(), &mut old) };
        assert_eq!(rc, 0, "sigaction(SIGINT) query failed");
        old.sa_sigaction
    }

    /// Set this process's disposition for `sig`, asserting it took.
    #[cfg(unix)]
    fn set_disposition(sig: libc::c_int, handler: libc::sighandler_t) {
        // SAFETY: a single disposition change in a process dedicated to this
        // test, which exits immediately afterwards.
        let prev = unsafe { libc::signal(sig, handler) };
        assert_ne!(
            prev,
            libc::SIG_ERR,
            "fixture: signal({sig}) failed: {}",
            std::io::Error::last_os_error()
        );
    }

    /// Block or unblock `sig` in this process, asserting it took — the SECOND
    /// half of the hazard (`SIG_DFL` is still undeliverable while the signal
    /// is blocked; it just goes pending instead of being discarded).
    ///
    /// `sigprocmask`, not `pthread_sigmask`, for the reason the helper under
    /// test uses it: it follows the -1 + `errno` convention, so
    /// `last_os_error()` reports the real cause. (`pthread_sigmask` RETURNS
    /// the error number and leaves `errno` alone, which would print
    /// "Success".)
    #[cfg(unix)]
    fn set_blocked(sig: libc::c_int, blocked: bool) {
        let mut set: libc::sigset_t = unsafe { std::mem::zeroed() };
        // SAFETY: well-formed sigset operations on a local we own.
        unsafe {
            libc::sigemptyset(&mut set);
            libc::sigaddset(&mut set, sig);
            let how = if blocked {
                libc::SIG_BLOCK
            } else {
                libc::SIG_UNBLOCK
            };
            assert_eq!(
                libc::sigprocmask(how, &set, std::ptr::null_mut()),
                0,
                "fixture: sigprocmask failed: {}",
                std::io::Error::last_os_error()
            );
        }
        assert_eq!(
            is_blocked(sig),
            blocked,
            "fixture precondition: signal {sig} blocked must be {blocked}"
        );
    }

    /// Is `sig` blocked in this process?
    #[cfg(unix)]
    fn is_blocked(sig: libc::c_int) -> bool {
        let mut current: libc::sigset_t = unsafe { std::mem::zeroed() };
        // SAFETY: a pure QUERY — a null `set` reads the mask into `current`.
        unsafe {
            libc::sigemptyset(&mut current);
            assert_eq!(
                libc::sigprocmask(libc::SIG_BLOCK, std::ptr::null(), &mut current),
                0,
                "fixture: sigprocmask query failed: {}",
                std::io::Error::last_os_error()
            );
            libc::sigismember(&current, sig) == 1
        }
    }

    /// Remove `pidfile` LOUDLY: it is the only channel by which a later arm
    /// learns which process to assert on, and a stale one would name the
    /// WRONG process while every downstream message blamed the code.
    #[cfg(unix)]
    fn clear_pidfile(pidfile: &Path) {
        match std::fs::remove_file(pidfile) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("harness: could not clear {}: {e}", pidfile.display()),
        }
        assert!(
            !pidfile.exists(),
            "harness: a stale pidfile would name the WRONG process"
        );
    }

    /// Block until the stand-in has reported its pid, then (on Linux) until
    /// it has `exec`ed past `/bin/sh`, so a `/proc` read cannot catch the
    /// shell instead of the leader it becomes. Each phase gets its OWN
    /// budget, so a slow report cannot starve the exec wait and then be
    /// misreported as a failure to exec.
    #[cfg(unix)]
    fn await_stand_in(pidfile: &Path) -> u32 {
        let deadline = Instant::now() + Duration::from_secs(10);
        let pid = loop {
            // Each cause gets its OWN diagnosis: "it never ran" (ENOENT),
            // "it could not be read" (anything else) and "it wrote nonsense"
            // are three different problems, and two of the three would be
            // slandered by a single catch-all message.
            let why = match std::fs::read_to_string(pidfile) {
                Ok(raw) => match raw.trim().parse::<u32>() {
                    Ok(pid) => break pid,
                    Err(e) => format!("it holds {raw:?}, which is not a pid: {e}"),
                },
                Err(e) => format!("{e}"),
            };
            assert!(
                Instant::now() < deadline,
                "the `ros2` stand-in never reported a usable pid at {} — {why}",
                pidfile.display()
            );
            std::thread::sleep(Duration::from_millis(10));
        };
        await_exec(pid);
        pid
    }

    /// Block until `pid` is the `sleep` image, so a `/proc` read cannot catch
    /// the process BEFORE its exec — where, on the fork+exec path, the
    /// `pre_exec` closure may not have run yet and the state read would be the
    /// parent's rather than the child's. Its OWN budget.
    #[cfg(unix)]
    fn await_exec(pid: u32) {
        #[cfg(target_os = "linux")]
        {
            let deadline = Instant::now() + Duration::from_secs(10);
            loop {
                let why = match std::fs::read_to_string(format!("/proc/{pid}/comm")) {
                    Ok(comm) if comm.trim() == "sleep" => return,
                    Ok(comm) => format!("comm is {:?}, not \"sleep\"", comm.trim()),
                    Err(e) => format!("/proc/{pid}/comm: {e}"),
                };
                assert!(
                    Instant::now() < deadline,
                    "harness: the stand-in (pid {pid}) never exec'ed `sleep` — {why}"
                );
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            // No `/proc` to watch, and nothing here reads per-process state —
            // the arms on this platform are behavioural. Settle briefly so the
            // common case is the post-exec one.
            let _ = pid;
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// Is `sig` IGNORED / BLOCKED in `pid`? Reads `/proc/<pid>/status`'s
    /// `SigIgn:` and `SigBlk:` — the direct, LOAD-INDEPENDENT oracle for the
    /// signal state the spawner handed this child, and the reason every arm
    /// below has a real oracle on Linux rather than only a wall.
    ///
    /// Both halves, because they are the two ways one inherited state defeats
    /// the same stop: an ignored signal is DISCARDED, a blocked one goes
    /// PENDING, and neither reaches the child.
    #[cfg(target_os = "linux")]
    fn proc_signal_state(pid: u32, sig: libc::c_int) -> (bool, bool) {
        let status = std::fs::read_to_string(format!("/proc/{pid}/status"))
            .unwrap_or_else(|e| panic!("harness: /proc/{pid}/status: {e}"));
        let field = |name: &str| -> u64 {
            let raw = status
                .lines()
                .find_map(|l| l.strip_prefix(name))
                .map(str::trim)
                .unwrap_or_else(|| {
                    panic!("harness: no {name} line in /proc/{pid}/status:\n{status}")
                });
            u64::from_str_radix(raw, 16)
                .unwrap_or_else(|e| panic!("harness: {name} `{raw}` is not hex: {e}"))
        };
        // The masks are 1-based by signal number: SIGINT (2) is bit 1.
        debug_assert!(sig > 0, "signal numbers are 1-based");
        let bit = 1u64 << (sig as u32 - 1);
        (field("SigIgn:") & bit != 0, field("SigBlk:") & bit != 0)
    }

    /// Spawn the stand-in — `with_helper` chooses the production rule or the
    /// control shape that omits it — send it `sig`, and assert whether it was
    /// DELIVERED.
    ///
    /// The two expectations get different windows on purpose. A delivery arm
    /// polls to a generous ceiling and exits the instant the child dies: load
    /// can DELAY a delivered signal but never prevent it, so a ceiling cannot
    /// turn a green arm red.
    ///
    /// A survival arm is the one to be careful about. It cannot produce a
    /// spurious RED — load only makes a survivor likelier to still be alive —
    /// but it CAN go quietly vacuous: a child load has not killed YET reads
    /// the same as one that cannot be killed, and the survival arms are the
    /// premise the arms after them rest on. On Linux the `/proc` oracle
    /// covers exactly that; on macOS the 750 ms window is the whole defence,
    /// which is generous against a signal that kills `sleep` in microseconds
    /// but is not a proof.
    #[cfg(unix)]
    fn probe_signal(
        sleep_bin: &Path,
        with_helper: bool,
        sig: libc::c_int,
        expect_delivered: bool,
        why: &str,
    ) {
        // `sleep` DIRECTLY — no `#!/bin/sh` wrapper. These arms own the
        // `Command`, so they do not need the wrapper phases 4-5 use to absorb
        // production's fixed argv, and every layer removed is a layer that
        // cannot change the signal state under test: a shell is a real program
        // with its own startup, and `/bin/sh` is bash on some hosts and dash on
        // others. (Measured on a host whose `/bin/sh` is bash: bash, dash AND
        // `/bin/sh` all preserve a blocked SIGINT across startup + exec on
        // BOTH spawn paths — so such a host cannot reproduce the behaviour
        // seen on Linux, and the remedy must not depend on knowing which layer
        // causes it.)
        let mut cmd = std::process::Command::new(sleep_bin);
        cmd.arg("30");
        {
            use std::os::unix::process::CommandExt;
            // Exactly as `Ros2Children::spawn` does.
            cmd.process_group(0);
        }
        // The stand-in prints NOTHING (its pid goes to a file), and it must
        // not INHERIT — and so hold open — the pipes the parent is draining:
        // a panic between here and the reap below would strand it for the
        // length of its `sleep`, the parent's capture would time out, and the
        // panic message that explains the failure would be replaced by a
        // placeholder. The failing case is the one where the output matters.
        cmd.stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null());
        if with_helper {
            crate::child_signals::make_sigint_deliverable(&mut cmd);
        } else {
            // PATH-MATCHED control, and it is load-bearing rather than tidy.
            //
            // `std` picks between `posix_spawn` and fork+exec, and a NON-EMPTY
            // `pre_exec` closure list is one of the things that disqualifies
            // `posix_spawn` (`sys/process/unix/unix.rs`, the early `Ok(None)`
            // guard). So a control built by simply OMITTING the helper differs
            // from the helper arm in TWO ways — the signal work, and the whole
            // spawn path — which is a confounded experiment.
            //
            // The confound is measurable: on
            // Linux a `posix_spawn` control's child came back WITHOUT the
            // parent's blocked SIGINT (`/proc` said `blocked=false`) while the
            // ignored-disposition arm, in the same process, inherited fine.
            // Whatever the mechanism there, the helper-less shape's spawn path is
            // not the one the production code takes, so comparing against it
            // measures the wrong thing.
            //
            // An empty closure restores the variable of interest: both arms
            // take fork+exec — what the production code does, since the helper
            // installs a closure — and the ONLY difference left is whether the
            // signal work runs.
            //
            // SAFETY: the closure does nothing at all; there is no call between
            // fork and exec to be unsafe about.
            use std::os::unix::process::CommandExt;
            unsafe {
                cmd.pre_exec(|| Ok(()));
            }
        }
        // Armed on the line after the spawn: every check below can panic, and
        // a raw `Child` dropped while unwinding leaves the `sleep` running.
        let mut child = StandInGuard::new(
            cmd.spawn()
                .unwrap_or_else(|e| panic!("harness: could not spawn the stand-in: {e}")),
        );
        // No pidfile: this arm spawned the leader itself, so `Child::id` IS the
        // pid — one fewer thing to get wrong than the phase-4/5 path, which has
        // to learn it from a process it does not own.
        let pid = child.pid();
        await_exec(pid);
        // The DIRECT oracle, where the kernel will tell us: whether the
        // signal can reach the child is a fact about its inherited state,
        // readable before anything is sent and immune to how loaded the machine
        // is. It ADDS to the outcome arms below rather than replacing them —
        // and on macOS, where it is compiled out, they are all there is.
        #[cfg(target_os = "linux")]
        {
            let (ignored, blocked) = proc_signal_state(pid, sig);
            assert_eq!(
                !(ignored || blocked),
                expect_delivered,
                "/proc says the child has signal {sig} ignored={ignored} blocked={blocked}, \
                 which contradicts this arm's expectation that it would \
                 {} — {why}",
                if expect_delivered {
                    "ARRIVE"
                } else {
                    "be DEFEATED"
                }
            );
        }
        // SAFETY: our own un-reaped child's pid, so the kernel cannot have
        // recycled the number.
        let rc = unsafe { libc::kill(pid as libc::pid_t, sig) };
        assert_eq!(
            rc,
            0,
            "harness: the probe signal {sig} must be SENT — the arm is about whether \
             it is DELIVERED, not whether it left: {}",
            std::io::Error::last_os_error()
        );

        let died = if expect_delivered {
            let deadline = Instant::now() + PROBE_DELIVERY_CEILING;
            loop {
                match child.held().try_wait() {
                    Ok(Some(_)) => break true,
                    Ok(None) if Instant::now() >= deadline => break false,
                    Ok(None) => std::thread::sleep(Duration::from_millis(20)),
                    Err(e) => panic!("harness: try_wait on the stand-in failed: {e}"),
                }
            }
        } else {
            std::thread::sleep(PROBE_SURVIVAL_WINDOW);
            match child.held().try_wait() {
                Ok(None) => false,
                Ok(Some(_)) => true,
                Err(e) => panic!("harness: try_wait on the stand-in failed: {e}"),
            }
        };

        if !died {
            // SAFETY: the group our own live leader leads; ours to kill.
            let rc = unsafe { libc::kill(-(pid as libc::pid_t), libc::SIGKILL) };
            let err = std::io::Error::last_os_error();
            // ESRCH: already gone. EPERM: macOS's answer for a group holding
            // only the leader's zombie — MEASURED, and recorded by
            // `finish_judges_a_child_that_died_before_teardown_even_with_no_watcher`
            // in this same module. Tolerating only ESRCH would turn a cleanup
            // race into a red test on macOS.
            let benign = matches!(err.raw_os_error(), Some(libc::ESRCH) | Some(libc::EPERM));
            assert!(
                rc == 0 || benign,
                "harness: could not clean up the stand-in's group: {err}"
            );
        }
        child.reap();
        assert_eq!(died, expect_delivered, "{why}");
    }

    /// **THE production pin.** A `ros2:` entry is
    /// spawned so the graceful teardown SIGINT can REACH it, even when the
    /// launcher that started `cerulion` ignores or blocks SIGINT — so
    /// `ros2 launch` gets to wind its own nodes down instead of being
    /// SIGKILLed a grace window later.
    ///
    /// Drives the real [`Ros2Children::spawn`] + [`Ros2Children::finish`] in
    /// a subprocess whose own SIGINT is `SIG_IGN` — the shape
    /// `cerulion graph run … &` in a shell script and `nohup … &` were both
    /// MEASURED to produce (`crate::child_signals` lists which launchers do,
    /// and which do not). See [`ros2_sigint_disposition_child`] for the arms.
    ///
    /// Deleting `child_signals::make_sigint_deliverable` from
    /// `Ros2Children::spawn` fails the child's production arm — the stand-in
    /// ignores the teardown SIGINT, the whole [`ROS2_CHILD_SHUTDOWN_GRACE`]
    /// expires, and the SIGKILL backstop warns.
    #[cfg(unix)]
    #[test]
    fn a_ros2_entry_is_spawned_so_the_teardown_sigint_can_reach_it() {
        let exe = std::env::current_exe().expect("the test binary's own path");
        // Armed immediately: the two `take().expect(..)` calls just below, and
        // every assertion after them, can panic — and a raw `Child` dropped
        // while unwinding would leave a whole second test binary running.
        let mut child = StandInGuard::new({
            let mut cmd = std::process::Command::new(&exe);
            cmd.args(["--exact", SIGINT_CHILD_TEST, "--ignored", "--nocapture"])
                .env(SIGINT_CHILD_ENV, "1")
                .stdout(std::process::Stdio::piped())
                .stderr(std::process::Stdio::piped());
            {
                use std::os::unix::process::CommandExt;
                // Its OWN process group, like every other stand-in here. The
                // guard tears down a GROUP, and this child spawns stand-ins of
                // its own: without leadership a teardown would reap only the
                // harness and leave its descendants alive — still holding the
                // capture pipes the parent is draining, which turns a fast
                // failure into a 60 s wait for EOF.
                cmd.process_group(0);
            }
            cmd.spawn()
                .expect("re-exec this test binary as the subprocess harness")
        });
        // Drain both pipes on their own threads: the stand-ins INHERIT them
        // (nothing on the production path can be told otherwise), so a leaked
        // grandchild holds a pipe open after the child itself exits, and a
        // blocking read-then-wait would deadlock on a full pipe besides.
        let mut out = child.held().stdout.take().expect("piped stdout");
        let mut err = child.held().stderr.take().expect("piped stderr");
        let (tx_o, rx_o) = std::sync::mpsc::channel();
        let (tx_e, rx_e) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut buf = String::new();
            use std::io::Read;
            let _ = out.read_to_string(&mut buf);
            let _ = tx_o.send(buf);
        });
        std::thread::spawn(move || {
            let mut buf = String::new();
            use std::io::Read;
            let _ = err.read_to_string(&mut buf);
            let _ = tx_e.send(buf);
        });
        // A capture that does not report is a HARNESS fault, never a verdict:
        // reported as such rather than substituted with a placeholder that
        // would then be folded into an assertion about the code.
        let grab = |what: &str, rx: std::sync::mpsc::Receiver<String>| {
            rx.recv_timeout(CAPTURE_REPORT_TIMEOUT).unwrap_or_else(|e| {
                panic!(
                    "harness: the {what} capture thread did not report ({e}) — a HARNESS \
                     failure, not a verdict on the code under test"
                )
            })
        };
        let deadline = Instant::now() + SIGINT_CHILD_TIMEOUT;
        let status = loop {
            match child
                .held()
                .try_wait()
                .expect("waiting on the subprocess harness")
            {
                Some(status) => break status,
                None if Instant::now() >= deadline => {
                    let _ = child.held().kill();
                    child.reap();
                    // Drain FIRST: a hang's output is the only evidence there is.
                    let out = grab("stdout", rx_o);
                    let err = grab("stderr", rx_e);
                    panic!(
                        "harness: the subprocess did not finish within \
                         {SIGINT_CHILD_TIMEOUT:?} — a HARNESS failure, not a verdict on the \
                         code under test\n--- child stdout ---\n{out}--- child stderr \
                         ---\n{err}"
                    );
                }
                None => std::thread::sleep(Duration::from_millis(50)),
            }
        };
        // `try_wait` REAPED it on the way out of that loop, so the pid is back
        // in the kernel's pool. Disarm before anything below can panic: the
        // guard's teardown would otherwise `getpgid`/`kill` a number that may
        // already belong to somebody else. The timeout arm above disarms for
        // the same reason.
        child.reap();
        let stdout = grab("stdout", rx_o);
        let stderr = grab("stderr", rx_e);
        let report = format!("--- child stdout ---\n{stdout}--- child stderr ---\n{stderr}");
        assert!(
            status.success(),
            "the subprocess harness must pass ({status}):\n{report}"
        );
        // EVERY marker. `libtest` exits 0 when `--exact` matches nothing (a
        // renamed module ⇒ a stale `SIGINT_CHILD_TEST`), and a child that
        // skipped a phase would otherwise pass too. The three ARM markers are
        // what stop the production marker meaning "the child died quickly for
        // some other reason": between them they establish that the ambient
        // hazard is real, that it is inherited across `execve`, and that the
        // helper is what defeats it.
        for mark in [
            MARK_IGNORED,
            MARK_BLOCKED,
            MARK_SCOPE,
            MARK_PRODUCTION,
            MARK_BACKSTOP,
        ] {
            assert!(
                stdout.contains(mark),
                "the child must have run every phase — missing `{mark}`:\n{report}"
            );
        }
    }

    /// The subprocess body of
    /// [`a_ros2_entry_is_spawned_so_the_teardown_sigint_can_reach_it`].
    /// `#[ignore]` + env-gated: it mutates this process's GLOBAL SIGINT
    /// disposition, SIGINT mask and SIGTERM disposition, which must never
    /// happen inside the shared lib-test binary.
    ///
    /// Five phases. Phases 1-2 probe the helper directly, each PAIRED with
    /// the HELPER-LESS spawn shape so no assertion rests on an unestablished
    /// premise; phase 3 probes its SCOPE (both arms there use the helper —
    /// the second of each pair is an anti-tautology control, not a helper-less
    /// one); phase 4 drives the production path; phase 5 is the backstop's
    /// own positive control.
    ///
    /// 1. **IGNORED** — the measured launcher shape. Without the helper an
    ///    `exec`ed stand-in ignores a per-PID SIGINT; with it, the SIGINT
    ///    kills it.
    /// 2. **BLOCKED** — the other half of the same hazard. A `SIG_DFL`
    ///    disposition is still undeliverable while SIGINT is blocked (it goes
    ///    pending), with the identical end symptom, so the helper unblocks it
    ///    too.
    /// 3. **SCOPE**, and the two halves go OPPOSITE ways. The DISPOSITION
    ///    half is SIGINT-only: with SIGTERM also ignored, a SIGTERM must STILL
    ///    be defeated while a SIGINT in the same state lands. The MASK half is
    ///    total: a SIGTERM the launcher BLOCKED must REACH the child, with the
    ///    helper-less spawn as its control. Each pair carries its own
    ///    anti-tautology arm.
    /// 4. **PRODUCTION** — the real spawn → the real `finish`. On Linux the
    ///    entry's `/proc/<pid>/status` `SigIgn` is read directly; everywhere,
    ///    the stop OUTCOME is asserted — the log capture is live, the child
    ///    is ours and alive going in, no backstop warn, and reaped coming
    ///    out.
    /// 5. **BACKSTOP** — the same `finish`, driven deliberately against a
    ///    HELPER-LESS child, so the string phase 4 asserts is ABSENT is pinned to
    ///    one the code still emits. Runs last, because `logs_contain`
    ///    accumulates over the whole test.
    #[cfg(unix)]
    #[test]
    #[ignore = "subprocess body: mutates this process's GLOBAL signal state; driven by \
                a_ros2_entry_is_spawned_so_the_teardown_sigint_can_reach_it"]
    #[tracing_test::traced_test]
    fn ros2_sigint_disposition_child() {
        if std::env::var_os(SIGINT_CHILD_ENV).is_none() {
            // Run directly rather than by the parent (a bare
            // `cargo test -- --ignored`): do nothing, and SAY so — libtest
            // would otherwise print `ok` for a body that did nothing. The
            // parent is the only caller that may hand this process a signal
            // state to mutate.
            println!("SKIPPED: subprocess body, driven by its parent test — not directly");
            return;
        }

        let tmp = tempfile::tempdir().expect("tempdir");
        let bin_dir = tmp.path().join("bin");
        let lib_dir = tmp.path().join("lib");
        let home = tmp.path().join("home");
        for d in [&bin_dir, &lib_dir, &home] {
            std::fs::create_dir_all(d).expect("fixture dirs");
        }
        let pidfile = tmp.path().join("stand-in.pid");
        std::env::set_var(SIGINT_PIDFILE_ENV, &pidfile);
        // Phases 1-3 own their `Command`, so they exec `sleep` DIRECTLY.
        // Phases 4-5 must be reached through `Command::new("ros2")` carrying
        // production's own argv, so they keep the script wrapper.
        let sleep_bin = resolve_tool("sleep");
        let stand_in = write_ros2_stand_in(&bin_dir);
        // `stage_ros2_child_env` existence-checks this by name only.
        std::fs::write(lib_dir.join(ros2_cmd::RMW_LIB_FILENAME), b"").expect("rmw stub");

        // `PATH` holds ONLY the stand-in's directory, so `Command::new("ros2")`
        // can resolve to nothing else — a real `ros2` on the machine cannot be
        // picked up, and a stand-in that failed to be written is an ENOENT
        // spawn failure rather than a silent pass on someone else's binary.
        std::env::set_var("PATH", &bin_dir);
        std::env::set_var(ros2_cmd::LIB_DIR_ENV, &lib_dir);
        std::env::set_var("HOME", &home);
        // Anything that would make `stage_ros2_child_env` refuse, or inject
        // an `LD_PRELOAD` the stand-in cannot load.
        for k in [
            "CERULION_RMW_ADOPT_TAKE",
            "CERULION_ROS2_PRELOAD",
            "LD_PRELOAD",
        ] {
            std::env::remove_var(k);
        }

        // ---- phase 1: an IGNORED SIGINT ----------------------------------
        set_disposition(libc::SIGINT, libc::SIG_IGN);
        set_blocked(libc::SIGINT, false);
        assert_eq!(
            sigint_disposition(),
            libc::SIG_IGN,
            "fixture precondition: SIGINT must really be ignored in this process"
        );
        probe_signal(
            &sleep_bin,
            false,
            libc::SIGINT,
            false,
            "CONTROL: with SIGINT ignored by the launcher, a child spawned WITHOUT the reset \
             (the helper-less shape) must IGNORE the per-PID teardown SIGINT — if it died here \
             there is nothing ambient for the helper to defeat and every arm below is vacuous",
        );
        probe_signal(
            &sleep_bin,
            true,
            libc::SIGINT,
            true,
            "the helper must hand the child SIG_DFL, so the same SIGINT now KILLS it",
        );
        println!("{MARK_IGNORED}");

        // ---- phase 2: a BLOCKED SIGINT -----------------------------------
        // The second half of the hazard: a SIG_DFL disposition is still
        // undeliverable while the signal is BLOCKED — it goes pending rather
        // than being discarded — and the end symptom is identical.
        //
        // Both arms take fork+exec (see `probe_signal`'s path-matched
        // control): on Linux the mask a child inherits was MEASURED to depend
        // on which spawn path `std` chose, so a control on the other path
        // would answer a different question. Fork+exec is the path the PRODUCTION
        // code takes, and the one whose mask behaviour std documents.
        //
        // The LITERAL helper-less shape — a spawn with no closure at all — is
        // still exercised end to end, by phase 5.
        set_disposition(libc::SIGINT, libc::SIG_DFL);
        set_blocked(libc::SIGINT, true);
        probe_signal(
            &sleep_bin,
            false,
            libc::SIGINT,
            false,
            "CONTROL: with SIGINT BLOCKED by the launcher, a child spawned WITHOUT the helper \
             must not die of it either — on the fork+exec path BOTH arms take, std documents \
             that it inherits the parent's mask",
        );
        probe_signal(
            &sleep_bin,
            true,
            libc::SIGINT,
            true,
            "the helper must UNBLOCK SIGINT as well as reset its disposition — a SIG_DFL \
             signal that stays blocked is exactly as undeliverable",
        );
        println!("{MARK_BLOCKED}");

        // ---- phase 3: the helper's SCOPE ---------------------------------
        // `child_signals` claims SIGINT and nothing else. With SIGTERM also
        // ignored, a widened helper would make this SIGTERM land.
        set_blocked(libc::SIGINT, false);
        set_disposition(libc::SIGINT, libc::SIG_IGN);
        set_disposition(libc::SIGTERM, libc::SIG_IGN);
        probe_signal(
            &sleep_bin,
            true,
            libc::SIGTERM,
            false,
            "the helper must reset SIGINT ONLY: with SIGTERM ignored by the launcher it must \
             STAY ignored in the child, or an operator's own `kill` silently changes meaning",
        );
        probe_signal(
            &sleep_bin,
            true,
            libc::SIGINT,
            true,
            "anti-tautology for the arm above: under the SAME ambient state a SIGINT still \
             lands, so 'the SIGTERM did not arrive' cannot just mean the probe stopped working",
        );
        // The MASK half of SCOPE, and why it needs its OWN arm: the helper's
        // two halves are ASYMMETRIC, so each needs its own pin. The arm above
        // is the DISPOSITION half — SIGINT only. This is the MASK half, and it
        // goes the OTHER way: the whole inherited mask is CLEARED, so a
        // SIGTERM the launcher BLOCKED must REACH the child.
        //
        // That asymmetry is the point. A disposition survives exec by design
        // and is a durable choice; a blocked set is transient state a launcher
        // holds for its own `sigwait` loop, and a child that inherits it is
        // one the operator's own `kill` cannot stop. Unblocking SIGINT alone
        // would smuggle every OTHER blocked signal through to `ros2`,
        // where a helper-less `posix_spawn` path hands it none — a narrowing
        // the helper must not introduce.
        set_disposition(libc::SIGTERM, libc::SIG_DFL);
        set_blocked(libc::SIGTERM, true);
        probe_signal(
            &sleep_bin,
            true,
            libc::SIGTERM,
            true,
            "the helper must CLEAR the inherited mask: a SIGTERM the launcher BLOCKED must \
             still reach the child, or installing the pre_exec closure smuggles the \
             launcher's whole blocked set through to `ros2`",
        );
        probe_signal(
            &sleep_bin,
            false,
            libc::SIGTERM,
            false,
            "anti-tautology for the mask arm: WITHOUT the helper the same blocked SIGTERM is \
             DEFEATED, so 'the SIGTERM arrived' cannot just mean the launcher never blocked it",
        );
        set_blocked(libc::SIGTERM, false);
        println!("{MARK_SCOPE}");

        // ---- phase 4: PRODUCTION — the real spawn + the real teardown -----
        // Back to the measured launcher shape for the end-to-end arm.
        set_disposition(libc::SIGINT, libc::SIG_IGN);
        assert_eq!(
            sigint_disposition(),
            libc::SIG_IGN,
            "fixture precondition: the production arm's whole meaning is 'under an ambient \
             SIG_IGN', and three phases have run since it was last set"
        );
        assert!(
            !is_blocked(libc::SIGINT),
            "fixture precondition: this arm exercises the IGNORED hazard, not the blocked one"
        );
        clear_pidfile(&pidfile);
        let entries = [Ros2Entry {
            id: "sidecar".to_string(),
            def: Ros2NodeDef {
                package: Some("demo_nodes_cpp".to_string()),
                executable: Some("talker".to_string()),
                ..Default::default()
            },
        }];
        let set = Ros2Children::spawn(
            &entries,
            tmp.path(),
            "g",
            Arc::new(AtomicBool::new(true)),
            PeerLossPolicy::Continue,
        )
        .unwrap_or_else(|e| panic!("the production spawn path must succeed: {e}"));
        let pid = await_stand_in(&pidfile);
        // `is_reaped` is a `waitid` on OUR OWN children, so a false here is
        // the proof that `pid` is this process's live, un-reaped child —
        // without it the `is_reaped` after teardown would also pass for a pid
        // that was never ours.
        assert!(
            !is_reaped(pid),
            "the entry must be OUR live child before teardown"
        );
        #[cfg(target_os = "linux")]
        {
            let (ignored, blocked) = proc_signal_state(pid, libc::SIGINT);
            assert!(
                !ignored && !blocked,
                "PRODUCTION: the teardown SIGINT must be able to REACH the spawned ros2 entry \
                 — /proc says ignored={ignored} blocked={blocked}"
            );
        }

        let t0 = Instant::now();
        set.finish(Ok(()))
            .unwrap_or_else(|e| panic!("our own stop keeps the run Ok: {e}"));
        let elapsed = t0.elapsed();

        // `teardown_gracefully` emits this unconditionally, on THIS thread,
        // inside the span, on both the graceful and the backstop path — so it
        // is a pure capture-liveness control that can never become the
        // discriminator. Without it the negative assertion below would pass
        // just as happily with a broken capture, a reworded warn, or the warn
        // moved to another thread.
        assert!(
            logs_contain("ros2 entries stopped"),
            "harness: the log capture is not live, so the backstop assertion below would be \
             vacuous"
        );
        // The load-proof oracle: the backstop is exactly what runs when the
        // graceful SIGINT was not delivered, and it says so at WARN.
        assert!(
            !logs_contain("did not exit within the shutdown grace window"),
            "PRODUCTION: the SIGKILL backstop must NOT have run — the graceful SIGINT was not \
             deliverable (teardown took {elapsed:?})"
        );
        // A CATASTROPHE bound only, at the convention this file settled on
        // in the post-mortem on
        // `the_freeze_fallback_still_judges_an_exit_it_can_see`: `finish`'s grace
        // deadline starts AFTER its per-child signal loop, so a correct
        // graceful teardown starved past the deadline can legitimately exceed
        // one whole GRACE — asserting `< GRACE` here would be the same
        // load inversion that arm's comment records going red on CI.
        assert!(
            elapsed < ROS2_CHILD_SHUTDOWN_GRACE * 2,
            "PRODUCTION: teardown ran catastrophically long — {elapsed:?}"
        );
        assert!(
            is_reaped(pid),
            "PRODUCTION: the entry must be reaped by teardown"
        );
        println!("{MARK_PRODUCTION}");

        // ---- phase 5: the backstop's OWN positive control -----------------
        // The load-proof oracle of phase 4 is a NEGATIVE assertion over a string
        // literal, and a negative assertion is only ever as good as the proof
        // that the literal is still the one the code emits — reword
        // `teardown_gracefully`'s warn and it silently becomes `!false`
        // forever. (On Linux the `/proc` oracle would still catch it; on
        // macOS, where `/proc` is compiled out, nothing would.) So drive the
        // backstop DELIBERATELY through the SAME `finish`, with a child
        // spawned the way `Ros2Children::spawn` would spawn it WITHOUT the helper,
        // and require it to announce itself in exactly those words. This runs
        // AFTER phase 4 because `logs_contain` accumulates over the whole
        // test.
        //
        // It doubles as the end-to-end COST of a missing helper: a `ros2` child that
        // cannot receive the graceful SIGINT is SIGKILLed a whole grace window
        // later, taking `ros2 launch`'s nodes with it un-wound-down.
        // The same two preconditions phase 4 guards its meaning with: this
        // arm inherits its ambient state from phase 4 by POSITION, and a
        // future edit there would otherwise make it fail with "matching a
        // string nothing emits any more" — sending the reader to
        // `teardown_gracefully`'s warn text for an ambient-state problem.
        assert_eq!(
            sigint_disposition(),
            libc::SIG_IGN,
            "fixture precondition: the backstop control needs the IGNORED hazard"
        );
        assert!(
            !is_blocked(libc::SIGINT),
            "fixture precondition: the backstop control exercises the IGNORED hazard"
        );
        clear_pidfile(&pidfile);
        let mut cmd = std::process::Command::new(&stand_in);
        {
            use std::os::unix::process::CommandExt;
            // The helper-less shape: the process group, and NO signal reset.
            cmd.process_group(0);
        }
        // Same reason as `probe_signal`: never let a stand-in hold the pipes
        // the parent is draining.
        cmd.stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null());
        // Armed until `for_test` takes ownership: `await_stand_in` and the pid
        // cross-check below can both panic, and a raw `Child` dropped while
        // unwinding leaves the stand-in running.
        let mut child = StandInGuard::new(
            cmd.spawn()
                .unwrap_or_else(|e| panic!("harness: could not spawn the backstop control: {e}")),
        );
        let expected_pid = child.pid();
        let pid = await_stand_in(&pidfile);
        assert_eq!(
            pid, expected_pid,
            "harness: the backstop control must report its OWN pid"
        );
        let set = Ros2Children::for_test(
            "g",
            PeerLossPolicy::Continue,
            vec![("sidecar", child.release())],
        );
        let t0 = Instant::now();
        set.finish(Ok(()))
            .unwrap_or_else(|e| panic!("the backstop still leaves our own stop Ok: {e}"));
        let elapsed = t0.elapsed();
        assert!(
            logs_contain("did not exit within the shutdown grace window"),
            "CONTROL: a child that cannot receive the graceful SIGINT must reach the SIGKILL \
             backstop AND say so in those words — otherwise the production arm's negative \
             assertion above is matching a string nothing emits any more"
        );
        assert!(
            elapsed >= ROS2_CHILD_SHUTDOWN_GRACE,
            "CONTROL: reaching the backstop means the WHOLE grace window was spent — {elapsed:?}"
        );
        assert!(
            is_reaped(pid),
            "CONTROL: the backstop must still reap the entry"
        );
        println!("{MARK_BACKSTOP}");
    }
}
