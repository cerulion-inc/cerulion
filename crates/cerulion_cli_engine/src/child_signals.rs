// SPDX-License-Identifier: AGPL-3.0-only
//! The one place Cerulion makes sure a signal it will SEND a child can
//! actually REACH that child.
//!
//! **Why this exists at all.** A `std::process::Command` child inherits both
//! its parent's signal DISPOSITIONS and its parent's signal MASK, and both
//! survive `execve` (POSIX: exec resets HANDLERS to `SIG_DFL`, but an
//! IGNORED disposition and the blocked set are carried over). `std` resets
//! only `SIGPIPE` and deliberately inherits the mask
//! (`sys/process/unix/unix.rs`: "Inherit the signal mask from the parent
//! rather than resetting it" — stated on the fork/exec path and again for
//! `posix_spawn`), so everything else passes straight through.
//!
//! Which launchers actually do this, measured:
//!
//! - a shell script's `cmd &` — both `bash` and `sh` set SIGINT to `SIG_IGN`
//!   for an asynchronous job, as POSIX requires — and `nohup cmd &`: YES.
//! - a PLAIN systemd unit: NO. It resets every disposition and the mask
//!   before exec, so it only reproduces this through a shell wrapper that
//!   backgrounds the command.
//! - a supervisor using the block-plus-`sigwait` dedicated-signal-thread
//!   pattern, a JVM `ProcessBuilder` parent, a `subprocess` call with
//!   `restore_signals=False`: these hand over a BLOCKED SIGINT rather than an
//!   ignored one, which defeats the same stop by a different route.
//!
//! Either one matters wherever Cerulion uses a signal as ITS OWN
//! graceful-stop protocol: an ignored signal is discarded and a blocked one
//! goes pending, so in both cases the stop never happens, the grace window
//! expires, and the SIGKILL backstop turns every shutdown into a hard kill.
//! A FOREIGN child cannot be relied on to repair this for us — `ros2` is a
//! Python console script, and CPython preserves an inherited `SIG_IGN`
//! rather than installing its own `KeyboardInterrupt` handler over it.
//!
//! The rule this module encodes: **a child Cerulion will signal itself can
//! receive exactly the signals Cerulion sends it, and smuggles no inherited
//! BLOCK through to anything else.** The two halves are asymmetric on purpose
//! — see [`make_sigint_deliverable`]. `cerulion_core`'s
//! `state_carrier::child::reset_signals` goes further on the DISPOSITION axis
//! (every disposition 1..32 to `SIG_DFL`) because it owns that child's entire
//! environment; here the caller owns only its own protocol, so a disposition
//! the operator chose for a signal this protocol never sends is left alone. The MASK half
//! is the same in both: cleared.

/// Make SIGINT DELIVERABLE to the spawned child: `SIG_DFL` disposition, and
/// SIGINT unblocked — whatever this process's own SIGINT state is.
///
/// Call it on any `Command` whose child Cerulion will later send a SIGINT to
/// as a graceful-stop request. Without it, a launcher that ignores or blocks
/// SIGINT makes that request undeliverable (module docs above).
///
/// **Both halves, because either one alone leaves the same symptom.** A
/// `SIG_DFL` disposition is still undeliverable while the signal is BLOCKED
/// — it goes pending instead of being discarded — and the observable outcome
/// is identical: the grace expires and the backstop SIGKILLs the child's
/// whole process group. This mirrors the adjudication in
/// `cerulion_core::state_carrier::child::reset_signals` ("Both halves
/// matter … a child that inherited a blocked set would be deaf to exactly
/// the signals the reaper uses to end it").
///
/// **The two halves are treated ASYMMETRICALLY, deliberately.**
///
/// The DISPOSITION reset is SIGINT-only. A disposition survives `execve` BY
/// DESIGN, so an inherited `SIG_IGN` is a durable, deliberate choice. The
/// teardown protocol these call sites share sends SIGSTOP, SIGINT and SIGCONT
/// (`graph_cmd::ChildGuard::stop_running_child`) then a SIGKILL backstop, and
/// of those SIGSTOP and SIGKILL can be neither caught, ignored nor blocked,
/// while SIGCONT's *continue* action happens even when it is ignored or
/// blocked (only a handler for it would be deferred). SIGINT is therefore the
/// one whose DISPOSITION an inherited state can defeat, and rewriting any
/// other would silently change how an operator's own `kill` reaches a `ros2`
/// or `cerulion-connectd` child.
///
/// The MASK is cleared ENTIRELY, and that is a different question. A blocked
/// set is transient state a launcher holds for its OWN loop — the
/// block-plus-`sigwait` supervisor pattern blocks SIGTERM so a dedicated
/// thread can `sigwait` it — never a statement about what a grandchild should
/// be deaf to. A child that inherits a blocked SIGTERM is one the operator's
/// own `kill` cannot stop: the very bug class this helper exists to remove,
/// one signal over.
///
/// It is also the baseline a closure-free spawn gives. MEASURED on Linux:
/// a child spawned with NO `pre_exec` closure (so, `posix_spawn`) did not
/// carry the parent's blocked SIGINT. Installing a closure moves the child to
/// fork+exec, which `std` documents as INHERITING the mask — so unblocking
/// SIGINT alone would have handed `ros2` every OTHER signal the launcher had
/// blocked, where a closure-free spawn gave it none. Clearing restores that baseline rather
/// than inventing one, and matches the sibling
/// `state_carrier::child::reset_signals`, which clears the whole mask for the
/// same reason.
///
/// A caller's spawn-failure message should therefore not assume `ENOENT`:
/// this closure can, in principle, fail the spawn itself.
///
/// RESIDUAL, stated rather than implied. This is applied at the two FOREIGN-
/// child spawn sites (`connect_cmd::spawn_and_wait`,
/// `ros2_graph::Ros2Children::spawn`). Two `self_exe` children are ALSO sent
/// a per-PID SIGINT and do NOT call it — `graph run-worker` and
/// `run-gateway`. For the DISPOSITION half they self-heal once up, leaving
/// only a startup window: `cerulion`'s `setup_ctrlc_handler` uses
/// `ctrlc::set_handler`, whose `overwrite = true` path replaces an inherited
/// `SIG_IGN` (`try_set_handler` would NOT — it restores the old action and
/// returns `EEXIST` precisely under an inherited `SIG_IGN`). For the MASK
/// half they do NOT self-heal at all: `ctrlc` installs a handler through
/// `nix::sys::signal::sigaction` and never touches the process mask, and
/// nothing else in the tree unblocks SIGINT — so a launcher that BLOCKS
/// SIGINT leaves those two deaf for their whole life and every graceful stop
/// ends in the SIGKILL backstop.
///
/// Costs, stated: a `pre_exec` closure makes `std` take the fork+exec path
/// instead of `posix_spawn` (`get_closures().is_empty()` is one of its
/// preconditions). These are graph-startup spawns, not a hot path.
///
/// That path change is also why the two halves ship TOGETHER rather than the
/// mask being belt-and-braces. MEASURED on Linux, twice and
/// deterministically: a child spawned with NO `pre_exec` closure did not
/// carry the parent's BLOCKED SIGINT (`/proc` `SigBlk`), while it DID carry
/// the parent's ignored disposition. Installing this closure moves the child
/// onto the path whose mask behaviour `std` documents as inherited, so a
/// disposition reset shipped ALONE could hand a child a blocked SIGINT a
/// closure-free spawn would not have had. The unblock is what keeps the reset from
/// undoing its own benefit.
///
/// A no-op on non-Unix: there is no SIGINT to make deliverable, and the
/// graceful step of the teardown protocol is itself a no-op there — see
/// `graph_cmd::ChildGuard::stop_running_child`, whose non-Unix arm says so
/// loudly and leaves the grace-deadline SIGKILL as the only stop.
pub(crate) fn make_sigint_deliverable(cmd: &mut std::process::Command) {
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        // SAFETY: the closure runs in the forked child between fork and exec,
        // where the process is single-threaded, and calls only
        // `signal(2)`/`sigemptyset(3)`/`sigprocmask(2)` — all
        // async-signal-safe, and all four on the PLATFORM's own list as well
        // as POSIX's (`pthread_sigmask` is on POSIX's list but not on the
        // macOS `sigaction(2)` one; `sigprocmask` is well defined here
        // because the child is single-threaded, and it is what the sibling
        // `state_carrier::child::reset_signals` uses for the same reason). No
        // allocation, no locks, no shared state touched —
        // `io::Error::last_os_error` stores a raw OS error inline.
        unsafe {
            cmd.pre_exec(|| {
                // Failure is unreachable for SIGINT (`signal` fails only
                // EINVAL, for an invalid number or SIGKILL/SIGSTOP), but it is
                // reported rather than discarded for the same reason `std`
                // checks its own SIGPIPE reset a few lines above this closure:
                // if it ever DID fire, silence would produce exactly the
                // undeliverable stop this helper exists to prevent, and a loud
                // refusal at spawn beats a hard kill at shutdown.
                if libc::signal(libc::SIGINT, libc::SIG_DFL) == libc::SIG_ERR {
                    return Err(std::io::Error::last_os_error());
                }
                let mut empty: libc::sigset_t = std::mem::zeroed();
                if libc::sigemptyset(&mut empty) != 0
                    || libc::sigprocmask(libc::SIG_SETMASK, &empty, std::ptr::null_mut()) != 0
                {
                    return Err(std::io::Error::last_os_error());
                }
                Ok(())
            });
        }
    }
    #[cfg(not(unix))]
    {
        let _ = cmd;
    }
}
