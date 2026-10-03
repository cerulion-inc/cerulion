// SPDX-License-Identifier: AGPL-3.0-only
//! Where the listener drain is allowed to run, walked from the transport's own
//! source.
//!
//! # The rule, and why it is a cost rule
//!
//! A drain runs after a read that REMOVED frames from the queue, and nowhere else
//! on a consuming path. `wait_for_message` drains before its wait instead, which is
//! a different decision about a different hazard (an event left queued re-fires a
//! level-triggered wait forever) and is declared as such below.
//!
//! On iceoryx2 0.10 a drain of an EMPTY listener is two `recvmsg` calls, two
//! sequentially consistent atomic operations and a walk of the shared-memory
//! counting bitset, where 0.9.1 was one `recvmsg`. A call put back at the HEAD of a
//! read therefore pays that on every receive that finds nothing, which is the
//! common case for a colocated edge whose notify is elided at the source. No
//! behaviour test can see it: the extra drain changes no answer and only costs
//! time. That is what this walk is for.
//!
//! # What it pins, and why a spelling set alone is not enough
//!
//! Three things. The SPELLINGS, so a drain cannot slip in under another name. The
//! set of functions allowed to hold a drain site at all. And, per function, the
//! EXACT number of sites it holds together with the spellings it may hold them in.
//!
//! The third is the one that catches a cost put back inside a read entry. The read
//! entries own policy sites of their own, at their exits, so a walk that attributes
//! sites only to functions still reads a clean set when a drain is added at the head
//! of one of them: the owner was already declared and nothing it asserts moves.
//! Pinning the count per owner makes that addition a mismatch, named by its line,
//! and pinning the spelling per owner makes a read entry's direct wrapper or core
//! call a finding even where the count happens to match. The same count, read the
//! other way, is why deleting one exit's drain is a finding too.
//!
//! It is NOT a socket-fill guard. 0.10 removed that hazard at the source, which
//! `notify_delivery_latch`'s own module doc records: the event id and its repeat
//! count live in a shared-memory counting bitset, the doorbell carries one byte, a
//! full doorbell is swallowed rather than refused, and a notify into a listener
//! that already holds an unconsumed wake skips the send entirely.
//!
//! ```bash
//! cargo test -p cerulion_core --test receive_drain_policy_walk_test
//! ```

use std::path::Path;

/// The file the policy lives in.
const SUBSCRIBER: &str = "src/transport/subscriber.rs";

/// The infallible wrapper over the drain.
const WRAPPER: &str = "self.drain_stale_events()";
/// The fallible core, however its error is handled.
const CORE: &str = "self.try_drain_stale_events()";
/// The policy function, matched by its NAME PREFIX so every call site is seen
/// whatever it passes, including one that would hard-code "a frame was found" at a
/// site that found none.
const POLICY: &str = "self.drain_listener_after_removals(";

/// Every spelling that drains the listener, so a drain cannot slip in under a
/// different name. Matched anywhere in the line, comments excluded, because the
/// wrapper's own call is written as a `let _ =` binding.
///
/// `self.drain_event_notifications()` is deliberately absent, because its callers
/// live in another crate and a walk over this file could never hit it; the forwarder
/// still appears as an owner, through the core call in its own body.
const DRAIN_CALLS: &[&str] = &[WRAPPER, CORE, POLICY];

/// A function allowed to drain the listener: how many sites it holds, which
/// spellings it may hold them in, and the reason it may hold any.
struct Owner {
    /// The method name, as the walk reads it off a four-space-indented signature.
    name: &'static str,
    /// The exact number of drain sites in its body. Found-not-equal-to-declared is
    /// a finding in both directions.
    sites: usize,
    /// The spellings this owner may use. A site in it under any other spelling is a
    /// finding even when the count matches.
    spellings: &'static [&'static str],
    /// Why this function may drain at all.
    reason: &'static str,
}

/// Every function allowed to drain the listener, with its site count, its spellings
/// and the reason it may.
///
/// Seven entries, eleven sites. One place DECIDES, through the wrapper. Three read
/// bodies take the policy: the two single-frame loops, each at its three exits, and
/// the batch body once per call, which is why the two entries over that body own no
/// site of their own. One path drains before it waits as well as taking the policy
/// after its read. Two functions are the drain and nothing else, the infallible
/// wrapper and the public forwarder, each over the fallible core.
///
/// Out of this walk's reach: the raw-fd waiter in the ROS 2 middleware layer drains
/// through `drain_event_notifications`, which is a call in another crate.
const ALLOWED: &[Owner] = &[
    Owner {
        name: "drain_listener_after_removals",
        sites: 1,
        spellings: &[WRAPPER],
        reason: "the one place the policy is decided: a drain after a read that REMOVED frames, \
                 and nothing after a read that removed none",
    },
    Owner {
        name: "wait_for_message",
        sites: 1,
        spellings: &[WRAPPER],
        reason: "the iceoryx2 timed-wait path, which must drain BEFORE it waits: an event left \
                 in the queue re-fires a level-triggered wait forever, which is the spin class. \
                 It also takes the policy after its read, through the batch body",
    },
    Owner {
        name: "drain_stale_events",
        sites: 1,
        spellings: &[CORE],
        reason: "the infallible wrapper, which is the drain and nothing else: it swallows the \
                 error for the read paths, and its callers here are the policy function and the \
                 pre-wait drain in `wait_for_message`",
    },
    Owner {
        name: "try_receive_one",
        sites: 3,
        spellings: &[POLICY],
        reason: "a read entry that takes the policy at each of its three exits, reporting the \
                 removals it popped whether it delivered a frame or not",
    },
    Owner {
        name: "try_receive_one_owned",
        sites: 3,
        spellings: &[POLICY],
        reason: "the same, for the loaned take",
    },
    Owner {
        name: "drain_samples",
        sites: 1,
        spellings: &[POLICY],
        reason: "the batch body, which runs the policy once per call on every exit, so the two \
                 entries over it, `try_receive` and `try_receive_for_drain`, take the policy \
                 TRANSITIVELY and own no call site of their own",
    },
    Owner {
        name: "drain_event_notifications",
        sites: 1,
        spellings: &[CORE],
        reason: "the public forwarder over the same core, which is also a drain and nothing \
                 else: it exists so a caller that waits on the raw file descriptor can clear \
                 the queue and see the error, and the ROS 2 middleware layer's waiter calls it \
                 once per iteration from another crate",
    },
];

/// The enclosing function name for a line index, from the nearest preceding
/// signature at method indentation.
///
/// Deliberately literal: it matches a four-space-indented `fn`, which is what a
/// method of an `impl` block in this file is, and nothing else. A helper nested
/// inside a function body would be indented further and is not a site this policy
/// is about.
fn enclosing_fn(lines: &[&str], idx: usize) -> Option<String> {
    for line in lines[..=idx].iter().rev() {
        let trimmed = line.trim_start();
        let indent = line.len() - trimmed.len();
        if indent != 4 {
            continue;
        }
        let head = trimmed
            .strip_prefix("pub(crate) ")
            .or_else(|| trimmed.strip_prefix("pub "))
            .unwrap_or(trimmed);
        if let Some(rest) = head.strip_prefix("fn ") {
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() {
                return Some(name);
            }
        }
    }
    None
}

#[test]
fn only_the_declared_functions_drain_the_listener() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(SUBSCRIBER);
    let source = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("the walk must be able to read {}: {e}", path.display()));
    let lines: Vec<&str> = source.lines().collect();

    // One entry per drain site: its line, the spelling it is written in, and the
    // method it sits in.
    let mut sites: Vec<(usize, &str, String)> = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        let trimmed = line.trim_start();
        // A COMMENT naming a drain is not a drain. Everything else is matched
        // anywhere in the line, not just at its start, because the file's own
        // infallible wrapper writes `let _ = self.try_drain_stale_events();`.
        if trimmed.starts_with("//") {
            continue;
        }
        let Some(spelling) = DRAIN_CALLS.iter().find(|call| trimmed.contains(*call)) else {
            continue;
        };
        let owner = enclosing_fn(&lines, i).unwrap_or_else(|| {
            panic!(
                "{}:{}: a drain call outside any method, which the walk cannot \
                 attribute, so the policy cannot be checked",
                SUBSCRIBER,
                i + 1
            )
        });
        sites.push((i + 1, spelling, owner));
    }

    // Per-spelling hit counts, because a pattern that can never fire is the same
    // blindness as a walk that matched nothing, so the count per spelling is printed
    // beside the total.
    let per_call: Vec<(&str, usize)> = DRAIN_CALLS
        .iter()
        .map(|call| {
            let hits = sites.iter().filter(|(_, s, _)| s == call).count();
            (*call, hits)
        })
        .collect();
    let judged: Vec<String> = ALLOWED
        .iter()
        .map(|owner| {
            let found: Vec<usize> = sites
                .iter()
                .filter(|(_, _, f)| f == owner.name)
                .map(|(line, _, _)| *line)
                .collect();
            format!(
                "{} declared {} found {} at {:?}",
                owner.name,
                owner.sites,
                found.len(),
                found
            )
        })
        .collect();

    // Rule: a gate reports what it judged. A walk that matched nothing would
    // otherwise print the same nothing a correct one does.
    println!(
        "receive_drain_policy_walk: read {SUBSCRIBER} ({} lines), attributed {} drain call \
         sites to {} declared functions. Hits per spelling: {:?}. Per owner: {:?}",
        lines.len(),
        sites.len(),
        ALLOWED.len(),
        per_call,
        judged
    );
    assert!(
        !sites.is_empty(),
        "the walk found NO drain call in {SUBSCRIBER} under any of {} spellings. Zero judged \
         is a finding, never a pass: either every call was renamed, in which case this walk is \
         now blind, or the listener is never drained at all",
        DRAIN_CALLS.len()
    );

    let undeclared: Vec<String> = sites
        .iter()
        .filter(|(_, _, f)| !ALLOWED.iter().any(|owner| owner.name == f.as_str()))
        .map(|(line, spelling, f)| format!("{SUBSCRIBER}:{line} in `{f}` ({spelling})"))
        .collect();
    assert!(
        undeclared.is_empty(),
        "these functions drain the listener and are not declared here: {undeclared:?}.\n\n\
         The drain belongs AFTER a read that consumed frames, not at the head of a read. A \
         call at the head costs one syscall on every receive that finds nothing, which no \
         behaviour test can see because it only costs time. Either move it behind \
         `drain_listener_after_removals`, or declare it above WITH the reason it must run \
         before its read."
    );

    // A site in a declared owner, written in a spelling that owner does not declare.
    // A read entry takes the policy and nothing else; a direct wrapper or core call
    // in one of them is a drain the policy did not decide, and it reds here even
    // when the owner's site count happens to match.
    let wrong_spelling: Vec<String> = sites
        .iter()
        .filter_map(|(line, spelling, f)| {
            let owner = ALLOWED.iter().find(|owner| owner.name == f.as_str())?;
            (!owner.spellings.contains(spelling))
                .then(|| format!("{SUBSCRIBER}:{line} in `{f}` drains as `{spelling}`"))
        })
        .collect();
    assert!(
        wrong_spelling.is_empty(),
        "these drain sites are written in a spelling their function does not declare: \
         {wrong_spelling:?}.\n\n\
         A read entry takes the policy through `drain_listener_after_removals`, which drains \
         only when the read removed frames. A wrapper or core call written straight into one \
         drains whatever the read did, which is the cost this policy removed. Either route it \
         through the policy, or widen that function's declared spellings WITH the reason."
    );

    // The site COUNT per declared owner, in both directions: an extra site is a cost
    // put back inside a function that already drains, and a missing one is an exit
    // whose drain is gone.
    let counts: Vec<String> = ALLOWED
        .iter()
        .filter_map(|owner| {
            let found: Vec<usize> = sites
                .iter()
                .filter(|(_, _, f)| f == owner.name)
                .map(|(line, _, _)| *line)
                .collect();
            (found.len() != owner.sites).then(|| {
                format!(
                    "`{}` declares {} site(s) and holds {} at {:?} ({})",
                    owner.name,
                    owner.sites,
                    found.len(),
                    found,
                    owner.reason
                )
            })
        })
        .collect();
    assert!(
        counts.is_empty(),
        "these functions hold a different number of drain sites than they declare: \
         {counts:?}.\n\n\
         MORE than declared is a drain added inside a function that already drains, which \
         attribution by function alone cannot see: at a read's head it costs one syscall on \
         every receive that finds nothing. FEWER is an exit that does not drain after it \
         removed frames, which leaves events queued behind a consumed read. Move the call, or \
         change the declared count above WITH the reason."
    );
}
