// SPDX-License-Identifier: AGPL-3.0-only
//! Authorization (deny-by-default) + hash-chained receipt audit-log tests.

use cerud::authz::{Authorizer, DenyAllAuthorizer, PermissiveDevAuthorizer};
use std::io::Write;

use cerud::error::CerudError;
use cerud::receipt::{
    args_digest, compute_entry_hash, read_all, rotated_path, sync_parent_dir, verify_chain,
    ChainError, Receipt, ReceiptLog, ReceiptOutcome, GENESIS_HASH, ROTATION_VERB,
};
use cerud::transport::CallerIdentity;

// ─────────────────────────────────── authz ─────────────────────────────────

#[test]
fn deny_all_refuses_every_verb() {
    let authz = DenyAllAuthorizer;
    let caller = CallerIdentity::local_dev();
    for verb in ["inventory", "log-tail", "restart", "anything"] {
        let decision = authz.authorize(&caller, verb, &serde_json::json!({}));
        assert!(!decision.is_allowed(), "verb {verb} should be denied");
    }
    // A verified caller is still denied when no access list is configured.
    let verified = CallerIdentity::verified("iroh-node-abc");
    assert!(!authz
        .authorize(&verified, "inventory", &serde_json::json!({}))
        .is_allowed());
}

#[test]
fn permissive_dev_allows_every_verb() {
    let authz = PermissiveDevAuthorizer;
    let caller = CallerIdentity::local_dev();
    for verb in ["inventory", "log-tail", "restart"] {
        assert!(authz
            .authorize(&caller, verb, &serde_json::json!({}))
            .is_allowed());
    }
}

// ─────────────────────────────── args digest ───────────────────────────────

#[test]
fn args_digest_is_canonical_and_order_independent() {
    // Two objects with the SAME content but different key order → same digest.
    let a = serde_json::json!({"graph": "perception", "lines": 100});
    let b = serde_json::json!({"lines": 100, "graph": "perception"});
    assert_eq!(args_digest(&a), args_digest(&b));

    // A different value → a different digest.
    let c = serde_json::json!({"graph": "planning", "lines": 100});
    assert_ne!(args_digest(&a), args_digest(&c));

    // Digest is a 64-char lowercase-hex SHA-256.
    let d = args_digest(&a);
    assert_eq!(d.len(), 64);
    assert!(d
        .chars()
        .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()));
}

// ─────────────────────────── receipt hash chain ────────────────────────────

/// Verify a MID-CHAIN slice (a post-rotation segment that need NOT be
/// genesis-rooted): every entry's stored `entry_hash` recomputes from its own
/// fields (tamper detection) AND every entry after the first back-links to its
/// predecessor within the slice. Unlike [`verify_chain`], this does NOT require
/// the first entry to link to genesis (a surviving post-prune segment begins
/// mid-chain) — but it DOES recompute the FIRST entry's self-hash, so it covers
/// a SINGLE-ENTRY segment that a `windows(2)` iteration skips entirely. Returns
/// the failing entry index on the first fault.
fn verify_midchain_slice(entries: &[Receipt]) -> Result<(), usize> {
    let mut prior: Option<&Receipt> = None;
    for (i, r) in entries.iter().enumerate() {
        let recomputed = compute_entry_hash(
            &r.prev_hash,
            r.seq,
            r.timestamp_ns,
            &r.caller,
            &r.verb,
            &r.args_digest,
            &r.outcome,
        );
        if recomputed != r.entry_hash {
            return Err(i); // self-hash mismatch — a field was altered in place
        }
        if let Some(p) = prior {
            if r.prev_hash != p.entry_hash {
                return Err(i); // back-link broken
            }
        }
        prior = Some(r);
    }
    Ok(())
}

/// Hand-build a 3-entry chain via the pure hash function (the oracle).
fn hand_chain() -> Vec<Receipt> {
    let mut prev = GENESIS_HASH.to_string();
    let mut out = Vec::new();
    let rows = [
        (0u64, 1000u64, "alice", "inventory", ReceiptOutcome::Ok),
        (1, 2000, "bob", "restart", ReceiptOutcome::Denied),
        (
            2,
            3000,
            "alice",
            "log-tail",
            ReceiptOutcome::Error("path_traversal".to_string()),
        ),
    ];
    for (seq, ts, caller, verb, outcome) in rows {
        let digest = args_digest(&serde_json::json!({"seq": seq}));
        let entry_hash = compute_entry_hash(&prev, seq, ts, caller, verb, &digest, &outcome);
        out.push(Receipt {
            seq,
            timestamp_ns: ts,
            caller: caller.to_string(),
            verb: verb.to_string(),
            args_digest: digest,
            outcome,
            prev_hash: prev.clone(),
            entry_hash: entry_hash.clone(),
        });
        prev = entry_hash;
    }
    out
}

#[test]
fn intact_chain_verifies() {
    let chain = hand_chain();
    // First entry links to genesis; each links to the prior.
    assert_eq!(chain[0].prev_hash, GENESIS_HASH);
    assert_eq!(chain[1].prev_hash, chain[0].entry_hash);
    assert_eq!(chain[2].prev_hash, chain[1].entry_hash);
    verify_chain(&chain).unwrap();
    // An empty chain is vacuously intact.
    verify_chain(&[]).unwrap();
}

#[test]
fn tampering_a_field_is_detected() {
    let mut chain = hand_chain();
    // Alter entry 1's caller in place WITHOUT recomputing its hash — exactly
    // what an attacker editing the log would do.
    chain[1].caller = "mallory".to_string();
    match verify_chain(&chain) {
        Err(ChainError::Tampered { index }) => assert_eq!(index, 1),
        other => panic!("expected Tampered at 1, got {other:?}"),
    }
}

#[test]
fn tampering_the_outcome_is_detected() {
    let mut chain = hand_chain();
    // Flip a Denied into an Ok (the classic "hide the refusal" edit).
    chain[1].outcome = ReceiptOutcome::Ok;
    assert!(matches!(
        verify_chain(&chain),
        Err(ChainError::Tampered { index: 1 })
    ));
}

#[test]
fn removing_an_entry_breaks_the_link() {
    let mut chain = hand_chain();
    chain.remove(1); // now entry (old 2) links to entry 0's hash, not entry 1's
    match verify_chain(&chain) {
        Err(ChainError::BrokenLink { index, .. }) => assert_eq!(index, 1),
        other => panic!("expected BrokenLink at 1, got {other:?}"),
    }
}

#[test]
fn boundary_truncated_prefix_still_verifies_as_the_shorter_chain() {
    // Semantics pinned LOUDLY: a log truncated at a LINE BOUNDARY (a valid
    // prefix of a longer chain) verifies as exactly that prefix — a shorter
    // valid chain, not a tamper. `verify_chain` covers only the slice it is
    // given, so the reported length is the shorter one.
    let full = hand_chain();
    let prefix = &full[..2];
    verify_chain(prefix).unwrap();
    assert_eq!(prefix.len(), 2);
    // The prefix's last entry is entry 1; a chain re-extended from it links.
    assert_eq!(prefix.last().unwrap().seq, 1);
}

// ─────────────────── crash-torn tail recovery on open ──────────────────────

#[test]
fn open_recovers_a_crash_torn_final_line_and_continues_the_chain() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    // Write two valid, newline-terminated entries.
    {
        let mut log = ReceiptLog::open(&path).unwrap();
        log.record_at("a", "inventory", "d0", ReceiptOutcome::Ok, 10)
            .unwrap();
        log.record_at("b", "inventory", "d1", ReceiptOutcome::Ok, 20)
            .unwrap();
    }

    // Simulate a crash mid-append: a partial JSON line with NO trailing newline.
    {
        let mut f = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        f.write_all(b"{\"seq\":2,\"timestamp_ns\":30,\"caller\":\"c\"")
            .unwrap();
    }

    // Reopen: the torn tail is truncated (recovered) and the chain continues.
    let mut log = ReceiptLog::open(&path).unwrap();
    assert_eq!(
        log.next_seq(),
        2,
        "recovered next seq is 2 (torn line dropped)"
    );

    // read_all now shows exactly the two intact entries.
    let entries = read_all(&path).unwrap();
    assert_eq!(entries.len(), 2);
    verify_chain(&entries).unwrap();

    // A fresh append chains correctly onto the recovered tail.
    log.record_at("d", "inventory", "d2", ReceiptOutcome::Ok, 40)
        .unwrap();
    let entries = read_all(&path).unwrap();
    assert_eq!(entries.len(), 3);
    assert_eq!(entries[2].seq, 2);
    assert_eq!(entries[2].prev_hash, entries[1].entry_hash);
    verify_chain(&entries).unwrap();
}

#[test]
fn open_refuses_mid_file_corruption() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    // Three valid entries.
    {
        let mut log = ReceiptLog::open(&path).unwrap();
        for i in 0..3u64 {
            log.record_at("a", "inventory", "d", ReceiptOutcome::Ok, i)
                .unwrap();
        }
    }

    // Corrupt a NON-final (mid-file) COMPLETE line in place.
    let content = std::fs::read_to_string(&path).unwrap();
    let mut lines: Vec<String> = content.lines().map(|s| s.to_string()).collect();
    lines[0] = "THIS IS NOT JSON".to_string();
    let mut rewritten = lines.join("\n");
    rewritten.push('\n'); // still newline-terminated: NOT a torn tail
    std::fs::write(&path, rewritten).unwrap();

    // Mid-file corruption is a HARD error — tamper evidence, never recovered.
    // (ReceiptLog is not Debug, so handle the Ok arm without formatting it.)
    match ReceiptLog::open(&path) {
        Err(CerudError::Receipt(_)) => {}
        Err(other) => panic!("expected a Receipt error, got a different error: {other:?}"),
        Ok(_) => panic!("mid-file corruption must be refused, but open succeeded"),
    }
    // read_all likewise refuses (the corrupt line is not the torn final line).
    assert!(read_all(&path).is_err());
}

#[test]
fn open_refuses_a_sealed_final_entry_with_its_newline_deleted_as_tamper() {
    // The tamper-masking regression guard: normal operation
    // ALWAYS terminates a synced entry with '\n'. A COMPLETE, chain-valid final
    // entry whose ONLY defect is a missing newline means someone deleted the
    // terminator to erase the newest audit record — that must be a HARD error,
    // NOT silently truncated.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");
    {
        let mut log = ReceiptLog::open(&path).unwrap();
        log.record_at("a", "inventory", "d0", ReceiptOutcome::Ok, 10)
            .unwrap();
        log.record_at("b", "restart", "d1", ReceiptOutcome::Ok, 20)
            .unwrap();
    }

    // TAMPER: delete exactly the final newline byte.
    let bytes = std::fs::read(&path).unwrap();
    assert_eq!(*bytes.last().unwrap(), b'\n');
    std::fs::write(&path, &bytes[..bytes.len() - 1]).unwrap();

    // Reverting recover_chain_tail to an unconditional
    // truncate makes this open() succeed and silently drop entry 1 — the `Ok`
    // arm below then fires.
    match ReceiptLog::open(&path) {
        Err(CerudError::Receipt(msg)) => assert!(msg.contains("tamper"), "msg: {msg}"),
        Err(other) => panic!("expected a Receipt tamper error, got {other:?}"),
        Ok(_) => panic!("a sealed final entry with its newline deleted must be refused as tamper"),
    }
}

#[test]
fn sync_failure_rolls_back_the_append_and_the_retry_reuses_the_seq() {
    // If sync_data fails after write_all, the bytes must be
    // rolled back so on-disk state matches the un-advanced in-memory state and a
    // retry cleanly reuses the same seq (no duplicate-seq / broken chain).
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");
    let mut log = ReceiptLog::open(&path).unwrap();
    log.record_at("a", "inventory", "d0", ReceiptOutcome::Ok, 10)
        .unwrap(); // seq 0
    let len_before = std::fs::metadata(&path).unwrap().len();

    // Arm a durability-sync failure on the NEXT append (write succeeds, sync fails).
    log.fail_next_sync_for_test();
    let err = log
        .record_at("b", "inventory", "d1", ReceiptOutcome::Ok, 20)
        .unwrap_err(); // seq 1 attempt
    assert!(matches!(err, CerudError::Receipt(_)), "got {err:?}");

    // The appended bytes were rolled back — file length is unchanged.
    let len_after = std::fs::metadata(&path).unwrap().len();
    assert_eq!(
        len_after, len_before,
        "a failed sync must undo the appended bytes"
    );

    // The retry cleanly reuses seq 1 with a correct chain.
    let r = log
        .record_at("b2", "inventory", "d1b", ReceiptOutcome::Ok, 21)
        .unwrap();
    assert_eq!(r.seq, 1, "retry reuses the un-advanced seq");
    let entries = read_all(&path).unwrap();
    assert_eq!(entries.len(), 2);
    assert_eq!(entries[1].seq, 1);
    assert_eq!(entries[1].prev_hash, entries[0].entry_hash);
    verify_chain(&entries).unwrap();
}

#[test]
fn sync_parent_dir_is_best_effort_and_never_errs() {
    // The parent-dir fsync helper (called on create + rotate)
    // is best-effort — it returns Ok for a real dir AND for a missing parent
    // (warns internally, never fails the append).
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");
    std::fs::write(&path, b"x").unwrap();
    sync_parent_dir(&path).unwrap();
    sync_parent_dir(std::path::Path::new("/definitely/not/here/xyz.log")).unwrap();
}

// ─────────────────────────── size-based rotation ───────────────────────────

#[test]
fn rotation_shifts_files_and_keeps_the_chain_continuous() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    // Tiny cap forces rotation on nearly every record. A generous rotation
    // count RETAINS every segment (no pruning), so the concatenation is a full
    // genesis-rooted chain we can verify end to end.
    let mut log = ReceiptLog::open_with_limits(&path, 200, 20).unwrap();
    for i in 0..6u64 {
        log.record_at(
            "a",
            "inventory",
            "digestdigestdigest",
            ReceiptOutcome::Ok,
            i,
        )
        .unwrap();
    }
    drop(log);

    // At least one rotated file exists.
    assert!(
        rotated_path(&path, 1).exists(),
        "expected a rotated {}",
        rotated_path(&path, 1).display()
    );

    // Concatenate every retained segment oldest-first, then the current file,
    // and verify the whole chain is continuous across the rotation boundaries
    // (from genesis — no segment was pruned).
    let mut all = Vec::new();
    for i in (1..=20).rev() {
        let p = rotated_path(&path, i);
        if p.exists() {
            all.extend(read_all(&p).unwrap());
        }
    }
    all.extend(read_all(&path).unwrap());
    verify_chain(&all).unwrap();
    assert_eq!(all[0].prev_hash, GENESIS_HASH, "chain starts at genesis");

    // Rotation is itself receipted: at least one anchor entry exists, and each
    // links to the prior entry (tamper-evident across the boundary).
    let anchors: Vec<usize> = all
        .iter()
        .enumerate()
        .filter(|(_, r)| r.verb == ROTATION_VERB)
        .map(|(i, _)| i)
        .collect();
    assert!(
        !anchors.is_empty(),
        "rotation must be receipted via an anchor"
    );
    for pos in anchors {
        assert!(pos > 0, "an anchor is never the genesis entry");
        assert_eq!(all[pos].prev_hash, all[pos - 1].entry_hash);
    }
}

#[test]
fn rotation_prunes_old_segments_beyond_keep_last() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    // Small keep-last (2) + tiny cap + many records: old segments are pruned.
    let mut log = ReceiptLog::open_with_limits(&path, 200, 2).unwrap();
    for i in 0..10u64 {
        log.record_at(
            "a",
            "inventory",
            "digestdigestdigest",
            ReceiptOutcome::Ok,
            i,
        )
        .unwrap();
    }
    drop(log);

    // At most `max_rotations` (2) rotated files survive; the 3rd never persists.
    assert!(rotated_path(&path, 1).exists());
    assert!(rotated_path(&path, 2).exists());
    assert!(
        !rotated_path(&path, 3).exists(),
        "keep-last=2 must prune segment .3"
    );
    // Each surviving file is itself an intact (mid-chain) slice — every entry's
    // hash recomputes and links to its predecessor within the file. Uses the
    // per-entry helper (NOT `windows(2)`, which is blind to a single-entry
    // segment and never recomputes a segment's FIRST entry's self-hash).
    for i in 1..=2 {
        let entries = read_all(&rotated_path(&path, i)).unwrap();
        verify_midchain_slice(&entries).expect("surviving segment must be intact");
    }
    // ...and the surviving concatenation is continuous ACROSS the rotation
    // boundary (oldest-first): `.1[0].prev_hash` links to `.2[last].entry_hash`.
    let mut surviving = read_all(&rotated_path(&path, 2)).unwrap();
    surviving.extend(read_all(&rotated_path(&path, 1)).unwrap());
    verify_midchain_slice(&surviving).expect("surviving segments link across the boundary");
}

#[test]
fn a_corrupt_single_entry_rotated_segment_is_caught() {
    // REGRESSION for the `windows(2)` gap: a rotated segment holding exactly ONE
    // entry yields NOTHING under `windows(2)`, so a `windows(2)`-only integrity
    // check can never inspect (let alone reject) a corrupt single-entry segment.
    // Per-entry self-hash verification catches it.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    // Tiny cap → each entry rotates on the next record; generous keep-last → the
    // OLDEST segment (the pre-first-rotation `[entry0]`, a lone entry) is retained.
    let mut log = ReceiptLog::open_with_limits(&path, 80, 20).unwrap();
    for i in 0..5u64 {
        log.record_at(
            "a",
            "inventory",
            "digestdigestdigest",
            ReceiptOutcome::Ok,
            i,
        )
        .unwrap();
    }
    drop(log);

    // Locate a rotated segment with exactly ONE entry.
    let single = (1..=20)
        .map(|i| rotated_path(&path, i))
        .find(|p| p.exists() && read_all(p).unwrap().len() == 1)
        .expect("a single-entry rotated segment must exist");

    // It verifies clean BEFORE tampering (the per-entry check runs on 1 entry).
    let clean = read_all(&single).unwrap();
    assert_eq!(clean.len(), 1);
    verify_midchain_slice(&clean).expect("the intact single-entry segment verifies");

    // TAMPER: alter the single entry's caller in place WITHOUT recomputing its
    // hash — exactly the edit `windows(2)` cannot see for a 1-entry segment.
    let content = std::fs::read_to_string(&single).unwrap();
    let mut r: Receipt = serde_json::from_str(content.lines().next().unwrap()).unwrap();
    r.caller = "mallory".to_string();
    let mut line = serde_json::to_string(&r).unwrap();
    line.push('\n');
    std::fs::write(&single, line).unwrap();

    let tampered = read_all(&single).unwrap();
    assert_eq!(tampered.len(), 1);
    // DEMONSTRATE the gap a pairwise check leaves: `windows(2)` yields nothing for a 1-entry
    // segment, so a pairwise-only check would never even look at this tamper.
    assert_eq!(
        tampered.windows(2).count(),
        0,
        "windows(2) is structurally blind to a single-entry segment"
    );
    // The per-entry check CATCHES the corruption (at index 0).
    assert_eq!(
        verify_midchain_slice(&tampered),
        Err(0),
        "a corrupt single-entry segment must be caught by per-entry verification"
    );
}

// ─────────────────────────── receipt log I/O ───────────────────────────────

#[test]
fn log_appends_chain_and_reopen_recovers_tail() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("receipts.log");

    {
        let mut log = ReceiptLog::open(&path).unwrap();
        assert_eq!(log.next_seq(), 0);
        log.record_at(
            "alice",
            "inventory",
            &args_digest(&serde_json::json!({})),
            ReceiptOutcome::Ok,
            111,
        )
        .unwrap();
        log.record_at(
            "bob",
            "restart",
            &args_digest(&serde_json::json!({"graph": "x"})),
            ReceiptOutcome::Denied,
            222,
        )
        .unwrap();
        assert_eq!(log.next_seq(), 2);
    }

    // Reopen: the chain tail (seq + prev_hash) must be recovered so the NEXT
    // entry links correctly.
    let entries = read_all(&path).unwrap();
    assert_eq!(entries.len(), 2);
    verify_chain(&entries).unwrap();

    let mut log = ReceiptLog::open(&path).unwrap();
    assert_eq!(log.next_seq(), 2);
    log.record_at(
        "carol",
        "log-tail",
        &args_digest(&serde_json::json!({"name": "a.log"})),
        ReceiptOutcome::Error("verb_error".to_string()),
        333,
    )
    .unwrap();

    let entries = read_all(&path).unwrap();
    assert_eq!(entries.len(), 3);
    assert_eq!(entries[2].prev_hash, entries[1].entry_hash);
    verify_chain(&entries).unwrap();
}
