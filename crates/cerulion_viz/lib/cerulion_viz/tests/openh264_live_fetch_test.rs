// SPDX-License-Identifier: AGPL-3.0-only
//! The openh264 LIVE fetch, against Cisco's real CDN. **Needs network access.**
//!
//! Every other openh264-fetch arm is hermetic by construction: the pure halves
//! (`blob_source_for`, `verify_digest`, `decide_fetch_kick`, `fetch_enabled_from`,
//! the bzip2 decoder) are oracle-tested in `src/openh264_fetch.rs`, and the
//! pick-up path is driven through the real generation counter in
//! `video_decode_test.rs`. None of them opens a socket, which is deliberate: CI
//! has no business depending on a third party's CDN, and a test that silently
//! turned green because a proxy served something plausible would be worse than no
//! test.
//!
//! What CANNOT be proven hermetically is the claim the whole feature rests on:
//! **that the URL this code builds still serves bytes whose SHA-256 is the digest
//! this code pins.** Cisco could move the asset, re-cut the release, or change the
//! compression, and every hermetic arm would stay green while a fresh desk got
//! nothing. So that claim lives here, `#[ignore]`d, and is checked by hand.
//!
//! ```bash
//! cargo test -p cerulion_viz --test openh264_live_fetch_test -- --ignored --nocapture
//! ```
//!
//! # It will not touch your cache
//!
//! Both tests write only into a per-run temporary directory and never resolve the
//! shared cache path at all, so a desk that already has a working blob keeps it
//! byte-for-byte. Nothing here mutates process env — a set
//! `CERULION_OPENH264_BLOB` is a
//! REFUSAL rather than a redirect, since an operator-set override names a file we
//! must never overwrite.

#![cfg(unix)]

use std::time::Instant;

use cerulion_viz::openh264_fetch::{
    blob_source_for, download_from_cisco, fetch_blob_with, verify_loadable, FetchOutcome,
    CISCO_BLOB_SHA256,
};

/// SHA-256 of `bytes` as lowercase hex, computed INDEPENDENTLY of the fetcher.
///
/// The point is what it does NOT call. Asserting a fetched blob's digest with
/// `openh264_fetch::verify_digest` — the same function the fetch had just used to
/// decide whether to install it — routes the verdict through the code under test.
///
/// # What that independence is and is NOT worth (measured, both ways)
///
/// It does NOT, on its own, kill an always-`Ok` `verify_digest`: against the real
/// CDN the bytes really are correct, so a broken verifier and a correct one agree
/// and BOTH forms of this test pass. That broken verifier is not caught here; it is
/// killed by the hermetic arms in `openh264_fetch`'s own `mod tests`, which feed
/// bytes that genuinely do not match.
///
/// What it IS worth is the case this file exists for: the BYTES being wrong.
/// Cisco re-cutting a release, a CDN serving something else, or a typo in the
/// vendored table are all "the world moved", and the verdict on that must not
/// come from our own verifier. A
/// one-character change to the linux-arm64 digest fails the sweep below — on a
/// platform this machine is not, which is precisely what the host-only arm above
/// cannot see.
fn independent_sha256(bytes: &[u8]) -> String {
    use sha2::Digest as _;
    sha2::Sha256::digest(bytes)
        .iter()
        .fold(String::new(), |mut acc, b| {
            use std::fmt::Write as _;
            let _ = write!(acc, "{b:02x}");
            acc
        })
}

/// THE EXTERNAL-TRUTH ARM: Cisco still serves, at the URL we build, bytes that
/// decompress to exactly the library we pinned.
///
/// A failure here is NOT a code regression — it means the world moved, and the
/// remedy is to re-measure the digests (see `CISCO_BLOB_SHA256`'s provenance
/// note), never to relax the check.
#[test]
#[ignore = "box-only: reaches Cisco's CDN over the network"]
fn cisco_still_serves_the_blob_this_build_pins() {
    let (os, arch) = (std::env::consts::OS, std::env::consts::ARCH);
    let source = blob_source_for(os, arch).unwrap_or_else(|| {
        panic!(
            "{os}/{arch} is not a fetchable platform, so this machine cannot run this test; \
             the fetchable set is {:?}",
            CISCO_BLOB_SHA256
                .iter()
                .map(|(name, _)| *name)
                .collect::<Vec<_>>()
        )
    });
    println!("openh264 live fetch: {} -> {}", source.url, source.sha256);

    // A per-run scratch destination. The SHARED cache path is never resolved, so
    // a desk that already has a working blob is untouched by construction.
    let scratch = std::env::temp_dir().join(format!(
        "live-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("a clock after 1970")
            .as_nanos()
    ));
    std::fs::create_dir_all(&scratch).expect("create the scratch dir");
    let dest = scratch.join(source.filename);

    // `download_from_cisco` is the PRODUCTION client — same timeouts, same caps —
    // so this exercises the real network path, not a test double.
    let started = Instant::now();
    let outcome = fetch_blob_with(&source, &dest, false, &download_from_cisco);
    let elapsed = started.elapsed();

    let outcome = outcome.unwrap_or_else(|e| {
        panic!(
            "the live fetch failed: {e}\n\nIf this is a DIGEST mismatch, Cisco re-cut the \
             release: re-measure every entry in CISCO_BLOB_SHA256 (fetch, decompress, hash) \
             and update it. Do NOT relax the check — it is what keeps this desk running only \
             Cisco's own bits."
        )
    });
    assert_eq!(
        outcome,
        FetchOutcome::Fetched(dest.clone()),
        "a fresh scratch path must be a real FETCH, not a cache hit"
    );

    // The bytes on disk are the pinned release — hashed INDEPENDENTLY. The earlier
    // draft called `verify_digest`, the same function the fetch had just used to
    // decide whether to install, and claimed in its own comment to be independent
    // of it. It was not: an always-`Ok` verifier would install anything and pass
    // this line too.
    let installed = std::fs::read(&dest).expect("the fetched blob must be readable");
    assert_eq!(
        independent_sha256(&installed),
        source.sha256,
        "the installed bytes must be this platform's pinned Cisco release"
    );
    println!(
        "openh264 live fetch: {} bytes in {:.2?}",
        installed.len(),
        elapsed
    );

    // And it LOADS — which is what ultimately matters, and is the ONE claim only
    // a real blob on a real desk can settle. The loader re-checks the hash against
    // openh264-sys2's own list, so this also proves our pinned digest is one that
    // list accepts.
    //
    // UNCONDITIONAL. Gating this behind
    // `decoder-from-source` would print a note saying it had skipped — in the only
    // build this test ever runs under, so the dlopen half would be covered by nothing.
    // `from_blob_path` is gated on `libloading`, which this crate always enables,
    // so the symbol is there in both builds.
    verify_loadable(&dest).expect("the fetched blob must LOAD on this desk");

    // No temporary file survived a completed install.
    let leftovers: Vec<_> = std::fs::read_dir(&scratch)
        .expect("list the scratch dir")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.contains(".part-"))
        .collect();
    assert!(
        leftovers.is_empty(),
        "a stray temporary survived: {leftovers:?}"
    );

    // A SECOND call on the same path is a cache hit — PROVEN by a downloader that
    // panics if it is ever reached.
    let again = fetch_blob_with(&source, &dest, false, &|url: &str| {
        panic!("a cached blob must not be re-downloaded, but asked for {url}")
    });
    assert_eq!(
        again.expect("the second call must succeed"),
        FetchOutcome::AlreadyCached(dest.clone()),
        "a blob already on disk must short-circuit — this is what makes a working \
         desk's copy impossible to overwrite"
    );

    let _ = std::fs::remove_dir_all(&scratch);
}

/// **THE PRODUCTION COMPOSITION** — `fetch_blob` end to end, exactly as the
/// background thread calls it.
///
/// The arm above drives `fetch_blob_with` + `verify_loadable` as two statements,
/// which REPLICATES the composition instead of exercising it. `fetch_blob` itself
/// had zero test callers, so its `verify_loadable(..)?` — whose whole
/// point is that the success line must not claim "video now
/// decodes here" without a load — was deletable with a fully green suite.
///
/// This drives the REAL `fetch_blob`: platform resolution, cache-path resolution,
/// the override check, download, decompress, digest, atomic install and the load
/// verification, in the order the background thread runs them.
///
/// SCOPE: `fetch_blob` does NOT bump the cache generation and does NOT touch
/// `settled` — those live one layer up in `report_attempt`, covered hermetically by
/// `a_reported_attempt_announces_success_and_settles_only_when_it_should`.
///
/// `HOME` is redirected to a scratch dir so `cisco_blob_path()` resolves inside
/// it: the desk's own cached blob is never read, written, or consulted — and BOTH
/// fetches happen inside that window, since a call made after the restore would
/// resolve the real cache. The redirect is process-global, so this test takes the
/// file's env lock.
///
/// Deleting `verify_loadable(..)?` from
/// `fetch_blob` fails this.
#[test]
#[ignore = "box-only: reaches Cisco's CDN over the network"]
fn the_production_fetch_path_composes_end_to_end() {
    use cerulion_viz::openh264_fetch::{fetch_blob, FetchOutcome};

    let _env = env_lock();
    let (os, arch) = (std::env::consts::OS, std::env::consts::ARCH);
    if blob_source_for(os, arch).is_none() {
        println!("SKIP: {os}/{arch} is not a fetchable platform");
        return;
    }

    let scratch = std::env::temp_dir().join(format!(
        "prod-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("a clock after 1970")
            .as_nanos()
    ));
    std::fs::create_dir_all(&scratch).expect("create the scratch HOME");

    let restore_home = std::env::var_os("HOME");
    let restore_override = std::env::var_os("CERULION_OPENH264_BLOB");
    // SAFETY: this file's env lock is held; both are restored before any assert.
    unsafe {
        std::env::set_var("HOME", &scratch);
        // An override would make `fetch_blob` refuse (OverrideNotOurs) instead of
        // exercising the cache path, so it must be absent for this arm.
        std::env::remove_var("CERULION_OPENH264_BLOB");
    }

    // BOTH calls happen while HOME is redirected. A call made after the restore
    // would resolve the desk's real cache path — the one thing this test must
    // never touch.
    let first = fetch_blob(os, arch);
    let second = fetch_blob(os, arch);

    unsafe {
        match &restore_home {
            Some(v) => std::env::set_var("HOME", v),
            None => std::env::remove_var("HOME"),
        }
        if let Some(v) = &restore_override {
            std::env::set_var("CERULION_OPENH264_BLOB", v);
        }
    }

    let first = first.expect("the production fetch must succeed on a machine that reaches the CDN");
    let installed = match &first {
        FetchOutcome::Fetched(p) => p.clone(),
        FetchOutcome::AlreadyCached(p) => {
            panic!(
                "a fresh scratch HOME cannot be a cache hit, got {}",
                p.display()
            )
        }
    };
    assert!(
        installed.starts_with(&scratch),
        "the fetch must land under the redirected HOME, not the desk's cache: {}",
        installed.display()
    );

    // (1) The blob really LOADS. `fetch_blob` ran `verify_loadable` internally —
    // that is the line this test exists to make undeletable — and it is asserted
    // again here so the claim does not rest on the code under test.
    verify_loadable(&installed).expect("the production path must install a loadable blob");

    // (2) The digest, computed independently of the fetcher.
    let source = blob_source_for(os, arch).expect("fetchable");
    let bytes = std::fs::read(&installed).expect("read the installed blob");
    assert_eq!(independent_sha256(&bytes), source.sha256);

    // (3) The SECOND call short-circuits — the cache hit reached through the
    // production entry, not just through `fetch_blob_with`.
    match second.expect("the second production call must succeed") {
        FetchOutcome::AlreadyCached(p) => assert_eq!(p, installed),
        other => panic!("a cached blob must short-circuit on the production entry, got {other:?}"),
    }

    println!(
        "openh264 production composition: {} bytes installed under a redirected HOME, \
         second call was a cache hit",
        bytes.len()
    );
    let _ = std::fs::remove_dir_all(&scratch);
}

/// `HOME` and `CERULION_OPENH264_BLOB` are process-global; serialise the tests
/// that touch them.
fn env_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// EVERY vendored digest, not just this host's.
///
/// The arm above checks one row — whichever platform you happen to run it on —
/// so a one-character typo in another platform's digest ships and surfaces as a
/// permanently-failing fetch on a machine nobody ran this on. Downloading and
/// hashing does NOT require running on the target platform, so the sweep is four
/// GETs instead of one and turns a 1-of-4 external check into 4-of-4.
///
/// (The `dlopen` half above stays host-only for the obvious reason.)
#[test]
#[ignore = "box-only: reaches Cisco's CDN over the network"]
fn every_vendored_digest_matches_what_cisco_serves() {
    use cerulion_viz::openh264_fetch::{blob_url, CISCO_BLOB_SHA256};

    let mut checked = 0usize;
    for (filename, expected) in CISCO_BLOB_SHA256 {
        let url = blob_url(filename);
        let response = ureq::get(&url)
            .call()
            .unwrap_or_else(|e| panic!("GET {url} failed: {e}"));
        let mut compressed = Vec::new();
        std::io::Read::read_to_end(&mut response.into_body().into_reader(), &mut compressed)
            .unwrap_or_else(|e| panic!("reading {url} failed: {e}"));

        let mut library = Vec::new();
        std::io::Read::read_to_end(
            &mut bzip2_rs::DecoderReader::new(&compressed[..]),
            &mut library,
        )
        .unwrap_or_else(|e| panic!("decompressing {filename} failed: {e}"));

        // INDEPENDENT of the fetcher's verifier — this arm exists to check the
        // vendored table against an EXTERNAL truth, so routing it through the
        // code under test would defeat its whole purpose.
        assert_eq!(
            independent_sha256(&library),
            *expected,
            "the vendored digest for {filename} does NOT match what Cisco serves \
             ({} compressed -> {} bytes). Re-measure the whole table; do not \
             re-bless a single row from the actual value.",
            compressed.len(),
            library.len()
        );
        println!(
            "openh264 digest sweep: {filename} OK ({} bytes)",
            library.len()
        );
        checked += 1;
    }
    assert_eq!(
        checked,
        CISCO_BLOB_SHA256.len(),
        "every row must have been checked"
    );
    assert!(checked >= 4, "the table must not have silently shrunk");
}
