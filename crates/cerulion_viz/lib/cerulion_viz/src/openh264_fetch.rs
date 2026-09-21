// SPDX-License-Identifier: AGPL-3.0-only
//! The silent, checksum-pinned, cached background fetch of Cisco's
//! OpenH264 binary — the half [`crate::video_decode`] has been documenting as
//! "until the background fetch lands" while every fresh desk rode the 567 ms viewer-decodes
//! fallback.
//!
//! # The decision, and the legal shape that constrains it
//!
//! The request (verbatim): *"can we use cisco's binary but package it into
//! cerulion studio so that a user doesnt have to see a download/install bar?"*
//!
//! The second half is what this module delivers; the first half it deliberately
//! does NOT. Cisco's AVC patent grant covers only the binaries **Cisco**
//! distributes — bundling one into our installer makes US the distributor and
//! voids the grant, which is exactly why Firefox downloads the OpenH264 plugin
//! from Cisco's CDN on first run rather than shipping it. So the binary is
//! fetched from Cisco at runtime and never vendored, and that posture is not a
//! detail to be optimised away later.
//!
//! What the user sees is nothing at all: the fetch runs on a background thread on
//! FIRST NEED (the first access unit that finds no decoder), the decoders pick the
//! blob up on their next unit, and the only UI-visible consequence is that video
//! stops being slow. A FAILURE is loud — latched, never per-frame — because a desk
//! that is offline at first camera use keeps the earlier viewer-decodes path
//! and should be told why.
//!
//! # What is verified, and against what
//!
//! Two independent checks, deliberately not one:
//!
//! * **This module** verifies the decompressed bytes against a VENDORED
//!   per-platform SHA-256 ([`CISCO_BLOB_SHA256`]) BEFORE anything reaches the
//!   cache path. A digest mismatch writes nothing at all.
//! * **The loader** ([`openh264::OpenH264API::from_blob_path`]) re-verifies at
//!   load against `openh264-sys2`'s own embedded hash list.
//!
//! The vendored digests ARE that same list (see [`CISCO_BLOB_SHA256`]), so the
//! two agree by construction — but they answer different questions. The loader
//! asks "is this file SOME Cisco release?", which a wrong-architecture blob
//! passes; this module asks "is this file the release for THIS platform?", which
//! it does not. Without the second question a wrong-arch download would be cached,
//! refused at `dlopen`, and leave the desk in the "present but could NOT be
//! loaded" state whose only remedy is a human deleting the file.
//!
//! # Never overwrite a good blob
//!
//! [`fetch_blob`] SHORT-CIRCUITS on a cache hit: if the destination already holds
//! bytes matching the expected digest it returns [`FetchOutcome::AlreadyCached`]
//! without opening a socket. Only a MISSING or CORRUPT file is replaced, and only
//! ever by bytes that already passed the digest — the write goes to a temporary
//! sibling and is `rename`d over the destination, so a killed fetch can never
//! leave a half-file where a decoder will look.
//!
//! # Bounded, and never a busy loop
//!
//! `decode()` runs at frame rate, so "kick the fetch when there is no decoder"
//! must not mean "kick it 30 times a second". [`decide_fetch_kick`] is the pure
//! gate: one attempt in flight at a time, a [`RETRY_BACKOFF`] floor between
//! failures, and a TERMINAL `Settled` state once the blob is on disk or the
//! platform is one Cisco publishes nothing for. Downloads are size-capped on both
//! the compressed and decompressed sides so a hostile or broken response cannot
//! grow the desk's memory.
//!
//! ## What that bounds, and what it does not — stated precisely
//!
//! The floor bounds the RATE. It does not bound the TOTAL, and the two are
//! different claims: the terminal set is deliberately narrow, so `DigestMismatch`,
//! `TooLarge`, `Decompress`, `Write` and `Download` all retry — and because
//! nothing is written on those arms, the cache short-circuit misses and **every
//! attempt re-transfers the asset in full**.
//!
//! The per-attempt cost is therefore worth naming, because the "one attempt a
//! minute" figure is calibrated on the OFFLINE case, which is the cheap one (the
//! connect fails, so ~zero bytes cross):
//!
//! | condition | bytes per attempt |
//! |---|---|
//! | offline / DNS / refused | ~0 (the connect fails) |
//! | `DigestMismatch`, `Write`, `Decompress` | the full asset (634 KB largest, measured) |
//! | `TooLarge` | up to [`MAX_COMPRESSED_BYTES`] + 1 (8 MiB) — the reader takes the cap before refusing |
//!
//! [`backoff_for`] is what bounds the total: a repeat-IDENTICAL error doubles the
//! wait up to [`MAX_RETRY_BACKOFF`], so a permanently-broken desk settles at one
//! transfer an hour rather than sixty. A DIFFERENT error resets to the floor.
//!
//! "Identical" means the rendered [`FetchError`] message, so two shapes hold the
//! 60 s floor indefinitely: a `DigestMismatch` whose `actual` digest VARIES from
//! attempt to attempt, and any ALTERNATING pair of errors. Both are worth naming
//! rather than hiding — in each the cost equals the pre-escalation behaviour, not
//! worse, and the realistic permanent cases (a stale CDN edge serving the same
//! wrong bytes, a persistent ENOSPC) do escalate. Keying on the message is what
//! makes "the condition changed" mean something; keying on the variant would make
//! a recovering desk wait an hour for a condition that had already cleared.
//!
//! Nothing bad is ever installed on any of those arms — the digest gate precedes
//! `install_blob` — and the regime is loudly re-announced at each decade of
//! failures. `FailureRegimeLatch` starts at `next_decade = 10`, so the first
//! re-announcement lands at the 10th failure (~10 minutes at the floor, later once
//! the backoff escalates), not at the 100th.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

use crate::video_decode::{cisco_blob_filename_for, cisco_blob_path, CISCO_BLOB_VERSION};

/// Cisco's own distribution endpoint for the released OpenH264 binaries.
///
/// This host is load-bearing, not a mirror choice: the patent grant follows the
/// DISTRIBUTOR, so the bytes have to come from Cisco. (The `cisco/openh264`
/// GitHub release for v2.6.0 carries no assets at all — MEASURED — so this CDN is
/// not merely the canonical source, it is the only one.)
///
/// Every asset is bzip2-compressed; the `.bz2` suffix is appended by
/// [`blob_url`]. There is no uncompressed variant (a request for one answers
/// HTTP 403 — also measured).
pub const CISCO_CDN_BASE: &str = "https://ciscobinary.openh264.org/";

/// The expected SHA-256 of each platform's DECOMPRESSED library, keyed by Cisco's
/// own filename (the key [`cisco_blob_filename_for`] produces).
///
/// # Provenance
///
/// These are `openh264-sys2` 0.9.7's `src/blobs/hashes.txt` — the exact list
/// [`openh264::OpenH264API::from_blob_path`] checks a file against before it will
/// load it. Taking them from there rather than from a release note means the
/// fetcher cannot pin a digest the loader would then reject.
///
/// Every entry was independently MEASURED (2026-07-28) by fetching
/// `{CISCO_CDN_BASE}{name}.bz2`, decompressing, and hashing — they matched the
/// embedded list exactly. The mac-arm64 row is additionally the digest of the
/// blob a live desk was already running with, which the loader accepts.
///
/// A platform ABSENT from this table can still run a hand-placed blob through
/// `CERULION_OPENH264_BLOB`; it just gets no automatic fetch, because shipping a
/// digest nobody verified would make the strongest check here a guess.
pub const CISCO_BLOB_SHA256: &[(&str, &str)] = &[
    (
        "libopenh264-2.6.0-mac-arm64.dylib",
        "052e98bfcf7a9167d22f3bbb3f5988ef79065591f36af8b52924b22b13624551",
    ),
    (
        "libopenh264-2.6.0-mac-x64.dylib",
        "e3dc8bc01fe69363f61fd3c02fd27798537a585eadd38cd808f303d1ee505a19",
    ),
    (
        "libopenh264-2.6.0-linux64.8.so",
        "2f0cde7c6a6abcf5cae76942894ea42897fa677bce4ed6c91a24dd1b041d5f04",
    ),
    (
        "libopenh264-2.6.0-linux-arm64.8.so",
        "12e7b33623667cdab0e575170c147b1b36eadb77d0d2aa7ceb5afd3e58902140",
    ),
];

/// Ceiling on the COMPRESSED response. Cisco's largest 2.6.0 asset is 634 KB;
/// this is an order of magnitude of headroom and still refuses a response that
/// would grow without bound.
pub const MAX_COMPRESSED_BYTES: usize = 8 * 1024 * 1024;

/// Ceiling on the DECOMPRESSED library. The largest 2.6.0 library is 1.73 MB.
/// Separate from the compressed cap because bzip2 is a decompression bomb vector:
/// a small response can expand without limit.
pub const MAX_DECOMPRESSED_BYTES: usize = 32 * 1024 * 1024;

/// Cisco's LARGEST 2.6.0 asset, compressed — MEASURED 2026-07-28
/// (`libopenh264-2.6.0-linux64.8.so.bz2`). Named so the ceilings above are
/// visibly headroom over a real number rather than round guesses.
const LARGEST_CISCO_ASSET_BYTES: usize = 634_264;
/// The same library DECOMPRESSED (1.73 MB), also measured.
const LARGEST_CISCO_LIBRARY_BYTES: usize = 1_731_128;

// COMPILE-TIME drift guards. A ceiling trimmed to a hair above today's asset
// would refuse the NEXT release with a size error rather than a useful one, and
// a decompressed cap below the compressed one could not admit any expansion at
// all — which is the entire job of the second limit.
const _: () = assert!(MAX_COMPRESSED_BYTES > LARGEST_CISCO_ASSET_BYTES * 4);
const _: () = assert!(MAX_DECOMPRESSED_BYTES > LARGEST_CISCO_LIBRARY_BYTES * 4);
const _: () = assert!(MAX_DECOMPRESSED_BYTES > MAX_COMPRESSED_BYTES);

/// The floor between two failed attempts.
///
/// `decode()` runs at frame rate, so without this a desk with no network would
/// retry ~30 times a second forever. One minute keeps a desk that regains its
/// network decoding within a minute of the next camera frame, at a cost of one
/// attempt per minute while it does not.
pub const RETRY_BACKOFF: Duration = Duration::from_secs(60);

/// Ceiling on the escalated backoff — see [`backoff_for`].
pub const MAX_RETRY_BACKOFF: Duration = Duration::from_secs(3600);

/// How long to wait before the next attempt, given how many times the SAME error
/// has repeated. PURE (oracle-tested).
///
/// # Why the flat floor is not enough
///
/// The terminal set is deliberately narrow — `UnsupportedPlatform`,
/// `NoCacheLocation`, `OverrideNotOurs` — so `DigestMismatch`, `TooLarge`,
/// `Decompress`, `Write` and `Download` all retry. That is correct: each of them
/// CAN be transient (a stale CDN edge, a caching proxy, a full disk someone
/// clears), and settling on `DigestMismatch` in particular would turn a
/// recoverable edge-cache problem into exactly the silent permanent failure this
/// module's docs forbid.
///
/// But some of them are permanent in practice, and nothing is written on those
/// arms, so the cache short-circuit misses and EVERY attempt re-transfers in full.
/// At the flat floor that is ~38 MB/h for the largest asset, and up to ~480 MiB/h
/// in the `TooLarge` case (the reader takes `MAX_COMPRESSED_BYTES + 1` before
/// refusing). The rate was bounded; the total was not.
///
/// So a REPEAT-IDENTICAL error doubles the wait, capped at
/// [`MAX_RETRY_BACKOFF`]: ~7 attempts to reach the ceiling, after which a
/// permanently-broken desk costs one transfer an hour instead of sixty. A
/// DIFFERENT error resets to the floor — the condition changed, so the evidence
/// that it was permanent is gone.
pub fn backoff_for(repeats: u32) -> Duration {
    RETRY_BACKOFF
        .checked_mul(1u32.checked_shl(repeats.min(16)).unwrap_or(u32::MAX))
        .unwrap_or(MAX_RETRY_BACKOFF)
        .min(MAX_RETRY_BACKOFF)
}

/// Where a fetch would get this platform's blob, and what it must hash to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlobSource {
    /// Cisco's own filename — also the cache filename, so the two agree.
    pub filename: &'static str,
    /// The fully-qualified `.bz2` URL on Cisco's CDN.
    pub url: String,
    /// Expected SHA-256 of the DECOMPRESSED library, lowercase hex.
    pub sha256: &'static str,
}

/// The PURE plan: what this `(os, arch)` needs fetched, or `None` when Cisco
/// publishes nothing for it or nobody has verified a digest for it.
pub fn blob_source_for(os: &str, arch: &str) -> Option<BlobSource> {
    let filename = cisco_blob_filename_for(os, arch)?;
    let sha256 = sha256_for(filename)?;
    Some(BlobSource {
        filename,
        url: blob_url(filename),
        sha256,
    })
}

/// The `.bz2` URL Cisco serves `filename` at. PURE.
pub fn blob_url(filename: &str) -> String {
    format!("{CISCO_CDN_BASE}{filename}.bz2")
}

/// The vendored digest for one of Cisco's filenames. PURE.
pub fn sha256_for(filename: &str) -> Option<&'static str> {
    CISCO_BLOB_SHA256
        .iter()
        .find(|(name, _)| *name == filename)
        .map(|(_, sha)| *sha)
}

/// Why a fetch could not produce a usable blob. Every arm names something an
/// operator can act on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FetchError {
    /// Cisco publishes no binary for this platform, or no digest has been
    /// verified for it. TERMINAL — retrying cannot change the answer.
    UnsupportedPlatform { os: String, arch: String },
    /// There is nowhere to cache a blob (no `HOME`, no override). TERMINAL.
    NoCacheLocation,
    /// The download failed (offline, DNS, TLS, a non-200). Retryable.
    Download(String),
    /// The response exceeded [`MAX_COMPRESSED_BYTES`] or the library exceeded
    /// [`MAX_DECOMPRESSED_BYTES`].
    TooLarge { limit: usize, stage: &'static str },
    /// The response was not valid bzip2.
    Decompress(String),
    /// The bytes are not this platform's Cisco release. NOTHING was written.
    DigestMismatch { expected: String, actual: String },
    /// The verified bytes could not be placed in the cache.
    Write(String),
    /// The cache path came from the operator's own override and does not hold
    /// this platform's release. Refused rather than overwritten. TERMINAL.
    OverrideNotOurs { path: PathBuf },
    /// The blob is installed and its bytes ARE Cisco's, but this desk cannot
    /// `dlopen` it (hardened-runtime library validation, a `noexec` mount,
    /// missing runtime deps).
    InstalledButUnloadable { path: PathBuf, why: String },
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FetchError::UnsupportedPlatform { os, arch } => write!(
                f,
                "Cisco publishes no verified OpenH264 {CISCO_BLOB_VERSION} binary for {os}/{arch}"
            ),
            FetchError::NoCacheLocation => write!(
                f,
                "no cache location for the OpenH264 binary (no HOME, and no \
                 CERULION_OPENH264_BLOB override)"
            ),
            FetchError::Download(why) => write!(f, "download failed: {why}"),
            FetchError::TooLarge { limit, stage } => {
                write!(f, "the {stage} exceeded its {limit}-byte ceiling")
            }
            FetchError::Decompress(why) => write!(f, "the response was not valid bzip2: {why}"),
            FetchError::DigestMismatch { expected, actual } => write!(
                f,
                "the downloaded bytes are NOT this platform's Cisco release \
                 (expected sha256 {expected}, got {actual}) — nothing was cached"
            ),
            FetchError::Write(why) => write!(f, "could not write the verified blob: {why}"),
            FetchError::OverrideNotOurs { path } => write!(
                f,
                "{} is set to {}, which does not hold this platform's Cisco release — \
                 refusing to overwrite a file you told us you manage. Point it at a \
                 correct copy, or unset it to let the fetch use the shared cache.",
                crate::video_decode::BLOB_PATH_ENV,
                path.display()
            ),
            FetchError::InstalledButUnloadable { path, why } => write!(
                f,
                "the OpenH264 binary at {} is Cisco's own release but this desk cannot load \
                 it ({why}) — on macOS a signed app with library validation refuses a dylib \
                 it did not sign; a `noexec` mount does the same",
                path.display()
            ),
        }
    }
}

/// What one fetch attempt produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FetchOutcome {
    /// The destination already held the right bytes — no socket was opened.
    AlreadyCached(PathBuf),
    /// Downloaded, verified, and installed at this path.
    Fetched(PathBuf),
}

/// Verify `bytes` against `expected_hex`. PURE — the one check that stands
/// between the network and the cache path.
pub fn verify_digest(bytes: &[u8], expected_hex: &str) -> Result<(), FetchError> {
    use sha2::Digest as _;
    let actual = sha2::Sha256::digest(bytes);
    let actual_hex = actual.iter().fold(String::new(), |mut acc, b| {
        use std::fmt::Write as _;
        let _ = write!(acc, "{b:02x}");
        acc
    });
    if actual_hex == expected_hex {
        Ok(())
    } else {
        Err(FetchError::DigestMismatch {
            expected: expected_hex.to_string(),
            actual: actual_hex,
        })
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The kick gate — pure, because `decode()` calls it at frame rate
// ────────────────────────────────────────────────────────────────────────────

/// Operator kill switch: set to `off` to keep this desk from ever reaching
/// Cisco's CDN.
///
/// The desk then keeps the viewer-decodes path (slow, but rendering) unless
/// [`crate::video_decode::BLOB_PATH_ENV`] points at a copy of the binary — which
/// is the air-gapped install's answer: fetch once, distribute internally, point
/// every desk at it.
pub const FETCH_ENV: &str = "CERULION_OPENH264_FETCH";

/// Whether this build and this desk should fetch at all.
///
/// Two independent reasons not to, and neither is a test hook:
///
/// * A `decoder-from-source` build has the decoder COMPILED IN, so
///   `resolve_backend` never asks for a blob and downloading one would be pure
///   cost. (That this also makes the crate's own test suite hermetic is a
///   consequence, not the motivation — the alternative, `#[cfg]`-ing the fetch
///   body out under test, would ship a function no test had executed.)
/// * The operator said no.
pub fn fetch_enabled() -> bool {
    fetch_enabled_from(
        cfg!(feature = "decoder-from-source"),
        std::env::var(FETCH_ENV).ok().as_deref(),
    )
}

/// The PURE half of [`fetch_enabled`] (oracle-tested).
///
/// Split out because this crate's own tests build WITH `decoder-from-source`, so
/// the shipped `fetch_enabled()` answers `false` there for a reason that has
/// nothing to do with the env — which would make any env assertion on it vacuous.
/// Over this function both inputs are reachable from one machine.
pub fn fetch_enabled_from(decoder_compiled_in: bool, env: Option<&str>) -> bool {
    if decoder_compiled_in {
        return false;
    }
    env != Some("off")
}

/// What the caller should do about a "there is no decoder" observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetchGate {
    /// Start a fetch now.
    Start,
    /// Fetching is off for this build or this desk ([`fetch_enabled`]).
    Disabled,
    /// One is already in flight.
    AlreadyRunning,
    /// The last attempt failed too recently. Carries how long is left, so a log
    /// can say so rather than being silent.
    BackingOff(Duration),
    /// Nothing more to do, ever: the blob is cached, or this platform has none.
    Settled,
}

/// The observable state of the process-wide fetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FetchStatus {
    /// An attempt is in flight.
    pub running: bool,
    /// No further attempt will be made (cached, or terminally unsupported).
    pub settled: bool,
    /// Attempts that have failed, ever. Never reset — Principle #3.
    pub failures: u64,
}

/// The PURE gate (oracle-tested): given the current state and how long ago the
/// last failure was, should a fetch start?
///
/// `enabled` then `settled` win outright — including over `running`, which cannot
/// be true alongside them in practice but must not resolve to `Start` if it ever
/// were. A desk with no network is the case that motivates the backoff: without
/// it, a 30 Hz camera would open a socket 30 times a second forever.
pub fn decide_fetch_kick(
    enabled: bool,
    settled: bool,
    running: bool,
    since_last_failure: Option<Duration>,
    backoff: Duration,
) -> FetchGate {
    if !enabled {
        return FetchGate::Disabled;
    }
    if settled {
        return FetchGate::Settled;
    }
    if running {
        return FetchGate::AlreadyRunning;
    }
    match since_last_failure {
        Some(elapsed) if elapsed < backoff => FetchGate::BackingOff(backoff - elapsed),
        _ => FetchGate::Start,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The IO shell
// ────────────────────────────────────────────────────────────────────────────

/// Bumped every time the cache gains a usable blob, so a decoder pool can notice
/// without polling the filesystem at frame rate.
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// Every `note_decoder_needed` call, gate verdict or not. See [`consultations`].
static CONSULTATIONS: AtomicU64 = AtomicU64::new(0);

/// One `warn!` the first time [`STATE`] is found poisoned.
///
/// Recovering from a poisoned DIAGNOSTIC mutex is right — it must never wedge the
/// video path — but doing so in total silence is not: a poison means a thread died
/// holding it, possibly mid-update (`failures` bumped, `last_failure` not), and
/// every reading below it is then suspect. Once per process, because the
/// condition is sticky and this must not become its own flood.
static POISON_REPORTED: std::sync::Once = std::sync::Once::new();

/// Lock [`STATE`], recovering from (and REPORTING) a poisoning.
fn lock_state() -> std::sync::MutexGuard<'static, FetcherState> {
    match STATE.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            POISON_REPORTED.call_once(|| {
                tracing::warn!(
                    "cerulion_viz: the OpenH264 fetcher's state mutex was POISONED — a fetch \
                     thread died holding it, so its counters may be mid-update. Recovering \
                     (this must never wedge video), but the fetch state below is suspect."
                );
            });
            poisoned.into_inner()
        }
    }
}

/// The process-wide fetch state. A `Mutex` rather than atomics because the
/// decision reads three fields together and must not act on a torn view.
static STATE: Mutex<FetcherState> = Mutex::new(FetcherState::new());

struct FetcherState {
    running: bool,
    settled: bool,
    failures: u64,
    last_failure: Option<Instant>,
    /// The last error's rendered identity, and how many times it has repeated
    /// CONSECUTIVELY — the input to [`backoff_for`]. Rendered rather than the
    /// enum so two `Download` failures with different causes count as different.
    last_error: Option<String>,
    repeats: u32,
    latch: FailureRegimeLatch,
}

impl FetcherState {
    const fn new() -> Self {
        Self {
            running: false,
            settled: false,
            failures: 0,
            last_failure: None,
            last_error: None,
            repeats: 0,
            latch: FailureRegimeLatch::new(),
        }
    }
}

/// The generation counter — a pool re-resolves its backend when this MOVES.
///
/// A counter rather than a flag because a pool built AFTER a successful fetch
/// must not be told "something changed" it already has; comparing against the
/// value it recorded at construction answers that correctly.
pub fn cache_generation() -> u64 {
    GENERATION.load(Ordering::Acquire)
}

/// Announce a cache change WITHOUT running a fetch — the seam that makes the
/// pick-up path testable.
///
/// A test build compiles the decoder in, so no fetch ever succeeds there and the
/// generation would never move; without this the one thing the fetch buys (a
/// running vizd starting to decode locally, with no restart) would ship
/// unexercised. It bumps the SAME counter production bumps and the pool reads it
/// through the SAME accessor, so what it drives is the real path.
#[cfg(any(test, feature = "test-helpers"))]
#[doc(hidden)]
pub fn bump_cache_generation_for_test() -> u64 {
    GENERATION.fetch_add(1, Ordering::AcqRel) + 1
}

/// How many times a caller has reported "a decoder was wanted and there was
/// none" — bumped on EVERY [`note_decoder_needed`] call, whatever the gate then
/// decides.
///
/// Unconditional on purpose (Principle #3, and the same reason the failure
/// counter is): it is the ONE observable that the decode path is actually WIRED
/// to the fetcher. Every other signal here is downstream of the gate, so in a
/// build where the gate answers [`FetchGate::Disabled`] — which is every build of
/// this crate's own test suite — deleting the call site would change nothing any
/// of them can see, and the feature would ship on a code path no test executed.
pub fn consultations() -> u64 {
    CONSULTATIONS.load(Ordering::Acquire)
}

/// Observable fetch state (Principle #3 / tests).
pub fn status() -> FetchStatus {
    let s = lock_state();
    FetchStatus {
        running: s.running,
        settled: s.settled,
        failures: s.failures,
    }
}

/// Tell the fetcher a decoder was WANTED and none was available.
///
/// Idempotent and cheap: called from `decode()` at frame rate, it takes one
/// uncontended lock and returns. At most one fetch is ever in flight, a failed
/// attempt is not retried for [`RETRY_BACKOFF`], and a cached (or terminally
/// unsupported) platform settles permanently.
///
/// Returns the gate's verdict so a caller can log or assert on it.
pub fn note_decoder_needed() -> FetchGate {
    CONSULTATIONS.fetch_add(1, Ordering::AcqRel);
    let mut state = lock_state();
    let gate = decide_fetch_kick(
        fetch_enabled(),
        state.settled,
        state.running,
        state.last_failure.map(|t| t.elapsed()),
        backoff_for(state.repeats),
    );
    if gate != FetchGate::Start {
        return gate;
    }
    state.running = true;
    drop(state);

    // A DETACHED thread: the video path must never wait on a network round trip,
    // and there is nothing to join — the result is published through
    // `GENERATION`, which the decoder pool reads on its next access unit.
    let spawned = std::thread::Builder::new()
        .name("openh264-fetch".to_string())
        .spawn(run_one_attempt);
    if let Err(e) = spawned {
        // The thread could not START. Record it as a failure so the backoff
        // applies and this does not turn into a per-frame spawn storm.
        let mut state = match STATE.lock() {
            Ok(s) => s,
            Err(poisoned) => poisoned.into_inner(),
        };
        state.running = false;
        record_failure(&mut state, &FetchError::Download(format!("{e}")));
        // The attempt never began, so reporting `Start` would assert something
        // that did not happen. The failure was recorded, so the backoff applies.
        return FetchGate::BackingOff(backoff_for(state.repeats));
    }
    FetchGate::Start
}

/// Clears `running` when the attempt ends — INCLUDING on a panic.
///
/// `running` is what stops a second fetch starting, so a thread that dies without
/// clearing it wedges the fetcher for the life of the process: `decide_fetch_kick`
/// answers `AlreadyRunning` forever, no further attempt is ever made, and the desk
/// keeps the slow path with NO further log line — a silent permanent failure,
/// which is the one outcome this module must not have. The work happens in
/// third-party code (TLS, HTTP, bzip2), so "it cannot panic" is not something this
/// module gets to assume.
///
/// It is DISARMED before the normal path takes the lock, never dropped while the
/// lock is held — `STATE` is a plain `Mutex`, so a `Drop` that re-locked it from
/// inside the reporting section would deadlock the fetch thread rather than
/// protect it.
struct RunningGuard {
    armed: bool,
}

impl RunningGuard {
    /// The attempt reached its reporting section, which clears `running` itself.
    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for RunningGuard {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let mut state = match STATE.lock() {
            Ok(s) => s,
            Err(poisoned) => poisoned.into_inner(),
        };
        state.running = false;
        // Recorded as a FAILURE so the backoff applies. Without that, a reliably
        // panicking fetch would be retried on the very next access unit, at frame
        // rate — trading a silent wedge for a loud storm.
        let e = FetchError::Download(
            "the fetch thread ended without reporting an outcome (it panicked)".to_string(),
        );
        record_failure(&mut state, &e);
    }
}

/// One attempt, start to finish, with all of its reporting.
fn run_one_attempt() {
    let mut guard = RunningGuard { armed: true };
    let result = fetch_blob(std::env::consts::OS, std::env::consts::ARCH);
    // Past the work; the reporting section below owns the flag from here.
    guard.disarm();
    report_attempt(result);
}

/// The REPORTING half of an attempt, over an already-computed outcome.
///
/// Split from [`run_one_attempt`] so the three decisions that live here are
/// reachable from a hermetic test: the `GENERATION` bump (the entire
/// no-restart payoff), the `settled` writes on both the success and terminal-error
/// arms (the contract `decide_fetch_kick`'s oracles assume), and the failure
/// recording. Before the split, `run_one_attempt` had exactly one caller — a
/// `.spawn()` — and no test could reach any of it, so all three were deletable
/// with a fully green suite.
///
/// This is the same seam shape the netd ladder (`resolve_netd_bin_for`) and the
/// fetch decision (`fetch_blob_with`) already use: the caller supplies the effect,
/// the decision stays testable.
fn report_attempt(result: Result<FetchOutcome, FetchError>) {
    let mut state = lock_state();
    state.running = false;
    match result {
        Ok(outcome) => {
            let path = match &outcome {
                FetchOutcome::AlreadyCached(p) | FetchOutcome::Fetched(p) => p.clone(),
            };
            state.settled = true;
            state.repeats = 0;
            state.last_error = None;
            if let Some(suppressed) = state.latch.on_success() {
                tracing::info!(
                    suppressed_count = suppressed,
                    total_failures = state.latch.total_failures(),
                    "cerulion_viz: the OpenH264 fetch recovered"
                );
            }
            // The generation bump is what makes this visible, so it happens
            // whichever way the blob got there.
            GENERATION.fetch_add(1, Ordering::AcqRel);
            if matches!(outcome, FetchOutcome::Fetched(_)) {
                tracing::info!(
                    path = %path.display(),
                    version = CISCO_BLOB_VERSION,
                    "cerulion_viz: fetched and verified Cisco's OpenH264 binary — H.264 video \
                     now decodes on this desk instead of in the viewer"
                );
            } else {
                tracing::debug!(
                    path = %path.display(),
                    "cerulion_viz: Cisco's OpenH264 binary was already cached"
                );
            }
        }
        Err(e) => {
            // A platform Cisco publishes nothing for, or nowhere to put a blob,
            // will not change however many times it is retried.
            if matches!(
                e,
                FetchError::UnsupportedPlatform { .. }
                    | FetchError::NoCacheLocation
                    | FetchError::OverrideNotOurs { .. }
            ) {
                state.settled = true;
            }
            record_failure(&mut state, &e);
        }
    }
}

/// Report a failed attempt: flood-latched (this can repeat once a minute for as
/// long as a desk is offline), with an UNCONDITIONAL counter behind it.
fn record_failure(state: &mut FetcherState, e: &FetchError) {
    state.failures += 1;
    state.last_failure = Some(Instant::now());
    // Repeat-IDENTICAL errors escalate the backoff; a DIFFERENT one resets to the
    // floor, because the condition changed and the evidence that it was permanent
    // is gone. See `backoff_for`.
    let rendered = e.to_string();
    if state.last_error.as_deref() == Some(rendered.as_str()) {
        state.repeats = state.repeats.saturating_add(1);
    } else {
        state.repeats = 0;
        state.last_error = Some(rendered);
    }
    let terminal = state.settled;
    match state.latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(
            error = %e,
            total_failures = state.latch.total_failures(),
            terminal,
            "cerulion_viz: could not fetch Cisco's OpenH264 binary, so H.264 video is being \
             handed to the VIEWER to decode (that path works but measured 567 ms of lag). \
             Set CERULION_OPENH264_BLOB to an existing copy to skip the fetch."
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            error = %e,
            total_failures = total,
            suppressed,
            "cerulion_viz: STILL cannot fetch Cisco's OpenH264 binary"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            error = %e,
            suppressed,
            total_failures = state.latch.total_failures(),
            "cerulion_viz: OpenH264 fetch failed again"
        ),
    }
}

/// Fetch (or confirm) this platform's blob against Cisco's CDN.
///
/// The production entry: resolves the platform and the cache path, then defers
/// every decision to [`fetch_blob_with`].
pub fn fetch_blob(os: &str, arch: &str) -> Result<FetchOutcome, FetchError> {
    let source = blob_source_for(os, arch).ok_or_else(|| FetchError::UnsupportedPlatform {
        os: os.to_string(),
        arch: arch.to_string(),
    })?;
    let dest = cisco_blob_path().ok_or(FetchError::NoCacheLocation)?;
    let overridden = std::env::var_os(crate::video_decode::BLOB_PATH_ENV).is_some();
    let outcome = fetch_blob_with(&source, &dest, overridden, &download_from_cisco)?;
    // The digest proves the BYTES; only a LOAD proves this desk can USE them.
    // macOS hardened-runtime library validation (Studio ships as a signed
    // bundle), a `noexec` mount over $HOME, or missing runtime deps for a `.so`
    // all pass the hash and fail `dlopen`. Announcing "video now decodes here"
    // without this asserts an outcome nothing checked — and the success arm
    // SETTLES the fetcher, making that claim permanent.
    //
    // Checked HERE rather than inside `fetch_blob_with` because that function's
    // hermetic tests install stand-in bytes that are not a shared library; this
    // entry only ever handles a real Cisco blob.
    verify_loadable(match &outcome {
        FetchOutcome::AlreadyCached(p) | FetchOutcome::Fetched(p) => p,
    })?;
    Ok(outcome)
}

/// The whole fetch DECISION path, with the download injected.
///
/// # Why the seam exists
///
/// Everything that makes this module safe lives here: the cache-hit
/// short-circuit (which is what makes a good blob impossible to overwrite), the
/// corrupt-cache replace, the override refusal, and — above all — the rule that a
/// digest mismatch leaves `dest` BYTE-UNCHANGED. With the download hard-wired,
/// none of it was reachable from a hermetic test: `fetch_blob`'s only callers were
/// the background thread and an `#[ignore]`d live test, so deleting the cache-hit
/// block entirely left the suite green. A test that asserted "no network call
/// happened" while never invoking this function was asserting nothing.
///
/// Injected the same way the netd ladder injects its existence predicate: the
/// caller supplies the effect, the decision stays testable. A test passes a
/// closure that PANICS to prove a path never downloads.
///
/// `overridden` says the cache path came from the operator's own
/// [`crate::video_decode::BLOB_PATH_ENV`] — passed in rather than read here so
/// this stays free of process-global env for tests.
pub fn fetch_blob_with(
    source: &BlobSource,
    dest: &Path,
    overridden: bool,
    download: &dyn Fn(&str) -> Result<Vec<u8>, FetchError>,
) -> Result<FetchOutcome, FetchError> {
    let dest = dest.to_path_buf();
    match std::fs::read(&dest) {
        Ok(existing) => {
            if verify_digest(&existing, source.sha256).is_ok() {
                return Ok(FetchOutcome::AlreadyCached(dest));
            }
            // Present but WRONG: a truncated fetch, a foreign file, another
            // version. It is replaced below — but only by bytes that pass the
            // digest, so the desk cannot end up worse off than it is now.
            tracing::warn!(
                path = %dest.display(),
                "cerulion_viz: the cached OpenH264 blob is not this platform's Cisco release — \
                 re-fetching it"
            );
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => {
            // A read that failed for a REASON (EACCES, EISDIR, a failing disk)
            // is not "no cache". The fetch continues — a re-download may well
            // succeed — but the install below will likely fail on the same
            // condition, and without this line its error names only the write.
            tracing::debug!(
                path = %dest.display(),
                error = %e,
                "cerulion_viz: could not read the cached OpenH264 blob to check it"
            );
        }
    }

    // M1: an explicit override is a DECLARATION that the operator manages that
    // file — the air-gapped / distro-package / shared-mount case the env var is
    // documented for. Replacing it is not ours to do, and the module promises
    // never to overwrite a good blob. (An override that is ALREADY the right
    // bytes returned `AlreadyCached` above, so this arm is reached only when we
    // would otherwise clobber the operator's file.)
    if overridden {
        return Err(FetchError::OverrideNotOurs { path: dest });
    }

    let compressed = download(&source.url)?;
    if compressed.len() > MAX_COMPRESSED_BYTES {
        return Err(FetchError::TooLarge {
            limit: MAX_COMPRESSED_BYTES,
            stage: "download",
        });
    }
    let library = bunzip2(&compressed, MAX_DECOMPRESSED_BYTES)?;
    // The ONE gate between the network and the cache path. Everything after it
    // is known-good bytes; everything before it touches `dest` not at all.
    verify_digest(&library, source.sha256)?;
    install_blob(&dest, &library)?;
    Ok(FetchOutcome::Fetched(dest))
}

/// Confirm the blob actually loads on this desk.
///
/// UNCONDITIONAL, deliberately. `OpenH264API::from_blob_path` is gated on the
/// `libloading` feature, which this crate enables in every configuration, so the
/// symbol exists even in a `decoder-from-source` build. Gating this behind
/// that feature would disable the check in exactly the build the network-only
/// live test runs under, so the one place that could exercise dlopen against a
/// real Cisco blob would print a note saying it had skipped it.
pub fn verify_loadable(path: &Path) -> Result<(), FetchError> {
    openh264::OpenH264API::from_blob_path(path)
        .map(|_probe| ())
        .map_err(|e| FetchError::InstalledButUnloadable {
            path: path.to_path_buf(),
            why: e.to_string(),
        })
}

/// How long a single attempt may spend connecting.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);

/// How long a single attempt may spend in total, connect through last byte.
///
/// Generous against the real work (1.7 MB at worst; 771 ms measured on a desk),
/// because this is a CEILING on a stall, not a performance target — a slow hotel
/// link should still succeed.
pub const REQUEST_TIMEOUT: Duration = Duration::from_secs(120);

/// GET `url` into memory under [`MAX_COMPRESSED_BYTES`].
///
/// # Every timeout is set explicitly, and that is load-bearing
///
/// `ureq` 3.3's `Timeouts::default()` is `None` for connect, global, and every
/// body/response timeout (only `await_100` is set) — read from the resolved
/// dependency's source, not assumed. A bare `ureq::get(url).call()` therefore has
/// NO upper bound.
///
/// An unbounded call is not merely slow here: `run_one_attempt` clears the
/// in-flight flag only when this RETURNS, so a TLS handshake that stalls after
/// ACK, a body that stops mid-stream, or a half-open socket surviving a laptop
/// suspend leaves `running` set forever. The gate then answers `AlreadyRunning`
/// for the life of the process and the desk never retries — with no log line,
/// because nothing failed. That is the exact silent-permanent-failure the
/// backoff exists to prevent, and it would hit hardest on the flaky and proxied
/// networks that need the retry most.
///
/// A fetch that cannot finish must be a FAILED fetch, so it can be counted,
/// reported, and tried again.
pub fn download_from_cisco(url: &str) -> Result<Vec<u8>, FetchError> {
    use std::io::Read as _;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_connect(Some(CONNECT_TIMEOUT))
        .timeout_global(Some(REQUEST_TIMEOUT))
        // ureq defaults to `https_only: false` with 10 redirects, so a redirect
        // off Cisco's HTTPS CDN to plaintext would be followed. Not exploitable —
        // the digest gate precedes `install_blob`, so a downgraded transport can
        // only cause a `DigestMismatch` — but `CISCO_CDN_BASE` is a hardcoded
        // `https://` constant, so this client has no legitimate reason to ever
        // speak plaintext.
        .https_only(true)
        .build()
        .into();
    let response = agent
        .get(url)
        .call()
        .map_err(|e| FetchError::Download(format!("{e}")))?;
    let mut body = Vec::new();
    // `take` is the ceiling: a response with a lying (or absent) Content-Length
    // cannot grow this past the cap, because the READER stops there.
    response
        .into_body()
        .into_reader()
        .take(MAX_COMPRESSED_BYTES as u64 + 1)
        .read_to_end(&mut body)
        .map_err(|e| FetchError::Download(format!("{e}")))?;
    if body.len() > MAX_COMPRESSED_BYTES {
        return Err(FetchError::TooLarge {
            limit: MAX_COMPRESSED_BYTES,
            stage: "download",
        });
    }
    Ok(body)
}

/// Decompress Cisco's `.bz2` under `limit` bytes.
///
/// `limit` is a PARAMETER rather than the constant so the bomb ceiling — which
/// leads this module's docs — is reachable from a test without a multi-megabyte
/// fixture. Production always passes [`MAX_DECOMPRESSED_BYTES`].
fn bunzip2(compressed: &[u8], limit: usize) -> Result<Vec<u8>, FetchError> {
    use std::io::Read as _;
    let mut out = Vec::new();
    bzip2_rs::DecoderReader::new(compressed)
        .take(limit as u64 + 1)
        .read_to_end(&mut out)
        .map_err(|e| FetchError::Decompress(format!("{e}")))?;
    if out.len() > limit {
        return Err(FetchError::TooLarge {
            limit,
            stage: "decompressed library",
        });
    }
    // An EMPTY result is not a library. Without this it sails into the digest
    // check and is reported as a hash mismatch, which points an operator at
    // "Cisco re-cut the release" when the truth is an empty or non-bzip2 body
    // (a captive portal, a proxy error page, a truncated response).
    if out.is_empty() {
        return Err(FetchError::Decompress(
            "the response decompressed to zero bytes".to_string(),
        ));
    }
    Ok(out)
}

/// The `.{name}.part-{pid}` prefix every temporary of THIS process shares.
fn temp_prefix(dest: &Path) -> String {
    format!(
        ".{}.part-{}",
        dest.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "openh264".to_string()),
        std::process::id()
    )
}

/// Remove temporaries THIS process left behind on an earlier attempt.
///
/// Both failure arms unlink their own, so the only leak is a SIGKILL / OOM-kill /
/// power loss inside the millisecond write window. Scoped to this PID's prefix on
/// purpose: another PROCESS's temporary may be a fetch IN FLIGHT, and the whole
/// point of the temp-then-rename is that concurrent fetchers never collide.
///
/// # The gap this leaves, stated because a sibling comment implies otherwise
///
/// `install_blob`'s temp name carries a nanos suffix justified by two SAME-PROCESS
/// callers of the `pub fn fetch_blob_with` — and this sweep is scoped by PID
/// PREFIX, so for exactly that case it deletes the sibling call's in-flight
/// temporary. The two are consistent only once said out loud: the nanos stop two
/// same-process callers from writing the SAME file (which would corrupt one), and
/// this sweep can still delete the other's temp — which cannot corrupt anything,
/// because the victim's RENAME then fails `ENOENT` and surfaces as a retryable
/// `FetchError::Write`.
///
/// Precisely, because this paragraph exists for the mechanism: the WRITE does not
/// fail. On Unix an unlink removes the directory ENTRY, not the open file, so
/// `write_durably` completes happily against a descriptor whose name is already
/// gone — only the rename sees the absence. Same conclusion (one attempt lost,
/// nothing corrupted), correct reason.
///
/// Unreachable in production: `note_decoder_needed` holds the state mutex and the
/// `running` flag, so exactly one fetch is ever in flight per process. The
/// residual is two hand-written callers of the public entry — today, tests.
///
/// Best-effort — a sweep failure must never fail a fetch that would otherwise
/// succeed.
fn sweep_stale_temporaries(dir: &Path, dest: &Path) {
    let prefix = temp_prefix(dest);
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        if entry.file_name().to_string_lossy().starts_with(&prefix) {
            let _ = std::fs::remove_file(entry.path());
        }
    }
}

/// Write `bytes` to `path` and FSYNC before returning.
///
/// `fs::write` + `rename` alone makes the atomicity claim true across a PROCESS
/// crash but not a MACHINE one: on a filesystem without rename-durability
/// ordering, a power loss shortly after the rename can expose a short file at
/// exactly the loader's path — the one state `install_blob`'s docs say is worse
/// than an absent one. `sync_all` costs one flush per fetch (once per desk,
/// ever) and makes the sentence true.
///
/// The DIRECTORY entry is deliberately not fsynced: losing the rename loses the
/// blob, which is the absent case the fetcher already handles by re-fetching.
fn write_durably(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write as _;
    let mut f = std::fs::File::create(path)?;
    f.write_all(bytes)?;
    f.sync_all()
}

/// Place verified `bytes` at `dest` ATOMICALLY.
///
/// Write to a temporary sibling, then `rename`. A partial file at `dest` is the
/// one state that is worse than an absent one — the loader would find it,
/// refuse it, and report a condition only a human deleting the file can clear —
/// and a rename within one directory cannot produce it.
fn install_blob(dest: &Path, bytes: &[u8]) -> Result<(), FetchError> {
    let dir = dest
        .parent()
        .ok_or_else(|| FetchError::Write(format!("{} has no parent", dest.display())))?;
    std::fs::create_dir_all(dir).map_err(|e| FetchError::Write(format!("{e}")))?;
    // PID-scoped so two processes fetching at once cannot write one temp file.
    // PID *and* nanos: the PID alone is unique on every production path (one fetch
    // in flight per process, enforced by the state mutex + `running` flag), but
    // `fetch_blob_with` is `pub`, and a shared `$HOME` across PID namespaces can
    // collide PIDs outright.
    let tmp = dir.join(format!(
        "{}-{}",
        temp_prefix(dest),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    // Sweep any temporary a KILLED earlier attempt left behind. Both failure arms
    // below unlink their own, but a SIGKILL between the write and the rename leaks
    // one, and the name is unique-per-attempt so nothing would ever reuse it.
    sweep_stale_temporaries(dir, dest);
    if let Err(e) = write_durably(&tmp, bytes) {
        // A partial temp (ENOSPC, quota) is nobody's blob, and nothing will ever
        // look for that name again — so it would accumulate invisibly.
        let _ = std::fs::remove_file(&tmp);
        return Err(FetchError::Write(format!("{e}")));
    }
    if let Err(e) = std::fs::rename(&tmp, dest) {
        let _ = std::fs::remove_file(&tmp);
        return Err(FetchError::Write(format!("{e}")));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The digest table and the FILENAME table must agree for every platform
    /// this fetcher claims — they live in different modules, and
    /// `video_decode`'s own docs say the two "have to be changed together".
    ///
    /// Driven over the whole `(os, arch)` matrix rather than the host platform,
    /// which is the only way one machine can see all of it.
    #[test]
    fn every_fetchable_platform_agrees_with_the_filename_table() {
        let fetchable = [
            ("macos", "aarch64", "libopenh264-2.6.0-mac-arm64.dylib"),
            ("macos", "x86_64", "libopenh264-2.6.0-mac-x64.dylib"),
            ("linux", "x86_64", "libopenh264-2.6.0-linux64.8.so"),
            ("linux", "aarch64", "libopenh264-2.6.0-linux-arm64.8.so"),
        ];
        for (os, arch, expected_name) in fetchable {
            let source = blob_source_for(os, arch)
                .unwrap_or_else(|| panic!("{os}/{arch} must be fetchable"));
            assert_eq!(
                source.filename, expected_name,
                "{os}/{arch} must fetch the name the loader looks for"
            );
            assert_eq!(
                cisco_blob_filename_for(os, arch),
                Some(expected_name),
                "the filename table must agree with the fetch plan for {os}/{arch}"
            );
            assert_eq!(
                source.url,
                format!("https://ciscobinary.openh264.org/{expected_name}.bz2"),
                "the URL is Cisco's own asset for {os}/{arch}"
            );
            assert_eq!(
                source.sha256.len(),
                64,
                "a digest is 64 hex chars for {os}/{arch}"
            );
            assert!(
                source
                    .sha256
                    .chars()
                    .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()),
                "digests are lowercase hex ({os}/{arch}): {}",
                source.sha256
            );
        }

        // Every digest is DISTINCT — a copy-paste that pointed two platforms at
        // one hash would make one of them permanently unfetchable, and the
        // per-platform check is the whole reason this table exists apart from
        // the loader's.
        let mut seen: Vec<&str> = CISCO_BLOB_SHA256.iter().map(|(_, s)| *s).collect();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(before, seen.len(), "every platform digest must be distinct");
    }

    /// A platform Cisco publishes a binary for but nobody has VERIFIED a digest
    /// for gets no fetch — not a fetch against a guessed hash.
    #[test]
    fn a_platform_without_a_verified_digest_is_not_fetchable() {
        // Cisco does publish these, and `cisco_blob_filename_for` names them…
        for (os, arch) in [("windows", "x86_64"), ("linux", "arm"), ("linux", "x86")] {
            assert!(
                cisco_blob_filename_for(os, arch).is_some(),
                "{os}/{arch} has a Cisco filename"
            );
            // …but no digest was measured, so no fetch is planned.
            assert_eq!(
                blob_source_for(os, arch),
                None,
                "{os}/{arch} must not be fetched against an unverified digest"
            );
        }
        // And a platform Cisco publishes nothing for at all.
        assert_eq!(blob_source_for("freebsd", "aarch64"), None);
        assert_eq!(cisco_blob_filename_for("freebsd", "aarch64"), None);
    }

    /// THE VERIFICATION ORACLE: only the exact bytes pass.
    ///
    /// Hand-computed against a known vector — `sha256("")` — so the check is
    /// pinned to an external truth rather than to whatever this code produces.
    #[test]
    fn the_digest_check_accepts_only_the_expected_bytes() {
        const EMPTY_SHA256: &str =
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
        assert_eq!(verify_digest(b"", EMPTY_SHA256), Ok(()));

        // One byte more is a different file.
        match verify_digest(b"\0", EMPTY_SHA256) {
            Err(FetchError::DigestMismatch { expected, actual }) => {
                assert_eq!(expected, EMPTY_SHA256);
                assert_ne!(actual, EMPTY_SHA256);
                assert_eq!(actual.len(), 64, "the ACTUAL digest is reported in full");
            }
            other => panic!("a wrong blob must be refused, got {other:?}"),
        }

        // The failure text says nothing was cached — the operator's next step
        // depends on knowing the desk was not left in a broken state.
        let e = verify_digest(b"junk", EMPTY_SHA256).unwrap_err();
        assert!(
            e.to_string().contains("nothing was cached"),
            "a mismatch must say the cache was untouched: {e}"
        );
    }

    /// THE BUSY-LOOP GATE. `decode()` calls the kick at frame rate, so every
    /// arm of this decision is load-bearing on a desk with no network.
    #[test]
    fn the_kick_gate_starts_once_then_backs_off_then_settles() {
        let backoff = Duration::from_secs(60);

        // Nothing has happened yet: start.
        assert_eq!(
            decide_fetch_kick(true, false, false, None, backoff),
            FetchGate::Start
        );
        // One in flight: never a second.
        assert_eq!(
            decide_fetch_kick(true, false, true, None, backoff),
            FetchGate::AlreadyRunning
        );
        // A failure one second ago: back off, and say how long is left.
        assert_eq!(
            decide_fetch_kick(true, false, false, Some(Duration::from_secs(1)), backoff),
            FetchGate::BackingOff(Duration::from_secs(59))
        );
        // BOTH SIDES of the threshold — the arm that decides whether a desk
        // retries at all.
        assert_eq!(
            decide_fetch_kick(
                true,
                false,
                false,
                Some(backoff - Duration::from_nanos(1)),
                backoff
            ),
            FetchGate::BackingOff(Duration::from_nanos(1)),
            "one nanosecond short of the backoff is still backing off"
        );
        assert_eq!(
            decide_fetch_kick(true, false, false, Some(backoff), backoff),
            FetchGate::Start,
            "AT the backoff the desk tries again — an offline desk that regains \
             its network must recover on its own"
        );
        // Settled wins over everything, including a stale failure and a
        // (nonsensical) concurrent run.
        for running in [false, true] {
            for since in [None, Some(Duration::from_secs(3600))] {
                assert_eq!(
                    decide_fetch_kick(true, true, running, since, backoff),
                    FetchGate::Settled,
                    "a settled fetcher never starts another attempt"
                );
            }
        }
        // DISABLED outranks every other state — including the one that would
        // otherwise start. An operator saying `off` must never open a socket.
        for settled in [false, true] {
            for running in [false, true] {
                for since in [None, Some(Duration::from_secs(3600))] {
                    assert_eq!(
                        decide_fetch_kick(false, settled, running, since, backoff),
                        FetchGate::Disabled,
                        "a disabled fetcher never starts an attempt"
                    );
                }
            }
        }
    }

    /// A fetch thread that PANICS must not wedge the fetcher.
    ///
    /// `running` is what stops a second attempt starting, so a thread that died
    /// without clearing it would make `decide_fetch_kick` answer `AlreadyRunning`
    /// for the life of the process: no further attempt, ever, and no further log
    /// line — a silent permanent failure. The work runs in third-party code (TLS,
    /// HTTP, bzip2), so this is not a hypothetical the module gets to wave away.
    ///
    /// It must also be recorded as a FAILURE, not merely cleared: otherwise a
    /// reliably-panicking fetch is retried on the very next access unit, at frame
    /// rate, and a silent wedge becomes a loud storm.
    ///
    /// This is the ONLY test in this binary that touches the process-global
    /// `STATE`, so the counts below are exact.
    #[test]
    fn a_fetch_thread_that_panics_does_not_wedge_the_fetcher() {
        let _state = state_lock();
        let before = status();
        assert!(
            !before.running,
            "the state lock is held, so no sibling can have a fetch in flight"
        );
        // A sibling that ran first may have left a failure recorded; this arm is
        // about the DELTA, so start from a known-clean backoff state.
        {
            let mut state = lock_state();
            state.last_failure = None;
            state.last_error = None;
            state.repeats = 0;
        }

        // Exactly what `note_decoder_needed` does before spawning.
        {
            let mut state = STATE.lock().unwrap_or_else(|p| p.into_inner());
            state.running = true;
        }
        assert!(status().running, "arranged: an attempt is in flight");

        // A worker that takes the guard and dies without reaching the reporting
        // section — the shape of a panic inside `fetch_blob`.
        let died = std::thread::spawn(|| {
            let _guard = RunningGuard { armed: true };
            panic!("fetch test: a fetch thread dying mid-attempt");
        })
        .join();
        assert!(died.is_err(), "the worker must really have panicked");

        let after = status();
        assert!(
            !after.running,
            "a dead fetch thread must release the in-flight flag, or no attempt can \
             ever start again"
        );
        assert_eq!(
            after.failures,
            before.failures + 1,
            "the death must be COUNTED — an uncounted one is invisible, and the \
             backoff that stops a panic storm keys off it"
        );
        // And the gate agrees: the next kick is held off by the backoff rather
        // than blocked forever by a stale in-flight flag.
        {
            let state = STATE.lock().unwrap_or_else(|p| p.into_inner());
            let gate = decide_fetch_kick(
                true,
                state.settled,
                state.running,
                state.last_failure.map(|t| t.elapsed()),
                RETRY_BACKOFF,
            );
            assert!(
                matches!(gate, FetchGate::BackingOff(_)),
                "after a panic the fetcher backs off; it must not report \
                 AlreadyRunning (wedged) or Start (a frame-rate storm), got {gate:?}"
            );
        }

        // Leave the module as it was found, so this test cannot colour a sibling.
        {
            let mut state = STATE.lock().unwrap_or_else(|p| p.into_inner());
            state.last_failure = None;
            state.last_error = None;
            state.repeats = 0;
        }
    }

    /// The kill switch is EXACT, and a from-source build never fetches.
    ///
    /// Only the documented value disables the fetch: a typo must not silently
    /// turn a desk's video slow with no trace. The SHIPPED shape — no
    /// `decoder-from-source`, env unset: must come out ENABLED, or the fetch is
    /// inert on the one configuration that ships.
    #[test]
    fn only_the_documented_kill_switch_value_disables_the_fetch() {
        // THE SHIPPING ROW.
        assert!(
            fetch_enabled_from(false, None),
            "a shipped build with no kill switch MUST fetch — anything else ships the fetch inert"
        );
        assert!(!fetch_enabled_from(false, Some("off")), "`off` disables it");
        // Near-misses stay ENABLED rather than silently disabling video.
        for typo in ["OFF", "Off", "0", "false", "no", "off ", " off", ""] {
            assert!(
                fetch_enabled_from(false, Some(typo)),
                "{typo:?} is not the documented kill switch and must not disable the fetch"
            );
        }
        // A from-source build never fetches, whatever the env says — it has the
        // decoder compiled in and there is nothing to download.
        for env in [None, Some("off"), Some("on"), Some("")] {
            assert!(
                !fetch_enabled_from(true, env),
                "a decoder-from-source build never fetches (env {env:?})"
            );
        }
        // The SHIPPED wrapper, asserted at its REAL strength for this build.
        //
        // A round trip through `fetch_enabled_from` with the same arguments would
        // be a tautology: `cfg!(feature = "decoder-from-source")` is true in every
        // build of this suite, so the pure rule short-circuits before touching the
        // env and a shell reading a DIFFERENT variable would pass. What is
        // genuinely checkable here is the claim that keeps CI hermetic.
        assert!(
            !fetch_enabled(),
            "this crate's tests compile the decoder in, so the shipped wrapper must \
             refuse to fetch — this is what stops CI reaching Cisco's CDN"
        );
    }

    /// Serialises the tests that MUTATE the process-global [`STATE`].
    ///
    /// Not the same thing as `STATE`'s own mutex: that one is released between
    /// statements, so two tests can interleave a "reset then observe" sequence and
    /// each see the other's writes. Found the hard way — the `report_attempt`
    /// arm's reset cleared `last_failure` between the panic arm's `record_failure`
    /// and its gate assertion, turning `BackingOff` into `Start`.
    fn state_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: Mutex<()> = Mutex::new(());
        LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// **THE REPORTING HALF OF A REAL ATTEMPT** — the three lines that were
    /// deletable with a fully green suite.
    ///
    /// `run_one_attempt` had exactly one caller (a `.spawn()`) and no test could
    /// reach any of it, so the `GENERATION` bump — the entire no-restart
    /// payoff — and both `settled` writes could be removed without a single
    /// failure. The pool tests bump the generation through
    /// `bump_cache_generation_for_test`, so they stayed green while the SHIPPING
    /// path went inert.
    ///
    /// Drives `report_attempt` with hand-built outcomes. Touches the
    /// process-global `STATE` and `GENERATION`, so it restores both.
    ///
    /// Deleting the `GENERATION.fetch_add`
    /// fails the success arm; deleting either `settled = true` fails its own arm.
    #[test]
    fn a_reported_attempt_announces_success_and_settles_only_when_it_should() {
        let _state = state_lock();
        let restore = {
            let s = lock_state();
            (s.settled, s.running, s.failures, s.repeats)
        };
        let reset = || {
            let mut s = lock_state();
            s.settled = false;
            s.running = false;
            s.last_failure = None;
            s.last_error = None;
            s.repeats = 0;
        };

        // (1) SUCCESS: announces (generation bumps) and settles.
        reset();
        let before = cache_generation();
        report_attempt(Ok(FetchOutcome::Fetched(PathBuf::from("/c/blob.dylib"))));
        assert_eq!(
            cache_generation(),
            before + 1,
            "a completed fetch must ANNOUNCE itself — this is the whole no-restart \
             payoff, and nothing else in the process reaches this line"
        );
        assert!(status().settled, "a success settles the fetcher");
        assert!(!status().running, "the in-flight flag is cleared");

        // (2) A CACHE HIT announces too: the blob is on disk either way, and a
        // pool that resolved before it landed must still be told.
        reset();
        let before = cache_generation();
        report_attempt(Ok(FetchOutcome::AlreadyCached(PathBuf::from(
            "/c/blob.dylib",
        ))));
        assert_eq!(cache_generation(), before + 1);
        assert!(status().settled);

        // (3) A TERMINAL error settles and does NOT announce — nothing landed.
        for terminal in [
            FetchError::UnsupportedPlatform {
                os: "freebsd".into(),
                arch: "riscv64".into(),
            },
            FetchError::NoCacheLocation,
            FetchError::OverrideNotOurs {
                path: PathBuf::from("/opt/theirs.so"),
            },
        ] {
            reset();
            let before = cache_generation();
            report_attempt(Err(terminal.clone()));
            assert!(
                status().settled,
                "{terminal:?} cannot be fixed by retrying, so it must settle"
            );
            assert_eq!(
                cache_generation(),
                before,
                "{terminal:?} installed nothing, so nothing may be announced"
            );
        }

        // (4) A RETRYABLE error neither settles nor announces — settling on
        // `DigestMismatch` in particular would turn a stale CDN edge into the
        // silent permanent failure this module forbids.
        for retryable in [
            FetchError::DigestMismatch {
                expected: "aa".into(),
                actual: "bb".into(),
            },
            FetchError::Download("offline".into()),
            FetchError::Write("ENOSPC".into()),
            FetchError::TooLarge {
                limit: 8,
                stage: "download",
            },
        ] {
            reset();
            let before = cache_generation();
            report_attempt(Err(retryable.clone()));
            assert!(
                !status().settled,
                "{retryable:?} CAN be transient, so the fetcher must keep trying"
            );
            assert_eq!(cache_generation(), before);
            assert!(!status().running);
        }

        // Restore what this test changed.
        {
            let mut s = lock_state();
            s.settled = restore.0;
            s.running = restore.1;
            s.failures = restore.2;
            s.repeats = restore.3;
            s.last_failure = None;
            s.last_error = None;
        }
    }

    /// THE ESCALATING BACKOFF — the bound on TOTAL bytes, not just rate.
    ///
    /// The terminal set is narrow on purpose (settling on `DigestMismatch` would
    /// turn a stale CDN edge into the silent permanent failure this module
    /// forbids), so several permanent-in-practice conditions retry forever. And
    /// because nothing is written on those arms, the cache short-circuit misses
    /// and every attempt re-transfers in full — up to 8 MiB in the `TooLarge`
    /// case. A flat floor bounds the rate and nothing else.
    #[test]
    fn a_repeated_failure_backs_off_geometrically_up_to_a_ceiling() {
        assert_eq!(
            backoff_for(0),
            RETRY_BACKOFF,
            "the first retry is the floor"
        );
        assert_eq!(backoff_for(1), RETRY_BACKOFF * 2);
        assert_eq!(backoff_for(2), RETRY_BACKOFF * 4);
        // Monotone, and CAPPED — an unbounded doubling would silently become
        // "never retries again", which is the failure mode the narrow terminal
        // set exists to avoid.
        let mut previous = Duration::ZERO;
        for repeats in 0..64u32 {
            let d = backoff_for(repeats);
            assert!(d >= previous, "backoff must never shrink at {repeats}");
            assert!(
                d <= MAX_RETRY_BACKOFF,
                "backoff must never exceed the ceiling at {repeats}, got {d:?}"
            );
            previous = d;
        }
        assert_eq!(
            backoff_for(u32::MAX),
            MAX_RETRY_BACKOFF,
            "a huge repeat count saturates rather than overflowing"
        );
        // The ceiling is reached in a handful of attempts, not hundreds.
        let to_ceiling = (0..64u32)
            .find(|r| backoff_for(*r) == MAX_RETRY_BACKOFF)
            .expect("the ceiling must be reachable");
        assert!(
            to_ceiling <= 8,
            "a permanently-broken desk must reach the hourly ceiling quickly, took {to_ceiling}"
        );
    }

    /// A repeat-IDENTICAL error escalates; a DIFFERENT one resets to the floor.
    ///
    /// The reset is the load-bearing half: a changed error means the condition
    /// changed, so the evidence that it was permanent is gone. Without it, a desk
    /// that failed six times offline and then hit one transient error would still
    /// be waiting an hour to try the thing that now works.
    ///
    /// Uses a LOCAL `FetcherState`, so it touches no process-global state and has
    /// nothing to restore. Exactly two tests in this module touch the
    /// process-global `STATE`:
    /// `a_fetch_thread_that_panics_does_not_wedge_the_fetcher` and
    /// `a_reported_attempt_announces_success_and_settles_only_when_it_should`,
    /// and both take `state_lock`. A new test that drives the global must take it
    /// too.
    #[test]
    fn only_a_repeat_of_the_same_error_escalates_the_backoff() {
        let mut state = FetcherState::new();
        let offline = FetchError::Download("offline".to_string());
        let disk = FetchError::Write("ENOSPC".to_string());

        record_failure(&mut state, &offline);
        assert_eq!(state.repeats, 0, "the first failure is at the floor");
        record_failure(&mut state, &offline);
        record_failure(&mut state, &offline);
        assert_eq!(state.repeats, 2, "identical errors accumulate");
        assert_eq!(backoff_for(state.repeats), RETRY_BACKOFF * 4);

        // A DIFFERENT error resets.
        record_failure(&mut state, &disk);
        assert_eq!(state.repeats, 0, "a changed condition resets the backoff");
        assert_eq!(backoff_for(state.repeats), RETRY_BACKOFF);

        // Two `Write`s with DIFFERENT causes (ENOSPC then EROFS) are different
        // errors — the identity is the rendered MESSAGE, not the variant.
        record_failure(&mut state, &disk);
        assert_eq!(state.repeats, 1);
        record_failure(&mut state, &FetchError::Write("EROFS".to_string()));
        assert_eq!(
            state.repeats, 0,
            "same variant, different cause — the condition changed"
        );

        // The counter is UNCONDITIONAL regardless of the backoff bookkeeping.
        assert_eq!(state.failures, 6);
    }

    /// THE FLOOD-LATCH LEVELS at the production reporting site.
    ///
    /// `record_failure` is a `FailureRegimeLatch` consumer, and this repo pins
    /// every one of those by LEVEL TOKEN as well as message — the shared latch's first wave
    /// shipped three `StillFailing` emissions, level-pinned one, and reverting
    /// either surviving `warn!`/`error!` to `debug!` passed the whole suite.
    /// This is the layer whose entire justification is an offline
    /// desk retrying once a minute forever, so a repeat that is secretly loud (or
    /// a head that is secretly quiet) is the failure mode.
    ///
    /// Drives PAST the decade boundary so all three decisions are reachable.
    #[test]
    #[tracing_test::traced_test]
    fn the_failure_reporter_is_loud_once_then_quiet_then_loud_at_the_decade() {
        let mut state = FetcherState::new();
        let e = FetchError::Download("offline".to_string());
        for _ in 0..10 {
            record_failure(&mut state, &e);
        }

        logs_assert(|lines: &[&str]| {
            let head = lines
                .iter()
                .filter(|l| level_is(l, "WARN") && l.contains("could not fetch Cisco's"))
                .count();
            let decade = lines
                .iter()
                .filter(|l| level_is(l, "WARN") && l.contains("STILL cannot fetch"))
                .count();
            let quiet = lines
                .iter()
                .filter(|l| level_is(l, "DEBUG") && l.contains("fetch failed again"))
                .count();
            // 1 loud head + 8 suppressed + 1 loud decade re-announcement = 10.
            if (head, decade, quiet) != (1, 1, 8) {
                return Err(format!(
                    "expected (head, decade, suppressed) = (1, 1, 8), got {:?}\nlines:\n{}",
                    (head, decade, quiet),
                    lines.join("\n")
                ));
            }
            // The re-announcement must carry the RUNNING TOTAL — it exists for the
            // operator who missed the head.
            if !lines
                .iter()
                .any(|l| l.contains("STILL cannot fetch") && l.contains("total_failures=10"))
            {
                return Err("the decade line must carry total_failures=10".to_string());
            }
            Ok(())
        });
        assert_eq!(
            state.failures, 10,
            "the counter is UNCONDITIONAL — it does not care what was logged"
        );

        // RECOVERY closes the regime exactly once, carrying what was suppressed.
        let suppressed = state.latch.on_success().expect("a regime was open");
        assert_eq!(
            suppressed, 8,
            "recovery reports the SUPPRESSED count, not the total"
        );
    }

    /// A healthy fetcher says NOTHING — without this the "exactly N" counts above
    /// would pass a reporter that also fired on success.
    #[test]
    #[tracing_test::traced_test]
    fn a_fetcher_that_never_fails_logs_nothing() {
        let mut state = FetcherState::new();
        assert!(
            state.latch.on_success().is_none(),
            "a latch that never opened has nothing to recover from"
        );
        assert_eq!(state.failures, 0);
        logs_assert(|lines: &[&str]| {
            let noisy = lines
                .iter()
                .filter(|l| l.contains("fetch") && (l.contains("cannot") || l.contains("failed")))
                .count();
            if noisy != 0 {
                return Err(format!("a healthy fetcher logged {noisy} failure lines"));
            }
            Ok(())
        });
    }

    /// Whether a captured line carries `level` as a whole token in its HEADER.
    ///
    /// A bare substring match would also hit the SPAN NAME (`tracing-test` renders
    /// the test function's own name into every line), so a rename could silently
    /// invert an "exactly N" oracle, a known hazard.
    fn level_is(line: &str, level: &str) -> bool {
        line.split_whitespace().take(4).any(|tok| tok == level)
    }

    /// A per-test scratch directory, removed on drop (including after a panic).
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(tag: &str) -> Self {
            let dir = std::env::temp_dir().join(format!(
                "{tag}-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("a clock after 1970")
                    .as_nanos()
            ));
            std::fs::create_dir_all(&dir).expect("create the scratch dir");
            Self(dir)
        }
        fn join(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }
        /// Any surviving `.part-` temporary — the litter the atomic install exists
        /// to prevent.
        fn leftovers(&self) -> Vec<String> {
            std::fs::read_dir(&self.0)
                .expect("list the scratch dir")
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|n| n.contains(".part-"))
                .collect()
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A bzip2 stream of the ASCII bytes `cerulion`, produced by the system
    /// `bzip2 -9` — an EXTERNAL truth, not something our own code encoded.
    const BZ2_CERULION: &[u8] = &[
        0x42, 0x5a, 0x68, 0x39, 0x31, 0x41, 0x59, 0x26, 0x53, 0x59, 0xd6, 0x33, 0xd1, 0x0b, 0x00,
        0x00, 0x00, 0x01, 0x80, 0x0a, 0x25, 0x92, 0x00, 0x20, 0x00, 0x31, 0x0c, 0x08, 0x20, 0x33,
        0x48, 0xb1, 0x40, 0x27, 0x8b, 0xb9, 0x22, 0x9c, 0x28, 0x48, 0x6b, 0x19, 0xe8, 0x85, 0x80,
    ];
    /// `sha256("cerulion")` — what `BZ2_CERULION` decompresses to.
    const CERULION_SHA256: &str =
        "9e35abf7b3a5bf4275b94938ddf82dd1b0b292856d8f4b13b3ea8256670608db";

    fn source_for_tests(sha256: &'static str) -> BlobSource {
        BlobSource {
            filename: "libopenh264-test.dylib",
            url: "https://ciscobinary.openh264.org/libopenh264-test.dylib.bz2".to_string(),
            sha256,
        }
    }

    /// A download closure that must never be called.
    fn no_download() -> impl Fn(&str) -> Result<Vec<u8>, FetchError> {
        |url: &str| panic!("this path must NOT open a socket, but asked for {url}")
    }

    /// **THE "NEVER OVERWRITE A GOOD BLOB" ORACLE** — driven through the real
    /// decision path.
    ///
    /// A cache hit must short-circuit BEFORE any download, and the proof is that
    /// the injected downloader PANICS if it is ever reached. The previous version
    /// of this test claimed exactly that and never called `fetch_blob` at all —
    /// so deleting the whole cache-hit block left it green.
    ///
    /// Removing the cache-hit
    /// short-circuit from `fetch_blob_with` fails this test with the downloader's
    /// panic.
    #[test]
    fn a_matching_cache_short_circuits_before_any_network_call() {
        let sb = Scratch::new("cachehit");
        let dest = sb.join("blob");
        std::fs::write(&dest, b"cerulion").expect("seed the cache");

        let outcome = fetch_blob_with(
            &source_for_tests(CERULION_SHA256),
            &dest,
            false,
            &no_download(),
        );
        assert_eq!(
            outcome,
            Ok(FetchOutcome::AlreadyCached(dest.clone())),
            "bytes already matching the digest must be reported as cached"
        );
        assert_eq!(
            std::fs::read(&dest).expect("read the cached blob"),
            b"cerulion",
            "a good blob is byte-untouched"
        );
    }

    /// A CORRUPT cache IS replaced — and only by bytes that passed the digest.
    ///
    /// The anti-tautology partner of the arm above: without it, a `fetch_blob_with`
    /// that short-circuited on ANY existing file would pass that one.
    #[test]
    fn a_corrupt_cache_is_replaced_by_verified_bytes() {
        let sb = Scratch::new("corrupt");
        let dest = sb.join("blob");
        std::fs::write(&dest, b"truncated").expect("seed a corrupt cache");

        let outcome = fetch_blob_with(&source_for_tests(CERULION_SHA256), &dest, false, &|_url| {
            Ok(BZ2_CERULION.to_vec())
        });
        assert_eq!(outcome, Ok(FetchOutcome::Fetched(dest.clone())));
        assert_eq!(
            std::fs::read(&dest).expect("read the repaired blob"),
            b"cerulion",
            "the corrupt file is replaced by the verified download"
        );
        assert!(
            sb.leftovers().is_empty(),
            "a completed install leaves no temporary behind: {:?}",
            sb.leftovers()
        );
    }

    /// **THE STRONGEST SAFETY CLAIM**: a digest mismatch leaves the destination
    /// BYTE-UNCHANGED, and writes nothing at all.
    ///
    /// Asserted on both shapes that matter — an ABSENT destination must stay
    /// absent (no partial file for the loader to find and refuse), and an existing
    /// one must survive intact. Nothing anywhere asserted this before.
    #[test]
    fn a_digest_mismatch_writes_nothing_and_leaves_the_destination_alone() {
        // Shape 1: nothing there to begin with.
        let sb = Scratch::new("mismatch-absent");
        let dest = sb.join("blob");
        let wrong =
            source_for_tests("0000000000000000000000000000000000000000000000000000000000000000");
        match fetch_blob_with(&wrong, &dest, false, &|_url| Ok(BZ2_CERULION.to_vec())) {
            Err(FetchError::DigestMismatch { .. }) => {}
            other => panic!("a wrong blob must be refused, got {other:?}"),
        }
        assert!(
            !dest.exists(),
            "a refused download must not create the destination at all"
        );
        assert!(sb.leftovers().is_empty(), "nor any temporary");

        // Shape 2: an existing (stale) file must survive untouched.
        let sb2 = Scratch::new("mismatch-existing");
        let dest2 = sb2.join("blob");
        std::fs::write(&dest2, b"the operator's own file").expect("seed");
        let _ = fetch_blob_with(&wrong, &dest2, false, &|_url| Ok(BZ2_CERULION.to_vec()));
        assert_eq!(
            std::fs::read(&dest2).expect("read"),
            b"the operator's own file",
            "a refused download must leave what was there BYTE-unchanged"
        );
        assert!(sb2.leftovers().is_empty());
    }

    /// An operator-set `CERULION_OPENH264_BLOB` is never overwritten.
    ///
    /// That env var is documented for the air-gapped / distro-package / shared-mount
    /// case — the operator is declaring they manage that file. A mismatch there is
    /// a REFUSAL naming the variable, not a silent replacement (which could have
    /// landed on `/usr/lib/...`, creating directories on the way).
    #[test]
    fn an_operator_managed_override_is_refused_never_replaced() {
        let sb = Scratch::new("override");
        let dest = sb.join("their-libopenh264.so");
        std::fs::write(&dest, b"a distro package's own copy").expect("seed");

        match fetch_blob_with(
            &source_for_tests(CERULION_SHA256),
            &dest,
            true,
            &no_download(),
        ) {
            Err(FetchError::OverrideNotOurs { path }) => assert_eq!(path, dest),
            other => panic!("an override must be refused, got {other:?}"),
        }
        assert_eq!(
            std::fs::read(&dest).expect("read"),
            b"a distro package's own copy",
            "the operator's file is untouched"
        );

        // ANTI-TAUTOLOGY: an override that ALREADY holds the right bytes is a
        // plain cache hit, not a refusal — the refusal is about overwriting.
        let dest_ok = sb.join("good.so");
        std::fs::write(&dest_ok, b"cerulion").expect("seed");
        assert_eq!(
            fetch_blob_with(
                &source_for_tests(CERULION_SHA256),
                &dest_ok,
                true,
                &no_download()
            ),
            Ok(FetchOutcome::AlreadyCached(dest_ok))
        );
    }

    /// A download error and an over-size response both surface as themselves,
    /// and neither touches the destination.
    #[test]
    fn a_failed_or_oversized_download_is_reported_and_writes_nothing() {
        let sb = Scratch::new("dlfail");
        let dest = sb.join("blob");
        let source = source_for_tests(CERULION_SHA256);

        match fetch_blob_with(&source, &dest, false, &|_url| {
            Err(FetchError::Download("offline".to_string()))
        }) {
            Err(FetchError::Download(why)) => assert!(why.contains("offline")),
            other => panic!("a download failure must surface, got {other:?}"),
        }
        assert!(!dest.exists());

        // Over the COMPRESSED ceiling — refused before decompression is attempted.
        match fetch_blob_with(&source, &dest, false, &|_url| {
            Ok(vec![0u8; MAX_COMPRESSED_BYTES + 1])
        }) {
            Err(FetchError::TooLarge { limit, stage }) => {
                assert_eq!(limit, MAX_COMPRESSED_BYTES);
                assert_eq!(stage, "download");
            }
            other => panic!("an over-size response must be refused, got {other:?}"),
        }
        assert!(!dest.exists());

        // Not bzip2 at all (a captive portal's HTML, a proxy error page).
        match fetch_blob_with(&source, &dest, false, &|_url| Ok(b"<html>nope".to_vec())) {
            Err(FetchError::Decompress(_)) => {}
            other => panic!("a non-bzip2 body must be refused, got {other:?}"),
        }
        assert!(!dest.exists());
        assert!(sb.leftovers().is_empty());
    }

    /// `install_blob`'s failure arms: a parentless destination, and a directory
    /// that cannot be written.
    #[test]
    fn install_failures_are_reported_and_leave_no_temporary() {
        // No parent to put a temporary in.
        match install_blob(Path::new("/"), b"x") {
            Err(FetchError::Write(why)) => assert!(why.contains("no parent"), "{why}"),
            other => panic!("a parentless dest must be a Write error, got {other:?}"),
        }

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let sb = Scratch::new("readonly");
            let ro = sb.join("ro");
            std::fs::create_dir_all(&ro).expect("mkdir");
            std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o500))
                .expect("chmod read-only");
            let dest = ro.join("blob");
            let result = install_blob(&dest, b"x");
            // Restore before asserting so the scratch dir can be removed.
            let _ = std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o700));
            match result {
                Err(FetchError::Write(_)) => {}
                other => panic!("an unwritable dir must be a Write error, got {other:?}"),
            }
            assert!(!dest.exists(), "nothing was created");
            let parts: Vec<_> = std::fs::read_dir(&ro)
                .expect("list")
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .collect();
            assert!(parts.is_empty(), "no temporary survived: {parts:?}");
        }
    }

    /// Every `FetchError` says something an operator can act on.
    ///
    /// These strings are the ONLY window the operator gets — `record_failure`
    /// renders them as `error = %e` — so the repo treats them as contract.
    #[test]
    fn every_fetch_error_names_what_went_wrong() {
        let cases: Vec<(FetchError, Vec<&str>)> = vec![
            (
                FetchError::UnsupportedPlatform {
                    os: "freebsd".into(),
                    arch: "riscv64".into(),
                },
                vec!["freebsd", "riscv64"],
            ),
            (
                FetchError::NoCacheLocation,
                vec!["no cache location", "HOME"],
            ),
            (
                FetchError::Download("dns".into()),
                vec!["download failed", "dns"],
            ),
            (
                FetchError::TooLarge {
                    limit: 42,
                    stage: "download",
                },
                vec!["42", "download"],
            ),
            (
                FetchError::Decompress("bad magic".into()),
                vec!["bzip2", "bad magic"],
            ),
            (
                FetchError::DigestMismatch {
                    expected: "aa".into(),
                    actual: "bb".into(),
                },
                vec!["aa", "bb", "nothing was cached"],
            ),
            (FetchError::Write("enospc".into()), vec!["enospc"]),
            (
                FetchError::OverrideNotOurs {
                    path: PathBuf::from("/opt/theirs.so"),
                },
                vec!["/opt/theirs.so", "CERULION_OPENH264_BLOB", "manage"],
            ),
            (
                FetchError::InstalledButUnloadable {
                    path: PathBuf::from("/c/blob.dylib"),
                    why: "code signature".into(),
                },
                vec!["/c/blob.dylib", "code signature", "cannot load"],
            ),
        ];
        for (e, must_contain) in cases {
            let rendered = e.to_string();
            for needle in must_contain {
                assert!(
                    rendered.contains(needle),
                    "{e:?} must name {needle:?}, got: {rendered}"
                );
            }
        }
    }

    /// bzip2 round-trip through the REAL decoder, against a hand-built stream.
    ///
    /// The fixture is bzip2 produced by the system `bzip2`, so this pins that our
    /// decoder reads what Cisco's CDN actually serves rather than what our own
    /// encoder would produce (we have no encoder).
    #[test]
    fn the_bzip2_decoder_reads_a_real_stream_and_refuses_junk() {
        assert_eq!(
            bunzip2(BZ2_CERULION, MAX_DECOMPRESSED_BYTES).expect("a real bzip2 stream"),
            b"cerulion",
            "the decoder must read the shape Cisco's CDN serves"
        );

        // Junk is an error, not an empty success — an empty Vec would sail
        // straight into the digest check and report a confusing mismatch.
        match bunzip2(b"not bzip2 at all", MAX_DECOMPRESSED_BYTES) {
            Err(FetchError::Decompress(_)) => {}
            other => panic!("junk must be a decompress error, got {other:?}"),
        }
        // An EMPTY body is the same class and must not decompress to `Ok(vec![])`
        // — the test's own rationale above names this shape, and it is the one a
        // captive portal or a truncated response actually produces.
        match bunzip2(b"", MAX_DECOMPRESSED_BYTES) {
            Err(FetchError::Decompress(_)) => {}
            other => panic!("an empty body must be a decompress error, got {other:?}"),
        }
        // A TRUNCATED but validly-headed stream (a dropped connection).
        match bunzip2(
            &BZ2_CERULION[..BZ2_CERULION.len() - 8],
            MAX_DECOMPRESSED_BYTES,
        ) {
            Err(FetchError::Decompress(_)) => {}
            other => panic!("a truncated stream must be refused, got {other:?}"),
        }
    }

    /// THE DECOMPRESSION-BOMB CEILING, on both sides.
    ///
    /// It leads this module's docs and was previously enforced by a constant no
    /// test could reach: `bunzip2` took no limit, so exercising it needed a
    /// multi-megabyte fixture nobody was going to write. Threading the limit
    /// through makes it a two-line oracle.
    #[test]
    fn the_decompressed_ceiling_is_a_threshold_pinned_on_both_sides() {
        // "cerulion" is 8 bytes. AT the limit it decompresses…
        assert_eq!(
            bunzip2(BZ2_CERULION, 8).expect("exactly at the limit is allowed"),
            b"cerulion"
        );
        // …one byte under it is refused, naming the stage and the limit.
        match bunzip2(BZ2_CERULION, 7) {
            Err(FetchError::TooLarge { limit, stage }) => {
                assert_eq!(limit, 7);
                assert_eq!(stage, "decompressed library");
            }
            other => panic!("over the ceiling must be TooLarge, got {other:?}"),
        }
    }
}
