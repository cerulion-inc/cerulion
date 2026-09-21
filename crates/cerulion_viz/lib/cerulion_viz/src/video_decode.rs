// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side H.264 DECODER — the half the original video path deliberately left to
//! the viewer, moved here because leaving it there is what made the live view
//! unusable.
//!
//! # Why this module exists (the measurement, not a theory)
//!
//! The original video path logged each access unit VERBATIM as a [`rerun::VideoStream`] sample and
//! let the viewer decode. Rerun 0.34's native H.264 backend
//! (`re_video::decode::ffmpeg_cli`) does that by spawning an `ffmpeg` CLI process,
//! writing access units to its stdin and reading raw frames back — and it leaves
//! `-threads` unset, so libavcodec picks FRAME-LEVEL threading sized to the
//! viewer's core count. Frame threading holds a decoded frame until enough later
//! frames are in flight, so the delay is `≈ thread_count` FRAMES before anything
//! is displayed.
//!
//! Measured on the rig — a real x264 Annex-B stream (640x360, 600 access
//! units, one fed per frame period at 30 Hz) on a 16-core desk, with `-threads`
//! as the ONLY variable:
//!
//! | `-threads` | access units in before frame 0 came out | steady-state p50 |
//! |---:|---:|---:|
//! | unset (rerun's own args ⇒ auto ⇒ 16) | 18 | **567 ms** |
//! | 8 | 10 | 301 ms |
//! | 4 | 6 | 167 ms |
//! | 1 | 3 | 68 ms |
//! | in-process, this module | **1** | **0.5 ms** |
//!
//! The load-bearing column is the LEFT one. "How many access units does the
//! decoder swallow before it yields the first picture" is a COUNT, so it is
//! immune to how busy the measuring desk is — and it is what the latency column
//! then follows at `count / frame-rate`. (The 720p rendition measures the same
//! shape: 18 units / 592 ms for rerun's arguments, 1 unit / 1.8 ms here.)
//!
//! **The no-flush fix corrected that last row to TWO units on a non-baseline stream.**
//! The `1` was real but was bought with a leak: the crate's default
//! flush-after-decode forced openh264 to hand back a picture it had buffered,
//! down a path that never releases the picture's reference, and after three such
//! frames the decoder ran out of pictures and reset itself mid-GOP. See
//! `StreamDecoder::new` for the full chain. The decoder now lets openh264
//! release on its own path, which costs one picture of pipeline — so the first
//! access unit after a decoder opens yields [`DecodeOutcome::NoPicture`] and
//! every one after that yields the PREVIOUS unit's picture (~34 ms at 29.5 Hz,
//! against the 567 ms this module exists to remove). A picture therefore carries
//! its OWN [`DecodedFrame::timestamp_ns`] rather than the caller's.
//!
//! Two things follow. The lag is REAL and large (half a second at 30 Hz, and it
//! gets WORSE on a beefier desk, since the delay tracks core count). And it is not
//! reachable from here: rerun spawns that process with those arguments, and a
//! stock `rerun` viewer is the one the user runs.
//!
//! So the desk decodes. One access unit in, one frame out (one unit behind, per
//! the note above), no process boundary, no frame-threading delay — and
//! the result is logged as a plain
//! [`rerun::Image`], which the viewer REPLACES on arrival instead of scheduling
//! onto a media timeline. That is the "latest frame now" semantics a live robot
//! camera wants, and it is exactly what the JPEG route already had (the route
//! that measured as visibly better on the same link).
//!
//! # What is given up, stated plainly
//!
//! A `VideoStream` entity can be SCRUBBED in the viewer; a sequence of `Image`
//! logs cannot be re-decoded from an arbitrary point. That loss is nil in
//! practice today because `cerulion-vizd` hosts a LIVE-ONLY proxy
//! (`drop_temporal_history`, `MemoryLimit::ZERO` — see
//! `cerulion_vizd::host::server_options`), so there is no retained history to
//! scrub. When a scrubbing surface arrives it belongs to a recording path, not to
//! the live proxy.
//!
//! # Decoder choice, and why it is Cisco's binary rather than our own build
//!
//! [`openh264`] — in-process, cross-platform, no external binary. The
//! alternatives were measured or excluded:
//!
//! * **A long-lived `ffmpeg` process we spawn ourselves** (`-threads 1`) measures
//!   68 ms p50 / 3 frames of pipeline — 8x better than today but still 2 frames
//!   of lag, and it KEEPS the user-installed-`ffmpeg` dependency this module
//!   exists to remove; it drops that dependency for the camera path entirely.
//! * **Platform decoders** (VideoToolbox / VA-API) would be the lowest-power
//!   option but need per-OS code for a path that must work on an unseen robot's
//!   stream on any desk.
//!
//! **The shipped build DLOPENs Cisco's own distributed OpenH264 binary; it does
//! not compile the decoder in.** That is a licensing posture, not a technical
//! preference: Cisco's AVC patent grant covers the binaries CISCO distributes, so
//! it survives only while Cisco is the distributor — the Firefox model. Building
//! the same source into our binary would make us the distributor and put the
//! patent obligation on us. So `openh264-sys2`'s `libloading` feature is what
//! ships, and [`OpenH264API::from_blob_path`] SHA-256s the file against the
//! crate's checked-in list of known-Cisco hashes before loading it: a file that is
//! not a Cisco release is REFUSED, which is what makes "we only ever run Cisco's
//! bits" enforced rather than asserted.
//!
//! Fetching that binary is handled separately (a silent, checksum-pinned, cached background
//! download on first need — no visible install step). Until it lands, and whenever
//! the fetch has not happened yet, [`VideoDecoders`] reports
//! [`DecodeOutcome::DecoderUnavailable`] and the caller falls back to handing the
//! access unit to the viewer as the original path did — laggy, but rendering, and LOUD about
//! why. See `resolve_backend`.
//!
//! The `decoder-from-source` Cargo feature compiles the decoder in instead
//! (`openh264/source`). It exists so THIS CRATE'S TESTS and CI decode hermetically
//! with no network and no cached blob; it is **not** a shipping configuration, and
//! a build that enables it says so at runtime (see `resolve_backend`).
//!
//! `openh264`'s error-concealment default is the one this module wants and keeps:
//! OFF, so a frame whose references are missing is an ERROR rather than a
//! silently corrupt picture. Its flush-after-decode default is NOT — see
//! `StreamDecoder::new`, which turns it off.
//!
//! # One decoder per SUB-STREAM, never per topic
//!
//! An H.264 decoder cannot survive a resolution change mid-stream, and a live
//! camera can interleave 360p and 720p on ONE topic. [`crate::video::VideoDemux`] already
//! answers "which rendition is this access unit?"; this module keys its decoders
//! on that same [`StreamKey`], so each rendition gets its own decoder and neither
//! is ever fed the other's frames.
//!
//! The decoders live HERE rather than inside `VideoDemux` because the demux is a
//! pure, `Clone`-able decision machine (that is what makes its whole oracle suite
//! possible) and an `openh264::Decoder` is neither pure nor `Clone`.
//!
//! # A stall resumes at the next IDR, by construction
//!
//! P-frames chain off references. When a gap swallows one, every dependent frame
//! is undecodable — and with error concealment off, `openh264` says so instead of
//! emitting a corrupt picture. Those failures are counted and flood-latched (one
//! loud line per regime, not one per frame at 30 Hz), and decoding resumes on its
//! own at the next IDR. Nothing here has to detect the stall or hunt for the
//! resume point; refusing to render a frame that cannot be decoded IS the "skip to the
//! freshest decodable point" behaviour.

use std::collections::BTreeMap;
use std::path::PathBuf;

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use openh264::OpenH264API;

use crate::video::StreamKey;

// ────────────────────────────────────────────────────────────────────────────
// Backend resolution — Cisco's binary, or an explicit refusal
// ────────────────────────────────────────────────────────────────────────────

/// Environment override for where Cisco's OpenH264 binary is cached.
///
/// The escape hatch for an operator who has the binary somewhere else (an
/// air-gapped desk, a distro package, a shared mount).
///
/// **Pointing this at a file that is not a Cisco release does not break video —
/// it disables LOCAL decoding.** `resolve_backend` tries to LOAD the file, so a
/// junk / truncated / wrong-architecture path is refused there, reported once, and
/// the desk falls back to letting the viewer decode (slow, but rendering). It is
/// never a black pane, and it is never silent.
pub const BLOB_PATH_ENV: &str = "CERULION_OPENH264_BLOB";

/// The OpenH264 release this build's checksum list pins.
///
/// Not a free choice — `openh264-sys2` embeds the SHA-256s it will accept, so the
/// file named here has to be one of them or [`OpenH264API::from_blob_path`]
/// refuses it. Bumping the crate is what bumps this.
pub const CISCO_BLOB_VERSION: &str = "2.6.0";

/// Cisco's OWN filename for this platform's release, or `None` on a
/// platform/architecture pair this table does not cover (where the viewer-decodes
/// fallback is the only path).
///
/// The names are Cisco's, verbatim, so a file fetched from their release URL can
/// be cached under the name it arrived with. **What that buys is agreement with
/// the FETCHER, not with the checksum**: [`OpenH264API::from_blob_path`] hashes
/// the file's CONTENT and ignores its name entirely, so a correct blob under any
/// other name would load fine — it just would not be found, because this function
/// is what decides where to look. The coupling is therefore this table ↔ whatever
/// the fetcher writes to disk, and the two have to be changed together.
///
/// Note Windows drops the `lib` prefix and Cisco publishes it, so the `None` arm
/// is NOT "the desktop platforms we skipped" — it is genuinely uncovered pairs
/// (Android, the BSDs, a 32-bit ARM Mac). Cisco also ships `.7.so` and `.8.so`
/// Linux variants and BOTH are in the checksum list, so naming the `.8` line here
/// is purely a lookup choice: a cached `.7` file is a perfectly loadable Cisco
/// release that this function simply never looks for.
pub fn cisco_blob_filename() -> Option<&'static str> {
    cisco_blob_filename_for(std::env::consts::OS, std::env::consts::ARCH)
}

/// The PURE `(os, arch) -> Cisco filename` table behind [`cisco_blob_filename`].
///
/// Split out so the WHOLE matrix is reachable from a test on one machine — the
/// fetcher keys its digest table on these exact names, and a
/// cross-check that can only see the host platform would leave every other row
/// unverified.
pub fn cisco_blob_filename_for(os: &str, arch: &str) -> Option<&'static str> {
    Some(match (os, arch) {
        ("macos", "aarch64") => "libopenh264-2.6.0-mac-arm64.dylib",
        ("macos", "x86_64") => "libopenh264-2.6.0-mac-x64.dylib",
        ("linux", "x86_64") => "libopenh264-2.6.0-linux64.8.so",
        ("linux", "aarch64") => "libopenh264-2.6.0-linux-arm64.8.so",
        ("linux", "arm") => "libopenh264-2.6.0-linux-arm.8.so",
        ("linux", "x86") => "libopenh264-2.6.0-linux32.8.so",
        ("windows", "x86_64") => "openh264-2.6.0-win64.dll",
        ("windows", "x86") => "openh264-2.6.0-win32.dll",
        _ => return None,
    })
}

/// Where the fetcher caches the binary: `$HOME/.cerulion/openh264/<name>`,
/// or [`BLOB_PATH_ENV`] when set.
///
/// `None` when neither a home directory nor an override is available, or when
/// Cisco publishes nothing for this platform — all three are "there is no blob to
/// look for", which the caller reports as unavailability rather than an error.
pub fn cisco_blob_path() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os(BLOB_PATH_ENV) {
        return Some(PathBuf::from(p));
    }
    let home = std::env::var_os("HOME")?;
    Some(
        PathBuf::from(home)
            .join(".cerulion")
            .join("openh264")
            .join(cisco_blob_filename()?),
    )
}

/// Which decoder this build can actually use.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Backend {
    /// Cisco's binary, loaded from this path and checksum-verified.
    CiscoBlob(PathBuf),
    /// Compiled in via the `decoder-from-source` feature — tests and CI only.
    CompiledIn,
    /// No decoder. Carries the operator-facing reason.
    Unavailable(String),
}

/// Resolve the decoder backend ONCE per pool.
///
/// Order is deliberate: the `decoder-from-source` build wins outright, because it
/// exists so a hermetic test run decodes without a cached blob and a fallback
/// there would silently test the wrong path.
fn resolve_backend() -> Backend {
    #[cfg(feature = "decoder-from-source")]
    {
        tracing::warn!(
            "cerulion_viz: this build COMPILED the H.264 decoder in (the \
             `decoder-from-source` feature). That is a test/CI configuration — a \
             shipping build dlopens Cisco's distributed binary instead, because \
             Cisco's AVC patent grant covers the binaries THEY distribute."
        );
        Backend::CompiledIn
    }
    #[cfg(not(feature = "decoder-from-source"))]
    {
        let Some(path) = cisco_blob_path() else {
            return Backend::Unavailable(format!(
                "no cache location for Cisco's OpenH264 {CISCO_BLOB_VERSION} binary on this \
                 platform (set {BLOB_PATH_ENV} to point at one)"
            ));
        };
        // LOAD it, do not merely stat it. A file that EXISTS but cannot be loaded
        // — a truncated or half-written fetch, a blob for another architecture, a
        // junk path in the env override, anything whose SHA-256 is not a known
        // Cisco release — is exactly as un-decodable as an absent one, and the
        // ONLY safe answer for both is the viewer-decodes fallback.
        //
        // Gating on `exists()` inverted that: the unloadable case reported a
        // usable backend, every later access unit failed at decoder creation, and
        // the sink rendered NOTHING — a black pane forever, on the state a
        // partially-completed fetch leaves behind, while an ABSENT blob rendered
        // fine. The load result is the verdict.
        // The handle is dropped here: it is a LOADABILITY probe. It cannot be kept
        // and reused, because `OpenH264API` is not `Clone` and
        // `Decoder::with_api_config` consumes one per decoder — see `api_for`'s
        // note on the cost that leaves.
        let loaded = OpenH264API::from_blob_path(&path)
            .map(|_probe| ())
            .map_err(|e| e.to_string());
        let exists = path.exists();
        classify_blob_load(path, loaded, exists)
    }
}

/// PURE: turn a blob path plus the outcome of trying to LOAD it into a backend.
///
/// Split out of `resolve_backend` so the unloadable-blob rule is testable in every
/// build. The shipping arm is `#[cfg]`-ed out of this crate's own test build
/// (which compiles the decoder in so tests decode hermetically), so a decision
/// left inline there would be pinned by nothing at all — which is how the
/// `exists()` version shipped.
///
/// The rule: **loadability decides, not presence.** `Ok` is the only path to a
/// usable backend; every failure — absent, truncated, wrong architecture, a hash
/// that is not a known Cisco release, a junk env override — routes to
/// [`Backend::Unavailable`], whose one consequence is the viewer-decodes
/// fallback. `exists` only chooses which of the two messages an operator gets,
/// because "not fetched yet" and "fetched but broken" want different next steps.
///
/// `pub` rather than `pub(crate)` because its only SHIPPING caller sits behind
/// `#[cfg(not(feature = "decoder-from-source"))]`, so in this crate's own test
/// build — which enables that feature — it would otherwise be dead code.
pub fn classify_blob_load(path: PathBuf, loaded: Result<(), String>, exists: bool) -> Backend {
    match loaded {
        Ok(()) => Backend::CiscoBlob(path),
        Err(_) if !exists => Backend::Unavailable(format!(
            "Cisco's OpenH264 {CISCO_BLOB_VERSION} binary is not cached at {}",
            path.display()
        )),
        Err(e) => Backend::Unavailable(format!(
            "Cisco's OpenH264 binary at {} is present but could NOT be loaded ({e}) — a file that \
             is not a known Cisco release is refused on purpose. A truncated or half-written \
             download is the usual cause; delete it and let it be re-fetched.",
            path.display()
        )),
    }
}

#[cfg(test)]
mod backend_tests {
    use super::*;

    /// THE UNLOADABLE-BLOB ORACLE: a blob that is PRESENT but unloadable must reach
    /// the fallback, exactly like an absent one.
    ///
    /// A rule gated on `exists()` reports a usable
    /// backend for this case, every access unit then fails at decoder creation, and the sink
    /// renders NOTHING — a black pane forever, on precisely the state a
    /// partially-completed fetch leaves behind, while an ABSENT blob renders
    /// fine through the fallback. The inversion is what makes it the headline.
    #[test]
    fn a_present_but_unloadable_blob_is_unavailable_not_usable() {
        let path = PathBuf::from("/tmp/not-a-cisco-release.so");

        let present_broken =
            classify_blob_load(path.clone(), Err("InvalidHash(deadbeef)".into()), true);
        match &present_broken {
            Backend::Unavailable(why) => {
                assert!(
                    why.contains("present but could NOT be loaded"),
                    "the message must distinguish broken from absent: {why}"
                );
                assert!(why.contains("InvalidHash"), "carry the cause: {why}");
                assert!(
                    why.contains(&path.display().to_string()),
                    "name the path: {why}"
                );
            }
            other => panic!("a present-but-unloadable blob must be Unavailable, got {other:?}"),
        }

        // The absent case is ALSO Unavailable, with its own message — and the two
        // being distinguishable is the point of keeping `exists` at all.
        let absent = classify_blob_load(path.clone(), Err("No such file".into()), false);
        match &absent {
            Backend::Unavailable(why) => assert!(
                why.contains("is not cached at") && !why.contains("could NOT be loaded"),
                "an absent blob gets the not-cached message: {why}"
            ),
            other => panic!("an absent blob must be Unavailable, got {other:?}"),
        }
        assert_ne!(
            present_broken, absent,
            "broken and absent must not collapse to one message"
        );

        // ANTI-TAUTOLOGY: a blob that LOADS is usable, so the arms above are not
        // satisfied by a function that refuses everything.
        assert_eq!(
            classify_blob_load(path.clone(), Ok(()), true),
            Backend::CiscoBlob(path),
            "a loadable blob is the one path to a usable backend"
        );
    }

    /// `exists` chooses the MESSAGE and never the verdict — the property that
    /// makes a stat race harmless (a file deleted between the load attempt and the
    /// check still yields a fallback, just the other wording).
    #[test]
    fn existence_only_selects_the_message_never_the_verdict() {
        let path = PathBuf::from("/tmp/race.so");
        for exists in [true, false] {
            assert!(
                matches!(
                    classify_blob_load(path.clone(), Err("boom".into()), exists),
                    Backend::Unavailable(_)
                ),
                "a load failure is Unavailable whatever `exists` says (exists={exists})"
            );
            assert!(
                matches!(
                    classify_blob_load(path.clone(), Ok(()), exists),
                    Backend::CiscoBlob(_)
                ),
                "a load success is usable whatever `exists` says (exists={exists})"
            );
        }
    }
}

/// Build the decoder API for a resolved backend.
///
/// The blob arm goes through the CHECKSUM-verifying constructor, never the
/// `_unchecked` one: refusing a file that is not a known Cisco release is what
/// keeps the patent posture enforced rather than assumed.
///
/// **Cost this leaves, stated plainly:** the checksum is re-verified once per
/// RENDITION, because `OpenH264API` is not `Clone` and `Decoder::with_api_config`
/// consumes one, so the probe `resolve_backend` ran cannot be handed on. That is
/// one ~1 MB read + SHA-256 at each sub-stream's FIRST access unit (one or two per
/// camera), never per frame. Skipping it via `from_blob_path_unchecked` would
/// remove the very check the licensing posture rests on, so it is not skipped.
fn api_for(backend: &Backend) -> Result<OpenH264API, String> {
    match backend {
        #[cfg(not(feature = "decoder-from-source"))]
        Backend::CiscoBlob(path) => OpenH264API::from_blob_path(path).map_err(|e| {
            format!(
                "Cisco's OpenH264 binary at {} could not be loaded ({e}) — a file that is not a \
                 known Cisco release is refused on purpose",
                path.display()
            )
        }),
        // Unreachable rather than unsupported: `resolve_backend` returns
        // `CompiledIn` outright under this feature, so no blob backend is ever
        // constructed here. Kept as a loud arm rather than a panic.
        #[cfg(feature = "decoder-from-source")]
        Backend::CiscoBlob(_) => Err(
            "this build compiled the decoder in, so it never resolves a Cisco blob backend"
                .to_string(),
        ),
        #[cfg(feature = "decoder-from-source")]
        Backend::CompiledIn => Ok(OpenH264API::from_source()),
        #[cfg(not(feature = "decoder-from-source"))]
        Backend::CompiledIn => Err("this build did not compile the decoder in".to_string()),
        Backend::Unavailable(why) => Err(why.clone()),
    }
}

/// One decoded picture, ready to hand to [`rerun::Image`].
///
/// RGB8 rather than the decoder's native planar YUV: the repo's existing
/// [`crate::archetype::log_raw_image`] takes RGB24 directly, and the conversion
/// measured **0.4 ms** on the rig — 0.07 % of the 567 ms this module exists to
/// remove.
///
/// # The colour conversion is FIXED, not stream-derived — a known limitation
///
/// `openh264` 0.9.7's `write_rgb8` applies **BT.601 limited-range** coefficients
/// unconditionally: it never reads the SPS's VUI (`colour_primaries`,
/// `matrix_coefficients`, `video_full_range_flag`), so a stream that declares
/// anything else is converted with the wrong matrix. BT.709 content — which
/// includes the Go2's 720p rendition — therefore renders with a real hue and
/// saturation error. It is a colour error, not a structural one: the picture,
/// its geometry and its timing are all correct, which is why the decode path
/// is enabled despite it. The colour matrix is not corrected.
///
/// **The grey-ramp fixture cannot catch this class** and must not be read as
/// evidence against it: a neutral grey has `U == V == 128`, where every matrix
/// agrees by construction, so a chroma-plane oracle needs SATURATED colour (see
/// `tests/video_decode_test.rs`).
///
/// Handing rerun planar YUV instead would save the 0.4 ms and half the bytes, and
/// is where a real fix would go — `PixelFormat` carries the range, so the matrix
/// would stop being ours to guess. The bandwidth (a 720p frame is 2.7 MB,
/// ~80 MB/s at 29 Hz over the LOOPBACK gRPC hop) is the other reason to want it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedFrame {
    /// Picture width in pixels.
    pub width: u32,
    /// Picture height in pixels.
    pub height: u32,
    /// Tightly packed RGB8, `width * height * 3` bytes.
    pub rgb: Vec<u8>,
    /// The wire timestamp of the access unit THIS picture came from — which is
    /// not the one just handed to [`VideoDecoders::decode`].
    ///
    /// Openh264 holds a picture for one call on the streams this module
    /// sees (see `StreamDecoder::new`), so stamping the rendered image with the
    /// CALLER's current timestamp would label every frame one frame-period newer
    /// than it is. The decoder carries each access unit's own stamp through its
    /// pipeline and hands it back here.
    pub timestamp_ns: u64,
}

/// What one access unit produced.
///
/// The picture rides INSIDE the `Frame` arm rather than beside the outcome in a
/// tuple, so "decoded but no picture" and "failed but here is a picture" are
/// unrepresentable rather than merely never constructed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeOutcome {
    /// A picture came out.
    Frame(DecodedFrame),
    /// The decoder accepted the access unit and produced nothing — a
    /// parameter-set-only message (see the residual documented on
    /// [`crate::video`]), or, with flushing off, the FIRST unit a freshly-opened
    /// decoder sees on a stream openh264 buffers by one picture (see
    /// `StreamDecoder::new`). Not a failure, so it neither counts nor logs.
    NoPicture,
    /// The decoder refused. With error concealment off this is what a missing
    /// reference looks like, so it is the EXPECTED steady state across a stall
    /// until the next IDR — counted always, logged once per regime.
    Failed,
    /// There is NO local decoder for this stream — Cisco's binary is not cached
    /// yet (see the module docs), it could not be loaded, or this
    /// rendition's own decoder could not be created.
    ///
    /// Distinct from [`DecodeOutcome::Failed`] on purpose, and the distinction is
    /// the whole of the unloadable-blob rule: `Failed` is about ONE access unit and
    /// the next keyframe recovers from it, while this is a standing condition that
    /// no frame recovers from. So the caller must fall back to letting the VIEWER
    /// decode — slow, but rendering — instead of rendering nothing.
    DecoderUnavailable,
}

/// How many un-emitted access-unit timestamps [`StreamDecoder::pending`] may
/// hold before the oldest is dropped.
///
/// openh264's reordering list (`m_sPictInfoList`) is 16 entries, so a decoder
/// cannot legitimately owe more than that; the cap only bounds memory against a
/// stream that somehow never yields a picture. It is NOT a correctness knob —
/// on the B-frame-free streams this module supports the queue sits at one.
const MAX_PENDING_TIMESTAMPS: usize = 16;

/// Per-sub-stream decoder state.
struct StreamDecoder {
    decoder: openh264::decoder::Decoder,
    /// Wire timestamps of access units FED but whose picture has not come out
    /// yet, oldest first.
    ///
    /// openh264 holds a picture for one call on the streams this module sees
    /// (see `StreamDecoder::new`), so the picture a `decode` returns belongs to
    /// an EARLIER access unit than the one just handed in. Stamping it with the
    /// caller's current timestamp would label every frame one frame-period newer
    /// than it is; popping the front of this queue labels it with its own.
    ///
    /// FIFO is exact here because output order equals input order: this module
    /// scopes out B-frames (see the [`crate::video`] docs — rerun 0.34 carries no
    /// decode timestamp), and both of openh264's release paths pick the oldest
    /// held picture on such a stream — `ReleaseBufferedReadyPictureNoReorder` by
    /// `uiDecodingTimeStamp` and `…Reorder` by POC, which agree when no frame is
    /// reordered.
    pending: std::collections::VecDeque<u64>,
    /// Reused RGB scratch: the DECODE writes into this buffer, which is resized
    /// once per sub-stream and then reused.
    ///
    /// It does not make the path allocation-free. `decode` hands the caller an
    /// OWNED [`DecodedFrame`], so it clones this buffer once per picture — the
    /// scratch saves the conversion target, not the hand-off. Removing that clone
    /// means returning a borrow of the pool, which the caller cannot hold while
    /// the pool is mutably borrowed for the next unit; it is a real option, and it
    /// is not what this does.
    rgb: Vec<u8>,
    /// Pictures this sub-stream has produced.
    frames: u64,
    /// Flood suppression for the refusal regime, AND the unconditional refusal
    /// total: the shared latch already keeps a `total_failures` that no recovery
    /// resets (Principle #3), so a second counter here would be a copy that can
    /// drift. Flood suppression proper: one loud line at the head, a
    /// running count on the repeats, a loud re-announcement at each decade, and
    /// one recovery line when pictures come back. A stalled 30 Hz stream would
    /// otherwise emit 30 identical lines a second for as long as it is stalled —
    /// the disk-fill class.
    latch: FailureRegimeLatch,
}

impl StreamDecoder {
    /// # The decoder must NOT flush after each decode
    ///
    /// `DecoderConfig::new()` defaults to [`openh264::decoder::Flush::Flush`],
    /// and that default DESTROYS a live high-profile robot camera stream — three pictures per GOP
    /// decode and the remaining twenty-seven are refused. The mechanism, read out
    /// of the vendored OpenH264 source and then measured on captured robot bytes:
    ///
    /// 1. The Go2 declares `profile_idc` 100 (High), so openh264 takes its
    ///    NON-BASELINE path (`m_bIsBaseline` is true only for 66/83) and BUFFERS
    ///    each decoded picture in `m_sPictInfoList`, clearing `iBufferStatus`.
    /// 2. `ReorderPicturesInDisplay` then tries to release it inline, gated on
    ///    `iMinPOC - iLastWrittenPOC <= 1`. The Go2 uses `pic_order_cnt_type` 0
    ///    and steps POC by **2** (measured: 0, 2, 4, 6, …), so the gate NEVER
    ///    passes and the picture stays held.
    /// 3. With `iBufferStatus` at 0 the crate falls back to `FlushFrame`, which
    ///    calls `ReleaseBufferedReadyPictureNoReorder(pCtx = NULL, …)`. That
    ///    function decrements the picture's `iRefCount` only `if (pCtx ||
    ///    m_pPicBuff)`, and `m_pPicBuff` is assigned ONLY on openh264's threaded
    ///    path (`ThreadDecodeFrameInternal`) — this decoder is single-threaded, so
    ///    it is NULL for the process's whole life. The picture is handed to the
    ///    caller with its reference **never released**.
    /// 4. So every access unit permanently consumes one slot of a picture buffer
    ///    sized `pSps->iNumRefFrames + 2` = 1 + 2 = **3**. On the fourth,
    ///    `PrefetchPic` returns NULL, openh264 sets `dsOutOfMemory`, and
    ///    `DecodeFrame2WithCtx` calls `ResetDecoder()` — which re-initialises the
    ///    context and DISCARDS the parameter sets. Every later access unit then
    ///    fails `dsNoParamSets` in a few microseconds until the next IDR carries
    ///    SPS/PPS again.
    ///
    /// That is precisely the reported symptom: a fixed GOP phase, both
    /// renditions, `Native:16384` then a collapse to 5-12 µs, recovering at each
    /// keyframe.
    ///
    /// `NoFlush` keeps the release on openh264's own path, where `pCtx` is
    /// non-NULL and the reference IS dropped, so nothing leaks. **The cost is one
    /// picture of pipeline**: the buffered picture comes out on the NEXT
    /// `decode`, so a decoder returns [`DecodeOutcome::NoPicture`] once after it
    /// opens and is one access unit behind from then on (~34 ms at the Go2's
    /// 29.5 Hz). That delay is openh264's own behaviour for a stream it cannot
    /// release inline — the flush was not avoiding it so much as papering over it
    /// with a leak — and it is measured on the grey-ramp fixture too, so it is
    /// not Go2-specific. Against the 567 ms of viewer-side frame threading this
    /// module exists to remove, and against losing 27 of every 30 frames, it is
    /// the right trade.
    fn new(api: OpenH264API) -> Result<Self, openh264::Error> {
        Ok(Self {
            decoder: openh264::decoder::Decoder::with_api_config(
                api,
                openh264::decoder::DecoderConfig::new()
                    .flush_after_decode(openh264::decoder::Flush::NoFlush),
            )?,
            pending: std::collections::VecDeque::new(),
            rgb: Vec::new(),
            frames: 0,
            latch: FailureRegimeLatch::new(),
        })
    }
}

/// A rendition whose decoder could NOT be created.
///
/// It exists so the condition is COUNTABLE. A bare
/// `()`-valued marker in a map keyed apart from `streams`, with both public
/// counters reading `streams`, would let a rendition that had handed ten thousand access
/// units to this arm report `frames_decoded == 0` AND `decode_failures == 0`,
/// which is byte-for-byte what a topic carrying no video at all reports, with a
/// single `error!` at the very start as the only trace. An operator arriving after
/// that line would have no way to tell the two apart.
struct FailedStream {
    /// Access units handed to the VIEWER because this rendition has no local
    /// decoder. Surfaced by [`VideoDecoders::fallback_units`].
    fallback_units: u64,
    /// Flood suppression for the standing condition, and its unconditional total.
    /// Without it the arm spoke once and then never again, however long it lasted.
    latch: FailureRegimeLatch,
}

impl FailedStream {
    /// A record whose FIRST unit has already been reported by the caller (which
    /// owns the reason text), with the regime opened so the running total the
    /// decade re-announcements carry counts that unit.
    fn opened() -> Self {
        let mut failed = Self {
            fallback_units: 1,
            latch: FailureRegimeLatch::new(),
        };
        let _ = failed.latch.on_failure();
        failed
    }
}

/// The per-input pool of sub-stream decoders — the stateful half of the desk-side
/// video path, owned by [`crate::sink::SinkState`] beside the demux that keys it.
pub struct VideoDecoders {
    /// Resolved ONCE — a per-frame filesystem probe for a blob that is not there
    /// would cost a syscall per access unit on exactly the desks that have no
    /// decoder.
    backend: Backend,
    /// Keyed `(input, rendition)`: one decoder per sub-stream, never per topic.
    streams: BTreeMap<(String, StreamKey), StreamDecoder>,
    /// Renditions whose decoder could not be CREATED. Keyed exactly like
    /// `streams` and mutually exclusive with it.
    create_failed: BTreeMap<(String, StreamKey), FailedStream>,
    /// The value of [`crate::openh264_fetch::cache_generation`] this
    /// pool's `backend` was resolved at, or `None` for a pool that must NEVER
    /// re-resolve.
    ///
    /// A COUNTER, not a flag: a pool built after a fetch already succeeded must
    /// not re-resolve on its first unit, and comparing against the value recorded
    /// at construction answers that without a filesystem probe per frame.
    ///
    /// `None` is for the HAND-CHOSEN backends the test seams install. The counter
    /// is process-global (that is how one completed fetch reaches every pool in
    /// the daemon), so without a frozen arm a test that announces a fetch would
    /// silently re-resolve every other test's fixture — a fallback pin would stop
    /// exercising the fallback and pass for the wrong reason.
    backend_generation: Option<u64>,
    /// How many times `adopt_fetched_blob` actually RE-RESOLVED.
    /// Observable via `resolves_for_test` — see that accessor for why it exists.
    resolves: u64,
}

impl std::fmt::Debug for VideoDecoders {
    /// Hand-written because `openh264::decoder::Decoder` is not `Debug`, and
    /// `SinkState` is. Reports the shape (which sub-streams, how they are doing)
    /// rather than the opaque decoder handles.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut m = f.debug_struct("VideoDecoders");
        for ((input, key), s) in &self.streams {
            m.field(
                &format!("{input}@{}", key.segment()),
                &format_args!("frames={} failures={}", s.frames, s.latch.total_failures()),
            );
        }
        for ((input, key), f) in &self.create_failed {
            m.field(
                &format!("{input}@{} (no decoder)", key.segment()),
                &format_args!("fallback_units={}", f.fallback_units),
            );
        }
        m.finish()
    }
}

impl Default for VideoDecoders {
    fn default() -> Self {
        Self::new()
    }
}

impl VideoDecoders {
    /// A pool that has decoded nothing, with its backend resolved.
    pub fn new() -> Self {
        Self {
            backend: resolve_backend(),
            streams: BTreeMap::new(),
            create_failed: BTreeMap::new(),
            backend_generation: Some(crate::openh264_fetch::cache_generation()),
            resolves: 0,
        }
    }

    /// How many times this pool RE-RESOLVED its backend because the
    /// cache generation moved (test seam).
    ///
    /// The observable that distinguishes a generation COUNTER from a flag: a pool
    /// built after a fetch already landed must re-resolve ZERO times, and one that
    /// was running when the blob arrived exactly once. Without it both designs
    /// look identical from the outside.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn resolves_for_test(&self) -> u64 {
        self.resolves
    }

    /// A pool that resolved to NO decoder, for driving the fallback path.
    ///
    /// A seam rather than a fixture because the shipping condition — Cisco's
    /// binary absent — is unreachable in a test build: this crate's tests enable
    /// `decoder-from-source` so they can decode hermetically, which is exactly the
    /// build where `resolve_backend` never returns `Unavailable`. Without this the
    /// fallback would ship untested on the very configuration that ships.
    pub fn unavailable_for_test(reason: &str) -> Self {
        Self {
            backend: Backend::Unavailable(reason.to_string()),
            streams: BTreeMap::new(),
            create_failed: BTreeMap::new(),
            // FROZEN — see the field's docs.
            backend_generation: None,
            resolves: 0,
        }
    }

    /// A pool that has NO decoder **yet** — one that will adopt a blob
    /// the moment a fetch announces one.
    ///
    /// Distinct from [`VideoDecoders::unavailable_for_test`] because the two model
    /// different desks, and collapsing them would break both. That one is a FROZEN
    /// "this desk cannot decode, full stop" fixture, which is what the
    /// viewer-fallback pins need; this one tracks the process-global cache
    /// generation, which is what the pick-up path needs. A single seam doing both
    /// would let one test's announced fetch silently re-resolve another test's
    /// fallback fixture, and the fallback pin would then pass while exercising
    /// nothing.
    pub fn unavailable_pending_fetch_for_test(reason: &str) -> Self {
        Self {
            backend: Backend::Unavailable(reason.to_string()),
            streams: BTreeMap::new(),
            create_failed: BTreeMap::new(),
            backend_generation: Some(crate::openh264_fetch::cache_generation()),
            resolves: 0,
        }
    }

    /// A pool with a HAND-CHOSEN backend, for driving arms that no build reaches
    /// on its own.
    ///
    /// The one that matters is `Backend::CiscoBlob`: under `decoder-from-source`
    /// (which this crate's tests enable so they decode hermetically) `api_for`
    /// answers a blob backend with `Err`, so handing the pool one drives the
    /// CREATION-FAILURE fallback — the arm that hands the access unit to the
    /// viewer because this rendition could not get a decoder.
    ///
    /// Without this seam that arm was executed by NOTHING: `resolve_backend`
    /// never yields a blob backend in a test build, so a `panic!()` planted in
    /// both creation-failure arms left the entire suite green. A carve-out that
    /// no test can kill is a carve-out nobody is checking.
    pub fn with_backend_for_test(backend: Backend) -> Self {
        Self {
            backend,
            streams: BTreeMap::new(),
            create_failed: BTreeMap::new(),
            // FROZEN — see the field's docs.
            backend_generation: None,
            resolves: 0,
        }
    }

    /// The resolved backend (observability / test seam).
    pub fn backend(&self) -> &Backend {
        &self.backend
    }

    /// Whether this desk can decode at all.
    pub fn is_available(&self) -> bool {
        !matches!(self.backend, Backend::Unavailable(_))
    }

    /// Pick up a blob that arrived AFTER this pool resolved its backend.
    ///
    /// The backend is resolved once, deliberately (a per-frame filesystem probe
    /// on a desk with no blob costs a syscall per access unit). That is exactly
    /// what makes a fetch invisible without this: the bytes land on disk and
    /// nothing ever looks again, so the desk keeps the viewer-decodes path until
    /// vizd is restarted — which is precisely the "you have to restart it" shape
    /// this issue exists to remove.
    ///
    /// The cost is one relaxed atomic load per access unit; the re-resolve itself
    /// runs only when the generation actually MOVED, at most once per fetch.
    ///
    /// `create_failed` is cleared with it: those entries record "this rendition
    /// has no decoder", a verdict the new blob may overturn, and leaving them
    /// would keep every already-seen rendition on the fallback forever while only
    /// renditions first seen after the fetch decoded.
    fn adopt_fetched_blob(&mut self) {
        let Some(mine) = self.backend_generation else {
            return; // a hand-chosen backend keeps what it was given
        };
        let generation = crate::openh264_fetch::cache_generation();
        if generation == mine {
            return;
        }
        self.backend_generation = Some(generation);
        self.resolves += 1;
        let resolved = resolve_backend();
        let was_unavailable = matches!(self.backend, Backend::Unavailable(_));
        let recovered = was_unavailable && !matches!(resolved, Backend::Unavailable(_));
        self.backend = resolved;
        if recovered {
            // CLOSE each rendition's fallback regime rather than dropping it.
            //
            // What this restores is the recovery LINE, and only that — an earlier
            // comment here claimed it preserved the latch's unconditional counters
            // against a bare `.clear()`, which was wrong: `mem::take` destroys the
            // same `FailedStream` records, so the counters are equally gone either
            // way. What differs is that the counts are REPORTED before the record
            // dies, which is what every other consumer of this latch does when a
            // regime ends. A regime that opened loudly and closed in silence
            // leaves an operator reading the log with a standing complaint and no
            // retraction.
            for ((input, key), mut failed) in std::mem::take(&mut self.create_failed) {
                if let Some(suppressed) = failed.latch.on_success() {
                    tracing::info!(
                        input,
                        width = key.width,
                        height = key.height,
                        suppressed_count = suppressed,
                        fallback_units = failed.fallback_units,
                        "cerulion_viz: this rendition stops going to the viewer — a local \
                         decoder is available now"
                    );
                }
            }
            tracing::info!(
                "cerulion_viz: Cisco's OpenH264 binary is now cached — this desk decodes H.264 \
                 locally from the next access unit"
            );
        } else if was_unavailable {
            // A blob ARRIVED and this desk still cannot use it. The fetcher has
            // already announced success, so silence here leaves that claim
            // standing while every rendition keeps short-circuiting on a stale
            // reason string — the operator's log would say video is fast when it
            // is not.
            if let Backend::Unavailable(why) = &self.backend {
                tracing::warn!(
                    reason = %why,
                    "cerulion_viz: an OpenH264 blob arrived but this desk STILL cannot decode \
                     locally — video stays on the slow viewer-decodes path"
                );
            }
        }
    }

    /// Decode ONE access unit for `(input, key)`, creating that sub-stream's
    /// decoder on first use.
    ///
    /// The returned picture is copied out of this pool's reusable scratch, so
    /// the decoder keeps its own buffer across frames and the caller owns what it
    /// gets.
    ///
    /// **Every arm is accounted for through a PUBLIC counter, and none is
    /// silent**: a picture bumps [`VideoDecoders::frames_decoded`] (and closes any
    /// open refusal regime with one recovery line), a per-unit refusal bumps
    /// [`VideoDecoders::decode_failures`], and a unit handed to the viewer because
    /// this rendition has no decoder bumps [`VideoDecoders::fallback_units`] — each
    /// on its own flood latch, so a standing condition re-announces at each decade
    /// instead of speaking once. The caller decides what to render; this decides
    /// nothing about presentation.
    ///
    /// `timestamp_ns` is THIS access unit's wire timestamp. It is not
    /// necessarily the stamp that comes back on the picture: see
    /// [`DecodedFrame::timestamp_ns`].
    pub fn decode(
        &mut self,
        input: &str,
        key: StreamKey,
        access_unit: &[u8],
        timestamp_ns: u64,
    ) -> DecodeOutcome {
        self.adopt_fetched_blob();
        // THIS is "first need" — an access unit arrived and this desk has
        // no decoder at all. Consulting on EVERY such unit (rather than once per
        // rendition, below) is what gives a retry story: a desk that was offline
        // when its camera attached would otherwise never ask again, because the
        // rendition's `create_failed` record short-circuits before the
        // whole-desk arm is ever reached again.
        //
        // Safe at frame rate because the gate — not this call site — is what
        // bounds the work: one attempt in flight, a backoff floor between
        // failures, and a terminal settle once the blob is cached.
        //
        // Deliberately keyed on the WHOLE-DESK backend, not on `create_failed`: a
        // single rendition whose decoder could not be created on a desk that has
        // a perfectly good blob is not a fetch problem, and asking for one would
        // be noise.
        if matches!(self.backend, Backend::Unavailable(_)) {
            crate::openh264_fetch::note_decoder_needed();
        }
        let id = (input.to_string(), key);
        // A rendition already known to have no local decoder — whether this desk
        // has no blob at all or this one rendition's decoder could not be created.
        // Counted and flood-latched PER RENDITION, so the condition stays visible
        // for as long as it lasts instead of speaking at its first instant.
        if let Some(failed) = self.create_failed.get_mut(&id) {
            failed.fallback_units += 1;
            report_no_decoder_fallback(input, key, failed);
            return DecodeOutcome::DecoderUnavailable;
        }
        // The whole-desk case, on its FIRST unit for this rendition: loud, and
        // carrying the reason (the operator's next step lives in that string).
        if let Backend::Unavailable(why) = &self.backend {
            tracing::warn!(
                reason = %why,
                input,
                width = key.width,
                height = key.height,
                "cerulion_viz: no H.264 decoder on this desk, so video is being handed to the \
                 VIEWER to decode instead. That path works but is SLOW — rerun's own decoder \
                 spawns an ffmpeg whose default frame threading measured 567 ms of lag. \
                 Cerulion decodes locally once Cisco's OpenH264 binary is cached (it is \
                 fetched on demand; set {BLOB_PATH_ENV} to point at an existing copy).",
                BLOB_PATH_ENV = BLOB_PATH_ENV,
            );
            self.create_failed.insert(id, FailedStream::opened());
            return DecodeOutcome::DecoderUnavailable;
        }
        if !self.streams.contains_key(&id) {
            let api = match api_for(&self.backend) {
                Ok(api) => api,
                Err(why) => {
                    tracing::error!(
                        reason = %why,
                        input,
                        width = key.width,
                        height = key.height,
                        "cerulion_viz: the H.264 decoder library could not be loaded for this \
                         rendition, so it is being handed to the VIEWER to decode instead."
                    );
                    self.create_failed.insert(id, FailedStream::opened());
                    return DecodeOutcome::DecoderUnavailable;
                }
            };
            match StreamDecoder::new(api) {
                Ok(d) => {
                    self.streams.insert(id.clone(), d);
                    tracing::info!(
                        input,
                        width = key.width,
                        height = key.height,
                        "cerulion_viz: H.264 decoder opened for this rendition (desk-side decode)"
                    );
                }
                Err(e) => {
                    tracing::error!(
                        error = %e,
                        input,
                        width = key.width,
                        height = key.height,
                        "cerulion_viz: could not create an H.264 decoder for this rendition, so \
                         it is being handed to the VIEWER to decode instead."
                    );
                    self.create_failed.insert(id, FailedStream::opened());
                    return DecodeOutcome::DecoderUnavailable;
                }
            }
        }
        let Some(state) = self.streams.get_mut(&id) else {
            // Unreachable: the entry was just inserted under the same key.
            return DecodeOutcome::Failed;
        };

        // Record this unit's stamp BEFORE the decode: the picture that comes back
        // (if any) is the OLDEST one still owed, not this one.
        state.pending.push_back(timestamp_ns);
        while state.pending.len() > MAX_PENDING_TIMESTAMPS {
            state.pending.pop_front();
        }

        match state.decoder.decode(access_unit) {
            Ok(Some(yuv)) => {
                use openh264::formats::YUVSource as _;
                let (w, h) = yuv.dimensions();
                let need = w * h * 3;
                if state.rgb.len() != need {
                    state.rgb.resize(need, 0);
                }
                yuv.write_rgb8(&mut state.rgb);
                state.frames += 1;
                // The oldest unit still owed is the one that just came out. The
                // fallback cannot be reached while the push above runs first, and
                // stamping with the current unit is the least-wrong answer if a
                // future openh264 ever emits more pictures than it was fed.
                let stamp = state.pending.pop_front().unwrap_or(timestamp_ns);
                if let Some(suppressed) = state.latch.on_success() {
                    tracing::info!(
                        input,
                        width = key.width,
                        height = key.height,
                        suppressed_count = suppressed,
                        total_failures = state.latch.total_failures(),
                        "cerulion_viz: H.264 decoding recovered on this rendition"
                    );
                }
                DecodeOutcome::Frame(DecodedFrame {
                    width: w as u32,
                    height: h as u32,
                    rgb: state.rgb.clone(),
                    timestamp_ns: stamp,
                })
            }
            Ok(None) => DecodeOutcome::NoPicture,
            Err(e) => {
                // A refusal yields no picture, but whether the pictures openh264
                // was HOLDING survive it depends on which arm failed, and the
                // stamp queue has to follow that or it mis-labels every later
                // picture.
                //
                // `DecodeFrame2WithCtx` calls `ResetDecoder()` on exactly two
                // error arms — `dsOutOfMemory` and `dsRefListNullPtrs` — and a
                // reset re-initialises the context, so everything held is gone
                // and the stamps owed for it can never be claimed. Every OTHER
                // refusal (a missing reference, `dsNoParamSets`) leaves the
                // reordering list intact, and the picture it is holding comes out
                // on a later call — MEASURED: in
                // `a_stall_refuses_undecodable_units_and_resumes_at_the_next_keyframe`
                // the picture buffered before the orphaned unit is emitted by the
                // keyframe that follows it, so clearing unconditionally stamped
                // that picture with the keyframe's time.
                //
                // `decode_frame_no_delay` ORs the two inner return codes, so this
                // is a bit test rather than an equality.
                //
                // The NON-reset arm pops only the stamp pushed for THIS unit, not
                // the whole queue: this unit will never yield a picture, but the
                // ones openh264 was already holding still will. Both wrong rules
                // are reachable and both are pinned — clearing outright mis-stamps
                // the picture that comes out next, and keeping this unit's stamp
                // shifts every later picture by one unit for the rest of the run.
                const RESET_ARMS: i64 =
                    openh264_sys2::dsOutOfMemory as i64 | openh264_sys2::dsRefListNullPtrs as i64;
                if e.native_code() & RESET_ARMS != 0 {
                    state.pending.clear();
                } else {
                    state.pending.pop_back();
                }
                report_decode_failure(input, key, &e.to_string(), state);
                DecodeOutcome::Failed
            }
        }
    }

    /// Pictures produced for `(input, key)` so far (test / observability seam).
    pub fn frames_decoded(&self, input: &str, key: StreamKey) -> u64 {
        self.streams
            .get(&(input.to_string(), key))
            .map(|s| s.frames)
            .unwrap_or(0)
    }

    /// Access units `(input, key)` REFUSED by a live decoder — unconditional,
    /// never reset by a recovery, so it is readable whatever the log level is.
    ///
    /// A rendition that never got a decoder reports 0 here and a nonzero
    /// [`VideoDecoders::fallback_units`]: the two conditions are different (one
    /// frame could not be decoded / this stream is not being decoded here) and are
    /// deliberately not summed into one number.
    pub fn decode_failures(&self, input: &str, key: StreamKey) -> u64 {
        self.streams
            .get(&(input.to_string(), key))
            .map(|s| s.latch.total_failures())
            .unwrap_or(0)
    }

    /// Access units on `(input, key)` handed to the VIEWER because this rendition
    /// has no local decoder — unconditional, never reset.
    ///
    /// `frames_decoded == 0 && decode_failures == 0 && fallback_units == N` is the
    /// signature of a rendition that is rendering through the viewer; a topic with
    /// no video at all reports 0 on all three, so the two are distinguishable.
    pub fn fallback_units(&self, input: &str, key: StreamKey) -> u64 {
        self.create_failed
            .get(&(input.to_string(), key))
            .map(|f| f.fallback_units)
            .unwrap_or(0)
    }
}

/// Map one viewer-fallback unit onto the shared flood-suppression policy.
///
/// The FIRST unit opens the regime at its own call site (which carries the reason
/// — an absent blob and a failed decoder creation want different next steps), so
/// this reports the ones after it: quiet repeats, and a loud re-announcement at
/// each decade of the running total. A standing condition on a 30 Hz camera would
/// otherwise be a single line at the start of a run that nobody watching later
/// ever sees.
fn report_no_decoder_fallback(input: &str, key: StreamKey, failed: &mut FailedStream) {
    match failed.latch.on_failure() {
        RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(
            input,
            width = key.width,
            height = key.height,
            fallback_units = failed.fallback_units,
            total_failures = failed.latch.total_failures(),
            "cerulion_viz: this rendition still has no local H.264 decoder — every access unit is \
             being handed to the VIEWER to decode, which works but is SLOW (rerun's own decoder \
             spawns an ffmpeg whose default frame threading measured 567 ms of lag)."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            input,
            width = key.width,
            height = key.height,
            suppressed_count = suppressed,
            total_failures = failed.latch.total_failures(),
            "cerulion_viz: H.264 access unit handed to the viewer (no local decoder, repeat)"
        ),
    }
}

/// Map one refusal onto the shared flood-suppression policy's three arms.
///
/// Split out so the level mapping is one small readable function rather than a
/// third nesting level inside `decode`.
fn report_decode_failure(input: &str, key: StreamKey, error: &str, state: &mut StreamDecoder) {
    match state.latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(
            input,
            width = key.width,
            height = key.height,
            error,
            total_failures = state.latch.total_failures(),
            "cerulion_viz: an H.264 access unit could not be decoded, so this frame is NOT \
             rendered. A gap in the stream is the usual cause — every frame that depends on the \
             missing one fails too, and decoding resumes on its own at the next keyframe."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            input,
            width = key.width,
            height = key.height,
            error,
            suppressed_count = suppressed,
            total_failures = state.latch.total_failures(),
            "cerulion_viz: H.264 access unit undecodable (repeat)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            input,
            width = key.width,
            height = key.height,
            error,
            suppressed_count = suppressed,
            total_failures = total,
            "cerulion_viz: this rendition is STILL failing to decode — nothing has rendered on it \
             since the first failure"
        ),
    }
}
