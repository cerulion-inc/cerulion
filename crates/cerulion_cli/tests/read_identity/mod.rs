// SPDX-License-Identifier: AGPL-3.0-only
//! The READ-IDENTITY ORACLE: every read a graph's nodes make, logged as
//! `(consumer node, input, writer, sequence, payload hash)`, for a live run and
//! for its replay (`bag play`, plain or `--resim all`), compared per node and
//! per input, in order.
//!
//! # How a read is identified, with no runtime hook
//!
//! Every node of this workspace is a harness node, and every frame they publish
//! is a `geometry_msgs/Vector3` whose fields carry its identity:
//!
//! * `x` = the WRITER: the publishing node's `ri_writer` param. Every writer
//!   in one graph has its own nonzero value.
//! * `y` = the writer's SEQUENCE: its `tick_count` at the publish, one per fire.
//! * `z` = a payload value: a function of `(writer, seq)` on a source, and of
//!   the frame the relay read on a relay, so a relay that reads a different
//!   frame publishes a different payload downstream.
//!
//! A consumer pushes the three raw `f64` bit patterns of every frame it reads
//! onto an in-memory buffer (one `Vec` push per read, no I/O on the fire path;
//! the `det_scaffold` pattern) and writes the buffer ONCE, at its `shutdown`,
//! to a new file `$CER_RI_READS_DIR/<node id>_<pid>_<k>.reads` (`k` counts the
//! records one node library has written in its process), one line per read and
//! a closing count:
//!
//! ```text
//! <input> <x bits, 16 hex> <y bits, 16 hex> <z bits, 16 hex>
//! ...
//! end <number of read lines>
//! ```
//!
//! A node instance that read nothing writes no file (the supervisor's planning
//! build loads and tears down every node without a fire). A node with two
//! records in one run, or a record whose closing count is missing or wrong,
//! is refused by [`ReadLog::collect`]. The payload hash is
//! FNV-1a 64 over the 24 little-endian bytes of the three fields
//! ([`payload_hash`]), computed by the harness, never by the node.
//!
//! # The verdict
//!
//! [`judge`] compares a REFERENCE log (the live run) with a CANDIDATE log (its
//! resim) stream by stream, a stream being one `(node, input)` pair:
//!
//! * a stream the fixture expects that the reference lacks, or holds fewer
//!   than [`Floors::min_reads`] reads, is a HARNESS failure (`Err`): a check
//!   that compares nothing proves nothing;
//! * otherwise the first index where the two disagree is [`RowVerdict::Diverged`];
//! * agreeing streams whose lengths differ by more than the tail allowance are
//!   [`RowVerdict::Short`]: a live run is stopped by a Ctrl-C, so its last few
//!   reads may postdate the recorded trace; the allowance is
//!   `max(Floors::tail_min, live_len / Floors::tail_div)`, on the candidate's
//!   short side only;
//! * a candidate with any read past the reference's last is
//!   [`RowVerdict::Long`].
//!
//! The candidate must also have compared at least `min_reads` reads on a
//! stream for that stream to read as `Equal`; [`Verdict::compared_floor`]
//! refuses a verdict in which an exercised stream compared fewer.
//!
//! # Fixtures
//!
//! [`Fixture`] names the graph shapes the oracle runs: a slow consumer
//! that evicts live, a fan-out, a three-writer multi-publisher topic drained in
//! batches, and the two- and three-rank chain splits. [`record_live`] runs one
//! under `graph run --record` (as declared, or `--single-process`);
//! [`resim`] replays the recording with the read log armed.
//!
//! # Using it from a test binary
//!
//! ```ignore
//! mod det_scaffold;
//! mod mp_support;
//! mod read_identity;
//! mod scaffold_dir;
//! use read_identity::{judge_replayed, Fixture, Floors, Placement, ReplayMode};
//!
//! let ws = read_identity::workspace();
//! let run = read_identity::record_live(&ws, Fixture::Ranks2, Placement::Declared);
//! let replayed = read_identity::resim(&ws, &run, ReplayMode::ResimVerify, None);
//! let outcome = judge_replayed(Fixture::Ranks2, &run.live, &replayed, Floors::default()).unwrap();
//! outcome.print("ranks2", "declared/resim_verify");
//! let verdict = outcome.verdict().expect("the replay ran, it did not refuse");
//! assert!(verdict.is_equal(), "{}", verdict.render());
//! ```
//!
//! [`ReplayMode`] picks the replay: `Replay` (plain `bag play`),
//! `ResimVerify` (`--resim all --verify`, where exit 2 is a refusal the
//! outcome carries) and `ResimNeutral` (`--resim all`).
//!
//! The drivers spawn real `cerulion` processes that load node cdylibs, so a
//! binary that calls them is waived whole on macOS (iceoryx2 0.10.0 defect
//! 2034, see `cerulion_core/tests/upstream_waivers_test.rs`). The record
//! format, the judge and the workspace build are platform neutral.
//!
//! `#![allow(dead_code)]`: each binary uses part of it.
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use cerulion_cli_engine::node_cmd::{NodeCreateOptions, NodeLanguage};
use cerulion_cli_engine::{node_cmd, workspace};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::params::ParamValue;
use cerulion_core::MacroPolicy;

use crate::det_scaffold::SCHEMA;
use crate::mp_support::{self, ChildGuard};
use crate::scaffold_dir::{pin_target_dir, scaffold_target_dir, ScaffoldDir};

/// The env var a consumer's read buffer is armed by and written under.
pub const READS_DIR_ENV: &str = "CER_RI_READS_DIR";
/// A PLANTED misread, for the oracle's own mutant: `<node id>:<index>` makes
/// that node write read `index` with its writer field offset by 1000. Read at
/// the node's shutdown, so it changes the record and nothing the node did.
pub const PLANT_ENV: &str = "CER_RI_PLANT";
/// The env var the slow-does-not-block harness's `ri_fast` node writes its
/// samples under (`fast_<pid>.samples`: one `<fire real ns> <latency ns>` line
/// per fire).
pub const SAMPLES_DIR_ENV: &str = "CER_RI_SAMPLES_DIR";

// ===========================================================================
// Records.
// ===========================================================================

/// One read: which frame a consumer was served.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Read {
    pub writer: u64,
    pub seq: u64,
    pub hash: u64,
}

impl std::fmt::Display for Read {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(w={},s={},h={:016x})", self.writer, self.seq, self.hash)
    }
}

/// One stream: `(consumer node id, input name)`.
pub type StreamKey = (String, String);

/// Every read of one run, per stream, in read order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ReadLog {
    pub streams: BTreeMap<StreamKey, Vec<Read>>,
}

/// FNV-1a 64 over the 24 little-endian bytes of the three field bit patterns.
pub fn payload_hash(bits: [u64; 3]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for word in bits {
        for b in word.to_le_bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    }
    h
}

/// The read a record line's three field bit patterns describe.
pub fn read_from_bits(bits: [u64; 3]) -> Read {
    Read {
        writer: f64::from_bits(bits[0]) as u64,
        seq: f64::from_bits(bits[1]) as u64,
        hash: payload_hash(bits),
    }
}

impl ReadLog {
    /// Append node `node`'s record text (the file body a node writes) to this
    /// log. A malformed line is an error naming the line.
    pub fn add_record(&mut self, node: &str, text: &str) -> Result<(), String> {
        let lines: Vec<&str> = text.lines().collect();
        let Some((last, body)) = lines.split_last() else {
            return Err(format!("node {node}: an empty record"));
        };
        let count: Option<usize> = last.strip_prefix("end ").and_then(|n| n.parse().ok());
        if count != Some(body.len()) {
            return Err(format!(
                "node {node}: a torn record: it closes with `{last}` after {} read line(s)",
                body.len()
            ));
        }
        for (n, line) in body.iter().enumerate() {
            let fields: Vec<&str> = line.split_ascii_whitespace().collect();
            let [input, x, y, z] = fields.as_slice() else {
                return Err(format!(
                    "node {node} line {}: expected `<input> <x> <y> <z>`, got `{line}`",
                    n + 1
                ));
            };
            let mut bits = [0u64; 3];
            for (slot, hex) in bits.iter_mut().zip([x, y, z]) {
                *slot = u64::from_str_radix(hex, 16)
                    .map_err(|e| format!("node {node} line {}: `{hex}`: {e}", n + 1))?;
            }
            let read = read_from_bits(bits);
            if read.writer == 0 {
                return Err(format!(
                    "node {node} line {}: writer 0, a frame no harness writer stamped",
                    n + 1
                ));
            }
            self.streams
                .entry((node.to_string(), (*input).to_string()))
                .or_default()
                .push(read);
        }
        Ok(())
    }

    /// Read every `<node>_<pid>_<k>.reads` record under `dir`. An empty file
    /// is skipped. A node with two non-empty records is an error: one run
    /// executes a node once, in one process.
    pub fn collect(dir: &Path) -> Result<ReadLog, String> {
        let mut log = ReadLog::default();
        let mut owner: BTreeMap<String, PathBuf> = BTreeMap::new();
        let entries = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let mut paths: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
        paths.sort();
        for path in paths {
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            let Some(stem) = name.strip_suffix(".reads") else {
                continue;
            };
            let mut parts = stem.rsplitn(3, '_');
            let (k, pid, node) = (parts.next(), parts.next(), parts.next());
            let (Some(k), Some(pid), Some(node)) = (k, pid, node) else {
                return Err(format!("{name}: not `<node>_<pid>_<k>.reads`"));
            };
            if node.is_empty() || pid.parse::<u32>().is_err() || k.parse::<u32>().is_err() {
                return Err(format!("{name}: not `<node>_<pid>_<k>.reads`"));
            }
            let text =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            if text.is_empty() {
                continue;
            }
            if let Some(prev) = owner.insert(node.to_string(), path.clone()) {
                return Err(format!(
                    "node {node} wrote two records: {} and {}",
                    prev.display(),
                    path.display()
                ));
            }
            log.add_record(node, &text)?;
        }
        Ok(log)
    }

    /// Reads across every stream.
    pub fn total(&self) -> usize {
        self.streams.values().map(Vec::len).sum()
    }
}

/// The per-fire batch sizes the `ri_batch` node `node` wrote under `dir`
/// (`<node>_<pid>.batches`, one fire per line), in fire order. Panics
/// `RI_HARNESS:` when the node wrote more than one such file.
pub fn batch_sizes(dir: &Path, node: &str) -> Vec<u32> {
    let mut out = Vec::new();
    let mut files = 0;
    let Ok(entries) = std::fs::read_dir(dir) else {
        return out;
    };
    for e in entries.flatten() {
        let name = e.file_name().to_string_lossy().into_owned();
        let Some(stem) = name.strip_suffix(".batches") else {
            continue;
        };
        if stem.rsplit_once('_').map(|(n, _)| n) != Some(node) {
            continue;
        }
        files += 1;
        assert!(
            files == 1,
            "RI_HARNESS: node {node} wrote batch sizes from two processes under {}",
            dir.display()
        );
        let text = std::fs::read_to_string(e.path()).unwrap_or_default();
        out.extend(text.lines().filter_map(|l| l.trim().parse::<u32>().ok()));
    }
    out
}

// ===========================================================================
// The judge.
// ===========================================================================

/// The count floors a judgement needs.
#[derive(Clone, Copy, Debug)]
pub struct Floors {
    /// Each expected stream needs at least this many reference reads, and an
    /// `Equal` stream at least this many compared reads.
    pub min_reads: usize,
    /// The tail allowance's floor, in reads.
    pub tail_min: usize,
    /// The tail allowance's proportional part: `live_len / tail_div`.
    pub tail_div: usize,
}

impl Default for Floors {
    fn default() -> Self {
        Floors {
            min_reads: 50,
            tail_min: 20,
            tail_div: 50,
        }
    }
}

/// One stream's verdict.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RowVerdict {
    /// Every compared read agrees, the lengths agree within the allowance and
    /// at least `min_reads` reads were compared.
    Equal,
    /// The first compared read that differs.
    Diverged {
        index: usize,
        live: Read,
        other: Read,
    },
    /// The candidate stops early: every read it has agrees, but it is missing
    /// more than the allowance (or compared fewer than `min_reads`).
    Short,
    /// The candidate has more reads than the reference.
    Long,
}

/// One stream's judgement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    pub node: String,
    pub input: String,
    pub live_len: usize,
    pub other_len: usize,
    pub compared: usize,
    pub verdict: RowVerdict,
}

impl Row {
    fn label(&self) -> &'static str {
        match self.verdict {
            RowVerdict::Equal => "equal",
            RowVerdict::Diverged { .. } => "diverged",
            RowVerdict::Short => "short",
            RowVerdict::Long => "long",
        }
    }
}

/// Every stream's judgement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Verdict {
    pub rows: Vec<Row>,
}

impl Verdict {
    /// Every stream is `Equal`.
    pub fn is_equal(&self) -> bool {
        self.rows.iter().all(|r| r.verdict == RowVerdict::Equal)
    }

    /// The streams that are not `Equal`.
    pub fn divergent(&self) -> Vec<&Row> {
        self.rows
            .iter()
            .filter(|r| r.verdict != RowVerdict::Equal)
            .collect()
    }

    /// `Err` naming every stream the reference exercised (at least
    /// `floors.min_reads` reads) on which fewer than `floors.min_reads` reads
    /// were compared and none differed: a candidate that wrote no record, or
    /// stopped early, compared nothing there. A `Diverged` row carries its
    /// differing read and passes.
    pub fn compared_floor(&self, floors: Floors) -> Result<(), String> {
        let thin: Vec<String> = self
            .rows
            .iter()
            .filter(|r| {
                r.live_len >= floors.min_reads
                    && r.compared < floors.min_reads
                    && !matches!(r.verdict, RowVerdict::Diverged { .. })
            })
            .map(|r| format!("{}.{} compared {}", r.node, r.input, r.compared))
            .collect();
        if thin.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "under the compared floor of {}: {}",
                floors.min_reads,
                thin.join(", ")
            ))
        }
    }

    /// One `READ_IDENTITY` line per stream.
    pub fn render_lines(&self, fixture: &str, leg: &str) -> Vec<String> {
        self.rows
            .iter()
            .map(|r| {
                let diff = match &r.verdict {
                    RowVerdict::Diverged { index, live, other } => {
                        format!(" first_diff={index} live_read={live} other_read={other}")
                    }
                    _ => String::new(),
                };
                format!(
                    "READ_IDENTITY fixture={fixture} leg={leg} node={} input={} verdict={} \
                     live={} other={} compared={}{diff}",
                    r.node,
                    r.input,
                    r.label(),
                    r.live_len,
                    r.other_len,
                    r.compared
                )
            })
            .collect()
    }

    /// Every line of [`Self::render_lines`], joined.
    pub fn render(&self) -> String {
        self.render_lines("-", "-").join("\n")
    }

    /// Print [`Self::render_lines`] to stderr.
    pub fn print(&self, fixture: &str, leg: &str) {
        for line in self.render_lines(fixture, leg) {
            eprintln!("{line}");
        }
    }
}

/// Judge `other` against the reference `live` on the streams `expected`, plus
/// any further stream either log carries. `Err` is a harness failure: an
/// expected stream missing from the reference or under the floor.
pub fn judge_streams(
    expected: &[(&str, &str)],
    live: &ReadLog,
    other: &ReadLog,
    floors: Floors,
) -> Result<Verdict, String> {
    if expected.is_empty() {
        return Err("no expected streams: a judgement over nothing proves nothing".into());
    }
    let mut keys: Vec<StreamKey> = expected
        .iter()
        .map(|(n, i)| ((*n).to_string(), (*i).to_string()))
        .collect();
    for key in live.streams.keys().chain(other.streams.keys()) {
        if !keys.contains(key) {
            keys.push(key.clone());
        }
    }
    let mut rows = Vec::with_capacity(keys.len());
    for key in keys {
        let empty = Vec::new();
        let l = live.streams.get(&key).unwrap_or(&empty);
        let o = other.streams.get(&key).unwrap_or(&empty);
        let is_expected = expected.iter().any(|(n, i)| *n == key.0 && *i == key.1);
        if is_expected && l.len() < floors.min_reads {
            return Err(format!(
                "stream {}.{}: the reference holds {} read(s), under the floor of {}",
                key.0,
                key.1,
                l.len(),
                floors.min_reads
            ));
        }
        let compared = l.len().min(o.len());
        let allowance = floors.tail_min.max(l.len() / floors.tail_div.max(1));
        let verdict = match (0..compared).find(|&i| l[i] != o[i]) {
            Some(index) => RowVerdict::Diverged {
                index,
                live: l[index],
                other: o[index],
            },
            None if o.len() > l.len() => RowVerdict::Long,
            None if o.len() + allowance < l.len() || compared < floors.min_reads => {
                RowVerdict::Short
            }
            None => RowVerdict::Equal,
        };
        rows.push(Row {
            node: key.0,
            input: key.1,
            live_len: l.len(),
            other_len: o.len(),
            compared,
            verdict,
        });
    }
    Ok(Verdict { rows })
}

/// [`judge_streams`] on the fixture's expected streams.
pub fn judge(
    fixture: Fixture,
    live: &ReadLog,
    other: &ReadLog,
    floors: Floors,
) -> Result<Verdict, String> {
    judge_streams(fixture.streams(), live, other, floors)
}

// ===========================================================================
// Fire windows: a node's fires over one measured window.
// ===========================================================================

/// The equal parts a measured window is split into; [`check_fire_window`]
/// requires a fire in every one.
pub const WINDOW_PARTS: usize = 4;

/// One node's fires over the measured window `[from_ns, to_ns]`.
#[derive(Debug, Clone, PartialEq)]
pub struct FireWindow {
    pub from_ns: u64,
    pub to_ns: u64,
    /// Fires inside the window.
    pub fires: usize,
    /// `fires` over the window's length.
    pub rate_hz: f64,
    /// Latency percentiles of the fires inside the window; `u64::MAX` when
    /// there are none.
    pub p50_ns: u64,
    pub p99_ns: u64,
    /// The first and last fires inside the window lie within the edge slack
    /// of its ends.
    pub edges_ok: bool,
    /// Fires in each of the window's [`WINDOW_PARTS`] equal parts.
    pub parts: [usize; WINDOW_PARTS],
}

/// The [`FireWindow`] of `samples` (`(fire ns, latency ns)` per fire, any
/// order) over `[from_ns, to_ns]`, with `edge_slack_ns` for
/// [`FireWindow::edges_ok`]. Panics `RI_HARNESS:` when `to_ns <= from_ns`.
pub fn fire_window(
    samples: &[(u64, u64)],
    from_ns: u64,
    to_ns: u64,
    edge_slack_ns: u64,
) -> FireWindow {
    assert!(
        to_ns > from_ns,
        "RI_HARNESS: an empty measured window [{from_ns}, {to_ns}]"
    );
    let mut inside: Vec<(u64, u64)> = samples
        .iter()
        .copied()
        .filter(|(t, _)| *t >= from_ns && *t <= to_ns)
        .collect();
    inside.sort_unstable();
    let len = to_ns - from_ns;
    let mut parts = [0usize; WINDOW_PARTS];
    for (t, _) in &inside {
        let k = ((t - from_ns) as u128 * WINDOW_PARTS as u128 / len as u128) as usize;
        parts[k.min(WINDOW_PARTS - 1)] += 1;
    }
    let edges_ok = inside
        .first()
        .is_some_and(|f| f.0 <= from_ns + edge_slack_ns)
        && inside.last().is_some_and(|l| l.0 + edge_slack_ns >= to_ns);
    let mut lat: Vec<u64> = inside.iter().map(|(_, l)| *l).collect();
    lat.sort_unstable();
    let pct = |p: f64| {
        if lat.is_empty() {
            return u64::MAX;
        }
        lat[((lat.len() - 1) as f64 * p).round() as usize]
    };
    FireWindow {
        from_ns,
        to_ns,
        fires: inside.len(),
        rate_hz: inside.len() as f64 / (len as f64 / 1e9),
        p50_ns: pct(0.50),
        p99_ns: pct(0.99),
        edges_ok,
        parts,
    }
}

/// `Ok` when `w` holds at least `floor` fires (a floor of 0 counts as 1) and
/// at least one fire in each of its [`WINDOW_PARTS`] parts. `Err` names the
/// shortfall: an empty window, a count under the floor, or the first part
/// with no fire (the node stalled there).
pub fn check_fire_window(w: &FireWindow, floor: usize) -> Result<(), String> {
    let floor = floor.max(1);
    if w.fires < floor {
        return Err(format!(
            "{} fires in the measured window, under the floor of {floor}",
            w.fires
        ));
    }
    if let Some(k) = w.parts.iter().position(|&n| n == 0) {
        return Err(format!(
            "no fire in part {} of {WINDOW_PARTS} of the measured window (fires per part {:?}): \
             the node stalled there",
            k + 1,
            w.parts
        ));
    }
    Ok(())
}

// ===========================================================================
// Node types.
// ===========================================================================

/// The workspace's node types: `(type, policy, inputs, outputs, body)`, the
/// body being the `lib.rs` text from the node attribute on.
struct NodeType {
    name: &'static str,
    policy: MacroPolicy,
    trigger: Option<&'static str>,
    inputs: &'static [&'static str],
    outputs: &'static [&'static str],
    body: &'static str,
}

/// The node-side half of the read record: arm check, push, shutdown write and
/// the planted misread. Appended to every reading node's `lib.rs`.
const RECORD_FNS: &str = r#"
fn ri_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("CER_RI_READS_DIR").is_some())
}

fn ri_bits(r: [f64; 3]) -> [u64; 3] {
    [r[0].to_bits(), r[1].to_bits(), r[2].to_bits()]
}

/// FNV-1a 64 over the three fields, folded to 48 bits so an f64 holds it.
fn ri_fold(r: [f64; 3]) -> f64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for word in ri_bits(r) {
        for b in word.to_le_bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    }
    (h & ((1u64 << 48) - 1)) as f64
}

fn ri_flush(id: &str, input: &str, reads: &mut Vec<[u64; 3]>) {
    use std::io::Write as _;
    if reads.is_empty() {
        return;
    }
    let Some(dir) = std::env::var_os("CER_RI_READS_DIR") else {
        return;
    };
    let plant: Option<usize> = std::env::var("CER_RI_PLANT").ok().and_then(|v| {
        let (node, index) = v.split_once(':')?;
        if node == id { index.parse().ok() } else { None }
    });
    let mut out = String::with_capacity(reads.len() * 56);
    for (i, b) in reads.iter().enumerate() {
        let x = if plant == Some(i) {
            (f64::from_bits(b[0]) + 1000.0).to_bits()
        } else {
            b[0]
        };
        out.push_str(&format!("{input} {x:016x} {:016x} {:016x}\n", b[1], b[2]));
    }
    out.push_str(&format!("end {}\n", reads.len()));
    static K: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let k = K.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path = std::path::Path::new(&dir).join(format!("{id}_{}_{k}.reads", std::process::id()));
    if let Ok(mut f) = std::fs::OpenOptions::new().write(true).create_new(true).open(path) {
        let _ = f.write_all(out.as_bytes());
    }
    reads.clear();
}

fn ri_sleep(us: u64) {
    if us > 0 {
        std::thread::sleep(std::time::Duration::from_micros(us));
    }
}
"#;

const NODE_TYPES: &[NodeType] = &[
    // A 1 ms source stamping (writer, seq, value) on every frame.
    NodeType {
        name: "ri_source",
        policy: MacroPolicy::Period { period_ms: 1 },
        trigger: None,
        inputs: &[],
        outputs: &["out"],
        body: r#"#[cerulion_node(period_ms = 1)]
struct RiSourceNode {
    #[output]
    out: Vector3,
    #[param(default = 1, description = "this writer's identity")]
    ri_writer: u64,
    tick_count: u32,
}

#[cerulion_node_impl]
impl RiSourceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        let seq = u64::from(self.tick_count);
        self.out.x = self.ri_writer as f64;
        self.out.y = seq as f64;
        self.out.z = ((seq.wrapping_mul(2_654_435_761) ^ self.ri_writer) & ((1u64 << 48) - 1)) as f64;
        Ok(())
    }
}
"#,
    },
    // A data-triggered relay: records its read and republishes under its own
    // identity, with a payload derived from what it read.
    NodeType {
        name: "ri_relay",
        policy: MacroPolicy::DataTrigger {
            input_name: String::new(),
        },
        trigger: Some("inp"),
        inputs: &["inp"],
        outputs: &["out"],
        body: r#"#[cerulion_node]
struct RiRelayNode {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
    #[param(default = "", description = "the graph node id the read record is named after")]
    ri_id: String,
    #[param(default = 100, description = "this writer's identity")]
    ri_writer: u64,
    tick_count: u32,
    ri_reads: Vec<[u64; 3]>,
}

#[cerulion_node_impl]
impl RiRelayNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        let r = [self.inp.x, self.inp.y, self.inp.z];
        if ri_on() {
            self.ri_reads.push(ri_bits(r));
        }
        self.out.x = self.ri_writer as f64;
        self.out.y = f64::from(self.tick_count);
        self.out.z = ri_fold(r);
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        ri_flush(&self.ri_id, "inp", &mut self.ri_reads);
        Ok(())
    }
}
"#,
    },
    // A data-triggered sink with the default queue depth and an optional
    // per-fire sleep.
    NodeType {
        name: "ri_sink",
        policy: MacroPolicy::DataTrigger {
            input_name: String::new(),
        },
        trigger: Some("inp"),
        inputs: &["inp"],
        outputs: &[],
        body: r#"#[cerulion_node]
struct RiSinkNode {
    #[input(trigger)]
    inp: Vector3,
    #[param(default = "", description = "the graph node id the read record is named after")]
    ri_id: String,
    #[param(default = 0, description = "microseconds each fire sleeps")]
    ri_sleep_us: u64,
    ri_reads: Vec<[u64; 3]>,
}

#[cerulion_node_impl]
impl RiSinkNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let r = [self.inp.x, self.inp.y, self.inp.z];
        if ri_on() {
            self.ri_reads.push(ri_bits(r));
        }
        ri_sleep(self.ri_sleep_us);
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        ri_flush(&self.ri_id, "inp", &mut self.ri_reads);
        Ok(())
    }
}
"#,
    },
    // The same sink on a two-frame queue: a slow one evicts (drop_oldest).
    NodeType {
        name: "ri_sink_shallow",
        policy: MacroPolicy::DataTrigger {
            input_name: String::new(),
        },
        trigger: Some("inp"),
        inputs: &["inp"],
        outputs: &[],
        body: r#"#[cerulion_node]
struct RiSinkShallowNode {
    #[input(trigger, depth = 2)]
    inp: Vector3,
    #[param(default = "", description = "the graph node id the read record is named after")]
    ri_id: String,
    #[param(default = 0, description = "microseconds each fire sleeps")]
    ri_sleep_us: u64,
    ri_reads: Vec<[u64; 3]>,
}

#[cerulion_node_impl]
impl RiSinkShallowNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let r = [self.inp.x, self.inp.y, self.inp.z];
        if ri_on() {
            self.ri_reads.push(ri_bits(r));
        }
        ri_sleep(self.ri_sleep_us);
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        ri_flush(&self.ri_id, "inp", &mut self.ri_reads);
        Ok(())
    }
}
"#,
    },
    // A 4 ms batch reader: every frame since its last fire, from every writer
    // of a multi-publisher topic (about twelve per fire from three 1 ms
    // writers).
    NodeType {
        name: "ri_batch",
        policy: MacroPolicy::Period { period_ms: 4 },
        trigger: None,
        inputs: &["tf"],
        outputs: &[],
        body: r#"#[cerulion_node(period_ms = 4)]
struct RiBatchNode {
    #[input(accumulate, cap = 16)]
    tf: Vector3,
    #[param(default = "", description = "the graph node id the read record is named after")]
    ri_id: String,
    ri_reads: Vec<[u64; 3]>,
    ri_batches: Vec<u32>,
}

#[cerulion_node_impl]
impl RiBatchNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let on = ri_on();
        let mut n = 0u32;
        for frame in self.tf.iter() {
            let r = [frame.x, frame.y, frame.z];
            if on {
                self.ri_reads.push(ri_bits(r));
            }
            n += 1;
        }
        if on && n > 0 {
            self.ri_batches.push(n);
        }
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        use std::io::Write as _;
        if let (false, Some(dir)) = (self.ri_batches.is_empty(), std::env::var_os("CER_RI_READS_DIR")) {
            let text: String = self.ri_batches.iter().map(|n| format!("{n}\n")).collect();
            let path = std::path::Path::new(&dir)
                .join(format!("{}_{}.batches", self.ri_id, std::process::id()));
            if let Ok(mut f) = std::fs::OpenOptions::new().write(true).create_new(true).open(path) {
                let _ = f.write_all(text.as_bytes());
            }
        }
        ri_flush(&self.ri_id, "tf", &mut self.ri_reads);
        Ok(())
    }
}
"#,
    },
    // The slow-does-not-block pacer: a 1 ms source with two outputs, each
    // frame stamped with its publish time (`x`, real ns) and sequence (`y`).
    NodeType {
        name: "ri_pace",
        policy: MacroPolicy::Period { period_ms: 1 },
        trigger: None,
        inputs: &[],
        outputs: &["out", "side"],
        body: r#"#[cerulion_node(period_ms = 1)]
struct RiPaceNode {
    #[output]
    out: Vector3,
    #[output]
    side: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl RiPaceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        let now = self.real_ns() as f64;
        self.out.x = now;
        self.out.y = f64::from(self.tick_count);
        self.side.x = now;
        self.side.y = f64::from(self.tick_count);
        Ok(())
    }
}
"#,
    },
    // The fast consumer: one `<fire real ns> <latency ns>` sample per fire,
    // buffered and written at shutdown under CER_RI_SAMPLES_DIR.
    NodeType {
        name: "ri_fast",
        policy: MacroPolicy::DataTrigger {
            input_name: String::new(),
        },
        trigger: Some("inp"),
        inputs: &["inp"],
        outputs: &[],
        body: r#"#[cerulion_node]
struct RiFastNode {
    #[input(trigger)]
    inp: Vector3,
    samples: Vec<(u64, u64)>,
}

#[cerulion_node_impl]
impl RiFastNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let stamp = self.inp.x;
        let now = self.real_ns();
        self.samples.push((now, now.saturating_sub(stamp as u64)));
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        use std::io::Write as _;
        if self.samples.is_empty() {
            return Ok(());
        }
        let Some(dir) = std::env::var_os("CER_RI_SAMPLES_DIR") else {
            return Ok(());
        };
        let mut out = String::with_capacity(self.samples.len() * 24);
        for (t, lat) in &self.samples {
            out.push_str(&format!("{t} {lat}\n"));
        }
        let path = std::path::Path::new(&dir).join(format!("fast_{}.samples", std::process::id()));
        if let Ok(mut f) = std::fs::File::create(path) {
            let _ = f.write_all(out.as_bytes());
        }
        Ok(())
    }
}
"#,
    },
    // The slow consumer behind a declared `block` edge: its producer is
    // deferred while two frames are unserved.
    NodeType {
        name: "ri_slow_block",
        policy: MacroPolicy::DataTrigger {
            input_name: String::new(),
        },
        trigger: Some("inp"),
        inputs: &["inp"],
        outputs: &[],
        body: r#"#[cerulion_node]
struct RiSlowBlockNode {
    #[input(trigger, depth = 2, backpressure = block)]
    inp: Vector3,
    #[param(default = 0, description = "microseconds each fire sleeps")]
    ri_sleep_us: u64,
}

#[cerulion_node_impl]
impl RiSlowBlockNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _seq = self.inp.y;
        ri_sleep(self.ri_sleep_us);
        Ok(())
    }
}
"#,
    },
];

/// The built workspace the fixtures run in, removed when it drops.
pub struct Workspace {
    /// Owns the directory `root` lives in and removes it on drop.
    pub _dir: ScaffoldDir,
    pub root: PathBuf,
    pub graphs_dir: PathBuf,
    /// Where the node cdylibs are built; the drivers hand it to the
    /// `cerulion` processes they spawn as `CARGO_TARGET_DIR`.
    pub target_dir: PathBuf,
}

/// The `lib.rs` of node type `t`: the scaffold's header (lint line and
/// imports, everything before the node attribute), then `t.body`, then the
/// record functions.
fn node_source(scaffolded: &str, t: &NodeType) -> String {
    let at = scaffolded
        .find("#[cerulion_node")
        .unwrap_or_else(|| panic!("RI_HARNESS: {} scaffold carries no node attribute", t.name));
    format!("{}{}{RECORD_FNS}", &scaffolded[..at], t.body)
}

/// Scaffold and build the workspace in a new [`ScaffoldDir`], held by the
/// caller. Every workspace of one test binary builds into one target
/// directory ([`scaffold_target_dir`], keyed by the binary's crate name), so
/// the dependencies compile once per binary and a later workspace rebuilds
/// only the node crates and what depends on the workspace's schemas.
pub fn workspace() -> Workspace {
    let dir = ScaffoldDir::new("ri_ws");
    let ws =
        workspace::workspace_create(dir.path(), "ri_ws").expect("RI_HARNESS: workspace_create");
    let target_dir = scaffold_target_dir(&ws.root, env!("CARGO_CRATE_NAME"));
    pin_target_dir(&ws.root, &target_dir);
    let cargo_toml = ws.root.join("Cargo.toml");
    for t in NODE_TYPES {
        let policy = match &t.policy {
            MacroPolicy::DataTrigger { .. } => MacroPolicy::DataTrigger {
                input_name: t
                    .trigger
                    .expect("a data-trigger type names its trigger")
                    .to_string(),
            },
            other => other.clone(),
        };
        node_cmd::node_create_with_options(
            &ws.nodes_dir,
            &cargo_toml,
            t.name,
            Some(policy),
            &NodeCreateOptions {
                outputs: t
                    .outputs
                    .iter()
                    .map(|o| (SCHEMA.to_string(), (*o).to_string()))
                    .collect(),
                inputs: t
                    .inputs
                    .iter()
                    .map(|i| (SCHEMA.to_string(), (*i).to_string()))
                    .collect(),
                trigger: t.trigger.map(str::to_string),
                raw_ffi: false,
                language: NodeLanguage::Rust,
            },
        )
        .unwrap_or_else(|e| panic!("RI_HARNESS: node_create {}: {e}", t.name));
        let lib = ws.nodes_dir.join(t.name).join("src").join("lib.rs");
        let scaffolded = std::fs::read_to_string(&lib)
            .unwrap_or_else(|e| panic!("RI_HARNESS: read {}: {e}", lib.display()));
        std::fs::write(&lib, node_source(&scaffolded, t))
            .unwrap_or_else(|e| panic!("RI_HARNESS: write {}: {e}", lib.display()));
    }
    for t in NODE_TYPES {
        node_cmd::node_build(&ws.root, t.name, false)
            .unwrap_or_else(|e| panic!("RI_HARNESS: node_build {}: {e}", t.name));
    }
    Workspace {
        _dir: dir,
        root: ws.root.clone(),
        graphs_dir: ws.graphs_dir.clone(),
        target_dir,
    }
}

/// Every node type's debug cdylib, by type name.
pub fn built_cdylibs(ws: &Workspace) -> Vec<(&'static str, PathBuf)> {
    let debug = ws.target_dir.join("debug");
    NODE_TYPES
        .iter()
        .map(|t| (t.name, debug.join(mp_support::dylib_file(t.name))))
        .collect()
}

// ===========================================================================
// Fixtures.
// ===========================================================================

/// The graph shapes the oracle runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fixture {
    /// `src` (1 ms) -> `slow`, a two-frame queue whose fire sleeps 20 ms: it
    /// evicts (drop_oldest) live. Two processes.
    SlowEvict,
    /// `src` (1 ms) -> `sink_a` and `sink_b`: one topic, two consumers.
    /// Three processes.
    FanOut,
    /// `w1`, `w2`, `w3` (1 ms each) all publish `/<prefix>/tf`, listed in
    /// `multi_publisher_topics`; `tfbuf` (4 ms) reads it as an accumulate
    /// input, about twelve frames per fire. Four processes.
    MultiWriter,
    /// `src` (1 ms) -> `relay` -> `sink`, split `[src]` / `[relay, sink]`.
    Ranks2,
    /// The same chain split `[src]` / `[relay]` / `[sink]`.
    Ranks3,
}

/// A fixture's nodes, its process groups in rank order, and its
/// multi-publisher topics.
type GraphParts = (
    Vec<NodeDef>,
    Vec<(&'static str, Vec<&'static str>)>,
    Vec<String>,
);

/// Where a fixture's nodes run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Placement {
    /// The fixture's own `process_groups`.
    Declared,
    /// Every node in one process (`graph run --single-process`).
    SingleProcess,
}

impl Placement {
    pub fn label(self) -> &'static str {
        match self {
            Placement::Declared => "declared",
            Placement::SingleProcess => "single_process",
        }
    }
}

/// How long the slow consumer of [`Fixture::SlowEvict`] sleeps per fire.
pub const SLOW_EVICT_SLEEP_US: u64 = 20_000;

impl Fixture {
    pub const ALL: [Fixture; 5] = [
        Fixture::SlowEvict,
        Fixture::FanOut,
        Fixture::MultiWriter,
        Fixture::Ranks2,
        Fixture::Ranks3,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Fixture::SlowEvict => "slow_evict",
            Fixture::FanOut => "fan_out",
            Fixture::MultiWriter => "multi_writer",
            Fixture::Ranks2 => "ranks2",
            Fixture::Ranks3 => "ranks3",
        }
    }

    /// The `(node, input)` streams a run of this fixture must read.
    pub fn streams(self) -> &'static [(&'static str, &'static str)] {
        match self {
            Fixture::SlowEvict => &[("slow", "inp")],
            Fixture::FanOut => &[("sink_a", "inp"), ("sink_b", "inp")],
            Fixture::MultiWriter => &[("tfbuf", "tf")],
            Fixture::Ranks2 | Fixture::Ranks3 => &[("relay", "inp"), ("sink", "inp")],
        }
    }

    /// The fixture's graph, its topics under `/<prefix>/`.
    pub fn graph_config(self, graph: &str, prefix: &str) -> GraphConfig {
        let out = |name: &str, topic: Option<String>| OutputDef {
            name: name.to_string(),
            schema: SCHEMA.to_string(),
            max_slice_len: None,
            history_size: 0,
            topic,
        };
        let input = |name: &str, source: String| InputDef {
            name: name.to_string(),
            source,
        };
        let node = |id: &str,
                    ty: &str,
                    inputs: Vec<InputDef>,
                    outputs: Vec<OutputDef>,
                    params: &[(&str, ParamValue)]| NodeDef {
            fuse: None,
            ros2: None,
            params: params
                .iter()
                .map(|(k, v)| ((*k).to_string(), v.clone()))
                .collect(),
            id: id.to_string(),
            node_type: ty.to_string(),
            inputs,
            outputs,
        };
        let id = |s: &str| ("ri_id", ParamValue::Str(s.to_string()));
        let writer = |w: u64| ("ri_writer", ParamValue::UInt(w));
        let src = || {
            node(
                "src",
                "ri_source",
                vec![],
                vec![out("out", None)],
                &[writer(1)],
            )
        };
        let (nodes, groups, mp_topics): GraphParts = match self {
            Fixture::SlowEvict => (
                vec![
                    src(),
                    node(
                        "slow",
                        "ri_sink_shallow",
                        vec![input("inp", "src/out".into())],
                        vec![],
                        &[
                            id("slow"),
                            ("ri_sleep_us", ParamValue::UInt(SLOW_EVICT_SLEEP_US)),
                        ],
                    ),
                ],
                vec![("g_src", vec!["src"]), ("g_slow", vec!["slow"])],
                vec![],
            ),
            Fixture::FanOut => (
                vec![
                    src(),
                    node(
                        "sink_a",
                        "ri_sink",
                        vec![input("inp", "src/out".into())],
                        vec![],
                        &[id("sink_a")],
                    ),
                    node(
                        "sink_b",
                        "ri_sink",
                        vec![input("inp", "src/out".into())],
                        vec![],
                        &[id("sink_b")],
                    ),
                ],
                vec![
                    ("g_src", vec!["src"]),
                    ("g_a", vec!["sink_a"]),
                    ("g_b", vec!["sink_b"]),
                ],
                vec![],
            ),
            Fixture::MultiWriter => {
                let tf = format!("/{prefix}/tf");
                let w = |n: &str, k: u64| {
                    node(
                        n,
                        "ri_source",
                        vec![],
                        vec![out("out", Some(tf.clone()))],
                        &[writer(k)],
                    )
                };
                (
                    vec![
                        w("w1", 1),
                        w("w2", 2),
                        w("w3", 3),
                        node(
                            "tfbuf",
                            "ri_batch",
                            vec![input("tf", tf.clone())],
                            vec![],
                            &[id("tfbuf")],
                        ),
                    ],
                    vec![
                        ("g_w1", vec!["w1"]),
                        ("g_w2", vec!["w2"]),
                        ("g_w3", vec!["w3"]),
                        ("g_tf", vec!["tfbuf"]),
                    ],
                    vec![tf],
                )
            }
            Fixture::Ranks2 | Fixture::Ranks3 => {
                let nodes = vec![
                    src(),
                    node(
                        "relay",
                        "ri_relay",
                        vec![input("inp", "src/out".into())],
                        vec![out("out", None)],
                        &[id("relay"), writer(100)],
                    ),
                    node(
                        "sink",
                        "ri_sink",
                        vec![input("inp", "relay/out".into())],
                        vec![],
                        &[id("sink")],
                    ),
                ];
                let groups = if self == Fixture::Ranks2 {
                    vec![("g1", vec!["src"]), ("g2", vec!["relay", "sink"])]
                } else {
                    vec![
                        ("g1", vec!["src"]),
                        ("g2", vec!["relay"]),
                        ("g3", vec!["sink"]),
                    ]
                };
                (nodes, groups, vec![])
            }
        };
        graph_config(graph, prefix, nodes, &groups, mp_topics)
    }
}

/// One node of a harness graph: `inputs` as `(name, source)`, every output on
/// its derived topic.
pub fn node_def(
    id: &str,
    node_type: &str,
    inputs: &[(&str, &str)],
    outputs: &[&str],
    params: &[(&str, ParamValue)],
) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        params: params
            .iter()
            .map(|(k, v)| ((*k).to_string(), v.clone()))
            .collect(),
        id: id.to_string(),
        node_type: node_type.to_string(),
        inputs: inputs
            .iter()
            .map(|(name, source)| InputDef {
                name: (*name).to_string(),
                source: (*source).to_string(),
            })
            .collect(),
        outputs: outputs
            .iter()
            .map(|name| OutputDef {
                name: (*name).to_string(),
                schema: SCHEMA.to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            })
            .collect(),
    }
}

/// A graph config from its parts. `groups` in rank order.
pub fn graph_config(
    graph: &str,
    prefix: &str,
    nodes: Vec<NodeDef>,
    groups: &[(&str, Vec<&str>)],
    multi_publisher_topics: Vec<String>,
) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: graph.to_string(),
        prefix: prefix.to_string(),
        nodes,
        multi_publisher_topics,
        process_groups: groups
            .iter()
            .map(|(g, members)| {
                (
                    (*g).to_string(),
                    members.iter().map(|m| (*m).to_string()).collect(),
                )
            })
            .collect(),
        process_group_order: vec![],
    }
}

// ===========================================================================
// Drivers.
// ===========================================================================

/// One recorded live run: its private directory, the finalized bag and the
/// live read log.
pub struct RecordedRun {
    pub fixture: Fixture,
    pub placement: Placement,
    /// Kept when the test fails with `CERULION_KEEP_TEST_WS=1`
    /// ([`ScaffoldDir`]): the bag, the read logs, and each replay's stderr,
    /// `--report` JSON and re-recording.
    pub dir: ScaffoldDir,
    pub bag: PathBuf,
    pub live: ReadLog,
    /// Where the live run's nodes wrote their records.
    pub live_dir: PathBuf,
}

/// How long a live run records once its bag exists.
pub const LIVE_WINDOW: Duration = Duration::from_secs(4);
/// Live attempts before a lossy recording fails the harness.
pub const RECORD_ATTEMPTS: u32 = 3;

/// Environment a run must not inherit: the run-shaping knobs.
const SCRUBBED_ENV: &[&str] = &[
    "CERULION_EXECUTION_MODE",
    "CERULION_MP_PEER_LOSS",
    "CERULION_MP_DRAIN_MS",
    "CERULION_DETERMINISM",
];
const SCRUBBED_ENV_PREFIXES: &[&str] = &["CERULION_FLASHBACK", "CERULION_BARRIER_"];

fn scrub(cmd: &mut Command) {
    for name in SCRUBBED_ENV {
        cmd.env_remove(name);
    }
    for (name, _) in std::env::vars_os() {
        if let Some(n) = name.to_str() {
            if SCRUBBED_ENV_PREFIXES.iter().any(|p| n.starts_with(p)) {
                cmd.env_remove(n);
            }
        }
    }
}

/// A unique graph file stem and topic prefix for one run, so two runs in one
/// test process never share an iceoryx2 service name.
pub fn unique_names(fixture_name: &str) -> (String, String) {
    static N: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let pid = std::process::id() % 100_000;
    (
        format!("ri_{fixture_name}_{pid}_{n}"),
        format!("ri{pid}n{n}"),
    )
}

/// Write `config` to the workspace as `graphs/<identity>.yaml`.
pub fn write_graph(ws: &Workspace, config: &GraphConfig) {
    let yaml = serde_yaml::to_string(config).expect("RI_HARNESS: serialize the graph");
    let path = ws.graphs_dir.join(format!("{}.yaml", config.identity));
    std::fs::write(&path, yaml)
        .unwrap_or_else(|e| panic!("RI_HARNESS: write {}: {e}", path.display()));
}

/// What [`run_live`] observed: the supervisor's captured output and the
/// host monotonic clock ([`cerulion_core::real_ns`], the clock a node's
/// `real_ns()` reads) when `ready` first held and just before the Ctrl-C.
pub struct LiveRun {
    pub logs: String,
    pub ready_ns: u64,
    pub stop_ns: u64,
}

/// One live `graph run` with its output in `dir`: spawned, run for `window`
/// once `ready` holds, stopped by Ctrl-C, reaped. Panics `RI_HARNESS:` on any
/// bring-up or teardown failure. `envs` is applied after the harness's own
/// variables are removed, so only what the caller passes reaches the run.
pub fn run_live(
    ws: &Workspace,
    graph: &str,
    extra: &[&str],
    envs: &[(&str, &Path)],
    dir: &Path,
    ready: &dyn Fn() -> bool,
    window: Duration,
) -> LiveRun {
    let home = dir.join("cerhome");
    std::fs::create_dir_all(&home).expect("RI_HARNESS: create CERULION_HOME");
    // The login gate reads `auth.json` under CERULION_HOME.
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "read-identity-harness")
        .expect("RI_HARNESS: seed the run's sign-in state");
    let stdout = dir.join("run.stdout");
    let stderr = dir.join("run.stderr");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", graph])
        .args(extra)
        .current_dir(&ws.root)
        // The workers load the node cdylibs from this target directory.
        .env("CARGO_TARGET_DIR", &ws.target_dir)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_HOME", &home)
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        );
    scrub(&mut cmd);
    for name in [READS_DIR_ENV, PLANT_ENV, SAMPLES_DIR_ENV] {
        cmd.env_remove(name);
    }
    for (k, v) in envs {
        cmd.env(k, v);
    }
    // SIGINT back to its default disposition in the child: a test run as a
    // background job of a non-interactive shell inherits it ignored, and the
    // Ctrl-C below would then never reach the supervisor.
    // SAFETY: `signal` is async-signal-safe; nothing is allocated.
    unsafe {
        cmd.pre_exec(|| {
            libc::signal(libc::SIGINT, libc::SIG_DFL);
            Ok(())
        });
    }
    cmd.stdout(Stdio::from(
        std::fs::File::create(&stdout).expect("RI_HARNESS: stdout file"),
    ))
    .stderr(Stdio::from(
        std::fs::File::create(&stderr).expect("RI_HARNESS: stderr file"),
    ));
    let logs = || {
        format!(
            "stdout:\n{}\nstderr:\n{}",
            mp_support::read_file(&stdout),
            mp_support::read_file(&stderr)
        )
    };
    let mut guard = ChildGuard::spawn_group_leader(&mut cmd)
        .unwrap_or_else(|e| panic!("RI_HARNESS: spawn graph run {graph}: {e}"));
    let start = std::time::Instant::now();
    while !ready() {
        if let Some(status) = guard.try_wait_noting().expect("RI_HARNESS: poll") {
            panic!(
                "RI_HARNESS: graph run {graph} exited during bring-up ({status:?})\n{}",
                logs()
            );
        }
        assert!(
            start.elapsed() < Duration::from_secs(120),
            "RI_HARNESS: graph run {graph} not ready within 120 s\n{}",
            logs()
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    let ready_ns = cerulion_core::real_ns();
    guard.note_workers();
    std::thread::sleep(window);
    let stop_ns = cerulion_core::real_ns();
    if let Some(status) = guard.try_wait_noting().expect("RI_HARNESS: poll") {
        panic!(
            "RI_HARNESS: graph run {graph} exited on its own ({status:?}) before the Ctrl-C\n{}",
            logs()
        );
    }
    mp_support::send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .unwrap_or_else(|| {
            panic!(
                "RI_HARNESS: graph run {graph} did not exit after Ctrl-C\n{}",
                logs()
            )
        });
    assert!(
        status.success(),
        "RI_HARNESS: graph run {graph} must exit 0 on Ctrl-C, got {status:?}\n{}",
        logs()
    );
    guard.finish().assert_clean();
    LiveRun {
        logs: logs(),
        ready_ns,
        stop_ns,
    }
}

/// Why a recording is not fit to judge: frames the recorder lost
/// (`record_health.json`: `dropped_unwritten` plus every topic's
/// `frames_lost`), frames published before a tap opened, or coverage gaps
/// (`record_coverage.json`). `None` for a whole recording. Panics `RI_HARNESS:` on a
/// bag that is not finalized or lacks either attachment.
fn recording_defect(bag: &Path) -> Option<String> {
    let reader = cerulion_bag::BagReader::open(bag).expect("RI_HARNESS: open the bag");
    let (_msgs, completeness) = reader
        .recover_messages()
        .expect("RI_HARNESS: recover the bag");
    assert!(
        completeness.is_finalized(),
        "RI_HARNESS: the Ctrl-C teardown must finalize the bag, got {completeness:?}"
    );
    let attachment = |name: &str| {
        reader
            .attachment(name)
            .expect("RI_HARNESS: read attachments")
            .unwrap_or_else(|| panic!("RI_HARNESS: the bag carries no {name}"))
            .data
    };
    let health: cerulion_bagd::RecordHealth =
        serde_json::from_slice(&attachment(cerulion_bagd::RECORD_HEALTH_ATTACHMENT))
            .unwrap_or_else(|e| panic!("RI_HARNESS: record_health.json does not parse: {e}"));
    let lost: u64 =
        health.dropped_unwritten + health.topics.values().map(|t| t.frames_lost).sum::<u64>();
    let coverage: cerulion_bagd::RecordCoverage =
        serde_json::from_slice(&attachment(cerulion_bagd::RECORD_COVERAGE_ATTACHMENT))
            .unwrap_or_else(|e| panic!("RI_HARNESS: record_coverage.json does not parse: {e}"));
    let prefix_lost = coverage.prefix_lost_total();
    let gaps = coverage.gap_count();
    if lost == 0 && prefix_lost == 0 && gaps == 0 {
        return None;
    }
    Some(format!(
        "frames lost {lost}, published before the tap {prefix_lost}, coverage gaps {gaps}"
    ))
}

/// Record `fixture` live under `placement` with the read log armed: a whole,
/// finalized bag and the live reads. A recording with a defect
/// (`recording_defect`) is retried ([`RECORD_ATTEMPTS`]), never judged.
pub fn record_live(ws: &Workspace, fixture: Fixture, placement: Placement) -> RecordedRun {
    let mut last = String::new();
    for attempt in 0..RECORD_ATTEMPTS {
        let dir = ScaffoldDir::new("ri_run");
        let (graph, prefix) = unique_names(fixture.name());
        write_graph(ws, &fixture.graph_config(&graph, &prefix));
        let reads = dir.path().join("live_reads");
        let recordings = dir.path().join("recordings");
        std::fs::create_dir_all(&reads).expect("RI_HARNESS: reads dir");
        let record_arg = format!("--record={}", recordings.display());
        let mut extra = vec![record_arg.as_str()];
        if placement == Placement::SingleProcess {
            extra.push("--single-process");
        }
        let rec = recordings.clone();
        let ready = move || {
            std::fs::read_dir(&rec).is_ok_and(|d| {
                d.flatten()
                    .any(|e| e.path().extension().and_then(|x| x.to_str()) == Some("mcap"))
            })
        };
        run_live(
            ws,
            &graph,
            &extra,
            &[(READS_DIR_ENV, &reads)],
            dir.path(),
            &ready,
            LIVE_WINDOW,
        );
        let bag = mp_support::assert_single_bag(&recordings);
        if let Some(defect) = recording_defect(&bag) {
            last = format!("attempt {attempt}: {defect}");
            continue;
        }
        let live = ReadLog::collect(&reads).unwrap_or_else(|e| panic!("RI_HARNESS: {e}"));
        return RecordedRun {
            fixture,
            placement,
            dir,
            bag,
            live,
            live_dir: reads,
        };
    }
    panic!(
        "RI_HARNESS: no loss-free recording of {} in {RECORD_ATTEMPTS} attempts: {last}",
        fixture.name()
    );
}

/// How [`resim`] replays a recording.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplayMode {
    /// `bag play <BAG>`: plain playback, which republishes the recorded
    /// frames. It re-executes no node, so its read log is empty unless the
    /// playback feeds nodes, and an empty log fails
    /// [`Verdict::compared_floor`].
    Replay,
    /// `bag play <BAG> --resim all --verify`: the verdict mode. Exit 0 is a
    /// clean replay, 1 a data violation and 6 a diverged schedule (both still
    /// judged on their reads); exit 2 is a refusal ([`Replayed::Refused`]).
    ResimVerify,
    /// `bag play <BAG> --resim all`: the neutral re-execution, exit 0 on any
    /// completed run.
    ResimNeutral,
}

impl ReplayMode {
    pub fn label(self) -> &'static str {
        match self {
            ReplayMode::Replay => "replay",
            ReplayMode::ResimVerify => "resim_verify",
            ReplayMode::ResimNeutral => "resim_neutral",
        }
    }

    fn args(self) -> &'static [&'static str] {
        match self {
            ReplayMode::Replay => &[],
            ReplayMode::ResimVerify => &["--resim", "all", "--verify"],
            ReplayMode::ResimNeutral => &["--resim", "all"],
        }
    }

    /// Exit codes after which the run's reads are judged.
    fn judged_exits(self) -> &'static [i32] {
        match self {
            ReplayMode::ResimVerify => &[0, 1, 6],
            ReplayMode::Replay | ReplayMode::ResimNeutral => &[0],
        }
    }
}

/// How long one replay may run.
pub const REPLAY_TIMEOUT: Duration = Duration::from_secs(300);
/// Text the argument parser prints on a usage error, which also exits 2.
const USAGE_ERROR_MARKERS: &[&str] = &[
    "error: unexpected argument",
    "error: invalid value",
    "error: the following required arguments",
    "Usage: cerulion",
];

/// What one replay of a recording produced.
#[derive(Clone, Debug)]
pub enum Replayed {
    /// The replay ran: its exit code and every read its nodes made.
    Reads {
        exit: i32,
        log: ReadLog,
        artifacts: ReplayArtifacts,
    },
    /// The replay refused the recording (exit 2), with its stderr: a refusal
    /// by name, which is a verdict about the recording, not a harness failure.
    Refused { stderr: String },
}

/// The files one replay leaves in its run's directory.
#[derive(Clone, Debug)]
pub struct ReplayArtifacts {
    /// The replay's stderr.
    pub stderr: PathBuf,
    /// The `--report` JSON (verify legs).
    pub report: Option<PathBuf>,
    /// The `--record-out` re-recording (verify legs).
    pub record_out: Option<PathBuf>,
}

/// A judgement of one replay.
#[derive(Clone, Debug)]
pub enum Outcome {
    /// The replay ran and its reads were judged.
    Judged { exit: i32, verdict: Verdict },
    /// The replay refused the recording.
    Refused { stderr: String },
}

impl Outcome {
    /// The verdict, when the replay ran.
    pub fn verdict(&self) -> Option<&Verdict> {
        match self {
            Outcome::Judged { verdict, .. } => Some(verdict),
            Outcome::Refused { .. } => None,
        }
    }

    /// Print one `READ_IDENTITY` line per stream, or one refusal line.
    pub fn print(&self, fixture: &str, leg: &str) {
        match self {
            Outcome::Judged { exit, verdict } => {
                verdict.print(fixture, leg);
                eprintln!("READ_IDENTITY_EXIT fixture={fixture} leg={leg} exit={exit}");
            }
            Outcome::Refused { stderr } => {
                let first = stderr.lines().find(|l| !l.trim().is_empty()).unwrap_or("");
                eprintln!(
                    "READ_IDENTITY fixture={fixture} leg={leg} verdict=refused reason={first}"
                );
            }
        }
    }
}

/// [`judge`] a [`Replayed`]: a refusal is surfaced as [`Outcome::Refused`].
pub fn judge_replayed(
    fixture: Fixture,
    live: &ReadLog,
    replayed: &Replayed,
    floors: Floors,
) -> Result<Outcome, String> {
    match replayed {
        Replayed::Refused { stderr } => Ok(Outcome::Refused {
            stderr: stderr.clone(),
        }),
        Replayed::Reads { exit, log, .. } => Ok(Outcome::Judged {
            exit: *exit,
            verdict: judge(fixture, live, log, floors)?,
        }),
    }
}

/// Replay `run`'s bag under `mode` with the read log armed, `plant` naming one
/// read to misrecord (`<node>:<index>`, see [`PLANT_ENV`]). An exit outside
/// the mode's contract (a node failure, an internal error, a non-zero plain
/// playback other than a refusal) panics `RI_HARNESS:`.
pub fn resim(
    ws: &Workspace,
    run: &RecordedRun,
    mode: ReplayMode,
    plant: Option<(&str, usize)>,
) -> Replayed {
    let n = std::fs::read_dir(run.dir.path())
        .map(|d| d.count())
        .unwrap_or(0);
    let reads = run.dir.path().join(format!("{}_reads_{n}", mode.label()));
    std::fs::create_dir_all(&reads).expect("RI_HARNESS: replay reads dir");
    let home = run.dir.path().join("cerhome");
    let (report, record_out) = if mode == ReplayMode::ResimVerify {
        (
            Some(reads.with_extension("report.json")),
            Some(reads.with_extension("mcap")),
        )
    } else {
        (None, None)
    };
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["bag", "play"]).arg(&run.bag).args(mode.args());
    if let Some(path) = &report {
        cmd.arg("--report").arg(path);
    }
    if let Some(path) = &record_out {
        cmd.arg("--record-out").arg(path);
    }
    cmd.current_dir(&ws.root)
        // The replayed nodes load their cdylibs from this target directory.
        .env("CARGO_TARGET_DIR", &ws.target_dir)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_HOME", &home)
        .env(READS_DIR_ENV, &reads);
    scrub(&mut cmd);
    cmd.env_remove(SAMPLES_DIR_ENV);
    match plant {
        Some((node, index)) => {
            cmd.env(PLANT_ENV, format!("{node}:{index}"));
        }
        None => {
            cmd.env_remove(PLANT_ENV);
        }
    }
    let stdout_path = reads.with_extension("stdout");
    let stderr_path = reads.with_extension("stderr");
    cmd.stdout(Stdio::from(
        std::fs::File::create(&stdout_path).expect("RI_HARNESS: stdout file"),
    ))
    .stderr(Stdio::from(
        std::fs::File::create(&stderr_path).expect("RI_HARNESS: stderr file"),
    ));
    let mut guard = ChildGuard::single_process(cmd.spawn().expect("RI_HARNESS: spawn bag play"));
    let status = guard.wait_bounded(REPLAY_TIMEOUT).unwrap_or_else(|| {
        panic!(
            "RI_HARNESS: `bag play` ({}) of {} did not finish within {REPLAY_TIMEOUT:?}\nstderr:\n{}",
            mode.label(),
            run.bag.display(),
            mp_support::read_file(&stderr_path)
        )
    });
    let stderr = mp_support::read_file(&stderr_path);
    let code = status.code();
    match code {
        Some(2) => {
            assert!(
                !USAGE_ERROR_MARKERS.iter().any(|m| stderr.contains(m)),
                "RI_HARNESS: `bag play` ({}) exited 2 on a usage error, not a refusal\nstderr:\n{stderr}",
                mode.label()
            );
            Replayed::Refused { stderr }
        }
        Some(c) if mode.judged_exits().contains(&c) => Replayed::Reads {
            exit: c,
            log: ReadLog::collect(&reads).unwrap_or_else(|e| panic!("RI_HARNESS: {e}")),
            artifacts: ReplayArtifacts {
                stderr: stderr_path.clone(),
                report,
                record_out,
            },
        },
        _ => panic!(
            "RI_HARNESS: `bag play` ({}) of {} exited {code:?}, outside its contract\nstderr:\n{stderr}",
            mode.label(),
            run.bag.display()
        ),
    }
}

// ===========================================================================
// Divergence evidence.
// ===========================================================================

/// How many reads either side of a stream's first difference the evidence
/// prints.
pub const EVIDENCE_SPAN: usize = 6;

/// One kind-6 record of a bag's read log, decoded.
struct EvidenceRecord {
    step: u64,
    kind: u16,
    served: u64,
    popped: u32,
    run: u32,
    role: cerulion_core::read_outcome::ReadSiteRole,
}

/// A bag's per-rank node tables: `rank -> node ids` from its trace manifests.
fn bag_node_tables(reader: &cerulion_bag::BagReader) -> BTreeMap<u32, Vec<String>> {
    let mut out = BTreeMap::new();
    for rank in 0..64u32 {
        let Ok(Some(att)) =
            reader.attachment(&format!("__cerulion/trace_manifest_rank{rank}.json"))
        else {
            continue;
        };
        let ids = serde_json::from_slice::<serde_json::Value>(&att.data)
            .ok()
            .and_then(|v| {
                v.get("node_ids").and_then(|ids| {
                    ids.as_array().map(|a| {
                        a.iter()
                            .filter_map(|s| s.as_str().map(str::to_string))
                            .collect::<Vec<_>>()
                    })
                })
            })
            .unwrap_or_default();
        out.insert(rank, ids);
    }
    out
}

/// The input index of `input` on `node`, from the bag's trace manifests.
fn bag_input_index(reader: &cerulion_bag::BagReader, node: &str, input: &str) -> Option<u16> {
    for rank in 0..64u32 {
        let Ok(Some(att)) =
            reader.attachment(&format!("__cerulion/trace_manifest_rank{rank}.json"))
        else {
            continue;
        };
        let Ok(v) = serde_json::from_slice::<serde_json::Value>(&att.data) else {
            continue;
        };
        if let Some(list) = v
            .get("inputs")
            .and_then(|i| i.get(node))
            .and_then(|l| l.as_array())
        {
            return list
                .iter()
                .position(|n| n.as_str() == Some(input))
                .and_then(|p| u16::try_from(p).ok());
        }
    }
    None
}

/// The rank a trace record was written by: its `reserved` word without the
/// discard bit.
fn record_rank(r: &cerulion_core::trace_ring::TraceRingRecord) -> u32 {
    r.reserved & !cerulion_core::trace_ring::TRACE_DISCARD_BIT
}

/// What [`bag_evidence`] found in one bag.
struct BagEvidence {
    records: Vec<EvidenceRecord>,
    fires: BTreeMap<u64, usize>,
    /// Trace records scanned, and the ranks and node tables seen, so a lookup
    /// that matched nothing says so instead of printing zeros.
    scanned: usize,
    ranks: std::collections::BTreeSet<u32>,
    tables: BTreeMap<u32, Vec<String>>,
}

/// `node`'s read-log records on `input` and its fires per step, in file order.
///
/// A record's node is its rank's table entry at its `node_idx`; a record whose
/// rank has no table is matched against every table that holds `node` at that
/// index (a re-recording may write one table for every rank).
fn bag_evidence(bag: &Path, node: &str, input: &str) -> Result<BagEvidence, String> {
    use cerulion_core::trace_ring::{
        read_site_role, unpack_read_outcome_meta, unpack_read_outcome_popped,
        unpack_read_outcome_run, RECORD_TYPE_FIRE, RECORD_TYPE_READ_OUTCOME,
    };
    let reader = cerulion_bag::BagReader::open(bag).map_err(|e| format!("open {bag:?}: {e}"))?;
    let tables = bag_node_tables(&reader);
    let input_idx = bag_input_index(&reader, node, input);
    let is_node = |r: &cerulion_core::trace_ring::TraceRingRecord| match tables.get(&record_rank(r))
    {
        Some(t) => t.get(r.node_idx as usize).map(String::as_str) == Some(node),
        None => tables
            .values()
            .any(|t| t.get(r.node_idx as usize).map(String::as_str) == Some(node)),
    };
    let mut records = Vec::new();
    let mut fires: BTreeMap<u64, usize> = BTreeMap::new();
    let mut scanned = 0usize;
    let mut ranks = std::collections::BTreeSet::new();
    let iter = reader
        .trace_records()
        .map_err(|e| format!("trace of {bag:?}: {e}"))?;
    for item in iter {
        let r = item.map_err(|e| format!("trace record of {bag:?}: {e}"))?;
        scanned += 1;
        ranks.insert(record_rank(&r));
        if !is_node(&r) {
            continue;
        }
        if r.record_type == RECORD_TYPE_FIRE {
            *fires.entry(r.step).or_default() += 1;
        } else if r.record_type == RECORD_TYPE_READ_OUTCOME {
            let (idx, kind) = unpack_read_outcome_meta(r.global_level);
            if input_idx.is_some_and(|i| i != idx) {
                continue;
            }
            let annotation = kind == cerulion_core::trace_ring::READ_OUTCOME_PRODUCER;
            records.push(EvidenceRecord {
                step: r.step,
                kind,
                served: r.fire_time_ns,
                popped: if annotation {
                    0
                } else {
                    unpack_read_outcome_popped(r.duration_ns)
                },
                run: if annotation {
                    1
                } else {
                    unpack_read_outcome_run(r.duration_ns).max(1)
                },
                role: read_site_role(r.global_level),
            });
        }
    }
    Ok(BagEvidence {
        records,
        fires,
        scanned,
        ranks,
        tables,
    })
}

/// Print, for every stream of `verdict` that is not equal, what a box log needs
/// to explain it: the replay's violations and trace divergence (from its
/// `--report` and stderr), the recorded read-log records around the first
/// difference, and the live and replayed fire counts per step over the same
/// window. Every line starts `RI_EVIDENCE`.
pub fn print_divergence_evidence(run: &RecordedRun, replayed: &Replayed, verdict: &Verdict) {
    let Replayed::Reads {
        exit, artifacts, ..
    } = replayed
    else {
        return;
    };
    let divergent = verdict.divergent();
    if divergent.is_empty() && *exit != 6 {
        return;
    }
    eprintln!("RI_EVIDENCE exit={exit}");
    let stderr = mp_support::read_file(&artifacts.stderr);
    for line in stderr
        .lines()
        .filter(|l| {
            let l = l.to_ascii_lowercase();
            l.contains("violation")
                || l.contains("diverg")
                || l.contains("feed")
                || l.contains("shortfall")
                || l.contains("never_arrived")
                || l.contains("error")
        })
        .take(60)
    {
        eprintln!("RI_EVIDENCE stderr: {line}");
    }
    if let Some(report) = &artifacts.report {
        match std::fs::read(report)
            .ok()
            .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
        {
            Some(serde_json::Value::Object(map)) => {
                for (key, value) in &map {
                    let k = key.to_ascii_lowercase();
                    if k.contains("violation")
                        || k.contains("diverg")
                        || k.contains("read_identity")
                        || k.contains("read_log")
                        || k == "passed"
                    {
                        let text = value.to_string();
                        let cut: String = text.chars().take(4000).collect();
                        eprintln!("RI_EVIDENCE report {key}: {cut}");
                    }
                }
            }
            _ => eprintln!("RI_EVIDENCE report: absent or unreadable at {report:?}"),
        }
    }
    for row in divergent {
        let first = match &row.verdict {
            RowVerdict::Diverged { index, .. } => *index,
            _ => row.compared,
        };
        let (records, live_fires) = match bag_evidence(&run.bag, &row.node, &row.input) {
            Ok(v) => (v.records, v.fires),
            Err(e) => {
                eprintln!("RI_EVIDENCE recorded read log unreadable: {e}");
                continue;
            }
        };
        // Serving occurrences (a folded record counts its run) index the
        // live reads; the window holds every record between the serving reads
        // first - SPAN and first + SPAN, annotations and markers included.
        let lo = first.saturating_sub(EVIDENCE_SPAN);
        let hi = first + EVIDENCE_SPAN;
        let mut serving = 0usize;
        let mut window_steps: Vec<u64> = Vec::new();
        eprintln!(
            "RI_EVIDENCE node={} input={} first_diff={first} recorded records (serving index, step, kind, served, popped, role, run):",
            row.node, row.input
        );
        for r in &records {
            let serves = r.kind != cerulion_core::trace_ring::READ_OUTCOME_PRODUCER
                && r.kind != cerulion_core::trace_ring::READ_OUTCOME_TRUNCATED
                && r.served != cerulion_core::trace_ring::READ_OUTCOME_NO_FRAME
                && r.popped > 0;
            let start = serving;
            if serves {
                serving += r.run as usize;
            }
            if serving >= lo && start <= hi {
                window_steps.push(r.step);
                eprintln!(
                    "RI_EVIDENCE   [{start}] step={} kind={} served={} popped={} role={:?} run={}",
                    r.step,
                    r.kind,
                    if r.served == cerulion_core::trace_ring::READ_OUTCOME_NO_FRAME {
                        "none".to_string()
                    } else {
                        r.served.to_string()
                    },
                    r.popped,
                    r.role,
                    r.run
                );
            }
            if start > hi {
                break;
            }
        }
        let (Some(first_step), Some(last_step)) = (
            window_steps.iter().min().copied(),
            window_steps.iter().max().copied(),
        ) else {
            continue;
        };
        let replayed = artifacts
            .record_out
            .as_ref()
            .map(|p| (p, bag_evidence(p, &row.node, &row.input)));
        let replayed_fires = match &replayed {
            Some((_, Ok(v))) => {
                let total: usize = v.fires.values().sum();
                eprintln!(
                    "RI_EVIDENCE re-recording: {} trace record(s) scanned, ranks {:?}, node tables {:?}, \
                     {total} fire(s) of {} matched",
                    v.scanned, v.ranks, v.tables, row.node
                );
                Some(v.fires.clone())
            }
            Some((p, Err(e))) => {
                eprintln!("RI_EVIDENCE re-recording {p:?} unreadable: {e}");
                None
            }
            None => None,
        };
        for step in first_step..=last_step {
            eprintln!(
                "RI_EVIDENCE   fires step={step} live={} replayed={}",
                live_fires.get(&step).copied().unwrap_or(0),
                replayed_fires.as_ref().map_or_else(
                    || "unrecorded".to_string(),
                    |f| f.get(&step).copied().unwrap_or(0).to_string()
                )
            );
        }
    }
}
