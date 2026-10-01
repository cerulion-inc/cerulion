// SPDX-License-Identifier: AGPL-3.0-only
//! The `sample` verb: the latest few messages of an attached topic, structured.
//!
//! `sample` answers "what is this topic saying right now?" without a second
//! subscription. The daemon already drains every attached topic on its poll
//! thread; while a controller keeps sampling a topic, that drain also keeps the
//! newest frames in a small ring, and the verb reads the ring back.
//!
//! # Bounds
//!
//! Every bound is a named constant here, and each one is a hard cap rather than a
//! target:
//!
//! * **Rows**: a ring holds at most [`SAMPLE_MAX_ROWS`] frames; the oldest is
//!   evicted first, whatever the publish rate.
//! * **Bytes per frame**: only a frame of at most [`SAMPLE_MAX_FRAME_BYTES`] is
//!   kept whole. A larger frame (an image, a point cloud) keeps its header facts
//!   (`seq`, `ts_ns`, `size`) and no body, so a topic that is mostly bulk data
//!   costs a few dozen bytes per row.
//! * **Topics**: at most [`SAMPLE_MAX_TOPICS`] rings exist at once.
//! * **Time**: a ring lives for [`SAMPLE_ARM_TTL`] after the last `sample` that
//!   named its topic. A topic nobody is sampling keeps nothing, and costs the
//!   poll thread one empty-map lookup per drained topic.
//! * **Decode**: a decoded value is cut at [`FIELD_MAX_DEPTH`] levels,
//!   [`FIELD_MAX_NODES`] values, [`FIELD_MAX_ARRAY`] array elements and
//!   [`FIELD_MAX_STR`] string characters, so one reply stays small whatever the
//!   schema.
//!
//! # What it never does
//!
//! It never opens a tap, a subscriber or a netd demand. A topic that is not
//! attached has no ring to read, and the verb says so instead of attaching it
//! (an attach renders into the viewer, which a peek must not do as a side
//! effect). A frame is copied into a ring only on the poll thread's existing
//! drain, so sampling adds no wakeup and no thread.
//!
//! The decode runs on the controller's own thread against a snapshot of the ring,
//! never under the daemon's state lock.

use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::codegen::{FrameValue, FrameValueKind, FrameWalker, PrimArray, PrimType};
use cerulion_core::wire::WireHeader;
use serde_json::{Map, Number, Value};

use crate::protocol::SampleRow;

/// The most rows one `sample` reply carries, and the depth of every ring.
pub const SAMPLE_MAX_ROWS: usize = 20;

/// The `n` a request that names none gets.
pub const SAMPLE_DEFAULT_ROWS: usize = 5;

/// How long a ring survives after the last `sample` that named its topic.
pub const SAMPLE_ARM_TTL: Duration = Duration::from_secs(5);

/// The largest frame a ring keeps whole. Bigger frames keep their header facts
/// only (see the module docs).
pub const SAMPLE_MAX_FRAME_BYTES: usize = 16 * 1024;

/// The most topics that may hold a ring at once.
pub const SAMPLE_MAX_TOPICS: usize = 8;

/// Nesting levels a decoded `fields` object keeps before it is cut.
pub const FIELD_MAX_DEPTH: usize = 6;

/// Values (scalars, arrays and objects all count) one decoded frame emits.
pub const FIELD_MAX_NODES: usize = 512;

/// Array elements shown. A longer array renders as `{"len":N,"head":[...]}`.
pub const FIELD_MAX_ARRAY: usize = 16;

/// String characters shown; a longer string is cut and ends in `...`.
pub const FIELD_MAX_STR: usize = 256;

/// Top-level fields the one-line `summary` names before it says `(+N more)`.
const SUMMARY_MAX_FIELDS: usize = 4;

/// Characters the one-line `summary` may hold.
const SUMMARY_MAX_CHARS: usize = 160;

/// One retained frame: the header facts always, the body when it fits.
#[derive(Debug, Clone)]
pub struct RawSample {
    /// The wire sequence number the publisher stamped.
    pub seq: u32,
    /// The publisher's timestamp in nanoseconds.
    pub ts_ns: u64,
    /// The whole frame's length in bytes, header included.
    pub size: usize,
    /// The whole frame, when it is at most [`SAMPLE_MAX_FRAME_BYTES`].
    pub body: Option<Arc<[u8]>>,
}

/// One topic's ring and the moment it was last asked for.
#[derive(Debug)]
struct Ring {
    last_sampled: Instant,
    frames: VecDeque<RawSample>,
}

impl Ring {
    fn expired(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.last_sampled) >= SAMPLE_ARM_TTL
    }
}

/// Why a topic could not be armed for sampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmError {
    /// [`SAMPLE_MAX_TOPICS`] other topics hold a ring already.
    TooManyTopics,
}

/// Every live ring, keyed by absolute topic.
///
/// Lives inside the daemon's one state lock, so the poll thread (which feeds it)
/// and the controller threads (which arm and read it) never need a second lock.
#[derive(Debug, Default)]
pub struct SampleRings {
    rings: BTreeMap<String, Ring>,
}

impl SampleRings {
    /// Start (or keep alive) the ring for `topic`.
    ///
    /// Re-arming an existing ring only moves its deadline, so a controller that
    /// polls steadily keeps one ring and loses no frames between replies.
    pub fn arm(&mut self, topic: &str, now: Instant) -> Result<(), ArmError> {
        // A ring past its deadline is as good as absent: drop it first so a stale
        // ring cannot hand a new controller frames from before it asked.
        if self.rings.get(topic).is_some_and(|r| r.expired(now)) {
            self.rings.remove(topic);
        }
        if let Some(ring) = self.rings.get_mut(topic) {
            ring.last_sampled = now;
            return Ok(());
        }
        if self.rings.len() >= SAMPLE_MAX_TOPICS {
            return Err(ArmError::TooManyTopics);
        }
        self.rings.insert(
            topic.to_string(),
            Ring {
                last_sampled: now,
                frames: VecDeque::with_capacity(SAMPLE_MAX_ROWS),
            },
        );
        Ok(())
    }

    /// Feed one drained batch of `topic`'s frames to its ring, if it has one.
    ///
    /// Called from the poll thread for every drained topic. With no ring for the
    /// topic this is one map lookup and returns. With one, only the last
    /// [`SAMPLE_MAX_ROWS`] frames of the batch are looked at, so the copy cost of
    /// a pass is bounded by `SAMPLE_MAX_ROWS * SAMPLE_MAX_FRAME_BYTES` per ring.
    pub fn observe(&mut self, topic: &str, frames: &[Vec<u8>], now: Instant) {
        let Some(ring) = self.rings.get_mut(topic) else {
            return;
        };
        if ring.expired(now) {
            self.rings.remove(topic);
            return;
        }
        let skip = frames.len().saturating_sub(SAMPLE_MAX_ROWS);
        for frame in &frames[skip..] {
            // A frame shorter than a header is not a frame; the poll thread's own
            // schema resolution already reports it.
            let Some(header) = WireHeader::read_from_buf(frame) else {
                continue;
            };
            if ring.frames.len() == SAMPLE_MAX_ROWS {
                ring.frames.pop_front();
            }
            ring.frames.push_back(RawSample {
                seq: header.sequence,
                ts_ns: header.timestamp_ns,
                size: frame.len(),
                body: (frame.len() <= SAMPLE_MAX_FRAME_BYTES).then(|| Arc::from(frame.as_slice())),
            });
        }
    }

    /// Drop every ring past its deadline. Called once per poll pass.
    pub fn sweep(&mut self, now: Instant) {
        if !self.rings.is_empty() {
            self.rings.retain(|_, ring| !ring.expired(now));
        }
    }

    /// Drop `topic`'s ring (its tap is gone).
    pub fn forget(&mut self, topic: &str) {
        self.rings.remove(topic);
    }

    /// The newest `n` frames of `topic`, oldest first. Cheap: a frame body is a
    /// shared handle, so the copy made under the state lock holds no frame bytes.
    pub fn latest(&self, topic: &str, n: usize) -> Vec<RawSample> {
        let Some(ring) = self.rings.get(topic) else {
            return Vec::new();
        };
        let skip = ring.frames.len().saturating_sub(n);
        ring.frames.iter().skip(skip).cloned().collect()
    }

    /// How many topics hold a ring now (an observable for tests and logs).
    pub fn len(&self) -> usize {
        self.rings.len()
    }

    /// True when no topic holds a ring.
    pub fn is_empty(&self) -> bool {
        self.rings.is_empty()
    }
}

/// Turn retained frames into wire rows, decoding each against `walker`.
///
/// A row never fails the reply: a frame that cannot be decoded still yields its
/// header facts, with `fields` null and the reason in `summary`.
pub fn rows_for(samples: Vec<RawSample>, walker: &FrameWalker) -> Vec<SampleRow> {
    samples.into_iter().map(|s| row_for(s, walker)).collect()
}

fn row_for(sample: RawSample, walker: &FrameWalker) -> SampleRow {
    let size = u32::try_from(sample.size).unwrap_or(u32::MAX);
    let base = |fields: Option<Value>, summary: String| SampleRow {
        seq: u64::from(sample.seq),
        ts_ns: sample.ts_ns,
        size,
        fields,
        summary,
    };
    let Some(body) = sample.body.as_deref() else {
        return base(
            None,
            format!(
                "{} bytes, larger than the {SAMPLE_MAX_FRAME_BYTES} byte sampling limit \
                 (header only)",
                sample.size
            ),
        );
    };
    match walker.walk_by_hash(body) {
        Ok(value) => {
            let mut budget = FIELD_MAX_NODES;
            let fields = fields_json(&value, 0, &mut budget);
            let summary = summarize(&value, &fields);
            base(Some(fields), summary)
        }
        Err(e) => base(
            None,
            truncate_chars(
                &format!("{} bytes, not decoded: {e}", sample.size),
                SUMMARY_MAX_CHARS,
            ),
        ),
    }
}

/// A decoded message as a JSON object, `{field: value}`. Once the value budget
/// is spent the remaining fields are dropped and one `"..."` entry marks the cut,
/// so a very wide schema cannot widen the reply past the bound.
fn fields_json(value: &FrameValue<'_>, depth: usize, budget: &mut usize) -> Value {
    let mut map = Map::new();
    for field in &value.fields {
        if *budget == 0 {
            map.insert("...".to_string(), cut());
            break;
        }
        map.insert(field.name.clone(), kind_json(&field.value, depth, budget));
    }
    Value::Object(map)
}

/// The marker for a value cut by the depth or node bound.
fn cut() -> Value {
    Value::String("...".to_string())
}

fn kind_json(kind: &FrameValueKind<'_>, depth: usize, budget: &mut usize) -> Value {
    if *budget == 0 {
        return cut();
    }
    *budget -= 1;
    match kind {
        FrameValueKind::Bool(v) => Value::Bool(*v),
        FrameValueKind::I8(v) => Value::from(*v),
        FrameValueKind::U8(v) => Value::from(*v),
        FrameValueKind::I16(v) => Value::from(*v),
        FrameValueKind::U16(v) => Value::from(*v),
        FrameValueKind::I32(v) => Value::from(*v),
        FrameValueKind::U32(v) => Value::from(*v),
        FrameValueKind::I64(v) => Value::from(*v),
        FrameValueKind::U64(v) => Value::from(*v),
        FrameValueKind::F32(v) => f32_json(*v),
        FrameValueKind::F64(v) => float_json(*v),
        FrameValueKind::Str(s) => Value::String(truncate_chars(s, FIELD_MAX_STR)),
        FrameValueKind::Bytes(b) => array_json(b.len(), b.iter().map(|x| Value::from(*x)), budget),
        FrameValueKind::PrimArray(a) => prim_array_json(a, budget),
        FrameValueKind::Nested(inner) => {
            if depth >= FIELD_MAX_DEPTH {
                cut()
            } else {
                fields_json(inner, depth + 1, budget)
            }
        }
        FrameValueKind::Array(elements) | FrameValueKind::NestedArray { elements, .. } => {
            if depth >= FIELD_MAX_DEPTH {
                return cut();
            }
            let head: Vec<Value> = elements
                .iter()
                .take(FIELD_MAX_ARRAY)
                .map(|e| kind_json(e, depth + 1, budget))
                .collect();
            wrap_array(elements.len(), head)
        }
        FrameValueKind::NestedArrayOpaque(raw) => {
            let mut map = Map::new();
            map.insert("opaque_bytes".to_string(), Value::from(raw.len()));
            Value::Object(map)
        }
    }
}

fn prim_array_json(a: &PrimArray<'_>, budget: &mut usize) -> Value {
    let width = a.elem.size();
    let items = a.bytes.chunks_exact(width).take(a.count);
    // Each element is read at its own width, so a 64-bit integer keeps every bit
    // (a detour through `f64` would round it above 2^53).
    let elem = |c: &[u8]| -> Value {
        match a.elem {
            PrimType::F32 => f32_json(f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
            PrimType::F64 => float_json(f64::from_le_bytes([
                c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
            ])),
            PrimType::I16 => Value::from(i16::from_le_bytes([c[0], c[1]])),
            PrimType::U16 => Value::from(u16::from_le_bytes([c[0], c[1]])),
            PrimType::I32 => Value::from(i32::from_le_bytes([c[0], c[1], c[2], c[3]])),
            PrimType::U32 => Value::from(u32::from_le_bytes([c[0], c[1], c[2], c[3]])),
            PrimType::I64 => Value::from(i64::from_le_bytes([
                c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
            ])),
            PrimType::U64 => Value::from(u64::from_le_bytes([
                c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
            ])),
        }
    };
    array_json(a.count, items.map(elem), budget)
}

/// An array of `len` items as its first [`FIELD_MAX_ARRAY`] values, each one
/// charged to the frame's value budget (the array itself was charged by the
/// caller), so many short arrays cannot exceed [`FIELD_MAX_NODES`].
fn array_json(len: usize, items: impl Iterator<Item = Value>, budget: &mut usize) -> Value {
    let take = FIELD_MAX_ARRAY.min(*budget);
    let head: Vec<Value> = items.take(take).collect();
    *budget -= head.len();
    wrap_array(len, head)
}

/// An array renders as itself only when `head` holds every element; a longer
/// one, or one the value budget cut short, as `{"len":N,"head":[...]}`, so a
/// client can tell "these are all of them" from "these are the first few".
fn wrap_array(len: usize, head: Vec<Value>) -> Value {
    if len <= FIELD_MAX_ARRAY && head.len() == len {
        return Value::Array(head);
    }
    let mut map = Map::new();
    map.insert("len".to_string(), Value::from(len));
    map.insert("head".to_string(), Value::Array(head));
    Value::Object(map)
}

/// A finite float as a JSON number; NaN and the infinities (which JSON cannot
/// carry) as the strings `"NaN"`, `"inf"` and `"-inf"`.
fn float_json(v: f64) -> Value {
    match Number::from_f64(v) {
        Some(n) => Value::Number(n),
        None if v.is_nan() => Value::String("NaN".to_string()),
        None if v.is_sign_negative() => Value::String("-inf".to_string()),
        None => Value::String("inf".to_string()),
    }
}

/// An `f32` through the shortest decimal that names it, so `0.1f32` reports
/// `0.1` rather than `0.10000000149011612`.
fn f32_json(v: f32) -> Value {
    float_json(v.to_string().parse::<f64>().unwrap_or(f64::from(v)))
}

fn truncate_chars(s: &str, max: usize) -> String {
    if s.chars().count() <= max {
        return s.to_string();
    }
    let mut out: String = s.chars().take(max).collect();
    out.push_str("...");
    out
}

/// One line naming the schema and the first few fields, for a client that wants
/// a label and not a tree.
fn summarize(value: &FrameValue<'_>, fields: &Value) -> String {
    let Some(map) = fields.as_object() else {
        return value.schema_name.clone();
    };
    let mut parts: Vec<String> = Vec::new();
    for field in value.fields.iter().take(SUMMARY_MAX_FIELDS) {
        let Some(v) = map.get(&field.name) else {
            continue;
        };
        parts.push(format!("{}={}", field.name, brief(v)));
    }
    let more = value.fields.len().saturating_sub(SUMMARY_MAX_FIELDS);
    let mut line = if parts.is_empty() {
        value.schema_name.clone()
    } else {
        format!("{}: {}", value.schema_name, parts.join(", "))
    };
    if more > 0 {
        line.push_str(&format!(" (+{more} more)"));
    }
    // A decoded string may hold a newline or other control character; the
    // summary stays one line whatever the message says.
    let line: String = line
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect();
    truncate_chars(&line, SUMMARY_MAX_CHARS)
}

/// A value as a short token: a scalar to three significant figures, a container
/// to its size.
fn brief(v: &Value) -> String {
    match v {
        Value::Null => "null".to_string(),
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => match n.as_f64() {
            Some(f) if n.is_f64() => sig3(f),
            _ => n.to_string(),
        },
        Value::String(s) => truncate_chars(s, 24),
        Value::Array(a) => format!("[{}]", a.len()),
        Value::Object(o) => match o.get("len").and_then(Value::as_u64) {
            Some(len) if o.contains_key("head") => format!("[{len}]"),
            _ => format!("{{{}}}", o.len()),
        },
    }
}

/// A float to three significant figures, trailing zeros dropped. Magnitudes
/// below 1e-4 or from 1e3 up use scientific notation (`1.5e-10`, `1.23e4`), so a
/// tiny nonzero value never reads as `0` and a large one is still three figures.
fn sig3(f: f64) -> String {
    if f == 0.0 {
        return "0".to_string();
    }
    if !f.is_finite() {
        return f.to_string();
    }
    let magnitude = f.abs().log10().floor() as i32;
    if !(-4..3).contains(&magnitude) {
        let text = format!("{f:.2e}");
        return match text.split_once('e') {
            Some((m, e)) => {
                let m = if m.contains('.') {
                    m.trim_end_matches('0').trim_end_matches('.')
                } else {
                    m
                };
                format!("{m}e{e}")
            }
            None => text,
        };
    }
    let decimals = (2 - magnitude).clamp(0, 6) as usize;
    let text = format!("{f:.decimals$}");
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::NamedValue;

    fn frame(seq: u32, ts: u64, payload_len: usize) -> Vec<u8> {
        let total = WireHeader::SIZE + payload_len;
        let header = WireHeader {
            schema_hash: 0xDEAD_BEEF,
            total_size: total as u32,
            offset_table_offset: total as u32,
            offset_table_count: 0,
            sequence: seq,
            timestamp_ns: ts,
        };
        let mut buf = vec![0u8; total];
        header.write_to_buf(&mut buf[..WireHeader::SIZE]);
        buf
    }

    fn t0() -> Instant {
        Instant::now()
    }

    #[test]
    fn a_topic_with_no_ring_keeps_nothing() {
        let mut rings = SampleRings::default();
        rings.observe("/a", &[frame(1, 10, 8)], t0());
        assert!(rings.is_empty());
        assert!(rings.latest("/a", 5).is_empty());
    }

    #[test]
    fn the_ring_holds_the_newest_twenty_oldest_first() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        // 45 frames across three batches: only seq 25..=44 may survive.
        let batch = |range: std::ops::Range<u32>| -> Vec<Vec<u8>> {
            range.map(|s| frame(s, u64::from(s) * 100, 8)).collect()
        };
        rings.observe("/a", &batch(0..15), now);
        rings.observe("/a", &batch(15..30), now);
        rings.observe("/a", &batch(30..45), now);
        let all = rings.latest("/a", 100);
        let seqs: Vec<u32> = all.iter().map(|s| s.seq).collect();
        assert_eq!(seqs, (25..45).collect::<Vec<u32>>());
        let last3: Vec<u32> = rings.latest("/a", 3).iter().map(|s| s.seq).collect();
        assert_eq!(last3, vec![42, 43, 44]);
        assert_eq!(all[0].ts_ns, 2500);
    }

    #[test]
    fn one_huge_batch_is_cut_to_its_last_twenty_before_any_copy() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        let frames: Vec<Vec<u8>> = (0..1000u32).map(|s| frame(s, 0, 8)).collect();
        rings.observe("/a", &frames, now);
        let seqs: Vec<u32> = rings.latest("/a", 100).iter().map(|s| s.seq).collect();
        assert_eq!(seqs, (980..1000).collect::<Vec<u32>>());
    }

    #[test]
    fn a_frame_over_the_byte_limit_keeps_its_header_and_no_body() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/img", now).unwrap();
        let at_limit = frame(1, 5, SAMPLE_MAX_FRAME_BYTES - WireHeader::SIZE);
        let over = frame(2, 6, SAMPLE_MAX_FRAME_BYTES - WireHeader::SIZE + 1);
        assert_eq!(at_limit.len(), SAMPLE_MAX_FRAME_BYTES);
        rings.observe("/img", &[at_limit, over], now);
        let got = rings.latest("/img", 5);
        assert!(
            got[0].body.is_some(),
            "a frame exactly at the limit is kept"
        );
        assert!(got[1].body.is_none(), "one byte over keeps no body");
        assert_eq!(got[1].size, SAMPLE_MAX_FRAME_BYTES + 1);
        assert_eq!((got[1].seq, got[1].ts_ns), (2, 6));
    }

    #[test]
    fn a_short_frame_is_not_recorded() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        rings.observe("/a", &[vec![0u8; WireHeader::SIZE - 1]], now);
        assert!(rings.latest("/a", 5).is_empty());
    }

    #[test]
    fn a_ring_expires_five_seconds_after_the_last_sample_and_not_before() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        rings.observe("/a", &[frame(1, 1, 8)], now);

        let almost = now + SAMPLE_ARM_TTL - Duration::from_millis(1);
        rings.sweep(almost);
        assert_eq!(rings.len(), 1, "alive just inside the deadline");

        // A re-arm moves the deadline and keeps the frames.
        rings.arm("/a", almost).unwrap();
        rings.sweep(almost + SAMPLE_ARM_TTL - Duration::from_millis(1));
        assert_eq!(rings.latest("/a", 5).len(), 1);

        rings.sweep(almost + SAMPLE_ARM_TTL);
        assert!(rings.is_empty(), "gone at the deadline");
    }

    #[test]
    fn an_expired_ring_is_not_fed_and_a_fresh_arm_starts_empty() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        let later = now + SAMPLE_ARM_TTL + Duration::from_secs(1);
        // The poll thread has not swept yet; observe must still not keep frames.
        rings.observe("/a", &[frame(9, 9, 8)], later);
        assert!(rings.is_empty());
        // And an arm over a stale ring never resurrects its frames.
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        rings.observe("/a", &[frame(1, 1, 8)], now);
        rings.arm("/a", later).unwrap();
        assert!(rings.latest("/a", 5).is_empty());
    }

    #[test]
    fn at_most_eight_topics_hold_a_ring() {
        let now = t0();
        let mut rings = SampleRings::default();
        for i in 0..SAMPLE_MAX_TOPICS {
            rings.arm(&format!("/t{i}"), now).unwrap();
        }
        assert_eq!(
            rings.arm("/one_too_many", now),
            Err(ArmError::TooManyTopics)
        );
        // An existing topic can still be re-armed at the cap.
        assert_eq!(rings.arm("/t0", now), Ok(()));
        // Freeing one slot admits a newcomer.
        rings.forget("/t3");
        assert_eq!(rings.arm("/one_too_many", now), Ok(()));
        assert_eq!(rings.len(), SAMPLE_MAX_TOPICS);
    }

    #[test]
    fn forget_drops_the_ring_and_its_frames() {
        let now = t0();
        let mut rings = SampleRings::default();
        rings.arm("/a", now).unwrap();
        rings.observe("/a", &[frame(1, 1, 8)], now);
        rings.forget("/a");
        assert!(rings.is_empty());
        rings.observe("/a", &[frame(2, 2, 8)], now);
        assert!(
            rings.is_empty(),
            "a forgotten topic is not re-armed by a drain"
        );
    }

    // ---- decode oracles: every expected value is written by hand ----

    fn nv<'a>(name: &str, value: FrameValueKind<'a>) -> NamedValue<'a> {
        NamedValue {
            name: name.to_string(),
            value,
        }
    }

    fn msg<'a>(schema: &str, fields: Vec<NamedValue<'a>>) -> FrameValue<'a> {
        FrameValue {
            schema_name: schema.to_string(),
            fields,
        }
    }

    fn json_of(value: &FrameValue<'_>) -> Value {
        let mut budget = FIELD_MAX_NODES;
        fields_json(value, 0, &mut budget)
    }

    #[test]
    fn scalars_decode_to_their_json_types() {
        let v = msg(
            "t/All",
            vec![
                nv("b", FrameValueKind::Bool(true)),
                nv("i", FrameValueKind::I32(-7)),
                nv("u", FrameValueKind::U64(u64::MAX)),
                nv("f", FrameValueKind::F64(2.5)),
                nv("s", FrameValueKind::Str("hi")),
            ],
        );
        assert_eq!(
            json_of(&v),
            serde_json::json!({"b": true, "i": -7, "u": u64::MAX, "f": 2.5, "s": "hi"})
        );
    }

    #[test]
    fn an_f32_reports_its_shortest_decimal() {
        let v = msg("t/F", vec![nv("x", FrameValueKind::F32(0.1))]);
        assert_eq!(json_of(&v), serde_json::json!({"x": 0.1}));
    }

    #[test]
    fn non_finite_floats_become_strings() {
        let v = msg(
            "t/N",
            vec![
                nv("a", FrameValueKind::F64(f64::NAN)),
                nv("b", FrameValueKind::F64(f64::INFINITY)),
                nv("c", FrameValueKind::F64(f64::NEG_INFINITY)),
            ],
        );
        assert_eq!(
            json_of(&v),
            serde_json::json!({"a": "NaN", "b": "inf", "c": "-inf"})
        );
    }

    #[test]
    fn a_short_array_is_whole_and_a_long_one_says_how_long() {
        let sixteen: Vec<u8> = (0..16).collect();
        let seventeen: Vec<u8> = (0..17).collect();
        let v = msg(
            "t/A",
            vec![
                nv("short", FrameValueKind::Bytes(&sixteen)),
                nv("long", FrameValueKind::Bytes(&seventeen)),
            ],
        );
        let got = json_of(&v);
        assert_eq!(
            got["short"],
            serde_json::json!((0..16).collect::<Vec<u8>>())
        );
        assert_eq!(got["long"]["len"], 17);
        assert_eq!(
            got["long"]["head"],
            serde_json::json!((0..16).collect::<Vec<u8>>())
        );
    }

    #[test]
    fn a_numeric_array_decodes_by_element_type() {
        let mut bytes = Vec::new();
        for x in [1.5f64, -2.0, 0.25] {
            bytes.extend_from_slice(&x.to_le_bytes());
        }
        let arr = PrimArray {
            elem: PrimType::F64,
            bytes: &bytes,
            count: 3,
        };
        let v = msg("t/P", vec![nv("cov", FrameValueKind::PrimArray(arr))]);
        assert_eq!(json_of(&v), serde_json::json!({"cov": [1.5, -2.0, 0.25]}));

        let mut ibytes = Vec::new();
        for x in [-3i32, 4] {
            ibytes.extend_from_slice(&x.to_le_bytes());
        }
        let iarr = PrimArray {
            elem: PrimType::I32,
            bytes: &ibytes,
            count: 2,
        };
        let v = msg("t/P", vec![nv("k", FrameValueKind::PrimArray(iarr))]);
        assert_eq!(json_of(&v), serde_json::json!({"k": [-3, 4]}));
    }

    #[test]
    fn a_long_string_is_cut_with_an_ellipsis() {
        let long = "x".repeat(FIELD_MAX_STR + 10);
        let v = msg("t/S", vec![nv("s", FrameValueKind::Str(&long))]);
        let got = json_of(&v);
        let s = got["s"].as_str().unwrap();
        assert_eq!(s.chars().count(), FIELD_MAX_STR + 3);
        assert!(s.ends_with("..."));
    }

    #[test]
    fn nesting_is_cut_at_the_depth_bound() {
        // depth 0 object -> nested x7. The 7th nested level (depth 6) is cut.
        let mut inner = msg("t/Leaf", vec![nv("v", FrameValueKind::I32(1))]);
        for _ in 0..8 {
            inner = msg(
                "t/Wrap",
                vec![nv("n", FrameValueKind::Nested(Box::new(inner)))],
            );
        }
        let got = json_of(&inner);
        let mut cur = &got;
        let mut levels = 0;
        while let Some(next) = cur.get("n") {
            cur = next;
            levels += 1;
            if cur.is_string() {
                break;
            }
        }
        assert_eq!(cur, &Value::String("...".to_string()));
        assert_eq!(levels, FIELD_MAX_DEPTH + 1);
    }

    #[test]
    fn the_node_budget_cuts_a_wide_message() {
        let fields: Vec<NamedValue<'_>> = (0..FIELD_MAX_NODES + 50)
            .map(|i| nv(&format!("f{i:04}"), FrameValueKind::I32(i as i32)))
            .collect();
        let v = msg("t/Wide", fields);
        let got = json_of(&v);
        let obj = got.as_object().unwrap();
        assert_eq!(
            obj.len(),
            FIELD_MAX_NODES + 1,
            "the budget, then one marker"
        );
        let real = obj.values().filter(|x| x.is_number()).count();
        assert_eq!(real, FIELD_MAX_NODES);
        assert_eq!(obj.get("..."), Some(&Value::String("...".to_string())));
    }

    #[test]
    fn many_short_arrays_stay_inside_the_node_budget() {
        let bytes = [7u8; FIELD_MAX_ARRAY];
        let fields: Vec<NamedValue<'_>> = (0..FIELD_MAX_NODES)
            .map(|i| nv(&format!("a{i:04}"), FrameValueKind::Bytes(&bytes)))
            .collect();
        let v = msg("t/Arrays", fields);
        let got = json_of(&v);
        let mut values = 0usize;
        for arr in got.as_object().unwrap().values() {
            values += 1;
            if let Some(a) = arr.as_array() {
                values += a.len();
            }
        }
        assert!(values <= FIELD_MAX_NODES + 1, "{values} values emitted");
    }

    #[test]
    fn a_short_array_cut_by_the_budget_keeps_its_length() {
        let bytes = [7u8; FIELD_MAX_ARRAY];
        let mut fields: Vec<NamedValue<'_>> = (0..FIELD_MAX_NODES - 1)
            .map(|i| nv(&format!("f{i:04}"), FrameValueKind::I32(i as i32)))
            .collect();
        fields.push(nv("zdata", FrameValueKind::Bytes(&bytes)));
        let got = json_of(&msg("t/Cut", fields));
        let data = got.get("zdata").expect("the array field");
        assert_eq!(data.get("len").and_then(Value::as_u64), Some(16), "{data}");
        let head = data.get("head").and_then(Value::as_array).expect("head");
        assert!(head.is_empty(), "no budget left for elements: {data}");
    }

    #[test]
    fn a_newline_in_a_string_field_keeps_the_summary_on_one_line() {
        let v = msg("t/Text", vec![nv("s", FrameValueKind::Str("a\nb\r\tc"))]);
        let line = summarize(&v, &json_of(&v));
        assert_eq!(line, "t/Text: s=a b  c");
        assert!(!line.chars().any(char::is_control));
    }

    #[test]
    fn the_summary_names_the_schema_and_three_significant_figures() {
        let v = msg(
            "geometry_msgs/Vector3",
            vec![
                nv("x", FrameValueKind::F64(1.23456)),
                nv("y", FrameValueKind::F64(1234.5678)),
                nv("z", FrameValueKind::F64(0.0)),
            ],
        );
        let fields = json_of(&v);
        assert_eq!(
            summarize(&v, &fields),
            "geometry_msgs/Vector3: x=1.23, y=1.23e3, z=0"
        );
    }

    #[test]
    fn the_summary_counts_fields_it_leaves_out() {
        let fields: Vec<NamedValue<'_>> = (0..6)
            .map(|i| nv(&format!("f{i}"), FrameValueKind::I32(i)))
            .collect();
        let v = msg("t/Six", fields);
        let j = json_of(&v);
        assert_eq!(summarize(&v, &j), "t/Six: f0=0, f1=1, f2=2, f3=3 (+2 more)");
    }

    #[test]
    fn sig3_oracles() {
        assert_eq!(sig3(0.0), "0");
        assert_eq!(sig3(1.0), "1");
        assert_eq!(sig3(0.000123456), "0.000123");
        assert_eq!(sig3(-45.678), "-45.7");
        assert_eq!(sig3(99999.9), "1e5");
        assert_eq!(sig3(1.0e-10), "1e-10");
        assert_eq!(sig3(-1.5e-10), "-1.5e-10");
        assert_eq!(sig3(12345.0), "1.23e4");
        assert_eq!(sig3(999.0), "999");
        assert_eq!(sig3(0.0001), "0.0001");
    }

    #[test]
    fn a_row_without_a_body_or_a_known_schema_has_null_fields_and_a_reason() {
        let (walker, _) = FrameWalker::new(Vec::new());
        let header_only = RawSample {
            seq: 3,
            ts_ns: 30,
            size: 99_999,
            body: None,
        };
        let unknown = RawSample {
            seq: 4,
            ts_ns: 40,
            size: WireHeader::SIZE,
            body: Some(Arc::from(frame(4, 40, 0).as_slice())),
        };
        let rows = rows_for(vec![header_only, unknown], &walker);
        assert_eq!(rows[0].seq, 3);
        assert_eq!(rows[0].size, 99_999);
        assert!(rows[0].fields.is_none());
        assert!(rows[0].summary.starts_with("99999 bytes, larger than"));
        assert_eq!(rows[1].ts_ns, 40);
        assert!(rows[1].fields.is_none());
        assert!(
            rows[1].summary.contains("not decoded"),
            "{}",
            rows[1].summary
        );
    }
}
