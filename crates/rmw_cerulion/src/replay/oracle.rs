// SPDX-License-Identifier: AGPL-3.0-only
//! The divergence oracle and the replay verdict.
//!
//! The unit of comparison is a SEGMENT: the outputs one thread makes between
//! two consecutive rmw events on that thread (a window's wait return or a
//! take). A take opens the segment of the subscription callback it feeds; a
//! timer's outputs fall in the segment of the wait return before the next take
//! (rcl timer calls are invisible to the rmw, so back-to-back timers share one
//! segment and the culprit is named down to the segment, not the timer).
//!
//! Each replayed segment's `(entity, payload hash, return)` list is compared
//! with the recorded segment of the same anchor. The FIRST segment whose
//! inputs matched and whose outputs differ is the culprit; a difference after
//! an input mismatch is inherited and counted, never named as the culprit.
//! Outputs on `/rosout` are recorded and counted but never compared: rcl stamps
//! every log line with the wall clock. Nor are a thread's outputs before its
//! first rmw event (construction time): they cannot depend on a delivery, and
//! rclpy's construction-time `/parameter_events` publish carries a wall-clock
//! stamp, because `use_sim_time` takes effect only after it.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde::Serialize;

use super::log::{EntityId, OutputRec, Record};

/// The topic whose outputs are excluded from the comparison.
pub(crate) const ROSOUT: &str = "/rosout";

/// The comparable part of an output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Out {
    /// The RECORDED entity id (an unrecorded publisher is [`UNKNOWN_ENTITY`]).
    pub entity: EntityId,
    pub hash: u64,
    pub len: u32,
    pub has_payload: bool,
    pub ret: i32,
}

/// The entity id of a replayed output whose publisher the recording lacks.
pub(crate) const UNKNOWN_ENTITY: EntityId = u32::MAX;

impl Out {
    pub(crate) fn of(rec: &OutputRec) -> Out {
        Out {
            entity: rec.entity,
            hash: rec.hash,
            len: rec.len,
            has_payload: rec.has_payload,
            ret: rec.ret,
        }
    }
}

/// Where a recorded event sits, for naming a culprit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct AnchorInfo {
    /// Index of the window (in the process's window order) at or before it.
    window: usize,
    /// Deliveries taken on that thread up to and including it.
    delivery: usize,
}

/// The first divergent segment whose inputs matched.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub(crate) struct Culprit {
    pub node: String,
    pub topic: String,
    pub entity: String,
    /// The process-wide window index of the segment's window.
    pub window: usize,
    /// Deliveries the thread had taken when the segment opened.
    pub delivery: usize,
    /// The recorded thread.
    pub thread: u32,
    /// The order key of the recorded event the segment follows (0: before the
    /// thread's first event).
    pub anchor_k: u64,
    /// Position of the first differing output in the segment.
    pub output_index: usize,
    pub recorded_outputs: usize,
    pub replayed_outputs: usize,
}

/// The first input that differed from the recording.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub(crate) struct InputMismatch {
    pub what: String,
    pub thread: Option<u32>,
}

/// A watchdog report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub(crate) struct WatchdogReport {
    pub holder_thread: u32,
    pub entity: String,
    pub node: String,
    pub window: usize,
    pub waiting_thread: u32,
}

/// The verdict written to `<log>.verdict.json`. Contains no time or pid, so
/// two replays of one log write byte-identical verdicts when they behave the
/// same.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub(crate) struct Verdict {
    /// `identical`, `diverged`, `input_mismatch`, `log_exhausted_early`,
    /// `replay_stopped_early`, `watchdog` or `refused`.
    pub verdict: String,
    pub process: String,
    pub log: String,
    pub log_complete: bool,
    pub windows_recorded: usize,
    pub units_recorded: usize,
    pub units_served: usize,
    pub windows_served_as_sets: usize,
    pub windows_split: usize,
    pub segments_compared: usize,
    pub segments_divergent: usize,
    pub segments_inherited: usize,
    pub outputs_compared: usize,
    pub outputs_excluded: usize,
    pub outputs_before_first_event: usize,
    pub outputs_off_window: usize,
    pub guards_not_triggered_live: usize,
    pub culprit: Option<Culprit>,
    pub input_mismatch: Option<InputMismatch>,
    pub watchdog: Option<WatchdogReport>,
    pub refused: Option<String>,
}

#[derive(Default)]
struct LiveSegment {
    /// The recorded thread this live thread replays (set at its first wait).
    rec_thread: Option<u32>,
    anchor: u64,
    outs: Vec<Out>,
    /// Outputs made before the thread was mapped (construction time): kept
    /// until the mapping names the recorded thread.
    pending_unmapped: bool,
}

/// What the oracle needs to know about each recorded entity.
#[derive(Clone, Debug, Default)]
pub(crate) struct EntityNames {
    pub name: HashMap<EntityId, String>,
    pub topic: HashMap<EntityId, String>,
    pub node: HashMap<EntityId, String>,
}

/// The segment oracle.
pub(crate) struct Oracle {
    names: EntityNames,
    excluded: HashSet<EntityId>,
    recorded: HashMap<(u32, u64), Vec<Out>>,
    anchors: HashMap<(u32, u64), AnchorInfo>,
    /// Recorded threads that ever waited or took (outputs of other threads are
    /// off-window and never compared).
    event_threads: HashSet<u32>,
    visited: HashSet<(u32, u64)>,
    live: BTreeMap<u32, LiveSegment>,
    inputs_ok: bool,
    pub(crate) culprit: Option<Culprit>,
    pub(crate) input_mismatch: Option<InputMismatch>,
    pub(crate) segments_compared: usize,
    pub(crate) segments_divergent: usize,
    pub(crate) segments_inherited: usize,
    pub(crate) outputs_compared: usize,
    pub(crate) outputs_excluded: usize,
    pub(crate) outputs_before_first_event: usize,
    pub(crate) outputs_off_window: usize,
}

impl Oracle {
    /// Build the recorded side from a log's records (in `k` order).
    pub(crate) fn new(records: &[Record], names: EntityNames) -> Oracle {
        let excluded: HashSet<EntityId> = names
            .topic
            .iter()
            .filter(|(_, t)| t.as_str() == ROSOUT)
            .map(|(id, _)| *id)
            .collect();
        let mut recorded: HashMap<(u32, u64), Vec<Out>> = HashMap::new();
        let mut anchors = HashMap::new();
        let mut current: HashMap<u32, u64> = HashMap::new();
        let mut deliveries: HashMap<u32, usize> = HashMap::new();
        let mut last_window: HashMap<u32, usize> = HashMap::new();
        let mut window_index = 0usize;
        let mut event_threads = HashSet::new();
        for rec in records {
            match rec {
                Record::Wait(w) => {
                    event_threads.insert(w.thread);
                    current.insert(w.thread, w.k);
                    last_window.insert(w.thread, window_index);
                    anchors.insert(
                        (w.thread, w.k),
                        AnchorInfo {
                            window: window_index,
                            delivery: deliveries.get(&w.thread).copied().unwrap_or(0),
                        },
                    );
                    window_index += 1;
                }
                Record::Take(t) => {
                    event_threads.insert(t.thread);
                    current.insert(t.thread, t.k);
                    let d = deliveries.entry(t.thread).or_default();
                    if t.taken {
                        *d += 1;
                    }
                    anchors.insert(
                        (t.thread, t.k),
                        AnchorInfo {
                            window: last_window.get(&t.thread).copied().unwrap_or(0),
                            delivery: *d,
                        },
                    );
                }
                Record::Output(o) => {
                    let anchor = current.get(&o.thread).copied().unwrap_or(0);
                    recorded
                        .entry((o.thread, anchor))
                        .or_default()
                        .push(Out::of(o));
                }
                _ => {}
            }
        }
        Oracle {
            names,
            excluded,
            recorded,
            anchors,
            event_threads,
            visited: HashSet::new(),
            live: BTreeMap::new(),
            inputs_ok: true,
            culprit: None,
            input_mismatch: None,
            segments_compared: 0,
            segments_divergent: 0,
            segments_inherited: 0,
            outputs_compared: 0,
            outputs_excluded: 0,
            outputs_before_first_event: 0,
            outputs_off_window: 0,
        }
    }

    /// The live thread `live` replays recorded thread `rec` (its first wait).
    /// Construction-time outputs made before this are compared against the
    /// recorded thread's pre-event segment.
    pub(crate) fn map_thread(&mut self, live: u32, rec: u32) {
        let seg = self.live.entry(live).or_default();
        if seg.rec_thread.is_none() {
            seg.rec_thread = Some(rec);
            seg.pending_unmapped = false;
        }
    }

    /// An input differed from the recording: every later difference is
    /// inherited.
    pub(crate) fn input_mismatch(&mut self, what: String, thread: Option<u32>) {
        self.inputs_ok = false;
        if self.input_mismatch.is_none() {
            self.input_mismatch = Some(InputMismatch { what, thread });
        }
    }

    /// A replayed rmw event on `live`, standing for the recorded event with key
    /// `anchor`: closes the open segment and opens the next.
    pub(crate) fn event(&mut self, live: u32, anchor: u64) {
        self.close(live);
        let seg = self.live.entry(live).or_default();
        seg.anchor = anchor;
    }

    /// A replayed output on `live`.
    pub(crate) fn output(&mut self, live: u32, out: Out) {
        if self.excluded.contains(&out.entity) {
            self.outputs_excluded += 1;
            return;
        }
        let seg = self.live.entry(live).or_default();
        if seg.rec_thread.is_none() {
            seg.pending_unmapped = true;
        }
        seg.outs.push(out);
    }

    fn close(&mut self, live: u32) {
        let Some(seg) = self.live.get_mut(&live) else {
            return;
        };
        let outs = std::mem::take(&mut seg.outs);
        let anchor = seg.anchor;
        let Some(rec_thread) = seg.rec_thread else {
            // A thread that never waits: its outputs are off-window.
            self.outputs_off_window += outs.len();
            return;
        };
        if !self.event_threads.contains(&rec_thread) {
            self.outputs_off_window += outs.len();
            return;
        }
        let key = (rec_thread, anchor);
        let first_visit = self.visited.insert(key);
        if anchor == 0 {
            // Construction time: counted, never compared (see the module docs).
            self.outputs_before_first_event += outs.len();
            return;
        }
        let recorded: Vec<Out> = if first_visit {
            self.recorded
                .get(&key)
                .map(|v| {
                    v.iter()
                        .copied()
                        .filter(|o| !self.excluded.contains(&o.entity))
                        .collect()
                })
                .unwrap_or_default()
        } else {
            // A second segment for one anchor (a split window's later units
            // continue it): its outputs were already compared.
            Vec::new()
        };
        if outs.is_empty() && recorded.is_empty() {
            return;
        }
        self.segments_compared += 1;
        self.outputs_compared += outs.len().max(recorded.len());
        if outs == recorded {
            return;
        }
        if !self.inputs_ok {
            self.segments_inherited += 1;
            return;
        }
        self.segments_divergent += 1;
        if self.culprit.is_some() {
            return;
        }
        let at = outs
            .iter()
            .zip(&recorded)
            .position(|(a, b)| a != b)
            .unwrap_or(outs.len().min(recorded.len()));
        let entity = outs
            .get(at)
            .or_else(|| recorded.get(at))
            .map(|o| o.entity)
            .unwrap_or(UNKNOWN_ENTITY);
        let info = self.anchors.get(&key).copied().unwrap_or_default();
        let lookup = |m: &HashMap<EntityId, String>| {
            m.get(&entity)
                .cloned()
                .unwrap_or_else(|| "<an entity the recording does not have>".to_string())
        };
        self.culprit = Some(Culprit {
            node: lookup(&self.names.node),
            topic: lookup(&self.names.topic),
            entity: lookup(&self.names.name),
            window: info.window,
            delivery: info.delivery,
            thread: rec_thread,
            anchor_k: anchor,
            output_index: at,
            recorded_outputs: recorded.len(),
            replayed_outputs: outs.len(),
        });
    }

    /// Close every open segment (replay end).
    pub(crate) fn close_all(&mut self) {
        let keys: Vec<u32> = self.live.keys().copied().collect();
        for k in keys {
            self.close(k);
        }
    }

    /// The node and entity names of a recorded entity, for diagnostics.
    pub(crate) fn describe(&self, entity: EntityId) -> (String, String) {
        (
            self.names
                .name
                .get(&entity)
                .cloned()
                .unwrap_or_else(|| format!("entity#{entity}")),
            self.names.node.get(&entity).cloned().unwrap_or_default(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::replay::log::{OutputKind, TakeKind, TakeRec, WaitRec};

    const SUB: EntityId = 0;
    const OUT: EntityId = 1;
    const ROS: EntityId = 2;

    fn names() -> EntityNames {
        let mut n = EntityNames::default();
        for (id, name, topic) in [
            (SUB, "sub:/a#0", "/a"),
            (OUT, "pub:/out#0", "/out"),
            (ROS, "pub:/rosout#0", ROSOUT),
        ] {
            n.name.insert(id, name.into());
            n.topic.insert(id, topic.into());
            n.node.insert(id, "mixer".into());
        }
        n
    }

    fn wait(k: u64) -> Record {
        Record::Wait(WaitRec {
            k,
            thread: 0,
            ret: 0,
            timeout_ns: 0,
            attached: vec![SUB],
            ready: vec![SUB],
        })
    }
    fn take(k: u64) -> Record {
        Record::Take(TakeRec {
            k,
            thread: 0,
            entity: SUB,
            kind: TakeKind::Message,
            ret: 0,
            taken: true,
            seq: 0,
            source_ts: 0,
            received_ts: 0,
            guid: [0; 16],
            body: vec![],
        })
    }
    fn out(k: u64, entity: EntityId, hash: u64) -> Record {
        Record::Output(OutputRec {
            k,
            thread: 0,
            entity,
            kind: OutputKind::Publish,
            ret: 0,
            has_payload: true,
            len: 8,
            hash,
        })
    }
    fn live(entity: EntityId, hash: u64) -> Out {
        Out {
            entity,
            hash,
            len: 8,
            has_payload: true,
            ret: 0,
        }
    }

    /// wait(1) [timer outputs 2] take(3) [callback output 4] wait(5) [6]
    fn recording() -> Vec<Record> {
        vec![
            wait(1),
            out(2, OUT, 100),
            out(3, ROS, 7),
            take(4),
            out(5, OUT, 200),
            wait(6),
            out(7, OUT, 300),
        ]
    }

    #[test]
    fn identical_outputs_compare_clean_and_rosout_is_excluded() {
        let mut o = Oracle::new(&recording(), names());
        o.map_thread(0, 0);
        o.event(0, 1);
        o.output(0, live(OUT, 100));
        o.output(0, live(ROS, 999)); // wall-clock stamped: never compared
        o.event(0, 4);
        o.output(0, live(OUT, 200));
        o.event(0, 6);
        o.output(0, live(OUT, 300));
        o.close_all();
        assert_eq!(o.culprit, None);
        assert_eq!(o.segments_compared, 3);
        assert_eq!(o.outputs_excluded, 1);
    }

    #[test]
    fn the_first_divergent_segment_with_matching_inputs_is_named() {
        let mut o = Oracle::new(&recording(), names());
        o.map_thread(0, 0);
        o.event(0, 1);
        o.output(0, live(OUT, 100));
        o.event(0, 4);
        o.output(0, live(OUT, 201)); // differs
        o.event(0, 6);
        o.output(0, live(OUT, 301)); // differs too, but not the culprit
        o.close_all();
        let c = o.culprit.expect("culprit");
        assert_eq!(
            (
                c.node.as_str(),
                c.topic.as_str(),
                c.window,
                c.delivery,
                c.anchor_k
            ),
            ("mixer", "/out", 0, 1, 4)
        );
        assert_eq!(o.segments_divergent, 2);
    }

    #[test]
    fn a_difference_after_an_input_mismatch_is_inherited_not_named() {
        let mut o = Oracle::new(&recording(), names());
        o.map_thread(0, 0);
        o.event(0, 1);
        o.output(0, live(OUT, 100));
        o.input_mismatch("took beyond the recording".into(), Some(0));
        o.event(0, 4);
        o.output(0, live(OUT, 201));
        o.close_all();
        assert_eq!(o.culprit, None);
        assert_eq!(o.segments_inherited, 1);
        assert!(o.input_mismatch.is_some());
    }

    #[test]
    fn a_missing_output_names_the_recorded_publisher() {
        let mut o = Oracle::new(&recording(), names());
        o.map_thread(0, 0);
        o.event(0, 1);
        // The replayed timer did not fire (a wall timer): nothing published.
        o.event(0, 4);
        o.close_all();
        let c = o.culprit.expect("culprit");
        assert_eq!(
            (c.topic.as_str(), c.window, c.replayed_outputs),
            ("/out", 0, 0)
        );
    }

    #[test]
    fn construction_time_outputs_are_counted_and_never_compared() {
        let mut rec = vec![out(1, OUT, 50)];
        rec.extend(recording().into_iter().map(|mut r| {
            r.set_k(r.k() + 10);
            r
        }));
        let mut o = Oracle::new(&rec, names());
        // A wall-clock-stamped construction publish differs from the recording.
        o.output(0, live(OUT, 51));
        o.map_thread(0, 0);
        o.event(0, 11);
        o.output(0, live(OUT, 100));
        o.close_all();
        assert_eq!(o.culprit, None);
        assert_eq!(o.outputs_before_first_event, 1);
        assert_eq!(o.segments_compared, 1);
    }
}
