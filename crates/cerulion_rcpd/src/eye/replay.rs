// SPDX-License-Identifier: AGPL-3.0-only
//! Recorded answers for replay and resim: no network, byte for byte.
//!
//! [`RecordedAnswers`] holds a recording's answer frames by `ask_id`, in
//! recorded order. [`RecordedAnswers::answer`] returns a recorded frame's
//! text unchanged for an ask whose text is the one that answer was given
//! for (its `ask_sha256` is the SHA-256 of the replayed ask), and a
//! `not_recorded` frame otherwise, so a gate re-run on a recording sees
//! exactly the answers it saw live and sees Jev absent where the recording
//! has nothing for that exact ask. A replayed ask whose id has recorded
//! answers but whose text matches none of them (another frame, question set
//! or numbers) is a divergence: it gets `not_recorded`, the recorded
//! answers stay, and [`RecordedAnswers::divergences`] counts it. Nothing
//! here opens a connection. `cerulion bag play --resim` does not use it: it
//! re-injects the recorded answer frames at their recorded steps, and
//! [`EyeAnswer::parse`](crate::eye::verdict::EyeAnswer::parse) reads one
//! given for another ask text as `not_recorded`.
//!
//! A bag stores each frame as its 40-byte wire header followed by the
//! message; [`string_frames`] and [`image_frames`] read a topic's frames
//! back.

use std::collections::{BTreeMap, VecDeque};
use std::path::Path;

use cerulion_bag::BagReader;
use cerulion_core::message::ShmMessage;
use cerulion_core::WireHeader;
use native_ros2_messages::sensor_msgs::CompressedImage;
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::Value;

use crate::eye::answer::{ask_id_of, Answer, ThirdParty};
use crate::eye::client::sha256_hex;
use crate::eye::verdict::Outcome;

/// The key an ask id is filed under: its JSON text.
fn id_key(id: &Value) -> String {
    id.to_string()
}

/// One recorded answer frame and the `ask_sha256` it carries.
#[derive(Debug, Clone)]
struct Recorded {
    ask_sha256: Option<String>,
    text: String,
}

/// A recording's answers, by ask.
#[derive(Debug, Clone, Default)]
pub struct RecordedAnswers {
    by_ask: BTreeMap<String, VecDeque<Recorded>>,
    divergences: usize,
}

impl RecordedAnswers {
    /// File `texts` (answer frames in recorded order) by their `ask_id`.
    /// A frame without a usable `ask_id` is skipped.
    pub fn new(texts: impl IntoIterator<Item = String>) -> Self {
        let mut by_ask: BTreeMap<String, VecDeque<Recorded>> = BTreeMap::new();
        for text in texts {
            if let Some(id) = ask_id_of(&text) {
                let ask_sha256 = serde_json::from_str::<Value>(&text)
                    .ok()
                    .and_then(|v| v.get("ask_sha256")?.as_str().map(str::to_owned));
                by_ask
                    .entry(id_key(&id))
                    .or_default()
                    .push_back(Recorded { ask_sha256, text });
            }
        }
        Self {
            by_ask,
            divergences: 0,
        }
    }

    /// The answers on `answer_topic` in the bag at `path`.
    ///
    /// # Errors
    ///
    /// Why the bag could not be read.
    pub fn from_bag(path: &Path, answer_topic: &str) -> Result<Self, String> {
        let reader = BagReader::open(path).map_err(|e| e.to_string())?;
        let frames = string_frames(&reader, answer_topic)?;
        Ok(Self::new(frames.into_iter().map(|(_, text)| text)))
    }

    /// The answer for the ask frame `ask_text`: the oldest recorded answer
    /// to its id whose `ask_sha256` is the SHA-256 of `ask_text`, unchanged,
    /// or a `not_recorded` frame. `None` for an ask with no usable id.
    pub fn answer(&mut self, ask_text: &str) -> Option<String> {
        let id = ask_id_of(ask_text)?;
        let digest = sha256_hex(ask_text.as_bytes());
        let queue = self.by_ask.get_mut(&id_key(&id));
        let recorded = match queue {
            Some(queue) if !queue.is_empty() => {
                let found = queue
                    .iter()
                    .position(|r| r.ask_sha256.as_deref() == Some(digest.as_str()));
                if found.is_none() {
                    self.divergences += 1;
                }
                found.and_then(|i| queue.remove(i))
            }
            _ => None,
        };
        Some(recorded.map_or_else(|| not_recorded(id, ask_text).to_json(), |r| r.text))
    }

    /// Recorded answers not yet served.
    pub fn remaining(&self) -> usize {
        self.by_ask.values().map(VecDeque::len).sum()
    }

    /// Replayed asks whose id has recorded answers, none of them for that
    /// ask's text.
    pub fn divergences(&self) -> usize {
        self.divergences
    }
}

/// The answer frame for a replayed ask with no recorded answer.
pub fn not_recorded(ask_id: Value, ask_text: &str) -> Answer {
    Answer {
        ask_id,
        outcome: Outcome::NotRecorded,
        detail: None,
        question_set: None,
        frame: None,
        ask_sha256: sha256_hex(ask_text.as_bytes()),
        route: String::new(),
        model_requested: String::new(),
        model_answered: None,
        request_sha256: None,
        image_sha256: None,
        image: None,
        questions: None,
        state: None,
        answers: None,
        usage: None,
        latency_ms: None,
        provenance: ThirdParty,
    }
}

/// The message bytes of one recorded frame, copied to an 8-byte aligned
/// buffer, with its header. `None` when the frame is shorter than its
/// header or than the size the header declares.
fn message_of(frame: &[u8]) -> Option<(WireHeader, Vec<u64>, usize)> {
    let header = WireHeader::read_from_buf(frame)?;
    let total = usize::try_from(header.total_size).ok()?;
    let body = frame.get(WireHeader::SIZE..total.max(WireHeader::SIZE))?;
    let mut words = vec![0u64; body.len().div_ceil(8)];
    cerulion_core::shm_runtime::u64s_as_bytes_mut(&mut words)[..body.len()].copy_from_slice(body);
    Some((header, words, body.len()))
}

/// Every `std_msgs/String` frame on `topic`, in file order: `(seq, text)`.
/// Frames of another type, or that do not decode, are skipped.
///
/// # Errors
///
/// Why the bag could not be read.
pub fn string_frames(reader: &BagReader, topic: &str) -> Result<Vec<(u32, String)>, String> {
    let mut out = Vec::new();
    for message in reader.messages().map_err(|e| e.to_string())? {
        let message = message.map_err(|e| e.to_string())?;
        if message.topic != topic {
            continue;
        }
        let Some((header, words, len)) = message_of(&message.data) else {
            continue;
        };
        if header.schema_hash != RosString::SCHEMA_HASH {
            continue;
        }
        let bytes = &cerulion_core::shm_runtime::u64s_as_bytes(&words)[..len];
        if let Ok(text) = crate::journal::decode_string(bytes) {
            out.push((header.sequence, text));
        }
    }
    Ok(out)
}

/// Every `sensor_msgs/CompressedImage` frame's image bytes on `topic`, by
/// wire sequence number; a later frame with the same number replaces an
/// earlier one.
///
/// # Errors
///
/// Why the bag could not be read.
pub fn image_frames(reader: &BagReader, topic: &str) -> Result<BTreeMap<u32, Vec<u8>>, String> {
    let mut out = BTreeMap::new();
    for message in reader.messages().map_err(|e| e.to_string())? {
        let message = message.map_err(|e| e.to_string())?;
        if message.topic != topic {
            continue;
        }
        let Some((header, words, len)) = message_of(&message.data) else {
            continue;
        };
        let min = CompressedImage::WIRE_FIXED_SIZE + 8 * CompressedImage::VARIABLE_FIELD_COUNT;
        if header.schema_hash != CompressedImage::SCHEMA_HASH || len < min {
            continue;
        }
        let bytes = &cerulion_core::shm_runtime::u64s_as_bytes(&words)[..len];
        let data = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            CompressedImage::build_reader(bytes).data().to_vec()
        }));
        if let Ok(data) = data {
            out.insert(header.sequence, data);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::RecordedAnswers;
    use crate::eye::client::sha256_hex;
    use crate::eye::verdict::{EyeAnswer, Look, Outcome};

    /// An answer frame to `ask` with `outcome`.
    fn answered(ask: &str, outcome: &str) -> String {
        let id: serde_json::Value =
            serde_json::from_str::<serde_json::Value>(ask).expect("ask")["ask_id"].clone();
        serde_json::json!({
            "ask_id": id,
            "outcome": outcome,
            "ask_sha256": sha256_hex(ask.as_bytes()),
            "answers": {},
        })
        .to_string()
    }

    #[test]
    fn a_recorded_answer_is_served_unchanged_and_a_new_ask_is_not_recorded() {
        let ask_1 = r#"{"ask_id":1}"#;
        let ask_b = r#"{"ask_id":"b"}"#;
        let first = answered(ask_1, "ok");
        let second = answered(ask_1, "timeout");
        let other = answered(ask_b, "refused");
        let mut recorded = RecordedAnswers::new([first.clone(), second.clone(), other.clone()]);
        assert_eq!(recorded.remaining(), 3);
        assert_eq!(recorded.answer(ask_1).as_deref(), Some(first.as_str()));
        assert_eq!(recorded.answer(ask_1).as_deref(), Some(second.as_str()));
        let third = recorded.answer(ask_1).expect("answer");
        assert_eq!(
            EyeAnswer::parse(&third, ask_1).expect("parse").outcome,
            Outcome::NotRecorded
        );
        assert_eq!(recorded.answer(ask_b).as_deref(), Some(other.as_str()));
        assert_eq!(recorded.answer("not json"), None);
        assert_eq!(recorded.remaining(), 0);
        assert_eq!(recorded.divergences(), 0);
    }

    #[test]
    fn a_replayed_ask_for_another_frame_is_not_given_the_recorded_answer() {
        let live = r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/wrist","seq":41}}"#;
        let resim = r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/wrist","seq":42}}"#;
        let recorded_answer = answered(live, "ok");
        let mut recorded = RecordedAnswers::new([recorded_answer.clone()]);
        let given = recorded.answer(resim).expect("answer");
        let parsed = EyeAnswer::parse(&given, resim).expect("parse");
        assert_eq!(parsed.outcome, Outcome::NotRecorded);
        assert_eq!(
            Look::compare(&parsed, |_| Some(1.0), 1.0, 8.0),
            Look::Absent(Outcome::NotRecorded),
            "a diverged ask never reads as agreement"
        );
        assert_eq!(recorded.divergences(), 1);
        assert_eq!(recorded.remaining(), 1, "the recorded answer stays");
        // The ask the recording answered still gets its answer.
        assert_eq!(
            recorded.answer(live).as_deref(),
            Some(recorded_answer.as_str())
        );
        assert_eq!(recorded.remaining(), 0);
    }
}
