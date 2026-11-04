// SPDX-License-Identifier: AGPL-3.0-only
//! Recorded answers for replay and resim: no network, byte for byte.
//!
//! [`RecordedAnswers`] holds a recording's answer frames by `ask_id`, in
//! recorded order. [`RecordedAnswers::answer`] returns the recorded frame's
//! text unchanged for an ask with a recorded answer, and a `not_recorded`
//! frame for one without, so a gate re-run on a recording sees exactly the
//! answers it saw live and sees Jev absent where the recording has nothing.
//! Nothing here opens a connection.
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

use crate::eye::answer::{ask_id_of, Answer};
use crate::eye::client::sha256_hex;
use crate::eye::verdict::Outcome;

/// The key an ask id is filed under: its JSON text.
fn id_key(id: &Value) -> String {
    id.to_string()
}

/// A recording's answers, by ask.
#[derive(Debug, Clone, Default)]
pub struct RecordedAnswers {
    by_ask: BTreeMap<String, VecDeque<String>>,
}

impl RecordedAnswers {
    /// File `texts` (answer frames in recorded order) by their `ask_id`.
    /// A frame without a usable `ask_id` is skipped.
    pub fn new(texts: impl IntoIterator<Item = String>) -> Self {
        let mut by_ask: BTreeMap<String, VecDeque<String>> = BTreeMap::new();
        for text in texts {
            if let Some(id) = ask_id_of(&text) {
                by_ask.entry(id_key(&id)).or_default().push_back(text);
            }
        }
        Self { by_ask }
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

    /// The answer for the ask frame `ask_text`: the next recorded answer to
    /// its id, unchanged, or a `not_recorded` frame. `None` for an ask with
    /// no usable id.
    pub fn answer(&mut self, ask_text: &str) -> Option<String> {
        let id = ask_id_of(ask_text)?;
        let recorded = self
            .by_ask
            .get_mut(&id_key(&id))
            .and_then(VecDeque::pop_front);
        Some(recorded.unwrap_or_else(|| not_recorded(id, ask_text).to_json()))
    }

    /// Recorded answers not yet served.
    pub fn remaining(&self) -> usize {
        self.by_ask.values().map(VecDeque::len).sum()
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
    use crate::eye::verdict::{EyeAnswer, Outcome};

    #[test]
    fn a_recorded_answer_is_served_unchanged_and_a_new_ask_is_not_recorded() {
        let first = r#"{"ask_id":1,"outcome":"ok","answers":{}}"#.to_owned();
        let second = r#"{"ask_id":1,"outcome":"timeout"}"#.to_owned();
        let other = r#"{"ask_id":"b","outcome":"refused"}"#.to_owned();
        let mut recorded = RecordedAnswers::new([first.clone(), second.clone(), other.clone()]);
        assert_eq!(recorded.remaining(), 3);
        assert_eq!(
            recorded.answer(r#"{"ask_id":1}"#).as_deref(),
            Some(first.as_str())
        );
        assert_eq!(
            recorded.answer(r#"{"ask_id":1}"#).as_deref(),
            Some(second.as_str())
        );
        let third = recorded.answer(r#"{"ask_id":1}"#).expect("answer");
        assert_eq!(
            EyeAnswer::parse(&third).expect("parse").outcome,
            Outcome::NotRecorded
        );
        assert_eq!(
            recorded.answer(r#"{"ask_id":"b"}"#).as_deref(),
            Some(other.as_str())
        );
        assert_eq!(recorded.answer("not json"), None);
        assert_eq!(recorded.remaining(), 0);
    }
}
