// SPDX-License-Identifier: AGPL-3.0-only
//! How a gate reads Jev: three states, and absent is never agreement.
//!
//! [`Look`] is `Agree`, `Disagree` or `Absent(outcome)`. It has no `Default`
//! and no conversion to `bool`, so code that wants a yes has to match
//! `Look::Agree` by name. Jev can only veto: `Disagree` refuses, while
//! `Agree` never passes a gate by itself and `Absent` is never agreement.
//!
//! [`EyeAnswer::parse`] reads an answer frame for the ask frame the gate
//! published: a frame whose `ask_sha256` is not the SHA-256 of that ask's
//! text answers another ask (another frame, question set or numbers under
//! the same `ask_id`) and reads as `not_recorded`. [`EyeAnswer::for_image`]
//! does the same for an `ok` answer whose `image_sha256` is not the SHA-256
//! of the frame the gate holds. [`Look::compare`] reads
//! Jev's measurement out of an answer, compares it with the gate's other
//! measurement against a tolerance, and gives `Absent` for every answer
//! whose outcome is not `ok`, and for an `ok` answer that lacks the
//! reading. A model's confidence plays no part.

use serde_json::{Map, Value};

use crate::eye::answer::{frame_ref, FrameRef};
use crate::eye::client::sha256_hex;

/// What happened to one ask. Every outcome but [`Outcome::Ok`] is Jev
/// absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Outcome {
    /// Jev answered with the pinned model and every question.
    Ok,
    /// The owner's switch is off, or the eye has no `eyes.jev` section.
    Disabled,
    /// No usable key: none on file, or the route refused it (401, 403).
    NoCredential,
    /// The route or the model is unreachable (5xx, 529, a connection error).
    Unavailable,
    /// No answer by the deadline.
    Timeout,
    /// The route said 429; the robot backs off and does not retry.
    RateLimited,
    /// A spend limit or budget was reached (402, or 403 naming a limit).
    Quota,
    /// The robot's own rate or in-flight limit.
    Throttled,
    /// The request shape was refused, here or by the route (400, 413, 422).
    Refused,
    /// The answer came from a model other than the pinned one.
    WrongModel,
    /// An ask or an answer with a missing or malformed field.
    Malformed,
    /// The question set's visibility Noul was below its threshold.
    NotVisible,
    /// No answer to this exact ask: a replay or resim ask with no recorded
    /// answer, or an answer frame given for another ask text.
    NotRecorded,
}

impl Outcome {
    /// Every outcome, in declaration order.
    pub const ALL: [Self; 13] = [
        Self::Ok,
        Self::Disabled,
        Self::NoCredential,
        Self::Unavailable,
        Self::Timeout,
        Self::RateLimited,
        Self::Quota,
        Self::Throttled,
        Self::Refused,
        Self::WrongModel,
        Self::Malformed,
        Self::NotVisible,
        Self::NotRecorded,
    ];

    /// The code an answer frame and a receipt carry.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Ok => "ok",
            Self::Disabled => "disabled",
            Self::NoCredential => "no_credential",
            Self::Unavailable => "unavailable",
            Self::Timeout => "timeout",
            Self::RateLimited => "rate_limited",
            Self::Quota => "quota",
            Self::Throttled => "throttled",
            Self::Refused => "refused",
            Self::WrongModel => "wrong_model",
            Self::Malformed => "malformed",
            Self::NotVisible => "not_visible",
            Self::NotRecorded => "not_recorded",
        }
    }

    /// The outcome a code names.
    pub fn parse(code: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|o| o.as_str() == code)
    }

    /// Whether the outcome counts as a limit reached, for the status file.
    pub fn is_limit(self) -> bool {
        matches!(self, Self::Quota | Self::Throttled | Self::RateLimited)
    }
}

impl std::fmt::Display for Outcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// What the gate makes of one look. No `Default`, no `bool`.
///
/// Jev is a cross-check that can only veto: `Disagree` refuses, `Agree`
/// never passes a gate by itself, and `Absent` is never agreement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
pub enum Look {
    /// Jev's reading is within the tolerance of the gate's own; the gate
    /// still decides on its own measurements.
    Agree,
    /// Jev's reading is outside the tolerance; the gate refuses.
    Disagree,
    /// Jev gave no usable reading, and why.
    Absent(Outcome),
}

/// One edge of a Box answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoxEdge {
    /// Measured from the top, as a fraction of the height.
    Top,
    /// Measured from the top, as a fraction of the height.
    Bottom,
    /// Measured from the left, as a fraction of the width.
    Left,
    /// Measured from the left, as a fraction of the width.
    Right,
}

impl BoxEdge {
    fn key(self) -> &'static str {
        match self {
            Self::Top => "top",
            Self::Bottom => "bottom",
            Self::Left => "left",
            Self::Right => "right",
        }
    }

    fn vertical(self) -> bool {
        matches!(self, Self::Top | Self::Bottom)
    }
}

/// One answer frame, parsed for the ask frame a gate published. Its fields
/// are read through methods, so [`EyeAnswer::parse`] is the only way to
/// make one.
#[derive(Debug, Clone, PartialEq)]
pub struct EyeAnswer {
    ask_id: Value,
    outcome: Outcome,
    ask_sha256: Option<String>,
    frame: Option<FrameRef>,
    image_sha256: Option<String>,
    answers: Option<Map<String, Value>>,
    image_width: Option<u32>,
    image_height: Option<u32>,
}

impl EyeAnswer {
    /// The ask this answers, as the ask wrote it.
    pub fn ask_id(&self) -> &Value {
        &self.ask_id
    }

    /// What happened to this ask: `not_recorded` when the frame answers
    /// another ask text.
    pub fn outcome(&self) -> Outcome {
        self.outcome
    }

    /// The SHA-256 of the ask text the frame answers, as the frame carries
    /// it.
    pub fn ask_sha256(&self) -> Option<&str> {
        self.ask_sha256.as_deref()
    }

    /// The frame the answered ask named, as the frame carries it.
    pub fn frame(&self) -> Option<&FrameRef> {
        self.frame.as_ref()
    }

    /// The SHA-256 of the image bytes sent, as the frame carries it, so a
    /// gate can check it against the bytes of the frame it holds.
    pub fn image_sha256(&self) -> Option<&str> {
        self.image_sha256.as_deref()
    }

    /// The frame's width and height in pixels, when a frame was read.
    pub fn image_size(&self) -> Option<(u32, u32)> {
        self.image_width.zip(self.image_height)
    }

    /// Parse the answer frame `text` for the ask frame `ask_text`, the exact
    /// text the gate published. A frame whose `ask_sha256` is missing or is
    /// not the SHA-256 of `ask_text` reads as `not_recorded` with no
    /// answers, so an answer given for another frame, question set or
    /// numbers under the same `ask_id` is never read as this ask's.
    ///
    /// # Errors
    ///
    /// Why the text is not an answer frame.
    pub fn parse(text: &str, ask_text: &str) -> Result<Self, String> {
        let value: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let ask_id = value
            .get("ask_id")
            .cloned()
            .filter(|v| !v.is_null())
            .ok_or("no ask_id")?;
        let recorded = value
            .get("outcome")
            .and_then(Value::as_str)
            .and_then(Outcome::parse)
            .ok_or("no known outcome")?;
        let ask_sha256 = value
            .get("ask_sha256")
            .and_then(Value::as_str)
            .map(str::to_owned);
        let for_this_ask = ask_sha256.as_deref() == Some(sha256_hex(ask_text.as_bytes()).as_str());
        let outcome = if for_this_ask {
            recorded
        } else {
            Outcome::NotRecorded
        };
        let image = |key: &str| {
            value
                .get("image")
                .and_then(|i| i.get(key))
                .and_then(Value::as_u64)
                .and_then(|n| u32::try_from(n).ok())
        };
        Ok(Self {
            ask_id,
            outcome,
            ask_sha256,
            frame: frame_ref(value.get("frame")),
            image_sha256: value
                .get("image_sha256")
                .and_then(Value::as_str)
                .map(str::to_owned),
            answers: value
                .get("answers")
                .and_then(Value::as_object)
                .filter(|_| for_this_ask)
                .cloned(),
            image_width: image("width"),
            image_height: image("height"),
        })
    }

    /// This answer read against the JPEG bytes `image` of the frame the gate
    /// holds for its ask: an `ok` answer whose `image_sha256` is missing or is
    /// not the SHA-256 of `image` reads as `not_recorded` with no answers.
    /// `bag play` re-publishes a recording's answer frames byte for byte, and
    /// ask ids and frame sequence numbers restart with each run, so a
    /// recorded answer can carry a live gate's exact ask text while naming
    /// another frame's image.
    #[must_use]
    pub fn for_image(mut self, image: &[u8]) -> Self {
        let own = self.image_sha256.as_deref() == Some(sha256_hex(image).as_str());
        if self.outcome == Outcome::Ok && !own {
            self.outcome = Outcome::NotRecorded;
            self.answers = None;
        }
        self
    }

    /// The answer to the question named `question`, when Jev answered.
    pub fn answer(&self, question: &str) -> Option<&Value> {
        if self.outcome != Outcome::Ok {
            return None;
        }
        self.answers.as_ref()?.get(question)
    }

    /// A Noul answer's probability.
    pub fn noul(&self, question: &str) -> Option<f64> {
        let answer = self.answer(question)?;
        (answer.get("type")?.as_str()? == "noul")
            .then(|| answer.get("noul")?.as_f64())
            .flatten()
            .filter(|p| (0.0..=1.0).contains(p))
    }

    /// A Box answer's edge in pixels of the frame sent.
    pub fn box_edge_px(&self, question: &str, edge: BoxEdge) -> Option<f64> {
        let answer = self.answer(question)?;
        if answer.get("type")?.as_str()? != "box" {
            return None;
        }
        let fraction = answer
            .get("box")?
            .get(edge.key())?
            .as_f64()
            .filter(|f| (0.0..=1.0).contains(f))?;
        let span = if edge.vertical() {
            self.image_height?
        } else {
            self.image_width?
        };
        Some(fraction * f64::from(span))
    }
}

impl Look {
    /// Compare Jev's reading, taken from `answer` by `reading`, with the
    /// gate's `other` measurement: within `tolerance` is [`Look::Agree`],
    /// outside is [`Look::Disagree`]. An answer whose outcome is not `ok`,
    /// including one [`EyeAnswer::parse`] found was given for another ask,
    /// is `Absent` with that outcome; an `ok` answer without the reading,
    /// or a non-finite measurement or tolerance, is `Absent(Malformed)`.
    pub fn compare(
        answer: &EyeAnswer,
        reading: impl FnOnce(&EyeAnswer) -> Option<f64>,
        other: f64,
        tolerance: f64,
    ) -> Self {
        if answer.outcome != Outcome::Ok {
            return Self::Absent(answer.outcome);
        }
        if !other.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return Self::Absent(Outcome::Malformed);
        }
        match reading(answer).filter(|r| r.is_finite()) {
            Some(jev) if (jev - other).abs() <= tolerance => Self::Agree,
            Some(_) => Self::Disagree,
            None => Self::Absent(Outcome::Malformed),
        }
    }

    /// The outcome behind an absent look.
    pub fn absent_because(self) -> Option<Outcome> {
        match self {
            Self::Absent(outcome) => Some(outcome),
            Self::Agree | Self::Disagree => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{BoxEdge, EyeAnswer, Look, Outcome};
    use crate::eye::answer::FrameRef;
    use crate::eye::client::sha256_hex;

    const ASK: &str = r#"{"ask_id":1,"question_set":"press_gate","frame":{"topic":"/wrist/image","seq":412},"deadline_ms":1500}"#;

    /// The JPEG bytes of the frame [`ASK`] names.
    const IMAGE: &[u8] = b"the wrist frame's JPEG bytes";

    /// An answer frame to [`ASK`] with `outcome`.
    fn frame(outcome: &str) -> String {
        json!({
            "ask_id": 1,
            "outcome": outcome,
            "frame": { "topic": "/wrist/image", "seq": 412 },
            "ask_sha256": sha256_hex(ASK.as_bytes()),
            "image_sha256": sha256_hex(IMAGE),
            "image": { "width": 640, "height": 480 },
            "answers": {
                "edge_visible": { "type": "noul", "noul": 0.93 },
                "rim": { "type": "box", "box": { "top": 0.44, "bottom": 0.68, "left": 0.25, "right": 0.63 }, "confidence": 0.67 }
            }
        })
        .to_string()
    }

    fn answer(outcome: &str) -> EyeAnswer {
        EyeAnswer::parse(&frame(outcome), ASK).expect("parse")
    }

    #[test]
    fn every_code_round_trips() {
        for outcome in Outcome::ALL {
            assert_eq!(Outcome::parse(outcome.as_str()), Some(outcome));
        }
        assert_eq!(Outcome::parse("late"), None);
    }

    #[test]
    fn a_reading_within_the_tolerance_agrees_and_outside_disagrees() {
        let ok = answer("ok");
        let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
        let row = rim(&ok).expect("row");
        assert!((row - 326.4).abs() < 1e-9, "{row}");
        assert_eq!(Look::compare(&ok, rim, 330.0, 8.0), Look::Agree);
        assert_eq!(Look::compare(&ok, rim, 340.0, 8.0), Look::Disagree);
        assert_eq!(ok.noul("edge_visible"), Some(0.93));
    }

    #[test]
    fn absent_is_never_agreement() {
        let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
        for outcome in Outcome::ALL.into_iter().filter(|o| *o != Outcome::Ok) {
            // The recorded answers are present, yet only `ok` is read.
            let look = Look::compare(&answer(outcome.as_str()), rim, 326.4, 100.0);
            assert_eq!(look, Look::Absent(outcome));
            assert_eq!(look.absent_because(), Some(outcome));
        }
        let ok = answer("ok");
        assert_eq!(
            Look::compare(
                &ok,
                |a| a.box_edge_px("missing", BoxEdge::Bottom),
                326.4,
                100.0
            ),
            Look::Absent(Outcome::Malformed)
        );
        assert_eq!(
            Look::compare(
                &ok,
                |a| a.box_edge_px("edge_visible", BoxEdge::Bottom),
                1.0,
                1.0
            ),
            Look::Absent(Outcome::Malformed),
            "a noul is not a box"
        );
        assert_eq!(
            Look::compare(&ok, rim, f64::NAN, 8.0),
            Look::Absent(Outcome::Malformed)
        );
    }

    #[test]
    fn an_answer_given_for_another_ask_text_is_not_recorded() {
        let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
        let recorded = frame("ok");
        // The same id, question set and deadline; only the frame differs.
        let next_frame = ASK.replace(r#""seq":412"#, r#""seq":413"#);
        assert_ne!(next_frame, ASK);
        let read = EyeAnswer::parse(&recorded, &next_frame).expect("parse");
        assert_eq!(read.outcome, Outcome::NotRecorded);
        assert_eq!(read.answer("rim"), None);
        assert_eq!(
            read.frame,
            Some(FrameRef {
                topic: String::from("/wrist/image"),
                seq: 412
            }),
            "the frame the answer was given for"
        );
        assert_eq!(
            Look::compare(&read, rim, 326.4, 100.0),
            Look::Absent(Outcome::NotRecorded)
        );
        let own = EyeAnswer::parse(&recorded, ASK).expect("parse");
        assert_eq!(own.outcome, Outcome::Ok);
        assert_eq!(
            own.ask_sha256.as_deref(),
            Some(sha256_hex(ASK.as_bytes()).as_str())
        );
        assert_eq!(Look::compare(&own, rim, 326.4, 8.0), Look::Agree);
        // A frame without the digest answers no ask.
        let mut bare: serde_json::Value = serde_json::from_str(&recorded).expect("json");
        bare.as_object_mut().expect("object").remove("ask_sha256");
        let read = EyeAnswer::parse(&bare.to_string(), ASK).expect("parse");
        assert_eq!(
            Look::compare(&read, rim, 326.4, 100.0),
            Look::Absent(Outcome::NotRecorded)
        );
    }

    #[test]
    fn an_answer_for_another_image_is_not_recorded() {
        let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
        // The gate's own frame: the answer reads as given.
        let own = answer("ok").for_image(IMAGE);
        assert_eq!(own.outcome(), Outcome::Ok);
        assert_eq!(Look::compare(&own, rim, 326.4, 8.0), Look::Agree);
        // The same ask text, another frame's bytes: a recorded answer played
        // beside a live gate whose ask id and frame seq restarted.
        let other = answer("ok").for_image(b"another wrist frame");
        assert_eq!(other.outcome(), Outcome::NotRecorded);
        assert_eq!(other.answer("rim"), None);
        assert_eq!(
            Look::compare(&other, rim, 326.4, 100.0),
            Look::Absent(Outcome::NotRecorded)
        );
        // No image digest answers no image.
        let mut bare: serde_json::Value = serde_json::from_str(&frame("ok")).expect("json");
        bare.as_object_mut().expect("object").remove("image_sha256");
        let read = EyeAnswer::parse(&bare.to_string(), ASK)
            .expect("parse")
            .for_image(IMAGE);
        assert_eq!(read.outcome(), Outcome::NotRecorded);
        // An absent answer keeps its reason.
        let off = answer("disabled").for_image(b"another wrist frame");
        assert_eq!(off.outcome(), Outcome::Disabled);
    }
}
