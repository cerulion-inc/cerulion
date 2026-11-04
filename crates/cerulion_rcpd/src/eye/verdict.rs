// SPDX-License-Identifier: AGPL-3.0-only
//! How a gate reads Jev: three states, and absent is never agreement.
//!
//! [`Look`] is `Agree`, `Disagree` or `Absent(outcome)`. It has no `Default`
//! and no conversion to `bool`, so code that wants a yes has to match
//! `Look::Agree` by name. [`Look::compare`] reads Jev's measurement out of an
//! answer, compares it with the gate's other measurement against a
//! tolerance, and gives `Absent` for every answer whose outcome is not `ok`,
//! and for an `ok` answer that lacks the reading. A model's confidence plays
//! no part.

use serde_json::{Map, Value};

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
    /// A replay or resim ask with no recorded answer.
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
pub enum Look {
    /// Jev's reading is within the tolerance of the gate's own.
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

/// One answer frame, parsed for a gate.
#[derive(Debug, Clone, PartialEq)]
pub struct EyeAnswer {
    /// The ask this answers, as the ask wrote it.
    pub ask_id: Value,
    /// What happened.
    pub outcome: Outcome,
    /// The answers by question name; `None` unless Jev answered.
    pub answers: Option<Map<String, Value>>,
    /// The frame's width in pixels, when a frame was read.
    pub image_width: Option<u32>,
    /// The frame's height in pixels, when a frame was read.
    pub image_height: Option<u32>,
}

impl EyeAnswer {
    /// Parse an answer frame's text.
    ///
    /// # Errors
    ///
    /// Why the text is not an answer frame.
    pub fn parse(text: &str) -> Result<Self, String> {
        let value: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let ask_id = value
            .get("ask_id")
            .cloned()
            .filter(|v| !v.is_null())
            .ok_or("no ask_id")?;
        let outcome = value
            .get("outcome")
            .and_then(Value::as_str)
            .and_then(Outcome::parse)
            .ok_or("no known outcome")?;
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
            answers: value.get("answers").and_then(Value::as_object).cloned(),
            image_width: image("width"),
            image_height: image("height"),
        })
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
    /// outside is [`Look::Disagree`]. An answer whose outcome is not `ok`
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

    fn answer(outcome: &str) -> EyeAnswer {
        let frame = json!({
            "ask_id": 1,
            "outcome": outcome,
            "image": { "width": 640, "height": 480 },
            "answers": {
                "edge_visible": { "type": "noul", "noul": 0.93 },
                "rim": { "type": "box", "box": { "top": 0.44, "bottom": 0.68, "left": 0.25, "right": 0.63 }, "confidence": 0.67 }
            }
        });
        EyeAnswer::parse(&frame.to_string()).expect("parse")
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
}
