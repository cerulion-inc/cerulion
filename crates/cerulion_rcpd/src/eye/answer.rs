// SPDX-License-Identifier: AGPL-3.0-only
//! Asks, answer frames, and how a reply maps to an outcome.
//!
//! An answer frame is one `std_msgs/String` JSON object, its keys in this
//! order: `ask_id`, `outcome`, `detail`, `question_set`, `frame`,
//! `ask_sha256`, `route`, `model_requested`, `model_answered`,
//! `request_sha256`, `image_sha256`, `image` (`{width, height, bytes}`),
//! `questions`, `state`, `answers`, `usage` (`{input_tokens, cost}`),
//! `latency_ms` and `provenance`. Fields that do not apply are `null`.
//! `provenance` is always `third_party`, a field a training or labelling
//! export filters Jev's output out on.
//!
//! [`classify`] maps a reply as the design's failure table says:
//!
//! | Reply | Outcome |
//! |---|---|
//! | 200 with the pinned model and every question answered | `ok`, or `not_visible` under the set's threshold |
//! | 200 from another model | `wrong_model` |
//! | 200 with a missing or malformed field | `malformed` |
//! | 401, or any other 403 | `no_credential` |
//! | 402, or 403 whose message says `key limit exceeded`, `insufficient credits` or `budget` | `quota` |
//! | 429 | `rate_limited` |
//! | 400, 413, 422, or another 4xx | `refused` |
//! | 5xx (529 included), anything else, no reply | `unavailable` |

use serde::Serialize;
use serde_json::{Map, Value};

use crate::eye::client::{HttpFailure, HttpReply};
use crate::eye::config::{model_matches, JevConfig, QuestionSet};
use crate::eye::limits::JpegSize;
use crate::eye::verdict::Outcome;

/// The frame an ask names.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct FrameRef {
    /// The frame topic.
    pub topic: String,
    /// The frame's wire sequence number.
    pub seq: u32,
}

/// The image a request sent, without its bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct ImageInfo {
    /// Pixels across.
    pub width: u32,
    /// Pixels down.
    pub height: u32,
    /// The JPEG's size.
    pub bytes: usize,
}

impl ImageInfo {
    /// The info for a checked frame of `bytes` bytes.
    pub fn new(size: JpegSize, bytes: usize) -> Self {
        Self {
            width: size.width,
            height: size.height,
            bytes,
        }
    }
}

/// What a request cost.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Usage {
    /// Billable input tokens.
    pub input_tokens: u64,
    /// USD, when the route reports it.
    pub cost: Option<f64>,
}

/// One answer frame.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Answer {
    /// The ask's id, as the ask wrote it.
    pub ask_id: Value,
    /// What happened.
    #[serde(serialize_with = "outcome_code")]
    pub outcome: Outcome,
    /// A short machine-readable reason, when there is one.
    pub detail: Option<String>,
    /// The question set the ask named.
    pub question_set: Option<String>,
    /// The frame the ask named.
    pub frame: Option<FrameRef>,
    /// The SHA-256 of the ask frame's text.
    pub ask_sha256: String,
    /// The endpoint the request went to, or would have.
    pub route: String,
    /// The pinned model.
    pub model_requested: String,
    /// The model the route says answered.
    pub model_answered: Option<String>,
    /// The SHA-256 of the request body sent.
    pub request_sha256: Option<String>,
    /// The SHA-256 of the image sent.
    pub image_sha256: Option<String>,
    /// The image sent, without its bytes.
    pub image: Option<ImageInfo>,
    /// The questions sent.
    pub questions: Option<Value>,
    /// The state sent, with the image placeholder.
    pub state: Option<Value>,
    /// The answers, as the route returned them.
    pub answers: Option<Value>,
    /// What the request cost.
    pub usage: Option<Usage>,
    /// From the ask's arrival to the reply, milliseconds.
    pub latency_ms: Option<u64>,
    /// Always [`ThirdParty`].
    pub provenance: ThirdParty,
}

/// The `provenance` every answer frame carries: `third_party`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ThirdParty;

impl Serialize for ThirdParty {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str("third_party")
    }
}

fn outcome_code<S: serde::Serializer>(outcome: &Outcome, s: S) -> Result<S::Ok, S::Error> {
    s.serialize_str(outcome.as_str())
}

impl Answer {
    /// The frame's text, as published.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// The receipt's `args_digest`: the request body's SHA-256 when one was
    /// sent, else the ask's.
    pub fn receipt_digest(&self) -> &str {
        self.request_sha256.as_deref().unwrap_or(&self.ask_sha256)
    }

    /// The receipt outcome: `ok`, or an error carrying the code.
    pub fn receipt_outcome(&self) -> cerud::receipt::ReceiptOutcome {
        match self.outcome {
            Outcome::Ok => cerud::receipt::ReceiptOutcome::Ok,
            other => cerud::receipt::ReceiptOutcome::Error(other.as_str().to_owned()),
        }
    }
}

/// Where requests go, as answer frames record it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    /// The endpoint URL.
    pub endpoint: String,
    /// The pinned model.
    pub model: String,
}

/// An ask, checked against the configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct Ask {
    /// The id the answer echoes.
    pub ask_id: Value,
    /// The SHA-256 of the ask frame's text.
    pub ask_sha256: String,
    /// The question set.
    pub question_set: String,
    /// The frame.
    pub frame: FrameRef,
    /// The ask's own deadline, milliseconds.
    pub deadline_ms: Option<u64>,
    /// The numbers to send.
    pub numbers: Map<String, Value>,
}

/// Why an ask is not taken.
#[derive(Debug, Clone, PartialEq)]
pub enum AskProblem {
    /// Not a JSON object with a usable `ask_id`: nothing can be answered.
    NoId(String),
    /// An answerable ask that breaks the shape: answered `malformed`.
    Malformed {
        /// The ask's id.
        ask_id: Value,
        /// The ask's partial reading, for the answer frame.
        question_set: Option<String>,
        /// The frame it named, if it named one.
        frame: Option<FrameRef>,
        /// What is wrong.
        detail: &'static str,
    },
}

/// The largest integer an `ask_id` may be: exact as a JSON double.
const MAX_ASK_ID: u64 = (1 << 53) - 1;

fn valid_ask_id(id: &Value) -> bool {
    match id {
        Value::Number(n) => n.as_u64().is_some_and(|n| n <= MAX_ASK_ID),
        Value::String(s) => {
            !s.is_empty()
                && s.len() <= 128
                && s.bytes()
                    .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b':' | b'-'))
        }
        _ => false,
    }
}

fn frame_ref(value: Option<&Value>) -> Option<FrameRef> {
    let object = value?.as_object()?;
    if object.len() != 2 {
        return None;
    }
    let topic = object.get("topic")?.as_str()?.to_owned();
    let seq = u32::try_from(object.get("seq")?.as_u64()?).ok()?;
    Some(FrameRef { topic, seq })
}

/// The id an ask frame carries, when it carries a usable one.
pub fn ask_id_of(text: &str) -> Option<Value> {
    let value: Value = serde_json::from_str(text).ok()?;
    value.get("ask_id").filter(|id| valid_ask_id(id)).cloned()
}

/// Read one ask frame against `config`.
///
/// # Errors
///
/// [`AskProblem`] when the ask is not taken.
pub fn parse_ask(text: &str, config: &JevConfig) -> Result<Ask, AskProblem> {
    let value: Value =
        serde_json::from_str(text).map_err(|e| AskProblem::NoId(format!("not JSON: {e}")))?;
    let Some(object) = value.as_object() else {
        return Err(AskProblem::NoId(String::from("not a JSON object")));
    };
    let ask_id = match object.get("ask_id") {
        Some(id) if valid_ask_id(id) => id.clone(),
        _ => {
            return Err(AskProblem::NoId(String::from(
                "ask_id must be an integer below 2^53 or 1 to 128 characters from \
                 [A-Za-z0-9_.:-]",
            )))
        }
    };
    let question_set = object
        .get("question_set")
        .and_then(Value::as_str)
        .map(str::to_owned);
    let frame = frame_ref(object.get("frame"));
    let malformed = |detail: &'static str| AskProblem::Malformed {
        ask_id: ask_id.clone(),
        question_set: question_set.clone(),
        frame: frame.clone(),
        detail,
    };
    for key in object.keys() {
        if !matches!(
            key.as_str(),
            "ask_id" | "question_set" | "frame" | "deadline_ms" | "numbers"
        ) {
            return Err(malformed("unknown_key"));
        }
    }
    let Some(set) = question_set.as_deref().and_then(|n| config.question_set(n)) else {
        return Err(malformed("unknown_question_set"));
    };
    let Some(named) = frame.clone().filter(|f| f.topic == config.frame_topic) else {
        return Err(malformed("frame"));
    };
    let deadline_ms = match object.get("deadline_ms") {
        None | Some(Value::Null) => None,
        Some(v) => match v.as_u64().filter(|ms| *ms > 0) {
            Some(ms) => Some(ms),
            None => return Err(malformed("deadline_ms")),
        },
    };
    let numbers = match object.get("numbers") {
        None | Some(Value::Null) => Map::new(),
        Some(Value::Object(map)) => {
            let declared = |k: &String| set.numbers.iter().any(|n| n == k);
            let finite = |v: &Value| v.as_f64().is_some_and(f64::is_finite);
            if !map.iter().all(|(k, v)| declared(k) && finite(v)) {
                return Err(malformed("numbers"));
            }
            map.clone()
        }
        Some(_) => return Err(malformed("numbers")),
    };
    Ok(Ask {
        ask_id,
        ask_sha256: crate::eye::client::sha256_hex(text.as_bytes()),
        question_set: set.name.clone(),
        frame: named,
        deadline_ms,
        numbers,
    })
}

/// The question names and the answer type each must carry.
pub type Expected = Vec<(String, String)>;

/// The option names of each choice question.
pub type Choices = Vec<(String, Vec<String>)>;

/// Everything an answer frame records about a request that was built.
#[derive(Debug, Clone, PartialEq)]
pub struct Prepared {
    /// The ask.
    pub ask: Ask,
    /// The endpoint.
    pub route: String,
    /// The pinned model.
    pub model: String,
    /// The request body's SHA-256.
    pub request_sha256: String,
    /// The image's SHA-256.
    pub image_sha256: String,
    /// The image sent.
    pub image: ImageInfo,
    /// The questions sent.
    pub questions: Value,
    /// The state sent.
    pub state: Value,
    /// The question names and types the answers must carry.
    pub expected: Expected,
    /// Choice option names per choice question.
    pub choices: Choices,
    /// The visibility rule.
    pub visible: Option<(String, f64)>,
}

impl Prepared {
    /// The questions, the visibility rule and the expected answer shapes of
    /// `set`.
    pub fn expectations(set: &QuestionSet) -> (Expected, Choices) {
        let mut expected = Vec::new();
        let mut choices = Vec::new();
        for (name, question) in &set.questions {
            let kind = question
                .get("type")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_owned();
            if kind == "choice" {
                let options = question
                    .get("criteria")
                    .and_then(Value::as_object)
                    .map(|m| m.keys().cloned().collect())
                    .unwrap_or_default();
                choices.push((name.clone(), options));
            }
            expected.push((name.clone(), kind));
        }
        (expected, choices)
    }

    /// An answer frame for this request with `outcome` and no reply.
    pub fn answer(
        &self,
        outcome: Outcome,
        detail: Option<&str>,
        latency_ms: Option<u64>,
    ) -> Answer {
        Answer {
            ask_id: self.ask.ask_id.clone(),
            outcome,
            detail: detail.map(str::to_owned),
            question_set: Some(self.ask.question_set.clone()),
            frame: Some(self.ask.frame.clone()),
            ask_sha256: self.ask.ask_sha256.clone(),
            route: self.route.clone(),
            model_requested: self.model.clone(),
            model_answered: None,
            request_sha256: Some(self.request_sha256.clone()),
            image_sha256: Some(self.image_sha256.clone()),
            image: Some(self.image),
            questions: Some(self.questions.clone()),
            state: Some(self.state.clone()),
            answers: None,
            usage: None,
            latency_ms,
            provenance: ThirdParty,
        }
    }
}

/// An answer frame for an ask that sent no request.
pub fn unsent(ask: &Ask, route: &Route, outcome: Outcome, detail: Option<&str>) -> Answer {
    Answer {
        ask_id: ask.ask_id.clone(),
        outcome,
        detail: detail.map(str::to_owned),
        question_set: Some(ask.question_set.clone()),
        frame: Some(ask.frame.clone()),
        ask_sha256: ask.ask_sha256.clone(),
        route: route.endpoint.clone(),
        model_requested: route.model.clone(),
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

/// An answer frame for an ask read only as far as `problem` says, answered
/// `outcome` (`malformed`, or `disabled` when the eye is off).
pub fn unread(
    ask_text: &str,
    problem: &AskProblem,
    route: &Route,
    outcome: Outcome,
) -> Option<Answer> {
    let AskProblem::Malformed {
        ask_id,
        question_set,
        frame,
        detail,
    } = problem
    else {
        return None;
    };
    Some(Answer {
        ask_id: ask_id.clone(),
        outcome,
        detail: (outcome == Outcome::Malformed).then(|| (*detail).to_owned()),
        question_set: question_set.clone(),
        frame: frame.clone(),
        ask_sha256: crate::eye::client::sha256_hex(ask_text.as_bytes()),
        route: route.endpoint.clone(),
        model_requested: route.model.clone(),
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
    })
}

/// Whether an answer of `kind` is well formed.
fn well_formed(answer: &Value, kind: &str, options: Option<&[String]>) -> bool {
    let unit = |v: Option<&Value>| {
        v.and_then(Value::as_f64)
            .is_some_and(|f| (0.0..=1.0).contains(&f))
    };
    if answer.get("type").and_then(Value::as_str) != Some(kind) {
        return false;
    }
    match kind {
        "noul" => unit(answer.get("noul")),
        "box" => answer.get("box").is_some_and(|b| {
            ["top", "bottom", "left", "right"]
                .iter()
                .all(|edge| unit(b.get(edge)))
        }),
        "choice" => answer
            .get("choice")
            .and_then(Value::as_str)
            .is_some_and(|c| options.is_none_or(|o| o.iter().any(|x| x == c))),
        "score" => answer
            .get("score")
            .and_then(Value::as_f64)
            .is_some_and(f64::is_finite),
        _ => false,
    }
}

/// Whether a 403 names a spend limit rather than a bad key: its
/// `error.message` (the whole body when it has none) holds `key limit
/// exceeded` (a key's daily, monthly or total limit), `insufficient credits`
/// or `budget` (a workspace budget), in any case.
fn names_a_limit(body: &[u8]) -> bool {
    let message = serde_json::from_slice::<Value>(body)
        .ok()
        .and_then(|v| v.get("error")?.get("message")?.as_str().map(str::to_owned))
        .unwrap_or_else(|| String::from_utf8_lossy(body).into_owned())
        .to_ascii_lowercase();
    ["key limit exceeded", "insufficient credits", "budget"]
        .iter()
        .any(|w| message.contains(w))
}

/// The answer frame for `prepared` given the route's reply.
pub fn classify(
    prepared: &Prepared,
    reply: Result<&HttpReply, HttpFailure>,
    latency_ms: u64,
) -> Answer {
    let latency = Some(latency_ms);
    let reply = match reply {
        Ok(reply) => reply,
        Err(failure) => {
            return prepared.answer(Outcome::Unavailable, Some(failure.code()), latency);
        }
    };
    let status_detail = format!("http_{}", reply.status);
    let fail = |outcome: Outcome| prepared.answer(outcome, Some(&status_detail), latency);
    match reply.status {
        200 => {}
        401 => return fail(Outcome::NoCredential),
        403 if names_a_limit(&reply.body) => return fail(Outcome::Quota),
        403 => return fail(Outcome::NoCredential),
        402 => return fail(Outcome::Quota),
        429 => return fail(Outcome::RateLimited),
        400..=499 => return fail(Outcome::Refused),
        _ => return fail(Outcome::Unavailable),
    }
    let malformed = |detail: &str| prepared.answer(Outcome::Malformed, Some(detail), latency);
    let Ok(body) = serde_json::from_slice::<Value>(&reply.body) else {
        return malformed("not_json");
    };
    let Some(model) = body.get("model").and_then(Value::as_str) else {
        return malformed("no_model");
    };
    let answers = body.get("answers").and_then(Value::as_object);
    let usage = body.get("usage").and_then(|u| {
        Some(Usage {
            input_tokens: u.get("input_tokens")?.as_u64()?,
            cost: u.get("cost").and_then(Value::as_f64),
        })
    });
    let mut out = prepared.answer(Outcome::Ok, None, latency);
    out.model_answered = Some(model.to_owned());
    out.answers = answers.cloned().map(Value::Object);
    out.usage = usage.clone();
    if !model_matches(&prepared.model, model) {
        out.outcome = Outcome::WrongModel;
        out.detail = Some(String::from("model_answered"));
        return out;
    }
    let Some(answers) = answers else {
        out.outcome = Outcome::Malformed;
        out.detail = Some(String::from("no_answers"));
        return out;
    };
    if usage.is_none() {
        out.outcome = Outcome::Malformed;
        out.detail = Some(String::from("no_usage"));
        return out;
    }
    for (name, kind) in &prepared.expected {
        let options = prepared
            .choices
            .iter()
            .find(|(q, _)| q == name)
            .map(|(_, o)| o.as_slice());
        if !answers
            .get(name)
            .is_some_and(|a| well_formed(a, kind, options))
        {
            out.outcome = Outcome::Malformed;
            out.detail = Some(format!("answer_{name}"));
            return out;
        }
    }
    if let Some((question, threshold)) = &prepared.visible {
        let p = answers
            .get(question)
            .and_then(|a| a.get("noul"))
            .and_then(Value::as_f64)
            .unwrap_or(0.0);
        if p < *threshold {
            out.outcome = Outcome::NotVisible;
            out.detail = Some(format!("{question}_below_threshold"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use serde_json::{json, Value};

    use super::{classify, parse_ask, Ask, AskProblem, FrameRef, ImageInfo, Prepared};
    use crate::eye::client::{HttpFailure, HttpReply};
    use crate::eye::config::JevConfig;
    use crate::eye::verdict::Outcome;

    fn config() -> JevConfig {
        let yaml = "
frame_topic: /wrist/image
question_sets:
  - name: press
    state: scene
    numbers: [gap]
    questions:
      edge_visible: { type: noul, instructions: The edge is visible in `frame`. }
      rim: { type: box, instructions: The rim in `frame`. }
      side: { type: choice, instructions: Which side?, criteria: { left: on the left, right: on the right } }
    visible: { question: edge_visible, threshold: 0.7 }
";
        let config: JevConfig = serde_yaml::from_str(yaml).expect("yaml");
        config.check().expect("check");
        config
    }

    fn prepared() -> Prepared {
        let config = config();
        let set = config.question_set("press").expect("set");
        let (expected, choices) = Prepared::expectations(set);
        Prepared {
            ask: Ask {
                ask_id: json!(1),
                ask_sha256: String::from("a"),
                question_set: String::from("press"),
                frame: FrameRef {
                    topic: String::from("/wrist/image"),
                    seq: 3,
                },
                deadline_ms: None,
                numbers: serde_json::Map::new(),
            },
            route: String::from("http://127.0.0.1:9/v1/systemone"),
            model: String::from("typesafe/jev-1.14"),
            request_sha256: String::from("r"),
            image_sha256: String::from("i"),
            image: ImageInfo {
                width: 640,
                height: 480,
                bytes: 10,
            },
            questions: json!({}),
            state: json!({}),
            expected,
            choices,
            visible: set
                .visible
                .as_ref()
                .map(|v| (v.question.clone(), v.threshold)),
        }
    }

    fn reply(status: u16, body: &Value) -> HttpReply {
        HttpReply {
            status,
            retry_after: None,
            body: body.to_string().into_bytes(),
        }
    }

    fn good(model: &str, visible: f64) -> Value {
        json!({
            "id": "gen-1", "model": model, "provider": "TypeSafe",
            "answers": {
                "edge_visible": { "type": "noul", "noul": visible },
                "rim": { "type": "box", "box": { "top": 0.4, "bottom": 0.68, "left": 0.2, "right": 0.6 }, "confidence": 0.6 },
                "side": { "type": "choice", "choice": "left", "confidence": 0.9, "probabilities": { "left": 0.9, "right": 0.1 } }
            },
            "usage": { "input_tokens": 1425, "output_tokens": 56, "cost": 0.00006 }
        })
    }

    #[test]
    fn every_reply_maps_to_the_outcome_the_table_gives() {
        let p = prepared();
        let cases: Vec<(HttpReply, Outcome)> = vec![
            (
                reply(200, &good("typesafe/jev-1.14-20261014", 0.93)),
                Outcome::Ok,
            ),
            (
                reply(200, &good("typesafe/jev-1.14-20261014", 0.5)),
                Outcome::NotVisible,
            ),
            (
                reply(200, &good("typesafe/jev-1.15-20261201", 0.93)),
                Outcome::WrongModel,
            ),
            (
                reply(200, &json!({ "model": "typesafe/jev-1.14" })),
                Outcome::Malformed,
            ),
            (
                reply(401, &json!({ "error": { "message": "No auth" } })),
                Outcome::NoCredential,
            ),
            (
                reply(403, &json!({ "error": { "message": "Key disabled" } })),
                Outcome::NoCredential,
            ),
            (
                reply(
                    403,
                    &json!({ "error": { "message": "Key limit exceeded (daily limit)" } }),
                ),
                Outcome::Quota,
            ),
            (
                reply(
                    403,
                    &json!({ "error": { "message": "Request blocked: image size limit" } }),
                ),
                Outcome::NoCredential,
            ),
            (reply(402, &json!({})), Outcome::Quota),
            (reply(429, &json!({})), Outcome::RateLimited),
            (reply(400, &json!({})), Outcome::Refused),
            (reply(413, &json!({})), Outcome::Refused),
            (reply(422, &json!({})), Outcome::Refused),
            (reply(500, &json!({})), Outcome::Unavailable),
            (reply(529, &json!({})), Outcome::Unavailable),
            (reply(503, &json!({})), Outcome::Unavailable),
        ];
        for (r, want) in cases {
            let got = classify(&p, Ok(&r), 5);
            assert_eq!(
                got.outcome,
                want,
                "status {} body {}",
                r.status,
                String::from_utf8_lossy(&r.body)
            );
            assert_eq!(got.receipt_digest(), "r");
        }
        let failed = classify(&p, Err(HttpFailure::Connect), 5);
        assert_eq!(failed.outcome, Outcome::Unavailable);
        assert_eq!(failed.detail.as_deref(), Some("connect"));
        let not_json = HttpReply {
            status: 200,
            retry_after: None,
            body: b"<html>".to_vec(),
        };
        assert_eq!(classify(&p, Ok(&not_json), 5).outcome, Outcome::Malformed);
    }

    #[test]
    fn an_answer_missing_a_question_or_of_the_wrong_type_is_malformed() {
        let p = prepared();
        let mut body = good("typesafe/jev-1.14", 0.9);
        body["answers"].as_object_mut().expect("map").remove("rim");
        let got = classify(&p, Ok(&reply(200, &body)), 1);
        assert_eq!(
            (got.outcome, got.detail.as_deref()),
            (Outcome::Malformed, Some("answer_rim"))
        );
        let mut body = good("typesafe/jev-1.14", 0.9);
        body["answers"]["side"]["choice"] = json!("up");
        assert_eq!(
            classify(&p, Ok(&reply(200, &body)), 1).outcome,
            Outcome::Malformed
        );
        let mut body = good("typesafe/jev-1.14", 0.9);
        body["answers"]["rim"]["box"]["bottom"] = json!(1.5);
        assert_eq!(
            classify(&p, Ok(&reply(200, &body)), 1).outcome,
            Outcome::Malformed
        );
        let mut body = good("typesafe/jev-1.14", 0.9);
        body.as_object_mut().expect("map").remove("usage");
        assert_eq!(
            classify(&p, Ok(&reply(200, &body)), 1).outcome,
            Outcome::Malformed
        );
    }

    #[test]
    fn an_ask_names_a_known_set_the_wrist_frame_and_declared_numbers_only() {
        let c = config();
        let ok = r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/wrist/image","seq":3},"deadline_ms":900,"numbers":{"gap":3.5}}"#;
        let ask = parse_ask(ok, &c).expect("ask");
        assert_eq!(ask.deadline_ms, Some(900));
        assert_eq!(ask.frame.seq, 3);
        let bad = |text: &str| match parse_ask(text, &c) {
            Err(AskProblem::Malformed { detail, .. }) => detail,
            other => panic!("{text}: {other:?}"),
        };
        assert_eq!(
            bad(r#"{"ask_id":7,"question_set":"nope","frame":{"topic":"/wrist/image","seq":3}}"#),
            "unknown_question_set"
        );
        assert_eq!(
            bad(r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/other","seq":3}}"#),
            "frame"
        );
        assert_eq!(
            bad(
                r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/wrist/image","seq":3},"numbers":{"x":1}}"#
            ),
            "numbers"
        );
        assert_eq!(
            bad(
                r#"{"ask_id":7,"question_set":"press","frame":{"topic":"/wrist/image","seq":3},"prompt":"anything"}"#
            ),
            "unknown_key"
        );
        assert_eq!(bad(r#"{"ask_id":"ab","question_set":"press"}"#), "frame");
        assert!(matches!(
            parse_ask(r#"{"ask_id":{"x":1}}"#, &c),
            Err(AskProblem::NoId(_))
        ));
        assert!(matches!(
            parse_ask("not json", &c),
            Err(AskProblem::NoId(_))
        ));
    }
}
