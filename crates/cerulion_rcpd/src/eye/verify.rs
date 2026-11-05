// SPDX-License-Identifier: AGPL-3.0-only
//! `--verify-bag`: a recording's Jev answers joined to the receipt log.
//!
//! [`verify_bag`] checks, and reports every failure:
//!
//! 1. The receipt log's hash chain holds ([`crate::audit::verify_log`]).
//! 2. Every answer frame has its own `eye.jev` receipt from `rcp-eye`: the
//!    receipt's `args_digest` is the answer's `request_sha256` (or its
//!    `ask_sha256` when no request was sent) and its outcome is the answer's.
//!    One receipt joins one answer.
//! 3. Every answer names an ask the recording holds.
//! 4. Every `late` receipt follows a `timeout` answer with the same request.
//! 5. Where the recording holds the frame an answer sent, the image's
//!    SHA-256 and the request body's SHA-256 recompute from the frame and
//!    the answer's recorded model, state and questions. An answer that sent
//!    a request but does not record its frame, model, state or questions
//!    fails.
//!
//! Asks without an answer are counted, not failed: a recording can stop
//! between an ask and its answer.

use std::collections::BTreeMap;
use std::path::Path;

use cerud::receipt::{read_all, ReceiptOutcome};
use cerulion_bag::BagReader;
use serde_json::Value;

use crate::eye::client::{request_body, sha256_hex};
use crate::eye::replay::{image_frames, string_frames};
use crate::eye::{EYE_CALLER, JEV_VERB};

/// The topics to read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EyeTopics {
    /// The ask topic.
    pub ask: String,
    /// The answer topic.
    pub answer: String,
    /// The frame topic, when known.
    pub frame: Option<String>,
}

impl Default for EyeTopics {
    fn default() -> Self {
        Self {
            ask: String::from("/rcp/eye/ask"),
            answer: String::from("/rcp/eye/answer"),
            frame: None,
        }
    }
}

/// What [`verify_bag`] found.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EyeBagReport {
    /// Ask frames in the recording.
    pub asks: usize,
    /// Answer frames in the recording.
    pub answers: usize,
    /// Answers joined to a receipt.
    pub joined: usize,
    /// Asks with no answer in the recording.
    pub unanswered: usize,
    /// `late` receipts.
    pub late: usize,
    /// Answers whose request was recomputed from the recorded frame.
    pub requests_checked: usize,
    /// Every failed check, in the order found.
    pub problems: Vec<String>,
}

fn receipt_code(outcome: &ReceiptOutcome) -> String {
    match outcome {
        ReceiptOutcome::Ok => String::from("ok"),
        ReceiptOutcome::Denied => String::from("denied"),
        ReceiptOutcome::Intent => String::from("intent"),
        ReceiptOutcome::Error(code) => code.clone(),
    }
}

/// Join the answers in the bag at `bag` to the receipt log at `receipts`.
///
/// # Errors
///
/// Why the bag or the receipt log could not be read; failed checks are in
/// [`EyeBagReport::problems`].
pub fn verify_bag(bag: &Path, receipts: &Path, topics: &EyeTopics) -> Result<EyeBagReport, String> {
    let mut report = EyeBagReport::default();
    let audit = crate::audit::verify_log(receipts);
    let files = match audit {
        Ok(audit) => audit.files,
        Err(e) => {
            report.problems.push(format!("receipt chain: {e}"));
            vec![receipts.to_path_buf()]
        }
    };
    let mut pool: BTreeMap<(String, String), usize> = BTreeMap::new();
    let mut late: Vec<String> = Vec::new();
    for file in &files {
        for receipt in read_all(file).map_err(|e| e.to_string())? {
            if receipt.caller != EYE_CALLER || receipt.verb != JEV_VERB {
                continue;
            }
            let code = receipt_code(&receipt.outcome);
            if code == "late" {
                late.push(receipt.args_digest);
            } else {
                *pool.entry((receipt.args_digest, code)).or_default() += 1;
            }
        }
    }
    let reader = BagReader::open(bag).map_err(|e| e.to_string())?;
    let asks = string_frames(&reader, &topics.ask)?;
    let answers = string_frames(&reader, &topics.answer)?;
    let images = match &topics.frame {
        Some(topic) => image_frames(&reader, topic)?,
        None => BTreeMap::new(),
    };
    report.asks = asks.len();
    report.answers = answers.len();
    let mut asked: BTreeMap<String, usize> = BTreeMap::new();
    for (_, text) in &asks {
        if let Some(id) = crate::eye::answer::ask_id_of(text) {
            *asked.entry(id.to_string()).or_default() += 1;
        }
    }
    let mut answered: BTreeMap<String, usize> = BTreeMap::new();
    let mut timeouts: BTreeMap<String, usize> = BTreeMap::new();
    for (seq, text) in &answers {
        let Ok(answer) = serde_json::from_str::<Value>(text) else {
            report
                .problems
                .push(format!("answer frame {seq} is not JSON"));
            continue;
        };
        let id = answer.get("ask_id").cloned().unwrap_or(Value::Null);
        let outcome = answer.get("outcome").and_then(Value::as_str).unwrap_or("");
        let request = answer.get("request_sha256").and_then(Value::as_str);
        let Some(digest) = request.or_else(|| answer.get("ask_sha256").and_then(Value::as_str))
        else {
            report
                .problems
                .push(format!("answer to ask {id} carries no digest"));
            continue;
        };
        let key = (digest.to_owned(), outcome.to_owned());
        match pool.get_mut(&key) {
            Some(n) if *n > 0 => {
                *n -= 1;
                report.joined += 1;
            }
            _ => report.problems.push(format!(
                "answer to ask {id} ({outcome}) has no receipt with digest {digest}"
            )),
        }
        if outcome == "timeout" {
            if let Some(request) = request {
                *timeouts.entry(request.to_owned()).or_default() += 1;
            }
        }
        *answered.entry(id.to_string()).or_default() += 1;
        if !asked.contains_key(&id.to_string()) {
            report
                .problems
                .push(format!("answer to ask {id} has no ask in the recording"));
        }
        check_request(&answer, &images, &mut report);
    }
    for digest in late {
        report.late += 1;
        match timeouts.get_mut(&digest) {
            Some(n) if *n > 0 => *n -= 1,
            _ => report.problems.push(format!(
                "a late receipt (digest {digest}) follows no timeout answer"
            )),
        }
    }
    report.unanswered = asked
        .iter()
        .map(|(id, n)| n.saturating_sub(*answered.get(id).unwrap_or(&0)))
        .sum();
    Ok(report)
}

/// Recompute the image and request digests of `answer` from the recorded
/// frame it names, when it sent a request and the recording holds that
/// frame. [`EyeBagReport::requests_checked`] counts the requests whose
/// digests were recomputed; each failure goes to
/// [`EyeBagReport::problems`].
fn check_request(answer: &Value, images: &BTreeMap<u32, Vec<u8>>, report: &mut EyeBagReport) {
    let Some(request) = answer.get("request_sha256").and_then(Value::as_str) else {
        return;
    };
    let id = answer.get("ask_id").cloned().unwrap_or(Value::Null);
    let Some(seq) = answer
        .get("frame")
        .and_then(|f| f.get("seq"))
        .and_then(Value::as_u64)
        .and_then(|seq| u32::try_from(seq).ok())
    else {
        report.problems.push(format!(
            "answer to ask {id} sent a request but names no frame"
        ));
        return;
    };
    let Some(image) = images.get(&seq) else {
        return;
    };
    let field = |name: &str| answer.get(name).filter(|v| !v.is_null());
    let model = field("model_requested").and_then(Value::as_str);
    let (Some(model), Some(state), Some(questions)) = (model, field("state"), field("questions"))
    else {
        report.problems.push(format!(
            "answer to ask {id} sent a request but does not record its model, state and questions"
        ));
        return;
    };
    let image_sha256 = sha256_hex(image);
    let body = request_body(model, state, questions, image);
    report.requests_checked += 1;
    if answer.get("image_sha256").and_then(Value::as_str) != Some(image_sha256.as_str()) {
        report.problems.push(format!(
            "answer to ask {id}: the recorded frame {seq} is not the image it sent"
        ));
    }
    if sha256_hex(&body) != request {
        report.problems.push(format!(
            "answer to ask {id}: request_sha256 does not recompute from the recording"
        ));
    }
}
