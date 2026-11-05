// SPDX-License-Identifier: AGPL-3.0-only
//! The manifest's `eyes.jev` section.
//!
//! ```yaml
//! eyes:
//!   jev:
//!     base_url: https://openrouter.ai/api/v1 # the client posts to {base_url}/systemone
//!     model: typesafe/jev-1.14               # pinned: no alias
//!     credential_file: keys/jev.key          # optional; relative to the manifest
//!     region: US                             # optional; picks the data notice
//!     frame_topic: /wrist/image              # sensor_msgs/CompressedImage, JPEG
//!     ask_topic: /rcp/eye/ask                # optional; the default
//!     answer_topic: /rcp/eye/answer          # optional; the default
//!     timeout_ms: 1500                       # optional; 1..=1500, the default 1500
//!     question_sets:
//!       - name: press_gate
//!         state: Wrist camera frame from a small arm in front of a coffee machine.
//!         numbers: [jaw_gap_mm]              # optional; the numbers an ask may carry
//!         questions:
//!           edge_visible: { type: noul, instructions: The rim's lower edge is visible in `frame`. }
//!           rim: { type: box, instructions: The chrome rim band in `frame`. }
//!         visible: { question: edge_visible, threshold: 0.7 } # optional
//! ```
//!
//! With no `eyes.jev` section the daemon serves no eye. With one, Jev is on
//! whenever the credential file holds a key and the owner's switch is not
//! off ([`crate::eye::switch`]); while the switch is off every ask is
//! answered `disabled`. Unknown keys are refused.

use std::collections::BTreeMap;
use std::path::PathBuf;

use serde::Deserialize;
use serde_json::Value;

use crate::eye::limits;

/// The route used when the manifest names none: OpenRouter's API base, the
/// `route.base_url` the account service issues with a robot's key.
pub const DEFAULT_BASE_URL: &str = "https://openrouter.ai/api/v1";

/// The model used when the manifest names none.
pub const DEFAULT_MODEL: &str = "typesafe/jev-1.14";

/// The path the client appends to the base URL.
pub const SYSTEM_ONE_PATH: &str = "/systemone";

/// The most question sets a manifest declares.
pub const MAX_QUESTION_SETS: usize = 16;

/// The most questions one set asks.
pub const MAX_QUESTIONS: usize = 16;

/// The most numbers one set declares.
pub const MAX_NUMBERS: usize = 16;

/// The `eyes` section.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EyesConfig {
    /// Jev, when configured.
    #[serde(default)]
    pub jev: Option<JevConfig>,
}

/// The `eyes.jev` section as written.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JevConfig {
    /// The route: the client posts to this URL plus [`SYSTEM_ONE_PATH`].
    #[serde(default = "default_base_url")]
    pub base_url: String,
    /// The pinned model name sent in every request.
    #[serde(default = "default_model")]
    pub model: String,
    /// The credential file, relative to the manifest's directory. The
    /// [`crate::eye::JEV_KEY_FILE_ENV`] environment variable comes first;
    /// without either, `jev.key` in the receipt log's directory.
    #[serde(default)]
    pub credential_file: Option<PathBuf>,
    /// The region whose data notice applies; see [`crate::eye::switch`].
    #[serde(default)]
    pub region: Option<String>,
    /// The `sensor_msgs/CompressedImage` topic asks name frames on.
    pub frame_topic: String,
    /// The topic asks arrive on.
    #[serde(default = "default_ask_topic")]
    pub ask_topic: String,
    /// The topic answers are published on.
    #[serde(default = "default_answer_topic")]
    pub answer_topic: String,
    /// The longest an ask waits for its answer, milliseconds.
    #[serde(default = "default_timeout_ms")]
    pub timeout_ms: u64,
    /// The question sets an ask may name.
    pub question_sets: Vec<QuestionSet>,
}

/// One fixed question set.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QuestionSet {
    /// The name an ask uses.
    pub name: String,
    /// The fixed scene text sent as `state.scene`.
    pub state: String,
    /// The numbers an ask may carry, sent as `state.numbers`.
    #[serde(default)]
    pub numbers: Vec<String>,
    /// The typed questions, by name, sent as written.
    pub questions: BTreeMap<String, Value>,
    /// The Noul whose answer must reach a threshold for the look to count.
    #[serde(default)]
    pub visible: Option<Visible>,
}

/// The visibility rule of a question set.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Visible {
    /// The Noul question.
    pub question: String,
    /// Answers below this are `not_visible`.
    pub threshold: f64,
}

fn default_base_url() -> String {
    String::from(DEFAULT_BASE_URL)
}

fn default_model() -> String {
    String::from(DEFAULT_MODEL)
}

fn default_ask_topic() -> String {
    String::from("/rcp/eye/ask")
}

fn default_answer_topic() -> String {
    String::from("/rcp/eye/answer")
}

fn default_timeout_ms() -> u64 {
    limits::TIMEOUT_MS
}

/// Whether `name` is a question set, question or number name: 1 to 64
/// characters from `[A-Za-z0-9_]`.
pub fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 64
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
}

/// Whether `host` is a loopback host, the only kind a plain `http` route may
/// name.
fn loopback_host(host: &url::Host<&str>) -> bool {
    match host {
        url::Host::Domain(d) => d.eq_ignore_ascii_case("localhost"),
        url::Host::Ipv4(ip) => ip.is_loopback(),
        url::Host::Ipv6(ip) => ip.is_loopback(),
    }
}

/// The System One endpoint for `base_url`, or why it is refused: `https`
/// anywhere, plain `http` to a loopback host only, no user info, query or
/// fragment.
///
/// # Errors
///
/// The reason the URL is refused.
pub fn endpoint(base_url: &str) -> Result<url::Url, String> {
    let url = url::Url::parse(base_url).map_err(|e| format!("base_url {base_url:?}: {e}"))?;
    let Some(host) = url.host() else {
        return Err(format!("base_url {base_url:?} names no host"));
    };
    match url.scheme() {
        "https" => {}
        "http" if loopback_host(&host) => {}
        "http" => {
            return Err(format!(
                "base_url {base_url:?}: plain http is allowed to a loopback host only"
            ))
        }
        other => {
            return Err(format!(
                "base_url {base_url:?}: scheme {other:?} is not https"
            ))
        }
    }
    if !url.username().is_empty() || url.password().is_some() {
        return Err(String::from("base_url must not carry user info"));
    }
    if url.query().is_some() || url.fragment().is_some() {
        return Err(String::from("base_url must not carry a query or fragment"));
    }
    let joined = format!("{}{SYSTEM_ONE_PATH}", url.as_str().trim_end_matches('/'));
    url::Url::parse(&joined).map_err(|e| format!("base_url {base_url:?}: {e}"))
}

/// Whether `model` is pinned: not an alias (`~...` or `...latest...`).
pub fn pinned_model(model: &str) -> bool {
    !model.is_empty()
        && model.len() <= 128
        && !model.starts_with('~')
        && !model.contains("latest")
        && model
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'/' | b'.' | b'-' | b'_'))
}

/// Whether `answered` is the model `requested` names: the same name, or the
/// same name with a `-YYYYMMDD` release suffix. A leading `typesafe/` on
/// either side is ignored, since OpenRouter answers a bare TypeSafe name
/// under its `typesafe/` namespace.
pub fn model_matches(requested: &str, answered: &str) -> bool {
    let strip = |m: &str| m.strip_prefix("typesafe/").unwrap_or(m).to_owned();
    let requested = strip(requested);
    let answered = strip(answered);
    if answered == requested {
        return true;
    }
    answered
        .strip_prefix(requested.as_str())
        .and_then(|rest| rest.strip_prefix('-'))
        .is_some_and(|date| date.len() == 8 && date.bytes().all(|b| b.is_ascii_digit()))
}

/// Check one question as the System One API takes it, limited to text:
/// `type` is `noul`, `choice`, `score` or `box`; `instructions` is
/// non-empty text; `criteria` is what the type takes.
fn check_question(name: &str, question: &Value) -> Result<&'static str, String> {
    let fail = |why: &str| Err(format!("question {name:?}: {why}"));
    let Some(object) = question.as_object() else {
        return fail("must be a mapping");
    };
    for key in object.keys() {
        if !matches!(key.as_str(), "type" | "instructions" | "criteria") {
            return fail(&format!("unknown key {key:?}"));
        }
    }
    let kind = match object.get("type").and_then(Value::as_str) {
        Some("noul") => "noul",
        Some("choice") => "choice",
        Some("score") => "score",
        Some("box") => "box",
        _ => return fail("type must be noul, choice, score or box"),
    };
    match object.get("instructions").and_then(Value::as_str) {
        Some(text) if !text.trim().is_empty() => {}
        _ => return fail("instructions must be non-empty text"),
    }
    let criteria = object.get("criteria");
    fn all_text<'a>(mut values: impl Iterator<Item = &'a Value>) -> bool {
        values.all(Value::is_string)
    }
    match (kind, criteria) {
        ("box", None) | ("noul", None) => {}
        ("box", Some(_)) => return fail("a box question takes no criteria"),
        ("noul", Some(Value::Object(map)))
            if map.keys().all(|k| k == "true" || k == "false") && all_text(map.values()) => {}
        ("noul", Some(_)) => return fail("noul criteria name only `true` and `false`, as text"),
        ("choice", Some(Value::Object(map)))
            if (1..=255).contains(&map.len())
                && map.keys().all(|k| valid_name(k))
                && all_text(map.values()) => {}
        ("choice", _) => {
            return fail("choice criteria map 1 to 255 option names to text descriptions")
        }
        ("score", Some(Value::Array(levels)))
            if (1..=10).contains(&levels.len()) && all_text(levels.iter()) => {}
        ("score", _) => return fail("score criteria are 1 to 10 text levels"),
        _ => return fail("criteria do not match the type"),
    }
    Ok(kind)
}

impl JevConfig {
    /// Refuse a section the eye could not serve as written.
    ///
    /// # Errors
    ///
    /// The first problem found.
    pub fn check(&self) -> Result<(), String> {
        endpoint(&self.base_url)?;
        if !pinned_model(&self.model) {
            return Err(format!(
                "eyes.jev.model {:?} must be a pinned model name, not an alias",
                self.model
            ));
        }
        for topic in [&self.frame_topic, &self.ask_topic, &self.answer_topic] {
            if !topic.starts_with('/') {
                return Err(format!("eyes.jev topic {topic:?} must be absolute"));
            }
        }
        let topics = [&self.frame_topic, &self.ask_topic, &self.answer_topic];
        let distinct: std::collections::BTreeSet<_> = topics.iter().collect();
        if distinct.len() != topics.len() {
            return Err(String::from(
                "eyes.jev frame_topic, ask_topic and answer_topic must differ",
            ));
        }
        if self.timeout_ms == 0 || self.timeout_ms > limits::TIMEOUT_MS {
            return Err(format!(
                "eyes.jev.timeout_ms must lie in 1..={}",
                limits::TIMEOUT_MS
            ));
        }
        if let Some(region) = &self.region {
            crate::eye::switch::normalize_region(region)?;
        }
        if self.question_sets.is_empty() || self.question_sets.len() > MAX_QUESTION_SETS {
            return Err(format!(
                "eyes.jev.question_sets must hold 1 to {MAX_QUESTION_SETS} sets"
            ));
        }
        let mut names = std::collections::BTreeSet::new();
        for set in &self.question_sets {
            set.check()?;
            if !names.insert(set.name.as_str()) {
                return Err(format!("question set {:?} is declared twice", set.name));
            }
        }
        Ok(())
    }

    /// The question set named `name`.
    pub fn question_set(&self, name: &str) -> Option<&QuestionSet> {
        self.question_sets.iter().find(|s| s.name == name)
    }
}

impl QuestionSet {
    /// The bytes of fixed text this set sends: the scene and the questions.
    pub fn text_bytes(&self) -> usize {
        self.state.len() + serde_json::to_string(&self.questions).map_or(0, |s| s.len())
    }

    fn check(&self) -> Result<(), String> {
        let name = &self.name;
        if !valid_name(name) {
            return Err(format!(
                "question set name {name:?} must be 1 to 64 characters from [A-Za-z0-9_]"
            ));
        }
        if self.state.trim().is_empty() {
            return Err(format!(
                "question set {name:?}: state must be non-empty text"
            ));
        }
        if self.questions.is_empty() || self.questions.len() > MAX_QUESTIONS {
            return Err(format!(
                "question set {name:?} must ask 1 to {MAX_QUESTIONS} questions"
            ));
        }
        let mut kinds = BTreeMap::new();
        for (question, body) in &self.questions {
            if !valid_name(question) {
                return Err(format!(
                    "question set {name:?}: question name {question:?} must be 1 to 64 \
                     characters from [A-Za-z0-9_]"
                ));
            }
            let kind = check_question(question, body)
                .map_err(|e| format!("question set {name:?}: {e}"))?;
            kinds.insert(question.as_str(), kind);
        }
        if self.numbers.len() > MAX_NUMBERS {
            return Err(format!(
                "question set {name:?} declares more than {MAX_NUMBERS} numbers"
            ));
        }
        let mut numbers = std::collections::BTreeSet::new();
        for number in &self.numbers {
            if !valid_name(number) || !numbers.insert(number.as_str()) {
                return Err(format!(
                    "question set {name:?}: number {number:?} must be a unique name of 1 to 64 \
                     characters from [A-Za-z0-9_]"
                ));
            }
        }
        if let Some(visible) = &self.visible {
            if kinds.get(visible.question.as_str()) != Some(&"noul") {
                return Err(format!(
                    "question set {name:?}: visible.question {:?} must name one of its noul \
                     questions",
                    visible.question
                ));
            }
            if !(visible.threshold > 0.0 && visible.threshold < 1.0) {
                return Err(format!(
                    "question set {name:?}: visible.threshold must lie strictly between 0 and 1"
                ));
            }
        }
        let budget = limits::max_fixed_text_bytes();
        if self.text_bytes() > budget {
            return Err(format!(
                "question set {name:?}: its text is {} bytes; with the largest frame a request \
                 would exceed {} input tokens (the text budget is {budget} bytes)",
                self.text_bytes(),
                limits::MAX_INPUT_TOKENS
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{endpoint, model_matches, pinned_model, DEFAULT_BASE_URL, DEFAULT_MODEL};

    #[test]
    fn the_defaults_are_the_route_the_account_service_issues_with_a_key() {
        // The `route` object of the key endpoint's 200 response.
        let route = serde_json::json!({
            "base_url": "https://openrouter.ai/api/v1",
            "model": "typesafe/jev-1.14",
        });
        assert_eq!(route["base_url"], DEFAULT_BASE_URL);
        assert_eq!(route["model"], DEFAULT_MODEL);
        assert_eq!(
            endpoint(DEFAULT_BASE_URL).expect("ok").as_str(),
            "https://openrouter.ai/api/v1/systemone"
        );
    }

    #[test]
    fn a_route_is_https_or_loopback_http_with_nothing_attached() {
        assert_eq!(
            endpoint("https://openrouter.ai/api/v1")
                .expect("ok")
                .as_str(),
            "https://openrouter.ai/api/v1/systemone"
        );
        assert_eq!(
            endpoint("https://api.typesafe.ai/v1/")
                .expect("ok")
                .as_str(),
            "https://api.typesafe.ai/v1/systemone"
        );
        assert!(endpoint("http://127.0.0.1:9/api").is_ok());
        assert!(endpoint("http://localhost:9").is_ok());
        assert!(endpoint("http://[::1]:9").is_ok());
        assert!(endpoint("http://openrouter.ai/api").is_err());
        assert!(endpoint("https://user:pw@openrouter.ai/api").is_err());
        assert!(endpoint("https://openrouter.ai/api?key=x").is_err());
        assert!(endpoint("ftp://openrouter.ai").is_err());
    }

    #[test]
    fn an_alias_is_not_a_pinned_model() {
        assert!(pinned_model("typesafe/jev-1.14"));
        assert!(pinned_model("jev-1.14.0"));
        assert!(!pinned_model("~typesafe/jev-latest"));
        assert!(!pinned_model("jev-latest"));
        assert!(!pinned_model(""));
    }

    #[test]
    fn the_answered_model_is_the_pinned_one_or_its_dated_release() {
        assert!(model_matches("typesafe/jev-1.14", "typesafe/jev-1.14"));
        assert!(model_matches(
            "typesafe/jev-1.14",
            "typesafe/jev-1.14-20261014"
        ));
        assert!(model_matches("jev-1.14", "typesafe/jev-1.14-20261014"));
        assert!(model_matches("jev-1.14.0", "jev-1.14.0"));
        assert!(!model_matches(
            "typesafe/jev-1.14",
            "typesafe/jev-1.15-20261101"
        ));
        assert!(!model_matches(
            "typesafe/jev-1.14",
            "typesafe/jev-1.14-preview"
        ));
        assert!(!model_matches("typesafe/jev-1.14", "typesafe/jev-1.1"));
        assert!(!model_matches("typesafe/jev-1.14", ""));
    }
}
