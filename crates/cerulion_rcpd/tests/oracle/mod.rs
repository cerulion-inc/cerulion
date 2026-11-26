// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-written event/effect vectors for the tool surface.
//!
//! A [`Vector`] is the [`Event`]s an MCP client sends and the [`Effect`]s
//! `docs/rcp.md` says they have. Each expected effect is written by hand from
//! the doc line its vector names; none is copied from a run.
//!
//! The effects are what is visible from outside the daemon: the reply the
//! client reads, the frames the graph receives on the calls topic, the
//! receipt log's lines, the `tools/list` answers and the text of the latch
//! file `<receipts>.estop`. The daemon sits behind [`Daemon`], so any host
//! that serves the tools over MCP runs the same vectors.
//!
//! Effects are checked per channel: replies by request; frames, receipt
//! lines, `tools/list` answers, tool descriptions and latch file probes in
//! order. No vector writes a `call_id` as a number: a reply names the
//! frame whose `call_id` it carries.

pub mod graph;
pub mod vectors;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::manifest::{LoadedManifest, RcpManifest};
use native_ros2_messages::std_msgs::String as RosString;
use rmcp::model::{CallToolRequestParams, CallToolResult};
use rmcp::service::RunningService;
use rmcp::{RoleClient, ServiceExt};
use serde_json::Value;
use tokio::io::DuplexStream;

static COUNTER: AtomicU64 = AtomicU64::new(0);

/// A name unique to this process and call, for SHM nodes and topics.
pub fn unique(tag: &str) -> String {
    format!(
        "rcpd_tools_{tag}_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

/// A transport manager on a shared-memory root of its own.
pub fn manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(tag),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager")
}

/// The manifest every vector runs on, on topics private to `prefix`: three
/// reversible tools (`arm.mute`, whose timeout is 300 ms, is never answered),
/// one read tool, one irreversible tool and one state topic.
pub fn manifest(dir: &Path, prefix: &str) -> LoadedManifest {
    manifest_with(dir, prefix, "")
}

/// [`manifest`] with `extra` (top-level YAML, such as a `limits` section)
/// appended.
pub fn manifest_with(dir: &Path, prefix: &str, extra: &str) -> LoadedManifest {
    let step = |name: &str, timeout_ms: u32| {
        format!(
            "  - name: {name}
    description: one period
    safety_class: reversible
    timeout_ms: {timeout_ms}
    input_schema:
      type: object
      properties:
        q: {{ type: array, items: {{ type: number }}, minItems: 2, maxItems: 2 }}
      required: [q]
"
        )
    };
    let yaml = format!(
        "robot: tools_fixture
journal:
  calls_topic: /{prefix}/calls
  results_topic: /{prefix}/results
state:
  - name: arm
    topic: /{prefix}/state
tools:
{}{}{}  - name: arm.peek
    description: read only
    safety_class: read
    input_schema: {{ type: object, properties: {{}} }}
  - name: arm.cut
    description: cut the cable
    safety_class: irreversible
    input_schema: {{ type: object, properties: {{}} }}
{extra}",
        step("arm.step", 3000),
        step("arm.slow", 3000),
        step("arm.mute", 300),
    );
    let path = dir.join("rcp.yaml");
    std::fs::write(&path, yaml).expect("write manifest");
    RcpManifest::load_full(&path).expect("load manifest")
}

/// Publish `value` as one `std_msgs/String` frame.
pub fn publish_json(publisher: &mut cerulion_core::CerulionPublisher, value: &Value) {
    let mut proxy = publisher.loan_proxy::<RosString>().expect("loan");
    proxy.set_data(&value.to_string()).expect("set");
}

/// A daemon that serves the manifest's tools over MCP.
pub trait Daemon: Sized {
    /// Start the daemon for `manifest` on `mgr`, with its receipt log at
    /// `receipts`. It serves MCP on the far end of the returned stream.
    fn boot(
        mgr: &Arc<TransportManager>,
        manifest: &LoadedManifest,
        receipts: &Path,
    ) -> (Self, DuplexStream);

    /// Every receipt line so far, oldest first, as `(verb, outcome)` in the
    /// words `docs/rcp.md` uses: `intent`, `ok`, `denied`, `error:<class>`.
    fn entries(&self) -> Vec<(String, String)>;
}

/// A request's label inside one vector.
pub type Req = u8;

/// What the client does.
pub enum Event {
    /// `tools/call` of `tool` with `args`, then wait for its reply.
    Call { req: Req, tool: String, args: Value },
    /// `tools/list`.
    List,
    /// `tools/list`, keeping the description of this tool.
    Describe(&'static str),
    /// Read the latch file `<receipts>.estop`, if it exists.
    ProbeLatchFile,
}

/// `tools/call` of `tool` with `args` as request `req`.
pub fn call(req: Req, tool: impl Into<String>, args: Value) -> Event {
    Event::Call {
        req,
        tool: tool.into(),
        args,
    }
}

/// How a reply ends.
#[derive(Debug)]
pub enum Outcome {
    /// `isError` is not set.
    Ran,
    /// `isError: true` with this `error.code`.
    Refused(&'static str),
    /// `isError: true`, `error.code` `timeout` and `may_have_executed: true`.
    TimedOut,
}

/// A receipt line's outcome.
#[derive(Debug, Clone, Copy)]
pub enum Stage {
    Intent,
    Ok,
    Denied,
    /// `error:<class>`.
    Error(&'static str),
}

impl Stage {
    /// The outcome as `docs/rcp.md` writes it.
    fn word(self) -> String {
        match self {
            Self::Intent => String::from("intent"),
            Self::Ok => String::from("ok"),
            Self::Denied => String::from("denied"),
            Self::Error(class) => format!("error:{class}"),
        }
    }
}

/// What the client, the graph and the receipt log see.
pub enum Effect {
    /// The reply to `req`. When `frame` is set, the reply's `call_id` is the
    /// `call_id` of that frame (counted from 0 on the calls topic), and its
    /// `result` is the stand-in graph's answer to that frame. When
    /// `receipted` is set, its `receipt` is `{"seq", "hash", "prev"}`. Each of
    /// `carries` is a JSON pointer into the reply body and the value there;
    /// each of `says` a pointer to a string and what that string says.
    Reply {
        req: Req,
        outcome: Outcome,
        frame: Option<usize>,
        receipted: bool,
        carries: Vec<(&'static str, Value)>,
        says: Vec<(&'static str, Says)>,
    },
    /// The next frame on the calls topic.
    Frame { tool: &'static str, args: Value },
    /// The next receipt line.
    Entry { verb: &'static str, stage: Stage },
    /// The next `tools/list` answer: the tool names, in order.
    Listed(Vec<&'static str>),
    /// The next latch file probe: the file's text, `None` when it does not
    /// exist.
    LatchFile(Option<String>),
    /// The next [`Event::Describe`]: what that tool's description says.
    Described(Vec<Says>),
}

/// What a piece of text says.
#[derive(Debug)]
pub enum Says {
    /// It contains this text.
    Mentions(&'static str),
    /// It does not contain this text.
    Omits(&'static str),
}

impl Says {
    /// Whether `text` says it.
    fn holds(&self, text: &str) -> bool {
        match self {
            Self::Mentions(part) => text.contains(part),
            Self::Omits(part) => !text.contains(part),
        }
    }
}

/// The next latch file probe finds the file holding `text`.
pub fn latch_holds(text: impl Into<String>) -> Effect {
    Effect::LatchFile(Some(text.into()))
}

/// The next latch file probe finds no file.
pub fn no_latch_file() -> Effect {
    Effect::LatchFile(None)
}

/// The reply to `req` ends with `outcome`.
pub fn reply(req: Req, outcome: Outcome) -> Effect {
    Effect::Reply {
        req,
        outcome,
        frame: None,
        receipted: false,
        carries: Vec::new(),
        says: Vec::new(),
    }
}

/// The next frame on the calls topic is `tool` with `args`.
pub fn frame(tool: &'static str, args: Value) -> Effect {
    Effect::Frame { tool, args }
}

/// The next receipt line is `verb` with `stage`.
pub fn entry(verb: &'static str, stage: Stage) -> Effect {
    Effect::Entry { verb, stage }
}

impl Effect {
    /// This reply answers frame `n`.
    pub fn answering(mut self, n: usize) -> Self {
        let Self::Reply { frame, .. } = &mut self else {
            panic!("only a reply answers a frame");
        };
        *frame = Some(n);
        self
    }

    /// This reply's body holds `value` at the JSON `pointer`.
    pub fn carrying(mut self, pointer: &'static str, value: Value) -> Self {
        let Self::Reply { carries, .. } = &mut self else {
            panic!("only a reply carries fields");
        };
        carries.push((pointer, value));
        self
    }

    /// This reply carries its `receipt` as `{"seq", "hash", "prev"}`.
    pub fn receipted(mut self) -> Self {
        let Self::Reply { receipted, .. } = &mut self else {
            panic!("only a reply carries a receipt");
        };
        *receipted = true;
        self
    }

    /// This reply's body holds a string containing `text` at the JSON
    /// `pointer`.
    pub fn mentioning(self, pointer: &'static str, text: &'static str) -> Self {
        self.saying(pointer, Says::Mentions(text))
    }

    /// This reply's body holds a string at the JSON `pointer` that does not
    /// contain `text`.
    pub fn omitting(self, pointer: &'static str, text: &'static str) -> Self {
        self.saying(pointer, Says::Omits(text))
    }

    fn saying(mut self, pointer: &'static str, said: Says) -> Self {
        let Self::Reply { says, .. } = &mut self else {
            panic!("only a reply holds text");
        };
        says.push((pointer, said));
        self
    }
}

/// What is on disk before the daemon starts.
pub enum Latch {
    /// No latch file.
    Absent,
    /// A latch file an earlier daemon left.
    LeftClosed,
    /// A latch file that cannot be deleted: a non-empty directory at its path.
    Undeletable,
}

/// Who clears the e-stop: the manifest's `limits.estop_clear`.
pub enum Clearing {
    /// `agent`, the default.
    Agent,
    /// `operator`.
    Operator,
}

/// The state a vector starts from.
pub struct Given {
    pub clearing: Clearing,
    pub latch: Latch,
}

impl Given {
    /// A first start under the default limits: no latch file.
    pub fn fresh() -> Self {
        Self {
            clearing: Clearing::Agent,
            latch: Latch::Absent,
        }
    }
}

/// One scenario. `expect` lists the effects in the order `docs/rcp.md`
/// gives them; only the order within each channel is checked.
pub struct Vector {
    pub given: Given,
    pub events: Vec<Event>,
    pub expect: Vec<Effect>,
}

/// The latch file for the receipt log at `receipts`, by its documented name.
fn latch_path_of(receipts: &Path) -> PathBuf {
    let mut name = receipts.as_os_str().to_owned();
    name.push(".estop");
    PathBuf::from(name)
}

/// What one vector's events produced, per channel.
pub struct Trace {
    /// `(isError, body)` per request.
    pub replies: BTreeMap<Req, (bool, Value)>,
    /// Every frame the graph received on the calls topic, in order.
    pub frames: Vec<Value>,
    /// The receipt log, as [`Daemon::entries`] gives it.
    pub entries: Vec<(String, String)>,
    /// The tool names of each `tools/list`, in order.
    pub listed: Vec<Vec<String>>,
    /// Each latch file probe, in order: the file's text, if it exists.
    pub latch: Vec<Option<String>>,
    /// The description each [`Event::Describe`] kept, in order.
    pub described: Vec<String>,
}

/// A daemon serving the vector manifest, the stand-in graph, and one MCP
/// client of the daemon.
pub struct Session<D> {
    client: RunningService<RoleClient, ()>,
    daemon: D,
    graph: graph::Graph,
    latch_file: PathBuf,
    _dir: tempfile::TempDir,
}

impl<D: Daemon> Session<D> {
    /// Boot `D` from `given` on the vector manifest, with the stand-in
    /// graph answering.
    pub async fn open(given: &Given) -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        let prefix = unique("vector");
        let limits = match given.clearing {
            Clearing::Agent => "",
            Clearing::Operator => "limits:\n  estop_clear: operator\n",
        };
        let loaded = manifest_with(dir.path(), &prefix, limits);
        let mgr = manager("vector");
        let graph = graph::Graph::spawn(mgr.clone(), &prefix);
        let receipts: PathBuf = dir.path().join("receipts/log.jsonl");
        let latch_file = latch_path_of(&receipts);
        match given.latch {
            Latch::Absent => {}
            Latch::LeftClosed => {
                std::fs::create_dir_all(dir.path().join("receipts")).expect("mkdir");
                std::fs::write(&latch_file, "earlier stop\n").expect("write latch file");
            }
            Latch::Undeletable => {
                std::fs::create_dir_all(latch_file.join("pinned")).expect("mkdir");
            }
        }
        let (daemon, stream) = D::boot(&mgr, &loaded, &receipts);
        let client = ().serve(stream).await.expect("client session");
        Self {
            client,
            daemon,
            graph,
            latch_file,
            _dir: dir,
        }
    }

    /// `tools/call`: `(isError, body)`; the body is the one text block's JSON.
    pub async fn call(&self, tool: &str, args: &Value) -> (bool, Value) {
        let params = CallToolRequestParams::new(tool.to_string())
            .with_arguments(args.as_object().cloned().expect("object args"));
        let result: CallToolResult = self.client.call_tool(params).await.expect("call_tool");
        let text = result
            .content
            .first()
            .and_then(|c| c.as_text())
            .map(|t| t.text.clone())
            .expect("one text block");
        (
            result.is_error == Some(true),
            serde_json::from_str(&text).expect("JSON body"),
        )
    }

    /// `tools/list`: each tool's name and description, in order.
    pub async fn list(&self) -> Vec<(String, String)> {
        self.client
            .list_all_tools()
            .await
            .expect("list")
            .into_iter()
            .map(|t| {
                let description = t.description.as_deref().unwrap_or_default().to_string();
                (t.name.to_string(), description)
            })
            .collect()
    }

    /// The latch file's text, `None` when it does not exist.
    fn read_latch_file(&self) -> Option<String> {
        match std::fs::read_to_string(&self.latch_file) {
            Ok(text) => Some(text),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
            Err(e) => panic!("the latch file cannot be read: {e}"),
        }
    }
}

/// Run `events` against a `D` started from `given` and record what each
/// channel saw.
pub async fn play<D: Daemon>(given: &Given, events: &[Event]) -> Trace {
    let session = Session::<D>::open(given).await;
    let mut replies = BTreeMap::new();
    let mut listed = Vec::new();
    let mut described = Vec::new();
    let mut latch = Vec::new();
    for event in events {
        match event {
            Event::Call { req, tool, args } => {
                let answer = session.call(tool, args).await;
                assert!(
                    replies.insert(*req, answer).is_none(),
                    "request {req} is sent twice"
                );
            }
            Event::List => {
                let names = session.list().await.into_iter().map(|(name, _)| name);
                listed.push(names.collect());
            }
            Event::Describe(tool) => {
                let tools = session.list().await;
                let found = tools.into_iter().find(|(name, _)| name == tool);
                let (_, description) = found.unwrap_or_else(|| panic!("{tool} is not listed"));
                described.push(description);
            }
            Event::ProbeLatchFile => latch.push(session.read_latch_file()),
        }
    }
    let frames = session.graph.stop();
    let entries = session.daemon.entries();
    Trace {
        replies,
        frames,
        entries,
        listed,
        latch,
        described,
    }
}

/// Run `vector` against a fresh `D` and check every channel.
pub async fn run<D: Daemon>(vector: Vector) {
    let trace = play::<D>(&vector.given, &vector.events).await;
    check(&vector.expect, &trace);
}

/// Check `trace` against `expect`, channel by channel.
fn check(expect: &[Effect], trace: &Trace) {
    let mut replies = BTreeMap::new();
    let mut frames: Vec<(String, Value)> = Vec::new();
    let mut entries: Vec<(String, String)> = Vec::new();
    let mut listed: Vec<Vec<String>> = Vec::new();
    let mut latch = Vec::new();
    let mut described = Vec::new();
    for effect in expect {
        match effect {
            Effect::Reply {
                req,
                outcome,
                frame,
                receipted,
                carries,
                says,
            } => {
                assert!(
                    replies
                        .insert(*req, (outcome, frame, receipted, carries, says))
                        .is_none(),
                    "request {req} has two expected replies"
                );
            }
            Effect::Frame { tool, args } => frames.push((tool.to_string(), args.clone())),
            Effect::Entry { verb, stage } => entries.push((verb.to_string(), stage.word())),
            Effect::Listed(names) => listed.push(names.iter().map(|n| n.to_string()).collect()),
            Effect::LatchFile(text) => latch.push(text.clone()),
            Effect::Described(says) => described.push(says),
        }
    }

    let seen: Vec<(String, Value)> = trace
        .frames
        .iter()
        .map(|f| {
            let tool = f["tool"].as_str().unwrap_or_default().to_string();
            (tool, f["args"].clone())
        })
        .collect();
    assert_eq!(seen, frames, "the frames on the calls topic");
    let ids: Vec<&Value> = trace.frames.iter().map(|f| &f["call_id"]).collect();
    for (i, id) in ids.iter().enumerate() {
        let exact = id.as_u64().is_some_and(|id| id < 1 << 53);
        assert!(
            exact,
            "frame {i} has no call_id a JSON number holds exactly: {id}"
        );
        assert!(!ids[..i].contains(id), "frame {i} reuses call_id {id}");
    }
    assert_eq!(trace.entries, entries, "the receipt log");
    assert_eq!(trace.listed, listed, "the tools/list answers");
    assert_eq!(trace.latch, latch, "the latch file probes");
    assert_eq!(
        trace.described.len(),
        described.len(),
        "every described tool has its expected description"
    );
    for (text, says) in trace.described.iter().zip(described) {
        for said in says {
            assert!(said.holds(text), "the description {said:?}: {text}");
        }
    }

    assert_eq!(
        trace.replies.keys().collect::<Vec<_>>(),
        replies.keys().collect::<Vec<_>>(),
        "every request has exactly one expected reply"
    );
    for (req, (outcome, frame, receipted, carries, says)) in replies {
        let (is_error, body) = &trace.replies[&req];
        match outcome {
            Outcome::Ran => assert!(!is_error, "request {req} ran: {body}"),
            Outcome::Refused(code) => {
                assert!(is_error, "request {req} is refused: {body}");
                assert_eq!(body["error"]["code"], *code, "request {req}: {body}");
            }
            Outcome::TimedOut => {
                assert!(is_error, "request {req} timed out: {body}");
                assert_eq!(body["error"]["code"], "timeout", "request {req}: {body}");
                assert_eq!(body["may_have_executed"], true, "request {req}: {body}");
            }
        }
        if let Some(n) = frame {
            let answered = &trace.frames[*n];
            let id = &answered["call_id"];
            assert_eq!(&body["call_id"], id, "request {req} answers frame {n}");
            assert_eq!(
                body["result"],
                graph::answer(answered),
                "request {req} carries the graph's answer to frame {n}"
            );
        }
        for (pointer, value) in carries {
            assert_eq!(
                body.pointer(pointer),
                Some(value),
                "request {req} at {pointer}: {body}"
            );
        }
        if *receipted {
            let receipt = &body["receipt"];
            let shaped = receipt["seq"].is_u64()
                && receipt["hash"].is_string()
                && receipt["prev"].is_string();
            assert!(shaped, "request {req} carries its receipt: {body}");
        }
        for (pointer, said) in says {
            let found = body.pointer(pointer).and_then(Value::as_str);
            assert!(
                found.is_some_and(|found| said.holds(found)),
                "request {req} at {pointer} {said:?}: {body}"
            );
        }
    }
}
