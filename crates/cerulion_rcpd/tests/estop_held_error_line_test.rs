// SPDX-License-Identifier: AGPL-3.0-only
//! A tool call writes the error line of a latch file or a receipt it could
//! not write after `latch_io` and the receipt log's lock are released, and
//! `estop` after its frame is queued, so a log writer that holds that line
//! does not hold the frame the graph is stopped with.
//!
//! Each test makes one tool call on a thread of its own under a scoped
//! subscriber whose writer holds one line, and keeps a second
//! dispatcher alive, so tracing-core computes each callsite's interest over
//! every live dispatcher rather than one thread's default. While the line is
//! held a second `estop`, which takes `latch_io` and the receipt log's lock,
//! runs to its answer.
//! Unix only: the latch file is made unwritable with a symlink. Transport
//! isolation is per test (`init_for_test` SHM roots, unique topics), so the
//! tests run in parallel. Every wait is bounded.
#![cfg(unix)]

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::time::Duration;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::JournalConfig;
use cerulion_rcpd::tools::{latch_path, ReceiptLock, ToolHost, ESTOP_CLEAR_TOOL, ESTOP_TOOL};
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};

/// How long a test waits for each event: the stand-in graph to start, the
/// held line to be written, a frame to reach the graph, and the second
/// `estop`'s answer. The journal publishes an urgent frame within one loop
/// pass (about 1 ms), and 5 s leaves room for a loaded host.
const OBSERVE: Duration = Duration::from_secs(5);

/// How long the writer holds its line if the test never releases it: longer
/// than the at most three [`OBSERVE`] waits made while it is held.
const HOLD_BOUND: Duration = Duration::from_secs(60);

/// How long the held call, and a second `estop` that did not answer while
/// the line was held, get to answer after the release: the graph answers at
/// once, and `estop` and `estop_clear` wait at most 2 s for it.
const ANSWER: Duration = Duration::from_secs(10);

/// The `reason` every call passes.
const REASON: &str = "held";

static COUNTER: AtomicU64 = AtomicU64::new(0);

#[derive(Default)]
struct Held {
    lines: Vec<String>,
    reached: bool,
    released: bool,
    /// `Some(true)` when [`HeldLine::release`] ended the hold, `Some(false)`
    /// when [`HOLD_BOUND`] did.
    ended_by_release: Option<bool>,
}

/// A log writer that keeps every line and holds the first line containing
/// `needle` until [`HeldLine::release`], at most [`HOLD_BOUND`].
#[derive(Clone)]
struct HeldLine {
    needle: &'static str,
    state: Arc<(Mutex<Held>, Condvar)>,
}

impl HeldLine {
    fn new(needle: &'static str) -> Self {
        Self {
            needle,
            state: Arc::default(),
        }
    }

    /// Whether the held line was written within [`OBSERVE`].
    fn wait_reached(&self) -> bool {
        let (state, wake) = &*self.state;
        let held = state.lock().unwrap();
        let (held, _) = wake
            .wait_timeout_while(held, OBSERVE, |h| !h.reached)
            .unwrap();
        held.reached
    }

    fn release(&self) {
        let (state, wake) = &*self.state;
        state.lock().unwrap().released = true;
        wake.notify_all();
    }

    fn ended_by_release(&self) -> Option<bool> {
        self.state.0.lock().unwrap().ended_by_release
    }

    fn lines(&self) -> Vec<String> {
        self.state.0.lock().unwrap().lines.clone()
    }
}

impl Write for HeldLine {
    fn write(&mut self, line: &[u8]) -> std::io::Result<usize> {
        let text = String::from_utf8_lossy(line).into_owned();
        let (state, wake) = &*self.state;
        let mut held = state.lock().unwrap();
        let hold = !held.reached && text.contains(self.needle);
        held.lines.push(text);
        if hold {
            held.reached = true;
            wake.notify_all();
            let (mut after, waited) = wake
                .wait_timeout_while(held, HOLD_BOUND, |h| !h.released)
                .unwrap();
            after.ended_by_release = Some(!waited.timed_out());
        }
        Ok(line.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// Make `<receipts>.estop` a symlink into a directory that does not exist:
/// the latch file reads absent and cannot be created.
fn break_latch_file(receipts: &Path) {
    let missing = receipts
        .parent()
        .expect("receipts dir")
        .join("missing/latch");
    std::os::unix::fs::symlink(missing, latch_path(receipts)).expect("symlink");
}

/// Forwards every call frame on `calls` to `frames`, then answers it with
/// `{call_id, ok}` on `results`, until `stop` is set.
fn spawn_graph(
    mgr: Arc<TransportManager>,
    config: JournalConfig,
    stop: Arc<AtomicBool>,
    frames: mpsc::Sender<Value>,
) -> std::thread::JoinHandle<()> {
    let (ready_tx, ready_rx) = mpsc::channel();
    let stop_unready = stop.clone();
    let handle = std::thread::spawn(move || {
        let calls = mgr
            .create_subscriber(&config.calls_topic)
            .expect("calls subscriber");
        let mut results = mgr
            .create_publisher_simple(&config.results_topic, MaxSliceLen::const_new(4096))
            .expect("results publisher");
        ready_tx.send(()).expect("ready");
        while !stop.load(Ordering::Relaxed) {
            let mut batch = Vec::new();
            calls
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    batch.push(serde_json::from_str::<Value>(&text).expect("call is JSON"));
                })
                .expect("receive");
            if batch.is_empty() {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            for call in batch {
                let answer = json!({ "call_id": call["call_id"], "ok": true });
                let _ = frames.send(call);
                let mut proxy = results.loan_proxy::<RosString>().expect("loan");
                proxy.set_data(&answer.to_string()).expect("set");
            }
        }
    });
    if ready_rx.recv_timeout(OBSERVE).is_err() {
        stop_unready.store(true, Ordering::Relaxed);
        panic!("the stand-in graph did not start");
    }
    handle
}

/// A tool call's answer receiver and thread.
type Call = (mpsc::Receiver<Value>, std::thread::JoinHandle<()>);

/// Call `tool` on a thread of its own, every line it logs written to `line`
/// (to no subscriber when `None`). The receiver yields its answer's body.
fn spawn_call(host: Arc<ToolHost>, tool: &'static str, line: Option<HeldLine>) -> Call {
    let (done_tx, done_rx) = mpsc::channel();
    let handle = std::thread::spawn(move || {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("runtime");
        let args = json!({ "reason": REASON }).as_object().cloned();
        let call = || runtime.block_on(host.call(tool, args));
        let result = match line {
            Some(line) => {
                let subscriber = tracing_subscriber::fmt()
                    .without_time()
                    .with_ansi(false)
                    .with_writer(move || line.clone())
                    .finish();
                tracing::subscriber::with_default(subscriber, call)
            }
            None => call(),
        };
        let text = result
            .content
            .first()
            .and_then(|c| c.as_text())
            .map(|t| t.text.clone())
            .expect("one text block");
        let _ = done_tx.send(serde_json::from_str(&text).expect("JSON body"));
    });
    (done_rx, handle)
}

/// Releases the held line and stops the stand-in graph when dropped, so a
/// panic in [`held_call`] leaves no thread holding or polling.
struct Cleanup {
    line: HeldLine,
    stop: Arc<AtomicBool>,
}

impl Drop for Cleanup {
    fn drop(&mut self) {
        self.line.release();
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// `<verb>:<outcome>`, then `:<detail>` when it has one, of every receipt in
/// the log, oldest first.
fn outcomes(receipts: &Path) -> Vec<String> {
    cerud::receipt::read_all(receipts)
        .expect("read receipts")
        .iter()
        .map(|r| {
            let outcome = serde_json::to_value(&r.outcome).expect("outcome");
            let tag = outcome["outcome"].as_str().unwrap_or("");
            match outcome.get("detail").and_then(Value::as_str) {
                Some(detail) => format!("{}:{tag}:{detail}", r.verb),
                None => format!("{}:{tag}", r.verb),
            }
        })
        .collect()
}

/// What one call under a held line showed. `_while_held` fields were read
/// once the line was held (after an `estop`'s own frame reached the graph)
/// and before the second `estop` ran.
struct Observed {
    receipts: PathBuf,
    reached: bool,
    /// The held `estop`'s frame; not read for any other tool.
    frame: Option<Value>,
    outcomes_while_held: Vec<String>,
    /// The latch file's text; `None` when it cannot be read.
    latch_file_while_held: Option<String>,
    /// The second `estop`'s frame and answer, both while the line was held.
    second_frame: Option<Value>,
    second_answer: Option<Value>,
    ended_by_release: Option<bool>,
    answer: Value,
    lines: Vec<String>,
    outcomes: Vec<String>,
}

/// Open a host on topics private to `tag` (after `prepare` ran on the receipt
/// log's path), arm a failed receipt when `fail_receipt`, call `tool` with
/// the line containing `needle` held and, once the line is held (and an
/// `estop`'s frame arrived), a second `estop`; then release the line.
fn held_call(
    tag: &str,
    tool: &'static str,
    needle: &'static str,
    prepare: impl FnOnce(&Path),
    fail_receipt: bool,
) -> Observed {
    let _keep = tracing::Dispatch::new(tracing::subscriber::NoSubscriber::default());
    let dir = tempfile::tempdir().expect("tempdir");
    let receipts = dir.path().join("receipts/log.jsonl");
    std::fs::create_dir_all(receipts.parent().expect("receipts dir")).expect("mkdir");
    prepare(&receipts);
    let tag = format!(
        "rcpd_held_line_{tag}_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    );
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: tag.clone(),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("transport");
    let config = JournalConfig {
        calls_topic: format!("/{tag}/calls"),
        results_topic: format!("/{tag}/results"),
    };
    let stop = Arc::new(AtomicBool::new(false));
    let (frames_tx, frames) = mpsc::channel();
    let graph = spawn_graph(mgr.clone(), config.clone(), stop.clone(), frames_tx);
    let line = HeldLine::new(needle);
    let _cleanup = Cleanup {
        line: line.clone(),
        stop: stop.clone(),
    };
    let lock = ReceiptLock::acquire(&receipts).expect("lock");
    let latch = Arc::new(AtomicBool::new(false));
    let journal = Journal::spawn(mgr, config, Vec::new(), latch.clone()).expect("journal");
    let host = Arc::new(ToolHost::open(Vec::new(), journal, latch, &receipts, lock).expect("host"));
    if fail_receipt {
        host.receipt_sink().fail_next_sync_for_test();
    }

    let (done, held) = spawn_call(host.clone(), tool, Some(line.clone()));
    let reached = line.wait_reached();
    let frame = if reached && tool == ESTOP_TOOL {
        frames.recv_timeout(OBSERVE).ok()
    } else {
        None
    };
    let outcomes_while_held = outcomes(&receipts);
    let latch_file_while_held = std::fs::read_to_string(latch_path(&receipts)).ok();
    let second = (reached && (tool != ESTOP_TOOL || frame.is_some()))
        .then(|| spawn_call(host.clone(), ESTOP_TOOL, None));
    let second_frame = second
        .as_ref()
        .and_then(|_| frames.recv_timeout(OBSERVE).ok());
    let second_answer = second
        .as_ref()
        .and_then(|(done, _)| done.recv_timeout(OBSERVE).ok());
    line.release();
    let answer = done.recv_timeout(ANSWER).expect("the held call answered");
    held.join().expect("held call thread");
    if let Some((done, handle)) = second {
        if second_answer.is_none() {
            done.recv_timeout(ANSWER).expect("second estop answered");
        }
        handle.join().expect("second estop thread");
    }
    stop.store(true, Ordering::Relaxed);
    graph.join().expect("graph thread");
    cerulion_rcpd::audit::verify_log(&receipts).expect("chain intact");

    let observed = Observed {
        receipts: receipts.clone(),
        reached,
        frame,
        outcomes_while_held,
        latch_file_while_held,
        second_frame,
        second_answer,
        ended_by_release: line.ended_by_release(),
        answer,
        lines: line.lines(),
        outcomes: outcomes(&receipts),
    };
    drop(host);
    drop(dir);
    observed
}

/// The line was held until the release, and a second `estop`, which needs
/// `latch_io` and the receipt log's lock, ran to its answer while it was.
fn assert_second_estop_while_held(o: &Observed) {
    assert!(o.reached, "the held line was never written: {:?}", o.lines);
    assert_eq!(
        o.ended_by_release,
        Some(true),
        "the release, not the bound, ended the hold"
    );
    let Some(second) = o.second_frame.as_ref() else {
        panic!("a second estop's frame did not reach the graph while the line was held");
    };
    let second_answer = o
        .second_answer
        .as_ref()
        .expect("a second estop answered while the line was held");
    assert_eq!(second["tool"], json!(ESTOP_TOOL));
    assert_eq!(second["call_id"], second_answer["call_id"]);
    assert_eq!(second_answer["graph_acknowledged"], json!(true));
}

/// [`assert_second_estop_while_held`], after the held `estop`'s own frame
/// reached the graph while the line was held; it answered once released.
fn assert_frame_while_held(o: &Observed) {
    assert!(o.reached, "the held line was never written: {:?}", o.lines);
    let Some(frame) = o.frame.as_ref() else {
        panic!(
            "the e-stop frame did not reach the graph while the error line was held; lines: {:?}",
            o.lines
        );
    };
    assert_eq!(frame["tool"], json!(ESTOP_TOOL));
    assert_eq!(frame["args"], json!({ "reason": REASON }));
    assert_eq!(frame["call_id"], o.answer["call_id"]);
    assert_second_estop_while_held(o);
    assert_eq!(o.answer["estopped"], json!(true));
    assert_eq!(o.answer["graph_acknowledged"], json!(true), "{}", o.answer);
}

fn latch_line(receipts: &Path) -> String {
    format!(
        "ERROR cerulion_rcpd::tools: latch file not written; the e-stop will not survive a \
         restart error=No such file or directory (os error 2) latch={}\n",
        latch_path(receipts).display()
    )
}

/// The line for a receipt of `verb` whose `sync_data` the seam failed.
fn receipt_line(verb: &str) -> String {
    format!(
        "ERROR cerulion_rcpd::tools: receipt not written verb=\"{verb}\" \
         error=receipt log error: injected sync_data failure (test seam)\n"
    )
}

const ENGAGED_LINE: &str = " WARN cerulion_rcpd::tools: e-stop engaged reason=held\n";

#[test]
fn a_held_latch_file_error_line_does_not_hold_the_estop_frame() {
    let o = held_call(
        "latch",
        ESTOP_TOOL,
        "latch file not written",
        break_latch_file,
        false,
    );
    assert_frame_while_held(&o);
    assert_eq!(o.lines, [latch_line(&o.receipts), ENGAGED_LINE.to_string()]);
    assert!(o.answer["receipt"]["seq"].is_u64(), "{}", o.answer);
    assert_eq!(o.outcomes_while_held, ["estop:intent"]);
    assert_eq!(o.latch_file_while_held, None);
    // The first intent, the second estop's intent and ok, the first's ok.
    let all = ["estop:intent", "estop:intent", "estop:ok", "estop:ok"];
    assert_eq!(o.outcomes, all);
}

#[test]
fn a_held_receipt_error_line_does_not_hold_the_estop_frame() {
    let o = held_call("receipt", ESTOP_TOOL, "receipt not written", |_| {}, true);
    assert_frame_while_held(&o);
    assert_eq!(
        o.lines,
        [receipt_line(ESTOP_TOOL), ENGAGED_LINE.to_string()]
    );
    assert_eq!(
        o.answer["receipt"],
        Value::Null,
        "the intent was not written"
    );
    assert_eq!(o.outcomes_while_held, Vec::<String>::new());
    assert_eq!(o.latch_file_while_held.as_deref(), Some("held\n"));
    // The second estop's intent and ok, the first's ok.
    assert_eq!(o.outcomes, ["estop:intent", "estop:ok", "estop:ok"]);
}

#[test]
fn with_both_writes_failed_the_error_lines_follow_the_frame_in_order() {
    let o = held_call(
        "both",
        ESTOP_TOOL,
        "latch file not written",
        break_latch_file,
        true,
    );
    assert_frame_while_held(&o);
    assert_eq!(
        o.lines,
        [
            latch_line(&o.receipts),
            receipt_line(ESTOP_TOOL),
            ENGAGED_LINE.to_string()
        ]
    );
    assert_eq!(o.outcomes_while_held, Vec::<String>::new());
    assert_eq!(o.latch_file_while_held, None);
    assert_eq!(o.outcomes, ["estop:intent", "estop:ok", "estop:ok"]);
}

#[test]
fn a_held_clear_receipt_error_line_does_not_hold_an_estop_frame() {
    let o = held_call(
        "clear",
        ESTOP_CLEAR_TOOL,
        "receipt not written",
        |_| {},
        true,
    );
    assert_second_estop_while_held(&o);
    assert_eq!(o.lines, [receipt_line(ESTOP_CLEAR_TOOL)]);
    assert_eq!(
        o.answer["error"]["code"],
        json!("receipt_failed"),
        "{}",
        o.answer
    );
    assert_eq!(o.outcomes_while_held, Vec::<String>::new());
    assert_eq!(o.latch_file_while_held, None);
    // The clear's intent was not written; the second estop's intent and ok.
    assert_eq!(o.outcomes, ["estop:intent", "estop:ok"]);
}

#[test]
fn a_held_denied_call_receipt_error_line_does_not_hold_an_estop_frame() {
    let o = held_call(
        "denied",
        "no_such_tool",
        "receipt not written",
        |_| {},
        true,
    );
    assert_second_estop_while_held(&o);
    assert_eq!(o.lines, [receipt_line("no_such_tool")]);
    assert_eq!(
        o.answer["error"]["code"],
        json!("not_in_manifest"),
        "{}",
        o.answer
    );
    assert_eq!(o.outcomes_while_held, Vec::<String>::new());
    // The denied call's receipt was not written; the second estop's intent and ok.
    assert_eq!(o.outcomes, ["estop:intent", "estop:ok"]);
}
