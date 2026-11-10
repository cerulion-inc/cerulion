// SPDX-License-Identifier: AGPL-3.0-only
//! `estop` queues its frame before it writes the error line of a latch file
//! or an intent receipt it could not write, so a log writer that holds that
//! line does not hold the frame the graph is stopped with.
//!
//! Each test calls `estop` on a thread of its own under a scoped subscriber
//! whose writer holds one line, and keeps a second dispatcher alive, so
//! tracing-core computes each callsite's interest over every live dispatcher
//! rather than one thread's default. Transport isolation is per test
//! (`init_for_test` SHM roots, unique topics), so the tests run in parallel.
//! Every wait is bounded.

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
use cerulion_rcpd::tools::{latch_path, ReceiptLock, ToolHost, ESTOP_TOOL};
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};

/// How long a test waits for the held line to be written, and then for the
/// frame to reach the graph: the journal publishes an urgent frame within
/// one loop pass (about 1 ms), and 5 s leaves room for a loaded host.
const OBSERVE: Duration = Duration::from_secs(5);

/// How long the writer holds its line if the test never releases it: longer
/// than both [`OBSERVE`] waits, so the frame is observed while it is held.
const HOLD_BOUND: Duration = Duration::from_secs(30);

/// How long `estop` gets to answer after the release: the graph answers at
/// once, and `estop` waits at most 2 s for it.
const ANSWER: Duration = Duration::from_secs(10);

/// The `reason` every test passes to `estop`.
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
#[cfg(unix)]
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
    ready_rx.recv_timeout(OBSERVE).expect("graph thread ready");
    handle
}

/// Call `estop` on a thread of its own, every line it logs written to
/// `line`. The receiver yields its answer's JSON body.
fn spawn_estop(
    host: Arc<ToolHost>,
    line: HeldLine,
) -> (mpsc::Receiver<Value>, std::thread::JoinHandle<()>) {
    let (done_tx, done_rx) = mpsc::channel();
    let handle = std::thread::spawn(move || {
        let subscriber = tracing_subscriber::fmt()
            .without_time()
            .with_ansi(false)
            .with_writer(move || line.clone())
            .finish();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("runtime");
        let args = json!({ "reason": REASON }).as_object().cloned();
        let result = tracing::subscriber::with_default(subscriber, || {
            runtime.block_on(host.call(ESTOP_TOOL, args))
        });
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

/// What one `estop` under a held line showed.
struct Observed {
    receipts: PathBuf,
    reached: bool,
    /// The frame the graph received while the line was held.
    frame: Option<Value>,
    latched_while_held: bool,
    ended_by_release: Option<bool>,
    answer: Value,
    lines: Vec<String>,
    /// `(verb, outcome)` of every receipt, oldest first.
    outcomes: Vec<(String, String)>,
    /// The latch file's text; `None` when it cannot be read.
    latch_file: Option<String>,
}

/// Open a host on topics private to `tag` (after `prepare` ran on the receipt
/// log's path), arm a failed intent receipt when `fail_intent`, call `estop`
/// with the line containing `needle` held, then release it.
fn estop_with_held_line(
    tag: &str,
    needle: &'static str,
    prepare: impl FnOnce(&Path),
    fail_intent: bool,
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
    let lock = ReceiptLock::acquire(&receipts).expect("lock");
    let latch = Arc::new(AtomicBool::new(false));
    let journal = Journal::spawn(mgr, config, Vec::new(), latch.clone()).expect("journal");
    let host = Arc::new(ToolHost::open(Vec::new(), journal, latch, &receipts, lock).expect("host"));
    if fail_intent {
        host.receipt_sink().fail_next_sync_for_test();
    }

    let line = HeldLine::new(needle);
    let (done, estop) = spawn_estop(host.clone(), line.clone());
    let reached = line.wait_reached();
    let frame = if reached {
        frames.recv_timeout(OBSERVE).ok()
    } else {
        None
    };
    let latched_while_held = host.estopped();
    line.release();
    let answer = done.recv_timeout(ANSWER).expect("estop answered");
    estop.join().expect("estop thread");
    stop.store(true, Ordering::Relaxed);
    graph.join().expect("graph thread");

    cerulion_rcpd::audit::verify_log(&receipts).expect("chain intact");
    let outcomes = cerud::receipt::read_all(&receipts)
        .expect("read receipts")
        .iter()
        .map(|r| {
            let outcome = serde_json::to_value(&r.outcome).expect("outcome");
            let tag = match outcome.get("detail").and_then(Value::as_str) {
                Some(detail) => format!("error:{detail}"),
                None => outcome["outcome"].as_str().unwrap_or("").to_string(),
            };
            (r.verb.clone(), tag)
        })
        .collect();
    let observed = Observed {
        receipts: receipts.clone(),
        reached,
        frame,
        latched_while_held,
        ended_by_release: line.ended_by_release(),
        answer,
        lines: line.lines(),
        outcomes,
        latch_file: std::fs::read_to_string(latch_path(&receipts)).ok(),
    };
    drop(host);
    drop(dir);
    observed
}

/// The asserts every case shares: the line was held, the frame reached the
/// graph while it was, and `estop` answered once it was released.
fn assert_frame_while_held(o: &Observed) {
    assert!(o.reached, "the held line was never written: {:?}", o.lines);
    assert_eq!(
        o.ended_by_release,
        Some(true),
        "the release, not the bound, ended the hold"
    );
    let Some(frame) = o.frame.as_ref() else {
        panic!(
            "the e-stop frame did not reach the graph while the error line was held; lines: {:?}",
            o.lines
        );
    };
    assert_eq!(frame["tool"], json!(ESTOP_TOOL));
    assert_eq!(frame["args"], json!({ "reason": REASON }));
    assert_eq!(frame["call_id"], o.answer["call_id"]);
    assert!(o.latched_while_held, "the latch read closed while held");
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

const RECEIPT_LINE: &str = "ERROR cerulion_rcpd::tools: receipt not written verb=\"estop\" \
                            error=receipt log error: injected sync_data failure (test seam)\n";

const ENGAGED_LINE: &str = " WARN cerulion_rcpd::tools: e-stop engaged reason=held\n";

fn pairs(items: &[(&str, &str)]) -> Vec<(String, String)> {
    items
        .iter()
        .map(|(a, b)| ((*a).to_string(), (*b).to_string()))
        .collect()
}

#[cfg(unix)]
#[test]
fn a_held_latch_file_error_line_does_not_hold_the_estop_frame() {
    let o = estop_with_held_line("latch", "latch file not written", break_latch_file, false);
    assert_frame_while_held(&o);
    assert_eq!(o.lines, [latch_line(&o.receipts), ENGAGED_LINE.to_string()]);
    assert!(o.answer["receipt"]["seq"].is_u64(), "{}", o.answer);
    assert_eq!(
        o.outcomes,
        pairs(&[(ESTOP_TOOL, "intent"), (ESTOP_TOOL, "ok")])
    );
    assert_eq!(o.latch_file, None, "the latch file was not written");
}

#[test]
fn a_held_receipt_error_line_does_not_hold_the_estop_frame() {
    let o = estop_with_held_line("receipt", "receipt not written", |_| {}, true);
    assert_frame_while_held(&o);
    assert_eq!(o.lines, [RECEIPT_LINE, ENGAGED_LINE]);
    assert_eq!(
        o.answer["receipt"],
        Value::Null,
        "the intent was not written"
    );
    assert_eq!(o.outcomes, pairs(&[(ESTOP_TOOL, "ok")]));
    assert_eq!(
        o.latch_file.as_deref(),
        Some("held\n"),
        "the latch file was written"
    );
}

#[cfg(unix)]
#[test]
fn with_both_writes_failed_the_error_lines_follow_the_frame_in_order() {
    let o = estop_with_held_line("both", "latch file not written", break_latch_file, true);
    assert_frame_while_held(&o);
    assert_eq!(
        o.lines,
        [
            latch_line(&o.receipts),
            RECEIPT_LINE.to_string(),
            ENGAGED_LINE.to_string()
        ]
    );
    assert_eq!(o.outcomes, pairs(&[(ESTOP_TOOL, "ok")]));
    assert_eq!(o.latch_file, None, "the latch file was not written");
}
