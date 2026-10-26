// SPDX-License-Identifier: AGPL-3.0-only
//! The tool surface: deny by default, a receipt per call, e-stop always.
//!
//! The tools are exactly the manifest's `tools` plus the built-in
//! [`ESTOP_TOOL`]. A call runs, in order:
//!
//! 1. Lookup. A name the manifest does not declare is refused and receipted
//!    as `denied`.
//! 2. Argument check against the tool's `input_schema` ([`crate::schema`]),
//!    and a size check of the frame it would publish. A failure is receipted
//!    as `error:invalid_args` or `error:args_too_large`.
//! 3. For a mutating tool: while the e-stop latch is closed the call is
//!    receipted as `denied` and never reaches the graph.
//! 4. For a mutating tool: an `intent` receipt, durable on disk, BEFORE the
//!    call is published. The journal checks the latch again immediately
//!    before publishing.
//! 5. The journaled call and the graph's answer ([`crate::journal`]), then an
//!    `ok` or `error:<class>` receipt.
//!
//! Calls other than `estop` are serialized: one is in flight at a time, in the
//! order they were accepted. `estop` takes no lock a call can hold across an
//! await: it closes the latch first, then writes its receipt and publishes on
//! the journal's urgent queue, ahead of anything waiting.
//!
//! The latch is persisted as a file beside the receipt log
//! (`<receipts>.estop`). A daemon that starts and finds the file starts with
//! the latch closed; deleting the file and restarting the daemon is the only
//! way to clear it.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerud::receipt::{args_digest, Receipt, ReceiptLog, ReceiptOutcome};
use rmcp::model::{CallToolResult, ContentBlock, JsonObject, Tool};
use serde_json::{json, Value};

use crate::journal::{Journal, JournalError, MAX_CALL_JSON_BYTES};
use crate::manifest::ToolSpec;
use crate::RcpdError;

/// The built-in emergency stop tool.
pub const ESTOP_TOOL: &str = "estop";

/// The caller every receipt names.
pub const CALLER: &str = "rcp-agent";

/// The verb a receipt records for a refused name that is not a valid tool
/// name (too long, or outside `[A-Za-z0-9_.-]`).
pub const INVALID_NAME_VERB: &str = "<invalid-tool-name>";

/// The longest `reason` an `estop` call carries into the graph, characters.
pub const ESTOP_REASON_MAX_CHARS: usize = 200;

/// How long `estop` waits for the graph to acknowledge.
const ESTOP_TIMEOUT: Duration = Duration::from_secs(2);

/// Whether `name` is a valid tool name: 1 to 128 characters from
/// `[A-Za-z0-9_.-]`.
pub fn valid_tool_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 128
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-'))
}

/// The latch file that keeps the e-stop closed across daemon restarts.
pub fn latch_path(receipts: &Path) -> PathBuf {
    let mut name = receipts.as_os_str().to_owned();
    name.push(".estop");
    PathBuf::from(name)
}

fn lock_path(receipts: &Path) -> PathBuf {
    let mut name = receipts.as_os_str().to_owned();
    name.push(".lock");
    PathBuf::from(name)
}

/// The exclusive lock on a receipt log, held for a daemon's life. Taken
/// before anything else opens, so a second daemon pointed at the same log
/// stops before it touches the transport or the log.
#[derive(Debug)]
pub struct ReceiptLock {
    /// Never read: the lock lives exactly as long as this open file.
    #[allow(dead_code, reason = "held only to keep the file lock")]
    file: File,
}

impl ReceiptLock {
    /// Create the log's directory and take the lock file beside the log.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Receipts`] when another daemon holds the lock or the lock
    /// file cannot be opened or locked.
    pub fn acquire(receipts: &Path) -> Result<Self, RcpdError> {
        let fail = |reason: String| RcpdError::Receipts {
            path: receipts.to_path_buf(),
            reason,
        };
        if let Some(dir) = receipts.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir).map_err(|e| fail(e.to_string()))?;
        }
        let file = File::options()
            .create(true)
            .truncate(false)
            .write(true)
            .open(lock_path(receipts))
            .map_err(|e| fail(format!("cannot open its lock file: {e}")))?;
        match file.try_lock() {
            Ok(()) => Ok(Self { file }),
            Err(std::fs::TryLockError::WouldBlock) => Err(fail(String::from(
                "another cerulion-rcpd holds this receipt log; give each daemon its own --receipts",
            ))),
            Err(std::fs::TryLockError::Error(e)) => {
                Err(fail(format!("cannot lock its lock file: {e}")))
            }
        }
    }
}

/// The tools, the receipt log, the e-stop latch and the journal.
pub struct ToolHost {
    tools: Vec<ToolSpec>,
    journal: Journal,
    latch: Arc<AtomicBool>,
    latch_file: PathBuf,
    receipts: Mutex<ReceiptLog>,
    /// Serializes calls other than `estop`.
    call_lock: tokio::sync::Mutex<()>,
    /// The high bits of every `call_id`: the start time's low 20 bits of
    /// milliseconds, so an answer meant for an earlier daemon does not match a
    /// call of this one unless the two started a multiple of about 17 minutes
    /// apart to the millisecond.
    call_epoch: u64,
    next_call: AtomicU64,
    /// Held for the daemon's life.
    _log_lock: ReceiptLock,
}

impl std::fmt::Debug for ToolHost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ToolHost")
            .field("tools", &self.tools.len())
            .field("estopped", &self.latch.load(Ordering::SeqCst))
            .finish_non_exhaustive()
    }
}

fn receipt_json(receipt: &Receipt) -> Value {
    json!({ "seq": receipt.seq, "hash": receipt.entry_hash, "prev": receipt.prev_hash })
}

fn refusal(code: &str, message: &str, receipt: Option<&Receipt>, extra: Value) -> CallToolResult {
    let mut body = json!({
        "error": { "code": code, "message": message },
        "receipt": receipt.map(receipt_json),
    });
    if let (Some(body), Value::Object(extra)) = (body.as_object_mut(), extra) {
        body.extend(extra);
    }
    CallToolResult::error(vec![ContentBlock::text(body.to_string())])
}

/// Write `text` to `path` and sync it to disk before returning.
fn write_synced(path: &Path, text: &str) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = File::create(path)?;
    file.write_all(text.as_bytes())?;
    file.sync_all()
}

fn journal_class(error: &JournalError) -> &'static str {
    match error {
        JournalError::Timeout { .. } => "timeout",
        JournalError::Publish { .. } => "publish_failed",
        JournalError::Estopped => "estopped",
        JournalError::Stopped => "journal_stopped",
    }
}

fn estop_schema() -> Value {
    json!({
        "type": "object",
        "properties": {
            "reason": {
                "type": "string",
                "maxLength": ESTOP_REASON_MAX_CHARS,
                "description": "Why the robot is being stopped."
            }
        },
        "additionalProperties": false
    })
}

impl ToolHost {
    /// Open the receipt log at `receipts`, whose `lock` the caller holds, read
    /// the persisted e-stop latch, and serve `tools` over `journal`. `latch`
    /// must be the one the journal checks.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Receipts`] when the log cannot be opened.
    pub fn open(
        tools: Vec<ToolSpec>,
        journal: Journal,
        latch: Arc<AtomicBool>,
        receipts: &Path,
        lock: ReceiptLock,
    ) -> Result<Self, RcpdError> {
        let fail = |reason: String| RcpdError::Receipts {
            path: receipts.to_path_buf(),
            reason,
        };
        let log = ReceiptLog::open(receipts).map_err(|e| fail(e.to_string()))?;
        let latch_file = latch_path(receipts);
        if latch_file.exists() {
            latch.store(true, Ordering::SeqCst);
            tracing::error!(
                latch = %latch_file.display(),
                "the e-stop latch file exists: starting with the e-stop engaged; \
                 delete the file and restart to clear it"
            );
        }
        let call_epoch = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| (d.as_millis() & 0xF_FFFF) as u64);
        Ok(Self {
            tools,
            journal,
            latch,
            latch_file,
            receipts: Mutex::new(log),
            call_lock: tokio::sync::Mutex::new(()),
            call_epoch,
            next_call: AtomicU64::new(1),
            _log_lock: lock,
        })
    }

    /// The handle for state reads.
    pub fn journal(&self) -> &Journal {
        &self.journal
    }

    /// Whether the e-stop latch is closed.
    pub fn estopped(&self) -> bool {
        self.latch.load(Ordering::SeqCst)
    }

    /// A fresh `call_id`: the start epoch's 20 bits above a 32-bit counter,
    /// which keeps every id below 2^52, exact as a JSON double.
    fn next_call_id(&self) -> u64 {
        let counter = self.next_call.fetch_add(1, Ordering::Relaxed) & 0xFFFF_FFFF;
        (self.call_epoch << 32) | counter
    }

    fn record(&self, verb: &str, digest: &str, outcome: ReceiptOutcome) -> Option<Receipt> {
        let mut log = self.receipts.lock().unwrap_or_else(PoisonError::into_inner);
        match log.record(CALLER, verb, digest, outcome) {
            Ok(receipt) => Some(receipt),
            Err(e) => {
                tracing::error!(verb, error = %e, "receipt not written");
                None
            }
        }
    }

    /// `tools/list`: the manifest's tools, then `estop`.
    pub fn list(&self) -> Vec<Tool> {
        let mut out: Vec<Tool> = self
            .tools
            .iter()
            .map(|spec| {
                let schema = spec.input_schema.as_object().cloned().unwrap_or_default();
                Tool::new(
                    spec.name.clone(),
                    spec.description.clone(),
                    Arc::new(schema),
                )
            })
            .collect();
        let estop = estop_schema().as_object().cloned().unwrap_or_default();
        out.push(Tool::new(
            ESTOP_TOOL,
            "Stop the robot now. Never waits behind another call; every later motion call is \
             refused until an operator clears the latch.",
            Arc::new(estop),
        ));
        out
    }

    /// `tools/call`.
    pub async fn call(&self, name: &str, args: Option<JsonObject>) -> CallToolResult {
        let args = args.unwrap_or_default();
        if name == ESTOP_TOOL {
            return self.estop(&args).await;
        }
        let args_value = Value::Object(args.clone());
        let digest = args_digest(&args_value);
        let Some(spec) = self.tools.iter().find(|t| t.name == name) else {
            let verb = if valid_tool_name(name) {
                name
            } else {
                INVALID_NAME_VERB
            };
            let receipt = self.record(verb, &digest, ReceiptOutcome::Denied);
            return refusal(
                "not_in_manifest",
                "no such tool: only the manifest's tools and estop exist",
                receipt.as_ref(),
                Value::Null,
            );
        };
        if let Err(reason) = crate::schema::validate(&spec.input_schema, &args) {
            let receipt = self.record(name, &digest, ReceiptOutcome::Error("invalid_args".into()));
            return refusal("invalid_args", &reason, receipt.as_ref(), Value::Null);
        }
        let _serial = self.call_lock.lock().await;
        let call_id = self.next_call_id();
        let frame = Journal::frame(call_id, name, &args_value);
        if frame.len() > MAX_CALL_JSON_BYTES {
            let receipt = self.record(
                name,
                &digest,
                ReceiptOutcome::Error("args_too_large".into()),
            );
            return refusal(
                "args_too_large",
                &format!(
                    "the call serializes to {} bytes; the limit is {MAX_CALL_JSON_BYTES}",
                    frame.len()
                ),
                receipt.as_ref(),
                Value::Null,
            );
        }
        let mutating = spec.safety_class.is_mutating();
        if mutating && self.estopped() {
            let receipt = self.record(name, &digest, ReceiptOutcome::Denied);
            return refusal(
                "estopped",
                "the e-stop is engaged; an operator must delete the latch file and restart \
                 cerulion-rcpd",
                receipt.as_ref(),
                Value::Null,
            );
        }
        // A mutating call's intent is on disk before the call is published.
        let intent = if mutating {
            match self.record(name, &digest, ReceiptOutcome::Intent) {
                Some(receipt) => Some(receipt),
                None => {
                    return refusal(
                        "receipt_failed",
                        "the call was not published: its receipt could not be written",
                        None,
                        Value::Null,
                    );
                }
            }
        } else {
            None
        };
        let timeout = Duration::from_millis(spec.timeout_ms);
        let answer = self.journal.call(call_id, frame, timeout, mutating).await;
        let outcome = match &answer {
            Ok(_) => ReceiptOutcome::Ok,
            Err(e) => ReceiptOutcome::Error(journal_class(e).to_string()),
        };
        let closing = self.record(name, &digest, outcome);
        let receipt = intent.or(closing);
        match answer {
            Ok(result) => {
                let body = json!({
                    "call_id": call_id,
                    "receipt": receipt.as_ref().map(receipt_json),
                    "result": result,
                });
                CallToolResult::success(vec![ContentBlock::text(body.to_string())])
            }
            Err(e) => {
                // A timed-out call was published; the graph may still act on it.
                let extra = match e {
                    JournalError::Timeout { .. } => json!({ "may_have_executed": true }),
                    _ => Value::Null,
                };
                refusal(journal_class(&e), &e.to_string(), receipt.as_ref(), extra)
            }
        }
    }

    async fn estop(&self, args: &JsonObject) -> CallToolResult {
        // Close the latch before anything that can wait or fail.
        let was_estopped = self.latch.swap(true, Ordering::SeqCst);
        let reason: String = args
            .get("reason")
            .and_then(Value::as_str)
            .unwrap_or("")
            .chars()
            .take(ESTOP_REASON_MAX_CHARS)
            .collect();
        tracing::warn!(reason = %reason, "e-stop engaged");
        if !was_estopped {
            if let Err(e) = write_synced(&self.latch_file, &format!("{reason}\n")) {
                tracing::error!(
                    error = %e,
                    latch = %self.latch_file.display(),
                    "latch file not written; the e-stop will not survive a restart"
                );
            }
        }
        let clean = json!({ "reason": reason });
        let digest = args_digest(&clean);
        let intent = self.record(ESTOP_TOOL, &digest, ReceiptOutcome::Intent);
        let call_id = self.next_call_id();
        let frame = Journal::frame(call_id, ESTOP_TOOL, &clean);
        let answer = self.journal.urgent(call_id, frame, ESTOP_TIMEOUT).await;
        let (result, graph_error) = match answer {
            Ok(result) => {
                self.record(ESTOP_TOOL, &digest, ReceiptOutcome::Ok);
                (result, None)
            }
            Err(e) => {
                self.record(
                    ESTOP_TOOL,
                    &digest,
                    ReceiptOutcome::Error(String::from("graph_unacknowledged")),
                );
                (Value::Null, Some(e.to_string()))
            }
        };
        // Not an error either way: the latch is closed and every later motion
        // call is refused, whether or not the graph answered.
        let body = json!({
            "call_id": call_id,
            "estopped": true,
            "graph_acknowledged": graph_error.is_none(),
            "graph_error": graph_error,
            "receipt": intent.as_ref().map(receipt_json),
            "result": result,
        });
        CallToolResult::success(vec![ContentBlock::text(body.to_string())])
    }
}
