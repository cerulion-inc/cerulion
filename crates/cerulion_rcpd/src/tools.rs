// SPDX-License-Identifier: AGPL-3.0-only
//! The tool surface: deny by default, a receipt per call, e-stop always.
//!
//! The tools are exactly the manifest's `tools` plus the built-in
//! [`ESTOP_TOOL`] and, under `limits.estop_clear: agent`, the built-in
//! [`ESTOP_CLEAR_TOOL`]. A call runs, in order:
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
//! the latch closed (so does a file whose presence cannot be read). Under
//! `limits.estop_clear: agent` (the default) `estop_clear` writes an `intent`
//! receipt, deletes the file, opens the latch and journals its own
//! latch-gated frame; deleting the file and restarting the daemon also clears
//! it. Under `operator` `estop_clear` is not listed, a call to it is receipted
//! as `denied`, and deleting the file and restarting the daemon is the only
//! way to clear it.
//!
//! `estop` closes the latch first, then takes `latch_io`, closes it again,
//! writes the file and its `intent` receipt. `estop_clear` writes its `intent`,
//! deletes the file and opens the latch inside `latch_io`. The lock is a std
//! mutex never held across an await, so the receipt log orders engages and
//! clears as they took effect.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerud::receipt::{args_digest, Receipt, ReceiptLog, ReceiptOutcome};
use rmcp::model::{CallToolResult, ContentBlock, JsonObject, Tool, ToolAnnotations};
use serde_json::{json, Value};

use crate::journal::{Journal, JournalError, MAX_CALL_JSON_BYTES};
use crate::manifest::{EstopClear, SafetyClass, ToolSpec};
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

/// The built-in tool that clears the e-stop latch, served under
/// `limits.estop_clear: agent`.
pub const ESTOP_CLEAR_TOOL: &str = "estop_clear";

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
    /// Who clears the latch.
    estop_clear: EstopClear,
    /// Held while the latch and its file change together.
    latch_io: Mutex<()>,
    /// How many times `estop` has been called; a clear refuses when it moved
    /// between the clear's request and the clear taking effect.
    engages: AtomicU64,
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

/// The MCP annotations for a safety class: `read` is `readOnlyHint`,
/// `irreversible` is `destructiveHint`, and every class says it acts on the
/// robot alone (`openWorldHint: false`).
pub fn safety_annotations(class: SafetyClass) -> ToolAnnotations {
    ToolAnnotations::new()
        .read_only(class == SafetyClass::Read)
        .destructive(class == SafetyClass::Irreversible)
        .open_world(false)
}

/// `estop` and `estop_clear`: idempotent, closed world, not destructive; the
/// `estop_clear` listing sets `destructiveHint` on top.
fn builtin_annotations() -> ToolAnnotations {
    ToolAnnotations::new()
        .read_only(false)
        .destructive(false)
        .idempotent(true)
        .open_world(false)
}

/// One manifest tool as `tools/list` serves it: the description ends with
/// its safety class, and the annotations carry it.
fn manifest_tool(spec: &ToolSpec) -> Tool {
    let schema = spec.input_schema.as_object().cloned().unwrap_or_default();
    Tool::new(
        spec.name.clone(),
        format!(
            "{} (safety class: {})",
            spec.description.trim_end(),
            spec.safety_class.as_str()
        ),
        Arc::new(schema),
    )
    .annotate(safety_annotations(spec.safety_class))
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
        // A file whose presence cannot be read counts as present.
        if !matches!(latch_file.try_exists(), Ok(false)) {
            latch.store(true, Ordering::SeqCst);
            tracing::error!(
                latch = %latch_file.display(),
                "the e-stop latch file exists: starting with the e-stop engaged"
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
            estop_clear: EstopClear::Agent,
            latch_io: Mutex::new(()),
            engages: AtomicU64::new(0),
            call_epoch,
            next_call: AtomicU64::new(1),
            _log_lock: lock,
        })
    }

    /// Set who clears the e-stop latch (`limits.estop_clear`). A host built
    /// without this call serves [`EstopClear::Agent`].
    #[must_use]
    pub fn with_estop_clear(mut self, estop_clear: EstopClear) -> Self {
        self.estop_clear = estop_clear;
        self
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

    /// `tools/list`: the manifest's tools, then `estop`, then `estop_clear`
    /// under `limits.estop_clear: agent`.
    pub fn list(&self) -> Vec<Tool> {
        let mut out: Vec<Tool> = self.tools.iter().map(manifest_tool).collect();
        let estop = estop_schema().as_object().cloned().unwrap_or_default();
        out.push(
            Tool::new(
                ESTOP_TOOL,
                match self.estop_clear {
                    EstopClear::Agent => {
                        "Stop the robot now. The latch closes before anything waits, never behind a \
                     call in flight; every later motion call is refused until estop_clear is \
                     called or an operator clears the latch."
                    }
                    EstopClear::Operator => {
                        "Stop the robot now. The latch closes before anything waits, never behind a \
                     call in flight; every later motion call is refused until an operator \
                     clears the latch."
                    }
                },
                Arc::new(estop),
            )
            .annotate(builtin_annotations()),
        );
        if self.estop_clear == EstopClear::Agent {
            out.push(
                Tool::new(
                    ESTOP_CLEAR_TOOL,
                    "Clear the e-stop latch so motion calls run again. Waits behind a call in \
                     flight; the graph is told with its own journaled frame.",
                    Arc::new(estop_schema().as_object().cloned().unwrap_or_default()),
                )
                .annotate(builtin_annotations().destructive(true)),
            );
        }
        out
    }

    /// `tools/call`.
    pub async fn call(&self, name: &str, args: Option<JsonObject>) -> CallToolResult {
        let args = args.unwrap_or_default();
        if name == ESTOP_TOOL {
            return self.estop(&args).await;
        }
        if name == ESTOP_CLEAR_TOOL {
            return self.estop_clear(&args).await;
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
                match self.estop_clear {
                    EstopClear::Agent => {
                        "no such tool: only the manifest's tools, estop and estop_clear exist"
                    }
                    EstopClear::Operator => {
                        "no such tool: only the manifest's tools and estop exist"
                    }
                },
                receipt.as_ref(),
                Value::Null,
            );
        };
        let class = json!({ "safety_class": spec.safety_class.as_str() });
        if let Err(reason) = crate::schema::validate(&spec.input_schema, &args) {
            let receipt = self.record(name, &digest, ReceiptOutcome::Error("invalid_args".into()));
            return refusal("invalid_args", &reason, receipt.as_ref(), class);
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
                class,
            );
        }
        let mutating = spec.safety_class.is_mutating();
        if mutating && self.estopped() {
            let receipt = self.record(name, &digest, ReceiptOutcome::Denied);
            let message = match self.estop_clear {
                EstopClear::Agent => {
                    "the e-stop is engaged; call estop_clear, or delete the latch file and \
                     restart cerulion-rcpd"
                }
                EstopClear::Operator => {
                    "the e-stop is engaged; an operator must delete the latch file and restart \
                     cerulion-rcpd"
                }
            };
            return refusal("estopped", message, receipt.as_ref(), class);
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
                        class,
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
                    "safety_class": spec.safety_class.as_str(),
                });
                CallToolResult::success(vec![ContentBlock::text(body.to_string())])
            }
            Err(e) => {
                // A timed-out call was published; the graph may still act on it.
                let extra = match e {
                    JournalError::Timeout { .. } => json!({
                        "may_have_executed": true,
                        "safety_class": spec.safety_class.as_str(),
                    }),
                    _ => class,
                };
                refusal(journal_class(&e), &e.to_string(), receipt.as_ref(), extra)
            }
        }
    }

    async fn estop(&self, args: &JsonObject) -> CallToolResult {
        // Count the engage, then close the latch, before anything that can
        // fail or wait.
        self.engages.fetch_add(1, Ordering::SeqCst);
        self.latch.store(true, Ordering::SeqCst);
        let reason = reason_arg(args);
        let clean = json!({ "reason": reason });
        let digest = args_digest(&clean);
        let intent = self.engage_latch(&reason, &digest);
        tracing::warn!(reason = %reason, "e-stop engaged");
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

    /// The locked half of `estop`, after its lock-free close: inside
    /// `latch_io`, close the latch again, write the file when it is absent,
    /// and write the `intent` receipt.
    fn engage_latch(&self, reason: &str, digest: &str) -> Option<Receipt> {
        let _io = self.latch_io.lock().unwrap_or_else(PoisonError::into_inner);
        // Closed again under the lock: a clear that opened the latch after
        // the lock-free close has released the lock by now.
        self.latch.store(true, Ordering::SeqCst);
        if !matches!(self.latch_file.try_exists(), Ok(true)) {
            if let Err(e) = write_synced(&self.latch_file, &format!("{reason}\n")) {
                tracing::error!(
                    error = %e,
                    latch = %self.latch_file.display(),
                    "latch file not written; the e-stop will not survive a restart"
                );
            }
        }
        self.record(ESTOP_TOOL, digest, ReceiptOutcome::Intent)
    }

    /// The locked half of `estop_clear`: inside `latch_io`, refuse when an
    /// `estop` was called since `seen` was read from `engages` (receipted
    /// `error:estop_overtaken`, nothing changed); otherwise write the `intent`
    /// receipt, delete the file and open the latch. Returns the intent and
    /// whether the latch was closed, or the refusal to return.
    fn open_latch(&self, digest: &str, seen: u64) -> Result<(Receipt, bool), CallToolResult> {
        let _io = self.latch_io.lock().unwrap_or_else(PoisonError::into_inner);
        if self.engages.load(Ordering::SeqCst) != seen {
            let receipt = self.record(
                ESTOP_CLEAR_TOOL,
                digest,
                ReceiptOutcome::Error(String::from("estop_overtaken")),
            );
            return Err(refusal(
                "estop_overtaken",
                "an estop engaged after the clear was requested; the e-stop is engaged",
                receipt.as_ref(),
                json!({ "estopped": self.estopped() }),
            ));
        }
        let Some(intent) = self.record(ESTOP_CLEAR_TOOL, digest, ReceiptOutcome::Intent) else {
            return Err(refusal(
                "receipt_failed",
                "the e-stop was not cleared: its receipt could not be written",
                None,
                Value::Null,
            ));
        };
        match std::fs::remove_file(&self.latch_file) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                self.record(
                    ESTOP_CLEAR_TOOL,
                    digest,
                    ReceiptOutcome::Error(String::from("clear_failed")),
                );
                return Err(refusal(
                    "clear_failed",
                    &format!(
                        "the e-stop stays engaged: the latch file {} could not be deleted: {e}",
                        self.latch_file.display()
                    ),
                    Some(&intent),
                    json!({ "estopped": self.estopped() }),
                ));
            }
        }
        Ok((intent, self.latch.swap(false, Ordering::SeqCst)))
    }

    /// Journal an `estop_clear` frame, latch-gated: an `estop` that closes the
    /// latch before this frame is published keeps it off the calls topic, so
    /// the graph's last frame is that `estop`.
    async fn journal_clear(&self, call_id: u64, frame: String) -> Result<Value, JournalError> {
        self.journal.call(call_id, frame, ESTOP_TIMEOUT, true).await
    }

    async fn estop_clear(&self, args: &JsonObject) -> CallToolResult {
        let digest = args_digest(&Value::Object(args.clone()));
        if self.estop_clear == EstopClear::Operator {
            let receipt = self.record(ESTOP_CLEAR_TOOL, &digest, ReceiptOutcome::Denied);
            return refusal(
                "estop_clear_disabled",
                "limits.estop_clear is operator: an operator clears the e-stop by deleting the \
                 latch file and restarting cerulion-rcpd",
                receipt.as_ref(),
                Value::Null,
            );
        }
        if let Err(reason) = crate::schema::validate(&estop_schema(), args) {
            let receipt = self.record(
                ESTOP_CLEAR_TOOL,
                &digest,
                ReceiptOutcome::Error("invalid_args".into()),
            );
            return refusal("invalid_args", &reason, receipt.as_ref(), Value::Null);
        }
        let reason = reason_arg(args);
        let clean = json!({ "reason": reason });
        // Read before waiting behind a call in flight: an estop engaged while
        // this clear waits is not cleared by it.
        let seen = self.engages.load(Ordering::SeqCst);
        let _serial = self.call_lock.lock().await;
        let (intent, was_estopped) = match self.open_latch(&digest, seen) {
            Ok(opened) => opened,
            Err(refused) => return refused,
        };
        if !was_estopped {
            // Nothing was latched: the graph is not told.
            self.record(ESTOP_CLEAR_TOOL, &digest, ReceiptOutcome::Ok);
            let body = json!({
                "call_id": Value::Null,
                "estopped": self.estopped(),
                "was_estopped": false,
                "graph_acknowledged": Value::Null,
                "graph_error": Value::Null,
                "receipt": receipt_json(&intent),
                "result": Value::Null,
            });
            return CallToolResult::success(vec![ContentBlock::text(body.to_string())]);
        }
        tracing::warn!(reason = %reason, "e-stop cleared");
        let call_id = self.next_call_id();
        let frame = Journal::frame(call_id, ESTOP_CLEAR_TOOL, &clean);
        let answer = self.journal_clear(call_id, frame).await;
        let (result, graph_error) = match answer {
            Ok(result) => {
                self.record(ESTOP_CLEAR_TOOL, &digest, ReceiptOutcome::Ok);
                (result, None)
            }
            Err(JournalError::Estopped) => {
                self.record(
                    ESTOP_CLEAR_TOOL,
                    &digest,
                    ReceiptOutcome::Error(String::from("estop_overtaken")),
                );
                return refusal(
                    "estop_overtaken",
                    "an estop engaged before the clear reached the graph; the e-stop is engaged",
                    Some(&intent),
                    json!({ "estopped": self.estopped() }),
                );
            }
            Err(e) => {
                self.record(
                    ESTOP_CLEAR_TOOL,
                    &digest,
                    ReceiptOutcome::Error(String::from("graph_unacknowledged")),
                );
                (Value::Null, Some(e.to_string()))
            }
        };
        let body = json!({
            "call_id": call_id,
            "estopped": self.estopped(),
            "was_estopped": true,
            "graph_acknowledged": graph_error.is_none(),
            "graph_error": graph_error,
            "receipt": receipt_json(&intent),
            "result": result,
        });
        CallToolResult::success(vec![ContentBlock::text(body.to_string())])
    }
}

/// The `reason` argument of `estop` and `estop_clear`, cut to
/// [`ESTOP_REASON_MAX_CHARS`] characters; empty when absent.
fn reason_arg(args: &JsonObject) -> String {
    args.get("reason")
        .and_then(Value::as_str)
        .unwrap_or("")
        .chars()
        .take(ESTOP_REASON_MAX_CHARS)
        .collect()
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    use cerulion_core::{TransportConfig, TransportManager};

    use super::{latch_path, ReceiptLock, ToolHost, ESTOP_CLEAR_TOOL};
    use crate::journal::{Journal, JournalError};
    use crate::manifest::JournalConfig;

    /// A host with no tools on topics private to `tag`, with no graph
    /// answering, and its latch.
    fn host(tag: &str, receipts: &Path) -> (ToolHost, Arc<AtomicBool>) {
        let tag = format!("rcpd_latch_unit_{tag}_{}", std::process::id());
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: tag.clone(),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("transport");
        let journal_config = JournalConfig {
            calls_topic: format!("/{tag}/calls"),
            results_topic: format!("/{tag}/results"),
        };
        let latch = Arc::new(AtomicBool::new(false));
        let lock = ReceiptLock::acquire(receipts).expect("lock");
        let journal =
            Journal::spawn(mgr, journal_config, Vec::new(), latch.clone()).expect("journal");
        let host =
            ToolHost::open(Vec::new(), journal, latch.clone(), receipts, lock).expect("host");
        (host, latch)
    }

    #[test]
    fn a_clear_between_the_lock_free_close_and_the_lock_does_not_open_an_engage() {
        let dir = tempfile::tempdir().expect("tempdir");
        let receipts = dir.path().join("log.jsonl");
        let (host, latch) = host("reclose", &receipts);
        // `estop` has done its lock-free close; a clear takes the lock first.
        latch.store(true, Ordering::SeqCst);
        let seen = host.engages.load(Ordering::SeqCst);
        let (_, was) = host.open_latch("d", seen).expect("clear");
        assert!(was);
        assert!(!host.estopped());
        // Then `estop` takes the lock.
        host.engage_latch("late", "d").expect("intent");
        assert!(
            host.estopped(),
            "the engage closes the latch again under the lock"
        );
        assert!(latch_path(&receipts).exists(), "and writes its file");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_clear_frame_is_not_published_once_an_estop_closed_the_latch() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (host, latch) = host("gated", &dir.path().join("log.jsonl"));
        latch.store(true, Ordering::SeqCst);
        let frame = Journal::frame(1, ESTOP_CLEAR_TOOL, &serde_json::json!({}));
        assert_eq!(
            host.journal_clear(1, frame).await,
            Err(JournalError::Estopped)
        );
    }
}
