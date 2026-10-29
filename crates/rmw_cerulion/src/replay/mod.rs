// SPDX-License-Identifier: AGPL-3.0-only
//! Exact replay of one ROS 2 process: the delivery log and the replay gate.
//!
//! `CERULION_RMW_DELIVERY_LOG=<dir>` RECORDS what this process consumed, in
//! the order it consumed it, into `<dir>/<exe>-<pid>.rdl.mcap`: every take
//! (plain, loaned, sequence and adopted subscription takes, service requests
//! and client responses, with the payload body), every `rmw_wait` return (the
//! ready entities, the return code, the timeout rcl passed), every graph-query
//! answer, and a 64-bit FNV-1a hash of every output.
//!
//! `CERULION_RMW_REPLAY=<file>` REPLAYS one recorded process against its log:
//! each `rmw_wait` reports the recorded readiness (see [`gate`]) without
//! probing the transport or waiting for wall time, each take is served from
//! the log and decoded by the same bridge, and each output's hash is compared
//! segment by segment with the recording (see [`oracle`]). The result is
//! written to `<file>.verdict.json`.
//!
//! Setting both variables is refused. A multi-threaded executor is refused by
//! name ([`detect`]). With neither variable set every hook costs one
//! `OnceLock` load: [`mode`] is initialized at the first rmw call and every
//! hook site branches on [`Mode::Off`] before doing anything else.
//!
//! The user page is `docs/ros2_replay.md`; the internals are the "Delivery
//! log and replay gate" section of `docs/internals/rmw.md`.

pub(crate) mod detect;
pub(crate) mod gate;
pub(crate) mod log;
pub(crate) mod oracle;

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Mutex, Once, OnceLock};
use std::time::{Duration, Instant};

use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_TIMEOUT};
use crate::runtime::GuardConditionState;

use detect::Detector;
use gate::{Gate, GraphAnswers, Schedule, TakeQueues, Turn};
use log::{
    EntityId, EntityKind, EntityRec, GraphQuery, GraphRec, LoadedLog, LogEnding, LogWriter,
    OutputKind, OutputRec, ProcessHeader, Record, TakeKind, TakeRec, WaitRec, LOG_SUFFIX,
    RECORD_VERSION,
};
use oracle::{EntityNames, Oracle, Out, Verdict, WatchdogReport, UNKNOWN_ENTITY};

/// Record this process's deliveries into a log in this directory.
pub const DELIVERY_LOG_ENV: &str = "CERULION_RMW_DELIVERY_LOG";
/// Replay this process against this delivery log.
pub const REPLAY_ENV: &str = "CERULION_RMW_REPLAY";
/// The verdict file's suffix, appended to the log path.
pub const VERDICT_SUFFIX: &str = ".verdict.json";

/// How long a replayed `rmw_wait` with nothing left to serve sleeps before it
/// returns `RMW_RET_TIMEOUT`, so a spinning executor does not burn a core.
const IDLE_AFTER_LOG: Duration = Duration::from_millis(20);
/// How long a served window waits for the entities the recording created
/// before it (see `Player::creation_fence`). Below the watchdog's 5 s, so a
/// fence never reads as a stalled token.
const CREATION_FENCE: Duration = Duration::from_secs(2);
/// How long a recording thread waits for the clock section (see
/// `Recorder::clock_section`) before it goes on without it.
const CLOCK_SECTION_WAIT: Duration = Duration::from_millis(5);

/// What this process does with its deliveries.
pub(crate) enum Mode {
    /// Neither variable set: every hook is one branch.
    Off,
    Record(Box<Recorder>),
    Replay(Box<Player>),
    /// A requested record or replay that cannot run: `rmw_init` refuses with
    /// this message.
    Refused(String),
}

static MODE: OnceLock<Mode> = OnceLock::new();

/// Set once, when the mode resolves to record or replay. The capture points
/// inside the live take and send paths read this relaxed atomic before they
/// touch any thread-local, so with recording and replay off they cost one
/// plain load.
static HOOKS_ON: AtomicBool = AtomicBool::new(false);

/// The process's mode, resolved from the environment on first use.
#[inline]
pub(crate) fn mode() -> &'static Mode {
    MODE.get_or_init(Mode::from_env)
}

/// Whether any hook has work to do.
#[inline]
pub(crate) fn active() -> bool {
    !matches!(mode(), Mode::Off)
}

/// What the two variables ask for. Pure, so the refusal is testable.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Requested {
    Off,
    Record(PathBuf),
    Replay(PathBuf),
}

/// Resolve the two variables (an empty value counts as unset).
pub(crate) fn requested(
    record: Option<OsString>,
    replay: Option<OsString>,
) -> Result<Requested, String> {
    let record = record.filter(|v| !v.is_empty());
    let replay = replay.filter(|v| !v.is_empty());
    match (record, replay) {
        (None, None) => Ok(Requested::Off),
        (Some(dir), None) => Ok(Requested::Record(PathBuf::from(dir))),
        (None, Some(file)) => Ok(Requested::Replay(PathBuf::from(file))),
        (Some(_), Some(_)) => Err(format!(
            "rmw_cerulion: {DELIVERY_LOG_ENV} and {REPLAY_ENV} are both set; a process either \
             records its deliveries or replays a log, never both. Unset one."
        )),
    }
}

impl Mode {
    fn from_env() -> Mode {
        let req = requested(
            std::env::var_os(DELIVERY_LOG_ENV),
            std::env::var_os(REPLAY_ENV),
        );
        let req = match req {
            Ok(Requested::Off) => return Mode::Off,
            Ok(r) => r,
            Err(e) => {
                crate::runtime::install_tracing();
                return Mode::Refused(e);
            }
        };
        // Errors from here on must reach a foreign host's stderr.
        crate::runtime::install_tracing();
        HOOKS_ON.store(true, Ordering::Release);
        match req {
            Requested::Off => Mode::Off,
            Requested::Record(dir) => match Recorder::start(&dir) {
                Ok(r) => Mode::Record(Box::new(r)),
                Err(e) => Mode::Refused(e),
            },
            Requested::Replay(file) => match Player::load(&file) {
                Ok(p) => Mode::Replay(Box::new(p)),
                Err(e) => {
                    write_verdict_file(&file, &refused_verdict(&file, &e));
                    Mode::Refused(e)
                }
            },
        }
    }
}

// ===========================================================================
// Threads
// ===========================================================================

static NEXT_THREAD: AtomicU32 = AtomicU32::new(0);

thread_local! {
    static THREAD: Cell<u32> = const { Cell::new(u32::MAX) };
    static WAITED: Cell<bool> = const { Cell::new(false) };
    static TAKE_ARMED: Cell<bool> = const { Cell::new(false) };
    static TAKE_CAPTURE: RefCell<TakeCapture> = const {
        RefCell::new(TakeCapture { have: false, seq: 0, source_ts: 0, body: Vec::new() })
    };
    static OUTPUT_ARMED: Cell<bool> = const { Cell::new(false) };
    static OUTPUT_CAPTURE: Cell<Option<(u64, u32)>> = const { Cell::new(None) };
    /// Recording: this thread took `/clock` and has not re-entered `rmw_wait`.
    static CLOCK_HELD: Cell<bool> = const { Cell::new(false) };
    /// Recording: this thread's `rmw_wait` returned and it has made no rmw
    /// call since (rcl checks its timers in that span).
    static POST_WAIT: Cell<bool> = const { Cell::new(false) };
}

/// Whether this thread holds the Python GIL (`PyGILState_Check`, looked up
/// once in the process; `false` in a process without Python).
#[cfg(any(target_os = "linux", target_os = "android"))]
fn holds_python_gil() -> bool {
    static CHECK: OnceLock<Option<unsafe extern "C" fn() -> libc::c_int>> = OnceLock::new();
    let f = CHECK.get_or_init(|| {
        // SAFETY: dlsym with a NUL-terminated name; a non-null result is the
        // C function `int PyGILState_Check(void)`.
        let p = unsafe { libc::dlsym(libc::RTLD_DEFAULT, c"PyGILState_Check".as_ptr()) };
        (!p.is_null()).then(|| unsafe {
            std::mem::transmute::<*mut libc::c_void, unsafe extern "C" fn() -> libc::c_int>(p)
        })
    });
    // SAFETY: PyGILState_Check may be called from any thread at any time.
    f.is_some_and(|f| unsafe { f() } == 1)
}

/// No `libc` here (a Linux-only dependency); replay runs on Linux only.
#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn holds_python_gil() -> bool {
    false
}

/// This thread's index: the order in which threads first reached a hook.
fn thread_ix() -> u32 {
    THREAD.with(|t| {
        let v = t.get();
        if v != u32::MAX {
            return v;
        }
        let n = NEXT_THREAD.fetch_add(1, Ordering::Relaxed);
        t.set(n);
        n
    })
}

/// Arms one capture flag for the life of the guard, so a live call that
/// unwinds (into its entry point's `ffi_guard`) never leaves it armed.
struct Armed(&'static std::thread::LocalKey<Cell<bool>>);

impl Armed {
    fn new(flag: &'static std::thread::LocalKey<Cell<bool>>) -> Armed {
        flag.with(|a| a.set(true));
        Armed(flag)
    }
}

impl Drop for Armed {
    fn drop(&mut self) {
        self.0.with(|a| a.set(false));
    }
}

struct TakeCapture {
    have: bool,
    seq: i64,
    source_ts: i64,
    body: Vec<u8>,
}

/// Called by every take path at the point it has decoded a payload body:
/// copies the body for the delivery log while a recorded take is in flight on
/// this thread. One thread-local flag read otherwise.
#[inline]
pub(crate) fn capture_take_body(body: &[u8], seq: i64, source_ts: i64) {
    if !HOOKS_ON.load(Ordering::Relaxed) || !TAKE_ARMED.with(Cell::get) {
        return;
    }
    TAKE_CAPTURE.with(|c| {
        let mut c = c.borrow_mut();
        c.have = true;
        c.seq = seq;
        c.source_ts = source_ts;
        c.body.clear();
        c.body.extend_from_slice(body);
    });
}

/// Whether an output's body hash is wanted on this thread right now (a
/// recorded or replayed output is in flight). One thread-local flag read.
#[inline]
pub(crate) fn output_capture_armed() -> bool {
    HOOKS_ON.load(Ordering::Relaxed) && OUTPUT_ARMED.with(Cell::get)
}

/// Called by every output path with the payload body it is about to send:
/// hashes it while a recorded or replayed output is in flight on this thread.
/// One thread-local flag read otherwise.
#[inline]
pub(crate) fn capture_output_body(body: &[u8]) {
    if !output_capture_armed() {
        return;
    }
    OUTPUT_CAPTURE.with(|c| c.set(Some((log::fnv1a(body), body.len() as u32))));
}

// ===========================================================================
// Entities
// ===========================================================================

#[derive(Clone, Debug)]
struct Ent {
    id: EntityId,
    /// Replay: the recorded entity of the same stable name.
    rec: Option<EntityId>,
    name: String,
    topic: String,
    node: String,
}

#[derive(Default)]
struct Registry {
    by_key: HashMap<usize, Ent>,
    per_name: HashMap<String, u32>,
    next_id: EntityId,
    guards: u32,
    nodes: Vec<String>,
    /// Replay: every recorded entity a live entity has been bound to (kept
    /// after the live entity is destroyed).
    bound: HashSet<EntityId>,
}

impl Registry {
    /// Name and register a new entity; returns it.
    fn add(&mut self, kind: EntityKind, key: usize, topic: &str, node: &str) -> Ent {
        let name = if kind == EntityKind::Guard {
            let n = self.guards;
            self.guards += 1;
            format!("gc#{n}")
        } else {
            let base = format!("{}:{}", kind.prefix(), topic);
            let c = self.per_name.entry(base.clone()).or_insert(0);
            let full = format!("{base}#{c}");
            *c += 1;
            full
        };
        let ent = Ent {
            id: self.next_id,
            rec: None,
            name,
            topic: topic.to_string(),
            node: node.to_string(),
        };
        self.next_id += 1;
        self.by_key.insert(key, ent.clone());
        ent
    }
}

/// The four entity arrays of one `rmw_wait` call.
#[derive(Clone, Copy)]
pub(crate) struct WaitArrays {
    pub subscriptions: *mut ffi::rmw_subscriptions_t,
    pub guard_conditions: *mut ffi::rmw_guard_conditions_t,
    pub services: *mut ffi::rmw_services_t,
    pub clients: *mut ffi::rmw_clients_t,
}

impl WaitArrays {
    /// Visit every slot (subscriptions, guards, services, clients, in array
    /// order) with its current handle.
    ///
    /// # Safety
    /// The rmw_wait contract: each array is valid for its count.
    unsafe fn for_each_slot(&self, mut f: impl FnMut(EntityKind, *mut *mut std::ffi::c_void)) {
        if !self.subscriptions.is_null() {
            let a = &*self.subscriptions;
            for i in 0..a.subscriber_count {
                f(EntityKind::Subscription, a.subscribers.add(i));
            }
        }
        if !self.guard_conditions.is_null() {
            let a = &*self.guard_conditions;
            for i in 0..a.guard_condition_count {
                f(EntityKind::Guard, a.guard_conditions.add(i));
            }
        }
        if !self.services.is_null() {
            let a = &*self.services;
            for i in 0..a.service_count {
                f(EntityKind::Service, a.services.add(i));
            }
        }
        if !self.clients.is_null() {
            let a = &*self.clients;
            for i in 0..a.client_count {
                f(EntityKind::Client, a.clients.add(i));
            }
        }
    }

    /// The registered entities in the arrays' non-null slots.
    unsafe fn entities(&self, reg: &Registry) -> Vec<Ent> {
        let mut v = Vec::new();
        self.for_each_slot(|_, slot| {
            let h = *slot;
            if !h.is_null() {
                if let Some(e) = reg.by_key.get(&(h as usize)) {
                    v.push(e.clone());
                }
            }
        });
        v
    }

    unsafe fn null_all(&self) {
        self.for_each_slot(|_, slot| *slot = std::ptr::null_mut());
    }
}

/// Null every event slot (this implementation produces no events).
unsafe fn null_events(events: *mut ffi::rmw_events_t) {
    if !events.is_null() {
        let evs = &mut *events;
        for i in 0..evs.event_count {
            *evs.events.add(i) = std::ptr::null_mut();
        }
    }
}

// ===========================================================================
// Process identity
// ===========================================================================

fn live_cmdline() -> Vec<String> {
    #[cfg(target_os = "linux")]
    if let Ok(b) = std::fs::read("/proc/self/cmdline") {
        return b
            .split(|&c| c == 0)
            .filter(|s| !s.is_empty())
            .map(|s| String::from_utf8_lossy(s).into_owned())
            .collect();
    }
    std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect()
}

/// FNV-1a 64 of the file this code was loaded from, hex.
fn library_digest() -> String {
    #[repr(C)]
    struct DlInfo {
        dli_fname: *const std::os::raw::c_char,
        dli_fbase: *mut std::ffi::c_void,
        dli_sname: *const std::os::raw::c_char,
        dli_saddr: *mut std::ffi::c_void,
    }
    extern "C" {
        fn dladdr(addr: *const std::ffi::c_void, info: *mut DlInfo) -> std::os::raw::c_int;
    }
    let mut info = DlInfo {
        dli_fname: std::ptr::null(),
        dli_fbase: std::ptr::null_mut(),
        dli_sname: std::ptr::null(),
        dli_saddr: std::ptr::null_mut(),
    };
    // SAFETY: dladdr fills `info` for an address inside a mapped object; the
    // address is this function's own.
    let ok = unsafe { dladdr(library_digest as *const std::ffi::c_void, &mut info) };
    if ok == 0 || info.dli_fname.is_null() {
        return "unknown".to_string();
    }
    // SAFETY: dladdr returned a NUL-terminated path owned by the loader.
    let path = unsafe { std::ffi::CStr::from_ptr(info.dli_fname) };
    let Ok(path) = path.to_str() else {
        return "unknown".to_string();
    };
    match std::fs::read(path) {
        Ok(bytes) => format!("{:016x}", log::fnv1a(&bytes)),
        Err(_) => "unknown".to_string(),
    }
}

fn live_header() -> ProcessHeader {
    ProcessHeader {
        format: RECORD_VERSION,
        cmdline: live_cmdline(),
        ros_distro: std::env::var("ROS_DISTRO").unwrap_or_default(),
        built_for: crate::era::built_for().to_string(),
        library_digest: library_digest(),
        pid: std::process::id(),
    }
}

/// Refuse a replay whose recorded process is not this one. Pure.
pub(crate) fn check_identity(recorded: &ProcessHeader, live: &ProcessHeader) -> Result<(), String> {
    let mut diffs = Vec::new();
    if recorded.cmdline != live.cmdline {
        diffs.push(format!(
            "command line: recorded `{}`, this process `{}`",
            recorded.cmdline.join(" "),
            live.cmdline.join(" ")
        ));
    }
    if recorded.ros_distro != live.ros_distro {
        diffs.push(format!(
            "ROS_DISTRO: recorded '{}', this process '{}'",
            recorded.ros_distro, live.ros_distro
        ));
    }
    if recorded.built_for != live.built_for {
        diffs.push(format!(
            "rmw_cerulion built for: recorded '{}', this library '{}'",
            recorded.built_for, live.built_for
        ));
    }
    if recorded.library_digest != live.library_digest {
        diffs.push(format!(
            "rmw_cerulion library digest: recorded {}, this library {}",
            recorded.library_digest, live.library_digest
        ));
    }
    if diffs.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "rmw_cerulion replay refused: the log was recorded by a different process ({})",
            diffs.join("; ")
        ))
    }
}

/// A short label for a process in messages: its command line.
fn process_label(cmdline: &[String]) -> String {
    let mut s = cmdline.join(" ");
    if s.len() > 200 {
        let mut cut = 200;
        while !s.is_char_boundary(cut) {
            cut -= 1;
        }
        s.truncate(cut);
        s.push('…');
    }
    s
}

/// Why a process is not replayable, as the refusal names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NotReplayable {
    /// A multi-threaded executor (see [`detect`]).
    MultiThreaded,
    /// A serialized take, which bypasses the bridge the log records at.
    SerializedTake,
    /// A thread that never calls `rmw_wait` publishes after the process's
    /// first wait (see `oracle.rs`).
    UnorderedThread,
}

impl NotReplayable {
    fn cause(self) -> &'static str {
        match self {
            NotReplayable::MultiThreaded => "MultiThreadedExecutor",
            NotReplayable::SerializedTake => "serialized take",
            NotReplayable::UnorderedThread => "publishing thread outside every executor",
        }
    }
}

/// The refusal message of a process that is not replayable, naming the
/// cause, the process and its nodes.
fn refusal_message(
    cause: NotReplayable,
    cmdline: &[String],
    nodes: &[String],
    detail: &str,
) -> String {
    format!(
        "ros2 process {} (nodes {}): {}, not replayable ({detail})",
        process_label(cmdline),
        if nodes.is_empty() {
            "none".to_string()
        } else {
            nodes.join(", ")
        },
        cause.cause()
    )
}

/// The log file stem: the executable's name, or the script's for an
/// interpreter (`python3 mixer.py` logs as `mixer`).
fn log_stem(cmdline: &[String]) -> String {
    let base = |s: &str| {
        Path::new(s)
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default()
    };
    let mut stem = cmdline.first().map(|s| base(s)).unwrap_or_default();
    if stem.starts_with("python") {
        if let Some(script) = cmdline.get(1).filter(|a| !a.starts_with('-')) {
            stem = base(script);
            if let Some(s) = stem.strip_suffix(".py") {
                stem = s.to_string();
            }
        }
    }
    let clean: String = stem
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == '.' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if clean.is_empty() {
        "process".to_string()
    } else {
        clean
    }
}

// ===========================================================================
// atexit
// ===========================================================================

static AT_EXIT: Once = Once::new();
/// Set when the atexit handler runs: thread-locals, the tracing dispatcher's
/// among them, may already be destroyed, so [`note`] writes to fd 2 itself.
static EXITING: AtomicBool = AtomicBool::new(false);

/// Log one line: through `tracing`, or straight to stderr at process exit
/// (a `tracing` call there panics on its destroyed thread-local).
fn note(level: tracing::Level, line: &str) {
    if !EXITING.load(Ordering::Relaxed) {
        match level {
            tracing::Level::ERROR => tracing::error!("{line}"),
            tracing::Level::WARN => tracing::warn!("{line}"),
            _ => tracing::info!("{line}"),
        }
        return;
    }
    #[cfg(unix)]
    {
        use std::io::Write;
        use std::os::fd::FromRawFd;
        // SAFETY: fd 2 is the process's stderr; ManuallyDrop never closes it.
        let mut err = std::mem::ManuallyDrop::new(unsafe { std::fs::File::from_raw_fd(2) });
        let _ = err.write_all(format!("{level} {line}\n").as_bytes());
    }
}

extern "C" fn at_exit() {
    EXITING.store(true, Ordering::Relaxed);
    let _ = std::panic::catch_unwind(|| {
        if let Some(m) = MODE.get() {
            match m {
                Mode::Record(r) => r.finish(),
                Mode::Replay(p) => p.finish(),
                _ => {}
            }
        }
    });
}

fn register_at_exit() {
    extern "C" {
        fn atexit(cb: extern "C" fn()) -> std::os::raw::c_int;
    }
    AT_EXIT.call_once(|| {
        // SAFETY: registers a plain `extern "C"` function with libc.
        if unsafe { atexit(at_exit) } != 0 {
            tracing::error!(
                "rmw_cerulion delivery log: atexit registration failed; the log is closed only \
                 at rmw_shutdown flushes and reads as incomplete"
            );
        }
    });
}

// ===========================================================================
// Hooks called by the rmw entry points
// ===========================================================================

/// `rmw_init`: refuse a requested record or replay that cannot run.
pub(crate) fn on_init() -> Result<(), String> {
    match mode() {
        Mode::Off => Ok(()),
        Mode::Refused(e) => Err(e.clone()),
        Mode::Record(_) | Mode::Replay(_) => {
            register_at_exit();
            Ok(())
        }
    }
}

/// `rmw_shutdown`: flush what is recorded; write the replay verdict so far.
pub(crate) fn on_shutdown() {
    match mode() {
        Mode::Record(r) => r.flush(),
        Mode::Replay(p) => p.write_verdict_if_unwritten(),
        _ => {}
    }
}

/// `rmw_create_node`: record the node; on replay, refuse a node the recording
/// did not create at this point.
pub(crate) fn on_node(name: &str, namespace: &str) -> Result<(), String> {
    match mode() {
        Mode::Record(r) => {
            r.node(name, namespace);
            Ok(())
        }
        Mode::Replay(p) => p.node(name, namespace),
        _ => Ok(()),
    }
}

/// An entity was created. `key` is its `data` pointer (what `rmw_wait`'s
/// arrays carry).
pub(crate) fn on_entity(kind: EntityKind, key: usize, topic: &str, node: &str) {
    match mode() {
        Mode::Record(r) => r.entity(kind, key, topic, node),
        Mode::Replay(p) => p.entity(kind, key, topic, node),
        _ => {}
    }
}

/// An entity is being destroyed.
pub(crate) fn on_entity_destroyed(key: usize) {
    let reg = match mode() {
        Mode::Record(r) => &r.registry,
        Mode::Replay(p) => &p.registry,
        _ => return,
    };
    reg.lock()
        .unwrap_or_else(|e| e.into_inner())
        .by_key
        .remove(&key);
}

/// `rmw_wait` with recording or replay on.
///
/// # Safety
/// The rmw_wait contract (see `rmw_wait`).
pub(crate) unsafe fn wait(
    arrays: WaitArrays,
    events: *mut ffi::rmw_events_t,
    owner: &AtomicU32,
    timeout: *const ffi::rmw_time_t,
    live: impl FnOnce() -> rmw_ret_t,
) -> rmw_ret_t {
    match mode() {
        Mode::Record(r) => r.wait(arrays, owner, timeout, live),
        Mode::Replay(p) => p.wait(arrays, events, owner),
        _ => live(),
    }
}

/// What a take does under recording or replay.
pub(crate) enum TakeAction<'a> {
    /// Take live.
    Live,
    /// Take live and record it.
    Record(&'a Recorder),
    /// Serve this recorded take (`None`: nothing to serve, report not taken).
    Serve(Option<TakeRec>),
    /// Refused (a multi-threaded executor detected during replay).
    Refuse,
}

/// Decide a take of `kind` on the entity `key`.
pub(crate) fn take_action(key: usize, kind: TakeKind) -> TakeAction<'static> {
    match mode() {
        Mode::Record(r) => TakeAction::Record(r),
        Mode::Replay(p) => match p.serve_take(key, kind) {
            Ok(rec) => TakeAction::Serve(rec),
            Err(()) => TakeAction::Refuse,
        },
        _ => TakeAction::Live,
    }
}

/// The metadata of a take the live path produced.
pub(crate) struct TakeMeta {
    pub taken: bool,
    pub received_ts: i64,
    /// Requests and responses: the request id the live take reported.
    pub request: Option<(i64, [u8; 16], i64)>,
}

impl Recorder {
    /// Run the live take with body capture armed, then record it.
    pub(crate) fn take(
        &self,
        key: usize,
        kind: TakeKind,
        live: impl FnOnce() -> rmw_ret_t,
        meta: impl FnOnce(rmw_ret_t) -> TakeMeta,
    ) -> rmw_ret_t {
        TAKE_CAPTURE.with(|c| c.borrow_mut().have = false);
        self.clock_leave(false);
        if !CLOCK_HELD.with(Cell::get) && self.is_clock_sub(key) {
            self.clock_section(true);
        }
        let ret = {
            let _armed = Armed::new(&TAKE_ARMED);
            live()
        };
        let m = meta(ret);
        let thread = thread_ix();
        if let Some(d) = self.detector.on_take(thread, WAITED.with(Cell::get)) {
            self.mark_not_replayable(NotReplayable::MultiThreaded, &d);
        }
        let entity = self.entity_id(key);
        let (seq, source_ts, body) = TAKE_CAPTURE.with(|c| {
            let mut c = c.borrow_mut();
            if m.taken && c.have {
                (c.seq, c.source_ts, std::mem::take(&mut c.body))
            } else {
                (0, 0, Vec::new())
            }
        });
        let (seq, guid, source_ts) = match m.request {
            Some((s, g, ts)) => (s, g, ts),
            None => (seq, [0u8; 16], source_ts),
        };
        let rec = TakeRec {
            k: 0,
            thread,
            entity,
            kind,
            ret,
            taken: m.taken,
            seq,
            source_ts,
            received_ts: m.received_ts,
            guid,
            body,
        };
        self.append(Record::Take(rec));
        ret
    }
}

/// An output under recording or replay: `live` runs the real send with the
/// body hash captured.
pub(crate) fn output(key: usize, kind: OutputKind, live: impl FnOnce() -> rmw_ret_t) -> rmw_ret_t {
    let m = mode();
    if matches!(m, Mode::Off | Mode::Refused(_)) {
        return live();
    }
    OUTPUT_CAPTURE.with(|c| c.set(None));
    if let Mode::Record(r) = m {
        r.clock_leave(false);
    }
    let ret = {
        let _armed = Armed::new(&OUTPUT_ARMED);
        live()
    };
    let captured = OUTPUT_CAPTURE.with(Cell::take);
    let (has_payload, hash, len) = match captured {
        Some((h, l)) => (true, h, l),
        None => (false, 0, 0),
    };
    match m {
        Mode::Record(r) => {
            let rec = OutputRec {
                k: 0,
                thread: thread_ix(),
                entity: r.entity_id(key),
                kind,
                ret,
                has_payload,
                len,
                hash,
            };
            r.append(Record::Output(rec));
        }
        Mode::Replay(p) => p.output(
            key,
            Out {
                entity: UNKNOWN_ENTITY,
                hash,
                len,
                has_payload,
                ret,
            },
        ),
        _ => {}
    }
    ret
}

/// A graph query under recording or replay: `live` computes the live answer.
pub(crate) fn graph(
    query: GraphQuery,
    name: &str,
    live: impl FnOnce() -> (rmw_ret_t, u64),
) -> (rmw_ret_t, u64) {
    match mode() {
        Mode::Record(r) => {
            let (ret, answer) = live();
            r.append(Record::Graph(GraphRec {
                k: 0,
                thread: thread_ix(),
                query,
                name: name.to_string(),
                ret,
                answer,
            }));
            (ret, answer)
        }
        Mode::Replay(p) => p.graph(query, name, live),
        _ => live(),
    }
}

/// Whether a subscription created now may offer loaned takes. Replay serves
/// takes from the log by copy, so it turns loans off (rclcpp then uses the
/// copying take, with the same message delivered).
pub(crate) fn subscription_loans_allowed() -> bool {
    !matches!(mode(), Mode::Replay(_))
}

/// A serialized take bypasses the bridge, so the delivery log cannot record
/// what it delivered: recording marks the log not replayable.
pub(crate) fn report_serialized_take(topic: &str) {
    if let Mode::Record(r) = mode() {
        r.mark_not_replayable(
            NotReplayable::SerializedTake,
            &format!("on {topic}; the delivery log records decoded takes only"),
        );
    }
}

// ===========================================================================
// Recorder
// ===========================================================================

/// The record side.
pub(crate) struct Recorder {
    header: ProcessHeader,
    writer: Mutex<LogWriter>,
    registry: Mutex<Registry>,
    detector: Detector,
    not_replayable: AtomicBool,
    /// The clock section: (threads holding a `/clock` take, threads inside a
    /// post-wait span). See [`Recorder::clock_section`].
    clock: Mutex<(u32, u32)>,
    clock_cv: std::sync::Condvar,
    clock_waits_timed_out: std::sync::atomic::AtomicU64,
    /// Clock-section entries that found the other side busy and waited, and
    /// the nanoseconds they waited in all (logged when the log closes).
    clock_waits: std::sync::atomic::AtomicU64,
    clock_wait_ns: std::sync::atomic::AtomicU64,
}

impl Recorder {
    fn start(dir: &Path) -> Result<Recorder, String> {
        std::fs::create_dir_all(dir).map_err(|e| {
            format!(
                "rmw_cerulion: {DELIVERY_LOG_ENV}={}: cannot create the directory: {e}",
                dir.display()
            )
        })?;
        let header = live_header();
        let stem = format!("{}-{}", log_stem(&header.cmdline), header.pid);
        // Process ids repeat (every container starts its own at 1), so an
        // existing log is never overwritten: the next free `.<n>` is taken.
        let mut claimed = None;
        for n in 0..1000u32 {
            let path = if n == 0 {
                dir.join(format!("{stem}{LOG_SUFFIX}"))
            } else {
                dir.join(format!("{stem}.{n}{LOG_SUFFIX}"))
            };
            match std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
            {
                Ok(file) => {
                    claimed = Some((path, file));
                    break;
                }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(e) => {
                    return Err(format!(
                        "rmw_cerulion: {DELIVERY_LOG_ENV}: cannot create {}: {e}",
                        path.display()
                    ))
                }
            }
        }
        let (path, file) = claimed.ok_or_else(|| {
            format!(
                "rmw_cerulion: {DELIVERY_LOG_ENV}: {stem}{LOG_SUFFIX} and 999 numbered \
                siblings already exist in {}",
                dir.display()
            )
        })?;
        let writer = LogWriter::with_file(&path, file, &header)
            .map_err(|e| format!("rmw_cerulion: {DELIVERY_LOG_ENV}: {e}"))?;
        tracing::info!(log = %path.display(), "rmw_cerulion: recording deliveries");
        Ok(Recorder {
            header,
            writer: Mutex::new(writer),
            registry: Mutex::new(Registry::default()),
            detector: Detector::default(),
            not_replayable: AtomicBool::new(false),
            clock: Mutex::new((0, 0)),
            clock_cv: std::sync::Condvar::new(),
            clock_waits_timed_out: std::sync::atomic::AtomicU64::new(0),
            clock_waits: std::sync::atomic::AtomicU64::new(0),
            clock_wait_ns: std::sync::atomic::AtomicU64::new(0),
        })
    }

    /// The clock section keeps two things the rmw cannot see apart in the
    /// recording. rcl checks a thread's timers right after its `rmw_wait`
    /// returns, against the ROS time a `/clock` callback (on rclcpp's clock
    /// thread) sets after its take. Live, those two race; replayed, the token
    /// serves them one at a time in the recorded order. So while recording:
    /// a `rmw_wait` return waits until no `/clock` take is between its take
    /// and its thread's next `rmw_wait` (its callback has set the time), and
    /// a `/clock` take waits until no thread is between its `rmw_wait` return
    /// and its next rmw call (its timer check is done). Each side waits at
    /// most [`CLOCK_SECTION_WAIT`], then goes on, counted and reported when
    /// the log closes.
    fn clock_section(&self, enter_clock: bool) {
        let mut g = self.clock.lock().unwrap_or_else(|e| e.into_inner());
        let start = std::time::Instant::now();
        let deadline = start + CLOCK_SECTION_WAIT;
        let mut waited = false;
        loop {
            let busy = if enter_clock { g.1 > 0 } else { g.0 > 0 };
            if !busy {
                break;
            }
            waited = true;
            let now = std::time::Instant::now();
            if now >= deadline {
                self.clock_waits_timed_out.fetch_add(1, Ordering::Relaxed);
                break;
            }
            g = self
                .clock_cv
                .wait_timeout(g, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
        if waited {
            self.clock_waits.fetch_add(1, Ordering::Relaxed);
            self.clock_wait_ns
                .fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
        }
        if enter_clock {
            g.0 += 1;
            CLOCK_HELD.with(|c| c.set(true));
        } else {
            g.1 += 1;
            POST_WAIT.with(|c| c.set(true));
        }
    }

    /// This thread made an rmw call: its post-wait span (if any) ends; on
    /// `rmw_wait` entry its `/clock` take (if any) ends too.
    fn clock_leave(&self, wait_entry: bool) {
        let post = POST_WAIT.with(|c| c.replace(false));
        let clock = wait_entry && CLOCK_HELD.with(|c| c.replace(false));
        if post || clock {
            let mut g = self.clock.lock().unwrap_or_else(|e| e.into_inner());
            if post {
                g.1 = g.1.saturating_sub(1);
            }
            if clock {
                g.0 = g.0.saturating_sub(1);
            }
            self.clock_cv.notify_all();
        }
    }

    fn is_clock_sub(&self, key: usize) -> bool {
        self.registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .by_key
            .get(&key)
            .is_some_and(|e| e.topic == "/clock" && e.name.starts_with("sub:"))
    }

    fn append(&self, rec: Record) -> u64 {
        self.writer
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .append(rec)
    }

    fn flush(&self) {
        self.writer
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .flush();
    }

    fn finish(&self) {
        let mut w = self.writer.lock().unwrap_or_else(|e| e.into_inner());
        let trailer = w.finish();
        let timed_out = self.clock_waits_timed_out.load(Ordering::Relaxed);
        let waits = self.clock_waits.load(Ordering::Relaxed);
        let log = w.path().display().to_string();
        if waits > 0 {
            let ns = self.clock_wait_ns.load(Ordering::Relaxed);
            note(
                tracing::Level::INFO,
                &format!(
                    "rmw_cerulion delivery log: the clock section waited {waits} times, {} us \
                     in all ({log})",
                    ns / 1000
                ),
            );
        }
        if timed_out > 0 {
            note(
                tracing::Level::WARN,
                &format!(
                    "rmw_cerulion delivery log: {timed_out} clock-section waits gave up after {} \
                     ms; a replay may diverge at a timer that fired next to a /clock update \
                     ({log})",
                    CLOCK_SECTION_WAIT.as_millis()
                ),
            );
        }
        if !trailer.complete || trailer.dropped > 0 {
            note(
                tracing::Level::ERROR,
                &format!(
                    "rmw_cerulion delivery log closed INCOMPLETE: {log}, {} records dropped, \
                     error: {}",
                    trailer.dropped,
                    w.failed().unwrap_or("unknown")
                ),
            );
        }
    }

    fn node(&self, name: &str, namespace: &str) {
        self.registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .nodes
            .push(name.to_string());
        self.append(Record::Node {
            k: 0,
            name: name.to_string(),
            namespace: namespace.to_string(),
        });
    }

    fn entity(&self, kind: EntityKind, key: usize, topic: &str, node: &str) {
        let ent = self
            .registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .add(kind, key, topic, node);
        self.append(Record::Entity(EntityRec {
            k: 0,
            id: ent.id,
            kind,
            name: ent.name,
            topic: ent.topic,
            node: ent.node,
        }));
    }

    fn entity_id(&self, key: usize) -> EntityId {
        self.registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .by_key
            .get(&key)
            .map(|e| e.id)
            .unwrap_or(UNKNOWN_ENTITY)
    }

    fn mark_not_replayable(&self, cause: NotReplayable, detail: &str) {
        if self.not_replayable.swap(true, Ordering::AcqRel) {
            return;
        }
        let nodes = self
            .registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .nodes
            .clone();
        let reason = refusal_message(cause, &self.header.cmdline, &nodes, detail);
        let mut w = self.writer.lock().unwrap_or_else(|e| e.into_inner());
        w.append(Record::NotReplayable {
            k: 0,
            reason: reason.clone(),
        });
        w.flush();
        tracing::warn!(
            log = %w.path().display(),
            reason = %reason,
            "rmw_cerulion delivery log marked NOT REPLAYABLE"
        );
    }

    unsafe fn wait(
        &self,
        arrays: WaitArrays,
        owner: &AtomicU32,
        timeout: *const ffi::rmw_time_t,
        live: impl FnOnce() -> rmw_ret_t,
    ) -> rmw_ret_t {
        let thread = thread_ix();
        WAITED.with(|w| w.set(true));
        self.clock_leave(true);
        if let Some(d) = self.detector.on_wait(owner, thread) {
            self.mark_not_replayable(NotReplayable::MultiThreaded, &d);
        }
        let attached: Vec<EntityId> = {
            let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
            arrays.entities(&reg).iter().map(|e| e.id).collect()
        };
        let timeout_ns = if timeout.is_null() {
            -1
        } else {
            let t = &*timeout;
            (t.sec as i128 * 1_000_000_000 + t.nsec as i128).min(i64::MAX as i128) as i64
        };
        let ret = live();
        let ready: Vec<EntityId> = {
            let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
            arrays.entities(&reg).iter().map(|e| e.id).collect()
        };
        // After the live wait, before the record's order key is minted.
        self.clock_section(false);
        self.append(Record::Wait(WaitRec {
            k: 0,
            thread,
            ret,
            timeout_ns,
            attached,
            ready,
        }));
        ret
    }
}

// ===========================================================================
// Player
// ===========================================================================

struct PlayState {
    takes: TakeQueues,
    graph: GraphAnswers,
    oracle: Oracle,
    /// Live thread → recorded thread.
    thread_map: HashMap<u32, u32>,
    claimed: HashSet<u32>,
    node_cursor: usize,
    guards_not_triggered: usize,
    refused: Option<String>,
    /// See [`Verdict::refused_at`].
    refused_at: Option<&'static str>,
    watchdog: Option<WatchdogReport>,
    verdict_written: bool,
    exhausted: bool,
    /// Each recorded thread's last served take: the callback a stalled token
    /// holder is in.
    last_take: HashMap<u32, EntityId>,
    /// Served takes not ordered across threads (made holding the Python GIL).
    takes_unordered: usize,
    /// Torn log: takes and graph queries past the recording (see `oracle.rs`).
    inputs_past_log_end: usize,
    /// Outputs made after exhaustion closed every segment.
    outputs_after_exhaustion: usize,
    /// The end-of-replay checks ran (see `Player::final_checks`).
    final_checked: bool,
}

/// The replay side.
pub(crate) struct Player {
    path: PathBuf,
    header: ProcessHeader,
    ending: LogEnding,
    schedule: Schedule,
    gate: Gate,
    registry: Mutex<Registry>,
    detector: Detector,
    rec_by_name: HashMap<String, EntityRec>,
    rec_nodes: Vec<(String, String)>,
    /// Each recorded thread's first attached set (maps a replaying thread to
    /// the recorded one at its first wait).
    first_attached: HashMap<u32, Vec<EntityId>>,
    /// Every recorded entity as (creation order key, id), by key.
    created: Vec<(u64, EntityId)>,
    /// The creation fence's position in `created` (`usize::MAX` once lifted).
    fence_cursor: Mutex<usize>,
    /// The unit a stale wait gave back, and since when (see `stale_wait`).
    stale_since: Mutex<Option<(usize, Instant)>>,
    state: Mutex<PlayState>,
}

fn refused_verdict(log: &Path, why: &str) -> Verdict {
    Verdict {
        verdict: "refused".to_string(),
        process: process_label(&live_cmdline()),
        log: log.display().to_string(),
        log_complete: false,
        windows_recorded: 0,
        units_recorded: 0,
        units_served: 0,
        windows_served_as_sets: 0,
        windows_split: 0,
        takes_unordered: 0,
        segments_compared: 0,
        segments_divergent: 0,
        segments_inherited: 0,
        outputs_compared: 0,
        outputs_excluded: 0,
        outputs_before_first_event: 0,
        outputs_off_window: 0,
        outputs_unordered: 0,
        outputs_past_log_end: 0,
        outputs_after_exhaustion: 0,
        inputs_past_log_end: 0,
        takes_not_replayed: 0,
        guards_not_triggered_live: 0,
        culprit: None,
        input_mismatch: None,
        watchdog: None,
        refused: Some(why.to_string()),
        refused_at: Some("start".to_string()),
    }
}

/// The verdict file of a log.
pub(crate) fn verdict_path(log: &Path) -> PathBuf {
    let mut s = log.as_os_str().to_os_string();
    s.push(VERDICT_SUFFIX);
    PathBuf::from(s)
}

fn write_verdict_file(log: &Path, v: &Verdict) {
    let path = verdict_path(log);
    let tmp = {
        let mut s = path.as_os_str().to_os_string();
        s.push(".tmp");
        PathBuf::from(s)
    };
    let mut bytes = serde_json::to_vec_pretty(v).unwrap_or_default();
    bytes.push(b'\n');
    let res = std::fs::write(&tmp, &bytes).and_then(|()| std::fs::rename(&tmp, &path));
    if let Err(e) = res {
        tracing::error!(
            verdict = %path.display(),
            error = %e,
            "rmw_cerulion replay: cannot write the verdict file"
        );
    }
}

impl Player {
    fn load(path: &Path) -> Result<Player, String> {
        let log = LoadedLog::read(path).map_err(|e| format!("rmw_cerulion replay refused: {e}"))?;
        let live = live_header();
        check_identity(&log.header, &live)?;
        let mut waits = Vec::new();
        let mut takes = Vec::new();
        let mut graphs = Vec::new();
        let mut rec_by_name = HashMap::new();
        let mut rec_nodes = Vec::new();
        let mut names = EntityNames::default();
        for r in &log.records {
            match r {
                Record::Wait(w) => waits.push(w.clone()),
                Record::Take(t) => takes.push(t.clone()),
                Record::Graph(g) => graphs.push(g.clone()),
                Record::Entity(e) => {
                    names.name.insert(e.id, e.name.clone());
                    names.topic.insert(e.id, e.topic.clone());
                    names.node.insert(e.id, e.node.clone());
                    rec_by_name.insert(e.name.clone(), e.clone());
                }
                Record::Node {
                    name, namespace, ..
                } => rec_nodes.push((name.clone(), namespace.clone())),
                Record::NotReplayable { reason, .. } => {
                    return Err(format!(
                        "rmw_cerulion replay refused: {reason} (the log {} is marked not \
                         replayable)",
                        path.display()
                    ));
                }
                _ => {}
            }
        }
        let clock_subs: HashSet<EntityId> = rec_by_name
            .values()
            .filter(|e| e.kind == EntityKind::Subscription && e.topic == "/clock")
            .map(|e| e.id)
            .collect();
        let schedule = Schedule::build(&waits, &takes, &clock_subs);
        let mut first_attached = HashMap::new();
        for w in &schedule.windows {
            first_attached
                .entry(w.thread)
                .or_insert_with(|| w.attached.clone());
        }
        let gate = Gate::new(&schedule);
        let torn = log.ending != LogEnding::Complete;
        let mut created: Vec<(u64, EntityId)> = rec_by_name.values().map(|e| (e.k, e.id)).collect();
        created.sort_unstable();
        let oracle = Oracle::new(&log.records, names, torn);
        tracing::info!(
            log = %path.display(),
            windows = schedule.windows.len(),
            units = schedule.units.len(),
            "rmw_cerulion: replaying deliveries"
        );
        let player = Player {
            path: path.to_path_buf(),
            header: log.header,
            ending: log.ending,
            gate,
            registry: Mutex::new(Registry::default()),
            detector: Detector::default(),
            rec_by_name,
            rec_nodes,
            first_attached,
            created,
            fence_cursor: Mutex::new(0),
            stale_since: Mutex::new(None),
            state: Mutex::new(PlayState {
                takes: TakeQueues::new(&takes, torn),
                graph: GraphAnswers::new(&graphs),
                oracle,
                thread_map: HashMap::new(),
                claimed: HashSet::new(),
                node_cursor: 0,
                guards_not_triggered: 0,
                refused: None,
                refused_at: None,
                watchdog: None,
                verdict_written: false,
                exhausted: false,
                last_take: HashMap::new(),
                takes_unordered: 0,
                inputs_past_log_end: 0,
                outputs_after_exhaustion: 0,
                final_checked: false,
            }),
            schedule,
        };
        player.write_running();
        Ok(player)
    }

    /// Before the replay starts: a `running` verdict replaces any earlier
    /// one, so a replay that dies before it writes its own (a crash, an
    /// abort, SIGKILL) never leaves a previous run's verdict in place.
    fn write_running(&self) {
        let st = self.lock();
        let mut v = self.verdict(&st);
        v.verdict = "running".to_string();
        write_verdict_file(&self.path, &v);
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, PlayState> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn node(&self, name: &str, namespace: &str) -> Result<(), String> {
        self.registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .nodes
            .push(name.to_string());
        let mut st = self.lock();
        let i = st.node_cursor;
        st.node_cursor += 1;
        let ok = self
            .rec_nodes
            .get(i)
            .is_some_and(|(n, ns)| n == name && ns == namespace);
        if ok {
            return Ok(());
        }
        let why = match self.rec_nodes.get(i) {
            Some((n, ns)) => format!(
                "rmw_cerulion replay refused: node {} is node {i} of this process, the recording's \
                 node {i} is {}",
                join_node(namespace, name),
                join_node(ns, n)
            ),
            None => format!(
                "rmw_cerulion replay refused: node {} is node {i} of this process; the recording \
                 created only {} nodes",
                join_node(namespace, name),
                self.rec_nodes.len()
            ),
        };
        self.refuse(&mut st, why.clone(), "start");
        Err(why)
    }

    fn entity(&self, kind: EntityKind, key: usize, topic: &str, node: &str) {
        let mut reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
        let ent = reg.add(kind, key, topic, node);
        let rec = self.rec_by_name.get(&ent.name).map(|r| r.id);
        if let Some(e) = reg.by_key.get_mut(&key) {
            e.rec = rec;
        }
        if let Some(r) = rec {
            reg.bound.insert(r);
        }
    }

    /// The creation fence: before window `k` is served, every entity the
    /// recording created before `k` must exist live. Entity creation is not a
    /// served event, so without the fence a thread can be served a window
    /// (an rclcpp clock thread's "entities changed" guard) before another
    /// thread creates the entity the recorded wake-up collected, and that
    /// thread's next wait then lacks it. Waits at most [`CREATION_FENCE`];
    /// an entity still missing then is reported as an input mismatch and the
    /// fence is lifted for the rest of the replay.
    fn creation_fence(&self, k: u64, rec_thread: u32) {
        let deadline = Instant::now() + CREATION_FENCE;
        loop {
            let missing = {
                let mut cur = self.fence_cursor.lock().unwrap_or_else(|e| e.into_inner());
                if *cur == usize::MAX {
                    return;
                }
                let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
                while *cur < self.created.len()
                    && self.created[*cur].0 < k
                    && reg.bound.contains(&self.created[*cur].1)
                {
                    *cur += 1;
                }
                if *cur >= self.created.len() || self.created[*cur].0 >= k {
                    return;
                }
                if Instant::now() >= deadline {
                    let id = self.created[*cur].1;
                    *cur = usize::MAX;
                    Some(id)
                } else {
                    None
                }
            };
            if let Some(id) = missing {
                let mut st = self.lock();
                let (name, _) = st.oracle.describe(id);
                st.oracle.input_mismatch(
                    format!(
                        "the recording created {name} before window order key {k}, and this \
                         process had not created it {}s later",
                        CREATION_FENCE.as_secs()
                    ),
                    Some(rec_thread),
                );
                return;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Whether this wait cannot take unit `i` yet: the unit reports ready an
    /// entity this process has created (bound) that the wait does not
    /// attach. rclcpp's clock thread can enter `rmw_wait` before the main
    /// thread creates `/clock` in its callback group; the recording's wait
    /// that reports `/clock` ready came after a collection that held it.
    /// Answered `true` for at most [`CREATION_FENCE`] per unit; after that
    /// the unit is served and the missing entity is an input mismatch.
    unsafe fn stale_wait(&self, arrays: WaitArrays, unit: &gate::Unit, i: usize) -> bool {
        let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
        let mut attached: HashSet<EntityId> = HashSet::new();
        arrays.for_each_slot(|_, slot| {
            let h = *slot;
            if let Some(r) = (!h.is_null())
                .then(|| reg.by_key.get(&(h as usize)).and_then(|e| e.rec))
                .flatten()
            {
                attached.insert(r);
            }
        });
        let stale = unit
            .ready
            .iter()
            .any(|r| !attached.contains(r) && reg.bound.contains(r));
        drop(reg);
        let mut since = self.stale_since.lock().unwrap_or_else(|e| e.into_inner());
        if !stale {
            if since.is_some_and(|(u, _)| u == i) {
                *since = None;
            }
            return false;
        }
        match *since {
            Some((u, at)) if u == i => at.elapsed() < CREATION_FENCE,
            _ => {
                *since = Some((i, Instant::now()));
                true
            }
        }
    }

    fn rec_of(&self, key: usize) -> Option<(EntityId, String)> {
        let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
        reg.by_key
            .get(&key)
            .map(|e| (e.rec.unwrap_or(UNKNOWN_ENTITY), e.name.clone()))
    }

    /// Refuse the rest of the replay: logged once, verdict written.
    fn refuse(&self, st: &mut PlayState, why: String, at: &'static str) {
        if st.refused.is_some() {
            return;
        }
        tracing::error!("{why}");
        let _ = ffi::rcutils_set_error_state_best_effort(&why);
        st.refused = Some(why);
        st.refused_at = Some(at);
        self.write_verdict(st);
    }

    /// Map a live thread to a recorded one (at its first wait, by the set of
    /// entities it waits on; before any wait, by thread index).
    fn map_thread(
        &self,
        st: &mut PlayState,
        live: u32,
        attached: Option<&[EntityId]>,
    ) -> Option<u32> {
        if let Some(r) = st.thread_map.get(&live) {
            return Some(*r);
        }
        let pick = match attached {
            Some(att) => {
                let att: HashSet<EntityId> = att.iter().copied().collect();
                let mut best: Option<(usize, u32)> = None;
                let mut threads: Vec<&u32> = self.first_attached.keys().collect();
                threads.sort();
                for t in threads {
                    if st.claimed.contains(t) {
                        continue;
                    }
                    let rec: HashSet<EntityId> = self.first_attached[t].iter().copied().collect();
                    let overlap = rec.intersection(&att).count();
                    if rec == att {
                        best = Some((usize::MAX, *t));
                        break;
                    }
                    if overlap > 0 && best.is_none_or(|(b, _)| overlap > b) {
                        best = Some((overlap, *t));
                    }
                }
                best.map(|(_, t)| t)
            }
            None => Some(live).filter(|t| !st.claimed.contains(t)),
        };
        if let Some(rec) = pick {
            st.claimed.insert(rec);
            st.thread_map.insert(live, rec);
            st.oracle.map_thread(live, rec);
        }
        pick
    }

    unsafe fn wait(
        &self,
        arrays: WaitArrays,
        events: *mut ffi::rmw_events_t,
        owner: &AtomicU32,
    ) -> rmw_ret_t {
        null_events(events);
        let live = thread_ix();
        WAITED.with(|w| w.set(true));
        self.gate.note_progress();
        let detected = self.detector.on_wait(owner, live);
        let ents = {
            let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
            arrays.entities(&reg)
        };
        let rec_thread = {
            let mut st = self.lock();
            if let Some(d) = detected {
                let nodes = self.live_nodes();
                let why = refusal_message(
                    NotReplayable::MultiThreaded,
                    &self.header.cmdline,
                    &nodes,
                    &d,
                );
                self.refuse(
                    &mut st,
                    format!("rmw_cerulion replay refused: {why}"),
                    "run",
                );
            }
            if st.refused.is_some() {
                drop(st);
                arrays.null_all();
                return RMW_RET_ERROR;
            }
            st.oracle.waited(live);
            let attached: Vec<EntityId> = ents.iter().filter_map(|e| e.rec).collect();
            let mapped = self.map_thread(&mut st, live, Some(&attached));
            if mapped.is_none() {
                st.oracle.input_mismatch(
                    format!("thread {live} waits, and the recording has no thread that waits on its entities"),
                    None,
                );
            }
            mapped
        };
        let Some(rec_thread) = rec_thread else {
            arrays.null_all();
            std::thread::sleep(IDLE_AFTER_LOG);
            return RMW_RET_TIMEOUT;
        };
        let turn = self
            .gate
            .wait_turn(rec_thread, |stall| self.on_stall(stall));
        match turn {
            Turn::Serve(i) => {
                let unit = &self.schedule.units[i];
                let window = &self.schedule.windows[unit.window];
                if unit.first {
                    self.creation_fence(window.k, rec_thread);
                }
                if self.stale_wait(arrays, unit, i) {
                    // This wait's arrays were collected before an entity the
                    // unit reports ready was created (the creation fence ran
                    // inside it): give the unit back and return nothing
                    // ready, so the executor collects again and waits with it.
                    self.gate.give_back(rec_thread, i);
                    arrays.null_all();
                    std::thread::sleep(IDLE_AFTER_LOG);
                    return RMW_RET_TIMEOUT;
                }
                let mut st = self.lock();
                if unit.first {
                    st.oracle.event(live, window.k);
                }
                // Apply the recorded readiness by entity, never probing live.
                let mut attached_rec: HashSet<EntityId> = HashSet::new();
                let reg = self.registry.lock().unwrap_or_else(|e| e.into_inner());
                let mut not_triggered = 0usize;
                arrays.for_each_slot(|kind, slot| {
                    let h = *slot;
                    if h.is_null() {
                        return;
                    }
                    let rec = reg.by_key.get(&(h as usize)).and_then(|e| e.rec);
                    let keep = rec.is_some_and(|r| unit.ready.contains(&r));
                    if let Some(r) = rec {
                        attached_rec.insert(r);
                    }
                    if !keep {
                        *slot = std::ptr::null_mut();
                    } else if kind == EntityKind::Guard {
                        let state = &*(h as *const GuardConditionState);
                        if !state.triggered.swap(false, Ordering::AcqRel) {
                            not_triggered += 1;
                        }
                        if let Some(d) = &state.doorbell {
                            d.drain();
                        }
                    }
                });
                drop(reg);
                st.guards_not_triggered += not_triggered;
                if let Some(missing) = unit.ready.iter().find(|r| !attached_rec.contains(r)) {
                    let (name, _) = st.oracle.describe(*missing);
                    st.oracle.input_mismatch(
                        format!(
                            "window {} reports {name} ready, and this wait does not attach it",
                            unit.window
                        ),
                        Some(rec_thread),
                    );
                }
                unit.ret
            }
            Turn::NoMore | Turn::Exhausted => {
                if turn == Turn::Exhausted {
                    let mut st = self.lock();
                    if !st.exhausted {
                        st.exhausted = true;
                        st.oracle.close_all();
                        self.final_checks(&mut st);
                        self.write_verdict(&mut st);
                    }
                }
                arrays.null_all();
                std::thread::sleep(IDLE_AFTER_LOG);
                RMW_RET_TIMEOUT
            }
        }
    }

    fn live_nodes(&self) -> Vec<String> {
        self.registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .nodes
            .clone()
    }

    fn on_stall(&self, stall: gate::Stall) {
        let unit = &self.schedule.units[stall.unit];
        let mut st = self.lock();
        // The holder is inside the callback of its last served take; with no
        // take yet, inside whatever its unit released.
        let (entity, node) = st
            .last_take
            .get(&stall.holder_thread)
            .or(unit.ready.first())
            .map(|e| st.oracle.describe(*e))
            .unwrap_or_else(|| ("<no entity>".to_string(), String::new()));
        let report = WatchdogReport {
            holder_thread: stall.holder_thread,
            entity: entity.clone(),
            node: node.clone(),
            window: unit.window,
            waiting_thread: stall.waiting_thread,
        };
        tracing::error!(
            holder_thread = stall.holder_thread,
            waiting_thread = stall.waiting_thread,
            window = unit.window,
            entity = %entity,
            node = %node,
            stalled_s = gate::WATCHDOG.as_secs(),
            "rmw_cerulion replay WATCHDOG: the holder thread has held the replay token with no \
             rmw progress since it was served the window; a callback is blocked on something \
             the replay cannot serve (another executor's thread waits for this one). The \
             replay cannot continue."
        );
        if st.watchdog.is_none() {
            st.watchdog = Some(report);
            self.write_verdict(&mut st);
        }
    }

    fn serve_take(&self, key: usize, kind: TakeKind) -> Result<Option<TakeRec>, ()> {
        let live = thread_ix();
        self.gate.note_work();
        let mut st = self.lock();
        if let Some(d) = self.detector.on_take(live, WAITED.with(Cell::get)) {
            let nodes = self.live_nodes();
            let why = refusal_message(
                NotReplayable::MultiThreaded,
                &self.header.cmdline,
                &nodes,
                &d,
            );
            self.refuse(
                &mut st,
                format!("rmw_cerulion replay refused: {why}"),
                "run",
            );
        }
        if st.refused.is_some() {
            return Err(());
        }
        let rec_thread = self.map_thread(&mut st, live, None);
        let Some((entity, name)) = self.rec_of(key) else {
            st.oracle
                .input_mismatch("a take from an unregistered entity".to_string(), rec_thread);
            return Ok(None);
        };
        if entity == UNKNOWN_ENTITY {
            st.oracle.input_mismatch(
                format!("a take from {name}, which the recording does not have"),
                rec_thread,
            );
            return Ok(None);
        }
        let served = st.takes.serve(rec_thread, entity, kind, &name);
        if served.past_end {
            if st.oracle.at_tail(live) {
                // A torn log: the recording may have made this take after its
                // last surviving record. Served as not taken.
                st.inputs_past_log_end += 1;
            } else {
                st.oracle.input_mismatch(
                    format!("took from {name} after this thread's last recorded take"),
                    rec_thread,
                );
            }
        }
        if let Some(m) = served.mismatch {
            st.oracle.input_mismatch(m, rec_thread);
        }
        if let (Some(r), Some(t)) = (&served.rec, rec_thread) {
            // The take's turn across threads (gate.rs, "Takes in order across
            // threads"), with the state unlocked: other threads' waits and
            // takes run meanwhile. Not while this thread holds the Python GIL
            // (rclpy takes with it held): the thread whose turn comes first
            // may need the GIL to reach its rmw call, so the take runs at
            // once. rclpy keeps each node's clock on that node's executor
            // thread, so its timer checks do not race another thread.
            if holds_python_gil() {
                st.takes_unordered += 1;
                return self.finish_take(st, live, rec_thread, entity, served.rec);
            }
            drop(st);
            self.gate.take_turn(t, r.k, |stall| self.on_stall(stall));
            st = self.lock();
        }
        self.finish_take(st, live, rec_thread, entity, served.rec)
    }

    fn finish_take(
        &self,
        mut st: std::sync::MutexGuard<'_, PlayState>,
        live: u32,
        rec_thread: Option<u32>,
        entity: EntityId,
        rec: Option<TakeRec>,
    ) -> Result<Option<TakeRec>, ()> {
        if let Some(r) = &rec {
            st.oracle.event(live, r.k);
            if let Some(t) = rec_thread {
                st.last_take.insert(t, entity);
            }
        }
        Ok(rec)
    }

    /// A served take could not be decoded into the caller's message.
    pub(crate) fn decode_failed(&self, what: &str) {
        let mut st = self.lock();
        st.oracle
            .input_mismatch(format!("a recorded payload failed to decode: {what}"), None);
    }

    fn output(&self, key: usize, mut out: Out) {
        let live = thread_ix();
        self.gate.note_work();
        if let Some((rec, _)) = self.rec_of(key) {
            out.entity = rec;
        }
        let mut st = self.lock();
        if st.exhausted {
            // Every segment is closed: nothing recorded is left to compare
            // this with. An output of a thread that never waits still counts
            // as unordered (the verdict is rewritten when the process ends).
            st.outputs_after_exhaustion += 1;
            st.oracle.output_after_close(live, out.entity);
            return;
        }
        st.oracle.output(live, out);
    }

    fn graph(
        &self,
        query: GraphQuery,
        name: &str,
        live: impl FnOnce() -> (rmw_ret_t, u64),
    ) -> (rmw_ret_t, u64) {
        self.gate.note_work();
        let mut st = self.lock();
        match st.graph.next(query, name) {
            Some(r) => (r.ret, r.answer),
            None if self.ending != LogEnding::Complete => {
                // A torn log: the answer may be past its last surviving record.
                st.inputs_past_log_end += 1;
                drop(st);
                live()
            }
            None => {
                st.oracle.input_mismatch(
                    format!("a {query:?} query on {name} beyond the recording"),
                    None,
                );
                drop(st);
                live()
            }
        }
    }

    /// The checks that need the whole replay: a recorded take the replay
    /// never made is named (once every unit is served). Runs once.
    fn final_checks(&self, st: &mut PlayState) {
        if st.final_checked {
            return;
        }
        st.final_checked = true;
        if self.gate.served() < self.schedule.units.len() {
            return;
        }
        // A torn log's last take on a thread may be one the recording made
        // just before it stopped: not named (`takes_not_replayed` counts it).
        let first = st
            .takes
            .first_unserved_where(|thread, k| !st.oracle.is_tail(thread, k));
        if let Some((thread, entity, k)) = first {
            st.oracle.untaken(thread, entity, k);
        }
    }

    /// Why the process is refused at the end: a thread outside every executor
    /// published after the first wait (see `oracle.rs`).
    fn unordered_refusal(&self, st: &PlayState) -> Option<String> {
        let (n, entity) = st.oracle.unordered();
        if n == 0 {
            return None;
        }
        let topic = entity
            .map(|e| st.oracle.describe(e).0)
            .unwrap_or_else(|| "<an entity the recording does not have>".to_string());
        Some(format!(
            "rmw_cerulion replay refused: {}",
            refusal_message(
                NotReplayable::UnorderedThread,
                &self.header.cmdline,
                &self.live_nodes(),
                &format!(
                    "{n} outputs, the first on {topic}, came from a thread that never calls \
                     rmw_wait; the replay orders executor threads only, so these outputs \
                     were not compared"
                ),
            )
        ))
    }

    fn verdict(&self, st: &PlayState) -> Verdict {
        let o = &st.oracle;
        let served = self.gate.served();
        let total = self.schedule.units.len();
        let complete = self.ending == LogEnding::Complete;
        let (refused, refused_at) = match &st.refused {
            Some(why) => (Some(why.clone()), st.refused_at),
            None => match self.unordered_refusal(st) {
                Some(why) => (Some(why), Some("end")),
                None => (None, None),
            },
        };
        let verdict = if refused.is_some() {
            "refused"
        } else if st.watchdog.is_some() {
            "watchdog"
        } else if o.culprit.is_some() {
            "diverged"
        } else if o.input_mismatch.is_some() {
            "input_mismatch"
        } else if served < total {
            "replay_stopped_early"
        } else if !complete {
            "log_exhausted_early"
        } else {
            "identical"
        };
        Verdict {
            verdict: verdict.to_string(),
            process: process_label(&self.header.cmdline),
            log: self.path.display().to_string(),
            log_complete: complete,
            windows_recorded: self.schedule.windows.len(),
            units_recorded: total,
            units_served: served,
            windows_served_as_sets: self.schedule.set_served_windows,
            windows_split: self.schedule.split_windows,
            takes_unordered: st.takes_unordered,
            segments_compared: o.segments_compared,
            segments_divergent: o.segments_divergent,
            segments_inherited: o.segments_inherited,
            outputs_compared: o.outputs_compared,
            outputs_excluded: o.outputs_excluded,
            outputs_before_first_event: o.outputs_before_first_event,
            outputs_off_window: o.outputs_off_window,
            outputs_unordered: o.unordered().0,
            outputs_past_log_end: o.outputs_past_log_end,
            outputs_after_exhaustion: st.outputs_after_exhaustion,
            inputs_past_log_end: st.inputs_past_log_end,
            takes_not_replayed: st.takes.unserved(),
            guards_not_triggered_live: st.guards_not_triggered,
            culprit: o.culprit.clone(),
            input_mismatch: o.input_mismatch.clone(),
            watchdog: st.watchdog.clone(),
            refused,
            refused_at: refused_at.map(str::to_string),
        }
    }

    fn write_verdict(&self, st: &mut PlayState) {
        let v = self.verdict(st);
        // The file first: at process exit a log line may not get out.
        write_verdict_file(&self.path, &v);
        st.verdict_written = true;
        let file = verdict_path(&self.path).display().to_string();
        match (
            v.verdict.as_str(),
            &v.culprit,
            &v.input_mismatch,
            &v.refused,
        ) {
            ("diverged", Some(c), _, _) => note(
                tracing::Level::ERROR,
                &format!(
                    "rmw_cerulion replay DIVERGED: process `{}`, node '{}', topic {}: window {} \
                     (after delivery index {}) inputs matched, {} (output {} of {} recorded, {} \
                     replayed). Verdict: {file}",
                    v.process,
                    c.node,
                    c.topic,
                    c.window,
                    c.delivery,
                    c.what,
                    c.output_index,
                    c.recorded_outputs,
                    c.replayed_outputs,
                ),
            ),
            ("input_mismatch", _, Some(m), _) => note(
                tracing::Level::ERROR,
                &format!(
                    "rmw_cerulion replay INPUT MISMATCH: process `{}`: {}. Verdict: {file}",
                    v.process, m.what
                ),
            ),
            ("refused", _, _, Some(why)) => {
                note(tracing::Level::ERROR, &format!("{why}. Verdict: {file}"))
            }
            (other, ..) => note(
                tracing::Level::INFO,
                &format!("rmw_cerulion replay verdict written: {other}, {file}"),
            ),
        }
    }

    fn write_verdict_if_unwritten(&self) {
        let mut st = self.lock();
        if !st.verdict_written {
            st.oracle.close_all();
            self.final_checks(&mut st);
            self.write_verdict(&mut st);
        }
    }

    /// At process exit: the verdict if none was written, else rewritten once
    /// the schedule was used up, so the outputs made after exhaustion are
    /// counted in it.
    fn finish(&self) {
        let mut st = self.lock();
        if !st.verdict_written {
            st.oracle.close_all();
            self.final_checks(&mut st);
            self.write_verdict(&mut st);
        } else if st.exhausted {
            self.write_verdict(&mut st);
        }
    }
}

fn join_node(namespace: &str, name: &str) -> String {
    if namespace.ends_with('/') {
        format!("{namespace}{name}")
    } else {
        format!("{namespace}/{name}")
    }
}

/// A take served from the log could not be decoded into the caller's
/// message: an input mismatch (the caller then returns `RMW_RET_ERROR`).
pub(crate) fn served_decode_failed(what: &str) {
    if let Mode::Replay(p) = mode() {
        p.decode_failed(what);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neither_variable_is_off_and_both_are_refused_by_name() {
        assert_eq!(requested(None, None), Ok(Requested::Off));
        assert_eq!(
            requested(Some("".into()), Some("".into())),
            Ok(Requested::Off),
            "an empty value counts as unset"
        );
        assert_eq!(
            requested(Some("/tmp/d".into()), None),
            Ok(Requested::Record("/tmp/d".into()))
        );
        assert_eq!(
            requested(None, Some("/tmp/f".into())),
            Ok(Requested::Replay("/tmp/f".into()))
        );
        let e = requested(Some("/tmp/d".into()), Some("/tmp/f".into())).unwrap_err();
        assert!(
            e.contains(DELIVERY_LOG_ENV) && e.contains(REPLAY_ENV),
            "{e}"
        );
    }

    fn header() -> ProcessHeader {
        ProcessHeader {
            format: RECORD_VERSION,
            cmdline: vec!["python3".into(), "mixer.py".into()],
            ros_distro: "humble".into(),
            built_for: "humble".into(),
            library_digest: "abc".into(),
            pid: 1,
        }
    }

    #[test]
    fn identity_matches_on_everything_but_the_pid() {
        let mut live = header();
        live.pid = 999;
        assert_eq!(check_identity(&header(), &live), Ok(()));
    }

    #[test]
    fn identity_mismatches_are_refused_and_named() {
        let mut live = header();
        live.cmdline.push("--wallclock".into());
        live.ros_distro = "jazzy".into();
        live.library_digest = "def".into();
        let e = check_identity(&header(), &live).unwrap_err();
        assert!(e.contains("command line"), "{e}");
        assert!(
            e.contains("ROS_DISTRO: recorded 'humble', this process 'jazzy'"),
            "{e}"
        );
        assert!(e.contains("library digest"), "{e}");
        assert!(!e.contains("built for"), "{e}");
    }

    #[test]
    fn the_log_stem_names_the_script_for_an_interpreter() {
        let s = |v: &[&str]| log_stem(&v.iter().map(|x| x.to_string()).collect::<Vec<_>>());
        assert_eq!(
            s(&["/usr/bin/python3", "/x/mixer.py", "--ros-args"]),
            "mixer"
        );
        assert_eq!(s(&["/opt/ros/humble/lib/demo_nodes_cpp/talker"]), "talker");
        assert_eq!(s(&["python3", "-c", "x"]), "python3");
        assert_eq!(s(&[]), "process");
        assert_eq!(s(&["a b"]), "a_b");
    }

    #[test]
    fn each_refusal_names_its_own_cause_the_process_and_its_nodes() {
        let cmd = ["python3".to_string(), "mixer.py".to_string()];
        let m = refusal_message(
            NotReplayable::MultiThreaded,
            &cmd,
            &["mixer".into()],
            "detail",
        );
        assert_eq!(
            m,
            "ros2 process python3 mixer.py (nodes mixer): MultiThreadedExecutor, not replayable \
             (detail)"
        );
        let m = refusal_message(
            NotReplayable::SerializedTake,
            &cmd,
            &["mixer".into()],
            "on /a",
        );
        assert_eq!(
            m,
            "ros2 process python3 mixer.py (nodes mixer): serialized take, not replayable (on /a)"
        );
        assert!(!m.contains("MultiThreadedExecutor"), "{m}");
        let m = refusal_message(NotReplayable::UnorderedThread, &cmd, &[], "x");
        assert!(
            m.contains("(nodes none): publishing thread outside every executor, not replayable"),
            "{m}"
        );
    }

    #[test]
    fn entities_are_named_stably_per_kind_and_topic() {
        let mut r = Registry::default();
        assert_eq!(
            r.add(EntityKind::Subscription, 1, "/a", "n").name,
            "sub:/a#0"
        );
        assert_eq!(
            r.add(EntityKind::Subscription, 2, "/a", "n").name,
            "sub:/a#1"
        );
        assert_eq!(r.add(EntityKind::Publisher, 3, "/a", "n").name, "pub:/a#0");
        assert_eq!(r.add(EntityKind::Guard, 4, "", "").name, "gc#0");
        assert_eq!(r.add(EntityKind::Guard, 5, "", "").name, "gc#1");
        assert_eq!(r.by_key[&3].id, 2);
    }

    #[test]
    fn the_verdict_path_appends_to_the_log_path() {
        assert_eq!(
            verdict_path(Path::new("/d/mixer-1.rdl.mcap")),
            PathBuf::from("/d/mixer-1.rdl.mcap.verdict.json")
        );
    }

    #[test]
    fn captures_are_inert_unless_armed() {
        capture_take_body(b"abc", 1, 2);
        TAKE_CAPTURE.with(|c| assert!(!c.borrow().have));
        capture_output_body(b"abc");
        OUTPUT_CAPTURE.with(|c| assert!(c.get().is_none()));
        {
            // Armed but with the hooks off (this test binary's mode is Off):
            // still inert.
            let _armed = Armed::new(&OUTPUT_ARMED);
            capture_output_body(b"abc");
            OUTPUT_CAPTURE.with(|c| assert!(c.get().is_none()));
            HOOKS_ON.store(true, Ordering::Relaxed);
            capture_output_body(b"abc");
            HOOKS_ON.store(false, Ordering::Relaxed);
        }
        assert!(!OUTPUT_ARMED.with(Cell::get), "the guard disarms on drop");
        assert_eq!(
            OUTPUT_CAPTURE.with(Cell::take),
            Some((log::fnv1a(b"abc"), 3))
        );
    }
}
