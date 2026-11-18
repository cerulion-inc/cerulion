// SPDX-License-Identifier: AGPL-3.0-only
//! The vizd telemetry module: event specs, properties and the heartbeat
//! thread's cadence and prompt stop without a network, then the start,
//! heartbeat, live opt-out, abandon and shutdown lifecycle against a loopback
//! collector. The lifecycle tests set `CERULION_HOME` for the process, so they
//! serialize on one lock and the suite runs with `--test-threads=1`.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime};

use cerulion_telemetry::{consent, guard, rfc3339, Client, Value};
use cerulion_vizd::telemetry::{
    common, heartbeat_props, started_props, Heartbeat, HeartbeatStop, Recorded, StarterEnd,
    Starting, StartingShutdown, Telemetry, HEARTBEAT_INTERVAL, VIZD_HEARTBEAT, VIZD_STARTED,
};

/// The shipped `DEFAULT_SHUTDOWN_BUDGET`, written by hand: the records the
/// stops and shutdowns keep are checked against this value, not against the
/// constant they were built from.
const SHUTDOWN_BUDGET: Duration = Duration::from_millis(300);

/// How long a test waits for an event that must happen (a thread ending, a
/// starter reaching its abandon check), how long a blocked starter or a stuck
/// tick waits for the test to release it, and the budget of the explicit
/// long-interval stop. Never a measure of how long the code took: only a
/// regression that waits on what the test is holding, or never wakes a
/// thread, outlasts it, and then the arm fails instead of hanging.
const HANG_GUARD: Duration = Duration::from_secs(30);

/// The one wall-clock bound in this file, on the two abandon arms' shutdowns:
/// 20 times the [`SHUTDOWN_BUDGET`] the shutdown waits for the start. A
/// starter that held the abandon lock across a wait would block the shutdown
/// with every record still correct, so this bound is the only catch for that
/// hang; scheduler delay adds a small fraction of it to the 300 ms wait.
const ABANDON_LOCK_BOUND: Duration = Duration::from_secs(6);

/// The value `record` holds once its thread has set it, waiting up to
/// [`HANG_GUARD`]; `None` if it is still unset then.
fn recorded<T: Copy>(record: &Recorded<T>) -> Option<T> {
    let deadline = Instant::now() + HANG_GUARD;
    loop {
        if let Some(value) = record.get() {
            return Some(*value);
        }
        if Instant::now() >= deadline {
            return None;
        }
        thread::sleep(Duration::from_millis(5));
    }
}

/// Serializes the tests that set process environment.
static ENV: Mutex<()> = Mutex::new(());

#[test]
fn heartbeat_interval_is_fifteen_minutes() {
    assert_eq!(HEARTBEAT_INTERVAL, Duration::from_secs(900));
}

#[test]
fn event_names_and_common_values_pass_the_guard() {
    assert_eq!(VIZD_STARTED.name, "vizd_started");
    assert_eq!(VIZD_HEARTBEAT.name, "vizd_heartbeat");
    for spec in [VIZD_STARTED, VIZD_HEARTBEAT] {
        guard::check_event_name(spec.name).expect(spec.name);
    }
    let c = common();
    assert_eq!(c.surface, "vizd");
    for value in [&c.surface, &c.env, &c.app_version] {
        guard::check_str(value).expect(value);
    }
}

#[test]
fn started_carries_the_platform() {
    assert_eq!(
        started_props(),
        vec![
            ("os".to_string(), Value::Str(std::env::consts::OS.into())),
            (
                "arch".to_string(),
                Value::Str(std::env::consts::ARCH.into())
            ),
        ]
    );
}

#[test]
fn properties_stay_inside_their_allowlists_and_pass_the_guard() {
    for (spec, props) in [
        (VIZD_STARTED, started_props()),
        (VIZD_HEARTBEAT, heartbeat_props(3 * HEARTBEAT_INTERVAL)),
    ] {
        let (kept, dropped) = guard::filter(props.clone(), spec.allowlist);
        assert!(dropped.is_empty(), "{}: {dropped:?}", spec.name);
        assert_eq!(kept, props, "{}", spec.name);
    }
}

#[test]
fn uptime_is_whole_minutes_of_elapsed_time() {
    let props = heartbeat_props(4 * HEARTBEAT_INTERVAL);
    assert_eq!(props, vec![("uptime_minutes".to_string(), Value::Int(60))]);
    assert_eq!(
        heartbeat_props(Duration::from_secs(179))[0].1,
        Value::Int(2)
    );
    assert_eq!(
        heartbeat_props(Duration::from_secs(180))[0].1,
        Value::Int(3)
    );
    assert_eq!(
        heartbeat_props(Duration::MAX)[0].1,
        Value::Int((u64::MAX / 60) as i64)
    );
}

#[test]
fn heartbeat_ticks_in_order_and_stops_promptly_on_drop() {
    let (tx, rx) = mpsc::channel();
    let beat = Heartbeat::spawn(Duration::from_millis(20), move |n| {
        let _ = tx.send(n);
    })
    .expect("spawn");
    let seen: Vec<u64> = (0..3)
        .map(|_| rx.recv_timeout(Duration::from_secs(5)).expect("tick"))
        .collect();
    assert_eq!(seen, vec![1, 2, 3]);
    drop(beat);
    while rx.try_recv().is_ok() {}
    assert!(
        rx.recv_timeout(Duration::from_millis(100)).is_err(),
        "no tick after drop"
    );
}

/// Dropping a heartbeat whose interval is an hour wakes its thread instead of
/// leaving it to sleep the hour out: the thread ends, which drops the tick
/// closure and the sender it owns, and the drop waited for it with the one
/// shutdown budget. The tick itself never runs.
#[test]
fn dropping_a_long_interval_heartbeat_does_not_wait_for_the_interval() {
    let (alive, ended) = mpsc::channel::<()>();
    let beat = Heartbeat::spawn(Duration::from_secs(3600), move |_| {
        let _ = alive.send(());
    })
    .expect("spawn");
    let stop = beat.stop_record_for_test();
    drop(beat);
    assert_eq!(
        ended.recv_timeout(HANG_GUARD),
        Err(mpsc::RecvTimeoutError::Disconnected),
        "the dropped heartbeat's thread must end, not sleep out its hour"
    );
    assert_eq!(
        stop.get().map(|s| s.budget),
        Some(SHUTDOWN_BUDGET),
        "the drop waits for the thread with the one shutdown budget"
    );
}

/// Stopping a heartbeat whose interval is an hour wakes its thread at once:
/// with a budget of [`HANG_GUARD`], the stop ends by joining a thread that
/// finished, never by the budget running out. A stop that kept the stop
/// channel open while it waited would wait out the whole budget and leave the
/// thread running, and in production every stop would then spend the whole
/// shutdown budget before the client's flush got any of it.
#[test]
fn stopping_a_long_interval_heartbeat_wakes_and_joins_its_thread() {
    let mut beat = Heartbeat::spawn(Duration::from_secs(3600), |_| {}).expect("spawn");
    let stop = beat.stop_record_for_test();
    beat.stop_within(HANG_GUARD);
    assert_eq!(
        stop.get().copied(),
        Some(HeartbeatStop {
            budget: HANG_GUARD,
            joined: true,
        }),
        "the stop wakes the thread, which finishes inside the budget and is joined"
    );
}

/// Dropping a heartbeat whose tick is stuck (a consent read on a hung
/// filesystem) waits the one shutdown budget and leaves the thread running,
/// rather than joining it and waiting the tick out.
///
/// The tick blocks on a channel this test holds and reports when it returns,
/// so the drop is checked against the tick itself as well as against the
/// stop's record: a drop that joined the thread could only return after the
/// tick had returned.
#[test]
fn dropping_a_heartbeat_stuck_in_a_tick_is_bounded() {
    let (entered_tx, entered) = mpsc::channel();
    let (release, held) = mpsc::channel::<()>();
    let (tick_done_tx, tick_done) = mpsc::channel();
    let beat = Heartbeat::spawn(Duration::from_millis(10), move |_| {
        let _ = entered_tx.send(());
        let _ = held.recv_timeout(HANG_GUARD);
        let _ = tick_done_tx.send(());
    })
    .expect("spawn");
    entered.recv_timeout(HANG_GUARD).expect("tick entered");
    let stop = beat.stop_record_for_test();
    drop(beat);
    assert_eq!(
        tick_done.try_recv(),
        Err(mpsc::TryRecvError::Empty),
        "the drop returned while the stuck tick was still running"
    );
    assert_eq!(
        stop.get().copied(),
        Some(HeartbeatStop {
            budget: SHUTDOWN_BUDGET,
            joined: false,
        }),
        "the drop waits one shutdown budget and leaves the stuck tick running"
    );
    release.send(()).expect("the stuck tick is still waiting");
    tick_done
        .recv_timeout(HANG_GUARD)
        .expect("the released tick returns");
}

/// A fresh consent home for one test: `CERULION_HOME` points at an empty
/// directory under the system temp dir, and the environment opt-outs and
/// both network kill-switch variables are cleared, so consent resolves
/// enabled, the switch is off and the anonymous id is minted there. The
/// variable is left set: a starter thread that outlives its test must never
/// fall back to the real home.
fn isolated_home(tag: &str) -> PathBuf {
    let home = std::env::temp_dir().join(format!(
        "cerulion-vizd-telemetry-{}-{tag}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&home);
    std::env::set_var("CERULION_HOME", &home);
    std::env::remove_var("DO_NOT_TRACK");
    std::env::remove_var("CERULION_TELEMETRY");
    std::env::remove_var("CERULION_NETWORK");
    std::env::remove_var("CERULION_VIZD_NETWORK");
    home
}

fn loopback_client(host: &str) -> Client {
    Client::new("phc_test".into(), host, common()).expect("loopback client")
}

/// A loopback `/batch` collector: every POST is answered 200 and its events
/// are read back in arrival order, batch by batch.
struct Collector {
    host: String,
    addr: SocketAddr,
    batches: mpsc::Receiver<serde_json::Value>,
    pending: VecDeque<serde_json::Value>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Collector {
    fn start() -> Collector {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("addr");
        let (tx, batches) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let thread = thread::spawn(move || {
            while !stopping.load(Ordering::Acquire) {
                let Ok((mut stream, _)) = listener.accept() else {
                    break;
                };
                let Some(body) = read_body(&mut stream) else {
                    continue;
                };
                let _ = stream.write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                );
                if tx.send(body).is_err() {
                    break;
                }
            }
        });
        Collector {
            host: format!("http://{addr}"),
            addr,
            batches,
            pending: VecDeque::new(),
            stop,
            thread: Some(thread),
        }
    }

    fn push_batch(&mut self, batch: &serde_json::Value) {
        let events = batch["batch"].as_array().expect("batch array");
        self.pending.extend(events.iter().cloned());
    }

    /// The next event in arrival order, waiting up to `wait` for a batch.
    fn next(&mut self, wait: Duration) -> serde_json::Value {
        if self.pending.is_empty() {
            let batch = self
                .batches
                .recv_timeout(wait)
                .unwrap_or_else(|_| panic!("no event within {wait:?}"));
            self.push_batch(&batch);
        }
        self.pending.pop_front().expect("a batch carries an event")
    }

    /// Every event that arrives until `quiet` passes with no new batch, or
    /// `at_most` has elapsed, in arrival order.
    fn settled(&mut self, quiet: Duration, at_most: Duration) -> Vec<serde_json::Value> {
        let deadline = Instant::now() + at_most;
        loop {
            let wait = quiet.min(deadline.saturating_duration_since(Instant::now()));
            let Ok(batch) = self.batches.recv_timeout(wait) else {
                break;
            };
            self.push_batch(&batch);
        }
        self.pending.drain(..).collect()
    }
}

impl Drop for Collector {
    /// Raise the stop flag, then knock on the listener so a blocked `accept`
    /// returns and the thread sees it.
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        drop(TcpStream::connect(self.addr));
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// One HTTP/1.1 request body as JSON; `None` for a connection that closes
/// before a complete request (the stop knock).
fn read_body(stream: &mut TcpStream) -> Option<serde_json::Value> {
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .expect("timeout");
    let mut buf = Vec::new();
    let mut chunk = [0_u8; 4096];
    let (body_start, content_length) = loop {
        let n = stream.read(&mut chunk).ok()?;
        if n == 0 {
            return None;
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(pos) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            let head = String::from_utf8_lossy(&buf[..pos]);
            let len = head.lines().find_map(|l| {
                let (k, v) = l.split_once(':')?;
                k.eq_ignore_ascii_case("content-length")
                    .then(|| v.trim().parse::<usize>().ok())?
            })?;
            break (pos + 4, len);
        }
    };
    while buf.len() < body_start + content_length {
        let n = stream.read(&mut chunk).ok()?;
        if n == 0 {
            return None;
        }
        buf.extend_from_slice(&chunk[..n]);
    }
    serde_json::from_slice(&buf[body_start..body_start + content_length]).ok()
}

#[test]
fn start_sends_started_then_heartbeats_until_a_live_opt_out_and_stops_within_budget() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("lifecycle");
    let mut collector = Collector::start();
    let telemetry =
        Telemetry::start_with(loopback_client(&collector.host), Duration::from_millis(20))
            .expect("start");

    // The started event is the first thing on the wire, then the beats.
    let started = collector.next(Duration::from_secs(5));
    assert_eq!(started["event"], "vizd_started");
    let anon_id = started["distinct_id"].as_str().expect("distinct_id");
    assert!(anon_id.starts_with("anon:"), "{anon_id}");
    assert_eq!(started["properties"]["os"], std::env::consts::OS);
    assert_eq!(started["properties"]["arch"], std::env::consts::ARCH);
    assert_eq!(started["properties"]["surface"], "vizd");
    assert_eq!(started["properties"]["$process_person_profile"], false);

    let beat = collector.next(Duration::from_secs(5));
    assert_eq!(beat["event"], "vizd_heartbeat");
    assert_eq!(beat["distinct_id"], anon_id, "one id for the whole run");
    assert_eq!(beat["properties"]["uptime_minutes"], 0);

    consent::set_enabled(false).expect("opt out");
    // Every event is stamped as it is queued, under the consent lock the
    // opt-out wrote under, so a beat that lands after the opt-out returned
    // is legitimate only if it was stamped before. Collecting until a full
    // quiet second has passed gives a late beat fifty chances to arrive.
    let cutoff = rfc3339::format(SystemTime::now());
    let after = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(
        after.iter().all(|e| e["event"] == "vizd_heartbeat"),
        "only heartbeats follow the start: {after:?}"
    );
    let late: Vec<&str> = after
        .iter()
        .filter_map(|e| e["timestamp"].as_str())
        .filter(|stamp| *stamp > cutoff.as_str())
        .collect();
    assert!(
        late.is_empty(),
        "heartbeats stamped after the opt-out: {late:?}"
    );

    // Stopping within budget is a property of what the shutdown hands out: the
    // heartbeat's stop gets at most the one budget, and the client's flush,
    // handed what is left once the heartbeat has stopped, gets less than a full
    // budget, so never a fresh one.
    let record = telemetry.shutdown_record_for_test();
    telemetry.shutdown();
    let shut = record
        .get()
        .copied()
        .expect("the shutdown records what it handed out");
    let heartbeat = shut
        .heartbeat_budget
        .expect("the shutdown stops the heartbeat");
    let client = shut.client_budget.expect("the shutdown flushes the client");
    assert!(
        heartbeat <= SHUTDOWN_BUDGET,
        "the heartbeat's stop gets at most the one budget: {shut:?}"
    );
    assert!(
        client < SHUTDOWN_BUDGET,
        "the client's flush is handed less than a full budget, never a fresh one: {shut:?}"
    );
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

#[test]
fn a_start_still_blocked_at_the_deadline_is_abandoned_and_sends_nothing() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("abandon");
    let mut collector = Collector::start();
    let host = collector.host.clone();
    let (release, blocked) = mpsc::channel::<()>();
    let starting = Starting::spawn_with(
        move || {
            blocked.recv_timeout(HANG_GUARD).ok()?;
            Some(loopback_client(&host))
        },
        Duration::from_millis(20),
    );
    let records = starting.records_for_test();

    let start = Instant::now();
    starting.shutdown();
    let took = start.elapsed();
    assert_eq!(
        records.shutdown.get().copied(),
        Some(StartingShutdown::Abandoned {
            waited: SHUTDOWN_BUDGET,
            late: false,
        }),
        "shutdown waits one budget for the blocked start, then abandons it"
    );
    // A starter that held the abandon lock across a wait would block this
    // shutdown with the record above still correct, so this bound is the only
    // catch for that hang. It is 20 times the budget the shutdown waits, far
    // above what scheduler delay adds to that wait.
    assert!(took < ABANDON_LOCK_BOUND, "shutdown took {took:?}");

    // The start finishes only now, with a client that would send: it finds
    // the abandon flag set and queues nothing.
    release.send(()).expect("starter still waiting");
    assert_eq!(
        recorded(&records.starter),
        Some(StarterEnd::AbandonedBeforeBegin),
        "the released start returns at its abandon check, before it queues anything"
    );
    let sent = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(sent.is_empty(), "events from an abandoned start: {sent:?}");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

#[test]
fn a_start_blocked_on_the_consent_lock_at_the_deadline_is_abandoned_and_sends_nothing() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("abandon-locked");
    // Mint the consent file first, so the lock file exists to be held.
    consent::anon_id().expect("mint the consent file");
    let mut collector = Collector::start();
    let host = collector.host.clone();
    // Hold the consent lock as another process would: the starter has its
    // client, then blocks on this lock for the anonymous id.
    let held = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(home.join("telemetry.json.lock"))
        .expect("open the consent lock file");
    held.lock().expect("hold the consent lock");
    let (client_built, built) = mpsc::channel::<()>();
    let starting = Starting::spawn_with(
        move || {
            let client = loopback_client(&host);
            client_built.send(()).expect("test waits for the client");
            Some(client)
        },
        Duration::from_millis(20),
    );
    built.recv().expect("starter built its client");
    let records = starting.records_for_test();

    // On a helper thread, so a shutdown that blocks on what this test holds
    // is caught: the starter waits for the consent lock until the test drops
    // it, so a starter holding the abandon lock across that wait, or a
    // shutdown that waits for the start, would otherwise deadlock here. Past
    // ABANDON_LOCK_BOUND the lock is let go, the shutdown is joined if it then
    // returns within HANG_GUARD, and the arm fails.
    let (shut_tx, shut) = mpsc::channel::<()>();
    let shutting = thread::spawn(move || {
        starting.shutdown();
        let _ = shut_tx.send(());
    });
    match shut.recv_timeout(ABANDON_LOCK_BOUND) {
        Ok(()) => shutting.join().expect("the shutdown thread"),
        // The helper ended without reporting, so it panicked: carry that through.
        Err(mpsc::RecvTimeoutError::Disconnected) => match shutting.join() {
            Err(panic) => std::panic::resume_unwind(panic),
            Ok(()) => panic!("the shutdown thread ended without reporting"),
        },
        Err(mpsc::RecvTimeoutError::Timeout) => {
            // Let the lock go; the helper is joined only if it then reports, so
            // a starter that never lets the abandon lock go cannot hang the arm.
            drop(held);
            if shut.recv_timeout(HANG_GUARD).is_ok() {
                let _ = shutting.join();
            }
            panic!(
                "shutdown did not return within {ABANDON_LOCK_BOUND:?} on a start blocked \
                 by the consent lock"
            );
        }
    }
    assert_eq!(
        records.shutdown.get().copied(),
        Some(StartingShutdown::Abandoned {
            waited: SHUTDOWN_BUDGET,
            late: false,
        }),
        "shutdown waits one budget for the start blocked by the consent lock, \
         then abandons it"
    );

    // The lock is released only now, with a client that would send: the
    // start finds itself abandoned and queues nothing.
    drop(held);
    assert_eq!(
        recorded(&records.starter),
        Some(StarterEnd::AbandonedBeforeBegin),
        "the released start returns at its abandon check, before it queues anything"
    );
    let sent = collector.settled(Duration::from_secs(1), Duration::from_secs(5));
    assert!(sent.is_empty(), "events from an abandoned start: {sent:?}");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

/// Clears the CI variables and puts them back on drop, passing or failing.
struct CiVarsCleared(Vec<(&'static str, std::ffi::OsString)>);

impl CiVarsCleared {
    fn new() -> Self {
        let saved = cerulion_telemetry::ci::CI_ENV_VARS
            .iter()
            .filter_map(|name| Some((*name, std::env::var_os(name)?)))
            .collect();
        for name in cerulion_telemetry::ci::CI_ENV_VARS {
            std::env::remove_var(name);
        }
        CiVarsCleared(saved)
    }
}

impl Drop for CiVarsCleared {
    fn drop(&mut self) {
        for (name, value) in self.0.drain(..) {
            std::env::set_var(name, value);
        }
    }
}

#[test]
fn the_daemon_sends_nothing_until_the_cli_has_shown_its_notice() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("notice");
    let _restore = CiVarsCleared::new();
    let mut collector = Collector::start();
    std::env::set_var("POSTHOG_API_KEY", "phc_test");
    std::env::set_var("POSTHOG_HOST", &collector.host);

    assert!(
        Telemetry::start().is_none(),
        "no notice shown on this machine yet, so the daemon starts no telemetry"
    );
    consent::mark_notice_shown().expect("notice shown");
    let telemetry = Telemetry::start().expect("starts once the notice was shown");
    let started = collector.next(Duration::from_secs(5));
    assert_eq!(started["event"], "vizd_started");
    assert_eq!(started["properties"]["ci"], false);

    telemetry.shutdown();
    std::env::remove_var("POSTHOG_API_KEY");
    std::env::remove_var("POSTHOG_HOST");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

/// The network kill switch stops the daemon's telemetry: `CERULION_NETWORK`
/// or `CERULION_VIZD_NETWORK` set to anything that is not blank, under the
/// CLI's fail-closed parse, starts no client, so no `vizd_started` and no
/// heartbeat reach the collector. A blank value leaves the switch off.
#[test]
fn the_daemon_sends_nothing_under_the_network_kill_switch() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("switch");
    let _restore = CiVarsCleared::new();
    let mut collector = Collector::start();
    std::env::set_var("POSTHOG_API_KEY", "phc_test");
    std::env::set_var("POSTHOG_HOST", &collector.host);
    consent::mark_notice_shown().expect("notice shown");

    for (name, value) in [
        ("CERULION_NETWORK", "off"),
        ("CERULION_NETWORK", " OFF "),
        ("CERULION_NETWORK", "typo"),
        ("CERULION_VIZD_NETWORK", "off"),
        ("CERULION_VIZD_NETWORK", "OFF"),
    ] {
        std::env::set_var(name, value);
        assert!(
            Telemetry::start().is_none(),
            "{name}={value:?}: the daemon starts no telemetry under the switch"
        );
        Telemetry::start_in_background().shutdown();
        std::env::remove_var(name);
    }
    let sent = collector.settled(Duration::from_millis(500), Duration::from_secs(3));
    assert!(sent.is_empty(), "sent under the switch: {sent:?}");

    std::env::set_var("CERULION_NETWORK", "  ");
    let telemetry = Telemetry::start().expect("a blank value leaves the switch off");
    let started = collector.next(Duration::from_secs(5));
    assert_eq!(started["event"], "vizd_started");
    telemetry.shutdown();

    std::env::remove_var("CERULION_NETWORK");
    std::env::remove_var("POSTHOG_API_KEY");
    std::env::remove_var("POSTHOG_HOST");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}

/// The heartbeat stops with the switch too: a background start on the
/// production path, beating every 20 ms, sends no `vizd_started` and no
/// `vizd_heartbeat` under the switch; the same start without it sends both.
#[test]
fn no_heartbeat_reaches_the_collector_under_the_network_kill_switch() {
    let _env = ENV.lock().unwrap_or_else(|p| p.into_inner());
    let home = isolated_home("switch-beat");
    let _restore = CiVarsCleared::new();
    let mut collector = Collector::start();
    std::env::set_var("POSTHOG_API_KEY", "phc_test");
    std::env::set_var("POSTHOG_HOST", &collector.host);
    consent::mark_notice_shown().expect("notice shown");
    let beat = Duration::from_millis(20);

    std::env::set_var("CERULION_NETWORK", "off");
    let starting = Telemetry::start_in_background_every(beat);
    std::thread::sleep(Duration::from_millis(300));
    starting.shutdown();
    let sent = collector.settled(Duration::from_millis(500), Duration::from_secs(3));
    assert!(sent.is_empty(), "sent under the switch: {sent:?}");

    std::env::remove_var("CERULION_NETWORK");
    let starting = Telemetry::start_in_background_every(beat);
    let started = collector.next(Duration::from_secs(5));
    assert_eq!(started["event"], "vizd_started");
    let beat_event = collector.next(Duration::from_secs(5));
    assert_eq!(beat_event["event"], "vizd_heartbeat");
    // The heartbeat beats before the starter hands over, so the handover is
    // waited for on the starter's record, which is set once it is decided.
    // Then the shutdown takes the handed over branch, and the client's flush
    // is handed less than a full budget, so never a fresh one.
    let records = starting.records_for_test();
    assert_eq!(
        recorded(&records.starter),
        Some(StarterEnd::WentOn),
        "the start went on and handed over"
    );
    starting.shutdown();
    assert_eq!(
        records.shutdown.get().copied(),
        Some(StartingShutdown::ShutDown),
        "a start that handed over is shut down, not abandoned"
    );
    let handed_over = records
        .handed_over
        .get()
        .copied()
        .expect("the handed over telemetry is shut down by the deadline");
    assert!(
        handed_over
            .client_budget
            .is_some_and(|client| client < SHUTDOWN_BUDGET),
        "the client's flush is handed less than a full budget: {handed_over:?}"
    );

    std::env::remove_var("POSTHOG_API_KEY");
    std::env::remove_var("POSTHOG_HOST");
    drop(collector);
    let _ = std::fs::remove_dir_all(home);
}
