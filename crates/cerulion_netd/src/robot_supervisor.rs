// SPDX-License-Identifier: AGPL-3.0-only
//! One automatic registration attempt and one owned robot-server child.
//!
//! LAN startup never waits for account HTTP or this supervisor. Failure leaves
//! LAN serving; shutdown kills and reaps the child currently owned by the worker.

use std::io::{self, Read};
use std::path::{Component, Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::Deserialize;

const BOOTSTRAP_TIMEOUT: Duration = Duration::from_secs(120);
const POLL_INTERVAL: Duration = Duration::from_millis(50);
const FACT_REFRESH_WINDOW: Duration = Duration::from_secs(10);
const FACT_REFRESH_INTERVAL: Duration = Duration::from_millis(250);
const MAX_RESULT_BYTES: u64 = 16 * 1024;
/// Attempts of the registration/writer/server sequence before the supervisor
/// stops retrying for the life of this daemon.
const MAX_ATTEMPTS: u32 = 30;
const FIRST_RETRY_DELAY: Duration = Duration::from_secs(5);
/// A sequence that stayed up this long counts as healthy: its failure starts
/// the attempt count again from 1.
const HEALTHY_RUN: Duration = Duration::from_secs(600);

/// The attempt number of a failure: `attempt` (1-based) unless the failed
/// sequence ran for at least [`HEALTHY_RUN`], which makes it attempt 1.
fn counted_attempt(attempt: u32, ran_for: Duration) -> u32 {
    if ran_for >= HEALTHY_RUN {
        1
    } else {
        attempt
    }
}
const MAX_RETRY_DELAY: Duration = Duration::from_secs(300);

/// The wait before attempt `attempt + 1` after attempt `attempt` (1-based)
/// failed: 5 s doubling to a 300 s cap; `None` once [`MAX_ATTEMPTS`] are spent.
fn retry_delay(attempt: u32) -> Option<Duration> {
    if attempt >= MAX_ATTEMPTS {
        return None;
    }
    let doublings = attempt.saturating_sub(1).min(16);
    Some(
        FIRST_RETRY_DELAY
            .saturating_mul(1u32 << doublings)
            .min(MAX_RETRY_DELAY),
    )
}

/// Sleep for `delay` in [`POLL_INTERVAL`] steps; `false` when `running` cleared.
fn wait_while_running(running: &AtomicBool, delay: Duration) -> bool {
    let until = Instant::now() + delay;
    while running.load(Ordering::SeqCst) {
        let now = Instant::now();
        if now >= until {
            return true;
        }
        std::thread::sleep(POLL_INTERVAL.min(until - now));
    }
    false
}

/// Background owner of the registration/writer/server process sequence.
pub struct RobotSupervisor {
    running: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl RobotSupervisor {
    /// Start once, after netd has acquired its singleton and opened its LAN plane.
    pub fn start(
        running: Arc<AtomicBool>,
        gateway: Arc<crate::GatewayEgressPlane>,
    ) -> io::Result<Self> {
        let worker_running = Arc::clone(&running);
        let thread = std::thread::Builder::new()
            .name("netd-robot-supervisor".into())
            .spawn(move || {
                let mut attempt = 0u32;
                while worker_running.load(Ordering::SeqCst) {
                    attempt += 1;
                    let started = Instant::now();
                    let result = run(&worker_running, |eid| {
                        gateway
                            .refresh_robot_beacon(eid)
                            .map_err(|error| error.to_string())
                    });
                    let Err(error) = result else { return };
                    attempt = counted_attempt(attempt, started.elapsed());
                    if !worker_running.load(Ordering::SeqCst) {
                        return;
                    }
                    match retry_delay(attempt) {
                        Some(delay) => {
                            tracing::warn!(
                                error = %error,
                                attempt,
                                retry_in_s = delay.as_secs(),
                                "robot WAN startup failed; LAN remains available"
                            );
                            if !wait_while_running(&worker_running, delay) {
                                return;
                            }
                        }
                        None => {
                            tracing::error!(
                                error = %error,
                                attempts = attempt,
                                "robot WAN startup failed and will not be retried until \
                                 cerulion-netd restarts; LAN remains available"
                            );
                            return;
                        }
                    }
                }
            })?;
        Ok(Self {
            running,
            thread: Some(thread),
        })
    }

    /// Stop and reap the owned child before netd tears down its LAN plane.
    pub fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(thread) = self.thread.take() {
            if thread.join().is_err() {
                tracing::error!("robot supervisor thread panicked during shutdown");
            }
        }
    }
}

impl Drop for RobotSupervisor {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Prepared {
    version: u16,
    registration_bundle: PathBuf,
    robot_id: String,
    endpoint_id: String,
    owner_account: String,
}

#[derive(Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct Provisioned {
    version: u16,
    robot_id: String,
    endpoint_id: String,
    owner_account: String,
}

fn absolute_without_traversal(path: &Path) -> bool {
    path.is_absolute()
        && !path
            .components()
            .any(|part| matches!(part, Component::ParentDir | Component::CurDir))
}

fn is_identity(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_prepared(prepared: &Prepared, root: &Path) -> Result<(), String> {
    if prepared.version != 1
        || !is_identity(&prepared.robot_id)
        || !is_identity(&prepared.endpoint_id)
        || !is_identity(&prepared.owner_account)
        || !absolute_without_traversal(&prepared.registration_bundle)
        || !prepared.registration_bundle.starts_with(root)
    {
        return Err("registration worker returned an invalid identity or bundle path".into());
    }
    Ok(())
}

fn run(
    running: &AtomicBool,
    refresh_facts: impl Fn(&str) -> Result<bool, String>,
) -> Result<(), String> {
    let root = cerulion_discovery::robot_state::resolve()
        .ok_or_else(|| "no home directory is available for robot state".to_string())?;
    if !absolute_without_traversal(&root) {
        return Err("robot state root must be absolute and contain no traversal".into());
    }
    let current = std::env::current_exe().map_err(|error| error.to_string())?;
    let canonical = std::fs::canonicalize(&current).ok();
    let cli = resolve_sibling("cerulion", &current, canonical.as_deref(), is_executable)?;
    let remoted = resolve_sibling(
        "cerulion-remoted",
        &current,
        canonical.as_deref(),
        is_executable,
    )?;
    let deadline = Instant::now() + BOOTSTRAP_TIMEOUT;
    let mut registration = Command::new(&cli);
    registration
        .arg("bootstrap-robot")
        .arg("--state-root")
        .arg(&root);
    let prepared: Prepared = run_json(registration, "account registration", deadline, running)?;
    validate_prepared(&prepared, &root)?;
    let mut writer = Command::new(&remoted);
    writer
        .arg("--state-root")
        .arg(&root)
        .arg("--provision-bundle")
        .arg(&prepared.registration_bundle);
    let provisioned: Provisioned = run_json(writer, "robot state writer", deadline, running)?;
    if provisioned
        != (Provisioned {
            version: 1,
            robot_id: prepared.robot_id,
            endpoint_id: prepared.endpoint_id,
            owner_account: prepared.owner_account,
        })
    {
        return Err("robot state writer returned a different registered identity".into());
    }
    if !running.load(Ordering::SeqCst) {
        return Ok(());
    }
    let mut command = Command::new(remoted);
    command
        .arg("--state-root")
        .arg(&root)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit());
    let mut child = OwnedChild(
        command
            .spawn()
            .map_err(|error| format!("starting robot server: {error}"))?,
    );
    tracing::info!(endpoint_id = %provisioned.endpoint_id, "registered robot server started");
    let period = epoch_sync_period_from_env();
    let mut epoch_sync = EpochSync::new(cli, root.clone(), period, Instant::now());
    let result = supervise_server(
        &mut child,
        &mut epoch_sync,
        running,
        &provisioned.endpoint_id,
        &refresh_facts,
    );
    epoch_sync.stop();
    result
}

/// While `running`: watch the server child (its exit is an error), advance the
/// revocation sync, and refresh the LAN advertisement's endpoint facts during
/// the first [`FACT_REFRESH_WINDOW`].
fn supervise_server(
    child: &mut OwnedChild,
    epoch_sync: &mut EpochSync,
    running: &AtomicBool,
    endpoint_id: &str,
    refresh_facts: &impl Fn(&str) -> Result<bool, String>,
) -> Result<(), String> {
    let refresh_deadline = Instant::now() + FACT_REFRESH_WINDOW;
    let mut next_refresh = Instant::now();
    let mut refresh_failed = false;
    while running.load(Ordering::SeqCst) {
        if let Some(status) = child.0.try_wait().map_err(|error| error.to_string())? {
            return Err(format!("robot server exited ({status})"));
        }
        let now = Instant::now();
        epoch_sync.poll(now);
        if !refresh_failed && now < refresh_deadline && now >= next_refresh {
            match refresh_facts(endpoint_id) {
                Ok(true) => {
                    tracing::info!(%endpoint_id, "robot endpoint facts refreshed in the LAN advertisement")
                }
                Ok(false) => {}
                Err(error) => {
                    refresh_failed = true;
                    tracing::warn!(error = %error, "robot endpoint facts refresh failed; LAN advertisement remains available");
                }
            }
            next_refresh = now + FACT_REFRESH_INTERVAL;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
    Ok(())
}

/// The environment variable setting how often a serving robot fetches its own
/// revocation epoch, in whole seconds ([`MIN_EPOCH_SYNC_PERIOD`] to
/// [`MAX_EPOCH_SYNC_PERIOD`]).
pub const EPOCH_SYNC_PERIOD_ENV: &str = "CERULION_NETD_EPOCH_SYNC_PERIOD_S";
/// The default fetch period. It bounds the revocation lag for a key that stays
/// away, at one HTTPS request per robot per period to the account service.
const DEFAULT_EPOCH_SYNC_PERIOD: Duration = Duration::from_secs(300);
/// The shortest accepted period: below it the fetch would run about as often as
/// the early fetch a connecting key already triggers.
const MIN_EPOCH_SYNC_PERIOD: Duration = Duration::from_secs(30);
/// The longest accepted period (one day).
const MAX_EPOCH_SYNC_PERIOD: Duration = Duration::from_secs(86_400);

/// The fetch period from an [`EPOCH_SYNC_PERIOD_ENV`] value: whole seconds
/// within the accepted range, else [`DEFAULT_EPOCH_SYNC_PERIOD`] with a warning.
pub fn resolve_epoch_sync_period(env_value: Option<&str>) -> Duration {
    let Some(value) = env_value.map(str::trim).filter(|v| !v.is_empty()) else {
        return DEFAULT_EPOCH_SYNC_PERIOD;
    };
    match value.parse::<u64>().map(Duration::from_secs) {
        Ok(period) if (MIN_EPOCH_SYNC_PERIOD..=MAX_EPOCH_SYNC_PERIOD).contains(&period) => period,
        _ => {
            tracing::warn!(
                "{EPOCH_SYNC_PERIOD_ENV}={value:?} is not whole seconds from {} to {}; using \
                 the default {} s",
                MIN_EPOCH_SYNC_PERIOD.as_secs(),
                MAX_EPOCH_SYNC_PERIOD.as_secs(),
                DEFAULT_EPOCH_SYNC_PERIOD.as_secs()
            );
            DEFAULT_EPOCH_SYNC_PERIOD
        }
    }
}

/// The fetch period this process's environment sets ([`EPOCH_SYNC_PERIOD_ENV`]).
fn epoch_sync_period_from_env() -> Duration {
    resolve_epoch_sync_period(std::env::var(EPOCH_SYNC_PERIOD_ENV).ok().as_deref())
}
/// The bound on one revocation fetch: above the worker's own worst case (a
/// 30 s wait for the session refresh lock, a 30 s refresh request, a 30 s
/// access request), so the deadline never cuts a refresh in flight.
const EPOCH_SYNC_DEADLINE: Duration = Duration::from_secs(120);
/// How long shutdown waits for a running fetch before stopping it.
const EPOCH_SYNC_STOP_GRACE: Duration = Duration::from_secs(5);
/// The shortest gap between two fetches started by connection requests: caps
/// the account-service load connecting keys can cause at 12 requests a minute.
const EPOCH_EARLY_FETCH_GAP: Duration = Duration::from_secs(5);
/// The first wait after a failed fetch, doubling per consecutive failure, so a
/// brief outage costs one short delay and a long one backs off.
const EPOCH_SYNC_FIRST_RETRY: Duration = Duration::from_secs(5);
/// The longest wait between failed fetches (the default period); fetching
/// never stops.
const EPOCH_SYNC_MAX_RETRY: Duration = Duration::from_secs(300);
/// The file remoted writes to ask for an early fetch
/// (`cerulion_remoted::epoch_pull::EPOCH_REQUEST_FILE`).
const EPOCH_REQUEST_FILE: &str = "epoch.request";
const REMOTED_SUBDIR: &str = "remoted";

/// The wait after `failures` consecutive failed fetches (at least 1), capped
/// at the configured `period`.
fn epoch_sync_delay(failures: u32, period: Duration) -> Duration {
    EPOCH_SYNC_FIRST_RETRY
        .saturating_mul(1u32 << failures.saturating_sub(1).min(16))
        .min(EPOCH_SYNC_MAX_RETRY)
        .min(period)
}

/// The robot's revocation sync: runs the hidden `robot-sync-epoch` worker at
/// start, every configured period ([`EPOCH_SYNC_PERIOD_ENV`]), and early (at most every
/// [`EPOCH_EARLY_FETCH_GAP`]) when remoted asks; backs off on failure. A
/// failure never stops the robot: remoted applies only a verified epoch.
struct EpochSync {
    cli: PathBuf,
    root: PathBuf,
    period: Duration,
    child: Option<(OwnedChild, Instant)>,
    next: Instant,
    last_start: Option<Instant>,
    failures: u32,
}

impl EpochSync {
    fn new(cli: PathBuf, root: PathBuf, period: Duration, now: Instant) -> Self {
        Self {
            cli,
            root,
            period,
            child: None,
            next: now,
            last_start: None,
            failures: 0,
        }
    }

    /// Advance at `now`: reap a finished or overdue worker, honor a request,
    /// start the next fetch when due. Never blocks.
    fn poll(&mut self, now: Instant) {
        if let Some((child, deadline)) = &mut self.child {
            let finished = match child.0.try_wait() {
                Ok(Some(status)) => Some(status.success()),
                Ok(None) if now >= *deadline => Some(false),
                Ok(None) => None,
                Err(_) => Some(false),
            };
            match finished {
                None => return,
                Some(ok) => {
                    // Dropping the child kills an overdue worker and reaps it.
                    self.child = None;
                    self.finish(ok, now);
                }
            }
        }
        let request = self.root.join(REMOTED_SUBDIR).join(EPOCH_REQUEST_FILE);
        if std::fs::remove_file(&request).is_ok() && self.failures == 0 {
            let earliest = self
                .last_start
                .map_or(now, |last| last + EPOCH_EARLY_FETCH_GAP);
            self.next = self.next.min(earliest.max(now));
        }
        if now >= self.next {
            self.start(now);
        }
    }

    fn start(&mut self, now: Instant) {
        self.last_start = Some(now);
        let spawned = Command::new(&self.cli)
            .arg("robot-sync-epoch")
            .arg("--state-root")
            .arg(&self.root)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit())
            .spawn();
        match spawned {
            Ok(child) => self.child = Some((OwnedChild(child), now + EPOCH_SYNC_DEADLINE)),
            Err(error) => {
                tracing::warn!(error = %error, "robot revocation sync could not start");
                self.finish(false, now);
            }
        }
    }

    /// Shutdown: give a running fetch [`EPOCH_SYNC_STOP_GRACE`] to finish, then
    /// stop it (dropping the child kills and reaps it).
    fn stop(&mut self) {
        let Some((mut child, _)) = self.child.take() else {
            return;
        };
        let until = Instant::now() + EPOCH_SYNC_STOP_GRACE;
        while Instant::now() < until {
            if !matches!(child.0.try_wait(), Ok(None)) {
                return;
            }
            std::thread::sleep(POLL_INTERVAL);
        }
        drop(child);
    }

    fn finish(&mut self, ok: bool, now: Instant) {
        if ok {
            self.failures = 0;
            self.next = now + self.period;
        } else {
            self.failures = self.failures.saturating_add(1);
            let delay = epoch_sync_delay(self.failures, self.period);
            tracing::warn!(
                failures = self.failures,
                retry_in_s = delay.as_secs(),
                "robot revocation sync failed; the robot keeps its current access list"
            );
            self.next = now + delay;
        }
    }
}

struct OwnedChild(Child);

impl Drop for OwnedChild {
    fn drop(&mut self) {
        if !matches!(self.0.try_wait(), Ok(Some(_))) {
            let _ = self.0.kill();
        }
        if let Err(error) = self.0.wait() {
            tracing::error!(error = %error, "could not reap the owned robot child");
        }
    }
}

fn run_json<T: serde::de::DeserializeOwned>(
    mut command: Command,
    stage: &str,
    deadline: Instant,
    running: &AtomicBool,
) -> Result<T, String> {
    if !running.load(Ordering::SeqCst) || Instant::now() >= deadline {
        return Err(format!(
            "{stage} cancelled or exceeded the bootstrap deadline"
        ));
    }
    command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit());
    let mut child = OwnedChild(
        command
            .spawn()
            .map_err(|error| format!("{stage}: {error}"))?,
    );
    let stdout = child
        .0
        .stdout
        .take()
        .ok_or_else(|| format!("{stage}: no result pipe"))?;
    let (sender, receiver) = mpsc::sync_channel(1);
    // Both child entry points are leaf workers. A concurrent bounded reader
    // prevents pipe backpressure; dropping/killing the owned child closes its EOF.
    let reader = std::thread::Builder::new()
        .name("robot-bootstrap-result".into())
        .spawn(move || {
            let mut bytes = Vec::new();
            let result = stdout
                .take(MAX_RESULT_BYTES + 1)
                .read_to_end(&mut bytes)
                .map(|_| bytes);
            let _ = sender.send(result);
        })
        .map_err(|error| format!("{stage} result reader: {error}"))?;
    let status = wait_child(&mut child.0, stage, deadline, running);
    drop(child);
    // A cancelled leaf worker has been reaped above. Bound pipe cleanup too,
    // so shutdown cannot spend the remaining registration timeout waiting on I/O.
    let pipe_deadline = deadline.min(Instant::now() + Duration::from_millis(250));
    let output = receiver.recv_timeout(pipe_deadline.saturating_duration_since(Instant::now()));
    if output.is_ok() {
        reader
            .join()
            .map_err(|_| format!("{stage} result reader panicked"))?;
    }
    let status = status?;
    let bytes = output
        .map_err(|_| format!("{stage} result did not close before the bootstrap deadline"))?
        .map_err(|error| format!("{stage} result: {error}"))?;
    if !status.success() {
        return Err(format!(
            "{stage} exited ({status}); see the child error above"
        ));
    }
    if bytes.len() as u64 > MAX_RESULT_BYTES {
        return Err(format!("{stage} result exceeded the size limit"));
    }
    serde_json::from_slice(&bytes)
        .map_err(|_| format!("{stage} returned an invalid machine result"))
}

fn wait_child(
    child: &mut Child,
    stage: &str,
    deadline: Instant,
    running: &AtomicBool,
) -> Result<ExitStatus, String> {
    loop {
        if let Some(status) = child
            .try_wait()
            .map_err(|error| format!("{stage}: {error}"))?
        {
            return Ok(status);
        }
        if !running.load(Ordering::SeqCst) || Instant::now() >= deadline {
            return Err(format!(
                "{stage} cancelled or exceeded the bootstrap deadline"
            ));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn is_executable(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    std::fs::metadata(path)
        .is_ok_and(|meta| meta.is_file() && meta.permissions().mode() & 0o111 != 0)
}

fn resolve_sibling(
    name: &str,
    current: &Path,
    canonical: Option<&Path>,
    exists: impl Fn(&Path) -> bool,
) -> Result<PathBuf, String> {
    let mut tried = Vec::new();
    for executable in [Some(current), canonical].into_iter().flatten() {
        if let Some(path) = executable.parent().map(|parent| parent.join(name)) {
            if tried.contains(&path) {
                continue;
            }
            if exists(&path) {
                return Ok(path);
            }
            tried.push(path);
        }
    }
    Err(format!(
        "no executable {name} beside netd; tried {}",
        tried
            .iter()
            .map(|path| path.display().to_string())
            .collect::<Vec<_>>()
            .join(", ")
    ))
}

#[cfg(test)]
mod tests;
