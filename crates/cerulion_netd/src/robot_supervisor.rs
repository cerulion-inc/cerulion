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
                if let Err(error) = run(&worker_running, |eid| gateway.refresh_robot_beacon(eid).map_err(|error| error.to_string())) {
                    if worker_running.load(Ordering::SeqCst) {
                        tracing::error!(error = %error, "robot WAN startup failed; LAN remains available");
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
    let mut registration = Command::new(cli);
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
    let refresh_deadline = Instant::now() + FACT_REFRESH_WINDOW;
    let mut next_refresh = Instant::now();
    let mut refresh_failed = false;
    while running.load(Ordering::SeqCst) {
        if let Some(status) = child.0.try_wait().map_err(|error| error.to_string())? {
            return Err(format!(
                "robot server exited ({status}); automatic restart is disabled"
            ));
        }
        let now = Instant::now();
        if !refresh_failed && now < refresh_deadline && now >= next_refresh {
            match refresh_facts(&provisioned.endpoint_id) {
                Ok(true) => {
                    tracing::info!(endpoint_id = %provisioned.endpoint_id, "robot endpoint facts refreshed in the LAN advertisement")
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
