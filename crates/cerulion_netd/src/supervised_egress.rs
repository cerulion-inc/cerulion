// SPDX-License-Identifier: AGPL-3.0-only
//! Connection-scoped ownership of the machine's authenticated remote server.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_core::{GatewayPlan, SchemaServing};

use crate::egress::{EgressError, EgressId, EgressPlane, GatewayEgressPlane};
use crate::mirror::MirrorPlane;
use crate::robot_supervisor::RobotSupervisor;

struct State {
    registrations: BTreeSet<EgressId>,
    supervisor: Option<RobotSupervisor>,
    stopped: bool,
}

/// Production egress wrapper. Query-only daemons construct it without starting
/// a server. First successful egress starts one; last release reaps it after
/// admitted work, owed e-stop delivery and its latch have cleared. An
/// explicitly configured standing gateway retains it until shutdown.
pub struct SupervisedEgressPlane {
    gateway: Arc<GatewayEgressPlane>,
    mirror: Arc<dyn MirrorPlane>,
    standing: bool,
    holds_endpoint: Arc<AtomicBool>,
    stop_grace: Duration,
    state: Arc<Mutex<State>>,
    reaper: Mutex<Option<JoinHandle<()>>>,
}

impl SupervisedEgressPlane {
    /// Construction opens no endpoint and starts no worker.
    pub fn new(
        gateway: Arc<GatewayEgressPlane>,
        mirror: Arc<dyn MirrorPlane>,
        standing: bool,
    ) -> Self {
        Self {
            gateway,
            mirror,
            standing,
            holds_endpoint: Arc::new(AtomicBool::new(false)),
            stop_grace: crate::robot_supervisor::resolve_epoch_sync_stop_grace(
                std::env::var(crate::robot_supervisor::EPOCH_SYNC_STOP_GRACE_ENV)
                    .ok()
                    .as_deref(),
            )
            .saturating_add(crate::robot_supervisor::remoted_stop_grace()),
            state: Arc::new(Mutex::new(State {
                registrations: BTreeSet::new(),
                supervisor: None,
                stopped: false,
            })),
            reaper: Mutex::new(None),
        }
    }

    fn ensure_server(&self, state: &mut State) -> Result<(), EgressError> {
        if state.supervisor.is_some() {
            return Ok(());
        }
        let root =
            cerulion_discovery::robot_state::resolve().ok_or_else(|| EgressError::Server {
                reason: "no robot state root is available".into(),
            })?;
        if !root.is_absolute()
            || root.components().any(|part| {
                matches!(
                    part,
                    std::path::Component::ParentDir | std::path::Component::CurDir
                )
            })
        {
            return Err(EgressError::Server {
                reason: "robot state root must be absolute and contain no traversal".into(),
            });
        }
        let directory = root.join("remoted");
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&directory)
            .map_err(|error| EgressError::Server {
                reason: error.to_string(),
            })?;
        // Create the stable fencing inode before a child can admit ops work.
        let fence = crate::serving_hold::IdleFence::try_acquire(&directory.join("serving.hold"))
            .map_err(|error| EgressError::Server {
                reason: error.to_string(),
            })?
            .ok_or_else(|| EgressError::Server {
                reason: "the remote serving endpoint still has active work".into(),
            })?;
        drop(fence);
        self.holds_endpoint.store(true, Ordering::Release);
        if let Err(reason) = self.mirror.begin_serving() {
            self.mirror.end_serving();
            self.holds_endpoint.store(false, Ordering::Release);
            return Err(EgressError::Server { reason });
        }
        match RobotSupervisor::start(Arc::new(AtomicBool::new(true)), Arc::clone(&self.gateway)) {
            Ok(supervisor) => state.supervisor = Some(supervisor),
            Err(error) => {
                self.mirror.end_serving();
                self.holds_endpoint.store(false, Ordering::Release);
                return Err(EgressError::Server {
                    reason: error.to_string(),
                });
            }
        }
        Ok(())
    }

    /// Explicit LISTEN serving starts after the daemon acquires its singleton.
    pub fn start_standing(&self) -> Result<(), EgressError> {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if self.standing && !state.stopped {
            self.ensure_server(&mut state)?;
        }
        Ok(())
    }

    fn stop_server(&self, state: &mut State) {
        if let Some(mut supervisor) = state.supervisor.take() {
            supervisor.shutdown();
            self.mirror.end_serving();
            self.holds_endpoint.store(false, Ordering::Release);
        }
    }

    /// Stop accepting registrations and reap before the singleton is released.
    pub fn shutdown(&self) {
        {
            let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
            state.stopped = true;
            state.registrations.clear();
            self.stop_server(&mut state);
        }
        if let Some(reaper) = self.reaper.lock().unwrap_or_else(|e| e.into_inner()).take() {
            let _ = reaper.join();
        }
    }

    fn reap_when_idle(&self) {
        let mut reaper = self.reaper.lock().unwrap_or_else(|e| e.into_inner());
        if reaper.as_ref().is_some_and(|thread| !thread.is_finished()) {
            return;
        }
        if let Some(thread) = reaper.take() {
            let _ = thread.join();
        }
        let state = Arc::clone(&self.state);
        let mirror = Arc::clone(&self.mirror);
        let held = Arc::clone(&self.holds_endpoint);
        let spawned = std::thread::Builder::new()
            .name("netd-idle-server".into())
            .spawn(move || loop {
                {
                    let mut state = state.lock().unwrap_or_else(|e| e.into_inner());
                    if state.stopped || state.supervisor.is_none() {
                        return;
                    }
                    if state.registrations.is_empty() {
                        let fence = cerulion_discovery::robot_state::resolve().ok_or_else(|| std::io::Error::other("no robot state root is available")).and_then(|root| crate::serving_hold::IdleFence::try_acquire(&root.join("remoted/serving.hold")));
                        let fence = match fence {
                            Ok(fence) => fence,
                            Err(error) => {
                                tracing::error!(error = %error, "remote server idle fence failed; serving retained until shutdown");
                                return;
                            }
                        };
                        if let Some(_fence) = fence {
                            if let Some(mut supervisor) = state.supervisor.take() {
                                supervisor.shutdown();
                            }
                            mirror.end_serving();
                            held.store(false, Ordering::Release);
                            return;
                        }
                    }
                }
                std::thread::sleep(Duration::from_millis(100));
            });
        match spawned {
            Ok(thread) => *reaper = Some(thread),
            Err(error) => {
                tracing::error!(error = %error, "remote server idle reaper could not start; serving retained until shutdown")
            }
        }
    }

    /// Extra shutdown budget for graceful server stop and a revocation fetch.
    pub fn stop_grace(&self) -> Duration {
        if self.holds_serving_endpoint() {
            self.stop_grace
        } else {
            Duration::ZERO
        }
    }
}

impl EgressPlane for SupervisedEgressPlane {
    fn holds_serving_endpoint(&self) -> bool {
        self.holds_endpoint.load(Ordering::Acquire)
    }

    fn register_egress(
        &self,
        id: EgressId,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if state.stopped {
            return Err(EgressError::Server {
                reason: "network daemon is shutting down".into(),
            });
        }
        let booted = self
            .gateway
            .register_egress(id, plan, serving, ix_config_json)?;
        if let Err(error) = self.ensure_server(&mut state) {
            self.gateway.release_egress(id);
            return Err(error);
        }
        state.registrations.insert(id);
        Ok(booted)
    }

    fn release_egress(&self, id: EgressId) {
        let idle = {
            let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
            self.gateway.release_egress(id);
            state.registrations.remove(&id);
            state.registrations.is_empty() && !self.standing && state.supervisor.is_some()
        };
        if idle {
            self.reap_when_idle();
        }
    }

    fn serving_schema_snapshot(&self) -> Result<crate::protocol::ServingSchemaSnapshot, String> {
        self.gateway.serving_schema_snapshot()
    }

    fn lan_access(&self) -> Option<cerulion_core::LanAccess> {
        self.gateway.lan_access()
    }

    fn egress_stats(&self) -> Option<Vec<cerulion_core::EgressTopicStats>> {
        self.gateway.egress_stats()
    }
}

impl Drop for SupervisedEgressPlane {
    fn drop(&mut self) {
        self.shutdown();
    }
}
