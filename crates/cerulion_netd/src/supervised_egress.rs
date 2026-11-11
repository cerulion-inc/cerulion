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

#[derive(Debug, PartialEq, Eq)]
enum IdleAction {
    Stop,
    Wait,
    Reap,
}

fn idle_action(stopped: bool, has_registrations: bool, has_server: bool) -> IdleAction {
    if stopped {
        IdleAction::Stop
    } else if has_registrations || !has_server {
        IdleAction::Wait
    } else {
        IdleAction::Reap
    }
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
        // Create the stable fencing inode before a child can admit ops work.
        let fence =
            crate::serving_hold::IdleFence::try_acquire(&crate::serving_hold::fence_path(&root))
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
                    let action = idle_action(state.stopped, !state.registrations.is_empty(), state.supervisor.is_some());
                    if action == IdleAction::Stop {
                        return;
                    }
                    if action == IdleAction::Reap {
                        let fence = cerulion_discovery::robot_state::resolve().ok_or_else(|| std::io::Error::other("no robot state root is available")).and_then(|root| crate::serving_hold::IdleFence::try_acquire(&crate::serving_hold::fence_path(&root)));
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
                            // Keep this worker for later server generations: a
                            // release can race the old generation's reap.
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

    /// Extra shutdown budget for graceful server stop and a revocation fetch,
    /// including a registration still starting before it claims the endpoint.
    pub fn stop_grace(&self) -> Duration {
        self.stop_grace
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

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::testing::IsolatedRoot;
    use cerulion_core::{TransportConfig, TransportManager};

    #[test]
    fn idle_worker_survives_reaped_generations_until_shutdown() {
        for (stopped, registered, server, expected) in [
            (false, false, false, IdleAction::Wait),
            (false, false, true, IdleAction::Reap),
            (false, true, false, IdleAction::Wait),
            (false, true, true, IdleAction::Wait),
            (true, false, false, IdleAction::Stop),
            (true, false, true, IdleAction::Stop),
            (true, true, false, IdleAction::Stop),
            (true, true, true, IdleAction::Stop),
        ] {
            assert_eq!(idle_action(stopped, registered, server), expected);
        }
    }

    #[test]
    fn shutdown_reserves_server_grace_before_pending_registration_claims_endpoint() {
        let root = IsolatedRoot::mint("r2");
        let manager =
            TransportManager::init_for_test(TransportConfig::default(), root.config()).unwrap();
        let plane = Arc::new(SupervisedEgressPlane::new(
            Arc::new(GatewayEgressPlane::new(Arc::clone(&manager))),
            Arc::new(crate::mirror::GatewayMirrorPlane::new(manager)),
            false,
        ));
        // A registration owns this mutex while the gateway boots, before the
        // endpoint flag changes. Reading the watchdog budget must not wait on it.
        let registration = plane.state.lock().unwrap();
        assert!(!plane.holds_serving_endpoint());
        let (send, receive) = std::sync::mpsc::channel();
        let pending = Arc::clone(&plane);
        let reader = std::thread::spawn(move || send.send(pending.stop_grace()).unwrap());
        let budget = receive.recv_timeout(Duration::from_secs(1));
        drop(registration);
        reader.join().unwrap();
        assert!(budget.unwrap() >= crate::robot_supervisor::remoted_stop_grace());
    }
}
