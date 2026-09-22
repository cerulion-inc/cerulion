// SPDX-License-Identifier: AGPL-3.0-only
//! One bounded preparation lane; only the render worker may install its result.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use super::{sync_channel, SyncSender, TrySendError, VizMsg};
use crate::skeleton::{BoundModelStatus, Skeleton, UrdfConfig};

/// Observed stage, not a promise that pixels have reached a viewer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelLoadPhase {
    /// Accepted by the preparation queue.
    Queued,
    /// Reading and validating the URDF and its assets off the render thread.
    Loading,
    /// Frozen model ready for the render worker.
    Prepared,
    /// Render worker is submitting statics; cancellation is now too late.
    Installing,
    /// Statics submitted and the exact input route bound; no GPU acknowledgement.
    Installed,
    /// Preparation, handoff, installation, or worker lifetime failed.
    Failed,
    /// Cancelled before installation; outstanding filesystem work may finish later.
    Cancelled,
}

/// Latest operation only. Retaining one snapshot bounds status memory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModelLoadStatus {
    /// Monotonic identifier used to reject stale prepared work.
    pub operation_id: u64,
    /// Current observable stage.
    pub phase: ModelLoadPhase,
    /// Requested model root; usable for layout only once Installed.
    pub root: String,
    /// Exact daemon attachment route, never a topic-name match.
    pub route_key: String,
    /// Failure diagnostic, including resource/validation context.
    pub error: Option<String>,
    /// Last render-worker snapshot, refreshed after each batch and reconnect probe.
    pub binding: Option<BoundModelStatus>,
}

/// Immediate rejection of a model-control request. No waiting or retry loop.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ModelLoadError {
    /// Another preparation or installation still owns the single slot.
    #[error("model loader busy; retry after the current operation finishes")]
    Busy,
    /// The worker has shut down or its receiver has disappeared.
    #[error("visualization worker is closed")]
    WorkerGone,
    /// A model is already bound. Replacement and unloading are unsupported.
    #[error("a model is installed; restart vizd with a fresh worker and recording store before replacing it or detaching its route")]
    AlreadyInstalled,
    /// SDK submission has started and cannot be retracted.
    #[error("model installation has started; its route cannot be detached")]
    Installing,
    /// An installation failed after SDK submission began; partial statics may remain.
    #[error("model installation failed after submission began; restart vizd with a fresh worker and recording store before loading or detaching")]
    RestartRequired,
    /// There is no exact route to bind.
    #[error("model input route must not be empty")]
    InvalidRoute,
}

#[derive(Debug)]
struct Request {
    id: u64,
    path: PathBuf,
    config: UrdfConfig,
    route_key: String,
}

#[derive(Default)]
struct State {
    next_id: u64,
    status: Option<ModelLoadStatus>,
    preparing: bool,
    installation_attempted: bool,
    closed: bool,
    prepare_tx: Option<SyncSender<Request>>,
    render_tx: Option<SyncSender<VizMsg>>,
}

/// All locks cover only short state edits/try_send, never filesystem or SDK work.
#[derive(Default)]
pub(super) struct ModelLoader(Mutex<State>);

impl std::fmt::Debug for ModelLoader {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ModelLoader")
            .field("status", &self.status())
            .finish()
    }
}

impl ModelLoader {
    pub(super) fn spawn(render_tx: SyncSender<VizMsg>) -> std::io::Result<Arc<Self>> {
        let (prepare_tx, rx) = sync_channel::<Request>(1);
        let loader = Arc::new(Self(Mutex::new(State {
            prepare_tx: Some(prepare_tx),
            render_tx: Some(render_tx),
            ..State::default()
        })));
        // Weak ownership keeps an idle preparation thread from keeping itself alive.
        let weak = Arc::downgrade(&loader);
        std::thread::Builder::new()
            .name("viz-model-load".into())
            .spawn(move || {
                while let Ok(request) = rx.recv() {
                    let Some(loader) = weak.upgrade() else { break };
                    if !loader.start(request.id) {
                        continue;
                    }
                    loader.prepare(request.id, || {
                        Skeleton::try_load(&request.path, &request.config)
                            .and_then(|skeleton| {
                                skeleton.validate_binding(&request.route_key)?;
                                Ok(skeleton)
                            })
                            .map_err(|error| error.to_string())
                    });
                }
            })?;
        Ok(loader)
    }

    pub(super) fn load(
        &self,
        path: PathBuf,
        config: UrdfConfig,
        route_key: String,
    ) -> Result<ModelLoadStatus, ModelLoadError> {
        Skeleton::validate_binding_route(&route_key).map_err(|_| ModelLoadError::InvalidRoute)?;
        let mut state = self.0.lock().unwrap();
        if state.closed {
            return Err(ModelLoadError::WorkerGone);
        }
        if let Some(status) = &state.status {
            match status.phase {
                ModelLoadPhase::Installed => return Err(ModelLoadError::AlreadyInstalled),
                ModelLoadPhase::Queued
                | ModelLoadPhase::Loading
                | ModelLoadPhase::Prepared
                | ModelLoadPhase::Installing => return Err(ModelLoadError::Busy),
                _ => {}
            }
        }
        if state.installation_attempted {
            return Err(ModelLoadError::RestartRequired);
        }
        if state.preparing {
            return Err(ModelLoadError::Busy);
        }
        let id = state.next_id.checked_add(1).ok_or(ModelLoadError::Busy)?;
        let status = ModelLoadStatus {
            operation_id: id,
            phase: ModelLoadPhase::Queued,
            root: config.robot_root.clone(),
            route_key,
            error: None,
            binding: None,
        };
        let tx = state
            .prepare_tx
            .as_ref()
            .ok_or(ModelLoadError::WorkerGone)?;
        match tx.try_send(Request {
            id,
            path,
            config,
            route_key: status.route_key.clone(),
        }) {
            Ok(()) => {
                state.next_id = id;
                state.status = Some(status.clone());
                state.preparing = true;
                Ok(status)
            }
            Err(TrySendError::Full(_)) => Err(ModelLoadError::Busy),
            Err(TrySendError::Disconnected(_)) => Err(ModelLoadError::WorkerGone),
        }
    }

    pub(super) fn status(&self) -> Option<ModelLoadStatus> {
        self.0.lock().unwrap().status.clone()
    }

    pub(super) fn cancel(&self, route: &str) -> Result<bool, ModelLoadError> {
        let mut state = self.0.lock().unwrap();
        if state.closed {
            return Err(ModelLoadError::WorkerGone);
        }
        let installation_attempted = state.installation_attempted;
        let Some(status) = state.status.as_mut().filter(|s| s.route_key == route) else {
            return Ok(false);
        };
        match status.phase {
            ModelLoadPhase::Installed => Err(ModelLoadError::AlreadyInstalled),
            ModelLoadPhase::Installing => Err(ModelLoadError::Installing),
            ModelLoadPhase::Failed if installation_attempted => {
                Err(ModelLoadError::RestartRequired)
            }
            ModelLoadPhase::Cancelled | ModelLoadPhase::Failed => Ok(false),
            _ => {
                status.phase = ModelLoadPhase::Cancelled;
                Ok(true)
            }
        }
    }

    fn start(&self, id: u64) -> bool {
        let mut state = self.0.lock().unwrap();
        if Self::transition(
            &mut state,
            id,
            ModelLoadPhase::Queued,
            ModelLoadPhase::Loading,
        ) {
            true
        } else {
            state.preparing = false;
            false
        }
    }

    fn prepare(&self, id: u64, load: impl FnOnce() -> Result<Skeleton, String>) {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(load))
            .unwrap_or_else(|_| Err("model preparation panicked".into()));
        self.prepared(id, result);
    }

    fn prepared(&self, id: u64, result: Result<Skeleton, String>) {
        let mut state = self.0.lock().unwrap();
        state.preparing = false;
        if !Self::transition(
            &mut state,
            id,
            ModelLoadPhase::Loading,
            ModelLoadPhase::Prepared,
        ) {
            return;
        }
        let error = match result {
            Err(error) => Some(error),
            Ok(skeleton) => match state.render_tx.as_ref().map(|tx| {
                tx.try_send(VizMsg::InstallModel {
                    id,
                    skeleton: Box::new(skeleton),
                })
            }) {
                Some(Ok(())) => None,
                Some(Err(TrySendError::Full(_))) => {
                    Some("render queue full; model not installed; retry".into())
                }
                _ => Some("render worker closed before model installation".into()),
            },
        };
        if let Some(error) = error {
            Self::fail(&mut state, id, error);
        }
    }

    pub(super) fn reject_prepared(&self, id: u64, error: String) {
        let mut state = self.0.lock().unwrap();
        if Self::transition(
            &mut state,
            id,
            ModelLoadPhase::Prepared,
            ModelLoadPhase::Failed,
        ) {
            state.status.as_mut().unwrap().error = Some(error);
        }
    }

    // Claim under the SAME mutex as cancellation, then release before SDK calls.
    pub(super) fn begin_install(&self, id: u64) -> Option<String> {
        let mut state = self.0.lock().unwrap();
        if Self::transition(
            &mut state,
            id,
            ModelLoadPhase::Prepared,
            ModelLoadPhase::Installing,
        ) {
            state.installation_attempted = true;
            state.status.as_ref().map(|s| s.route_key.clone())
        } else {
            None
        }
    }

    pub(super) fn finish_install(
        &self,
        id: u64,
        result: Result<(), String>,
        binding: Option<BoundModelStatus>,
    ) -> bool {
        let mut state = self.0.lock().unwrap();
        if !Self::transition(
            &mut state,
            id,
            ModelLoadPhase::Installing,
            ModelLoadPhase::Installed,
        ) {
            return false;
        }
        if let Err(error) = result {
            Self::fail(
                &mut state,
                id,
                format!("{error}; statics may be partially submitted; restart vizd with a fresh worker and recording store"),
            );
            false
        } else {
            state.status.as_mut().unwrap().binding = binding;
            true
        }
    }

    pub(super) fn refresh(&self, binding: Option<&BoundModelStatus>) {
        let mut state = self.0.lock().unwrap();
        if let Some(status) = state
            .status
            .as_mut()
            .filter(|s| s.phase == ModelLoadPhase::Installed)
        {
            status.binding = binding.cloned();
        }
    }

    pub(super) fn close(&self) {
        let mut state = self.0.lock().unwrap();
        state.closed = true;
        state.prepare_tx = None;
        state.render_tx = None;
        if let Some(status) = state.status.as_mut() {
            if !matches!(
                status.phase,
                ModelLoadPhase::Cancelled | ModelLoadPhase::Failed
            ) {
                status.phase = ModelLoadPhase::Failed;
                status.error = Some("visualization worker closed".into());
            }
        }
    }

    fn transition(state: &mut State, id: u64, from: ModelLoadPhase, to: ModelLoadPhase) -> bool {
        if state.closed {
            return false;
        }
        if let Some(status) = state
            .status
            .as_mut()
            .filter(|s| s.operation_id == id && s.phase == from)
        {
            status.phase = to;
            true
        } else {
            false
        }
    }

    fn fail(state: &mut State, id: u64, error: String) {
        if let Some(status) = state.status.as_mut().filter(|s| s.operation_id == id) {
            status.phase = ModelLoadPhase::Failed;
            status.error = Some(error);
        }
    }
}

/// Also closes observable operations if a panic escapes the render worker.
pub(super) struct WorkerLifetime(pub Arc<ModelLoader>);
impl Drop for WorkerLifetime {
    fn drop(&mut self) {
        self.0.close();
    }
}

#[cfg(test)]
mod tests {
    use super::super::VizLogWorker;
    use super::*;
    use crate::sink::SinkState;
    use cerulion_core::codegen::{layout::LayoutResolver, parse_rosmsg, FrameWalker};
    use cerulion_core::wire::WireHeader;
    use std::sync::mpsc::Receiver;
    use std::time::{Duration, Instant};

    fn config() -> UrdfConfig {
        UrdfConfig {
            robot_root: "models/test".into(),
            motor_joints: vec!["hinge".into()],
            ..UrdfConfig::default()
        }
    }

    fn fixture() -> (tempfile::TempDir, PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robot.urdf");
        std::fs::write(&path, r#"<robot name="test"><link name="base"/><link name="arm"/>
            <joint name="hinge" type="continuous"><parent link="base"/><child link="arm"/></joint>
            <link name="tip"/><joint name="mount" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>"#).unwrap();
        (dir, path)
    }

    // No threads: advance precisely the states a real preparation thread owns.
    fn harness(capacity: usize) -> (ModelLoader, Receiver<Request>, Receiver<VizMsg>) {
        let (prepare_tx, requests) = sync_channel(1);
        let (render_tx, messages) = sync_channel(capacity);
        (
            ModelLoader(Mutex::new(State {
                prepare_tx: Some(prepare_tx),
                render_tx: Some(render_tx),
                ..State::default()
            })),
            requests,
            messages,
        )
    }

    #[test]
    fn model_load_cancelled_preparation_holds_slot_and_stale_result_cannot_install() {
        let (loader, requests, messages) = harness(1);
        let (_dir, path) = fixture();
        let first = loader.load(path.clone(), config(), "exact".into()).unwrap();
        assert_eq!(first.phase, ModelLoadPhase::Queued);
        let request = requests.recv().unwrap();
        assert!(loader.start(request.id));
        assert_eq!(loader.status().unwrap().phase, ModelLoadPhase::Loading);
        assert!(!loader.cancel("other").unwrap());
        assert!(loader.cancel("exact").unwrap());
        assert_eq!(
            loader.load(path.clone(), config(), "exact".into()),
            Err(ModelLoadError::Busy)
        );
        loader.prepared(
            request.id,
            Skeleton::try_load(&path, &config()).map_err(|e| e.to_string()),
        );
        assert!(messages.try_recv().is_err());
        let second = loader.load(path, config(), "exact".into()).unwrap();
        assert!(second.operation_id > first.operation_id);
        assert!(loader.begin_install(first.operation_id).is_none());
        assert_eq!(loader.status().unwrap(), second);
    }

    #[test]
    fn model_load_full_preparation_queue_does_not_accept_or_overwrite_status() {
        let (loader, _requests, _messages) = harness(1);
        loader
            .0
            .lock()
            .unwrap()
            .prepare_tx
            .as_ref()
            .unwrap()
            .try_send(Request {
                id: 0,
                path: PathBuf::new(),
                config: config(),
                route_key: "exact".into(),
            })
            .unwrap();
        assert_eq!(
            loader.load(PathBuf::new(), config(), "exact".into()),
            Err(ModelLoadError::Busy)
        );
        assert!(loader.status().is_none());
    }

    #[test]
    fn model_load_full_or_closed_render_queue_is_observable_failure() {
        for closed in [false, true] {
            let (loader, requests, messages) = harness(0);
            let (_dir, path) = fixture();
            loader.load(path.clone(), config(), "exact".into()).unwrap();
            let request = requests.recv().unwrap();
            assert!(loader.start(request.id));
            if closed {
                drop(messages);
            }
            loader.prepared(
                request.id,
                Skeleton::try_load(&path, &config()).map_err(|e| e.to_string()),
            );
            let status = loader.status().unwrap();
            assert_eq!(status.phase, ModelLoadPhase::Failed);
            assert!(status
                .error
                .unwrap()
                .contains(if closed { "closed" } else { "queue full" }));
        }
    }

    #[test]
    fn model_load_blank_routes_reject_before_admission_and_preserve_exact_identity() {
        let (loader, requests, _messages) = harness(1);
        let (_dir, path) = fixture();
        for route in ["", "   ", "\t\r\n", "\u{2003}"] {
            assert_eq!(
                loader.load(path.clone(), config(), route.into()),
                Err(ModelLoadError::InvalidRoute)
            );
            assert!(loader.status().is_none());
            assert!(matches!(
                requests.try_recv(),
                Err(std::sync::mpsc::TryRecvError::Empty)
            ));
        }
        let route = "  exact\u{1}/topic  ";
        let status = loader.load(path, config(), route.into()).unwrap();
        assert_eq!(status.route_key, route);
        assert_eq!(requests.recv().unwrap().route_key, route);
    }

    #[test]
    fn model_load_prepared_cancellation_and_install_claim_are_exclusive() {
        for cancel_first in [true, false] {
            let (loader, requests, messages) = harness(1);
            let (_dir, path) = fixture();
            loader.load(path.clone(), config(), "exact".into()).unwrap();
            let request = requests.recv().unwrap();
            assert!(loader.start(request.id));
            loader.prepared(
                request.id,
                Skeleton::try_load(&path, &config()).map_err(|e| e.to_string()),
            );
            assert_eq!(loader.status().unwrap().phase, ModelLoadPhase::Prepared);
            let VizMsg::InstallModel { id, .. } = messages.recv().unwrap() else {
                panic!("expected model");
            };
            if cancel_first {
                assert!(loader.cancel("exact").unwrap());
                loader.reject_prepared(id, "late state rejection".into());
                assert_eq!(loader.status().unwrap().phase, ModelLoadPhase::Cancelled);
                assert!(loader.begin_install(id).is_none());
            } else {
                assert_eq!(loader.begin_install(id).as_deref(), Some("exact"));
                assert_eq!(loader.cancel("exact"), Err(ModelLoadError::Installing));
                assert!(loader.finish_install(id, Ok(()), None));
                assert_eq!(
                    loader.cancel("exact"),
                    Err(ModelLoadError::AlreadyInstalled)
                );
                assert_eq!(
                    loader.load(path, config(), "new".into()),
                    Err(ModelLoadError::AlreadyInstalled)
                );
            }
        }
    }

    #[test]
    fn model_load_failed_install_requires_restart_to_avoid_residual_statics() {
        let (loader, requests, _messages) = harness(1);
        let (_dir, path) = fixture();
        loader.load(path.clone(), config(), "exact".into()).unwrap();
        let request = requests.recv().unwrap();
        assert!(loader.start(request.id));
        loader.prepared(
            request.id,
            Skeleton::try_load(&path, &config()).map_err(|e| e.to_string()),
        );
        assert!(loader.begin_install(request.id).is_some());
        assert!(!loader.finish_install(request.id, Err("SDK submission failed".into()), None));
        let status = loader.status().unwrap();
        assert_eq!(status.phase, ModelLoadPhase::Failed);
        assert!(status
            .error
            .unwrap()
            .contains("fresh worker and recording store"));
        assert_eq!(
            loader.load(path, config(), "new".into()),
            Err(ModelLoadError::RestartRequired)
        );
        assert_eq!(loader.cancel("exact"), Err(ModelLoadError::RestartRequired));
        assert!(!loader.cancel("other").unwrap());
    }

    #[test]
    fn model_load_preparation_panic_is_terminal_and_releases_slot() {
        let (loader, requests, _messages) = harness(1);
        loader
            .load(PathBuf::new(), config(), "exact".into())
            .unwrap();
        let request = requests.recv().unwrap();
        assert!(loader.start(request.id));
        loader.prepare(request.id, || panic!("injected filesystem loader panic"));
        assert_eq!(loader.status().unwrap().phase, ModelLoadPhase::Failed);
        assert_eq!(
            loader.status().unwrap().error.as_deref(),
            Some("model preparation panicked")
        );
        assert!(loader
            .load(PathBuf::new(), config(), "exact".into())
            .is_ok());
    }

    #[test]
    fn model_load_shutdown_fails_pending_operation_and_ignores_late_completion() {
        let (loader, requests, _messages) = harness(1);
        loader
            .load(PathBuf::new(), config(), "exact".into())
            .unwrap();
        let request = requests.recv().unwrap();
        assert!(loader.start(request.id));
        loader.close();
        loader.prepared(request.id, Err("late error".into()));
        let status = loader.status().unwrap();
        assert_eq!(status.phase, ModelLoadPhase::Failed);
        assert_eq!(status.error.as_deref(), Some("visualization worker closed"));
        assert_eq!(loader.cancel("exact"), Err(ModelLoadError::WorkerGone));
        assert_eq!(
            loader.load(PathBuf::new(), config(), "exact".into()),
            Err(ModelLoadError::WorkerGone)
        );
    }

    fn wait_status(control: &super::super::VizControl, phase: ModelLoadPhase) -> ModelLoadStatus {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let status = control.model_status().unwrap();
            if status.phase == phase {
                return status;
            }
            assert!(
                Instant::now() < deadline,
                "expected {phase:?}, got {status:?}"
            );
            std::thread::yield_now();
        }
    }

    #[test]
    fn model_load_real_worker_installs_and_publishes_layout_signal() {
        let _statics = crate::test_support::blueprint_statics_guard();
        let (_dir, path) = fixture();
        let (rec, storage) = rerun::RecordingStreamBuilder::new("model-worker")
            .memory()
            .unwrap();
        let schemas = vec![
            parse_rosmsg("float32 q\n", "MotorState", Some("unitree_go")).unwrap(),
            parse_rosmsg(
                "unitree_go/MotorState[20] motor_state\n",
                "LowState",
                Some("unitree_go"),
            )
            .unwrap(),
        ];
        let (mut resolver, _) = LayoutResolver::new(schemas.clone());
        let layout = resolver.layout_of("unitree_go/LowState").unwrap();
        let (walker, _) = FrameWalker::new(schemas);
        let mut worker = VizLogWorker::spawn(rec.clone(), walker, SinkState::new()).unwrap();
        let control = worker.control();
        let other_control = worker.control();
        assert_eq!(
            control
                .load_model(path, config(), "exact".into())
                .unwrap()
                .phase,
            ModelLoadPhase::Queued
        );
        let status = wait_status(&control, ModelLoadPhase::Installed);
        assert_eq!(other_control.model_status().unwrap(), status);
        let binding = status.binding.unwrap();
        assert_eq!(binding.route_key, "exact");
        assert!(binding.statics_submitted);
        assert_eq!(binding.joint_frames_submitted, 0);
        rec.flush_blocking().unwrap();
        assert!(storage.take().into_iter().any(|message| match message {
            rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                let chunk = rerun::log::Chunk::from_arrow_msg(&arrow).unwrap();
                chunk.is_static()
                    && chunk.entity_path() == &"models/test/arm/tip".into()
                    && chunk
                        .iter_component::<rerun::components::Translation3D>(
                            rerun::Transform3D::descriptor_translation().component,
                        )
                        .flat_map(|batch| batch.iter().map(|v| v.0 .0).collect::<Vec<_>>())
                        .collect::<Vec<_>>()
                        == vec![[1.0, 0.0, 0.0]]
            }
            _ => false,
        }));
        worker.sync();
        assert_eq!(
            worker
                .counters()
                .layout_signal_generation
                .load(std::sync::atomic::Ordering::Relaxed),
            1
        );
        for (sequence, angle) in [0.5f32, f32::NAN].into_iter().enumerate() {
            let mut wire = vec![0; WireHeader::SIZE + 80];
            WireHeader {
                schema_hash: layout.schema_hash,
                total_size: wire.len() as u32,
                offset_table_offset: wire.len() as u32,
                offset_table_count: 0,
                sequence: sequence as u32,
                timestamp_ns: 1_000_000_000 + sequence as u64,
            }
            .write_to_buf(&mut wire[..WireHeader::SIZE]);
            wire[WireHeader::SIZE..WireHeader::SIZE + 4].copy_from_slice(&angle.to_le_bytes());
            worker.try_enqueue(vec![super::super::InputFrames {
                name: "exact".into(),
                frames: vec![wire],
            }]);
            worker.sync();
        }
        let binding = control.model_status().unwrap().binding.unwrap();
        assert_eq!(binding.joint_frames_submitted, 1);
        assert_eq!(binding.rejected_frames, 1);
        assert!(binding.last_error.unwrap().contains("motor_state[0].q"));
        rec.flush_blocking().unwrap();
        assert!(storage.take().into_iter().any(|message| match message {
            rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                let chunk = rerun::log::Chunk::from_arrow_msg(&arrow).unwrap();
                !chunk.is_static() && chunk.entity_path() == &"models/test/arm".into()
            }
            _ => false,
        }));
        control.close();
        other_control.close();
        drop(worker);
        assert_eq!(
            control.model_status().unwrap().phase,
            ModelLoadPhase::Failed
        );
    }

    #[test]
    fn model_load_closing_one_control_preserves_other_live_controls() {
        let _statics = crate::test_support::blueprint_statics_guard();
        let (_dir, path) = fixture();
        let (rec, _) = rerun::RecordingStreamBuilder::new("model-independent-controls")
            .memory()
            .unwrap();
        let (walker, _) = FrameWalker::new(Vec::new());
        let worker = VizLogWorker::spawn(rec, walker, SinkState::new()).unwrap();
        let closed = worker.control();
        let closed_clone = Arc::clone(&closed);
        let live = worker.control();
        closed.close();
        worker.sync();
        assert_eq!(
            closed_clone.load_model(path.clone(), config(), "exact".into()),
            Err(ModelLoadError::WorkerGone)
        );
        assert_eq!(
            closed.cancel_model_load("exact"),
            Err(ModelLoadError::WorkerGone)
        );
        live.load_model(path, config(), "exact".into()).unwrap();
        wait_status(&live, ModelLoadPhase::Installed);
        live.close();
        drop(worker);
        assert_eq!(live.model_status().unwrap().phase, ModelLoadPhase::Failed);
    }

    #[test]
    fn model_load_render_preflight_rejects_without_poisoning_retry() {
        let _statics = crate::test_support::blueprint_statics_guard();
        let (_dir, path) = fixture();
        for disabled in [true, false] {
            let rec = if disabled {
                rerun::RecordingStream::disabled()
            } else {
                rerun::RecordingStreamBuilder::new("existing-bound-model")
                    .memory()
                    .unwrap()
                    .0
            };
            let mut state = SinkState::new();
            if !disabled {
                state
                    .install_bound_model(
                        &rec,
                        "existing",
                        Skeleton::try_load(&path, &config()).unwrap(),
                    )
                    .unwrap();
            }
            let (walker, _) = FrameWalker::new(Vec::new());
            let worker = VizLogWorker::spawn(rec, walker, state).unwrap();
            let control = worker.control();
            for _ in 0..2 {
                control
                    .load_model(path.clone(), config(), "exact".into())
                    .unwrap();
                let status = wait_status(&control, ModelLoadPhase::Failed);
                let error = status.error.unwrap();
                assert!(error.contains(if disabled {
                    "disabled"
                } else {
                    "already installed"
                }));
                assert!(!error.contains("restart"));
                assert_eq!(control.cancel_model_load("exact"), Ok(false));
            }
            control.close();
        }
    }

    #[test]
    fn model_load_invalid_bindings_fail_before_submission_and_allow_valid_retry() {
        let _statics = crate::test_support::blueprint_statics_guard();
        let (rec, storage) = rerun::RecordingStreamBuilder::new("model-preflight-retry")
            .memory()
            .unwrap();
        let (walker, _) = FrameWalker::new(Vec::new());
        let worker = VizLogWorker::spawn(rec.clone(), walker, SinkState::new()).unwrap();
        let control = worker.control();
        worker.sync();
        rec.flush_blocking().unwrap();
        storage.take();
        let (dir, path) = fixture();
        let static_path = dir.path().join("static.urdf");
        std::fs::write(
            &static_path,
            r#"<robot name="static"><link name="base"/></robot>"#,
        )
        .unwrap();
        let incomplete_path = dir.path().join("incomplete.urdf");
        let xml = std::fs::read_to_string(&path).unwrap().replace("</robot>", r#"<link name="extra"/><joint name="unmapped" type="continuous"><parent link="base"/><child link="extra"/></joint></robot>"#);
        std::fs::write(&incomplete_path, xml).unwrap();
        let mut static_config = config();
        static_config.motor_joints.clear();
        let mut wrong_root = config();
        wrong_root.robot_root = "world/robot".into();
        for (invalid_path, invalid_config) in [
            (static_path, static_config),
            (incomplete_path, config()),
            (path.clone(), wrong_root),
        ] {
            control
                .load_model(invalid_path, invalid_config, "exact".into())
                .unwrap();
            let failed = wait_status(&control, ModelLoadPhase::Failed);
            assert!(!failed.error.unwrap().contains("restart"));
            assert_eq!(control.cancel_model_load("exact"), Ok(false));
            rec.flush_blocking().unwrap();
            assert!(
                storage
                    .take()
                    .into_iter()
                    .all(|message| { !matches!(message, rerun::log::LogMsg::ArrowMsg(..)) }),
                "preflight must submit no rows, including under an invalid root"
            );
        }
        control.load_model(path, config(), "exact".into()).unwrap();
        assert_eq!(
            wait_status(&control, ModelLoadPhase::Installed)
                .binding
                .unwrap()
                .joint_frames_submitted,
            0
        );
        control.close();
    }

    #[test]
    fn model_load_missing_resource_reports_failure_and_allows_retry() {
        let (tx, _rx) = sync_channel(1);
        let loader = ModelLoader::spawn(tx).unwrap();
        let dir = tempfile::tempdir().unwrap();
        loader
            .load(dir.path().join("missing.urdf"), config(), "exact".into())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while loader.status().unwrap().phase != ModelLoadPhase::Failed {
            assert!(Instant::now() < deadline);
            std::thread::yield_now();
        }
        assert!(loader
            .status()
            .unwrap()
            .error
            .unwrap()
            .contains("missing.urdf"));
        assert!(loader
            .load(dir.path().join("missing.urdf"), config(), "exact".into())
            .is_ok());
        loader.close();
    }
}
