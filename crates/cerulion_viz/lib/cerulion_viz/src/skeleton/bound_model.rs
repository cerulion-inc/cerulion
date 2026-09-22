// SPDX-License-Identifier: AGPL-3.0-only
//! One explicit model/route/recording binding; no schema or frame-name inference.

use super::{joint_transform3d, read_leg_motor_qs, Skeleton, UrdfError, UrdfModel};
use cerulion_core::codegen::FrameValue;
use rerun::RecordingStream;

/// SDK submission state. These counters do not prove GPU rendering or live data.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct BoundModelStatus {
    /// Exact attachment route key selected by the caller.
    pub route_key: String,
    /// All static rows submitted in this connection epoch.
    pub statics_submitted: bool,
    /// Complete measured frames submitted successfully.
    pub joint_frames_submitted: u64,
    /// Selected frames rejected before completing submission.
    pub rejected_frames: u64,
    /// Most recent submission error; cleared by a successful frame or pending static retry.
    pub last_error: Option<String>,
}

#[derive(Debug)]
pub(crate) struct BoundModel {
    model: UrdfModel,
    recording_id: rerun::StoreId,
    status: BoundModelStatus,
    next_static_row: usize,
}

enum StaticRow<'a> {
    Transform(&'a str, Box<rerun::Transform3D>),
    Asset(&'a str, &'a rerun::Asset3D),
}

fn submission(error: impl ToString) -> UrdfError {
    UrdfError::Submission(error.to_string())
}

impl BoundModel {
    pub(crate) fn prepare(
        rec: &RecordingStream,
        route_key: &str,
        skeleton: Skeleton,
    ) -> Result<Self, UrdfError> {
        if route_key.trim().is_empty() {
            return Err(submission("an exact attached route key is required"));
        }
        if !skeleton.strict_loaded {
            return Err(submission("model must come from Skeleton::try_load"));
        }
        let model = skeleton.model.ok_or_else(|| submission("model is inert"))?;
        let root = &model.link_entity[&model.root_link];
        let segments: Vec<_> = root.split('/').collect();
        if segments.len() != 2 || segments[0] != "models" || segments[1].is_empty() {
            return Err(submission(
                "explicit model root must be models/<id>, outside topic and TF namespaces",
            ));
        }
        if model.motor_bindings.is_empty()
            || model.motor_bindings.iter().any(Option::is_none)
            || model
                .joints
                .iter()
                .filter(|joint| matches!(joint.kind, super::JointKind::Revolute))
                .count()
                != model.motor_bindings.len()
        {
            return Err(submission(
                "model requires explicit measured motor bindings",
            ));
        }
        Ok(Self {
            model,
            recording_id: rec
                .store_info()
                .ok_or_else(|| submission("recording is disabled"))?
                .store_id,
            next_static_row: 0,
            status: BoundModelStatus {
                route_key: route_key.into(),
                ..Default::default()
            },
        })
    }

    #[cfg(test)]
    fn install(
        rec: &RecordingStream,
        route_key: &str,
        skeleton: Skeleton,
    ) -> Result<Self, UrdfError> {
        let mut binding = Self::prepare(rec, route_key, skeleton)?;
        binding.submit_statics(rec)?;
        Ok(binding)
    }

    pub(crate) fn status(&self) -> &BoundModelStatus {
        &self.status
    }

    pub(crate) fn matches(&self, route_key: &str) -> bool {
        self.status.route_key == route_key
    }

    pub(crate) fn rearm_statics(&mut self) {
        self.status.statics_submitted = false;
        self.next_static_row = 0;
    }

    pub(crate) fn submit_statics(&mut self, rec: &RecordingStream) -> Result<(), UrdfError> {
        if rec.store_info().map(|info| info.store_id).as_ref() != Some(&self.recording_id) {
            let error = submission("statics belong to a different or disabled recording");
            self.status.last_error = Some(error.to_string());
            return Err(error);
        }
        let pending = !self.status.statics_submitted;
        let result = self.submit_statics_with(|row| match row {
            StaticRow::Transform(entity, transform) => rec
                .log_static(entity, transform.as_ref())
                .map_err(submission),
            StaticRow::Asset(entity, asset) => rec.log_static(entity, asset).map_err(submission),
        });
        match &result {
            Err(error) => self.status.last_error = Some(error.to_string()),
            Ok(()) if pending => self.status.last_error = None,
            Ok(()) => {}
        }
        result
    }

    fn submit_statics_with(
        &mut self,
        mut write: impl FnMut(StaticRow<'_>) -> Result<(), UrdfError>,
    ) -> Result<(), UrdfError> {
        if self.status.statics_submitted {
            return Ok(());
        }
        // Static components shadow temporal rows. Movable joints therefore get
        // their full transform only when a complete measured frame arrives.
        let fixed = self
            .model
            .joints
            .iter()
            .filter(|joint| matches!(joint.kind, super::JointKind::Fixed))
            .map(|joint| {
                StaticRow::Transform(
                    &self.model.link_entity[&joint.child],
                    // hot-path-alloc-ok: pending model statics only, never completed frames.
                    Box::new(joint_transform3d(joint.xyz, joint.rpy, [0.0; 3], 0.0)),
                )
            });
        let meshes = self.model.mesh_assets.iter().flat_map(|asset| {
            let mesh = &self.model.prepared_meshes[&asset.glb_path];
            std::iter::once(StaticRow::Asset(asset.entity.as_str(), mesh)).chain(
                asset
                    .origin_transform()
                    .map(|tf| StaticRow::Transform(asset.entity.as_str(), Box::new(tf))),
            )
        });
        for (index, row) in fixed.chain(meshes).enumerate().skip(self.next_static_row) {
            write(row)?;
            // Advance only after acceptance: retries never append a successful prefix.
            self.next_static_row = index + 1;
        }
        self.status.statics_submitted = true;
        Ok(())
    }

    pub(crate) fn submit_frame(
        &mut self,
        rec: &RecordingStream,
        route_key: &str,
        timestamp_ns: u64,
        frame: &FrameValue,
    ) {
        if !self.matches(route_key) {
            return;
        }
        match self.try_submit_frame(rec, timestamp_ns, frame) {
            Ok(()) => {
                self.status.joint_frames_submitted += 1;
                self.status.last_error = None;
            }
            Err(error) => {
                self.status.rejected_frames += 1;
                self.status.last_error = Some(error.to_string());
            }
        }
    }

    fn try_submit_frame(
        &mut self,
        rec: &RecordingStream,
        timestamp_ns: u64,
        frame: &FrameValue,
    ) -> Result<(), UrdfError> {
        if rec.store_info().map(|info| info.store_id).as_ref() != Some(&self.recording_id) {
            return Err(submission("frame belongs to a different recording"));
        }
        if frame.schema_name != "unitree_go/LowState" {
            return Err(submission("expected unitree_go/LowState"));
        }
        let angles = read_leg_motor_qs(frame);
        // Validate the whole required bank before writing ANY measured transform.
        for (index, _) in self.model.motor_bindings.iter().enumerate() {
            if !angles[index].is_some_and(|q| q.is_finite() && (q as f32).is_finite()) {
                return Err(submission(format!(
                    "motor_state[{index}].q must be finite and present"
                )));
            }
        }
        if !self.status.statics_submitted {
            self.submit_statics(rec)?;
        }
        super::set_robot_time(rec, timestamp_ns);
        for (binding, angle) in self.model.motor_bindings.iter().zip(angles) {
            let binding = binding.as_ref().expect("installation checks every binding");
            let q = angle.expect("the complete required bank was checked");
            let tf = joint_transform3d(binding.xyz, binding.rpy, binding.axis, q);
            rec.log(binding.child_entity.clone(), &tf)
                .map_err(submission)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    // No process globals: each test owns its model, sink, and memory recording.
    use super::*;
    use crate::sink::{dispatch_frame, SinkState};
    use crate::skeleton::UrdfConfig;
    use cerulion_core::codegen::layout::LayoutResolver;
    use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker, NamedValue};
    use cerulion_core::wire::WireHeader;

    const XML: &str = r#"<robot name="test"><link name="base"/><link name="arm"/>
        <link name="tip"/><joint name="hinge" type="revolute"><parent link="base"/>
        <child link="arm"/><origin xyz="1 2 3"/><axis xyz="0 0 1"/></joint>
        <joint name="mount" type="fixed"><parent link="arm"/><child link="tip"/>
        <origin xyz="2 0 0"/></joint></robot>"#;

    fn loaded() -> Skeleton {
        load_xml(XML, vec!["hinge".into()])
    }

    fn load_xml(xml: &str, motors: Vec<String>) -> Skeleton {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("model.urdf");
        std::fs::write(&path, xml).unwrap();
        Skeleton::try_load(
            &path,
            &UrdfConfig {
                default_path: String::new(),
                robot_root: "models/test".into(),
                motor_joints: motors,
            },
        )
        .unwrap()
    }

    fn memory() -> (RecordingStream, rerun::sink::MemorySinkStorage) {
        rerun::RecordingStreamBuilder::new("bound-model")
            .memory()
            .unwrap()
    }

    fn frame(angles: &[f32]) -> FrameValue<'static> {
        FrameValue {
            schema_name: "unitree_go/LowState".into(),
            fields: vec![NamedValue {
                name: "motor_state".into(),
                value: FrameValueKind::Array(
                    angles
                        .iter()
                        .map(|q| {
                            FrameValueKind::Nested(Box::new(FrameValue {
                                schema_name: "unitree_go/MotorState".into(),
                                fields: vec![NamedValue {
                                    name: "q".into(),
                                    value: FrameValueKind::F32(*q),
                                }],
                            }))
                        })
                        .collect(),
                ),
            }],
        }
    }

    fn chunks(
        rec: &RecordingStream,
        storage: &rerun::sink::MemorySinkStorage,
    ) -> Vec<rerun::log::Chunk> {
        rec.flush_blocking().unwrap();
        storage
            .take()
            .into_iter()
            .filter_map(|msg| match msg {
                rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                    Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
                }
                _ => None,
            })
            .collect()
    }

    #[test]
    fn selected_route_records_exact_hinge_transform_without_animating_other_routes() {
        let (rec, storage) = memory();
        let mut model = BoundModel::install(&rec, "robot::lowstate", loaded()).unwrap();
        let installation = chunks(&rec, &storage);
        model.submit_frame(&rec, "other::lowstate", 10, &frame(&[0.5]));
        assert!(chunks(&rec, &storage).is_empty());
        model.submit_frame(
            &rec,
            "robot::lowstate",
            20,
            &frame(&[std::f32::consts::FRAC_PI_2]),
        );
        let rows = chunks(&rec, &storage);
        let mut store = rerun::ChunkStore::new(
            rec.store_info().unwrap().store_id,
            rerun::ChunkStoreConfig::ALL_DISABLED,
        );
        for row in installation.into_iter().chain(rows) {
            store.insert_chunk(&std::sync::Arc::new(row)).unwrap();
        }
        use rerun::external::re_chunk_store::{ChunkTrackingMode, LatestAtQuery};
        let query = LatestAtQuery::new(
            "robot_time".into(),
            rerun::external::re_log_types::TimeInt::MAX,
        );
        let component = rerun::Transform3D::descriptor_quaternion().component;
        let rows = store
            .latest_at_relevant_chunks(
                ChunkTrackingMode::PanicOnMissing,
                &query,
                &"models/test/arm".into(),
                component,
            )
            .chunks;
        assert_eq!(rows.len(), 1);
        let rows: Vec<_> = rows
            .iter()
            .filter_map(|row| row.latest_at(&query, component))
            .collect();
        let translations: Vec<_> = rows[0]
            .iter_component::<rerun::components::Translation3D>(
                rerun::Transform3D::descriptor_translation().component,
            )
            .flat_map(|batch| batch.iter().map(|v| v.0 .0).collect::<Vec<_>>())
            .collect();
        assert_eq!(translations, vec![[1.0, 2.0, 3.0]]);
        let rotations: Vec<_> = rows[0]
            .iter_component::<rerun::components::RotationQuat>(
                rerun::Transform3D::descriptor_quaternion().component,
            )
            .flat_map(|batch| batch.iter().map(|v| v.0 .0).collect::<Vec<_>>())
            .collect();
        let q = rotations[0];
        assert_eq!([q[0], q[1]], [0.0, 0.0]);
        assert!((q[2] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert!((q[3] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert_eq!(model.status.joint_frames_submitted, 1);
    }

    #[test]
    fn missing_nonfinite_wrong_schema_and_wrong_recording_reject_without_joint_rows() {
        let (rec, storage) = memory();
        let mut model = BoundModel::install(&rec, "selected", loaded()).unwrap();
        chunks(&rec, &storage);
        for invalid in [frame(&[]), frame(&[f32::NAN]), frame(&[f32::INFINITY])] {
            model.submit_frame(&rec, "selected", 1, &invalid);
        }
        let mut wrong = frame(&[0.5]);
        wrong.schema_name = "other/LowState".into();
        model.submit_frame(&rec, "selected", 1, &wrong);
        let (other, _) = memory();
        model.submit_frame(&other, "selected", 1, &frame(&[0.5]));
        assert_eq!(model.status.rejected_frames, 5);
        assert_eq!(model.status.joint_frames_submitted, 0);
        assert!(chunks(&rec, &storage).is_empty());
        model.submit_frame(&rec, "selected", 2, &frame(&[0.5]));
        assert_eq!(model.status.last_error, None);
        assert_eq!(model.status.joint_frames_submitted, 1);
    }

    #[test]
    fn invalid_later_motor_cannot_partially_update_an_earlier_joint() {
        let xml = XML.replace(
            "name=\"mount\" type=\"fixed\"",
            "name=\"mount\" type=\"revolute\"",
        );
        let (rec, storage) = memory();
        let mut model = BoundModel::install(
            &rec,
            "selected",
            load_xml(&xml, vec!["hinge".into(), "mount".into()]),
        )
        .unwrap();
        chunks(&rec, &storage);
        for angles in [&[0.5][..], &[0.5, f32::NAN][..]] {
            model.submit_frame(&rec, "selected", 1, &frame(angles));
            assert!(chunks(&rec, &storage).is_empty());
        }
        assert_eq!(model.status.rejected_frames, 2);
    }

    #[test]
    fn statics_are_per_recording_and_only_rearmed_explicitly() {
        for _ in 0..2 {
            let (rec, storage) = memory();
            let mut state = SinkState::new();
            state
                .install_bound_model(&rec, "selected", loaded())
                .unwrap();
            assert_eq!(
                chunks(&rec, &storage)
                    .iter()
                    .filter(|c| c.is_static()
                        && c.entity_path()
                            .to_string()
                            .trim_start_matches('/')
                            .starts_with("models/"))
                    .count(),
                1
            );
            state.clear_rebroadcast_dedup();
            assert!(!state.bound_model_status().unwrap().statics_submitted);
            let (rec, storage) = memory();
            let mut model = BoundModel::install(&rec, "selected", loaded()).unwrap();
            chunks(&rec, &storage);
            model.submit_frame(&rec, "selected", 1, &frame(&[0.0]));
            assert!(chunks(&rec, &storage).iter().all(|c| !c.is_static()));
            model.rearm_statics();
            model.submit_frame(&rec, "selected", 2, &frame(&[0.0]));
            assert_eq!(
                chunks(&rec, &storage)
                    .iter()
                    .filter(|c| c.is_static()
                        && c.entity_path()
                            .to_string()
                            .trim_start_matches('/')
                            .starts_with("models/"))
                    .count(),
                1
            );
        }
    }

    #[test]
    fn reconnect_submits_to_the_current_stream_with_the_same_recording_id() {
        let (first, first_storage) = rerun::RecordingStreamBuilder::new("reconnect")
            .recording_id("same-recording")
            .memory()
            .unwrap();
        let (second, second_storage) = rerun::RecordingStreamBuilder::new("reconnect")
            .recording_id("same-recording")
            .memory()
            .unwrap();
        let mut model = BoundModel::install(&first, "selected", loaded()).unwrap();
        chunks(&first, &first_storage);
        model.rearm_statics();
        model.submit_frame(&second, "selected", 1, &frame(&[0.5]));
        let rows = chunks(&second, &second_storage);
        assert_eq!(
            rows.iter()
                .filter(|c| c.is_static()
                    && c.entity_path()
                        .to_string()
                        .trim_start_matches('/')
                        .starts_with("models/"))
                .count(),
            1
        );
        assert_eq!(rows.iter().filter(|c| !c.is_static()).count(), 1);
        assert!(chunks(&first, &first_storage).is_empty());
        assert!(model.status.statics_submitted);
        assert_eq!(model.status.joint_frames_submitted, 1);
    }

    #[test]
    fn initial_static_failure_blocks_reinstallation_without_repeating_accepted_rows() {
        let xml = XML.replace("</robot>", r#"<link name="sensor"/><joint name="sensor_mount" type="fixed"><parent link="tip"/><child link="sensor"/></joint></robot>"#);
        let (rec, storage) = memory();
        chunks(&rec, &storage); // Drain SDK recording metadata before the operation.
        let mut state = SinkState::new();
        let error = state
            .install_bound_model_with(
                &rec,
                "selected",
                load_xml(&xml, vec!["hinge".into()]),
                |model| {
                    model.submit_statics_with(|row| {
                        let StaticRow::Transform(entity, transform) = row else {
                            panic!("expected transform")
                        };
                        if entity.ends_with("sensor") {
                            return Err(submission("injected second-row failure"));
                        }
                        rec.log_static(entity, transform.as_ref())
                            .map_err(submission)
                    })
                },
            )
            .unwrap_err();
        assert!(error.to_string().contains("fresh sink and recording store"));
        let rows = chunks(&rec, &storage);
        assert_eq!(rows.len(), 1);
        assert_eq!(
            rows[0].entity_path(),
            &rerun::EntityPath::from("models/test/arm/tip")
        );
        assert!(rows[0].is_static());
        let translations: Vec<_> = rows[0]
            .iter_component::<rerun::components::Translation3D>(
                rerun::Transform3D::descriptor_translation().component,
            )
            .flat_map(|batch| batch.iter().map(|t| t.0 .0).collect::<Vec<_>>())
            .collect();
        assert_eq!(translations, [[2.0, 0.0, 0.0]]);
        for _ in 0..2 {
            assert!(state
                .install_bound_model(&rec, "selected", loaded())
                .unwrap_err()
                .to_string()
                .contains("fresh sink and recording store"));
            state.rearm_bound_model_statics();
            assert!(state.submit_bound_model_statics(&rec).is_err());
            assert!(
                state.bound_model_status().is_none(),
                "failed model cannot animate"
            );
            assert!(chunks(&rec, &storage).is_empty());
        }
        let (fresh_rec, fresh_storage) = memory();
        chunks(&fresh_rec, &fresh_storage);
        SinkState::new()
            .install_bound_model(&fresh_rec, "selected", loaded())
            .unwrap();
        assert_eq!(chunks(&fresh_rec, &fresh_storage).len(), 1);
    }

    #[test]
    fn initial_static_panic_also_blocks_public_retry() {
        let (rec, storage) = memory();
        chunks(&rec, &storage); // Drain SDK recording metadata before the operation.
        let mut state = SinkState::new();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            state.install_bound_model_with(&rec, "selected", loaded(), |_| {
                panic!("injected SDK unwind")
            })
        }));
        assert!(result.is_err());
        assert!(state
            .install_bound_model(&rec, "selected", loaded())
            .is_err());
        assert!(state.bound_model_status().is_none());
        assert!(chunks(&rec, &storage).is_empty());
    }

    #[test]
    fn initial_binding_and_disabled_recording_preflight_allow_valid_retry() {
        let (rec, storage) = memory();
        chunks(&rec, &storage); // Drain SDK recording metadata before the operation.
        let mut state = SinkState::new();
        assert!(state
            .install_bound_model(
                &rec,
                "selected",
                load_xml(&XML.replace("revolute", "fixed"), Vec::new())
            )
            .is_err());
        assert!(state
            .install_bound_model(&RecordingStream::disabled(), "selected", loaded())
            .is_err());
        assert!(chunks(&rec, &storage).is_empty());
        state
            .install_bound_model(&rec, "selected", loaded())
            .unwrap();
        assert!(state.bound_model_status().unwrap().statics_submitted);
        assert_eq!(chunks(&rec, &storage).len(), 1);
    }

    #[test]
    fn static_retry_resumes_after_the_successful_prefix_and_rearm_starts_a_new_epoch() {
        let xml = XML.replace("</robot>", r#"<link name="sensor"/><joint name="sensor_mount" type="fixed"><parent link="tip"/><child link="sensor"/></joint></robot>"#);
        let (rec, _) = memory();
        let mut model =
            BoundModel::install(&rec, "selected", load_xml(&xml, vec!["hinge".into()])).unwrap();
        model.rearm_statics();
        let mut accepted = Vec::new();
        for _ in 0..3 {
            assert!(model
                .submit_statics_with(|row| {
                    let StaticRow::Transform(entity, _) = row else {
                        panic!("fixture has only transforms")
                    };
                    if entity.ends_with("sensor") {
                        return Err(submission("injected later-row failure"));
                    }
                    accepted.push(entity.to_owned());
                    Ok(())
                })
                .is_err());
            assert_eq!(model.next_static_row, 1);
        }
        assert_eq!(accepted, ["models/test/arm/tip"]);
        model
            .submit_statics_with(|row| {
                let StaticRow::Transform(entity, _) = row else {
                    panic!("expected transform")
                };
                accepted.push(entity.to_owned());
                Ok(())
            })
            .unwrap();
        assert_eq!(
            accepted,
            ["models/test/arm/tip", "models/test/arm/tip/sensor"]
        );
        assert!(model.status.statics_submitted);
        model
            .submit_statics_with(|_| panic!("completed statics must be a no-op"))
            .unwrap();
        model.rearm_statics();
        assert_eq!(model.next_static_row, 0);
    }

    #[test]
    fn idle_statics_resubmit_uses_current_stream_without_a_joint_frame() {
        let (first, _) = rerun::RecordingStreamBuilder::new("idle")
            .recording_id("idle")
            .memory()
            .unwrap();
        let (second, storage) = rerun::RecordingStreamBuilder::new("idle")
            .recording_id("idle")
            .memory()
            .unwrap();
        let mut state = SinkState::new();
        state
            .install_bound_model(&first, "selected", loaded())
            .unwrap();
        state.rearm_bound_model_statics();
        let (wrong, _) = memory();
        assert!(state.submit_bound_model_statics(&wrong).is_err());
        assert!(!state.bound_model_status().unwrap().statics_submitted);
        state.submit_bound_model_statics(&second).unwrap();
        assert!(chunks(&second, &storage).iter().any(|row| row.is_static()
            && row.entity_path() == &rerun::EntityPath::from("models/test/arm/tip")));
        state.submit_bound_model_statics(&second).unwrap();
        assert!(chunks(&second, &storage).is_empty());
        assert_eq!(
            state.bound_model_status().unwrap().joint_frames_submitted,
            0
        );
    }

    #[test]
    fn blank_routes_reject_without_installation_and_preserve_exact_identity() {
        let (rec, storage) = memory();
        chunks(&rec, &storage);
        let mut state = SinkState::new();
        for route in ["", "   ", "\t\r\n", "\u{2003}"] {
            let error = state
                .install_bound_model(&rec, route, loaded())
                .unwrap_err();
            assert!(error.to_string().contains("exact attached route key"));
            assert!(state.bound_model_status().is_none());
            assert!(chunks(&rec, &storage).is_empty());
        }
        let route = "  exact/topic  ";
        state.install_bound_model(&rec, route, loaded()).unwrap();
        assert_eq!(state.bound_model_status().unwrap().route_key, route);
    }

    #[test]
    fn disabled_legacy_empty_binding_and_replacement_fail_loudly() {
        let (rec, _) = memory();
        assert!(BoundModel::install(&rec, "", loaded()).is_err());
        assert!(BoundModel::install(&rec, "selected", Skeleton::inert()).is_err());
        let disabled = RecordingStream::disabled();
        assert!(BoundModel::install(&disabled, "selected", loaded()).is_err());
        let extra_joint = XML.replace(
            "name=\"mount\" type=\"fixed\"",
            "name=\"mount\" type=\"revolute\"",
        );
        let mut partial = load_xml(&extra_joint, vec!["hinge".into(), "mount".into()]);
        partial.model.as_mut().unwrap().motor_bindings.pop();
        assert!(BoundModel::install(&rec, "selected", partial).is_err());
        let mut empty = loaded();
        empty.model.as_mut().unwrap().motor_bindings.clear();
        assert!(BoundModel::install(&rec, "selected", empty).is_err());
        let mut state = SinkState::new();
        state
            .install_bound_model(&rec, "selected", loaded())
            .unwrap();
        assert!(state
            .install_bound_model(&rec, "replacement", loaded())
            .is_err());
        assert!(state.route_for("robot_odom").drives_robot_root);
        assert_eq!(state.route_for("cloud").frame, None);
    }

    #[test]
    fn real_dispatch_preserves_plots_while_every_selected_frame_updates_joints() {
        let schemas = vec![
            parse_rosmsg("float32 q\n", "MotorState", Some("unitree_go")).unwrap(),
            parse_rosmsg(
                "unitree_go/MotorState[20] motor_state\n",
                "LowState",
                Some("unitree_go"),
            )
            .unwrap(),
        ];
        let mut schemas = schemas;
        schemas.push(
            parse_rosmsg(
                "unitree_go/MotorState[20] motor_state\n",
                "OtherState",
                Some("unitree_go"),
            )
            .unwrap(),
        );
        let (mut resolver, _) = LayoutResolver::new(schemas.clone());
        let layout = resolver.layout_of("unitree_go/LowState").unwrap();
        let other_hash = resolver
            .layout_of("unitree_go/OtherState")
            .unwrap()
            .schema_hash;
        let (walker, _) = FrameWalker::new(schemas);
        let (rec, storage) = memory();
        let mut state = SinkState::new();
        state
            .install_bound_model(&rec, "selected", loaded())
            .unwrap();
        chunks(&rec, &storage);
        let mut scalars = Vec::new();
        for tick in 0..100 {
            let mut wire = vec![0; WireHeader::SIZE + 80];
            WireHeader {
                schema_hash: layout.schema_hash,
                total_size: wire.len() as u32,
                offset_table_offset: wire.len() as u32,
                offset_table_count: 0,
                sequence: tick,
                timestamp_ns: 1_000_000_000 + u64::from(tick),
            }
            .write_to_buf(&mut wire[..WireHeader::SIZE]);
            wire[WireHeader::SIZE..WireHeader::SIZE + 4].copy_from_slice(&0.5f32.to_le_bytes());
            dispatch_frame(&rec, &walker, "selected", &wire, &mut state);
            let rows = chunks(&rec, &storage);
            let rotations: Vec<_> = rows
                .iter()
                .filter(|row| row.entity_path() == &rerun::EntityPath::from("models/test/arm"))
                .flat_map(|row| {
                    row.iter_component::<rerun::components::RotationQuat>(
                        rerun::Transform3D::descriptor_quaternion().component,
                    )
                    .flat_map(|batch| batch.iter().map(|q| q.0 .0).collect::<Vec<_>>())
                })
                .collect();
            assert_eq!(rotations.len(), 1, "one measured pose per input frame");
            assert!((rotations[0][2] - 0.24740396).abs() < 1e-6);
            scalars.extend(rows.iter().flat_map(|row| {
                row.iter_component::<rerun::components::Scalar>(
                    rerun::Scalars::descriptor_scalars().component,
                )
                .flat_map(|batch| batch.iter().map(|scalar| scalar.0 .0).collect::<Vec<_>>())
            }));
        }
        assert_eq!(
            state.bound_model_status().unwrap().joint_frames_submitted,
            100
        );
        assert!(state.plot_frames_decimated() > 0);
        assert!(
            scalars.contains(&0.5),
            "generic motor plots retain their measured value"
        );
        for tick in 0..100 {
            let mut wire = vec![0; WireHeader::SIZE + 80];
            WireHeader {
                schema_hash: other_hash,
                total_size: wire.len() as u32,
                offset_table_offset: wire.len() as u32,
                offset_table_count: 0,
                sequence: tick,
                timestamp_ns: 2_000_000_000 + u64::from(tick),
            }
            .write_to_buf(&mut wire[..WireHeader::SIZE]);
            dispatch_frame(&rec, &walker, "selected", &wire, &mut state);
        }
        assert!(
            state.pre_walk_drops() > 0,
            "wrong-schema telemetry retains its cheap pre-walk gate"
        );
        assert_eq!(
            state.bound_model_status().unwrap().joint_frames_submitted,
            100
        );
        assert!(state.bound_model_status().unwrap().rejected_frames > 0);
    }
}
