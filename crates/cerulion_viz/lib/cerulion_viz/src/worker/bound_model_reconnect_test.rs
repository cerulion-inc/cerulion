// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use crate::skeleton::{Skeleton, UrdfConfig};
use std::path::PathBuf;

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

#[test]
fn model_load_idle_reconnect_submits_statics_without_replaying_joint_frames() {
    verify_idle_reconnect(false);
}

#[test]
fn idle_reconnect_does_not_replay_a_previously_delivered_joint_frame() {
    verify_idle_reconnect(true);
}

fn verify_idle_reconnect(seed_joint_frame: bool) {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("model-idle-reconnect")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    let (walker, hash) = lowstate_walker();
    if seed_joint_frame {
        process_batch(
            &rec,
            &walker,
            &mut state,
            vec![InputFrames {
                name: "exact".into(),
                frames: vec![lowstate_frame(hash, 0, 0.5)],
            }],
        );
    }
    assert_eq!(
        state.bound_model_status().unwrap().joint_frames_submitted,
        u64::from(seed_joint_frame)
    );
    rec.flush_blocking().unwrap();
    let initial = storage.take();
    if seed_joint_frame {
        assert!(
            initial.iter().any(|message| match message {
                rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                    let chunk = rerun::log::Chunk::from_arrow_msg(arrow).unwrap();
                    chunk.entity_path() == &"models/test/arm".into() && !chunk.is_static()
                }
                _ => false,
            }),
            "the seed must deliver an actual temporal joint row"
        );
    }
    let first = std::sync::atomic::AtomicBool::new(true);
    let hooks = super::ReconnectHooks::with_hooks(
        move |_| {
            if first.swap(false, std::sync::atomic::Ordering::Relaxed) {
                Err(rerun::sink::SinkFlushError::failed("server bounced"))
            } else {
                Ok(())
            }
        },
        |_| Ok(()),
    );
    let counters = super::VizWorkerCounters::default();
    let mut last_probe = Instant::now();
    let mut latch = super::FieldsWarnLatch::new();
    for expected_rows in [1, 0] {
        super::handle_message(
            &rec,
            &walker,
            &mut state,
            &hooks,
            &counters,
            &mut last_probe,
            &mut latch,
            Duration::ZERO,
            None,
        );
        rec.flush_blocking().unwrap();
        let model_rows: Vec<_> = storage
            .take()
            .into_iter()
            .filter_map(|message| match message {
                rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                    Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
                }
                _ => None,
            })
            .filter(|chunk| {
                chunk
                    .entity_path()
                    .to_string()
                    .trim_start_matches('/')
                    .starts_with("models/")
            })
            .collect();
        assert_eq!(model_rows.len(), expected_rows);
        assert!(model_rows.iter().all(|chunk| chunk.is_static()));
        assert!(state.bound_model_status().unwrap().statics_submitted);
        assert_eq!(
            state.bound_model_status().unwrap().joint_frames_submitted,
            u64::from(seed_joint_frame)
        );
    }
    assert_eq!(
        counters
            .reconnects
            .load(std::sync::atomic::Ordering::Relaxed),
        1
    );
}

fn lowstate_walker() -> (FrameWalker, u64) {
    use cerulion_core::codegen::{layout::LayoutResolver, parse_rosmsg};
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
    let hash = resolver
        .layout_of("unitree_go/LowState")
        .unwrap()
        .schema_hash;
    let (walker, _) = FrameWalker::new(schemas);
    (walker, hash)
}

fn lowstate_frame(hash: u64, tick: u32, angle: f32) -> Vec<u8> {
    use cerulion_core::wire::WireHeader;
    let mut frame = vec![0; WireHeader::SIZE + 80];
    WireHeader {
        schema_hash: hash,
        total_size: frame.len() as u32,
        offset_table_offset: frame.len() as u32,
        offset_table_count: 0,
        sequence: tick,
        timestamp_ns: 1_000_000_000 + u64::from(tick),
    }
    .write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..WireHeader::SIZE + 4].copy_from_slice(&angle.to_le_bytes());
    frame
}

#[test]
fn one_worker_batch_submits_latest_valid_pose_and_preserves_unrelated_plots() {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("batch-articulation")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    rec.flush_blocking().unwrap();
    storage.take();
    let (walker, hash) = lowstate_walker();
    state.rearm_bound_model_statics();
    let mut frames: Vec<_> = (0..99)
        .map(|tick| lowstate_frame(hash, tick, 0.0))
        .collect();
    frames.push(lowstate_frame(hash, 99, std::f32::consts::FRAC_PI_2));
    frames.push(lowstate_frame(hash, 100, f32::NAN));
    process_batch(
        &rec,
        &walker,
        &mut state,
        vec![
            InputFrames {
                name: "exact".into(),
                frames,
            },
            InputFrames {
                name: "other".into(),
                frames: vec![lowstate_frame(hash, 101, 0.25)],
            },
        ],
    );
    assert_eq!(
        state.bound_model_status().unwrap().joint_frames_submitted,
        1
    );
    assert_eq!(state.bound_model_status().unwrap().rejected_frames, 1);
    assert!(state
        .bound_model_status()
        .unwrap()
        .last_error
        .as_ref()
        .unwrap()
        .contains("finite"));
    rec.flush_blocking().unwrap();
    let chunks: Vec<_> = storage
        .take()
        .into_iter()
        .filter_map(|message| match message {
            rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
            }
            _ => None,
        })
        .collect();
    let rotations: Vec<_> = chunks
        .iter()
        .filter(|chunk| chunk.entity_path() == &"models/test/arm".into())
        .flat_map(|chunk| {
            chunk
                .iter_component::<rerun::components::RotationQuat>(
                    rerun::Transform3D::descriptor_quaternion().component,
                )
                .flat_map(|batch| batch.iter().map(|q| q.0 .0).collect::<Vec<_>>())
        })
        .collect();
    assert_eq!(rotations.len(), 1);
    assert!((rotations[0][0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
    let scalars: Vec<_> = chunks
        .iter()
        .filter(|chunk| !chunk.entity_path().to_string().starts_with("models/"))
        .flat_map(|chunk| {
            chunk
                .iter_component::<rerun::components::Scalar>(
                    rerun::Scalars::descriptor_scalars().component,
                )
                .flat_map(|batch| batch.iter().map(|v| v.0 .0).collect::<Vec<_>>())
        })
        .collect();
    assert!(
        scalars.contains(&0.25),
        "unrelated measured telemetry must still render"
    );
    assert!(
        scalars.contains(&0.0),
        "selected telemetry retains its plot admission"
    );
}

fn model_rotations(storage: &rerun::sink::MemorySinkStorage) -> Vec<[f32; 4]> {
    storage
        .take()
        .into_iter()
        .filter_map(|message| match message {
            rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
            }
            _ => None,
        })
        .filter(|chunk| chunk.entity_path() == &"models/test/arm".into() && !chunk.is_static())
        .flat_map(|chunk| {
            chunk
                .iter_component::<rerun::components::RotationQuat>(
                    rerun::Transform3D::descriptor_quaternion().component,
                )
                .flat_map(|batch| batch.iter().map(|q| q.0 .0).collect::<Vec<_>>())
                .collect::<Vec<_>>()
        })
        .collect()
}

#[test]
fn articulation_deadline_retains_latest_across_batches_and_flushes_without_input() {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("articulation-deadline")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    let (walker, hash) = lowstate_walker();
    let start = Instant::now();
    for tick in 0..100 {
        state.begin_bound_model_batch();
        let angle = if tick == 99 {
            std::f32::consts::FRAC_PI_2
        } else {
            0.0
        };
        dispatch_frame(
            &rec,
            &walker,
            "exact",
            &lowstate_frame(hash, tick, angle),
            &mut state,
        );
        state.finish_bound_model_batch(&rec, start + Duration::from_micros(u64::from(tick)));
    }
    assert_eq!(
        state.bound_model_status().unwrap().joint_frames_submitted,
        1
    );
    rec.flush_blocking().unwrap();
    assert_eq!(model_rotations(&storage), vec![[0.0, 0.0, 0.0, 1.0]]);
    state.flush_due_bound_model(&rec, start + Duration::from_nanos(16_666_666));
    assert_eq!(
        state.bound_model_status().unwrap().joint_frames_submitted,
        1
    );
    state.flush_due_bound_model(&rec, start + Duration::from_nanos(16_666_667));
    assert_eq!(
        state.bound_model_status().unwrap().joint_frames_submitted,
        2
    );
    assert_eq!(state.bound_model_submission_wait(start), None);
    rec.flush_blocking().unwrap();
    let rotations = model_rotations(&storage);
    assert_eq!(rotations.len(), 1);
    assert!((rotations[0][0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
}

#[test]
fn idle_worker_delivers_retained_final_pose() {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("articulation-idle-tail")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    let (walker, hash) = lowstate_walker();
    let start = Instant::now();
    for (tick, angle) in [(0, 0.0), (1, std::f32::consts::FRAC_PI_2)] {
        state.begin_bound_model_batch();
        dispatch_frame(
            &rec,
            &walker,
            "exact",
            &lowstate_frame(hash, tick, angle),
            &mut state,
        );
        state.finish_bound_model_batch(&rec, start);
    }
    rec.flush_blocking().unwrap();
    storage.take();
    let worker = VizLogWorker::spawn(rec.clone(), walker, state).unwrap();
    let until = Instant::now() + Duration::from_secs(2);
    let mut rotations = Vec::new();
    while rotations.is_empty() && Instant::now() < until {
        rec.flush_blocking().unwrap();
        rotations.extend(model_rotations(&storage));
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(rotations.len(), 1);
    assert!((rotations[0][0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
    assert_eq!(worker.render_panics(), 0);
}

#[test]
fn worker_panic_discards_pending_pose_and_recovers_on_fresh_input() {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("articulation-panic")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    let (walker, hash) = lowstate_walker();
    state.begin_bound_model_batch();
    dispatch_frame(
        &rec,
        &walker,
        "exact",
        &lowstate_frame(hash, 0, 0.0),
        &mut state,
    );
    rec.flush_blocking().unwrap();
    storage.take();
    // Queue before entering the production loop: no startup scheduling race
    // can let its idle path flush the staged pose before the injected panic.
    let (tx, rx) = sync_channel(3);
    tx.send(VizMsg::PanicForTest).unwrap();
    tx.send(VizMsg::Batch(Vec::new())).unwrap();
    tx.send(VizMsg::Batch(vec![InputFrames {
        name: "exact".into(),
        frames: vec![lowstate_frame(hash, 1, std::f32::consts::FRAC_PI_2)],
    }]))
    .unwrap();
    drop(tx);
    let counters = Arc::new(VizWorkerCounters::default());
    run(
        rec.clone(),
        walker,
        state,
        rx,
        counters.clone(),
        ReconnectHooks::production(),
        PROBE_INTERVAL,
    );
    assert_eq!(counters.render_panics.load(Ordering::Relaxed), 1);
    rec.flush_blocking().unwrap();
    let rotations = model_rotations(&storage);
    assert_eq!(
        rotations.len(),
        1,
        "only the fresh pose may reach the recording"
    );
    assert!((rotations[0][0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
}

#[test]
fn clean_shutdown_delivers_a_pose_retained_after_the_last_batch() {
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("articulation-shutdown")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    let (walker, hash) = lowstate_walker();
    let start = Instant::now();
    for (tick, angle) in [(0, 0.0), (1, std::f32::consts::FRAC_PI_2)] {
        state.begin_bound_model_batch();
        dispatch_frame(
            &rec,
            &walker,
            "exact",
            &lowstate_frame(hash, tick, angle),
            &mut state,
        );
        state.finish_bound_model_batch(&rec, start);
    }
    rec.flush_blocking().unwrap();
    storage.take();
    let (tx, rx) = sync_channel(1);
    drop(tx);
    run(
        rec.clone(),
        walker,
        state,
        rx,
        Arc::new(VizWorkerCounters::default()),
        ReconnectHooks::production(),
        PROBE_INTERVAL,
    );
    rec.flush_blocking().unwrap();
    let rotations = model_rotations(&storage);
    assert_eq!(rotations.len(), 1);
    assert!((rotations[0][0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
}
