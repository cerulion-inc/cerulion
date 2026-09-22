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
    let _statics = crate::test_support::blueprint_statics_guard();
    let (_dir, path) = fixture();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("model-idle-reconnect")
        .memory()
        .unwrap();
    let mut state = SinkState::new();
    state
        .install_bound_model(&rec, "exact", Skeleton::try_load(&path, &config()).unwrap())
        .unwrap();
    rec.flush_blocking().unwrap();
    storage.take(); // The replacement server starts empty.
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
    let (walker, _) = FrameWalker::new(Vec::new());
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
            .filter(|chunk| chunk.entity_path() == &"models/test/arm/tip".into())
            .collect();
        assert_eq!(model_rows.len(), expected_rows);
        assert!(model_rows.iter().all(|chunk| chunk.is_static()));
        assert!(state.bound_model_status().unwrap().statics_submitted);
        assert_eq!(
            state.bound_model_status().unwrap().joint_frames_submitted,
            0
        );
    }
    assert_eq!(
        counters
            .reconnects
            .load(std::sync::atomic::Ordering::Relaxed),
        1
    );
}
