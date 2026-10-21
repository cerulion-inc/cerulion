// SPDX-License-Identifier: AGPL-3.0-only
use cerulion_android::{codes, EmbedError, EmbeddedRuntime, RuntimeState};
use tempfile::tempdir;

#[test]
fn process_transport_identity_is_fixed_by_the_first_start() {
    let temp = tempdir().unwrap();
    let first_dir = temp.path().join("first");
    let second_dir = temp.path().join("second");
    std::fs::create_dir(&first_dir).unwrap();
    std::fs::create_dir(&second_dir).unwrap();

    let mut first = EmbeddedRuntime::new(first_dir, "robot_a".into()).unwrap();
    first.start("{}").unwrap();
    first.stop().unwrap();
    first.start("{}").unwrap();
    first.stop().unwrap();

    let mut second = EmbeddedRuntime::new(second_dir, "robot_b".into()).unwrap();
    let error = second
        .start("{}")
        .expect_err("the process cannot change robot identity after startup");
    let expected = "the transport identity (robot name) is fixed for the process lifetime by the first start; restart the app process to change it";
    assert!(matches!(
        error,
        EmbedError::BadState(ref message) if message == expected
    ));
    let status_handle = second.status_handle();
    let status = status_handle.lock().unwrap();
    assert_eq!(status.state, RuntimeState::Failed);
    assert_eq!(
        status.last_error.as_ref().map(|error| error.code),
        Some(codes::BAD_STATE)
    );
    assert_eq!(
        status
            .last_error
            .as_ref()
            .map(|error| error.message.as_str()),
        Some(expected)
    );
}
