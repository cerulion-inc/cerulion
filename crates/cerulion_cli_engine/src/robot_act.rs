// SPDX-License-Identifier: AGPL-3.0-only
//! `--robot NAME` on `topic pub`, `topic snapshot` and `service call`: run
//! the operation on a paired robot through its ops-plane act verbs
//! (`topic-publish`, `topic-snapshot`, `service-call`) instead of on this
//! machine.
//!
//! The robot is resolved as `cerulion connect` resolves one (a
//! `~/.cerulion/robots.toml` name or a 64-char-hex endpoint id) and dialed
//! with this machine's paired device key by `cerulion-connectd act`, whose
//! stdout carries the verb's JSON result and whose exit code says how the
//! call ended. A verb error comes back as the robot's `<class>: <message>`
//! text, so the CLI reports the same exit code it would for a local run.

use std::process::Command;
use std::time::Duration;

use serde_json::Value;

use crate::message_text::VerbError;

/// How long the dial and handshake may take on top of the verb's own wait.
const DIAL_ALLOWANCE: Duration = Duration::from_secs(20);

/// Call the act verb `verb` with `args` on the robot `robot`, allowing the
/// verb `verb_wait` for its own waits, and return its JSON result.
pub fn call_robot_verb(
    robot: &str,
    verb: &str,
    args: &Value,
    verb_wait: Duration,
) -> Result<Value, VerbError> {
    let robots = crate::connect_cmd::load_robots();
    let eid = crate::connect_cmd::resolve_eid_with(Some(robot), None, &robots)?;
    let key = crate::connect_cmd::default_connect_key_file()?.ok_or_else(|| {
        VerbError::Usage(format!(
            "this machine has no paired device key, so robot '{robot}' would refuse it: pair \
             first with `cerulion pair {robot}`"
        ))
    })?;
    let bin = crate::connect_cmd::resolve_connectd_bin()?;
    let timeout = verb_wait.saturating_add(DIAL_ALLOWANCE);
    let output = Command::new(&bin)
        .arg("act")
        .arg("--eid")
        .arg(&eid)
        .arg("--key-file")
        .arg(&key)
        .arg("--verb")
        .arg(verb)
        .arg("--args")
        .arg(args.to_string())
        .arg("--timeout")
        .arg(timeout.as_secs_f64().to_string())
        .output()
        .map_err(|e| VerbError::Internal(format!("could not run {}: {e}", bin.display())))?;
    let stderr = String::from_utf8_lossy(&output.stderr);
    outcome(
        output.status.code(),
        &output.stdout,
        &error_line(&stderr),
        robot,
    )
}

/// The prefix of the stderr line `cerulion-connectd act` writes its error
/// on (its log lines share stderr).
const ERROR_LINE_PREFIX: &str = "act-error: ";

/// The longest robot text a line or response may carry.
const MAX_ROBOT_TEXT: usize = 12 * 1024 * 1024;

/// The text of the last `act-error: ` line of `stderr`; the whole trimmed
/// stderr when there is none. PURE.
pub fn error_line(stderr: &str) -> String {
    stderr
        .lines()
        .rev()
        .find_map(|l| l.strip_prefix(ERROR_LINE_PREFIX))
        .unwrap_or_else(|| stderr.trim())
        .to_string()
}

/// The result of one `cerulion-connectd act` run from its exit code, stdout
/// and stderr. PURE.
pub fn outcome(
    code: Option<i32>,
    stdout: &[u8],
    stderr: &str,
    robot: &str,
) -> Result<Value, VerbError> {
    match code {
        Some(0) => serde_json::from_slice(stdout).map_err(|e| {
            VerbError::Internal(format!(
                "robot '{robot}' answered with text that is not JSON: {e}"
            ))
        }),
        Some(2) => Err(VerbError::from_classed(stderr)),
        Some(3) => Err(VerbError::Unavailable(format!("robot '{robot}': {stderr}"))),
        Some(4) => Err(VerbError::Internal(format!("robot '{robot}': {stderr}"))),
        Some(1) => Err(VerbError::Usage(format!("cerulion-connectd act: {stderr}"))),
        other => Err(VerbError::Internal(format!(
            "cerulion-connectd act ended with {other:?}: {stderr}"
        ))),
    }
}

/// `text` with control characters escaped, bounded to 256 characters.
fn escaped_bounded(text: &str) -> String {
    text.chars()
        .take(256)
        .flat_map(char::escape_default)
        .collect()
}

/// A string field of a verb result.
pub fn result_str<'a>(result: &'a Value, key: &str) -> Result<&'a str, VerbError> {
    result
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| VerbError::Internal(format!("the robot's result has no string `{key}`")))
}

/// `topic pub --robot`: publish on the robot and return the line the local
/// verb prints.
pub fn robot_pub(
    robot: &str,
    req: &crate::topic_cmd::publish::PubRequest<'_>,
) -> Result<String, VerbError> {
    let period = crate::topic_cmd::publish::publish_period(req.times, req.rate_hz)?;
    let frames_wait = cerulion_msgio::publish::frames_span(period, req.times);
    let args = serde_json::json!({
        "topic": req.topic,
        "type": req.type_name,
        "message": req.message,
        "times": req.times,
        "rate_hz": req.rate_hz,
        "wait_subscribers_s": req.wait_subscribers.as_secs_f64(),
    });
    let wait = req.wait_subscribers.saturating_add(frames_wait);
    let result = call_robot_verb(robot, "topic-publish", &args, wait)?;
    let count = |key: &str| {
        result
            .get(key)
            .and_then(Value::as_u64)
            .ok_or_else(|| VerbError::Internal(format!("the robot's result has no count `{key}`")))
    };
    let type_name = result_str(&result, "type")?;
    if type_name.is_empty()
        || !type_name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || "/_".contains(c))
    {
        return Err(VerbError::Internal(
            "the robot's result names an invalid type".to_string(),
        ));
    }
    Ok(crate::topic_cmd::publish::PubReport {
        topic: req.topic.to_string(),
        type_name: type_name.to_string(),
        frames: count("frames")?,
        subscribers: usize::try_from(count("subscribers")?).unwrap_or(usize::MAX),
    }
    .line())
}

/// `service call --robot`: call the service on the robot and return the
/// response line the local verb prints.
pub fn robot_service_call(
    robot: &str,
    req: &crate::service_cmd::CallRequest<'_>,
) -> Result<String, VerbError> {
    let args = serde_json::json!({
        "service": req.service,
        "type": req.type_name,
        "request": req.request,
        "timeout_s": req.timeout.as_secs_f64(),
    });
    let result = call_robot_verb(robot, "service-call", &args, req.timeout)?;
    response_line(result_str(&result, "response_text")?).map(str::to_string)
}

/// `text` when it is JSON within the size bound with no raw control
/// character (a terminal acts on DEL, C1 and carriage return), the shape
/// the local verb prints. PURE.
pub fn response_line(text: &str) -> Result<&str, VerbError> {
    if text.len() > MAX_ROBOT_TEXT
        || text.chars().any(char::is_control)
        || serde_json::from_str::<Value>(text).is_err()
    {
        return Err(VerbError::Internal(
            "the robot's response text is not one line of JSON free of control characters"
                .to_string(),
        ));
    }
    Ok(text)
}

/// `topic snapshot --robot`: take the snapshot on the robot, write the JPEG
/// on this machine (`req.output`, or the default file in a new private
/// directory under the system temp directory) and return the JSON line the
/// local verb prints.
pub fn robot_snapshot(
    robot: &str,
    req: &crate::topic_cmd::snapshot::SnapshotRequest<'_>,
) -> Result<String, VerbError> {
    let args = serde_json::json!({
        "topic": req.topic,
        "timeout_s": req.timeout.as_secs_f64(),
        "quality": req.quality,
    });
    let result = call_robot_verb(robot, "topic-snapshot", &args, req.timeout)?;
    write_snapshot(&result, req)
}

/// Write the JPEG of a `topic-snapshot` result on this machine
/// ([`cerulion_msgio::snapshot::write_jpeg`]: `req.output`, or the default
/// file in a new private directory under the system temp directory) and
/// return the JSON line the local verb prints, naming the local path.
pub fn write_snapshot(
    result: &Value,
    req: &crate::topic_cmd::snapshot::SnapshotRequest<'_>,
) -> Result<String, VerbError> {
    use cerulion_msgio::snapshot::{base64_decode, write_jpeg, SnapshotReport};
    let field = |key: &str| {
        result.get(key).and_then(Value::as_i64).ok_or_else(|| {
            VerbError::Internal(format!("the robot's result has no integer `{key}`"))
        })
    };
    let jpeg = base64_decode(result_str(result, "jpeg_base64")?)
        .ok_or_else(|| VerbError::Internal("the robot's JPEG is not base64".to_string()))?;
    let stamp_ns = field("stamp_ns")?;
    let path = write_jpeg(req.output, req.topic, stamp_ns, &jpeg)?;
    let dimension = |key: &str| {
        field(key).and_then(|v| {
            u32::try_from(v)
                .map_err(|_| VerbError::Internal(format!("the robot's `{key}` is out of range")))
        })
    };
    Ok(SnapshotReport {
        path,
        width: dimension("width")?,
        height: dimension("height")?,
        encoding: escaped_bounded(result_str(result, "encoding")?),
        stamp_ns,
        bytes: jpeg.len(),
    }
    .json_line())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message_text::{EXIT_ENCODE, EXIT_UNAVAILABLE, EXIT_USAGE};

    #[test]
    fn a_verb_error_keeps_the_local_exit_code_of_its_class() {
        let e = outcome(
            Some(2),
            b"",
            "unavailable: no frame on '/cam' within 5 s",
            "arm",
        )
        .unwrap_err();
        assert_eq!(e.exit_code(), EXIT_UNAVAILABLE);
        assert_eq!(e.to_string(), "no frame on '/cam' within 5 s");
        let e = outcome(Some(2), b"", "encode: data: 256 is out of range", "arm").unwrap_err();
        assert_eq!(e.exit_code(), EXIT_ENCODE);
        let e = outcome(Some(2), b"", "usage: unknown message type 'x/Y'", "arm").unwrap_err();
        assert_eq!(e.exit_code(), EXIT_USAGE);
    }

    #[test]
    fn transport_outcomes_map_to_unavailable_and_refusal() {
        assert_eq!(
            outcome(Some(3), b"", "no answer", "arm")
                .unwrap_err()
                .exit_code(),
            EXIT_UNAVAILABLE
        );
        assert_eq!(
            outcome(Some(4), b"", "denied", "arm")
                .unwrap_err()
                .exit_code(),
            1
        );
        assert_eq!(outcome(None, b"", "", "arm").unwrap_err().exit_code(), 1);
        assert_eq!(
            outcome(Some(1), b"", "--args is not JSON", "arm")
                .unwrap_err()
                .exit_code(),
            EXIT_USAGE
        );
    }

    #[test]
    fn the_error_text_is_the_last_marked_stderr_line() {
        let stderr =
            "\u{1b}[2m2026 INFO endpoint bound\u{1b}[0m\nact-error: unavailable: no frame\n";
        assert_eq!(error_line(stderr), "unavailable: no frame");
        let e = outcome(Some(2), b"", &error_line(stderr), "arm").unwrap_err();
        assert_eq!(e.exit_code(), EXIT_UNAVAILABLE);
        assert_eq!(error_line("  plain text \n"), "plain text");
    }

    #[test]
    fn a_response_line_with_a_raw_control_character_is_refused() {
        assert_eq!(response_line(r#"{"ok":true}"#).unwrap(), r#"{"ok":true}"#);
        for bad in [
            "{\"a\":\"\u{7f}\"}",
            "{\"a\":\"\u{9b}2J\"}",
            "{\"ok\":true}\r",
            "{\"ok\":\ntrue}",
            "not json",
        ] {
            assert!(response_line(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn a_robot_snapshot_is_written_on_this_machine_and_the_line_names_the_local_path() {
        use crate::topic_cmd::snapshot::SnapshotRequest;
        use crate::topic_cmd::TopicScope;
        let jpeg = cerulion_msgio::snapshot::to_jpeg(
            &cerulion_msgio::snapshot::RawImage {
                width: 2,
                height: 1,
                step: 2,
                encoding: "mono8",
                data: &[10, 200],
            },
            80,
        )
        .unwrap();
        let result = serde_json::json!({
            "jpeg_base64": cerulion_msgio::snapshot::base64_encode(&jpeg),
            "width": 2, "height": 1, "encoding": "mono8", "stamp_ns": 42, "bytes": jpeg.len(),
        });
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("cam.jpg");
        let req = SnapshotRequest {
            topic: "/wrist/image_raw",
            output: Some(&path),
            timeout: std::time::Duration::from_secs(1),
            quality: 80,
            scope: TopicScope::Automatic,
        };
        let line = write_snapshot(&result, &req).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), jpeg);
        let v: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(
            v,
            serde_json::json!({"path": path.to_string_lossy(), "width": 2, "height": 1,
                "encoding": "mono8", "stamp_ns": 42, "bytes": jpeg.len()})
        );
        // No -o: the default file in a new private directory under the
        // system temp directory, past a symlink planted at the name the
        // default path had in the temp directory itself.
        let req = SnapshotRequest {
            topic: "/wrist/image_raw_symlink_probe",
            output: None,
            ..req
        };
        let planted = std::env::temp_dir().join(format!(
            "cerulion-snapshot-wrist_image_raw_symlink_probe-{}.jpg",
            std::process::id()
        ));
        let mut stamped = result.clone();
        stamped["stamp_ns"] = Value::from(std::process::id());
        let victim = dir.path().join("victim");
        std::fs::write(&victim, b"keep").unwrap();
        let _ = std::fs::remove_file(&planted);
        #[cfg(unix)]
        std::os::unix::fs::symlink(&victim, &planted).unwrap();
        let line = write_snapshot(&stamped, &req);
        let _ = std::fs::remove_file(&planted);
        let v: Value = serde_json::from_str(&line.unwrap()).unwrap();
        let written = std::path::PathBuf::from(v["path"].as_str().unwrap());
        assert_eq!(std::fs::read(&victim).unwrap(), b"keep");
        assert_eq!(written.file_name(), planted.file_name());
        assert_eq!(
            written.parent().and_then(std::path::Path::parent),
            Some(std::env::temp_dir().as_path())
        );
        assert_eq!(std::fs::read(&written).unwrap(), jpeg);
        std::fs::remove_dir_all(written.parent().unwrap()).unwrap();
        // A result whose JPEG is not base64 is refused.
        let mut bad = result.clone();
        bad["jpeg_base64"] = Value::from("***");
        assert!(write_snapshot(&bad, &req).is_err());
    }

    #[test]
    fn a_result_is_the_json_on_stdout() {
        let v = outcome(Some(0), br#"{"line":"published: x"}"#, "", "arm").unwrap();
        assert_eq!(result_str(&v, "line").unwrap(), "published: x");
        assert!(result_str(&v, "missing").is_err());
        assert!(outcome(Some(0), b"not json", "", "arm").is_err());
    }
}
