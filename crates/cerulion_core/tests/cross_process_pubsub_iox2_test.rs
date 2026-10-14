// SPDX-License-Identifier: AGPL-3.0-only

use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wake::{FdWake, FdWakeSet};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

const TOPIC_ENV: &str = "CERULION_XPROC_TOPIC";
const COUNT_ENV: &str = "CERULION_XPROC_COUNT";
const IDLE_FILE_ENV: &str = "CERULION_XPROC_IDLE_FILE";
const DEFAULT_COUNT: usize = 1_000;
const WAIT_TIMEOUT: Duration = Duration::from_secs(10);

fn message_count() -> usize {
    let count = std::env::var(COUNT_ENV)
        .map(|value| value.parse::<usize>().expect("valid message count"))
        .unwrap_or(DEFAULT_COUNT);
    assert!(count > 0, "message count must be positive");
    count
}

fn role_topic() -> Option<String> {
    std::env::var(TOPIC_ENV).ok()
}

fn idle_file() -> PathBuf {
    std::env::var_os(IDLE_FILE_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| panic!("{IDLE_FILE_ENV} must be set for role processes"))
}

fn record_sequence(received: &mut Vec<usize>, payload: &[u8], count: usize) {
    let sequence = sequence_from_payload(payload, count);
    if let Some(previous) = received.last() {
        assert!(
            sequence > *previous,
            "sequences are strictly increasing: {sequence} after {previous}"
        );
    }
    received.push(sequence);
}

/// How one subscriber read ended.
#[derive(Debug, PartialEq, Eq)]
enum ReadOutcome {
    /// Samples were already queued; nothing waited.
    Queued,
    /// The listener descriptor became readable and samples followed.
    Woken,
    /// The listener descriptor became readable with nothing to read.
    WokenEmpty,
    /// The poll reached its timeout, whatever was read afterwards.
    TimedOut,
}

/// Classifies one read: `queued` samples taken without waiting, then, only
/// when none were queued, whether the listener descriptor `fired` before
/// the timeout and how many samples were `delivered` after it.
fn classify_read(queued: usize, fired: bool, delivered: usize) -> ReadOutcome {
    if queued > 0 {
        ReadOutcome::Queued
    } else if !fired {
        ReadOutcome::TimedOut
    } else if delivered > 0 {
        ReadOutcome::Woken
    } else {
        ReadOutcome::WokenEmpty
    }
}

#[test]
fn a_fired_listener_followed_by_samples_is_a_wake() {
    assert_eq!(classify_read(0, true, 1), ReadOutcome::Woken);
    assert_eq!(classify_read(0, true, 7), ReadOutcome::Woken);
}

#[test]
fn a_poll_that_reaches_its_timeout_is_never_a_wake() {
    assert_eq!(classify_read(0, false, 0), ReadOutcome::TimedOut);
    assert_eq!(classify_read(0, false, 5), ReadOutcome::TimedOut);
}

#[test]
fn samples_buffered_before_a_read_are_never_a_wake() {
    assert_eq!(classify_read(3, false, 0), ReadOutcome::Queued);
    assert_eq!(classify_read(1, false, 0), ReadOutcome::Queued);
}

#[test]
fn a_fired_listener_with_nothing_to_read_is_not_a_wake() {
    assert_eq!(classify_read(0, true, 0), ReadOutcome::WokenEmpty);
}

fn sequence_from_payload(payload: &[u8], count: usize) -> usize {
    assert_eq!(payload.len(), 24, "fixed-size Vector3 payload");
    let x = f64::from_le_bytes(payload[0..8].try_into().expect("x bytes"));
    let y = f64::from_le_bytes(payload[8..16].try_into().expect("y bytes"));
    let z = f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"));
    assert!(
        x.is_finite() && x >= 0.0,
        "sequence is a nonnegative number"
    );
    assert_eq!((y, z), (-0.25, 42.0), "payload sentinels");
    let sequence = x as usize;
    assert_eq!(sequence as f64, x, "sequence is an integer");
    assert!(sequence < count, "sequence is below message count");
    sequence
}

#[test]
#[ignore = "role process; started by the parent test or by hand"]
fn subscriber_role() {
    let Some(topic) = role_topic() else {
        return;
    };
    let count = message_count();
    let idle_file = idle_file();
    match fs::remove_file(&idle_file) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => panic!(
            "remove stale subscriber idle file {}: {error}",
            idle_file.display()
        ),
    }
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("xproc_subscriber_{}", std::process::id()),
            ..Default::default()
        },
        cerulion_core::testing::child_iceoryx_config().unwrap_or_default(),
    )
    .expect("initialize subscriber transport");
    let subscriber = manager
        .create_subscriber_with_buffers(
            &topic,
            TopicServiceConfig::for_topology(
                manager.default_topic_config(),
                count,
                1,
                PublisherProvisioning::SingleWriter,
                0,
                0,
            ),
            count,
        )
        .expect("create subscriber with capacity for every message");

    println!("READY");
    std::io::stdout().flush().expect("flush READY");

    // The publisher waits for the idle file before sending the final sample,
    // so the final read can prove that a cross-process event woke an idle
    // subscriber. Earlier queued reads remain informational.
    let mut wake_set = FdWakeSet::new();
    wake_set.watch(subscriber.event_listener_fd());
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut received = Vec::with_capacity(count);
    let mut wakes = 0_usize;
    let mut queued_reads = 0_usize;
    let mut timeout_drains = 0_usize;
    while count > 1 && received.last().copied() != Some(count - 2) && Instant::now() < deadline {
        subscriber
            .drain_event_notifications()
            .expect("drain stale notifications");
        let queued = subscriber
            .try_receive(|message| record_sequence(&mut received, message.payload(), count))
            .expect("read queued messages");
        let (fired, delivered) = if queued > 0 {
            (false, 0)
        } else {
            let fired = wake_set.wait(WAIT_TIMEOUT) == FdWake::Fired;
            let delivered = subscriber
                .try_receive(|message| record_sequence(&mut received, message.payload(), count))
                .expect("read woken messages");
            (fired, delivered)
        };
        match classify_read(queued, fired, delivered) {
            ReadOutcome::Queued => queued_reads += 1,
            ReadOutcome::Woken => wakes += 1,
            ReadOutcome::TimedOut => timeout_drains += 1,
            ReadOutcome::WokenEmpty => {}
        }
    }

    subscriber
        .drain_event_notifications()
        .expect("drain notifications before idle handshake");
    let queued_before_idle = subscriber
        .try_receive(|message| record_sequence(&mut received, message.payload(), count))
        .expect("read queued messages before idle handshake");
    assert_eq!(
        queued_before_idle, 0,
        "no message beyond count-1 before the idle handshake"
    );
    File::create(&idle_file).expect("create subscriber idle handshake file");
    let final_deadline = Instant::now() + WAIT_TIMEOUT;
    let final_read = loop {
        let remaining = final_deadline.saturating_duration_since(Instant::now());
        let fired = wake_set.wait(remaining) == FdWake::Fired;
        let delivered = subscriber
            .try_receive(|message| record_sequence(&mut received, message.payload(), count))
            .expect("read final woken message");
        let final_read = classify_read(0, fired, delivered);
        if final_read != ReadOutcome::WokenEmpty || Instant::now() >= final_deadline {
            break final_read;
        }
        subscriber
            .drain_event_notifications()
            .expect("drain empty wake notifications");
    };

    let first = received.first().copied().unwrap_or_default();
    let last = received.last().copied().unwrap_or_default();
    let gaps = received
        .windows(2)
        .map(|pair| pair[1] - pair[0] - 1)
        .sum::<usize>();
    println!(
        "received={} first={first} last={last} gaps={gaps}",
        received.len()
    );
    println!("wakes={wakes} queued_reads={queued_reads} timeout_drains={timeout_drains}");
    println!("final_read={final_read:?}");
    assert_eq!(received.len(), count, "receive every published message");
    assert_eq!(first, 0);
    assert_eq!(last, count - 1);
    assert_eq!(gaps, 0);
    assert_eq!(
        final_read,
        ReadOutcome::Woken,
        "the publisher's final event woke the idle subscriber"
    );
    assert_eq!(timeout_drains, 0, "no wait ended by timeout");
}

#[test]
#[ignore = "role process; started by the parent test or by hand"]
fn publisher_role() {
    let Some(topic) = role_topic() else {
        return;
    };
    let count = message_count();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("xproc_publisher_{}", std::process::id()),
            ..Default::default()
        },
        cerulion_core::testing::child_iceoryx_config().unwrap_or_default(),
    )
    .expect("initialize publisher transport");
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(64))
        .expect("create publisher");
    let idle_file = idle_file();

    let deadline = Instant::now() + Duration::from_secs(10);
    while manager.topic_subscriber_count(&topic) == 0 {
        assert!(
            Instant::now() < deadline,
            "subscriber connects before publisher deadline"
        );
        thread::sleep(Duration::from_millis(10));
    }

    let mut publish_sequence = |sequence: usize| {
        let mut message = publisher.loan_proxy::<Vector3>().expect("loan message");
        message.x = sequence as f64;
        message.y = -0.25;
        message.z = 42.0;
        drop(message);
    };
    for sequence in 0..count - 1 {
        publish_sequence(sequence);
        thread::sleep(Duration::from_millis(1));
    }
    let idle_deadline = Instant::now() + Duration::from_secs(20);
    while !idle_file.exists() {
        assert!(
            Instant::now() < idle_deadline,
            "subscriber did not create idle handshake file before publisher deadline: {}",
            idle_file.display()
        );
        thread::sleep(Duration::from_millis(10));
    }
    publish_sequence(count - 1);
    thread::sleep(Duration::from_millis(1));
    println!("published={count}");
}

fn role_command(
    role: &str,
    topic: &str,
    count: usize,
    root: &Path,
    prefix: &str,
    idle_file: &Path,
    log_path: &Path,
) -> Command {
    let stdout = File::create(log_path).expect("create child log");
    let stderr = stdout.try_clone().expect("clone child log");
    let mut command = Command::new(std::env::current_exe().expect("current test executable"));
    command
        .args([
            "--exact",
            role,
            "--include-ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(TOPIC_ENV, topic)
        .env(COUNT_ENV, count.to_string())
        .env(IDLE_FILE_ENV, idle_file)
        .env(cerulion_core::testing::CHILD_IOX2_ROOT_ENV, root)
        .env(cerulion_core::testing::CHILD_IOX2_PREFIX_ENV, prefix)
        .stdout(Stdio::from(stdout))
        .stderr(Stdio::from(stderr));
    command
}

/// Stops and reaps a role process that is still running when the parent
/// returns or panics, before the isolated root it uses is removed.
struct RoleProcess(Child);

impl Drop for RoleProcess {
    fn drop(&mut self) {
        if matches!(self.0.try_wait(), Ok(None)) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

fn wait_for_ready(child: &mut Child, log_path: &Path) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let output = fs::read_to_string(log_path).map_err(|error| error.to_string())?;
        if output
            .lines()
            .any(|line| line.split_whitespace().any(|word| word == "READY"))
        {
            return Ok(());
        }
        if let Some(status) = child.try_wait().map_err(|error| error.to_string())? {
            return Err(format!(
                "subscriber exited before READY with {status}:\n{output}"
            ));
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let status = child.wait().map_err(|error| error.to_string())?;
            return Err(format!(
                "subscriber timed out waiting for READY and was stopped with {status}:\n{output}"
            ));
        }
        thread::sleep(Duration::from_millis(10));
    }
}

fn wait_for_child(child: &mut Child, timeout: Duration) -> Result<ExitStatus, String> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(status) = child.try_wait().map_err(|error| error.to_string())? {
            return Ok(status);
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let status = child.wait().map_err(|error| error.to_string())?;
            return Err(format!(
                "child exceeded {timeout:?} and was stopped with {status}"
            ));
        }
        thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn a_subscriber_process_receives_every_message_a_publisher_process_sends() {
    let count = DEFAULT_COUNT;
    let isolated_root = tempfile::tempdir().expect("create isolated root parent");
    let root = isolated_root.path().join("iox2");
    fs::create_dir_all(&root).expect("create isolated iceoryx2 root");
    let prefix = format!("xproc_{}", std::process::id());
    let topic = format!("/xproc/{}", std::process::id());
    let subscriber_log = isolated_root.path().join("subscriber.log");
    let publisher_log = isolated_root.path().join("publisher.log");
    let idle_file = isolated_root.path().join("subscriber.idle");

    let mut subscriber = RoleProcess(
        role_command(
            "subscriber_role",
            &topic,
            count,
            &root,
            &prefix,
            &idle_file,
            &subscriber_log,
        )
        .spawn()
        .expect("spawn subscriber process"),
    );
    wait_for_ready(&mut subscriber.0, &subscriber_log).unwrap_or_else(|error| panic!("{error}"));

    let mut publisher = RoleProcess(
        role_command(
            "publisher_role",
            &topic,
            count,
            &root,
            &prefix,
            &idle_file,
            &publisher_log,
        )
        .spawn()
        .expect("spawn publisher process"),
    );

    let subscriber_status = wait_for_child(&mut subscriber.0, Duration::from_secs(35));
    let publisher_status = wait_for_child(&mut publisher.0, Duration::from_secs(15));
    let subscriber_output = fs::read_to_string(&subscriber_log).expect("read subscriber output");
    let publisher_output = fs::read_to_string(&publisher_log).expect("read publisher output");
    assert!(
        subscriber_status.as_ref().is_ok_and(ExitStatus::success),
        "subscriber process failed: {subscriber_status:?}\n{subscriber_output}"
    );
    assert!(
        publisher_status.as_ref().is_ok_and(ExitStatus::success),
        "publisher process failed: {publisher_status:?}\n{publisher_output}"
    );
    assert!(
        subscriber_output.contains(&format!(
            "received={count} first=0 last={} gaps=0",
            count - 1
        )),
        "subscriber did not report a complete gap-free stream:\n{subscriber_output}"
    );
    assert!(
        subscriber_output.contains("timeout_drains=0"),
        "subscriber reported a timed out phase 1 read:\n{subscriber_output}"
    );
    assert!(
        subscriber_output.contains("final_read=Woken"),
        "subscriber's final read was not woken by the publisher:\n{subscriber_output}"
    );
    assert!(
        publisher_output.contains(&format!("published={count}")),
        "publisher did not report every message:\n{publisher_output}"
    );
}
