// SPDX-License-Identifier: AGPL-3.0-only

use std::fs::{self, File};
use std::io::Write;
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

const TOPIC_ENV: &str = "CERULION_XPROC_TOPIC";
const COUNT_ENV: &str = "CERULION_XPROC_COUNT";
const DEFAULT_COUNT: usize = 1_000;

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

    let deadline = Instant::now() + Duration::from_secs(30);
    let mut received = Vec::with_capacity(count);
    while received.last().copied() != Some(count - 1) && Instant::now() < deadline {
        subscriber
            .try_receive(|message| {
                let sequence = sequence_from_payload(message.payload(), count);
                if let Some(previous) = received.last() {
                    assert!(
                        sequence > *previous,
                        "sequences are strictly increasing: {sequence} after {previous}"
                    );
                }
                received.push(sequence);
            })
            .expect("receive messages");
        if received.last().copied() != Some(count - 1) {
            thread::sleep(Duration::from_millis(1));
        }
    }

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
    assert_eq!(received.len(), count, "receive every published message");
    assert_eq!(first, 0);
    assert_eq!(last, count - 1);
    assert_eq!(gaps, 0);
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

    let deadline = Instant::now() + Duration::from_secs(10);
    while manager.topic_subscriber_count(&topic) == 0 {
        assert!(
            Instant::now() < deadline,
            "subscriber connects before publisher deadline"
        );
        thread::sleep(Duration::from_millis(10));
    }

    for sequence in 0..count {
        let mut message = publisher.loan_proxy::<Vector3>().expect("loan message");
        message.x = sequence as f64;
        message.y = -0.25;
        message.z = 42.0;
        drop(message);
        thread::sleep(Duration::from_millis(1));
    }
    println!("published={count}");
}

fn role_command(
    role: &str,
    topic: &str,
    count: usize,
    root: &Path,
    prefix: &str,
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
        .env(cerulion_core::testing::CHILD_IOX2_ROOT_ENV, root)
        .env(cerulion_core::testing::CHILD_IOX2_PREFIX_ENV, prefix)
        .stdout(Stdio::from(stdout))
        .stderr(Stdio::from(stderr));
    command
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

    let mut subscriber = role_command(
        "subscriber_role",
        &topic,
        count,
        &root,
        &prefix,
        &subscriber_log,
    )
    .spawn()
    .expect("spawn subscriber process");
    wait_for_ready(&mut subscriber, &subscriber_log).unwrap_or_else(|error| panic!("{error}"));

    let mut publisher = role_command(
        "publisher_role",
        &topic,
        count,
        &root,
        &prefix,
        &publisher_log,
    )
    .spawn()
    .expect("spawn publisher process");

    let subscriber_status = wait_for_child(&mut subscriber, Duration::from_secs(35));
    let publisher_status = wait_for_child(&mut publisher, Duration::from_secs(15));
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
        publisher_output.contains(&format!("published={count}")),
        "publisher did not report every message:\n{publisher_output}"
    );
}
