// SPDX-License-Identifier: AGPL-3.0-only
//! `log echo` over real iceoryx2 shared memory: a live publisher on the log
//! topic sends `rcl_interfaces/Log` records and the stream is checked line by
//! line against hand-written expected lines, through both filters, a closed
//! reader, an interrupt under load and a frame of the wrong type.
//!
//! The log topic name is fixed and host-wide, so this binary reads a private
//! iceoryx2 namespace: before anything opens shared memory it moves into a
//! temporary working directory whose `config/iceoryx2.toml` (the first place
//! iceoryx2 looks) names a private root and prefix. A graph or a ROS 2 node
//! running on the host cannot reach it, and it cannot reach them. The tests
//! share the one process-global transport, so they are `#[serial]`.
//!
//! Run:
//! ```bash
//! cargo test -p cerulion_cli_engine --test log_echo_iox2_test -- --test-threads=1
//! ```

#![cfg(unix)]

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, OnceLock};
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::log_cmd::{log_echo, LogFilter, LogLevel, LOG_TOPIC};
use cerulion_core::{PubSubEvent, ShmMessage, WireHeader};
use native_ros2_messages::rcl_interfaces::Log;
use native_ros2_messages::std_msgs::String as RosString;
use serial_test::serial;

type RawService = iceoryx2::service::ipc::Service;

/// Move the process into a private iceoryx2 namespace, once, before the first
/// shared-memory open. The directory lives as long as the process.
fn private_namespace() {
    static DIR: OnceLock<tempfile::TempDir> = OnceLock::new();
    DIR.get_or_init(|| {
        let tmp = tempfile::tempdir_in("/tmp").expect("tempdir");
        std::fs::create_dir_all(tmp.path().join("config")).expect("config dir");
        let prefix = tmp
            .path()
            .file_name()
            .and_then(|n| n.to_str())
            .expect("utf-8 tempdir name")
            .replace('.', "_");
        std::fs::write(
            tmp.path().join("config").join("iceoryx2.toml"),
            format!(
                "[global]\nroot-path = \"{}\"\nprefix = \"logecho_{prefix}_\"\n",
                tmp.path().join("shm").display()
            ),
        )
        .expect("iceoryx2 config");
        std::env::set_current_dir(tmp.path()).expect("enter the namespace directory");
        tmp
    });
}

/// A raw iceoryx2 writer on `{LOG_TOPIC}/data` that rings `{LOG_TOPIC}/event`
/// after every frame, the way a notifying publisher does.
struct LogTopicWriter {
    _node: iceoryx2::node::Node<RawService>,
    _data: iceoryx2::service::port_factory::publish_subscribe::PortFactory<RawService, [u8], ()>,
    publisher: iceoryx2::port::publisher::Publisher<RawService, [u8], ()>,
    _event: iceoryx2::service::port_factory::event::PortFactory<RawService>,
    notifier: iceoryx2::port::notifier::Notifier<RawService>,
}

impl LogTopicWriter {
    fn new() -> Self {
        private_namespace();
        let node = iceoryx2::node::NodeBuilder::new()
            .create::<RawService>()
            .expect("iceoryx2 node");
        let data_name: iceoryx2::service::service_name::ServiceName = format!("{LOG_TOPIC}/data")
            .as_str()
            .try_into()
            .expect("data name");
        let data = node
            .service_builder(&data_name)
            .publish_subscribe::<[u8]>()
            .subscriber_max_buffer_size(16)
            .open_or_create()
            .expect("data service");
        let publisher = data
            .publisher_builder()
            .initial_max_slice_len(4096)
            .create()
            .expect("publisher");
        let event_name: iceoryx2::service::service_name::ServiceName = format!("{LOG_TOPIC}/event")
            .as_str()
            .try_into()
            .expect("event name");
        let event = node
            .service_builder(&event_name)
            .event()
            .open_or_create()
            .expect("event service");
        let notifier = event.notifier_builder().create().expect("notifier");
        Self {
            _node: node,
            _data: data,
            publisher,
            _event: event,
            notifier,
        }
    }

    fn publish(&self, frame: &[u8]) {
        let mut sample = self.publisher.loan_slice_uninit(frame.len()).expect("loan");
        for (slot, &byte) in sample.payload_mut().iter_mut().zip(frame) {
            slot.write(byte);
        }
        // SAFETY: every byte of the loan was written above.
        let sample = unsafe { sample.assume_init() };
        sample.send().expect("send");
        self.notifier
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .expect("notify");
    }
}

/// One `rcl_interfaces/Log` frame laid out by hand: the wire header, the
/// 16-byte fixed section (`stamp`, `level` and its padding, `line`), the
/// 4-entry offset table and the strings `name`, `msg`, `file`, `function`.
fn log_frame(seq: u32, sec: i32, level: u8, line: u32, strings: [&str; 4]) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.extend_from_slice(&sec.to_le_bytes());
    payload.extend_from_slice(&5u32.to_le_bytes());
    payload.push(level);
    payload.extend_from_slice(&[0, 0, 0]);
    payload.extend_from_slice(&line.to_le_bytes());
    let mut cursor = 16 + 8 * 4;
    for s in strings {
        payload.extend_from_slice(&(cursor as u32).to_le_bytes());
        payload.extend_from_slice(&(s.len() as u32).to_le_bytes());
        cursor += s.len();
    }
    for s in strings {
        payload.extend_from_slice(s.as_bytes());
    }
    frame_with(<Log as ShmMessage>::SCHEMA_HASH, seq, &payload)
}

fn frame_with(schema_hash: u64, seq: u32, payload: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + payload.len();
    let mut frame = vec![0u8; total];
    let mut header = WireHeader::new(schema_hash, seq, 1_000 + u64::from(seq));
    header.total_size = total as u32;
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// The four records every filter test publishes, in order.
fn four_records() -> Vec<Vec<u8>> {
    vec![
        log_frame(0, 100, 20, 7, ["camera", "started", "src/cam.rs", "cam"]),
        log_frame(
            1,
            101,
            30,
            8,
            ["camera", "frame dropped", "src/cam.rs", "cam"],
        ),
        log_frame(2, 102, 40, 9, ["planner", "no path", "src/plan.rs", "plan"]),
        log_frame(3, 103, 10, 10, ["planner", "tick", "src/plan.rs", "plan"]),
    ]
}

type Outcome = (CliResult<()>, String);

/// A writer that hands everything it is given to a shared buffer.
#[derive(Clone, Default)]
struct Shared(Arc<std::sync::Mutex<Vec<u8>>>);

impl Write for Shared {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.lock().expect("buffer").extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Shared {
    fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().expect("buffer")).into_owned()
    }
}

fn spawn_echo<W: Write + Send + 'static>(
    filter: LogFilter,
    running: Arc<AtomicBool>,
    mut writer: W,
) -> (JoinHandle<()>, mpsc::Receiver<CliResult<()>>) {
    let (tx, rx) = mpsc::channel();
    let handle = std::thread::spawn(move || {
        let res = log_echo(&filter, running, &mut writer);
        let _ = tx.send(res);
    });
    (handle, rx)
}

/// Publish `frames` to a live `log echo` with `filter`, stop it, and return
/// its result and everything it printed.
fn echo_records(filter: LogFilter, frames: &[Vec<u8>]) -> Outcome {
    let writer = LogTopicWriter::new();
    let running = Arc::new(AtomicBool::new(true));
    let out = Shared::default();
    let (handle, rx) = spawn_echo(filter, running.clone(), out.clone());
    // The subscriber attaches before the burst, so every frame reaches it.
    std::thread::sleep(Duration::from_millis(500));
    for frame in frames {
        writer.publish(frame);
    }
    std::thread::sleep(Duration::from_millis(500));
    running.store(false, Ordering::Relaxed);
    let res = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("log echo ends once running clears");
    handle.join().expect("echo thread");
    (res, out.text())
}

#[test]
#[serial]
fn every_published_record_prints_once_in_order_with_no_filter() {
    let (res, out) = echo_records(LogFilter::default(), &four_records());
    res.expect("a live log topic streams cleanly");
    assert_eq!(
        out,
        "100.000000005 INFO camera cam src/cam.rs:7 started\n\
         101.000000005 WARN camera cam src/cam.rs:8 frame dropped\n\
         102.000000005 ERROR planner plan src/plan.rs:9 no path\n\
         103.000000005 DEBUG planner plan src/plan.rs:10 tick\n"
    );
}

#[test]
#[serial]
fn the_node_filter_keeps_only_that_nodes_records() {
    let filter = LogFilter {
        nodes: vec!["camera".into()],
        min_level: None,
    };
    let (res, out) = echo_records(filter, &four_records());
    res.expect("streams");
    assert_eq!(
        out,
        "100.000000005 INFO camera cam src/cam.rs:7 started\n\
         101.000000005 WARN camera cam src/cam.rs:8 frame dropped\n"
    );
}

#[test]
#[serial]
fn the_level_filter_keeps_records_at_that_level_or_above() {
    let filter = LogFilter {
        nodes: Vec::new(),
        min_level: Some(LogLevel::Warn),
    };
    let (res, out) = echo_records(filter, &four_records());
    res.expect("streams");
    assert_eq!(
        out,
        "101.000000005 WARN camera cam src/cam.rs:8 frame dropped\n\
         102.000000005 ERROR planner plan src/plan.rs:9 no path\n"
    );
}

#[test]
#[serial]
fn both_filters_together_keep_only_records_that_pass_both() {
    let filter = LogFilter {
        nodes: vec!["planner".into()],
        min_level: Some(LogLevel::Info),
    };
    let (res, out) = echo_records(filter, &four_records());
    res.expect("streams");
    assert_eq!(
        out,
        "102.000000005 ERROR planner plan src/plan.rs:9 no path\n"
    );
}

#[test]
#[serial]
fn two_runs_over_the_same_records_print_identical_bytes() {
    let (first, a) = echo_records(LogFilter::default(), &four_records());
    let (second, b) = echo_records(LogFilter::default(), &four_records());
    first.expect("first run");
    second.expect("second run");
    assert!(!a.is_empty());
    assert_eq!(a, b);
}

#[test]
#[serial]
fn a_frame_of_another_type_ends_the_stream_with_an_error_and_prints_nothing() {
    let writer = LogTopicWriter::new();
    let running = Arc::new(AtomicBool::new(true));
    let out = Shared::default();
    let (handle, rx) = spawn_echo(LogFilter::default(), running.clone(), out.clone());
    std::thread::sleep(Duration::from_millis(500));
    let string_hash = <RosString as ShmMessage>::SCHEMA_HASH;
    writer.publish(&frame_with(string_hash, 0, &[0u8; 12]));
    // The verb ends by itself: `running` is never cleared here.
    let res = rx
        .recv_timeout(Duration::from_secs(5))
        .expect("a wrong-type frame ends the stream without an interrupt");
    running.store(false, Ordering::Relaxed);
    handle.join().expect("echo thread");
    let err = res.expect_err("a wrong-type frame is an error");
    assert_eq!(
        err.to_string(),
        format!(
            "a frame on /rosout is not an rcl_interfaces/Log record: it carries schema \
             0x{string_hash:016x}, not 0x{:016x}, so a publisher puts another type on the topic",
            <Log as ShmMessage>::SCHEMA_HASH
        )
    );
    assert_eq!(out.text(), "");
}

/// Takes `keep` lines, then fails every write as a closed pipe does.
struct ClosesAfter {
    keep: usize,
    seen: Shared,
}

impl Write for ClosesAfter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.keep == 0 {
            return Err(io::Error::from(io::ErrorKind::BrokenPipe));
        }
        if buf.contains(&b'\n') {
            self.keep -= 1;
        }
        self.seen.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Publish records one after another until `stop` is set.
fn spawn_steady_publisher(stop: Arc<AtomicBool>, gap: Duration) -> JoinHandle<u32> {
    std::thread::spawn(move || {
        let writer = LogTopicWriter::new();
        let mut seq = 0u32;
        while !stop.load(Ordering::Relaxed) {
            writer.publish(&log_frame(seq, 200, 20, 1, ["camera", "tick", "f.rs", "t"]));
            seq = seq.wrapping_add(1);
            std::thread::sleep(gap);
        }
        seq
    })
}

#[test]
#[serial]
fn a_closed_reader_ends_the_stream_cleanly_while_records_still_flow() {
    // The topic exists before the verb looks for it.
    let holder = LogTopicWriter::new();
    let stop = Arc::new(AtomicBool::new(false));
    let publisher = spawn_steady_publisher(stop.clone(), Duration::from_millis(2));
    let running = Arc::new(AtomicBool::new(true));
    let seen = Shared::default();
    let (handle, rx) = spawn_echo(
        LogFilter::default(),
        running.clone(),
        ClosesAfter {
            keep: 2,
            seen: seen.clone(),
        },
    );
    let res = rx.recv_timeout(Duration::from_secs(10));
    stop.store(true, Ordering::Relaxed);
    running.store(false, Ordering::Relaxed);
    let published = publisher.join().expect("publisher");
    handle.join().expect("echo thread");
    drop(holder);
    res.expect("a closed reader ends the stream without an interrupt")
        .expect("a closed reader is a clean end, exit 0");
    assert!(published > 2, "records were still flowing ({published})");
    assert_eq!(
        seen.text(),
        "200.000000005 INFO camera t f.rs:1 tick\n".repeat(2)
    );
}

/// Accepts every write, slowly: a reader that cannot keep up with the topic.
/// It counts the writes it took, so a test can tell a stream that ended
/// while records flowed from one that never wrote a record.
struct SlowReader {
    writes: Arc<AtomicUsize>,
}

impl Write for SlowReader {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        self.writes.fetch_add(1, Ordering::Relaxed);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
#[serial]
fn an_interrupt_ends_a_stream_that_the_reader_cannot_keep_up_with() {
    let holder = LogTopicWriter::new();
    let stop = Arc::new(AtomicBool::new(false));
    // About ten records per write the reader can take, so the queue is never
    // empty while every record is rendered and written.
    let publisher = spawn_steady_publisher(stop.clone(), Duration::from_micros(500));
    let running = Arc::new(AtomicBool::new(true));
    let writes = Arc::new(AtomicUsize::new(0));
    let (handle, rx) = spawn_echo(
        LogFilter::default(),
        running.clone(),
        SlowReader {
            writes: writes.clone(),
        },
    );
    std::thread::sleep(Duration::from_millis(800));
    running.store(false, Ordering::Relaxed);
    let res = rx.recv_timeout(Duration::from_secs(3));
    stop.store(true, Ordering::Relaxed);
    let publisher_count = publisher.join().expect("publisher");
    handle.join().expect("echo thread");
    drop(holder);
    res.expect("the interrupt ends the stream while records still flow")
        .expect("an interrupt is a clean end");
    assert!(
        writes.load(Ordering::Relaxed) > 0,
        "records were written before the interrupt, so the stream was flowing"
    );
    assert!(
        publisher_count > 0,
        "the publisher published ({publisher_count})"
    );
}
