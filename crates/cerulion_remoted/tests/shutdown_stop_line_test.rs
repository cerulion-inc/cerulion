// SPDX-License-Identifier: AGPL-3.0-only
//! A blocked stop log cannot delay the shutdown future's drop. Both stop paths
//! run under capturing subscribers in this test process, keeping their shared
//! tracing callsites separate from serving tests without subscribers.
//! Real iroh endpoints with relays disabled; every wait is bounded.

mod common;

use std::io::Write;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use cerulion_link::RelayConfig;
use cerulion_remoted::{run, serve_planes, RemotedConfig};

use common::{bounded, build_robot, disabled_endpoint};

/// Set, with a wake, when the shutdown future is dropped.
#[derive(Clone, Default)]
struct Dropped(Arc<(Mutex<bool>, Condvar)>);

impl Drop for SetOnDropAndWake {
    fn drop(&mut self) {
        let (flag, wake) = &*(self.0).0;
        *flag.lock().unwrap() = true;
        wake.notify_all();
    }
}

/// Wakes [`Dropped`] when dropped.
struct SetOnDropAndWake(Dropped);

/// A log writer that holds the line containing `needle` until the shutdown
/// future is dropped, up to 5 s, and records whether the drop came first.
#[derive(Clone)]
struct HoldsTheStopLine {
    needle: &'static str,
    dropped: Dropped,
    drop_came_first: Arc<Mutex<Vec<bool>>>,
}

impl Write for HoldsTheStopLine {
    fn write(&mut self, line: &[u8]) -> std::io::Result<usize> {
        if String::from_utf8_lossy(line).contains(self.needle) {
            let (flag, wake) = &*self.dropped.0;
            let held = flag.lock().unwrap();
            let (held, waited) = wake
                .wait_timeout_while(held, Duration::from_secs(5), |dropped| !*dropped)
                .unwrap();
            drop(held);
            self.drop_came_first
                .lock()
                .unwrap()
                .push(!waited.timed_out());
        }
        Ok(line.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn when_the_endpoint_closes_on_its_own_a_blocked_stop_line_cannot_hold_the_drop() {
    let dir = tempfile::tempdir().unwrap();
    let endpoint = disabled_endpoint([0x63; 32]).await;
    let robot = build_robot(dir.path(), *endpoint.id().as_bytes(), true, &[]);
    let dropped = Dropped::default();
    let writer = HoldsTheStopLine {
        needle: "stopping accept loop",
        dropped: dropped.clone(),
        drop_came_first: Arc::new(Mutex::new(Vec::new())),
    };
    let lines = writer.drop_came_first.clone();
    let subscriber = tracing_subscriber::fmt()
        .with_ansi(false)
        .with_writer(move || writer.clone())
        .finish();
    // This thread's own subscriber: serve_planes runs on it, iroh's tasks run
    // on the runtime's workers.
    let _default = tracing::subscriber::set_default(subscriber);
    let guard = SetOnDropAndWake(dropped);
    let shutdown = async move {
        let _guard = guard;
        std::future::pending::<()>().await;
    };
    let closer = endpoint.clone();
    let closing = tokio::spawn(async move { closer.close().await });
    let served = bounded(
        "serve_planes",
        serve_planes(
            &endpoint,
            robot.classify.clone(),
            Some(robot.ops.clone()),
            None,
            None,
            None,
            shutdown,
        ),
    )
    .await;
    assert!(served.is_ok(), "{served:?}");
    bounded("endpoint close", closing).await.unwrap();
    assert_eq!(
        *lines.lock().unwrap(),
        [true],
        "the shutdown future was dropped before the stop line was written"
    );
}

#[tokio::test]
async fn under_the_kill_switch_a_blocked_warn_line_cannot_hold_the_drop() {
    let dir = tempfile::tempdir().unwrap();
    let config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, true);
    let dropped = Dropped::default();
    let writer = HoldsTheStopLine {
        needle: "network kill-switch engaged",
        dropped: dropped.clone(),
        drop_came_first: Arc::new(Mutex::new(Vec::new())),
    };
    let lines = writer.drop_came_first.clone();
    let subscriber = tracing_subscriber::fmt()
        .with_ansi(false)
        .with_writer(move || writer.clone())
        .finish();
    // `run` refuses to serve on this thread, under this subscriber.
    let _default = tracing::subscriber::set_default(subscriber);
    let guard = SetOnDropAndWake(dropped);
    let shutdown = async move {
        let _guard = guard;
        std::future::pending::<()>().await;
    };
    let ran = bounded("run", run(config, shutdown)).await;
    assert!(ran.is_ok(), "{ran:?}");
    assert_eq!(
        *lines.lock().unwrap(),
        [true],
        "the shutdown future was dropped before the kill-switch line was written"
    );
}
