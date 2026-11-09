// SPDX-License-Identifier: AGPL-3.0-only
//! `serve_planes` drops its shutdown future before it runs the ops plane's
//! shutdown stage, on a stop request and when the endpoint closes on its own, so
//! a host that watches the future (the binary arms its hard-exit deadline then)
//! learns the accept loop stopped before any participant runs. The endpoint is
//! open during the stage after a stop request and closed when it closed on its
//! own. The drop also comes before the line that says the loop stopped, so a
//! log write that blocks cannot hold it back. `run` under the network kill
//! switch drops it before its warn line, pinned the same way. Real iroh
//! endpoints with relays disabled; every wait is bounded.

mod common;

use std::future::Future;
use std::io::Write;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use cerulion_link::{Endpoint, RelayConfig};
use cerulion_remoted::shutdown::ShutdownParticipant;
use cerulion_remoted::{run, serve_planes, RemotedConfig};

use common::{bounded, build_robot, disabled_endpoint};

/// Sets its flag when dropped.
struct SetOnDrop(Arc<AtomicBool>);

impl Drop for SetOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::SeqCst);
    }
}

/// Records, when the stage runs it, whether the shutdown future was dropped
/// and whether the endpoint was closed.
struct Witness {
    dropped: Arc<AtomicBool>,
    endpoint: Endpoint,
    seen: Arc<Mutex<Vec<(bool, bool)>>>,
}

impl ShutdownParticipant for Witness {
    fn name(&self) -> &'static str {
        "witness"
    }

    fn shut_down(&self) -> Pin<Box<dyn Future<Output = std::io::Result<()>> + Send + '_>> {
        Box::pin(async move {
            let seen = (
                self.dropped.load(Ordering::SeqCst),
                self.endpoint.is_closed(),
            );
            self.seen.lock().unwrap().push(seen);
            Ok(())
        })
    }
}

/// Serve with the stop request already sent (`stop_by_request`) or with the
/// endpoint already closed and the stop request never sent, and return what
/// the stage saw.
async fn stage_sees(seed: u8, stop_by_request: bool) -> Vec<(bool, bool)> {
    let dir = tempfile::tempdir().unwrap();
    let endpoint = disabled_endpoint([seed; 32]).await;
    let robot = build_robot(dir.path(), *endpoint.id().as_bytes(), true, &[]);
    let dropped = Arc::new(AtomicBool::new(false));
    let seen = Arc::new(Mutex::new(Vec::new()));
    robot.ops.shutdown_stage().register(Arc::new(Witness {
        dropped: dropped.clone(),
        endpoint: endpoint.clone(),
        seen: seen.clone(),
    }));
    let guard = SetOnDrop(dropped);
    let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
    let shutdown = async move {
        let _guard = guard;
        let _ = stopped.await;
    };
    // `stop` stays alive in this frame on the closed-on-its-own path, so only
    // the endpoint closing ends the accept loop there.
    let _stop = if stop_by_request {
        let _ = stop.send(());
        None
    } else {
        bounded("endpoint close", endpoint.close()).await;
        assert!(endpoint.is_closed());
        Some(stop)
    };
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
    bounded("endpoint close", endpoint.close()).await;
    let seen = seen.lock().unwrap();
    seen.clone()
}

#[tokio::test(flavor = "multi_thread")]
async fn on_a_stop_request_the_stage_runs_after_the_drop_with_the_endpoint_open() {
    assert_eq!(stage_sees(0x61, true).await, [(true, false)]);
}

#[tokio::test(flavor = "multi_thread")]
async fn when_the_endpoint_closes_on_its_own_the_future_is_dropped_before_the_stage() {
    assert_eq!(stage_sees(0x62, false).await, [(true, true)]);
}

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
