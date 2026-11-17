// SPDX-License-Identifier: AGPL-3.0-only
//! `serve_planes` drops its shutdown future before it runs the ops plane's
//! shutdown stage, on a stop request and when the endpoint closes on its own, so
//! a host that watches the future (the binary arms its hard-exit deadline then)
//! learns the accept loop stopped before any participant runs. The endpoint is
//! open during the stage after a stop request and closed when it closed on its
//! own. Real iroh endpoints with relays disabled; every wait is bounded.

mod common;

use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use cerulion_link::Endpoint;
use cerulion_remoted::serve_planes;
use cerulion_remoted::shutdown::ShutdownParticipant;

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
