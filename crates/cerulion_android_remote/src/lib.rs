// SPDX-License-Identifier: AGPL-3.0-only
//! The paired remote plane an Android app hosts.
//!
//! [`Host`] serves the plane of `cerulion_remoted` (one iroh endpoint for
//! `cerulion/wire/2` and `cerulion/ops/1`, every connection checked against the
//! trust store) behind `cerulion_android`'s remote plane seam. An app runtime
//! library that depends on this crate lists it first in `embed_nodes!`:
//!
//! ```ignore
//! cerulion_android::embed_nodes! {
//!     remote: cerulion_android_remote::Host,
//!     "timer" => timer::TimerNodeEntry,
//! }
//! ```
//!
//! A start with `"remote": {"enabled": true}` then binds the plane over the
//! state directory: the device key (`remoted/device_key`), the trust-store
//! MAC key (`remoted/trust_store.mac_key`), the trust store and the device
//! index. A missing or unverifiable one fails the start as not provisioned.
//! The plane reads the app's topics on the app's own transport and names
//! their schemas from the start's schema serving, so the app runs no network
//! daemon. A desk reads the catalog, schemas and topic frames; it cannot
//! publish into the app, and `restart` and `engage-estop` are refused.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_android::remote::{
    BoundRemote, RemoteExit, RemoteHost, RemoteStart, RemoteStartError,
};
use cerulion_android::RemoteRelay;
use cerulion_link::{RelayConfig, RelayUrl};
use cerulion_remoted::{
    BoundPlane, KeyFiles, RemotePlane, RemotedConfig, RemotedError, SchemaSource,
};
use tokio::runtime::Runtime;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;

/// How long a stop waits, in all, for the plane to close its endpoint and
/// for its runtime to shut down.
const STOP_BUDGET: Duration = Duration::from_secs(5);

/// The iroh remote plane host. Register it with `embed_nodes!` (see the
/// [crate docs](crate)).
#[derive(Debug, Clone, Copy, Default)]
pub struct Host;

impl RemoteHost for Host {
    fn bind(&self, start: RemoteStart) -> Result<Box<dyn BoundRemote>, RemoteStartError> {
        let relay = relay_config(&start.relay)?;
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .thread_name("cerulion-remote")
            .enable_all()
            .build()
            .map_err(|error| {
                RemoteStartError::Failed(format!("the remote plane runtime did not start: {error}"))
            })?;
        let layout =
            RemotedConfig::from_state_root(&start.state_root, RelayConfig::Disabled, false);
        let keys = KeyFiles {
            key_file: layout.key_file,
            store_mac_key_file: layout.store_mac_key_file,
        };
        let serving = start.serving;
        let schemas: SchemaSource = Arc::new(move || serving.clone());
        let plane = RemotePlane::in_process(&start.state_root, start.robot, relay, schemas)
            .with_transport(start.manager);
        let bound = runtime.block_on(plane.bind(&keys)).map_err(start_error)?;
        tracing::info!(
            endpoint_id = %bound.endpoint_id(),
            "remote plane bound; it serves once the start commits"
        );
        Ok(Box::new(Hosted {
            endpoint_id: bound.endpoint_id().to_string(),
            bound_sockets: bound.bound_sockets(),
            state: State::Bound(Box::new(bound)),
            runtime,
        }))
    }
}

/// The relay posture of the `relay` key. A URL that does not parse fails the
/// start before anything is bound.
fn relay_config(relay: &RemoteRelay) -> Result<RelayConfig, RemoteStartError> {
    match relay {
        RemoteRelay::Public => Ok(RelayConfig::N0Default),
        RemoteRelay::Disabled => Ok(RelayConfig::Disabled),
        RemoteRelay::Custom(urls) => urls
            .iter()
            .map(|url| {
                RelayUrl::from_str(url).map_err(|error| {
                    RemoteStartError::Failed(format!(
                        "remote.relay URL '{url}' is not a relay URL: {error}"
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()
            .map(RelayConfig::Custom),
    }
}

/// A missing or unverifiable key or trust state is a provisioning gap; the
/// rest is a plane failure.
fn start_error(error: RemotedError) -> RemoteStartError {
    match error {
        RemotedError::DeviceKey(_)
        | RemotedError::MacKey(_)
        | RemotedError::Store(_)
        | RemotedError::Index(_)
        | RemotedError::ProvisioningMismatch(_) => RemoteStartError::NotProvisioned(format!(
            "the remote plane is not provisioned: {error}"
        )),
        other => RemoteStartError::Failed(format!("the remote plane did not bind: {other}")),
    }
}

enum State {
    Bound(Box<BoundPlane>),
    Serving {
        stop: oneshot::Sender<()>,
        stopping: Arc<AtomicBool>,
        task: JoinHandle<Result<(), String>>,
    },
    /// Only while `serve` swaps the state.
    Moving,
}

/// Wait for the serve task to end. When nothing asked it to stop, report how
/// it ended through `exit` and return `Ok`; after a stop, return how it ended
/// for the stop to report, and never call `exit`.
async fn watch(
    serving: JoinHandle<Result<(), RemotedError>>,
    stopping: Arc<AtomicBool>,
    exit: RemoteExit,
) -> Result<(), String> {
    let ended = match serving.await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(error.to_string()),
        Err(error) => Err(format!("the plane task ended: {error}")),
    };
    if stopping.load(Ordering::SeqCst) {
        return ended;
    }
    exit(match ended {
        Ok(()) => "the endpoint closed".to_string(),
        Err(reason) => reason,
    });
    Ok(())
}

struct Hosted {
    endpoint_id: String,
    bound_sockets: Vec<SocketAddr>,
    state: State,
    runtime: Runtime,
}

impl BoundRemote for Hosted {
    fn endpoint_id(&self) -> String {
        self.endpoint_id.clone()
    }

    fn bound_sockets(&self) -> Vec<SocketAddr> {
        self.bound_sockets.clone()
    }

    fn serve(&mut self, exit: RemoteExit) -> Result<(), String> {
        if !matches!(self.state, State::Bound(_)) {
            return Err("the remote plane is already serving".to_string());
        }
        let State::Bound(bound) = std::mem::replace(&mut self.state, State::Moving) else {
            unreachable!("the state was checked above");
        };
        let (stop, stopped) = oneshot::channel::<()>();
        let stopping = Arc::new(AtomicBool::new(false));
        let task_stopping = Arc::clone(&stopping);
        // The plane serves on its own task, so a panic in it ends that task
        // and this one still reports it.
        let serving = self.runtime.spawn(bound.serve(async move {
            let _ = stopped.await;
        }));
        let task = self.runtime.spawn(watch(serving, task_stopping, exit));
        self.state = State::Serving {
            stop,
            stopping,
            task,
        };
        Ok(())
    }

    fn stop(self: Box<Self>) -> Result<(), String> {
        let Hosted { state, runtime, .. } = *self;
        let deadline = std::time::Instant::now() + STOP_BUDGET;
        let result = match state {
            State::Serving {
                stop,
                stopping,
                task,
            } => {
                stopping.store(true, Ordering::SeqCst);
                let _ = stop.send(());
                // The timer is made inside the runtime: it needs its clock.
                match runtime.block_on(async { tokio::time::timeout(STOP_BUDGET, task).await }) {
                    Ok(Ok(Ok(()))) => Ok(()),
                    Ok(Ok(Err(reason))) => {
                        Err(format!("the remote plane did not stop cleanly: {reason}"))
                    }
                    Ok(Err(error)) => Err(format!("the remote plane task failed: {error}")),
                    Err(_) => Err(format!(
                        "the remote plane did not close its endpoint within {} s",
                        STOP_BUDGET.as_secs()
                    )),
                }
            }
            // The endpoint closes as the bound plane drops, inside the
            // runtime it was made in.
            State::Bound(bound) => {
                runtime.block_on(async move { drop(bound) });
                Ok(())
            }
            State::Moving => Ok(()),
        };
        runtime.shutdown_timeout(deadline.saturating_duration_since(std::time::Instant::now()));
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// `watch` over a serve task that ends with `ends`, after a stop or not:
    /// what it returns and every reason it passed to `exit`.
    fn watched(
        ends: impl std::future::Future<Output = Result<(), RemotedError>> + Send + 'static,
        stopped: bool,
    ) -> (Result<(), String>, Vec<String>) {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let exits = Arc::new(Mutex::new(Vec::new()));
        let seen = Arc::clone(&exits);
        let exit: RemoteExit = Box::new(move |reason| seen.lock().unwrap().push(reason));
        let result = runtime.block_on(async {
            let serving = tokio::spawn(ends);
            watch(serving, Arc::new(AtomicBool::new(stopped)), exit).await
        });
        let exits = exits.lock().unwrap().clone();
        (result, exits)
    }

    #[test]
    fn a_plane_that_ends_on_its_own_is_reported_through_exit() {
        assert_eq!(
            watched(async { Ok(()) }, false),
            (Ok(()), vec!["the endpoint closed".to_string()])
        );
        assert_eq!(
            watched(
                async { Err(RemotedError::Ops("no receipt log".into())) },
                false
            ),
            (Ok(()), vec!["ops plane: no receipt log".to_string()])
        );
        let (result, exits) = watched(async { panic!("serve broke") }, false);
        assert_eq!(result, Ok(()));
        assert_eq!(exits.len(), 1);
        assert!(
            exits[0].starts_with("the plane task ended: ") && exits[0].contains("panicked"),
            "{exits:?}"
        );
    }

    #[test]
    fn a_stopped_plane_never_calls_exit_and_returns_how_it_ended() {
        assert_eq!(watched(async { Ok(()) }, true), (Ok(()), Vec::new()));
        assert_eq!(
            watched(
                async { Err(RemotedError::Ops("close failed".into())) },
                true
            ),
            (Err("ops plane: close failed".to_string()), Vec::new())
        );
    }

    fn code(error: RemotedError) -> &'static str {
        match start_error(error) {
            RemoteStartError::NotProvisioned(_) => "not provisioned",
            RemoteStartError::Failed(_) => "failed",
        }
    }

    #[test]
    fn a_missing_or_unverifiable_key_or_trust_state_is_not_provisioned() {
        let store = cerulion_pairing::PairingError::Serialization("bad MAC".into());
        for (error, want) in [
            (RemotedError::DeviceKey("absent".into()), "not provisioned"),
            (RemotedError::MacKey("absent".into()), "not provisioned"),
            (RemotedError::Store(store), "not provisioned"),
            (RemotedError::Index("bad MAC".into()), "not provisioned"),
            (
                RemotedError::ProvisioningMismatch("another device".into()),
                "not provisioned",
            ),
            (RemotedError::Config("bad".into()), "failed"),
            (RemotedError::Provision("bad".into()), "failed"),
        ] {
            let shown = error.to_string();
            assert_eq!(code(error), want, "{shown}");
        }
    }

    #[test]
    fn the_message_says_what_failed() {
        let RemoteStartError::NotProvisioned(message) =
            start_error(RemotedError::DeviceKey("no file".into()))
        else {
            panic!("a missing device key is a provisioning gap");
        };
        assert_eq!(
            message,
            "the remote plane is not provisioned: device key: no file"
        );
        let RemoteStartError::Failed(message) = start_error(RemotedError::Config("x".into()))
        else {
            panic!("a config error is a plane failure");
        };
        assert_eq!(message, "the remote plane did not bind: config: x");
    }
}
