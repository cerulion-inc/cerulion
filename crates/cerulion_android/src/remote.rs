// SPDX-License-Identifier: AGPL-3.0-only
//! The paired remote plane, hosted in the app process.
//!
//! A start with `"remote": {"enabled": true}` hosts the plane a paired desk
//! reads the app's catalog and topics through. This crate only drives it:
//! the plane itself is served by a [`RemoteHost`](crate::remote::RemoteHost) the app library links and
//! registers, `cerulion_android_remote::Host` for the iroh plane. An app
//! that hosts no plane links no host and none of its dependencies.
//!
//! A start asks the host to [`bind`](crate::remote::RemoteHost::bind) before any other
//! component is built: the host loads its keys and trust state and binds its
//! endpoint, and serves nothing yet. A fresh install has no keys, so the start
//! fails there with [`codes::REMOTE_NOT_PROVISIONED`](crate::codes) and
//! nothing runs. The commit then asks the bound plane to
//! [`serve`](crate::remote::BoundRemote::serve), as it opens LAN access, and a stop stops
//! the plane before any other component.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use cerulion_core::{SchemaServing, TransportManager};

use crate::config::RemoteRelay;
use crate::error::EmbedError;
use crate::runtime::RemoteStatus;

/// What a start hands the host to bind its plane.
pub struct RemoteStart {
    /// The plane's state directory: `<files_dir>/<state_dir>`. Its keys and
    /// trust state live under `remoted/` in it.
    pub state_root: PathBuf,
    /// The robot identity every reply names.
    pub robot: String,
    /// How desks reach the plane.
    pub relay: RemoteRelay,
    /// The process transport manager: the plane reads the app's topics
    /// through it, so the process keeps one transport node.
    pub manager: Arc<TransportManager>,
    /// The topics this start serves and their schemas: graph outputs, then
    /// publish topics.
    pub serving: SchemaServing,
}

/// Why a host could not bind its plane.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RemoteStartError {
    /// A key or the trust state is absent or does not verify: the plane must
    /// be provisioned before it can serve. Reported as
    /// [`EmbedError::RemoteNotProvisioned`].
    NotProvisioned(String),
    /// Any other failure. Reported as [`EmbedError::Remote`].
    Failed(String),
}

impl From<RemoteStartError> for EmbedError {
    fn from(error: RemoteStartError) -> Self {
        match error {
            RemoteStartError::NotProvisioned(message) => EmbedError::RemoteNotProvisioned(message),
            RemoteStartError::Failed(message) => EmbedError::Remote(message),
        }
    }
}

/// Called once, with the reason, when a plane stops serving on its own. A
/// plane that [`BoundRemote::stop`] stopped never calls it.
pub type RemoteExit = Box<dyn FnOnce(String) + Send>;

/// The remote plane implementation an app library links.
pub trait RemoteHost: Send + Sync {
    /// Load the plane's keys and trust state and bind its endpoint. Serve
    /// nothing until [`BoundRemote::serve`].
    fn bind(&self, start: RemoteStart) -> Result<Box<dyn BoundRemote>, RemoteStartError>;
}

/// A plane whose endpoint is bound.
pub trait BoundRemote: Send {
    /// The endpoint id desks dial and authenticate.
    fn endpoint_id(&self) -> String;
    /// The local sockets the endpoint is bound to.
    fn bound_sockets(&self) -> Vec<SocketAddr>;
    /// Start serving and return. `exit` must not be called from inside this
    /// call: the commit that calls it holds the lock a run failure takes.
    fn serve(&mut self, exit: RemoteExit) -> Result<(), String>;
    /// Stop serving, or drop a plane that never served, and wait until the
    /// endpoint is closed.
    fn stop(self: Box<Self>) -> Result<(), String>;
}

static HOST: Mutex<Option<Box<dyn RemoteHost>>> = Mutex::new(None);

fn host() -> MutexGuard<'static, Option<Box<dyn RemoteHost>>> {
    HOST.lock().unwrap_or_else(|error| error.into_inner())
}

/// Register the remote plane host linked into this library.
///
/// [`embed_nodes!`](crate::embed_nodes) calls it from `JNI_OnLoad` when its
/// list starts with `remote: <host>,`. An app that writes its own
/// `JNI_OnLoad` calls it there. A second host is refused with
/// [`EmbedError::BadState`] and the first stays.
pub fn register_host(host_impl: Box<dyn RemoteHost>) -> Result<(), EmbedError> {
    let mut slot = host();
    if slot.is_some() {
        return Err(EmbedError::BadState(
            "a remote plane host is already registered".to_string(),
        ));
    }
    *slot = Some(host_impl);
    Ok(())
}

/// No runtime in this process hosts a remote plane.
const PLANE_FREE: u8 = 0;
/// One runtime in this process hosts a remote plane.
const PLANE_HELD: u8 = 1;
/// A plane in this process did not stop cleanly and may still answer, so no
/// other plane may bind for the rest of the process.
const PLANE_UNSTOPPED: u8 = 2;

/// Whether a runtime in this process hosts a remote plane.
static REMOTE_HOSTED: AtomicU8 = AtomicU8::new(PLANE_FREE);

/// The plane that did not stop, with its stop error: the record every later
/// start and status read in this process name it from.
static UNSTOPPED: Mutex<Option<RemoteStatus>> = Mutex::new(None);

/// The plane in this process that did not stop, if one did not.
pub(crate) fn unstopped_plane() -> Option<RemoteStatus> {
    UNSTOPPED
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
}

/// The one remote plane a process hosts. Every runtime reads the same device
/// key, so two planes would answer under one endpoint id for two different
/// runs; a second runtime with the plane enabled is refused while one holds
/// this lease, and none binds after a plane failed to stop.
struct RemoteLease(());

/// What [`RemoteLease::acquire`] finds.
enum Acquired {
    Lease(RemoteLease),
    /// A plane in this process did not stop: the process record names it.
    Stuck(RemoteStatus),
}

impl RemoteLease {
    fn acquire() -> Result<Acquired, EmbedError> {
        match REMOTE_HOSTED.compare_exchange(
            PLANE_FREE,
            PLANE_HELD,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => Ok(Acquired::Lease(Self(()))),
            // The failed stop writes the record before it sets
            // PLANE_UNSTOPPED, so the lease and the record give one answer
            // in this one step.
            Err(PLANE_UNSTOPPED) => unstopped_plane().map(Acquired::Stuck).ok_or_else(|| {
                EmbedError::Remote(
                    "a remote plane in this process did not stop cleanly and may still answer \
                     under this device key; restart the app process before starting a runtime \
                     with remote.enabled"
                        .to_string(),
                )
            }),
            // A stop that failed writes the record under its mutex before
            // its Release store of PLANE_UNSTOPPED, so between the two the
            // lease still reads held. The record is never cleared, so a
            // record here means that plane did not stop. A stop that still
            // runs has no record: its plane is still up, and the start can
            // retry.
            Err(_) => unstopped_plane().map(Acquired::Stuck).ok_or_else(|| {
                EmbedError::Remote(
                    "another embedded runtime in this process hosts the remote plane; stop it \
                     before starting a runtime with remote.enabled"
                        .to_string(),
                )
            }),
        }
    }

    /// Keep the process refusing every later plane: the plane this lease
    /// guarded may still be serving.
    fn keep_unstopped(self) {
        REMOTE_HOSTED.store(PLANE_UNSTOPPED, Ordering::Release);
        std::mem::forget(self);
    }
}

impl Drop for RemoteLease {
    fn drop(&mut self) {
        REMOTE_HOSTED.store(PLANE_FREE, Ordering::Release);
    }
}

/// A bound plane and the process's remote plane lease, which is freed only
/// once the plane has stopped.
pub(crate) struct HostedPlane {
    plane: Box<dyn BoundRemote>,
    _lease: RemoteLease,
}

impl HostedPlane {
    pub(crate) fn plane(&mut self) -> &mut dyn BoundRemote {
        self.plane.as_mut()
    }

    /// Stop the plane, then free the lease. A plane that does not stop
    /// cleanly keeps the process from binding another one, and
    /// [`unstopped_plane`] names it.
    pub(crate) fn stop(self) -> Result<(), String> {
        let HostedPlane {
            plane,
            _lease: lease,
        } = self;
        let endpoint_id = plane.endpoint_id();
        let bound_sockets = plane
            .bound_sockets()
            .iter()
            .map(ToString::to_string)
            .collect();
        let result = plane.stop();
        if let Err(error) = &result {
            *UNSTOPPED.lock().unwrap_or_else(PoisonError::into_inner) = Some(RemoteStatus {
                endpoint_id,
                bound_sockets,
                stop_error: Some(error.clone()),
                off_reason: None,
            });
            lease.keep_unstopped();
        }
        result
    }
}

/// TEST SEAM: `plane` as the process's hosted plane, under its lease, with
/// no host involved.
#[cfg(test)]
pub(crate) fn hosted_for_test(plane: Box<dyn BoundRemote>) -> Result<HostedPlane, EmbedError> {
    match RemoteLease::acquire()? {
        Acquired::Lease(lease) => Ok(HostedPlane {
            plane,
            _lease: lease,
        }),
        Acquired::Stuck(_) => Err(EmbedError::Remote("a plane did not stop".to_string())),
    }
}

/// TEST SEAM: free the process's remote plane lease and forget a plane
/// that failed to stop.
#[cfg(test)]
pub(crate) fn free_lease_for_test() {
    *UNSTOPPED.lock().unwrap_or_else(PoisonError::into_inner) = None;
    REMOTE_HOSTED.store(PLANE_FREE, Ordering::Release);
}

/// TEST SEAM: write the process record of a plane that did not stop, and
/// leave the lease as it is.
#[cfg(test)]
pub(crate) fn record_unstopped_for_test(stuck: RemoteStatus) {
    *UNSTOPPED.lock().unwrap_or_else(PoisonError::into_inner) = Some(stuck);
}

/// What [`bind`] gives a start.
pub(crate) enum Bound {
    /// The plane bound under the process's lease.
    Hosted(HostedPlane),
    /// A plane in this process did not stop, so none binds: the start goes
    /// on with remote off.
    Stuck(RemoteStatus),
}

/// Take the process's remote plane lease and bind the plane through the
/// registered host. After a plane in this process did not stop, no plane
/// binds and the host is not asked: [`Bound::Stuck`] names the stuck plane.
/// Every other failure is an error with its code.
pub(crate) fn bind(start: RemoteStart) -> Result<Bound, EmbedError> {
    let lease = match RemoteLease::acquire()? {
        Acquired::Lease(lease) => lease,
        Acquired::Stuck(stuck) => return Ok(Bound::Stuck(stuck)),
    };
    let slot = host();
    let Some(host_impl) = slot.as_ref() else {
        return Err(EmbedError::Remote(
            "remote.enabled is set but this app library links no remote plane host; link \
             cerulion_android_remote and list `remote: cerulion_android_remote::Host` first in \
             cerulion_android::embed_nodes!"
                .to_string(),
        ));
    };
    let plane = host_impl.bind(start).map_err(EmbedError::from)?;
    Ok(Bound::Hosted(HostedPlane {
        plane,
        _lease: lease,
    }))
}
