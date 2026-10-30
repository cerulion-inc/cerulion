// SPDX-License-Identifier: AGPL-3.0-only
//! The paired remote plane, hosted in the app process.
//!
//! A start with `"remote": {"enabled": true}` hosts the plane a paired desk
//! reads the app's catalog and topics through. This crate only drives it:
//! the plane itself is served by a [`RemoteHost`] the app library links and
//! registers, `cerulion_android_remote::Host` for the iroh plane. An app
//! that hosts no plane links no host and none of its dependencies.
//!
//! A start asks the host to [`bind`](RemoteHost::bind) before any other
//! component is built: the host loads its keys and trust state and binds its
//! endpoint, and serves nothing yet. A fresh install has no keys, so the start
//! fails there with [`codes::REMOTE_NOT_PROVISIONED`](crate::codes) and
//! nothing runs. The commit then asks the bound plane to
//! [`serve`](BoundRemote::serve), as it opens LAN access, and a stop stops
//! the plane before any other component.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};

use cerulion_core::{SchemaServing, TransportManager};

use crate::config::RemoteRelay;
use crate::error::EmbedError;

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

/// Bind the plane through the registered host.
pub(crate) fn bind(start: RemoteStart) -> Result<Box<dyn BoundRemote>, EmbedError> {
    let slot = host();
    let Some(host_impl) = slot.as_ref() else {
        return Err(EmbedError::Remote(
            "remote.enabled is set but this app library links no remote plane host; link \
             cerulion_android_remote and list `remote: cerulion_android_remote::Host` first in \
             cerulion_android::embed_nodes!"
                .to_string(),
        ));
    };
    host_impl.bind(start).map_err(EmbedError::from)
}
