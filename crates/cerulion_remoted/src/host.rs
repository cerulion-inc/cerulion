// SPDX-License-Identifier: AGPL-3.0-only
//! The remote plane as a library: one entry that any process can host.
//!
//! [`RemotePlane`] is the serving entry the `cerulion-remoted` binary calls and
//! a host process embeds. Three things are injected:
//!
//! - the **state root**: the trust store, the device side map, the receipt log,
//!   the `log-tail` root and the beacon facts live under `<root>/remoted/`, the
//!   same layout as [`RemotedConfig::from_state_root`];
//! - the **keys**: a [`KeyProvider`] supplies the device key and the trust-store
//!   MAC key. The daemon reads them from files ([`KeyFiles`]); a host can hold
//!   them anywhere;
//! - the **control connect**: the local network daemon's control socket that the
//!   `catalog` and `schema` replies read through, each read bounded by its own
//!   deadline.
//!
//! [`RemotePlane::bind`] loads the trust state and binds the ONE endpoint;
//! [`BoundPlane::serve`] runs the accept loop until shutdown. The two steps let a
//! host learn the endpoint id and bound sockets before it serves.
//!
//! ## Daemon-only verbs
//!
//! Two ops verbs act on things only the daemon has, so a host built with
//! [`RemotePlane::new`] refuses them by name with a clear `unsupported_host`
//! error, after the authorizer has classified the caller as usual:
//!
//! - `restart` restarts a graph's systemd unit;
//! - `engage-estop` engages the daemon's control lease and asks for an incident
//!   capture. A host that served it would report an e-stop that stops nothing.
//!
//! ## What a host takes on
//!
//! - **No kill switch.** The library entry does not read `--network` or
//!   `CERULION_NETWORK`; that switch belongs to the daemon's
//!   [`run`](crate::daemon::run). A host that wants the same switch resolves it
//!   with [`resolve_network_disabled_loud`](crate::resolve_network_disabled_loud)
//!   and does not call [`RemotePlane::bind`] when it is off.
//! - **Files under the state root.** Besides the trust state, the plane appends
//!   the ops receipt log, writes the public beacon facts at bind, and polls for
//!   revocation epochs that a sync worker leaves next to the trust store
//!   (`epoch.pending`), asking for one early (`epoch.request`) when a key it has
//!   not seen recently connects.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use cerud::lease::ControlLease;
use cerulion_core::TransportManager;
use cerulion_link::{build_endpoint, Endpoint, EndpointConfig, EndpointId, RelayConfig};
use cerulion_pairing::verify::TrustStore;

use crate::clock::RemotedClock;
use crate::config::RemotedConfig;
use crate::device_index::DeviceAccountIndex;
use crate::error::RemotedError;
use crate::wire::WirePlane;

/// The ops handshake name the daemon announces.
pub(crate) const DAEMON_SERVER_NAME: &str = "cerulion-remoted";
/// The ops handshake name a library host announces: it is not the daemon.
const EMBEDDED_SERVER_NAME: &str = "cerulion-remote-plane";

/// Where a remote plane's two secrets come from.
pub trait KeyProvider: Send + Sync {
    /// The 32-byte ed25519 device secret. Its public half is the endpoint id and
    /// must equal the trust store's recorded transport key.
    fn device_key(&self) -> Result<[u8; 32], RemotedError>;

    /// The key that authenticates the trust store and the device side map.
    ///
    /// `required` is `true` when a trust store or a side map already exists under
    /// the state root, so the key is needed to verify them; an empty key is then
    /// refused with [`RemotedError::MacKey`]. When it is `false` and the provider
    /// has no key, it may return an empty key: it verifies nothing, and the
    /// absent trust store reports its own provisioning gap.
    fn store_mac_key(&self, required: bool) -> Result<Vec<u8>, RemotedError>;
}

/// The daemon's key provider: the two key files under the state root, or the
/// files the `--key-file` and `--store-mac-key-file` flags name.
#[derive(Debug, Clone)]
pub struct KeyFiles {
    /// The 32-byte device secret file.
    pub key_file: PathBuf,
    /// The trust-store MAC key file.
    pub store_mac_key_file: PathBuf,
}

impl KeyProvider for KeyFiles {
    fn device_key(&self) -> Result<[u8; 32], RemotedError> {
        crate::daemon::load_device_key(&self.key_file)
    }

    fn store_mac_key(&self, required: bool) -> Result<Vec<u8>, RemotedError> {
        crate::daemon::load_mac_key(&self.store_mac_key_file, required)
    }
}

/// Which ops verb set a plane serves, with what each set needs.
pub(crate) enum OpsHost {
    /// The `cerulion-remoted` daemon: every verb. `engage-estop` engages `lease`.
    Daemon { lease: Arc<Mutex<ControlLease>> },
    /// A library host: `restart` and `engage-estop` are refused by name, so it
    /// has no lease to engage.
    Embedded,
}

/// The kind of host a plane is bound for; the daemon's lease is made at serve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HostKind {
    Daemon,
    Embedded,
}

/// The state files a plane reads and writes.
#[derive(Debug, Clone)]
pub(crate) struct PlaneFiles {
    pub(crate) store_file: PathBuf,
    pub(crate) index_file: PathBuf,
    pub(crate) receipt_file: PathBuf,
    pub(crate) log_root: PathBuf,
    pub(crate) beacon_facts_file: PathBuf,
}

impl PlaneFiles {
    pub(crate) fn of(config: &RemotedConfig) -> Self {
        PlaneFiles {
            store_file: config.store_file.clone(),
            index_file: config.index_file.clone(),
            receipt_file: config.receipt_file.clone(),
            log_root: config.log_root.clone(),
            beacon_facts_file: config.beacon_facts_file.clone(),
        }
    }
}

/// The remote plane, ready to bind: what the `cerulion-remoted` daemon serves,
/// hosted in any process. See the [module docs](self).
pub struct RemotePlane {
    files: PlaneFiles,
    relay: RelayConfig,
    robot: String,
    control_socket: PathBuf,
    transport: Option<Arc<TransportManager>>,
    host: HostKind,
}

impl RemotePlane {
    /// A library host's plane over `state_root` (files under
    /// `<state_root>/remoted/`). `robot` is the identity every wire reply names;
    /// `control_socket` is the local network daemon's control socket the
    /// `catalog` and `schema` replies read; `relay` is the endpoint's relay
    /// posture. `restart` and `engage-estop` are refused, and the kill switch is
    /// not read (see the module docs).
    pub fn new(
        state_root: impl AsRef<Path>,
        robot: impl Into<String>,
        control_socket: PathBuf,
        relay: RelayConfig,
    ) -> Self {
        let layout = RemotedConfig::from_state_root(state_root, relay, false);
        RemotePlane {
            files: PlaneFiles::of(&layout),
            relay: layout.relay,
            robot: robot.into(),
            control_socket,
            transport: None,
            host: HostKind::Embedded,
        }
    }

    /// The daemon's plane: the configured (possibly overridden) state files and
    /// relay, the robot identity from the environment, the default control
    /// socket and every ops verb.
    pub(crate) fn daemon(config: &RemotedConfig) -> Self {
        RemotePlane {
            files: PlaneFiles::of(config),
            relay: config.relay.clone(),
            robot: cerulion_core::graph::robot_identity_from_env(),
            control_socket: cerulion_netd::hygiene::default_socket_path(),
            transport: None,
            host: HostKind::Daemon,
        }
    }

    /// Tap topics through `manager` instead of the process's transport
    /// singleton, which the plane otherwise starts with the default
    /// configuration on the first wire request. A host that already runs its own
    /// manager passes it here, so the process keeps one iceoryx2 node. The
    /// control socket's daemon must serve the same shared-memory namespace, or
    /// each metadata reply refuses.
    pub fn with_transport(mut self, manager: Arc<TransportManager>) -> Self {
        self.transport = Some(manager);
        self
    }

    /// Load the trust state through `keys` and bind the endpoint.
    ///
    /// The device key is read first, then the MAC key (required when a trust
    /// store or side map exists), then the store and the side map. A store whose
    /// recorded transport key is not this device's public key is refused with
    /// [`RemotedError::ProvisioningMismatch`] before anything is served. On
    /// success the public beacon facts (`<root>/remoted/beacon_facts.json`: the
    /// endpoint id, the bound port and whether the device is claimable, for a
    /// local gateway's discovery record) are written, best effort.
    pub async fn bind(self, keys: &dyn KeyProvider) -> Result<BoundPlane, RemotedError> {
        let device_key = keys.device_key()?;
        // The MAC key authenticates both the store and the side map, so it is
        // required as soon as either exists. On a fresh install neither does, and
        // the absent store reports its own provisioning gap below.
        let mac_required = self.files.store_file.exists() || self.files.index_file.exists();
        let mac_key = keys.store_mac_key(mac_required)?;
        if mac_required && mac_key.is_empty() {
            return Err(RemotedError::MacKey(
                "the key provider returned an empty trust-store MAC key, but a trust store \
                 or device index exists and must be verified with it. An empty key is only \
                 a fresh install's placeholder; provide this device's MAC key"
                    .into(),
            ));
        }
        let trust_store =
            TrustStore::load(&self.files.store_file, &mac_key).map_err(RemotedError::Store)?;
        let device_index = DeviceAccountIndex::load(&self.files.index_file, &mac_key)?;
        tracing::info!(
            claimed = trust_store.is_claimed(),
            bound_devices = device_index.len(),
            "cerulion_remoted: state loaded"
        );

        let endpoint =
            build_endpoint(EndpointConfig::new(device_key).with_relay(self.relay.clone())).await?;
        // A transplanted, restored or misprovisioned store would otherwise serve
        // this device under another device's owner and access list.
        if let Err(e) = crate::daemon::check_store_device_binding(
            endpoint.id().as_bytes(),
            &trust_store.robot_transport_key(),
        ) {
            tracing::error!(error = %e, "cerulion_remoted: refusing to serve; the trust store is not bound to this device");
            endpoint.close().await;
            return Err(e);
        }
        // Best effort: a failed write warns and never fails the plane.
        crate::beacon_facts::write_at_startup(
            &self.files.beacon_facts_file,
            &endpoint,
            trust_store.ownership(),
        );
        Ok(BoundPlane {
            plane: self,
            endpoint,
            trust_store,
            device_index,
            mac_key,
        })
    }
}

/// A plane whose trust state is loaded and whose endpoint is bound. Dropping it
/// without [`BoundPlane::serve`] drops the endpoint; the beacon facts written at
/// bind stay on disk.
pub struct BoundPlane {
    plane: RemotePlane,
    endpoint: Endpoint,
    trust_store: TrustStore,
    device_index: DeviceAccountIndex,
    mac_key: Vec<u8>,
}

impl BoundPlane {
    /// The endpoint id peers dial and authenticate: the device key's public half.
    pub fn endpoint_id(&self) -> EndpointId {
        self.endpoint.id()
    }

    /// The local sockets the endpoint is bound to. An unspecified address
    /// (`0.0.0.0` or `[::]`) means every interface.
    pub fn bound_sockets(&self) -> Vec<SocketAddr> {
        self.endpoint.bound_sockets()
    }

    /// Serve the wire and ops planes until `shutdown` resolves or the endpoint
    /// closes, then close the endpoint, on an error too. Access checks read the
    /// wall clock.
    pub async fn serve(
        self,
        shutdown: impl std::future::Future<Output = ()>,
    ) -> Result<(), RemotedError> {
        let BoundPlane {
            plane,
            endpoint,
            trust_store,
            device_index,
            mac_key,
        } = self;
        let base = match plane.transport {
            Some(manager) => WirePlane::with_manager(plane.robot, manager),
            None => WirePlane::lazy(plane.robot),
        }
        .with_local_schema_provider(plane.control_socket);
        let (ops_host, server_name) = match plane.host {
            HostKind::Daemon => (
                OpsHost::Daemon {
                    lease: Arc::new(Mutex::new(ControlLease::with_default_window())),
                },
                DAEMON_SERVER_NAME,
            ),
            HostKind::Embedded => (OpsHost::Embedded, EMBEDDED_SERVER_NAME),
        };
        let result = crate::daemon::serve_core(
            &plane.files,
            ops_host,
            server_name,
            trust_store,
            device_index,
            mac_key,
            &endpoint,
            RemotedClock::wall(),
            None,
            base,
            shutdown,
        )
        .await;
        endpoint.close().await;
        result
    }
}

impl RemotedConfig {
    /// The `cerulion-remoted` daemon's own entry, which [`run`](crate::daemon::run)
    /// serves; a host embeds [`RemotePlane::new`] instead. The daemon's plane over this configuration: its state files and relay,
    /// the robot identity from the environment (`CERULION_ROBOT_IDENTITY`, else
    /// the hostname), the default control socket, and every ops verb including
    /// `restart` and `engage-estop`. [`run`](crate::daemon::run) serves it with
    /// [`RemotedConfig::key_files`].
    #[doc(hidden)]
    pub fn remote_plane(&self) -> RemotePlane {
        RemotePlane::daemon(self)
    }

    /// The daemon's key provider: this configuration's two key files.
    #[doc(hidden)]
    pub fn key_files(&self) -> KeyFiles {
        KeyFiles {
            key_file: self.key_file.clone(),
            store_mac_key_file: self.store_mac_key_file.clone(),
        }
    }
}
