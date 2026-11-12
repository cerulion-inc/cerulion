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
//! - the **schema source**: where the `catalog` and `schema` replies read the
//!   served topics' schemas. [`RemotePlane::new`] reads the local network
//!   daemon's control socket, each read bounded by its own deadline;
//!   [`RemotePlane::in_process`] calls a [`SchemaSource`] the host supplies, for
//!   a host that serves its own topics and runs no network daemon.
//!
//! [`RemotePlane::bind`] loads the trust state and binds the ONE endpoint;
//! [`BoundPlane::serve`] runs the accept loop until shutdown. The two steps let a
//! host learn the endpoint id and bound sockets before it serves.
//!
//! ## Daemon-only verbs
//!
//! Six ops verbs act on things only the daemon has, so a host built with
//! [`RemotePlane::new`] refuses them by name with a clear `unsupported_host`
//! error, after the authorizer has classified the caller as usual:
//!
//! - `restart` restarts a graph's systemd unit;
//! - `engage-estop` engages the daemon's control lease, sends an `estop` to the
//!   robot's `cerulion-rcpd` and asks for an incident capture. A host that
//!   served it would report an e-stop that stops nothing;
//! - `topic-publish` and `service-call` send through the process's transport
//!   singleton only while the daemon's e-stop is clear, and a host serves no
//!   e-stop that could stop them;
//! - `topic-snapshot` and `topic-echo-once` read through that singleton and the
//!   daemon's schema directory, not the host's transport and schemas. A desk
//!   reads a host's topics over the wire plane.
//!
//! The rcp plane (`cerulion/rcp/1`, [`crate::rcp`]) is daemon only as well: a
//! library host serves none, so a desk's rcp session there is answered with
//! the accept decision instead of a session.
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
//!   not seen recently connects. `CERULION_REMOTED_EPOCH_RECHECK_S` sets how long
//!   a key counts as recently seen, as on the daemon.

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
use crate::estop_lease::EstopLease;
use crate::local_schema::{LocalSchemaSource, SchemaSource};
use crate::rcp::{RcpLink, RcpdEstop};
use crate::trust::SharedTrust;
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
    /// An error is returned to the caller of [`RemotePlane::bind`] as is, so it
    /// must never carry key bytes.
    ///
    /// `required` is `true` when a trust store or a side map already exists under
    /// the state root, so the key is needed to verify them; an empty key is then
    /// refused with [`RemotedError::MacKey`]; [`KeyFiles`] refuses an empty file
    /// itself and names its path. When it is `false` and the provider
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
    /// The `cerulion-remoted` daemon: every verb. `engage-estop` engages
    /// `lease`, the act verbs check it, `restart` checks it before its stop and
    /// again before its start, and the rcp plane engages and checks a clone of
    /// it, so all of them share one latch and one engage count.
    /// With `rcpd`, each `engage-estop` also sends an `estop` to the robot's
    /// rcpd until rcpd answers ([`crate::rcp`]).
    Daemon {
        lease: EstopLease,
        rcpd: Option<Arc<RcpdEstop>>,
    },
    /// A library host: `restart`, `engage-estop` and the act verbs are refused
    /// by name, so it has no lease to engage and no rcpd to send to.
    Embedded,
}

impl OpsHost {
    /// The daemon's verb set over a fresh control lease, sending nothing to
    /// rcpd until [`OpsHost::with_rcpd_estop`] gives it a link.
    pub(crate) fn daemon() -> Self {
        OpsHost::Daemon {
            lease: EstopLease::new(Arc::new(Mutex::new(ControlLease::with_default_window()))),
            rcpd: None,
        }
    }

    /// A clone of the daemon's e-stop lease, sharing its latch and its engage
    /// count; `None` on a library host, which has none.
    pub(crate) fn estop_lease(&self) -> Option<EstopLease> {
        match self {
            OpsHost::Daemon { lease, .. } => Some(lease.clone()),
            OpsHost::Embedded => None,
        }
    }

    /// The daemon's verb set whose `engage-estop` also sends an `estop` to
    /// the rcpd of `link`, until rcpd answers, naming the engager's account
    /// from `shared`, on the current tokio runtime. A library host, or a
    /// daemon with no `link`, is returned as it is.
    ///
    /// # Panics
    ///
    /// When the daemon is given a `link` outside a tokio runtime.
    pub(crate) fn with_rcpd_estop(self, link: Option<&RcpLink>, shared: &SharedTrust) -> Self {
        match (self, link) {
            (OpsHost::Daemon { lease, .. }, Some(link)) => OpsHost::Daemon {
                lease,
                rcpd: Some(Arc::new(RcpdEstop::new(
                    link,
                    shared.clone(),
                    tokio::runtime::Handle::current(),
                ))),
            },
            (host, _) => host,
        }
    }

    /// The daemon's rcpd e-stop sender, which [`RcpdEstop::shut_down`] ends
    /// when the daemon stops; `None` on a library host and on a daemon with
    /// no rcp link.
    pub(crate) fn rcpd_estop(&self) -> Option<Arc<RcpdEstop>> {
        match self {
            OpsHost::Daemon { rcpd, .. } => rcpd.clone(),
            OpsHost::Embedded => None,
        }
    }
}

/// The kind of host a plane is bound for; the daemon's lease is made at serve.
#[derive(Debug, Clone)]
enum HostKind {
    /// The daemon, which also serves the rcp plane to `rcp`'s rcpd.
    Daemon {
        rcp: RcpLink,
    },
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
    schemas: LocalSchemaSource,
    transport: Option<Arc<TransportManager>>,
    host: HostKind,
}

impl RemotePlane {
    /// A library host's plane over `state_root` (files under
    /// `<state_root>/remoted/`). `robot` is the identity every wire reply names;
    /// `control_socket` is the local network daemon's control socket the
    /// `catalog` and `schema` replies read; `relay` is the endpoint's relay
    /// posture. `restart`, `engage-estop` and the act verbs are refused, and the
    /// kill switch is not read (see the module docs).
    pub fn new(
        state_root: impl AsRef<Path>,
        robot: impl Into<String>,
        control_socket: impl Into<PathBuf>,
        relay: RelayConfig,
    ) -> Self {
        Self::embedded(
            state_root,
            robot.into(),
            relay,
            LocalSchemaSource::Socket(control_socket.into()),
        )
    }

    /// A library host's plane that reads the served topics' schemas from
    /// `schemas` in this process instead of a network daemon's control socket.
    /// Everything else is as [`RemotePlane::new`]: files under
    /// `<state_root>/remoted/`, `restart`, `engage-estop` and the act verbs
    /// refused, no kill switch read. The `catalog` still lists the live topics of the plane's
    /// transport; `schemas` names their schemas and serves the `schema` verb.
    pub fn in_process(
        state_root: impl AsRef<Path>,
        robot: impl Into<String>,
        relay: RelayConfig,
        schemas: SchemaSource,
    ) -> Self {
        Self::embedded(
            state_root,
            robot.into(),
            relay,
            LocalSchemaSource::InProcess(schemas),
        )
    }

    fn embedded(
        state_root: impl AsRef<Path>,
        robot: String,
        relay: RelayConfig,
        schemas: LocalSchemaSource,
    ) -> Self {
        let layout = RemotedConfig::from_state_root(state_root, relay, false);
        RemotePlane {
            files: PlaneFiles::of(&layout),
            relay: layout.relay,
            robot,
            schemas,
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
            schemas: LocalSchemaSource::Socket(cerulion_netd::hygiene::default_socket_path()),
            transport: None,
            host: HostKind::Daemon {
                rcp: config.rcp.clone(),
            },
        }
    }

    /// Tap topics through `manager` instead of the process's transport
    /// singleton, which the plane otherwise starts with the default
    /// configuration on the first wire request. A host that already runs its own
    /// manager passes it here, so the process keeps one iceoryx2 node. A control
    /// socket's daemon must serve the same shared-memory namespace, or each
    /// metadata reply refuses.
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
        // An empty key authenticates nothing, and the live trust state cannot
        // persist a claim or pair with it, so existing trust state is never
        // served under one.
        if mac_required && mac_key.is_empty() {
            return Err(RemotedError::MacKey(
                "the key provider returned an empty trust-store MAC key (this provider has \
                 no key file to name), but a trust store or device index exists and must \
                 be verified with it. An empty key is only a fresh install's placeholder; \
                 provide this device's MAC key"
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
        if let HostKind::Daemon { rcp } = &self.host {
            crate::rcp::prepare(rcp).map_err(RemotedError::Config)?;
        }

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
            device_key,
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
    device_key: [u8; 32],
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

    /// Serve the wire and ops planes (and the rcp plane, on the daemon) until
    /// `shutdown` resolves or the endpoint closes, then close the endpoint, on
    /// an error too. Access checks read the wall clock. `shutdown` is dropped
    /// as soon as the accept loop stops, before the shutdown stage runs.
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
            device_key,
        } = self;
        // Principle #8: serving and outbound mirrors use the same iceoryx2 node
        // and the same device endpoint.
        let outbound = if matches!(&plane.host, HostKind::Daemon { .. }) {
            let manager = match &plane.transport {
                Some(manager) => manager.clone(),
                None => TransportManager::init(cerulion_core::TransportConfig::default())
                    .map_err(|error| RemotedError::Config(error.to_string()))?,
            };
            let socket = plane
                .files
                .store_file
                .parent()
                .ok_or_else(|| {
                    RemotedError::Config("remote state directory is unavailable".into())
                })?
                .join("outbound.sock");
            Some(
                cerulion_netd::shared_endpoint::OutboundServer::start_with_holds(
                    socket,
                    endpoint.clone(),
                    tokio::runtime::Handle::current(),
                    device_key,
                    manager,
                    cerulion_netd::serving_hold::ServingHolds::open(
                        &cerulion_netd::serving_hold::fence_path(
                            plane
                                .files
                                .store_file
                                .parent()
                                .and_then(Path::parent)
                                .ok_or_else(|| {
                                    RemotedError::Config("remote state root is unavailable".into())
                                })?,
                        ),
                    )
                    .map_err(|error| RemotedError::Config(error.to_string()))?,
                )
                .map_err(RemotedError::Config)?,
            )
        } else {
            None
        };
        let base = match plane.transport {
            Some(manager) => WirePlane::with_manager(plane.robot, manager),
            None => WirePlane::lazy(plane.robot),
        };
        let base = match plane.schemas {
            LocalSchemaSource::Socket(socket) => base.with_local_schema_provider(socket),
            LocalSchemaSource::InProcess(source) => base.with_in_process_schema_provider(source),
        };
        let (ops_host, server_name, rcp) = match plane.host {
            HostKind::Daemon { rcp } => (OpsHost::daemon(), DAEMON_SERVER_NAME, Some(rcp)),
            HostKind::Embedded => {
                tracing::info!(
                    "cerulion_remoted: serving as a library host; restart, engage-estop \
                     and the act verbs are refused and no rcp plane is served (daemon only)"
                );
                (OpsHost::Embedded, EMBEDDED_SERVER_NAME, None)
            }
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
            rcp,
            shutdown,
        )
        .await;
        // Local readers settle on this runtime; keep its drivers running while
        // the blocking control threads join, including current-thread hosts.
        tokio::task::spawn_blocking(move || drop(outbound))
            .await
            .map_err(|_| RemotedError::Config("outbound control shutdown failed".into()))?;
        endpoint.close().await;
        result
    }
}

impl RemotedConfig {
    /// The `cerulion-remoted` daemon's plane over this configuration.
    ///
    /// It uses the configured state files and relay, the robot identity from the
    /// environment (`CERULION_ROBOT_IDENTITY`, else the hostname), the default
    /// control socket, and every ops verb including `restart`, `engage-estop`
    /// and the act verbs.
    /// [`run`](crate::daemon::run) serves it with [`RemotedConfig::key_files`].
    /// A host embeds [`RemotePlane::new`] instead.
    #[doc(hidden)]
    pub fn daemon_plane(&self) -> RemotePlane {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn the_daemon_lends_its_estop_lease_and_a_library_host_has_none() {
        assert!(OpsHost::Embedded.estop_lease().is_none());
        let host = OpsHost::daemon();
        let lent = host.estop_lease().expect("the daemon has an e-stop lease");
        let OpsHost::Daemon { lease, .. } = &host else {
            unreachable!("OpsHost::daemon is the daemon")
        };
        // An engage through the lent clone, as the rcp plane makes, waits for
        // the lock this test holds; the daemon's own lease, which the act
        // verbs read, must count it.
        let held = lease.lock().unwrap();
        let engaging = std::thread::spawn(move || {
            lent.engage("iroh:ab");
        });
        let until = Instant::now() + Duration::from_secs(10);
        while !lease.engage_pending() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(1));
        }
        let counted = lease.engage_pending();
        drop(held);
        engaging.join().unwrap();
        assert!(
            counted,
            "an engage through the lent lease is not counted on the daemon's"
        );
        assert!(!matches!(
            lease.lock().unwrap().estop(),
            cerud::lease::EstopState::Clear
        ));
    }
}
