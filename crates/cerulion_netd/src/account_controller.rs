// SPDX-License-Identifier: AGPL-3.0-only
//! One account-bound WAN plane alongside the existing LAN mirror manager.
//!
//! Membership is installed locally; listing never opens an endpoint. Every WAN
//! operation checks the current login transaction, and replies are fenced again
//! after I/O. Route pins live separately from the query mutex so a slow robot
//! cannot block LAN routing. Pins survive until daemon registry retirement.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard, TryLockError};
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{CatalogReply, SchemaReply};
use cerulion_core::transport::demand_authorizer::DemandAuthorizer;
use cerulion_core::transport::TransportManager;
use cerulion_link::EndpointId;

use crate::account_access::{self, AccountSnapshot};
use crate::identity_snapshot::{self, IdentitySnapshot, SnapshotError};
use crate::iroh_plane::IrohMirrorPlane;
use crate::mirror::{GatewayMirrorPlane, MirrorPlane};
use crate::registry::{DemandRegistry, TopicKey};
use crate::wan::{WanRegistry, WanRobot};

mod operations;

const BUSY: &str = "account robot controller is busy; retry the operation";
const IDENTITY_BUSY: &str = "login identity is being updated; retry the operation";
const NEED_INSTALL: &str = "account robot identity changed; refresh the account robot list";
const NETWORK_OFF: &str = "WAN access is disabled by CERULION_NETD_NETWORK=off";
const QUERY_BUDGET: Duration = Duration::from_millis(account_access::MAX_PROBE_BUDGET_MS);
const BUSY_GRACE: Duration = Duration::from_secs(5);

/// Resolved production posture, with explicit paths and optional trusted direct
/// evidence for controlled deployments. No hostname supplies identity evidence.
pub struct AccountControllerConfig {
    /// Existing login configuration home; absence refuses account installation.
    pub config_home: Option<PathBuf>,
    /// Resolved network configuration; false prevents every WAN network operation.
    pub network_enabled: bool,
    /// A machine serving LISTEN owns its endpoint in remoted, not another client.
    pub serving_gateway: bool,
    /// Shared revocation cache directory for the current login identity.
    pub epoch_dir: Option<PathBuf>,
    /// Addresses are valid only for this exact robot id and endpoint key pair.
    pub trusted_direct: HashMap<([u8; 32], [u8; 32]), Vec<SocketAddr>>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Route {
    Lan,
    Wan,
}

struct State {
    registry: Arc<WanRegistry>,
    plane: Option<IrohMirrorPlane>,
    account_mode: bool,
    identity: Option<IdentitySnapshot>,
    snapshot: Option<AccountSnapshot>,
    catalogs: HashMap<[u8; 32], CatalogReply>,
    pending_all: bool,
    pending_robots: BTreeSet<String>,
    busy_since: Option<Instant>,
}

/// Owns at most one outgoing endpoint. This type never exposes its device seed.
pub struct AccountWanController {
    lan: GatewayMirrorPlane,
    manager: Arc<TransportManager>,
    authorizer: Arc<dyn DemandAuthorizer>,
    config: AccountControllerConfig,
    manual: HashMap<String, WanRobot>,
    pins: Mutex<BTreeMap<TopicKey, Route>>,
    state: Mutex<State>,
}

impl AccountWanController {
    /// Construct without an Iroh runtime, endpoint, login read or network request.
    pub fn new(
        manager: Arc<TransportManager>,
        manual: WanRegistry,
        authorizer: Arc<dyn DemandAuthorizer>,
        config: AccountControllerConfig,
    ) -> Result<Self, String> {
        let robots = manual.membership_snapshot()?;
        if robots
            .keys()
            .any(|name| name.is_empty() || name.trim() != name)
        {
            return Err("manual WAN robot names must be nonempty canonical identifiers".into());
        }
        if robots.keys().any(|name| {
            name.trim()
                .starts_with(account_access::ACCOUNT_ROUTE_PREFIX)
        }) {
            return Err("manual WAN robot names cannot use the reserved account: prefix".into());
        }
        Ok(Self {
            lan: GatewayMirrorPlane::new(Arc::clone(&manager)),
            manager,
            authorizer,
            config,
            manual: robots,
            pins: Mutex::new(BTreeMap::new()),
            state: Mutex::new(State {
                registry: Arc::new(manual),
                plane: None,
                account_mode: false,
                identity: None,
                snapshot: None,
                catalogs: HashMap::new(),
                pending_all: false,
                pending_robots: BTreeSet::new(),
                busy_since: None,
            }),
        })
    }

    fn state(&self) -> Result<MutexGuard<'_, State>, String> {
        self.state.try_lock().map_err(|error| match error {
            TryLockError::WouldBlock => BUSY.into(),
            TryLockError::Poisoned(_) => "account robot controller state is poisoned".into(),
        })
    }

    fn pins(&self) -> Result<MutexGuard<'_, BTreeMap<TopicKey, Route>>, String> {
        self.pins
            .lock()
            .map_err(|_| "account robot route state is poisoned".into())
    }

    fn route(&self, key: &TopicKey) -> Result<Route, String> {
        // Parse before looking up a pin: malformed reserved identities never fall
        // through to LAN, including before an account snapshot has been installed.
        let account = account_access::parse_robot_route(&key.robot)?.is_some();
        if let Some(route) = self.pins()?.get(key) {
            return Ok(*route);
        }
        Ok(if account || self.manual.contains_key(&key.robot) {
            Route::Wan
        } else {
            Route::Lan
        })
    }

    fn network_allowed(&self) -> Result<(), String> {
        if !self.config.network_enabled {
            return Err(NETWORK_OFF.into());
        }
        if self.config.serving_gateway {
            return Err(crate::wan::SERVING_MACHINE_WAN_REFUSAL.into());
        }
        Ok(())
    }

    fn read_identity(&self) -> Result<IdentitySnapshot, SnapshotError> {
        let home = self
            .config
            .config_home
            .as_deref()
            .ok_or(SnapshotError::NoConfigHome)?;
        identity_snapshot::load_at(home)
    }

    fn ensure_plane<'a>(&self, state: &'a mut State) -> Result<&'a IrohMirrorPlane, String> {
        self.network_allowed()?;
        if state.pending_all || !state.pending_robots.is_empty() {
            return Err("previous WAN mirrors are awaiting registry retirement".into());
        }
        if state.account_mode && state.identity.is_none() {
            return Err(NEED_INSTALL.into());
        }
        if state.plane.is_none() {
            state.plane = Some(
                IrohMirrorPlane::new(Arc::clone(&self.manager), Arc::clone(&state.registry))?
                    .with_authorizer(Arc::clone(&self.authorizer)),
            );
        }
        state
            .plane
            .as_ref()
            .ok_or_else(|| "WAN plane was not constructed".into())
    }

    fn invalidate(state: &mut State) -> Result<(), String> {
        state.pending_all = true;
        state.pending_robots.clear();
        state.identity = None;
        state.snapshot = None;
        state.catalogs.clear();
        match state.plane.take() {
            Some(plane) => plane.invalidate_account(),
            None => state.registry.replace_owner_certificate(None),
        }
    }

    fn pending_keys(&self, state: &State) -> Result<BTreeSet<TopicKey>, String> {
        Ok(self
            .pins()?
            .iter()
            .filter(|(key, route)| {
                **route == Route::Wan
                    && (state.pending_all || state.pending_robots.contains(&key.robot))
            })
            .map(|(key, _)| key.clone())
            .collect())
    }

    fn retire_pending(
        &self,
        state: &mut State,
        registry: &mut DemandRegistry,
    ) -> Result<(), String> {
        let keys = self.pending_keys(state)?;
        registry.invalidate_mirrors(&keys).map_err(|_| {
            "previous WAN mirrors are still being retired; retry the operation".to_owned()
        })?;
        let mut pins = self.pins()?;
        for key in keys {
            pins.remove(&key);
        }
        state.pending_all = false;
        state.pending_robots.clear();
        Ok(())
    }

    fn sync_identity(&self, state: &mut State) -> Result<(), String> {
        if !state.account_mode {
            return Ok(());
        }
        let current = match self.read_identity() {
            Ok(current) => current,
            Err(SnapshotError::Busy) => return Err(IDENTITY_BUSY.into()),
            Err(_) => {
                Self::invalidate(state)?;
                return Err(NEED_INSTALL.into());
            }
        };
        let Some(previous) = state.identity.as_ref() else {
            return Err(NEED_INSTALL.into());
        };
        if !same_identity(previous, &current) {
            Self::invalidate(state)?;
            return Err(NEED_INSTALL.into());
        }
        if previous.stamp() != current.stamp() {
            state
                .registry
                .replace_owner_certificate(current.owner_chain().cloned().map(Arc::new))?;
            state.identity = Some(current);
            state.catalogs.clear();
        }
        Ok(())
    }

    fn fence_reply(
        &self,
        state: &mut State,
        before: &crate::identity_snapshot::IdentityStamp,
    ) -> Result<(), String> {
        self.sync_identity(state)?;
        if state
            .identity
            .as_ref()
            .is_none_or(|identity| identity.stamp() != before)
        {
            return Err("login proof changed during the operation; retry the operation".into());
        }
        Ok(())
    }

    fn membership(&self, snapshot: &AccountSnapshot) -> Result<HashMap<String, WanRobot>, String> {
        let mut robots = self.manual.clone();
        for robot in &snapshot.robots {
            if let Some(key) = robot.endpoint_key {
                let eid = EndpointId::from_bytes(&key)
                    .map_err(|_| "account robot has an invalid endpoint key".to_owned())?;
                let direct_addrs = self
                    .config
                    .trusted_direct
                    .get(&(robot.robot_id, key))
                    .cloned()
                    .unwrap_or_default();
                robots.insert(
                    account_access::robot_route(&robot.robot_id),
                    WanRobot { eid, direct_addrs },
                );
            }
        }
        Ok(robots)
    }

    fn install(
        &self,
        snapshot: &AccountSnapshot,
        registry: &mut DemandRegistry,
    ) -> Result<(), String> {
        snapshot.validate()?;
        let identity = self.read_identity().map_err(|error| {
            if error == SnapshotError::Busy {
                IDENTITY_BUSY
            } else {
                NEED_INSTALL
            }
            .to_owned()
        })?;
        if snapshot.auth_account_id != identity.auth_account_id()
            || snapshot.pairing_account_id != identity.account().0
            || snapshot.device_key != identity.device_key().0
            || snapshot.owner_chain.as_deref() != identity.owner_chain_wire()
        {
            return Err("account robot snapshot does not match the current local login".into());
        }
        let robots = self.membership(snapshot)?;
        let mut state = self.state()?;
        self.retire_pending(&mut state, registry)?;
        let unchanged_identity = state
            .identity
            .as_ref()
            .is_some_and(|old| same_identity(old, &identity));
        if !unchanged_identity {
            state.account_mode = true;
            Self::invalidate(&mut state)?;
            self.retire_pending(&mut state, registry)?;
            state.registry = Arc::new(identity.wan_registry(
                robots,
                state.registry.relay().clone(),
                self.config.epoch_dir.clone(),
            )?);
        } else {
            let old = state.registry.membership_snapshot()?;
            let changed: BTreeSet<_> = old
                .iter()
                .filter(|(name, peer)| robots.get(*name) != Some(*peer))
                .map(|(name, _)| name.clone())
                .collect();
            let keys: BTreeSet<_> = self
                .pins()?
                .iter()
                .filter(|(key, route)| **route == Route::Wan && changed.contains(&key.robot))
                .map(|(key, _)| key.clone())
                .collect();
            if keys.iter().any(|key| registry.is_tearing(key)) {
                return Err(
                    "affected WAN mirrors are still being retired; retry the account refresh"
                        .into(),
                );
            }
            let updated = if let Some(plane) = state.plane.as_ref() {
                plane.update_membership(robots).map(|_| ())
            } else {
                state.registry.replace_membership(robots)
            };
            if let Err(error) = updated {
                let _cleanup = Self::invalidate(&mut state);
                let _retirement = self.retire_pending(&mut state, registry);
                return Err(error);
            }
            registry
                .invalidate_mirrors(&keys)
                .map_err(|_| "WAN retirement changed during account refresh".to_owned())?;
            {
                let mut pins = self.pins()?;
                for key in keys {
                    pins.remove(&key);
                }
            }
            state
                .registry
                .replace_owner_certificate(identity.owner_chain().cloned().map(Arc::new))?;
        }
        state.account_mode = true;
        state.identity = Some(identity);
        state.snapshot = Some(snapshot.clone());
        state.catalogs.clear();
        state.busy_since = None;
        // The local transaction may have changed while old mirrors were closed.
        // Refuse publication and close this plane if that happened.
        let stamp = state.identity.as_ref().ok_or(NEED_INSTALL)?.stamp().clone();
        self.fence_reply(&mut state, &stamp)
    }

    fn refresh(&self, registry: &mut DemandRegistry) -> Result<(), String> {
        let mut state = self.state()?;
        self.retire_pending(&mut state, registry)?;
        if !state.account_mode {
            return Ok(());
        }
        if state.identity.is_none() {
            return Err(NEED_INSTALL.into());
        }
        match self.read_identity() {
            Err(SnapshotError::Busy) => {
                let now = Instant::now();
                let since = *state.busy_since.get_or_insert(now);
                if busy_grace_elapsed(since, now) {
                    Self::invalidate(&mut state)?;
                    self.retire_pending(&mut state, registry)?;
                }
                Err(IDENTITY_BUSY.into())
            }
            _ => {
                state.busy_since = None;
                let result = self.sync_identity(&mut state);
                self.retire_pending(&mut state, registry)?;
                result
            }
        }
    }
}

fn same_identity(left: &IdentitySnapshot, right: &IdentitySnapshot) -> bool {
    left.account() == right.account() && left.device_key() == right.device_key()
}

fn busy_grace_elapsed(since: Instant, now: Instant) -> bool {
    now.saturating_duration_since(since) >= BUSY_GRACE
}

fn remaining(deadline: Instant) -> Result<Duration, String> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|value| !value.is_zero())
        .ok_or_else(|| "account robot operation exceeded its time budget".into())
}

#[cfg(test)]
mod tests;
