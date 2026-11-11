// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded metadata discovery through the controller's one outgoing endpoint.

use super::*;
use crate::account_access::{
    PairedRobot, RobotCatalog, RobotPath, RobotPresence, RobotTrustSource,
};

const DISCOVERY_BATCH_WIDTH: usize = 16;
const _: () = assert!(DISCOVERY_BATCH_WIDTH <= crate::shared_endpoint::MAX_CLIENTS);

impl AccountWanController {
    pub(super) fn install_paired(
        &self,
        pins: &[PairedRobot],
        demands: &mut DemandRegistry,
    ) -> Result<(), String> {
        let mut state = self.state()?;
        self.retire_pending(&mut state, demands)?;
        self.sync_identity(&mut state)?;
        if !pins.is_empty() {
            if let Some(home) = &self.config.config_home {
                let seed =
                    cerulion_wireclient::config::resolve_desk_seed(Some(&home.join("desk.key")))
                        .map_err(|error| format!("paired desk identity unavailable: {error}"))?;
                if state.registry.desk_seed() != seed {
                    if state.account_mode {
                        return Err(NEED_INSTALL.into());
                    }
                    let relay = state.registry.relay().clone();
                    Self::invalidate(&mut state)?;
                    self.retire_pending(&mut state, demands)?;
                    state.registry = Arc::new(
                        WanRegistry::new(self.manual.clone(), seed, relay).with_desk_paths(
                            crate::wan::DeskPathInputs {
                                key_file: Some(home.join("desk.key")),
                                device_cert_env: None,
                                epoch_dir_env: None,
                            },
                        ),
                    );
                }
            }
        }
        let mut paired = HashMap::new();
        for pin in pins {
            let eid = EndpointId::from_bytes(&pin.endpoint_key)
                .map_err(|_| "paired robot has an invalid endpoint key")?;
            if self
                .manual
                .get(&pin.name)
                .is_some_and(|robot| robot.eid != eid)
            {
                return Err("paired robot conflicts with a configured WAN endpoint".into());
            }
            let direct_addrs = self
                .manual
                .get(&pin.name)
                .map(|robot| robot.direct_addrs.clone())
                .unwrap_or_default();
            paired.insert(pin.name.clone(), WanRobot { eid, direct_addrs });
        }
        let mut old_paired = self
            .paired
            .lock()
            .map_err(|_| "paired robot state is poisoned")?;
        let old = state.registry.membership_snapshot()?;
        let mut membership = old.clone();
        for name in old_paired.keys() {
            if !self.manual.contains_key(name) {
                membership.remove(name);
            }
        }
        membership.extend(paired.clone());
        let changed: BTreeSet<_> = old
            .iter()
            .filter(|(name, peer)| {
                membership
                    .get(*name)
                    .is_none_or(|next| next.eid != peer.eid)
            })
            .map(|(name, _)| name.clone())
            .collect();
        let keys: BTreeSet<_> = self
            .pins()?
            .iter()
            .filter(|(key, route)| route.route == Route::Wan && changed.contains(&key.robot))
            .map(|(key, _)| key.clone())
            .collect();
        if keys.iter().any(|key| demands.is_tearing(key)) {
            return Err("paired robot mirrors are still being retired; retry the operation".into());
        }
        if let Some(plane) = state.plane.as_ref() {
            plane.update_membership(membership)?;
        } else {
            state.registry.replace_membership(membership)?;
        }
        demands
            .invalidate_mirrors(&keys)
            .map_err(|_| "paired robot mirrors could not be retired")?;
        let mut routes = self.pins()?;
        for key in keys {
            routes.remove(&key);
        }
        *old_paired = paired;
        Ok(())
    }

    pub(super) fn paired_metadata(
        &self,
        robot: &str,
        requested: Option<&str>,
    ) -> Option<Result<(CatalogReply, Option<SchemaReply>), String>> {
        let paired = match self.paired.lock() {
            Ok(paired) => paired.contains_key(robot),
            Err(_) => return Some(Err("paired robot state is poisoned".into())),
        };
        if !paired && !self.manual.contains_key(robot) {
            return None;
        }
        Some((|| {
            self.network_allowed()?;
            let deadline = Instant::now() + QUERY_BUDGET;
            let mut state = self.state()?;
            self.sync_identity(&mut state)?;
            let stamp = state
                .identity
                .as_ref()
                .map(|identity| identity.stamp().clone());
            let plane = self.ensure_plane(&mut state)?;
            let mut catalog = plane.query_catalog(robot, remaining(deadline)?)?;
            catalog.robot = robot.into();
            let schema = if let Some(requested) = requested {
                match account_access::schema_topic_for_type(&catalog, requested)? {
                    Some(topic) => {
                        let mut schema = plane.query_schema(robot, &topic, remaining(deadline)?)?;
                        if schema
                            .docs
                            .first()
                            .is_some_and(|root| root.qualified != requested)
                        {
                            return Err(
                                "paired robot schema root does not match the requested type".into(),
                            );
                        }
                        schema.requested = requested.into();
                        Some(schema)
                    }
                    None => Some(SchemaReply::not_found(
                        robot,
                        requested,
                        "the robot catalog has no matching schema",
                    )),
                }
            } else {
                None
            };
            if let Some(stamp) = stamp {
                self.fence_reply(&mut state, &stamp)?;
            }
            Ok((catalog, schema))
        })())
    }

    fn cached_direct(&self) -> std::collections::BTreeMap<String, SocketAddr> {
        let Some(home) = &self.config.config_home else {
            return Default::default();
        };
        use std::io::Read;
        use std::os::unix::fs::OpenOptionsExt;
        let mut options = std::fs::OpenOptions::new();
        options.read(true).custom_flags(libc::O_NONBLOCK);
        let Ok(file) = options.open(home.join("robot-direct.json")) else {
            return Default::default();
        };
        if !file
            .metadata()
            .is_ok_and(|metadata| metadata.is_file() && metadata.len() <= 1024 * 1024)
        {
            return Default::default();
        }
        let mut bytes = Vec::new();
        if file.take(1024 * 1024 + 1).read_to_end(&mut bytes).is_err() {
            return Default::default();
        }
        if bytes.len() > 1024 * 1024 {
            return Default::default();
        }
        serde_json::from_slice::<std::collections::BTreeMap<String, SocketAddr>>(&bytes)
            .ok()
            .filter(|cache| cache.len() <= account_access::MAX_ROBOTS * 2)
            .unwrap_or_default()
    }

    pub(super) fn discover(&self, budget: Duration) -> Result<Vec<RobotCatalog>, String> {
        self.network_allowed()?;
        let deadline = Instant::now() + budget;
        let mut state = self.state()?;
        self.sync_identity(&mut state)?;
        let stamp = state
            .identity
            .as_ref()
            .map(|identity| identity.stamp().clone());
        let mut rows = Vec::new();
        let mut owned_keys = BTreeSet::new();
        if let Some(snapshot) = &state.snapshot {
            for robot in &snapshot.robots {
                if let Some(key) = robot.endpoint_key {
                    owned_keys.insert(key);
                }
                rows.push(RobotCatalog {
                    route: account_access::robot_route(&robot.robot_id),
                    name: robot.hostname.clone(),
                    endpoint_key: robot.endpoint_key,
                    trust: RobotTrustSource::Account,
                    presence: RobotPresence::Unknown {
                        reason: "not checked within the listing budget".into(),
                    },
                    path: RobotPath::Unknown,
                    catalog: None,
                });
            }
        }
        let paired = self
            .paired
            .lock()
            .map_err(|_| "paired robot state is poisoned")?
            .clone();
        for (name, robot) in paired {
            let key = *robot.eid.as_bytes();
            if owned_keys.contains(&key) {
                continue;
            }
            rows.push(RobotCatalog {
                route: name.clone(),
                name,
                endpoint_key: Some(key),
                trust: RobotTrustSource::Paired,
                presence: RobotPresence::Unknown {
                    reason: "not checked within the listing budget".into(),
                },
                path: RobotPath::Unknown,
                catalog: None,
            });
        }
        rows.sort_by(|a, b| (&a.name, &a.route).cmp(&(&b.name, &b.route)));
        if rows.is_empty() {
            return Ok(rows);
        }
        let mut membership = state.registry.membership_snapshot()?;
        let mut cached_direct = self.cached_direct();
        let browse_budget = deadline
            .saturating_duration_since(Instant::now())
            .min(Duration::from_millis(250));
        let advertised = cerulion_mdns::browse_iroh_peers(browse_budget);
        for robot in membership.values_mut() {
            let key = robot.eid.to_string();
            if let Some(address) = cached_direct.get(&key) {
                robot.direct_addrs.push(*address);
            }
            for (_, address) in advertised.iter().filter(|(eid, _)| eid == &key).take(16) {
                if !robot.direct_addrs.contains(address) {
                    robot.direct_addrs.push(*address);
                }
            }
        }
        // Address candidates do not change endpoint identity or retire live readers.
        state.registry.replace_membership(membership)?;
        let plane = self.ensure_plane(&mut state)?;
        for batch in rows.chunks_mut(DISCOVERY_BATCH_WIDTH) {
            if Instant::now() >= deadline {
                break;
            }
            let observations = std::thread::scope(|scope| {
                let workers: Vec<_> = batch
                    .iter_mut()
                    .map(|row| {
                        scope.spawn(move || {
                            if row.endpoint_key.is_none() {
                                row.presence = RobotPresence::Unknown {
                                    reason: "registration has no endpoint key".into(),
                                };
                                return None;
                            }
                            let remaining = deadline.saturating_duration_since(Instant::now());
                            if remaining.as_millis() == 0 {
                                return None;
                            }
                            match plane.query_catalog_with_path(&row.route, remaining) {
                                Ok((mut catalog, path, direct)) => {
                                    catalog.robot = row.route.clone();
                                    row.presence = RobotPresence::Online;
                                    row.path = path;
                                    row.catalog = Some(catalog);
                                    row.endpoint_key.zip(direct)
                                }
                                Err(reason) => {
                                    let remaining = deadline
                                        .saturating_duration_since(Instant::now())
                                        .min(Duration::from_millis(100));
                                    row.presence = match plane.probe_robot(&row.route, remaining) {
                                        Ok(crate::iroh_plane::RobotProbeOutcome::Online) => {
                                            row.catalog =
                                                Some(CatalogReply::refused(&row.route, reason));
                                            RobotPresence::Online
                                        }
                                        _ => RobotPresence::NotReached { reason },
                                    };
                                    None
                                }
                            }
                        })
                    })
                    .collect();
                workers
                    .into_iter()
                    .filter_map(|worker| worker.join().ok().flatten())
                    .collect::<Vec<_>>()
            });
            for (key, address) in observations {
                if let Ok(eid) = EndpointId::from_bytes(&key) {
                    cached_direct.insert(eid.to_string(), address);
                }
            }
        }
        if let Some(stamp) = stamp {
            self.fence_reply(&mut state, &stamp)?;
        }
        if let Some(home) = &self.config.config_home {
            if let Ok(bytes) = serde_json::to_vec(&cached_direct) {
                let temporary = home.join(format!("robot-direct.{}.tmp", std::process::id()));
                use std::io::Write;
                use std::os::unix::fs::OpenOptionsExt;
                let saved = std::fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .mode(0o600)
                    .open(&temporary)
                    .and_then(|mut file| {
                        let result = file
                            .write_all(&bytes)
                            .and_then(|()| file.sync_all())
                            .and_then(|()| {
                                std::fs::rename(&temporary, home.join("robot-direct.json"))
                            });
                        if result.is_err() {
                            let _ = std::fs::remove_file(&temporary);
                        }
                        result
                    });
                if saved.is_err() {
                    tracing::warn!("direct robot address cache could not be saved");
                }
            }
        }
        Ok(rows)
    }
}
