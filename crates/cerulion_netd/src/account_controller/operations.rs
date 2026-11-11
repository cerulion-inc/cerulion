// SPDX-License-Identifier: AGPL-3.0-only
//! Mirror and metadata operations; query paths never acquire the route mutex.

use super::*;
use crate::account_access::{AccountAccessReply, AccountAccessRequest, RobotPresence};
use crate::iroh_plane::RobotProbeOutcome;
use crate::mirror::{MirrorError, MirrorRelease};

impl AccountWanController {
    fn account_robot<'a>(
        state: &'a State,
        robot_id: &[u8; 32],
    ) -> Result<&'a crate::account_access::AccountRobot, String> {
        state
            .snapshot
            .as_ref()
            .ok_or(NEED_INSTALL)?
            .robots
            .iter()
            .find(|robot| &robot.robot_id == robot_id)
            .ok_or_else(|| "robot is absent from the current account snapshot".into())
    }

    fn query_start(
        &self,
        state: &mut State,
        robot_id: &[u8; 32],
    ) -> Result<crate::identity_snapshot::IdentityStamp, String> {
        self.network_allowed()?;
        self.sync_identity(state)?;
        Self::account_robot(state, robot_id)?;
        state
            .identity
            .as_ref()
            .map(|identity| identity.stamp().clone())
            .ok_or_else(|| NEED_INSTALL.into())
    }

    fn account_operation(
        &self,
        action: &AccountAccessRequest,
    ) -> Result<AccountAccessReply, String> {
        action.validate()?;
        if let AccountAccessRequest::Discover { budget_ms } = action {
            return self
                .discover(Duration::from_millis(*budget_ms))
                .map(|robots| AccountAccessReply::Discovery { robots });
        }
        let (robot_id, budget) = match action {
            AccountAccessRequest::Install { .. } | AccountAccessRequest::InstallPaired { .. } => {
                return Err("account installation requires daemon registry coordination".into())
            }
            AccountAccessRequest::Discover { .. } => unreachable!("discovery returned above"),
            AccountAccessRequest::Probe {
                robot_id,
                budget_ms,
            } => (*robot_id, Duration::from_millis(*budget_ms)),
            AccountAccessRequest::Catalog { robot_id }
            | AccountAccessRequest::Schema { robot_id, .. } => (*robot_id, QUERY_BUDGET),
        };
        let deadline = Instant::now() + budget;
        let mut state = self.state()?;
        let stamp = self.query_start(&mut state, &robot_id)?;
        if Self::account_robot(&state, &robot_id)?
            .endpoint_key
            .is_none()
        {
            if matches!(action, AccountAccessRequest::Probe { .. }) {
                self.fence_reply(&mut state, &stamp)?;
                return Ok(AccountAccessReply::Presence {
                    robot_id,
                    presence: RobotPresence::Unknown {
                        reason: "account service supplied no endpoint key for this robot".into(),
                    },
                });
            }
            return Err(
                "account robot has no pinned endpoint key; refresh its registration".into(),
            );
        }
        let route = account_access::robot_route(&robot_id);
        let plane = self.ensure_plane(&mut state)?;
        let outcome = match action {
            AccountAccessRequest::Probe { .. } => plane
                .probe_robot(&route, remaining(deadline)?)
                .map(|outcome| AccountAccessReply::Presence {
                    robot_id,
                    presence: match outcome {
                        RobotProbeOutcome::Online => RobotPresence::Online,
                        RobotProbeOutcome::NotReached { reason } => {
                            RobotPresence::NotReached { reason }
                        }
                    },
                }),
            AccountAccessRequest::Catalog { .. } => plane
                .query_catalog(&route, remaining(deadline)?)
                .map(|catalog| AccountAccessReply::Catalog { robot_id, catalog }),
            AccountAccessRequest::Schema { topic, .. } => plane
                .query_schema(&route, topic, remaining(deadline)?)
                .map(|schema| AccountAccessReply::Schema { robot_id, schema }),
            AccountAccessRequest::Install { .. }
            | AccountAccessRequest::InstallPaired { .. }
            | AccountAccessRequest::Discover { .. } => {
                unreachable!("install was refused before identity and I/O")
            }
        };
        self.fence_reply(&mut state, &stamp)?;
        let reply = outcome?;
        if let AccountAccessReply::Catalog { catalog, .. } = &reply {
            if catalog.error.is_none() {
                state.catalogs.insert(robot_id, catalog.clone());
            }
        }
        Ok(reply)
    }

    fn named_schema(&self, robot_id: [u8; 32], requested: &str) -> Result<SchemaReply, String> {
        let deadline = Instant::now() + QUERY_BUDGET;
        let mut state = self.state()?;
        let stamp = self.query_start(&mut state, &robot_id)?;
        if Self::account_robot(&state, &robot_id)?
            .endpoint_key
            .is_none()
        {
            return Err(
                "account robot has no pinned endpoint key; refresh its registration".into(),
            );
        }
        let route = account_access::robot_route(&robot_id);
        let catalog = if let Some(catalog) = state.catalogs.get(&robot_id) {
            catalog.clone()
        } else {
            let result = self
                .ensure_plane(&mut state)?
                .query_catalog(&route, remaining(deadline)?);
            self.fence_reply(&mut state, &stamp)?;
            let catalog = result?;
            if catalog.error.is_none() {
                state.catalogs.insert(robot_id, catalog.clone());
            }
            catalog
        };
        let Some(topic) = account_access::schema_topic_for_type(&catalog, requested)? else {
            self.fence_reply(&mut state, &stamp)?;
            return Ok(SchemaReply::not_found(
                &route,
                requested,
                "the authorized robot catalog has no exact matching schema type",
            ));
        };
        let result =
            self.ensure_plane(&mut state)?
                .query_schema(&route, &topic, remaining(deadline)?);
        self.fence_reply(&mut state, &stamp)?;
        let mut schema = result?;
        if schema
            .docs
            .first()
            .is_some_and(|root| root.qualified != requested)
        {
            return Err("robot schema root does not match the requested exact type".into());
        }
        schema.requested = requested.into();
        Ok(schema)
    }

    fn restore_readers(&self, state: &mut State, deadline: Instant) -> Result<(), String> {
        let Some(readers) = state.handover.take() else {
            return Ok(());
        };
        let ready = match self.sync_identity(state) {
            Err(error) => Err(error),
            Ok(()) if state.serving_gateway => loop {
                let result = self.ensure_plane(state).and_then(|plane| {
                    plane
                        .synchronize_shared_endpoint_until(deadline)
                        .map(|_| ())
                });
                match result {
                    Ok(()) => break Ok(()),
                    Err(error) if Instant::now() >= deadline => break Err(error),
                    Err(_) => std::thread::sleep(Duration::from_millis(20)),
                }
            },
            Ok(()) => Ok(()),
        };
        let mut first_error = None;
        let mut failed = BTreeSet::new();
        let mut restored_keys: BTreeSet<TopicKey> = BTreeSet::new();
        for (key, hash) in readers {
            if !self
                .pins()?
                .get(&key)
                .is_some_and(|pin| pin.route == Route::Wan && pin.live && pin.schema_hash == hash)
            {
                continue;
            }
            let restored = match &ready {
                Ok(()) if restoration_time_left(deadline, Instant::now()).is_some() => self
                    .ensure_plane(state)
                    .and_then(|plane| plane.ensure_mirror_until(&key, hash, deadline)),
                Ok(()) => Err("outbound reader restoration exceeded its time budget".into()),
                Err(error) => Err(error.clone()),
            };
            if let Err(error) = restored {
                tracing::debug!(robot = %key.robot, topic = %key.topic, observer = key.observer,
                    serving = state.serving_gateway, error = %error, "outbound reader pin restoration failed");
                if let Some(plane) = state
                    .plane
                    .as_ref()
                    .filter(|plane| !plane.shares_owned_endpoint())
                {
                    // Native cancellation closes the affected robot/mode wire
                    // connection, so earlier successes on it may also be lost.
                    // This is an in-memory check, not another restoration attempt.
                    for previous in &restored_keys {
                        if previous.robot == key.robot
                            && previous.observer == key.observer
                            && !plane.has_reader(previous).unwrap_or(false)
                        {
                            failed.insert(previous.clone());
                        }
                    }
                }
                failed.insert(key.clone());
                first_error.get_or_insert_with(|| {
                    format!(
                        "remote mirror lost for robot '{}' topic '{}': {error}",
                        key.robot, key.topic
                    )
                });
            } else {
                restored_keys.insert(key);
            }
        }
        if state
            .plane
            .as_ref()
            .is_some_and(|plane| plane.take_shared_transport_loss())
        {
            // A partially framed persistent exchange fences every shared reader,
            // including pins restored before the failing exchange.
            state.pending_all = true;
            first_error.get_or_insert_with(|| {
                "outbound readers lost their shared control transport during restoration".into()
            });
        }
        state.pending_restores.extend(failed);
        first_error.map_or(Ok(()), Err)
    }

    fn failed_robot(&self, state: &mut State, robot: &str) -> Result<(), String> {
        state.pending_robots.insert(robot.into());
        if let Some(plane) = state.plane.as_ref() {
            if let Err(error) = plane.retire_robot(robot) {
                let _cleanup = Self::invalidate(state);
                return Err(error);
            }
        }
        if let Some(id) = account_access::parse_robot_route(robot)? {
            state.catalogs.remove(&id);
        }
        Ok(())
    }
}

impl MirrorPlane for AccountWanController {
    fn begin_serving(&self) -> Result<(), String> {
        let mut state = self
            .state
            .lock()
            .map_err(|_| "account robot controller state is poisoned")?;
        if state.handover.is_some() {
            return Ok(());
        }
        // Keep the registry generation, holders and schema pins while endpoint
        // ownership changes. Only failed restorations retire a demanded key.
        state.handover = Some(
            self.pins()?
                .iter()
                .filter(|(_, pin)| pin.route == Route::Wan && pin.live)
                .map(|(key, pin)| (key.clone(), pin.schema_hash))
                .collect(),
        );
        state.serving_gateway = true;
        let proof = state.registry.owner_certificate()?;
        let cleanup = state
            .plane
            .take()
            .map_or(Ok(()), |plane| plane.invalidate_account());
        let restored = state.registry.replace_owner_certificate(proof);
        cleanup.and(restored)
    }

    fn serving_ready(&self) -> Result<(), String> {
        let deadline = Instant::now() + RESTORATION_BUDGET;
        let mut state = match self.state.try_lock() {
            Ok(state) => state,
            Err(TryLockError::Poisoned(_)) => {
                return Err("account robot controller state is poisoned".into());
            }
            Err(TryLockError::WouldBlock) => {
                // begin_serving already retired the previous endpoint. Do not
                // wait for unrelated metadata I/O before supervising the child.
                // Registry retirement observes these canceled pins independently.
                let mut first = None;
                for (key, pin) in self.pins()?.iter_mut() {
                    if pin.route == Route::Wan && pin.live {
                        pin.live = false;
                        first.get_or_insert_with(|| format!(
                            "remote mirror lost for robot '{}' topic '{}': controller busy during restoration",
                            key.robot, key.topic
                        ));
                    }
                }
                return Err(first.unwrap_or_else(|| "controller busy during restoration".into()));
            }
        };
        self.restore_readers(&mut state, deadline)
    }

    fn holds_outbound_readers(&self) -> bool {
        // Busy is not evidence that an idle stop is safe. The shared plane also
        // retains a native admission hold, which closes the sampled-check race.
        let Ok(state) = self.state.try_lock() else {
            return true;
        };
        if !state.serving_gateway {
            return false;
        }
        self.pins.try_lock().map_or(true, |pins| {
            pins.values().any(|pin| pin.route == Route::Wan && pin.live)
        })
    }

    fn end_serving(&self) {
        let Ok(mut state) = self.state.lock() else {
            return;
        };
        if !state.serving_gateway && state.handover.is_none() {
            return;
        }
        let readers = self.pins().map(|pins| {
            pins.iter()
                .filter(|(_, pin)| pin.route == Route::Wan && pin.live)
                .map(|(key, pin)| (key.clone(), pin.schema_hash))
                .collect()
        });
        if let Ok(readers) = readers {
            state.handover = Some(readers);
        }
        let proof = state.registry.owner_certificate();
        if let Some(plane) = state.plane.take() {
            let _cleanup = plane.invalidate_account();
        }
        if let Ok(proof) = proof {
            let _restored = state.registry.replace_owner_certificate(proof);
        }
        state.serving_gateway = false;
        if let Err(error) = self.restore_readers(&mut state, Instant::now() + RESTORATION_BUDGET) {
            tracing::warn!(error = %error, "outbound reader restoration failed after serving stopped");
        }
    }

    fn account_access(&self, action: &AccountAccessRequest) -> Result<AccountAccessReply, String> {
        self.account_operation(action)
    }

    fn install_account_snapshot(
        &self,
        snapshot: &AccountSnapshot,
        registry: &mut DemandRegistry,
    ) -> Result<AccountAccessReply, String> {
        self.install(snapshot, registry)?;
        Ok(AccountAccessReply::Installed {
            robot_count: snapshot.robots.len(),
        })
    }

    fn account_schema_by_type(
        &self,
        robot_id: [u8; 32],
        requested: &str,
    ) -> Result<SchemaReply, String> {
        self.named_schema(robot_id, requested)
    }

    fn install_paired_robots(
        &self,
        robots: &[crate::account_access::PairedRobot],
        registry: &mut DemandRegistry,
    ) -> Result<(), String> {
        self.install_paired(robots, registry)
    }

    fn paired_catalog(&self, robot: &str) -> Option<Result<CatalogReply, String>> {
        self.paired_metadata(robot, None)
            .map(|result| result.map(|(catalog, _)| catalog))
    }

    fn paired_schema(&self, robot: &str, requested: &str) -> Option<Result<SchemaReply, String>> {
        self.paired_metadata(robot, Some(requested)).map(|result| {
            result.and_then(|(_, schema)| {
                schema.ok_or_else(|| "paired schema response was absent".into())
            })
        })
    }

    fn refresh_identity(&self, registry: &mut DemandRegistry) -> Result<(), String> {
        self.refresh(registry)
    }

    /// Observer channel on WAN; legacy LAN mirrors keep their canonical channel.
    fn local_channel(&self, key: &TopicKey) -> String {
        if matches!(self.route(key), Ok(Route::Wan)) {
            key.local_topic()
        } else {
            key.topic.clone()
        }
    }

    /// The route this controller PINNED when it ensured the mirror, never the route
    /// it would choose again: membership is mutable here, so a fresh decision could
    /// name a plane that never carried a frame for this key. An unpinned key has no
    /// live mirror, so it has no serving plane to report.
    ///
    /// A poisoned route lock also reports unknown rather than propagating. This is
    /// a diagnostic read on the status path: refusing the whole demand table because
    /// one row cannot be attributed would lose the rows that can be, and the caller
    /// already has to handle unknown for every older daemon.
    fn serving_plane(&self, key: &TopicKey) -> Option<crate::protocol::ServingPlane> {
        match self.pins().ok()?.get(key)?.route {
            Route::Lan => Some(crate::protocol::ServingPlane::Zenoh),
            Route::Wan => Some(crate::protocol::ServingPlane::Iroh),
        }
    }

    fn prepare_demand(&self, key: &TopicKey, registry: &mut DemandRegistry) -> Result<(), String> {
        if self.route(key)? == Route::Lan {
            return Ok(());
        }
        self.network_allowed()?;
        let mut state = self.state()?;
        if state.handover.is_some() {
            return Err(BUSY.into());
        }
        self.retire_pending(&mut state, registry)?;
        let result = self.sync_identity(&mut state);
        self.retire_pending(&mut state, registry)?;
        result?;
        if state.registry.get(&key.robot)?.is_none() {
            return Err("robot has no current pinned WAN endpoint".into());
        }
        if self
            .pins()?
            .get(key)
            .is_some_and(|pin| pin.route == Route::Wan)
        {
            let live = state
                .plane
                .as_ref()
                .map_or(Ok(false), |plane| plane.has_reader(key))?;
            if !live {
                self.failed_robot(&mut state, &key.robot)?;
                self.retire_pending(&mut state, registry)?;
            }
        }
        Ok(())
    }

    fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError> {
        let wan_error = |reason| MirrorError::Iroh {
            key: key.clone(),
            reason,
        };
        let route = self.route(key).map_err(wan_error)?;
        if route == Route::Lan {
            self.lan.ensure_mirror(key, schema_hash)?;
        } else {
            let mut state = self.state().map_err(wan_error)?;
            if state.handover.is_some() {
                return Err(wan_error(BUSY.into()));
            }
            self.sync_identity(&mut state).map_err(wan_error)?;
            let stamp = state
                .identity
                .as_ref()
                .map(|identity| identity.stamp().clone());
            let result = self
                .ensure_plane(&mut state)
                .map_err(wan_error)?
                .ensure_mirror(key, schema_hash);
            if let Some(stamp) = stamp {
                if let Err(error) = self.fence_reply(&mut state, &stamp) {
                    self.failed_robot(&mut state, &key.robot)
                        .map_err(wan_error)?;
                    return Err(wan_error(error));
                }
            }
            if let Err(error) = result {
                if !state
                    .plane
                    .as_ref()
                    .is_some_and(IrohMirrorPlane::shares_owned_endpoint)
                {
                    self.failed_robot(&mut state, &key.robot)
                        .map_err(wan_error)?;
                }
                return Err(error);
            }
        }
        self.pins().map_err(wan_error)?.insert(
            key.clone(),
            Pin {
                route,
                schema_hash,
                live: true,
            },
        );
        Ok(())
    }

    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
        let route = match self.route(key) {
            Ok(route) => route,
            Err(_) => return MirrorRelease::Lingering,
        };
        if route == Route::Lan {
            return self.lan.release_mirror(key);
        }
        // Record the last holder's departure even while migration owns the WAN
        // state. Restoration must never revive a reader released in that window.
        if let Ok(mut pins) = self.pins() {
            if let Some(pin) = pins.get_mut(key) {
                pin.live = false;
            }
        }
        let Ok(state) = self.state() else {
            return MirrorRelease::Lingering;
        };
        // A full invalidation or failed-robot cleanup already closed the physical
        // reader. Keep the pin until the daemon confirms registry retirement.
        if state.pending_all
            || state.pending_robots.contains(&key.robot)
            || state.pending_restores.contains(key)
        {
            return MirrorRelease::Retired;
        }
        state
            .plane
            .as_ref()
            .map_or(MirrorRelease::Retired, |plane| plane.release_mirror(key))
    }

    fn mirror_retired(&self, key: &TopicKey) {
        if let Ok(mut pins) = self.pins() {
            pins.remove(key);
        }
    }
}
