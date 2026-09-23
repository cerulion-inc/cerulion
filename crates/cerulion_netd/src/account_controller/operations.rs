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
        let (robot_id, budget) = match action {
            AccountAccessRequest::Install { .. } => {
                return Err("account installation requires daemon registry coordination".into())
            }
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
            AccountAccessRequest::Install { .. } => {
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

    fn refresh_identity(&self, registry: &mut DemandRegistry) -> Result<(), String> {
        self.refresh(registry)
    }

    fn prepare_demand(&self, key: &TopicKey, registry: &mut DemandRegistry) -> Result<(), String> {
        if self.route(key)? == Route::Lan {
            return Ok(());
        }
        self.network_allowed()?;
        let mut state = self.state()?;
        self.retire_pending(&mut state, registry)?;
        let result = self.sync_identity(&mut state);
        self.retire_pending(&mut state, registry)?;
        result?;
        if state.registry.get(&key.robot)?.is_none() {
            return Err("robot has no current pinned WAN endpoint".into());
        }
        if self.pins()?.get(key) == Some(&Route::Wan) {
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
                self.failed_robot(&mut state, &key.robot)
                    .map_err(wan_error)?;
                return Err(error);
            }
        }
        self.pins().map_err(wan_error)?.insert(key.clone(), route);
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
        let Ok(state) = self.state() else {
            return MirrorRelease::Lingering;
        };
        // A full invalidation or failed-robot cleanup already closed the physical
        // reader. Keep the pin until the daemon confirms registry retirement.
        if state.pending_all || state.pending_robots.contains(&key.robot) {
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
