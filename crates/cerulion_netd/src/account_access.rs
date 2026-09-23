// SPDX-License-Identifier: AGPL-3.0-only
//! Public identity messages for account robot access over netd's existing socket.
//!
//! These messages carry no bearer token or private key. Receiving a snapshot is
//! not authentication: the controller must bind it to the current local login,
//! device key and certificate before using it. Listing and probing never enroll
//! a device; only an authorized metadata query or demand may do that.

use std::collections::BTreeSet;

use cerulion_core::transport::cerulion_q::{CatalogReply, SchemaReply};
use serde::{Deserialize, Serialize};

/// First daemon protocol with the account access vocabulary.
pub const MIN_DAEMON_VERSION: u32 = 8;
/// Bound the account-service catalog and local control payload identically.
pub const MAX_ROBOTS: usize = 4096;
/// Total probe budget, below the control client's five-second round trip.
pub const MAX_PROBE_BUDGET_MS: u64 = 4000;
const MAX_CHAIN_BYTES: usize = 32 * 1024;

/// Reserved identity route carried through existing robot-string fields.
pub const ACCOUNT_ROUTE_PREFIX: &str = "account:";

/// Encode a stable account robot identity, independently of its display name.
pub fn robot_route(robot_id: &[u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut route = String::with_capacity(ACCOUNT_ROUTE_PREFIX.len() + 64);
    route.push_str(ACCOUNT_ROUTE_PREFIX);
    for byte in robot_id {
        route.push(char::from(HEX[usize::from(byte >> 4)]));
        route.push(char::from(HEX[usize::from(byte & 15)]));
    }
    route
}

/// Ordinary names return `None`; malformed reserved routes fail closed.
pub fn parse_robot_route(route: &str) -> Result<Option<[u8; 32]>, String> {
    let normalized = route.trim();
    let Some(encoded) = normalized.strip_prefix(ACCOUNT_ROUTE_PREFIX) else {
        return Ok(None);
    };
    let invalid = || "account robot route requires 64 lowercase hexadecimal digits".to_string();
    if route != normalized || encoded.len() != 64 {
        return Err(invalid());
    }
    let digit = |byte: u8| match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    };
    let mut robot_id = [0; 32];
    for (index, pair) in encoded.as_bytes().as_chunks::<2>().0.iter().enumerate() {
        let high = digit(pair[0]).ok_or_else(invalid)?;
        let low = digit(pair[1]).ok_or_else(invalid)?;
        robot_id[index] = (high << 4) | low;
    }
    Ok(Some(robot_id))
}

/// Resolve an exact type name using one authorized robot catalog. Multiple topics
/// are equivalent only when every matching entry carries the same known hash.
/// No package suffix or bare-name fallback is attempted.
pub fn schema_topic_for_type(
    catalog: &CatalogReply,
    requested: &str,
) -> Result<Option<String>, String> {
    validate_label(requested, 1024, "requested schema name")?;
    if catalog.version != cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION {
        return Err("robot catalog has an unsupported version".into());
    }
    if catalog.error.is_some() {
        return Err("robot catalog refused schema resolution".into());
    }
    let mut matches: Vec<_> = catalog
        .entries
        .iter()
        .filter(|entry| entry.schema_name.as_deref() == Some(requested))
        .collect();
    if matches.iter().any(|entry| {
        !entry.topic.starts_with('/')
            || entry.topic.len() > 1024
            || entry.topic.chars().any(char::is_control)
    }) {
        return Err("robot catalog contains an invalid schema topic".into());
    }
    matches.sort_by(|a, b| a.topic.cmp(&b.topic));
    let Some(first) = matches.first() else {
        return Ok(None);
    };
    if matches.len() > 1
        && (first.schema_hash.is_none()
            || matches
                .iter()
                .any(|entry| entry.schema_hash != first.schema_hash))
    {
        let mut topics = matches
            .iter()
            .take(8)
            .map(|entry| entry.topic.as_str())
            .collect::<Vec<_>>()
            .join(", ");
        if matches.len() > 8 {
            topics.push_str(&format!(" (and {} more topics)", matches.len() - 8));
        }
        return Err(format!(
            "schema '{requested}' is ambiguous across topics: {topics}"
        ));
    }
    Ok(Some(first.topic.clone()))
}

/// One account-owned robot. Its identifier, not its label, selects a WAN peer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AccountRobot {
    /// Account-service robot identifier.
    pub robot_id: [u8; 32],
    /// Display label; duplicate labels must never choose a robot implicitly.
    pub hostname: String,
    /// Pinned transport public key. Absence means presence is unknown.
    pub endpoint_key: Option<[u8; 32]>,
}

/// Public snapshot produced from one checked account-service response.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AccountSnapshot {
    /// Original persisted auth identifier (UUID on the hosted service).
    pub auth_account_id: String,
    /// Mapped pairing account identifier.
    pub pairing_account_id: [u8; 32],
    /// Public half of the current local desk key.
    pub device_key: [u8; 32],
    /// Optional postcard owner certificate presentation; public proof, no seed.
    pub owner_chain: Option<Vec<u8>>,
    /// Owned robots, including entries whose older service omitted an endpoint.
    pub robots: Vec<AccountRobot>,
}

impl AccountSnapshot {
    /// Structural limits only. Local identity and cryptographic checks are separate.
    pub fn validate(&self) -> Result<(), String> {
        validate_label(&self.auth_account_id, 256, "account identifier")?;
        if self.robots.len() > MAX_ROBOTS {
            return Err("account robot snapshot exceeds the robot limit".into());
        }
        if self
            .owner_chain
            .as_ref()
            .is_some_and(|chain| chain.is_empty() || chain.len() > MAX_CHAIN_BYTES)
        {
            return Err("owner certificate chain has an invalid size".into());
        }
        let mut ids = BTreeSet::new();
        let mut endpoints = BTreeSet::new();
        for robot in &self.robots {
            validate_label(&robot.hostname, 253, "robot display label")?;
            if !ids.insert(robot.robot_id) {
                return Err("account robot snapshot repeats a robot identifier".into());
            }
            if robot.endpoint_key.is_some_and(|key| !endpoints.insert(key)) {
                return Err(
                    "account robot snapshot assigns one endpoint to multiple robots".into(),
                );
            }
        }
        Ok(())
    }
}

fn validate_label(value: &str, max: usize, field: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.len() > max || value.chars().any(char::is_control) {
        return Err(format!("invalid {field}"));
    }
    Ok(())
}

/// Account operations on the existing owner-only Unix socket.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
pub enum AccountAccessRequest {
    /// Install a locally verified snapshot without connecting to any robot.
    Install {
        /// Complete replacement membership; no secret material.
        snapshot: Box<AccountSnapshot>,
    },
    /// Confirm TLS identity within one total budget; no enrollment or demand.
    Probe {
        /// Exact account-service robot identifier.
        robot_id: [u8; 32],
        /// Includes local wait, endpoint initialization and handshake.
        budget_ms: u64,
    },
    /// Authorized metadata access may once enroll the owner's device.
    Catalog {
        /// Exact account-service robot identifier.
        robot_id: [u8; 32],
    },
    /// Retrieve the schema for one topic after current authorization.
    Schema {
        /// Exact account-service robot identifier.
        robot_id: [u8; 32],
        /// Canonical topic whose schema is requested.
        topic: String,
    },
}

impl AccountAccessRequest {
    /// Validate bounded public input before any state change or network operation.
    pub fn validate(&self) -> Result<(), String> {
        match self {
            Self::Install { snapshot } => snapshot.validate(),
            Self::Probe { budget_ms, .. } if !(1..=MAX_PROBE_BUDGET_MS).contains(budget_ms) => {
                Err("robot probe budget must be between 1 and 4000 milliseconds".into())
            }
            Self::Schema { topic, .. } => {
                validate_label(topic, 1024, "schema topic")?;
                if !topic.starts_with('/') {
                    return Err("schema topic must be an absolute canonical topic".into());
                }
                Ok(())
            }
            Self::Probe { .. } | Self::Catalog { .. } => Ok(()),
        }
    }

    /// Reject a reply for a different operation even when its correlation matches.
    pub fn accepts(&self, reply: &AccountAccessReply) -> bool {
        match (self, reply) {
            (Self::Install { snapshot }, AccountAccessReply::Installed { robot_count }) => {
                snapshot.robots.len() == *robot_count
            }
            (
                Self::Probe { robot_id, .. },
                AccountAccessReply::Presence {
                    robot_id: actual, ..
                },
            )
            | (
                Self::Catalog { robot_id },
                AccountAccessReply::Catalog {
                    robot_id: actual, ..
                },
            ) => robot_id == actual,
            (
                Self::Schema { robot_id, topic },
                AccountAccessReply::Schema {
                    robot_id: actual,
                    schema,
                },
            ) => robot_id == actual && schema.requested == *topic,
            _ => false,
        }
    }
}

/// Reachability evidence, deliberately independent from ownership and enrollment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
pub enum RobotPresence {
    /// Completed TLS handshake confirmed the catalog's pinned endpoint key.
    Online,
    /// The pinned endpoint did not complete a handshake within this attempt.
    NotReached {
        /// Diagnostic for this attempt; never an enduring offline assertion.
        reason: String,
    },
    /// An older account service supplied no transport key to probe.
    Unknown {
        /// Why a probe cannot establish presence.
        reason: String,
    },
}

/// Typed result; errors use the existing netd error response.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
pub enum AccountAccessReply {
    /// Snapshot verified and installed. No reachability claim.
    Installed {
        /// Number of catalog entries, including unknown endpoints.
        robot_count: usize,
    },
    /// Evidence from a probe, without enrollment or topic demands.
    Presence {
        /// Exact requested robot identifier.
        robot_id: [u8; 32],
        /// Typed evidence, with no absent-field positive default.
        presence: RobotPresence,
    },
    /// Current authorized catalog.
    Catalog {
        /// Exact requested robot identifier.
        robot_id: [u8; 32],
        /// Shared wire catalog type, preserving robot refusal information.
        catalog: CatalogReply,
    },
    /// Current authorized schema, including structured not-found.
    Schema {
        /// Exact requested robot identifier.
        robot_id: [u8; 32],
        /// Shared wire schema type.
        schema: SchemaReply,
    },
}

/// Unique required key disambiguates this result from other untagged responses.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AccountAccessResponse {
    /// Echoed correlation identifier.
    pub id: u64,
    /// Required discriminator; never a default empty list or presence claim.
    pub account_access: AccountAccessReply,
}
