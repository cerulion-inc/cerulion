// SPDX-License-Identifier: AGPL-3.0-only
//! The live half: ONE ros2-client publisher of `unitree_api/Request` on the
//! robot's `/api/sport/request` topic (ros2-client applies the ROS `rt/`
//! prefix and the type-name mangling, so this is exactly the DDS topic the
//! Go2's sport service reads).
//!
//! The participant is the example's one-per-process `Go2Participant`
//! (`lib/cerulion_go2_dds`): `only_networks` / `GO2_IFACE` pins discovery to
//! the robot-LAN interface on a multi-homed companion, the same constraint
//! the ingress bridge documents. QoS is RELIABLE + VOLATILE, the vendor
//! SDK's request QoS; a request is a command, never best effort.
//!
//! Field order below is the DROP order: the publisher goes first, then the
//! node that owns its writer identity, then the participant everything
//! hangs off. Dropping the participant sends the SPDP/SEDP disposes, so the
//! robot unregisters our writer at once rather than after the lease.

use cerulion_go2_dds::messages::Request;
use cerulion_go2_dds::ros2_client::{MessageTypeName, Name, Node, Publisher};
use cerulion_go2_dds::{reliable_volatile_qos, Go2Participant, ParticipantConfig};

/// The ROS topic the Go2 sport service reads requests on.
pub const SPORT_REQUEST_TOPIC: &str = "/api/sport/request";

/// The ROS node name this writer registers under (in the example's
/// `/cerulion_go2` namespace).
const NODE_NAME: &str = "sport_driver";

/// A live DDS writer of sport requests.
pub struct DdsRequestWriter {
    publisher: Publisher<Request>,
    _node: Node,
    _participant: Go2Participant,
}

impl std::fmt::Debug for DdsRequestWriter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DdsRequestWriter")
            .field("topic", &SPORT_REQUEST_TOPIC)
            .finish()
    }
}

impl DdsRequestWriter {
    /// Build the participant, the node, the topic and the writer. Every
    /// failure is a `String` naming the step, for the node's `init` to turn
    /// into a graph-build refusal.
    pub fn open(config: &ParticipantConfig) -> Result<Self, String> {
        let participant =
            Go2Participant::new(config).map_err(|e| format!("DDS participant: {e}"))?;
        let mut node = participant
            .create_node(NODE_NAME)
            .map_err(|e| format!("DDS node {NODE_NAME:?}: {e}"))?;
        let name = Name::new("/api/sport", "request")
            .map_err(|e| format!("DDS topic name {SPORT_REQUEST_TOPIC:?}: {e:?}"))?;
        let qos = reliable_volatile_qos();
        let topic = node
            .create_topic(&name, MessageTypeName::new("unitree_api", "Request"), &qos)
            .map_err(|e| format!("create_topic {SPORT_REQUEST_TOPIC:?}: {e:?}"))?;
        let publisher = node
            .create_publisher::<Request>(&topic, Some(qos))
            .map_err(|e| format!("create_publisher {SPORT_REQUEST_TOPIC:?}: {e:?}"))?;
        tracing::info!(
            topic = SPORT_REQUEST_TOPIC,
            domain = config.domain_id,
            "sport_driver: DDS request writer open"
        );
        Ok(Self {
            publisher,
            _node: node,
            _participant: participant,
        })
    }

    /// Publish one request. A reliable writer may block up to the QoS's
    /// `max_blocking_time` (100 ms) when the robot's reader is saturated;
    /// that is the inherent cost of a reliable command.
    pub fn publish(&self, request: Request) -> Result<(), String> {
        self.publisher
            .publish(request)
            .map_err(|e| format!("publish {SPORT_REQUEST_TOPIC:?}: {e}"))
    }
}
