// SPDX-License-Identifier: AGPL-3.0-only
//! Read the local network daemon's current metadata without discovery or startup.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{collect_schema_closure, SchemaDoc, SchemaReply};
use cerulion_core::transport::{ix_config_shm_identity_from_json, IceoryxShmIdentity};
use cerulion_core::SchemaServing;
use cerulion_netd::{ClientError, NetdClient};

use crate::wire::WireError;

const QUERY_BUDGET: Duration = Duration::from_secs(5);

pub(crate) struct LocalSchemas {
    topic_types: BTreeMap<String, String>,
    hash_types: BTreeMap<u64, String>,
    docs: BTreeMap<String, SchemaDoc>,
}

impl LocalSchemas {
    fn from_serving(serving: SchemaServing) -> Self {
        let mut result = Self {
            topic_types: BTreeMap::new(),
            hash_types: BTreeMap::new(),
            docs: BTreeMap::new(),
        };
        // The egress source merges first-wins; retain that rule when indexing.
        for binding in serving.topic_schemas {
            result
                .topic_types
                .entry(binding.topic)
                .or_insert(binding.schema_name);
        }
        for binding in serving.schema_hashes {
            result
                .hash_types
                .entry(binding.schema_hash)
                .or_insert(binding.qualified);
        }
        for doc in serving.schema_docs {
            result.docs.entry(doc.qualified.clone()).or_insert(doc);
        }
        result
    }

    pub(crate) fn root_type(&self, topic: &str, hash: Option<u64>) -> Option<&str> {
        self.topic_types
            .get(topic)
            .or_else(|| hash.and_then(|hash| self.hash_types.get(&hash)))
            .map(String::as_str)
    }

    pub(crate) fn reply(&self, robot: &str, topic: &str, hash: Option<u64>) -> SchemaReply {
        match self.root_type(topic, hash) {
            Some(root) => match collect_schema_closure(root, &self.docs) {
                Some(docs) => SchemaReply::found(robot, topic, docs),
                None => SchemaReply::not_found(
                    robot,
                    topic,
                    "the local source has no custom schema closure for this topic",
                ),
            },
            None => SchemaReply::not_found(
                robot,
                topic,
                "the local source has no schema binding for this topic",
            ),
        }
    }
}

pub(crate) async fn load(
    socket: PathBuf,
    expected_namespace: IceoryxShmIdentity,
) -> Result<LocalSchemas, WireError> {
    let deadline = Instant::now() + QUERY_BUDGET;
    let operation = tokio::task::spawn_blocking(move || {
        let mut client =
            NetdClient::connect_existing_at_until(socket, deadline).map_err(client_failure)?;
        let snapshot = client
            .query_serving_schema_until(deadline)
            .map_err(client_failure)?;
        let namespace =
            ix_config_shm_identity_from_json(&snapshot.ix_config_json).map_err(|_| {
                failure("local schema source supplied an invalid shared-memory namespace")
            })?;
        if namespace != expected_namespace {
            return Err(failure(
                "local schema source uses a different shared-memory namespace",
            ));
        }
        Ok(LocalSchemas::from_serving(snapshot.schema_serving))
    });
    // The worker has the same deadline for connect and every partial I/O. This
    // timeout also counts time waiting for Tokio's blocking pool to schedule it.
    tokio::time::timeout_at(tokio::time::Instant::from_std(deadline), operation)
        .await
        .map_err(|_| failure("local schema query exceeded its deadline"))?
        .map_err(|_| failure("local schema query worker failed"))?
}

fn failure(reason: &str) -> WireError {
    WireError::SchemaSource(reason.to_owned())
}

fn client_failure(error: ClientError) -> WireError {
    // The remote peer receives no local path or unchecked daemon-supplied prose.
    // ErrorKind is a bounded OS category, not the peer's text.
    match error {
        ClientError::Connect { source, .. } => failure(&format!(
            "cannot reach the local network daemon ({}); check that it is running",
            source.kind()
        )),
        ClientError::Io(source) => failure(&format!(
            "local schema query I/O failed ({})",
            source.kind()
        )),
        ClientError::Protocol(_) => {
            failure("local schema protocol is invalid or incompatible; restart the network daemon")
        }
        ClientError::Netd { .. } => failure("local network daemon refused the schema snapshot"),
        ClientError::Spawn { .. } => {
            failure("local schema client attempted an unsupported daemon startup")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::{SchemaEncoding, SchemaHashName, TopicSchema};
    use cerulion_netd::client::ConnectAttempt;

    #[test]
    fn indexing_preserves_first_wins_and_topic_binding_precedes_hash_binding() {
        let schemas = LocalSchemas::from_serving(SchemaServing {
            topic_schemas: vec![
                TopicSchema {
                    topic: "/reading".into(),
                    schema_name: "example/First".into(),
                },
                TopicSchema {
                    topic: "/reading".into(),
                    schema_name: "example/Second".into(),
                },
            ],
            schema_hashes: vec![
                SchemaHashName {
                    schema_hash: 7,
                    qualified: "example/FromHash".into(),
                },
                SchemaHashName {
                    schema_hash: 7,
                    qualified: "example/WrongHash".into(),
                },
            ],
            schema_docs: vec![
                SchemaDoc {
                    qualified: "example/First".into(),
                    encoding: SchemaEncoding::Msg,
                    text: "uint64 first\n".into(),
                    deps: vec![],
                },
                SchemaDoc {
                    qualified: "example/First".into(),
                    encoding: SchemaEncoding::Msg,
                    text: "uint32 second\n".into(),
                    deps: vec![],
                },
            ],
        });
        assert_eq!(
            schemas.root_type("/reading", Some(7)),
            Some("example/First")
        );
        assert_eq!(schemas.root_type("/raw", Some(7)), Some("example/FromHash"));
        assert_eq!(schemas.root_type("/unknown", Some(8)), None);
        let reply = schemas.reply("robot", "/reading", Some(7));
        assert_eq!(reply.docs.len(), 1);
        assert_eq!(reply.docs[0].text, "uint64 first\n");
        assert!(reply.error.is_none());
    }

    #[test]
    fn client_error_mapping_never_forwards_paths_or_peer_prose() {
        const PRIVATE: &str = "private-detail-sentinel\n";
        let io_error = || std::io::Error::new(std::io::ErrorKind::PermissionDenied, PRIVATE);
        let errors = [
            ClientError::Connect {
                socket: PRIVATE.into(),
                attempt: ConnectAttempt::ExistingOnly,
                source: io_error(),
            },
            ClientError::Io(io_error()),
            ClientError::Protocol(PRIVATE.into()),
            ClientError::Netd {
                error: PRIVATE.into(),
                robot: Some(PRIVATE.into()),
                topic: Some(PRIVATE.into()),
            },
            ClientError::Spawn {
                bin: PRIVATE.into(),
                source: io_error(),
            },
        ];
        for error in errors {
            let message = client_failure(error).to_string();
            assert!(!message.contains("private-detail-sentinel"), "{message}");
            assert!(!message.contains('\n'), "{message}");
            assert!(message.starts_with("wire schema source:"));
        }
    }
}
