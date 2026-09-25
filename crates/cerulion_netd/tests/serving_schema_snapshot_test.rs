// SPDX-License-Identifier: AGPL-3.0-only
//! Local snapshot dispatch, freshness and wire discrimination. The real gateway
//! and remoted provider have separate transport integration coverage.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::{GatewayPlan, SchemaDoc, SchemaEncoding, SchemaServing};
use cerulion_netd::egress::{EgressError, EgressPlane, NoopEgressPlane};
use cerulion_netd::protocol::{
    parse_request, Request, Response, ServingSchemaSnapshot, ServingSchemaSnapshotResponse,
};
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdConfig, TopicKey};

struct NoNetwork;

impl MirrorPlane for NoNetwork {
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        panic!("a local schema read must not create a mirror")
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        panic!("a local schema read must not hold a mirror")
    }
}

impl QueryPlane for NoNetwork {
    fn query_catalog(&self, _: Option<&str>) -> Result<CatalogGather, QueryError> {
        panic!("a local schema read must not query the network")
    }
    fn query_schema(&self, _: Option<&str>, _: &str) -> Result<SchemaGather, QueryError> {
        panic!("a local schema read must not query the network")
    }
    fn query_runs(&self, _: Option<&str>) -> Result<RunsGather, QueryError> {
        panic!("a local schema read must not query the network")
    }
}

struct LocalMetadata(Mutex<SchemaServing>);

impl EgressPlane for LocalMetadata {
    fn serving_schema_snapshot(&self) -> Result<ServingSchemaSnapshot, String> {
        Ok(ServingSchemaSnapshot {
            schema_serving: self.0.lock().unwrap().clone(),
            ix_config_json: "namespace-oracle".into(),
        })
    }
    fn register_egress(
        &self,
        _: u64,
        _: &GatewayPlan,
        _: &SchemaServing,
        _: Option<&str>,
    ) -> Result<bool, EgressError> {
        panic!("a local schema read must not start egress")
    }
    fn release_egress(&self, _: u64) {
        panic!("a local schema read must not own egress")
    }
}

fn exchange(stream: &mut BufReader<UnixStream>, id: u64) -> (Response, String) {
    writeln!(
        stream.get_mut(),
        "{}",
        Request::ServingSchemaSnapshot { id }.to_json_line()
    )
    .unwrap();
    let mut line = String::new();
    stream.read_line(&mut line).unwrap();
    (serde_json::from_str(&line).unwrap(), line)
}

fn connect(path: &std::path::Path) -> BufReader<UnixStream> {
    let stream = UnixStream::connect(path).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let mut reader = BufReader::new(stream);
    let mut hello = String::new();
    reader.read_line(&mut hello).unwrap();
    let hello: cerulion_netd::protocol::Hello = serde_json::from_str(&hello).unwrap();
    assert!(hello.protocol >= 8);
    reader
}

#[test]
fn local_snapshot_uses_a_distinct_required_key_and_version_floor() {
    let request = Request::ServingSchemaSnapshot { id: 12 };
    assert_eq!(request.id(), 12);
    assert_eq!(request.method_name(), "serving_schema_snapshot");
    assert_eq!(request.min_daemon_version(), 8);
    assert_eq!(parse_request(&request.to_json_line()).unwrap(), request);
    let response = Response::ServingSchemaSnapshot(ServingSchemaSnapshotResponse {
        id: 12,
        serving_schema: ServingSchemaSnapshot {
            schema_serving: SchemaServing::default(),
            ix_config_json: "namespace-oracle".into(),
        },
    });
    assert_eq!(
        serde_json::from_str::<Response>(&response.to_json_line()).unwrap(),
        response
    );
    for invalid in [
        r#"{"id":12}"#,
        r#"{"id":12,"serving_schema":{}}"#,
        r#"{"id":12,"serving_schema":{"schema_serving":{}}}"#,
    ] {
        assert!(serde_json::from_str::<Response>(invalid).is_err());
    }
}

#[test]
fn local_snapshot_is_fresh_without_demand_egress_or_network_query() {
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let path = directory.path().join("netd.sock");
    let metadata = Arc::new(LocalMetadata(Mutex::new(SchemaServing::default())));
    let mut daemon = cerulion_netd::daemon::start_with_planes(
        path.clone(),
        Arc::new(NoNetwork),
        metadata.clone(),
        Arc::new(NoNetwork),
        NetdConfig::default(),
    )
    .unwrap();
    let mut connection = connect(&path);
    let (before, _) = exchange(&mut connection, 1);
    let Response::ServingSchemaSnapshot(before) = before else {
        panic!("expected the local snapshot")
    };
    assert!(before.serving_schema.schema_serving.schema_docs.is_empty());
    metadata.0.lock().unwrap().schema_docs.push(SchemaDoc {
        qualified: "test/Reading".into(),
        encoding: SchemaEncoding::Msg,
        text: "uint32 count\n".into(),
        deps: vec![],
    });
    let (after, first_bytes) = exchange(&mut connection, 2);
    let (_, second_bytes) = exchange(&mut connection, 2);
    assert_eq!(
        first_bytes, second_bytes,
        "unchanged metadata serializes identically"
    );
    let Response::ServingSchemaSnapshot(after) = after else {
        panic!("expected the updated local snapshot")
    };
    assert_eq!(after.id, 2);
    assert_eq!(after.serving_schema.ix_config_json, "namespace-oracle");
    assert_eq!(after.serving_schema.schema_serving.schema_docs.len(), 1);
    assert_eq!(
        after.serving_schema.schema_serving.schema_docs[0].text,
        "uint32 count\n"
    );
    drop(connection);
    daemon.shutdown();
}

#[test]
fn absent_provider_is_a_correlated_refusal_and_connection_stays_usable() {
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let path = directory.path().join("netd.sock");
    let mut daemon = cerulion_netd::daemon::start_with_egress(
        path.clone(),
        Arc::new(NoNetwork),
        Arc::new(NoopEgressPlane),
        NetdConfig::default(),
    )
    .unwrap();
    let mut connection = connect(&path);
    for id in [1, 2] {
        let (reply, _) = exchange(&mut connection, id);
        let Response::Error(reply) = reply else {
            panic!("no provider must not claim an empty snapshot")
        };
        assert_eq!(reply.id, Some(id));
        assert_eq!(
            reply.error,
            "this daemon has no local serving-schema provider"
        );
    }
    drop(connection);
    daemon.shutdown();
}
