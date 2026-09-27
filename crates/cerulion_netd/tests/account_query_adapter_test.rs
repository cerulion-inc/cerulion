// SPDX-License-Identifier: AGPL-3.0-only
//! Stable account routes through existing viewer query verbs, over the real UDS.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use cerulion_core::{CatalogReply, SchemaReply};
use cerulion_netd::account_access::{robot_route, AccountAccessReply, AccountAccessRequest};
use cerulion_netd::egress::NoopEgressPlane;
use cerulion_netd::protocol::DiscoveryState;
use cerulion_netd::query::{CatalogGather, QueryError, QueryPlane, RunsGather, SchemaGather};
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdClient, NetdConfig, TopicKey};

#[derive(Default)]
struct Routes {
    account_calls: AtomicUsize,
    lan_calls: AtomicUsize,
    mode: AtomicUsize,
}

impl MirrorPlane for Routes {
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        panic!("metadata queries cannot demand a topic")
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        panic!("metadata queries cannot hold a mirror")
    }
    fn account_access(&self, action: &AccountAccessRequest) -> Result<AccountAccessReply, String> {
        self.account_calls.fetch_add(1, Ordering::SeqCst);
        let AccountAccessRequest::Catalog { robot_id } = action else {
            panic!("the legacy catalog adapter must issue Catalog")
        };
        assert_eq!(*robot_id, [0xab; 32]);
        match self.mode.load(Ordering::SeqCst) {
            1 => Err("owner observation revoked".into()),
            mode => Ok(AccountAccessReply::Catalog {
                robot_id: if mode == 2 { [0; 32] } else { *robot_id },
                catalog: CatalogReply {
                    version: if mode == 3 { 999 } else { 1 },
                    robot: "display-name".into(),
                    entries: vec![],
                    error: None,
                },
            }),
        }
    }
    fn account_schema_by_type(
        &self,
        robot_id: [u8; 32],
        requested: &str,
    ) -> Result<SchemaReply, String> {
        self.account_calls.fetch_add(1, Ordering::SeqCst);
        assert_eq!(robot_id, [0xab; 32]);
        assert_eq!(requested, "sample/Reading");
        if self.mode.load(Ordering::SeqCst) == 1 {
            return Err("schema observation revoked".into());
        }
        let mut reply = SchemaReply::not_found("display-name", requested, "no exact type match");
        match self.mode.load(Ordering::SeqCst) {
            2 => reply.requested = "other/Reading".into(),
            3 => reply.version = 999,
            _ => {}
        }
        Ok(reply)
    }
}

impl QueryPlane for Routes {
    fn query_catalog(&self, robot: Option<&str>) -> Result<CatalogGather, QueryError> {
        assert_eq!(robot, Some("lan-robot"));
        self.lan_calls.fetch_add(1, Ordering::SeqCst);
        Ok(CatalogGather {
            catalogs: vec![],
            discovery: DiscoveryState::NotConverged,
            unsettled_for: None,
        })
    }
    fn query_schema(&self, _: Option<&str>, _: &str) -> Result<SchemaGather, QueryError> {
        panic!("an account schema must never reach the LAN query plane")
    }
    fn query_runs(&self, _: Option<&str>) -> Result<RunsGather, QueryError> {
        panic!("an unsupported account runs query must never reach the LAN query plane")
    }
}

#[test]
fn stable_account_queries_preserve_refusals_and_never_fall_back_to_lan() {
    let dir = tempfile::tempdir_in("/tmp").unwrap();
    let socket = dir.path().join("netd.sock");
    let routes = Arc::new(Routes::default());
    let mut daemon = cerulion_netd::daemon::start_with_planes(
        socket.clone(),
        routes.clone(),
        Arc::new(NoopEgressPlane),
        routes.clone(),
        NetdConfig::default(),
    )
    .unwrap();
    let mut client = NetdClient::connect_existing_at(socket).unwrap();
    let route = robot_route(&[0xab; 32]);
    let answer = client.query_catalog_with_discovery(Some(&route)).unwrap();
    assert_eq!(answer.discovery, DiscoveryState::Settled);
    assert_eq!(answer.catalogs.len(), 1);
    assert_eq!(answer.catalogs[0].robot, route);
    assert!(answer.catalogs[0].entries.is_empty());
    let schema = client.query_schema(Some(&route), "sample/Reading").unwrap();
    assert_eq!(schema.len(), 1);
    assert_eq!(schema[0].robot, route);
    assert_eq!(schema[0].requested, "sample/Reading");
    assert_eq!(schema[0].error.as_deref(), Some("no exact type match"));
    assert!(schema[0].docs.is_empty());

    for mode in [1, 2, 3] {
        routes.mode.store(mode, Ordering::SeqCst);
        let catalog_error = client.query_catalog(Some(&route)).unwrap_err().to_string();
        let schema_error = client
            .query_schema(Some(&route), "sample/Reading")
            .unwrap_err()
            .to_string();
        if mode == 1 {
            assert!(catalog_error.contains("owner observation revoked"));
            assert!(schema_error.contains("schema observation revoked"));
        } else {
            assert!(catalog_error.contains("did not match its request"));
            assert!(schema_error.contains("did not match its request"));
        }
    }
    assert!(client
        .query_runs(Some(&route))
        .unwrap_err()
        .to_string()
        .contains("live run queries are not supported over account robot access"));
    let calls = routes.account_calls.load(Ordering::SeqCst);
    for malformed in ["account:bad".to_string(), format!(" {route} ")] {
        for error in [
            client.query_catalog(Some(&malformed)).unwrap_err(),
            client
                .query_schema(Some(&malformed), "sample/Reading")
                .unwrap_err(),
            client.query_runs(Some(&malformed)).unwrap_err(),
        ] {
            assert!(error
                .to_string()
                .contains("64 lowercase hexadecimal digits"));
        }
    }
    assert_eq!(routes.account_calls.load(Ordering::SeqCst), calls);
    assert_eq!(routes.lan_calls.load(Ordering::SeqCst), 0);
    let lan = client
        .query_catalog_with_discovery(Some("lan-robot"))
        .unwrap();
    assert_eq!(lan.discovery, DiscoveryState::NotConverged);
    assert_eq!(routes.lan_calls.load(Ordering::SeqCst), 1);
    drop(client);
    daemon.shutdown();
}
