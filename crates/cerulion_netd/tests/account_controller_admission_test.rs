// SPDX-License-Identifier: AGPL-3.0-only
//! Real UDS dispatch oracles; the spy records admission and retirement ordering.
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use cerulion_netd::registry::DemandRegistry;
use cerulion_netd::{MirrorError, MirrorPlane, MirrorRelease, NetdClient, NetdConfig, TopicKey};

#[derive(Default)]
struct AdmissionPlane {
    deny: AtomicBool,
    retire: AtomicBool,
    prepared: AtomicUsize,
    calls: Mutex<Vec<&'static str>>,
}
impl MirrorPlane for AdmissionPlane {
    fn prepare_demand(&self, _: &TopicKey, _: &mut DemandRegistry) -> Result<(), String> {
        self.prepared.fetch_add(1, Ordering::SeqCst);
        if self.deny.load(Ordering::SeqCst) {
            Err("identity no longer matches".into())
        } else {
            Ok(())
        }
    }
    fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
        self.calls.lock().unwrap().push("ensure");
        Ok(())
    }
    fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
        self.calls.lock().unwrap().push("release");
        if self.retire.load(Ordering::SeqCst) {
            MirrorRelease::Retired
        } else {
            MirrorRelease::Lingering
        }
    }
    fn mirror_retired(&self, _: &TopicKey) {
        self.calls.lock().unwrap().push("retired");
    }
}

#[test]
fn padded_reserved_routes_refuse_before_trimming_or_mirror_dispatch() {
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let socket = directory.path().join("netd.sock");
    let plane = Arc::new(AdmissionPlane::default());
    let mut daemon =
        cerulion_netd::start(socket.clone(), plane.clone(), NetdConfig::default()).unwrap();
    let mut client = NetdClient::connect_existing_at(socket).unwrap();
    let route = cerulion_netd::account_access::robot_route(&[5; 32]);
    for padded in [
        format!(" {route}"),
        format!("{route} "),
        format!("\t{route}"),
    ] {
        assert!(client.demand(&padded, "/state", 1).is_err());
        assert!(client.release(&padded, "/state").is_err());
    }
    assert_eq!(plane.prepared.load(Ordering::SeqCst), 0);
    assert!(plane.calls.lock().unwrap().is_empty());
    assert_eq!(daemon.active_demand_count(), 0);
    assert!(client.demand(" lan-robot ", "/state", 1).is_ok());
    assert!(client.release(" lan-robot ", "/state").is_ok());
    drop(client);
    daemon.shutdown();
}

#[test]
fn every_held_mirror_admission_rechecks_identity_and_pin_retires_after_transport() {
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let socket = directory.path().join("netd.sock");
    let plane = Arc::new(AdmissionPlane::default());
    let mut daemon =
        cerulion_netd::start(socket.clone(), plane.clone(), NetdConfig::default()).unwrap();
    let mut first = NetdClient::connect_existing_at(socket.clone()).unwrap();
    let mut second = NetdClient::connect_existing_at(socket).unwrap();
    first.demand("robot", "/state", 1).unwrap();
    plane.deny.store(true, Ordering::SeqCst);
    assert!(second
        .demand("robot", "/state", 1)
        .unwrap_err()
        .to_string()
        .contains("identity no longer matches"));
    assert_eq!(plane.prepared.load(Ordering::SeqCst), 2);
    assert_eq!(daemon.active_demand_count(), 1);
    assert_eq!(*plane.calls.lock().unwrap(), ["ensure"]);
    first.release("robot", "/state").unwrap();
    assert_eq!(
        *plane.calls.lock().unwrap(),
        ["ensure", "release"],
        "lingering transport keeps its route pin"
    );
    plane.deny.store(false, Ordering::SeqCst);
    second.demand("robot", "/state", 1).unwrap();
    plane.retire.store(true, Ordering::SeqCst);
    second.release("robot", "/state").unwrap();
    assert_eq!(
        *plane.calls.lock().unwrap(),
        ["ensure", "release", "release", "retired"]
    );
    assert_eq!(daemon.active_demand_count(), 0);
    assert_eq!(daemon.lingering_count(), 0);
    drop(first);
    drop(second);
    daemon.shutdown();
}
