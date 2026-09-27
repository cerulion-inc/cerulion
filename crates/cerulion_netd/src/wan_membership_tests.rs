// SPDX-License-Identifier: AGPL-3.0-only
//! Owned membership snapshots remain fail-closed if the shared lock is poisoned.
use super::*;

fn peer(seed: u8) -> WanRobot {
    let key = cerulion_pairing::client::DeviceIdentity::from_seed(&[seed; 32]).public_key();
    WanRobot {
        eid: EndpointId::from_bytes(&key.0).unwrap(),
        direct_addrs: Vec::new(),
    }
}

#[test]
fn clones_share_membership_but_return_independent_owned_snapshots() {
    let first = peer(10);
    let second = peer(11);
    let registry = WanRegistry::new(
        HashMap::from([("robot".into(), first.clone())]),
        [12; 32],
        RelayConfig::Disabled,
    );
    let clone = registry.clone();
    let held = clone.get("robot").unwrap().unwrap();
    registry
        .replace_membership(HashMap::from([("robot".into(), second.clone())]))
        .unwrap();
    assert_eq!(held, first);
    assert_eq!(clone.get("robot").unwrap(), Some(second));
    assert_eq!(clone.robot_count().unwrap(), 1);
    registry.replace_membership(HashMap::new()).unwrap();
    assert_eq!(clone.robot_count().unwrap(), 0);
    assert!(!clone.is_wan_robot("robot").unwrap());
    assert_eq!(clone.get("robot").unwrap(), None);
}

#[test]
fn poisoned_membership_never_means_absent_or_lan_fallback() {
    let registry = WanRegistry::new(
        HashMap::from([("robot".into(), peer(20))]),
        [21; 32],
        RelayConfig::Disabled,
    );
    let shared = Arc::clone(&registry.robots);
    assert!(std::thread::spawn(move || {
        let _held = shared.write().unwrap();
        panic!("deliberately poison the membership lock");
    })
    .join()
    .is_err());
    assert!(registry.get("robot").is_err());
    assert!(registry.get("missing").is_err());
    assert!(registry.is_wan_robot("robot").is_err());
    assert!(registry.is_wan_robot("missing").is_err());
    assert!(registry.robot_count().is_err());
    assert!(registry.membership_snapshot().is_err());
    assert!(registry.replace_membership(HashMap::new()).is_err());
    assert!(pick_plane(&TopicKey::new("robot", "/state"), &registry).is_err());
    assert!(pick_plane(&TopicKey::new("missing", "/state"), &registry).is_err());
}
