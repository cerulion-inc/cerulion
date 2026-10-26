// SPDX-License-Identifier: AGPL-3.0-only
//! Which plane carried a mirror's frames, asserted rather than assumed.
//!
//! A robot on the same local network is reachable over BOTH planes, and the
//! frames are identical either way: both planes re-inject into local shared
//! memory under the same topic name, with the same wire bytes. So a test that
//! demands a topic and checks the frames arrived passes just as happily when the
//! local plane carried every byte and the internet plane carried none. That is
//! not a hypothetical: a serving machine opens the local plane on every
//! interface it has, so a desk that learns one locator gets the topic locally
//! and the internet path is never exercised.
//!
//! `MirrorPlane::serving_plane` is the answer, and `status` reports it per
//! demand row. These arms pin it against the delivery it describes.

use super::*;
use cerulion_netd::protocol::ServingPlane;

/// The headline: frames really crossed the internet plane, and the observable the
/// daemon reports says so.
///
/// Three facts in one body, because each alone is passable by a wrong
/// implementation. The frames match a hand oracle (something was delivered); the
/// plane holds exactly one connection and one reader for the key (this plane did
/// the delivering, not a bystander); and `serving_plane` reads `iroh` through the
/// same trait method the status handler calls.
#[test]
fn the_internet_plane_reports_itself_and_holds_the_reader_that_delivered() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("plane_iroh", authz);
    let topic = format!("/wan/plane/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("plane_iroh_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let iroh = IrohMirrorPlane::new(manager_b.clone(), Arc::clone(&registry))
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);

    let key = TopicKey::new("ubuntu", &topic);
    iroh.ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("the robot is registered and reachable on loopback");

    let collected = collect_frames(&manager_b, &topic, 5);
    assert_oracle_and_gap_free(&collected);

    // The delivering plane holds the connection and the reader for this key. A
    // mirror served by anything else would leave both at zero.
    assert_eq!(
        iroh.connection_count(),
        1,
        "the frames arrived over one dialed connection"
    );
    assert_eq!(iroh.reader_count(), 1, "one reader re-injected them");
    assert!(iroh.has_reader(&key).expect("reader state"));

    // The observable, read through the trait method the status handler calls.
    let plane: &dyn MirrorPlane = &iroh;
    assert_eq!(
        plane.serving_plane(&key),
        Some(ServingPlane::Iroh),
        "the plane that delivered must report itself"
    );

    assert_eq!(iroh.release_mirror(&key), MirrorRelease::Retired);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

/// The same claim through the composing plane the daemon actually installs, plus
/// the control that makes it mean something.
///
/// The desk manager here has no network configuration at all, so the local plane
/// cannot register an ingress mirror for anything. A delivered topic on this desk
/// therefore cannot have been carried locally, which is what turns "status says
/// iroh" from a label into evidence. The unregistered robot is the other half: the
/// same observable answers `zenoh` for a key that routes locally, so a stuck
/// implementation that always says `iroh` fails here.
#[test]
fn the_composed_plane_attributes_a_delivered_topic_to_the_internet_and_a_lan_name_to_local() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("plane_dual", authz);
    let topic = format!("/wan/plane/dual/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("plane_dual_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let iroh = IrohMirrorPlane::new(manager_b.clone(), Arc::clone(&registry))
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let zenoh = GatewayMirrorPlane::new(manager_b.clone());
    let dual = DualMirrorPlane::new(zenoh, iroh, Arc::clone(&registry));
    let plane: &dyn MirrorPlane = &dual;

    // The control, on a topic nothing else touches: this desk's local plane cannot
    // mirror at all, so nothing below can have arrived over it.
    let local_probe = TopicKey::new("lan-bot", format!("/wan/plane/probe/{}", unique_id()));
    let refusal = GatewayMirrorPlane::new(manager_b.clone())
        .ensure_mirror(&local_probe, ORACLE_SCHEMA_HASH)
        .expect_err("a network-less desk cannot register a local mirror");
    assert!(
        matches!(refusal, MirrorError::Register { .. }),
        "expected the local registration to be refused, got {refusal:?}"
    );

    let key = TopicKey::new("ubuntu", &topic);
    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("the composed plane routes the registered robot to the internet plane");
    let collected = collect_frames(&manager_b, &topic, 5);
    assert_oracle_and_gap_free(&collected);

    assert_eq!(
        plane.serving_plane(&key),
        Some(ServingPlane::Iroh),
        "a topic delivered on a desk whose local plane cannot mirror must read as \
         the internet plane"
    );
    assert_eq!(
        plane.serving_plane(&local_probe),
        Some(ServingPlane::Zenoh),
        "an unregistered robot's key reads as local, so the answer is not a constant"
    );

    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

/// The local plane is a single transport, so it attributes every key to itself,
/// including a robot it has never mirrored. The composed plane is the only one
/// that needs a route to answer, and this pins that the lean daemon's answer is
/// not accidentally the unknown one.
#[test]
fn a_lan_only_daemon_attributes_its_keys_to_the_local_plane() {
    let manager = test_manager("plane_lan_only");
    let zenoh = GatewayMirrorPlane::new(manager);
    let plane: &dyn MirrorPlane = &zenoh;
    assert_eq!(
        plane.serving_plane(&TopicKey::new("ubuntu", "/tf")),
        Some(ServingPlane::Zenoh)
    );
}
