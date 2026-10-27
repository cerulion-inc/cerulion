// SPDX-License-Identifier: MIT OR Apache-2.0
//! Team credentials on disk: `identity/active_team`, `identity/<team>/leaf.der`
//! and `identity/<team>/bundle`, written by `IdentityLayout` and accepted by
//! `load_team_credentials` only when the bundle chains to a given root set,
//! the leaf is a leaf of the bundle's current generation, and the leaf
//! certifies this machine's key. With feature `dev-issuer`, the development
//! issuer enrolls a machine end to end and a second machine joins by import.

mod common;

use cerulion_pairing::format::{PublicKey, RootSet};
use cerulion_pairing::identity::{IdentityLayout, TeamDirName};
use cerulion_pairing::team::*;
use cerulion_pairing::PairingError;
use common::{make_intermediate, pk, sk, ISSUED, T_NOW};
use ed25519_dalek::SigningKey;

fn acme() -> TeamShort {
    TeamShort::new("acme").unwrap()
}

fn ca_key(generation: u32) -> SigningKey {
    sk(0x40 + generation as u8)
}

fn ca_der(generation: u32) -> Vec<u8> {
    issue_team_ca(&acme(), generation, &ca_key(generation), ISSUED).unwrap()
}

/// A root set, and a bundle for acme at generation 2 signed through it.
fn roots_and_bundle(root_seed: u8) -> (RootSet, SignedTeamTrustBundle) {
    let root = sk(root_seed);
    let root_set = RootSet::new(vec![pk(&root)], 1).unwrap();
    let int = sk(root_seed.wrapping_add(1));
    let signed_int = make_intermediate(
        pk(&int),
        cerulion_pairing::format::Validity {
            not_before_ns: 0,
            not_after_ns: u64::MAX,
        },
        ISSUED,
    )
    .sign_by_roots(&[&root]);
    let bundle = TeamTrustBundle {
        version: 1,
        team_id: TeamId([0x7E; 32]),
        team_short: acme(),
        generation: 2,
        issued_at_ns: ISSUED,
        issuing_ca_der: ca_der(2),
        next_ca_der: ca_der(3),
        retired: vec![RetiredGeneration {
            generation: 1,
            ca_der: ca_der(1),
        }],
        robot_leaves: vec![],
        issuer_key: pk(&int),
    };
    (root_set, bundle.sign(&int, signed_int))
}

fn leaf_for(layout: &IdentityLayout, role: TeamRole, generation: u32) -> Vec<u8> {
    let (key, _) = layout.load_or_create_machine_key().unwrap();
    issue_leaf(
        &LeafParams {
            team: acme(),
            generation,
            role,
            device_key: PublicKey(key.public_key()),
            account: None,
            issued_at_ns: T_NOW,
        },
        &ca_key(generation),
    )
    .unwrap()
}

fn team() -> TeamDirName {
    TeamDirName::new("acme").unwrap()
}

fn enrolled() -> (tempfile::TempDir, IdentityLayout, RootSet) {
    let dir = tempfile::tempdir().unwrap();
    let layout = IdentityLayout::at(dir.path());
    let (roots, bundle) = roots_and_bundle(0x10);
    let leaf = leaf_for(&layout, TeamRole::Operator, 2);
    layout
        .write_team_credentials(&team(), &leaf, &bundle)
        .unwrap();
    layout.set_active_team(&team()).unwrap();
    (dir, layout, roots)
}

fn reason(err: PairingError) -> String {
    match err {
        PairingError::TeamCredentials { path, reason } => format!("{} {reason}", path.display()),
        other => panic!("expected TeamCredentials, got {other:?}"),
    }
}

#[test]
fn written_credentials_load_back_checked() {
    let (_dir, layout, roots) = enrolled();
    let creds = layout
        .load_team_credentials(&[roots], T_NOW)
        .unwrap()
        .expect("an active team");
    assert_eq!(creds.team, team());
    assert_eq!(creds.leaf.role, TeamRole::Operator);
    assert_eq!(creds.leaf.generation, 2);
    assert_eq!(creds.bundle.get().bundle.next_ca_der, ca_der(3));
    assert_eq!(
        std::fs::read(layout.leaf_path(&team())).unwrap(),
        creds.leaf_der
    );
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        for p in [
            layout.leaf_path(&team()),
            layout.bundle_path(&team()),
            layout.active_team_path(),
        ] {
            let mode = std::fs::metadata(&p).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600, "{}", p.display());
        }
        let mode = std::fs::metadata(layout.team_dir(&team()))
            .unwrap()
            .permissions()
            .mode();
        assert_eq!(mode & 0o777, 0o700);
    }
}

#[test]
fn no_active_team_is_none() {
    let dir = tempfile::tempdir().unwrap();
    let layout = IdentityLayout::at(dir.path());
    let (roots, _) = roots_and_bundle(0x10);
    assert!(layout
        .load_team_credentials(&[roots], T_NOW)
        .unwrap()
        .is_none());
}

#[test]
fn a_bundle_from_another_root_set_is_refused() {
    let (_dir, layout, _) = enrolled();
    let (other_roots, _) = roots_and_bundle(0x20);
    let err = reason(
        layout
            .load_team_credentials(&[other_roots], T_NOW)
            .unwrap_err(),
    );
    assert!(
        err.contains("does not verify against a trusted root set"),
        "{err}"
    );
    let err = reason(layout.load_team_credentials(&[], T_NOW).unwrap_err());
    assert!(err.contains("trusts no root set"), "{err}");
}

#[test]
fn a_leaf_of_another_generation_or_machine_is_refused() {
    let (_dir, layout, roots) = enrolled();
    let (_, bundle) = roots_and_bundle(0x10);
    let old = leaf_for(&layout, TeamRole::Operator, 1);
    layout
        .write_team_credentials(&team(), &old, &bundle)
        .unwrap();
    let err = reason(
        layout
            .load_team_credentials(std::slice::from_ref(&roots), T_NOW)
            .unwrap_err(),
    );
    assert!(
        err.contains("leaf.der") && err.contains("current generation"),
        "{err}"
    );

    let other = tempfile::tempdir().unwrap();
    let foreign = leaf_for(&IdentityLayout::at(other.path()), TeamRole::Operator, 2);
    layout
        .write_team_credentials(&team(), &foreign, &bundle)
        .unwrap();
    let err = reason(layout.load_team_credentials(&[roots], T_NOW).unwrap_err());
    assert!(err.contains("another machine's key"), "{err}");
}

#[test]
fn missing_or_garbled_files_name_the_file() {
    let (_dir, layout, roots) = enrolled();
    std::fs::remove_file(layout.leaf_path(&team())).unwrap();
    let err = reason(
        layout
            .load_team_credentials(std::slice::from_ref(&roots), T_NOW)
            .unwrap_err(),
    );
    assert!(err.contains("leaf.der") && err.contains("missing"), "{err}");

    std::fs::write(layout.bundle_path(&team()), b"not a bundle").unwrap();
    let err = reason(
        layout
            .load_team_credentials(std::slice::from_ref(&roots), T_NOW)
            .unwrap_err(),
    );
    assert!(
        err.contains("bundle") && err.contains("does not decode"),
        "{err}"
    );

    std::fs::write(layout.active_team_path(), "Not A Label\n").unwrap();
    assert!(matches!(
        layout.load_team_credentials(&[roots], T_NOW),
        Err(PairingError::InvalidTeamName(_))
    ));
}

#[test]
fn a_bundle_for_another_team_than_the_active_one_is_refused() {
    let (_dir, layout, roots) = enrolled();
    let beta = TeamDirName::new("beta").unwrap();
    let (_, bundle) = roots_and_bundle(0x10);
    let leaf = std::fs::read(layout.leaf_path(&team())).unwrap();
    layout
        .write_team_credentials(&beta, &leaf, &bundle)
        .unwrap();
    layout.set_active_team(&beta).unwrap();
    let err = reason(layout.load_team_credentials(&[roots], T_NOW).unwrap_err());
    assert!(
        err.contains("names team 'acme', not the active team 'beta'"),
        "{err}"
    );
}

#[test]
fn a_build_without_the_feature_trusts_no_root_set() {
    assert!(release_root_sets().is_empty());
    let dir = tempfile::tempdir().unwrap();
    let layout = IdentityLayout::at(dir.path());
    let state = dir.path().join("dev-issuer");
    std::fs::create_dir_all(&state).unwrap();
    std::fs::write(state.join("state"), b"any bytes").unwrap();
    if DEV_ISSUER_COMPILED {
        assert!(
            trusted_root_sets(&layout).is_err(),
            "a garbled state file is refused"
        );
    } else {
        assert_eq!(
            trusted_root_sets(&layout).unwrap(),
            Vec::<RootSet>::new(),
            "a state file on disk adds no root set without the feature"
        );
    }
}

#[test]
fn an_expired_desk_leaf_is_refused_and_a_robot_leaf_never_is() {
    let (_dir, layout, roots) = enrolled();
    let creds = layout
        .load_team_credentials(std::slice::from_ref(&roots), T_NOW)
        .unwrap()
        .unwrap();
    let after = (creds.leaf.not_after_unix_secs + 1) * 1_000_000_000;
    let err = reason(
        layout
            .load_team_credentials(std::slice::from_ref(&roots), after)
            .unwrap_err(),
    );
    assert!(err.contains("expired at"), "{err}");

    let (_, bundle) = roots_and_bundle(0x10);
    let robot = leaf_for(&layout, TeamRole::Robot, 2);
    layout
        .write_team_credentials(&team(), &robot, &bundle)
        .unwrap();
    assert!(layout
        .load_team_credentials(&[roots], u64::MAX - 1)
        .unwrap()
        .is_some());
}

#[test]
fn a_leaf_signed_by_a_key_that_is_not_the_ca_is_refused() {
    let (_dir, layout, roots) = enrolled();
    let (_, bundle) = roots_and_bundle(0x10);
    let (key, _) = layout.load_or_create_machine_key().unwrap();
    let forged = issue_leaf(
        &LeafParams {
            team: acme(),
            generation: 2,
            role: TeamRole::Owner,
            device_key: PublicKey(key.public_key()),
            account: None,
            issued_at_ns: T_NOW,
        },
        &sk(0x99),
    )
    .unwrap();
    layout
        .write_team_credentials(&team(), &forged, &bundle)
        .unwrap();
    let err = reason(layout.load_team_credentials(&[roots], T_NOW).unwrap_err());
    assert!(
        err.contains("leaf.der") && err.contains("current generation"),
        "{err}"
    );
}

#[cfg(feature = "dev-issuer")]
mod dev {
    use super::*;
    use cerulion_pairing::team::dev_issuer::{
        dev_issuer_dir, DevIssuer, DEV_ISSUER_STATE_FILENAME,
    };

    fn now_ns() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    #[test]
    fn a_dev_issuer_enrolls_two_machines_into_one_team() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        let (la, lb) = (IdentityLayout::at(a.path()), IdentityLayout::at(b.path()));
        let now = now_ns();
        let issuer = DevIssuer::create(&la, TeamShort::new("lab").unwrap(), now).unwrap();
        issuer.enroll(&la, TeamRole::Robot, now).unwrap();
        let state = dev_issuer_dir(&la).join(DEV_ISSUER_STATE_FILENAME);
        let joined = DevIssuer::import(&lb, &state).unwrap();
        joined.enroll(&lb, TeamRole::Viewer, now).unwrap();

        let ca = trusted_root_sets(&la).unwrap();
        let cb = trusted_root_sets(&lb).unwrap();
        assert_eq!(ca, cb, "both machines trust the same development root set");
        let robot = la.load_team_credentials(&ca, now).unwrap().unwrap();
        let viewer = lb.load_team_credentials(&cb, now).unwrap().unwrap();
        assert_eq!(robot.leaf.role, TeamRole::Robot);
        assert_eq!(viewer.leaf.role, TeamRole::Viewer);
        assert_eq!(
            robot.bundle.get().bundle.issuing_ca_der,
            viewer.bundle.get().bundle.issuing_ca_der,
            "one issuing CA"
        );
        assert_eq!(robot.bundle.get().bundle.generation + 1, {
            let next = check_team_ca(&robot.bundle.get().bundle.next_ca_der).unwrap();
            next.generation
        });
        assert_ne!(robot.key.public_key(), viewer.key.public_key());

        // A machine of another development team does not verify here.
        let c = tempfile::tempdir().unwrap();
        let lc = IdentityLayout::at(c.path());
        DevIssuer::create(&lc, TeamShort::new("lab").unwrap(), now)
            .unwrap()
            .enroll(&lc, TeamRole::Owner, now)
            .unwrap();
        let err = reason(lc.load_team_credentials(&ca, now).unwrap_err());
        assert!(
            err.contains("does not verify against a trusted root set"),
            "{err}"
        );
        let own = trusted_root_sets(&lc).unwrap();
        assert!(
            lc.load_team_credentials(&own, now).unwrap().is_some(),
            "positive control"
        );

        // Creating over an existing state file is refused, never replaced.
        let before = std::fs::read(&state).unwrap();
        match DevIssuer::create(&la, TeamShort::new("lab").unwrap(), now) {
            Err(PairingError::IdentityIo { source, .. }) => {
                assert_eq!(source.kind(), std::io::ErrorKind::AlreadyExists)
            }
            other => panic!("expected AlreadyExists, got {other:?}"),
        }
        assert_eq!(std::fs::read(&state).unwrap(), before);

        // An import that does not decode writes nothing.
        let d = tempfile::tempdir().unwrap();
        let ld = IdentityLayout::at(d.path());
        let junk = d.path().join("junk");
        std::fs::write(&junk, b"junk").unwrap();
        assert!(DevIssuer::import(&ld, &junk).is_err());
        assert!(!dev_issuer_dir(&ld).join(DEV_ISSUER_STATE_FILENAME).exists());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&state).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600);
        }
    }

    #[cfg(unix)]
    #[test]
    fn a_loose_state_file_is_restricted_and_a_symlink_refused() {
        use std::os::unix::fs::PermissionsExt;
        let a = tempfile::tempdir().unwrap();
        let la = IdentityLayout::at(a.path());
        DevIssuer::create(&la, TeamShort::new("lab").unwrap(), now_ns()).unwrap();
        let state = dev_issuer_dir(&la).join(DEV_ISSUER_STATE_FILENAME);
        std::fs::set_permissions(&state, std::fs::Permissions::from_mode(0o644)).unwrap();
        assert!(DevIssuer::load(&la).unwrap().is_some());
        let mode = std::fs::metadata(&state).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600);

        let b = tempfile::tempdir().unwrap();
        let lb = IdentityLayout::at(b.path());
        std::fs::create_dir_all(dev_issuer_dir(&lb)).unwrap();
        std::os::unix::fs::symlink(&state, dev_issuer_dir(&lb).join(DEV_ISSUER_STATE_FILENAME))
            .unwrap();
        let err = reason(DevIssuer::load(&lb).unwrap_err());
        assert!(err.contains("symbolic link"), "{err}");
    }

    #[test]
    fn a_machine_whose_clock_is_behind_still_joins() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        let (la, lb) = (IdentityLayout::at(a.path()), IdentityLayout::at(b.path()));
        let now = now_ns();
        DevIssuer::create(&la, TeamShort::new("lab").unwrap(), now).unwrap();
        let state = dev_issuer_dir(&la).join(DEV_ISSUER_STATE_FILENAME);
        let behind = now - 3600 * 1_000_000_000;
        DevIssuer::import(&lb, &state)
            .unwrap()
            .enroll(&lb, TeamRole::Robot, behind)
            .unwrap();
        let roots = trusted_root_sets(&lb).unwrap();
        assert!(lb.load_team_credentials(&roots, behind).unwrap().is_some());
        // Its clock moves back further after enrollment: the credentials load.
        let later_behind = behind - 24 * 3600 * 1_000_000_000;
        assert!(lb
            .load_team_credentials(&roots, later_behind)
            .unwrap()
            .is_some());
    }

    #[test]
    fn debug_of_the_issuer_names_no_key() {
        let a = tempfile::tempdir().unwrap();
        let la = IdentityLayout::at(a.path());
        let issuer = DevIssuer::create(&la, TeamShort::new("lab").unwrap(), now_ns()).unwrap();
        let text = format!("{issuer:?}");
        let root = issuer.root_set().keys[0].0;
        let hex: String = root.iter().map(|b| format!("{b:02x}")).collect();
        assert!(text.contains("lab") && !text.contains("seed"), "{text}");
        assert!(!text.contains(&hex), "{text}");
    }
}
