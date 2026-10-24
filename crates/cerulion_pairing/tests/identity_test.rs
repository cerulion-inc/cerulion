// SPDX-License-Identifier: MIT OR Apache-2.0
//! The machine identity on disk: the config directory rule, the identity
//! layout, the one machine key (canonical file, legacy `desk.key` alias,
//! adoption and conflict refusal), and the MAC'd high-water file.
//!
//! Oracles are hand-written: path strings, the high-water byte layout, and the
//! MAC recomputed here from the documented derivation with `hmac` and `sha2`
//! directly, never through the module under test.

use std::ffi::OsStr;
use std::path::{Path, PathBuf};

use cerulion_pairing::identity::{
    config_dir_from, decode_high_water, encode_high_water, load_or_create_key_file, read_key_file,
    HighWater, IdentityLayout, KeyOrigin, MachineKey, TeamDirName, HIGH_WATER_LEN,
};
use cerulion_pairing::PairingError;
use hmac::{Hmac, Mac};
use sha2::{Digest, Sha256};

const SEED_A: [u8; 32] = [7u8; 32];
const SEED_B: [u8; 32] = [9u8; 32];

fn write_raw(path: &Path, bytes: &[u8]) {
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path, bytes).unwrap();
}

fn layout(dir: &tempfile::TempDir) -> IdentityLayout {
    IdentityLayout::at(dir.path().join("home"))
}

fn team(name: &str) -> TeamDirName {
    TeamDirName::new(name).unwrap()
}

#[cfg(unix)]
fn mode(path: &Path) -> u32 {
    use std::os::unix::fs::PermissionsExt;
    std::fs::metadata(path).unwrap().permissions().mode() & 0o777
}

// ---------------------------------------------------------------------------
// Config directory and layout
// ---------------------------------------------------------------------------

#[test]
fn cerulion_home_is_used_verbatim_when_set() {
    assert_eq!(
        config_dir_from(Some(OsStr::new("/srv/rig")), Some(Path::new("/home/u"))),
        Some(PathBuf::from("/srv/rig"))
    );
}

#[test]
fn an_empty_cerulion_home_falls_back_to_the_home_directory() {
    assert_eq!(
        config_dir_from(Some(OsStr::new("")), Some(Path::new("/home/u"))),
        Some(PathBuf::from("/home/u/.cerulion"))
    );
    assert_eq!(
        config_dir_from(None, Some(Path::new("/home/u"))),
        Some(PathBuf::from("/home/u/.cerulion"))
    );
}

#[test]
fn no_override_and_no_home_resolves_nothing() {
    assert_eq!(config_dir_from(None, None), None);
    assert_eq!(config_dir_from(Some(OsStr::new("")), None), None);
}

#[test]
fn the_layout_names_every_identity_path() {
    let l = IdentityLayout::at("/c");
    let t = team("acme");
    assert_eq!(l.config_dir(), Path::new("/c"));
    assert_eq!(l.identity_dir(), PathBuf::from("/c/identity"));
    assert_eq!(l.device_key_path(), PathBuf::from("/c/identity/device.key"));
    assert_eq!(l.legacy_desk_key_path(), PathBuf::from("/c/desk.key"));
    assert_eq!(l.team_dir(&t), PathBuf::from("/c/identity/acme"));
    assert_eq!(
        l.high_water_path(&t),
        PathBuf::from("/c/identity/acme/high_water")
    );
}

#[test]
fn team_dir_names_are_one_dns_label() {
    for ok in ["a", "acme", "a-b1", "0team", &"x".repeat(63)] {
        assert!(TeamDirName::new(ok).is_ok(), "{ok:?} must be accepted");
        assert_eq!(TeamDirName::new(ok).unwrap().as_str(), ok);
    }
    for bad in [
        "",
        "-a",
        "a-",
        "Acme",
        "a/b",
        "..",
        ".",
        "a.b",
        "a b",
        "a_b",
        "\u{e9}",
        &"x".repeat(64),
    ] {
        match TeamDirName::new(bad) {
            Err(PairingError::InvalidTeamName(n)) => assert_eq!(n, bad),
            other => panic!("{bad:?} must be refused as InvalidTeamName, got {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// The machine key
// ---------------------------------------------------------------------------

#[test]
fn a_fresh_machine_gets_one_key_at_the_canonical_path_and_the_legacy_alias() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let (key, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Created);
    let canonical = std::fs::read(l.device_key_path()).unwrap();
    assert_eq!(canonical.as_slice(), key.seed());
    assert_eq!(std::fs::read(l.legacy_desk_key_path()).unwrap(), canonical);
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        assert_eq!(mode(&l.device_key_path()), 0o600);
        assert_eq!(mode(&l.identity_dir()), 0o700);
        assert_eq!(mode(l.config_dir()), 0o700);
        let a = std::fs::metadata(l.device_key_path()).unwrap();
        let b = std::fs::metadata(l.legacy_desk_key_path()).unwrap();
        assert_eq!(
            a.ino(),
            b.ino(),
            "the legacy name is a hard link to the one key"
        );
    }
}

#[test]
fn a_second_load_returns_the_same_key() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let (first, _) = l.load_or_create_machine_key().unwrap();
    let (second, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Existing);
    assert_eq!(first.seed(), second.seed());
}

#[test]
fn an_existing_desk_key_is_adopted_byte_for_byte() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.legacy_desk_key_path(), &SEED_A);
    let (key, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::AdoptedLegacy);
    assert_eq!(key.seed(), &SEED_A);
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), SEED_A);
    assert_eq!(std::fs::read(l.legacy_desk_key_path()).unwrap(), SEED_A);
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        assert_eq!(
            std::fs::metadata(l.device_key_path()).unwrap().ino(),
            std::fs::metadata(l.legacy_desk_key_path()).unwrap().ino(),
            "adoption links the existing file"
        );
    }
}

#[test]
fn leftover_temporary_files_from_a_crashed_writer_do_not_block_creation() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    std::fs::create_dir_all(l.identity_dir()).unwrap();
    let pid = std::process::id();
    for n in 0..8 {
        std::fs::write(
            l.identity_dir().join(format!(".device.key.new.{pid}.{n}")),
            b"partial",
        )
        .unwrap();
    }
    let (key, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Created);
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), key.seed());
}

#[cfg(unix)]
#[test]
fn a_symlinked_desk_key_is_adopted_as_a_copy_that_still_resolves() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.config_dir().join("keys").join("real.key"), &SEED_A);
    std::os::unix::fs::symlink("keys/real.key", l.legacy_desk_key_path()).unwrap();
    let (key, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::AdoptedLegacy);
    assert_eq!(key.seed(), &SEED_A);
    assert!(!std::fs::symlink_metadata(l.device_key_path())
        .unwrap()
        .file_type()
        .is_symlink());
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), SEED_A);
    assert_eq!(mode(&l.device_key_path()), 0o600);
    let (again, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Existing);
    assert_eq!(again.seed(), &SEED_A);
}

#[test]
fn equal_canonical_and_legacy_files_are_one_key() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.device_key_path(), &SEED_A);
    write_raw(&l.legacy_desk_key_path(), &SEED_A);
    let (key, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Existing);
    assert_eq!(key.seed(), &SEED_A);
}

#[test]
fn different_canonical_and_legacy_keys_are_refused_and_left_alone() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.device_key_path(), &SEED_A);
    write_raw(&l.legacy_desk_key_path(), &SEED_B);
    for result in [
        l.load_or_create_machine_key().map(|(k, _)| k),
        l.load_machine_key().map(|k| k.unwrap()),
    ] {
        match result {
            Err(PairingError::MachineKeyConflict { canonical, legacy }) => {
                assert_eq!(canonical, l.device_key_path());
                assert_eq!(legacy, l.legacy_desk_key_path());
            }
            other => panic!("expected MachineKeyConflict, got {other:?}"),
        }
    }
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), SEED_A);
    assert_eq!(std::fs::read(l.legacy_desk_key_path()).unwrap(), SEED_B);
}

#[test]
fn a_missing_legacy_alias_is_recreated_from_the_canonical_key() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.device_key_path(), &SEED_A);
    let (_, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::Existing);
    assert_eq!(std::fs::read(l.legacy_desk_key_path()).unwrap(), SEED_A);
}

#[test]
fn a_wrong_size_key_is_refused_and_never_regenerated() {
    for which in ["canonical", "legacy"] {
        let dir = tempfile::tempdir().unwrap();
        let l = layout(&dir);
        let path = if which == "canonical" {
            l.device_key_path()
        } else {
            l.legacy_desk_key_path()
        };
        write_raw(&path, &[1u8; 31]);
        match l.load_or_create_machine_key() {
            Err(PairingError::MachineKeyMalformed { path: p, len }) => {
                assert_eq!(p, path);
                assert_eq!(len, 31);
            }
            other => panic!("{which}: expected MachineKeyMalformed, got {other:?}"),
        }
        assert_eq!(
            std::fs::read(&path).unwrap(),
            vec![1u8; 31],
            "{which} untouched"
        );
    }
}

#[test]
fn a_read_only_load_creates_nothing_on_an_empty_machine() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    assert!(l.load_machine_key().unwrap().is_none());
    assert!(
        !l.config_dir().exists(),
        "a read-only load must not create the config dir"
    );
}

#[test]
fn a_read_only_load_adopts_a_legacy_key() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.legacy_desk_key_path(), &SEED_B);
    let key = l.load_machine_key().unwrap().unwrap();
    assert_eq!(key.seed(), &SEED_B);
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), SEED_B);
}

#[test]
fn an_explicit_key_file_is_created_private_once_and_reused() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keys").join("desk.key");
    let (first, origin) = load_or_create_key_file(&path).unwrap();
    assert_eq!(origin, KeyOrigin::Created);
    #[cfg(unix)]
    {
        assert_eq!(mode(&path), 0o600);
        assert_eq!(mode(path.parent().unwrap()), 0o700);
    }
    let (second, origin) = load_or_create_key_file(&path).unwrap();
    assert_eq!(origin, KeyOrigin::Existing);
    assert_eq!(first.seed(), second.seed());
    assert_eq!(read_key_file(&path).unwrap().seed(), first.seed());
}

#[test]
fn reading_a_missing_key_file_names_the_path() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("absent.key");
    match read_key_file(&path) {
        Err(PairingError::IdentityIo { path: p, .. }) => assert_eq!(p, path),
        other => panic!("expected IdentityIo, got {other:?}"),
    }
}

#[test]
fn the_public_key_and_fingerprint_follow_from_the_seed() {
    let key = MachineKey::from_seed(SEED_A);
    let expected_pk = ed25519_dalek::SigningKey::from_bytes(&SEED_A)
        .verifying_key()
        .to_bytes();
    assert_eq!(key.public_key(), expected_pk);
    let expected_fp: [u8; 32] = Sha256::digest(expected_pk).into();
    assert_eq!(key.fingerprint(), expected_fp);
    let hex: String = expected_fp[..3]
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    assert_eq!(key.short_fingerprint(), hex);
    assert_eq!(key.short_fingerprint().len(), 6);
    // Determinism: a second derivation is bit-identical.
    assert_eq!(
        MachineKey::from_seed(SEED_A).fingerprint(),
        key.fingerprint()
    );
}

#[test]
fn no_seed_bytes_reach_debug_or_error_text() {
    let seed_hex: String = SEED_B.iter().map(|b| format!("{b:02x}")).collect();
    let seed_list = format!("{:?}", SEED_B);
    let key = MachineKey::from_seed(SEED_B);
    let debug = format!("{key:?}");
    assert!(debug.contains(&key.short_fingerprint()), "{debug}");
    for text in [debug, format!("{:#?}", key)] {
        assert!(!text.contains(&seed_hex), "{text}");
        assert!(!text.contains(&seed_list), "{text}");
        assert!(!text.contains("9, 9, 9"), "{text}");
    }

    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.device_key_path(), &SEED_A);
    write_raw(&l.legacy_desk_key_path(), &SEED_B);
    let err = l.load_or_create_machine_key().unwrap_err();
    for text in [err.to_string(), format!("{err:?}")] {
        assert!(!text.contains(&seed_hex), "{text}");
        assert!(!text.contains("9, 9, 9"), "{text}");
        assert!(!text.contains("7, 7, 7"), "{text}");
    }
    assert!(err.to_string().contains("desk.key"), "{err}");
    assert!(err.to_string().contains("device.key"), "{err}");
}

// ---------------------------------------------------------------------------
// The high-water file
// ---------------------------------------------------------------------------

/// The documented MAC derivation, recomputed independently.
fn oracle_tag(seed: &[u8; 32], framed: &[u8]) -> [u8; 32] {
    let mut sub = <Hmac<Sha256> as Mac>::new_from_slice(seed).unwrap();
    sub.update(b"cerulion-identity-high-water:v1");
    let subkey = sub.finalize().into_bytes();
    let mut mac = <Hmac<Sha256> as Mac>::new_from_slice(&subkey).unwrap();
    mac.update(framed);
    mac.finalize().into_bytes().into()
}

#[test]
fn the_high_water_layout_is_pinned() {
    let hw = HighWater {
        clock_unix_s: 0x0102_0304_0506_0708,
        bundle_generation: 0x0a0b_0c0d,
        status_seq: 0x1112_1314_1516_1718,
    };
    let key = MachineKey::from_seed(SEED_A);
    let bytes = encode_high_water(&hw, &key);
    let framed: Vec<u8> = [
        b"CERHW001".as_slice(),
        &[0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01],
        &[0x0d, 0x0c, 0x0b, 0x0a],
        &[0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11],
    ]
    .concat();
    assert_eq!(HIGH_WATER_LEN, 60);
    assert_eq!(bytes.len(), HIGH_WATER_LEN);
    assert_eq!(&bytes[..28], framed.as_slice());
    assert_eq!(&bytes[28..], &oracle_tag(&SEED_A, &framed));
    assert_eq!(decode_high_water(&bytes, &key).unwrap(), hw);
    // Determinism: encoding twice is bit-identical.
    assert_eq!(encode_high_water(&hw, &key), bytes);
}

#[test]
fn a_tampered_or_foreign_high_water_is_refused() {
    let hw = HighWater {
        clock_unix_s: 1_800_000_000,
        bundle_generation: 3,
        status_seq: 12,
    };
    let key = MachineKey::from_seed(SEED_A);
    let good = encode_high_water(&hw, &key);
    for i in 0..good.len() {
        let mut bad = good.clone();
        bad[i] ^= 0x01;
        assert!(
            matches!(
                decode_high_water(&bad, &key),
                Err(PairingError::HighWaterTampered)
            ),
            "flipping byte {i} must be refused as tampered"
        );
    }
    let other = MachineKey::from_seed(SEED_B);
    assert!(matches!(
        decode_high_water(&good, &other),
        Err(PairingError::HighWaterTampered)
    ));
}

#[test]
fn a_short_long_or_wrong_magic_high_water_is_corrupt() {
    let key = MachineKey::from_seed(SEED_A);
    let good = encode_high_water(&HighWater::default(), &key);
    for bytes in [&good[..59], &[good.as_slice(), &[0]].concat()[..], &[][..]] {
        assert!(matches!(
            decode_high_water(bytes, &key),
            Err(PairingError::HighWaterCorrupt(_))
        ));
    }
    // A valid tag over the wrong magic: the MAC passes, the magic check refuses.
    let mut framed = good[..28].to_vec();
    framed[..8].copy_from_slice(b"CERHW002");
    let tag = oracle_tag(&SEED_A, &framed);
    let forged = [framed.as_slice(), &tag].concat();
    assert!(matches!(
        decode_high_water(&forged, &key),
        Err(PairingError::HighWaterCorrupt("bad magic"))
    ));
}

#[test]
fn raising_takes_the_fieldwise_maximum() {
    let a = HighWater {
        clock_unix_s: 10,
        bundle_generation: 5,
        status_seq: 1,
    };
    let b = HighWater {
        clock_unix_s: 4,
        bundle_generation: 9,
        status_seq: 1,
    };
    let want = HighWater {
        clock_unix_s: 10,
        bundle_generation: 9,
        status_seq: 1,
    };
    assert_eq!(a.raised_to(b), want);
    assert_eq!(b.raised_to(a), want);
    assert_eq!(a.raised_to(a), a);
}

#[test]
fn the_high_water_file_starts_at_zero_and_never_goes_down() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let t = team("acme");
    let key = MachineKey::from_seed(SEED_A);
    assert_eq!(l.load_high_water(&t, &key).unwrap(), HighWater::default());
    let high = HighWater {
        clock_unix_s: 2_000,
        bundle_generation: 4,
        status_seq: 30,
    };
    assert_eq!(l.raise_high_water(&t, &key, high).unwrap(), high);
    let lower = HighWater {
        clock_unix_s: 1_000,
        bundle_generation: 5,
        status_seq: 2,
    };
    let after = l.raise_high_water(&t, &key, lower).unwrap();
    assert_eq!(
        after,
        HighWater {
            clock_unix_s: 2_000,
            bundle_generation: 5,
            status_seq: 30,
        }
    );
    assert_eq!(l.load_high_water(&t, &key).unwrap(), after);
    #[cfg(unix)]
    {
        assert_eq!(mode(&l.high_water_path(&t)), 0o600);
        assert_eq!(mode(&l.team_dir(&t)), 0o700);
    }
    let other = team("other");
    assert_eq!(
        l.load_high_water(&other, &key).unwrap(),
        HighWater::default()
    );
}

#[test]
fn a_tampered_high_water_file_is_refused_not_reset() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let t = team("acme");
    let key = MachineKey::from_seed(SEED_A);
    let hw = HighWater {
        clock_unix_s: 99,
        bundle_generation: 2,
        status_seq: 3,
    };
    l.raise_high_water(&t, &key, hw).unwrap();
    let path = l.high_water_path(&t);
    let mut bytes = std::fs::read(&path).unwrap();
    bytes[10] ^= 0xff;
    std::fs::write(&path, &bytes).unwrap();
    assert!(matches!(
        l.load_high_water(&t, &key),
        Err(PairingError::HighWaterTampered)
    ));
    assert!(matches!(
        l.raise_high_water(&t, &key, HighWater::default()),
        Err(PairingError::HighWaterTampered)
    ));
    assert_eq!(
        std::fs::read(&path).unwrap(),
        bytes,
        "a refused raise writes nothing"
    );
}

#[test]
fn concurrent_raises_keep_the_maximum() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let t = team("acme");
    let threads: Vec<_> = (1..=16u64)
        .map(|i| {
            let l = l.clone();
            let t = t.clone();
            std::thread::spawn(move || {
                let key = MachineKey::from_seed(SEED_A);
                for j in 0..8u64 {
                    let v = i * 100 + j;
                    l.raise_high_water(
                        &t,
                        &key,
                        HighWater {
                            clock_unix_s: v,
                            bundle_generation: u32::try_from(v).unwrap(),
                            status_seq: v,
                        },
                    )
                    .unwrap();
                }
            })
        })
        .collect();
    for th in threads {
        th.join().unwrap();
    }
    let key = MachineKey::from_seed(SEED_A);
    assert_eq!(
        l.load_high_water(&t, &key).unwrap(),
        HighWater {
            clock_unix_s: 1_607,
            bundle_generation: 1_607,
            status_seq: 1_607,
        }
    );
}

#[test]
fn racing_first_loads_on_an_empty_machine_agree_on_one_key() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let barrier = std::sync::Arc::new(std::sync::Barrier::new(16));
    let threads: Vec<_> = (0..16)
        .map(|_| {
            let l = l.clone();
            let barrier = barrier.clone();
            std::thread::spawn(move || {
                barrier.wait();
                *l.load_or_create_machine_key().unwrap().0.seed()
            })
        })
        .collect();
    let seeds: Vec<[u8; 32]> = threads.into_iter().map(|t| t.join().unwrap()).collect();
    assert!(
        seeds.iter().all(|s| s == &seeds[0]),
        "every racer got the same key"
    );
    assert_eq!(std::fs::read(l.device_key_path()).unwrap(), seeds[0]);
    assert_eq!(std::fs::read(l.legacy_desk_key_path()).unwrap(), seeds[0]);
    let leftovers: Vec<_> = std::fs::read_dir(l.identity_dir())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(
        leftovers,
        vec!["device.key".to_string()],
        "no temporary files remain"
    );
    let mut top: Vec<_> = std::fs::read_dir(l.config_dir())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    top.sort();
    assert_eq!(top, vec!["desk.key".to_string(), "identity".to_string()]);
}

#[cfg(unix)]
#[test]
fn an_adopted_key_readable_by_others_is_restricted_to_its_owner() {
    use std::os::unix::fs::PermissionsExt;
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    write_raw(&l.legacy_desk_key_path(), &SEED_A);
    std::fs::set_permissions(
        l.legacy_desk_key_path(),
        std::fs::Permissions::from_mode(0o644),
    )
    .unwrap();
    let (_, origin) = l.load_or_create_machine_key().unwrap();
    assert_eq!(origin, KeyOrigin::AdoptedLegacy);
    assert_eq!(mode(&l.device_key_path()), 0o600);
    assert_eq!(mode(&l.legacy_desk_key_path()), 0o600);
}

#[test]
fn a_raise_waits_for_the_high_water_lock() {
    let dir = tempfile::tempdir().unwrap();
    let l = layout(&dir);
    let t = team("acme");
    std::fs::create_dir_all(l.team_dir(&t)).unwrap();
    let lock = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(l.team_dir(&t).join("high_water.lock"))
        .unwrap();
    lock.lock().unwrap();
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let raiser = {
        let l = l.clone();
        let t = t.clone();
        std::thread::spawn(move || {
            let key = MachineKey::from_seed(SEED_A);
            let hw = HighWater {
                clock_unix_s: 5,
                bundle_generation: 5,
                status_seq: 5,
            };
            let got = l.raise_high_water(&t, &key, hw).unwrap();
            done_tx.send(()).unwrap();
            got
        })
    };
    assert!(
        done_rx
            .recv_timeout(std::time::Duration::from_millis(300))
            .is_err(),
        "a raise must not complete while another holder has the lock"
    );
    assert!(!l.high_water_path(&t).exists());
    lock.unlock().unwrap();
    let got = raiser.join().unwrap();
    assert_eq!(got.status_seq, 5);
    assert_eq!(
        l.load_high_water(&t, &MachineKey::from_seed(SEED_A))
            .unwrap(),
        got
    );
}
