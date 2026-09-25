// SPDX-License-Identifier: AGPL-3.0-only
//! Consent precedence (pure table, both feature states) and, with `posthog`,
//! the on-disk file: first-use creation, atomic rewrite, 0600, corrupt-file
//! recovery. Env-touching tests share one lock (env vars are process-global).

use cerulion_telemetry::consent::{resolve, Source, Status};

fn st(enabled: bool, source: Source) -> Status {
    Status { enabled, source }
}

#[test]
fn precedence_table() {
    type Row = (
        Option<&'static str>,
        Option<&'static str>,
        Option<bool>,
        Status,
    );
    // (DO_NOT_TRACK, CERULION_TELEMETRY, file.enabled) -> expected
    let table: &[Row] = &[
        (None, None, None, st(true, Source::Default)),
        (None, None, Some(true), st(true, Source::File)),
        (None, None, Some(false), st(false, Source::File)),
        (None, Some("0"), Some(true), st(false, Source::EnvVar)),
        (None, Some("1"), Some(false), st(true, Source::EnvVar)),
        (None, Some(" 1 "), None, st(true, Source::EnvVar)),
        (Some(" 1 "), Some("1"), None, st(false, Source::DoNotTrack)),
        (Some("1\n"), None, Some(true), st(false, Source::DoNotTrack)),
        (None, Some("yes"), Some(false), st(false, Source::File)),
        (None, Some(""), None, st(true, Source::Default)),
        (
            Some("1"),
            Some("1"),
            Some(true),
            st(false, Source::DoNotTrack),
        ),
        (Some("true"), None, None, st(false, Source::DoNotTrack)),
        (Some("TRUE"), Some("1"), None, st(false, Source::DoNotTrack)),
        (Some("0"), None, None, st(true, Source::Default)),
        (Some("0"), Some("0"), None, st(false, Source::EnvVar)),
        (Some(""), Some("1"), Some(false), st(true, Source::EnvVar)),
    ];
    for (dnt, env, file, want) in table {
        assert_eq!(
            resolve(*dnt, *env, *file),
            *want,
            "DO_NOT_TRACK={dnt:?} CERULION_TELEMETRY={env:?} file={file:?}"
        );
    }
}

#[cfg(not(feature = "posthog"))]
mod feature_off {
    use super::*;
    use cerulion_telemetry::consent;

    #[test]
    fn everything_is_a_no_op_and_nothing_touches_disk() {
        let dir = tempfile::tempdir().expect("tempdir");
        let home = dir.path().join("home");
        // No lock needed: this is the only env-touching test in the feature-off binary.
        std::env::set_var("CERULION_HOME", &home);
        assert_eq!(consent::status(), st(false, Source::NotCompiled));
        assert_eq!(consent::anon_id().expect("ok"), None);
        consent::set_enabled(false).expect("ok");
        consent::mark_notice_shown().expect("ok");
        assert!(
            !home.exists(),
            "feature off must never create telemetry.json"
        );
    }
}

#[cfg(feature = "posthog")]
mod feature_on {
    use super::*;
    use cerulion_telemetry::consent::{self, TelemetryFile};
    use std::sync::{Mutex, MutexGuard};

    static ENV: Mutex<()> = Mutex::new(());

    struct Env {
        _guard: MutexGuard<'static, ()>,
        _dir: tempfile::TempDir,
        home: std::path::PathBuf,
    }

    fn isolated() -> Env {
        let guard = ENV.lock().unwrap_or_else(|p| p.into_inner());
        let dir = tempfile::tempdir().expect("tempdir");
        let home = dir.path().join("cerulion-home");
        std::env::set_var("CERULION_HOME", &home);
        std::env::remove_var("DO_NOT_TRACK");
        std::env::remove_var("CERULION_TELEMETRY");
        Env {
            _guard: guard,
            _dir: dir,
            home,
        }
    }

    /// `xxxxxxxx-xxxx-4xxx-[89ab]xxx-xxxxxxxxxxxx`, lowercase hex.
    fn is_hyphenated_uuid_v4(s: &str) -> bool {
        let groups: Vec<&str> = s.split('-').collect();
        let lower_hex = |g: &str| {
            g.bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        };
        groups.len() == 5
            && [8, 4, 4, 4, 12]
                .iter()
                .zip(&groups)
                .all(|(n, g)| g.len() == *n && lower_hex(g))
            && groups[2].starts_with('4')
            && matches!(groups[3].as_bytes()[0], b'8' | b'9' | b'a' | b'b')
    }

    /// `YYYY-MM-DDTHH:MM:SS.mmmZ` with every digit a digit and in range.
    fn is_rfc3339_utc_millis(s: &str) -> bool {
        let b = s.as_bytes();
        if b.len() != 24 {
            return false;
        }
        let digits_at = |idx: &[usize]| idx.iter().all(|&i| b[i].is_ascii_digit());
        let num = |r: std::ops::Range<usize>| s[r].parse::<u32>().unwrap_or(u32::MAX);
        digits_at(&[0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18, 20, 21, 22])
            && b[4] == b'-'
            && b[7] == b'-'
            && b[10] == b'T'
            && b[13] == b':'
            && b[16] == b':'
            && b[19] == b'.'
            && b[23] == b'Z'
            && (1..=12).contains(&num(5..7))
            && (1..=31).contains(&num(8..10))
            && num(11..13) < 24
            && num(14..16) < 60
            && num(17..19) < 60
    }

    fn read_file(env: &Env) -> TelemetryFile {
        let text = std::fs::read_to_string(env.home.join("telemetry.json")).expect("file exists");
        serde_json::from_str(&text).expect("valid telemetry.json")
    }

    #[test]
    fn status_never_creates_the_file() {
        let env = isolated();
        assert_eq!(consent::status(), st(true, Source::Default));
        std::env::set_var("DO_NOT_TRACK", "1");
        assert_eq!(consent::status(), st(false, Source::DoNotTrack));
        std::env::remove_var("DO_NOT_TRACK");
        std::env::set_var("CERULION_TELEMETRY", "0");
        assert_eq!(consent::status(), st(false, Source::EnvVar));
        assert!(!env.home.exists());
    }

    #[test]
    fn anon_id_mints_the_file_once_with_contract_shape_and_0600() {
        let env = isolated();
        let first = consent::anon_id().expect("ok").expect("Some");
        let second = consent::anon_id().expect("ok").expect("Some");
        assert_eq!(first, second, "anon id is stable across calls");
        let file = read_file(&env);
        assert_eq!(file.anon_id, first);
        assert!(file.enabled);
        assert!(!file.notice_shown);
        let uuid_part = first.strip_prefix("anon:").expect("anon: prefix");
        assert!(is_hyphenated_uuid_v4(uuid_part), "{uuid_part}");
        assert!(
            is_rfc3339_utc_millis(&file.updated_at),
            "{}",
            file.updated_at
        );
        let raw: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(env.home.join("telemetry.json")).unwrap(),
        )
        .unwrap();
        let mut keys: Vec<&String> = raw.as_object().unwrap().keys().collect();
        keys.sort();
        assert_eq!(keys, ["anon_id", "enabled", "notice_shown", "updated_at"]);
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(env.home.join("telemetry.json"))
                .unwrap()
                .permissions()
                .mode();
            assert_eq!(mode & 0o777, 0o600);
        }
        let mut leftovers: Vec<_> = std::fs::read_dir(&env.home)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        leftovers.sort();
        assert_eq!(
            leftovers,
            ["telemetry.json", "telemetry.json.lock"],
            "no temp file left behind"
        );
    }

    #[test]
    fn set_enabled_and_notice_persist_and_win_over_default_but_lose_to_env() {
        let env = isolated();
        consent::set_enabled(false).expect("ok");
        assert_eq!(consent::status(), st(false, Source::File));
        let anon = read_file(&env).anon_id;
        consent::mark_notice_shown().expect("ok");
        let file = read_file(&env);
        assert!(!file.enabled, "notice update keeps the opt-out");
        assert!(file.notice_shown);
        assert_eq!(file.anon_id, anon, "updates keep the anon id");
        std::env::set_var("CERULION_TELEMETRY", "1");
        assert_eq!(consent::status(), st(true, Source::EnvVar));
        std::env::set_var("DO_NOT_TRACK", "1");
        assert_eq!(consent::status(), st(false, Source::DoNotTrack));
        std::env::remove_var("DO_NOT_TRACK");
        std::env::remove_var("CERULION_TELEMETRY");
        consent::set_enabled(true).expect("ok");
        assert_eq!(consent::status(), st(true, Source::File));
    }

    #[test]
    fn claim_notice_is_granted_exactly_once() {
        let env = isolated();
        assert!(
            consent::claim_notice().expect("ok"),
            "fresh machine: claimed"
        );
        assert!(read_file(&env).notice_shown);
        assert!(consent::notice_shown().expect("ok"));
        let file = env.home.join("telemetry.json");
        let before = std::fs::read(&file).expect("read");
        #[cfg(unix)]
        let inode_before = {
            use std::os::unix::fs::MetadataExt;
            std::fs::metadata(&file).expect("meta").ino()
        };
        assert!(!consent::claim_notice().expect("ok"), "second claim loses");
        assert_eq!(
            std::fs::read(&file).expect("read"),
            before,
            "a lost claim must not rewrite the file"
        );
        #[cfg(unix)]
        {
            use std::os::unix::fs::MetadataExt;
            assert_eq!(
                std::fs::metadata(&file).expect("meta").ino(),
                inode_before,
                "a lost claim must not atomically replace the file either"
            );
        }
        let anon = read_file(&env).anon_id;
        consent::set_enabled(false).expect("ok");
        assert!(
            !consent::claim_notice().expect("ok"),
            "opt-out keeps the claim"
        );
        assert_eq!(read_file(&env).anon_id, anon);
    }

    #[test]
    fn concurrent_first_runs_claim_the_notice_once() {
        let env = isolated();
        let winners = std::sync::atomic::AtomicUsize::new(0);
        std::thread::scope(|s| {
            for _ in 0..8 {
                s.spawn(|| {
                    if consent::claim_notice().expect("ok") {
                        winners.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    }
                });
            }
        });
        assert_eq!(winners.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert!(read_file(&env).notice_shown);
    }

    #[test]
    fn concurrent_first_runs_show_the_notice_once() {
        let env = isolated();
        let shown = std::sync::atomic::AtomicUsize::new(0);
        let printers = std::sync::atomic::AtomicUsize::new(0);
        std::thread::scope(|s| {
            for _ in 0..8 {
                s.spawn(|| {
                    let printed = consent::show_notice_once(|| {
                        shown.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    })
                    .expect("ok");
                    if printed {
                        printers.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    }
                });
            }
        });
        assert_eq!(shown.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert_eq!(printers.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert!(read_file(&env).notice_shown);
        assert!(!consent::show_notice_once(|| panic!("already shown")).expect("ok"));
    }

    #[test]
    fn a_malformed_file_shows_the_notice_and_is_repaired() {
        let env = isolated();
        std::fs::create_dir_all(&env.home).unwrap();
        std::fs::write(env.home.join("telemetry.json"), b"{not json").unwrap();
        let mut shown = 0;
        assert!(consent::show_notice_once(|| shown += 1).expect("ok"));
        assert_eq!(shown, 1);
        assert!(read_file(&env).notice_shown);
    }

    #[test]
    fn partial_file_keeps_its_opt_out_and_is_completed_on_write() {
        let env = isolated();
        std::fs::create_dir_all(&env.home).unwrap();
        std::fs::write(env.home.join("telemetry.json"), r#"{"enabled": false}"#).unwrap();
        assert_eq!(consent::status(), st(false, Source::File));
        let id = consent::anon_id().expect("ok").expect("Some");
        let file = read_file(&env);
        assert!(!file.enabled, "opt-out survives the repair");
        assert_eq!(file.anon_id, id);
        assert!(is_hyphenated_uuid_v4(id.strip_prefix("anon:").unwrap()));
        assert!(!file.notice_shown);
        assert!(is_rfc3339_utc_millis(&file.updated_at));
        assert_eq!(consent::anon_id().expect("ok").expect("Some"), id);
        consent::mark_notice_shown().expect("ok");
        assert_eq!(consent::status(), st(false, Source::File));
    }

    #[test]
    fn corrupt_file_is_ignored_by_status_and_replaced_on_write() {
        let env = isolated();
        std::fs::create_dir_all(&env.home).unwrap();
        std::fs::write(env.home.join("telemetry.json"), "{not json").unwrap();
        assert_eq!(consent::status(), st(true, Source::Default));
        let id = consent::anon_id().expect("ok").expect("Some");
        assert_eq!(read_file(&env).anon_id, id);
    }

    #[test]
    fn malformed_stored_anon_id_is_replaced_keeping_the_other_fields() {
        let env = isolated();
        std::fs::create_dir_all(&env.home).unwrap();
        std::fs::write(
            env.home.join("telemetry.json"),
            r#"{"enabled":false,"anon_id":"legacy-id","notice_shown":true}"#,
        )
        .unwrap();
        let id = consent::anon_id().expect("ok").expect("Some");
        assert!(cerulion_telemetry::guard::is_anon_id(&id), "{id}");
        let file = read_file(&env);
        assert_eq!(file.anon_id, id);
        assert!(!file.enabled);
        assert!(file.notice_shown);
    }

    #[test]
    fn empty_cerulion_home_falls_back_to_home_dir() {
        let _env = isolated();
        std::env::set_var("CERULION_HOME", "");
        let path = consent::file_path().expect("home dir");
        let expected = dirs::home_dir()
            .expect("home dir")
            .join(".cerulion")
            .join("telemetry.json");
        assert_eq!(path, expected);
    }

    #[test]
    fn first_write_creates_the_home_root_0700() {
        let env = isolated();
        consent::anon_id().expect("ok");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&env.home).unwrap().permissions().mode();
            assert_eq!(mode & 0o777, 0o700, "{mode:o}");
        }
        assert!(env.home.join("telemetry.json").is_file());
    }

    #[cfg(unix)]
    #[test]
    fn unreadable_consent_file_fails_closed() {
        use std::os::unix::fs::PermissionsExt;
        let env = isolated();
        consent::set_enabled(false).expect("ok");
        let path = env.home.join("telemetry.json");
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o000)).unwrap();
        if std::fs::read(&path).is_ok() {
            eprintln!("running as a user that ignores mode bits; skipping");
            return;
        }
        assert_eq!(consent::status(), st(false, Source::File));
        std::env::set_var("CERULION_TELEMETRY", "1");
        assert_eq!(
            consent::status(),
            st(true, Source::EnvVar),
            "env var still outranks the file"
        );
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).unwrap();
    }

    #[test]
    fn concurrent_writers_each_land_a_complete_file() {
        let env = isolated();
        let first = consent::anon_id().expect("ok").expect("Some");
        let workers: Vec<_> = (0..8)
            .map(|i| {
                std::thread::spawn(move || {
                    for _ in 0..50 {
                        if i % 2 == 0 {
                            consent::mark_notice_shown().expect("ok");
                        } else {
                            consent::set_enabled(true).expect("ok");
                        }
                    }
                })
            })
            .collect();
        for w in workers {
            w.join().expect("writer thread");
        }
        let file = read_file(&env);
        assert_eq!(
            file.anon_id, first,
            "no writer saw a torn file and re-minted"
        );
        assert!(file.enabled);
        let leftovers: Vec<_> = std::fs::read_dir(&env.home)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .filter(|n| n != "telemetry.json" && n != "telemetry.json.lock")
            .collect();
        assert!(leftovers.is_empty(), "{leftovers:?}");
    }

    #[test]
    fn concurrent_opt_out_and_notice_are_both_preserved() {
        let env = isolated();
        consent::anon_id().expect("ok");
        let barrier = std::sync::Arc::new(std::sync::Barrier::new(2));
        let workers: Vec<_> = (0..2)
            .map(|i| {
                let barrier = std::sync::Arc::clone(&barrier);
                std::thread::spawn(move || {
                    for _ in 0..25 {
                        barrier.wait();
                        if i == 0 {
                            consent::set_enabled(false).expect("ok");
                        } else {
                            consent::mark_notice_shown().expect("ok");
                        }
                    }
                })
            })
            .collect();
        for w in workers {
            w.join().expect("writer thread");
        }
        let file = read_file(&env);
        assert!(
            !file.enabled,
            "opt-out survived the concurrent notice write"
        );
        assert!(file.notice_shown, "notice survived the concurrent opt-out");
        assert_eq!(consent::status(), st(false, Source::File));
    }
}
