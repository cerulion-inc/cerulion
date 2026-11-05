// SPDX-License-Identifier: AGPL-3.0-only
//! `account.certificates`: the device certificate this machine signs in and
//! pairs with, read from local files.
//!
//! Reads `device.cert` under the Cerulion config directory, decodes it with
//! [`crate::device_binding::decode_device_cert`] (no signature or chain check,
//! no network), and compares its validity window with the clock and the key
//! it attests with this machine's device key, read with
//! [`cerulion_pairing::identity::read_key_file`], which creates nothing.

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const CERTIFICATES: Check = Check {
    id: "account.certificates",
    why: "remote access and pairing present this machine's device certificate; an expired \
          one, or one issued for another device key, is refused by the robot and the account \
          service.",
    network: false,
    run,
};

/// What was read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum CertReading {
    /// No config directory resolves.
    NoConfigDir,
    /// No `device.cert`.
    Absent { path: String },
    /// The file exists but cannot be read or decoded.
    Unusable { path: String, error: String },
    /// The certificate decoded.
    Read {
        path: String,
        not_before_ns: u64,
        not_after_ns: u64,
        /// Whether it attests this machine's device key; `None` when this
        /// machine has no device key file.
        matches_device_key: Option<bool>,
        /// Why the device key could not be compared: a key file that exists
        /// but cannot be read, or two key files that disagree.
        key_problem: Option<String>,
    },
}

pub(super) fn decide(r: &CertReading, now_ns: u64) -> Outcome {
    const NS: u64 = 1_000_000_000;
    let login = "run `cerulion login`, which issues a new one";
    match r {
        CertReading::NoConfigDir => Outcome::new(
            Status::Skip,
            "not run: there is no Cerulion config directory to read",
        ),
        CertReading::Absent { path } => Outcome::new(
            Status::Skip,
            format!("not run: no device certificate at {path}"),
        ),
        CertReading::Unusable { path, error } => {
            Outcome::new(Status::Fail, format!("{path} is unusable: {error}")).with_fix(login)
        }
        CertReading::Read {
            path,
            not_before_ns,
            not_after_ns,
            matches_device_key,
            key_problem,
        } => {
            let base = |status, summary: String| {
                let mut o = Outcome::new(status, summary)
                    .reading("path", Reading::Text(path.clone()))
                    .reading("not_before_unix_s", Reading::Int(not_before_ns / NS))
                    .reading("not_after_unix_s", Reading::Int(not_after_ns / NS));
                if let Some(m) = matches_device_key {
                    o = o.reading("matches_device_key", Reading::Bool(*m));
                }
                o
            };
            if let Some(problem) = key_problem {
                base(
                    Status::Fail,
                    format!("this machine's device key cannot be read: {problem}"),
                )
                .with_fix(login)
            } else if *matches_device_key == Some(false) {
                base(
                    Status::Fail,
                    "the device certificate attests a different device key than this machine \
                     holds"
                        .to_string(),
                )
                .with_fix(login)
            } else if now_ns >= *not_after_ns {
                base(
                    Status::Fail,
                    format!(
                        "the device certificate expired {} ago",
                        super::account::duration_words((now_ns - not_after_ns) / NS)
                    ),
                )
                .with_fix(login)
            } else if now_ns < *not_before_ns {
                base(
                    Status::Warn,
                    format!(
                        "the device certificate is valid only in {}: this clock may be behind",
                        super::account::duration_words((not_before_ns - now_ns) / NS)
                    ),
                )
                .with_fix("check `cerulion doctor --check clock.sync`, then `cerulion login`")
            } else {
                base(
                    Status::Pass,
                    format!(
                        "the device certificate is valid for another {}",
                        super::account::duration_words((not_after_ns - now_ns) / NS)
                    ),
                )
            }
        }
    }
}

fn read() -> CertReading {
    let Some(path) = crate::auth::device_cert_path() else {
        return CertReading::NoConfigDir;
    };
    let shown = path.display().to_string();
    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return CertReading::Absent { path: shown }
        }
        Err(e) => {
            return CertReading::Unusable {
                path: shown,
                error: e.to_string(),
            }
        }
    };
    let signed = match crate::device_binding::decode_device_cert(text.trim()) {
        Ok(signed) => signed,
        Err(e) => {
            return CertReading::Unusable {
                path: shown,
                error: e.to_string(),
            }
        }
    };
    let (matches_device_key, key_problem) = match crate::auth::identity_layout() {
        None => (None, None),
        Some(layout) => match device_key(&layout) {
            Ok(key) => (key.map(|k| k == signed.cert.device_key.0), None),
            Err(problem) => (None, Some(problem)),
        },
    };
    CertReading::Read {
        path: shown,
        not_before_ns: signed.cert.validity.not_before_ns,
        not_after_ns: signed.cert.validity.not_after_ns,
        matches_device_key,
        key_problem,
    }
}

/// The public half of this machine's device key, read the way the CLI
/// resolves it: `identity/device.key`, else the legacy `desk.key`. A file that
/// does not exist is skipped; one that exists but cannot be read, or two that
/// hold different keys, is an error.
fn device_key(
    layout: &cerulion_pairing::identity::IdentityLayout,
) -> Result<Option<[u8; 32]>, String> {
    let read = |path: std::path::PathBuf| -> Result<Option<[u8; 32]>, String> {
        match std::fs::symlink_metadata(&path) {
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            _ => cerulion_pairing::identity::read_key_file(&path)
                .map(|k| Some(k.public_key()))
                .map_err(|e| format!("{}: {e}", path.display())),
        }
    };
    let canonical = read(layout.device_key_path())?;
    let legacy = read(layout.legacy_desk_key_path())?;
    match (canonical, legacy) {
        (Some(a), Some(b)) if a != b => Err(format!(
            "{} and {} hold different keys",
            layout.device_key_path().display(),
            layout.legacy_desk_key_path().display()
        )),
        (Some(key), _) | (None, Some(key)) => Ok(Some(key)),
        (None, None) => Ok(None),
    }
}

fn run(_ctx: &Context) -> Outcome {
    decide(&read(), crate::auth::now_unix_ns())
}

#[cfg(test)]
mod tests {
    use super::*;

    const NS: u64 = 1_000_000_000;
    const NOW: u64 = 1_800_000_000 * NS;

    fn cert(not_before_s: u64, not_after_s: u64, matches: Option<bool>) -> CertReading {
        CertReading::Read {
            path: "/h/.cerulion/device.cert".to_string(),
            not_before_ns: not_before_s * NS,
            not_after_ns: not_after_s * NS,
            matches_device_key: matches,
            key_problem: None,
        }
    }

    #[test]
    fn a_valid_certificate_passes_with_its_time_left() {
        let o = decide(
            &cert(1_700_000_000, 1_800_000_000 + 3 * 86_400, Some(true)),
            NOW,
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "the device certificate is valid for another 3 days"
        );
        assert_eq!(
            o.readings.get("matches_device_key"),
            Some(&Reading::Bool(true))
        );
    }

    #[test]
    fn an_expired_certificate_fails_at_its_exclusive_bound() {
        let o = decide(&cert(1_700_000_000, 1_800_000_000, Some(true)), NOW);
        assert_eq!(o.status, Status::Fail);
        assert_eq!(o.summary, "the device certificate expired 0 seconds ago");
        assert_eq!(
            o.fix.as_deref(),
            Some("run `cerulion login`, which issues a new one")
        );
    }

    #[test]
    fn a_certificate_for_another_key_fails_before_its_dates_are_judged() {
        let o = decide(&cert(1_700_000_000, 1_900_000_000, Some(false)), NOW);
        assert_eq!(o.status, Status::Fail);
        assert!(o.summary.contains("different device key"));
    }

    #[test]
    fn the_validity_window_includes_not_before() {
        let o = decide(&cert(1_800_000_000, 1_800_000_001, Some(true)), NOW);
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "the device certificate is valid for another 1 second"
        );
    }

    #[test]
    fn an_unreadable_device_key_fails_before_the_dates() {
        let r = CertReading::Read {
            path: "/h/.cerulion/device.cert".to_string(),
            not_before_ns: 0,
            not_after_ns: u64::MAX,
            matches_device_key: None,
            key_problem: Some("/h/.cerulion/identity/device.key: malformed".to_string()),
        };
        let o = decide(&r, NOW);
        assert_eq!(o.status, Status::Fail);
        assert_eq!(
            o.summary,
            "this machine's device key cannot be read: /h/.cerulion/identity/device.key: malformed"
        );
    }

    #[test]
    fn the_device_key_falls_back_only_when_the_canonical_file_is_absent() {
        let dir = tempfile::tempdir().unwrap();
        let layout = cerulion_pairing::identity::IdentityLayout::at(dir.path());
        assert_eq!(device_key(&layout), Ok(None));
        std::fs::write(layout.legacy_desk_key_path(), b"not a key").unwrap();
        assert!(
            device_key(&layout).is_err(),
            "a malformed legacy key is an error"
        );
        std::fs::create_dir_all(layout.device_key_path().parent().unwrap()).unwrap();
        std::fs::write(layout.device_key_path(), b"also not a key").unwrap();
        let err = device_key(&layout).unwrap_err();
        assert!(
            err.contains("device.key"),
            "the canonical file is read first: {err}"
        );
    }

    #[test]
    fn a_certificate_from_the_future_warns_about_the_clock() {
        let o = decide(&cert(1_800_000_000 + 7_200, 1_900_000_000, None), NOW);
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "the device certificate is valid only in 2 hours: this clock may be behind"
        );
        assert_eq!(o.readings.get("matches_device_key"), None);
    }

    #[test]
    fn no_certificate_is_a_skip_and_an_unusable_one_fails() {
        let absent = decide(
            &CertReading::Absent {
                path: "/h/.cerulion/device.cert".to_string(),
            },
            NOW,
        );
        assert_eq!(absent.status, Status::Skip);
        let bad = decide(
            &CertReading::Unusable {
                path: "/h/.cerulion/device.cert".to_string(),
                error: "not base64url".to_string(),
            },
            NOW,
        );
        assert_eq!(bad.status, Status::Fail);
        assert_eq!(
            bad.summary,
            "/h/.cerulion/device.cert is unusable: not base64url"
        );
        assert_eq!(decide(&CertReading::NoConfigDir, NOW).status, Status::Skip);
    }
}
