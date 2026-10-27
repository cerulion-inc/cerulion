// SPDX-License-Identifier: AGPL-3.0-only
//! `account.login`: whether this machine is signed in, read from local state.
//!
//! Reads `auth.json` under the Cerulion config directory with
//! [`crate::auth::load_from`] and classifies it with [`crate::auth::local_gate`],
//! the same classification the login gate in front of every command makes.
//! Reaches no network and refreshes nothing. The account id is not printed.

use super::{Check, Context, Outcome, Reading, Status};
use crate::auth::{LoadedAuth, LocalGate};

pub(super) const LOGIN: Check = Check {
    id: "account.login",
    why: "every cerulion command except login, logout, clean, completions, telemetry and \
          doctor runs only on a machine that has signed in; on a machine that never has, or \
          that signed out, a command started from a terminal first runs the sign-in and one \
          started elsewhere is refused with exit 7.",
    network: false,
    run,
};

/// The decision. `path` is where `auth.json` was looked for, `None` when no
/// config directory resolves (no home directory and no `CERULION_HOME`).
pub(super) fn decide(path: Option<&str>, loaded: &LoadedAuth, now_ns: u64) -> Outcome {
    let Some(path) = path else {
        return Outcome::new(
            Status::Fail,
            "no Cerulion config directory: there is no home directory and CERULION_HOME is \
             not set",
        )
        .with_fix("set CERULION_HOME to a directory this user can write, then `cerulion login`");
    };
    let path_reading = Reading::Text(path.to_string());
    if let LoadedAuth::Corrupt(err) = loaded {
        return corrupt(path, err).reading("auth_json", path_reading);
    }
    let outcome = match crate::auth::local_gate(loaded, now_ns) {
        LocalGate::RefuseNeverLoggedIn => Outcome::new(
            Status::Fail,
            format!("this machine has never signed in (no session in {path})"),
        )
        .with_fix("run `cerulion login`"),
        LocalGate::RefuseSignedOut => Outcome::new(
            Status::Fail,
            format!("this machine is signed out ({path} keeps no session)"),
        )
        .with_fix("run `cerulion login`"),
        LocalGate::ProceedValidSession => {
            let left_s = loaded.state().map_or(0, |s| {
                s.expires_at_ns.saturating_sub(now_ns) / 1_000_000_000
            });
            Outcome::new(
                Status::Pass,
                format!(
                    "signed in; the session is valid for another {}",
                    duration_words(left_s)
                ),
            )
        }
        LocalGate::ProceedExpiredLocalForever => {
            let ago_s = loaded.state().map_or(0, |s| {
                now_ns.saturating_sub(s.expires_at_ns) / 1_000_000_000
            });
            Outcome::new(
                Status::Info,
                format!(
                    "signed in; the session expired {} ago, so local, LAN and existing remote \
                     work continue and the next account call refreshes it",
                    duration_words(ago_s)
                ),
            )
        }
    };
    let mut outcome = outcome.reading("auth_json", path_reading);
    if let Some(state) = loaded.state() {
        outcome = outcome
            .reading("logged_in_ever", Reading::Bool(state.logged_in_ever))
            .reading(
                "session_expires_unix_s",
                Reading::Int(state.expires_at_ns / 1_000_000_000),
            );
    }
    outcome
}

/// The outcome for a sign-in file `load_from` reported as `err`: `read
/// <path>: <io error>` or `parse <path>: line <L> column <C>`, which carries
/// the parse error's location and never its text.
fn corrupt(path: &str, err: &str) -> Outcome {
    if let Some(io) = err.strip_prefix(&format!("read {path}: ")) {
        return Outcome::new(
            Status::Fail,
            format!("the sign-in file {path} cannot be read: {io}"),
        )
        .with_fix(format!("make {path} readable by this user"));
    }
    let location = err
        .strip_prefix(&format!("parse {path}: "))
        .filter(|loc| {
            loc.strip_prefix("line ")
                .and_then(|rest| rest.split_once(" column "))
                .is_some_and(|(l, c)| {
                    [l, c]
                        .iter()
                        .all(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
                })
        })
        .map_or(String::new(), |loc| format!(" at {loc}"));
    Outcome::new(
        Status::Fail,
        format!("the sign-in file {path} cannot be parsed{location}"),
    )
    .with_fix("run `cerulion login`, which writes a new one")
}

/// `secs` as the largest whole unit: days, hours, minutes or seconds.
pub(super) fn duration_words(secs: u64) -> String {
    let (n, unit) = if secs >= 86_400 {
        (secs / 86_400, "day")
    } else if secs >= 3_600 {
        (secs / 3_600, "hour")
    } else if secs >= 60 {
        (secs / 60, "minute")
    } else {
        (secs, "second")
    };
    if n == 1 {
        format!("1 {unit}")
    } else {
        format!("{n} {unit}s")
    }
}

fn run(_ctx: &Context) -> Outcome {
    let path = crate::auth::auth_json_path();
    let loaded = path
        .as_deref()
        .map_or(LoadedAuth::Absent, crate::auth::load_from);
    let shown = path.as_ref().map(|p| p.display().to_string());
    decide(shown.as_deref(), &loaded, crate::auth::now_unix_ns())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::auth::AuthState;

    const NOW: u64 = 1_800_000_000 * 1_000_000_000;
    const PATH: &str = "/h/.cerulion/auth.json";

    fn state(expires_at_ns: u64, logged_in_ever: bool) -> LoadedAuth {
        LoadedAuth::Present(AuthState {
            account_id: "acct".to_string(),
            session_token: "s".to_string(),
            refresh_token: "r".to_string(),
            expires_at_ns,
            logged_in_ever,
            role: None,
        })
    }

    #[test]
    fn a_valid_session_passes_and_says_how_long() {
        let o = decide(
            Some(PATH),
            &state(NOW + 3 * 3_600 * 1_000_000_000, true),
            NOW,
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "signed in; the session is valid for another 3 hours"
        );
        assert!(!o.summary.contains("acct"));
    }

    #[test]
    fn an_expired_session_is_info_not_a_failure() {
        let o = decide(
            Some(PATH),
            &state(NOW - 2 * 86_400 * 1_000_000_000, true),
            NOW,
        );
        assert_eq!(o.status, Status::Info);
        assert!(o.summary.contains("expired 2 days ago"), "{}", o.summary);
        assert_eq!(o.fix, None);
    }

    #[test]
    fn never_signed_in_fails_with_login() {
        for loaded in [LoadedAuth::Absent, state(NOW + 1, false)] {
            let o = decide(Some(PATH), &loaded, NOW);
            assert_eq!(o.status, Status::Fail);
            assert_eq!(o.fix.as_deref(), Some("run `cerulion login`"));
        }
    }

    #[test]
    fn signed_out_fails_with_its_own_line() {
        let o = decide(
            Some(PATH),
            &LoadedAuth::SignedOut {
                account_id: Some("acct".to_string()),
                role: None,
            },
            NOW,
        );
        assert_eq!(o.status, Status::Fail);
        assert!(o.summary.contains("signed out"));
    }

    #[test]
    fn a_corrupt_file_fails_naming_only_the_line_and_column() {
        let o = decide(
            Some(PATH),
            &LoadedAuth::Corrupt(format!("parse {PATH}: line 1 column 42")),
            NOW,
        );
        assert_eq!(o.status, Status::Fail);
        assert_eq!(
            o.summary,
            "the sign-in file /h/.cerulion/auth.json cannot be parsed at line 1 column 42"
        );
        assert_eq!(
            o.fix.as_deref(),
            Some("run `cerulion login`, which writes a new one")
        );
        let no_location = decide(
            Some(PATH),
            &LoadedAuth::Corrupt(format!("parse {PATH}: something else")),
            NOW,
        );
        assert_eq!(
            no_location.summary,
            "the sign-in file /h/.cerulion/auth.json cannot be parsed"
        );
        let unreadable = decide(
            Some(PATH),
            &LoadedAuth::Corrupt(format!("read {PATH}: permission denied")),
            NOW,
        );
        assert_eq!(
            unreadable.summary,
            "the sign-in file /h/.cerulion/auth.json cannot be read: permission denied"
        );
        assert_eq!(
            unreadable.fix.as_deref(),
            Some("make /h/.cerulion/auth.json readable by this user")
        );
    }

    #[test]
    fn a_malformed_file_never_echoes_a_token_from_it() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            r#"{"account_id":"a","session_token":"s","refresh_token":"r","expires_at_ns":"tok_SECRET_FRAGMENT","logged_in_ever":true}"#,
        )
        .unwrap();
        let loaded = crate::auth::load_from(&path);
        let LoadedAuth::Corrupt(err) = &loaded else {
            panic!("expected a corrupt file, got another state");
        };
        assert!(
            !err.contains("tok_SECRET"),
            "load_from kept the token: {err}"
        );
        assert!(err.contains(": line 1 column "), "{err}");
        let shown = path.display().to_string();
        let o = decide(Some(&shown), &loaded, NOW);
        assert_eq!(o.status, Status::Fail);
        let text = format!("{o:?}");
        assert!(!text.contains("tok_SECRET"), "{text}");
        assert!(
            o.summary.contains(" cannot be parsed at line 1 column "),
            "{}",
            o.summary
        );
        // The oracle: serde_json's own message for this file quotes the token,
        // which is what `load_from` must not pass on.
        let raw = serde_json::from_str::<crate::auth::AuthState>(
            &std::fs::read_to_string(&path).unwrap(),
        )
        .unwrap_err()
        .to_string();
        assert!(raw.contains("tok_SECRET_FRAGMENT"), "{raw}");
    }

    #[test]
    fn no_config_dir_fails() {
        let o = decide(None, &LoadedAuth::Absent, NOW);
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().contains("CERULION_HOME"));
    }

    #[test]
    fn duration_words_picks_the_largest_unit() {
        assert_eq!(duration_words(0), "0 seconds");
        assert_eq!(duration_words(1), "1 second");
        assert_eq!(duration_words(119), "1 minute");
        assert_eq!(duration_words(7_200), "2 hours");
        assert_eq!(duration_words(86_400), "1 day");
    }
}
