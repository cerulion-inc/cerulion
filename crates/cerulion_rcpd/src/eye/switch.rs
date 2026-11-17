// SPDX-License-Identifier: AGPL-3.0-only
//! The owner's switch, the data notice, and the status file.
//!
//! All three live in the receipt log's directory (the state directory):
//!
//! - `jev.switch` holds `on` or `off`, written by `cerulion-rcpd jev on|off`.
//!   It is read before every ask, so `off` stops new requests at once.
//!   Without the file, Jev is on unless the region's notice level is
//!   `opt_in`.
//! - `jev.notice` marks that the quiet notice line was printed.
//! - `jev.status.json` is rewritten after every answer, and at most every
//!   250 ms while asks are dropped unanswered: on or off, route, pinned
//!   model, the last outcome, today's asks and looks (UTC), the asks dropped
//!   from a replay and the other asks dropped unanswered, and whether a
//!   limit was reached today. `cerulion-rcpd jev status` reads it.
//!
//! The notice level comes from [`REGION_NOTICE`], a table of region codes;
//! a region it does not list, and no region at all, is `quiet`. `quiet`
//! prints one line, once; `clear` prints it at every start; `opt_in` keeps
//! Jev off until the owner runs `cerulion-rcpd jev on`.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

/// How loudly the data notice is given.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NoticeLevel {
    /// One line, once, with the off switch; Jev on by default.
    Quiet,
    /// The line at every start; Jev on by default.
    Clear,
    /// Jev off until the owner turns it on.
    OptIn,
}

impl NoticeLevel {
    /// The level as the status file spells it.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Quiet => "quiet",
            Self::Clear => "clear",
            Self::OptIn => "opt_in",
        }
    }
}

/// Region codes whose law requires more than the quiet notice. Every region
/// not listed is [`NoticeLevel::Quiet`].
pub const REGION_NOTICE: &[(&str, NoticeLevel)] = &[];

/// A region code in upper case: two letters, optionally `-` and 1 to 3
/// letters or digits (`US`, `US-CA`).
///
/// # Errors
///
/// Why the code is refused.
pub fn normalize_region(region: &str) -> Result<String, String> {
    let upper = region.trim().to_ascii_uppercase();
    let (country, sub) = match upper.split_once('-') {
        Some((c, s)) => (c, Some(s)),
        None => (upper.as_str(), None),
    };
    let country_ok = country.len() == 2 && country.bytes().all(|b| b.is_ascii_uppercase());
    let sub_ok = sub
        .is_none_or(|s| (1..=3).contains(&s.len()) && s.bytes().all(|b| b.is_ascii_alphanumeric()));
    if country_ok && sub_ok {
        Ok(upper)
    } else {
        Err(format!(
            "eyes.jev.region {region:?} must be a region code such as US or US-CA"
        ))
    }
}

/// The notice level for `region` under `table`: the subdivision's entry,
/// else the country's, else quiet.
pub fn notice_level(region: Option<&str>, table: &[(&str, NoticeLevel)]) -> NoticeLevel {
    let Some(region) = region.and_then(|r| normalize_region(r).ok()) else {
        return NoticeLevel::Quiet;
    };
    let find = |code: &str| table.iter().find(|(c, _)| *c == code).map(|(_, l)| *l);
    let country = region.split('-').next().unwrap_or("");
    find(&region)
        .or_else(|| find(country))
        .unwrap_or(NoticeLevel::Quiet)
}

/// The owner's switch as the file holds it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Switch {
    /// No switch file.
    Unset,
    /// `on`.
    On,
    /// `off`, or a file whose content is not `on`.
    Off,
}

impl Switch {
    /// The switch as the status file spells it.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unset => "default",
            Self::On => "on",
            Self::Off => "off",
        }
    }
}

/// The switch file in `state_dir`.
pub fn switch_path(state_dir: &Path) -> PathBuf {
    state_dir.join("jev.switch")
}

/// The notice marker in `state_dir`.
pub fn notice_path(state_dir: &Path) -> PathBuf {
    state_dir.join("jev.notice")
}

/// The status file in `state_dir`.
pub fn status_path(state_dir: &Path) -> PathBuf {
    state_dir.join("jev.status.json")
}

/// Read the switch. A file that cannot be read, or holds anything but `on`
/// or `off`, reads as [`Switch::Off`].
pub fn read_switch(state_dir: &Path) -> Switch {
    match std::fs::read_to_string(switch_path(state_dir)) {
        Ok(text) if text.trim() == "on" => Switch::On,
        Ok(_) => Switch::Off,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Switch::Unset,
        Err(_) => Switch::Off,
    }
}

/// Write the switch.
///
/// # Errors
///
/// The I/O error.
pub fn write_switch(state_dir: &Path, on: bool) -> std::io::Result<()> {
    std::fs::create_dir_all(state_dir)?;
    write_atomic(&switch_path(state_dir), if on { "on\n" } else { "off\n" })
}

/// Whether the owner's switch and the notice level leave Jev on.
pub fn switched_on(switch: Switch, level: NoticeLevel) -> bool {
    match switch {
        Switch::On => true,
        Switch::Off => false,
        Switch::Unset => level != NoticeLevel::OptIn,
    }
}

/// The notice line for a route whose endpoint host is `host`.
pub fn notice_line(host: &str) -> String {
    let route = match host {
        "openrouter.ai" => String::from("TypeSafe, through OpenRouter"),
        "api.typesafe.ai" => String::from("TypeSafe"),
        other => format!("TypeSafe, through {other}"),
    };
    format!(
        "RCP checks wrist-camera frames with Jev ({route}) before a press, so those frames \
         leave this robot; turn it off with `cerulion-rcpd jev off` (details: \
         https://github.com/cerulion-inc/cerulion/blob/main/docs/rcp.md#jev-the-gates-second-eye)"
    )
}

/// What to print at start, if anything: `(warn, line)`. The data notice is
/// logged at warn, the once-only line included, so a warn filter still shows
/// it; the line that Jev waits for `jev on` is logged at info.
pub fn startup_notice(
    state_dir: &Path,
    level: NoticeLevel,
    on: bool,
    host: &str,
) -> Option<(bool, String)> {
    if !on {
        return (level == NoticeLevel::OptIn).then(|| {
            (
                false,
                String::from(
                    "Jev is off on this robot until its owner runs `cerulion-rcpd jev on`",
                ),
            )
        });
    }
    match level {
        NoticeLevel::Clear => Some((true, notice_line(host))),
        NoticeLevel::Quiet | NoticeLevel::OptIn => {
            let marker = notice_path(state_dir);
            if marker.exists() {
                return None;
            }
            if let Err(e) =
                std::fs::create_dir_all(state_dir).and_then(|()| write_atomic(&marker, "shown\n"))
            {
                tracing::debug!(error = %e, "notice marker not written");
            }
            Some((true, notice_line(host)))
        }
    }
}

/// The status file's content.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct JevStatus {
    /// Whether Jev is sending: switched on and a key on file.
    pub on: bool,
    /// The switch: `on`, `off` or `default`.
    pub switch: String,
    /// `present`, or the credential problem's code.
    pub credential: String,
    /// The endpoint.
    pub route: String,
    /// The pinned model.
    pub model: String,
    /// The notice level.
    pub notice: String,
    /// The UTC day the counts are for, `YYYY-MM-DD`.
    pub day: String,
    /// Asks answered today.
    pub asks_today: u64,
    /// Requests sent today.
    pub looks_today: u64,
    /// Asks from a replay today: no request, no answer, no receipt.
    #[serde(default)]
    pub replay_asks_today: u64,
    /// Asks dropped today over the ask bound or from a publisher whose node
    /// cannot be named: no answer, no receipt.
    #[serde(default)]
    pub dropped_asks_today: u64,
    /// Whether a rate, in-flight or spend limit was met today.
    pub limit_reached_today: bool,
    /// The newest answer's outcome.
    pub last_outcome: Option<String>,
}

/// Read the status file.
pub fn read_status(state_dir: &Path) -> Option<JevStatus> {
    let text = std::fs::read_to_string(status_path(state_dir)).ok()?;
    serde_json::from_str(&text).ok()
}

/// Write the status file by replacing it.
///
/// # Errors
///
/// The I/O error.
pub fn write_status(state_dir: &Path, status: &JevStatus) -> std::io::Result<()> {
    let text = serde_json::to_string_pretty(status).unwrap_or_default();
    write_atomic(&status_path(state_dir), &text)
}

/// Write `text` to a temporary file beside `path`, then rename it over
/// `path`.
fn write_atomic(path: &Path, text: &str) -> std::io::Result<()> {
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, path)
}

/// Today's UTC date, `YYYY-MM-DD`.
pub fn utc_day(now: SystemTime) -> String {
    let days = now
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() / 86_400);
    let (y, m, d) = civil_from_days(i64::try_from(days).unwrap_or(0));
    format!("{y:04}-{m:02}-{d:02}")
}

/// The proleptic Gregorian date `days` after 1970-01-01.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    (
        year,
        u32::try_from(month).unwrap_or(1),
        u32::try_from(day).unwrap_or(1),
    )
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, UNIX_EPOCH};

    use super::{
        normalize_region, notice_level, read_switch, startup_notice, switched_on, utc_day,
        write_switch, NoticeLevel, Switch,
    };

    #[test]
    fn a_region_the_table_does_not_list_is_quiet_and_listed_ones_follow_the_table() {
        assert_eq!(notice_level(None, &[]), NoticeLevel::Quiet);
        assert_eq!(notice_level(Some("de"), &[]), NoticeLevel::Quiet);
        let table = [("DE", NoticeLevel::OptIn), ("US-CA", NoticeLevel::Clear)];
        assert_eq!(notice_level(Some("de"), &table), NoticeLevel::OptIn);
        assert_eq!(notice_level(Some("US-CA"), &table), NoticeLevel::Clear);
        assert_eq!(notice_level(Some("US-NY"), &table), NoticeLevel::Quiet);
        assert_eq!(notice_level(Some("DE-BY"), &table), NoticeLevel::OptIn);
        assert!(normalize_region("USA").is_err());
        assert!(normalize_region("us-").is_err());
    }

    #[test]
    fn the_switch_turns_jev_off_at_once_and_opt_in_waits_for_on() {
        let dir = tempfile::tempdir().expect("tempdir");
        assert_eq!(read_switch(dir.path()), Switch::Unset);
        assert!(switched_on(Switch::Unset, NoticeLevel::Quiet));
        assert!(!switched_on(Switch::Unset, NoticeLevel::OptIn));
        write_switch(dir.path(), false).expect("off");
        assert_eq!(read_switch(dir.path()), Switch::Off);
        write_switch(dir.path(), true).expect("on");
        assert_eq!(read_switch(dir.path()), Switch::On);
        assert!(switched_on(Switch::On, NoticeLevel::OptIn));
        std::fs::write(dir.path().join("jev.switch"), "maybe").expect("write");
        assert_eq!(read_switch(dir.path()), Switch::Off);
    }

    #[test]
    fn the_quiet_line_prints_once_and_the_clear_line_every_start() {
        let dir = tempfile::tempdir().expect("tempdir");
        let first = startup_notice(dir.path(), NoticeLevel::Quiet, true, "openrouter.ai");
        let (warn, line) = first.expect("printed once");
        assert!(warn, "the once-only line is logged at warn");
        assert!(line.contains("cerulion-rcpd jev off"), "{line}");
        assert!(
            line.contains("docs/rcp.md#jev-the-gates-second-eye"),
            "{line}"
        );
        assert!(line.contains("through OpenRouter"), "{line}");
        assert!(
            !line
                .chars()
                .any(|c| matches!(u32::from(c), 0x2013 | 0x2014)),
            "no en or em dash: {line}"
        );
        assert_eq!(
            startup_notice(dir.path(), NoticeLevel::Quiet, true, "openrouter.ai"),
            None
        );
        for _ in 0..2 {
            assert!(matches!(
                startup_notice(dir.path(), NoticeLevel::Clear, true, "openrouter.ai"),
                Some((true, _))
            ));
        }
        assert!(startup_notice(dir.path(), NoticeLevel::OptIn, false, "openrouter.ai").is_some());
        assert_eq!(
            startup_notice(dir.path(), NoticeLevel::Quiet, false, "openrouter.ai"),
            None
        );
    }

    #[test]
    fn the_utc_day_is_the_calendar_date() {
        assert_eq!(utc_day(UNIX_EPOCH), "1970-01-01");
        let nov_4_2026 = UNIX_EPOCH + Duration::from_secs(1_793_750_400 + 3_600);
        assert_eq!(utc_day(nov_4_2026), "2026-11-04");
        let leap = UNIX_EPOCH + Duration::from_secs(951_782_400);
        assert_eq!(utc_day(leap), "2000-02-29");
    }
}
