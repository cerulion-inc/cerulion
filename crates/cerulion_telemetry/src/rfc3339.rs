// SPDX-License-Identifier: AGPL-3.0-only
//! `SystemTime` to `YYYY-MM-DDTHH:MM:SS.mmmZ` without a date-time dependency.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Whether `s` has exactly the `YYYY-MM-DDTHH:MM:SS.mmmZ` shape [`format()`]
/// produces (ASCII digits in every numeric position, fixed separators) and
/// names a real instant: month 01 to 12, a day that exists in that month,
/// hour 00 to 23, minute and second 00 to 59.
pub fn is_well_formed(s: &str) -> bool {
    const SHAPE: &[u8; 24] = b"dddd-dd-ddTdd:dd:dd.dddZ";
    let shaped = s.len() == SHAPE.len()
        && s.bytes().zip(SHAPE.iter()).all(|(b, &want)| match want {
            b'd' => b.is_ascii_digit(),
            _ => b == want,
        });
    if !shaped {
        return false;
    }
    let num = |range: std::ops::Range<usize>| -> u32 {
        s.as_bytes()[range]
            .iter()
            .fold(0, |n, b| n * 10 + u32::from(b - b'0'))
    };
    let (year, month, day) = (num(0..4), num(5..7), num(8..10));
    (1..=12).contains(&month)
        && (1..=days_in_month(year, month)).contains(&day)
        && num(11..13) <= 23
        && num(14..16) <= 59
        && num(17..19) <= 59
}

fn days_in_month(year: u32, month: u32) -> u32 {
    match month {
        2 if year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400)) => {
            29
        }
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// Millisecond-precision UTC RFC 3339, always with a `Z` suffix. Instants
/// before 1970 format as the epoch: every caller stamps `SystemTime::now()`,
/// and a clock that far off has no meaningful time to report.
pub fn format(t: SystemTime) -> String {
    let since = t.duration_since(UNIX_EPOCH).unwrap_or(Duration::ZERO);
    let secs = since.as_secs();
    let millis = since.subsec_millis();
    let (year, month, day) = civil_from_days((secs / 86_400) as i64);
    let rem = secs % 86_400;
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{millis:03}Z",
        rem / 3600,
        (rem / 60) % 60,
        rem % 60
    )
}

/// Howard Hinnant's `civil_from_days` (proleptic Gregorian, days since 1970-01-01).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_instants_format_as_rfc3339_utc() {
        let cases: &[(u64, u32, &str)] = &[
            (0, 0, "1970-01-01T00:00:00.000Z"),
            (951_782_400, 0, "2000-02-29T00:00:00.000Z"),
            (1_709_164_799, 999, "2024-02-28T23:59:59.999Z"),
            (1_757_451_600, 5, "2025-09-09T21:00:00.005Z"),
            (4_102_444_800, 0, "2100-01-01T00:00:00.000Z"),
        ];
        for (secs, millis, want) in cases {
            let t =
                UNIX_EPOCH + Duration::from_secs(*secs) + Duration::from_millis(u64::from(*millis));
            assert_eq!(format(t), *want, "secs={secs} millis={millis}");
            assert!(is_well_formed(want), "{want}");
        }
    }

    #[test]
    fn impossible_instants_are_not_well_formed() {
        for bad in [
            "2026-13-01T00:00:00.000Z",
            "2026-00-01T00:00:00.000Z",
            "2026-04-31T00:00:00.000Z",
            "2026-02-29T00:00:00.000Z",
            "1900-02-29T00:00:00.000Z",
            "2026-01-00T00:00:00.000Z",
            "2026-01-01T24:00:00.000Z",
            "2026-01-01T00:60:00.000Z",
            "2026-01-01T00:00:60.000Z",
            "2026-99-99T99:99:99.999Z",
        ] {
            assert!(!is_well_formed(bad), "{bad}");
        }
        assert!(is_well_formed("2024-02-29T23:59:59.999Z"));
    }
}
