// SPDX-License-Identifier: AGPL-3.0-only
//! The property guard: the last line between a caller and a PII leak.
//!
//! Every caller-provided property passes through [`filter`] before it is
//! serialized. A property is DROPPED (never truncated or redacted, a partial
//! secret is still a secret) and counted when:
//!
//! * its key is not in the event's [`Allowlist`], or
//! * its string value looks like a URL (`http://` / `https://`, any case),
//!   contains `@` (emails, `user@host`), contains a path separator (`/`, `\`),
//!   or is longer than [`MAX_STR_LEN`] chars.
//!
//! Integers and booleans always pass the value check; floats must be finite
//! (NaN/±inf would serialize as JSON `null`). Reserved keys, `$`-prefixed and
//! the [`RESERVED_KEYS`] the client stamps from `Common`, are never accepted
//! from callers even if an allowlist names them: the client owns them.
//!
//! Identifiers are stricter still: [`check_id`] accepts only the two shapes
//! the identity contract defines, a lowercase hyphenated UUID (the Supabase
//! `sub`) or `anon:<lowercase uuid>`, so a static catch-all such as
//! `"unknown"` or `"cli"` can never become a `distinct_id`.
//!
//! Event names go through [`check_event_name`]: snake_case ASCII
//! (`[a-z][a-z0-9_]*`, at most [`MAX_EVENT_NAME_LEN`] chars), so a name can
//! never smuggle an email, path or free text into PostHog's `event` field.

use crate::{Allowlist, Props, Value};
use std::sync::atomic::{AtomicU64, Ordering};

/// Longest string value the guard lets through.
pub const MAX_STR_LEN: usize = 128;

/// Longest event name the guard lets through.
pub const MAX_EVENT_NAME_LEN: usize = 64;

/// Property names the client sets on every event; a caller property with one
/// of these names is dropped so it can never spoof process metadata.
pub const RESERVED_KEYS: &[&str] = &["surface", "env", "app_version", "channel"];

/// Why a property was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rejection {
    KeyNotAllowed,
    LooksLikeUrl,
    ContainsAt,
    ContainsPathSeparator,
    TooLong,
    NonFiniteFloat,
    Empty,
    NotAnId,
    NotAnEventName,
}

/// Process-wide count of dropped properties, observable for tests and a
/// future `cerulion telemetry status --verbose`.
static DROPPED: AtomicU64 = AtomicU64::new(0);

/// Total properties and identifiers dropped by [`filter`] / [`check_id`]
/// since process start.
pub fn dropped_count() -> u64 {
    DROPPED.load(Ordering::Relaxed)
}

/// Prefix of every anonymous identifier.
pub const ANON_PREFIX: &str = "anon:";

/// Check an identifier (`distinct_id`, `anon_id`, `sub`): it must be a
/// lowercase hyphenated UUID or `anon:` followed by one. Anything else,
/// empty, an email, a static sentinel like `"unknown"`, is rejected and
/// counted like a dropped property.
pub fn check_id(id: &str) -> Result<(), Rejection> {
    let result = if id.is_empty() {
        Err(Rejection::Empty)
    } else if let Err(why) = check_str(id) {
        Err(why)
    } else if is_lowercase_uuid(id.strip_prefix(ANON_PREFIX).unwrap_or(id)) {
        Ok(())
    } else {
        Err(Rejection::NotAnId)
    };
    if result.is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    result
}

/// Check an event name: non-empty snake_case ASCII starting with a letter,
/// at most [`MAX_EVENT_NAME_LEN`] bytes. Anything else drops the whole
/// event and counts once.
pub fn check_event_name(name: &str) -> Result<(), Rejection> {
    let result = if name.is_empty() {
        Err(Rejection::Empty)
    } else if name.len() > MAX_EVENT_NAME_LEN {
        Err(Rejection::TooLong)
    } else if name.as_bytes()[0].is_ascii_lowercase()
        && name
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_')
    {
        Ok(())
    } else {
        Err(Rejection::NotAnEventName)
    };
    if result.is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    result
}

/// Check one string value against the value rules (no key check).
pub fn check_str(value: &str) -> Result<(), Rejection> {
    if value.chars().count() > MAX_STR_LEN {
        return Err(Rejection::TooLong);
    }
    if looks_like_url(value) {
        return Err(Rejection::LooksLikeUrl);
    }
    if value.contains('@') {
        return Err(Rejection::ContainsAt);
    }
    if value.contains(['/', '\\']) {
        return Err(Rejection::ContainsPathSeparator);
    }
    Ok(())
}

/// Check one property (key + value) against an allowlist.
pub fn check(key: &str, value: &Value, allowlist: Allowlist) -> Result<(), Rejection> {
    if key.starts_with('$') || RESERVED_KEYS.contains(&key) || !allowlist.contains(&key) {
        return Err(Rejection::KeyNotAllowed);
    }
    match value {
        Value::Str(s) => check_str(s),
        Value::Float(f) if !f.is_finite() => Err(Rejection::NonFiniteFloat),
        Value::Bool(_) | Value::Int(_) | Value::Float(_) => Ok(()),
    }
}

/// Keep the properties that pass; drop, count and report the rest.
pub fn filter(props: Props, allowlist: Allowlist) -> (Props, Vec<(String, Rejection)>) {
    let mut kept = Vec::with_capacity(props.len());
    let mut dropped = Vec::new();
    for (key, value) in props {
        match check(&key, &value, allowlist) {
            Ok(()) => kept.push((key, value)),
            Err(why) => dropped.push((key, why)),
        }
    }
    if !dropped.is_empty() {
        DROPPED.fetch_add(dropped.len() as u64, Ordering::Relaxed);
    }
    (kept, dropped)
}

/// `8-4-4-4-12` lowercase hex, exactly as Supabase and `Uuid::hyphenated()`
/// render it; uppercase or braces are not normalized, they are rejected.
fn is_lowercase_uuid(s: &str) -> bool {
    s.len() == 36
        && s.bytes().enumerate().all(|(i, b)| match i {
            8 | 13 | 18 | 23 => b == b'-',
            _ => b.is_ascii_digit() || (b'a'..=b'f').contains(&b),
        })
}

fn looks_like_url(value: &str) -> bool {
    value
        .as_bytes()
        .windows(8)
        .any(|w| w.eq_ignore_ascii_case(b"https://"))
        || value
            .as_bytes()
            .windows(7)
            .any(|w| w.eq_ignore_ascii_case(b"http://"))
}
