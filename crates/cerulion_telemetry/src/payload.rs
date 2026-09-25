// SPDX-License-Identifier: AGPL-3.0-only
//! Pure construction of PostHog `/batch` JSON. No I/O, no clock, no randomness:
//! the caller supplies `uuid` and `timestamp` so tests can pin golden bytes.
//!
//! [`Event`] can only be built through its constructors, and every constructor
//! that takes caller properties runs them through [`guard::filter`], so there
//! is no route to a serialized batch that skips the guard.

use crate::guard;
use crate::Common;
use crate::{EventSpec, Props, Value, LIB_NAME, LIB_VERSION};
use serde_json::{json, Map};

/// The `$set_once` person keys any surface may send. Nothing else.
pub const SET_ONCE_ALLOWLIST: crate::Allowlist = &[
    "created_at",
    "first_signed_in_at",
    "first_surface",
    "first_cli_login_at",
    "first_workspace_at",
];

/// One event, already guarded, waiting for the worker.
#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    name: String,
    distinct_id: String,
    uuid: String,
    timestamp: String,
    kind: Kind,
}

#[derive(Debug, Clone, PartialEq)]
enum Kind {
    Capture { props: Props },
    CaptureAnonymous { props: Props },
    Alias { alias: String },
    SetOnce { props: Props },
}

impl Event {
    /// A regular event for a known person; `props` are guarded by `spec.allowlist`.
    /// `None` (dropped and counted) when `spec.name` or `distinct_id` fails the guard.
    pub fn capture(
        spec: EventSpec,
        distinct_id: &str,
        uuid: String,
        timestamp: String,
        props: Props,
    ) -> Option<Event> {
        guard::check_event_name(spec.name).ok()?;
        let (props, _) = guard::filter(props, spec.allowlist);
        Event::build(
            spec.name,
            distinct_id,
            uuid,
            timestamp,
            Kind::Capture { props },
        )
    }

    /// A regular event for an `anon:` id; sets `$process_person_profile: false`.
    /// `None` (dropped and counted) when `spec.name` or `anon_id` fails the
    /// guard, the id comes from a user-editable file, so it is caller input
    /// like any other property.
    pub fn capture_anonymous(
        spec: EventSpec,
        anon_id: &str,
        uuid: String,
        timestamp: String,
        props: Props,
    ) -> Option<Event> {
        guard::check_event_name(spec.name).ok()?;
        let (props, _) = guard::filter(props, spec.allowlist);
        Event::build(
            spec.name,
            anon_id,
            uuid,
            timestamp,
            Kind::CaptureAnonymous { props },
        )
    }

    /// `$create_alias`: merge `anon_id` into `sub`. `None` (dropped and
    /// counted) when either id fails the guard's value rules.
    pub fn alias(sub: &str, anon_id: &str, uuid: String, timestamp: String) -> Option<Event> {
        guard::check_id(anon_id).ok()?;
        Event::build(
            "$create_alias",
            sub,
            uuid,
            timestamp,
            Kind::Alias {
                alias: anon_id.to_owned(),
            },
        )
    }

    /// `$set` carrying `$set_once` person properties, guarded by
    /// [`SET_ONCE_ALLOWLIST`]. `None` when `sub` fails the guard or nothing
    /// survives it.
    pub fn set_once(sub: &str, uuid: String, timestamp: String, props: Props) -> Option<Event> {
        let (props, _) = guard::filter(props, SET_ONCE_ALLOWLIST);
        guard::check_id(sub).ok()?;
        if props.is_empty() {
            return None;
        }
        Event::build("$set", sub, uuid, timestamp, Kind::SetOnce { props })
    }

    /// Every identifier that ends up in `distinct_id` passes the same value
    /// rules as a property: an email, path or URL there is exactly as much a
    /// leak as in a property. Failing ids drop the event and count once.
    fn build(
        name: &str,
        distinct_id: &str,
        uuid: String,
        timestamp: String,
        kind: Kind,
    ) -> Option<Event> {
        guard::check_id(distinct_id).ok()?;
        Some(Event {
            name: name.to_owned(),
            distinct_id: distinct_id.to_owned(),
            uuid,
            timestamp,
            kind,
        })
    }
}

fn to_json(v: &Value) -> serde_json::Value {
    match v {
        Value::Bool(b) => json!(b),
        Value::Int(i) => json!(i),
        Value::Float(f) => json!(f),
        Value::Str(s) => json!(s),
    }
}

fn insert_props(into: &mut Map<String, serde_json::Value>, props: &Props) {
    for (k, v) in props {
        into.insert(k.clone(), to_json(v));
    }
}

/// Render one event as a `/batch` element.
pub fn event_json(event: &Event, common: &Common) -> serde_json::Value {
    let mut properties = Map::new();
    properties.insert("$lib".into(), json!(LIB_NAME));
    properties.insert("$lib_version".into(), json!(LIB_VERSION));
    properties.insert("surface".into(), json!(common.surface));
    properties.insert("env".into(), json!(common.env));
    properties.insert("app_version".into(), json!(common.app_version));
    if let Some(channel) = &common.channel {
        properties.insert("channel".into(), json!(channel));
    }
    match &event.kind {
        Kind::Capture { props } => insert_props(&mut properties, props),
        Kind::CaptureAnonymous { props } => {
            properties.insert("$process_person_profile".into(), json!(false));
            insert_props(&mut properties, props);
        }
        Kind::Alias { alias } => {
            properties.insert("alias".into(), json!(alias));
        }
        Kind::SetOnce { props } => {
            let mut once = Map::new();
            insert_props(&mut once, props);
            properties.insert("$set_once".into(), serde_json::Value::Object(once));
        }
    }
    json!({
        "event": event.name,
        "distinct_id": event.distinct_id,
        "uuid": event.uuid,
        "timestamp": event.timestamp,
        "properties": properties,
    })
}

/// Render a whole `/batch` body.
pub fn batch_json(api_key: &str, events: &[Event], common: &Common) -> serde_json::Value {
    json!({
        "api_key": api_key,
        "batch": events.iter().map(|e| event_json(e, common)).collect::<Vec<_>>(),
    })
}
