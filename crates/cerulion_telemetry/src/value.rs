// SPDX-License-Identifier: AGPL-3.0-only
//! Property types shared by both feature states.

/// The keys one event may carry. Anything else is dropped and counted.
pub type Allowlist = &'static [&'static str];

/// An event name plus the properties it is allowed to carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EventSpec {
    /// snake_case, past tense outcome (`cli_command_run`, `node_build_failed`).
    pub name: &'static str,
    /// Caller keys; the common properties are added by the sender.
    pub allowlist: Allowlist,
}

/// A coarse property value. There is deliberately no nested object or array
/// variant: every property is a scalar the guard can inspect.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Value::Bool(v)
    }
}

impl From<i64> for Value {
    fn from(v: i64) -> Self {
        Value::Int(v)
    }
}

impl From<u32> for Value {
    fn from(v: u32) -> Self {
        Value::Int(i64::from(v))
    }
}

impl From<f64> for Value {
    fn from(v: f64) -> Self {
        Value::Float(v)
    }
}

impl From<&str> for Value {
    fn from(v: &str) -> Self {
        Value::Str(v.to_owned())
    }
}

impl From<String> for Value {
    fn from(v: String) -> Self {
        Value::Str(v)
    }
}

/// Caller properties for one event, in insertion order.
pub type Props = Vec<(String, Value)>;

/// Properties every event carries, fixed for the process. Each string must
/// pass the guard's value rules, or the sender refuses to start: a bad
/// `app_version` disables telemetry rather than leaking.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Common {
    /// `cli`, `vizd`, `studio`.
    pub surface: String,
    /// `dev`, `prod`.
    pub env: String,
    /// The binary's own version.
    pub app_version: String,
    /// Release channel where one exists (`stable`, `nightly`).
    pub channel: Option<String>,
}
