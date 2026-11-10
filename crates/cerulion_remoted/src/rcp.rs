// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion/rcp/1` plane: an MCP session between a desk on the robot's
//! access list and the robot's local `cerulion-rcpd`.
//!
//! ## Wire
//!
//! Each bidi stream the desk opens carries ONE MCP session. Every frame
//! ([`cerulion_link::write_frame`]) is one JSON-RPC 2.0 message, desk to robot
//! and robot to desk. The robot side is an MCP streamable HTTP client of
//! `cerulion-rcpd` on loopback: each admitted desk message becomes one `POST`
//! carrying the rcpd bearer token, the session's `Mcp-Session-Id` and
//! `MCP-Protocol-Version` once rcpd has issued them, and the
//! [`CALLER_HEADER`] naming the desk's device key and account. Every JSON-RPC
//! message rcpd answers with (a JSON body, or each `data:` event of an SSE
//! body) goes back to the desk as one frame. After `notifications/initialized`
//! the plane also opens rcpd's standalone `GET` event stream, when rcpd
//! serves one, and relays its messages.
//!
//! ## Authorization
//!
//! [`rcp_authorize`] is the one authorization entry point. At accept it
//! requires a claimed robot and a key whose account is on the access list.
//! Then EVERY desk message is parsed by [`classify_message`] and checked by
//! [`rcp_authorize`] for its class against the LIVE trust state before it is
//! forwarded. Before each frame goes back to the desk, the class that produced it is checked
//! again, and the session floor ([`RcpClass::Session`]) is checked every
//! [`RcpLimits::revoke_check`] as well; a key that fails the floor has its
//! connection closed.
//!
//! The e-stop takes no Observe/Act slot; its own bound is
//! [`RcpLimits::estops_in_flight`]. A notification takes no slot. Requests
//! past either bound are answered with an error, never queued. A task of its
//! own reads the stream's frames, and the session reader never waits on rcpd
//! or on the outbound queue: it hands each admitted message to the stream's
//! forwarder task, which sends them to rcpd in the order they were admitted
//! and waits for a lifecycle message's response head, and for the
//! `initialize` result, within [`RcpLimits::head_timeout`], before the next.
//! Each Observe, Act and Estop request is sent by a task of its own. Once its
//! connection to rcpd is open, just before it sends an Observe or Act message,
//! that task checks the key's access for the message's class again, and for
//! an Act message the e-stop latch. A `notifications/cancelled` naming a
//! request whose response head is not in yet waits for that head, so it
//! reaches rcpd after rcpd has taken the tool call it names, or once
//! `--rcp-head-timeout-ms` ([`RcpLimits::head_timeout`]) passes. Queued
//! notifications have [`FORWARD_NOTIFICATION_ROOM`] places of their own; a
//! notification past them is dropped, so a request that holds a slot always
//! fits the queue. The forwarder stops when the stream's writer has stopped
//! and ends with its session; an error reply the outbound queue cannot take
//! is dropped.
//!
//! ## The e-stop latch
//!
//! The plane obeys the robot's one e-stop latch, the
//! `cerud::lease::ControlLease` that the ops plane's `engage-estop` engages
//! and its act verbs check, through the same [`EstopLease`] they share. An
//! admitted `estop` message engages that latch through
//! [`EstopLease::engage`] before it is forwarded, so an unreachable rcpd or a
//! spent e-stop bound still leaves it engaged, and the engage is counted
//! before it waits for the lease's lock, so an act verb sends nothing more
//! once it is counted. While the latch is engaged or an engage is counted,
//! every Act message is refused with [`DENIED_CODE`] before rcpd
//! ([`ESTOP_ENGAGED_REFUSAL`]), and `estop_clear` with
//! [`ESTOP_CLEAR_REFUSAL`]; Session, Observe and Estop messages still pass.
//! The latch is read when a desk's message arrives and again on the open
//! connection to rcpd just before the plane sends it. rcpd's own latch closes
//! on an `estop` tool call, so an ops `engage-estop` on the daemon also sends
//! one into rcpd once it has engaged the latch, again until rcpd answers (see
//! [the ops e-stop in rcpd](#the-ops-e-stop-in-rcpd)): rcpd then refuses each
//! mutating call it holds and has not yet published, and the call it is
//! running, already published, runs on. The latch clears only when
//! `cerulion-remoted` restarts. While it is clear, `estop_clear` reaches rcpd
//! as any tool call does.
//!
//! ## The ops e-stop in rcpd
//!
//! For each ops `engage-estop`, the daemon's `engage-estop` verb sends rcpd
//! an [`ESTOP_TOOL`] call over the plane's [`RcpLink`] and sends it again
//! until rcpd answers that its latch closed. The delivery runs on a task of
//! its own, started after the latch is engaged; `engage-estop` answers
//! without waiting for it, and its outcome changes neither the answer nor
//! the latch.
//!
//! Each attempt opens an MCP session of its own: it reads the token file
//! with its mode check, sends `initialize`, `notifications/initialized` and
//! the `estop` call with the `reason` [`OPS_ESTOP_REASON`], each response head
//! within [`RcpLimits::head_timeout`] and each answer within 10 s after it,
//! and then deletes the session. [`CALLER_HEADER`] names the engager as
//! [`caller_identity`] does, so rcpd receipts and journals the `estop` under
//! the same identity as that desk's rcp calls. An attempt succeeds when the
//! call's result says `"estopped": true`. It fails when rcpd is unreachable,
//! the token file fails its check, rcpd answers with an error, an HTTP error
//! status or a 404 for the session, the result does not say `estopped`, or
//! no head or answer comes in time. After a failure the delivery waits
//! 100 ms, twice as long after each further failure, at most 2 s, and tries
//! again. The first failure is logged at error with its reason, then at most
//! one line per 30 s with the attempt count; the closed latch is logged at
//! warn. A delivery ends early, logged at warn, when the daemon stops.
//!
//! One delivery is in flight at a time: an engage while one is in flight
//! starts none and logs that, since the delivery in flight sends until rcpd
//! answers; an engage after it ended starts a new one. An `estop` over this
//! plane reaches rcpd in its own stream and starts no delivery. A library
//! host has no rcp link and refuses `engage-estop`.
//!
//! A request sent with rcpd's session id that rcpd answers with HTTP `404`
//! (rcpd restarted and no longer knows the session) is answered with
//! [`UPSTREAM_CODE`]. The reader then reads on until the desk ends its side
//! or [`RcpLimits::drain`] passes, and sends nothing more to rcpd: an
//! authorized `estop` engages the ops latch and is answered with
//! [`UPSTREAM_CODE`] and a message that says only the ops latch is engaged,
//! every other request gets the session-lost error, and a notification is
//! dropped. Then the stream ends. A response body that ends without the
//! response to its request is answered with [`UPSTREAM_CODE`] too.
//!
//! | Method | Class | Requirement ([`requirement`]) |
//! |---|---|---|
//! | `initialize`, `ping`, `notifications/initialized`, `notifications/cancelled` | Session | any account on the access list |
//! | `resources/list`, `resources/read`, `resources/templates/list`, `resources/subscribe`, `resources/unsubscribe`, `subscriptions/listen`, `tools/list` | Observe | `CAP_OBSERVE` and role value <= `VIEWER` |
//! | `tools/call` with `params.name` `estop` | Estop | any account on the access list (the e-stop floor) |
//! | `tools/call`, any other tool | Act | `CAP_TELEOP` and role value <= `OPERATOR` |
//! | anything else, a batch, a response, or any message that fails the strict parse | none | denied |
//!
//! A denied request is answered with a JSON-RPC error carrying its id; a
//! message whose id cannot be read is answered with id `null`; a denied
//! notification gets no answer. Nothing denied reaches rcpd. The plane adds
//! and removes no rcpd tool limit: what rcpd refuses, it refuses.
//!
//! ## The rcpd link
//!
//! [`RcpLink`] names rcpd's loopback address and the robot-local token file.
//! The file holds the token rcpd reads from `CERULION_RCP_TOKEN`, either bare
//! or as one `CERULION_RCP_TOKEN=<token>` line, so the same file can be a
//! systemd `EnvironmentFile=` for rcpd. The daemon runs [`ensure_token_file`]
//! at startup through [`prepare`], which logs a failure at error and lets the
//! daemon serve on; the token is read, with the same mode check, when each
//! session opens, and a session that cannot read it answers every request
//! with [`UPSTREAM_CODE`]. The link must be a loopback address: [`prepare`]
//! refuses any other.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use cerulion_link::{accept_frame_stream, read_frame, write_frame, Connection, SendStream};
use cerulion_pairing::format::{AccountId, Role, Scope};
use cerulion_pairing::verify::AccessRow;
use http_body_util::{BodyExt, Empty, Full};
use hyper::body::{Bytes, Incoming};
use hyper::header::{HeaderValue, ACCEPT, AUTHORIZATION, CONTENT_TYPE, HOST};
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use serde_json::{json, Map, Value};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, watch, Notify, OwnedSemaphorePermit, Semaphore};
use tokio::task::JoinSet;

use crate::estop_lease::EstopLease;
use crate::trust::{KeyAccess, SharedTrust};

/// The header naming the desk's device key and account on each request to rcpd.
pub const CALLER_HEADER: &str = "x-cerulion-caller";

/// The environment variable rcpd reads its token from; also the optional
/// prefix of a token file's line.
pub const TOKEN_ENV: &str = "CERULION_RCP_TOKEN";

/// rcpd's default `--http` address, 127.0.0.1:7801.
pub const DEFAULT_RCPD_ADDR: SocketAddr = SocketAddr::V4(std::net::SocketAddrV4::new(
    std::net::Ipv4Addr::LOCALHOST,
    7801,
));

/// The largest desk message accepted, bytes: 16 times rcpd's own largest
/// call frame (64 KiB), so the bound never refuses a call rcpd would take.
pub const MAX_REQUEST_FRAME: usize = 1024 * 1024;

/// The largest message relayed back to the desk, bytes: the link's frame cap,
/// the most one frame can carry.
pub const MAX_RESPONSE_MESSAGE: usize = cerulion_link::DEFAULT_MAX_FRAME_LEN;

/// The QUIC close code of a connection whose key lost rcp access.
pub const CLOSE_REVOKED: u32 = 0x5243_0001;

// 0x5243_0002 is RETIRED (it meant "too many rcp connections for this key",
// a refusal at the per-key cap); it is never reissued.

/// The QUIC close code of a key's oldest connection, closed when a newer one
/// from the same key would pass [`RcpLimits::connections_per_key`].
pub const CLOSE_SUPERSEDED: u32 = 0x5243_0003;

/// The JSON-RPC error code of a message the plane refuses to forward.
pub const DENIED_CODE: i64 = -32001;

/// The JSON-RPC error code of a message that fails the strict parse.
pub const INVALID_REQUEST_CODE: i64 = -32600;

/// The JSON-RPC error code of a request rcpd could not be reached for.
pub const UPSTREAM_CODE: i64 = -32603;

/// How long a TCP connect to rcpd may take. rcpd is on loopback, where a
/// connect either completes at once or is refused, so this only bounds a
/// wedged host; not tunable. [`RcpdEstop::shut_down`] marks the daemon
/// stopping, which ends a delivery in flight, then waits this long for the
/// delivery's task to end; the delivery gets no more time to reach rcpd.
pub(crate) const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// Frames queued for the desk per session; the byte budget
/// ([`RcpLimits::session_buffer_bytes`]) is the bound that matters, this only
/// caps the count of tiny frames.
const OUTBOUND_QUEUE: usize = 32;

/// The places for notifications in a stream's forward queue, past its request
/// slots ([`RcpLimits::in_flight`] plus [`RcpLimits::estops_in_flight`]): a
/// notification takes no slot but holds one of these places while it is
/// queued and sent.
pub const FORWARD_NOTIFICATION_ROOM: usize = 4;

/// The refusal of a request the stream cannot take: its slots, or its
/// forward queue, are full.
const TOO_MANY_IN_FLIGHT: &str = "refused: too many requests in flight on this rcp stream";

/// The rcp plane's bounds and timers. Each default states its reason; each is
/// a `cerulion-remoted` flag ([`RcpFlags`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcpLimits {
    /// Concurrent sessions (bidi streams) per connection
    /// (`--rcp-sessions-per-connection`, default 4): one MCP client uses one;
    /// a few allow a reconnect overlapping its predecessor.
    pub sessions_per_connection: usize,
    /// Concurrent rcp connections per device key
    /// (`--rcp-connections-per-key`, default 4): one desk runs a few MCP
    /// clients at most; the cap stops one key exhausting robot sockets. A
    /// connection past it closes the key's oldest, so a desk that crashed is
    /// never locked out.
    pub connections_per_key: usize,
    /// Concurrent in-flight Observe, Act and lifecycle requests per session
    /// (`--rcp-in-flight`, default 8): rcpd serializes tool calls, so more than
    /// a few in flight only queue inside rcpd. A request past it is answered
    /// with an error, never queued.
    pub in_flight: usize,
    /// Concurrent e-stop calls per session (`--rcp-estops-in-flight`, default
    /// 2): rcpd latches on the first, so a second only covers a lost answer.
    pub estops_in_flight: usize,
    /// Bytes queued for the desk per session before producers wait
    /// (`--rcp-session-buffer-bytes`, default 32 MiB): two of the largest
    /// relayable messages ([`MAX_RESPONSE_MESSAGE`]).
    pub session_buffer_bytes: usize,
    /// How often a session re-checks the session floor with no traffic
    /// (`--rcp-revoke-check-ms`, default 5000): the longest a revoked, idle
    /// desk keeps its connection; a frame sent or answered is checked at once.
    pub revoke_check: Duration,
    /// How long rcpd may take to send a response head (`--rcp-head-timeout-ms`,
    /// default 30000): rcpd sends a streamed head before running the tool, so
    /// this bounds only a wedged rcpd, never a long tool call.
    pub head_timeout: Duration,
    /// How long forwarded requests may still answer after the desk ends its
    /// side of the stream (`--rcp-drain-ms`, default 30000). The drain starts
    /// once the forwarder is done: it has sent every message the reader
    /// admitted, or handed it to a task that sends or refuses it, or it
    /// stopped when the stream's writer stopped. The drain matches the head
    /// bound, so a request sent just before the end is not cut off. When rcpd
    /// lost the stream's session, the drain runs from the loss, and the reader
    /// reads on within it.
    pub drain: Duration,
}

impl Default for RcpLimits {
    fn default() -> Self {
        Self {
            sessions_per_connection: 4,
            connections_per_key: 4,
            in_flight: 8,
            estops_in_flight: 2,
            session_buffer_bytes: 2 * MAX_RESPONSE_MESSAGE,
            revoke_check: Duration::from_secs(5),
            head_timeout: Duration::from_secs(30),
            drain: Duration::from_secs(30),
        }
    }
}

/// The security class of one desk message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcpClass {
    /// Session lifecycle: initialize, ping, initialized, cancelled.
    Session,
    /// Reading the robot: resources, subscriptions, the tool list.
    Observe,
    /// Calling a tool.
    Act,
    /// Calling rcpd's built-in [`ESTOP_TOOL`]: the e-stop floor, any account
    /// on the access list, as the ops plane's `engage-estop`.
    Estop,
}

/// The name of rcpd's built-in e-stop tool.
pub const ESTOP_TOOL: &str = "estop";

/// The name of rcpd's built-in tool that clears its own e-stop latch.
pub const ESTOP_CLEAR_TOOL: &str = "estop_clear";

/// The error message of an Act message refused while the robot's e-stop
/// latch is engaged.
pub const ESTOP_ENGAGED_REFUSAL: &str = "refused: the robot's e-stop is engaged";

/// The error message of an [`ESTOP_CLEAR_TOOL`] call refused while the
/// robot's e-stop latch is engaged.
pub const ESTOP_CLEAR_REFUSAL: &str =
    "refused: the robot's ops e-stop is engaged; it clears when cerulion-remoted restarts";

/// The error message of a request rcpd answered with HTTP 404 for the session
/// id it carried, and of every later request on that stream but `estop`.
const SESSION_LOST: &str = "rcpd lost this MCP session (it restarted); restart this MCP server";

/// The error message of an `estop` read after rcpd lost the stream's session:
/// it engaged the ops latch and was not sent to rcpd.
const ESTOP_SESSION_LOST: &str =
    "rcpd lost this MCP session (it restarted), so this estop did not \
                                  reach it; the robot's ops e-stop is engaged; restart this MCP \
                                  server";

/// What an account's scope must hold for a class: ALL of `required_caps`, and
/// a role value <= `max_role` (a lower [`Role`] value is more privileged).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcpRequirement {
    /// Capability bits that must all be present.
    pub required_caps: u64,
    /// The least privileged role admitted.
    pub max_role: Role,
}

/// The requirement of each class. The single place the rcp policy lives.
pub fn requirement(class: RcpClass) -> RcpRequirement {
    match class {
        RcpClass::Session => RcpRequirement {
            required_caps: 0,
            max_role: Role(u16::MAX),
        },
        RcpClass::Observe => RcpRequirement {
            required_caps: Scope::CAP_OBSERVE,
            max_role: Role::VIEWER,
        },
        RcpClass::Act => RcpRequirement {
            required_caps: Scope::CAP_TELEOP,
            max_role: Role::OPERATOR,
        },
        RcpClass::Estop => RcpRequirement {
            required_caps: 0,
            max_role: Role(u16::MAX),
        },
    }
}

/// Whether `scope` meets `class`'s [`requirement`].
pub fn scope_meets(scope: Scope, class: RcpClass) -> bool {
    let req = requirement(class);
    (scope.caps & req.required_caps) == req.required_caps && scope.role.0 <= req.max_role.0
}

/// THE rcp authorization entry point, used at accept (`class` = `None`) and
/// for every message (`class` = `Some`), over one torn-free read of the
/// robot's trust state: `claimed`, the key's `access`, and the access `row`
/// that allows it (`Some` exactly when `access` is [`KeyAccess::Allowed`]).
///
/// Admits when the robot is claimed, the key is allowed by `row`, and for a
/// message, `row.scope` meets the class's [`requirement`]. A condition on the
/// caller's account or principal kind belongs here, so it holds at accept and
/// on every message at once.
///
/// # Errors
///
/// The refusal reason.
pub fn rcp_authorize(
    claimed: bool,
    access: KeyAccess,
    row: Option<&AccessRow>,
    class: Option<RcpClass>,
) -> Result<(), String> {
    if let Some(reason) = crate::authorizer::cloud_refusal(access) {
        return Err(format!("rcp plane refused: {reason}"));
    }
    if !claimed {
        return Err("rcp plane refused: robot is UNCLAIMED".to_string());
    }
    let row = match (access, row) {
        (KeyAccess::Allowed(_), Some(row)) => row,
        (KeyAccess::Allowed(_), None) => {
            return Err("rcp plane refused: no access row for an allowed key".to_string())
        }
        (KeyAccess::Unpaired, _) => {
            return Err("rcp plane refused: unpaired device key".to_string())
        }
        (KeyAccess::NotAllowed, _) => {
            return Err("rcp plane refused: the account is not on the access list \
                        (revoked, removed, or its owner-signed grant expired)"
                .to_string())
        }
        (KeyAccess::CloudRefused, _) => {
            return Err("rcp plane refused: cloud access is off on this robot".to_string())
        }
        (KeyAccess::DeviceRevoked, _) => {
            return Err(
                "rcp plane refused: this device key is revoked by the current \
                        access-list epoch"
                    .to_string(),
            )
        }
    };
    let Some(class) = class else {
        return Ok(());
    };
    if scope_meets(row.scope, class) {
        Ok(())
    } else {
        let req = requirement(class);
        Err(format!(
            "scope (role {}, caps {:#06x}) lacks the {class:?} requirement (caps {:#06x}, role \
             value <= {})",
            row.scope.role.0, row.scope.caps, req.required_caps, req.max_role.0
        ))
    }
}

/// [`rcp_authorize`] over the LIVE trust state for `device_key`.
///
/// # Errors
///
/// The refusal reason.
pub fn authorize_key(
    shared: &SharedTrust,
    device_key: &[u8; 32],
    class: Option<RcpClass>,
) -> Result<(), String> {
    let (claimed, access, row) = shared.snapshot_row_for_key(device_key);
    rcp_authorize(claimed, access, row.as_ref(), class)
}

/// One desk message that passed [`classify_message`].
#[derive(Debug, Clone, PartialEq)]
pub struct RcpMessage {
    /// Its class.
    pub class: RcpClass,
    /// Its method.
    pub method: String,
    /// Its id; `None` for a notification.
    pub id: Option<Value>,
    /// The message re-serialized from the parsed value: what is forwarded.
    pub canonical: Vec<u8>,
}

/// A desk message [`classify_message`] refused.
#[derive(Debug, Clone, PartialEq)]
pub struct RcpRefusal {
    /// The id to answer with: the message's id when readable, `Null` when not,
    /// `None` for a notification (no answer).
    pub reply_id: Option<Value>,
    /// Why.
    pub reason: String,
}

fn request_class(method: &str) -> Option<RcpClass> {
    Some(match method {
        "initialize" | "ping" => RcpClass::Session,
        "resources/list"
        | "resources/read"
        | "resources/templates/list"
        | "resources/subscribe"
        | "resources/unsubscribe"
        | "subscriptions/listen"
        | "tools/list" => RcpClass::Observe,
        "tools/call" => RcpClass::Act,
        _ => return None,
    })
}

fn notification_class(method: &str) -> Option<RcpClass> {
    match method {
        "notifications/initialized" | "notifications/cancelled" => Some(RcpClass::Session),
        _ => None,
    }
}

fn valid_id(id: &Value) -> bool {
    match id {
        Value::String(_) => true,
        Value::Number(n) => n.is_i64() || n.is_u64(),
        _ => false,
    }
}

/// Parse one desk frame strictly and classify it by its MCP method.
///
/// Admitted: a single JSON object with exactly `"jsonrpc": "2.0"`, a string
/// `method`, an optional `params` object, and for a request an `id` that is a
/// string or an integer; no other top-level key. A `tools/call` must carry a
/// `params.name` string. Anything else is refused, including batches and
/// responses.
///
/// # Errors
///
/// [`RcpRefusal`] naming the reply id and the reason.
pub fn classify_message(frame: &[u8]) -> Result<RcpMessage, RcpRefusal> {
    let unreadable = |reason: &str| RcpRefusal {
        reply_id: Some(Value::Null),
        reason: reason.to_string(),
    };
    let value: Value =
        serde_json::from_slice(frame).map_err(|e| unreadable(&format!("not JSON: {e}")))?;
    let Value::Object(obj) = value else {
        return Err(unreadable(
            "not a single JSON-RPC object (batches are refused)",
        ));
    };
    let id = obj.get("id").cloned();
    if let Some(bad) = id.as_ref().filter(|i| !valid_id(i)) {
        return Err(unreadable(&format!(
            "id must be a string or an integer, got {bad}"
        )));
    }
    let refuse = |reason: String| RcpRefusal {
        reply_id: id.clone(),
        reason,
    };
    if obj.get("jsonrpc") != Some(&Value::String("2.0".into())) {
        return Err(refuse("jsonrpc must be \"2.0\"".into()));
    }
    if let Some(key) = obj
        .keys()
        .find(|k| !matches!(k.as_str(), "jsonrpc" | "id" | "method" | "params"))
    {
        return Err(refuse(format!(
            "unexpected member {key:?} (responses and extensions are refused)"
        )));
    }
    let Some(Value::String(method)) = obj.get("method") else {
        return Err(refuse("method must be a string".into()));
    };
    let params = obj.get("params");
    if params.is_some_and(|p| !p.is_object()) {
        return Err(refuse("params must be an object".into()));
    }
    let class = if id.is_some() {
        request_class(method)
    } else {
        notification_class(method)
    };
    let Some(class) = class else {
        return Err(refuse(format!(
            "method {method:?} is not served on the rcp plane{}",
            if id.is_some() {
                ""
            } else {
                " as a notification"
            }
        )));
    };
    let class = if class == RcpClass::Act {
        match params.and_then(|p| p.get("name")) {
            Some(Value::String(name)) if name == ESTOP_TOOL => RcpClass::Estop,
            Some(Value::String(_)) => RcpClass::Act,
            _ => return Err(refuse("tools/call needs a string params.name".into())),
        }
    } else {
        class
    };
    if let Some(bad) = inexact_number(&Value::Object(obj.clone())) {
        return Err(refuse(format!(
            "the number {bad} is outside the range JSON numbers are forwarded exactly"
        )));
    }
    let canonical = serde_json::to_vec(&Value::Object(obj.clone()))
        .map_err(|e| refuse(format!("cannot re-serialize: {e}")))?;
    Ok(RcpMessage {
        class,
        method: method.clone(),
        id,
        canonical,
    })
}

/// The first number in `value` that re-serialization could change: a float of
/// magnitude 2^53 or more (an integer literal past `u64` parses as one).
fn inexact_number(value: &Value) -> Option<f64> {
    match value {
        Value::Number(n) => n
            .as_f64()
            .filter(|f| n.is_f64() && f.abs() >= 9_007_199_254_740_992.0),
        Value::Array(items) => items.iter().find_map(inexact_number),
        Value::Object(map) => map.values().find_map(inexact_number),
        _ => None,
    }
}

/// A JSON-RPC error message.
pub fn error_frame(id: Value, code: i64, message: &str) -> Vec<u8> {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message } })
        .to_string()
        .into_bytes()
}

/// The caller identity rcpd receipts a forwarded call under.
pub fn caller_identity(device_key: &[u8; 32], account: Option<AccountId>) -> String {
    match account {
        Some(a) => format!(
            "iroh:{}/account:{}",
            hex::encode(device_key),
            hex::encode(a.0)
        ),
        None => format!("iroh:{}", hex::encode(device_key)),
    }
}

/// Read the rcpd token from `path`: the file's trimmed content, minus an
/// optional `CERULION_RCP_TOKEN=` prefix. The file must pass the same checks
/// as [`ensure_token_file`]; the token must be visible ASCII.
///
/// # Errors
///
/// A reason naming the file.
pub fn load_token(path: &Path) -> Result<String, String> {
    let meta = std::fs::symlink_metadata(path)
        .map_err(|e| format!("rcpd token file {}: {e}", path.display()))?;
    check_token_file(path, &meta)?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("rcpd token file {}: {e}", path.display()))?;
    let line = text.trim();
    let token = line
        .strip_prefix(TOKEN_ENV)
        .and_then(|rest| rest.strip_prefix('='))
        .unwrap_or(line);
    if token.is_empty() || !token.bytes().all(|b| b.is_ascii_graphic()) {
        return Err(format!(
            "rcpd token file {} must hold one token of visible ASCII (bare, or \
             {TOKEN_ENV}=<token>)",
            path.display()
        ));
    }
    Ok(token.to_string())
}

/// Check `link` at daemon startup: a loopback rcpd address, then a private
/// token file ([`ensure_token_file`]). A token file that fails the check is
/// logged at error and is not a startup failure: the ops and wire planes
/// serve on, and each rcp session answers its requests with
/// [`UPSTREAM_CODE`] until the file passes.
///
/// # Errors
///
/// The reason the daemon refuses to start: a non-loopback rcpd address.
pub fn prepare(link: &RcpLink) -> Result<(), String> {
    if !link.addr.ip().is_loopback() {
        return Err(format!(
            "--rcp-addr {} is not a loopback address; the rcp plane sends the rcpd token \
             in cleartext HTTP, so rcpd must listen on loopback",
            link.addr
        ));
    }
    if let Err(reason) = ensure_token_file(&link.token_file) {
        tracing::error!(
            %reason,
            "cerulion_remoted: the rcp plane serves no session until the rcpd token file passes \
             its check"
        );
    }
    Ok(())
}

/// Make sure the token file at `path` exists and is private: create it when
/// absent (one `CERULION_RCP_TOKEN=<64 hex>` line from 32 random bytes,
/// created exclusively, mode 0600 on unix, synced), and refuse an existing file
/// that is not a regular file or, on unix, gives its group or others any
/// permission (read, write or execute).
///
/// # Errors
///
/// A reason naming the file.
pub fn ensure_token_file(path: &Path) -> Result<(), String> {
    match std::fs::symlink_metadata(path) {
        Ok(meta) => return check_token_file(path, &meta),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(format!("rcpd token file {}: {e}", path.display())),
    }
    let mut seed = [0u8; 32];
    getrandom::getrandom(&mut seed)
        .map_err(|e| format!("rcpd token file {}: no randomness: {e}", path.display()))?;
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let created = options.open(path).and_then(|mut file| {
        use std::io::Write;
        file.write_all(format!("{TOKEN_ENV}={}\n", hex::encode(seed)).as_bytes())?;
        file.sync_all()
    });
    match created {
        Ok(()) => {
            tracing::info!(path = %path.display(), "cerulion_remoted: created the rcpd token file");
            Ok(())
        }
        // Another process created it first: check what is there.
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
            let meta = std::fs::symlink_metadata(path)
                .map_err(|e| format!("rcpd token file {}: {e}", path.display()))?;
            check_token_file(path, &meta)
        }
        Err(e) => Err(format!(
            "rcpd token file {}: cannot create: {e}",
            path.display()
        )),
    }
}

fn check_token_file(path: &Path, meta: &std::fs::Metadata) -> Result<(), String> {
    if !meta.file_type().is_file() {
        return Err(format!(
            "rcpd token file {} is not a regular file",
            path.display()
        ));
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mode = meta.permissions().mode() & 0o777;
        if mode & 0o077 != 0 {
            return Err(format!(
                "rcpd token file {} has mode {mode:o}; it must be private to its owner \
                 (chmod 600)",
                path.display()
            ));
        }
    }
    Ok(())
}

/// The token file's name in the daemon's state directory.
pub const RCP_TOKEN_FILENAME: &str = "rcp.token";

/// Where the robot's rcpd listens and where its token lives.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RcpLink {
    /// rcpd's `--http` address (loopback).
    pub addr: SocketAddr,
    /// The robot-local token file.
    pub token_file: PathBuf,
    /// The plane's bounds and timers.
    pub limits: RcpLimits,
}

impl RcpLink {
    /// The default link for the daemon state directory `dir`:
    /// [`DEFAULT_RCPD_ADDR`] and `dir/`[`RCP_TOKEN_FILENAME`].
    pub fn under(dir: &Path) -> Self {
        Self {
            addr: DEFAULT_RCPD_ADDR,
            token_file: dir.join(RCP_TOKEN_FILENAME),
            limits: RcpLimits::default(),
        }
    }
}

/// The daemon's rcp flags.
#[derive(Debug, Clone, Default, clap::Args)]
pub struct RcpFlags {
    /// The local cerulion-rcpd `--http` address the rcp plane forwards to.
    /// Default: 127.0.0.1:7801.
    #[arg(long, value_name = "ADDR")]
    pub rcp_addr: Option<SocketAddr>,

    /// Override the file holding the local cerulion-rcpd bearer token.
    /// Default: `<state-root>/remoted/rcp.token`.
    #[arg(long, value_name = "PATH")]
    pub rcp_token_file: Option<PathBuf>,

    /// Concurrent rcp sessions per connection. Default: 4.
    #[arg(long, value_name = "N", value_parser = clap::value_parser!(u32).range(1..))]
    pub rcp_sessions_per_connection: Option<u32>,

    /// Concurrent rcp connections per desk key. Default: 4.
    #[arg(long, value_name = "N", value_parser = clap::value_parser!(u32).range(1..))]
    pub rcp_connections_per_key: Option<u32>,

    /// Concurrent resource and tool requests per rcp session. Default: 8.
    #[arg(long, value_name = "N", value_parser = clap::value_parser!(u32).range(1..))]
    pub rcp_in_flight: Option<u32>,

    /// Concurrent e-stop calls per rcp session. Default: 2.
    #[arg(long, value_name = "N", value_parser = clap::value_parser!(u32).range(1..))]
    pub rcp_estops_in_flight: Option<u32>,

    /// Bytes queued for the desk per rcp session. Default: 33554432.
    #[arg(long, value_name = "BYTES", value_parser = clap::value_parser!(u32).range(16_777_216..))]
    pub rcp_session_buffer_bytes: Option<u32>,

    /// How often an idle rcp session re-checks access, milliseconds (at most
    /// 60000). Default: 5000.
    #[arg(long, value_name = "MS", value_parser = clap::value_parser!(u64).range(1..=60_000))]
    pub rcp_revoke_check_ms: Option<u64>,

    /// How long rcpd may take to send a response head, milliseconds. Default: 30000.
    #[arg(long, value_name = "MS", value_parser = clap::value_parser!(u64).range(1..))]
    pub rcp_head_timeout_ms: Option<u64>,

    /// How long forwarded requests may answer after the desk closes its
    /// stream, milliseconds. Default: 30000.
    #[arg(long, value_name = "MS", value_parser = clap::value_parser!(u64).range(0..))]
    pub rcp_drain_ms: Option<u64>,
}

impl RcpFlags {
    /// Apply the flags given over `link`.
    pub fn apply(self, link: &mut RcpLink) {
        if let Some(a) = self.rcp_addr {
            link.addr = a;
        }
        if let Some(p) = self.rcp_token_file {
            link.token_file = p;
        }
        let l = &mut link.limits;
        let count = |n: u32| usize::try_from(n).unwrap_or(usize::MAX);
        if let Some(n) = self.rcp_sessions_per_connection {
            l.sessions_per_connection = count(n);
        }
        if let Some(n) = self.rcp_connections_per_key {
            l.connections_per_key = count(n);
        }
        if let Some(n) = self.rcp_in_flight {
            l.in_flight = count(n);
        }
        if let Some(n) = self.rcp_estops_in_flight {
            l.estops_in_flight = count(n);
        }
        if let Some(n) = self.rcp_session_buffer_bytes {
            l.session_buffer_bytes = count(n);
        }
        if let Some(ms) = self.rcp_revoke_check_ms {
            l.revoke_check = Duration::from_millis(ms);
        }
        if let Some(ms) = self.rcp_head_timeout_ms {
            l.head_timeout = Duration::from_millis(ms);
        }
        if let Some(ms) = self.rcp_drain_ms {
            l.drain = Duration::from_millis(ms);
        }
    }
}

/// The rcp plane's serving context.
#[derive(Debug)]
pub struct RcpPlane {
    link: RcpLink,
    shared: SharedTrust,
    lease: EstopLease,
    connections: Mutex<HashMap<[u8; 32], Vec<LiveConnection>>>,
    next_connection: std::sync::atomic::AtomicU64,
    #[cfg(any(test, feature = "test-seam"))]
    connect_holds: Mutex<Vec<(Value, Arc<ConnectHold>)>>,
    #[cfg(any(test, feature = "test-seam"))]
    loss_seen: Notify,
}

/// TEST SEAM: a hold on the forward of one request, between its open
/// connection to rcpd and the checks just before it is sent
/// ([`RcpPlane::hold_after_connect_for_test`]). Feature-gated, so it never
/// compiles into the production daemon.
#[doc(hidden)]
#[cfg(any(test, feature = "test-seam"))]
#[derive(Debug, Default)]
pub struct ConnectHold {
    connected: Notify,
    release: Notify,
}

#[cfg(any(test, feature = "test-seam"))]
impl ConnectHold {
    /// Resolves once the held request's connection to rcpd is open.
    pub async fn connected(&self) {
        self.connected.notified().await;
    }

    /// Let the held request go on to its checks and its send.
    pub fn release(&self) {
        self.release.notify_one();
    }
}

/// One admitted rcp connection, as the per-key table holds it.
#[derive(Debug)]
struct LiveConnection {
    id: u64,
    since: std::time::Instant,
    connection: Connection,
}

impl RcpPlane {
    /// A plane forwarding to `link`, authorizing against `shared` (the same
    /// live handle the accept gate reads), under the e-stop latch of `lease`:
    /// a clone of the [`EstopLease`] the ops plane's `engage-estop` and act
    /// verbs use, so the plane's engages are counted where the act verbs read
    /// the count.
    pub fn new(link: RcpLink, shared: SharedTrust, lease: EstopLease) -> Self {
        Self {
            link,
            shared,
            lease,
            connections: Mutex::new(HashMap::new()),
            next_connection: std::sync::atomic::AtomicU64::new(0),
            #[cfg(any(test, feature = "test-seam"))]
            connect_holds: Mutex::new(Vec::new()),
            #[cfg(any(test, feature = "test-seam"))]
            loss_seen: Notify::new(),
        }
    }

    /// The rcpd link.
    pub fn link(&self) -> &RcpLink {
        &self.link
    }

    /// TEST SEAM: resolves once a stream's reader has seen rcpd lose the
    /// stream's MCP session. Feature-gated, so it never compiles into the
    /// production daemon.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub async fn session_loss_seen_for_test(&self) {
        self.loss_seen.notified().await;
    }

    /// TEST SEAM: hold the next forward of the request with JSON-RPC id `id`
    /// once its connection to rcpd is open, before the checks just before it
    /// is sent, until the returned hold is released. Feature-gated, so it
    /// never compiles into the production daemon.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn hold_after_connect_for_test(&self, id: Value) -> Arc<ConnectHold> {
        let hold = Arc::new(ConnectHold::default());
        self.connect_holds
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push((id, hold.clone()));
        hold
    }

    /// The [`RcpPlane::hold_after_connect_for_test`] hold for `id`, taken once.
    #[cfg(any(test, feature = "test-seam"))]
    async fn after_connect_for_test(&self, id: Option<&Value>) {
        let hold = {
            let mut holds = self
                .connect_holds
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            match holds.iter().position(|(held, _)| Some(held) == id) {
                Some(at) => holds.remove(at).1,
                None => return,
            }
        };
        hold.connected.notify_one();
        hold.release.notified().await;
    }

    /// Admit `connection` for `key`. At [`RcpLimits::connections_per_key`]
    /// the key's oldest connection is closed with [`CLOSE_SUPERSEDED`]: the
    /// same key is the same device, so a reconnect after a crash (whose old
    /// connection the robot only drops at the QUIC idle timeout) is admitted
    /// at once.
    fn admit_connection(
        self: &Arc<Self>,
        key: [u8; 32],
        connection: &Connection,
    ) -> ConnectionSlot {
        let id = self
            .next_connection
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let mut map = self
            .connections
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let live = map.entry(key).or_default();
        while live.len() >= self.link.limits.connections_per_key.max(1) {
            let oldest = live.remove(0);
            tracing::warn!(
                key = %hex::encode(&key[..8]),
                age_ms = oldest.since.elapsed().as_millis() as u64,
                "cerulion_remoted: rcp connection superseded by a newer one from the same key"
            );
            oldest.connection.close(
                CLOSE_SUPERSEDED.into(),
                b"superseded by a newer rcp connection from this key",
            );
        }
        live.push(LiveConnection {
            id,
            since: std::time::Instant::now(),
            connection: connection.clone(),
        });
        ConnectionSlot {
            plane: self.clone(),
            key,
            id,
        }
    }
}

/// One admitted connection; removed from the per-key table on drop.
struct ConnectionSlot {
    plane: Arc<RcpPlane>,
    key: [u8; 32],
    id: u64,
}

impl Drop for ConnectionSlot {
    fn drop(&mut self) {
        let mut map = self
            .plane
            .connections
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if let Some(live) = map.get_mut(&self.key) {
            live.retain(|c| c.id != self.id);
            if live.is_empty() {
                map.remove(&self.key);
            }
        }
    }
}

/// Serve one admitted rcp connection from `device_key` until it closes: each
/// bidi stream is one session, at most [`RcpLimits::sessions_per_connection`]
/// at once; past [`RcpLimits::connections_per_key`] a key's oldest connection
/// is closed in favour of this one.
pub async fn serve_rcp_connection(
    connection: Connection,
    device_key: [u8; 32],
    plane: Arc<RcpPlane>,
) {
    let _slot = plane.admit_connection(device_key, &connection);
    let slots = Arc::new(Semaphore::new(plane.link.limits.sessions_per_connection));
    let mut sessions = JoinSet::new();
    loop {
        while sessions.try_join_next().is_some() {}
        let Ok(permit) = slots.clone().acquire_owned().await else {
            break;
        };
        let (send, recv) = match accept_frame_stream(&connection).await {
            Ok(pair) => pair,
            Err(e) => {
                tracing::debug!(error = %e, "cerulion_remoted: rcp connection ended");
                break;
            }
        };
        let plane = plane.clone();
        let conn = connection.clone();
        sessions.spawn(async move {
            serve_session(send, recv, conn, device_key, plane).await;
            drop(permit);
        });
    }
    sessions.shutdown().await;
}

/// Per-session state shared by the request tasks.
struct Upstream {
    addr: SocketAddr,
    head_timeout: Duration,
    token: String,
    caller: String,
    session_id: Mutex<Option<String>>,
    protocol: Mutex<Option<String>>,
    /// Notified when rcpd answers HTTP 404 for the session id: the reader
    /// stops reading and the stream ends.
    lost: Notify,
}

impl Upstream {
    fn session_id(&self) -> Option<String> {
        self.session_id
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    fn protocol(&self) -> Option<String> {
        self.protocol
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    fn request(&self, method: Method, accept: &'static str) -> hyper::http::request::Builder {
        let mut b = Request::builder()
            .method(method)
            .uri("/mcp")
            .header(HOST, self.addr.to_string())
            .header(AUTHORIZATION, format!("Bearer {}", self.token))
            .header(ACCEPT, accept)
            .header(CALLER_HEADER, &self.caller);
        if let Some(id) = self.session_id() {
            b = b.header("mcp-session-id", id);
        }
        if let Some(v) = self.protocol() {
            b = b.header("mcp-protocol-version", v);
        }
        b
    }
}

/// One frame queued for the desk: the class whose requirement it must still
/// meet when written, and its share of the session's byte budget.
struct Outbound {
    class: RcpClass,
    frame: Vec<u8>,
    _budget: OwnedSemaphorePermit,
}

/// The sending half of a session's outbound queue, bounded by
/// [`RcpLimits::session_buffer_bytes`].
#[derive(Clone)]
struct Outlet {
    tx: mpsc::Sender<Outbound>,
    budget: Arc<Semaphore>,
    /// The budget's size: no one frame takes more.
    capacity: usize,
}

impl Outlet {
    /// Queue `frame`; `false` once the writer has stopped.
    async fn send(&self, class: RcpClass, frame: Vec<u8>) -> bool {
        let bytes = u32::try_from(frame.len().clamp(1, self.capacity)).unwrap_or(u32::MAX);
        let Ok(permit) = self.budget.clone().acquire_many_owned(bytes).await else {
            return false;
        };
        self.tx
            .send(Outbound {
                class,
                frame,
                _budget: permit,
            })
            .await
            .is_ok()
    }

    async fn error(&self, id: Value, code: i64, message: &str) -> bool {
        self.send(RcpClass::Session, error_frame(id, code, message))
            .await
    }

    /// Queue an error frame without waiting; `false` (the frame is dropped)
    /// when the queue or the byte budget is full or the writer has stopped.
    fn try_error(&self, id: Value, code: i64, message: &str) -> bool {
        let frame = error_frame(id, code, message);
        let bytes = u32::try_from(frame.len().clamp(1, self.capacity)).unwrap_or(u32::MAX);
        let Ok(permit) = self.budget.clone().try_acquire_many_owned(bytes) else {
            return false;
        };
        self.tx
            .try_send(Outbound {
                class: RcpClass::Session,
                frame,
                _budget: permit,
            })
            .is_ok()
    }

    fn is_closed(&self) -> bool {
        self.tx.is_closed()
    }
}

/// The id of a JSON-RPC response frame (`result` or `error`), if it is one.
fn response_id(frame: &[u8]) -> Option<Value> {
    let Ok(Value::Object(obj)) = serde_json::from_slice::<Value>(frame) else {
        return None;
    };
    if obj.contains_key("result") || obj.contains_key("error") {
        obj.get("id").cloned()
    } else {
        None
    }
}

/// Whether `message` is the response to the request `request_id`.
fn answers(request_id: Option<&Value>, message: &[u8]) -> bool {
    request_id.is_some_and(|id| response_id(message).as_ref() == Some(id))
}

/// Engage the e-stop latch on `lease` through [`EstopLease::engage`], naming
/// `caller` as the engager when it was clear: the engage is counted in before
/// it waits for the lease's lock and out after the latch is engaged and the
/// lock released, and a poisoned lock is taken as it is.
fn engage_estop(lease: &EstopLease, caller: &str) {
    lease.engage(caller);
    tracing::warn!(by = %caller, "cerulion_remoted: e-stop ENGAGED by an rcp estop");
}

/// The refusal for the Act message `message` while the e-stop on `lease`
/// holds ([`EstopLease::holds`]: the latch is engaged or an engage is
/// counted), `None` while it reads clear.
fn estop_refusal(lease: &EstopLease, message: &RcpMessage) -> Option<&'static str> {
    estop_refusal_of(lease, &message.canonical)
}

/// [`estop_refusal`] for the Act message whose canonical form is `canonical`.
fn estop_refusal_of(lease: &EstopLease, canonical: &[u8]) -> Option<&'static str> {
    if !lease.holds() {
        return None;
    }
    if calls_tool(canonical, ESTOP_CLEAR_TOOL) {
        Some(ESTOP_CLEAR_REFUSAL)
    } else {
        Some(ESTOP_ENGAGED_REFUSAL)
    }
}

/// Whether the canonical message `canonical` is a `tools/call` of `tool`.
fn calls_tool(canonical: &[u8], tool: &str) -> bool {
    serde_json::from_slice::<Value>(canonical).is_ok_and(|v| {
        v.get("method").and_then(Value::as_str) == Some("tools/call")
            && v.pointer("/params/name").and_then(Value::as_str) == Some(tool)
    })
}

/// The checks a stream's request tasks run on the open connection to rcpd,
/// just before each send: the key's access for an Observe or Act message's
/// class, then for an Act message the e-stop latch. Lifecycle messages and the
/// e-stop are never refused here.
struct SendCheck {
    plane: Arc<RcpPlane>,
    device_key: [u8; 32],
    caller: String,
}

impl SendCheck {
    /// The reply refusing the message of `class` and `method`, whose canonical
    /// form is `canonical`, now; `None` when it may be sent.
    fn refusal(&self, class: RcpClass, method: &str, canonical: &[u8]) -> Option<String> {
        if !matches!(class, RcpClass::Observe | RcpClass::Act) {
            return None;
        }
        let allowed = authorize_key(&self.plane.shared, &self.device_key, Some(class));
        if let Err(reason) = allowed {
            tracing::info!(%method, %reason, caller = %self.caller, "cerulion_remoted: queued rcp message denied before it was sent");
            return Some(format!("denied: {method} needs more access: {reason}"));
        }
        if class == RcpClass::Act {
            let refusal = estop_refusal_of(&self.plane.lease, canonical);
            if let Some(text) = refusal {
                tracing::info!(%method, caller = %self.caller, "cerulion_remoted: queued rcp tool call refused under the e-stop");
                return Some(text.to_string());
            }
        }
        None
    }
}

/// Resolves when rcpd has answered HTTP 404 for this session's id; never when
/// the session has no upstream.
async fn session_lost(upstream: &Result<Arc<Upstream>, String>) {
    match upstream {
        Ok(u) => u.lost.notified().await,
        Err(_) => std::future::pending().await,
    }
}

async fn serve_session(
    mut send: SendStream,
    mut recv: cerulion_link::RecvStream,
    connection: Connection,
    device_key: [u8; 32],
    plane: Arc<RcpPlane>,
) {
    let limits = plane.link.limits;
    let caller = caller_identity(&device_key, plane.shared.account_for_key(&device_key));
    let upstream = load_token(&plane.link.token_file).map(|token| {
        Arc::new(Upstream {
            addr: plane.link.addr,
            head_timeout: limits.head_timeout,
            token,
            caller: caller.clone(),
            session_id: Mutex::new(None),
            protocol: Mutex::new(None),
            lost: Notify::new(),
        })
    });
    if let Err(reason) = &upstream {
        tracing::warn!(%reason, "cerulion_remoted: rcp session cannot reach rcpd");
    }
    let (out_tx, mut out_rx) = mpsc::channel::<Outbound>(OUTBOUND_QUEUE);
    let out = Outlet {
        tx: out_tx,
        budget: Arc::new(Semaphore::new(limits.session_buffer_bytes)),
        capacity: limits.session_buffer_bytes,
    };

    // The writer: each frame must still meet the requirement of the class that
    // produced it; the session floor is also re-checked on a timer, and a key
    // that fails it has its whole connection closed.
    let shared = plane.shared.clone();
    let writer_conn = connection.clone();
    let writer = tokio::spawn(async move {
        let mut tick = tokio::time::interval(limits.revoke_check);
        let end = |reason: String| {
            tracing::warn!(%reason, "cerulion_remoted: rcp session ended by the live trust state");
            writer_conn.close(CLOSE_REVOKED.into(), b"rcp access revoked");
        };
        loop {
            tokio::select! {
                next = out_rx.recv() => {
                    let Some(item) = next else { break };
                    if let Err(reason) = authorize_key(&shared, &device_key, Some(RcpClass::Session)) {
                        end(reason);
                        break;
                    }
                    let frame = match authorize_key(&shared, &device_key, Some(item.class)) {
                        Ok(()) => item.frame,
                        // The scope no longer meets the class: a response is
                        // answered with a denial, anything else is dropped.
                        Err(reason) => match response_id(&item.frame) {
                            Some(id) => error_frame(
                                id,
                                DENIED_CODE,
                                &format!(
                                    "result withheld: {reason}; rcpd may already have acted on \
                                     this request"
                                ),
                            ),
                            None => continue,
                        },
                    };
                    if write_frame(&mut send, &frame).await.is_err() {
                        break;
                    }
                }
                _ = tick.tick() => {
                    if let Err(reason) = authorize_key(&shared, &device_key, Some(RcpClass::Session)) {
                        end(reason);
                        break;
                    }
                }
            }
        }
        let _ = send.finish();
    });

    let in_flight = Arc::new(Semaphore::new(limits.in_flight));
    let estops = Arc::new(Semaphore::new(limits.estops_in_flight));
    let notification_room = Arc::new(Semaphore::new(FORWARD_NOTIFICATION_ROOM));
    // The forwarder: every admitted message reaches rcpd from this task, in
    // the order the reader admitted it, so the reader never waits on rcpd.
    // The guard aborts it, and its tasks, when this session ends or is
    // aborted.
    let (to_forwarder, mut forwarding) = match &upstream {
        Ok(u) => {
            let capacity = limits
                .in_flight
                .saturating_add(limits.estops_in_flight)
                .saturating_add(FORWARD_NOTIFICATION_ROOM)
                .min(Semaphore::MAX_PERMITS);
            let (tx, rx) = mpsc::channel::<Admitted>(capacity);
            let check = Arc::new(SendCheck {
                plane: plane.clone(),
                device_key,
                caller: caller.clone(),
            });
            let task = tokio::spawn(forwarder(rx, u.clone(), out.clone(), check));
            (Some(tx), AbortOnDrop(Some(task)))
        }
        Err(_) => (None, AbortOnDrop(None)),
    };
    // The reader task owns `recv`: `read_frame` is not cancel-safe, so the
    // loop below selects on this channel, never on the framing. It reads one
    // frame ahead and ends after the first read that fails; the guard aborts
    // it when the session ends.
    let (frame_tx, mut frames) = mpsc::channel::<Result<Vec<u8>, cerulion_link::LinkError>>(1);
    let _reader = AbortOnDrop(Some(tokio::spawn(async move {
        loop {
            let read = read_frame(&mut recv, MAX_REQUEST_FRAME).await;
            let failed = read.is_err();
            if frame_tx.send(read).await.is_err() || failed {
                break;
            }
        }
    })));
    let mut initialized = false;
    let mut initialize_seen = false;
    // When rcpd lost the stream's MCP session. The reader then reads on until
    // the desk ends its side or the drain bound passes, so an `estop` sent
    // after the loss still engages the ops latch and every request is answered.
    let mut lost_at: Option<tokio::time::Instant> = None;
    loop {
        if lost_at.is_some_and(|at| at.elapsed() >= limits.drain) {
            break;
        }
        let deadline = lost_at.map_or_else(tokio::time::Instant::now, |at| at + limits.drain);
        // Biased: a frame already read is taken before the loss, so a queued
        // `estop` goes the normal way.
        let frame = tokio::select! {
            biased;
            read = frames.recv() => match read {
                Some(Ok(f)) => f,
                Some(Err(e)) => {
                    tracing::debug!(error = %e, "cerulion_remoted: rcp session closed by the desk");
                    break;
                }
                None => break,
            },
            () = session_lost(&upstream), if lost_at.is_none() => {
                tracing::warn!(%caller, "cerulion_remoted: rcpd lost the rcp stream's MCP session; the stream ends after the drain bound");
                #[cfg(any(test, feature = "test-seam"))]
                plane.loss_seen.notify_one();
                lost_at = Some(tokio::time::Instant::now());
                continue;
            }
            () = tokio::time::sleep_until(deadline), if lost_at.is_some() => break,
        };
        if out.is_closed() {
            break;
        }
        let message = match classify_message(&frame) {
            Ok(m) => m,
            Err(refusal) => {
                tracing::info!(reason = %refusal.reason, %caller, "cerulion_remoted: rcp message refused");
                if let Some(id) = refusal.reply_id {
                    let reason = format!("refused: {}", refusal.reason);
                    out.try_error(id, INVALID_REQUEST_CODE, &reason);
                }
                continue;
            }
        };
        if let Err(reason) = authorize_key(&plane.shared, &device_key, Some(message.class)) {
            tracing::info!(method = %message.method, %reason, %caller, "cerulion_remoted: rcp message denied");
            if let Some(id) = message.id {
                let text = format!("denied: {} needs more access: {reason}", message.method);
                out.try_error(id, DENIED_CODE, &text);
            }
            continue;
        }
        // rcpd no longer knows this stream's session: nothing more is sent to
        // it. An `estop` still engages the ops latch; a notification is dropped.
        if lost_at.is_some() {
            let estop = message.class == RcpClass::Estop;
            if estop {
                engage_estop(&plane.lease, &caller);
            }
            if let Some(id) = message.id {
                let text = if estop {
                    ESTOP_SESSION_LOST
                } else {
                    SESSION_LOST
                };
                out.try_error(id, UPSTREAM_CODE, text);
            }
            continue;
        }
        // The e-stop latch is read and engaged here, after authorization and
        // before the upstream check and the e-stop bound.
        match message.class {
            RcpClass::Estop => engage_estop(&plane.lease, &caller),
            RcpClass::Act => {
                if let Some(text) = estop_refusal(&plane.lease, &message) {
                    tracing::info!(method = %message.method, %caller, "cerulion_remoted: rcp tool call refused under the e-stop");
                    if let Some(id) = message.id {
                        out.try_error(id, DENIED_CODE, text);
                    }
                    continue;
                }
            }
            RcpClass::Session | RcpClass::Observe => {}
        }
        let Some(to_forwarder) = &to_forwarder else {
            if let Some(id) = message.id {
                out.try_error(id, UPSTREAM_CODE, "rcpd is not reachable on this robot");
            }
            continue;
        };
        // One handshake per stream; each flag is set once its message is
        // queued.
        let repeat = match message.method.as_str() {
            "initialize" => initialize_seen,
            "notifications/initialized" => initialized,
            _ => false,
        };
        if repeat {
            if let Some(id) = message.id {
                out.try_error(
                    id,
                    INVALID_REQUEST_CODE,
                    "refused: one initialize per rcp stream",
                );
            }
            continue;
        }
        let permit = match message.class {
            // The e-stop takes no Observe/Act slot and never waits: it has its
            // own small bound, past which it is answered at once.
            RcpClass::Estop => {
                let Ok(permit) = estops.clone().try_acquire_owned() else {
                    if let Some(id) = message.id {
                        out.try_error(
                            id,
                            UPSTREAM_CODE,
                            "refused: an e-stop is already in flight on this rcp stream",
                        );
                    }
                    continue;
                };
                permit
            }
            // A notification takes no slot (rcpd answers it 202 with no body),
            // but one of the forward queue's notification places.
            RcpClass::Session if message.id.is_none() => {
                match notification_room.clone().try_acquire_owned() {
                    Ok(permit) => permit,
                    Err(_) => {
                        tracing::debug!(
                            method = %message.method,
                            "cerulion_remoted: rcp notification dropped: the stream's forward \
                             queue has no room for notifications"
                        );
                        continue;
                    }
                }
            }
            // A lifecycle request (its body relayed by a task) holds an
            // in-flight slot like any other request, so a desk that stops
            // reading cannot grow the task set. Lifecycle messages keep their
            // order at the forwarder: the response head (and the body of
            // `initialize`) is in before the next message is sent.
            RcpClass::Session | RcpClass::Observe | RcpClass::Act => {
                let Ok(permit) = in_flight.clone().try_acquire_owned() else {
                    if let Some(id) = message.id {
                        out.try_error(id, UPSTREAM_CODE, TOO_MANY_IN_FLIGHT);
                    }
                    continue;
                };
                permit
            }
        };
        let method = message.method.clone();
        if hand_off(to_forwarder, &out, message, Some(permit)) {
            match method.as_str() {
                "initialize" => initialize_seen = true,
                "notifications/initialized" => initialized = true,
                _ => {}
            }
        }
    }
    // The desk sent its last message: the forwarder sends what the reader
    // admitted, then requests already forwarded may still answer, within the
    // drain bound, which runs from the loss when rcpd lost the session; the
    // standalone stream never ends by itself.
    drop(to_forwarder);
    let (mut tasks, mut streams) = forwarding.join().await.unwrap_or_default();
    streams.shutdown().await;
    let drain = lost_at.map_or(limits.drain, |at| limits.drain.saturating_sub(at.elapsed()));
    let drained =
        tokio::time::timeout(drain, async { while tasks.join_next().await.is_some() {} }).await;
    if drained.is_err() {
        tasks.shutdown().await;
    }
    if let Ok(u) = &upstream {
        if u.session_id().is_some() {
            let _ = tokio::time::timeout(CONNECT_TIMEOUT, delete_session(u)).await;
        }
    }
    drop(out);
    let _ = writer.await;
}

/// One admitted message on its way to the stream's forwarder, with the slot it
/// holds until its answer is relayed.
struct Admitted {
    message: RcpMessage,
    permit: Option<OwnedSemaphorePermit>,
}

/// Hand `message` to the stream's forwarder without waiting; whether it was
/// queued. A full (or ended) forward queue answers a request with
/// [`UPSTREAM_CODE`] and drops a notification; the slot or notification
/// place is released either way.
fn hand_off(
    to_forwarder: &mpsc::Sender<Admitted>,
    out: &Outlet,
    message: RcpMessage,
    permit: Option<OwnedSemaphorePermit>,
) -> bool {
    let Err(full) = to_forwarder.try_send(Admitted { message, permit }) else {
        return true;
    };
    let Admitted { message, .. } = full.into_inner();
    match message.id {
        Some(id) => {
            out.try_error(id, UPSTREAM_CODE, TOO_MANY_IN_FLIGHT);
        }
        None => tracing::debug!(
            method = %message.method,
            "cerulion_remoted: rcp notification dropped: the stream's forward queue is full"
        ),
    }
    false
}

/// A spawned task aborted when this guard drops, so a session that ends or is
/// aborted takes its forwarder, and the forwarder's tasks, with it.
struct AbortOnDrop<T>(Option<tokio::task::JoinHandle<T>>);

impl<T> AbortOnDrop<T> {
    /// Wait for the task's output; `None` when there is no task or it failed.
    /// The guard keeps the handle while it waits, so dropping this future
    /// still aborts the task.
    async fn join(&mut self) -> Option<T> {
        let output = self.0.as_mut()?.await.ok();
        self.0 = None;
        output
    }
}

impl<T> Drop for AbortOnDrop<T> {
    fn drop(&mut self) {
        if let Some(task) = self.0.take() {
            task.abort();
        }
    }
}

/// The stream's forwarder: sends each message the reader admitted to rcpd,
/// in order. A lifecycle message is sent inline ([`forward`] waits for its
/// response head, and for the `initialize` result), so a session id rcpd
/// issues is recorded before the next message is sent; after
/// `notifications/initialized` it opens rcpd's standalone stream. Observe, Act
/// and Estop requests are forwarded and relayed by their own tasks, which run
/// `check` on the open connection to rcpd just before each send, so a request
/// is refused after a revoke or an engage that lands while it waits in the
/// queue or for its connection. A `notifications/cancelled` naming a request
/// whose response head is not in yet is sent by a task of its own once that
/// head is in, within [`RcpLimits::head_timeout`]; the forwarder goes on at
/// once. It stops once the stream's writer has stopped. Returns the tasks and
/// the standalone stream once the reader has closed the queue and every queued
/// message is sent or handed to a task that sends or refuses it, or once it
/// stopped.
async fn forwarder(
    mut admitted: mpsc::Receiver<Admitted>,
    upstream: Arc<Upstream>,
    out: Outlet,
    check: Arc<SendCheck>,
) -> (JoinSet<()>, JoinSet<()>) {
    let mut tasks = JoinSet::new();
    let mut streams = JoinSet::new();
    // The requests whose response head is not in yet, by id. Each request's
    // task holds the sender of its entry and drops it once the head is in or
    // the request will not be sent.
    let mut unsent: HashMap<String, watch::Receiver<()>> = HashMap::new();
    while let Some(Admitted { message, permit }) = admitted.recv().await {
        // The writer has stopped (a revoke closes the connection from it):
        // nothing more is sent to rcpd, and the queued slots are released.
        if out.is_closed() {
            break;
        }
        // Finished tasks are reaped as the session goes, not at its end.
        while tasks.try_join_next().is_some() {}
        while streams.try_join_next().is_some() {}
        unsent.retain(|_, head| head.has_changed().is_ok());
        if message.class != RcpClass::Session {
            let (head_in, head) = watch::channel(());
            if let Some(id) = &message.id {
                unsent.insert(id.to_string(), head);
            }
            tasks.spawn(forward_and_relay(
                upstream.clone(),
                message,
                out.clone(),
                check.clone(),
                permit,
                head_in,
            ));
            continue;
        }
        // rcpd (rmcp's stateful streamable HTTP server) answers a request with
        // its SSE response head as soon as the request is in the MCP session,
        // before any tool runs, and takes a cancel into that session in the
        // order it receives it. So a cancel sent once the head of the request
        // it names is in never reaches rcpd first, and requests stay
        // concurrent up to `--rcp-in-flight`. Waiting here would stop every
        // later message, the e-stop included, so the cancel waits in its own
        // task.
        if let Some(mut head) = cancelled_request(&message).and_then(|id| unsent.get(&id).cloned())
        {
            let (upstream, out, check) = (upstream.clone(), out.clone(), check.clone());
            tasks.spawn(async move {
                let _ = tokio::time::timeout(upstream.head_timeout, head.changed()).await;
                let _ = forward(upstream, message, out, false, check, None).await;
                drop(permit);
            });
            continue;
        }
        let is_initialized = message.method == "notifications/initialized";
        let is_request = message.id.is_some();
        let body = forward(
            upstream.clone(),
            message,
            out.clone(),
            true,
            check.clone(),
            None,
        )
        .await;
        // A body rcpd sends for a notification is not relayed; a request's
        // body task keeps its slot.
        if let (Some(body), true) = (body, is_request) {
            tasks.spawn(async move {
                body.await;
                drop(permit);
            });
        }
        if is_initialized && upstream.session_id().is_some() {
            streams.spawn(standalone_stream(upstream.clone(), out.clone()));
        }
    }
    (tasks, streams)
}

/// The id a `notifications/cancelled` message names (`params.requestId`), in
/// the form the forwarder keys its requests by; `None` for any other message.
fn cancelled_request(message: &RcpMessage) -> Option<String> {
    if message.method != "notifications/cancelled" {
        return None;
    }
    let value: Value = serde_json::from_slice(&message.canonical).ok()?;
    value.pointer("/params/requestId").map(Value::to_string)
}

async fn forward_and_relay(
    upstream: Arc<Upstream>,
    message: RcpMessage,
    out: Outlet,
    check: Arc<SendCheck>,
    permit: Option<OwnedSemaphorePermit>,
    head_in: watch::Sender<()>,
) {
    if let Some(body) = forward(upstream, message, out, false, check, Some(head_in)).await {
        body.await;
    }
    drop(permit);
}

/// Open a fresh loopback HTTP/1 connection to rcpd.
async fn connect<B>(addr: SocketAddr) -> Result<hyper::client::conn::http1::SendRequest<B>, String>
where
    B: hyper::body::Body + Send + 'static,
    B::Data: Send,
    B::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
{
    let stream = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(addr))
        .await
        .map_err(|_| format!("rcpd at {addr} did not accept within {CONNECT_TIMEOUT:?}"))?
        .map_err(|e| format!("rcpd at {addr}: {e}"))?;
    let (sender, conn) = hyper::client::conn::http1::handshake(TokioIo::new(stream))
        .await
        .map_err(|e| format!("rcpd at {addr}: {e}"))?;
    tokio::spawn(async move {
        let _ = conn.await;
    });
    Ok(sender)
}

/// POST one admitted message to rcpd. Waits for the response head, within
/// [`RcpLimits::head_timeout`], so a session id rcpd issues is recorded before
/// the next message is sent, and returns the task that relays the body, if
/// any. For `initialize` the body is relayed before returning, so the
/// protocol version it carries is recorded too.
///
/// `inline` is true on the stream's forwarder: its failure replies never wait
/// for queue space (they are dropped when it is full), and only its relay of
/// the `initialize` result waits, within [`RcpLimits::head_timeout`]. A
/// spawned task's replies wait for queue space.
///
/// The connection to rcpd is opened first, then `check` runs, then the
/// request is sent, so a revoke or an engage that lands during the connect
/// refuses it with [`DENIED_CODE`] before rcpd sees it. `head_in` is dropped
/// once the response head is in or the request will not be sent.
///
/// An HTTP 404 to a request that carried the session id answers it with
/// [`UPSTREAM_CODE`] and notifies the session's `Upstream::lost`.
async fn forward(
    upstream: Arc<Upstream>,
    message: RcpMessage,
    out: Outlet,
    inline: bool,
    check: Arc<SendCheck>,
    head_in: Option<watch::Sender<()>>,
) -> Option<impl std::future::Future<Output = ()> + Send + 'static> {
    let RcpMessage {
        class,
        method,
        id,
        canonical,
    } = message;
    let fail = |reason: String| {
        let out = out.clone();
        let id = id.clone();
        async move {
            tracing::warn!(%reason, "cerulion_remoted: rcp request to rcpd failed");
            if let Some(id) = id {
                if inline {
                    out.try_error(id, UPSTREAM_CODE, "rcpd did not answer this request");
                } else {
                    out.error(id, UPSTREAM_CODE, "rcpd did not answer this request")
                        .await;
                }
            }
        }
    };
    let sent_session = upstream.session_id().is_some();
    let canonical = Bytes::from(canonical);
    let request = upstream
        .request(Method::POST, "application/json, text/event-stream")
        .header(CONTENT_TYPE, "application/json")
        .body(Full::new(canonical.clone()));
    let request = match request {
        Ok(r) => r,
        Err(e) => {
            fail(format!("cannot build the rcpd request: {e}")).await;
            return None;
        }
    };
    let head = async {
        let mut sender = connect::<Full<Bytes>>(upstream.addr)
            .await
            .map_err(NoHead::Failed)?;
        #[cfg(any(test, feature = "test-seam"))]
        check.plane.after_connect_for_test(id.as_ref()).await;
        if let Some(text) = check.refusal(class, &method, &canonical) {
            return Err(NoHead::Refused(text));
        }
        sender
            .send_request(request)
            .await
            .map_err(|e| NoHead::Failed(format!("rcpd request failed: {e}")))
    };
    let head = tokio::time::timeout(upstream.head_timeout, head).await;
    drop(head_in);
    let response = match head {
        Ok(Ok(r)) => r,
        Ok(Err(NoHead::Refused(text))) => {
            if let Some(id) = id.clone() {
                if inline {
                    out.try_error(id, DENIED_CODE, &text);
                } else {
                    out.error(id, DENIED_CODE, &text).await;
                }
            }
            return None;
        }
        Ok(Err(NoHead::Failed(reason))) => {
            fail(reason).await;
            return None;
        }
        Err(_) => {
            fail(format!(
                "rcpd sent no response head within {:?}",
                upstream.head_timeout
            ))
            .await;
            return None;
        }
    };
    let is_initialize = method == "initialize";
    if is_initialize {
        if let Some(sid) = header_text(&response, "mcp-session-id") {
            *upstream
                .session_id
                .lock()
                .unwrap_or_else(PoisonError::into_inner) = Some(sid);
        }
    }
    let status = response.status();
    if status == StatusCode::ACCEPTED {
        return None;
    }
    if status == StatusCode::NOT_FOUND && sent_session {
        tracing::warn!("cerulion_remoted: rcpd answered HTTP 404 for the rcp stream's MCP session");
        if let Some(id) = id.clone() {
            if inline {
                out.try_error(id, UPSTREAM_CODE, SESSION_LOST);
            } else {
                out.error(id, UPSTREAM_CODE, SESSION_LOST).await;
            }
        }
        upstream.lost.notify_one();
        return None;
    }
    if !status.is_success() {
        fail(format!("rcpd answered HTTP {status}")).await;
        return None;
    }
    let head_timeout = upstream.head_timeout;
    let relay = relay_body(response, upstream, id, is_initialize, class, out);
    if is_initialize {
        if tokio::time::timeout(head_timeout, relay).await.is_err() {
            tracing::warn!("cerulion_remoted: rcpd's initialize result did not finish in time");
        }
        return None;
    }
    Some(relay)
}

/// Why [`forward`] got no response head.
enum NoHead {
    /// The checks just before the send refused the request; the reply text.
    Refused(String),
    /// rcpd could not be reached or failed the request.
    Failed(String),
}

fn header_text(response: &Response<Incoming>, name: &str) -> Option<String> {
    response
        .headers()
        .get(name)
        .and_then(|v: &HeaderValue| v.to_str().ok())
        .map(str::to_string)
}

/// Relay a response body's JSON-RPC messages to the desk, each under `class`.
/// Records the protocol version from the initialize result before relaying
/// it. A body that fails or outgrows the relay bound, or ends without the
/// response to `request_id`, answers `request_id` with an error.
async fn relay_body(
    response: Response<Incoming>,
    upstream: Arc<Upstream>,
    request_id: Option<Value>,
    is_initialize: bool,
    class: RcpClass,
    out: Outlet,
) {
    let sse = header_text(&response, CONTENT_TYPE.as_str())
        .is_some_and(|ct| ct.starts_with("text/event-stream"));
    let mut body = response.into_body();
    let mut events = SseBuffer::default();
    let mut whole: Vec<u8> = Vec::new();
    let mut answered = false;
    let failed = loop {
        let Some(next) = body.frame().await else {
            break false;
        };
        let Ok(frame) = next else {
            break true;
        };
        let Ok(chunk) = frame.into_data() else {
            continue;
        };
        if sse {
            let Ok(messages) = events.push(&chunk) else {
                tracing::warn!("cerulion_remoted: rcpd event exceeded the relay bound");
                break true;
            };
            for m in messages {
                if is_initialize {
                    record_protocol(&upstream, request_id.as_ref(), &m);
                }
                answered = answered || answers(request_id.as_ref(), &m);
                if !out.send(class, m).await {
                    return;
                }
            }
        } else {
            if whole.len() + chunk.len() > MAX_RESPONSE_MESSAGE {
                tracing::warn!("cerulion_remoted: rcpd response exceeded the relay bound");
                break true;
            }
            whole.extend_from_slice(&chunk);
        }
    };
    if failed {
        if let Some(id) = request_id.filter(|_| !answered) {
            out.error(
                id,
                UPSTREAM_CODE,
                "rcpd's answer to this request was cut off",
            )
            .await;
        }
        return;
    }
    if !sse && !whole.is_empty() {
        if is_initialize {
            record_protocol(&upstream, request_id.as_ref(), &whole);
        }
        answered = answered || answers(request_id.as_ref(), &whole);
        if !out.send(class, whole).await {
            return;
        }
    }
    if let Some(id) = request_id.filter(|_| !answered) {
        out.error(
            id,
            UPSTREAM_CODE,
            "rcpd ended its answer without a response",
        )
        .await;
    }
}

fn record_protocol(upstream: &Upstream, initialize_id: Option<&Value>, message: &[u8]) {
    let Some(init_id) = initialize_id else {
        return;
    };
    if let Ok(Value::Object(obj)) = serde_json::from_slice::<Value>(message) {
        if obj.get("id") == Some(init_id) {
            if let Some(v) = protocol_version(&obj) {
                *upstream
                    .protocol
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner) = Some(v);
            }
        }
    }
}

fn protocol_version(obj: &Map<String, Value>) -> Option<String> {
    obj.get("result")?
        .get("protocolVersion")?
        .as_str()
        .filter(|v| !v.is_empty() && v.len() <= 64 && v.bytes().all(|b| b.is_ascii_graphic()))
        .map(str::to_string)
}

/// rcpd's standalone `GET` event stream, relayed as Observe until it ends.
async fn standalone_stream(upstream: Arc<Upstream>, out: Outlet) {
    let Ok(request) = upstream
        .request(Method::GET, "text/event-stream")
        .body(Empty::<Bytes>::new())
    else {
        return;
    };
    let head = async {
        let mut sender = connect::<Empty<Bytes>>(upstream.addr).await?;
        sender
            .send_request(request)
            .await
            .map_err(|e| e.to_string())
    };
    let Ok(Ok(response)) = tokio::time::timeout(upstream.head_timeout, head).await else {
        return;
    };
    if response.status() != StatusCode::OK {
        tracing::debug!(status = %response.status(), "cerulion_remoted: rcpd serves no standalone stream");
        return;
    }
    relay_body(response, upstream, None, false, RcpClass::Observe, out).await;
}

async fn delete_session(upstream: &Upstream) {
    let Ok(mut sender) = connect::<Empty<Bytes>>(upstream.addr).await else {
        return;
    };
    if let Ok(request) = upstream
        .request(Method::DELETE, "application/json")
        .body(Empty::<Bytes>::new())
    {
        let _ = sender.send_request(request).await;
    }
}

/// The `reason` of the [`ESTOP_TOOL`] call an ops `engage-estop` sends to
/// rcpd ([the ops e-stop in rcpd](self#the-ops-e-stop-in-rcpd)); rcpd puts it
/// in the e-stop frame the graph receives.
pub const OPS_ESTOP_REASON: &str = "the robot's ops e-stop was engaged (engage-estop)";

/// The MCP protocol version an ops e-stop session offers rcpd.
const OPS_ESTOP_PROTOCOL: &str = "2025-06-18";

/// The JSON-RPC id of an ops e-stop session's `initialize`.
const OPS_ESTOP_INITIALIZE_ID: i64 = 0;

/// The JSON-RPC id of an ops e-stop session's [`ESTOP_TOOL`] call.
const OPS_ESTOP_CALL_ID: i64 = 1;

/// How long an ops e-stop session's answer may take once its response head
/// is in: rcpd answers an `estop` once the graph acknowledges its frame, or
/// after 2 s without that.
const OPS_ESTOP_ANSWER: Duration = Duration::from_secs(10);

/// How long an ops e-stop waits after its first failed attempt before the
/// next; each later wait doubles, up to [`OPS_ESTOP_RETRY_CAP`].
const OPS_ESTOP_RETRY_FIRST: Duration = Duration::from_millis(100);

/// The longest wait between two attempts of an ops e-stop.
const OPS_ESTOP_RETRY_CAP: Duration = Duration::from_secs(2);

/// The least time between two error lines about the failed attempts of one
/// ops e-stop; the first failure is always logged.
const OPS_ESTOP_LOG_EVERY: Duration = Duration::from_secs(30);

/// Where and within what bounds an ops e-stop reaches rcpd: one attempt's
/// connection and response heads within `head_timeout`, each answer within
/// `answer_timeout` after its head.
#[derive(Debug, Clone)]
struct EstopTarget {
    addr: SocketAddr,
    token_file: PathBuf,
    head_timeout: Duration,
    answer_timeout: Duration,
}

/// Sends the robot's rcpd an [`ESTOP_TOOL`] call for an ops `engage-estop`
/// and sends it again until rcpd answers that its latch closed
/// ([the ops e-stop in rcpd](self#the-ops-e-stop-in-rcpd)). One delivery runs
/// at a time. The daemon builds one from its [`RcpLink`], and only its
/// `engage-estop` verb holds it.
pub(crate) struct RcpdEstop {
    target: EstopTarget,
    shared: SharedTrust,
    runtime: tokio::runtime::Handle,
    /// The wait after a delivery's first failed attempt.
    retry_first: Duration,
    /// The longest wait between two attempts.
    retry_cap: Duration,
    /// Set while a delivery runs; cleared by its [`InFlight`] guard.
    in_flight: Arc<AtomicBool>,
    /// `true` once the daemon stops; a delivery ends when it reads `true` or
    /// this sender is dropped.
    stop: watch::Sender<bool>,
    /// The latest delivery's task, awaited by [`RcpdEstop::shut_down`].
    task: Mutex<Option<tokio::task::JoinHandle<()>>>,
}

impl RcpdEstop {
    /// A sender to the rcpd of `link`, its attempts bounded by
    /// [`RcpLimits::head_timeout`] and [`OPS_ESTOP_ANSWER`], that names each
    /// engager's account from `shared` and runs each delivery on `runtime`.
    pub(crate) fn new(
        link: &RcpLink,
        shared: SharedTrust,
        runtime: tokio::runtime::Handle,
    ) -> Self {
        Self {
            target: EstopTarget {
                addr: link.addr,
                token_file: link.token_file.clone(),
                head_timeout: link.limits.head_timeout,
                answer_timeout: OPS_ESTOP_ANSWER,
            },
            shared,
            runtime,
            retry_first: OPS_ESTOP_RETRY_FIRST,
            retry_cap: OPS_ESTOP_RETRY_CAP,
            in_flight: Arc::new(AtomicBool::new(false)),
            stop: watch::channel(false).0,
            task: Mutex::new(None),
        }
    }

    /// Start delivering the `estop` for an engage by the device key
    /// `engager` on a task of its own, and return at once: `true` when a
    /// delivery started, `false` when one was in flight, which sends again
    /// until rcpd answers, so this engage starts none. The task looks up the
    /// engager's account and runs [`Delivery::run`].
    pub(crate) fn send(&self, engager: &[u8; 32]) -> bool {
        if self
            .in_flight
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            tracing::info!(
                by = %hex::encode(engager),
                "cerulion_remoted: an estop to rcpd is already in flight; this engage-estop \
                 starts none"
            );
            return false;
        }
        let in_flight = InFlight(self.in_flight.clone());
        let delivery = Delivery {
            target: self.target.clone(),
            shared: self.shared.clone(),
            engager: *engager,
            retry_first: self.retry_first,
            retry_cap: self.retry_cap,
            stop: self.stop.subscribe(),
        };
        let run = async move {
            let _in_flight = in_flight;
            delivery.run().await;
        };
        let task = self.runtime.spawn(run);
        *self.task.lock().unwrap_or_else(PoisonError::into_inner) = Some(task);
        true
    }

    /// Whether [`RcpdEstop::shut_down`] has marked the daemon stopping.
    #[cfg(feature = "test-seam")]
    pub(crate) fn is_stopping(&self) -> bool {
        *self.stop.borrow()
    }

    /// End the delivery in flight, if any: mark the daemon stopping, then wait
    /// for the delivery's task, within [`CONNECT_TIMEOUT`]. A delivery that
    /// rcpd had not answered logs that at warn.
    pub(crate) async fn shut_down(&self) {
        self.stop.send_replace(true);
        let task = self
            .task
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        if let Some(task) = task {
            let _ = tokio::time::timeout(CONNECT_TIMEOUT, task).await;
        }
    }
}

/// Clears an [`RcpdEstop`]'s in-flight flag when dropped: when its delivery
/// ends, and when the runtime drops its task.
struct InFlight(Arc<AtomicBool>);

impl Drop for InFlight {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

/// One ops e-stop on its way to rcpd: attempts of [`estop_rcpd`] until one
/// returns `Ok`.
struct Delivery {
    target: EstopTarget,
    shared: SharedTrust,
    engager: [u8; 32],
    retry_first: Duration,
    retry_cap: Duration,
    stop: watch::Receiver<bool>,
}

impl Delivery {
    /// Attempt until rcpd answers that its latch closed, then log that at
    /// warn. After a failed attempt it waits `retry_first`, then twice as
    /// long after each further failure, up to `retry_cap`, and logs the
    /// first failure at error and then at most one line per
    /// [`OPS_ESTOP_LOG_EVERY`]. It ends early, with a warn line, when the
    /// daemon stops.
    async fn run(mut self) {
        let mut attempts: u64 = 0;
        let mut retry_in = self.retry_first;
        let mut logged_at: Option<tokio::time::Instant> = None;
        loop {
            attempts += 1;
            let caller = caller_identity(&self.engager, self.shared.account_for_key(&self.engager));
            let attempt = tokio::select! {
                outcome = estop_rcpd(&self.target, caller.clone()) => outcome,
                () = stopping(&mut self.stop) => {
                    stopped_before_rcpd_answered(&caller, attempts);
                    return;
                }
            };
            let reason = match attempt {
                Ok(graph_acknowledged) => {
                    tracing::warn!(
                        by = %caller,
                        attempts,
                        graph_acknowledged,
                        "cerulion_remoted: rcpd's e-stop latch closed by an ops engage-estop"
                    );
                    return;
                }
                Err(reason) => reason,
            };
            if logged_at.is_none_or(|at| at.elapsed() >= OPS_ESTOP_LOG_EVERY) {
                tracing::error!(
                    by = %caller,
                    attempts,
                    %reason,
                    "cerulion_remoted: the estop an ops engage-estop sent to rcpd failed; the \
                     ops e-stop stays engaged and the estop is sent again until rcpd answers"
                );
                logged_at = Some(tokio::time::Instant::now());
            }
            tokio::select! {
                () = tokio::time::sleep(retry_in) => {}
                () = stopping(&mut self.stop) => {
                    stopped_before_rcpd_answered(&caller, attempts);
                    return;
                }
            }
            retry_in = retry_in.saturating_mul(2).min(self.retry_cap);
        }
    }
}

/// Resolves once `stop` reads `true` or its sender is dropped.
async fn stopping(stop: &mut watch::Receiver<bool>) {
    let _ = stop.wait_for(|stopping| *stopping).await;
}

fn stopped_before_rcpd_answered(caller: &str, attempts: u64) {
    tracing::warn!(
        by = %caller,
        attempts,
        "cerulion_remoted: the daemon is stopping before rcpd answered the ops estop; \
         rcpd's own latch may be open"
    );
}

/// One attempt of an ops e-stop at `target`, as `caller`: the token from its
/// token file ([`load_token`]), then `initialize`,
/// `notifications/initialized` and the [`ESTOP_TOOL`] call with
/// [`OPS_ESTOP_REASON`] on an MCP session of its own, each response head
/// within the target's head timeout and each answer within its answer
/// timeout after it, then a `DELETE` of the session rcpd issued. Returns
/// whether the graph acknowledged the e-stop frame.
///
/// # Errors
///
/// Why the `estop` got no answer from rcpd, or rcpd's error answer, its text
/// passed through `sanitize_peer_text`.
async fn estop_rcpd(target: &EstopTarget, caller: String) -> Result<bool, String> {
    let token = load_token(&target.token_file)?;
    let upstream = Upstream {
        addr: target.addr,
        head_timeout: target.head_timeout,
        token,
        caller,
        session_id: Mutex::new(None),
        protocol: Mutex::new(None),
        lost: Notify::new(),
    };
    let outcome = estop_session(&upstream, target.answer_timeout).await;
    if upstream.session_id().is_some() {
        let _ = tokio::time::timeout(CONNECT_TIMEOUT, delete_session(&upstream)).await;
    }
    outcome
}

/// The three messages of [`estop_rcpd`] on `upstream`, each answer within
/// `answer_timeout` after its head.
async fn estop_session(upstream: &Upstream, answer_timeout: Duration) -> Result<bool, String> {
    let initialize = json!({
        "jsonrpc": "2.0",
        "id": OPS_ESTOP_INITIALIZE_ID,
        "method": "initialize",
        "params": {
            "protocolVersion": OPS_ESTOP_PROTOCOL,
            "capabilities": {},
            "clientInfo": { "name": "cerulion-remoted", "version": env!("CARGO_PKG_VERSION") },
        },
    });
    let answer = ops_estop_post(upstream, &initialize, answer_timeout)
        .await?
        .ok_or_else(|| "rcpd answered initialize with no body".to_string())?;
    rpc_result(&answer, "initialize")?;
    record_protocol(upstream, initialize.get("id"), &answer);
    let initialized = json!({ "jsonrpc": "2.0", "method": "notifications/initialized" });
    ops_estop_post(upstream, &initialized, answer_timeout).await?;
    let call = json!({
        "jsonrpc": "2.0",
        "id": OPS_ESTOP_CALL_ID,
        "method": "tools/call",
        "params": { "name": ESTOP_TOOL, "arguments": { "reason": OPS_ESTOP_REASON } },
    });
    let answer = ops_estop_post(upstream, &call, answer_timeout)
        .await?
        .ok_or_else(|| "rcpd answered the estop call with no body".to_string())?;
    estop_outcome(&rpc_result(&answer, ESTOP_TOOL)?)
}

/// Whether the graph acknowledged the e-stop, from the result of rcpd's
/// [`ESTOP_TOOL`] call, whose text block must say `"estopped": true`.
///
/// # Errors
///
/// A result with `isError: true`, or one whose text block does not say
/// `"estopped": true`, its text passed through `sanitize_peer_text`.
fn estop_outcome(result: &Value) -> Result<bool, String> {
    let text = result
        .pointer("/content/0/text")
        .and_then(Value::as_str)
        .unwrap_or("");
    if result.get("isError") == Some(&Value::Bool(true)) {
        return Err(format!(
            "rcpd's estop answered an error: {}",
            cerulion_wireclient::epoch::sanitize_peer_text(text)
        ));
    }
    let body = serde_json::from_str::<Value>(text).unwrap_or(Value::Null);
    if body.get("estopped") != Some(&Value::Bool(true)) {
        return Err(format!(
            "rcpd's estop answer does not say estopped: {}",
            cerulion_wireclient::epoch::sanitize_peer_text(text)
        ));
    }
    Ok(body.get("graph_acknowledged") == Some(&Value::Bool(true)))
}

/// The `result` of the JSON-RPC answer `answer` to `method`.
///
/// # Errors
///
/// An error answer, its message passed through `sanitize_peer_text`, or an
/// answer with no `result`.
fn rpc_result(answer: &[u8], method: &str) -> Result<Value, String> {
    let value: Value = serde_json::from_slice(answer)
        .map_err(|e| format!("rcpd's answer to {method} is not JSON: {e}"))?;
    if let Some(error) = value.get("error") {
        let code = error.get("code").and_then(Value::as_i64).unwrap_or(0);
        let message = error.get("message").and_then(Value::as_str).unwrap_or("");
        return Err(format!(
            "rcpd answered {method} with error {code}: {}",
            cerulion_wireclient::epoch::sanitize_peer_text(message)
        ));
    }
    value
        .get("result")
        .cloned()
        .ok_or_else(|| format!("rcpd's answer to {method} has no result"))
}

/// POST `message` on a fresh connection to rcpd, recording the session id an
/// `initialize` is answered with. The connection and the response head are
/// bounded by the session's head timeout ([`RcpLimits::head_timeout`]), the
/// body by `answer_timeout`. Returns the JSON-RPC message answering a
/// request, or `None` for a notification rcpd accepted.
///
/// # Errors
///
/// Why rcpd gave no answer: no connection, an HTTP error status, a lost
/// session, a body cut off or past the relay bound, or no head or answer in
/// time.
async fn ops_estop_post(
    upstream: &Upstream,
    message: &Value,
    answer_timeout: Duration,
) -> Result<Option<Vec<u8>>, String> {
    let method = message
        .get("method")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_string();
    let sent_session = upstream.session_id().is_some();
    let request = upstream
        .request(Method::POST, "application/json, text/event-stream")
        .header(CONTENT_TYPE, "application/json")
        .body(Full::new(Bytes::from(message.to_string())))
        .map_err(|e| format!("cannot build the rcpd request: {e}"))?;
    let head = async {
        let mut sender = connect::<Full<Bytes>>(upstream.addr).await?;
        sender
            .send_request(request)
            .await
            .map_err(|e| format!("rcpd request failed: {e}"))
    };
    let response = tokio::time::timeout(upstream.head_timeout, head)
        .await
        .map_err(|_| {
            format!(
                "rcpd sent no response head to {method} within {:?}",
                upstream.head_timeout
            )
        })??;
    if method == "initialize" {
        if let Some(sid) = header_text(&response, "mcp-session-id") {
            *upstream
                .session_id
                .lock()
                .unwrap_or_else(PoisonError::into_inner) = Some(sid);
        }
    }
    let status = response.status();
    if status == StatusCode::ACCEPTED {
        return Ok(None);
    }
    if status == StatusCode::NOT_FOUND && sent_session {
        return Err("rcpd lost the e-stop session (it restarted)".to_string());
    }
    if !status.is_success() {
        return Err(format!("rcpd answered HTTP {status}"));
    }
    let Some(id) = message.get("id") else {
        return Ok(None);
    };
    tokio::time::timeout(answer_timeout, answer_of(response, id))
        .await
        .map_err(|_| {
            format!("rcpd did not finish its answer to {method} within {answer_timeout:?}")
        })?
        .map(Some)
}

/// The JSON-RPC message in `response`'s body that answers the request `id`,
/// from a JSON body or any `data:` event of an SSE body, within
/// [`MAX_RESPONSE_MESSAGE`].
async fn answer_of(response: Response<Incoming>, id: &Value) -> Result<Vec<u8>, String> {
    let sse = header_text(&response, CONTENT_TYPE.as_str())
        .is_some_and(|ct| ct.starts_with("text/event-stream"));
    let mut body = response.into_body();
    let mut events = SseBuffer::default();
    let mut whole: Vec<u8> = Vec::new();
    while let Some(next) = body.frame().await {
        let frame = next.map_err(|e| format!("rcpd's answer was cut off: {e}"))?;
        let Ok(chunk) = frame.into_data() else {
            continue;
        };
        if sse {
            let messages = events
                .push(&chunk)
                .map_err(|()| "an event of rcpd's answer exceeded the relay bound".to_string())?;
            if let Some(m) = messages.into_iter().find(|m| answers(Some(id), m)) {
                return Ok(m);
            }
        } else {
            if whole.len() + chunk.len() > MAX_RESPONSE_MESSAGE {
                return Err("rcpd's answer exceeded the relay bound".to_string());
            }
            whole.extend_from_slice(&chunk);
        }
    }
    if answers(Some(id), &whole) {
        return Ok(whole);
    }
    Err("rcpd ended its answer without a response".to_string())
}

/// An incremental `text/event-stream` parser yielding each event's `data`.
#[derive(Debug, Default)]
pub struct SseBuffer {
    pending: Vec<u8>,
    /// How far `pending` is known to hold no event boundary.
    scanned: usize,
}

impl SseBuffer {
    /// Feed `chunk`; return the `data` of every event it completes, in order.
    /// Events with no `data` (priming events, comments) yield nothing.
    ///
    /// # Errors
    ///
    /// `Err(())` when an unfinished event grows past [`MAX_RESPONSE_MESSAGE`],
    /// or a completed event's `data` is longer than it.
    #[allow(clippy::result_unit_err)]
    pub fn push(&mut self, chunk: &[u8]) -> Result<Vec<Vec<u8>>, ()> {
        self.pending
            .extend(chunk.iter().copied().filter(|&b| b != b'\r'));
        let mut out = Vec::new();
        loop {
            let from = self.scanned.saturating_sub(1);
            let Some(at) = self.pending[from..].windows(2).position(|w| w == b"\n\n") else {
                self.scanned = self.pending.len();
                break;
            };
            let end = from + at;
            let event: Vec<u8> = self.pending.drain(..end + 2).collect();
            self.scanned = 0;
            let mut data: Vec<u8> = Vec::new();
            let mut any = false;
            for line in event.split(|&b| b == b'\n') {
                if let Some(rest) = line.strip_prefix(b"data:") {
                    let rest = rest.strip_prefix(b" ").unwrap_or(rest);
                    if any {
                        data.push(b'\n');
                    }
                    data.extend_from_slice(rest);
                    any = true;
                }
            }
            if data.len() > MAX_RESPONSE_MESSAGE {
                return Err(());
            }
            if any && !data.iter().all(u8::is_ascii_whitespace) {
                out.push(data);
            }
        }
        if self.pending.len() > MAX_RESPONSE_MESSAGE {
            return Err(());
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerud::lease::{ControlLease, EstopState};

    fn class_of(text: &str) -> Result<RcpClass, RcpRefusal> {
        classify_message(text.as_bytes()).map(|m| m.class)
    }

    #[test]
    fn requirement_table_is_the_ruled_policy() {
        // Hand oracle: the class table, using only the existing capability bits.
        assert_eq!(
            requirement(RcpClass::Act),
            RcpRequirement {
                required_caps: 0b0010,
                max_role: Role(2)
            }
        );
        assert_eq!(
            requirement(RcpClass::Observe),
            RcpRequirement {
                required_caps: 0b0100,
                max_role: Role(3)
            }
        );
        assert_eq!(
            requirement(RcpClass::Session),
            RcpRequirement {
                required_caps: 0,
                max_role: Role(u16::MAX)
            }
        );
        assert_eq!(
            requirement(RcpClass::Estop),
            RcpRequirement {
                required_caps: 0,
                max_role: Role(u16::MAX)
            }
        );
    }

    #[test]
    fn scope_meets_checks_caps_and_role_independently() {
        let s = |role: u16, caps: u64| Scope {
            role: Role(role),
            caps,
        };
        // Act: TELEOP and OPERATOR-or-better.
        assert!(scope_meets(s(2, 0b0010), RcpClass::Act));
        assert!(scope_meets(s(1, 0b1111), RcpClass::Act));
        assert!(!scope_meets(s(3, 0b1111), RcpClass::Act), "viewer role");
        assert!(!scope_meets(s(2, 0b1101), RcpClass::Act), "no teleop");
        // Observe: OBSERVE and VIEWER-or-better.
        assert!(scope_meets(s(3, 0b0100), RcpClass::Observe));
        assert!(!scope_meets(s(3, 0b1011), RcpClass::Observe), "no observe");
        assert!(
            !scope_meets(s(4, 0b0100), RcpClass::Observe),
            "role past viewer"
        );
        // Session: any role up to VIEWER, no caps.
        assert!(scope_meets(s(3, 0), RcpClass::Session));
        assert!(
            scope_meets(s(u16::MAX, 0), RcpClass::Session),
            "any allowed row"
        );
        // Estop: any scope at all.
        assert!(scope_meets(s(3, 0), RcpClass::Estop));
        assert!(scope_meets(s(u16::MAX, 0), RcpClass::Estop));
    }

    #[test]
    fn methods_classify_by_the_table() {
        let req = |m: &str| format!(r#"{{"jsonrpc":"2.0","id":1,"method":"{m}","params":{{}}}}"#);
        for m in ["initialize", "ping"] {
            assert_eq!(class_of(&req(m)), Ok(RcpClass::Session), "{m}");
        }
        for m in [
            "resources/list",
            "resources/read",
            "resources/templates/list",
            "resources/subscribe",
            "resources/unsubscribe",
            "subscriptions/listen",
            "tools/list",
        ] {
            assert_eq!(class_of(&req(m)), Ok(RcpClass::Observe), "{m}");
        }
        assert_eq!(
            class_of(r#"{"jsonrpc":"2.0","id":"a","method":"tools/call","params":{"name":"x"}}"#),
            Ok(RcpClass::Act)
        );
        assert_eq!(
            class_of(
                r#"{"jsonrpc":"2.0","id":"a","method":"tools/call","params":{"name":"estop"}}"#
            ),
            Ok(RcpClass::Estop)
        );
        for near in ["Estop", "estop ", "estop.x", "e-stop", "estop_clear"] {
            let m = format!(
                r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"{near}"}}}}"#
            );
            assert_eq!(
                class_of(&m),
                Ok(RcpClass::Act),
                "{near:?} is an ordinary tool"
            );
        }
        for m in ["notifications/initialized", "notifications/cancelled"] {
            let n = format!(r#"{{"jsonrpc":"2.0","method":"{m}"}}"#);
            assert_eq!(class_of(&n), Ok(RcpClass::Session), "{m}");
        }
        for m in [
            "prompts/list",
            "completion/complete",
            "logging/setLevel",
            "tools/Call",
            "resources/",
            "",
        ] {
            assert!(class_of(&req(m)).is_err(), "{m} must be refused");
        }
    }

    #[test]
    fn strict_parse_refuses_every_malformed_shape() {
        let cases: &[(&str, Option<Value>)] = &[
            ("not json", Some(Value::Null)),
            ("[]", Some(Value::Null)),
            (
                r#"[{"jsonrpc":"2.0","id":1,"method":"ping"}]"#,
                Some(Value::Null),
            ),
            (r#""ping""#, Some(Value::Null)),
            (
                r#"{"jsonrpc":"2.0","id":null,"method":"ping"}"#,
                Some(Value::Null),
            ),
            (
                r#"{"jsonrpc":"2.0","id":1.5,"method":"ping"}"#,
                Some(Value::Null),
            ),
            (
                r#"{"jsonrpc":"2.0","id":{},"method":"ping"}"#,
                Some(Value::Null),
            ),
            (
                r#"{"jsonrpc":"1.0","id":1,"method":"ping"}"#,
                Some(json!(1)),
            ),
            (r#"{"id":1,"method":"ping"}"#, Some(json!(1))),
            (r#"{"jsonrpc":"2.0","id":1,"method":7}"#, Some(json!(1))),
            (r#"{"jsonrpc":"2.0","id":1}"#, Some(json!(1))),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"ping","params":[]}"#,
                Some(json!(1)),
            ),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"ping","extra":1}"#,
                Some(json!(1)),
            ),
            (r#"{"jsonrpc":"2.0","id":1,"result":{}}"#, Some(json!(1))),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{}}"#,
                Some(json!(1)),
            ),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":3}}"#,
                Some(json!(1)),
            ),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call"}"#,
                Some(json!(1)),
            ),
            // A request method sent as a notification, and the reverse.
            (
                r#"{"jsonrpc":"2.0","method":"tools/call","params":{"name":"x"}}"#,
                None,
            ),
            (
                r#"{"jsonrpc":"2.0","id":2,"method":"notifications/initialized"}"#,
                Some(json!(2)),
            ),
        ];
        for (text, reply) in cases {
            let refusal = classify_message(text.as_bytes()).expect_err(text);
            assert_eq!(&refusal.reply_id, reply, "{text}");
        }
    }

    #[test]
    fn a_duplicate_method_key_forwards_only_what_was_classified() {
        // serde_json keeps the last duplicate; the canonical form carries only it.
        let m = classify_message(
            br#"{"jsonrpc":"2.0","id":1,"method":"tools/call","method":"resources/list"}"#,
        )
        .expect("classified");
        assert_eq!(m.class, RcpClass::Observe);
        let fwd: Value = serde_json::from_slice(&m.canonical).expect("json");
        assert_eq!(fwd["method"], json!("resources/list"));
        assert_eq!(
            String::from_utf8(m.canonical)
                .unwrap()
                .matches("method")
                .count(),
            1
        );
    }

    fn row(role: u16, caps: u64) -> AccessRow {
        AccessRow {
            account: AccountId([3; 32]),
            device: None,
            scope: Scope {
                role: Role(role),
                caps,
            },
            principal_kind: cerulion_pairing::format::PrincipalKind::Human,
            source: cerulion_pairing::verify::PairingSource::StrongChain,
            name: None,
            added_at_ns: 0,
            expires_at_ns: None,
            revoked: false,
        }
    }

    fn allowed(r: &AccessRow) -> KeyAccess {
        KeyAccess::Allowed(r.scope)
    }

    #[test]
    fn the_accept_gate_requires_claimed_and_an_allowing_row() {
        let r = row(3, 0);
        assert_eq!(rcp_authorize(true, allowed(&r), Some(&r), None), Ok(()));
        assert!(rcp_authorize(false, allowed(&r), Some(&r), None).is_err());
        assert!(rcp_authorize(true, allowed(&r), None, None).is_err());
        for a in [
            KeyAccess::Unpaired,
            KeyAccess::NotAllowed,
            KeyAccess::DeviceRevoked,
        ] {
            assert!(rcp_authorize(true, a, None, None).is_err(), "{a:?}");
        }
    }

    #[test]
    fn a_cloud_refusal_wins_over_every_other_check() {
        for claimed in [true, false] {
            for class in [
                None,
                Some(RcpClass::Session),
                Some(RcpClass::Estop),
                Some(RcpClass::Observe),
                Some(RcpClass::Act),
            ] {
                let err = rcp_authorize(claimed, KeyAccess::CloudRefused, None, class)
                    .expect_err("a cloud refusal admits nothing");
                assert!(
                    err.contains("allow-from-cloud switch is off"),
                    "{claimed} {class:?}: {err}"
                );
            }
        }
    }

    #[test]
    fn a_message_needs_the_gate_and_its_class() {
        // (claimed, row role, row caps, class, admitted): hand oracle.
        let cases = [
            (true, 2, 0b0110, RcpClass::Act, true),
            (true, 2, 0b0110, RcpClass::Observe, true),
            (true, 3, 0b0110, RcpClass::Act, false),
            (true, 2, 0b0100, RcpClass::Act, false),
            (true, 3, 0b0100, RcpClass::Observe, true),
            (true, 3, 0b0000, RcpClass::Observe, false),
            (true, 3, 0b0000, RcpClass::Session, true),
            (false, 1, 0b1111, RcpClass::Session, false),
            (true, 3, 0b0000, RcpClass::Estop, true),
            (false, 1, 0b1111, RcpClass::Estop, false),
        ];
        for (claimed, role, caps, class, admitted) in cases {
            let r = row(role, caps);
            assert_eq!(
                rcp_authorize(claimed, allowed(&r), Some(&r), Some(class)).is_ok(),
                admitted,
                "{claimed} role {role} caps {caps:#b} {class:?}"
            );
        }
        for a in [
            KeyAccess::Unpaired,
            KeyAccess::NotAllowed,
            KeyAccess::DeviceRevoked,
        ] {
            assert!(rcp_authorize(true, a, None, Some(RcpClass::Session)).is_err());
        }
    }

    #[test]
    fn sse_buffer_yields_data_across_chunk_boundaries() {
        let mut b = SseBuffer::default();
        assert_eq!(
            b.push(b"id: 0\nretry: 3000\ndata:\n\n").unwrap(),
            Vec::<Vec<u8>>::new()
        );
        assert_eq!(b.push(b"data: {\"a\"").unwrap(), Vec::<Vec<u8>>::new());
        assert_eq!(
            b.push(b":1}\r\n\r\n: ping\n\ndata: x\ndata: y\n\n")
                .unwrap(),
            vec![b"{\"a\":1}".to_vec(), b"x\ny".to_vec()]
        );
    }

    #[test]
    fn an_event_completed_past_the_relay_bound_is_refused() {
        // The event stays within the bound while it is unfinished, and passes
        // it in the chunk that holds its final blank line.
        let mut b = SseBuffer::default();
        let mut first = b"data: ".to_vec();
        first.resize(first.len() + MAX_RESPONSE_MESSAGE - 8, b'a');
        assert_eq!(b.push(&first).unwrap(), Vec::<Vec<u8>>::new());
        assert!(b.push(&[b'a'; 16]).is_err());
        let mut b = SseBuffer::default();
        assert_eq!(b.push(&first).unwrap(), Vec::<Vec<u8>>::new());
        let mut last = vec![b'a'; 16];
        last.extend_from_slice(b"\n\n");
        assert!(b.push(&last).is_err());
        // At the bound the event is relayed.
        let mut b = SseBuffer::default();
        assert_eq!(b.push(&first).unwrap(), Vec::<Vec<u8>>::new());
        let mut last = vec![b'a'; 8];
        last.extend_from_slice(b"\n\n");
        let out = b.push(&last).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len(), MAX_RESPONSE_MESSAGE);
    }

    #[test]
    fn token_file_accepts_bare_and_env_forms_only() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("rcp.token");
        std::fs::write(&p, "abc123\n").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)).unwrap();
        }
        assert_eq!(load_token(&p).unwrap(), "abc123");
        std::fs::write(&p, "CERULION_RCP_TOKEN=xyz\n").unwrap();
        assert_eq!(load_token(&p).unwrap(), "xyz");
        for bad in ["", "  \n", "a b", "CERULION_RCP_TOKEN=", "t\u{7f}"] {
            std::fs::write(&p, bad).unwrap();
            assert!(load_token(&p).is_err(), "{bad:?}");
        }
        assert!(load_token(&dir.path().join("absent")).is_err());
    }

    #[test]
    fn ensure_token_file_creates_once_then_reuses() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("rcp.token");
        ensure_token_file(&p).expect("created");
        let first = std::fs::read_to_string(&p).unwrap();
        let token = load_token(&p).expect("loads");
        assert_eq!(first, format!("CERULION_RCP_TOKEN={token}\n"));
        assert_eq!(token.len(), 64);
        assert!(token.bytes().all(|b| b.is_ascii_hexdigit()));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&p).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600);
        }
        ensure_token_file(&p).expect("reused");
        assert_eq!(
            std::fs::read_to_string(&p).unwrap(),
            first,
            "never rewritten"
        );
        let other = dir.path().join("other.token");
        ensure_token_file(&other).unwrap();
        assert_ne!(
            load_token(&other).unwrap(),
            token,
            "each file gets its own token"
        );
    }

    #[cfg(unix)]
    #[test]
    fn ensure_token_file_refuses_a_shared_or_odd_file() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("rcp.token");
        std::fs::write(&p, "CERULION_RCP_TOKEN=abc\n").unwrap();
        for mode in [0o640, 0o604, 0o620, 0o602, 0o644] {
            std::fs::set_permissions(&p, std::fs::Permissions::from_mode(mode)).unwrap();
            let err = ensure_token_file(&p).expect_err("shared mode refused");
            assert!(err.contains("chmod 600"), "{mode:o}: {err}");
        }
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o400)).unwrap();
        assert!(
            ensure_token_file(&p).is_ok(),
            "an owner-only file is reused"
        );
        let d = dir.path().join("a-dir");
        std::fs::create_dir(&d).unwrap();
        assert!(ensure_token_file(&d).is_err());
        let link = dir.path().join("link.token");
        std::os::unix::fs::symlink(&p, &link).unwrap();
        assert!(ensure_token_file(&link).is_err(), "a symlink is refused");
    }

    #[test]
    fn numbers_that_would_change_on_forwarding_are_refused() {
        let call = |n: &str| {
            format!(
                r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"a","arguments":{{"q":[{n}]}}}}}}"#
            )
        };
        for ok in [
            "0",
            "-1",
            "18446744073709551615",
            "-9223372036854775808",
            "1.5",
            "9007199254740991.0",
        ] {
            assert!(classify_message(call(ok).as_bytes()).is_ok(), "{ok}");
        }
        for bad in ["18446744073709551616", "1e300", "-9007199254740992.0"] {
            assert!(classify_message(call(bad).as_bytes()).is_err(), "{bad}");
        }
    }

    #[cfg(unix)]
    #[test]
    fn load_token_repeats_the_mode_check() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("rcp.token");
        std::fs::write(&p, "abc\n").unwrap();
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)).unwrap();
        assert_eq!(load_token(&p).unwrap(), "abc");
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o644)).unwrap();
        assert!(load_token(&p).is_err());
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)).unwrap();
        let link = dir.path().join("link");
        std::os::unix::fs::symlink(&p, &link).unwrap();
        assert!(load_token(&link).is_err(), "a symlink is refused");
    }

    #[test]
    fn prepare_refuses_a_non_loopback_rcpd() {
        let dir = tempfile::tempdir().unwrap();
        let mut link = RcpLink::under(dir.path());
        link.addr = "10.0.0.5:7801".parse().unwrap();
        assert!(prepare(&link).unwrap_err().contains("loopback"));
        assert!(
            !link.token_file.exists(),
            "refused before any file is written"
        );
        link.addr = "[::1]:7801".parse().unwrap();
        assert!(prepare(&link).is_ok());
        assert!(link.token_file.exists());
    }

    #[tokio::test]
    async fn the_outlet_waits_once_the_byte_budget_is_spent() {
        let (tx, mut rx) = mpsc::channel(8);
        let out = Outlet {
            tx,
            budget: Arc::new(Semaphore::new(10)),
            capacity: 10,
        };
        assert!(out.send(RcpClass::Session, vec![0; 8]).await);
        let second = out.send(RcpClass::Session, vec![0; 8]);
        tokio::pin!(second);
        assert!(
            tokio::time::timeout(Duration::from_millis(100), &mut second)
                .await
                .is_err(),
            "the second frame waits for budget"
        );
        drop(rx.recv().await.expect("first frame"));
        assert!(tokio::time::timeout(Duration::from_secs(5), second)
            .await
            .unwrap());
    }

    #[test]
    fn every_limit_flag_overrides_its_default() {
        use clap::Parser;
        #[derive(Parser)]
        struct Cli {
            #[command(flatten)]
            rcp: RcpFlags,
        }
        let mut link = RcpLink::under(Path::new("/s"));
        assert_eq!(link.limits, RcpLimits::default());
        Cli::try_parse_from([
            "x",
            "--rcp-sessions-per-connection",
            "2",
            "--rcp-connections-per-key",
            "3",
            "--rcp-in-flight",
            "5",
            "--rcp-estops-in-flight",
            "1",
            "--rcp-session-buffer-bytes",
            "20000000",
            "--rcp-revoke-check-ms",
            "250",
            "--rcp-head-timeout-ms",
            "1000",
            "--rcp-drain-ms",
            "0",
        ])
        .unwrap()
        .rcp
        .apply(&mut link);
        assert_eq!(
            link.limits,
            RcpLimits {
                sessions_per_connection: 2,
                connections_per_key: 3,
                in_flight: 5,
                estops_in_flight: 1,
                session_buffer_bytes: 20_000_000,
                revoke_check: Duration::from_millis(250),
                head_timeout: Duration::from_millis(1000),
                drain: Duration::ZERO,
            }
        );
        for bad in [
            &["x", "--rcp-in-flight", "0"][..],
            &["x", "--rcp-session-buffer-bytes", "1024"],
            &["x", "--rcp-revoke-check-ms", "0"],
            &["x", "--rcp-revoke-check-ms", "60001"],
            &["x", "--rcp-sessions-per-connection", "0"],
            &["x", "--rcp-connections-per-key", "0"],
            &["x", "--rcp-estops-in-flight", "0"],
            &["x", "--rcp-head-timeout-ms", "0"],
        ] {
            assert!(Cli::try_parse_from(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn caller_identity_names_device_and_account() {
        assert_eq!(
            caller_identity(&[0xab; 32], Some(AccountId([0x01; 32]))),
            format!("iroh:{}/account:{}", "ab".repeat(32), "01".repeat(32))
        );
        assert_eq!(
            caller_identity(&[0xab; 32], None),
            format!("iroh:{}", "ab".repeat(32))
        );
    }

    #[test]
    fn the_estop_latch_refuses_act_messages_and_fails_closed_on_poison() {
        let call = |name: &str| {
            classify_message(
                format!(
                    r#"{{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{{"name":"{name}","arguments":{{}}}}}}"#
                )
                .as_bytes(),
            )
            .unwrap()
        };
        let (step, clear) = (call("arm.step"), call("estop_clear"));
        assert_eq!(clear.class, RcpClass::Act, "estop_clear stays an Act call");
        let lease = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
        assert_eq!(estop_refusal(&lease, &step), None);
        assert_eq!(estop_refusal(&lease, &clear), None);

        engage_estop(&lease, "iroh:ab/account:01");
        engage_estop(&lease, "iroh:cd");
        assert_eq!(
            lease.lock().unwrap().estop(),
            &EstopState::Engaged {
                by: "iroh:ab/account:01".into()
            },
            "the first engager is kept"
        );
        assert_eq!(
            estop_refusal(&lease, &step),
            Some("refused: the robot's e-stop is engaged")
        );
        assert_eq!(
            estop_refusal(&lease, &clear),
            Some(
                "refused: the robot's ops e-stop is engaged; it clears when cerulion-remoted \
                 restarts"
            )
        );

        // A lock poisoned while clear reads engaged, and an engage on it
        // still engages.
        let poisoned = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
        let p = poisoned.clone();
        let _ = std::thread::spawn(move || {
            let _guard = p.lock().unwrap();
            panic!("poison the lease");
        })
        .join();
        assert!(poisoned.lease().is_poisoned());
        assert_eq!(
            estop_refusal(&poisoned, &step),
            Some("refused: the robot's e-stop is engaged")
        );
        engage_estop(&poisoned, "iroh:ef");
        assert!(matches!(
            poisoned
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .estop(),
            EstopState::Engaged { .. }
        ));
    }

    /// Polls `cond` until it holds or 10 s pass; whether it held. The poll
    /// ends on the condition, the bound only stops a regression.
    fn eventually(cond: impl Fn() -> bool) -> bool {
        let until = std::time::Instant::now() + Duration::from_secs(10);
        while !cond() {
            if std::time::Instant::now() >= until {
                return false;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        true
    }

    #[test]
    fn an_rcp_estop_is_counted_before_it_waits_for_the_lease() {
        let lease = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
        let held = lease.lock().unwrap();
        let engaging = {
            let lease = lease.clone();
            std::thread::spawn(move || engage_estop(&lease, "iroh:ab/account:01"))
        };
        assert!(
            eventually(|| lease.engage_pending()),
            "an rcp estop waiting for the lease's lock is not counted, so an act verb \
             would keep sending"
        );
        drop(held);
        engaging.join().unwrap();
        assert!(!lease.engage_pending(), "the engage was not counted out");
        assert!(matches!(
            lease.lock().unwrap().estop(),
            EstopState::Engaged { .. }
        ));
    }

    #[test]
    fn act_messages_are_refused_while_an_engage_is_counted() {
        let step = classify_message(
            br#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"arm.step","arguments":{}}}"#,
        )
        .unwrap();
        let lease = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
        // An engage counted in and waiting for the lock this test holds: the
        // latch still reads clear under the lock.
        let held = lease.lock().unwrap();
        let engaging = {
            let lease = lease.clone();
            std::thread::spawn(move || lease.engage("ops"))
        };
        assert!(eventually(|| lease.engage_pending()));
        let (tx, rx) = std::sync::mpsc::channel();
        let reading = {
            let lease = lease.clone();
            std::thread::spawn(move || {
                let _ = tx.send(estop_refusal(&lease, &step));
            })
        };
        // The refusal comes from the count, without the lock this test holds.
        let refusal = rx.recv_timeout(Duration::from_secs(10));
        drop(held);
        engaging.join().unwrap();
        reading.join().unwrap();
        assert_eq!(
            refusal,
            Ok(Some(ESTOP_ENGAGED_REFUSAL)),
            "an Act message was not refused while an e-stop engage was counted"
        );
    }

    /// A provisioned robot's trust state with no device bound to an account.
    fn bare_trust() -> SharedTrust {
        use cerulion_pairing::format::{PublicKey, RobotId, RootSet};
        let root = PublicKey(
            ed25519_dalek::SigningKey::from_bytes(&[1; 32])
                .verifying_key()
                .to_bytes(),
        );
        let store = cerulion_pairing::verify::TrustStore::provision(
            RobotId([5; 32]),
            PublicKey([7; 32]),
            RootSet::new(vec![root], 1).unwrap(),
            b"rcp-unit-chassis-secret",
            0,
        )
        .unwrap();
        SharedTrust::new(store, crate::DeviceAccountIndex::new(), Vec::new())
    }

    /// A link to `addr` whose token file under `dir` has `mode`. Its head
    /// timeout outlasts every bounded step, so a send that connected to a
    /// stand-in that never answers is still in flight when a step ends.
    fn link_to(dir: &Path, addr: SocketAddr, mode: u32) -> RcpLink {
        let mut link = RcpLink::under(dir);
        link.addr = addr;
        link.limits.head_timeout = Duration::from_secs(3600);
        std::fs::write(&link.token_file, "CERULION_RCP_TOKEN=unit-token\n").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&link.token_file, std::fs::Permissions::from_mode(mode))
                .unwrap();
        }
        #[cfg(not(unix))]
        let _ = mode;
        link
    }

    async fn within<F: std::future::Future>(what: &str, fut: F) -> F::Output {
        tokio::time::timeout(Duration::from_secs(20), fut)
            .await
            .unwrap_or_else(|_| panic!("'{what}' timed out"))
    }

    /// Wait until the delivery in flight on `rcpd` has ended.
    async fn until_delivered(rcpd: &RcpdEstop) {
        within("the delivery ends", async {
            while rcpd.in_flight.load(Ordering::Acquire) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await;
    }

    /// What the scripted rcpd does on a connection once it has read the
    /// request on it.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Turn {
        /// Close the connection without a byte.
        Close,
        /// Write nothing until the stand-in's release, then close.
        HoldUntilReleased,
        /// Keep the connection open and write nothing.
        Hold,
        /// Write a `200` event-stream head with session id `s`, no body, and
        /// keep the connection open.
        HeadOnly,
        /// Answer `404`, as rcpd answers a session id it does not know.
        Lost,
        /// Answer the request with a JSON-RPC error.
        Error,
        /// Answer as rcpd does.
        Answer,
    }

    /// A loopback rcpd that plays `script` on its connections in order and
    /// answers as rcpd does past it. Each request's kind (its JSON-RPC
    /// method, or `DELETE`) and whole text go on the receiver as it is read.
    struct ScriptedRcpd {
        addr: SocketAddr,
        requests: mpsc::UnboundedReceiver<(String, String)>,
        release: Arc<Notify>,
    }

    impl ScriptedRcpd {
        async fn start(script: Vec<Turn>) -> Self {
            use tokio::io::{AsyncReadExt, AsyncWriteExt};
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let addr = listener.local_addr().unwrap();
            let (tx, requests) = mpsc::unbounded_channel();
            let release = Arc::new(Notify::new());
            let released = release.clone();
            tokio::spawn(async move {
                let mut held = Vec::new();
                let mut turns = script.into_iter();
                while let Ok((mut conn, _)) = listener.accept().await {
                    let mut buf = Vec::new();
                    let mut chunk = [0u8; 4096];
                    let request = loop {
                        if let Some(end) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
                            let head = String::from_utf8_lossy(&buf[..end]).to_ascii_lowercase();
                            let length = head
                                .lines()
                                .find_map(|l| l.strip_prefix("content-length:"))
                                .map_or(0, |v| v.trim().parse::<usize>().unwrap());
                            if buf.len() >= end + 4 + length {
                                break String::from_utf8_lossy(&buf).into_owned();
                            }
                        }
                        match conn.read(&mut chunk).await {
                            Ok(0) | Err(_) => break String::from_utf8_lossy(&buf).into_owned(),
                            Ok(n) => buf.extend_from_slice(&chunk[..n]),
                        }
                    };
                    let body: Value = request
                        .split("\r\n\r\n")
                        .nth(1)
                        .and_then(|b| serde_json::from_str(b).ok())
                        .unwrap_or(Value::Null);
                    let kind = if request.starts_with("DELETE ") {
                        "DELETE".to_string()
                    } else {
                        body["method"].as_str().unwrap_or_default().to_string()
                    };
                    let _ = tx.send((kind.clone(), request));
                    let json = |status: &str, extra: &str, answer: Value| {
                        let text = answer.to_string();
                        format!(
                            "HTTP/1.1 {status}\r\ncontent-type: application/json\r\n{extra}\
                             content-length: {}\r\nconnection: close\r\n\r\n{text}",
                            text.len()
                        )
                    };
                    let empty = |status: &str| {
                        format!(
                            "HTTP/1.1 {status}\r\ncontent-length: 0\r\nconnection: close\r\n\r\n"
                        )
                    };
                    let id = body["id"].clone();
                    let reply = match turns.next().unwrap_or(Turn::Answer) {
                        Turn::Close => None,
                        Turn::HoldUntilReleased => {
                            released.notified().await;
                            None
                        }
                        Turn::Hold => {
                            held.push(conn);
                            continue;
                        }
                        Turn::HeadOnly => {
                            let head = "HTTP/1.1 200 OK\r\ncontent-type: text/event-stream\r\n\
                                        mcp-session-id: s\r\n\r\n";
                            let _ = conn.write_all(head.as_bytes()).await;
                            held.push(conn);
                            continue;
                        }
                        Turn::Lost => Some(empty("404 Not Found")),
                        Turn::Error => Some(json(
                            "200 OK",
                            "",
                            json!({ "jsonrpc": "2.0", "id": id,
                                    "error": { "code": -32603, "message": "unit error" } }),
                        )),
                        Turn::Answer if kind == "DELETE" => Some(empty("200 OK")),
                        Turn::Answer if id.is_null() => Some(empty("202 Accepted")),
                        Turn::Answer if kind == "initialize" => Some(json(
                            "200 OK",
                            "mcp-session-id: s\r\n",
                            json!({ "jsonrpc": "2.0", "id": id,
                                    "result": { "protocolVersion": "2025-06-18" } }),
                        )),
                        Turn::Answer => Some(json(
                            "200 OK",
                            "",
                            json!({ "jsonrpc": "2.0", "id": id, "result": { "content": [{
                                "type": "text",
                                "text": r#"{"estopped":true,"graph_acknowledged":true}"#,
                            }] } }),
                        )),
                    };
                    if let Some(reply) = reply {
                        let _ = conn.write_all(reply.as_bytes()).await;
                    }
                }
            });
            ScriptedRcpd {
                addr,
                requests,
                release,
            }
        }

        /// The kind and text of the next request rcpd reads.
        async fn next(&mut self) -> (String, String) {
            within("rcpd reads a request", self.requests.recv())
                .await
                .expect("the scripted rcpd runs")
        }

        /// The kinds of every request read so far and not taken.
        fn kinds_so_far(&mut self) -> Vec<String> {
            std::iter::from_fn(|| self.requests.try_recv().ok())
                .map(|(kind, _)| kind)
                .collect()
        }
    }

    /// The kinds of one attempt rcpd answers in full.
    ///
    /// A function, not a `const`: the walk in `ci_doc_pin_walk_test` counts a
    /// `tools/`-rooted literal in a `const` the source reads as a read of the
    /// `tools` tree, and `tools/call` is a method name.
    fn attempt() -> [&'static str; 4] {
        [
            "initialize",
            "notifications/initialized",
            "tools/call",
            "DELETE",
        ]
    }

    /// A sender to `rcpd` whose token file is under `dir`.
    fn sender_to(dir: &Path, rcpd: SocketAddr) -> RcpdEstop {
        let link = link_to(dir, rcpd, 0o600);
        RcpdEstop::new(&link, bare_trust(), tokio::runtime::Handle::current())
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn an_ops_estop_that_fails_is_sent_again_with_no_new_engage() {
        let dir = tempfile::tempdir().unwrap();
        // rcpd here accepts each connection and never answers on it.
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let rcpd = sender_to(dir.path(), listener.local_addr().unwrap());
        assert!(rcpd.send(&[1; 32]), "the first engage starts a delivery");
        let (held, _) = within("the estop connects", listener.accept())
            .await
            .unwrap();
        assert!(
            !rcpd.send(&[2; 32]),
            "a second engage started a delivery while one was in flight"
        );
        // rcpd closes the connection: that attempt fails, and the delivery
        // connects again with no new engage.
        drop(held);
        let (_again, _) = within("the estop connects again", listener.accept())
            .await
            .unwrap();
        assert!(!rcpd.send(&[3; 32]), "the delivery ended after one failure");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_second_engage_during_a_failing_attempt_starts_nothing_and_the_estop_arrives() {
        let dir = tempfile::tempdir().unwrap();
        let mut stand_in = ScriptedRcpd::start(vec![Turn::HoldUntilReleased]).await;
        let rcpd = sender_to(dir.path(), stand_in.addr);
        assert!(rcpd.send(&[1; 32]));
        assert_eq!(stand_in.next().await.0, "initialize");
        assert!(!rcpd.send(&[2; 32]), "the second engage started a delivery");
        // The held attempt fails; the one delivery tries again and arrives.
        stand_in.release.notify_one();
        until_delivered(&rcpd).await;
        let mut kinds = vec!["initialize".to_string()];
        kinds.extend(stand_in.kinds_so_far());
        assert_eq!(kinds, [&["initialize"][..], &attempt()[..]].concat());
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn an_ops_estop_answered_404_then_an_error_is_sent_until_rcpd_answers() {
        use Turn::{Answer, Error, Lost};
        let dir = tempfile::tempdir().unwrap();
        let mut stand_in =
            ScriptedRcpd::start(vec![Answer, Answer, Lost, Answer, Answer, Answer, Error]).await;
        let rcpd = sender_to(dir.path(), stand_in.addr);
        assert!(rcpd.send(&[1; 32]));
        until_delivered(&rcpd).await;
        // Each attempt is a session of its own, deleted at its end.
        assert_eq!(stand_in.kinds_so_far(), attempt().repeat(3));
        assert!(rcpd.send(&[2; 32]), "the delivery did not free its place");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_wait_between_attempts_stops_growing_at_its_cap() {
        let dir = tempfile::tempdir().unwrap();
        // Sixteen failed attempts: doubling from 1 ms past a 1 ms cap would
        // wait over 65 s in all.
        let mut stand_in = ScriptedRcpd::start(vec![Turn::Close; 16]).await;
        let mut rcpd = sender_to(dir.path(), stand_in.addr);
        rcpd.retry_first = Duration::from_millis(1);
        rcpd.retry_cap = Duration::from_millis(1);
        assert!(rcpd.send(&[1; 32]));
        until_delivered(&rcpd).await;
        let kinds = stand_in.kinds_so_far();
        assert_eq!(kinds.len(), 16 + attempt().len(), "{kinds:?}");
        assert_eq!(kinds[16..], attempt(), "{kinds:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn an_attempt_that_gets_no_head_ends_at_the_head_timeout() {
        let dir = tempfile::tempdir().unwrap();
        let mut stand_in = ScriptedRcpd::start(vec![Turn::Hold]).await;
        let mut rcpd = sender_to(dir.path(), stand_in.addr);
        rcpd.target.head_timeout = Duration::from_millis(100);
        assert!(rcpd.send(&[1; 32]));
        assert_eq!(stand_in.next().await.0, "initialize");
        // The first connection stays open with no head; the next attempt
        // starts anyway.
        assert_eq!(stand_in.next().await.0, "initialize");
        until_delivered(&rcpd).await;
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn an_attempt_whose_answer_never_ends_stops_at_the_answer_bound_and_deletes_its_session()
    {
        let dir = tempfile::tempdir().unwrap();
        let mut stand_in = ScriptedRcpd::start(vec![Turn::HeadOnly]).await;
        let mut rcpd = sender_to(dir.path(), stand_in.addr);
        rcpd.target.answer_timeout = Duration::from_millis(100);
        assert!(rcpd.send(&[1; 32]));
        assert_eq!(stand_in.next().await.0, "initialize");
        let (kind, delete) = stand_in.next().await;
        assert_eq!(kind, "DELETE", "{delete}");
        assert!(delete.contains("mcp-session-id: s\r\n"), "{delete}");
        assert_eq!(stand_in.next().await.0, "initialize");
        until_delivered(&rcpd).await;
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_delivery_rcpd_never_answers_ends_when_the_daemon_shuts_down() {
        let dir = tempfile::tempdir().unwrap();
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let rcpd = sender_to(dir.path(), listener.local_addr().unwrap());
        assert!(rcpd.send(&[1; 32]));
        let (_held, _) = within("the estop connects", listener.accept())
            .await
            .unwrap();
        within("the delivery stops", rcpd.shut_down()).await;
        assert!(
            !rcpd.in_flight.load(Ordering::Acquire),
            "the delivery outlived the shutdown"
        );
    }

    struct NoCapture;

    impl crate::flashback::EstopCaptureAsk for NoCapture {
        fn ask(&self, _by: &str) {}
    }

    #[cfg(unix)]
    #[tokio::test(flavor = "multi_thread")]
    async fn an_engage_whose_rcpd_token_fails_its_check_engages_and_sends_once_it_passes() {
        use cerud::transport::CallerIdentity;
        use cerud::verbs::VerbHandler;
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        // A token file its group can read fails the check.
        let link = link_to(dir.path(), listener.local_addr().unwrap(), 0o640);
        let lease = EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))));
        let rcpd = Arc::new(RcpdEstop::new(
            &link,
            bare_trust(),
            tokio::runtime::Handle::current(),
        ));
        let refused = estop_rcpd(&rcpd.target, "iroh:ab".into())
            .await
            .unwrap_err();
        assert!(refused.contains("has mode 640"), "{refused}");
        let verb = crate::pairing_verbs::EngageEstopVerb::with_capture_ask(
            lease.clone(),
            Arc::new(NoCapture),
        )
        .with_rcpd_estop(rcpd.clone());
        let engager = CallerIdentity::verified(hex::encode([7u8; 32]));
        let answer = verb
            .execute_with_caller(&engager, &json!({}))
            .expect("the engage answers");
        assert_eq!(answer["engaged"], json!(true), "{answer}");
        assert_eq!(
            lease.lock().unwrap().estop(),
            &EstopState::Engaged {
                by: hex::encode([7u8; 32])
            }
        );
        // Once the file passes its check, the estop reaches rcpd with no
        // second engage.
        std::fs::set_permissions(&link.token_file, std::fs::Permissions::from_mode(0o600)).unwrap();
        let (_conn, _) = within("the estop reaches rcpd", listener.accept())
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn only_the_daemon_given_an_rcp_link_sends_its_estop_to_rcpd() {
        use crate::host::OpsHost;
        let link = RcpLink::under(Path::new("unused-state-dir"));
        let shared = bare_trust();
        assert!(
            matches!(
                OpsHost::Embedded.with_rcpd_estop(Some(&link), &shared),
                OpsHost::Embedded
            ),
            "a library host got an rcpd e-stop sender"
        );
        assert!(matches!(
            OpsHost::daemon().with_rcpd_estop(None, &shared),
            OpsHost::Daemon { rcpd: None, .. }
        ));
        let daemon = OpsHost::daemon().with_rcpd_estop(Some(&link), &shared);
        assert!(
            matches!(daemon, OpsHost::Daemon { rcpd: Some(_), .. }),
            "the daemon with an rcp link sends no estop to rcpd"
        );
        assert!(daemon.rcpd_estop().is_some(), "the daemon cannot stop it");
        assert!(OpsHost::Embedded.rcpd_estop().is_none());
    }

    #[test]
    fn rcpds_error_text_is_sanitized_where_it_enters_the_reason() {
        let answer =
            br#"{"jsonrpc":"2.0","id":1,"error":{"code":-32602,"message":"bad\u001b[2Jcaller"}}"#;
        assert_eq!(
            rpc_result(answer, "estop"),
            Err("rcpd answered estop with error -32602: bad\u{fffd}[2Jcaller".to_string())
        );
        let answer = br#"{"jsonrpc":"2.0","id":1,"result":{"isError":false}}"#;
        assert_eq!(rpc_result(answer, "estop"), Ok(json!({ "isError": false })));
        let refused = json!({ "isError": true,
                              "content": [{ "type": "text", "text": "no\u{7}pe" }] });
        assert_eq!(
            estop_outcome(&refused),
            Err("rcpd's estop answered an error: no\u{fffd}pe".to_string())
        );
        let answered = |body: &str| json!({ "content": [{ "type": "text", "text": body }] });
        assert_eq!(
            estop_outcome(&answered(r#"{"estopped":true,"graph_acknowledged":true}"#)),
            Ok(true)
        );
        assert_eq!(
            estop_outcome(&answered(r#"{"estopped":true,"graph_acknowledged":false}"#)),
            Ok(false)
        );
        assert_eq!(
            estop_outcome(&answered("not\u{1b}json")),
            Err("rcpd's estop answer does not say estopped: not\u{fffd}json".to_string())
        );
    }
}
