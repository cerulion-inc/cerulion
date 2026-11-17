// SPDX-License-Identifier: AGPL-3.0-only
//! The desk half of the robot's act verbs (`topic-publish`, `service-call`,
//! `topic-snapshot`, `topic-echo-once`): dial the robot's ops plane with
//! this desk's paired device key, call ONE verb, and hand back its JSON
//! result or the robot's error.
//!
//! `cerulion topic pub --robot NAME` and its siblings spawn
//! `cerulion-connectd act`, which prints the result on stdout and maps the
//! outcome to an exit code ([`ActError::exit_code`]).
//!
//! Peer text (the robot's error `message`, a denial `reason`) is sanitized
//! and bounded where it enters an [`ActError`].

use std::net::SocketAddr;
use std::time::Duration;

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, EndpointAddr, EndpointConfig,
    EndpointId, QuicOpsStream, RelayConfig,
};
use cerulion_wireclient::epoch::sanitize_peer_text;

use crate::worker::bounded_peer_display;

/// Exit code: the verb ran; its JSON result is on stdout.
pub const EXIT_OK: u8 = 0;
/// Exit code: usage or configuration error on this desk.
pub const EXIT_USAGE: u8 = 1;
/// Exit code: the verb ran and failed; the stderr line starting with
/// [`ERROR_LINE_PREFIX`] carries the robot's `"<class>: <message>"` text.
pub const EXIT_VERB_ERROR: u8 = 2;
/// Exit code: the robot could not be reached in time.
pub const EXIT_UNREACHABLE: u8 = 3;
/// Exit code: the robot refused the call (not paired, or the access row
/// lacks the verb's capability), or does not serve the verb.
pub const EXIT_REFUSED: u8 = 4;

/// The prefix of the one stderr line that carries an [`ActError`]'s text
/// (log lines share stderr).
pub const ERROR_LINE_PREFIX: &str = "act-error: ";

/// The longest peer text an [`ActError`] keeps.
const MAX_PEER_TEXT: usize = 4096;

/// One act-verb call.
pub struct ActConfig {
    /// The robot's endpoint id.
    pub robot_eid: EndpointId,
    /// Direct `ip:port` addresses for a LAN dial; empty dials by id.
    pub direct_addrs: Vec<SocketAddr>,
    /// The desk device key seed the robot paired with.
    pub desk_seed: [u8; 32],
    /// Relay posture.
    pub relay: RelayConfig,
    /// The verb name.
    pub verb: String,
    /// The verb's JSON args.
    pub args: serde_json::Value,
    /// How long the dial, the handshake and the call may take in total.
    pub timeout: Duration,
}

/// Why an act-verb call did not produce a result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActError {
    /// The verb ran and failed; the text is the robot's, sanitized.
    Verb(String),
    /// The robot refused the call or does not serve the verb.
    Refused(String),
    /// The robot could not be reached.
    Unreachable(String),
}

impl std::fmt::Display for ActError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActError::Verb(m) => f.write_str(m),
            ActError::Refused(m) => write!(f, "the robot refused the call: {m}"),
            ActError::Unreachable(m) => write!(f, "the robot could not be reached: {m}"),
        }
    }
}

impl std::error::Error for ActError {}

impl ActError {
    /// The `cerulion-connectd act` exit code for this error.
    pub fn exit_code(&self) -> u8 {
        match self {
            ActError::Verb(_) => EXIT_VERB_ERROR,
            ActError::Refused(_) => EXIT_REFUSED,
            ActError::Unreachable(_) => EXIT_UNREACHABLE,
        }
    }
}

fn peer(text: &str) -> String {
    let clean = sanitize_peer_text(text);
    if clean.len() <= MAX_PEER_TEXT {
        return clean;
    }
    let mut end = MAX_PEER_TEXT;
    while !clean.is_char_boundary(end) {
        end -= 1;
    }
    format!("{}...", &clean[..end])
}

/// Map a `cerud` call error to an [`ActError`].
pub fn classify(err: CerudError) -> ActError {
    match err {
        CerudError::Remote { kind, message } => match kind.as_str() {
            "verb_error" => ActError::Verb(peer(
                message.strip_prefix("verb error: ").unwrap_or(&message),
            )),
            _ => ActError::Refused(peer(&format!("{kind}: {message}"))),
        },
        CerudError::Denied { reason, .. } => ActError::Refused(peer(&reason)),
        CerudError::HandshakeRejected { reason, .. } => ActError::Refused(peer(&reason)),
        other => ActError::Unreachable(bounded_peer_display(&other)),
    }
}

/// Dial the robot, call `config.verb` once and return its JSON result.
pub async fn run_act(config: ActConfig) -> Result<serde_json::Value, ActError> {
    let deadline = config.timeout;
    let attempt = async move {
        let endpoint = build_endpoint(
            EndpointConfig::new(config.desk_seed)
                .with_alpns(vec![alpn::OPS.to_vec()])
                .with_relay(config.relay.clone()),
        )
        .await
        .map_err(|e| ActError::Unreachable(format!("binding the desk endpoint failed: {e}")))?;
        let peer_addr: EndpointAddr = if config.direct_addrs.is_empty() {
            EndpointAddr::new(config.robot_eid)
        } else {
            direct_addr(config.robot_eid, config.direct_addrs.iter().copied())
        };
        let connection = dial(&endpoint, peer_addr, alpn::OPS)
            .await
            .map_err(|e| ActError::Unreachable(format!("dial failed: {e}")))?;
        let (send, recv) = open_frame_stream(&connection)
            .await
            .map_err(|e| ActError::Unreachable(format!("opening the ops stream failed: {e}")))?;
        let verb = config.verb;
        let args = config.args;
        // The sync `OpsClient` runs on a blocking task: its stream's
        // Read/Write may not run on an async worker thread.
        let call = tokio::task::spawn_blocking(move || {
            let stream = QuicOpsStream::new(send, recv);
            let mut client = OpsClient::connect_current(stream).map_err(classify)?;
            client.call(&verb, args).map_err(classify)
        })
        .await
        .map_err(|e| ActError::Unreachable(format!("the call task failed: {e}")))?;
        drop(connection);
        call
    };
    match tokio::time::timeout(deadline, attempt).await {
        Ok(result) => result,
        Err(_) => Err(ActError::Unreachable(format!(
            "no answer within {} s",
            deadline.as_secs_f64()
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_verb_error_keeps_its_class_text_and_other_kinds_are_refusals() {
        let e = classify(CerudError::Remote {
            kind: "verb_error".into(),
            message: "verb error: unavailable: no frame on '/cam' within 5 s".into(),
        });
        assert_eq!(
            e,
            ActError::Verb("unavailable: no frame on '/cam' within 5 s".into())
        );
        assert_eq!(e.exit_code(), EXIT_VERB_ERROR);
        let e = classify(CerudError::Remote {
            kind: "unknown_verb".into(),
            message: "unknown verb: 'topic-publish'".into(),
        });
        assert_eq!(e.exit_code(), EXIT_REFUSED);
        assert!(e.to_string().contains("unknown_verb"), "{e}");
        let e = classify(CerudError::ConnectionClosed);
        assert_eq!(e.exit_code(), EXIT_UNREACHABLE);
    }

    #[test]
    fn peer_text_is_sanitized_and_bounded() {
        let e = classify(CerudError::Remote {
            kind: "verb_error".into(),
            message: format!("usage: \u{1b}[2J{}", "x".repeat(10_000)),
        });
        let ActError::Verb(text) = e else {
            panic!("verb error expected")
        };
        assert!(!text.contains('\u{1b}'), "{text:?}");
        assert!(text.len() <= MAX_PEER_TEXT + 3);
        assert!(text.starts_with("usage: "));
    }
}
