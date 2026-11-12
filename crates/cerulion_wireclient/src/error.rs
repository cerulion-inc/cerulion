// SPDX-License-Identifier: AGPL-3.0-only
//! The unified crate error enum.

/// Everything that can go wrong dialing a robot and re-injecting its topics into
/// desk-local SHM.
#[derive(Debug, thiserror::Error)]
pub enum ConnectError {
    /// The robot's `--eid` could not be parsed into a 32-byte iroh
    /// [`EndpointId`](cerulion_link::EndpointId). Names the offending value.
    #[error("robot endpoint id: {0}")]
    Eid(String),

    /// A `--addr` direct socket address could not be parsed. Names the value.
    #[error("robot direct address: {0}")]
    Addr(String),

    /// The desk device key file could not be loaded (missing / wrong size). A
    /// key file, when given, MUST be exactly 32 raw bytes — never fabricated.
    #[error("desk device key: {0}")]
    DeskKey(String),

    /// Could not generate an ephemeral desk device seed (no OS entropy).
    #[error("ephemeral desk key generation: {0}")]
    KeyGen(String),

    /// The cached desk device cert (`~/.cerulion/device.cert`) could not be
    /// read/decoded, or it attests a DIFFERENT device key than the desk holds (the
    /// I1 binding violation — a stale/foreign cert). Names the offending cause; the
    /// caller treats a resolvable-but-mismatched cert as "no account" (loud, never a
    /// silent wrong account).
    #[error("desk device cert: {0}")]
    DeviceCert(String),

    /// The cached owner-signed grant bundle (`~/.cerulion/grants/<robot>.grant`)
    /// could not be read/decoded, or it grants a DIFFERENT account than
    /// the desk's own device cert binds (the operator was handed a grant for another
    /// account). Names the offending cause; the desk presents NO grant rather than a
    /// mismatched one (loud, never a silent wrong grant).
    #[error("owner-signed grant: {0}")]
    OwnerGrant(String),

    /// The cached revocation-epoch artifact (`~/.cerulion/epochs/<robot>.epoch`)
    /// could not be read or decoded. Names the offending cause. The caller
    /// LOGS this and dials anyway — a desk that cannot carry a revocation must still
    /// reach the robot (never block the connection on epoch freshness); the robot
    /// simply keeps the epoch it already has.
    #[error("revocation epoch cache: {0}")]
    EpochSync(String),

    /// An error from the iroh link layer (endpoint bind, dial, stream, framing).
    #[error("cerulion_link: {0}")]
    Link(#[from] cerulion_link::LinkError),

    /// The robot REFUSED the wire plane (unpaired desk key / unclaimed robot /
    /// revoked account / missing `CAP_OBSERVE`). Carries the robot's own stated
    /// reason; the desk exits non-zero. This is NOT a bug — pair the desk with the
    /// robot first (`cerulion pair <robot>`).
    ///
    /// The reason is PEER-CHOSEN text, so the desk-side constructor
    /// (`cerulion_connectd::worker::classify_catalog_reply`) passes it through
    /// [`crate::epoch::sanitize_peer_text`] before it lands here: control characters are
    /// neutered and the text is bounded. It is otherwise the robot's own words —
    /// verbatim in content, never in control bytes.
    #[error("robot refused the wire plane: {0}")]
    Refused(String),

    /// The robot's control-stream reply could not be decoded as either a
    /// [`WireResponse`](crate::protocol::WireResponse) or an accept decision —
    /// a protocol mismatch (incompatible robot / corrupt stream).
    #[error("undecodable robot reply: {0}")]
    Protocol(String),

    /// A control-frame verb failed on the robot (a `WireResponse::Error`), or the
    /// robot answered a verb with an unexpected reply. Carries the actionable
    /// message (which names the topic where relevant).
    ///
    /// When the message comes FROM the robot it is peer-chosen text, so the desk-side
    /// constructor sanitizes + bounds it ([`crate::epoch::sanitize_peer_text`]) before
    /// it lands here — see `Refused`.
    #[error("robot control error: {0}")]
    Control(String),

    /// A desk-local SHM error building the re-injection publisher for a topic
    /// (e.g. the topic is already owned by a live local single-writer producer).
    /// Names the topic + the transport reason.
    #[error("desk ingress for '{topic}': {reason}")]
    Ingress {
        /// The topic whose ingress injector could not be built.
        topic: String,
        /// The underlying transport reason.
        reason: String,
    },

    /// A schema `.msg`/YAML file could not be materialized into the desk store.
    #[error("schema materialize: {0}")]
    Schema(String),

    /// An I/O error not otherwise classified.
    #[error("i/o: {0}")]
    Io(#[from] std::io::Error),
}

/// Crate result alias.
pub type ConnectResult<T> = Result<T, ConnectError>;
