// SPDX-License-Identifier: AGPL-3.0-only
//! The unified crate error enum.

/// Everything that can go wrong bringing up or serving the remote plane.
#[derive(Debug, thiserror::Error)]
pub enum RemotedError {
    /// An error from the iroh link layer (endpoint bind, accept).
    #[error("cerulion_link: {0}")]
    Link(#[from] cerulion_link::LinkError),

    /// The device key could not be loaded (missing file, wrong size). A robot
    /// has no transport identity without it — this is a provisioning gap,
    /// never fabricated.
    #[error("device key: {0}")]
    DeviceKey(String),

    /// The trust store could not be loaded (missing / tampered / corrupt). The
    /// store is factory-provisioned; an absent store is a provisioning gap,
    /// never a silently-fabricated fresh store (a fabricated store with
    /// no recoverable chassis secret would be a BRICKED, unclaimable robot —
    /// Principle #13: no fake data).
    #[error("trust store: {0}")]
    Store(cerulion_pairing::PairingError),

    /// The device→account side-map (R1) could not be loaded/saved. A present but
    /// corrupt index is a LOUD error — it is never silently reset to empty
    /// (that would drop every pairing and could mask tampering; fail-closed).
    #[error("device index: {0}")]
    Index(String),

    /// A configuration value was invalid (e.g. an unparseable relay URL). A
    /// genuine operator config slip, NOT a security/provisioning failure — never
    /// used for a missing trust artifact or its key (see [`RemotedError::MacKey`]).
    #[error("config: {0}")]
    Config(String),

    /// The trust-store MAC key could not be loaded when it was REQUIRED — a trust
    /// store and/or MAC'd device index is present on disk, but the secure-storage
    /// key that authenticates them is missing or unreadable. This is a
    /// PROVISIONING-class / potential-tamper failure (the secret that verifies
    /// this robot's trust state is gone), classified like `DeviceKey` / `Store` /
    /// `Index` — NEVER `Config`. A caller distinguishing "misconfigured" from
    /// "compromised / transplanted trust state" would otherwise blunt the value of
    /// the `check_store_device_binding` transplant rejection + the MAC'd index by
    /// treating a missing verification key as a mere config slip. Fail-closed.
    /// On a genuinely-fresh robot the key is required by nothing and its
    /// absence is benign — the absent store surfaces its own `Store` provisioning
    /// gap instead.
    #[error("trust-store MAC key: {0}")]
    MacKey(String),

    /// The loaded trust store does not belong to THIS device: the endpoint id
    /// (== the device key's public half) does not match the store's recorded
    /// `robot_transport_key`. A transplanted / restored / fleet-misprovisioned
    /// store would otherwise govern the wrong robot (serving robot B under robot
    /// A's owner + access list). A LOUD provisioning-gap refusal, never a silent
    /// boot — the same call the daemon makes for an absent store.
    #[error("provisioning mismatch: {0}")]
    ProvisioningMismatch(String),

    /// Bringing up the ops plane failed (e.g. the ops receipt log could not be
    /// opened). The ops plane is the deploy/inventory/pair surface an always-on
    /// robot must offer at rest, so a failure to stand it up is a LOUD refusal.
    #[error("ops plane: {0}")]
    Ops(String),

    /// The PUBLIC beacon-facts file could not be written / read /
    /// serialized. Unlike the trust store + device index, this carries NO
    /// secrets, so its failure is never a security-class error — and it is
    /// BEST-EFFORT at startup (a write failure is demoted to a warn, never
    /// fatal; the remote plane serves regardless). The message names the path +
    /// the failed operation.
    #[error("beacon facts: {0}")]
    BeaconFacts(String),

    /// An I/O error not otherwise classified.
    #[error("i/o: {0}")]
    Io(#[from] std::io::Error),
}
