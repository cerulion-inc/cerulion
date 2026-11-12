// SPDX-License-Identifier: AGPL-3.0-only
use crate::*;

/// Synchronous state sink. Context and chunk borrows end when write returns.
/// Frozen concrete callback record, not a table: no header negotiation. Callee
/// validates record pointer/alignment and never retains it after capture returns.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct StateSinkV1 {
    /// Host-owned opaque callback context; only its owner dereferences it.
    pub context: *mut u8,
    /// Node bytes remain node-owned; sink neither retains nor frees them.
    pub write: unsafe extern "C" fn(*mut u8, Bytes) -> Status,
}

/// Immutable canonical state extension; a null pointer means capability absent.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct StateApiV1 {
    /// Complete state extension identity.
    pub header: TableHeaderV1,
    /// Canonical state shape hash.
    pub shape: unsafe extern "C" fn(*mut u64) -> Status,
    /// Capture canonical bytes. Sink lives for the call; u64 is native capacity_hint.
    pub capture: unsafe extern "C" fn(NodeHandle, *const StateSinkV1, u64) -> Status,
    /// Restore borrowed canonical bytes, validating shape before mutation.
    pub restore: unsafe extern "C" fn(NodeHandle, Bytes) -> Status,
    /// Zero or one in caller-owned output storage.
    pub inline_safe: unsafe extern "C" fn(NodeHandle, *mut u32) -> Status,
    /// Existing state-carrier eligibility probe.
    pub probe: unsafe extern "C" fn(NodeHandle) -> Status,
}

/// External source kind and ownership flags.
pub mod external {
    /// Node-owned device fd, borrowed by the host until deregistration.
    pub const DEVICE_FD: u32 = 1;
    /// Helper-doorbell read fd whose close obligation passes to the host.
    pub const DOORBELL_FD: u32 = 2;
    /// Existing host-driven ingress; fd is ignored.
    pub const HOST_DRIVEN: u32 = 3;
    /// Host closes the returned read fd after poll deregistration.
    pub const HOST_CLOSES: u32 = 1;
    /// Non-cancellable helper retains library and host context until process exit.
    pub const RETAIN_PROCESS: u32 = 2;
}

/// Primitive ingress source; no closure or Rust fd wrapper crosses.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ExternalSourceV1 {
    /// Raw fd interpreted only when kind specifies an fd source.
    pub fd: i64,
    /// External source tag.
    pub kind: u32,
    /// Explicit ownership/retention bits from external.
    pub flags: u32,
}

/// Immutable external ingress extension.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ExternalApiV1 {
    /// Complete external extension identity.
    pub header: TableHeaderV1,
    /// Queried once at live entry; replay supplies recorded ingress instead.
    pub describe: unsafe extern "C" fn(NodeHandle, *mut ExternalSourceV1) -> Status,
}

/// Library-lifetime node interface. Appending fields requires a new type, never
/// prefix acceptance. New services belong in separately versioned extensions.
/// Host serializes every NodeApi, StateApi and ExternalApi entry for one handle.
/// Helpers may call raw HostApi services concurrently, never ordinary tick ports.
/// Concurrent instance entry is INVALID_STATE, not a transient retry condition.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct NodeApiV1 {
    /// Complete node table identity.
    pub header: TableHeaderV1,
    /// Existing metadata JSON, immutable for library lifetime.
    pub metadata: Bytes,
    /// Host capabilities this node needs to preserve its declared behavior.
    pub required_host_capabilities: u64,
    /// Node capabilities, matching its non-null extension pointers exactly.
    pub capabilities: u64,
    /// Allocate/init node-owned state. Host table remains process-lifetime;
    /// params are borrowed for this call. Zero handle is never a success.
    /// Non-OK frees partial state and leaves no instance; host never destroys it.
    pub create:
        unsafe extern "C" fn(*const HostApiV1, ContextHandle, Bytes, *mut NodeHandle) -> Status,
    /// User tick; failure never arms unsent outputs.
    pub tick: unsafe extern "C" fn(NodeHandle) -> Status,
    /// User shutdown while host services remain live.
    pub shutdown: unsafe extern "C" fn(NodeHandle) -> Status,
    /// Node allocator destroys its own state after all calls/helpers finish.
    /// Called exactly once after successful create, even if shutdown failed.
    pub destroy: unsafe extern "C" fn(NodeHandle) -> Status,
    /// Copy library-scoped diagnostic. One pending slot per instance plus one for
    /// create failures; new failure replaces that slot, full copy consumes it,
    /// stale token returns INVALID_HANDLE, zero names none. Short copy retains.
    pub copy_error: unsafe extern "C" fn(u64, MutableBytes, *mut u64) -> Status,
    /// Query a separately versioned extension; unknown IDs return unsupported.
    pub query_extension: unsafe extern "C" fn(u64, u32, *mut ExtensionDescriptorV1) -> Status,
    /// Null or a complete immutable StateApiV1.
    pub state: *const StateApiV1,
    /// Null or a complete immutable ExternalApiV1.
    pub external: *const ExternalApiV1,
}

// SAFETY: sharing this descriptor never dereferences its raw pointers. All
// pointer use is through the documented unsafe boundary calls; exports require
// immutable library-lifetime metadata/extensions and per-instance exclusion.
// No writable sample or node state lives in the descriptor itself.
unsafe impl Sync for NodeApiV1 {}
