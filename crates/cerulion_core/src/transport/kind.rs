// SPDX-License-Identifier: AGPL-3.0-only
//! Transport kind shared by the host and macro-generated cdylibs.

/// The scope supported by this binary's iceoryx2 transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportKind {
    /// Shared-memory communication between processes.
    CrossProcess,
    /// Communication between threads in one process.
    InProcess,
}

/// Bit offset of the transport id inside a cdylib's `cerulion_abi_version()` value.
const TRANSPORT_ABI_SHIFT: u32 = 16;
const _: () = assert!(crate::CERULION_ABI_VERSION < (1 << TRANSPORT_ABI_SHIFT));

impl TransportKind {
    /// Stable identifier exported across the cdylib boundary.
    pub const fn ffi_id(self) -> u32 {
        match self {
            Self::CrossProcess => 0,
            Self::InProcess => 1,
        }
    }

    /// The `cerulion_abi_version()` value a cdylib built for this transport exports:
    /// `CERULION_ABI_VERSION` with the transport id above bit 16. Cross-process nodes export
    /// `CERULION_ABI_VERSION` unchanged; a host that predates the transport id refuses an
    /// in-process node as an ABI mismatch.
    pub const fn cdylib_abi_version(self) -> u32 {
        crate::CERULION_ABI_VERSION | (self.ffi_id() << TRANSPORT_ABI_SHIFT)
    }

    /// Resolve a stable transport identifier.
    pub const fn from_ffi_id(id: u32) -> Option<Self> {
        match id {
            0 => Some(Self::CrossProcess),
            1 => Some(Self::InProcess),
            _ => None,
        }
    }

    /// Human-readable transport description.
    pub const fn describe(self) -> &'static str {
        match self {
            Self::CrossProcess => "cross-process shared memory",
            Self::InProcess => "in-process (the `local-transport` feature)",
        }
    }
}

/// The transport selected when compiling this binary.
#[cfg(not(feature = "local-transport"))]
pub const TRANSPORT_KIND: TransportKind = TransportKind::CrossProcess;

/// The transport selected when compiling this binary.
#[cfg(feature = "local-transport")]
pub const TRANSPORT_KIND: TransportKind = TransportKind::InProcess;

pub(crate) fn cdylib_transport_mismatch(host: TransportKind, node_abi: u32) -> Option<String> {
    let base_mask = (1u32 << TRANSPORT_ABI_SHIFT) - 1;
    if node_abi & base_mask != crate::CERULION_ABI_VERSION {
        return None;
    }

    let id = node_abi >> TRANSPORT_ABI_SHIFT;
    let Some(node) = TransportKind::from_ffi_id(id) else {
        // hot-path-alloc-ok: invalid identifiers are rejected during cdylib loading.
        return Some(format!("node exports unknown transport kind id {id}"));
    };

    if host == node {
        None
    } else {
        // hot-path-alloc-ok: mismatched cdylibs are rejected during loading.
        Some(format!(
            "cdylib transport mismatch: host uses {}, node uses {}; rebuild the node with the \
             same cerulion_core features as the host",
            host.describe(),
            node.describe()
        ))
    }
}

pub(crate) fn refuse_cross_process_peer(what: &str) -> Result<(), String> {
    #[cfg(feature = "local-transport")]
    {
        cross_process_refusal(TRANSPORT_KIND, what)
    }

    #[cfg(not(feature = "local-transport"))]
    {
        let _ = what;
        Ok(())
    }
}

#[cfg(any(feature = "local-transport", test))]
fn cross_process_refusal(kind: TransportKind, what: &str) -> Result<(), String> {
    match kind {
        TransportKind::CrossProcess => Ok(()),
        // hot-path-alloc-ok: refusal text is constructed only during config validation.
        TransportKind::InProcess => Err(format!(
            "this binary is built with the cerulion_core `local-transport` feature, whose \
             transport is in-process only; {what} needs cross-process shared memory. Build \
             without `local-transport`"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        cdylib_transport_mismatch, cross_process_refusal, refuse_cross_process_peer, TransportKind,
    };

    #[test]
    fn transport_ids_and_descriptions_are_stable() {
        assert_eq!(TransportKind::CrossProcess.ffi_id(), 0);
        assert_eq!(TransportKind::InProcess.ffi_id(), 1);
        assert_eq!(
            TransportKind::from_ffi_id(0),
            Some(TransportKind::CrossProcess)
        );
        assert_eq!(
            TransportKind::from_ffi_id(1),
            Some(TransportKind::InProcess)
        );
        assert_eq!(TransportKind::from_ffi_id(u32::MAX), None);
        assert_eq!(
            TransportKind::CrossProcess.describe(),
            "cross-process shared memory"
        );
        assert_eq!(
            TransportKind::InProcess.describe(),
            "in-process (the `local-transport` feature)"
        );
    }

    #[test]
    fn cdylib_abi_version_carries_the_transport_kind() {
        assert_eq!(
            TransportKind::CrossProcess.cdylib_abi_version(),
            crate::CERULION_ABI_VERSION
        );
        assert_eq!(
            TransportKind::InProcess.cdylib_abi_version(),
            crate::CERULION_ABI_VERSION + 65_536
        );
        assert_ne!(
            TransportKind::InProcess.cdylib_abi_version(),
            crate::CERULION_ABI_VERSION
        );
    }

    #[test]
    fn matching_cdylib_transport_kinds_are_accepted() {
        assert_eq!(
            cdylib_transport_mismatch(
                TransportKind::CrossProcess,
                TransportKind::CrossProcess.cdylib_abi_version()
            ),
            None
        );
        assert_eq!(
            cdylib_transport_mismatch(
                TransportKind::InProcess,
                TransportKind::InProcess.cdylib_abi_version()
            ),
            None
        );
    }

    #[test]
    fn mismatched_cdylib_transport_kinds_name_both_sides() {
        assert_eq!(
            cdylib_transport_mismatch(
                TransportKind::InProcess,
                TransportKind::CrossProcess.cdylib_abi_version()
            ),
            Some(
                "cdylib transport mismatch: host uses in-process (the `local-transport` \
                 feature), node uses cross-process shared memory; rebuild the node with the \
                 same cerulion_core features as the host"
                    .to_string()
            )
        );
        assert_eq!(
            cdylib_transport_mismatch(
                TransportKind::CrossProcess,
                TransportKind::InProcess.cdylib_abi_version()
            ),
            Some(
                "cdylib transport mismatch: host uses cross-process shared memory, node uses \
                 in-process (the `local-transport` feature); rebuild the node with the same \
                 cerulion_core features as the host"
                    .to_string()
            )
        );
    }

    #[test]
    fn unknown_cdylib_transport_ids_are_rejected() {
        assert_eq!(
            cdylib_transport_mismatch(
                TransportKind::CrossProcess,
                crate::CERULION_ABI_VERSION | (7 << 16)
            ),
            Some("node exports unknown transport kind id 7".to_string())
        );
    }

    #[test]
    fn different_cdylib_abi_bases_are_reported_as_abi_mismatches() {
        assert_eq!(
            cdylib_transport_mismatch(TransportKind::CrossProcess, 9999),
            None
        );
        assert_eq!(
            cdylib_transport_mismatch(TransportKind::InProcess, 25 | (1 << 16)),
            None
        );
    }

    #[test]
    fn pure_cross_process_refusal_covers_both_kinds() {
        assert_eq!(
            cross_process_refusal(TransportKind::CrossProcess, "a worker"),
            Ok(())
        );
        assert_eq!(
            cross_process_refusal(TransportKind::InProcess, "a worker"),
            Err(
                "this binary is built with the cerulion_core `local-transport` feature, whose \
                 transport is in-process only; a worker needs cross-process shared memory. \
                 Build without `local-transport`"
                    .to_string()
            )
        );
    }

    #[test]
    fn public_refusal_matches_the_build_feature() {
        let result = refuse_cross_process_peer("a worker");
        if cfg!(feature = "local-transport") {
            assert_eq!(
                result,
                Err(
                    "this binary is built with the cerulion_core `local-transport` feature, \
                     whose transport is in-process only; a worker needs cross-process shared \
                     memory. Build without `local-transport`"
                        .to_string()
                )
            );
        } else {
            assert_eq!(result, Ok(()));
        }
    }
}
