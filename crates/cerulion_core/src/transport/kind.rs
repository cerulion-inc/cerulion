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

impl TransportKind {
    /// Stable identifier exported across the cdylib boundary.
    pub const fn ffi_id(self) -> u32 {
        match self {
            Self::CrossProcess => 0,
            Self::InProcess => 1,
        }
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

#[allow(dead_code)]
pub(crate) fn cdylib_transport_verdict(
    host: TransportKind,
    node_ffi_id: Option<u32>,
) -> Result<(), String> {
    let node = match node_ffi_id {
        Some(id) => TransportKind::from_ffi_id(id).ok_or_else(|| {
            // hot-path-alloc-ok: invalid identifiers are rejected during cdylib loading.
            format!("node exports unknown transport kind id {id}")
        })?,
        None => TransportKind::CrossProcess,
    };

    if host == node {
        Ok(())
    } else {
        // hot-path-alloc-ok: mismatched cdylibs are rejected during loading.
        Err(format!(
            "cdylib transport mismatch: host uses {}, node uses {}; rebuild the node with the \
             same cerulion_core features as the host",
            host.describe(),
            node.describe()
        ))
    }
}

#[allow(dead_code)]
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
#[allow(dead_code)]
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
        cdylib_transport_verdict, cross_process_refusal, refuse_cross_process_peer, TransportKind,
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
    fn matching_transport_kinds_are_accepted() {
        assert_eq!(
            cdylib_transport_verdict(TransportKind::CrossProcess, Some(0)),
            Ok(())
        );
        assert_eq!(
            cdylib_transport_verdict(TransportKind::InProcess, Some(1)),
            Ok(())
        );
    }

    #[test]
    fn missing_symbol_means_cross_process_transport() {
        assert_eq!(
            cdylib_transport_verdict(TransportKind::CrossProcess, None),
            Ok(())
        );
        assert_eq!(
            cdylib_transport_verdict(TransportKind::InProcess, None),
            Err(
                "cdylib transport mismatch: host uses in-process (the `local-transport` \
                 feature), node uses cross-process shared memory; rebuild the node with the \
                 same cerulion_core features as the host"
                    .to_string()
            )
        );
    }

    #[test]
    fn mismatched_transport_kinds_name_both_sides() {
        assert_eq!(
            cdylib_transport_verdict(TransportKind::CrossProcess, Some(1)),
            Err(
                "cdylib transport mismatch: host uses cross-process shared memory, node uses \
                 in-process (the `local-transport` feature); rebuild the node with the same \
                 cerulion_core features as the host"
                    .to_string()
            )
        );
        assert_eq!(
            cdylib_transport_verdict(TransportKind::InProcess, Some(0)),
            Err(
                "cdylib transport mismatch: host uses in-process (the `local-transport` \
                 feature), node uses cross-process shared memory; rebuild the node with the \
                 same cerulion_core features as the host"
                    .to_string()
            )
        );
    }

    #[test]
    fn unknown_transport_ids_are_rejected() {
        assert_eq!(
            cdylib_transport_verdict(TransportKind::CrossProcess, Some(7)),
            Err("node exports unknown transport kind id 7".to_string())
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
