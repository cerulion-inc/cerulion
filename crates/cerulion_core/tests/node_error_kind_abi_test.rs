// SPDX-License-Identifier: AGPL-3.0-only
use cerulion_core::{error::NodeErrorKind, node_abi::error_kind};

#[test]
fn native_node_error_kinds_equal_frozen_boundary_tags() {
    for (kind, tag) in [
        (NodeErrorKind::Logic, error_kind::LOGIC),
        (NodeErrorKind::InvalidInput, error_kind::INVALID_INPUT),
        (NodeErrorKind::Fatal, error_kind::FATAL),
        (NodeErrorKind::Custom, error_kind::CUSTOM),
        (NodeErrorKind::Transport, error_kind::TRANSPORT),
        (NodeErrorKind::Io, error_kind::IO),
    ] {
        assert_eq!(kind.code(), tag);
        assert_eq!(NodeErrorKind::from_code(tag), Some(kind));
    }
}
