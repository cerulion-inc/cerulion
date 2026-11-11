// SPDX-License-Identifier: AGPL-3.0-only
use crate::*;
use stabby::IStable;

/// Rejection before any plugin initialization or transport side effect.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum AbiError {
    /// Boundary majors differ.
    #[error("node boundary major {found} is unsupported; expected {expected}")]
    Major {
        /// Node boundary.
        found: u32,
        /// Host boundary.
        expected: u32,
    },
    /// A full immutable table has a different size.
    #[error("node boundary table size {found} differs from expected {expected}")]
    Size {
        /// Reported size.
        found: u32,
        /// Complete local size.
        expected: u32,
    },
    /// The complete Stabby report differs.
    #[error("node boundary table layout differs from the supported interface")]
    Layout,
    /// An extension required by the node is unavailable on the host.
    #[error("node requires unsupported host capabilities {missing:#x}")]
    Capabilities {
        /// Missing required bits.
        missing: u64,
    },
    /// Capability bits and extension presence disagree.
    #[error("node capability bits disagree with its extension tables")]
    Extensions,
}

/// Validate the complete immutable table, never a prefix or only field offsets.
pub fn validate_header<T: IStable>(header: &TableHeaderV1) -> Result<(), AbiError> {
    if header.boundary_major != BOUNDARY_MAJOR {
        return Err(AbiError::Major {
            found: header.boundary_major,
            expected: BOUNDARY_MAJOR,
        });
    }
    let expected = core::mem::size_of::<T>() as u32;
    if header.size_bytes != expected {
        return Err(AbiError::Size {
            found: header.size_bytes,
            expected,
        });
    }
    if header.layout_id != T::ID {
        return Err(AbiError::Layout);
    }
    Ok(())
}

/// Ignore unknown optional bits; refuse every unsupported required bit.
pub fn negotiate(required: u64, available: u64) -> Result<(), AbiError> {
    let missing = required & !available;
    if missing != 0 {
        return Err(AbiError::Capabilities { missing });
    }
    Ok(())
}

/// Verify advertised state/external extensions without dereferencing pointers.
pub fn validate_extensions(api: &NodeApiV1) -> Result<(), AbiError> {
    extension_presence(
        api.capabilities,
        !api.state.is_null(),
        !api.external.is_null(),
    )
}

fn extension_presence(bits: u64, state: bool, external: bool) -> Result<(), AbiError> {
    if state != (bits & capability::STATE != 0) || external != (bits & capability::EXTERNAL != 0) {
        return Err(AbiError::Extensions);
    }
    Ok(())
}

/// Supplement Stabby's prefix report comparison with the complete report ID.
pub fn bootstrap_matches(report: &stabby::abi::report::TypeReport) -> bool {
    stabby::abi::report::gen_id(report) == BootstrapFn::ID
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_complete_table_and_refuses_every_header_error() {
        let header = TableHeaderV1::for_type::<HostApiV1>();
        assert_eq!(validate_header::<HostApiV1>(&header), Ok(()));
        let mut bad = header;
        bad.boundary_major = 2;
        assert_eq!(
            validate_header::<HostApiV1>(&bad),
            Err(AbiError::Major {
                found: 2,
                expected: 1
            })
        );
        bad = header;
        bad.size_bytes = 0;
        assert_eq!(
            validate_header::<HostApiV1>(&bad),
            Err(AbiError::Size {
                found: 0,
                expected: header.size_bytes
            })
        );
        bad = header;
        bad.layout_id ^= 1;
        assert_eq!(validate_header::<HostApiV1>(&bad), Err(AbiError::Layout));
    }

    #[test]
    fn ignores_unused_capabilities_and_refuses_unknown_required_bits() {
        assert_eq!(negotiate(0, u64::MAX), Ok(()));
        assert_eq!(negotiate(capability::STATE, capability::KNOWN), Ok(()));
        assert_eq!(
            negotiate(1 << 63, capability::KNOWN),
            Err(AbiError::Capabilities { missing: 1 << 63 })
        );
    }

    #[test]
    fn extension_bits_match_presence_and_ignore_unknown_optional_bits() {
        assert_eq!(extension_presence(0, false, false), Ok(()));
        assert_eq!(extension_presence(1 << 63, false, false), Ok(()));
        assert_eq!(extension_presence(capability::STATE, true, false), Ok(()));
        for (bits, state, external) in [
            (0, true, false),
            (0, false, true),
            (1, false, false),
            (2, false, false),
        ] {
            assert_eq!(
                extension_presence(bits, state, external),
                Err(AbiError::Extensions)
            );
        }
    }

    #[test]
    fn complete_bootstrap_identity_is_deterministic_and_not_another_signature() {
        let report = BootstrapFn::REPORT;
        assert!(bootstrap_matches(report));
        assert!(bootstrap_matches(report));
        assert!(!bootstrap_matches(<extern "C" fn() -> u32>::REPORT));
        assert_eq!(core::mem::size_of::<Status>(), 16);
        assert_eq!(core::mem::size_of::<TableHeaderV1>(), 16);
        assert_eq!(status_code::OK, 0);
        assert_eq!(status_code::EMPTY, 1);
        assert_eq!(status_code::BUFFER_TOO_SMALL, 8);
    }

    #[test]
    fn frozen_port_event_accumulator_and_sync_tags_match_wire_oracles() {
        assert_eq!(core::mem::size_of::<PortBindingV1>(), 32);
        assert_eq!(core::mem::size_of::<AccumulateInfoV1>(), 24);
        assert_eq!(core::mem::size_of::<EventV1>(), 88);
        assert_eq!(
            [
                head_op::FILL_BOUNDARY,
                head_op::FILL_REFILL,
                head_op::PROBE_NEXT,
                head_op::PEEK_BOUNDARY,
                head_op::PEEK_REFILL,
                head_op::ADVANCE_BOUNDARY,
                head_op::ADVANCE_REFILL,
                head_op::VOID,
                head_op::DRAIN,
                head_op::REFILL
            ],
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        );
        assert_eq!(
            [
                head_kind::NOTHING,
                head_kind::PRESENT,
                head_kind::STAMP,
                head_kind::HEAD,
                head_kind::FAILED,
                head_kind::WITHHELD
            ],
            [0, 1, 2, 3, 4, 5]
        );
        assert_eq!(
            [
                event_kind::BACKPRESSURE,
                event_kind::EXPECT_WITHIN,
                event_kind::PROMISE_WITHIN,
                event_kind::LIVELINESS
            ],
            [1, 2, 3, 4]
        );
        assert_eq!(
            [
                event_detail::DROP_OLDEST,
                event_detail::BLOCK,
                event_detail::SAMPLE,
                event_detail::ALIVE,
                event_detail::LOST,
                event_detail::CONNECTED,
                event_detail::DISCONNECTED
            ],
            [0, 1, 2, 1, 2, 1, 2]
        );
        assert_eq!(
            [discard_reason::SPILL_RELEASE, discard_reason::INVARIANT],
            [6, 7]
        );
    }
}
