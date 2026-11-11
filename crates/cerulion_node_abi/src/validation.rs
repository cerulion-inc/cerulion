// SPDX-License-Identifier: AGPL-3.0-only
use crate::*;
use core::mem::{align_of, offset_of, size_of};

fn aligned<T>(ptr: *const T) -> Result<(), AbiError> {
    if ptr.is_null() || !(ptr as usize).is_multiple_of(align_of::<T>()) {
        return Err(AbiError::Descriptor {
            reason: "null or misaligned table pointer",
        });
    }
    Ok(())
}

// All supported POSIX targets use one native pointer word for C function
// pointers. Read initialized callback representation as an integer BEFORE any
// Rust reference/value containing a possibly-null function pointer is formed.
unsafe fn callback_present<T>(ptr: *const T) -> bool {
    assert_eq!(size_of::<T>(), size_of::<usize>());
    // SAFETY: caller points at one initialized callback field of a complete live
    // table. Width was checked; every integer bit pattern is valid, including 0.
    unsafe { ptr.cast::<usize>().read_unaligned() != 0 }
}

macro_rules! callbacks {
    ($ptr:ident, $table:ident, $($field:ident),+ $(,)?) => {
        $(
            // SAFETY: full table allocation and header were checked before this
            // projection; addr_of forms no reference to an invalid callback.
            let present = unsafe { callback_present(core::ptr::addr_of!((*$ptr).$field)) };
            if !present {
                return Err(AbiError::Callback {
                    table: stringify!($table), callback: stringify!($field),
                });
            }
        )+
    };
}

/// Validate and copy an immutable node table without first borrowing null callbacks.
///
/// # Safety
/// Non-null aligned pointers must name live initialized allocations covering the
/// header, and the complete advertised table if its header matches. Metadata and
/// advertised extension storage must obey the same library-lifetime contract.
/// Rejected null pointers and malformed scalar lengths need no backing span;
/// accepted metadata spans must retain their initialized readable allocation.
///
/// # Errors
/// Every malformed major, size, layout, span, extension and required callback.
pub unsafe fn read_node_api(ptr: *const NodeApiV1) -> Result<NodeApiV1, AbiError> {
    aligned(ptr)?;
    // SAFETY: caller guarantees readable header storage. Header contains only
    // scalars and is the first repr(C) field, so no callback validity is assumed.
    let header = unsafe { ptr.cast::<TableHeaderV1>().read() };
    validate_header::<NodeApiV1>(&header)?;
    callbacks!(
        ptr,
        NodeApiV1,
        create,
        tick,
        shutdown,
        destroy,
        copy_error,
        query_extension
    );
    // SAFETY: caller retains complete initialized table storage; all required
    // function pointers are now non-null. Remaining fields admit all bit patterns.
    let api = unsafe { ptr.read() };
    let metadata = api.metadata;
    if metadata.len == 0 || metadata.len > isize::MAX as u64 || metadata.ptr.is_null() {
        return Err(AbiError::Descriptor {
            reason: "metadata must be a nonempty initialized byte span",
        });
    }
    validate_extensions(&api)?;
    if !api.state.is_null() {
        let ptr = api.state;
        aligned(ptr)?;
        // SAFETY: caller retains extension header/storage; validate its complete
        // identity before projecting callbacks or borrowing the concrete table.
        let header = unsafe { ptr.cast::<TableHeaderV1>().read() };
        validate_header::<StateApiV1>(&header)?;
        callbacks!(ptr, StateApiV1, shape, capture, restore, inline_safe, probe);
    }
    if !api.external.is_null() {
        let ptr = api.external;
        aligned(ptr)?;
        // SAFETY: caller retains extension header/storage; exact identity proves
        // the complete field layout before checking its one required callback.
        let header = unsafe { ptr.cast::<TableHeaderV1>().read() };
        validate_header::<ExternalApiV1>(&header)?;
        callbacks!(ptr, ExternalApiV1, describe);
    }
    Ok(api)
}

/// Check a host table before a node forms any Rust reference to its callbacks.
///
/// # Safety
/// Pointer storage follows read_node_api's complete initialized allocation rule.
/// The host must keep the immutable table alive through every node/helper call.
///
/// # Errors
/// Null/misaligned pointers, incompatible headers and null mandatory callbacks.
pub unsafe fn read_host_api(ptr: *const HostApiV1) -> Result<HostApiV1, AbiError> {
    aligned(ptr)?;
    // SAFETY: caller guarantees readable header; it contains only scalar fields.
    let header = unsafe { ptr.cast::<TableHeaderV1>().read() };
    validate_header::<HostApiV1>(&header)?;
    callbacks!(
        ptr,
        HostApiV1,
        bind_port,
        loan_output,
        finish_output,
        submit_spill,
        discard_output,
        receive_input,
        release_input,
        input_head,
        set_snapshot_inputs,
        snapshot_inputs,
        pump_history,
        clock,
        request_shutdown,
        environment,
        poll_event,
        publisher_counters,
        copy_error,
        diagnostic,
        accumulate_count,
        accumulate_frame,
        mark_served,
        capture_accumulate,
        query_extension,
        restore_accumulate,
        create_raw_publisher,
        finish_raw_output,
        register_raw_egress,
        destroy_raw_publisher
    );
    // SAFETY: all callbacks are non-null, complete table storage is initialized
    // and live, and every other host-table field is an unrestricted integer.
    Ok(unsafe { ptr.read() })
}

const _: () = assert!(offset_of!(NodeApiV1, header) == 0);
const _: () = assert!(offset_of!(HostApiV1, header) == 0);
const _: () = assert!(offset_of!(StateApiV1, header) == 0);
const _: () = assert!(offset_of!(ExternalApiV1, header) == 0);

#[cfg(test)]
mod tests {
    use super::*;

    unsafe extern "C" fn create(
        _host: *const HostApiV1,
        _context: ContextHandle,
        _params: Bytes,
        _out: *mut NodeHandle,
    ) -> Status {
        Status {
            code: status_code::UNSUPPORTED,
            ..Status::default()
        }
    }
    unsafe extern "C" fn instance(_node: NodeHandle) -> Status {
        Status::default()
    }
    unsafe extern "C" fn error(_token: u64, _out: MutableBytes, _len: *mut u64) -> Status {
        Status {
            code: status_code::UNSUPPORTED,
            ..Status::default()
        }
    }
    unsafe extern "C" fn query(
        _id: u64,
        _version: u32,
        _out: *mut ExtensionDescriptorV1,
    ) -> Status {
        Status {
            code: status_code::UNSUPPORTED,
            ..Status::default()
        }
    }

    fn node() -> NodeApiV1 {
        NodeApiV1 {
            header: TableHeaderV1::for_type::<NodeApiV1>(),
            metadata: Bytes {
                ptr: b"{}".as_ptr(),
                len: 2,
            },
            required_host_capabilities: 0,
            capabilities: 0,
            create,
            tick: instance,
            shutdown: instance,
            destroy: instance,
            copy_error: error,
            query_extension: query,
            state: core::ptr::null(),
            external: core::ptr::null(),
        }
    }

    #[test]
    fn valid_descriptor_is_stable_across_two_reads() {
        let api = node();
        for _ in 0..2 {
            // SAFETY: actual initialized immutable descriptor with static bytes
            // and correctly typed callbacks remains live through the copy.
            let copy = unsafe { read_node_api(&api) }.expect("complete descriptor");
            assert_eq!(copy.header, TableHeaderV1::for_type::<NodeApiV1>());
            assert_eq!(copy.metadata.len, 2);
            assert_eq!(copy.capabilities, 0);
        }
    }

    #[test]
    fn null_and_misaligned_tables_refuse_without_dereferencing() {
        for ptr in [core::ptr::null(), core::ptr::without_provenance(1)] {
            // SAFETY: these are the explicitly rejected null/misaligned cases;
            // read_node_api must return before accessing pointed-to storage.
            assert!(matches!(
                unsafe { read_node_api(ptr) },
                Err(AbiError::Descriptor { .. })
            ));
        }
    }

    #[test]
    fn short_advertised_header_refuses_before_reading_the_table() {
        let mut header = TableHeaderV1::for_type::<NodeApiV1>();
        header.size_bytes = size_of::<TableHeaderV1>() as u32;
        // SAFETY: only header storage exists. Its incompatible size must refuse
        // before accessing any callback or other complete-table field.
        assert!(matches!(
            unsafe { read_node_api(core::ptr::from_ref(&header).cast()) },
            Err(AbiError::Size { .. })
        ));
    }

    #[test]
    fn every_missing_node_callback_refuses_before_typed_copy() {
        let api = node();
        for (offset, name) in [
            (offset_of!(NodeApiV1, create), "create"),
            (offset_of!(NodeApiV1, tick), "tick"),
            (offset_of!(NodeApiV1, shutdown), "shutdown"),
            (offset_of!(NodeApiV1, destroy), "destroy"),
            (offset_of!(NodeApiV1, copy_error), "copy_error"),
            (offset_of!(NodeApiV1, query_extension), "query_extension"),
        ] {
            let mut raw = vec![
                core::mem::MaybeUninit::<usize>::zeroed();
                size_of::<NodeApiV1>().div_ceil(size_of::<usize>())
            ];
            // SAFETY: aligned storage covers the complete descriptor. Raw copy
            // preserves initialized fields without interpreting any padding;
            // zeroing only the chosen callback makes a malformed C table. The
            // validator must refuse before constructing a typed NodeApi value.
            let result = unsafe {
                let bytes = raw.as_mut_ptr().cast::<u8>();
                core::ptr::copy_nonoverlapping(
                    core::ptr::from_ref(&api).cast::<u8>(),
                    bytes,
                    size_of::<NodeApiV1>(),
                );
                bytes.add(offset).cast::<usize>().write(0);
                read_node_api(bytes.cast())
            };
            assert!(
                matches!(result, Err(AbiError::Callback { table: "NodeApiV1", callback }) if callback == name)
            );
        }
    }

    #[test]
    fn invalid_metadata_spans_and_partial_extensions_refuse() {
        for (metadata, expected_extension) in [
            (
                Bytes {
                    ptr: core::ptr::null(),
                    len: 2,
                },
                false,
            ),
            (
                Bytes {
                    ptr: b"{}".as_ptr(),
                    len: 0,
                },
                false,
            ),
            (
                Bytes {
                    ptr: b"{}".as_ptr(),
                    len: u64::MAX,
                },
                false,
            ),
            (
                Bytes {
                    ptr: b"{}".as_ptr(),
                    len: 2,
                },
                true,
            ),
        ] {
            let mut api = node();
            api.metadata = metadata;
            if expected_extension {
                api.capabilities = capability::STATE;
            }
            // SAFETY: actual descriptor and callback storage remain initialized
            // and live; rejected length spans are never dereferenced.
            let result = unsafe { read_node_api(&api) };
            if expected_extension {
                assert!(matches!(result, Err(AbiError::Extensions)));
            } else {
                assert!(matches!(result, Err(AbiError::Descriptor { .. })));
            }
        }
    }

    #[test]
    fn every_missing_host_callback_refuses_before_any_rust_function_value_exists() {
        let fields = [
            (offset_of!(HostApiV1, bind_port), "bind_port"),
            (offset_of!(HostApiV1, loan_output), "loan_output"),
            (offset_of!(HostApiV1, finish_output), "finish_output"),
            (offset_of!(HostApiV1, submit_spill), "submit_spill"),
            (offset_of!(HostApiV1, discard_output), "discard_output"),
            (offset_of!(HostApiV1, receive_input), "receive_input"),
            (offset_of!(HostApiV1, release_input), "release_input"),
            (offset_of!(HostApiV1, input_head), "input_head"),
            (
                offset_of!(HostApiV1, set_snapshot_inputs),
                "set_snapshot_inputs",
            ),
            (offset_of!(HostApiV1, snapshot_inputs), "snapshot_inputs"),
            (offset_of!(HostApiV1, pump_history), "pump_history"),
            (offset_of!(HostApiV1, clock), "clock"),
            (offset_of!(HostApiV1, request_shutdown), "request_shutdown"),
            (offset_of!(HostApiV1, environment), "environment"),
            (offset_of!(HostApiV1, poll_event), "poll_event"),
            (
                offset_of!(HostApiV1, publisher_counters),
                "publisher_counters",
            ),
            (offset_of!(HostApiV1, copy_error), "copy_error"),
            (offset_of!(HostApiV1, diagnostic), "diagnostic"),
            (offset_of!(HostApiV1, accumulate_count), "accumulate_count"),
            (offset_of!(HostApiV1, accumulate_frame), "accumulate_frame"),
            (offset_of!(HostApiV1, mark_served), "mark_served"),
            (
                offset_of!(HostApiV1, capture_accumulate),
                "capture_accumulate",
            ),
            (offset_of!(HostApiV1, query_extension), "query_extension"),
            (
                offset_of!(HostApiV1, restore_accumulate),
                "restore_accumulate",
            ),
            (
                offset_of!(HostApiV1, create_raw_publisher),
                "create_raw_publisher",
            ),
            (
                offset_of!(HostApiV1, finish_raw_output),
                "finish_raw_output",
            ),
            (
                offset_of!(HostApiV1, register_raw_egress),
                "register_raw_egress",
            ),
            (
                offset_of!(HostApiV1, destroy_raw_publisher),
                "destroy_raw_publisher",
            ),
        ];
        for (missing, name) in fields {
            let mut raw = vec![0usize; size_of::<HostApiV1>().div_ceil(size_of::<usize>())];
            // SAFETY: aligned initialized scalar storage covers the whole host
            // table. We construct malformed callback bytes, never a typed table:
            // every case contains one null callback and MUST refuse before any
            // Rust function pointer value or reference is formed or invoked.
            let result = unsafe {
                let ptr = raw.as_mut_ptr().cast::<HostApiV1>();
                ptr.cast::<TableHeaderV1>()
                    .write(TableHeaderV1::for_type::<HostApiV1>());
                for (offset, _) in fields {
                    ptr.cast::<u8>().add(offset).cast::<usize>().write(1);
                }
                ptr.cast::<u8>().add(missing).cast::<usize>().write(0);
                read_host_api(ptr)
            };
            assert!(
                matches!(result, Err(AbiError::Callback { table: "HostApiV1", callback }) if callback == name)
            );
        }
    }

    #[test]
    fn extension_callbacks_are_checked_before_extension_references() {
        for state in [true, false] {
            let mut api = node();
            let size = if state {
                size_of::<StateApiV1>()
            } else {
                size_of::<ExternalApiV1>()
            };
            let mut raw = vec![0usize; size.div_ceil(size_of::<usize>())];
            // SAFETY: initialized aligned storage covers the complete extension.
            // Its scalar header is valid, but its first callback is deliberately
            // null. The validator must reject before forming an extension value.
            let result = unsafe {
                let ptr = raw.as_mut_ptr();
                if state {
                    ptr.cast::<TableHeaderV1>()
                        .write(TableHeaderV1::for_type::<StateApiV1>());
                    api.capabilities = capability::STATE;
                    api.state = ptr.cast();
                } else {
                    ptr.cast::<TableHeaderV1>()
                        .write(TableHeaderV1::for_type::<ExternalApiV1>());
                    api.capabilities = capability::EXTERNAL;
                    api.external = ptr.cast();
                }
                read_node_api(&api)
            };
            assert!(matches!(result, Err(AbiError::Callback { .. })));
        }
    }
}
