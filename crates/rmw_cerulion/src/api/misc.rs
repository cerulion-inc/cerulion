// SPDX-License-Identifier: AGPL-3.0-only
//! Remaining rmw surface: gids, serialization, graph introspection,
//! QoS compatibility, logging, and feature flags.

use std::os::raw::c_char;

use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use crate::runtime::{self, PublisherData};

// =====================================================================
// gid
// =====================================================================

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const ffi::rmw_publisher_t,
    gid: *mut ffi::rmw_gid_t,
) -> rmw_ret_t {
    if publisher.is_null() || gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if !ffi::is_our_identifier((*publisher).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    let data = &*((*publisher).data as *const PublisherData);
    let g = &mut *gid;
    g.implementation_identifier = ffi::implementation_identifier_ptr();
    // Size-agnostic across distros (RMW_GID_STORAGE_SIZE is 24 on
    // Foxy/Galactic/Humble, 16 on Iron and later — pinned per era in
    // ffi/era_pins.rs): zero the whole array, then write our 16 bytes —
    // the tail must never be uninitialized caller stack memory
    // (rmw_compare_gids_equal compares the FULL array).
    g.data = Default::default();
    g.data[..16].copy_from_slice(&data.gid);
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_compare_gids_equal(
    gid1: *const ffi::rmw_gid_t,
    gid2: *const ffi::rmw_gid_t,
    result: *mut bool,
) -> rmw_ret_t {
    if gid1.is_null() || gid2.is_null() || result.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *result = (*gid1).data == (*gid2).data;
    RMW_RET_OK
}

// =====================================================================
// Serialization ("cerulion" format = the native wire frame)
// =====================================================================

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_serialize(
    ros_message: *const std::os::raw::c_void,
    type_support: *const ffi::rosidl_message_type_support_t,
    serialized_message: *mut ffi::rmw_serialized_message_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if ros_message.is_null() || serialized_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // KNOWN LIMITATION: this
        // resolve runs per MESSAGE, which is why the C++ era gate inside
        // it is flood-latched. The three `error!`s below —
        // "bridge failed" / "flatten failed" — are the SAME class on the
        // SAME cadence and are NOT latched: pinning a latch needs a
        // hand-built typesupport whose bridge construction fails, which
        // no fixture here provides.
        let Some(handle) = super::resolve_introspection(type_support) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        let built = match handle {
            super::IntrospectionHandle::C(members) => {
                crate::type_bridge::BridgedMessage::new(members).map(crate::bridge::AnyBridge::C)
            }
            super::IntrospectionHandle::Cpp(members) => {
                crate::type_bridge_cpp::CppBridgedMessage::new(members)
                    .map(crate::bridge::AnyBridge::Cpp)
            }
        };
        let bridge = match built {
            Ok(b) => b,
            Err(e) => {
                tracing::error!(error = %e, "serialize: bridge failed");
                return RMW_RET_ERROR;
            }
        };
        let frame = match bridge.flatten(ros_message, 0, 0) {
            Ok(f) => f,
            Err(e) => {
                tracing::error!(error = %e, "serialize: flatten failed");
                return RMW_RET_ERROR;
            }
        };

        let out = &mut *serialized_message;
        if out.buffer_capacity < frame.len() {
            let new_buf = ffi::allocator_alloc(&out.allocator, frame.len()) as *mut u8;
            if new_buf.is_null() {
                return ffi::RMW_RET_BAD_ALLOC;
            }
            if !out.buffer.is_null() {
                if let Some(dealloc) = out.allocator.deallocate {
                    dealloc(out.buffer as *mut std::os::raw::c_void, out.allocator.state);
                }
            }
            out.buffer = new_buf;
            out.buffer_capacity = frame.len();
        }
        std::ptr::copy_nonoverlapping(frame.as_ptr(), out.buffer, frame.len());
        out.buffer_length = frame.len();
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_deserialize(
    serialized_message: *const ffi::rmw_serialized_message_t,
    type_support: *const ffi::rosidl_message_type_support_t,
    ros_message: *mut std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if serialized_message.is_null() || ros_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Some(handle) = super::resolve_introspection(type_support) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        let built = match handle {
            super::IntrospectionHandle::C(members) => {
                crate::type_bridge::BridgedMessage::new(members).map(crate::bridge::AnyBridge::C)
            }
            super::IntrospectionHandle::Cpp(members) => {
                crate::type_bridge_cpp::CppBridgedMessage::new(members)
                    .map(crate::bridge::AnyBridge::Cpp)
            }
        };
        let bridge = match built {
            Ok(b) => b,
            Err(e) => {
                tracing::error!(error = %e, "deserialize: bridge failed");
                return RMW_RET_ERROR;
            }
        };
        let input = &*serialized_message;
        // `from_raw_parts` requires a non-null pointer even for a
        // zero-length slice; a null buffer is valid for an uninitialised
        // `rmw_serialized_message_t`, so guard before constructing the
        // slice (UB otherwise, before the size check below fires).
        //
        // This returns SILENTLY, unlike its twin in
        // `rmw_publish_serialized_message`, which routes the same condition
        // into a latched, counted, loud reject. The asymmetry is deliberate
        // and is a property of the two ENTRY POINTS, not of the condition.
        // This function is a one-shot conversion: it owns no entity, so there
        // is nothing to hang a `FailureRegimeLatch` on and nothing a caller
        // could read a counter off — the same reason the decode arm below
        // says it needs no latch. Its two "these bytes are not a frame"
        // shapes (null buffer, too short for a header) are therefore both
        // silent, which is at least internally consistent; the publish site's
        // two are both loud. Giving this function the same treatment means
        // giving it an entity or a process-global latch, which is a design
        // change, so this function stays silent.
        if input.buffer.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let bytes = std::slice::from_raw_parts(input.buffer, input.buffer_length);
        if bytes.len() < cerulion_core::wire::WireHeader::SIZE {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let payload = &bytes[cerulion_core::wire::WireHeader::SIZE..];
        if bridge.unflatten(payload, ros_message) {
            RMW_RET_OK
        } else {
            // Without this line the arm returns an error code but says
            // NOTHING about why — leaving the caller a bare RMW_RET_ERROR
            // for a frame whose schema hash was fine. No latch here: this
            // is a one-shot call, not a stream, so there is no flood to
            // suppress.
            tracing::error!(
                r#type = %bridge.qualified_name(),
                payload_len = payload.len(),
                "rmw_deserialize failed — the bridge could not decode the payload. \
                 If the buffer came from another rmw_cerulion build, its wire framing may \
                 differ; both ends must come from the same build."
            );
            RMW_RET_ERROR
        }
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_serialized_message_size(
    _type_support: *const ffi::rosidl_message_type_support_t,
    _message_bounds: *const ffi::rosidl_runtime_c__Sequence__bound,
    _size: *mut usize,
) -> rmw_ret_t {
    // Frame size depends on per-message variable content.
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_serialized_message_resize(
    serialized_message: *mut ffi::rmw_serialized_message_t,
    new_size: usize,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if serialized_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let msg = &mut *serialized_message;
        if new_size <= msg.buffer_capacity {
            return RMW_RET_OK;
        }
        let new_buf = ffi::allocator_alloc(&msg.allocator, new_size) as *mut u8;
        if new_buf.is_null() {
            return ffi::RMW_RET_BAD_ALLOC;
        }
        if !msg.buffer.is_null() {
            std::ptr::copy_nonoverlapping(msg.buffer, new_buf, msg.buffer_length);
            if let Some(dealloc) = msg.allocator.deallocate {
                dealloc(msg.buffer as *mut std::os::raw::c_void, msg.allocator.state);
            }
        }
        msg.buffer = new_buf;
        msg.buffer_capacity = new_size;
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_publish_serialized_message(
    publisher: *const ffi::rmw_publisher_t,
    serialized_message: *const ffi::rmw_serialized_message_t,
    _allocation: *mut ffi::rmw_publisher_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if publisher.is_null() || serialized_message.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*publisher).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*publisher).data as *const PublisherData);
        let input = &*serialized_message;
        // An UNINITIALISED `rmw_serialized_message_t` (what
        // `rcutils_get_zero_initialized_uint8_array()` yields) carries a NULL
        // buffer, and `from_raw_parts` requires a non-null pointer even for a
        // zero-length slice — so the empty slice is built by hand here rather
        // than asked for. That is the whole of the UB guard; the CONDITION is
        // deliberately NOT special-cased. An unfilled message is a zero-byte
        // frame, i.e. "too short to hold the wire header" exactly like a
        // truncated one, so it takes the same latched, counted,
        // flood-suppressed reject arm below.
        //
        // Reporting it there rather than returning early is deliberate on two
        // counts. (1) The identical caller mistake reaches this function in TWO
        // shapes — a NULL buffer and a non-null zero-LENGTH one — and only the
        // first is distinguishable here; an early return would give one shape a
        // loud head and a counter and leave the other silent. That half is
        // MEASURED (both shapes are driven in
        // `an_unfilled_serialized_message_is_a_latched_malformed_header_reject`).
        // (2) Without the null handling this call is
        // UB on an unfilled message: a debug build ABORTS on the
        // `from_raw_parts` precondition (measured — deleting the handling
        // SIGABRTs that test), while a release build compiles the check out and
        // the resulting empty slice reaches the malformed-header `error!` below.
        // That second half is what the UB is EXPECTED to do rather than
        // something measured — reasoning about a UB path is not
        // evidence — but it is the only behaviour a release build has been
        // observed to have, so an early SILENT return would at best be
        // replacing a loud condition with nothing.
        //
        // `rmw_deserialize`'s null guard IS silent, and stays that way: it owns
        // no entity to hold a latch or counter on. See the note there.
        let bytes: &[u8] = if input.buffer.is_null() {
            &[]
        } else {
            std::slice::from_raw_parts(input.buffer, input.buffer_length)
        };
        // Validate the frame belongs on this topic BEFORE publishing —
        // a wrong-type serialized frame must fail loudly here, not be
        // silently dropped per-subscriber.
        // A BARE per-call `error!` on either reject arm below would flood: this
        // is the per-MESSAGE publish entry point, so a rosbag-class caller
        // (`ros2 bag play`, any serialized republisher) whose frames are
        // stale rejects EVERY frame at frame rate until it is re-recorded —
        // the disk-fill class, and the same shape the take-side latch
        // suppresses. Loud once per regime (plus a decade ladder),
        // counted always. The two conditions ride SEPARATE latches so
        // neither regime can swallow the other's loud head.
        let Some(header) = cerulion_core::wire::WireHeader::read_from_buf(bytes) else {
            crate::publish_reject_latch::report_malformed_header(
                &data.malformed_header_rejects,
                &data.topic,
                bytes.len(),
            );
            return RMW_RET_INVALID_ARGUMENT;
        };
        // The header PARSED — that is the whole malformed-header condition, so
        // recovery is observed here, before the (independent) hash gate.
        crate::publish_reject_latch::report_header_intact(
            &data.malformed_header_rejects,
            &data.topic,
        );
        let expected_hash = data.bridge.schema_hash();
        if header.schema_hash != expected_hash {
            crate::publish_reject_latch::report_schema_hash_reject(
                &data.schema_hash_rejects,
                &data.topic,
                expected_hash,
                header.schema_hash,
            );
            return RMW_RET_INVALID_ARGUMENT;
        }
        crate::publish_reject_latch::report_schema_hash_accepted(
            &data.schema_hash_rejects,
            &data.topic,
        );
        let now = match crate::runtime::runtime() {
            Ok(rt) => rt.transport.clock().now_ns(),
            Err(_) => 0,
        };
        // Re-stamp sequence/timestamp under the publisher lock —
        // rmw_serialize stamps 0/0, and republishing stale counters
        // would break the wire ordering the publisher maintains.
        let mut frame = bytes.to_vec();
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(mut inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        let mut header = header;
        header.sequence = data
            .sequence
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed) as u32;
        header.timestamp_ns = now;
        header.write_to_buf(&mut frame[..cerulion_core::wire::WireHeader::SIZE]);
        let bytes = &frame[..];
        match inner.publisher.publish_raw(bytes) {
            Ok(_) => {
                // Native history: publish_raw retained this
                // frame natively for late joiners (see rmw_publish). Drain
                // events so a SubscriberConnected delivers it via
                // update_connections.
                inner.publisher.check_subscriber_events();
                if let Err(e) = inner.publisher.notify_sent_sample() {
                    // Best-effort wake (frame is on the wire + in history);
                    // log at debug like the rmw_publish / loaned siblings.
                    tracing::debug!(topic = %data.topic, error = ?e, "notify failed");
                }
                RMW_RET_OK
            }
            Err(e) => {
                tracing::error!(topic = %data.topic, error = %e, "serialized publish failed");
                RMW_RET_ERROR
            }
        }
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_serialized_message(
    subscription: *const ffi::rmw_subscription_t,
    serialized_message: *mut ffi::rmw_serialized_message_t,
    taken: *mut bool,
    allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    rmw_take_serialized_message_with_info(
        subscription,
        serialized_message,
        taken,
        std::ptr::null_mut(),
        allocation,
    )
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_serialized_message_with_info(
    subscription: *const ffi::rmw_subscription_t,
    serialized_message: *mut ffi::rmw_serialized_message_t,
    taken: *mut bool,
    message_info: *mut ffi::rmw_message_info_t,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if subscription.is_null() || serialized_message.is_null() || taken.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*subscription).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        *taken = false;
        let data = &*((*subscription).data as *const crate::runtime::SubscriptionData);
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(inner) = runtime::lock_unpoisoned(&data.inner) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };

        let mut frame: Option<Vec<u8>> = None;
        let mut info_out: Option<(u64, u64)> = None;
        let receive = inner.subscriber.try_receive_one(|msg| {
            // Reassemble the full frame (header + payload) — "cerulion"
            // serialization format IS the wire frame.
            let header = msg.header();
            let mut buf = vec![0u8; cerulion_core::wire::WireHeader::SIZE + msg.payload().len()];
            header.write_to_buf(&mut buf[..cerulion_core::wire::WireHeader::SIZE]);
            buf[cerulion_core::wire::WireHeader::SIZE..].copy_from_slice(msg.payload());
            info_out = Some((header.timestamp_ns, header.sequence as u64));
            frame = Some(buf);
        });
        drop(inner);
        if let Err(e) = receive {
            tracing::error!(topic = %data.topic, error = %e, "serialized take failed");
            return RMW_RET_ERROR;
        }
        let Some(frame) = frame else {
            return RMW_RET_OK;
        };

        let out = &mut *serialized_message;
        if out.buffer_capacity < frame.len() {
            let ret = rmw_serialized_message_resize(serialized_message, frame.len());
            if ret != RMW_RET_OK {
                return ret;
            }
        }
        std::ptr::copy_nonoverlapping(frame.as_ptr(), out.buffer, frame.len());
        out.buffer_length = frame.len();
        *taken = true;
        // The _with_info contract fills message_info just
        // like the deserializing take path does.
        if !message_info.is_null() {
            let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
            if let Some((ts, seq)) = info_out {
                info.source_timestamp = ts as i64;
                info.received_timestamp = match crate::runtime::runtime() {
                    Ok(rt) => rt.transport.clock().now_ns() as i64,
                    Err(_) => 0,
                };
                info.publication_sequence_number = seq;
                info.reception_sequence_number = u64::MAX;
            }
            info.publisher_gid.implementation_identifier = ffi::implementation_identifier_ptr();
            info.from_intra_process = false;
            *message_info = info;
        }
        RMW_RET_OK
    })
}

// Dynamic typesupport takes: not supported.

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_dynamic_message(
    _subscription: *const ffi::rmw_subscription_t,
    _dynamic_message: *mut std::os::raw::c_void,
    taken: *mut bool,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if !taken.is_null() {
        *taken = false;
    }
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_dynamic_message_with_info(
    _subscription: *const ffi::rmw_subscription_t,
    _dynamic_message: *mut std::os::raw::c_void,
    taken: *mut bool,
    _message_info: *mut ffi::rmw_message_info_t,
    _allocation: *mut ffi::rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if !taken.is_null() {
        *taken = false;
    }
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_serialization_support_init(
    _serialization_lib_name: *const c_char,
    _allocator: *mut ffi::rcutils_allocator_t,
    _serialization_support: *mut std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

// =====================================================================
// Graph queries
// =====================================================================

unsafe fn fill_string_array(
    array: *mut ffi::rcutils_string_array_t,
    allocator: *mut ffi::rcutils_allocator_t,
    values: &[String],
) -> rmw_ret_t {
    let arr = &mut *array;
    arr.allocator = *allocator;
    if values.is_empty() {
        arr.size = 0;
        arr.data = std::ptr::null_mut();
        return RMW_RET_OK;
    }
    let data = ffi::allocator_alloc(allocator, values.len() * std::mem::size_of::<*mut c_char>())
        as *mut *mut c_char;
    if data.is_null() {
        // size stays 0 / data stays untouched-null — a consumer reading
        // size first must never walk a null data array.
        arr.size = 0;
        arr.data = std::ptr::null_mut();
        return ffi::RMW_RET_BAD_ALLOC;
    }
    for (i, v) in values.iter().enumerate() {
        let dup = ffi::allocator_strdup(allocator, v) as *mut c_char;
        if dup.is_null() {
            // Roll back: free what we already duplicated and the array
            // itself so the caller never sees a half-filled array with
            // null entries it would deref.
            if let Some(dealloc) = (*allocator).deallocate {
                for j in 0..i {
                    dealloc(
                        *data.add(j) as *mut std::os::raw::c_void,
                        (*allocator).state,
                    );
                }
                dealloc(data as *mut std::os::raw::c_void, (*allocator).state);
            }
            arr.size = 0;
            arr.data = std::ptr::null_mut();
            return ffi::RMW_RET_BAD_ALLOC;
        }
        *data.add(i) = dup;
    }
    arr.data = data;
    arr.size = values.len();
    RMW_RET_OK
}

/// Free a fill_string_array-built array (strings + the data buffer)
/// and zero it — rollback helper for error paths where the caller will
/// NOT run rcutils fini on a partially-built out-param.
unsafe fn free_string_array(
    array: *mut ffi::rcutils_string_array_t,
    allocator: *mut ffi::rcutils_allocator_t,
) {
    let arr = &mut *array;
    if let Some(dealloc) = (*allocator).deallocate {
        if !arr.data.is_null() {
            for k in 0..arr.size {
                dealloc(
                    *arr.data.add(k) as *mut std::os::raw::c_void,
                    (*allocator).state,
                );
            }
            dealloc(arr.data as *mut std::os::raw::c_void, (*allocator).state);
        }
    }
    arr.data = std::ptr::null_mut();
    arr.size = 0;
}

unsafe fn fill_names_and_types(
    out: *mut ffi::rmw_names_and_types_t,
    allocator: *mut ffi::rcutils_allocator_t,
    entries: &[(String, String)],
) -> rmw_ret_t {
    let nat = &mut *out;
    let names: Vec<String> = entries.iter().map(|(n, _)| n.clone()).collect();
    let ret = fill_string_array(&mut nat.names, allocator, &names);
    if ret != RMW_RET_OK {
        return ret;
    }
    if entries.is_empty() {
        nat.types = std::ptr::null_mut();
        return RMW_RET_OK;
    }
    let types = ffi::allocator_alloc(
        allocator,
        entries.len() * std::mem::size_of::<ffi::rcutils_string_array_t>(),
    ) as *mut ffi::rcutils_string_array_t;
    if types.is_null() {
        // Roll back the names array too — on error rcl does NOT fini a
        // half-filled rmw_names_and_types_t, so it would leak.
        free_string_array(&mut nat.names, allocator);
        return ffi::RMW_RET_BAD_ALLOC;
    }
    for (i, (_, type_name)) in entries.iter().enumerate() {
        let slot = types.add(i);
        std::ptr::write_bytes(slot, 0, 1);
        let ret = fill_string_array(slot, allocator, std::slice::from_ref(type_name));
        if ret != RMW_RET_OK {
            // Roll back filled slots + the buffer + the names array —
            // nat.types was never assigned, so the caller's fini can't
            // reach them, so they would leak.
            if let Some(dealloc) = (*allocator).deallocate {
                for j in 0..i {
                    let prior = types.add(j);
                    if !(*prior).data.is_null() {
                        for k in 0..(*prior).size {
                            dealloc(
                                *(*prior).data.add(k) as *mut std::os::raw::c_void,
                                (*allocator).state,
                            );
                        }
                        dealloc(
                            (*prior).data as *mut std::os::raw::c_void,
                            (*allocator).state,
                        );
                    }
                }
                dealloc(types as *mut std::os::raw::c_void, (*allocator).state);
            }
            free_string_array(&mut nat.names, allocator);
            return ret;
        }
    }
    nat.types = types;
    RMW_RET_OK
}

/// Direction-filtered names-and-types over ONE (type,count) registry map
/// — the services / clients by-node queries (they must not
/// conflate servers with clients). Publishers/subscriptions use the
/// endpoint-map variant below. Node-name filtering is a
/// documented limitation: the local registry is process-wide, and every
/// local node shares the process's entities — the filter is a no-op
/// without cross-process discovery.
unsafe fn fill_from_one_map(
    map: &std::collections::BTreeMap<String, (String, usize)>,
    allocator: *mut ffi::rcutils_allocator_t,
    out: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    let list: Vec<(String, String)> = map
        .iter()
        .map(|(name, (type_name, _))| (name.clone(), type_name.clone()))
        .collect();
    fill_names_and_types(out, allocator, &list)
}

/// Names-and-types over a publisher/subscription ENDPOINT map (note that
/// these maps store per-endpoint records, not a (type,count) tuple). The
/// type name is any record's `type_name` — all endpoints on a topic share
/// it, and a present topic always has >= 1 record.
unsafe fn fill_from_endpoint_map(
    map: &std::collections::BTreeMap<String, Vec<runtime::EndpointRecord>>,
    allocator: *mut ffi::rcutils_allocator_t,
    out: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    let list: Vec<(String, String)> = map
        .iter()
        .filter_map(|(topic, records)| {
            records
                .first()
                .map(|r| (topic.clone(), r.type_name.clone()))
        })
        .collect();
    fill_names_and_types(out, allocator, &list)
}

// Registry keys ARE the fully-qualified ROS names (the mapping is the
// identity — `runtime::ros_topic_to_cerulion`), so every names-and-types
// query serves them VERBATIM: a name round-trips create → query unchanged.

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_topic_names_and_types(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    _no_demangle: bool,
    topic_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || topic_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        let mut entries: std::collections::BTreeMap<String, String> =
            std::collections::BTreeMap::new();
        // Endpoint maps: the type name is any record's `type_name` (all
        // endpoints on a topic share it); a present topic always has >= 1
        // record (the entry is dropped when the last leaves).
        for (topic, records) in graph.publishers.iter().chain(graph.subscriptions.iter()) {
            if let Some(first) = records.first() {
                entries.insert(topic.clone(), first.type_name.clone());
            }
        }
        let list: Vec<(String, String)> = entries.into_iter().collect();
        fill_names_and_types(topic_names_and_types, allocator, &list)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_service_names_and_types(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    service_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || service_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        let mut entries: std::collections::BTreeMap<String, String> =
            std::collections::BTreeMap::new();
        for (svc, (type_name, _)) in graph.services.iter().chain(graph.clients.iter()) {
            entries.insert(svc.clone(), type_name.clone());
        }
        let list: Vec<(String, String)> = entries.into_iter().collect();
        fill_names_and_types(service_names_and_types, allocator, &list)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_node_names(
    node: *const ffi::rmw_node_t,
    node_names: *mut ffi::rcutils_string_array_t,
    node_namespaces: *mut ffi::rcutils_string_array_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || node_names.is_null() || node_namespaces.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        // Keys are (namespace, name) pairs: same-named
        // nodes in different namespaces are distinct.
        let names: Vec<String> = graph.nodes.keys().map(|(_, n)| n.clone()).collect();
        let namespaces: Vec<String> = graph.nodes.keys().map(|(ns, _)| ns.clone()).collect();
        // Use the default allocator embedded in the arrays (rcl pre-inits
        // them with one).
        let alloc_names = (*node_names).allocator;
        let alloc_ns = (*node_namespaces).allocator;
        let mut a1 = alloc_names;
        let mut a2 = alloc_ns;
        let ret = fill_string_array(node_names, &mut a1, &names);
        if ret != RMW_RET_OK {
            return ret;
        }
        let ret = fill_string_array(node_namespaces, &mut a2, &namespaces);
        if ret != RMW_RET_OK {
            // Roll back the already-filled names array — on error rcl
            // does not fini partially-built out-params, so it would
            // leak.
            free_string_array(node_names, &mut a1);
        }
        ret
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_node_names_with_enclaves(
    node: *const ffi::rmw_node_t,
    node_names: *mut ffi::rcutils_string_array_t,
    node_namespaces: *mut ffi::rcutils_string_array_t,
    enclaves: *mut ffi::rcutils_string_array_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || node_names.is_null() || node_namespaces.is_null() || enclaves.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        // ONE snapshot under ONE lock hold for all three arrays — a
        // second acquisition could observe a different node set and
        // return mismatched-length arrays (a TOCTOU).
        let (names, namespaces, values) = {
            let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
            let names: Vec<String> = graph.nodes.keys().map(|(_, n)| n.clone()).collect();
            let namespaces: Vec<String> = graph.nodes.keys().map(|(ns, _)| ns.clone()).collect();
            let values: Vec<String> = graph.nodes.keys().map(|_| "/".to_string()).collect();
            (names, namespaces, values)
        };
        let mut a1 = (*node_names).allocator;
        let ret = fill_string_array(node_names, &mut a1, &names);
        if ret != RMW_RET_OK {
            return ret;
        }
        let mut a2 = (*node_namespaces).allocator;
        let ret = fill_string_array(node_namespaces, &mut a2, &namespaces);
        if ret != RMW_RET_OK {
            let mut a1 = (*node_names).allocator;
            free_string_array(node_names, &mut a1);
            return ret;
        }
        let mut a3 = (*enclaves).allocator;
        let ret = fill_string_array(enclaves, &mut a3, &values);
        if ret != RMW_RET_OK {
            let mut a1 = (*node_names).allocator;
            free_string_array(node_names, &mut a1);
            let mut a2 = (*node_namespaces).allocator;
            free_string_array(node_namespaces, &mut a2);
        }
        ret
    })
}

unsafe fn count_in(
    map_fn: impl Fn(&runtime::GraphRegistry) -> usize,
    count: *mut usize,
) -> rmw_ret_t {
    if count.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let Ok(rt) = runtime::runtime() else {
        return RMW_RET_ERROR;
    };
    let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
    *count = map_fn(&graph);
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_count_publishers(
    node: *const ffi::rmw_node_t,
    topic_name: *const c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let Some(topic) = ffi::cstr(topic_name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    // A relative / empty name (unreachable through rcl, which validates
    // fully-qualified names first) is REFUSED, never prefixed.
    let Ok(key) = runtime::ros_topic_to_cerulion(topic) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    count_in(
        // The endpoint Vec IS the count — no parallel counter.
        move |g| g.publishers.get(&key).map(|v| v.len()).unwrap_or(0),
        count,
    )
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_count_subscribers(
    node: *const ffi::rmw_node_t,
    topic_name: *const c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let Some(topic) = ffi::cstr(topic_name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    // A relative / empty name (unreachable through rcl, which validates
    // fully-qualified names first) is REFUSED, never prefixed.
    let Ok(key) = runtime::ros_topic_to_cerulion(topic) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    count_in(
        move |g| g.subscriptions.get(&key).map(|v| v.len()).unwrap_or(0),
        count,
    )
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_count_services(
    node: *const ffi::rmw_node_t,
    service_name: *const c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let Some(name) = ffi::cstr(service_name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    // Refused, never prefixed — see `rmw_count_publishers`.
    let Ok(key) = runtime::ros_topic_to_cerulion(name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    count_in(
        move |g| g.services.get(&key).map(|(_, n)| *n).unwrap_or(0),
        count,
    )
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_count_clients(
    node: *const ffi::rmw_node_t,
    service_name: *const c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let Some(name) = ffi::cstr(service_name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    // Refused, never prefixed — see `rmw_count_publishers`.
    let Ok(key) = runtime::ros_topic_to_cerulion(name) else {
        return RMW_RET_INVALID_ARGUMENT;
    };
    count_in(
        move |g| g.clients.get(&key).map(|(_, n)| *n).unwrap_or(0),
        count,
    )
}

// Per-node and per-topic endpoint queries: local-process registry view
// (cross-process discovery is not implemented; it would ride Zenoh
// liveliness).

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_publisher_names_and_types_by_node(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    _node_name: *const c_char,
    _node_namespace: *const c_char,
    _no_demangle: bool,
    topic_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || topic_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        fill_from_endpoint_map(&graph.publishers, allocator, topic_names_and_types)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_subscriber_names_and_types_by_node(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    _node_name: *const c_char,
    _node_namespace: *const c_char,
    _no_demangle: bool,
    topic_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || topic_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        fill_from_endpoint_map(&graph.subscriptions, allocator, topic_names_and_types)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_service_names_and_types_by_node(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    _node_name: *const c_char,
    _node_namespace: *const c_char,
    service_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || service_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        // Servers only — not the services+clients union.
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        fill_from_one_map(&graph.services, allocator, service_names_and_types)
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_client_names_and_types_by_node(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    _node_name: *const c_char,
    _node_namespace: *const c_char,
    service_names_and_types: *mut ffi::rmw_names_and_types_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || service_names_and_types.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        // Clients only.
        let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
        fill_from_one_map(&graph.clients, allocator, service_names_and_types)
    })
}

/// QoS profile for an endpoint-info record.
///
/// **`depth` is the depth Cerulion actually PROVISIONED — its RETENTION**,
/// not an echo of what the caller asked for:
/// a `TRANSIENT_LOCAL` depth-1000 publisher reports
/// the 16 frames it really keeps, a depth-5 one reports 5, a VOLATILE one
/// reports 0, and a subscription reports the transport-default queue depth
/// it really got. A create whose depth was really CLAMPED warns LOUDLY at
/// create time naming both numbers, so the clamp is visible rather than
/// concealed by a surface that repeats the request. A VOLATILE publisher's
/// 0 is not a clamp — it is what VOLATILE means — so it reports 0 and
/// stays quiet.
///
/// The other three axes still carry the REQUESTED values, and the rest
/// is defaulted like `rmw_publisher_get_actual_qos` — those (and that
/// getter, which answers a constant that never touches the endpoint)
/// are not derived from the endpoint.
fn endpoint_qos_profile(qos: &runtime::QosSnapshot) -> ffi::rmw_qos_profile_t {
    let mut profile = super::pubsub::default_actual_qos();
    profile.reliability = qos.reliability;
    profile.durability = qos.durability;
    profile.history = qos.history;
    profile.depth = qos.provisioned_depth;
    profile
}

/// Free the per-slot strings of the first `filled` endpoint-info slots
/// and then the array buffer — the rollback helper for a strdup OOM
/// mid-fill (rcl does NOT fini a half-built out-param).
unsafe fn free_endpoint_info_array(
    info_array: *mut ffi::rmw_topic_endpoint_info_t,
    filled: usize,
    allocator: *mut ffi::rcutils_allocator_t,
) {
    if let Some(dealloc) = (*allocator).deallocate {
        for j in 0..filled {
            let slot = info_array.add(j);
            for p in [
                (*slot).node_name,
                (*slot).node_namespace,
                (*slot).topic_type,
            ] {
                if !p.is_null() {
                    dealloc(p as *mut std::os::raw::c_void, (*allocator).state);
                }
            }
        }
        dealloc(info_array as *mut std::os::raw::c_void, (*allocator).state);
    }
}

/// Fill `out` from `records` using the CALLER's rcutils allocator (rcl
/// frees with the same one), mirroring upstream rmw impls: zero the
/// array, allocate the info array, duplicate every string through the
/// allocator. Empty `records` → size 0 / null array (a valid empty
/// result). Rolls back fully on an allocation failure so the caller
/// never sees a half-built array.
unsafe fn fill_endpoint_info_array(
    out: *mut ffi::rmw_topic_endpoint_info_array_t,
    allocator: *mut ffi::rcutils_allocator_t,
    records: &[runtime::EndpointRecord],
    endpoint_type: ffi::rmw_endpoint_type_t,
) -> rmw_ret_t {
    let arr = &mut *out;
    arr.size = 0;
    arr.info_array = std::ptr::null_mut();
    if records.is_empty() {
        return RMW_RET_OK;
    }
    let info_array = ffi::allocator_alloc(
        allocator,
        records.len() * std::mem::size_of::<ffi::rmw_topic_endpoint_info_t>(),
    ) as *mut ffi::rmw_topic_endpoint_info_t;
    if info_array.is_null() {
        return ffi::RMW_RET_BAD_ALLOC;
    }
    for (i, rec) in records.iter().enumerate() {
        let slot = info_array.add(i);
        // Zero the whole slot first: leaves topic_type_hash and the qos
        // tail well-defined and the string pointers null until assigned.
        std::ptr::write_bytes(slot, 0, 1);
        let node_name = ffi::allocator_strdup(allocator, &rec.node_name);
        let node_namespace = ffi::allocator_strdup(allocator, &rec.node_namespace);
        let topic_type = ffi::allocator_strdup(allocator, &rec.type_name);
        if node_name.is_null() || node_namespace.is_null() || topic_type.is_null() {
            // Free any of THIS slot's strings that succeeded (the slot is
            // not yet in [0, i)), then slots [0, i) + the array buffer.
            if let Some(dealloc) = (*allocator).deallocate {
                for p in [node_name, node_namespace, topic_type] {
                    if !p.is_null() {
                        dealloc(p as *mut std::os::raw::c_void, (*allocator).state);
                    }
                }
            }
            free_endpoint_info_array(info_array, i, allocator);
            return ffi::RMW_RET_BAD_ALLOC;
        }
        (*slot).node_name = node_name;
        (*slot).node_namespace = node_namespace;
        (*slot).topic_type = topic_type;
        (*slot).endpoint_type = endpoint_type;
        (*slot).endpoint_gid = rec.endpoint_gid;
        (*slot).qos_profile = endpoint_qos_profile(&rec.qos);
    }
    arr.info_array = info_array;
    arr.size = records.len();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_publishers_info_by_topic(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    topic_name: *const c_char,
    _no_mangle: bool,
    publishers_info: *mut ffi::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || publishers_info.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Some(ros_topic) = ffi::cstr(topic_name) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        // Process-LOCAL endpoints only — cross-process endpoint discovery
        // is not implemented (mirrors the nodes-map
        // limitation). An unknown topic yields an empty array + OK.
        // Refused, never prefixed — see `rmw_count_publishers`.
        let Ok(key) = runtime::ros_topic_to_cerulion(ros_topic) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        // Clone the records out from under the read lock so the allocator
        // call-throughs never run while holding it.
        let records = {
            let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
            graph.publishers.get(&key).cloned().unwrap_or_default()
        };
        fill_endpoint_info_array(
            publishers_info,
            allocator,
            &records,
            ffi::RMW_ENDPOINT_PUBLISHER,
        )
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_subscriptions_info_by_topic(
    node: *const ffi::rmw_node_t,
    allocator: *mut ffi::rcutils_allocator_t,
    topic_name: *const c_char,
    _no_mangle: bool,
    subscriptions_info: *mut ffi::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || allocator.is_null() || subscriptions_info.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let Some(ros_topic) = ffi::cstr(topic_name) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        let Ok(rt) = runtime::runtime() else {
            return RMW_RET_ERROR;
        };
        // Refused, never prefixed — see `rmw_count_publishers`.
        let Ok(key) = runtime::ros_topic_to_cerulion(ros_topic) else {
            return RMW_RET_INVALID_ARGUMENT;
        };
        let records = {
            let graph = rt.graph.read().unwrap_or_else(|e| e.into_inner());
            graph.subscriptions.get(&key).cloned().unwrap_or_default()
        };
        fill_endpoint_info_array(
            subscriptions_info,
            allocator,
            &records,
            ffi::RMW_ENDPOINT_SUBSCRIPTION,
        )
    })
}

// =====================================================================
// QoS compatibility / misc
// =====================================================================

/// SHM pub/sub always matches (no QoS negotiation failure modes).
///
/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_qos_profile_check_compatible(
    _publisher_profile: ffi::rmw_qos_profile_t,
    _subscription_profile: ffi::rmw_qos_profile_t,
    compatibility: *mut ffi::rmw_qos_compatibility_type_t,
    reason: *mut c_char,
    reason_size: usize,
) -> rmw_ret_t {
    if compatibility.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *compatibility = ffi::RMW_QOS_COMPATIBILITY_OK;
    if !reason.is_null() && reason_size > 0 {
        *reason = 0;
    }
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_set_log_severity(_severity: ffi::rmw_log_severity_t) -> rmw_ret_t {
    // Logging rides Cerulion's `tracing` (RUST_LOG); the rmw severity
    // call is accepted as a no-op.
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_feature_supported(feature: ffi::rmw_feature_t) -> bool {
    // MESSAGE_INFO timestamps are filled by take; everything else
    // (dynamic typesupport variants) is not supported.
    feature == ffi::RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_get_network_flow_endpoints(
    _publisher: *const ffi::rmw_publisher_t,
    _allocator: *mut ffi::rcutils_allocator_t,
    _network_flow_endpoint_array: *mut ffi::rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_get_network_flow_endpoints(
    _subscription: *const ffi::rmw_subscription_t,
    _allocator: *mut ffi::rcutils_allocator_t,
    _network_flow_endpoint_array: *mut ffi::rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}
