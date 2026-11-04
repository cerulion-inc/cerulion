// SPDX-License-Identifier: AGPL-3.0-only
//! rmw services / clients over the deterministic
//! request-response layer. Actions need nothing extra — rcl composes
//! them from services + topics.

use std::os::raw::{c_char, c_void};

use cerulion_core::transport::service::{derive_client_guid, ServiceEnvelope};
use cerulion_core::wire::MaxSliceLen;

use crate::bridge::AnyBridge;
use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use crate::replay::log::{EntityKind, GraphQuery, OutputKind, TakeKind, TakeRec};
use crate::replay::{TakeAction, TakeMeta};
use crate::runtime::{self, ClientData, GraphRegistry, ServiceData};
use crate::type_bridge::BridgedMessage;
use crate::type_bridge_cpp::CppBridgedMessage;

const SERVICE_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(4 * 1024 * 1024);

struct ServiceTypeInfo {
    service_name_cerulion: String,
    type_name: String,
    request_bridge: std::sync::Arc<AnyBridge>,
    response_bridge: std::sync::Arc<AnyBridge>,
    request_hash: u64,
    response_hash: u64,
}

unsafe fn service_type_info(
    type_support: *const ffi::rosidl_service_type_support_t,
    ros_service_name: &str,
) -> Result<ServiceTypeInfo, rmw_ret_t> {
    // The ROS service name IS the Cerulion service name (identity — see
    // `runtime::ros_topic_to_cerulion`, shared with topics); the transport's
    // service layer derives its request/reply topics from it. A relative
    // name is refused loudly, never prefixed — and before any bridge work.
    let service_name_cerulion = match runtime::ros_topic_to_cerulion(ros_service_name) {
        Ok(name) => name,
        Err(error) => {
            tracing::error!(
                service = %ros_service_name,
                error = %error,
                "service entity creation refused: the ROS name is not a fully-qualified service name"
            );
            return Err(RMW_RET_INVALID_ARGUMENT);
        }
    };
    let handle =
        super::resolve_service_introspection(type_support).ok_or(RMW_RET_INVALID_ARGUMENT)?;
    let (request_bridge, response_bridge) = match handle {
        super::ServiceIntrospectionHandle::C(members) => {
            let m = &*members;
            let req = BridgedMessage::new(m.request_members_).map(AnyBridge::C);
            let resp = BridgedMessage::new(m.response_members_).map(AnyBridge::C);
            (req, resp)
        }
        super::ServiceIntrospectionHandle::Cpp(members) => {
            let m = &*members;
            let req = CppBridgedMessage::new(m.request_members_).map(AnyBridge::Cpp);
            let resp = CppBridgedMessage::new(m.response_members_).map(AnyBridge::Cpp);
            (req, resp)
        }
    };
    let request_bridge = request_bridge.map_err(|e| {
        tracing::error!(error = %e, "request bridge failed");
        RMW_RET_ERROR
    })?;
    let response_bridge = response_bridge.map_err(|e| {
        tracing::error!(error = %e, "response bridge failed");
        RMW_RET_ERROR
    })?;

    // "pkg/SetBool_Request" → service type "pkg/srv/SetBool".
    let type_name = request_bridge
        .qualified_name()
        .strip_suffix("_Request")
        .map(|q| match q.split_once('/') {
            Some((pkg, name)) => format!("{pkg}/srv/{name}"),
            None => q.to_string(),
        })
        .unwrap_or_else(|| request_bridge.qualified_name().to_string());

    Ok(ServiceTypeInfo {
        service_name_cerulion,
        type_name,
        request_hash: request_bridge.schema_hash(),
        response_hash: response_bridge.schema_hash(),
        request_bridge: std::sync::Arc::new(request_bridge),
        response_bridge: std::sync::Arc::new(response_bridge),
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_service(
    node: *const ffi::rmw_node_t,
    type_support: *const ffi::rosidl_service_type_support_t,
    service_name: *const c_char,
    _qos: *const ffi::rmw_qos_profile_t,
) -> *mut ffi::rmw_service_t {
    ffi::ffi_guard(std::ptr::null_mut(), || unsafe {
        if node.is_null() || !ffi::is_our_identifier((*node).implementation_identifier) {
            return std::ptr::null_mut();
        }
        let Some(name) = ffi::cstr(service_name) else {
            return std::ptr::null_mut();
        };
        let Ok(rt) = runtime::runtime() else {
            return std::ptr::null_mut();
        };
        let Ok(info) = service_type_info(type_support, name) else {
            return std::ptr::null_mut();
        };
        let server = match rt.transport.create_service_server(
            &info.service_name_cerulion,
            SERVICE_SLICE_LEN,
            info.request_hash,
            info.response_hash,
        ) {
            Ok(s) => s,
            Err(e) => {
                tracing::error!(service = %name, error = %e, "service server creation failed");
                return std::ptr::null_mut();
            }
        };

        let data = Box::new(ServiceData {
            service_name: info.service_name_cerulion,
            type_name: info.type_name,
            request_bridge: info.request_bridge,
            response_bridge: info.response_bridge,
            server: std::sync::Mutex::new(server),
            decode_failures: std::sync::Mutex::new(
                crate::decode_failure_latch::DecodeFailureLatch::new(),
            ),
        });
        let svc = Box::new(ffi::rmw_service_t {
            implementation_identifier: ffi::implementation_identifier_ptr(),
            data: Box::into_raw(data) as *mut c_void,
            service_name: super::leak_ros_name(name),
        });
        let ptr = Box::into_raw(svc);
        // Registry mutation LAST (see rmw_create_publisher).
        {
            let data_ref = &*((*ptr).data as *const ServiceData);
            let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
            GraphRegistry::add(
                &mut graph.services,
                &data_ref.service_name,
                &data_ref.type_name,
            );
            drop(graph);
            // Delivery log / replay: the stable name (`srv:<name>#n`).
            crate::replay::on_entity(
                EntityKind::Service,
                (*ptr).data as usize,
                &data_ref.service_name,
                &(*((*node).data as *const runtime::NodeData)).name,
            );
        }
        runtime::notify_graph_change();
        ptr
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_service(
    node: *mut ffi::rmw_node_t,
    service: *mut ffi::rmw_service_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || service.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*service).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        crate::replay::on_entity_destroyed((*service).data as usize);
        // Borrow-first, free-last.
        {
            let data = &*((*service).data as *const ServiceData);
            if let Ok(rt) = runtime::runtime() {
                let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
                GraphRegistry::remove(&mut graph.services, &data.service_name);
            }
            runtime::notify_graph_change();
        }
        super::unleak_ros_name((*service).service_name);
        drop(Box::from_raw((*service).data as *mut ServiceData));
        drop(Box::from_raw(service));
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_request(
    service: *const ffi::rmw_service_t,
    request_header: *mut ffi::rmw_service_info_t,
    ros_request: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    // Recording and replay off: one `OnceLock` load, then the live take.
    if !crate::replay::active() {
        return take_request_live(service, request_header, ros_request, taken);
    }
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if service.is_null() || request_header.is_null() || ros_request.is_null() || taken.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*service).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let key = (*service).data as usize;
        match crate::replay::take_action(key, TakeKind::Request) {
            TakeAction::Live => take_request_live(service, request_header, ros_request, taken),
            TakeAction::Record(r) => r.take(
                key,
                TakeKind::Request,
                || take_request_live(service, request_header, ros_request, taken),
                |_| service_take_meta(*taken, request_header),
            ),
            TakeAction::Serve(rec) => {
                let data = &*((*service).data as *const ServiceData);
                serve_recorded(
                    &data.request_bridge,
                    &data.service_name,
                    rec,
                    request_header,
                    ros_request,
                    taken,
                )
            }
            TakeAction::Refuse => RMW_RET_ERROR,
        }
    })
}

/// What a live request or response take reported, for the delivery log.
unsafe fn service_take_meta(taken: bool, header: *const ffi::rmw_service_info_t) -> TakeMeta {
    if !taken {
        return TakeMeta {
            taken,
            received_ts: 0,
            request: None,
        };
    }
    let h = &*header;
    // int8_t[16] before Iron, uint8_t[16] from Iron on (a no-op cast there).
    #[allow(clippy::unnecessary_cast)]
    let guid: [u8; 16] = h.request_id.writer_guid.map(|b| b as u8);
    TakeMeta {
        taken,
        received_ts: h.received_timestamp,
        request: Some((h.request_id.sequence_number, guid, h.source_timestamp)),
    }
}

/// Replay: deliver a recorded request or response into the caller's message,
/// decoded by the entity's own bridge, with the request id the live take
/// reported.
unsafe fn serve_recorded(
    bridge: &AnyBridge,
    service_name: &str,
    rec: Option<TakeRec>,
    header: *mut ffi::rmw_service_info_t,
    ros_message: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    *taken = false;
    let Some(rec) = rec else {
        return RMW_RET_OK;
    };
    if rec.taken {
        if !bridge.unflatten(&rec.body, ros_message) {
            crate::replay::served_decode_failed(service_name);
            return RMW_RET_ERROR;
        }
        *taken = true;
        let hdr = &mut *header;
        hdr.request_id.sequence_number = rec.seq;
        hdr.request_id.writer_guid = rec.guid.map(|b| b as _);
        hdr.source_timestamp = rec.source_ts;
        hdr.received_timestamp = rec.received_ts;
    }
    rec.ret
}

/// The live `rmw_take_request`.
///
/// # Safety
/// Same contract as [`rmw_take_request`].
unsafe fn take_request_live(
    service: *const ffi::rmw_service_t,
    request_header: *mut ffi::rmw_service_info_t,
    ros_request: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if service.is_null() || request_header.is_null() || ros_request.is_null() || taken.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*service).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        *taken = false;
        let data = &*((*service).data as *const ServiceData);
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(mut server) = runtime::lock_unpoisoned(&data.server) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };

        let mut got: Option<ServiceEnvelope> = None;
        let bridge = &data.request_bridge;
        // ONE request per call (rmw take semantics): try_take_one_request
        // takes at most one valid request and leaves the rest queued —
        // rcl is handed exactly one message per take, so nothing here may
        // drain more than it can hand back (Principle #6). It is also the
        // only request take path cerulion_core offers, by design.
        let result = server.try_take_one_request(|envelope, payload| {
            // A member this build's C++ typesupport gives no way to WRITE (a
            // `bool[]` before Humble, at the top level or inside a nested
            // message) makes every request of this type undeliverable.
            // Decided at registration, so this is one `Option` read per
            // request, and taken BEFORE `unflatten` so the caller's message
            // is untouched: a server that decoded half a request and then
            // failed would answer with fields from the previous call. The
            // request is consumed and dropped through the same latch and
            // regime as every other decode refusal, with the member named.
            if let Some((var_idx, path)) = bridge.unwritable_bool_seq() {
                crate::decode_failure_latch::report_decode_entry_refused(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceRequest,
                    &data.service_name,
                    &data.type_name,
                    payload.len(),
                    var_idx,
                    &crate::type_bridge_cpp::bool_seq_no_assign_detail(path),
                );
                return;
            }
            if bridge.unflatten(payload, ros_request) {
                crate::replay::capture_take_body(payload, envelope.sequence, 0);
                got = Some(envelope);
                crate::decode_failure_latch::report_decode_success(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceRequest,
                    &data.service_name,
                );
            } else {
                // Without this report a request the bridge could not
                // decode would be dropped with no log, and the client's call
                // would simply never get answered.
                crate::decode_failure_latch::report_decode_failure(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceRequest,
                    &data.service_name,
                    &data.type_name,
                    payload.len(),
                );
            }
        });
        drop(server);

        match result {
            Ok(_) => {}
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "take_request failed");
                return RMW_RET_ERROR;
            }
        }
        if let Some(envelope) = got {
            *taken = true;
            let hdr = &mut *request_header;
            hdr.request_id.sequence_number = envelope.sequence;
            // rmw_request_id_t.writer_guid is int8_t[16] before Iron and uint8_t[16]
            // from Iron on; the element cast follows the bindings' type.
            hdr.request_id.writer_guid = envelope.client_guid.map(|b| b as _);
            hdr.source_timestamp = 0;
            hdr.received_timestamp = match runtime::runtime() {
                Ok(rt) => rt.transport.clock().now_ns() as i64,
                Err(_) => 0,
            };
        }
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_send_response(
    service: *const ffi::rmw_service_t,
    request_header: *mut ffi::rmw_request_id_t,
    ros_response: *mut c_void,
) -> rmw_ret_t {
    // Recording and replay off: one `OnceLock` load, then the live send.
    if !crate::replay::active() || service.is_null() {
        return send_response_live(service, request_header, ros_response);
    }
    crate::replay::output((*service).data as usize, OutputKind::Response, || {
        send_response_live(service, request_header, ros_response)
    })
}

/// The live `rmw_send_response`.
///
/// # Safety
/// Same contract as [`rmw_send_response`].
unsafe fn send_response_live(
    service: *const ffi::rmw_service_t,
    request_header: *mut ffi::rmw_request_id_t,
    ros_response: *mut c_void,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if service.is_null() || request_header.is_null() || ros_response.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*service).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*service).data as *const ServiceData);
        // int8_t[16] before Iron, uint8_t[16] from Iron on (a no-op cast there).
        #[allow(clippy::unnecessary_cast)]
        let client_guid: [u8; 16] = (*request_header).writer_guid.map(|b| b as u8);
        let envelope = ServiceEnvelope {
            client_guid,
            sequence: (*request_header).sequence_number,
        };
        // Flatten the response through the bridge into the service payload.
        let _downgrades = crate::rosidl_buffer::PublishScope::open(&data.service_name);
        let payload = match data.response_bridge.flatten_payload_only(ros_response) {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "response flatten failed");
                return RMW_RET_ERROR;
            }
        };
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(mut server) = runtime::lock_unpoisoned(&data.server) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        crate::replay::capture_output_body(&payload);
        match server.send_response(&envelope, &payload) {
            Ok(()) => RMW_RET_OK,
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "send_response failed");
                RMW_RET_ERROR
            }
        }
    })
}

/// # Safety
/// Caller frees with `rmw_service_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_service_allocate() -> *mut ffi::rmw_service_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_service_t>()))
}

/// # Safety
/// `service` must come from `rmw_service_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_service_free(service: *mut ffi::rmw_service_t) {
    if !service.is_null() {
        drop(Box::from_raw(service));
    }
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_service_request_subscription_get_actual_qos(
    service: *const ffi::rmw_service_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if service.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = super::default_actual_qos();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_service_response_publisher_get_actual_qos(
    service: *const ffi::rmw_service_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if service.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = super::default_actual_qos();
    RMW_RET_OK
}

// =====================================================================
// Client
// =====================================================================

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_client(
    node: *const ffi::rmw_node_t,
    type_support: *const ffi::rosidl_service_type_support_t,
    service_name: *const c_char,
    _qos: *const ffi::rmw_qos_profile_t,
) -> *mut ffi::rmw_client_t {
    ffi::ffi_guard(std::ptr::null_mut(), || unsafe {
        if node.is_null() || !ffi::is_our_identifier((*node).implementation_identifier) {
            return std::ptr::null_mut();
        }
        let Some(name) = ffi::cstr(service_name) else {
            return std::ptr::null_mut();
        };
        let Ok(rt) = runtime::runtime() else {
            return std::ptr::null_mut();
        };
        let Ok(info) = service_type_info(type_support, name) else {
            return std::ptr::null_mut();
        };

        let node_data = &*((*node).data as *const crate::runtime::NodeData);
        // Deterministic GUID: node identity + service + per-process entity
        // counter (replay-stable; never random).
        let guid = derive_client_guid(
            &format!("{}/{}", node_data.namespace, node_data.name),
            &info.service_name_cerulion,
            runtime::next_entity_id(),
        );

        let client = match rt.transport.create_service_client(
            &info.service_name_cerulion,
            guid,
            SERVICE_SLICE_LEN,
            info.request_hash,
            info.response_hash,
        ) {
            Ok(c) => c,
            Err(e) => {
                tracing::error!(service = %name, error = %e, "client creation failed");
                return std::ptr::null_mut();
            }
        };

        let data = Box::new(ClientData {
            service_name: info.service_name_cerulion,
            type_name: info.type_name,
            request_bridge: info.request_bridge,
            response_bridge: info.response_bridge,
            client: std::sync::Mutex::new(client),
            gid: guid,
            decode_failures: std::sync::Mutex::new(
                crate::decode_failure_latch::DecodeFailureLatch::new(),
            ),
        });
        let cl = Box::new(ffi::rmw_client_t {
            implementation_identifier: ffi::implementation_identifier_ptr(),
            data: Box::into_raw(data) as *mut c_void,
            service_name: super::leak_ros_name(name),
        });
        let ptr = Box::into_raw(cl);
        // Registry mutation LAST (see rmw_create_publisher).
        {
            let data_ref = &*((*ptr).data as *const ClientData);
            let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
            GraphRegistry::add(
                &mut graph.clients,
                &data_ref.service_name,
                &data_ref.type_name,
            );
            drop(graph);
            // Delivery log / replay: the stable name (`cli:<name>#n`).
            crate::replay::on_entity(
                EntityKind::Client,
                (*ptr).data as usize,
                &data_ref.service_name,
                &(*((*node).data as *const runtime::NodeData)).name,
            );
        }
        runtime::notify_graph_change();
        ptr
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_client(
    node: *mut ffi::rmw_node_t,
    client: *mut ffi::rmw_client_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || client.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*client).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        crate::replay::on_entity_destroyed((*client).data as usize);
        // Borrow-first, free-last.
        {
            let data = &*((*client).data as *const ClientData);
            if let Ok(rt) = runtime::runtime() {
                let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
                GraphRegistry::remove(&mut graph.clients, &data.service_name);
            }
            runtime::notify_graph_change();
        }
        super::unleak_ros_name((*client).service_name);
        drop(Box::from_raw((*client).data as *mut ClientData));
        drop(Box::from_raw(client));
        RMW_RET_OK
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_send_request(
    client: *const ffi::rmw_client_t,
    ros_request: *const c_void,
    sequence_id: *mut i64,
) -> rmw_ret_t {
    // Recording and replay off: one `OnceLock` load, then the live send.
    if !crate::replay::active() || client.is_null() {
        return send_request_live(client, ros_request, sequence_id);
    }
    crate::replay::output((*client).data as usize, OutputKind::Request, || {
        send_request_live(client, ros_request, sequence_id)
    })
}

/// The live `rmw_send_request`.
///
/// # Safety
/// Same contract as [`rmw_send_request`].
unsafe fn send_request_live(
    client: *const ffi::rmw_client_t,
    ros_request: *const c_void,
    sequence_id: *mut i64,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if client.is_null() || ros_request.is_null() || sequence_id.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*client).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*client).data as *const ClientData);
        let _downgrades = crate::rosidl_buffer::PublishScope::open(&data.service_name);
        let payload = match data.request_bridge.flatten_payload_only(ros_request) {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "request flatten failed");
                return RMW_RET_ERROR;
            }
        };
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(mut client_guard) = runtime::lock_unpoisoned(&data.client) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        crate::replay::capture_output_body(&payload);
        match client_guard.send_request(&payload) {
            Ok(seq) => {
                *sequence_id = seq;
                RMW_RET_OK
            }
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "send_request failed");
                RMW_RET_ERROR
            }
        }
    })
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_response(
    client: *const ffi::rmw_client_t,
    request_header: *mut ffi::rmw_service_info_t,
    ros_response: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    // Recording and replay off: one `OnceLock` load, then the live take.
    if !crate::replay::active() {
        return take_response_live(client, request_header, ros_response, taken);
    }
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if client.is_null() || request_header.is_null() || ros_response.is_null() || taken.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*client).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let key = (*client).data as usize;
        match crate::replay::take_action(key, TakeKind::Response) {
            TakeAction::Live => take_response_live(client, request_header, ros_response, taken),
            TakeAction::Record(r) => r.take(
                key,
                TakeKind::Response,
                || take_response_live(client, request_header, ros_response, taken),
                |_| service_take_meta(*taken, request_header),
            ),
            TakeAction::Serve(rec) => {
                let data = &*((*client).data as *const ClientData);
                serve_recorded(
                    &data.response_bridge,
                    &data.service_name,
                    rec,
                    request_header,
                    ros_response,
                    taken,
                )
            }
            TakeAction::Refuse => RMW_RET_ERROR,
        }
    })
}

/// The live `rmw_take_response`.
///
/// # Safety
/// Same contract as [`rmw_take_response`].
unsafe fn take_response_live(
    client: *const ffi::rmw_client_t,
    request_header: *mut ffi::rmw_service_info_t,
    ros_response: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if client.is_null() || request_header.is_null() || ros_response.is_null() || taken.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*client).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        *taken = false;
        let data = &*((*client).data as *const ClientData);
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(mut client_guard) = runtime::lock_unpoisoned(&data.client) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };

        let bridge = &data.response_bridge;
        let mut got_seq: Option<i64> = None;
        let result = client_guard.try_take_one_response(|seq, payload| {
            // The request site's arm, on the client: refused BEFORE any
            // write, with the member named, so a caller reusing one response
            // message across calls never reads a mixture of two replies.
            if let Some((var_idx, path)) = bridge.unwritable_bool_seq() {
                crate::decode_failure_latch::report_decode_entry_refused(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceResponse,
                    &data.service_name,
                    &data.type_name,
                    payload.len(),
                    var_idx,
                    &crate::type_bridge_cpp::bool_seq_no_assign_detail(path),
                );
                return;
            }
            if bridge.unflatten(payload, ros_response) {
                crate::replay::capture_take_body(payload, seq, 0);
                got_seq = Some(seq);
                crate::decode_failure_latch::report_decode_success(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceResponse,
                    &data.service_name,
                );
            } else {
                // Without this report the failure is silent; see the request site above.
                crate::decode_failure_latch::report_decode_failure(
                    &data.decode_failures,
                    crate::decode_failure_latch::DecodeSite::ServiceResponse,
                    &data.service_name,
                    &data.type_name,
                    payload.len(),
                );
            }
        });
        drop(client_guard);

        match result {
            Ok(_) => {}
            Err(e) => {
                tracing::error!(service = %data.service_name, error = %e, "take_response failed");
                return RMW_RET_ERROR;
            }
        }
        if let Some(seq) = got_seq {
            *taken = true;
            let hdr = &mut *request_header;
            hdr.request_id.sequence_number = seq;
            hdr.request_id.writer_guid = data.gid.map(|b| b as _);
            hdr.source_timestamp = 0;
            hdr.received_timestamp = match runtime::runtime() {
                Ok(rt) => rt.transport.clock().now_ns() as i64,
                Err(_) => 0,
            };
        }
        RMW_RET_OK
    })
}

/// Server availability: true when the transport sees ≥1 subscriber on
/// the service's request topic (covers in-process and same-host
/// servers; any subscriber on that topic counts). Cross-machine
/// discovery is not implemented — local SHM graphs
/// are single-host by construction and the rcl retry loop tolerates
/// the latency.
///
/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_service_server_is_available(
    node: *const ffi::rmw_node_t,
    client: *const ffi::rmw_client_t,
    is_available: *mut bool,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() || client.is_null() || is_available.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*node).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*client).data as *const ClientData);
        // Poisoned = wedged: never re-enter torn iceoryx2 state.
        let Some(client_guard) = runtime::lock_unpoisoned(&data.client) else {
            tracing::error!("entity wedged by an earlier panic; failing call");
            return RMW_RET_ERROR;
        };
        let live = client_guard.server_available();
        drop(client_guard);
        let (ret, answer) =
            crate::replay::graph(GraphQuery::ServerIsAvailable, &data.service_name, || {
                (RMW_RET_OK, live as u64)
            });
        *is_available = answer != 0;
        ret
    })
}

/// # Safety
/// Caller frees with `rmw_client_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_client_allocate() -> *mut ffi::rmw_client_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_client_t>()))
}

/// # Safety
/// `client` must come from `rmw_client_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_client_free(client: *mut ffi::rmw_client_t) {
    if !client.is_null() {
        drop(Box::from_raw(client));
    }
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_client_request_publisher_get_actual_qos(
    client: *const ffi::rmw_client_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if client.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = super::default_actual_qos();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_client_response_subscription_get_actual_qos(
    client: *const ffi::rmw_client_t,
    qos: *mut ffi::rmw_qos_profile_t,
) -> rmw_ret_t {
    if client.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *qos = super::default_actual_qos();
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_get_gid_for_client(
    client: *const ffi::rmw_client_t,
    gid: *mut ffi::rmw_gid_t,
) -> rmw_ret_t {
    if client.is_null() || gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if !ffi::is_our_identifier((*client).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    let data = &*((*client).data as *const ClientData);
    let g = &mut *gid;
    g.implementation_identifier = ffi::implementation_identifier_ptr();
    // Zero tail, size-agnostic — see rmw_get_gid_for_publisher.
    g.data = Default::default();
    g.data[..16].copy_from_slice(&data.gid);
    RMW_RET_OK
}
