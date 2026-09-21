// SPDX-License-Identifier: AGPL-3.0-only
//! rmw services / clients over the deterministic
//! request-response layer. Actions need nothing extra — rcl composes
//! them from services + topics.

use std::os::raw::{c_char, c_void};

use cerulion_core::transport::service::{derive_client_guid, ServiceEnvelope};
use cerulion_core::wire::MaxSliceLen;

use crate::bridge::AnyBridge;
use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
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
            if bridge.unflatten(payload, ros_request) {
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
            hdr.request_id.writer_guid = envelope.client_guid;
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
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if service.is_null() || request_header.is_null() || ros_response.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*service).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*service).data as *const ServiceData);
        let envelope = ServiceEnvelope {
            client_guid: (*request_header).writer_guid,
            sequence: (*request_header).sequence_number,
        };
        // Flatten the response through the bridge into the service payload.
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
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if client.is_null() || ros_request.is_null() || sequence_id.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*client).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        let data = &*((*client).data as *const ClientData);
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
            if bridge.unflatten(payload, ros_response) {
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
            hdr.request_id.writer_guid = data.gid;
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
        *is_available = client_guard.server_available();
        RMW_RET_OK
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
