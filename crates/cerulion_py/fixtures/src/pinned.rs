// SPDX-License-Identifier: AGPL-3.0-only
//! Fresh-process oracle for a Python buffer that survives node destruction.
use super::{load_pynode, ExitCode, TransportManager};
use cerulion_core::graph::node_host_abi::{
    register_context, with_context, HostContext, HostPortSpec, HOST_API,
};
use cerulion_core::node_abi::{self as abi, Bytes, ContextHandle, NodeHandle};
use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
use std::sync::Arc;

pub(super) fn run(manager: &TransportManager, path: &str) -> Result<ExitCode, String> {
    let mut loaded = load_pynode(manager, path, None)?;
    // SAFETY (U01): this is the built Python fixture's checked stable export;
    // its immutable descriptors and native constructors obey the loader contract.
    let library =
        unsafe { abi::CheckedNodeLibrary::load(std::path::Path::new(path), HOST_API.capabilities) }
            .map_err(|error| error.to_string())?;
    let api = *library.api();
    // SAFETY (U12): metadata was validated by CheckedNodeLibrary and its owner is live.
    let metadata =
        unsafe { std::slice::from_raw_parts(api.metadata.ptr, api.metadata.len as usize) };
    let document: serde_json::Value =
        serde_json::from_slice(metadata).map_err(|error| error.to_string())?;
    assert_eq!(document["retain_process"], true);
    let mut specs = Vec::new();
    for (key, direction, read_mode) in [
        ("inputs", LeaseDirection::Read, abi::read_mode::HELD),
        ("outputs", LeaseDirection::Write, 0),
    ] {
        for fact in document[key].as_array().ok_or("missing ports")? {
            let scalar = |name| -> Result<u32, String> {
                u32::try_from(fact[name].as_u64().ok_or("missing wire fact")?)
                    .map_err(|error| error.to_string())
            };
            specs.push(HostPortSpec {
                name: fact["name"].as_str().ok_or("missing port name")?.into(),
                direction,
                layout: PluginWireLayout {
                    schema_hash: fact["schema_hash"].as_u64().ok_or("missing schema")?,
                    fixed_size: scalar("wire_fixed_size")?,
                    variable_fields: scalar("variable_count")?,
                    alignment: scalar("alignment")?,
                },
                read_mode,
                accumulate_cap: None,
            });
        }
    }
    let native = loaded.context.take().ok_or("missing context")?;
    let mut environment: std::collections::HashMap<String, String> = std::env::vars().collect();
    environment.insert(
        "CERULION_WORKSPACE".into(),
        native
            .env_opt("CERULION_WORKSPACE")
            .ok_or("missing workspace")?,
    );
    let host = HostContext::new(native, specs, Arc::new(environment), 8)
        .map_err(|error| format!("{error:?}"))?;
    let context = ContextHandle {
        value: register_context(host).map_err(|error| format!("{error:?}"))?,
    };
    // Dropping the actual loader entry retains its library according to the
    // descriptor. The separate checked owner below is released after destroy.
    drop(loaded.entry);
    let params = Bytes {
        ptr: std::ptr::null(),
        len: 0,
    };
    let mut first = NodeHandle { value: 0 };
    with_context(context.value, |host| host.begin_node_call())
        .map_err(|error| format!("{error:?}"))?;
    // SAFETY (U06): callbacks were fully checked, aligned outputs are local,
    // host context remains registered and each node call is strictly serial.
    unsafe {
        assert_eq!(
            (api.create)(&HOST_API, context, params, &mut first).code,
            abi::status_code::OK
        );
        with_context(context.value, |host| host.begin_fire())
            .map_err(|error| format!("{error:?}"))?;
        assert_eq!((api.tick)(first).code, abi::status_code::FAILED);
        assert_eq!((api.shutdown)(first).code, abi::status_code::OK);
        assert_eq!((api.destroy)(first).code, abi::status_code::OK);
    }
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        1
    );
    let mut binding = abi::PortBindingV1 {
        port: abi::PortHandle { value: 0 },
        topic: params,
        configured_capacity: 0,
        read_mode: 0,
    };
    let output = &document["outputs"][0];
    let request = abi::LoanRequestV1 {
        schema_hash: output["schema_hash"].as_u64().unwrap(),
        minimum_capacity: 40 + output["wire_fixed_size"].as_u64().unwrap() as u32,
        wire_fixed_size: output["wire_fixed_size"].as_u64().unwrap() as u32,
        variable_count: 0,
        alignment: output["alignment"].as_u64().unwrap() as u32,
    };
    let mut slot = abi::WriteSlotV1 {
        data: std::ptr::null_mut(),
        lease: abi::LeaseHandle { value: 0 },
        schema_hash: 0,
        capacity: 0,
        configured_capacity: 0,
        alignment: 0,
        prefix_size: 0,
    };
    // SAFETY (U05): real process table, registered context, aligned caller storage.
    let refused = unsafe {
        assert_eq!(
            (HOST_API.bind_port)(context, 0, abi::direction::OUTPUT, &mut binding).code,
            0
        );
        (HOST_API.loan_output)(context, binding.port, request, &mut slot)
    };
    assert_eq!(refused.code, abi::status_code::INVALID_STATE);
    assert_eq!(slot.lease.value, 0);
    assert!(loaded
        .output_subscribers
        .iter_mut()
        .all(|(_, receiver)| receiver.view_raw().unwrap().is_none()));
    println!("pinned after_destroy=1 outstanding=1 refused=9");
    drop(library);
    let mut second = NodeHandle { value: 0 };
    // SAFETY (U06): the actual loader entry retained the code for process
    // lifetime. The old buffer and its registered context still pin the lease;
    // new Python init reads the old bytes through CPython's real buffer owner.
    unsafe {
        assert_eq!(
            (api.create)(&HOST_API, context, params, &mut second).code,
            0
        );
        assert_eq!((api.shutdown)(second).code, 0);
        assert_eq!((api.destroy)(second).code, 0);
    }
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        1
    );
    println!("pinned after_unload=1 bytes=d4c3b2a1 outstanding=1");
    with_context(context.value, |host| host.end_node_call())
        .map_err(|error| format!("{error:?}"))?;
    // The escaped view intentionally remains in builtins until child exit.
    // Retain its registered host context as required while that lease is live.
    Ok(ExitCode::SUCCESS)
}
