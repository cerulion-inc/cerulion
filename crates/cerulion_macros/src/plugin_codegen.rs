// SPDX-License-Identifier: AGPL-3.0-only
//! Stable exports around a node-local context. Metadata comes from gen_cdylib.

use proc_macro2::TokenStream;
use quote::quote;

use crate::crate_root;
use crate::parse::{FieldInputAttr, FieldOutputAttr};

pub(crate) fn generate(
    entry: &syn::Ident,
    node: &syn::Ident,
    inputs: &[FieldInputAttr],
    outputs: &[FieldOutputAttr],
    external: bool,
) -> TokenStream {
    let root = crate_root::root();
    let input_types = inputs.iter().map(|input| &input.field_type);
    let input_caps = inputs.iter().map(|input| match input.accumulate {
        Some(cap) => {
            quote!(Some(#cap))
        }
        None => quote!(None),
    });
    let output_types = outputs.iter().map(|output| &output.field_type);
    let register = crate::input_accumulate::register_calls_for(inputs, &quote! { instance });
    let required = if inputs.iter().any(|input| input.accumulate.is_some()) {
        quote!(#root::node_abi::capability::ACCUMULATE)
    } else {
        quote!(0)
    };
    let external_source = if external {
        quote! {
            if let Some(source) = self.external { return Ok(source); }
            let source = match self.entry.inner.__cer_user_external_source() {
                #root::graph::node::ExternalSource::Fd(fd) => #root::node_abi::ExternalSourceV1 {
                    fd: i64::from(fd), kind: #root::node_abi::external::DEVICE_FD, flags: 0,
                },
                #root::graph::node::ExternalSource::Blocking(helper) => {
                    let fd = #root::graph::node::spawn_cdylib_blocking_doorbell(helper).ok_or_else(||
                        #root::error::TransportError::NodeError { node_id: stringify!(#node).into(),
                            reason: "failed to create the Blocking doorbell pipe/helper".into() })?;
                    #root::node_abi::ExternalSourceV1 { fd: i64::from(fd), kind: #root::node_abi::external::DOORBELL_FD,
                        flags: #root::node_abi::external::HOST_CLOSES | #root::node_abi::external::RETAIN_PROCESS }
                }
                #root::graph::node::ExternalSource::HostDriven => #root::node_abi::ExternalSourceV1 {
                    fd: -1, kind: #root::node_abi::external::HOST_DRIVEN, flags: 0,
                },
                _ => return Err(#root::error::TransportError::NodeError { node_id: stringify!(#node).into(),
                    reason: "unsupported external source variant".into() }),
            };
            self.external = Some(source);
            Ok(source)
        }
    } else {
        quote! { Err(#root::error::TransportError::NodeError { node_id: stringify!(#node).into(),
        reason: "node does not declare an external source".into() }) }
    };
    let external_pointer = if external {
        quote!(&__CER_STABLE_EXTERNAL)
    } else {
        quote!(::std::ptr::null())
    };
    let capabilities = if external {
        quote!(#root::node_abi::capability::STATE | #root::node_abi::capability::EXTERNAL)
    } else {
        quote!(#root::node_abi::capability::STATE)
    };

    quote! {
        struct __CerPluginEntry {
            entry: #entry,
            context: Option<#root::graph::node_plugin_abi::NodeContext>,
            __cer_accumulators: #root::graph::node_plugin_abi::AccumulatorSet,
            external: Option<#root::node_abi::ExternalSourceV1>,
        }

        impl #root::graph::node_plugin_abi::PluginNode for __CerPluginEntry {
            fn create(client: #root::graph::node_plugin_abi::HostClient, params: &[u8]) -> #root::error::TransportResult<Self> {
                use #root::graph::node::NodeEntry as _;
                #root::graph::node::cdylib_platform_check().map_err(|reason|
                    #root::error::TransportError::NodeError { node_id: stringify!(#node).into(), reason })?;
                let mut entry = #entry::new();
                if !params.is_empty() {
                    let values = ::std::str::from_utf8(params).map_err(|error|
                        #root::error::TransportError::NodeError { node_id: stringify!(#node).into(), reason: error.to_string() })
                        .and_then(|text| #root::graph::params::params_from_json(text).map_err(|reason|
                            #root::error::TransportError::NodeError { node_id: stringify!(#node).into(), reason }))?;
                    entry.inner.__cer_apply_params(&values).map_err(|error|
                        #root::error::TransportError::NodeError { node_id: stringify!(#node).into(), reason: error.to_string() })?;
                }
                entry.__cer_params_at_init = Some(entry.inner.__cer_param_values());
                let mut context = #root::graph::node_plugin_abi::NodeContext::new(client.clone(), &entry.info()?)?;
                let rust_log = context.env_str("RUST_LOG", "");
                #root::graph::node::install_cdylib_stderr_tracing(
                    if rust_log.is_empty() { None } else { Some(rust_log.as_str()) });
                let iox2_log = context.env_str("IOX2_LOG_LEVEL", "");
                #root::iceoryx_logger::init_iceoryx_log_level(
                    if iox2_log.is_empty() { None } else { Some(iox2_log.as_str()) });
                entry.inner.__cer_rt.clock = ::std::sync::Arc::clone(context.clock());
                entry.inner.__cer_rt.shutdown_signal = context.shutdown_signal().clone();
                entry.inner.__cer_plugin = Some(client);
                let mut instance = Self { entry, context: None, __cer_accumulators: Default::default(), external: None };
                // Register before user init, just as the native emitter does.
                #register
                instance.entry.inner.__cer_plugin_init(&mut context)?;
                instance.context = Some(context);
                Ok(instance)
            }

            fn flush_deferred(&self) -> #root::error::TransportResult<()> {
                self.entry.inner.__cer_plugin.as_ref()
                    .expect("plugin create installed local callbacks").flush_deferred()
            }

            fn tick(&mut self) -> #root::error::TransportResult<()> {
                let context = self.context.as_mut().ok_or_else(|| #root::error::TransportError::NodeError {
                    node_id: stringify!(#node).into(), reason: "node not initialized".into(),
                })?;
                let client = self.entry.inner.__cer_plugin.as_ref().expect("plugin create installed local callbacks").clone();
                let result = self.entry.inner.__cer_plugin_tick(context, &mut self.__cer_accumulators);
                if context.shutdown_signal().is_requested() { client.request_shutdown(); }
                if let Some(error) = client.take_failure() { return Err(error); }
                result
            }

            fn shutdown(&mut self) -> #root::error::TransportResult<()> {
                if self.context.is_none() { return Err(#root::error::TransportError::NodeError {
                    node_id: stringify!(#node).into(), reason: "node already shut down".into(),
                }); }
                let result = self.entry.inner.__cer_user_shutdown();
                self.context = None;
                if let Some(client) = &self.entry.inner.__cer_plugin {
                    if let Some(error) = client.take_failure() { return Err(error); }
                }
                result
            }

            fn state_shape() -> u64 { <#node as #root::state::CerulionState>::STATE_SHAPE }
            fn inline_safe(&self) -> bool { <#node as #root::state::CerulionState>::INLINE_SAFE }
            fn probe(&self) -> bool { #root::state::CerulionState::cer_probe(&self.entry.inner) }

            fn prepare_capture(&self) -> #root::error::TransportResult<()> {
                if let Some(reason) = self.entry.__cer_param_change() {
                    return Err(#root::error::TransportError::GraphError { reason });
                }
                Ok(())
            }

            fn capture(&self, sink: &mut dyn #root::state::StateSink) -> #root::error::TransportResult<()> {
                #root::state::CerulionState::cer_capture(&self.entry.inner, sink)
                    .map_err(|error| {
                        if matches!(error, #root::state::StateError::SinkFull) { sink.refuse(); }
                        #root::error::TransportError::GraphError { reason: error.to_string() }
                    })?;
                self.__cer_accumulators.capture(sink)
            }

            fn restore(&mut self, bytes: &[u8]) -> #root::error::TransportResult<()> {
                let mut cursor = #root::state::StateCursor::new(bytes);
                #root::state::CerulionState::cer_restore(&mut self.entry.inner, &mut cursor)
                    .map_err(|error| #root::error::TransportError::GraphError { reason: error.to_string() })?;
                self.__cer_accumulators.restore(&mut cursor)?;
                cursor.finish().map_err(|error| #root::error::TransportError::GraphError { reason: error.to_string() })
            }

            fn external_source(&mut self) -> #root::error::TransportResult<#root::node_abi::ExternalSourceV1> { #external_source }
        }

        static __CER_STABLE_NODES: ::std::sync::OnceLock<#root::graph::node_plugin_abi::NodeExports<__CerPluginEntry>> = ::std::sync::OnceLock::new();
        fn __cer_stable_nodes() -> &'static #root::graph::node_plugin_abi::NodeExports<__CerPluginEntry> {
            __CER_STABLE_NODES.get_or_init(Default::default)
        }

        unsafe extern "C" fn __cer_stable_create(host: *const #root::node_abi::HostApiV1, context: #root::node_abi::ContextHandle,
            params: #root::node_abi::Bytes, output: *mut #root::node_abi::NodeHandle) -> #root::node_abi::Status {
            // SAFETY (U06/U12): caller obeys the immutable create contract;
            // the local adapter checks pointers/shape before any node effect.
            unsafe { __cer_stable_nodes().create(host, context, params, output, #required) }
        }
        unsafe extern "C" fn __cer_stable_tick(handle: #root::node_abi::NodeHandle) -> #root::node_abi::Status { __cer_stable_nodes().tick(handle) }
        unsafe extern "C" fn __cer_stable_shutdown(handle: #root::node_abi::NodeHandle) -> #root::node_abi::Status { __cer_stable_nodes().shutdown(handle) }
        unsafe extern "C" fn __cer_stable_destroy(handle: #root::node_abi::NodeHandle) -> #root::node_abi::Status { __cer_stable_nodes().destroy(handle) }
        unsafe extern "C" fn __cer_stable_error(token: u64, bytes: #root::node_abi::MutableBytes, required: *mut u64) -> #root::node_abi::Status {
            // SAFETY (U06/U12): caller storage is borrowed only for this call;
            // the local adapter checks bounds before copying owner-local text.
            unsafe { __cer_stable_nodes().copy_error(token, bytes, required) }
        }
        unsafe extern "C" fn __cer_stable_extension(_id: u64, _version: u32, _output: *mut #root::node_abi::ExtensionDescriptorV1) -> #root::node_abi::Status {
            #root::node_abi::Status { code: #root::node_abi::status_code::UNSUPPORTED, ..Default::default() }
        }
        unsafe extern "C" fn __cer_stable_shape(output: *mut u64) -> #root::node_abi::Status {
            // SAFETY (U06/U12): local adapter checks caller-owned scalar output.
            unsafe { __cer_stable_nodes().shape(output) }
        }
        unsafe extern "C" fn __cer_stable_capture(handle: #root::node_abi::NodeHandle, sink: *const #root::node_abi::StateSinkV1, capacity: u64) -> #root::node_abi::Status {
            // SAFETY (U06/U12): the sink/context is borrowed synchronously;
            // local adapter checks its complete pointer/callback before use.
            unsafe { __cer_stable_nodes().capture(handle, sink, capacity) }
        }
        unsafe extern "C" fn __cer_stable_restore(handle: #root::node_abi::NodeHandle, bytes: #root::node_abi::Bytes) -> #root::node_abi::Status {
            // SAFETY (U06/U12): the local adapter validates the borrowed span.
            unsafe { __cer_stable_nodes().restore(handle, bytes) }
        }
        unsafe extern "C" fn __cer_stable_inline(handle: #root::node_abi::NodeHandle, output: *mut u32) -> #root::node_abi::Status {
            // SAFETY (U06/U12): checked caller-owned scalar output.
            unsafe { __cer_stable_nodes().inline_safe(handle, output) }
        }
        unsafe extern "C" fn __cer_stable_probe(handle: #root::node_abi::NodeHandle) -> #root::node_abi::Status { __cer_stable_nodes().probe(handle) }
        unsafe extern "C" fn __cer_stable_external(handle: #root::node_abi::NodeHandle, output: *mut #root::node_abi::ExternalSourceV1) -> #root::node_abi::Status {
            // SAFETY (U06/U12): checked caller-owned primitive source descriptor.
            unsafe { __cer_stable_nodes().external(handle, output) }
        }

        static __CER_STABLE_STATE: #root::node_abi::StateApiV1 = #root::node_abi::StateApiV1 {
            header: #root::node_abi::TableHeaderV1::for_type::<#root::node_abi::StateApiV1>(),
            shape: __cer_stable_shape, capture: __cer_stable_capture, restore: __cer_stable_restore,
            inline_safe: __cer_stable_inline, probe: __cer_stable_probe,
        };
        static __CER_STABLE_EXTERNAL: #root::node_abi::ExternalApiV1 = #root::node_abi::ExternalApiV1 {
            header: #root::node_abi::TableHeaderV1::for_type::<#root::node_abi::ExternalApiV1>(), describe: __cer_stable_external,
        };
        static __CER_STABLE_API: ::std::sync::OnceLock<&'static #root::node_abi::NodeApiV1> = ::std::sync::OnceLock::new();
        static __CER_STABLE_METADATA: ::std::sync::OnceLock<::std::ffi::CString> = ::std::sync::OnceLock::new();
        fn __cer_stable_api() -> *const #root::node_abi::NodeApiV1 {
            *__CER_STABLE_API.get_or_init(|| {
                let metadata = __CER_STABLE_METADATA.get_or_init(|| {
                    #root::graph::node_plugin_abi::metadata(
                        INFO_JSON.get_or_init(__cer_build_info_json).as_c_str(),
                        &[#(#root::graph::node_plugin_abi::HostClient::request::<#input_types>()),*],
                        &[#(#root::graph::node_plugin_abi::HostClient::request::<#output_types>()),*],
                        &[#(#input_caps),*],
                        <#node as #root::state::CerulionState>::INLINE_SAFE,
                    ).expect("generated wire facts match the common metadata")
                }).as_bytes();
                // hot-path-alloc-ok: bootstrap owns one immutable table for the library lifetime.
                ::std::boxed::Box::leak(::std::boxed::Box::new(#root::node_abi::NodeApiV1 {
                    header: #root::node_abi::TableHeaderV1::for_type::<#root::node_abi::NodeApiV1>(),
                    metadata: #root::node_abi::Bytes { ptr: metadata.as_ptr(), len: metadata.len() as u64 },
                    required_host_capabilities: #required, capabilities: #capabilities,
                    create: __cer_stable_create, tick: __cer_stable_tick, shutdown: __cer_stable_shutdown,
                    destroy: __cer_stable_destroy, copy_error: __cer_stable_error, query_extension: __cer_stable_extension,
                    state: &__CER_STABLE_STATE, external: #external_pointer,
                }))
            }) as *const _
        }
        #root::node_abi::export_node_api!(__cer_stable_api);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stable_author_metadata_emits_the_native_inline_safe_type_constant() {
        let tokens = generate(
            &syn::parse_quote!(Entry),
            &syn::parse_quote!(Node),
            &[],
            &[],
            false,
        );
        let compact: String = tokens.to_string().split_whitespace().collect();
        assert!(compact.contains("metadata("));
        assert!(compact.contains("<Nodeas"));
        assert!(compact.contains("::state::CerulionState>::INLINE_SAFE"));
        assert!(compact.contains("fninline_safe(&self)->bool"));
        assert!(!compact.contains("capability::INLINE_SAFE"));
    }
}
