// SPDX-License-Identifier: AGPL-3.0-only
//! Embedded runtime and JNI support for Android applications.
//!
//! Publishing parses JSON and polling builds JSON text; both allocate. They
//! are an app-side convenience path, not the zero-copy node path.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

/// Runtime configuration and input validation.
pub mod config;
/// Wire-frame-to-JSON decoding for app subscriptions.
pub mod decode;
/// JSON-to-wire-frame encoding for app publishers.
pub mod encode;
/// Stable error types and error-code constants.
pub mod error;
/// Graph node types linked into the app library.
pub mod nodes;
/// Synchronous runtime lifecycle and status types.
pub mod runtime;

mod host;
mod jni;
mod log;
mod subscribe;

/// Embedded runtime API and observable status types.
pub use config::{LanAccessConfig, LogLevelConfig};
/// Stable error-code constants.
pub use error::{codes, EmbedError};
/// TEST SEAM: install the logcat subscriber over a fake sink.
#[cfg(any(test, feature = "test-helpers"))]
#[doc(hidden)]
pub use log::{install_with_sink as install_log_sink_for_test, LogPriority, LogSink};
pub use runtime::{
    EmbeddedRuntime, NodeFailure, NodeFailures, RecorderSummary, RuntimeState, Status,
    MAX_PUBLISH_STAMP_LEAD_NS,
};
pub use subscribe::SubscriptionCounts;

/// Link graph node types into the app library.
///
/// Write it once, in the app runtime crate (the `cdylib` that Java loads
/// with `CerulionRuntime.loadLibrary`). Each pair maps a graph YAML `type:`
/// to the node's generated entry type, from a node crate the runtime crate
/// depends on with `default-features = false`:
///
/// ```ignore
/// cerulion_android::embed_nodes! {
///     "timer" => timer::TimerNodeEntry,
///     "printer" => printer::PrinterNodeEntry,
/// }
/// ```
///
/// It defines the library's `JNI_OnLoad`, which registers the node types
/// when Java loads the library, before any runtime call. An app that writes
/// its own `JNI_OnLoad` calls [`nodes::register`] from it instead.
#[macro_export]
macro_rules! embed_nodes {
    ($($node_type:literal => $entry:path),+ $(,)?) => {
        /// Registers the node types linked into this library.
        #[unsafe(no_mangle)]
        pub extern "system" fn JNI_OnLoad(
            vm: *mut ::core::ffi::c_void,
            reserved: *mut ::core::ffi::c_void,
        ) -> ::core::ffi::c_int {
            $crate::nodes::on_load(
                vm,
                reserved,
                &[$(
                    $crate::nodes::LinkedNode {
                        node_type: $node_type,
                        factory: || {
                            ::std::boxed::Box::new(<$entry>::new())
                                as ::std::boxed::Box<dyn $crate::nodes::NodeEntry>
                        },
                    }
                ),+],
            )
        }
    };
}
