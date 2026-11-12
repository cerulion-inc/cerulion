// SPDX-License-Identifier: AGPL-3.0-only
//! Frozen C tables for node plugins. Transport and allocation owners stay on
//! their originating side; only integer handles and borrowed byte spans cross.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
#![warn(missing_docs)]

mod checker;
mod host;
#[cfg(feature = "loader")]
mod loader;
mod node;
mod types;
mod validation;

pub use checker::*;
pub use host::*;
/// Native loader dependency matched to the maintained Stabby integration.
#[cfg(feature = "loader")]
pub use libloading;
#[cfg(feature = "loader")]
pub use loader::*;
pub use node::*;
/// The maintained layout checker, available through the runtime re-export.
pub use stabby;
pub use types::*;
pub use validation::*;

/// Boundary major supported by these permanently frozen records.
pub const BOUNDARY_MAJOR: u32 = 1;
/// Symbol exported by every stable node library.
pub const BOOTSTRAP_SYMBOL: &[u8] = b"cerulion_node_api_v1\0";
/// Permanently frozen bootstrap signature; tables are returned by pointer.
pub type BootstrapFn = extern "C" fn() -> *const NodeApiV1;

/// Export a node table constructor without requiring a direct Stabby dependency
/// in the node's manifest. The checked symbols use Stabby's pinned v3 discovery
/// protocol, and its complete report ID is checked before exposing the function.
/// The constructor returns immutable library-lifetime storage and may allocate
/// only during its first, cold invocation. Constructor panics return null.
/// Host checks _stabbied_v3 discovery before invoking the raw bootstrap; the
/// constructor executes before its returned table header can be inspected.
#[macro_export]
macro_rules! export_node_api {
    ($constructor:path) => {
        #[doc = "Return the immutable node interface for this library."]
        #[no_mangle]
        pub extern "C" fn cerulion_node_api_v1() -> *const $crate::NodeApiV1 {
            ::std::panic::catch_unwind($constructor).unwrap_or(::std::ptr::null())
        }
        #[doc = "Stabby v3 checked bootstrap discovery."]
        #[no_mangle]
        pub extern "C" fn cerulion_node_api_v1_stabbied_v3(
            report: &$crate::stabby::abi::report::TypeReport,
        ) -> ::core::option::Option<$crate::BootstrapFn> {
            $crate::bootstrap_matches(report).then_some(cerulion_node_api_v1)
        }
        #[doc = "Stabby v3 complete bootstrap type report."]
        #[no_mangle]
        pub extern "C" fn cerulion_node_api_v1_stabbied_v3_report(
        ) -> &'static $crate::stabby::abi::report::TypeReport {
            <$crate::BootstrapFn as $crate::stabby::IStable>::REPORT
        }
    };
}
