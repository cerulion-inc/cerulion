// SPDX-License-Identifier: AGPL-3.0-only
use crate::*;
use std::path::Path;

use stabby::libloading::StabbyLibrary;

/// Checked immutable node descriptor and the native library retaining its code.
/// No instance is created during discovery. Runtime ownership remains separate.
pub struct CheckedNodeLibrary {
    api: NodeApiV1,
    library: libloading::Library,
}

// SAFETY: moving this owner moves no pointee or writable node state. The native
// library is Send, and the checked descriptor refers only to immutable storage
// retained by that library. Executing callbacks still requires the documented
// unsafe instance exclusion and lifetime contract.
unsafe impl Send for CheckedNodeLibrary {}

impl CheckedNodeLibrary {
    /// Open and fully check the frozen interface before any node creation.
    ///
    /// # Safety
    /// The library must obey the stable bootstrap protocol, expose readable
    /// initialized descriptor allocations of their advertised complete size,
    /// and retain their immutable storage until unload. Native constructors and
    /// callbacks must satisfy the platform loader's safety requirements. These
    /// checks reject malformed scalar contracts, not arbitrary malicious code.
    ///
    /// # Errors
    /// Refuses missing stable symbols, incompatible layouts/major/capabilities,
    /// null required callbacks, invalid spans and unsupported native platforms.
    pub unsafe fn load(path: &Path, host_capabilities: u64) -> Result<Self, AbiError> {
        if !native_platform_supported(std::env::consts::OS) {
            return Err(AbiError::Platform);
        }
        // SAFETY: caller authorizes native initialization and guarantees the
        // library obeys the loader and bootstrap protocol. Owner retained below.
        let library = unsafe { libloading::Library::new(path) }.map_err(|e| AbiError::Library {
            reason: e.to_string(),
        })?;
        // SAFETY: the versioned report symbol has this permanently frozen Stabby
        // signature; its immutable report storage remains live with the library.
        let report = unsafe {
            library.get::<extern "C" fn() -> &'static stabby::abi::report::TypeReport>(
                b"cerulion_node_api_v1_stabbied_v3_report\0",
            )
        }
        .map_err(|e| AbiError::Bootstrap {
            reason: e.to_string(),
        })?;
        if !bootstrap_matches(report()) {
            return Err(AbiError::Layout);
        }
        // SAFETY: Stabby checks the bootstrap signature against its report;
        // the preceding full-ID comparison also rejects shorter report prefixes.
        // No embedded NUL is passed to Stabby's suffix concatenation.
        let bootstrap = unsafe {
            library.get_stabbied::<BootstrapFn>(&BOOTSTRAP_SYMBOL[..BOOTSTRAP_SYMBOL.len() - 1])
        }
        .map_err(|e| AbiError::Bootstrap {
            reason: e.to_string(),
        })?;
        // SAFETY: the bootstrap contract guarantees initialized immutable table
        // storage of the advertised size; all scalar/callback checks precede the
        // typed read. Library ownership retains every resulting borrowed pointer.
        let api = unsafe { read_node_api(bootstrap()) }?;
        negotiate(api.required_host_capabilities, host_capabilities)?;
        Ok(Self { api, library })
    }

    /// Borrow the checked descriptor while its native library stays retained.
    pub fn api(&self) -> &NodeApiV1 {
        &self.api
    }

    /// Retain code for a non-cancellable helper until process exit. The caller
    /// must separately retain its host context and any callback-owned resources.
    pub fn retain_for_process(self) {
        core::mem::forget(self.library);
    }
}

/// Native libraries use the supported POSIX loaders, including Android bionic.
fn native_platform_supported(os: &str) -> bool {
    matches!(os, "linux" | "macos" | "android")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn supported_native_platforms_are_frozen() {
        for os in ["linux", "macos", "android"] {
            assert!(native_platform_supported(os));
        }
        for os in ["windows", "ios", "freebsd", "unknown"] {
            assert!(!native_platform_supported(os));
        }
    }

    #[test]
    #[cfg(any(target_os = "linux", target_os = "macos", target_os = "android"))]
    fn non_library_file_reports_native_open_failure() {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
        // SAFETY: the crate's actual text manifest is not native code. The OS
        // loader refuses its format before any constructor can execute.
        assert!(matches!(
            unsafe { CheckedNodeLibrary::load(&manifest, 0) },
            Err(AbiError::Library { .. })
        ));
    }

    #[test]
    #[cfg(any(target_os = "linux", target_os = "macos", target_os = "android"))]
    fn existing_system_library_without_stable_exports_requires_rebuild() {
        #[cfg(target_os = "linux")]
        let path = Path::new("libc.so.6");
        #[cfg(target_os = "android")]
        let path = Path::new("libc.so");
        #[cfg(target_os = "macos")]
        let path = Path::new("/usr/lib/libSystem.B.dylib");
        // SAFETY: this is the process's trusted C runtime, whose constructors
        // obey the native loader. Missing discovery refuses before any node call.
        let result = unsafe { CheckedNodeLibrary::load(path, 0) };
        assert!(matches!(result, Err(AbiError::Bootstrap { .. })));
    }
}
