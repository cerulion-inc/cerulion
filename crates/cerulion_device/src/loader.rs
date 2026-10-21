// SPDX-License-Identifier: AGPL-3.0-only
//! Opening vendor libraries at run time.
//!
//! Nothing in this crate links a vendor library. Each backend names the
//! library files it tries and the symbols it needs; a missing library or
//! symbol is an error value, so the same binary runs on a machine without
//! the vendor stack and reports that backend as unavailable.

use crate::backend::BackendKind;
use crate::error::{DeviceError, DeviceResult};

/// An opened vendor library. A backend owns the `Library` its function
/// pointers came from and drops it last, so the library is closed only after
/// the backend is gone.
#[derive(Debug)]
pub(crate) struct Library {
    backend: BackendKind,
    name: &'static str,
    lib: libloading::Library,
}

impl Library {
    /// Open the first of `names` that loads.
    pub(crate) fn open(backend: BackendKind, names: &[&'static str]) -> DeviceResult<Self> {
        let mut reason = String::from("no library names given");
        for &name in names {
            // SAFETY: loading a vendor driver runs its initializers; these
            // are the vendors' own shipped libraries, loaded the way their
            // runtimes load them.
            match unsafe { libloading::Library::new(name) } {
                Ok(lib) => {
                    return Ok(Self { backend, name, lib });
                }
                Err(e) => reason = e.to_string(),
            }
        }
        Err(DeviceError::LibraryNotFound {
            backend,
            tried: names.to_vec(),
            reason,
        })
    }

    /// Read symbol `symbol` as a value of type `T` (a function pointer).
    ///
    /// # Safety
    ///
    /// `T` must be the symbol's real type.
    pub(crate) unsafe fn get<T: Copy>(&self, symbol: &'static str) -> DeviceResult<T> {
        // SAFETY: the caller states `T` is the symbol's type; the library is
        // kept open for the life of `self`, and the backend that copies the
        // pointer out owns `self`.
        unsafe { self.lib.get::<T>(symbol.as_bytes()) }
            .map(|sym| *sym)
            .map_err(|_| DeviceError::SymbolMissing {
                backend: self.backend,
                library: self.name,
                symbol,
            })
    }

    /// The file name that loaded.
    pub(crate) fn name(&self) -> &'static str {
        self.name
    }
}

/// Declare a struct of vendor function pointers and its loader.
///
/// Each row is `field: fn(args) -> ret = "symbol";`. `load` reads every
/// symbol, failing on the first one the library lacks, and `SYMBOLS` lists
/// them for the symbol-table tests.
macro_rules! fn_table {
    ($(#[$meta:meta])* $name:ident {
        $( $field:ident : fn($($arg:ty),* $(,)?) -> $ret:ty = $sym:literal; )*
    }) => {
        $(#[$meta])*
        #[derive(Clone, Copy)]
        pub(crate) struct $name {
            $( pub(crate) $field: unsafe extern "C" fn($($arg),*) -> $ret, )*
        }

        impl $name {
            /// Every symbol the table reads, in declaration order.
            #[cfg(test)]
            pub(crate) const SYMBOLS: &'static [&'static str] = &[$($sym),*];

            /// Read every symbol from `lib`.
            ///
            /// # Safety
            ///
            /// Each row's signature must match the vendor's declaration.
            pub(crate) unsafe fn load(lib: &$crate::loader::Library) -> $crate::error::DeviceResult<Self> {
                // SAFETY: forwarded to the caller: the rows match the vendor headers.
                Ok(Self { $( $field: unsafe { lib.get($sym)? }, )* })
            }
        }

        impl std::fmt::Debug for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(stringify!($name))
            }
        }
    };
}
pub(crate) use fn_table;
