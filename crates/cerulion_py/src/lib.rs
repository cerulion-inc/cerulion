// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion._native` - the PyO3 extension module behind the `cerulion`
//! Python facade.
//!
//! Raw zero-copy Cerulion client: publishers loan SHM slots and stamp
//! wire headers themselves; subscribers hand out owned frames whose
//! buffer-protocol views pin the SHM slot until released.

mod errors;

pub(crate) use errors::{
    BorrowLimitExceeded, CerulionError, DecodeError, EncodeError, ReleasedFrame, SchemaError,
    SchemaMismatch, TransportError as PyTransportError,
};

mod frame;
mod publisher;
mod session;
mod subscriber;
mod typed;

use cerulion_core::WireHeader;
use pyo3::prelude::*;

/// Native module `cerulion._native`.
#[pymodule(name = "_native")]
fn native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("CerulionError", m.py().get_type::<CerulionError>())?;
    m.add("SchemaMismatch", m.py().get_type::<SchemaMismatch>())?;
    m.add(
        "BorrowLimitExceeded",
        m.py().get_type::<BorrowLimitExceeded>(),
    )?;
    m.add("ReleasedFrame", m.py().get_type::<ReleasedFrame>())?;
    m.add("TransportError", m.py().get_type::<PyTransportError>())?;
    m.add("EncodeError", m.py().get_type::<EncodeError>())?;
    m.add("DecodeError", m.py().get_type::<DecodeError>())?;
    m.add("SchemaError", m.py().get_type::<SchemaError>())?;
    m.add("WIRE_HEADER_SIZE", WireHeader::SIZE)?;
    m.add_function(wrap_pyfunction!(session::connect, m)?)?;
    m.add_function(wrap_pyfunction!(session::real_ns, m)?)?;
    m.add_class::<publisher::Publisher>()?;
    m.add_class::<publisher::Loan>()?;
    m.add_class::<subscriber::Subscriber>()?;
    m.add_class::<frame::Frame>()?;
    m.add_class::<typed::PySchemaSet>()?;
    Ok(())
}
