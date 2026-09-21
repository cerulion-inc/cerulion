//! Follow-up: prove that a pathological
//! hand-written `impl ShmMessage` with `WIRE_FIXED_SIZE = usize::MAX`
//! fails to compile when `_SHM_INVARIANTS` is forced to evaluate.
//!
//! The `_SHM_INVARIANTS` trait const carries `assert!(header + fixed
//! + table <= u32::MAX as usize)`. Codegen-emitted impls trivially
//! satisfy this; hand-written impls might not. The trait const is
//! referenced from production code at every `loan_proxy<T>` site
//! AND at `TransportManager::create_publisher_typed_with_history`,
//! via `let _: () = <T as ShmMessage>::_SHM_INVARIANTS;` — this is
//! what forces monomorphization-time evaluation for production
//! types like `sensor_msgs::Image`.
//!
//! THIS TRYBUILD TEST forces evaluation at MODULE LOAD via its own
//! `const _: () = <Pathological as ShmMessage>::_SHM_INVARIANTS;`,
//! decoupled from the production references. The test catches a
//! specific class of regression: **someone weakening or removing
//! the `assert!` inside the trait const itself** (e.g. relaxing the
//! bound, replacing with `debug_assert!`, or stubbing it out). It
//! does NOT catch elision of the production reference sites —
//! that's a separate threat model best caught by either (a) a
//! comment in `loan_proxy` documenting the reference as load-bearing,
//! or (b) an integration test that monomorphizes
//! `loan_proxy::<PathologicalT>` at a use site.

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;

/// Hand-written pathological schema: WIRE_FIXED_SIZE at the usize
/// ceiling. The sum `WireHeader::SIZE + WIRE_FIXED_SIZE` overflows
/// u32 by orders of magnitude; `_SHM_INVARIANTS` must reject this
/// at const-eval.
struct Pathological;

impl ShmMessage for Pathological {
    type Reader<'a> = &'a [u8];
    type Writer<'a> = &'a mut [u8];
    const WIRE_FIXED_SIZE: usize = usize::MAX;
    const VARIABLE_FIELD_COUNT: usize = 0;
    const MAX_SLICE_LEN: Option<MaxSliceLen> = None;
    const SCHEMA_HASH: u64 = 0;
    fn build_reader(bytes: &[u8]) -> Self::Reader<'_> {
        bytes
    }
    fn build_writer<'a>(bytes: &'a mut [u8], _max_capacity: cerulion_core::wire::MaxPayloadCapacity, _topic: std::sync::Arc<str>) -> Self::Writer<'a> {
        bytes
    }
    fn payload_wire_size(_writer: &Self::Writer<'_>) -> usize {
        0
    }
    fn all_variables_written(_writer: &Self::Writer<'_>) -> bool {
        true
    }
}

// Force `_SHM_INVARIANTS` const-evaluation at module-load time.
// This MUST fail to compile.
const _: () = <Pathological as ShmMessage>::_SHM_INVARIANTS;

fn main() {}
