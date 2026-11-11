// SPDX-License-Identifier: AGPL-3.0-only
//! Native host-table ownership and delivery oracles. Each test builds one
//! isolated iceoryx2 manager and passes only C records through HOST_API.
//! Run this binary alone with --test-threads=1 under the build desk lock.
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::mem::MaybeUninit;
use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, ShutdownSignal};
use cerulion_core::graph::node_host_abi::{
    register_context, remove_context, with_context, HostContext, HostPortSpec, HOST_API,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::node_abi as abi;
use cerulion_core::testing::{iceoryx_test_config, upstream_2035_publish_allocs};
use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, StepTag, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

thread_local! { static ALLOCS: Cell<Option<u64>> = const { Cell::new(None) }; }
struct Allocator;
// SAFETY: every allocation/deallocation is forwarded unchanged to System.
// Thread-local observation counts operations and owns no allocation itself.
unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
        // SAFETY: GlobalAlloc caller supplies the allocation layout.
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: the pointer and original layout are forwarded to its owner.
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: Allocator = Allocator;
struct CountAllocations;
impl CountAllocations {
    fn begin() -> Self {
        ALLOCS.with(|count| {
            assert_eq!(count.replace(Some(0)), None);
        });
        Self
    }
    fn finish(self) -> u64 {
        ALLOCS.with(|count| count.replace(None).unwrap())
    }
}
impl Drop for CountAllocations {
    fn drop(&mut self) {
        ALLOCS.with(|count| count.set(None));
    }
}

macro_rules! abi_call {
    ($call:expr) => {{
        // SAFETY: tests keep argument records/buffers live and nonoverlapping
        // for each synchronous call. Deliberately malformed arguments are
        // null, misaligned, or oversized spans rejected before dereference.
        unsafe { $call }
    }};
}
fn bytes(bytes: &[u8]) -> abi::Bytes {
    abi::Bytes {
        ptr: bytes.as_ptr(),
        len: bytes.len() as u64,
    }
}
fn mutable(bytes: &mut [u8]) -> abi::MutableBytes {
    abi::MutableBytes {
        ptr: bytes.as_mut_ptr(),
        len: bytes.len() as u64,
    }
}
fn vector_layout() -> PluginWireLayout {
    PluginWireLayout {
        schema_hash: Vector3::SCHEMA_HASH,
        fixed_size: Vector3::WIRE_FIXED_SIZE as u32,
        variable_fields: Vector3::VARIABLE_FIELD_COUNT as u32,
        alignment: cerulion_core::shm_runtime::FRAME_ALIGN as u32,
    }
}
fn request(layout: PluginWireLayout) -> abi::LoanRequestV1 {
    abi::LoanRequestV1 {
        schema_hash: layout.schema_hash,
        minimum_capacity: layout.prefix_len().unwrap(),
        wire_fixed_size: layout.fixed_size,
        variable_count: layout.variable_fields,
        alignment: layout.alignment,
    }
}
struct Harness {
    context: abi::ContextHandle,
    input: abi::PortHandle,
    output: abi::PortHandle,
    clock: Arc<VirtualClock>,
    layout: PluginWireLayout,
    _manager: Arc<TransportManager>,
}
impl Harness {
    fn new(layout: PluginWireLayout, cap: Option<usize>, budget: usize) -> Self {
        let clock = Arc::new(VirtualClock::new());
        clock.set(777);
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: "node_host_abi_oracle".into(),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            iceoryx_test_config(),
        )
        .unwrap();
        let publisher = manager
            .create_publisher("abi/oracle", MaxSliceLen::try_new(4096).unwrap(), 0)
            .unwrap();
        let subscriber = manager.create_subscriber("abi/oracle").unwrap();
        let env = Arc::new(HashMap::from([(
            "ABI_ORACLE".to_string(),
            "frozen".to_string(),
        )]));
        let native = NodeContext::with_runtime_env(
            IndexMap::from([("out".to_string(), AnyPublisher::Ipc(publisher))]),
            IndexMap::from([("in".to_string(), AnySubscriber::Ipc(subscriber))]),
            clock.clone(),
            ShutdownSignal::new(),
            env.clone(),
        );
        // Outputs precede inputs here: binding ordinals are direction-scoped.
        let host = HostContext::new(
            native,
            vec![
                HostPortSpec {
                    name: "out".into(),
                    direction: LeaseDirection::Write,
                    layout,
                    read_mode: 0,
                    accumulate_cap: None,
                },
                HostPortSpec {
                    name: "in".into(),
                    direction: LeaseDirection::Read,
                    layout,
                    read_mode: abi::read_mode::ONE,
                    accumulate_cap: cap,
                },
            ],
            env,
            budget,
        )
        .unwrap();
        let context = abi::ContextHandle {
            value: register_context(host).unwrap(),
        };
        let mut binding = MaybeUninit::<abi::PortBindingV1>::uninit();
        assert_eq!(
            abi_call!((HOST_API.bind_port)(context, 0, 1, binding.as_mut_ptr())).code,
            0
        );
        // SAFETY: successful callback initialized the complete record.
        let output = unsafe { binding.assume_init() }.port;
        assert_eq!(
            abi_call!((HOST_API.bind_port)(context, 0, 0, binding.as_mut_ptr())).code,
            0
        );
        // SAFETY: successful callback initialized the complete record.
        let input = unsafe { binding.assume_init() }.port;
        Self {
            context,
            input,
            output,
            clock,
            layout,
            _manager: manager,
        }
    }
    fn loan(&self) -> abi::WriteSlotV1 {
        let mut out = MaybeUninit::uninit();
        assert_eq!(
            abi_call!((HOST_API.loan_output)(
                self.context,
                self.output,
                request(self.layout),
                out.as_mut_ptr()
            ))
            .code,
            0
        );
        // SAFETY: OK initialized out; host retains the native loan.
        unsafe { out.assume_init() }
    }
    fn receive(&self) -> abi::ReadSlotV1 {
        let mut out = MaybeUninit::uninit();
        assert_eq!(
            abi_call!((HOST_API.receive_input)(
                self.context,
                self.input,
                abi::read_mode::ONE,
                out.as_mut_ptr()
            ))
            .code,
            0
        );
        // SAFETY: OK initialized out; host retains the native sample.
        unsafe { out.assume_init() }
    }
    fn publish(&self, values: [f64; 3]) {
        let slot = self.loan();
        write_values(&slot, values);
        assert_eq!(
            abi_call!((HOST_API.finish_output)(self.context, slot.lease, 64)).code,
            0
        );
    }
    fn release(&self, slot: abi::ReadSlotV1) {
        assert_eq!(
            abi_call!((HOST_API.release_input)(self.context, slot.lease)).code,
            0
        );
    }
    fn discard(&self, slot: abi::WriteSlotV1, reason: u32) {
        assert_eq!(
            abi_call!((HOST_API.discard_output)(self.context, slot.lease, reason)).code,
            0
        );
    }
    fn empty(&self) {
        let mut out = MaybeUninit::<abi::ReadSlotV1>::uninit();
        assert_eq!(
            abi_call!((HOST_API.receive_input)(
                self.context,
                self.input,
                abi::read_mode::ONE,
                out.as_mut_ptr()
            ))
            .code,
            1
        );
    }
}
impl Drop for Harness {
    fn drop(&mut self) {
        remove_context(self.context.value).unwrap();
    }
}
fn write_values(slot: &abi::WriteSlotV1, values: [f64; 3]) {
    assert!(slot.capacity >= 64);
    // SAFETY: test calls only for a live exclusive write lease, and only writes
    // initialized payload bytes within the returned native mapping capacity.
    let frame = unsafe { std::slice::from_raw_parts_mut(slot.data, slot.capacity as usize) };
    for (offset, value) in [40, 48, 56].into_iter().zip(values) {
        frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
fn inspect(slot: &abi::ReadSlotV1, action: impl FnOnce(&[u8])) {
    // SAFETY: callers retain this immutable lease until action returns; size
    // was validated by the host and this helper never saves the pointer.
    action(unsafe { std::slice::from_raw_parts(slot.data, slot.size as usize) });
}
fn oracle_frame(sequence: u32, stamp: u64, values: [f64; 3]) -> [u8; 64] {
    let mut expected = [0; 64];
    expected[..8].copy_from_slice(&Vector3::SCHEMA_HASH.to_le_bytes());
    expected[8..12].copy_from_slice(&64u32.to_le_bytes());
    expected[12..16].copy_from_slice(&64u32.to_le_bytes());
    expected[20..24].copy_from_slice(&sequence.to_le_bytes());
    expected[24..32].copy_from_slice(&stamp.to_le_bytes());
    for (offset, value) in [40, 48, 56].into_iter().zip(values) {
        expected[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    expected
}

#[test]
fn callback_commit_delivers_handwritten_wire_oracle_in_two_independent_runs() {
    for _ in 0..2 {
        let h = Harness::new(vector_layout(), None, 2);
        let slot = h.loan();
        write_values(&slot, [1.25, -2.5, 9.0]);
        assert_eq!(slot.schema_hash, Vector3::SCHEMA_HASH);
        assert_eq!(slot.capacity, 64);
        assert_eq!(slot.configured_capacity, 4096);
        assert_eq!(slot.prefix_size, 64);
        h.clock.set(999); // Commit retains the loan stamp, not the later clock.
        assert_eq!(
            abi_call!((HOST_API.finish_output)(h.context, slot.lease, 64)).code,
            0
        );
        let read = h.receive();
        inspect(&read, |frame| {
            assert_eq!(frame, oracle_frame(0, 777, [1.25, -2.5, 9.0]))
        });
        h.release(read);
        h.empty();
        assert_eq!(
            abi_call!((HOST_API.finish_output)(h.context, slot.lease, 64)).code,
            4
        );
        let counts = with_context(h.context.value, |host| Ok(host.counters())).unwrap();
        assert_eq!(
            (counts.committed, counts.released, counts.discarded),
            (1, 1, 0)
        );
    }
}
#[test]
fn output_discard_does_not_deliver_or_consume_sequence() {
    let h = Harness::new(vector_layout(), None, 2);
    for reason in [
        abi::discard_reason::TICK_FAILED,
        abi::discard_reason::TICK_PANICKED,
        abi::discard_reason::MISSING_VARIABLE,
        abi::discard_reason::NESTED_FAILED,
        abi::discard_reason::REPLAY_SUPPRESSED,
        abi::discard_reason::INVARIANT,
    ] {
        let slot = h.loan();
        write_values(&slot, [88.0, 99.0, 100.0]);
        h.discard(slot, reason);
        h.empty();
        assert_eq!(
            abi_call!((HOST_API.discard_output)(h.context, slot.lease, reason)).code,
            4
        );
    }
    h.publish([3.0, 4.0, 5.0]);
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [3.0, 4.0, 5.0]))
    });
    h.release(read);
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.counters().discarded)),
        Ok(5)
    );
}
#[test]
fn budget_and_foreign_handles_leave_pending_delivery_intact() {
    let h = Harness::new(vector_layout(), None, 1);
    let foreign = Harness::new(vector_layout(), None, 1);
    let loan = h.loan();
    write_values(&loan, [6.0, 7.0, 8.0]);
    assert_eq!(
        abi_call!((HOST_API.discard_output)(
            foreign.context,
            loan.lease,
            abi::discard_reason::TICK_FAILED
        ))
        .code,
        4
    );
    let mut out = MaybeUninit::<abi::WriteSlotV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            h.output,
            request(h.layout),
            out.as_mut_ptr()
        ))
        .code,
        6
    );
    assert_eq!(
        abi_call!((HOST_API.finish_output)(h.context, loan.lease, 64)).code,
        0
    );
    let read = h.receive();
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            h.output,
            request(h.layout),
            out.as_mut_ptr()
        ))
        .code,
        6
    );
    assert_eq!(
        abi_call!((HOST_API.release_input)(foreign.context, read.lease)).code,
        4
    );
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [6.0, 7.0, 8.0]))
    });
    h.release(read);
    h.publish([10.0, 11.0, 12.0]);
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(1, 777, [10.0, 11.0, 12.0]))
    });
    h.release(read);
}
#[test]
fn malformed_descriptor_and_output_pointer_are_rejected_before_loan() {
    let h = Harness::new(vector_layout(), None, 1);
    let mut out = MaybeUninit::<abi::WriteSlotV1>::uninit();
    let mut bad = request(h.layout);
    bad.schema_hash ^= 1;
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            h.output,
            bad,
            out.as_mut_ptr()
        ))
        .code,
        5
    );
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            h.output,
            request(h.layout),
            std::ptr::null_mut()
        ))
        .code,
        5
    );
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(0)
    );
    h.publish([1.0, 2.0, 3.0]);
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [1.0, 2.0, 3.0]))
    });
    h.release(read);
}
#[test]
fn read_mode_and_borrow_exclusion_preserve_the_native_frame() {
    let h = Harness::new(vector_layout(), None, 2);
    h.publish([15.0, 16.0, 17.0]);
    let mut out = MaybeUninit::<abi::ReadSlotV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.receive_input)(
            h.context,
            h.input,
            abi::read_mode::LATEST,
            out.as_mut_ptr()
        ))
        .code,
        5
    );
    let read = h.receive();
    assert_eq!(
        abi_call!((HOST_API.receive_input)(
            h.context,
            h.input,
            abi::read_mode::ONE,
            out.as_mut_ptr()
        ))
        .code,
        6
    );
    let mut head = MaybeUninit::<abi::HeadOutcomeV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.input_head)(
            h.context,
            h.input,
            abi::head_op::DRAIN,
            head.as_mut_ptr()
        ))
        .code,
        6
    );
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [15.0, 16.0, 17.0]))
    });
    h.release(read);
    assert_eq!(
        abi_call!((HOST_API.release_input)(h.context, read.lease)).code,
        4
    );
}
#[test]
fn native_callback_publish_and_receive_add_no_heap_allocation() {
    let h = Harness::new(vector_layout(), None, 2);
    h.publish([1.0, 2.0, 3.0]);
    let warm = h.receive();
    h.release(warm);
    let count = CountAllocations::begin();
    for sequence in 1..=8 {
        h.publish([21.0, 22.0, 23.0]);
        let read = h.receive();
        inspect(&read, |frame| {
            assert_eq!(frame, oracle_frame(sequence, 777, [21.0, 22.0, 23.0]))
        });
        h.release(read);
    }
    let actual = count.finish();
    assert_eq!(actual.checked_sub(upstream_2035_publish_allocs(8)), Some(0));
    // Allocation detector control: copying a delivered frame onto the heap
    // must be visible, so a copied adapter cannot pass the zero-allocation gate.
    h.publish([21.0, 22.0, 23.0]);
    let read = h.receive();
    let count = CountAllocations::begin();
    let mut copied = Vec::new();
    inspect(&read, |frame| copied.extend_from_slice(frame));
    assert!(count.finish() > 0);
    assert_eq!(copied, oracle_frame(9, 777, [21.0, 22.0, 23.0]));
    h.release(read);
}
#[test]
fn spill_retains_original_mapping_on_success_and_refusal_until_release() {
    type Text = native_ros2_messages::std_msgs::String;
    let layout = PluginWireLayout {
        schema_hash: Text::SCHEMA_HASH,
        fixed_size: Text::WIRE_FIXED_SIZE as u32,
        variable_fields: Text::VARIABLE_FIELD_COUNT as u32,
        alignment: cerulion_core::shm_runtime::FRAME_ALIGN as u32,
    };
    let h = Harness::new(layout, None, 2);
    let warmup = cerulion_core::transport::adaptive_sizer::WINDOW_SIZE as u32;
    for sequence in 0..warmup {
        let small = h.loan();
        assert_eq!(
            abi_call!((HOST_API.finish_output)(h.context, small.lease, 48)).code,
            0
        );
        let read = h.receive();
        inspect(&read, |frame| {
            let header = WireHeader::read_from_buf(frame).unwrap();
            assert_eq!(
                (header.schema_hash, header.total_size, header.sequence),
                (Text::SCHEMA_HASH, 48, sequence)
            );
            assert_eq!(&frame[40..], &[0; 8]);
        });
        h.release(read);
    }
    let loan = h.loan();
    assert!(loan.capacity < 3040);
    let mut payload = vec![0; 3000];
    payload[0..4].copy_from_slice(&8u32.to_le_bytes());
    payload[4..8].copy_from_slice(&2992u32.to_le_bytes());
    payload[8..].fill(0x5a);
    h.clock.set(888);
    assert_eq!(
        abi_call!((HOST_API.submit_spill)(
            h.context,
            loan.lease,
            bytes(&payload),
            3040
        ))
        .code,
        0
    );
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(1)
    );
    // SAFETY: submission explicitly retains the original native loan; only
    // its initialized prefix is inspected before SPILL_RELEASE.
    let original = unsafe { std::slice::from_raw_parts(loan.data, loan.prefix_size as usize) };
    assert_eq!(
        WireHeader::read_from_buf(original).unwrap().timestamp_ns,
        777
    );
    assert_eq!(
        abi_call!((HOST_API.submit_spill)(
            h.context,
            loan.lease,
            bytes(&payload),
            3040
        ))
        .code,
        5
    );
    h.discard(loan, abi::discard_reason::SPILL_RELEASE);
    let read = h.receive();
    inspect(&read, |frame| {
        let header = WireHeader::read_from_buf(frame).unwrap();
        assert_eq!(
            (
                header.total_size,
                header.sequence,
                header.timestamp_ns,
                header.step_tag
            ),
            (3040, warmup, 777, StepTag::UNTAGGED)
        );
        assert_eq!(&frame[40..], payload);
    });
    h.release(read);
    let refused = h.loan();
    payload[0..4].copy_from_slice(&u32::MAX.to_le_bytes());
    assert_eq!(
        abi_call!((HOST_API.submit_spill)(
            h.context,
            refused.lease,
            bytes(&payload),
            3040
        ))
        .code,
        5
    );
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(1)
    );
    h.discard(refused, abi::discard_reason::SPILL_RELEASE);
    h.empty();
    let counters = with_context(h.context.value, |host| Ok(host.counters())).unwrap();
    assert_eq!(
        (counters.committed, counters.spills, counters.discarded),
        (u64::from(warmup) + 1, 1, 0)
    );
}

unsafe extern "C" fn capture_write(context: *mut u8, data: abi::Bytes) -> abi::Status {
    // SAFETY: capture() passes a live, exclusive BoundedSink for this call;
    // host supplies initialized encoder bytes valid until this callback returns.
    let sink = unsafe { &mut *context.cast::<cerulion_core::state::BoundedSink<'_>>() };
    // SAFETY: data is a synchronous borrow of the native state encoder slice.
    let data = unsafe { std::slice::from_raw_parts(data.ptr, data.len as usize) };
    use cerulion_core::state::StateSink;
    abi::Status {
        error: 0,
        kind: 0,
        code: if sink.write(data).is_ok() { 0 } else { 8 },
    }
}
fn capture(h: &Harness, storage: &mut [u8]) -> (abi::Status, usize) {
    let mut sink = cerulion_core::state::BoundedSink::new(storage);
    let record = abi::StateSinkV1 {
        context: std::ptr::from_mut(&mut sink).cast(),
        write: capture_write,
    };
    let status = abi_call!((HOST_API.capture_accumulate)(h.context, h.input, &record));
    (status, sink.len())
}
fn accumulated_info(h: &Harness) -> abi::AccumulateInfoV1 {
    let mut out = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.accumulate_count)(
            h.context,
            h.input,
            out.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful callback initialized the record.
    unsafe { out.assume_init() }
}
fn accumulated_frame(h: &Harness, index: u32) -> abi::ReadSlotV1 {
    let mut out = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.accumulate_frame)(
            h.context,
            h.input,
            index,
            out.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful callback initialized the record and retains the frame.
    unsafe { out.assume_init() }
}
fn begin_fire(h: &Harness) {
    with_context(h.context.value, |host| host.begin_fire()).unwrap();
}
fn collect(h: &Harness) {
    assert_eq!(
        abi_call!((HOST_API.set_snapshot_inputs)(
            h.context,
            bytes(br#"["in"]"#)
        ))
        .code,
        0
    );
    assert_eq!(abi_call!((HOST_API.snapshot_inputs)(h.context)).code, 0);
}
fn canonical_state(frames: &[[u8; 64]], total: u64, pending: u64) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&2u32.to_le_bytes());
    bytes.extend_from_slice(b"in");
    bytes.extend_from_slice(&total.to_le_bytes());
    bytes.extend_from_slice(&pending.to_le_bytes());
    bytes.extend_from_slice(&(frames.len() as u32).to_le_bytes());
    for frame in frames {
        bytes.extend_from_slice(&64u32.to_le_bytes());
        bytes.extend_from_slice(frame);
    }
    bytes
}
#[test]
fn accumulator_repeat_views_and_state_use_native_order_and_exact_bytes() {
    for _ in 0..2 {
        let h = Harness::new(vector_layout(), Some(2), 4);
        h.publish([1.0, 2.0, 3.0]);
        h.publish([4.0, 5.0, 6.0]);
        collect(&h);
        let mut storage = [0; 256];
        let (status, len) = capture(&h, &mut storage);
        assert_eq!(status.code, 0);
        assert_eq!(
            &storage[..len],
            canonical_state(
                &[
                    oracle_frame(0, 777, [1.0, 2.0, 3.0]),
                    oracle_frame(1, 777, [4.0, 5.0, 6.0])
                ],
                0,
                0
            )
        );
        begin_fire(&h);
        let info = accumulated_info(&h);
        assert_eq!((info.count, info.dropped, info.dropped_total), (2, 0, 0));
        for _ in 0..2 {
            let first = accumulated_frame(&h, 0);
            let second = accumulated_frame(&h, 1);
            assert_eq!(accumulated_info(&h).count, 2);
            assert_eq!(abi_call!((HOST_API.snapshot_inputs)(h.context)).code, 6);
            for slot in [&first, &second] {
                assert_eq!(
                    abi_call!((HOST_API.mark_served)(h.context, slot.lease)).code,
                    0
                );
                assert_eq!(
                    abi_call!((HOST_API.mark_served)(h.context, slot.lease)).code,
                    0
                );
            }
            inspect(&first, |frame| {
                assert_eq!(frame, oracle_frame(0, 777, [1.0, 2.0, 3.0]))
            });
            inspect(&second, |frame| {
                assert_eq!(frame, oracle_frame(1, 777, [4.0, 5.0, 6.0]))
            });
            h.release(first);
            h.release(second);
        }
        let (status, len) = capture(&h, &mut storage);
        assert_eq!(status.code, 0);
        assert_eq!(&storage[..len], canonical_state(&[], 0, 0));
        begin_fire(&h);
        let info = accumulated_info(&h);
        assert_eq!((info.count, info.dropped), (0, 0));
    }
}
#[test]
fn accumulator_restore_refusals_preserve_frames_and_counters() {
    let h = Harness::new(vector_layout(), Some(1), 2);
    h.publish([10.0, 20.0, 30.0]);
    collect(&h);
    let baseline = canonical_state(&[oracle_frame(0, 777, [10.0, 20.0, 30.0])], 0, 0);
    let mut wrong_name = baseline.clone();
    wrong_name[4] = b'x';
    let mut wrong_counters = baseline.clone();
    wrong_counters[14..22].copy_from_slice(&1u64.to_le_bytes());
    let mut wrong_schema = baseline.clone();
    wrong_schema[30] ^= 1;
    let mut trailing = baseline.clone();
    trailing.push(0);
    let over_capacity = canonical_state(
        &[
            oracle_frame(0, 777, [1.0, 2.0, 3.0]),
            oracle_frame(1, 777, [4.0, 5.0, 6.0]),
        ],
        0,
        0,
    );
    let truncated = baseline[..baseline.len() - 1].to_vec();
    for refused in [
        wrong_name,
        wrong_counters,
        wrong_schema,
        trailing,
        over_capacity,
        truncated,
    ] {
        assert_eq!(
            abi_call!((HOST_API.restore_accumulate)(
                h.context,
                h.input,
                bytes(&refused)
            ))
            .code,
            2
        );
        let mut storage = [0; 256];
        let (status, len) = capture(&h, &mut storage);
        assert_eq!(status.code, 0);
        assert_eq!(&storage[..len], baseline);
    }
    let restored = canonical_state(&[oracle_frame(7, 500, [40.0, 50.0, 60.0])], 9, 3);
    assert_eq!(
        abi_call!((HOST_API.restore_accumulate)(
            h.context,
            h.input,
            bytes(&restored)
        ))
        .code,
        0
    );
    begin_fire(&h);
    let info = accumulated_info(&h);
    assert_eq!((info.count, info.dropped, info.dropped_total), (1, 3, 9));
    let read = accumulated_frame(&h, 0);
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(7, 500, [40.0, 50.0, 60.0]))
    });
    h.release(read);
}
#[test]
fn accumulator_sink_refusal_does_not_consume_pending_state() {
    let h = Harness::new(vector_layout(), Some(1), 2);
    h.publish([3.0, 2.0, 1.0]);
    collect(&h);
    let mut short = [0; 4];
    assert_eq!(capture(&h, &mut short).0.code, 8);
    let mut complete = [0; 256];
    let (status, len) = capture(&h, &mut complete);
    assert_eq!(status.code, 0);
    assert_eq!(
        &complete[..len],
        canonical_state(&[oracle_frame(0, 777, [3.0, 2.0, 1.0])], 0, 0)
    );
}
#[test]
fn frozen_environment_and_diagnostics_support_complete_size_retry() {
    let h = Harness::new(vector_layout(), None, 2);
    let mut short = [0x5a; 2];
    let mut required = 99;
    assert_eq!(
        abi_call!((HOST_API.environment)(
            h.context,
            bytes(b"ABI_ORACLE"),
            mutable(&mut short),
            &mut required
        ))
        .code,
        8
    );
    assert_eq!(required, 6);
    assert_eq!(short, [0x5a; 2]);
    let mut full = [0; 6];
    assert_eq!(
        abi_call!((HOST_API.environment)(
            h.context,
            bytes(b"ABI_ORACLE"),
            mutable(&mut full),
            &mut required
        ))
        .code,
        0
    );
    assert_eq!(&full, b"frozen");
    assert_eq!(
        abi_call!((HOST_API.environment)(
            h.context,
            bytes(b"ABSENT"),
            mutable(&mut full),
            &mut required
        ))
        .code,
        1
    );
    assert_eq!(required, 0);
    let mut loan = MaybeUninit::<abi::WriteSlotV1>::uninit();
    let mut bad = request(h.layout);
    bad.alignment = 3;
    let failure = abi_call!((HOST_API.loan_output)(
        h.context,
        h.output,
        bad,
        loan.as_mut_ptr()
    ));
    assert_eq!(failure.code, 5);
    assert_ne!(failure.error, 0);
    assert_eq!(
        abi_call!((HOST_API.copy_error)(
            h.context,
            failure.error,
            mutable(&mut short),
            &mut required
        ))
        .code,
        8
    );
    assert_eq!(required, 17);
    assert_eq!(short, [0x5a; 2]);
    let mut text = [0; 17];
    assert_eq!(
        abi_call!((HOST_API.copy_error)(
            h.context,
            failure.error,
            mutable(&mut text),
            &mut required
        ))
        .code,
        0
    );
    assert_eq!(&text, b"InvalidDescriptor");
    assert_eq!(
        abi_call!((HOST_API.copy_error)(
            h.context,
            failure.error + 1,
            mutable(&mut text),
            &mut required
        ))
        .code,
        4
    );
}
#[test]
fn diagnostic_and_empty_event_calls_validate_all_scalar_and_pointer_arms() {
    let h = Harness::new(vector_layout(), None, 2);
    for port in [h.input, h.output, abi::PortHandle { value: 0 }] {
        for level in 1..=5 {
            assert_eq!(
                abi_call!((HOST_API.diagnostic)(
                    h.context,
                    port,
                    level,
                    bytes(b"diagnostic oracle")
                ))
                .code,
                0
            );
        }
        for level in [0, 6, u32::MAX] {
            assert_eq!(
                abi_call!((HOST_API.diagnostic)(
                    h.context,
                    port,
                    level,
                    bytes(b"diagnostic oracle")
                ))
                .code,
                5
            );
        }
    }
    assert_eq!(
        abi_call!((HOST_API.diagnostic)(h.context, h.input, 1, bytes(&[0xff]))).code,
        5
    );
    let mut event = MaybeUninit::<abi::EventV1>::uninit();
    for kind in [
        abi::event_kind::BACKPRESSURE,
        abi::event_kind::EXPECT_WITHIN,
        abi::event_kind::LIVELINESS,
    ] {
        assert_eq!(
            abi_call!((HOST_API.poll_event)(
                h.context,
                h.input,
                kind,
                event.as_mut_ptr()
            ))
            .code,
            1
        );
    }
    assert_eq!(
        abi_call!((HOST_API.poll_event)(
            h.context,
            h.output,
            abi::event_kind::PROMISE_WITHIN,
            event.as_mut_ptr()
        ))
        .code,
        1
    );
    assert_eq!(
        abi_call!((HOST_API.poll_event)(
            h.context,
            h.input,
            u32::MAX,
            event.as_mut_ptr()
        ))
        .code,
        5
    );
    assert_eq!(
        abi_call!((HOST_API.poll_event)(
            h.context,
            h.input,
            abi::event_kind::BACKPRESSURE,
            std::ptr::null_mut()
        ))
        .code,
        5
    );
    let mut clock = MaybeUninit::<abi::ClockValueV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.clock)(
            h.context,
            abi::clock_source::EXTERNAL,
            clock.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful callback initializes the complete value.
    let absent = unsafe { clock.assume_init() };
    assert_eq!((absent.timestamp_ns, absent.present), (0, 0));
    assert_eq!(
        abi_call!((HOST_API.clock)(h.context, u32::MAX, clock.as_mut_ptr())).code,
        5
    );
    assert_eq!(
        abi_call!((HOST_API.clock)(
            h.context,
            abi::clock_source::ACTIVE,
            std::ptr::null_mut()
        ))
        .code,
        5
    );
    for port in [h.input, h.output, abi::PortHandle { value: 0 }] {
        assert_eq!(
            with_context(h.context.value, |host| host
                .diagnostic_failures(h.context.value, port.value)),
            Ok(2)
        );
    }
    let mut extension = MaybeUninit::<abi::ExtensionDescriptorV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.query_extension)(
            h.context,
            u64::MAX,
            1,
            extension.as_mut_ptr()
        ))
        .code,
        7
    );
    assert_eq!(
        abi_call!((HOST_API.query_extension)(
            h.context,
            u64::MAX,
            1,
            std::ptr::null_mut()
        ))
        .code,
        5
    );
    assert_eq!(abi_call!((HOST_API.request_shutdown)(h.context)).code, 0);
}

#[test]
fn malformed_normal_finish_retires_without_delivery_or_sequence_gap() {
    let h = Harness::new(vector_layout(), None, 2);
    for size in [0, 39, 63, 65, u32::MAX] {
        let loan = h.loan();
        write_values(&loan, [7.0, 8.0, 9.0]);
        assert_eq!(
            abi_call!((HOST_API.finish_output)(h.context, loan.lease, size)).code,
            5
        );
        assert_eq!(
            with_context(h.context.value, |host| Ok(host.outstanding_leases())),
            Ok(0)
        );
        assert_eq!(
            abi_call!((HOST_API.finish_output)(h.context, loan.lease, 64)).code,
            4
        );
        h.empty();
    }
    h.publish([20.0, 30.0, 40.0]);
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [20.0, 30.0, 40.0]))
    });
    h.release(read);
}
#[test]
fn accumulator_loss_event_preserves_native_observations_and_surviving_delivery() {
    let h = Harness::new(vector_layout(), Some(1), 2);
    h.publish([1.0, 2.0, 3.0]);
    h.publish([4.0, 5.0, 6.0]);
    h.publish([7.0, 8.0, 9.0]);
    collect(&h);
    let mut event = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.poll_event)(
            h.context,
            h.input,
            abi::event_kind::BACKPRESSURE,
            event.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful event callback initialized the record.
    let event: abi::EventV1 = unsafe { event.assume_init() };
    assert_eq!(
        (event.port, event.kind, event.detail),
        (
            h.input,
            abi::event_kind::BACKPRESSURE,
            abi::event_detail::DROP_OLDEST
        )
    );
    assert_eq!(
        (
            event.timestamp_ns,
            event.total,
            event.in_regime,
            event.dropped,
            event.capacity
        ),
        (777, 2, 2, 2, 1)
    );
    assert_eq!(
        (
            event.window_ms,
            event.elapsed_ms,
            event.policy_value,
            event.cause
        ),
        (0, 0, 0, 0)
    );
    begin_fire(&h);
    let info = accumulated_info(&h);
    assert_eq!((info.count, info.dropped, info.dropped_total), (1, 2, 2));
    let read = accumulated_frame(&h, 0);
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(2, 777, [7.0, 8.0, 9.0]))
    });
    h.release(read);
}

fn change_publisher(
    h: &Harness,
    action: impl FnOnce(&mut cerulion_core::transport::publisher::CerulionPublisher),
) {
    with_context(h.context.value, |host| {
        host.with_native(|native| {
            let AnyPublisher::Ipc(publisher) = native.publisher_mut("out").unwrap();
            action(publisher);
        })
    })
    .unwrap();
}
#[test]
fn native_loan_and_send_failures_retire_owners_and_preserve_delivery_accounting() {
    let h = Harness::new(vector_layout(), None, 2);
    change_publisher(&h, |publisher| publisher.force_discard_next_fire());
    let mut slot = MaybeUninit::<abi::WriteSlotV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            h.output,
            request(h.layout),
            slot.as_mut_ptr()
        ))
        .code,
        2
    );
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(0)
    );
    h.empty();
    change_publisher(&h, |publisher| publisher.fault_inject_send_raw_loan());
    let loan = h.loan();
    write_values(&loan, [1.0, 2.0, 3.0]);
    assert_eq!(
        abi_call!((HOST_API.finish_output)(h.context, loan.lease, 64)).code,
        2
    );
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(0)
    );
    h.empty();
    let mut counters = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.publisher_counters)(
            h.context,
            h.output,
            counters.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: OK initialized counters completely.
    let counters: abi::PublisherCountersV1 = unsafe { counters.assume_init() };
    assert_eq!((counters.committed, counters.send_failures), (1, 1));
    h.publish([9.0, 8.0, 7.0]);
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(1, 777, [9.0, 8.0, 7.0]))
    });
    h.release(read);
}
#[test]
fn failed_trigger_drain_is_explicit_and_cannot_vouch_for_an_empty_queue() {
    let h = Harness::new(vector_layout(), None, 2);
    h.publish([2.0, 4.0, 6.0]);
    with_context(h.context.value, |host| {
        host.with_native(|native| {
            let AnySubscriber::Ipc(subscriber) = native.subscriber_mut("in").unwrap();
            subscriber.fault_inject_receive_after_for_test(0);
        })
    })
    .unwrap();
    let mut head = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.input_head)(
            h.context,
            h.input,
            abi::head_op::DRAIN,
            head.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful callback initialized the complete head record.
    let failed: abi::HeadOutcomeV1 = unsafe { head.assume_init() };
    assert_eq!(failed.kind, abi::head_kind::FAILED);
    let mut read = MaybeUninit::<abi::ReadSlotV1>::uninit();
    assert_eq!(
        abi_call!((HOST_API.receive_input)(
            h.context,
            h.input,
            abi::read_mode::ONE,
            read.as_mut_ptr()
        ))
        .code,
        2
    );
    assert_eq!(
        abi_call!((HOST_API.input_head)(
            h.context,
            h.input,
            abi::head_op::DRAIN,
            head.as_mut_ptr()
        ))
        .code,
        0
    );
    let read = h.receive();
    inspect(&read, |frame| {
        assert_eq!(frame, oracle_frame(0, 777, [2.0, 4.0, 6.0]))
    });
    h.release(read);
}

struct RawHarness {
    context: abi::ContextHandle,
    manager: Arc<TransportManager>,
}
impl RawHarness {
    fn new(budget: usize) -> Self {
        let clock = Arc::new(VirtualClock::new());
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: "raw_host_oracle".into(),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            iceoryx_test_config(),
        )
        .unwrap();
        let env = Arc::new(HashMap::new());
        let mut native = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock,
            ShutdownSignal::new(),
            env.clone(),
        );
        native.set_transport(manager.clone());
        let host = HostContext::new(native, Vec::new(), env, 1)
            .unwrap()
            .with_raw_publisher_budget(budget)
            .unwrap();
        Self {
            context: abi::ContextHandle {
                value: register_context(host).unwrap(),
            },
            manager,
        }
    }
    fn create(&self, topic: &str) -> abi::PortBindingV1 {
        let mut binding = MaybeUninit::uninit();
        let status = abi_call!((HOST_API.create_raw_publisher)(
            self.context,
            raw_request(topic),
            binding.as_mut_ptr()
        ));
        assert_eq!(status.code, 0);
        // SAFETY: successful callback initialized the output record.
        unsafe { binding.assume_init() }
    }
    fn loan(&self, port: abi::PortHandle, size: u32) -> abi::WriteSlotV1 {
        let mut slot = MaybeUninit::uninit();
        assert_eq!(
            abi_call!((HOST_API.loan_output)(
                self.context,
                port,
                raw_loan_request(size),
                slot.as_mut_ptr()
            ))
            .code,
            0
        );
        // SAFETY: successful callback initialized the output record.
        unsafe { slot.assume_init() }
    }
}
impl Drop for RawHarness {
    fn drop(&mut self) {
        remove_context(self.context.value).unwrap();
    }
}
fn raw_request(topic: &str) -> abi::RawPublisherRequestV1 {
    abi::RawPublisherRequestV1 {
        topic: bytes(topic.as_bytes()),
        schema_hash: Vector3::SCHEMA_HASH,
        configured_capacity: 4096,
        full_queue: abi::full_queue::DISCARD,
    }
}
fn raw_loan_request(size: u32) -> abi::LoanRequestV1 {
    abi::LoanRequestV1 {
        schema_hash: Vector3::SCHEMA_HASH,
        minimum_capacity: size,
        wire_fixed_size: 0,
        variable_count: 0,
        alignment: cerulion_core::shm_runtime::FRAME_ALIGN as u32,
    }
}
fn fill_raw(slot: &abi::WriteSlotV1, frame: &[u8]) {
    assert_eq!(slot.capacity as usize, frame.len());
    // SAFETY: live exclusive native raw write lease covers exactly frame.len;
    // caller data is disjoint, and the complete frame is initialized before finish.
    unsafe { std::slice::from_raw_parts_mut(slot.data, frame.len()) }.copy_from_slice(frame);
}

#[test]
fn raw_ingress_preserves_original_wire_bytes_and_recipient_count_in_two_runs() {
    let mut expected = oracle_frame(91, 123_456, [1.25, -2.5, 9.0]);
    expected[32..40].copy_from_slice(&17u64.to_le_bytes());
    for _ in 0..2 {
        let h = RawHarness::new(2);
        let binding = h.create("abi/raw/delivery");
        let mut subscriber = h.manager.create_subscriber("abi/raw/delivery").unwrap();
        let slot = h.loan(binding.port, 64);
        assert_eq!(
            (slot.prefix_size, slot.capacity, slot.configured_capacity),
            (40, 64, 4096)
        );
        assert_eq!(
            abi_call!((HOST_API.destroy_raw_publisher)(h.context, binding.port)).code,
            abi::status_code::BUSY
        );
        fill_raw(&slot, &expected);
        let mut recipients = 999;
        assert_eq!(
            abi_call!((HOST_API.finish_raw_output)(
                h.context,
                slot.lease,
                64,
                &mut recipients
            ))
            .code,
            0
        );
        assert_eq!(recipients, 1);
        let delivered = subscriber
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap();
        assert_eq!(&delivered[..], &expected);
        drop(delivered);
        assert_eq!(
            abi_call!((HOST_API.finish_raw_output)(
                h.context,
                slot.lease,
                64,
                &mut recipients
            ))
            .code,
            abi::status_code::INVALID_HANDLE
        );
        assert_eq!(recipients, 0);
        assert!(subscriber.view_raw().unwrap().is_none());
        assert_eq!(
            abi_call!((HOST_API.register_raw_egress)(h.context, binding.port)).code,
            0
        );
        assert_eq!(
            h.manager
                .dynamic_egress_record_hash_for_test("abi/raw/delivery"),
            Some(Vector3::SCHEMA_HASH)
        );
        assert_eq!(
            abi_call!((HOST_API.destroy_raw_publisher)(h.context, binding.port)).code,
            0
        );
        assert_eq!(
            abi_call!((HOST_API.destroy_raw_publisher)(h.context, binding.port)).code,
            abi::status_code::INVALID_HANDLE
        );
        // SAFETY: the topic allocation is retained until context removal even after destroy.
        assert_eq!(
            unsafe { std::slice::from_raw_parts(binding.topic.ptr, binding.topic.len as usize) },
            b"abi/raw/delivery"
        );
    }
}

#[test]
fn raw_creation_refuses_reserved_tags_and_budget_reuse_before_native_creation() {
    let h = RawHarness::new(1);
    let mut out = MaybeUninit::uninit();
    for tag in [abi::full_queue::BLOCK, 3, u32::MAX] {
        let mut request = raw_request("abi/raw/refusal");
        request.full_queue = tag;
        assert_eq!(
            abi_call!((HOST_API.create_raw_publisher)(
                h.context,
                request,
                out.as_mut_ptr()
            ))
            .code,
            abi::status_code::INVALID_ARGUMENT
        );
    }
    for topic in ["", "abi/raw/refusal"] {
        let mut request = raw_request(topic);
        request.configured_capacity = 39;
        assert_eq!(
            abi_call!((HOST_API.create_raw_publisher)(
                h.context,
                request,
                out.as_mut_ptr()
            ))
            .code,
            abi::status_code::INVALID_ARGUMENT
        );
    }
    let mut request = raw_request("abi/raw/refusal");
    request.topic = bytes(&[255]);
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            h.context,
            request,
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::INVALID_ARGUMENT
    );
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            h.context,
            raw_request("abi/raw/refusal"),
            std::ptr::null_mut()
        ))
        .code,
        abi::status_code::INVALID_ARGUMENT
    );
    let binding = h.create("abi/raw/refusal");
    assert_eq!(
        abi_call!((HOST_API.destroy_raw_publisher)(h.context, binding.port)).code,
        0
    );
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            h.context,
            raw_request("abi/raw/next"),
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::BUSY
    );
    assert_eq!(
        abi_call!((HOST_API.register_raw_egress)(h.context, binding.port)).code,
        abi::status_code::INVALID_HANDLE
    );
}

#[test]
fn malformed_raw_finishes_retire_without_delivery_and_leave_next_ingress_usable() {
    let h = RawHarness::new(1);
    let binding = h.create("abi/raw/malformed");
    let mut subscriber = h.manager.create_subscriber("abi/raw/malformed").unwrap();
    for variant in 0..6 {
        let slot = h.loan(binding.port, 64);
        let mut frame = oracle_frame(27, 900, [4.0, 5.0, 6.0]);
        let total = match variant {
            0 => 63,
            1 => {
                frame[0..8].copy_from_slice(&0u64.to_le_bytes());
                64
            }
            2 => {
                frame[8..12].copy_from_slice(&40u32.to_le_bytes());
                64
            }
            3 => {
                frame[12..16].copy_from_slice(&0u32.to_le_bytes());
                64
            }
            4 => {
                frame[16..20].copy_from_slice(&u32::MAX.to_le_bytes());
                64
            }
            _ => {
                frame[12..16].copy_from_slice(&40u32.to_le_bytes());
                frame[16..20].copy_from_slice(&1u32.to_le_bytes());
                frame[40..48].copy_from_slice(&u64::MAX.to_le_bytes());
                64
            }
        };
        fill_raw(&slot, &frame);
        let mut recipients = 999;
        assert_eq!(
            abi_call!((HOST_API.finish_raw_output)(
                h.context,
                slot.lease,
                total,
                &mut recipients
            ))
            .code,
            abi::status_code::INVALID_ARGUMENT
        );
        assert_eq!(recipients, 0);
        assert_eq!(
            with_context(h.context.value, |host| Ok(host.outstanding_leases())),
            Ok(0)
        );
        assert!(subscriber.view_raw().unwrap().is_none());
    }
    let expected = oracle_frame(29, 999, [7.0, 8.0, 9.0]);
    let slot = h.loan(binding.port, 64);
    fill_raw(&slot, &expected);
    let mut recipients = 0;
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            h.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        0
    );
    assert_eq!(recipients, 1);
    let delivered = subscriber
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .unwrap();
    assert_eq!(&delivered[..], &expected);
}

#[test]
fn raw_loan_and_foreign_completion_refusals_preserve_the_owner_and_delivery() {
    let h = RawHarness::new(2);
    let foreign = RawHarness::new(1);
    let binding = h.create("abi/raw/borrow");
    let other = h.create("abi/raw/other");
    let mut subscriber = h.manager.create_subscriber("abi/raw/borrow").unwrap();
    let mut out = MaybeUninit::uninit();
    for request in [
        abi::LoanRequestV1 {
            schema_hash: 0,
            ..raw_loan_request(64)
        },
        abi::LoanRequestV1 {
            wire_fixed_size: 24,
            ..raw_loan_request(64)
        },
        raw_loan_request(39),
        raw_loan_request(4097),
    ] {
        assert_eq!(
            abi_call!((HOST_API.loan_output)(
                h.context,
                binding.port,
                request,
                out.as_mut_ptr()
            ))
            .code,
            abi::status_code::INVALID_ARGUMENT
        );
    }
    let slot = h.loan(binding.port, 64);
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            binding.port,
            raw_loan_request(64),
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::BUSY
    );
    assert_eq!(
        abi_call!((HOST_API.loan_output)(
            h.context,
            other.port,
            raw_loan_request(64),
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::BUSY
    );
    let mut recipients = 999;
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            foreign.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        abi::status_code::INVALID_HANDLE
    );
    assert_eq!(recipients, 0);
    assert_eq!(
        with_context(h.context.value, |host| Ok(host.outstanding_leases())),
        Ok(1)
    );
    let expected = oracle_frame(55, 777, [11.0, 12.0, 13.0]);
    fill_raw(&slot, &expected);
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            h.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        0
    );
    assert_eq!(recipients, 1);
    let delivered = subscriber
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .unwrap();
    assert_eq!(&delivered[..], &expected);
}

#[test]
fn raw_egress_refusal_keeps_local_ingress_and_zero_recipient_send_usable() {
    let h = RawHarness::new(1);
    let mut request = raw_request("__cerulion/raw-oracle");
    request.full_queue = abi::full_queue::CONFIG_DEFAULT;
    let mut out = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            h.context,
            request,
            out.as_mut_ptr()
        ))
        .code,
        0
    );
    // SAFETY: successful callback initialized the binding.
    let binding = unsafe { out.assume_init() };
    // The native control namespace is refused for dynamic egress only.
    assert_eq!(
        abi_call!((HOST_API.register_raw_egress)(h.context, binding.port)).code,
        abi::status_code::FAILED
    );
    let slot = h.loan(binding.port, 64);
    let expected = oracle_frame(8, 900, [1.0, 2.0, 3.0]);
    fill_raw(&slot, &expected);
    let mut recipients = 999;
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            h.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        0
    );
    assert_eq!(recipients, 0);
    let mut subscriber = h
        .manager
        .create_subscriber("__cerulion/raw-oracle")
        .unwrap();
    let slot = h.loan(binding.port, 64);
    fill_raw(&slot, &expected);
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            h.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        0
    );
    assert_eq!(recipients, 1);
    let delivered = subscriber
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .unwrap();
    assert_eq!(&delivered[..], &expected);
}

#[test]
fn raw_creation_never_falls_back_to_another_context_transport() {
    let ordinary = Harness::new(vector_layout(), None, 2);
    let mut out = MaybeUninit::uninit();
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            ordinary.context,
            raw_request("abi/raw/unavailable"),
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::UNSUPPORTED
    );
    let zero_budget = RawHarness::new(0);
    assert_eq!(
        abi_call!((HOST_API.create_raw_publisher)(
            zero_budget.context,
            raw_request("abi/raw/none"),
            out.as_mut_ptr()
        ))
        .code,
        abi::status_code::BUSY
    );
    let mut recipients = 999;
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            abi::ContextHandle::default(),
            abi::LeaseHandle::default(),
            64,
            &mut recipients
        ))
        .code,
        abi::status_code::INVALID_HANDLE
    );
    assert_eq!(recipients, 0);
}

#[test]
fn raw_host_publish_and_receive_add_no_heap_allocation_after_warmup() {
    let h = RawHarness::new(1);
    let binding = h.create("abi/raw/allocs");
    let mut subscriber = h.manager.create_subscriber("abi/raw/allocs").unwrap();
    let expected = oracle_frame(19, 900, [11.0, 12.0, 13.0]);
    let slot = h.loan(binding.port, 64);
    fill_raw(&slot, &expected);
    let mut recipients = 0;
    assert_eq!(
        abi_call!((HOST_API.finish_raw_output)(
            h.context,
            slot.lease,
            64,
            &mut recipients
        ))
        .code,
        0
    );
    drop(
        subscriber
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap(),
    );
    let count = CountAllocations::begin();
    for _ in 0..8 {
        let slot = h.loan(binding.port, 64);
        fill_raw(&slot, &expected);
        assert_eq!(
            abi_call!((HOST_API.finish_raw_output)(
                h.context,
                slot.lease,
                64,
                &mut recipients
            ))
            .code,
            0
        );
        assert_eq!(recipients, 1);
        let delivered = subscriber
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap();
        assert_eq!(&delivered[..], &expected);
    }
    // Subtract exactly the pinned upstream 2035 allocation per native loan.
    let upstream = upstream_2035_publish_allocs(8);
    assert_eq!(count.finish() - upstream, 0);
}
