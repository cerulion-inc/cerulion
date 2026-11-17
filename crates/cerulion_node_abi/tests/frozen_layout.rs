// SPDX-License-Identifier: AGPL-3.0-only
//! Literal boundary-major-1 oracles. Never regenerate to accept an ABI change.
//! An intentional field, name, module or layout change needs a new boundary major.
use cerulion_node_abi::*;
use stabby::IStable;

macro_rules! frozen {
    ($ty:ty, $id:literal, $size:literal, $align:literal; $($field:ident = $offset:literal),* $(,)?) => {
        assert_eq!(<$ty as IStable>::ID, $id, concat!(stringify!($ty), " ID"));
        assert_eq!(std::mem::size_of::<$ty>(), $size, concat!(stringify!($ty), " size"));
        assert_eq!(std::mem::align_of::<$ty>(), $align, concat!(stringify!($ty), " alignment"));
        $(assert_eq!(std::mem::offset_of!($ty, $field), $offset,
            concat!(stringify!($ty), ".", stringify!($field)));)*
    };
}

#[test]
fn frozen_major_one_ids_sizes_alignments_and_every_field_offset() {
    frozen!(ContextHandle, 15632280398380246287, 8, 8;
        value = 0,
    );
    frozen!(NodeHandle, 3468830800515447964, 8, 8;
        value = 0,
    );
    frozen!(PortHandle, 11767676472938440724, 8, 8;
        value = 0,
    );
    frozen!(LeaseHandle, 636218546245591237, 8, 8;
        value = 0,
    );
    frozen!(Bytes, 5605723357302546148, 16, 8;
        ptr = 0,
        len = 8,
    );
    frozen!(MutableBytes, 17409903815292314277, 16, 8;
        ptr = 0,
        len = 8,
    );
    frozen!(Status, 6629459947330741213, 16, 8;
        error = 0,
        code = 8,
        kind = 12,
    );
    frozen!(TableHeaderV1, 17909758689325262928, 16, 8;
        layout_id = 0,
        size_bytes = 8,
        boundary_major = 12,
    );
    frozen!(LoanRequestV1, 13973241245201706960, 24, 8;
        schema_hash = 0,
        minimum_capacity = 8,
        wire_fixed_size = 12,
        variable_count = 16,
        alignment = 20,
    );
    frozen!(WriteSlotV1, 13591451500617333037, 40, 8;
        data = 0,
        lease = 8,
        schema_hash = 16,
        capacity = 24,
        configured_capacity = 28,
        alignment = 32,
        prefix_size = 36,
    );
    frozen!(ReadSlotV1, 2172199871240255417, 32, 8;
        data = 0,
        lease = 8,
        schema_hash = 16,
        size = 24,
        alignment = 28,
    );
    frozen!(PortBindingV1, 4184022173806012486, 32, 8;
        port = 0,
        topic = 8,
        configured_capacity = 24,
        read_mode = 28,
    );
    frozen!(HeadOutcomeV1, 1209123161935321887, 24, 8;
        popped = 0,
        timestamp_ns = 8,
        has_timestamp = 16,
        kind = 20,
    );
    frozen!(ClockValueV1, 557675686714801171, 16, 8;
        timestamp_ns = 0,
        present = 8,
        reserved = 12,
    );
    frozen!(EventV1, 13516972072697482723, 88, 8;
        port = 0,
        timestamp_ns = 8,
        total = 16,
        in_regime = 24,
        dropped = 32,
        capacity = 40,
        kind = 48,
        detail = 52,
        window_ms = 56,
        elapsed_ms = 64,
        policy_value = 72,
        cause = 80,
        reserved = 84,
    );
    frozen!(AccumulateInfoV1, 17445629654433222818, 24, 8;
        count = 0,
        reserved = 4,
        dropped = 8,
        dropped_total = 16,
    );
    frozen!(PublisherCountersV1, 14225412443818945004, 24, 8;
        committed = 0,
        discarded = 8,
        send_failures = 16,
    );
    frozen!(ExtensionDescriptorV1, 2511464008677996990, 32, 8;
        table = 0,
        id = 8,
        layout_id = 16,
        size_bytes = 24,
        version = 28,
    );
    frozen!(RawPublisherRequestV1, 5995853105066060488, 32, 8;
        topic = 0,
        schema_hash = 16,
        configured_capacity = 24,
        full_queue = 28,
    );
    frozen!(StateSinkV1, 15443573577566521242, 16, 8;
        context = 0,
        write = 8,
    );
    frozen!(StateApiV1, 17305561600594939313, 56, 8;
        header = 0,
        shape = 16,
        capture = 24,
        restore = 32,
        inline_safe = 40,
        probe = 48,
    );
    frozen!(ExternalSourceV1, 5543525071141940803, 16, 8;
        fd = 0,
        kind = 8,
        flags = 12,
    );
    frozen!(ExternalApiV1, 1471837350784579199, 24, 8;
        header = 0,
        describe = 16,
    );
    frozen!(NodeApiV1, 38166335489343417, 112, 8;
        header = 0,
        metadata = 16,
        required_host_capabilities = 32,
        capabilities = 40,
        create = 48,
        tick = 56,
        shutdown = 64,
        destroy = 72,
        copy_error = 80,
        query_extension = 88,
        state = 96,
        external = 104,
    );
    frozen!(HostApiV1, 13617436632655746881, 248, 8;
        header = 0,
        capabilities = 16,
        bind_port = 24,
        loan_output = 32,
        finish_output = 40,
        submit_spill = 48,
        discard_output = 56,
        receive_input = 64,
        release_input = 72,
        input_head = 80,
        set_snapshot_inputs = 88,
        snapshot_inputs = 96,
        pump_history = 104,
        clock = 112,
        request_shutdown = 120,
        environment = 128,
        poll_event = 136,
        publisher_counters = 144,
        copy_error = 152,
        diagnostic = 160,
        accumulate_count = 168,
        accumulate_frame = 176,
        mark_served = 184,
        capture_accumulate = 192,
        query_extension = 200,
        restore_accumulate = 208,
        create_raw_publisher = 216,
        finish_raw_output = 224,
        register_raw_egress = 232,
        destroy_raw_publisher = 240,
    );
    frozen!(BootstrapFn, 11651157913773210172, 8, 8;
    );
}
