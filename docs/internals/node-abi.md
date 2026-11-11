# Node interface

`cerulion_node_abi` defines immutable C records and function tables using Stabby
72.1.16. Boundary major 1 is independent of runtime version and rustc release.
Validate the full table size and complete IStable ID before accessing a table. The header is the first field. Appending fields requires a new table type;
additional capabilities are discovered through separately versioned extensions.
Unknown optional extensions are ignored without casting. Required unavailable capabilities refuse before node initialization.

Host context, port and sample leases use owner-local integer handles, never encoded pointers. The host owns transport, queues, clocks, snapshots, counters
and native sample drop glue. Node state and overflow allocations remain owned by the node. Borrowed byte spans retain their owner until the synchronous call
returns; slot leases end only through their explicit terminal host operation.

Loan headers retain loan-time timestamps. Commit consumes the wire sequence;
discard consumes none. `submit_spill` borrows initialized overflow payload bytes
and the original loan lease, preserving its header and timestamp while copying
once into a fresh host loan. Adaptive capacity and configured ceiling are both
carried by the loan descriptor. Submission records a single attempt, preserving
the original loan on success and failure. After the node destroys its typed
writer, `discard_output(SPILL_RELEASE)` releases the original loan without a
node-discard count. A duplicate submission cannot publish twice. A spill is the
copied exception to direct slots.

Every call catches panic on its originating side. Error tokens identify owner
records; diagnostics copy into caller storage, and short buffers retain pending
text. Non-cancellable external helpers retain their library and host context for process lifetime. Native plugins under local-transport remain unsupported.

`bind_port` indexes each direction's ports in declaration order and returns
`PortBindingV1`. Its topic borrows immutable host context storage until context
destruction; the node copies it into its own `Arc<str>` only at create. Input
read mode is the existing host classification and later reads must match it.
Every sync operation has its own `head_op` tag, including boundary versus refill
sites. `head_kind::FAILED` never enables alignment descent; `WITHHELD` preserves
the replay read plan's deliberate empty drain/refill answer.

Events are polled for one port and kind. Backpressure maps original regime time,
total, regime count, dropped and capacity directly; detail is DROP_OLDEST, BLOCK
or SAMPLE, with Sample(N)'s milliseconds in policy_value. Watchdogs map original
miss time and total plus configured window_ms and observed elapsed_ms.
Liveliness maps original change time, total and publisher count (capacity),
state (detail) and cause. All irrelevant fields are zero. Local port names are
cached at create, so reconstructing author events needs no allocation.

`accumulate_count` hands each pending batch to one fire and returns both per-fire
and lifetime drops. Repeated calls and indexed frame borrows during that fire
observe the same batch. Later fires without collection receive no frames or
per-fire drops. `mark_served` records a read once per distinct host frame per
fire, including when repeated iteration creates another lease for that frame.

Accumulator capture writes one native per-port record in little endian:
`u32 name_len`, UTF-8 field name, `u64 lifetime_drops`, `u64 pending_drops`,
`u32 pending_count`, then each `u32 frame_len` and full wire frame. Already served
pending batches encode zero pending drops and frames. The node writes the outer
`u32 accumulate_input_count` once, only if it has accumulate inputs, then appends
per-port records in input declaration order. Restore accepts exactly one matching
record and checks name, schema, capacity, counters, truncation and trailing bytes
before host mutation. This is the existing native canonical encoding.
`StateSinkV1` is a frozen concrete callback record without a table header; its
pointer and alignment are checked, and writes are synchronous borrowed spans.
