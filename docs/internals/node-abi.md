# Node interface

`cerulion_node_abi` defines immutable C records and function tables using Stabby
72.1.16. Boundary major 1 is independent of runtime version and rustc release.
Validate the full table size and complete IStable ID before accessing a table. The header is the first field. Appending fields requires a new table type;
additional capabilities are discovered through separately versioned extensions.
Unknown optional extensions are ignored without casting. Required unavailable capabilities refuse before node initialization.

Host context, port and sample leases use owner-local integer handles, never encoded pointers. The host owns transport, queues, clocks, snapshots, counters
and native sample drop glue. Node state and overflow allocations remain owned by the node. Borrowed byte spans retain their owner until the synchronous call
returns. Ordinary receive, accumulate and output leases obtained during a node
call reach their terminal host operation before it returns. Raw helper leases
are excepted. After destroy the host reclaims outstanding ordinary leases as
INVARIANT without publication; all corresponding pointers are then invalid.
Python and non-cancellable helper retention instead keeps library, context and
escaped leases alive until process exit.

Loan headers retain loan-time timestamps. Commit consumes the wire sequence;
discard consumes none. `submit_spill` borrows initialized overflow payload bytes
and the original loan lease, preserving its header and timestamp while copying
once into a fresh host loan. Adaptive capacity and configured ceiling are both
carried by the loan descriptor. Submission records a single attempt, preserving
the original loan on success and failure. After the node destroys its typed
writer, `discard_output(SPILL_RELEASE)` releases the original loan without a
node-discard count. The first submission fully accounts commit, send failure or an INVARIANT drop.
Require size == header_len + payload.len and capacity < size <= configured
capacity. After submission finish_output returns INVALID_HANDLE and discard
accepts only SPILL_RELEASE. SPILL_RELEASE without an attempt consumes the lease
as a counted INVARIANT. A duplicate submission cannot publish twice. A spill is the
copied exception to direct slots.

Every call catches panic on its originating side. Host error tokens are
context-scoped; node tokens are library-scoped, with one preallocated pending
slot per instance and one for create failures. A newer failure replaces pending
text, a complete copy consumes it, short buffers retain it, a stale token returns
INVALID_HANDLE and zero means none. Error-kind tags are frozen in the ABI crate;
unknown received kinds become untyped failures.

Every terminal output or input operation except INVALID_HANDLE consumes its
lease exactly once. A complete-frame size outside [prefix_size, capacity] is
consumed as a counted INVARIANT and returns INVALID_ARGUMENT. A non-OK create
frees partial node state and leaves no instance; the host never destroys it.
Successful create is paired with exactly one destroy even if shutdown fails.

The host serializes NodeApi, StateApi and ExternalApi entries for each instance.
Raw create/loan/finish/discard/egress calls may run on any helper thread,
concurrently with ticks and different publishers; short per-publisher locks do
not take the tick lock or return BUSY for contention. Raw helper leases never
block begin_fire. Ordinary port, state and sync calls require an admitted node
call; out-of-call or concurrent use returns counted INVALID_STATE. BUSY means
only destruction while leases remain outstanding: release them and retry. Non-cancellable external helpers retain their library and host context for process lifetime. Native plugins under local-transport remain unsupported.

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

Raw ingress publishers use owner-local handles and the ordinary slot/lease
records. ConfigDefault and Discard are supported; RESERVED_BLOCK refuses with
UNSUPPORTED and an explicit unsupported-strategy diagnostic. Unknown strategy
tags return INVALID_ARGUMENT before creation. Creation is bounded and cold;
publication copies caller bytes into the shared-memory loan once, preserves the
full header and returns the actual recipient count. Dynamic egress registration
refusal leaves local delivery usable.

Module paths, type names, field names, table sizes and every field offset are
frozen by committed literal oracles. Stabby remains exactly pinned: a checker
update must preserve those IDs. Bootstrap discovery must validate the complete
signature before invoking the constructor, which runs before its returned
header can be checked.
