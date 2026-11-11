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
discard consumes none. `finish_spill` borrows initialized overflow payload bytes
and the original loan lease, preserving its header and timestamp while copying
once into a fresh host loan. Adaptive capacity and configured ceiling are both
carried by the loan descriptor. A spill is the copied exception to direct slots.

Every call catches panic on its originating side. Error tokens identify owner
records; diagnostics copy into caller storage, and short buffers retain pending
text. Non-cancellable external helpers retain their library and host context for process lifetime. Native plugins under local-transport remain unsupported.
