# cerulion_node_abi

A compiler-independent interface between a Cerulion host and its node libraries.
Immutable C tables exchange owner-local integer handles and borrowed byte spans. Each owner allocates and releases its own objects; shared-memory payloads are
accessed in place. Adaptive output overflow is explicit and copies the spill payload once into a fresh host loan.

Boundary major 1 freezes every table's shape, full Stabby layout identity and operation meaning. Runtime release versions and compiler fingerprints are not
compatibility gates for this interface. Additional services use separately versioned tables. Unsupported required capabilities refuse before node init.

This crate defines the interface. Transport services and node behavior are supplied by table implementors.
The node-author interface remains the existing node macros.
