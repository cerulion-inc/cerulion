# cerulion_dds - agent notes

The DDS platform crate behind `cerulion ros2 attach`: one-per-process participant
construction, SPDP/SEDP discovery, and the wire-native type-resolution rung
(`~/get_type_description`) over
cerulion-ros2-client/cerulion-rustdds. Green unit tests prove the pure decision
engines only - the live DDS path is pinned by an ignored hardware test.

## Invariants

- This is the ONLY main-workspace crate that pulls the DDS stack. Keep it that way:
  `cerulion_cli_engine` depends with `default-features = false` (DDS-free vocabulary
  only, `live` off), so the engine's isolated builds never compile rustdds.
- BUILD-CRITICAL: cerulion-ros2-client 0.10.1 and cerulion-rustdds 0.14.2 are
  published forks. Upstream rustdds 0.14.2 provides endpoint USER_DATA parsing
  and DiscoveryDB snapshot accessors; cerulion-rustdds is retained only for the
  `participant_lease_duration` builder knob and its 10-second lease behavior.
  Remove both forks only when upstream rustdds provides a participant lease
  duration knob. Never bump or drop either fork without re-validating against a
  live peer (unit tests cannot see a SEDP USER_DATA / snapshot change; the failure
  is a silent zero-hash harvest): run the ignored `live_discovery_box_test` and a
  `ros2 attach --dry-run` against a Jazzy talker, diff the report with the prior pin.
- Distro features select the GID width: default `jazzy` = 16-byte (Iron and newer -
  the wire rung's functional domain); `humble` = explicit 24-byte pre-Iron opt-in.
  A default build CANNOT decode pre-Iron `ros_discovery_info`, so an empty node
  table on such a network is expected, not a bug.
- ONE discovery window: `SchemaAcquirer::acquire` consumes the engine's single
  `DiscoveryResult`. Never open a second window to re-enumerate.
- Drive DDS through the node spinner + async status stream on one
  `smol::LocalExecutor`, with DDS objects created on the drain thread.
- The pure decision engine (`wire.rs`) stays DDS-free and oracle-tested; the live
  half (`wire_acquirer.rs`, `live` feature) is composed via
  `AttachAcquirers::production(wire)` - a REQUIRED argument, which is the structural
  pin against shipping the rung inert.
- Skips carry distinct, true reasons (type-not-discovered / no-hash /
  no-owning-node) - never merge them into a generic failure.

## Testing

- `cargo test -p cerulion_dds` - inline unit oracles; parallel-safe, no DDS peer.
- `cargo check -p cerulion_dds --no-default-features` - the vocabulary-only build
  the CLI engine consumes must stay green.
- Live pin: `tests/live_discovery_box_test.rs` is `#[ignore]` (hardware-only). It
  precondition-panics with the full bring-up recipe (`DDS_IFACE` + a CycloneDDS
  talker on the interface) rather than hanging or silently passing on a peerless
  machine.

## Gotchas

- ros2-client's `wait_for_service` has a lost-event race - service waits go through
  the sliced `poll_wait` helper, never one long blocking wait.
- The endpoint USER_DATA blob arrives CDR-encapsulated (`[u32 len]key=value;`) and
  is tolerantly stripped at the ONE parse seam - do not add a second parser.
- Per-call budgets, the call-phase wall cap, and the per-node wait-failure cache are
  what keep a robot with dead nodes from stalling attach - keep every call threaded
  through them.

Deep reference: docs/internals/network-daemons.md - read before modifying
discovery, the wire rung, or the feature matrix. See also
docs/packaging/dds-forks.md before changing the published DDS dependencies.
