# rmw_cerulion internals: FFI invariants, bridge contracts, test map

`rmw_cerulion` is the ROS 2 middleware layer over the Cerulion transport: a `cdylib`
(`librmw_cerulion.so`) that stock rcl/rclcpp/rclpy `dlopen()`s when
`RMW_IMPLEMENTATION=rmw_cerulion` is set, plus an `rlib` so the conformance tests can
call the same `extern "C"` surface in-process. There is no DDS and no CDR step: fixed
POD messages ride the rmw loaned-message API straight into the shared-memory slot;
variable messages flatten into the native flat wire format and pay one memcpy into the
loan. Frames carry the same fully-qualified schema hashes and layouts as
`native_ros2_messages`, so native Cerulion nodes and the `cerulion topic` verbs
interoperate with ROS traffic directly.

Module map: `src/api/` (the `extern "C"` entry points, grouped by rmw header),
`src/ffi/` (C ABI types + hand-written helpers), `src/runtime.rs` (process singleton,
registries, name mapping), `src/bridge.rs` (`AnyBridge` dispatch), `src/type_bridge.rs`
(C introspection codec + the borrow-window SEAL), `src/type_bridge_cpp.rs`
(C++ introspection codec + the seal's C++ twin), `src/heaphook.rs` (the borrow-window
heap-hook CONSUMER: dlsym'd per-symbol handshake, cursor bisection, shutdown counters;
the rmw deliberately never links the `cerulion_heaphook` crate, whose rlib would compile
a second malloc interposer into the `.so`; the mirrored scalars are pinned against the
hook's source by `heaphook_source_parity` tests),
`src/decode_failure_latch.rs` / `src/publish_reject_latch.rs` / `src/loan_refusal_latch.rs`
/ `src/borrow_degrade_latch.rs` (flood-suppression reporters),
`shim/cppstring_shim.cpp` (the only compiled C++).

## The FFI seam

### Panic containment

A Rust panic unwinding across an `extern "C"` boundary is undefined behavior and in
practice aborts the entire host process (move_group, rviz, the user's whole stack) on
what should be a recoverable per-call error. Every entry-point body therefore runs
inside `ffi::ffi_guard(fail_value, || …)`, which catches the panic, logs it with the
payload message, and returns the site's failure value (`RMW_RET_ERROR`, a null pointer).
Every entry point that touches transport state, a bridge, a registry lock or a latch runs
inside the guard. A sizeable minority do not, and those are the bodies with no panic
surface at all: constant-return stubs (`RMW_RET_UNSUPPORTED`), a null check followed by a
plain struct read/write or zeroing, `Box` allocate/free pairs, and the `rmw_count_*` verbs
(one name lookup under the registry read lock). Anything ELSE without the guard is a bug. The conversion is
pinned by the `ffi_guard` unit tests in `src/ffi/mod.rs`.

### Identifier gating

Entry points validate `implementation_identifier` via `ffi::is_our_identifier` before
touching a handle; rcl can hand an rmw object created by a different middleware to
this library, and dereferencing a foreign handle's payload is UB. Wrong identifier ⇒
`RMW_RET_INCORRECT_RMW_IMPLEMENTATION` (or null), never a deref.

### `repr(C)` and padding zeroing

Every type crossing the seam is `#[repr(C)]`. The wire layout is computed by the same
`repr(C)` size/alignment algorithm the C compiler uses over the same field list, so
struct copies and wire offsets agree by construction. `repr(C)` padding bytes are
UNINITIALIZED on the C side; the codecs explicitly zero every padding byte-range they
copy (`zero_struct_padding`, `padding_ranges` in `src/type_bridge.rs`) so that the same
logical message always produces byte-identical frames; the replay-equals-live
determinism contract extends to rmw traffic. Never remove the zeroing; `bridge_test`
pins padding determinism with poisoned-padding fixtures.

### Allocator discipline

Allocate and free on the SAME side, with the SAME allocator. FOUR allocator domains
coexist and must never cross:

| Memory | Allocator | Who frees |
|---|---|---|
| rcl-owned out-params (endpoint-info arrays, name lists, strings rcl inspects) | the caller's `rcutils_allocator_t` function pointers (`ffi::allocator_alloc`/`allocator_strdup`) | rcl, with the same allocator it passed in |
| rosidl **C** message containers the bridge fills (C strings, sequences) | raw libc `malloc`/`free`/`calloc` shims in `src/type_bridge.rs` | either side; rosidl's default allocator IS malloc/free, so rosidl-generated `fini()` can free memory the bridge allocated and vice versa |
| **C++** (`introspection_cpp`) containers (`std::string`, `std::vector`) | the C++ runtime's own `std::allocator`, reached ONLY through the typesupport's `resize_function`/`assign_function` pointers and `shim/cppstring_shim.cpp` (the C++ codec allocates no libc memory at all) | the C++ runtime, via the typesupport |
| rmw handle structs we own (`rmw_node_t`, publishers, …) | Rust `Box` behind opaque pointers | this crate, on the destroy path |

The libc choice for rosidl containers is byte-compatibility with rosidl's default
allocator, not convenience; a Rust-allocated buffer freed by generated C `fini()` code
(or the reverse) is heap corruption.

### No trait objects across the seam

`Box<dyn>` / `Arc<dyn>` never cross the cdylib boundary: trait objects carry vtable
pointers that are not stable across compilation units. Type-bridge dispatch is the
two-variant `AnyBridge` enum (`C(BridgedMessage)` for `rosidl_typesupport_introspection_c`:
rclpy, rclc; `Cpp(CppBridgedMessage)` for `rosidl_typesupport_introspection_cpp`:
rclcpp, MoveIt). Adding a typesupport means adding a variant, not a `dyn` upcast.

### Send/Sync soundness and poisoned locks

The transport is instantiated over iceoryx2's thread-safe service flavor, so the port
structs are `Send + Sync` by construction and carry no manual impls. The manual
`unsafe impl Send`/`Sync` that remain fall into two groups, each sound for a DIFFERENT
reason. (a) Registry handles in `src/runtime.rs` (`PublisherPtr`, `GraphGuardPtr`,
`NodeData`), sound only under the discipline documented on the type: access through the
registry lock, atomic pointee state, or identity-comparison-only. (b) Bridge types holding
BORROWED rosidl introspection pointers, `BridgedMessage` and `NestedLayouts`
(`src/type_bridge.rs`), `CppBridgedMessage` (`src/type_bridge_cpp.rs`), sound because the
pointee is rosidl's immutable, process-lifetime static typesupport data the bridge only
ever READS. rcl shares each endpoint across executor threads
through type-erased `void*`, so each endpoint keeps its port AND its outstanding loans
behind ONE `Mutex`; splitting them would allow a slot-map mutation to race a loan.
When you add a field to such a type, keep it inside the existing guard or the
`unsafe impl` becomes a data race.

Lock entity mutexes through `runtime::lock_unpoisoned`, which returns `None` when the
mutex is poisoned: a panic mid-operation may have torn shared-memory bookkeeping, and
re-entering it risks use-after-free, strictly worse than failing the call. The entity
stays wedged until destroyed. Plain-data locks (registries, guard lists) instead use
`unwrap_or_else(|e| e.into_inner())`; a torn `BTreeMap` is still a valid `BTreeMap`.

## Bindings selection (build.rs)

`build.rs` emits the C ABI types one of two ways, signalled by cfgs declared in the
workspace root `Cargo.toml`'s `unexpected_cfgs` table
(`cerulion_rmw_generated_bindings` / `cerulion_rmw_vendored_bindings`):

- **Installed ROS distro** (deployment): `AMENT_PREFIX_PATH` set, or
  `CERULION_RMW_SYS_INCLUDE` pointing at include dirs ⇒ bindgen runs over the real
  headers into `OUT_DIR`.
- **Vendored fallback** (development/CI without ROS): the committed
  `src/ffi/vendored_bindings.rs`, generated from pinned source clones.

The two variables fail DIFFERENTLY, and the difference is a deployment hazard worth
knowing:

- `CERULION_RMW_SYS_INCLUDE` set to paths of which NONE is an existing directory is a
  hard build error (an `assert!` in `collect_include_dirs`); an explicit override that
  resolves to nothing is a typo, not a reason to ship possibly-ABI-mismatched bindings.
  Unset it to opt into vendored deliberately.
- `AMENT_PREFIX_PATH` set but yielding no USABLE package include dirs: selection is per
  prefix (`era_check::select_ros_prefixes`); a prefix with no `include/` is silently
  ignored (pure-python package prefixes), one whose `include/` carries none of the core
  ROS package namespaces (`rmw`, `rcutils`, `rosidl_*`) is an unrelated include tree and
  is SKIPPED with a `cargo:warning` naming it (it must not select the bindgen path, which
  would otherwise panic on the missing headers). What happens when NOTHING qualifies
  depends on `ROS_DISTRO` (the contract stated precisely under "What a headerless build
  does" below): with `ROS_DISTRO` unset, the build falls back to VENDORED bindings with a
  warning; with `ROS_DISTRO` set, a non-empty prefix variable that yields no usable
  headers is a HARD BUILD ERROR (`build.rs` refuses to ship the rolling-era vendored
  snapshot under a build that names a distro; point the prefix at that distro's
  install, or unset `ROS_DISTRO` to build the unclaimed dev `.so` deliberately). So a
  typo'd ament prefix is a warning-and-vendored build only when no distro was named.
  (Selection ≠ validation: a marker-bearing prefix with mismatched-era headers still
  faces the fingerprint/claim gates and the size pins.)

There is no override knob: the selection is signalled to `src/ffi/mod.rs` purely by the
`cargo:rustc-cfg` the build emits. Both paths re-export through `src/ffi/mod.rs`, where
the hand-written `RMW_RET_*` constants shadow bindgen's at the re-export (explicit items
beat glob imports) so the return-code contract has one authoritative spelling.

### Era scaffolding

Struct layouts are per-distro ABI (Foxy's 13-field introspection `MessageMember` vs
Humble's 15 vs Jazzy's 16 (`is_key_` lands MID-struct) vs Lyrical/Rolling's 17; GID
width 24 → 16 at Iron; `rmw_init_options_t` grows `discovery_options` at Iron), so
build.rs layers three era probes on top of the source selection:

- **Header probes** (bindgen path): each post-Foxy rmw header is filesystem-probed
  under the collected include dirs and, when present, passed to clang as a
  `-DCERULION_HAS_*_H` define; `wrapper.h` gates the matching `#include` on it. This is
  what lets bindgen run cleanly on Foxy AND Humble (`rmw/discovery_options.h` is Iron+,
  so an unconditional include aborted the build on both).
- **Capability cfgs**: the SELECTED bindings file (generated or vendored, one code
  path) is grepped for marker tokens (`fetch_function`, `is_key_`, `discovery_options`,
  `get_type_hash_func`, …; the table lives in build.rs) and a
  `cargo:rustc-cfg=cerulion_has_<cap>` is emitted per hit, pre-declared via
  `cargo:rustc-check-cfg`. A cfg is a fact about the bindings the crate compiles
  against, never a claim about a distro NAME; it cannot be mispaired. The vendored dev
  path emits the full rolling capability set. `src/ffi/era_pins.rs` keys per-era
  struct-size `const _` asserts on these cfgs, so a header/bindings contradiction dies
  at `cargo build`.
- **Load-time guard** (`src/era.rs`, applied in `src/api/init.rs`): a SELECTION-DERIVED
  distro claim and the capability fingerprint are baked into the `.so`
  (`era::built_for()`, logged in the `rmw_init` banner). The guard runs at the FIRST
  entry points that touch caller memory: `rmw_init_options_init` / `_copy` / `_fini`,
  before any read or write of the caller's `rmw_init_options_t` (whose layout the
  RUNTIME distro decides: 104 / 168 / 160 bytes across the eras, with only the
  `instance_id` / `implementation_identifier` / `domain_id` / `security_options` prefix
  layout-invariant), and again in `rmw_init` as defense in depth for a caller that
  skipped `options_init` with a static struct, and there too it sits IMMEDIATELY after the
  null check, before the identifier dereference (every guarded export orders
  null check → era guard → first caller-memory touch, so a mismatch with a foreign
  identifier is reported as an era mismatch and not as an identifier error). Its
  verdict needs no struct read at all:
  it is the baked claim against `ROS_DISTRO`, so a refusal leaves the caller's bytes
  untouched (pinned with poisoned buffers and a PROT_NONE page in `tests/rmw_era_guard_test.rs`). The claim derives from the bindings actually compiled against,
  never from the raw build env, and is header-provenance-VERIFIED: a distro NAME is
  baked only when the bindings were GENERATED against installed headers whose
  capability fingerprint is CONSISTENT with that distro's era; `ROS_DISTRO` alone is
  not proof of the headers' distro (the env can say jazzy while `AMENT_PREFIX_PATH` /
  `CERULION_RMW_SYS_INCLUDE` supplies Humble headers, and `rmw_init` would then see
  matching names and admit a layout-incompatible `.so`). The pure classifier
  (`src/era_check.rs`, shared between build.rs and the crate's oracle tests) maps the
  claim to its era's expected capability subset and FAILS THE BUILD on a contradiction,
  naming the claimed distro, the observed era, the missing/unexpected markers, and the
  two likely causes (stale `ROS_DISTRO` vs wrong prefix) with the remedy for each; an
  unknown (future) distro name is NEVER baked as-is (baking it would let a typo'd name
  with any headers pass the equality guard unverified): its claim comes
  from the same fingerprint + layout machinery as the no-env path
  (`claim_for_unknown_distro`), with a warning naming both, and an ambiguous or
  unrecognized fingerprint fails the build; a genuinely new distro needs the era
  table extended before its own name admits. `"vendored-dev"` itself and the `era:`
  namespace are RESERVED: a generated
  build whose `ROS_DISTRO` normalizes to the unclaimed marker or to an `era:` claim FAILS
  THE BUILD, because baking it would forge "no claim" (or era-membership admission) onto a
  distro-specific ABI. A BLANK or whitespace-only `ROS_DISTRO` is a different case and is
  not an error: `build.rs` trims and filters it before the
  classifier, so it is treated exactly as UNSET and the build takes the fingerprint-derived
  no-distro path. (`check_distro_claim` still refuses the empty spelling as a reserved
  collision; that arm is defence in depth for a future caller that stops filtering, not a
  path the environment can reach.) A generated build whose
  environment names NO distro bakes a claim DERIVED from the fingerprint
  (`era_claim_for_observed`): the concrete distro name when the matched era has one
  member OR when the bindings' own `rmw_init_options_t` layout test separates the
  jazzy/kilted pair (`probe_init_options_size`: 168 ⇒ jazzy, 160 ⇒ kilted; the pair is
  fingerprint-identical but layout-DIVERGENT, and the init-options exports are the first
  entry points to touch that struct; they carry the guard themselves, and a claim
  that cannot separate the pair could not refuse before a write, so `era:jazzy` is
  NEVER baked; an unreadable size fails the
  build rather than guess); the `era:<token>` label only where every member's pinned
  layout is verified IDENTICAL: `era:lyrical` covers lyrical+rolling, byte-identical
  (verified on the release branches) across message_introspection.h, rmw/types.h,
  init_options.h, discovery_options.h, security_options.h and
  message_type_support_struct.h; the runtime guard then admits exactly those
  layout-identical members and refuses everything outside them, and a LITERAL claim (a
  generated build under a named `ROS_DISTRO` bakes the concrete `lyrical` / `rolling`)
  admits by the SAME table: `lyrical` and `rolling` admit each other, jazzy/kilted never
  (a name-equality compare there would refuse a byte-identical library); and a hard build
  failure when the fingerprint matches no known era (it never bakes the unclaimed
  marker: one distro's real generated ABI must not run everywhere on the vendored
  snapshot's dev seam). An env-set jazzy or kilted claim is CROSS-CHECKED against the
  probed init-options size and a contradiction fails the build. Every
  VENDORED build bakes the unclaimed `"vendored-dev"` marker regardless of `ROS_DISTRO`
  (an env-derived claim would label the pinned ROLLING vendored ABI with a real distro
  name and make the guard vouch for exactly the skew it exists to refuse).

  **What a headerless build does, precisely** (this paragraph is the code): with NO
  prefix variable set
  the build falls back to the vendored bindings and WARNS: loudly under a set `ROS_DISTRO`
  (the operator thinks they are building for that distro; tell them at build time, not at
  the runtime refusal), quietly otherwise. With a prefix variable SET but carrying no usable
  ROS headers the build FAILS instead of falling back, and there are two such cases:
  `CERULION_RMW_SYS_INCLUDE` naming no existing directory is a hard error whatever
  `ROS_DISTRO` says (an explicit override that resolves to nothing is a typo, and silently
  shipping vendored bindings under it would hand the operator an ABI they did not ask for);
  and a set-and-non-empty prefix variable TOGETHER WITH a set `ROS_DISTRO`, where falling
  back would put the pinned rolling snapshot behind a build the operator labelled with their
  distro. That second refusal is the strict default: the build stops rather than
  substituting the pinned rolling snapshot for the distro the operator named. The hard
  error on an explicitly-invalid `CERULION_RMW_SYS_INCLUDE` applies in every case. Stated residual: capabilities distinguish ERAS, not every adjacent distro:
  `jazzy`/`kilted` share one fingerprint, as do `lyrical`/`rolling` (today identical);
  `lyrical`/`rolling` agree on every pinned axis, while `jazzy`/`kilted` diverge on
  exactly one: kilted dropped `localhost_only` from `rmw_init_options_t` (168 → 160
  bytes), invisible to the fingerprint; the claim layer separates the pair (size probe
  + env cross-check above), and the init-options pin asserts the CLAIM-specific size on
  that arm (the baked claim is a compile-time constant: iron/jazzy pin 168, kilted pins
  160, anything else fails loud), and an UNREADABLE layout test REFUSES a jazzy/kilted claim
  outright (`divergent_claim_unverifiable`: the size is the pair's only cross-check, so
  a claim it cannot read is refused rather than admitted; a bindgen format change refuses those
  claims until the probe learns the new spelling, while unambiguous claims stay
  non-fatal). Kilted stays outside the support matrix; the runtime guard still
  separates deployments by name. The era size pins gate everything size-visible, including
  `rmw_init_options_t` itself (104 pre-Iron / 168 iron-jazzy / 160 later, the
  mixed-include-root defense). `rolling` means CURRENT upstream: an outdated rolling tree fails as a
  Jazzy-era contradiction, deliberately.
  The baked value is the checker's NORMALIZED claim (trimmed, ASCII-lowercased), and
  the runtime comparison normalizes both operands the same way; `ROS_DISTRO=JAZZY` at
  build or at run can neither refuse a compatible library nor mask a real mismatch.
  `rmw_init` refuses, loudly, naming both sides, when the baked distro contradicts
  the runtime `ROS_DISTRO`, turning the "wrong-distro `.so` dlopens fine, SIGSEGVs at
  the first typed operation" class into a one-line
  refusal. An absent `ROS_DISTRO` passes (the environment makes no claim), but the
  unclaimed `vendored-dev` marker is NOT an absence: the vendored snapshot is the pinned
  ROLLING-era ABI, so under a NAMED runtime it admits exactly that era's layout-identical
  members (`lyrical`/`rolling`, from the one membership table via
  `era_check::VENDORED_SNAPSHOT_ERA_TOKEN`) and REFUSES every other name; a vendored `.so`
  admitted under `ROS_DISTRO=jazzy` would write rolling offsets into Jazzy's
  `rmw_init_options_t` and die at the first typed operation. Where it IS admitted (its own
  era, unverified against a named distro) the `rmw_init` banner `warn!` still announces it
  (a dev convenience, never a deployment posture). The refusal also reaches rcl's error
  channel (`rcutils_set_error_state`, resolved through `dlopen(librcutils, RTLD_NOLOAD)`
  first (RTLD_DEFAULT cannot see the `RTLD_LOCAL` pybind module an rclpy host loads it
  through), then RTLD_DEFAULT), so rclpy/rclcpp report the paragraph with the entry point
  and both distros instead of "error not set"; the tracing line records the outcome as
  `rcl_error_channel=set|unavailable`. A cargo test process has no librcutils loaded, so
  the Rust-only tests can only observe `unavailable`; the `set` reading requires a ROS 2
  host, which is what the container integration lane provides.
  **Consequence for the test suite**: the guard sits in
  `rmw_init_options_init` as well, so running ANY rmw test binary under a `ROS_DISTRO` the
  build's own claim does not admit refuses at the FIXTURE: every e2e binary here builds
  its context with `rmw_init_options_init` + `rmw_init`, and their setup asserts
  `RMW_RET_OK`. That is the guard working, not a harness bug: run the suite with
  `ROS_DISTRO` unset (every claim shape admits an absent runtime) or set to a name the
  build's claim admits. The two binaries that must survive a CONTRADICTING inherited
  environment (`src/era.rs`'s lib arms and `tests/rmw_era_guard_test.rs`) name an
  admitted runtime for the duration of the fixture call themselves
  (`EnvVarGuard::unset` / `agreeing_runtime_for(baked_distro())`), because their whole
  subject is the contradiction they then arm.
- **C++ bridge gate**: `ffi/introspection_cpp.rs` hand-mirrors the C++ introspection shape
  of the era the build compiles against: the JAZZY/KILTED shape (`is_key_` and
  `has_any_key_member_` arrived at Jazzy), plus the Lyrical/Rolling tail field
  `is_rosidl_buffer_` under `cfg(cerulion_has_is_rosidl_buffer)` (stride 112 to 120; each
  field landed in the C and C++ structs in the same rosidl release, so the C capability
  tokens are the build-time proxies for the C++ mirror's era, and a compile-time pin holds
  the mirror's size equal to the bindgen-generated C member's), and the Humble and Iron message shape (no `is_key_` or `has_any_key_member_`) under `cfg(not(cerulion_has_is_key))`; the service mirror's trailing `event_members_` is keyed on `cfg(cerulion_has_event_members)`, present from Iron on and absent on Humble. Pre-Galactic builds (Foxy, Galactic: 96-byte members) compile
  the C++ typesupport arm of both resolvers to a loud REGISTRATION refusal: one `error!`
  through the single `CppBridgeGate::emit_refusal` seam, a CONSTANT paragraph with the
  verdict in a structured `verdict=` field (never a `"{}"` pass-through, which the repo's
  tracing-discipline walk refuses); the bypass composite is `cfg(test)` SILENTLY (the lib's
  own unit tests, unreachable by any shipped artifact) or the `test-seams` feature LOUDLY;
  `test-seams` is a PUBLIC feature a deployable build can enable, so that path emits
  `era::CPP_BYPASS_SEAMS_WARN` on EVERY resolve ("this build must never deploy"), making
  production silence impossible while the resolver-routed C++ e2e binaries (fixtures
  hand-built against the compiled struct, layout-self-consistent by construction) keep
  their surface. `test-seams` is in no default feature set and no shipping recipe enables
  it. The pre-Galactic C++ bridge variant is not implemented. The C introspection path reads
  bindgen-generated members; its hand-written sequence mirrors are pinned to their bindgen
  twins on every era (primitive and string sequences carry the Lyrical Buffer flags,
  message sequences never do), and a Buffer-backed member or instance is never forged,
  freed, or read as bytes by the bridge, so rclpy and C-typesupport consumers are
  unaffected.

## Runtime

`runtime::runtime()` lazily initializes the process singleton: the one
`TransportManager`, the endpoint registries, and, because host processes (move_group,
rviz, ros2cli) install no Rust tracing subscriber, which would silently drop every
diagnostic in the crate, a stderr `tracing` subscriber via `try_init` (a host that did
install one wins). The default filter is `rmw_cerulion=warn,cerulion_core=warn`;
`cerulion_core` must stay in it because the root-cause diagnostics (delivery failures,
malformed-frame drops) are emitted on `cerulion_core` targets.

Name mapping: `ros_topic_to_cerulion` is the IDENTITY: the fully-qualified ROS name
is the Cerulion canonical name (ROS `/chatter` ⇔ `cerulion topic echo /chatter`,
iceoryx2 service `/chatter/data`), for topics and services alike. That is what lets a
network gateway's canonical-name probe and egress tap find an rmw topic, and a desk
mirror of a remote `/chatter` land where an rmw subscription reads; a
slash-stripped spelling (`chatter/data`) is not a canonical name on the network plane
and reaches no networked host in either direction. A name without a leading `/` is
refused loudly and never prefixed (rcl expands names before an rmw sees one, so only a
direct C-ABI caller can produce one), and there is deliberately no alias for the
slash-stripped spelling. `ros_graph_type_name` maps the introspection-derived `pkg/Name` to the ROS graph's
`pkg/msg/Type`.

Network egress: every `rmw_create_publisher` also registers its topic (canonical name
plus the bridge's schema hash) on the transport's runtime-egress registration channel,
exactly as the `ros2 attach` bridge does for its raw routes: best-effort and network-free
(a registration-hostile name warns per topic; a local control-plane failure warns once
per process, repeats at debug). A gateway on the machine drains that channel and
announces the topic; the rmw itself opens no network session, so without a gateway the
registration is inert. Subscriptions never register (egress is produced topics only).
There is no unregister and nothing to build one on: the channel is an additive
per-process set republished until the process exits, and a gateway keeps a runtime
registration until it restarts, so a destroyed publisher's topic stays announced with
no live producer for the life of its process. That standing row is ACCURATE on the desk
(the decision: presence-based registration + liveness is the intended
model): the catalog's `producer_count` probe reads `Some(0)` and its data-flow
`liveness` annotation ages the row out of `streaming`, pinned by
`gateway_iox2_test::a_destroyed_producers_topic_stays_cataloged_with_honest_liveness`. Graph guard conditions live in a process-global
`Mutex<Vec<GraphGuardPtr>>` registry and the destroy path unregisters before freeing. A
guard condition is an `AtomicBool` plus a `cerulion_core::wake::Doorbell`
(Linux eventfd / macOS pipe), NOT an iceoryx2 entity: the FLAG is the truth, the doorbell
only wakes a parked `rmw_wait`; `trigger()` stores the flag BEFORE ringing, which is what
makes the waiter's drain-then-probe race-free. `notify_graph_change` rings through the same
`trigger()`. What `rmw_wait` PUMPS is every live PUBLISHER's iceoryx2 events
(`runtime::pump_publisher_events`), so TRANSIENT_LOCAL history still reaches a late joiner
on an otherwise idle publisher, and the event-driven wait's kernel block is capped at the
pump interval (20 ms) so a PARKED waiter keeps pumping.

**`rmw_wait` is event-driven with THREE exits**: (1) ready-return after the
non-consuming probe; (2) the deadline arm (zero-timeout calls, rcl's timer-ready shape,
return after ONE probe, and the race-safe final consuming pass); (3) a
`poll(2)` block on the attached subscriptions' EXISTING event-listener fds
(`CerulionSubscriber::event_listener_fd`; no second wake source is minted, so a publish
costs nothing new), the service/client request/reply listeners, and each guard's doorbell.
Drain-before-wait each iteration (guard doorbells + listener notification queues; a wake
is a SIGNAL, never a count; an undrained level-readable fd re-fires every block).
`poll(2)` rather than the iceoryx2 WaitSet: fd snapshots need no `&Listener` borrow pinned
across the block (lock, snapshot an integer, UNLOCK, block; no entity mutex is held while
parked), poll has no FD_SETSIZE fd-number ceiling (the select path aborts on fd ≥ 1024 on
macOS), and a dead fd degrades to POLLNVAL neutralization instead of a process abort. All
wait state is per `rmw_wait_set_t` (`WaitSetData`: fd snapshot + monotonic
`fd_blocks`/`fd_wakes`/`timeout_wakes`/`spin_probes`/`spin_wakes`/`degraded_waits`
diagnostics), never process-global; concurrent executor loops (GraphListener, TimeSource)
wait on disjoint entity sets over distinct wait sets. **A listener drain that FAILS degrades
the wait set for that call** (`WaitSetData::degraded`, reset at entry): the drain error is
surfaced from core (`drain_event_notifications` returns `Err`, never swallows), and a
readable fd nobody can drain would re-fire every block, so exit 3 falls back to the legacy
100 µs sleep pacing for the rest of the call; progress stays timeout-paced, never a spin;
logged through a `FailureRegimeLatch` (loud once per regime, counted repeats, a recovery
line). The pure `block_strategy(event_wait, degraded)` decides exit 3 so the kill-switch and
the degraded fallback cannot disagree. **The block is an adaptive LADDER, not a flat
block**: a flat block costs the stock posture a deep C-state exit on both ping-pong hops,
which the ladder's shallow first rung avoids while still reaching a low idle wake rate.
`FdWakeSet::wait` is `ppoll` on Linux (ns precision), the first rung is 200 µs
(identical latency to 100 at half the idle wake rate), after 50 consecutive empty
timeouts the rung
doubles per iteration up to the cap, and any fd wake or ready-return resets it; the state
persists across calls on the wait set (a ping-pong executor must come back to a shallow
rung). The rung and threshold are INTERNAL constants; there are no env
knobs, and retuning it is a code change. Counters: `block_rung_resets`,
`block_backoffs`. Pinned three ways in `rmw_wait_event_test`: ping-pong stays on the first
rung (resets ≥ rounds, and the rung is back at the first one after the last wake; a loaded
runner can legitimately buy one backoff mid-run, so zero-backoffs is printed, not asserted),
an idle set backs off and burns far fewer wakeups than the flat cadence, and a publish after
backoff still wakes through the fd (cumulative fd-wake band, generous hang guard).

**The park tier (Linux).** Every rmw publisher arms its topic's SHM doorbell at
`rmw_create_publisher` (`CerulionPublisher::enable_doorbell`, the same producer-side ring
the native graph build arms; `notify_sent_sample` already stores to it after every send, so
a publish rings it for free). When the event path is on, `CERULION_MONITOR_WAIT` permits,
and at least one subscription topic's doorbell page is mapped
(`WaitSetData::refresh_park_bells`: the consumer-side open `O_CREAT`s, so a subscription
whose producer is not up yet CREATES the page the producer later joins by name; only a
genuinely failed open is retried, on a 256-call throttle, which is also why a
subscription-only wait set PARKS on Linux), the block PARKS on the doorbells instead of
sleeping in `ppoll`:
`park_block` arms the native monitor-wait on the primary doorbell line
(`cerulion_core::monitor_wait::monitor_wait_until_addr`: UMWAIT on WAITPKG x86_64, WFE on
aarch64, a bounded ~100 µs sleep-recheck where the CPU has no primitive; never a busy-spin),
re-checking every doorbell plus the fd snapshot (`FdWakeSet::check`, a zero-timeout poll)
each recheck slice; a publish wakes the waiter hardware-instantly on the primary topic;
guard/service/client fds wake within one slice. The ladder rung bounds every park, so the
pump cadence and the idle backoff are tier-independent. **Bell ownership:** rmw publishers open the bell UNOWNED
(`CerulionPublisher::enable_doorbell_shared`: create-if-absent, never unlink); a ROS topic is
provisioned at two publishers (the `/rosout` shape), so an owned bell would let the first
publisher to die pull the page from under the survivor, which keeps ringing its old inode
while a wait set created afterwards maps a fresh one and never hears it (pinned by the
survivor test; a per-process refcount would not help across two processes). Residual: one
64-byte page per topic can outlive every publisher on the machine until the next creator, and a
re-created publisher joins the same page. When an OWNED creator (the native graph build's
bell, a stale-name cleanup) replaces the page under a wait set, that wait set maps a page
nobody rings: the first frame then arrives
via the fd path, ONE recheck slice (~100 µs), never the 20 ms timer, with the bell silent,
and `reconcile_stale_bells` re-maps the bell on exactly that evidence (frame-without-ring,
the pure `bell_is_stale`: pending sample AND the bell still at its park-entry seq; a
healthy bell has always advanced by the time the fd wake lands, since the ring precedes the
notify; a silent bell with no frame is someone else's fd wake and is left alone), so the
next publish rings again, the rmw analog of the native `DoorbellRegistry::reopen` on a
producer-reconnect `LivelinessEvent`. A ROS topic carries iceoryx2's default two
publisher slots and the rmw path runs no single-writer pre-check, so two publishers on one
topic are ordinary; the UNOWNED bell is what keeps the survivor's page valid when one of
them exits, and re-create is the whole ownership residual. Off Linux the doorbell is a no-op stub and the fd block serves
everything. Counters: `park_blocks`, `park_wakes_doorbell`, `park_bell_reopens`.

**The park is bounded and OS-cooperative.** A parked thread is RUNNING as far as the
scheduler is concerned, so an unbounded park on a shared CPU hands a co-located publisher
the core only on a scheduler tick, which costs a millisecond-class p50. Every recheck slice
that times out therefore YIELDS (`park_yields`), and the slice is 20 µs as requested, which
`UMWAIT` honors; on the aarch64 `WFE` tier (the park's default home) the effective slice is
one generic-timer event-stream period, which is machine-specific, so a co-located peer runs
inside one slice. Busy waiting of any kind, the park and a long spin alike, also costs a
flat penalty against a kernel-sleeping waiter at message periods from 5 ms to 40 ms and
none at 2 ms: a scheduler and CPU-topology effect around the peer's wake, not a frequency
one (a full-P-state spin reads the same). `park_policy()` is therefore a platform table:
aarch64 `WFE` → `ThroughRung` (park every block, yielding per slice); x86 (`UMWAIT` or
none) and off-Linux → `NoPark` (the `ppoll` tier), because the bounded park does not pay
for itself on x86 at either period while it does on aarch64. The shared
`CERULION_MONITOR_WAIT` flag resolves over that default (`resolve_park`): unset = default,
`0` = off, `1` = force on (on a no-park Linux target the bounded one-rung `FirstRungOnce`
shape). The Linux park pins run under that forced shape. The native
`monitor_wait_block` slices on the same shared `PARK_RECHECK` bound and yields after every
completed slice (counted in `park_yields`); its park horizon follows the live loop's own
deadline rather than this table.

**Fired-only drain/probe.** A stock rclcpp executor's set is 1 subscription + 6
parameter services + 2 guards; a loop that drains every listener and probes every
entity twice per wake costs ~19 non-blocking syscalls + ~21 SHM probes per hop, most of the
measured RTT. The fd snapshot carries an `EntityRef` table and `FdWakeSet::fired`
(`revents`) names exactly what woke the block: a wake drains and probes only those entities
(plus the subscriptions on a doorbell-named topic) and nulls the rest from the mask in ONE
consuming pass; a timed-out block drains and probes nothing; the call's ENTRY still runs one
`poll(fds,0)` + one FULL SHM probe (`probes_entry`) because rclcpp takes one message per
ready subscription per spin and a second queued sample has no notification left to fire an
fd for (pinned by the second-queued-sample contract test); an iteration whose
TRANSIENT_LOCAL pump ran probes everything. Counters `drain_calls` / `probes_entry` /
`probes_wake`; the 9-entity pin asserts ≤3 drains and ≤3 post-wake probes per hop with
exactly 7 entry probes. Kill-switch and degraded calls keep the legacy drain-all loop.

Knobs (ONE surface, shared with the native live loop; both parses live in
`cerulion_core::monitor_wait` and have exactly one definition each, which is the drift
protection): `CERULION_LIVE_SPIN_US` (`classify_live_spin_us`, one 100 ms ceiling, the
busy-probe in front of the FIRST block of a call; the one asymmetry, stated plainly: unset
means the consumer's own default, and the rmw wait's default is PARK-FIRST, no spin,
because it has no graph to derive wake-imminence from, where the native loop's derived
default spins only while a wake is imminent. The measurements behind park-first: spin off is
best-or-equal in every posture, spin on put about half the processes into a slow per-lifetime
same-core mode in the stock and C0 postures, never under a C1 cap, reproduced and
root-caused via `rmw_wait_pingpong_discriminator_test` [two wait sets, client + server
threads, counters per side, a Linux same-core pin arm]; the spin runs at most twice per call
and yields between probes); `CERULION_MONITOR_WAIT` (`classify_env_flag`: `0` disables the
park tier, so the fd/`ppoll` block serves every wake; `1` forces the park on, taking the
platform's own shape where it has one and otherwise the bounded one-rung shape on Linux;
unset or `auto` takes the platform default above, which parks only on Linux aarch64); and the
rmw-only `CERULION_RMW_EVENT_WAIT=off` kill-switch (selects the simple polling loop as
exit 3: probe + 100 µs sleep, no fd block, no park, no spin; `CERULION_LIVE_SPIN_US` is
ignored); see "ROS 2 rmw wait tuning" in `docs/user-api.md`.

Timer slack: on Linux the first `rmw_wait` on a thread sets that thread's timer slack to
1 ns via `prctl(PR_SET_TIMERSLACK, 1)` (`set_timer_slack_once`), DELIBERATELY thread-wide
(no narrower scope exists); without it every timed block pays the default 50 µs
`timer_slack_ns`: the kill-switch's 100 µs sleep overshoots by about that slack, which is
most of the small-payload RTT p50, and the ladder's 200 µs `ppoll` rung would pay the same.
Never pass 0 to that prctl: the kernel treats `arg2 <= 0` as "restore the 50 µs default".
The spin is read ONCE PER CALL (the SHARED `CERULION_LIVE_SPIN_US` knob, park-first when
unset); `rmw_wait_spin_test` carries the explicit-env clamp-at-`u64::MAX` pins and
`rmw_wait_event_test` carries the spin=0 arms.

## Type bridges (codec contracts)

- **Wire identity.** Bridged frames carry the schema hash computed by
  `MessageSchema::schema_hash` in `cerulion_core`, the same recipe
  `native_ros2_messages` compiles against. Never hand-compute or copy hash literals;
  the regeneration rule lives with the pinned-hash test in `cerulion_cli_engine`.
- **Zero-copy loan path, publish side.** A recursively-fixed, all-primitive message
  sets `can_loan`: the pointer rclcpp writes through IS the shared-memory slot.
- **Zero-copy loan path, publish side for UNBOUNDED types.** A type
  with at least one forgeable primitive sequence (`can_borrow_windowed`) also serves
  `rmw_borrow_loaned_message`, but only while the heap hook's versioned
  handshake is Active in the process (the `cerulion ros2` launcher preloads it; absent /
  version-skewed / foreign-allocator all degrade to the copying publish surface, loudly
  once). The borrow loans `[WireHeader][C struct][fill tail]`, CONSTRUCTS the message in
  the slot through the typesupport's `init_function` (the init-in-slot rule: never a
  zeroed never-constructed message), and arms the
  hook's borrow window over the tail on the borrowing thread, so a stock
  `resize`/`assign` fill bump-allocates straight into shared memory. The publish
  recovers the window's bump cursor (a bisection over the zero-length adopt test; the
  v2 ABI exports no cursor read), disarms, and SEALS the struct in place
  (`seal_borrowed_frame`): adopted members publish as wire-legality gap-frame placements;
  strings/nested/non-forgeable members, and every ESCAPEE (other thread, growth past
  the tail, foreign allocator, misaligned landing), are copied AT/ABOVE the cursor
  (bytes below it may be LIVE caller objects; bump-region gap bytes therefore ship
  VERBATIM, bounded by `RMW_BORROW_MAX_GAP_BYTES`, past which (or past the slot) the
  frame republishes through an exact-size copy loan reading the still-intact struct:
  refusals are pre-mutation, so the copy fallback is ALWAYS available). Window
  lifecycle rules, each load-bearing: only the FIRST outstanding borrow on a thread
  owns its window (a second proceeds windowless; disarming a live sibling window
  would cross-contaminate two loans' fills); a loan finished on the WRONG thread is
  copied and its slot HELD unsent until the owning thread borrows again (an armed
  window over a recycled slot would bump a later fill into a shipped frame),
  self-healing there, and the self-heal runs BEFORE the fresh loan is requested,
  because an orphaned slot charges the publisher's loan budget: below the loan, a
  budget-exhausted borrow would fail before the heal could ever run (regression-pinned); the window's quarantine extent outlives the frame until the
  transport re-issues the SAME slot (`retire_slot` on reuse, the only reclamation
  signal an rmw can observe; retire ends this rmw's coverage obligation for the
  slot and the rc is deliberately ignored. Hook-side disposition: retire
  TOMBSTONES the quarantine entry and a re-armed slot revives it, so a
  straggling in-slot free hits a counted no-op. Either way the rmw's re-arm on
  slot reuse re-establishes coverage, which is why no rmw code depends on the
  disposition). Before any `fini`, every top-level container header
  whose storage lies inside the slot is emptied (slot bytes are never allocator-owned)
  so the destructor cannot hand an SHM address to `free` even hookless; a nested
  member's internal containers remain the hook quarantine's job. Degrades ride
  `borrow_degrade_latch` (`kind=` names why) plus the unconditional
  `PublisherData::{borrow_adopted_count, borrow_copied_count, borrow_degrade_count}`,
  the only observables that discriminate an adopted frame from a copy that landed at
  the same offsets. A wrong-thread loan RETURN is NOT a publish degrade (nothing
  published, nothing copied; the slot is held for the owner's next borrow): it
  rides its OWN latch + counter (`kind=wrong_thread_return`,
  `borrow_wrong_thread_return_count`), recovered by the owner-thread self-heal
  sweep; `wrong_thread` stays the PUBLISH-path token (that one genuinely copies). The windowed surface is additionally CEILING-GATED at create
  (`windowed_borrow_geometry`, pure + boundary-pinned): the borrow writes
  `WireHeader + tail_off` unconditionally (header zero + the typesupport init over
  the struct), so a per-type / `CERULION_RMW_SLICE_CEILING` ceiling below that turns
  the windowed surface OFF for the topic (loud warn naming the remedy) instead of
  constructing past the loan; a ceiling that fits the geometry but leaves a tiny or
  zero fill tail stays ON (a zero-capacity window is sound: every fill escapes and
  copies). PUBLISHER DESTROY enforces the hold: a windowed loan whose armed window
  lives on ANOTHER thread is deliberately LEAKED (`mem::forget`; the pool never
  reclaims it, so the live window can never bump into recycled memory), counted by
  `borrow_destroy_leak_count()` and warned; same-thread windows are disarmed and
  released normally with `fini`. The teardown runs EVEN WHEN a panic poisoned the
  loan bookkeeping (`into_inner`, the documented exception to the
  `lock_unpoisoned` rule, in the destroy block's comment): the refusal shape would
  fall through to the `Box` drop and release every held slot under a possibly-armed
  window with zero counter evidence, so the poisoned arm trusts nothing logically:
  every windowed loan and every orphan is leaked (counted, warned), only Fixed
  loans release, and NO disarm is attempted (the ownership inference reads the same
  torn tables; a panic can leave an armed window absent from them or an entry
  stale, so every armed window is preserved; that thread's later borrows degrade
  windowless, latched, never corrupted). Loanable topics (fixed and windowed) create their
  publisher port at `publisher_max_loaned_samples = RMW_LOANED_SAMPLES_BUDGET` (4),
  and CREATE reserves all three loan-bookkeeping vectors to that budget (an
  in-budget borrow's push never allocates on the loan path). The windowed PUBLISH
  path reaches ZERO steady-state allocation for fixed and primitive-sequence
  members: both bridges' seals thread a per-publisher
  reusable `SealScratch` (plan items, placement offsets, offset-table entries, the
  off-slot head build, the owned-encode arena, cleared per publish, capacity
  retained), pinned by a thread-scoped counting-allocator arm; the one remaining
  per-publish allocation source is a Complex member whose nested type is VARIABLE
  (a header-class member), whose canonical sub-frame encode goes through the shared
  `CanonicalBodyBuilder`, the same cost the ingress decode path pays per frame.
  `rmw_shutdown` logs the hook's four diagnostic counters (their production
  reader). Known residuals: a same-thread incidental allocation made during a fill
  ships verbatim inside the frame's gap bytes (bounded by the gap budget); with two
  interleaved outstanding borrows on one thread, the second loan's fill bumps into
  the FIRST loan's window and can appear in ITS gaps (same process, same budget); a
  destroy-leaked slot (and its quarantine extent, and its message's HEAP-side
  containers) is unreclaimable for the process life, and the leak is WHOLE-LOAN by
  construction: the containers' headers are reachable only through the slot-resident
  struct the owner thread's live fill may be concurrently rewriting, so no partial
  "free the heap half" fini exists (reading the headers races the writer; freeing a
  block a concurrent assign still touches is the UAF class the leak prevents), and
  destroy cannot block on the owner thread's next borrow (it may never come, an
  unbounded wait on rcl's teardown path). Per publisher the leak is bounded by the
  loan budget + pool; PROCESS-life growth requires repeated
  create → cross-thread-arm → destroy-mid-borrow incidents, a contract-violating
  lifecycle, every incident counted (`borrow_destroy_leak_count`) and warned. The
  bounded-leak-over-corruption direction.
  **The registry consequence:** a destroy that leaks ANY slot also leaks
  the whole `PublisherData`, so the iceoryx2 `Publisher` is never dropped and the port
  stays REGISTERED on the topic. It must: iceoryx2's `Publisher::drop`
  (`src/port/publisher.rs:380-395`) only deregisters the port, while the port's
  on-disk `.port_tag` is the LAST field of the `Arc`-shared `PublisherSharedState`
  (`:194`) every forgotten `SampleMut` keeps alive, so dropping the port then leaves a
  deregistered port with a live tag, and at process exit the dead-node sweep's
  service-tags pass removed tags only for REGISTERED ports (`src/service/mod.rs:795`),
  its port-tags pass never deleted the tag, and `remove_node`'s `rmdir` failed
  ENOTEMPTY (`src/node/mod.rs:821-855`) on EVERY sweep forever; `cerulion clean`
  never converged and the exit-hygiene `.shm_state` reclamation it gates on was skipped
  for good. With the port still
  registered the exit is the ordinary "died with live publishers" shape and one sweep
  reclaims everything (deleting the tag by path instead would strand the port's data
  segment; the port-tags pass is what reclaims it). Cost, in full: forgetting the `Box`
  also skips `CerulionPublisher::drop`, and that type owns THREE iceoryx2 ports (the data
  publisher plus the event notifier and listener; `crates/cerulion_core/src/transport/publisher.rs`,
  the struct's first three port fields), so all three stay registered for the process
  life: one of the topic's publisher slots AND one notifier + one listener slot on its
  event service are held, against every process on the machine. `CerulionPublisher::drop` is
  also the only sender of `PubSubEvent::PublisherDisconnected`, so no disconnect edge is
  ever sent; and the transport's liveliness is COUNT-based
  (`ExternalPublisherProbe::live_publishers` → the runtime's liveliness sweep mints the
  `Lost`/`PublisherDisconnected` edge from the live count), so a native subscriber of the
  topic never sees a `Lost` edge for this publisher until the process exits and the
  dead-node sweep reclaims it. The disconnect notify is NOT sent before forgetting: the
  sweep keys on the count, not the event, so it would change nothing there, while an
  event-keyed consumer would be told "disconnected" about a port that still counts as
  attached. Slot cost: the port holds one of the topic's publisher
  slots for the process life; rmw
  topics carry iceoryx2's default `max_publishers = 2` (`default_topic_config` leaves
  the cap unset + `PublisherProvisioning::External`, so no single-writer pre-check),
  so a same-topic re-create still succeeds only while the topic's OTHER slot is free
  (a second live publisher on the topic, the `/rosout` shape, makes the first
  re-create the refused one), and the next returns NULL
  (`ExceedsMaxSupportedPublishers`, logged `publisher creation failed`). Pinned
  in-process by `rmw_borrow_publish_test` (all three leak arms read the live port count;
  the cross-thread arm pins the 2-slot consequence) and cross-process by
  `rmw_leak_at_destroy_registry_test`.
- **Zero-copy loan path, take side.** `can_loan_take()` is the TAKE
  gate and is wider than `can_loan()`. A fixed type hands out the received frame's SHM
  payload pointer directly. A type carrying at least one FORGEABLE primitive sequence
  (unbounded, non-`bool`, default-less: `Image.data`, `PointCloud2.data`,
  `CompressedImage.data`, `LaserScan.ranges`) is served through an rmw-owned SHADOW
  message: the small remainder (fixed fields, strings, nested messages, bounded and
  `bool` sequences) is unflattened INTO the shadow exactly as the copying take does,
  and each forgeable member's container header (the rosidl `{data, size, capacity}`
  sequence, or the libstdc++/libc++ three-pointer `std::vector` triplet) is aimed at
  the entry's bytes inside the HELD sample with `capacity == size` (every growth path
  reallocates). The iceoryx2 sample is held in `pending_takes` across the C ABI; the
  return UN-FORGES the shadow (inside `TakeShadowPool::release`, unconditionally)
  BEFORE the sample's borrow is released and recycles it; a destroy with loans
  outstanding drops each `PendingTake` shadow-first, so the typesupport's `fini` only
  ever runs over empty sequences; no allocator is ever handed a forged buffer.
  Shadows are built lazily through the typesupport's own `init_function` (a
  typesupport missing `init`/`fini` is not eligible) up to the loan budget and then
  recycled, so a steady-state forged take allocates nothing on the rmw side. The
  three-pointer layout is verified ONCE per process, in release builds too, through
  the compiled shim (`vector_triplet_layout_verified`); a mismatch disables forging for
  every C++ type, loudly. Frames are wire input: a forge target that is not
  element-aligned, or not a whole number of elements, is refused all-or-nothing
  through the decode latch (the shadow stays un-forged). PLACEMENT is checked
  before alignment and is a different kind of fact: an entry whose offset sits
  BELOW the frame's data floor (`WireLayout::data_floor()`, the first byte past
  the offset table, the ONE definition the `FrameWalker`'s audit also enforces)
  would alias the fixed section or the table itself, so that member is never
  forged. It is COPIED into the shadow instead (exactly the bytes the copying
  `rmw_take` serves for the same frame; the two takes never disagree), counted
  (`SubscriptionData::forge_fallback_count`; `loan_refusal_latch::report_forge_fallback`:
  `warn!` once per regime, `debug!` repeats, decade re-announcement, one `info!`
  recovery when a well-placed frame follows), left OUT of the `ForgeOutcome::forged`
  mask so the un-forge never nulls the copy, and the shadow is RETIRED on return
  (destroyed; its `fini` frees the copy) rather than recycled, so the next forge
  cannot leak it. The same holds when a LATER entry of that frame turns out
  malformed: `unflatten_forged` returns `Err(ForgeOutcome)` carrying the copies made
  before the failure (nothing forged survives), and the take records it so the shadow
  is retired, not recycled. An empty entry is always forged to the empty header.
- **The loaned-take contract.** The message is valid only inside the
  callback (as on every loaned rmw), and the subscriber mapping is READ-ONLY
  (iceoryx2 opens subscriber data segments `AccessMode::Read`), so an in-place write
  through a forged `data()` (a mutable `SharedPtr` callback editing pixels) faults the
  process: SIGSEGV on Linux, SIGBUS on macOS: loud, never silent, never another
  subscriber's view. `rmw_shadow_take_test` pins that fault in a child process. What
  the loaned take does NOT make loud: a mutable callback that GROWS the forged vector
  (`resize`/`push_back`/`assign` past `size()`) reallocates and then frees the forged
  SHM range through the C++ allocator; glibc's `free` aborts on most size words it
  finds in front of the pointer but can silently poison `tcache` for a plausible one.
  The heap hook's `free()` interposer closes that by address range when the hook is
  preloaded; without it, the loaned take is for the const-callback shape and the
  mutable-growth shape is unsupported, not merely discouraged. rclcpp routes EVERY callback of a
  `can_loan_messages` subscription through the loaned take once
  `ROS_DISABLE_LOANED_MESSAGES=0` (Jazzy's opt-in), so that variable is the user gate.
- **Budgets.** The borrow budget is `RMW_TAKE_LOAN_BORROW_BUDGET` = 4, provisioned at
  service create via `TopicServiceConfig::create_borrow_floor` on BOTH create paths,
  keyed on `can_loan_take()` (create provisions, open tolerates); the shadow pool has
  the same capacity. A consumer retaining loans past either is REFUSED (before any
  frame is consumed on the pool arm, on iceoryx2's `ExceedsMaxBorrows` on the
  transport arm) through one `FailureRegimeLatch` (`loan_refusal_latch`:
  `kind=shadow_pool_exhausted` / `shadow_allocation_failed` / `receive_failed`,
  unconditional `SubscriptionData::loan_refusal_count`), never a per-take flood.
  One regime, but each `kind=` carries its OWN headline and remedy: a pool
  exhausted by retained loans is cured by returning one; a failed shadow
  allocation is memory pressure with nothing to return; a receive failure carries
  the transport's own reason (`ExceedsMaxBorrows` is the budget, anything else is
  not a loan the consumer can return).
- **Adopt-take (`--adopt-take`).** Zero-copy PLAIN takes
  for retaining consumers: under the gate, `take_impl` swaps `try_receive_one` for
  `try_receive_one_owned`, runs `unflatten_forged` against the **caller-owned**
  message (fixed fields / strings / nested members copy exactly as `unflatten`
  copies them; each forgeable sequence's container header aims at the held sample's
  bytes with `capacity == size`), registers each forged member's EXACT byte range
  with the preloaded heap hook (per-field sub-ranges, the hook's granularity
  contract; cookie = `Arc::into_raw` of an `Arc<AdoptedSample>` clone, one per
  range), and returns with the sample HELD. The app's eventual `fini`/destructor
  frees each forged `data` pointer; the hook's interposed `free` classifies it by
  range, fires the process-global release callback, which drops one clone; the
  LAST drop releases the SHM borrow + publisher-pool slot. Double release is
  structurally impossible (the hook fires at most once per registration, by cookie
  identity) and the sample provably outlives the last forged free with no separate
  counter to get wrong.
- **The adopt-take gate is a witness, not a flag.** `CERULION_RMW_ADOPT_TAKE=1` is
  read at subscription CREATE (it keys the borrow floor; per-subscription latching
  avoids mid-stream mode flips), but the adoption branch takes an
  `&AdoptTakeGrant`, constructible ONLY from an Active heap-hook handshake with
  the three take-side symbols resolved (`register_segment` / `unregister_segment` /
  `set_release_callback`, all present at hook ABI v2; `HookApi` resolves
  all-or-degrade, so `active_hook()` returning `Some` IS the proof); no grant, no
  branch; without it NO ADOPTED TAKE can put an SHM address where an app `free`
  would meet it, so the misconfigured shape (env set, nothing preloaded) is never
  UB ON THIS BRANCH: it degrades to the copy path with ONE loud once-latched warn
  naming the env var, the missing preload, and the DIRECT-launch remedy (the
  launcher flag is not a remedy any more: it refuses; see the next bullet).
  Scope, stated
  exactly: this is a claim about the adoption branch, NOT about every free in the
  process. The loaned take exposes forgeable SHM ranges of its own and is
  governed by its own borrow window; a growth operation there can still hand an
  SHM pointer to libc `free` when the hook is absent, which is why that path
  documents growth as unsupported without the hook rather than relying on this
  paragraph. `cerulion ros2 run|launch --adopt-take` REFUSES
  at launch (exit 69), see the next bullet; adoption is armed by launching the
  node executable DIRECTLY with the hook preloaded and
  `CERULION_RMW_ADOPT_TAKE=1`.
- **`--adopt-take` is refused under `ros2 run` / `ros2 launch`**. Those verbs are a Python CLI that
  spawns the node as a FURTHER subprocess, and Python's spawn closes inherited
  descriptors by default (`close_fds=True`; measured: a descriptor whose
  `FD_CLOEXEC` was cleared reads `EBADF` in the grandchild, and survives only
  under `close_fds=False`). The node process therefore re-reads `LD_PRELOAD`,
  finds `/proc/self/fd/<N>` naming a descriptor that is closed in ITS table, and
  `ld.so` drops the entry: the hook does not load, `CERULION_RMW_ADOPT_TAKE=1`
  is still set, and every take is served by COPY. That is loud on the rmw side
  (the once-latched env-armed-but-no-hook warn fires in the node) but only AFTER
  the launcher has reported success, so the launcher refuses at its own
  `exec` instead, with a message naming the cause and the direct-launch
  alternative: as a pasteable command when the paths can carry it, and
  otherwise as a plain statement that they cannot plus the relocation that
  fixes it (the loader splits `LD_PRELOAD` on spaces and colons, and a
  non-UTF-8 path cannot be rendered as text; printing a command either way
  would hand the operator one that silently runs the copy path). The refusal is HOST-INDEPENDENT: "this verb cannot deliver the
  hook to the node it launches" is true on Linux/GNU too, so it must not hide
  behind the platform gate, which only answers whether adoption could run on
  this machine at all. The descriptor binding still applies under these verbs; falling
  back to naming the hook's path for the `ros2` case would re-open exactly the
  substitution window that binding closes. The gate behind the flag (host,
  staged preload, hook inspection, preload order, descriptor binding) is
  RETAINED in `ros2_cmd.rs`'s private `build_plan_with_adopt_gate` and stays
  covered by `crates/cerulion_cli_engine/tests/ros2_cmd_test.rs` through a
  `test-seams`-gated seam, so a shipped binary carries no one-call way around
  the refusal. Adoption under these verbs needs a binding that survives two process
  hops; there is none today. Pins:
  `crates/cerulion_cli/tests/ros2_run_e2e_test.rs::adopt_take_is_refused_by_both_verbs_and_never_execs`
  over the real binary, plus the engine's `adopt_take_is_refused_under_the_ros2_cli_*`
  and `the_default_entry_point_refuses_adoption_*` arms.
- **The validated hook reaches the child by DESCRIPTOR, not by name** (part of
  adopt-take, RETAINED behind the refusal above; no `cerulion ros2` launch
  reaches it today).
  The launcher validates a preload by READING it; if it
  then handed `ld.so` the path STRING, anything able to replace that file between
  the read and the child's `dlopen` would be loaded unvalidated. So the hook is
  opened ONCE, identified by `fstat` on that open handle, and staged into
  `LD_PRELOAD` as `/proc/self/fd/<N>`; the descriptor's close-on-exec bit is
  cleared just before `exec`, and the child's loader resolves that path to the
  INHERITED descriptor, the exact inode the launcher validated. No name is
  resolved a second time, so there is no window to race. `--adopt-take` is
  Linux/GNU-only already (the platform gate), so `/proc` is always present.
  What an operator sees changes with it: the child's `LD_PRELOAD` reads
  `/proc/self/fd/<N>` in place of the hook's path. Entries the launcher does
  NOT validate (ambient preloads) and the rmw library (which rides
  `LD_LIBRARY_PATH` as a directory, so there is no descriptor to bind) keep their
  names and are covered instead by an identity re-check immediately before `exec`,
  which narrows their window without closing it.
- **Adopt-take fallbacks all serve the copying take's exact bytes.** `forged == 0`
  (every forgeable entry below the data floor / past the mask) drops the sample
  immediately, exactly as the copying take does. A registration failure ("should be
  never") unregisters the already-made siblings, un-forges the message, copies the
  masked members through `copy_forged_members` (ONLY the `PrimSeq` copy arm,
  never a re-run of the whole `unflatten` over a partially-populated message,
  which would double-assign strings/sequences), drops the sample, latches, and
  still returns the successful take. A decode `Err` takes the plain path's
  decode-latch handling; below-floor copies made before the failure are owned by the
  caller's `fini`, the copying take's own convention. Reuse safety:
  `release_forgeable_members` runs before the adopting decode, because the plain
  `unflatten` frees a reused message's previous allocation inside its copy arms
  while the forge arm OVERWRITES headers; without the pre-pass, a caller legally
  reusing one initialized message across takes would leak the previous buffer, and
  a previously-FORGED header would lose the only pointer that can ever release its
  sample, pinning a borrow slot forever.
- **The fault-on-mutate rule (adopt-take inherits the loaned take's contract).** The forged
  bytes live in the subscriber's READ-ONLY mapping: `capacity == size` makes
  GROWTH the safe path (libstdc++/rosidl reallocate, then `free(old_begin)` /
  `realloc` routes through the hook to a release; `realloc_move_registered` pins
  the range during the copy), while an IN-PLACE write through a forged `data()`
  faults the process loudly (SIGSEGV/SIGBUS), never silently and never into
  another subscriber's view. Documented-unsupported, opt-in only, and acceptable
  for the flag's primary audience: rclpy never mutates the C message
  (`convert_to_py` reads, copies to Python, `fini`s).
- **Adopt-take: where the win is real.** For rclpy the win is BOUNDED:
  adopt-take removes copy 1 of 2 (the wire→C `unflatten` memcpy); the C→Python
  conversion copy and interpreter overhead remain. Real for a 1 MB Image on
  Jetson-class hardware, noise for small messages, and the `fini` right after
  conversion releases the borrow slot immediately, so rclpy is budget-benign. The
  win is FULL (payload retained, never re-copied) for bare-rmw / rcl C/C++
  consumers. Under stock rclcpp the forgeable types mostly never reach `rmw_take`
  at all: rclcpp routes to the LOANED take whenever `can_loan_messages` is true,
  and `can_loan_take()` true is exactly the forgeable case, so the chosen
  "UniquePtr callbacks" audience is reached today only where rclcpp's loaned
  dispatch copies out of the loan or loaning is disabled (a `can_loan_messages =
  false` knob for adopt-armed subscriptions does not exist and is not part of adopt-take).
  `rmw_take_serialized_message` (rosbag2) is NOT covered.
- **Adopt-take budgets and lifecycle.** Adopted samples pin borrow units for as
  long as the APP retains messages, so an armed create floors
  `subscriber_max_borrowed_samples` at `RMW_ADOPT_TAKE_BORROW_BUDGET` = 16
  (env-overridable `CERULION_RMW_ADOPT_TAKE_BUDGET`, CREATE-leg-only per the
  open-tolerates-smaller rule) instead of the loaned-take 4. `ExceedsMaxBorrows`
  under adopt is `taken = false` + a latched warn, NEVER `RMW_RET_ERROR`: the
  exhaustion is app-induced retention and self-heals when the holder releases.
  WHICH holder is classified, because an adopt-armed type is always
  `can_loan_take` too and outstanding LOANS consume the same budget: the line
  carries `adopted_outstanding=`/`loaned_outstanding=` and one of four kinds:
  `adopted_budget_exhausted` (free adopted messages; the only one that names
  the knob), `loaned_borrows_exhausted` (return loans), `mixed_borrows_exhausted`
  (either), `borrow_budget_unaccounted` (the counters say nobody, usually a
  release landing between the refusal and the read, so no remedy is prescribed).
  Adoption is NOT ARMED at all below a service borrow ceiling of
  `MIN_ADOPT_EFFECTIVE_BUDGET` (2: one unit for the message the app holds, one
  for the arriving receive): below it a caller reusing one buffer wedges, since
  the release that frees its borrow runs inside the take that borrow blocks.
  Such a subscription is served by the copying take, with one loud warn. The
  adopted decode also runs a READ-ONLY pre-write entry walk
  (`frame_entries_readable`) before it touches the caller's message, so a frame
  whose offset-table entry does not resolve, or whose primitive sequence is a
  ragged number of elements, is refused with that message byte-untouched
  (naming `var_idx=` and `reason=`); an allocation failure, a malformed body
  inside a NESTED member, and an unaligned forged entry still fail mid-decode,
  exactly as the plain copying take does.
  Destroy with adopted ranges outstanding: one loud warn with the count, and the
  registrations are LEFT IN PLACE (the app legitimately holds forged pointers;
  reclaiming would dangle them; the callback and the Arcs reference nothing
  subscription-scoped, so later frees still release). `rmw_shutdown` with ranges
  outstanding: the summary line, then (on the FINAL live context's shutdown
  only; live contexts are tracked by context IDENTITY, so an intermediate
  shutdown keeps the process-global callback (loudly) and a repeat shutdown of
  an already-shut-down context is the rmw.h no-op, never a second decrement)
  the release callback is CLEARED (a
  post-shutdown free must never call into a possibly-unmapped module; the hook
  then leaks-never-frees, counter kind 0) and the outstanding Arcs are
  deliberately leaked to process exit; the rmw runtime is a process-lifetime
  `OnceLock`, so the SHM mappings stay valid and the app's pointers stay readable.
  Never block shutdown on app frees. RESIDUAL: a take RACING that final
  `rmw_shutdown` can return a sample whose release path is already gone; the app takes
  delivery when `rmw_take` returns, so no check inside the take closes the window, it
  only moves it. The shutdown precondition is therefore that executors are stopped
  first, which rcl does before calling `rmw_shutdown`, so a well-formed shutdown has no
  take in flight. Closing the window instead would require either a lock on every
  adopted take or deferring teardown until samples drain, and the second contradicts the
  clear-on-final-shutdown contract above and its three pins. The window is SHUTDOWN-ONLY
  and BOUNDED, pinned by `the_lost_registration_window_is_shutdown_only_and_bounded`.
  Diagnostics: `adopt-take summary` with the
  stable fields `takes=` `adopted=` `released=` `outstanding=` `fallbacks=`
  `budget_refusals=` at destroy (per subscription) and shutdown (aggregate),
  beside `log_hook_counters`; the bench citability gate reads them
  (`adopted == takes`, `fallbacks == 0`, `outstanding == 0`).
- **The copy floor is structural for everything that is not a primitive sequence.**
  Strings and arrays of nested messages copy on take by wire design (their wire shape
  is not the C object), so there is no "skip unflatten when the ROS 2 layout matches the
  wire" path to take: the ROS 2 and wire layouts differ for exactly these members, and
  the two loan gates above are the only copy-free ones. For
  those members the codec's byte movement, not the transport, dominates the bridge's
  cost.
- **RX zero-init elision.** `assign_prim_sequence` allocates with `malloc`, not
  `calloc`; every byte is immediately overwritten, so a zero-fill would be pure waste.
  The C++ side fills `uint8` sequences through the shim's single-pass
  `rmw_cerulion_vector_u8_assign` (`std::vector<uint8_t>::assign`), gated by the Rust
  caller to exactly UNBOUNDED, non-bool, dynamic `uint8` sequences
  (`is_unbounded_u8_vector`: `type_id_ == UINT8 && !is_upper_bound_`). `int8`, `octet`
  (`vector<std::byte>`, layout-compatible but NOT type-`uint8`, kept off the path so the
  boundary stays exactly UINT8), `bool`, BOUNDED `uint8` and every multi-byte primitive
  keep the resize+copy path; reinterpreting any of them as `vector<uint8_t>` is UB.
- **`vector<bool>` is bit-packed** on the C++ side and is coded element-by-element
  through the container function pointers, never through a byte-block fast path.
- **Hostile counts are rejected BEFORE allocation.** Wire-controlled lengths and
  sequence counts are validated against the remaining frame budget and
  `MAX_FRAME_BYTES` before any resize/assign/`calloc`; rejection is loud (latched),
  never a silent truncation. `bridge_test` and `cpp_bridge_test` pin this with
  corrupt-size and over-bound fixtures.
- **Canonical element framing.** Variable arrays of nested messages and `string[]`
  bodies are built through the shared `cerulion_core::codegen::element_codec` (the
  same routine the DDS-attach ingress path uses), so every producer in the system
  emits one framing the `FrameWalker` can decode. `canonical_element_body_test`
  cross-validates it the only way that can catch a convention fork: rmw encodes, the
  INDEPENDENT walker decodes, and both are asserted against hand-built byte oracles.
  A symmetric encode/decode round-trip on the same bridge is self-consistent no matter
  what convention it invents, so it can never pin the wire format; write oracle-byte
  tests for framing changes.
- **Paired rollout.** Element-body bytes carry no version signal and the schema hash
  does not cover framing, so an rmw publisher and the desk-side walker must deploy
  together. A skew in either direction degrades nested arrays to the loud opaque-text
  fallback (`NestedArrayOpaque`), never a wrong decode.

## Serialized-message path

`rmw_serialize` produces a wire frame stamped with sequence 0, so
`rmw_publish_serialized_message` re-stamps the publisher's own commit sequence at
publish; a bag replayed through it would otherwise publish sequence 0 forever. A
rejected frame puts nothing on the wire and burns no sequence.

There are exactly TWO reject gates, and it is worth being precise about what the second
one does NOT cover: the header must PARSE (`WireHeader::read_from_buf`), and its
`schema_hash` must match the publisher's. `read_from_buf` checks only that the buffer
holds the 32-byte header; it does not validate `total_size` against the buffer length.
So a buffer shorter than the header is rejected under the malformed-header arm, but a
frame carrying a VALID header and a payload truncated relative to `total_size` is
**published**. Truncation is not an independently detected reject condition; do not
rely on this path to catch a short payload.

An unfilled serialized message (NULL buffer, or non-null with zero length) is handled
as a latched malformed-header REJECT, not a silent early return: both shapes are the
same caller mistake and both must be loud, with `buffer_len` and `header_size`
diagnostics on every head. The empty slice is hand-built because
`slice::from_raw_parts` over a NULL pointer is UB even at length zero.

## Failure reporting (flood latches)

High-frequency failure paths follow the repo-wide latch convention (root `AGENTS.md`)
with rmw-specific additions:

- **Reporters.** `decode_failure_latch` (take-side framing failures, per
  `DecodeSite`: subscription / service request / service response),
  `publish_reject_latch` (publish-side rejects on `rmw_publish_serialized_message`:
  malformed header and hash mismatch, one latch EACH) and `loan_refusal_latch`
  (a loaned take refused on a budget, the shadow pool or the transport's borrow
  budget; `kind=` names which) are reporters over `cerulion_core`'s shared
  `FailureRegimeLatch`. Subscriptions additionally keep a hash-mismatch latch at the
  `rmw_take` site.
- **Vocabulary is side-specific.** Take-side lines say a frame was DROPPED; publish
  lines say the caller's frame was REJECTED (it never reached the topic, and the
  caller is told synchronously via `RMW_RET_INVALID_ARGUMENT`). Take-side wording on
  the publish path would point a bag-player operator at the wrong end of their
  problem.
- **Latch separateness.** A subscription's hash-mismatch latch and decode-failure
  latch are deliberately independent: type disagreement and framing disagreement have
  different remedies, and one open regime must not swallow the other's loud head.
  Pinned by `an_open_decode_regime_does_not_swallow_the_hash_mismatch_head`.
- **Decade re-announcement.** The counters sit behind the opaque handles of the
  standardized rmw C ABI; no accessor can exist, so the log is a ROS user's ONLY
  window. An open regime re-announces loudly at each power of ten of the running
  total (clock-free, log-bounded). Pinned at the production take site by
  `an_open_decode_regime_re_announces_at_the_decade_at_error`.
- **Keys.** The running total logs as `total_failures=` at every latch consumer; the
  per-site field key is `topic=` for messages and `service=` for services, never a
  generic `name=`; operators grep by key. Which condition a line reports rides the
  message text and `kind=`.
- **Counters are unconditional** (`SubscriptionData::hash_mismatches`,
  `PublisherData::{malformed_header_rejects, schema_hash_rejects}`): they bump on
  every failure regardless of log regime and are never reset by recovery.

## QoS contracts

- **TRANSIENT_LOCAL ceiling raise.** A TRANSIENT_LOCAL publisher provisions the
  subscriber buffer ceiling up by the `(history * 4).div_ceil(3)` rule in
  `src/api/pubsub.rs`, so its history request is satisfiable under the transport's
  history-vs-buffer check without touching that (correct) core warning. Endpoint info
  reports the depth Cerulion actually PROVISIONED (the endpoint's real retention), never
  an echo of the ask: a TRANSIENT_LOCAL depth-1000 publisher reports the 16 frames it
  keeps, a VOLATILE one reports 0, and a subscription reports the queue depth it really
  got. The other three QoS axes still carry the requested values, and a create whose depth
  was genuinely clamped warns LOUDLY at create time naming BOTH numbers (a VOLATILE 0 is
  not a clamp and stays quiet). `rmw_{publisher,subscription}_get_actual_qos` returns a
  constant profile and does NOT inspect the endpoint, so it does not reflect the
  provisioned depth. Late-joiner delivery itself rides the `rmw_wait` event pump.
- **Endpoint info by topic.** `rmw_get_{publishers,subscriptions}_info_by_topic` serves
  the process-LOCAL registry (the rviz2 / `ros2 topic info -v` surface); cross-process
  endpoint discovery is not implemented. Out-arrays are filled through the
  caller's rcutils allocator, and every allocation-failure path rolls back cleanly
  (`RMW_RET_BAD_ALLOC`, zeroed out-params, every prior allocation freed exactly once).

## Test map

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `tests/rmw_e2e_test.rs` | Full rmw C ABI over iceoryx2: pub/sub + wait, loaned zero-copy BOTH directions (publish-side borrow + the loaned take: hold-across-publish, exact borrow-budget 4/5 boundary, destroy-with-outstanding-loan, variable-type UNSUPPORTED control), services, guard conditions, variable messages, TRANSIENT_LOCAL late-joiner pump (+ VOLATILE negative control) | yes; `#[serial]` + `--test-threads=1` (SHM singleton, global guard registry) | none |
| `tests/rmw_wait_event_test.rs` | The event-driven `rmw_wait`: blocked-wait publish wake + take oracle with cumulative `fd_wakes` (a sleep-poll variant scores 0), exact multi-subscription ready set, THE guard-doorbell re-fire pin (second wait: full timeout + `fd_wakes` delta exactly 0; skip any drain and the residual ring byte fires the block instantly), zero-timeout never blocks (`fd_blocks == 0` × 100), services/clients wake via their own listeners (per-side cumulative `fd_wakes`), TRANSIENT_LOCAL delivery while the only waiter is PARKED (the ≤20 ms block-cap pin), the `CERULION_RMW_EVENT_WAIT=off` kill-switch (`fd_blocks == 0` AND `spin_probes == 0`), no-entity bounded sleep, the spin front catching an imminent publish (cumulative `spin_wakes`/`spin_probes`), and the DEGRADED arm: a sticky listener-drain fault (`fault_inject_drain_events_err_for_test`) with a readable-but-not-ready fd must time out by sleep pacing with `fd_blocks == 0` / `fd_wakes == 0` / `degraded_waits` counted, then heal on the next call | yes; `#[serial]` + `--test-threads=1` (SHM singleton; env knobs) | none |
| `tests/bridge_test.rs` | C introspection codec: hand-built rosidl fixtures, native-reader interop, padding determinism, hostile-count rejection, complex roundtrips | no | none |
| `tests/cpp_bridge_test.rs` | C++ codec + shim: real `std::string`, bit-packed `vector<bool>`, resize spy, over-bound UB guard, padding poison, corrupt-size caps, cpp↔native byte identity | no (its `#[serial]` arms self-serialize) | none |
| `tests/canonical_element_body_test.rs` | Variable-element bodies match the canonical framing: rmw encodes, independent `FrameWalker` decodes, vs hand-built byte oracles | no | none |
| `tests/rmw_endpoint_info_test.rs` | Endpoint info by topic: field round-trips, gid cross-check, QoS (three axes echoed, depth reported as PROVISIONED), null-arg guards, OOM rollback (countdown allocator + pointer ledger), remove-by-gid exactness | yes | none |
| `tests/rmw_transient_local_ceiling_test.rs` | Ceiling raise + provisioned-depth reporting (clamp, VOLATILE-0, deeper-service read-back) + create-time clamp warn + pump delivery + positive warn control | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_schema_mismatch_test.rs` | Take-side hash-mismatch flood latch at `rmw_take`; latch separateness; decode-latch decade re-announcement | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_publish_reject_test.rs` | `rmw_publish_serialized_message` e2e: both reject latches, sequence re-stamp, unfilled-message reject, field-key pins | yes; own binary (every test `#[traced_test]`) | none |
| `tests/forged_take_bridge_test.rs` | Forged loaned take, bridge half: the eligibility oracle (string + unbounded sequence IS take-loanable; string-only, bounded, `bool`, defaulted and no-`init`/`fini` types are not), the forge aiming into a hand-built frame with `capacity == size`, un-forge to `{NULL, 0, 0}` / the null triplet, `fini` exactly once, misaligned / ragged targets refused all-or-nothing, the data-floor placement (an entry AT the floor forged; one BELOW it (in the fixed section, in the table) copied, counted, outside the mask, freed by `fini`, on both bridges), determinism | no (the counter-reading arms are `#[serial]` within the binary) | none |
| `tests/rmw_shadow_take_test.rs` | Forged loaned take over real iceoryx2: `data()` INSIDE the held sample (C++ Image shape with a Header, C LaserScan shape), `capacity == size`, un-forge on return, shadow reuse with zero Rust-heap allocations, pool exhaustion refused before a frame is consumed + recovery, string-only UNSUPPORTED, fixed path unchanged, destroy with a forged loan outstanding, determinism, misaligned wire target refused + counted, a below-floor entry on the wire served by copy + counted + the shadow retired (the natural frame's first entry sits exactly AT the floor and forges), the read-only fault in a child process that must first PROVE it reached the forged write (non-null loan, `data()` inside the held sample, a marker flushed before the write) | yes; own binary (`#[global_allocator]` probe + child spawn) | none |
| `tests/rmw_loan_refusal_latch_test.rs` | Both loan-refusal arms with their per-kind headlines: loud once, `debug!` repeats, decade re-announcement, one recovery carrying the suppressed count, unconditional counter, healthy-consumer control; the forge-FALLBACK latch (below-floor frames served by copy: `warn!` once, repeats at `debug!`, recovery, counter, never through the refusal latch) | yes; own binary (`#[traced_test]`) | none |
| `tests/borrow_seal_bridge_test.rs` | The borrow-window SEAL, bridge half: the borrow-window SEAL on both bridges over hand-built typesupports + a hand-laid slot (no hook; the seal is pure arithmetic once the window numbers are handed in): adopted whole-payload byte oracles round-tripped through `unflatten`, the fini-safety rule (in-slot headers emptied; the fixture's `fini` freeing a slot byte is the crash detector), escapees (heap + misaligned-in-window) copied element-aligned, gap-verbatim below the cursor + zeroed slivers above it, the two-sided gap-budget threshold, pre-mutation refusals (struct intact for the fallback flatten; every original field asserted, not just one), determinism (ONE `SealScratch` reused across both seals, the reset-not-reallocate contract) | no (counter-reading arms `#[serial]` within) | none |
| `tests/rmw_borrow_publish_test.rs` | The borrow-window publish, C-ABI half over real iceoryx2 with a FAKE hook installed through the `test-seams` seam: borrow→(simulated bump)fill→publish→raw-take with hand byte oracles; adopted-vs-copied discriminated by the Principle-#3 counters; escaped copy correctness; windowless second borrow (a live sibling window is never disarmed); return + retire-on-reuse; hookless degrade (borrow UNSUPPORTED, plain publish untouched); wrong-thread copy + orphan-hold + self-heal; the hook-counters consumer read; destroy leaks a cross-thread-armed slot (counter + untouched owner window); the budget-exhausted borrow self-heals by releasing its orphan BEFORE the loan (anti-vacuity: a fourth held borrow really fails); a POISONED publisher still leaks armed slots at destroy: the held loan deliberately OWN-thread, so ignoring the poisoned flag (which would fini + release it) is caught here; counted, the armed window PRESERVED (no disarm rides torn tables), and the poison-safe diagnostic accessor answers mid-poison; the own-thread-ORPHAN mirror arm pins the orphan drain's disjunct the same way; a wrong-thread RETURN moves only `borrow_wrong_thread_return_count`, never the publish-degrade latch; the registry consequence: after every leaking destroy the transport's live port count stays `Some(1)` (the leaked publisher's port is still REGISTERED; a deregistering drop would read `Some(0)`), and the cross-thread arm pins the documented 2-slot cost (with the topic's other slot free, one same-topic re-create succeeds; a third publisher is NULL; a clean destroy releases only its own slot) | yes; `--test-threads=1` (SHM singleton + process-global fake hook) | none |
| `tests/rmw_leak_at_destroy_registry_test.rs` | The registry consequence, CROSS-PROCESS half: a leaking destroy must leave the process's iceoryx2 node RECLAIMABLE. Self-re-exec (crib of `cdylib_iox2_log_level_test`): the parent mints a unique iceoryx2 root + prefix under `/tmp/iceoryx2/<pid>_<nanos>/`, spawns this binary as a child that initialises the transport singleton on that config (`TransportManager::init_with_config`, the singleton seam `runtime()` adopts; `Arc::ptr_eq`-asserted, so the leak can never land on the desk's real root), runs the real C-ABI cross-thread leak sequence (fake hook, borrow on a spawned thread, wrong-thread return, destroy), REPORTS the live port count, and `process::exit`s with its ports live; the parent asserts the dead node's directory holds a `<prefix>*.port_tag` (precondition: the arm ran), runs `Node::try_cleanup_dead_nodes` exactly as `cerulion clean` does, and requires `cleanups == 1 && failed_cleanups == 0`, no tag, no node directory, plus the child's `count == 1`. TWO independent oracles listed by ONE failure: reverting to drop the `PublisherData` again reads `count 0`, `failed_cleanups 1`, the orphan tag and the node directory surviving. The root is removed in a `Drop` guard | yes; `--test-threads=1` (a full rmw runtime in the child; process-global fake hook + rmw singleton) | none |
| `tests/rmw_borrow_zero_alloc_test.rs` | The borrow-window publish, the steady-state ZERO-ALLOC pin: own binary (`#[global_allocator]` = a thread-scoped counting allocator counting `alloc`+`alloc_zeroed`+`realloc`), fake hook + the unbounded fixture; after warmup, the counter is open across each `rmw_publish_loaned_message` and must read 0 over 64 cycles (adopted-counter anti-vacuity + a counter-bites `Box` control); plus the structural loan-bookkeeping reserve pin on a COLD publisher (`loan_bookkeeping_capacities` ≥ the loan budget; the borrow half cannot carry a counter pin: the typesupport `init` heap-allocates by design) | yes; `--test-threads=1` (SHM singleton + process-global fake hook + process-global allocator) | none |
| `tests/rmw_borrow_window_linux_test.rs` | The REAL `LD_PRELOAD` arm: the rmw test binary re-execs itself with `libcerulion_heaphook.so` preloaded: the genuine dlsym handshake resolves Active, a real interposed `malloc` fill bumps into the armed window, the publish adopts (counters + in-tail entry placement + byte oracle); the no-preload control child degrades (borrow UNSUPPORTED) | `#[ignore]`; needs Linux with glibc (a container works) | `cargo build -p cerulion_heaphook` (the cdylib) under the same profile |
| `tests/rmw_adopt_take_test.rs` | Adopt-take over real iceoryx2 with a FAKE hook carrying a real SEGMENT REGISTRY (`test-seams`): pointer identity (`data` IS the registered range start, exact byte length, `capacity == size`) + liveness (bytes survive a later publish while held) + the copy control; the empty-entry skip; the gate matrix (env+hook+forgeable arms; hook-no-env, fixed-type, and env-no-hook all stay `None`; the witness is load-bearing) with the once-latched warn counter; registration-failure rollback (sibling unregistered, byte-equal copy, nothing leaked, latched); budget refuse/self-heal (`taken=false`, never an error); destroy-with-outstanding LEAVES registrations + later frees release; reverse-order + interior-pointer frees release by cookie; the C++ twin through the real libstdc++ shim; subprocess stderr pins (the once-warn level-matched + the destroy/shutdown `adopt-take summary` proof lines with the six stable fields); the pre-write entry gate (an unresolvable offset and a ragged length each leave the caller's message byte-identical, with the decode-failure counter proving the frame ARRIVED, against a well-formed control that changes it); the borrow-ceiling non-arming (plus its warn, and an ordinary-service control that still arms); the borrow refusal attributed to LOANS rather than adopted messages | yes; `--test-threads=1` (SHM singleton + process-global fake hook + env gate) | none |
| `tests/rmw_adopt_take_linux_test.rs` | Adopt-take's REAL-`LD_PRELOAD` arms: adopted take → real interposed `free` → release callback → borrow slot observably freed (budget refusal recovers); adopt → `fork()` → the fork child's inherited free is a counted quarantine NO-OP (cleared callback, counter kind 3), the parent's free still releases | `#[ignore]`; needs Linux with glibc (a container works) | `cargo build -p cerulion_heaphook` (the cdylib) under the same profile |
| `tests/rmw_adopt_zero_alloc_test.rs` | Adopt-take's steady-state Rust-heap allocation contract under a counting global allocator: an ADOPTED take costs EXACTLY ONE allocation, the `Arc<AdoptedSample>`, which is STRUCTURAL (the registration cookies are clones that outlive the call on whatever thread the app frees from), while the refused and copy-served paths are pinned against their own oracles. Its own binary because `#[global_allocator]` is process-wide, the same discipline as `rmw_borrow_zero_alloc_test`. | yes; own binary, `--test-threads=1` (process-wide allocator counter + the process-global fake hook) | none |
| `tests/rclpy_xproc_test.rs` | Cross-process: real `python3` rclpy process ↔ native transport over one SHM root, both directions, the only dlopen coverage | `#[ignore]`, container-only | ROS 2 Jazzy container (`tools/ros2_toolchain/Dockerfile`), `.so` staged in an ament prefix |
| `tests/rmw_cpp_bypass_warn_test.rs` | The `test-seams` C++ bypass is LOUD: two calls to `era::emit_cpp_bypass_warn` produce two `WARN` lines (deliberately unlatched, registration-cadence), each carrying the deploy ban and `built_for`; `WARN` as a whole header token (never a substring), and no other loud line in the capture | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_cpp_bridge_refusal_test.rs` | The C++ bridge REFUSAL seam both resolvers call (`CppBridgeGate::emit_refusal`): each refusing verdict emits exactly one `ERROR` line whose message is the `{SCREAMING_CONST}` capture of its OWN paragraph (cross-checked against `refusal_message()`; a const swap in the seam dies here), the `verdict=` field as a whole token, `built_for`; `Supported` emits nothing and returns `false`, with a same-capture re-arm so the zero is not a blind capture; the line BODY (after the `<ts> LEVEL span: target: ` header) must equal the paragraph + `built_for=` + `verdict=` EXACTLY, and no other loud line | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_era_guard_test.rs` | The load-time era guard HOISTED to the first entry points that touch caller memory: under an armed jazzy-vs-kilted contradiction, `rmw_init_options_init` / `_copy` / `_fini` return the refusal and leave a POISONED caller buffer byte-untouched (identifier field nulled so an unguarded path would write), `rmw_init` still refuses a hand-stamped static struct before its first context write (defense in depth), each with exactly one `ERROR` line carrying `entry=`, `baked_ros_distro=`, `runtime_ros_distro=`; the happy path (build's own claim + an agreeing override) really writes and logs nothing; the body must equal `DISTRO_MISMATCH_REFUSAL` + its four fields EXACTLY, every arm rejects any other ERROR/WARN/INFO line (DEBUG/TRACE never counted, release-profile class), pass 1 of the happy path runs with NO override against the build's OWN claim (the env set so `runtime` is `Some`), and the `EraGuardPanicGuard` seam proves a panic inside the guard (which runs under `ffi_guard` in all three `options_*` exports) degrades to `RMW_RET_ERROR` with the caller's bytes untouched (an unguarded variant ABORTS the test process); a mismatched era plus a FOREIGN identifier ⇒ `rmw_init` refuses the mismatch (not `RMW_RET_INCORRECT_RMW_IMPLEMENTATION`), with an unarmed control; pass 1's runtime is DERIVED from the build's own claim (`era:<token>` ⇒ the first concrete member of era_check's own table, never a literal) and a shape oracle pins that every bakeable claim admits its first-pass runtime; READ-detecting fixture: the caller struct is an `mmap`'d PROT_NONE page handed to each of the four exports in a re-exec'd child (the `rmw_shadow_take_test` pattern): under a MISMATCH the child exits `40 + RMW_RET_ERROR` (any pre-guard read faults ⇒ signal ⇒ fail), under an AGREEING era the same page must kill the child by SIGSEGV/SIGBUS (the fixture provably faults on first touch); every fixture is BUILT under a runtime the build's own claim ADMITS (`initialized_options` names `agreeing_runtime_for(baked_distro())` for the duration of its call; a Foxy runtime is not admitted by a vendored build's claim, so no fixture here can be built under one; the lib test builds its fixture with `ROS_DISTRO` unset, which every claim shape admits), the env RAII is the ONE `test_seams::EnvVarGuard` (one `var_os` snapshot site shared with the lib tests; a non-UTF-8 prior is restored byte-for-byte, and drop ENFORCES LIFO nesting rather than silently mis-restoring), and the probe child refuses an env value it cannot read LOUDLY rather than exiting 0 having probed nothing | yes; own binary (`#[traced_test]` + `#[serial]`: override + `ROS_DISTRO` are process-global) | none |
| lib unit tests (`--lib`) | `ffi_guard` panic conversion; `DecodeSite::field_name` oracle; the take paths' pure decision oracles (`take_gate`'s schema-hash verdict and per-entry verdict tables, `BorrowHolders::classify` + its per-holder kind/remedy/whitespace pins, `adopt_viability`'s both-sided boundary) and the two bridges' pre-write entry walks; reporters emit the field key their site promises; the era guard's lib pins: `era`/`era_check` classifier oracle vectors (claim-vs-fingerprint verdicts, the era/layout-identity admission tables and their shape, the capability fingerprint this build actually carries, `rcl_error_text`'s NUL + backslash rendering) and `test_seams`'s `EnvVarGuard` pins (a non-UTF-8 prior restored byte-for-byte; an out-of-order drop refused rather than mis-restoring) | `--test-threads=1`; **two reasons**: the traced-subscriber slot AND **seven `#[serial]` env-mutating tests** (`ROS_DISTRO` / `CERULION_RMW_EXPECT_BINDINGS` are process-global; one of them drives the real `rmw_init_options_init` + `rmw_init` C ABI under a hand-armed contradiction). Still no shared memory | none |
| `tests/rmw_canonical_names_test.rs` | ROS names ARE Cerulion canonical names verbatim (an rmw publisher on `/chatter` is the service `/chatter/data`, read by a plain core subscriber, a DELIVERY pin, with the slash-stripped spelling as a negative control) + network-egress registration | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_heaphook_counter_log_test.rs` | The OPERATOR half of the hook counter shutdown line: a kind the hook does not serve renders `unavailable`, never the raw `u64::MAX` sentinel dressed as a real count (the data half is `rmw_borrow_publish_test`'s stale-v2 arm, whose binary installs no subscriber) | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_slice_ceiling_e2e_test.rs` | The per-type slice ceiling GOVERNS the negotiated iceoryx2 buffer on the rmw path: the oracle is the LOAN VERDICT (an over-ceiling `rmw_publish` fails, an under-ceiling one delivers), which the pre-ceiling blanket would have passed | yes; own binary (`#[traced_test]`) | none |
| `tests/rmw_wait_pingpong_discriminator_test.rs` | The ping-pong discriminator for the event-driven wait: two wait sets on two threads in the bench's exact shape, spin ON and OFF, per-round wake-mode tally + RTT distribution | yes; `--test-threads=1` and `--nocapture`; the same-core Linux arm is `--ignored` | none |
| `tests/rmw_wait_spin_test.rs` | `rmw_wait` spin behaviour with the shared spin knob set to `u64::MAX`: the clamped budget keeps zero-timeout waits immediate, mid-wait wakes landing, and no core pinned past the cap | yes; `#[serial]` + `--test-threads=1` (process env) | none |

Traced-suite conventions: `#[traced_test]` installs the process-global tracing
subscriber, so any suite using it needs its own test binary (a sibling test that brings
the runtime up first takes the slot). Assert structured fields as whole-whitespace
`key=value` tokens, never bare substrings (prose in a message satisfies a substring
check; `suppressed_count=5` substring-matches `suppressed_count=50`); parse log levels
as tokens out of the line header, since the span (test-function) name is rendered into
every captured line.

## Known coverage gaps

- `rmw_deserialize` has no test coverage anywhere, including its null-buffer guard;
  add pins with any change that touches it.
- Endpoint discovery is process-local only (see QoS contracts above).
