# native_ros2_messages: internals

The vendored ROS 2 `.msg` corpus and the build-time code generator that turns it into
zero-copy `ShmMessage` types. Two contracts live here: the **build pipeline** (how
`.msg` text becomes generated Rust, deterministically) and the **upstream drift gate**
(how the corpus is held true to the ROS 2 distros it was vendored from).

## Build pipeline (`build.rs`)

`src/lib.rs` contains no types; it is `include!(concat!(env!("OUT_DIR"), "/ros2_msgs.rs"))`
plus the embedded-registry include. All generated code lives in `OUT_DIR`; the edit
unit is a `.msg` file under `msg/<pkg>/`, and `cargo build` regenerates everything.

Pipeline order (each stage feeds the next; the order is load-bearing):

1. **Scan** `msg/<pkg>/*.msg` into per-package sets. An unreadable or non-UTF-8 file
   panics the build; a vendored input that silently vanished would let references
   to it rebind through the bare-name fallback.
2. **Emit `BUILTIN_MSGS`** (`msg_registry.rs`): the exact vendored `.msg` text frozen
   at build time, one `(package, name, text)` tuple in sorted order. CLI schema
   introspection parses THIS text (not the `msg/` tree at CLI runtime), so a
   resolved layout can never skew from what the generated types compiled against.
3. **Parse** each message via `parse_rosmsg` with its package name. The package is
   part of the schema-hash input, so identically named messages in different
   packages get distinct wire hashes. A parse failure panics.
4. **`resolve_fixed_nested` over the flattened whole-corpus view**, BEFORE any code
   generation. This decides which nested references inline into the parent's fixed
   section (staying zero-copy) versus ride the variable section. Any resolver
   warning panics: an unresolved or ambiguous reference silently changes the
   parent's WIRE LAYOUT while the schema hash stays equal, a wire-skew hazard,
   never a cosmetic one.
5. **`generate_schema`** per schema, emitting three items per message: the unit
   marker (`pub struct <Name>;` carrying `impl ShmMessage`), the SHM-backed accessor
   `<Name>Shm`, and the heap-owned `<Name>Snapshot` replay/round-trip companion.
6. **Module assembly**: one file per package plus `ros2_msgs.rs` re-exports.

Cross-package `use` edges are derived by `collect_field_deps`, which mirrors the
runtime resolver's lookup precedence exactly (qualified > same-package > bare
`Header` → `std_msgs` iff `std_msgs/Header` exists in the schema set > global
bare-name fallback), so the dependency graph and layout resolution can never
disagree about which package an unqualified name binds to. Bare-name collisions
across packages get a loud `cargo:warning`; an unqualified reference to a collided
name is ambiguous.

**Determinism.** Every collection in `build.rs` is a `BTreeMap`/`BTreeSet` or
explicitly sorted, so generated output is byte-identical across builds. Never swap
in a `HashMap`.

**Failure policy.** `build.rs` panics, never skips, on parse errors, unreadable
files, and resolver warnings. Treat a build panic as a bug in the `.msg` input, not
flaky tooling.

**Stale out-dir trap.** After schema changes, many
`target/debug/build/native_ros2_messages-*/out/` directories with DIFFERENT
generated code coexist. The only ground truth for a generated constant is a probe
compiled against the linked crate (`<T as ShmMessage>::CONST`), never any file
found under `target/`.

## Why drift matters (consequence model)

Cerulion resolves layout by schema hash over the field list. Where the vendored
text disagrees with a stock robot's, the hashes disagree and `walk_by_hash` refuses
at the hash gate before framing is consulted; the topic renders nothing and the
user is told nothing. The same property makes hash-affecting corpus changes a
**paired rollout**: nothing on the wire signals which spelling a peer uses, so a
mixed-version desk↔robot pair renders nothing on affected types. Deploy both sides
in lockstep. A framing skew degrades to a loud opaque-text fallback; a hash skew is
silent.

## Upstream drift gate (`tests/upstream_drift_test.rs`)

The ONLY test comparing the vendored corpus against an EXTERNAL truth. The sibling
tests (`layout_equivalence_test`, `schema_hash_pin_test`) diff the vendored tree or
its generated constants against themselves, so a self-consistent transcription
error passes them. Hermetic by design (CI has no network and no ROS install), so
the external truth is checked in: `crates/native_ros2_messages/upstream_msg_manifest.txt`,
the normalized upstream signature of every vendored message.

### Manifest semantics

- One `=<pkg>/<Name>` block per vendored message: `f <type> <name>` field lines in
  declaration order (order IS layout) and `c <type> <NAME>=<val>` constant lines
  sorted by name (position carries no meaning).
- **Provenance**: one distro pin per package (`!source` lines), taken from
  rosdistro's `<distro>/distribution.yaml`. Most packages pin a distro branch, a
  MOVING ref, so the manifest's git history, not the `!source` line, dates a
  branch-pinned signature. A few pin an immutable ros2-gbp `release/...` tag. The
  `autoware_*` pair has no distro pin and is recorded `UNVERIFIED`. The trust token
  is a closed parsed set (`DistroBranch`/`ReleaseTag`/`Unverified`), and
  `DECLARED_UNVERIFIED_PACKAGES` pins WHICH packages may be unverified; flipping
  any other package to unverified fails loudly instead of riding a manifest diff.
- The distro pins are written twice (the refresh script's clone table and the
  manifest's `!source` lines), and
  `refresh_script_clone_refs_match_the_manifest_source_pins` cross-checks them in
  CI, so a bump on one side cannot leave the other claiming an upstream the
  signatures did not come from.

### Waiver semantics

- `!accept` waives CONSTANTS ONLY. The field list is always compared, so a
  wire-affecting divergence under an `!accept` still fails.
- `!accept-fields` is the only directive that can silence field drift, and it is
  REQUIRED for a message absent from the pinned upstream: absence compares
  nothing, fields included, so it must be an explicit, loud judgement.
- `every_live_waiver_is_constants_only_and_its_fields_match_upstream` asserts every
  shipping `!accept` really is constants-only with matching field halves.
- `!not-vendored` is not part of the vocabulary; a legacy line fails at parse with
  migration guidance. The orphaned-waiver remedy is re-running the refresh, which
  deletes the stale block itself.

### Fails closed

No path reports "cannot verify" as success. The gate fails on: a missing,
unparseable, or empty manifest; a vendored message with no manifest entry; a
manifest entry whose file vanished; a STALE waiver that no longer describes a real
divergence; an ORPHANED waiver naming a message that is not vendored.

### What it cannot catch, and the refresh

Upstream itself changing after the snapshot needs network; that is
`tools/scripts/refresh_upstream_msg_manifest.sh`'s job. Its output is a reviewable
manifest diff: the corpus is immutable-without-review in CI, and adopting an
upstream change is an explicit act. The refresh rewrites ONLY the signature blocks,
preserves the hand-authored `!source`/`!accept`/`!accept-fields` lines verbatim,
and REFUSES to run without an existing manifest (a silent regeneration would delete
every human judgement while reporting success). Remediation ladder, in order:
follow upstream → re-pin the distro → constants-only `!accept` → `!accept-fields`
last, with its cost stated. Never steer a user first toward the mechanism that
hides a fork.

### Normalizer

ONE normalizer serves both comparison sides and is itself oracle-tested rather than
trusted for self-agreement. Field types come from the real `parse_rosmsg`,
inheriting its alias collapsing: `byte`/`char` → `uint8`, `wstring` → `string`,
`time`/`duration` → `builtin_interfaces/*`, bounded `T[<=N]` → `T[]` (a bound is an
upper limit, not a layout fact); a genuine signedness change, a fixed-length
array, a reordering, a missing field, and a missing or retyped constant all still
differ. Same-package nested refs normalize to the QUALIFIED form on both sides, a
collapse the gate is structurally blind to, deliberately paired with the
zero-tolerance bare-ref gate below rather than trusted.

### Known residual

`control_msgs/PidState` and `control_msgs/SteeringControllerStatus` diverge from
Jazzy in field names (hash-affecting; the vendored files follow the package's
later-distro pin). A stock Jazzy robot hash-skews on those two: the rmw path
refuses at `walk_by_hash`; the raw DDS-attach path misdecodes `PidState` rather
than refusing. The gate cannot flag this; it compares against the pinned distro by
design. Remediation: resolve the schema FROM the robot via the wire-service
schema-acquisition rung. The residual is recorded in the manifest header.

## Bare nested refs: zero tolerance

The vendored corpus is uniformly QUALIFIED (`pkg/Type`) for nested references. Bare
and qualified refs hash differently, nothing in the pipeline normalizes them
(`resolve_fixed_nested`'s rewrite mutates only the fixed flag), and `rmw_cerulion`
always emits the qualified rosidl-introspection form, so a bare ref in the corpus
hashes differently from what an rmw publisher produces (`UnknownSchemaHash`, silent
no-render). `the_vendored_corpus_declares_no_bare_nested_refs` fails loudly on any
bare ref with the remediation spelled out; there is deliberately NO waiver mechanism.

## Wire-hash external pin

`schema_hash_pin_test::vendored_hashes_match_the_hashes_a_live_rmw_publisher_puts_on_the_wire`
is the only place a vendored hash is checked against an external MEASURED truth
(hashes captured from a live rmw publisher's wire frames). Every other assertion in
that file is a self-check, since `build.rs` and the test run the same
parse→resolve→hash recipe. Its failure message forbids re-blessing from the actual
value: a failure there is a wire-compatibility break. The two gates are
deliberately separate: the bare-ref gate catches the CAUSE on any type; the wire
pin catches the CONSEQUENCE on the measured set.

## Adding or changing a `.msg`: checklist

1. Edit/add the file under `msg/<pkg>/`; `cargo build -p native_ros2_messages`.
   A build panic is a bug in the new `.msg`.
2. Extend `layout_equivalence_test`: bump the hard corpus-count assert, add an
   `assert_layout_matches!` line, and add an `offset_of!` spot-check for complex
   fixed layouts.
3. Add a `roundtrip_fixed!`/`roundtrip_var!` case in `roundtrip_test`.
4. Add the manifest entry: run the refresh script and review the diff. The drift
   gate fails closed on a missing entry.
5. Variable schemas: check the `MAX_SLICE_LEN` tier; `max_slice_len_test`
   spot-checks representative schemas only, and a new schema falls through to the
   user-defined catch-all tier unless given an explicit arm.
6. If the change is hash-affecting, plan the paired desk+robot rollout.
7. Run the FULL crate suite (`cargo test -p native_ros2_messages`); a curated
   local run masks the exhaustive tests.

## Test map

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `layout_equivalence_test.rs` | Runtime `LayoutResolver` layout == compile-time `#[repr(C)]` layout, exhaustively over every vendored schema (a hard corpus-count assert guards the exhaustive claim); `offset_of!` spot-checks on nested/padding/large-array layouts | no | none |
| `roundtrip_test.rs` | Every schema round-trips `<Name>Snapshot` → SHM publish → `try_view` snapshot, over an isolated iceoryx2 `TestTransport` per test | no | none |
| `schema_hash_pin_test.rs` | Generated `SCHEMA_HASH`/`WIRE_FIXED_SIZE` == values recomputed through the same parse→resolve pipeline (whole-corpus resolve is required: a recursively-fixed nested field folds the target's hash into the parent); plus the wire-capture hash pin (never re-bless) | no | none |
| `upstream_drift_test.rs` | Corpus-vs-manifest adjudication (a pure `adjudicate` function driven by synthetic-input oracles), provenance cross-check, waiver scoping, the bare-ref zero-tolerance gate, and fail-closed behavior | no | none |
| `vendored_types_test.rs` | Acceptance for later-vendored packages (vision/radar/grid_map + the UUID dep): real loan/fill round-trips over `TestTransport`, pairwise-distinct hashes including bare-name twins across packages | no | none |
| `generated_bindings_test.rs` | Package-qualified schema-hash linkage, rosidl scalar-default parity, and bool-array wire semantics, against the real generated bindings | no | none |
| `max_slice_len_test.rs` | `MAX_SLICE_LEN` lands on the expected tier for representative schemas (NOT exhaustive; see checklist step 5) | no | none |
| `stamp_helper_test.rs` | `builtin_interfaces/Time::from_ns` against hand-computed oracles; a seconds count past the `i32` horizon fires a `debug_assert!` in debug/test builds | no | none |

All binaries are parallel-safe within this crate: the transport-using tests build
isolated per-test SHM roots via `TestTransport`; the rest are pure file-parse or
compute. CI runs plain `cargo test -p native_ros2_messages` on every PR.
