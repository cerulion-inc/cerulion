// SPDX-License-Identifier: AGPL-3.0-only
//! Procedural macros for defining Cerulion node types.
//!
//! A node crate does not depend on this crate directly. `cerulion_core`
//! re-exports all three macros: the two attribute macros arrive with
//! `use cerulion_core::prelude::*;`, and the derive is
//! `cerulion_core::state::CerulionState`.
//!
//! | Macro | Goes on | What it declares |
//! |---|---|---|
//! | [`macro@cerulion_node`] | the node struct | the node TYPE: its trigger policy, and its ports through `#[input]` / `#[output]` field attributes |
//! | [`macro@cerulion_node_impl`] | the `impl` block beside it | the node's behaviour: `tick`, optional `init` and `shutdown`, `external_source`, `#[on_event]` handlers |
//! | [`CerulionState`](derive@CerulionState) | a type a node HOLDS | that the type can be captured and restored, so a recording can put the node back the way it was |
//!
//! One node type is one crate at `nodes/<type>/` in a Cerulion workspace. It is
//! created by `cerulion node create <type>` and built by
//! `cerulion node build <type>`, and the folder name is the type name a graph
//! file refers to. Instances of the type, their topic wiring and their
//! deployment live in `graphs/<name>.yaml`, never in code: a node author
//! writes no `main`, builds no graph in code and never starts the runtime.
//!
//! # Example
//!
//! `nodes/sensor/src/lib.rs`, complete. This is what
//! `cerulion node create sensor --policy period_ms=100 -o geometry_msgs/Vector3 reading`
//! writes, with the `tick` body filled in:
//!
//! ```text
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::geometry_msgs::Vector3;
//!
//! #[cerulion_node(period_ms = 100)]
//! #[derive(Default)]
//! struct SensorNode {
//!     #[output]
//!     reading: Vector3,
//!     tick_count: u32,
//! }
//!
//! #[cerulion_node_impl]
//! impl SensorNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.tick_count += 1;
//!         // A fixed field is written by assignment, straight into the
//!         // loaned shared-memory slot. The frame is published when `tick`
//!         // returns `Ok`; a tick that returns `Err` publishes nothing.
//!         self.reading.x = f64::from(self.tick_count);
//!         Ok(())
//!     }
//! }
//! ```
//!
//! This crate has no dependency on `cerulion_core` or
//! `native_ros2_messages`, so the examples in its docs are shown as text and
//! are not compiled here. Each one is compiled and run as a doctest in
//! `cerulion_core`: this one on that crate's front page, the others in its
//! `prelude` and `state` modules.

// `unsafe_code = "deny"` is the ONE crate-local lint this crate
// carries beyond the shared table. Cargo refuses to mix `[lints] workspace =
// true` with crate-local lint keys, so the rule lives here — same strength,
// same crate, and visible in the source it governs. A proc-macro
// crate has no business writing `unsafe`: it emits tokens.
#![deny(unsafe_code)]
// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod codegen;
// MUST stay private (`mod`, not `pub mod`): a `proc-macro = true` crate cannot
// export any item other than its macros — `pub mod determinism;` is a HARD
// COMPILE ERROR here ("proc-macro crate types currently cannot export any items
// other than functions tagged with #[proc_macro...]"). So the banned-symbol
// table is the macro half's INTERNAL source of truth; the deferred core
// half cannot `use cerulion_macros::determinism::BANNED` and instead consumes a
// copy relocated to a shared non-proc-macro crate in part 2. See the
// `determinism.rs` module header.
mod determinism;
mod impl_macro;
mod parse;
mod registry;
mod state_derive;
mod validate;

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

/// Derive `CerulionState`: the state-capture machinery for a type a node
/// holds.
///
/// A node struct does NOT need this: `#[cerulion_node]` emits the same impl
/// itself, so an ordinary node costs zero new lines.
/// This derive is for the types a node holds (the `Pose`, the `PadState`, the
/// `SampleQueue`), and it is the fix the compiler already names when a field
/// cannot be captured.
///
/// ```text
/// use cerulion_core::state::CerulionState;
///
/// #[derive(CerulionState)]
/// struct Pose { x: f64, y: f64 }
/// ```
///
/// (Shown as text because this crate does not depend on `cerulion_core`. The
/// derive is compiled and run as a doctest in `cerulion_core::state`.)
///
/// # Per-field escapes, for the exceptional field only
///
/// | Attribute | Meaning |
/// |---|---|
/// | `#[cerulion(reconstruct)]` | a HANDLE: not captured, and left untouched on restore. For something that must be rebuilt rather than restored (a connection, a device) |
/// | `#[cerulion(serde)]` | capture through the field's own `Serialize`/`Deserialize` |
/// | `#[cerulion(unordered)]` | a hash-like container whose key has no total order |
///
/// A resource type the framework already recognises (`File`, `TcpStream`,
/// `JoinHandle`, a `Box<dyn Trait>`, `Arc<TransportManager>`) is treated as
/// `reconstruct` WITHOUT the attribute. An unknown key is a compile error, not
/// a silently ignored one.
#[proc_macro_derive(CerulionState, attributes(cerulion))]
pub fn derive_cerulion_state(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);
    state_derive::derive(&input).into()
}

/// Prepare the user's struct for re-emission on an ERROR path.
///
/// Both error arms below re-emit the user's struct so a validation failure
/// does not also produce "cannot find type" everywhere the node is named.
/// Two things on that struct are addressed to a macro that is not going
/// to run, so left in place they add errors BELOW the real diagnostic:
///
/// 1. **`#[cerulion(...)]` field escapes.** Nothing registers an inert helper
///    attribute for an ATTRIBUTE macro (only a `#[proc_macro_derive(..,
///    attributes(cerulion))]` does that, and a node goes through no derive),
///    so each one reaches rustc as `cannot find attribute 'cerulion' in this
///    scope`. Pinned by `tests/ui/node_escape_on_a_failing_node_does_not_cascade.rs`.
/// 2. **An explicit `#[derive(CerulionState)]`.** The success path drops it
///    (`codegen::strip_state_derive`, beside the `compile_error!` that names
///    the real fix), and the error arms must too — otherwise a node that BOTH fails
///    validation and carries the redundant derive runs the derive over its
///    `#[input]`/`#[output]` ports and produces (MEASURED) a `E0277: 'Vector3' cannot be
///    part of a Cerulion node's state` whose help text says "add
///    `#[derive(CerulionState)]` to `Vector3`" — advice that is actively
///    wrong for a zero-sized SHM marker, stacked on top of the diagnostic the
///    user actually needed. The strip is silent here rather than a second
///    `compile_error!`: this path exists to surface the REAL error, and the
///    redundant derive is still reported in full on the next compile, once
///    the validation failure is fixed and the success path runs. Pinned by
///    `tests/ui/node_state_derive_on_a_failing_node_does_not_cascade.rs`.
///
/// Deliberately scoped to those two. `#[input]`/`#[output]` cascade the same
/// way (`tests/ui/input_unknown_attribute.stderr` records
/// it), but that snapshot is a rustc RENDERING and re-blessing the whole
/// `tests/ui/` set to tidy that cascade is exactly the toolchain-drift
/// risk the `#[ignore]`d group exists for. Closing the two above
/// costs no snapshot at all.
fn salvage_struct_for_error_path(input: &DeriveInput) -> DeriveInput {
    let mut out = input.clone();
    codegen::strip_state_derive(&mut out.attrs);
    if let syn::Data::Struct(ref mut data) = out.data {
        for field in data.fields.iter_mut() {
            field.attrs.retain(|a| !a.path().is_ident("cerulion"));
        }
    }
    out
}

/// Declares a Cerulion node type. Goes on the node struct.
///
/// The struct's ports are the fields that carry `#[input(...)]` or
/// `#[output(...)]`; at least one is required. A port field's type is the
/// message type (for example `native_ros2_messages::sensor_msgs::LaserScan`)
/// and its name is the port name a graph file wires. Every other field is the
/// node's own state. The struct is paired with an `impl` block carrying
/// [`macro@cerulion_node_impl`], which holds `tick`.
///
/// `#[derive(Default)]` is the documented shape and what the scaffold writes.
/// If the struct does not derive `Default`, the macro derives it.
///
/// ```text
/// use cerulion_core::prelude::*;
/// use native_ros2_messages::geometry_msgs::Vector3;
/// use native_ros2_messages::sensor_msgs::LaserScan;
///
/// #[cerulion_node]
/// #[derive(Default)]
/// struct SafetyControllerNode {
///     #[input(trigger, depth = 1, expect_within_ms = 100)]
///     scan: LaserScan,
///     #[output(promise_within_ms = 100)]
///     linear_velocity: Vector3,
/// }
///
/// #[cerulion_node_impl]
/// impl SafetyControllerNode {
///     fn tick(&mut self) -> Result<(), NodeError> {
///         let ranges = self.scan.ranges();
///         let stop = ranges.is_empty() || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
///         self.linear_velocity.x = if stop { 0.0 } else { 0.3 };
///         Ok(())
///     }
/// }
/// ```
///
/// (Compiled and run as a doctest in `cerulion_core::prelude`.)
///
/// # Node attributes
///
/// This is the complete set. Any other key is a compile error that lists
/// these eight.
///
/// **Trigger policy: what makes the node fire.** A node needs exactly one
/// source of policy: one of the first four attributes, or exactly one
/// `#[input(trigger)]` field and no policy attribute. The policy is part of
/// the node TYPE. A graph file carries no policy.
///
/// | Attribute | The node fires |
/// |---|---|
/// | `period_ms = N` | every `N` milliseconds |
/// | `sync_window_ms = N` | once per complete aligned set: every `#[input(trigger)]` field has a message, and the set's timestamps lie within `N` ms of each other |
/// | `unbounded_sync` | once per complete set: as soon as every `#[input(trigger)]` field has an unconsumed message, with no timing bound. Not recommended for control loops: the worst-case fire latency is the slowest publisher's interval, and unbounded if that publisher stops |
/// | `external` | when its own `external_source()` signals (a device fd or a blocking SDK call). See [`macro@cerulion_node_impl`] |
/// | none of these | each time a message arrives on the ONE field marked `#[input(trigger)]` |
///
/// Under both sync policies each trigger message is consumed by at most one
/// set, so a burst holding several complete sets gives one fire per set, in
/// order, and each tick reads its own set's members. Only
/// `#[input(trigger)]` fields are aligned; a plain `#[input]` on a sync node
/// is a latest-value read that never gates the fire.
///
/// **Execution limits.** These stack with any trigger policy.
///
/// | Attribute | Meaning |
/// |---|---|
/// | `tick_within_ms = N` | Tick deadline. A `tick` that runs longer than `N` ms is counted (`tick_within_missed_count`) and logged at `warn`. It is not interrupted |
/// | `throttle_ms = N` | Rate cap. The tick is deferred while fewer than `N` ms have passed since the node last fired |
///
/// **Determinism opt-outs.** Bare flags.
///
/// | Attribute | Meaning |
/// |---|---|
/// | `allow_non_deterministic` | Turns off the determinism check of [`macro@cerulion_node_impl`] for this node. A recording of such a node cannot be re-executed byte for byte |
/// | `uses_live_io` | Declares that the node performs live IO and opts it out of the check's IO-class rows only. The one IO-class row (`std::fs::read_dir`) is warn-class, and the macro reports deny-class rows only, so this flag marks the node and changes no diagnostic |
///
/// # What is rejected at compile time
///
/// - No source of policy: no policy attribute and no `#[input(trigger)]`
///   field. A node with outputs only therefore needs `period_ms` or
///   `external`.
/// - `#[input(trigger)]` together with `period_ms` or `external`.
/// - More than one of `period_ms`, `sync_window_ms`, `unbounded_sync`,
///   `external`.
/// - Two or more `#[input(trigger)]` fields with neither `sync_window_ms` nor
///   `unbounded_sync`.
/// - `sync_window_ms` or `unbounded_sync` on a node with no
///   `#[input(trigger)]` field.
/// - `throttle_ms` together with `period_ms` (the period already fixes the
///   rate).
/// - A zero value for `period_ms`, `sync_window_ms`, `tick_within_ms`,
///   `throttle_ms`, `expect_within_ms` or `promise_within_ms`.
/// - `depth` outside `1..=64`.
/// - A duplicate port name, a struct with no `#[input]` / `#[output]` field,
///   and the macro on anything that is not a struct.
/// - `type_name`, `inputs(...)` and `outputs(...)` as macro arguments. The
///   node type is the folder name `nodes/<type>/`, and ports are field
///   attributes.
///
/// One combination is accepted here and degraded at graph build instead:
/// `sync_window_ms` or `unbounded_sync` on a node with exactly ONE
/// `#[input(trigger)]` field compiles. When the graph is built the runtime
/// logs a `warn` naming the node, and the node fires as a data trigger on
/// that one input.
///
/// A graph build also checks what no macro can see: every `#[input(trigger)]`
/// port of a sync node must be wired in the graph file, and `depth` is
/// range-checked again when the graph loads.
///
/// # `#[input(...)]` field attribute
///
/// Declares an input port. The complete set of keys, all optional; an unknown
/// key is a compile error naming the supported set.
///
/// | Key | Meaning |
/// |---|---|
/// | `trigger` | A message arriving on this input fires the node (one trigger field), or takes part in set alignment (a sync node) |
/// | `depth = N` | Queue depth in messages, `1..=64`. Default 10. Every unit of depth reserves a full message-sized shared-memory slot, so prefer a backpressure policy to a deep queue |
/// | `backpressure = drop_oldest` | Default. When the queue is full the oldest message is reclaimed |
/// | `backpressure = sample(N)` | Admit at most one message per `N` ms, by the message's own timestamp |
/// | `backpressure = block` | Lossless: the producer's tick is deferred while this consumer's queue is full. Installed only when EVERY consumer of the topic declares `block`; on a mixed topic this input is degraded to `drop_oldest`, with a `warn` at graph build |
/// | `expect_within_ms = N` | Arrival watchdog. If `N` ms pass with no new message, `expect_within_missed_count` increments and a `warn` is logged. Independent of `trigger` |
///
/// An input that is not a trigger is a latest-value read: `tick` sees the most
/// recent message, held across steps, and until the first message arrives the
/// tick body does not run at all.
///
/// # `#[output(...)]` field attribute
///
/// Declares an output port. Two forms, and nothing else is accepted:
///
/// | Form | Meaning |
/// |---|---|
/// | `#[output]` | The form for every message type. There is no field list: assignment in `tick` resolves fixed and variable-length fields at compile time |
/// | `#[output(promise_within_ms = N)]` | Publish watchdog. If `N` ms pass without a publish, `promise_within_missed_count` increments and a `warn` is logged |
///
/// # Internals: what the macro generates
///
/// None of the following is node-author API. The `cerulion` CLI and the
/// runtime consume it; a node author never names or calls any of it, and it
/// can change between releases.
///
/// For a struct `FooNode` the macro emits a `FooNodeEntry` wrapper that
/// implements the runtime's `NodeEntry` trait, the exported symbols a built
/// node library is loaded through (behind the node crate's `cdylib` feature,
/// which the scaffolded `Cargo.toml` turns on), and
/// `impl CerulionState for FooNode`, described next.
///
/// # This macro OWNS the node's `CerulionState` impl
///
/// The fold-in means an ordinary node is capturable with zero new user
/// lines, over exactly its NON-PORT fields. Two consequences a user meets
/// in the compiler:
///
/// - Writing `#[derive(CerulionState)]` on a node as well is REFUSED in our
///   own words, in either attribute order, naming the fix, and through an
///   ALIASED import of this macro (`use ... cerulion_node as my_node;`) too.
///   A proc macro resolves no names, so the alias defeats every name match;
///   what catches it is SHAPE, in the one order where a name cannot: a node
///   must declare at least one `#[input]`/`#[output]` field, and when the
///   derive is written ABOVE the attribute those field attributes are still
///   present, because the attribute macro that strips them has not run yet.
///   The one spelling still left to rustc is a renamed DERIVE
///   (`use CerulionState as Capturable`) with the attribute written FIRST:
///   there the attribute macro holds the derive list and cannot resolve
///   `Capturable`, and a derive list carries no shape to read instead. That
///   is E0119, below.
/// - Writing a hand-rolled `impl CerulionState for FooNode` beside the node
///   is rustc's `E0119` ("conflicting implementations"), pointing at this
///   attribute as the other impl. An attribute macro is handed only its own
///   item, so a sibling `impl` block is not something it can see: there is
///   no earlier or friendlier place to catch this, and E0119 already names
///   both sites. It is not the supported way to customise what a node
///   captures: **the per-field escapes are** (`#[cerulion(reconstruct)]`,
///   `#[cerulion(serde)]`, `#[cerulion(unordered)]`), and they are read by
///   the fold-in from the same place the derive reads them. A hand impl could
///   not do better anyway: it cannot name the hidden `__cer_rt` field this
///   macro injects, and a node's port types are zero-sized SHM markers that
///   carry no `CerulionState`.
///
/// # The impl block
///
/// This macro is always paired with [`macro@cerulion_node_impl`] on an
/// `impl <Struct> { ... }` block in the same file, below the struct. That
/// block must define `fn tick(&mut self) -> Result<(), NodeError>`; see the
/// `cerulion_node_impl` docs for the rest of its contract.
#[proc_macro_attribute]
pub fn cerulion_node(attr: TokenStream, item: TokenStream) -> TokenStream {
    let node_attr = parse_macro_input!(attr as parse::NodeAttr);
    let input = parse_macro_input!(item as DeriveInput);

    // Extract field-level #[input] / #[output] attributes
    let field_attrs = match parse::extract_field_attrs(&input) {
        Ok(fa) => fa,
        Err(e) => {
            let error_tokens = e.to_compile_error();
            let salvaged = salvage_struct_for_error_path(&input);
            return TokenStream::from(quote! {
                #salvaged
                #error_tokens
            });
        }
    };

    // Validate
    let errors = validate::validate(&node_attr, &field_attrs);
    if !errors.is_empty() {
        let mut combined = errors[0].clone();
        for e in &errors[1..] {
            combined.combine(e.clone());
        }
        let error_tokens = combined.to_compile_error();
        // Emit original struct to prevent cascading "cannot find type" errors
        let salvaged = salvage_struct_for_error_path(&input);
        return TokenStream::from(quote! {
            #salvaged
            #error_tokens
        });
    }

    codegen::generate(&node_attr, &field_attrs, &input).into()
}

/// Declares a node's behaviour. Goes on the `impl` block of a
/// [`macro@cerulion_node`] struct.
///
/// Takes no arguments: the port names and types come from the struct. The
/// struct must appear above the impl block in the same file, and a block
/// whose struct cannot be found is a compile error.
///
/// # Methods the block may define
///
/// | Method | Required | Contract |
/// |---|---|---|
/// | `fn tick(&mut self) -> Result<(), NodeError>` | yes | Runs on every fire. A block without `tick` is a compile error |
/// | `fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError>` | no | Runs once before the first tick. Read configuration here with `ctx.env(..)` and `ctx.env_str(..)` |
/// | `fn shutdown(&mut self) -> Result<(), NodeError>` | no | Runs once after the last tick |
/// | `fn external_source(&mut self) -> ExternalSource` | only on an `external` node | Queried once when the live loop starts. Missing on an `external` node, present on any other node, or written with a different signature: each is a compile error |
/// | a method carrying `#[on_event(input = "port")]` or `#[on_event(output = "port")]` | no | An event handler taking `&mut self` and one event parameter. The parameter TYPE routes it: `BackpressureEvent`, `ExpectWithinEvent` and `LivelinessEvent` go on an input, `PromiseWithinEvent` on an output. Handlers run after the `tick` body, only when it returned `Ok`, in source order. An unknown port, a duplicate handler for one port and event type, or a handler on `tick` / `init` / `shutdown` is a compile error |
/// | any other method | no | A helper. A helper that writes a port field must return `Result`, because every port write is fallible |
///
/// Method names that start with `__cer_` are reserved and rejected.
///
/// ```text
/// use cerulion_core::prelude::*;
/// use native_ros2_messages::geometry_msgs::Vector3;
///
/// #[cerulion_node]
/// #[derive(Default)]
/// struct GainNode {
///     #[input(trigger, backpressure = sample(10))]
///     value_in: Vector3,
///     #[output]
///     value_out: Vector3,
///     gain: f64,
///     decimated: u64,
/// }
///
/// #[cerulion_node_impl]
/// impl GainNode {
///     fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
///         // Read from the snapshot of the environment taken when the graph
///         // was built, never from live `std::env`, so a replay reads the
///         // same value.
///         self.gain = ctx.env("GAIN", 2.0);
///         Ok(())
///     }
///
///     fn tick(&mut self) -> Result<(), NodeError> {
///         self.value_out.x = self.value_in.x * self.gain;
///         Ok(())
///     }
///
///     #[on_event(input = "value_in")]
///     fn on_value_in_backpressure(&mut self, _event: BackpressureEvent) {
///         self.decimated += 1;
///     }
/// }
/// ```
///
/// (Compiled and run as a doctest in `cerulion_core::prelude`.)
///
/// # Inside `tick`
///
/// Every `self.<port>` access in the block is rewritten so that it reads or
/// writes the message in shared memory directly. There is no intermediate
/// copy of the message.
///
/// | You write | Effect |
/// |---|---|
/// | `self.<input>.<fixed_field>` | reads the field from shared memory |
/// | `self.<input>.<variable_field>()` | variable-length fields are read through an accessor method (`self.scan.ranges()` is a `&[f32]`; a string field is a `Result<&str, WireError>`) |
/// | `self.<output>.<field> = expr;` | writes the field. One rule for fixed and variable-length fields; a variable-length field costs one copy from `expr` |
/// | `self.<output>.<variable_field>.fill_from(producer)?;` | the producer writes straight into the loaned region, with no copy |
/// | `self.<output>.<nested>.<leaf> = expr;` | writes a leaf of a nested message, at any depth |
/// | `self.<output>.emit()?;` | publishes a message type that has no fields |
/// | `self.now_ns()` | the time of the clock the graph runs on. The replay-safe clock read |
/// | `self.request_shutdown()` | asks the runtime to drain and exit |
///
/// An output the tick never touches is not loaned and publishes nothing. An
/// output that was written publishes when `tick` returns `Ok`; a tick that
/// returns `Err` publishes nothing. A message type with variable-length
/// fields must have every one of them written in the tick, or the frame is
/// discarded with an `error` log.
///
/// A port access inside another macro's arguments (`tracing::debug!`,
/// `format!`) is not rewritten and is a compile error: bind it to a local
/// first.
///
/// # The determinism check
///
/// A recording can be re-executed byte for byte only if the node reads
/// nothing that differs between runs. This macro walks every method in the
/// block and refuses to compile a call to `Instant::now()`,
/// `SystemTime::now()` or `thread::spawn(...)`, matched on the last two path
/// segments. Use `self.now_ns()` for time. The opt-outs are
/// `allow_non_deterministic` and `uses_live_io` on [`macro@cerulion_node`].
#[proc_macro_attribute]
pub fn cerulion_node_impl(
    attr: proc_macro::TokenStream,
    item: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    impl_macro::cerulion_node_impl(attr, item)
}
