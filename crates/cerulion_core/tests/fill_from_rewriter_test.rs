// SPDX-License-Identifier: AGPL-3.0-only
//! Positive (no-false-positive) tests for the schema-blind
//! `SelfPortRewriter` — `fill_from` method calls + `=` assignment.
//!
//! Since the uniform-shim rewrite, the rewriter carries no per-port variable-field tables:
//! `self.<port>.<field>.fill_from(src)` rewrites uniformly to
//! `__cer_<port>.__cer_fill_from_<field>(src)` and `self.<port>.<field> =
//! expr` to `__cer_<port>.__cer_assign_<field>(&expr)?`; the codegen-emitted
//! shims resolve simple-vs-complex (`fill_from_<f>` vs `fill_from_<f>_bytes`)
//! at compile time. Misuse shapes (`[i]`, `+=`) are diagnosed by the
//! generated write-only proxy's gated operator impls, not by the macro.
//!
//! This file must compile cleanly, proving the rewriter doesn't
//! false-positive on legitimate code:
//! - Fixed-section field writes on a variable schema (`self.image.height
//!   = 1;`) — now ALSO shimmed (`__cer_assign_height`), behavior unchanged.
//! - Helper-arg pass-through on a fixed schema (`self.helper(...)`) —
//!   threads ports correctly.
//! - Variable-field `=` rewrite (`self.image.encoding = "rgb";`).
//! - `self.image.data.fill_from(closure)?` — dispatches through the
//!   uniform `__cer_fill_from_data` shim.
//! - Mixed-shape tick body — both `=` and `fill_from` in one node.

use cerulion_core::graph::node::NodeEntry;
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

// ============================================================
// Node 1: fill_from-only — proves the new rewriter case
// ============================================================

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct FillFromOnlyNode {
    #[output]
    image: Image,
    frame_count: u32,
}

#[cerulion_node_impl]
impl FillFromOnlyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame_count += 1;
        // Fixed-section writes (unchanged path).
        self.image.height = self.frame_count;
        self.image.width = 1;
        self.image.step = 4;

        // NEW: fill_from rewrite — closure variant.
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..4].copy_from_slice(&[1, 2, 3, 4]);
            Ok(4)
        })?;

        // Existing `=` rewrites for the other two variable fields
        // so the publish gate (all_variables_written) clears.
        self.image.encoding = "rgba";
        // Complex variable (header is a Nested Header type) — `=`
        // rewrites to `set_header_bytes(...)` on the complex-variable path.
        self.image.header = &[][..];

        Ok(())
    }
}

#[test]
fn fill_from_only_node_compiles_and_runs() {
    // Compile-coverage: if any rewriter case false-positives, this won't
    // even build. info() lets us also assert metadata in passing.
    let entry = FillFromOnlyNodeEntry::new();
    let info = entry.info().expect("info should parse");
    assert_eq!(info.output_names(), &["image".to_string()]);
}

// ============================================================
// Node 2: SliceSource via fill_from
// ============================================================

#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct FillFromSliceSourceNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl FillFromSliceSourceNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // SliceSource — exercises the wrapper type through the
        // `fill_from` rewriter.
        self.image.data.fill_from(SliceSource::new(b"hello"))?;
        self.image.encoding = "rgb8";
        self.image.header = &[][..];
        Ok(())
    }
}

#[test]
fn fill_from_with_slice_source_compiles() {
    let _entry = FillFromSliceSourceNodeEntry::new();
}

// ============================================================
// Node 3: mixed `=` and `fill_from` in same tick
// ============================================================

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct MixedShapeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl MixedShapeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Different rewrite paths for different fields in one tick:
        // - encoding via `=` (the existing rewrite path)
        // - data via fill_from (the `fill_from` rewrite path)
        // - header via `=` complex-variable path (the existing rewrite,
        //   `set_header_bytes` under the hood)
        self.image.encoding = "mono16";
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..2].copy_from_slice(&[0xAB, 0xCD]);
            Ok(2)
        })?;
        self.image.header = &[][..];
        Ok(())
    }
}

#[test]
fn mixed_shape_node_compiles() {
    let _entry = MixedShapeNodeEntry::new();
}

// ============================================================
// Node 3b: fill_from on a COMPLEX variable field (`_bytes` suffix path)
// ============================================================
//
// Every other fill_from positive test exercises SIMPLE variable fields
// (data: Bytes). This node calls `self.image.header.fill_from(...)` where
// `header` is a complex (Nested) variable field — since the uniform-shim rewrite the macro
// emits the uniform `__cer_fill_from_header(...)`, and CODEGEN's shim
// delegates to `fill_from_header_bytes(...)` (the simple-vs-complex
// resolution moved from the macro tables into the generated type).
//
// If codegen's `__cer_fill_from_header` shim delegated
// to the wrong target (or were not emitted), this test fails to compile
// with "no method `__cer_fill_from_header` found".

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct FillFromComplexFieldNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl FillFromComplexFieldNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.data = &[][..];
        self.image.encoding = "g";
        // The KEY assertion: fill_from on a complex variable field
        // must route through `__cer_fill_from_header`, whose codegen
        // body delegates to `fill_from_header_bytes`.
        self.image.header.fill_from(|buf: &mut [u8]| {
            buf[..3].copy_from_slice(b"hdr");
            Ok(3)
        })?;
        Ok(())
    }
}

#[test]
fn fill_from_on_complex_variable_field_compiles() {
    let _entry = FillFromComplexFieldNodeEntry::new();
}

// ============================================================
// Node 3c: fill_from inside a HELPER method (not just tick)
// ============================================================
//
// The rewriter applies to helpers too; this
// test exercises fill_from via a helper. If the helper-rewriter
// construction ever desynchronized from the tick-rewriter (e.g., an
// empty `output_idents` set), the fill_from detection would silently
// no-op in helpers and fall back to rustc's generic
// "no method `fill_from`" diagnostic. This test pins the contract.

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct FillFromInHelperNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl FillFromInHelperNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // tick delegates to the helper. The `fill_from` rewriter must
        // detect `self.image.data.fill_from(...)` INSIDE the helper
        // body and rewrite it identically to the tick-body path.
        self.image.encoding = "g";
        self.image.header = &[][..];
        self.populate_data()?;
        Ok(())
    }

    fn populate_data(&mut self) -> Result<(), NodeError> {
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..2].copy_from_slice(b"hp");
            Ok(2)
        })?;
        Ok(())
    }
}

#[test]
fn fill_from_in_helper_body_compiles() {
    let _entry = FillFromInHelperNodeEntry::new();
}

// ============================================================
// Node 3d: parenthesized variable-field access — Paren/Group peeling
// ============================================================
//
// `classify_port_field` peels `Expr::Paren`
// and `Expr::Group` so `(self.image.data).fill_from(...)` and
// `(self.image.data) = ...` are recognized as port-field accesses.
// This node exercises both peeled paths so the contract is pinned.
//
// Removing the Paren peel in the classifier makes
// the parenthesized fill_from call rewrite-miss, falling through to
// the leaf rewrite — `__cer_image.data.fill_from(...)` hits the
// write-only proxy ZST (no `fill_from` method). Test fails to compile.

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ParenVarFieldNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl ParenVarFieldNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Parenthesized fill_from receiver — exercises Paren peeling
        // in case 1 (success path).
        (self.image.data).fill_from(|buf: &mut [u8]| {
            buf[..1].copy_from_slice(b"x");
            Ok(1)
        })?;
        // Parenthesized `=` LHS — exercises Paren peeling in case 2
        // (existing `=` rewrite path).
        (self.image.encoding) = "p";
        self.image.header = &[][..];
        Ok(())
    }
}

#[test]
fn paren_wrapped_var_field_access_compiles() {
    let _entry = ParenVarFieldNodeEntry::new();
}

// ============================================================
// Node 3e: fill_from chained with .map_err(...)? — pins the
// outer-call composition contract
// ============================================================
//
// `self.image.data.fill_from(closure).map_err(...)?` must compile
// and work — the `fill_from` rewriter rewrites the inner MethodCall to
// `__cer_image.fill_from_data(closure)` (no trailing `?`), and the
// outer `.map_err(...)?` chain composes naturally on the resulting
// `Result<(), TransportError>`.
//
// If the rewriter incorrectly added a trailing `?`
// to fill_from, the outer `.map_err(...)?` would be `Result::map_err
// on ()` which fails to compile. The rewriter deliberately emits no
// trailing `?`; this test pins it.

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct FillFromMapErrChainNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl FillFromMapErrChainNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.encoding = "g";
        self.image.header = &[][..];
        // The load-bearing chain — outer .map_err composes on the
        // fill_from result; the `?` propagates the user-mapped error.
        self.image
            .data
            .fill_from(|buf: &mut [u8]| {
                buf[..3].copy_from_slice(b"abc");
                Ok(3)
            })
            .map_err(|e| NodeError::Logic(format!("fill_from chain: {e}")))?;
        Ok(())
    }
}

#[test]
fn fill_from_chained_with_map_err_compiles() {
    let _entry = FillFromMapErrChainNodeEntry::new();
}

// ============================================================
// Node 4: pure-fixed schema (Vector3) + helper-arg path
// ============================================================
//
// Vector3 is a truly fixed schema (3 × f64). Writes (`self.out.x = ..`)
// route through the uniform `__cer_assign_x` shim; reads (`self.cmd.x`)
// go through the `self.<port>` → `__cer_<port>` leaf rewrite + Deref to
// the FixedSection — the rewriter must not disturb either.

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct PureFixedAndHelpersNode {
    #[input]
    cmd: Vector3,
    #[output]
    out: Vector3,
    multiplier: f64,
}

#[cerulion_node_impl]
impl PureFixedAndHelpersNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Helper that references the input port — exercises helper-arg
        // injection alongside the new failure-shape detection.
        let scaled = self.compute_speed();
        // Fixed-field writes — routed through the uniform assign shims
        // (`__cer_assign_x(&scaled)?` etc.), which write via direct
        // assignment on the fixed overlay.
        self.out.x = scaled;
        self.out.y = self.cmd.y;
        self.out.z = 0.0;
        Ok(())
    }

    fn compute_speed(&mut self) -> f64 {
        // Helper READS `self.cmd.x` (an input port) — reads are never
        // shimmed; the leaf rewrite + Deref serves the value.
        self.cmd.x * self.multiplier
    }
}

#[test]
fn pure_fixed_schema_and_helpers_compile_unchanged() {
    let _entry = PureFixedAndHelpersNodeEntry::new();
}

// ============================================================
// Node 5: `=` shim and `fill_from` shim coexist on one field
// ============================================================
//
// The uniform `__cer_assign_<f>` and `__cer_fill_from_<f>` shims are
// separate generated methods; a tick that first assigns and then
// overwrites the SAME variable field via fill_from must compile and use
// the last write (matching `set_<f>` + `fill_from_<f>` re-write
// semantics — the offset entry is overwritten, orphaning the first
// write's bytes).

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct AssignThenFillFromNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl AssignThenFillFromNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.encoding = "g";
        self.image.header = &[][..];
        // First write via the assign shim...
        self.image.data = b"\x00\x00";
        // ...then overwrite via the fill_from shim.
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..2].copy_from_slice(&[0x11, 0x22]);
            Ok(2)
        })?;
        Ok(())
    }
}

#[test]
fn assign_then_fill_from_on_same_field_compiles() {
    let _entry = AssignThenFillFromNodeEntry::new();
}
