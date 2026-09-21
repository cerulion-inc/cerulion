// SPDX-License-Identifier: AGPL-3.0-only
//! Schema-blind macro surgery: codegen-emitted uniform
//! `__cer_assign_<f>` write shims + the write-only diagnostic proxy for
//! variable fields.
//!
//! Two layers:
//!
//! 1. **Behavioral** — build a real `<Name>Shm` writer directly over a byte
//!    buffer (no iceoryx2, no transport) using the same
//!    `ShmMessage::build_writer` path production uses, call the emitted
//!    `__cer_assign_<f>` shim, and read the value back through the normal
//!    reader. Oracle values are hand-known, never a self-compare. Covers
//!    fixed scalar (f64/u32/u8), bool-as-u8, fixed array, `String`, `Bytes`,
//!    typed f64 slice, complex-variable, and `Err` propagation.
//!
//! 2. **String-level** — run `generate_schema` on hand-built schemas and pin
//!    the emitted SOURCE for field kinds a compiled fixture can't easily
//!    exercise (per-kind shim signatures, the diagnostic proxy machinery, the
//!    reserved-`__cer`-prefix guard, and the reserved-collision message). The
//!    trybuild pins for the RENDERED diagnostic text are not part of this file.

use cerulion_core::codegen::{generate_schema, FieldDef, FieldType, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxPayloadCapacity;
use cerulion_core::TransportError;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::{Image, Imu, JointState};
use native_ros2_messages::std_msgs::Bool;
use std::sync::Arc;

// ============================================================
// Helpers
// ============================================================

/// A zeroed byte buffer guaranteed 8-byte aligned (backed by `Vec<u64>`).
/// `Vec<u8>`'s allocation alignment is only 1 by the type contract, but
/// f64-bearing fixed sections (`Vector3`, `Imu`) need align 8 for
/// `from_bytes_mut`'s alignment assert.
struct AlignedBuf(Vec<u64>);

impl AlignedBuf {
    fn new(n_bytes: usize) -> Self {
        Self(vec![0u64; n_bytes.div_ceil(8).max(1)])
    }

    fn bytes(&mut self) -> &mut [u8] {
        let n = self.0.len() * 8;
        // SAFETY: `Vec<u64>` is 8-aligned; all `n` bytes are initialized (zeroed).
        unsafe { std::slice::from_raw_parts_mut(self.0.as_mut_ptr() as *mut u8, n) }
    }
}

fn topic() -> Arc<str> {
    Arc::from("assign_shim_test")
}

// ============================================================
// Behavioral: FIXED fields (fixed-only schema — `<Name>Shm` overlay)
// ============================================================

#[test]
fn fixed_scalar_f64_shim_writes_through() {
    let mut ab = AlignedBuf::new(64);
    // Fixed-only schema: `build_writer` returns `&mut Vector3Shm`.
    let writer = Vector3::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(64), topic());
    writer.__cer_assign_x(&1.5f64).expect("assign x");
    writer.__cer_assign_y(&-2.25f64).expect("assign y");
    writer.__cer_assign_z(&3.0f64).expect("assign z");
    assert_eq!(writer.x, 1.5, "x written through shim");
    assert_eq!(writer.y, -2.25, "y written through shim");
    assert_eq!(writer.z, 3.0, "z written through shim");
}

#[test]
fn bool_as_u8_shim_writes_through() {
    // std_msgs/Bool stores its `bool data` field as `u8` in the SHM overlay.
    let mut ab = AlignedBuf::new(8);
    let writer = Bool::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(8), topic());
    // The shim parameter is the overlay type `u8`, not `bool`.
    writer.__cer_assign_data(&1u8).expect("assign true");
    assert_eq!(writer.data, 1, "bool-as-u8 true → 1");

    let mut ab0 = AlignedBuf::new(8);
    let w0 = Bool::build_writer(ab0.bytes(), MaxPayloadCapacity::const_new(8), topic());
    w0.__cer_assign_data(&0u8).expect("assign false");
    assert_eq!(w0.data, 0, "bool-as-u8 false → 0");
}

// ============================================================
// Behavioral: FIXED fields on a variable schema's FixedSection (via Deref)
// ============================================================

#[test]
fn fixed_section_scalar_shims_write_through_deref() {
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    // `height`/`width`/`step` are u32, `is_bigendian` is u8 — all fixed-section
    // fields on ImageFixedSection reached through DerefMut.
    writer.__cer_assign_height(&1080u32).expect("height");
    writer.__cer_assign_width(&1920u32).expect("width");
    writer.__cer_assign_step(&7680u32).expect("step");
    writer
        .__cer_assign_is_bigendian(&1u8)
        .expect("is_bigendian");
    assert_eq!(writer.height, 1080);
    assert_eq!(writer.width, 1920);
    assert_eq!(writer.step, 7680);
    assert_eq!(writer.is_bigendian, 1);
}

#[test]
fn fixed_array_shim_writes_through() {
    // sensor_msgs/Imu carries `float64[9] orientation_covariance` inline in
    // ImuFixedSection — a `FixedArray<f64, 9>` fixed-section field.
    let mut ab = AlignedBuf::new(512);
    let mut writer = Imu::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(512), topic());
    let cov: [f64; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
    writer
        .__cer_assign_orientation_covariance(&cov)
        .expect("covariance");
    assert_eq!(
        writer.orientation_covariance, cov,
        "fixed array written through shim (by-ref, deref-copied)"
    );
}

// ============================================================
// Behavioral: VARIABLE fields (delegate to set_<f> / set_<f>_bytes)
// ============================================================

#[test]
fn string_shim_writes_through() {
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    writer.__cer_assign_encoding("rgb8").expect("encoding");
    assert_eq!(writer.encoding().expect("read encoding"), "rgb8");
}

#[test]
fn bytes_shim_writes_through() {
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    writer.__cer_assign_data(&[1u8, 2, 3, 4, 5]).expect("data");
    assert_eq!(writer.data(), &[1u8, 2, 3, 4, 5]);
}

#[test]
fn typed_f64_slice_shim_writes_through() {
    // sensor_msgs/JointState `float64[] position` → DynamicArray<f64>.
    let mut ab = AlignedBuf::new(512);
    let mut writer =
        JointState::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(512), topic());
    writer
        .__cer_assign_position(&[1.0f64, 2.0, 3.0])
        .expect("position");
    assert_eq!(writer.position(), &[1.0f64, 2.0, 3.0]);
}

#[test]
fn complex_variable_shim_writes_through() {
    // Image `header` is a `std_msgs/Header` (contains a String → variable),
    // so it is a complex-variable field exposed via raw-bytes accessors; the
    // shim delegates to `set_header_bytes`.
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    writer
        .__cer_assign_header(&[9u8, 8, 7])
        .expect("header bytes");
    assert_eq!(writer.header_bytes(), &[9u8, 8, 7]);
}

// ============================================================
// Behavioral: Err propagation matches the underlying setter
// ============================================================

#[test]
fn variable_shim_propagates_over_capacity_err() {
    // Small buffer AND small ceiling; a payload larger than both cannot fit
    // and cannot spill — the underlying `set_data` returns `PayloadTooLarge`,
    // and the shim must propagate it verbatim (oracle: over-capacity →
    // PayloadTooLarge is the documented behavior, not a self-compare).
    let mut ab = AlignedBuf::new(64);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(64), topic());
    let huge = vec![7u8; 500];
    let result = writer.__cer_assign_data(&huge);
    assert!(
        matches!(result, Err(TransportError::PayloadTooLarge { .. })),
        "over-capacity data via shim must propagate PayloadTooLarge, got {result:?}"
    );
}

// ============================================================
// String-level: per-kind shim signatures on the generated source
// ============================================================

/// Build a variable schema exercising every fixed AND variable field kind.
fn signal_schema() -> MessageSchema {
    let mut s = MessageSchema::new("Sig");
    s.add_field(FieldDef::new("count", FieldType::U32)); // fixed scalar
    s.add_field(FieldDef::new("flag", FieldType::Bool)); // fixed bool → u8
    s.add_field(FieldDef::new(
        "grid",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::U8),
            length: 4,
        },
    )); // fixed array
    s.add_field(FieldDef::new("name", FieldType::String)); // variable String
    s.add_field(FieldDef::new("blob", FieldType::Bytes)); // variable Bytes
    s.add_field(FieldDef::new(
        "samples",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::F32),
        },
    )); // variable typed slice
    s.add_field(FieldDef::new(
        "child",
        FieldType::Nested {
            schema_name: "Child".to_string(),
            package: None,
            fixed: None,
        },
    )); // complex variable
    s
}

#[test]
fn fixed_field_shim_signatures_emitted() {
    let code = generate_schema(&signal_schema());
    // Fixed-section shims: by-ref param of the exact overlay type, `*v` body.
    assert!(
        code.contains("pub fn __cer_assign_count(&mut self, v: &u32)"),
        "fixed scalar shim signature"
    );
    assert!(code.contains("self.count = *v;"), "fixed scalar shim body");
    assert!(
        code.contains("pub fn __cer_assign_flag(&mut self, v: &u8)"),
        "bool-as-u8 shim takes the overlay type u8, not bool"
    );
    assert!(code.contains("self.flag = *v;"));
    assert!(
        code.contains("pub fn __cer_assign_grid(&mut self, v: &[u8; 4])"),
        "fixed array shim takes &[u8; N]"
    );
    assert!(code.contains("self.grid = *v;"));
    // Every shim returns the TransportError result and is doc-hidden plumbing.
    assert!(code.contains("-> ::std::result::Result<(), ::cerulion_core::TransportError>"));
    assert!(code.contains("#[doc(hidden)]"));
}

#[test]
fn variable_field_shim_signatures_delegate_to_setters() {
    let code = generate_schema(&signal_schema());
    assert!(
        code.contains("pub fn __cer_assign_name(&mut self, v: &str)")
            && code.contains("self.set_name(v)"),
        "String shim takes &str and delegates to set_name"
    );
    assert!(
        code.contains("pub fn __cer_assign_blob(&mut self, v: &[u8])")
            && code.contains("self.set_blob(v)"),
        "Bytes shim takes &[u8] and delegates to set_blob"
    );
    assert!(
        code.contains("pub fn __cer_assign_samples(&mut self, v: &[f32])")
            && code.contains("self.set_samples(v)"),
        "DynamicArray<f32> shim takes &[f32] and delegates to set_samples"
    );
    assert!(
        code.contains("pub fn __cer_assign_child(&mut self, v: &[u8])")
            && code.contains("self.set_child_bytes(v)"),
        "complex-variable shim takes &[u8] and delegates to set_child_bytes"
    );
}

// ============================================================
// String-level: write-only diagnostic proxy machinery
// ============================================================

#[test]
fn proxy_emitted_for_variable_fields_only() {
    let code = generate_schema(&signal_schema());
    // Proxy field on <Name>Shm for each VARIABLE field, named after the field.
    for var in ["name", "blob", "samples", "child"] {
        assert!(
            code.contains(&format!("pub {var}: __cer_wp_Sig_{var},")),
            "expected proxy field decl for variable field `{var}`"
        );
        assert!(
            code.contains(&format!("pub struct __cer_wp_Sig_{var};")),
            "expected proxy ZST for `{var}`"
        );
    }
    // NO proxy for fixed-section fields.
    for fixed in ["count", "flag", "grid"] {
        assert!(
            !code.contains(&format!("__cer_wp_Sig_{fixed}")),
            "fixed field `{fixed}` must NOT get a diagnostic proxy"
        );
    }
}

#[test]
fn proxy_diagnostic_machinery_emitted() {
    let code = generate_schema(&signal_schema());
    // Never-implemented marker trait carrying the on_unimplemented diagnostic.
    assert!(code.contains("pub trait __cer_wg_Sig_name {}"));
    assert!(code.contains("#[diagnostic::on_unimplemented(message ="));
    // Index / IndexMut / compound-assign impls: the marker bound sits on the
    // impl's GENERIC param (never on the concrete proxy — that would be an
    // unstable `trivial_bounds` case).
    assert!(code.contains(
        "impl<__CerIdx> ::std::ops::Index<__CerIdx> for __cer_wp_Sig_name where __CerIdx: __cer_wg_Sig_name"
    ));
    assert!(code.contains(
        "impl<__CerIdx> ::std::ops::IndexMut<__CerIdx> for __cer_wp_Sig_name where __CerIdx: __cer_wg_Sig_name"
    ));
    assert!(code.contains(
        "impl<__CerRhs> ::std::ops::AddAssign<__CerRhs> for __cer_wp_Sig_name where __CerRhs: __cer_wg_Sig_name"
    ));
    assert!(code.contains(
        "impl<__CerRhs> ::std::ops::SubAssign<__CerRhs> for __cer_wp_Sig_name where __CerRhs: __cer_wg_Sig_name"
    ));
    // Approved message text (real field name substituted; `<port>` generic).
    assert!(
        code.contains("self.<port>.name[i] / += are unsupported on variable-length fields"),
        "proxy diagnostic must name the field and the unsupported ops"
    );
    assert!(
        code.contains("Replace the whole value with self.<port>.name = expr")
            && code.contains("self.<port>.name.fill_from(|buf|"),
        "diagnostic must point at the two supported write forms"
    );
    // clippy `-D warnings` on the generating crate: snake_case proxy idents.
    assert!(code.contains("#[allow(non_camel_case_types)]"));
}

// ============================================================
// String-level: reserved `__cer` field-name prefix guard (part c)
// ============================================================

const RESERVED_PREFIX_MARKER: &str = "use the reserved `__cer` prefix";

#[test]
fn reserved_cer_prefix_field_rejected() {
    // A `__cer`-prefixed field name emits a compile_error at codegen entry.
    let mut s = MessageSchema::new("BadPrefix");
    s.add_field(FieldDef::new("__cerfoo", FieldType::U32));
    let code = generate_schema(&s);
    assert!(code.contains("compile_error!"), "must emit a compile_error");
    assert!(code.contains(RESERVED_PREFIX_MARKER));
    assert!(
        code.contains("__cerfoo"),
        "message names the offending field"
    );

    // The exact name `__cer` is also rejected (starts_with is inclusive).
    let mut s2 = MessageSchema::new("BadExact");
    s2.add_field(FieldDef::new("__cer", FieldType::F32));
    assert!(generate_schema(&s2).contains(RESERVED_PREFIX_MARKER));

    // A field colliding with a generated shim name (`__cer_assign_x`) is a
    // `__cer`-prefixed name, so the prefix guard catches it too.
    let mut s3 = MessageSchema::new("BadShimName");
    s3.add_field(FieldDef::new("__cer_assign_x", FieldType::U8));
    assert!(generate_schema(&s3).contains(RESERVED_PREFIX_MARKER));
}

#[test]
fn legit_single_underscore_cer_field_accepted() {
    // `cer_x` (no leading double underscore) is NOT reserved: no prefix
    // error, and the field gets a normal write shim.
    let mut s = MessageSchema::new("GoodPrefix");
    s.add_field(FieldDef::new("cer_x", FieldType::F32));
    let code = generate_schema(&s);
    assert!(
        !code.contains(RESERVED_PREFIX_MARKER),
        "`cer_x` must not trip the __cer prefix guard"
    );
    assert!(
        code.contains("pub fn __cer_assign_cer_x(&mut self, v: &f32)"),
        "the legit field still gets a write shim"
    );
}

// ============================================================
// String-level: reserved-collision message includes the shim accessor names
// ============================================================

// ============================================================
// String-level: proxy must NOT shadow `<Name>Shm<'a>` private fields
// ============================================================

#[test]
fn variable_field_named_after_private_field_gets_no_proxy() {
    // `moveit_msgs/DisplayRobotState` has a `state` field; `<Name>Shm<'a>` has
    // private fields `state`/`len`/`topic`. A proxy FIELD of the same name
    // would be a duplicate-field compile error (Rust forbids two same-named
    // fields regardless of visibility). Those variable fields must get NO
    // proxy field/type — but STILL get the `__cer_assign_<f>` write shim.
    let mut s = MessageSchema::new("Robotic");
    s.add_field(FieldDef::new("state", FieldType::String));
    s.add_field(FieldDef::new("len", FieldType::Bytes));
    s.add_field(FieldDef::new("topic", FieldType::String));
    // A non-shadowing variable field DOES still get a proxy (control).
    // NOTE: the control name must itself avoid the reserved `<Name>Shm`
    // surface — `payload` is an inherent METHOD name and trips the (correct,
    // pre-existing) reserved-surface collision compile_error.
    s.add_field(FieldDef::new("blob", FieldType::Bytes));
    let code = generate_schema(&s);

    for shadow in ["state", "len", "topic"] {
        assert!(
            !code.contains(&format!("__cer_wp_Robotic_{shadow}")),
            "variable field `{shadow}` shadows a private Shm field — must get NO proxy"
        );
        assert!(
            !code.contains(&format!("pub {shadow}: __cer_wp_Robotic_{shadow},")),
            "no proxy FIELD decl for shadowing field `{shadow}`"
        );
        // The write shim (a method, not a field) is still emitted.
        assert!(
            code.contains(&format!("pub fn __cer_assign_{shadow}(&mut self")),
            "shadowing field `{shadow}` still gets its write shim"
        );
    }
    // Control: the non-shadowing field keeps its proxy.
    assert!(code.contains("pub struct __cer_wp_Robotic_blob;"));
    assert!(code.contains("pub blob: __cer_wp_Robotic_blob,"));
    // No collision compile_error (variable fields named after private fields
    // are legal).
    assert!(!code.contains("compile_error!"));
}

#[test]
fn reserved_collision_message_lists_assign_shim_accessor() {
    // A fixed field named `cursor` collides with the `<Name>Shm` inherent
    // method `cursor()`. The variable schema triggers the collision check;
    // the emitted compile_error's static reserved-surface list now includes
    // `__cer_assign_<v>` (pins the message-text update).
    let mut s = MessageSchema::new("Coll");
    s.add_field(FieldDef::new("cursor", FieldType::U32)); // collides with inherent method
    s.add_field(FieldDef::new("name", FieldType::String)); // makes it a variable schema
    let code = generate_schema(&s);
    assert!(
        code.contains("compile_error!") && code.contains("`cursor`"),
        "collision on `cursor` must emit a compile_error"
    );
    assert!(
        code.contains("__cer_assign_<v>"),
        "collision message must list the __cer_assign_<v> accessor name"
    );
}

// ============================================================
// Uniform `__cer_fill_from_<f>` shims
// ============================================================
//
// The schema-blind macro rewrites `self.<port>.<f>.fill_from(src)` to
// `__cer_<port>.__cer_fill_from_<f>(src)`; codegen resolves the
// simple-vs-complex delegate (`fill_from_<f>` vs `fill_from_<f>_bytes`)
// inside the generated shim.

#[test]
fn fill_from_shim_bytes_field_writes_through() {
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    writer
        .__cer_fill_from_data(|dst: &mut [u8]| {
            dst[..3].copy_from_slice(&[7u8, 8, 9]);
            Ok(3)
        })
        .expect("fill_from shim (bytes)");
    assert_eq!(writer.data(), &[7u8, 8, 9]);
}

#[test]
fn fill_from_shim_complex_field_delegates_to_bytes_variant() {
    // `header` is complex — the shim must delegate to fill_from_header_bytes.
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    writer
        .__cer_fill_from_header(|dst: &mut [u8]| {
            dst[..2].copy_from_slice(&[0xAA, 0xBB]);
            Ok(2)
        })
        .expect("fill_from shim (complex)");
    assert_eq!(writer.header_bytes(), &[0xAA, 0xBB]);
}

#[test]
fn fill_from_shim_typed_f64_field_writes_through() {
    let mut ab = AlignedBuf::new(512);
    let mut writer =
        JointState::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(512), topic());
    writer
        .__cer_fill_from_position(|dst: &mut [f64]| {
            dst[..2].copy_from_slice(&[3.5, -4.5]);
            Ok(2)
        })
        .expect("fill_from shim (typed f64)");
    assert_eq!(writer.position(), &[3.5, -4.5]);
}

#[test]
fn fill_from_shim_signatures_and_delegates_emitted() {
    let code = generate_schema(&signal_schema());
    // Simple String / Bytes / typed-array delegate to fill_from_<f>.
    assert!(
        code.contains("pub fn __cer_fill_from_name<__CerS>")
            && code.contains("self.fill_from_name(src)"),
        "String fill_from shim must delegate to fill_from_name"
    );
    assert!(
        code.contains("pub fn __cer_fill_from_blob<__CerS>")
            && code.contains("self.fill_from_blob(src)"),
        "Bytes fill_from shim must delegate to fill_from_blob"
    );
    assert!(
        code.contains("pub fn __cer_fill_from_samples<__CerS>")
            && code.contains("__CerS: ::cerulion_core::transport::fill_from::FillFrom<f32>")
            && code.contains("self.fill_from_samples(src)"),
        "typed-array fill_from shim must carry FillFrom<f32> and delegate"
    );
    // Complex delegates to the `_bytes` variant with a u8 producer.
    assert!(
        code.contains("pub fn __cer_fill_from_child<__CerS>")
            && code.contains("self.fill_from_child_bytes(src)"),
        "complex fill_from shim must delegate to fill_from_child_bytes"
    );
    // Fixed fields get NO fill_from shim.
    for fixed in ["count", "flag", "grid"] {
        assert!(
            !code.contains(&format!("__cer_fill_from_{fixed}")),
            "fixed field `{fixed}` must NOT get a fill_from shim"
        );
    }
}

// ============================================================
// #[deprecated] on the write-only proxy fields
// ============================================================
//
// The parens-typo on a variable-field READ (`self.<port>.<f>` instead of
// `self.<port>.<f>()`) would, with the proxy,
// silently bind a useless ZST. The proxy FIELD declaration carries
// `#[deprecated(note = ...)]` so the typo warns, pointing at the
// kind-correct reader. The generated constructors' own inits are wrapped
// in an `#[allow(deprecated)]` let-binding so the generated code itself
// stays warning-free; the shim METHODS carry no deprecation.

#[test]
fn proxy_fields_carry_deprecated_note_with_kind_correct_reader() {
    let code = generate_schema(&signal_schema());
    // One #[deprecated(...)] per proxy field (4 proxies in signal_schema:
    // name, blob, samples, child) — and NONE anywhere else (in particular
    // not on the __cer_assign_* / __cer_fill_from_* shim methods).
    assert_eq!(
        code.matches("#[deprecated(note = ").count(),
        4,
        "exactly one deprecated note per proxy field, none on shims"
    );
    // Simple variable field → reader is `name()`.
    assert!(
        code.contains(
            "write-only proxy for variable field `name` — reading it yields a useless \
             marker. Read with the accessor method `name()`; write with \
             `self.<port>.name = expr` or `.fill_from(...)`."
        ),
        "simple-field note must name the `name()` reader"
    );
    // Complex variable field → reader is `child_bytes()`.
    assert!(
        code.contains("Read with the accessor method `child_bytes()`"),
        "complex-field note must name the `child_bytes()` reader"
    );
    // The attribute sits on the field DECL (adjacent to the pub field line).
    assert!(
        code.contains("    pub name: __cer_wp_Sig_name,"),
        "proxy field decl still emitted after the deprecated attr"
    );
}

#[test]
fn constructors_allow_deprecated_at_the_let_binding() {
    let code = generate_schema(&signal_schema());
    // All three constructors (from_bytes + from_bytes_mut + the
    // __cer_staging_resume sibling) init the deprecated proxy fields
    // through an #[allow(deprecated)] let-binding — the tightest stable
    // scope; NOT a crate- or fn-wide allow.
    assert_eq!(
        code.matches("#[allow(deprecated)]\n        let __cer_shm = Self {")
            .count(),
        3,
        "exactly the three constructor struct-literals are allow-scoped"
    );
    assert!(
        !code.contains("#![allow(deprecated)]"),
        "no crate/module-wide deprecated allow"
    );
    // The shim methods are NOT deprecated (writes stay warning-free).
    assert!(!code.contains("#[deprecated]\n    pub fn __cer_assign_"));
}

// ============================================================
// Hardening: fill_from-shim Err witness, full
// skip-list sweep, keyword-named variable field escaping
// ============================================================

#[test]
fn fill_from_shim_propagates_producer_err_and_recovers() {
    // The uniform `__cer_fill_from_<f>` shim is a one-line delegate — this
    // behavioral witness catches a variant that swallows the Result. Oracle: the
    // producer's Err propagates verbatim, and a subsequent good fill on the
    // SAME writer lands cleanly (the Err path rewound the cursor — the
    // no-state-mutation contract of the zero-copy fill API inherited through the shim).
    let mut ab = AlignedBuf::new(256);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(256), topic());
    let err = writer
        .__cer_fill_from_data(|_dst: &mut [u8]| -> Result<usize, TransportError> {
            Err(TransportError::NodeError {
                node_id: "camera".into(),
                reason: "device disconnected".into(),
            })
        })
        .expect_err("producer Err must propagate through the shim");
    assert!(matches!(err, TransportError::NodeError { .. }));
    writer
        .__cer_fill_from_data(|dst: &mut [u8]| {
            dst[..2].copy_from_slice(&[1u8, 2]);
            Ok(2)
        })
        .expect("fill after Err must succeed (cursor rewound)");
    assert_eq!(writer.data(), &[1u8, 2]);
}

#[test]
fn every_nonunderscore_private_shadow_name_gets_no_proxy_but_keeps_shim() {
    // Full sweep of the SHM_PRIVATE_FIELDS skip-list
    // (`variable_field_named_after_private_field_gets_no_proxy`
    // covers 3 of the names): every non-underscore private overlay field
    // name, used as a VARIABLE schema field, must get NO proxy field/type
    // but STILL get its write shim. (`_phantom`/`_marker` are covered by
    // skip-list membership; leading-underscore schema field names are not
    // exercised here.)
    for shadow in ["ptr", "len", "state", "overflow", "max_capacity", "topic"] {
        let mut s = MessageSchema::new("Shadow");
        s.add_field(FieldDef::new(shadow, FieldType::Bytes));
        let code = generate_schema(&s);
        assert!(
            !code.contains(&format!("__cer_wp_Shadow_{shadow}")),
            "`{shadow}` must get NO proxy type"
        );
        assert!(
            code.contains(&format!("pub fn __cer_assign_{shadow}(&mut self")),
            "`{shadow}` must keep its write shim"
        );
        assert!(
            !code.contains("compile_error!"),
            "`{shadow}` as a variable field is legal — no collision error"
        );
    }
}

#[test]
fn keyword_named_variable_field_escapes_shim_proxy_and_note() {
    // A variable field named after a Rust keyword (`type` — legal in ROS2
    // .msg files) must: keep the RAW name inside generated suffixed idents
    // (`__cer_assign_type`, proxy type `__cer_wp_Kw_type`), use the ESCAPED
    // form for the proxy FIELD decl (`r#type`), and name the ESCAPED reader
    // (`r#type()`) + write form (`self.<port>.r#type = expr`) in the
    // deprecation note (the note must never render `type()`,
    // which is not callable syntax).
    let mut s = MessageSchema::new("Kw");
    s.add_field(FieldDef::new("type", FieldType::String));
    let code = generate_schema(&s);
    assert!(
        code.contains("pub fn __cer_assign_type(&mut self, v: &str)"),
        "shim keeps the raw name in the suffixed ident"
    );
    assert!(
        code.contains("pub r#type: __cer_wp_Kw_type,"),
        "proxy FIELD decl must be keyword-escaped"
    );
    assert!(
        code.contains("accessor method `r#type()`"),
        "deprecation note must name the ESCAPED reader"
    );
    assert!(
        code.contains("self.<port>.r#type = expr"),
        "deprecation note write form must be keyword-escaped"
    );
    assert!(!code.contains("compile_error!"));
}

// ============================================================
// Nested-writer sugar (codegen half)
// ============================================================
//
// Fixed-nested substructs project directly (`__cer_nested_<f>` /
// `with_<f>` on the FixedSection — the field IS the target's own overlay
// type, so the leaf shims compose recursively). Complex-nested
// variable fields STAGE writes in per-field heap scratch with persisted
// nested-writer state (fresh view per accessor call — the resume pin
// below is the load-bearing one) and FLUSH through `set_<f>_bytes`.
// The 3-segment ASSIGN rewrite (`self.image.header.frame_id = …`) is
// the rewriter's job; proxy-Drop-driven flush over real transport is
// covered by the e2e serial families — here the inherent `__cer_flush_staged`
// is called directly (the same method the trait override delegates to).

use native_ros2_messages::std_msgs::Header;

#[test]
fn fixed_nested_accessor_and_with_closure_write_through() {
    // Imu.orientation is an inline fixed-nested QuaternionShm in
    // ImuFixedSection — the accessor is a plain projection reached through
    // the ImuShm Deref chain, and the leaf shims compose.
    let mut ab = AlignedBuf::new(512);
    let mut writer = Imu::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(512), topic());
    writer
        .__cer_nested_orientation()
        .expect("projection is infallible")
        .__cer_assign_x(&1.5f64)
        .expect("leaf shim");
    writer
        .with_orientation(|q| q.__cer_assign_w(&0.25f64))
        .expect("with_ closure form");
    assert_eq!(writer.orientation.x, 1.5, "nested leaf write landed in SHM");
    assert_eq!(writer.orientation.w, 0.25, "closure-form write landed too");
}

/// Build the hand-oracle Header payload: a STANDALONE HeaderShm writer
/// (not the staged path) executing `ops`, returning the payload prefix
/// bytes `[fixed|offset table|variable…cursor]` — exactly the byte shape
/// `set_header_bytes` stores and `header_bytes()` returns.
fn header_oracle(ops: impl FnOnce(&mut native_ros2_messages::std_msgs::HeaderShm<'_>)) -> Vec<u8> {
    let mut ab = AlignedBuf::new(2048);
    let bytes = ab.bytes();
    let mut w = Header::build_writer(bytes, MaxPayloadCapacity::const_new(2048), topic());
    ops(&mut w);
    let end = w.cursor() as usize;
    ab.bytes()[..end].to_vec()
}

#[test]
fn staged_header_accumulates_across_calls_and_flushes_to_oracle() {
    let mut ab = AlignedBuf::new(4096);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(4096), topic());

    // Call 1: variable leaf (frame_id).
    writer
        .with_header(|h| h.set_frame_id("cam0"))
        .expect("stage frame_id");
    // Call 2 — the LOAD-BEARING resume pin: a SECOND accessor call must
    // RESUME the persisted nested state (frame_id's offset entry + cursor
    // survive), not reset it. Writes fixed leaves through Deref on the
    // staged view, composing the fixed-nested `stamp` accessor too.
    writer
        .with_header(|h| {
            h.stamp.sec = 5;
            h.with_stamp(|t| t.__cer_assign_nanosec(&7u32))?;
            Ok(())
        })
        .expect("stage stamp");

    // Satisfy the other variable fields, then flush the staged header.
    writer.__cer_assign_data(&[1u8, 2]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    writer.__cer_flush_staged().expect("flush");

    // Oracle: the SAME logical writes through a STANDALONE Header writer
    // (not the staged machinery) — byte equality proves both the encoding
    // AND the resume (a reset second call would zero frame_id's offset
    // entry / rewind the cursor and diverge from these bytes).
    let oracle = header_oracle(|h| {
        h.set_frame_id("cam0").expect("oracle frame_id");
        h.stamp.sec = 5;
        h.stamp.nanosec = 7;
    });
    assert_eq!(
        writer.header_bytes(),
        &oracle[..],
        "staged header must flush byte-identically to the hand-built oracle"
    );
    assert!(
        writer.all_variables_written(),
        "flush must mark the header field written"
    );
}

#[test]
fn untouched_staging_flushes_nothing_and_discard_gate_unchanged() {
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    writer.__cer_assign_data(&[1u8]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    // header never touched: flush is a no-op Ok, the field stays
    // unwritten, and the write-all-variable-fields discard gate fires
    // exactly as today.
    writer.__cer_flush_staged().expect("no-op flush");
    assert!(writer.header_bytes().is_empty(), "nothing flushed");
    assert!(
        !writer.all_variables_written(),
        "untouched staged field must still trip the discard gate"
    );
}

#[test]
fn staged_flush_failure_is_loud_at_capacity() {
    // Loan AND ceiling of 160 bytes: data+encoding fill ~150, the staged
    // header (~60+ bytes) cannot fit at flush → the flush's set_header_bytes
    // hits PayloadTooLarge and the error PROPAGATES (never a silent
    // partial frame; in production Drop routes this into the loud-discard
    // path).
    let mut ab = AlignedBuf::new(160);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(160), topic());
    writer.__cer_assign_data(&[7u8; 100]).expect("data fits");
    writer.__cer_assign_encoding("rgb8").expect("encoding fits");
    writer
        .with_header(|h| h.set_frame_id("a-frame-id-of-decent-length-here-padding"))
        .expect("staging itself fits (scratch is heap, not the loan)");
    let err = writer
        .__cer_flush_staged()
        .expect_err("flush past the ceiling must fail loudly");
    assert!(
        matches!(err, TransportError::PayloadTooLarge { .. }),
        "flush failure must surface the capacity error, got {err:?}"
    );
}

#[test]
fn staged_scratch_grows_past_floor_via_spill_harvest() {
    // frame_id of 1000 bytes overflows the 256-byte scratch floor mid-write;
    // the nested writer spills internally (bounded by the PARENT's
    // capacity) and the accessor adopts the spill as the new scratch — the
    // write SUCCEEDS transparently, and a SECOND call still resumes on the
    // grown scratch.
    let long_id = "x".repeat(1000);
    let mut ab = AlignedBuf::new(8192);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    writer
        .with_header(|h| h.set_frame_id(&long_id))
        .expect("over-floor staged write must grow, not error");
    writer
        .with_header(|h| {
            h.stamp.sec = 9;
            Ok(())
        })
        .expect("resume on grown scratch");
    writer.__cer_assign_data(&[]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    writer.__cer_flush_staged().expect("flush grown payload");

    let oracle = header_oracle(|h| {
        h.set_frame_id(&long_id).expect("oracle frame_id");
        h.stamp.sec = 9;
    });
    assert_eq!(
        writer.header_bytes(),
        &oracle[..],
        "grown staged payload must still flush byte-identically"
    );
}

// ---- string-level pins ----

#[test]
fn nested_sugar_emission_pins() {
    let code = generate_schema(&signal_schema());
    // Complex-nested field `child` (target `Child`): staging slot (private,
    // one decl + None inits in from_bytes, from_bytes_mut AND the resume
    // ctor), the public with_ + doc-hidden primitive, and the flush routing
    // through the existing raw-bytes setter.
    assert!(code.contains("__cer_staged_child: ::std::option::Option<::std::boxed::Box<::cerulion_core::shm_runtime::StagedNested>>,"));
    assert_eq!(
        code.matches("__cer_staged_child: ::std::option::Option::None,")
            .count(),
        3,
        "staging slot must init None in from_bytes + from_bytes_mut + __cer_staging_resume"
    );
    assert!(code.contains("pub fn with_child<__CerF>"));
    assert!(code.contains("pub fn __cer_with_nested_child<__CerF>"));
    assert!(code.contains("pub fn __cer_flush_staged"));
    assert!(code.contains("self.set_child_bytes(&::cerulion_core::shm_runtime::u64s_as_bytes(&staged.scratch)[..end])?;"));
    // The staging surface is on EVERY variable schema (any can be a target).
    assert!(code.contains("pub fn __cer_staging_resume"));
    assert!(code.contains("pub fn __cer_staging_export"));
    assert!(code.contains("pub fn __cer_staging_take_overflow"));
    // Simple variable fields get NO staging slot / with_ accessor.
    for simple in ["name", "blob", "samples"] {
        assert!(
            !code.contains(&format!("__cer_staged_{simple}")),
            "simple variable field `{simple}` must not stage"
        );
        assert!(
            !code.contains(&format!("pub fn with_{simple}<")),
            "simple variable field `{simple}` gets no with_ accessor"
        );
    }
    // The ShmMessage impl overrides flush_staged_nested for this schema.
    assert!(code.contains("fn flush_staged_nested(writer: &mut Self::Writer<'_>)"));
    assert!(code.contains("writer.__cer_flush_staged()"));
}

#[test]
fn fixed_nested_sugar_emission_pins() {
    // A schema with a RESOLVED-fixed nested field gets the projection +
    // closure accessors (on the fixed overlay), and NO staging machinery.
    let mut s = MessageSchema::new("Robot");
    s.add_field(FieldDef::new(
        "pose",
        FieldType::Nested {
            schema_name: "P3".to_string(),
            package: None,
            fixed: Some(cerulion_core::codegen::NestedFixedInfo {
                has_large_array: false,
                fixed_size: 24,
                alignment: 8,
                target_hash: 0xBEEF,
            }),
        },
    ));
    let code = generate_schema(&s);
    assert!(code.contains("pub fn __cer_nested_pose(&mut self)"));
    assert!(code.contains("-> ::std::result::Result<&mut P3Shm, ::cerulion_core::TransportError>"));
    assert!(code.contains("pub fn with_pose<__CerF>"));
    assert!(code.contains("pub fn __cer_with_nested_pose<__CerF>"));
    assert!(
        !code.contains("__cer_staged_pose"),
        "fixed nested is a direct projection — no staging"
    );
    // Keyword-named complex field: derived idents embed the RAW name (valid
    // as a substring — the r# escaping rule applies to the MACRO side).
    let mut k = MessageSchema::new("Kw");
    k.add_field(FieldDef::new(
        "type",
        FieldType::Nested {
            schema_name: "Child".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let kcode = generate_schema(&k);
    assert!(kcode.contains("pub fn with_type<__CerF>"));
    assert!(kcode.contains("__cer_staged_type: ::std::option::Option"));
}

#[test]
fn with_accessor_name_collision_is_caught() {
    // A fixed-section field named `with_child` collides with the generated
    // `with_child` accessor of the complex-nested sibling `child` — the
    // consolidated collision compile_error must name it.
    let mut s = MessageSchema::new("Coll2");
    s.add_field(FieldDef::new("with_child", FieldType::U32));
    s.add_field(FieldDef::new(
        "child",
        FieldType::Nested {
            schema_name: "Child".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let code = generate_schema(&s);
    assert!(
        code.contains("compile_error!") && code.contains("`with_child`"),
        "a field shadowing a generated with_ accessor must be rejected"
    );
}

// ============================================================
// Nested-write conflict guards (behavioral, at the writer level)
// ============================================================
//
// Mixing the staged leaf sugar with a whole-field write on ONE complex-nested
// field within one loan is a loud `NestedWriteConflict` (both orders). These
// pin the guards directly on a real `ImageShm` writer (`header` is the
// complex-nested field; its lone child variable is `frame_id`, its child fixed
// leaf is `stamp.sec`).

#[test]
fn staged_then_whole_field_write_is_conflict_error() {
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    // Stage a leaf write on `header`.
    writer
        .with_header(|h| h.set_frame_id("cam0"))
        .expect("stage frame_id");
    // A whole-field write on the SAME field while staging is in flight →
    // conflict, in ALL three whole-field entry points.
    let err = writer
        .set_header_bytes(&[1, 2, 3])
        .expect_err("set_<f>_bytes during staging must conflict");
    assert!(
        matches!(&err, TransportError::NestedWriteConflict { field: "header", detected }
            if *detected == "a staged nested-field write, then a whole-field write"),
        "got {err:?}"
    );
    let err2 = writer
        .loan_header_bytes(3)
        .expect_err("loan_<f>_bytes during staging must conflict");
    assert!(
        matches!(err2, TransportError::NestedWriteConflict { .. }),
        "got {err2:?}"
    );
    let err3 = writer
        .fill_from_header_bytes(|buf: &mut [u8]| {
            buf[..1].copy_from_slice(&[9]);
            Ok(1)
        })
        .expect_err("fill_from_<f>_bytes during staging must conflict (own cursor body)");
    assert!(
        matches!(err3, TransportError::NestedWriteConflict { .. }),
        "got {err3:?}"
    );
}

#[test]
fn whole_field_write_then_staged_is_conflict_error() {
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    writer
        .set_header_bytes(&[1, 2, 3])
        .expect("whole-field write");
    // A staged accessor after the field was written wholesale → conflict.
    let err = writer
        .with_header(|h| h.set_frame_id("cam0"))
        .expect_err("staging after a whole-field write must conflict");
    assert!(
        matches!(&err, TransportError::NestedWriteConflict { field: "header", detected }
            if *detected == "a whole-field write, then a staged nested-field write"),
        "got {err:?}"
    );
}

#[test]
fn double_whole_field_write_stays_legal_last_write_wins() {
    // NEGATIVE CONTROL: two whole-field writes never involve staging, so the
    // guard must NOT fire — pre-existing last-write-wins semantics hold.
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    writer.set_header_bytes(&[1, 2, 3]).expect("first write");
    writer
        .set_header_bytes(&[9, 9, 9, 9])
        .expect("second whole-field write must stay legal (no staging)");
    assert_eq!(writer.header_bytes(), &[9, 9, 9, 9], "last write wins");
}

#[test]
fn all_staged_writes_stay_legal_no_conflict() {
    // NEGATIVE CONTROL: multiple staged writes to the SAME field accumulate
    // (resume) with NO conflict, and flush cleanly (child fully written).
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    writer
        .with_header(|h| h.set_frame_id("cam0"))
        .expect("stage 1 (variable child)");
    writer
        .with_header(|h| {
            h.stamp.sec = 5;
            Ok(())
        })
        .expect("stage 2 (fixed child leaf) — resume, no conflict");
    writer.__cer_assign_data(&[1u8]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    writer
        .__cer_flush_staged()
        .expect("flush ok — all staged, child fully written");
    assert!(writer.all_variables_written());
}

#[test]
fn staged_child_missing_variable_field_is_loud_discard() {
    // Touch only the FIXED child leaf `stamp.sec`; the child's lone VARIABLE
    // field `frame_id` stays unwritten → the staged child fails the
    // every-variable-field gate at flush, naming the dotted location.
    let mut ab = AlignedBuf::new(1024);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(1024), topic());
    writer
        .with_header(|h| {
            h.stamp.sec = 5;
            Ok(())
        })
        .expect("stage stamp only");
    writer.__cer_assign_data(&[1u8]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    let err = writer
        .__cer_flush_staged()
        .expect_err("partial staged child must discard loudly");
    match err {
        TransportError::NestedChildIncomplete {
            parent_field,
            child_field,
        } => {
            assert_eq!(parent_field, "header");
            assert_eq!(
                child_field, "frame_id",
                "depth-1 miss carries the plain child name"
            );
        }
        other => panic!("expected NestedChildIncomplete, got {other:?}"),
    }
}

// ---- string-level pins: guard emission ----

#[test]
fn nested_write_conflict_and_child_gate_guards_are_emitted() {
    let code = generate_schema(&signal_schema());
    // Guard A in the whole-field writers of the complex-nested `child`.
    assert!(
        code.contains("if self.__cer_staged_child.is_some() {"),
        "guard A (staged-then-whole) must be emitted in the whole-field writers"
    );
    assert!(code.contains("a staged nested-field write, then a whole-field write"));
    // Guard B in the staged accessor.
    assert!(
        code.contains("a whole-field write, then a staged nested-field write"),
        "guard B (whole-then-staged) must be emitted in __cer_with_nested_*"
    );
    // The staged-child all-variables gate + the first-unwritten helper.
    assert!(code.contains("pub fn __cer_first_unwritten_variable"));
    assert!(code.contains("__cer_view.__cer_first_unwritten_variable()"));
    assert!(code.contains("NestedChildIncomplete"));
    // Recursive staging persistence: children take/restore emitted;
    // the accessor persists/restores grandchildren; the flush restores,
    // recurses via the TARGET's own flush, and RE-exports before the parent
    // `set_<f>_bytes`.
    assert!(code.contains("pub fn __cer_staging_take_children"));
    assert!(code.contains("pub fn __cer_staging_restore_children"));
    // Restore is fallible (unknown name = loud
    // Internal Err, never warn-only / debug_assert-in-Drop) — both call
    // sites carry `?`.
    assert!(code.contains(
        "__cer_view.__cer_staging_restore_children(::std::mem::take(&mut staged.children))?;"
    ));
    assert!(code.contains("staged.children = __cer_view.__cer_staging_take_children();"));
    // The recursive flush re-maps a deeper
    // NestedChildIncomplete by PREPENDING this level's field name (dotted
    // path), so the pin is on the match wrapper, not a bare `?` call.
    assert!(code.contains(
        "if let ::std::result::Result::Err(__cer_e) = __cer_view.__cer_flush_staged() {"
    ));
    assert!(code.contains("child_field: ::std::format!(\"{__cer_p}.{__cer_c}\"),"));
}

#[test]
fn no_complex_schema_still_emits_recursion_terminators() {
    // A variable schema WITHOUT complex-nested fields (a pure
    // String/Bytes schema like Header) must still emit the recursion
    // surface — a PARENT's flush calls `__cer_view.__cer_flush_staged()` /
    // take/restore on the TARGET type unconditionally (the parent codegen
    // cannot know the target's shape — registry gap), so the trivial forms
    // are load-bearing: no-op flush (clean termination), empty take,
    // warn-only restore. The `ShmMessage` trait override stays ABSENT
    // (Drop needs it only when slots exist).
    let mut s = MessageSchema::new("Plain");
    s.add_field(FieldDef::new("name", FieldType::String));
    let code = generate_schema(&s);
    assert!(
        code.contains("pub fn __cer_flush_staged"),
        "no-op flush must exist for recursion termination"
    );
    assert!(code.contains("pub fn __cer_staging_take_children"));
    assert!(code.contains("pub fn __cer_staging_restore_children"));
    // Restore on a slot-less schema is a loud
    // Err(Internal) on the FIRST entry (not warn-per-entry +
    // debug_assert-in-Drop) — never a silent drop, never a Drop panic.
    assert!(
        code.contains("unknown staged child field '{}' on PlainShm"),
        "restore on a slot-less schema errors loudly"
    );
    assert!(
        code.contains("children.into_iter().next()"),
        "the slot-less arm errors on the first entry, not per-entry warns"
    );
    assert!(
        !code.contains("fn flush_staged_nested(writer:"),
        "the Drop-path trait override must stay gated to embedding schemas"
    );
}

#[test]
fn keyword_nested_child_leaf_shim_is_r_hash_stripped() {
    // Keyword-nested pin (codegen route — a real native schema
    // `autoware_perception_msgs::PredictedObject.shape` → `Shape.type` also
    // has a COMPILE pin in tests/ui/pass/shim_assign_and_fill_from_passes.rs).
    // A nested child schema with a keyword `type` field must emit the
    // r#-STRIPPED assign shim ident `__cer_assign_type`, never the invalid
    // `__cer_assign_r#type` (which panics `format_ident!`).
    let mut child = MessageSchema::new("Shape");
    child.add_field(FieldDef::new("type", FieldType::U8));
    child.add_field(FieldDef::new("footprint", FieldType::Bytes)); // makes it variable
    let ccode = generate_schema(&child);
    assert!(
        ccode.contains("pub fn __cer_assign_type"),
        "keyword child leaf emits the r#-stripped shim ident"
    );
    assert!(
        !ccode.contains("__cer_assign_r#type"),
        "the r# must not leak into the emitted shim ident"
    );
    // And a parent complex-nested field emits the rewriter target.
    let mut parent = MessageSchema::new("Parent");
    parent.add_field(FieldDef::new(
        "shape",
        FieldType::Nested {
            schema_name: "Shape".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let pcode = generate_schema(&parent);
    assert!(
        pcode.contains("pub fn __cer_with_nested_shape<__CerF>"),
        "complex-nested field must emit the rewriter target"
    );
}

// ============================================================
// Recursive staging persistence — behavioral, real types
// ============================================================
//
// Native complex-inside-complex chain: `moveit_msgs/RobotTrajectory` has a
// scalar `joint_trajectory` (trajectory_msgs/JointTrajectory — VARIABLE, so
// complex-nested), and JointTrajectory itself has a complex-nested `header`
// (std_msgs/Header). Grandchild writes (`….joint_trajectory.header.frame_id`)
// stage into the transient VIEW's own slot — without persistence that staging dies when
// the view drops (silent data loss + a wrong-field NestedChildIncomplete).

/// Hand-oracle JointTrajectory payload via a STANDALONE writer (never the
/// staged path) — same shape as `header_oracle` above.
fn joint_trajectory_oracle(
    ops: impl FnOnce(&mut native_ros2_messages::trajectory_msgs::JointTrajectoryShm<'_>),
) -> Vec<u8> {
    use native_ros2_messages::trajectory_msgs::JointTrajectory;
    let mut ab = AlignedBuf::new(4096);
    let bytes = ab.bytes();
    let mut w = JointTrajectory::build_writer(bytes, MaxPayloadCapacity::const_new(4096), topic());
    ops(&mut w);
    let end = w.cursor() as usize;
    ab.bytes()[..end].to_vec()
}

#[test]
fn take_restore_children_round_trips_and_flushes_to_oracle() {
    // Part 1: the take/restore pair drains and reinstates the
    // staged-children slots losslessly — after a round-trip the flush lands
    // the same bytes a standalone oracle produces.
    let mut ab = AlignedBuf::new(2048);
    let mut writer = Image::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(2048), topic());
    writer
        .with_header(|h| h.set_frame_id("cam0"))
        .expect("stage frame_id");
    let children = writer.__cer_staging_take_children();
    assert_eq!(children.len(), 1, "one staged child drained");
    assert_eq!(children[0].0, "header", "keyed by the raw field name");
    writer
        .__cer_staging_restore_children(children)
        .expect("matched-name restore succeeds");
    writer.__cer_assign_data(&[1u8]).expect("data");
    writer.__cer_assign_encoding("g").expect("encoding");
    writer.__cer_flush_staged().expect("flush after round-trip");
    let oracle = header_oracle(|h| {
        h.set_frame_id("cam0").expect("oracle frame_id");
    });
    assert_eq!(
        writer.header_bytes(),
        &oracle[..],
        "take/restore round-trip must be lossless"
    );
}

#[test]
fn grandchild_staging_flushes_recursively_to_standalone_oracle() {
    // Part 2: full recursion through the PUBLIC nested accessors.
    // Three separate accessor entries on `joint_trajectory` (two grandchild
    // chains + the child's sibling fields) — the grandchild staging must
    // survive each view teardown (children persistence) and land bottom-up
    // at flush. Without children persistence the header staging dies with the
    // first transient view.
    use native_ros2_messages::moveit_msgs::RobotTrajectory;
    let mut ab = AlignedBuf::new(8192);
    let mut w =
        RobotTrajectory::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    // Chain 1: grandchild variable leaf.
    w.with_joint_trajectory(|jt| jt.with_header(|h| h.set_frame_id("base")))
        .expect("stage grandchild frame_id");
    // Chain 2: grandchild fixed leaf — must RESUME chain 1's staging.
    w.with_joint_trajectory(|jt| {
        jt.with_header(|h| {
            h.stamp.sec = 7;
            Ok(())
        })
    })
    .expect("stage grandchild stamp");
    // Chain 3: the child's other variable fields (whole-field bytes).
    w.with_joint_trajectory(|jt| {
        jt.set_joint_names_bytes(&[])?;
        jt.set_points_bytes(&[])
    })
    .expect("child siblings");
    // Parent's sibling variable field.
    w.set_multi_dof_joint_trajectory_bytes(&[])
        .expect("parent sibling");
    w.__cer_flush_staged().expect("recursive flush");

    // Hand oracle: same logical writes on a STANDALONE JointTrajectory
    // writer, in the order the staged path produces them (joint_names +
    // points in call order; the staged header flushes LAST at the tail).
    let header_bytes = header_oracle(|h| {
        h.set_frame_id("base").expect("oracle frame_id");
        h.stamp.sec = 7;
    });
    let jt_oracle = joint_trajectory_oracle(|o| {
        o.set_joint_names_bytes(&[]).expect("oracle names");
        o.set_points_bytes(&[]).expect("oracle points");
        o.set_header_bytes(&header_bytes).expect("oracle header");
    });
    assert_eq!(
        w.joint_trajectory_bytes(),
        &jt_oracle[..],
        "grandchild staging must flush recursively, byte-identical to the standalone oracle"
    );
    assert!(w.all_variables_written());
}

#[test]
fn partial_grandchild_discards_naming_the_grandchild_level() {
    // A depth-2 partial grandchild — the grandchild Header got
    // only its FIXED leaf (stamp.sec); its variable `frame_id` stays
    // unwritten. The recursive flush must fail loudly with the error naming
    // the GRANDCHILD's level (`header.frame_id`), not the child's.
    use native_ros2_messages::moveit_msgs::RobotTrajectory;
    let mut ab = AlignedBuf::new(8192);
    let mut w =
        RobotTrajectory::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    w.with_joint_trajectory(|jt| {
        jt.with_header(|h| {
            h.stamp.sec = 7;
            Ok(())
        })
    })
    .expect("stage grandchild stamp only");
    w.with_joint_trajectory(|jt| {
        jt.set_joint_names_bytes(&[])?;
        jt.set_points_bytes(&[])
    })
    .expect("child siblings");
    w.set_multi_dof_joint_trajectory_bytes(&[])
        .expect("parent sibling");
    let err = w
        .__cer_flush_staged()
        .expect_err("partial grandchild must discard loudly");
    // The recursive flush PREPENDS the intermediate
    // level, so the error names the FULL dotted path the user writes
    // (`self.<port>.joint_trajectory.header.frame_id`) — parent is the
    // top-level field, child the dotted remainder. Carrying only the
    // bare deepest pair (header/frame_id) would give a paste-ready remediation
    // path that does not compile from the port.
    match err {
        TransportError::NestedChildIncomplete {
            parent_field,
            child_field,
        } => {
            assert_eq!(parent_field, "joint_trajectory");
            assert_eq!(
                child_field, "header.frame_id",
                "depth-2 miss must carry the dotted path"
            );
        }
        other => panic!("expected NestedChildIncomplete, got {other:?}"),
    }
}

#[test]
fn grandchild_flush_spill_adopts_and_lands_byte_identical() {
    // Spill-during-flush coverage: the flush-time
    // re-export/spill-adopt branch is DISTINCT from the accessor-exit
    // harvest and only fires when the RECURSIVE flush (landing the staged
    // grandchild into the child's scratch via `set_header_bytes`) overflows
    // the child scratch AT FLUSH TIME. A multi-KB grandchild leaf forces
    // exactly that: the JointTrajectory scratch (floored well below 4 KiB)
    // must overflow-spill during `__cer_flush_staged`, be adopted as the new
    // scratch, and the parent land the FULL payload — a stale pre-spill
    // cursor or pre-realloc scratch read would truncate or corrupt it.
    use native_ros2_messages::moveit_msgs::RobotTrajectory;
    // 1 KiB: comfortably past the 256-byte child-scratch floor (forces the
    // flush-time spill) while fitting the shared 2048-cap oracle helpers.
    let big_frame = "f".repeat(1024);
    let mut ab = AlignedBuf::new(16384);
    let mut w =
        RobotTrajectory::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(16384), topic());
    w.with_joint_trajectory(|jt| jt.with_header(|h| h.set_frame_id(&big_frame)))
        .expect("stage large grandchild frame_id");
    w.with_joint_trajectory(|jt| {
        jt.set_joint_names_bytes(&[])?;
        jt.set_points_bytes(&[])
    })
    .expect("child siblings");
    w.set_multi_dof_joint_trajectory_bytes(&[])
        .expect("parent sibling");
    w.__cer_flush_staged()
        .expect("flush with in-flush spill adoption");

    let header_bytes = header_oracle(|h| {
        h.set_frame_id(&big_frame).expect("oracle frame_id");
    });
    let jt_oracle = joint_trajectory_oracle(|o| {
        o.set_joint_names_bytes(&[]).expect("oracle names");
        o.set_points_bytes(&[]).expect("oracle points");
        o.set_header_bytes(&header_bytes).expect("oracle header");
    });
    assert_eq!(
        w.joint_trajectory_bytes(),
        &jt_oracle[..],
        "flush-time spill adoption must land the full grandchild payload byte-identically"
    );
    assert!(w.all_variables_written());
}

#[test]
fn grandchild_staging_conflicts_with_whole_writes_at_every_level() {
    // Depth-2 NestedWriteConflict coverage: the
    // restore/guard composition.
    // Three arms pin it at both levels and in both orders.
    use native_ros2_messages::moveit_msgs::RobotTrajectory;

    // Arm A — grandchild sugar staged, then a whole-CHILD write on the
    // parent: guard A fires on the parent's staged slot.
    let mut ab = AlignedBuf::new(8192);
    let mut w =
        RobotTrajectory::build_writer(ab.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    w.with_joint_trajectory(|jt| jt.with_header(|h| h.set_frame_id("x")))
        .expect("stage grandchild");
    let err = w
        .set_joint_trajectory_bytes(&[])
        .expect_err("whole-child write while grandchild staging in flight must conflict");
    match err {
        TransportError::NestedWriteConflict { field, detected } => {
            assert_eq!(field, "joint_trajectory");
            assert_eq!(
                detected,
                "a staged nested-field write, then a whole-field write"
            );
        }
        other => panic!("expected NestedWriteConflict, got {other:?}"),
    }

    // Arm B — whole-child write first, then a grandchild sugar chain:
    // guard B fires at the parent-level accessor on the written field.
    let mut ab_b = AlignedBuf::new(8192);
    let mut w_b =
        RobotTrajectory::build_writer(ab_b.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    w_b.set_joint_trajectory_bytes(&[]).expect("whole child");
    let err_b = w_b
        .with_joint_trajectory(|jt| jt.with_header(|h| h.set_frame_id("x")))
        .expect_err("grandchild staging after a whole-child write must conflict");
    match err_b {
        TransportError::NestedWriteConflict { field, detected } => {
            assert_eq!(field, "joint_trajectory");
            assert_eq!(
                detected,
                "a whole-field write, then a staged nested-field write"
            );
        }
        other => panic!("expected NestedWriteConflict, got {other:?}"),
    }

    // Arm C — THE RESTORED-STAGING COMPOSITION (the untested prose claim):
    // grandchild sugar in one accessor entry, then a whole-GRANDCHILD write
    // inside a SECOND accessor entry on the same child. The second view is
    // fresh; only `__cer_staging_restore_children` makes the prior header
    // staging visible to its guard A. If restore/guard composition
    // regressed, this would silently go last-write-wins instead.
    let mut ab_c = AlignedBuf::new(8192);
    let mut w_c =
        RobotTrajectory::build_writer(ab_c.bytes(), MaxPayloadCapacity::const_new(8192), topic());
    w_c.with_joint_trajectory(|jt| jt.with_header(|h| h.set_frame_id("x")))
        .expect("stage grandchild");
    let err_c = w_c
        .with_joint_trajectory(|jt| jt.set_header_bytes(&[]))
        .expect_err("whole-grandchild write must see the RESTORED staging and conflict");
    match err_c {
        TransportError::NestedWriteConflict { field, detected } => {
            assert_eq!(field, "header", "the conflict is at the grandchild level");
            assert_eq!(
                detected,
                "a staged nested-field write, then a whole-field write"
            );
        }
        other => panic!("expected NestedWriteConflict, got {other:?}"),
    }
}
