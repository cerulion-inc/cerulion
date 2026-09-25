// SPDX-License-Identifier: AGPL-3.0-only
//! Node source and Cargo.toml template generation.
//!
//! Generates cdylib node source for the handle-based FFI convention. The
//! emitted `cerulion_abi_version()` tracks `cerulion_core::CERULION_ABI_VERSION`,
//! and the emitted `cerulion_rustc_fingerprint()` (ABI v22) tracks
//! `cerulion_core::rustc_fingerprint_cstr()`. Neither is pinned to a specific
//! value here, so a host rebuild propagates to every future generated node
//! crate with no template regeneration.
//! Uses programmatic `String` building (matching `codegen/generator/` pattern).

use crate::node_metadata::NodeMetadata;
use crate::utils::to_pascal_case;

/// Generate the `Cargo.toml` for a node crate.
///
/// Uses workspace dependency inheritance — the workspace `Cargo.toml` defines
/// the actual paths to `cerulion_core` and `native_ros2_messages`, so node
/// crates just reference `{ workspace = true }`. This avoids fragile relative
/// paths that break when the workspace is nested inside another project.
pub fn generate_cargo_toml(node_type: &str) -> String {
    format!(
        r#"[package]
name = "{node_type}"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib"]

[features]
cdylib = []
default = ["cdylib"]

[dependencies]
cerulion_core = {{ workspace = true }}
native_ros2_messages = {{ workspace = true }}
"#
    )
}

/// Generate the standalone manifest for an embedded Python node.
pub fn generate_python_cargo_toml(node_type: &str, pynode_path: &str) -> String {
    format!(
        r#"[package]
name = "{node_type}"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib"]

[dependencies]
cerulion_pynode = {{ path = "{pynode_path}" }}

[profile.release]
strip = true
"#
    )
}

/// Generate the build script for an embedded Python node.
pub fn generate_python_build_rs(python_libdir: &str) -> String {
    let libdir = format!("{python_libdir:?}");
    format!(
        r#"// SPDX-License-Identifier: AGPL-3.0-only
fn main() {{
    println!("cargo:rerun-if-changed=build.rs");
    // CERULION:LIBDIR_START
    const PYTHON_LIBDIR: &str = {libdir};
    if (cfg!(target_os = "linux") || cfg!(target_os = "macos")) && !PYTHON_LIBDIR.is_empty() {{
        println!("cargo:rustc-link-arg=-Wl,-rpath,{{}}", PYTHON_LIBDIR);
    }}
    // CERULION:LIBDIR_END
}}
"#
    )
}

/// Generate the marker-managed Python import search path block.
pub fn generate_python_sys_path_block(node_dir: &str, site_paths: &[String]) -> String {
    let entries = std::iter::once(node_dir.to_string())
        .chain(site_paths.iter().cloned())
        .map(|path| format!("    {path:?},"))
        .collect::<Vec<_>>()
        .join("\n");
    format!(
        "    sys_path: [
// CERULION:SYSPATH_START
{entries}
// CERULION:SYSPATH_END
    ]"
    )
}

/// Generate the ABI-21 export source for an embedded Python node.
pub fn generate_python_lib_rs(
    node_dir: &str,
    inputs: &[(String, String)],
    outputs: &[(String, String)],
    policy: &serde_json::Value,
    site_paths: &[String],
) -> Result<String, serde_json::Error> {
    let info = serde_json::json!({
        "inputs": inputs.iter().map(|(name, _)| serde_json::json!({"name": name, "schema_hash": 0})).collect::<Vec<_>>(),
        "outputs": outputs.iter().map(|(name, _)| serde_json::json!({"name": name, "schema_hash": 0, "max_slice_len_default": null, "promise_within_ms": null, "wire_fixed_size": null})).collect::<Vec<_>>(),
        "policy": policy,
    });
    let info_text = serde_json::to_string(&info)?;
    let sys_path = generate_python_sys_path_block(node_dir, site_paths);
    Ok(format!(
        "// SPDX-License-Identifier: AGPL-3.0-only
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b\"{escaped}\\0\";
// CERULION:INFO_END

cerulion_pynode::export_node! {{
    module: \"node\",
{sys_path},
    info: INFO_BYTES
}}
",
        escaped = info_text.replace('\\', "\\\\").replace('"', "\\\""),
    ))
}

/// Generate the starter Python implementation for a node.
pub fn generate_python_node_py(
    inputs: &[(String, String)],
    outputs: &[(String, String)],
    policy: &str,
    trigger: Option<&str>,
) -> String {
    let mut result = format!("import cerulion as cer\n\n\n@cer.node({policy})\nclass Node:\n");
    for (name, schema) in inputs {
        let trigger_suffix = if trigger == Some(name.as_str()) {
            ", trigger=True"
        } else {
            ""
        };
        result.push_str(&format!(
            "    {name} = cer.input(\"{schema}\"{trigger_suffix})\n"
        ));
    }
    for (name, schema) in outputs {
        result.push_str(&format!("    {name} = cer.output(\"{schema}\")\n"));
    }
    result.push_str("\n    def tick(self):\n");
    match (inputs.first(), outputs.first()) {
        (Some((input, _)), Some((output, _))) => {
            result.push_str(&format!("        msg = self.{input}\n"));
            result.push_str("        if msg is None:  # no frame received yet\n");
            result.push_str("            return\n");
            result.push_str(&format!(
                "        out = self.{output}  # first touch loans the output; it is committed at tick end\n"
            ));
            result.push_str("        # copy fields here, e.g. out.x = msg.x\n");
        }
        (Some((input, _)), None) => {
            result.push_str(&format!("        msg = self.{input}\n"));
            result.push_str("        if msg is None:  # no frame received yet\n");
            result.push_str("            return\n");
        }
        (None, Some((output, _))) => {
            result.push_str(&format!(
                "        out = self.{output}  # first touch loans the output; it is committed at tick end\n"
            ));
        }
        (None, None) => result.push_str("        return\n"),
    }
    result
}

/// Generate the `lib.rs` source for a node crate using the `#[cerulion_node]` +
/// `#[cerulion_node_impl]` macro pair.
///
/// Emits the declarative-mode macro with `#[input]` / `#[output]`
/// field attrs (the node type is the folder name; macro args carry
/// only the optional trigger policy). The `trigger` parameter
/// optionally names an input port whose arrival triggers the
/// node's tick.
pub fn generate_macro_lib_rs(metadata: &NodeMetadata, trigger: Option<&str>) -> String {
    let mut src = String::new();
    let pascal = to_pascal_case(&metadata.node_type);

    // Same rule as the raw-FFI scaffold: a node is a shared library the graph
    // runtime loads, so its `println!` lands on whatever stdout the host has,
    // bypassing the log filter. Scoped `not(test)` so the node's own unit tests
    // still print; delete the line if you have a reason to print from node code.
    //
    // The raw-FFI scaffold below emits the same two comment lines in its own
    // wording; that emit is frozen byte for byte by the
    // `test_node_raw_ffi_template_cdylib` fixture, so the two are edited apart.
    src.push_str("// A node is a LIBRARY the runtime loads: log with `tracing`,\n");
    src.push_str("// never `println!`. (Your own #[cfg(test)] tests may print.)\n");
    src.push_str("#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]\n\n");

    // Imports
    src.push_str("use cerulion_core::prelude::*;\n");

    // Schema imports
    let mut seen_imports = std::collections::HashSet::new();
    for port in metadata.inputs.iter().chain(metadata.outputs.iter()) {
        if let Some(ref schema) = port.schema {
            let import = schema_to_import(schema);
            if seen_imports.insert(import.clone()) {
                src.push_str(&format!("use {};\n", import));
            }
        }
    }
    src.push('\n');

    // Build macro attribute. The macro carries the trigger policy
    // when one is set on the metadata. When none is set the node gets a
    // bare `#[cerulion_node]` and NO default policy is invented (see the
    // `None` arm below). A single `#[input(trigger)]` field carries the
    // data-trigger policy implicitly; the macro emits no node-level
    // attribute in that case, and the validator turns the field attribute
    // into the policy.
    //
    // For `MacroPolicy::DataTrigger { input_name }`, the matching
    // field-level `#[input(trigger)]` must be emitted by the
    // caller via the `trigger` parameter. If the caller asks for
    // DataTrigger but didn't supply the corresponding trigger
    // input, we'd silently drop the policy to `period_ms = 100`
    // (the wrong behavior: a periodic node when the user asked
    // for data-triggered). Reject the mismatch with a debug-assert
    // so misuse is caught loudly in tests, and fall back to a
    // visible runtime warning in release.
    let has_trigger_input = trigger.is_some();
    // An `#[cerulion_node(external)]` node MUST define an
    // `external_source` method (the macro rejects the impl block without it), so
    // the External arm below scaffolds one alongside `tick`.
    let is_external = matches!(
        metadata.policy.as_ref(),
        Some(cerulion_core::MacroPolicy::External)
    );
    let macro_args: String = match metadata.policy.as_ref() {
        Some(cerulion_core::MacroPolicy::Period { period_ms }) => {
            format!("period_ms = {period_ms}")
        }
        Some(cerulion_core::MacroPolicy::Sync { window_ms }) => {
            format!("sync_window_ms = {window_ms}")
        }
        Some(cerulion_core::MacroPolicy::UnboundedSync) => "unbounded_sync".to_string(),
        Some(cerulion_core::MacroPolicy::External) => "external".to_string(),
        Some(cerulion_core::MacroPolicy::DataTrigger { input_name }) => {
            // Sanity: the trigger param must name the same input
            // as the policy. Both must agree, and at least one
            // must be set, otherwise we'd silently emit a periodic
            // node when the user asked for data-triggered.
            debug_assert!(
                trigger == Some(input_name.as_str()),
                "generate_macro_lib_rs: MacroPolicy::DataTrigger {{ input_name: {:?} }} \
                 requires the `trigger` parameter to be Some({:?}); got {:?}. \
                 The caller (typically `node_create_with_options`) must thread the \
                 policy's `input_name` through to `trigger`.",
                input_name,
                input_name,
                trigger,
            );
            if !has_trigger_input {
                tracing::warn!(
                    input_name = %input_name,
                    "DataTrigger policy without matching trigger input — falling back \
                     to period_ms = 100. The generated node will be periodic, not \
                     data-triggered."
                );
                "period_ms = 100".to_string()
            } else {
                // DataTrigger is field-driven via `#[input(trigger)]`;
                // no node-level macro attr is emitted.
                String::new()
            }
        }
        None => {
            // No policy declared in the metadata. With a trigger field
            // the bare `#[cerulion_node]` is complete: the field carries
            // the policy. With regular inputs only, the bare attribute
            // is all this function may write (it never invents a
            // policy), and the macro refuses that node at build time
            // ("no trigger policy"), so the emit below says so in a
            // `// TODO` the user reads before the compiler does.
            //
            // The `node_create_with_options` strict gate rejects
            // `policy=None && inputs empty && trigger none` at the
            // engine API boundary, so this function never sees
            // that shape. The `debug_assert!` below pins that
            // invariant: if it ever fires, a new caller has been
            // added that bypasses the engine API gate, and the
            // generator should not silently paper over the bug
            // with a default policy.
            debug_assert!(
                has_trigger_input || !metadata.inputs.is_empty(),
                "generate_macro_lib_rs called with policy=None, no trigger field, \
                 and no inputs. node_create_with_options should have rejected this \
                 shape — a new caller must have been added that bypasses the gate. \
                 metadata.node_type = {:?}",
                metadata.node_type,
            );
            String::new()
        }
    };
    // Two scaffold shapes cannot build as written, because `node create`
    // takes at most one `-i` and one `-T` per call and this function never
    // invents a trigger. Say what is missing in the file itself, above the
    // attribute the compiler will point at.
    //
    // The notes deliberately never spell the macro attribute or the trigger
    // field attribute in full: `set_macro_policy_arg`, `clear_macro_policy_args`
    // and `source_has_input_trigger_field` locate those by TEXT, so a comment
    // carrying either literal is rewritten (or counted) in place of the code.
    let needs_sync_triggers = matches!(
        metadata.policy.as_ref(),
        Some(cerulion_core::MacroPolicy::Sync { .. })
            | Some(cerulion_core::MacroPolicy::UnboundedSync)
    );
    if metadata.policy.is_none() && !has_trigger_input {
        src.push_str("// TODO: this node has no trigger policy yet, so it does not build.\n");
        src.push_str("// Give it one, then delete this note: for the input that should fire it,\n");
        src.push_str(&format!(
            "// `cerulion node modify {} --policy data_trigger=<INPUT>`; for a timer,\n",
            metadata.node_type
        ));
        src.push_str(&format!(
            "// `cerulion node modify {} --policy period_ms=<N>`.\n",
            metadata.node_type
        ));
    } else if needs_sync_triggers {
        src.push_str("// TODO: a sync node aligns two or more TRIGGER inputs, so this one does\n");
        src.push_str("// not build yet. Add the other inputs with\n");
        src.push_str(&format!(
            "// `cerulion node modify {} -i SCHEMA NAME`, then change the attribute on\n",
            metadata.node_type
        ));
        src.push_str(
            "// each input to align from `input` to `input(trigger)` and delete this note.\n",
        );
    }
    if macro_args.is_empty() {
        src.push_str("#[cerulion_node]\n");
    } else {
        src.push_str(&format!("#[cerulion_node({})]\n", macro_args));
    }
    src.push_str("#[derive(Default)]\n");
    src.push_str(&format!("struct {}Node {{\n", pascal));

    // Emit input fields with #[input] attrs (mark trigger if matching).
    for input in &metadata.inputs {
        let schema_type = input
            .schema
            .as_deref()
            .map(|s| schema_to_ident(s).to_string())
            .unwrap_or_else(|| "()".to_string());
        let trigger_attr = if Some(input.name.as_str()) == trigger {
            "(trigger)"
        } else {
            ""
        };
        src.push_str(&format!(
            "    #[input{}]\n    {}: {},\n",
            trigger_attr, input.name, schema_type
        ));
    }
    // Emit output fields with #[output] attrs.
    for output in &metadata.outputs {
        let schema_type = output
            .schema
            .as_deref()
            .map(|s| schema_to_ident(s).to_string())
            .unwrap_or_else(|| "()".to_string());
        src.push_str(&format!(
            "    #[output]\n    {}: {},\n",
            output.name, schema_type
        ));
    }
    src.push_str("    tick_count: u32,\n");
    src.push_str("}\n\n");

    // Generate tick impl via #[cerulion_node_impl] (zero-copy AST rewriter).
    src.push_str("#[cerulion_node_impl]\n");
    src.push_str(&format!("impl {}Node {{\n", pascal));
    src.push_str("    fn tick(&mut self) -> Result<(), NodeError> {\n");
    src.push_str("        self.tick_count += 1;\n");

    // For each output, write a no-op assignment so the AST rewriter wires
    // an OutputProxy into scope.
    for output in &metadata.outputs {
        if let Some(ref schema) = output.schema {
            let type_name = schema_to_ident(schema);
            src.push_str(&format!(
                "        // TODO: populate {} ({})\n",
                output.name, type_name
            ));
        }
    }

    // For each input, touch one field so the rewriter wires an InputView in.
    for input in &metadata.inputs {
        src.push_str(&format!("        // TODO: read from self.{}\n", input.name));
    }

    src.push_str("        Ok(())\n");
    src.push_str("    }\n");

    // An `#[cerulion_node(external)]` node MUST define an
    // `external_source` method — without it `#[cerulion_node_impl]` is a hard
    // compile error (the required-method contract). Scaffold the simplest valid
    // source, `HostDriven` (fires only via a host `trigger_external`), so the
    // freshly-created node compiles immediately; the user swaps in a device `Fd`
    // or a `Blocking` source as their driver requires. `cerulion graph run`
    // refuses a node that still returns `HostDriven`, so the stub says that in a
    // `// TODO` instead of leaving it for the launch refusal to explain.
    // `ExternalSource` comes in via the prelude glob imported at the top of the
    // file.
    if is_external {
        src.push_str("\n    fn external_source(&mut self) -> ExternalSource {\n");
        src.push_str(
            "        // TODO: return what wakes this driver: `ExternalSource::Fd(fd)` for a\n",
        );
        src.push_str(
            "        // device file descriptor, or `ExternalSource::Blocking(..)` for a\n",
        );
        src.push_str(
            "        // blocking SDK call. `HostDriven` compiles, but `cerulion graph run`\n",
        );
        src.push_str(
            "        // refuses to run a node that returns it: nothing would ever fire it.\n",
        );
        src.push_str("        ExternalSource::HostDriven\n");
        src.push_str("    }\n");
    }

    src.push_str("}\n");

    src
}

/// Generate the `lib.rs` source for a node crate using raw FFI (legacy template).
///
/// This is the original template that produces explicit `extern "C"` functions.
/// Use `--raw-ffi` flag to select this template.
pub fn generate_lib_rs(metadata: &NodeMetadata) -> String {
    let mut src = String::new();

    // A node is a shared library the graph runtime loads, not a program: its
    // `println!` lands on whatever stdout the host happens to have, bypassing
    // the log filter, and on a robot that is how a disk fills. So the scaffold
    // makes it a clippy error and points at the alternative. Scoped
    // `not(test)`, so the node's own unit tests still print freely; delete the
    // line if you have a reason to print from node code.
    src.push_str("// A node is a LIBRARY the runtime loads — log with `tracing`,\n");
    src.push_str("// never `println!`. (Your own #[cfg(test)] tests may print.)\n");
    src.push_str("#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]\n");
    src.push('\n');

    // Imports — the raw-FFI template's placeholder body never references
    // `cerulion_core::prelude::*`, so the prelude is NOT pre-imported.
    // Users replacing the placeholder body with real logic should add the
    // imports they need — same as any Rust crate. Pre-importing the
    // prelude would trip `unused_imports = deny` under the workspace's
    // strict lint contract (see AGENTS.md). The macro template
    // (`generate_macro_lib_rs`) is the recommended path and DOES import
    // the prelude because its generated body actually uses it.
    //
    // `RefCell` and `CString` are referenced via
    // fully-qualified `::std::cell::RefCell` and `::std::ffi::CString`
    // paths in the LAST_ERROR machinery so a `use
    // native_ros2_messages::std_msgs::CString;` (theoretical) cannot
    // shadow them. The realistic shadow risk is `String` (std_msgs/String
    // exists today); the rest are defense-in-depth. Pre-existing emit
    // (`HashMap`, `Mutex`, `AtomicU64`, `Ordering`) retains its bare
    // imports — those names don't collide with current ROS schemas.
    src.push_str("use std::collections::HashMap;\n");
    src.push_str("use std::sync::atomic::{AtomicU64, Ordering};\n");
    src.push_str("use std::sync::Mutex;\n");
    src.push('\n');

    // Do not pre-import port schema
    // types. The raw-FFI placeholder body never references them, so
    // emitting `use native_ros2_messages::sensor_msgs::Image;` (etc.)
    // would trip `unused_imports = deny` under the workspace's strict
    // lint contract for any node with one or more port schemas. Users
    // replacing the placeholder body add the imports they need — same
    // pattern as the `cerulion_core::prelude::*` drop (see the Imports
    // block comment above). The macro template (`generate_macro_lib_rs`)
    // is unaffected; its struct fields use the schema types, so the
    // imports are not unused there.

    // Node state struct
    src.push_str(&format!(
        "struct {}State {{\n    tick_count: u64,\n}}\n\n",
        to_pascal_case(&metadata.node_type)
    ));

    // Static state — handle-based multi-instance support
    src.push_str("static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);\n");
    src.push_str(&format!(
        "static NODES: Mutex<Option<HashMap<u64, {}State>>> = Mutex::new(None);\n\n",
        to_pascal_case(&metadata.node_type)
    ));

    // Per-cdylib thread-local for the most recent FFI error
    // message. Init/tick/shutdown stash a string here on every non-success
    // path; the host pulls it via `cerulion_take_last_error()` immediately
    // after the offending FFI call and releases it via
    // `cerulion_free_error()`. Thread-local rather than a global Mutex
    // because the graph runtime is single-threaded per node — a separate
    // tick on a different thread cannot clobber another thread's unread
    // error. `const { ... }` keeps first-touch zero-cost.
    //
    // Fully-qualified `::std::string::String` and
    // `::std::ffi::CString` paths in the emit — bare `String` would be
    // shadowed by a `use native_ros2_messages::std_msgs::String;` import
    // emitted alongside, breaking compilation for any raw-FFI node with
    // a `std_msgs/String` port. The macro form does the same. Tests
    // miss this when the unit-test metadata uses empty
    // inputs/outputs; it takes a port-bound
    // test of the emitted source to catch it.
    src.push_str(
        r#"thread_local! {
    static LAST_ERROR: ::std::cell::RefCell<Option<::std::ffi::CString>> = const { ::std::cell::RefCell::new(None) };
}

fn __cer_set_last_error(msg: ::std::string::String) {
    let cstring = ::std::ffi::CString::new(msg.replace('\0', "\\0"))
        .unwrap_or_else(|_| ::std::ffi::CString::new("error message contained nul byte").unwrap());
    LAST_ERROR.with(|cell| {
        *cell.borrow_mut() = Some(cstring);
    });
}

"#,
    );

    // ABI version. Emit the CONST NAME (not its baked value) so a generated
    // raw-FFI node auto-tracks the `cerulion_core` it is compiled against — a
    // host ABI bump then needs no regeneration of the user's node (the prior
    // `{value}` literal silently froze the version at `node create` time and
    // failed to load after the next bump).
    src.push_str("#[no_mangle]\npub extern \"C\" fn cerulion_abi_version() -> u32 {\n    ");
    src.push_str("::cerulion_core::CERULION_ABI_VERSION\n");
    src.push_str("}\n\n");

    // ABI v22 rustc fingerprint. Required at every ABI 22 and later host: it
    // reports the rustc that compiled THIS cdylib so the loader can refuse a
    // load where the host and this node disagree on how a `repr(Rust)` type
    // crossing the FFI (for example `Option<FrozenSlot>`) is laid out, a class
    // the ABI version alone cannot see. Emitted the same "const name, not
    // baked value" way as the ABI version above, for the same reason: a host
    // rebuild picks up the new fingerprint with no template regeneration.
    src.push_str(
        "#[no_mangle]\npub extern \"C\" fn cerulion_rustc_fingerprint() -> *const ::std::ffi::c_char {\n    ",
    );
    src.push_str("::cerulion_core::rustc_fingerprint_cstr()\n");
    src.push_str("}\n\n");

    // Node info function with marker comments
    src.push_str(&generate_info_fn(metadata));
    src.push('\n');

    // Init function (handle-based FFI: takes *mut u8, returns u64 handle).
    //
    // The warning block that follows lives in the
    // EMITTED source so users opening their generated `lib.rs` see it.
    // It documents the safety gap between the placeholder body (no-op
    // counter, cannot fail) and a user-written body that might panic or
    // return Err. Without this warning, users assume the cdylib carries
    // the same panic-safety + Err-propagation as `#[cerulion_node]` and
    // discover the gap only after a production crash.
    //
    // Generated function also mirrors the macro form's null-check on
    // `ctx_ptr`: a null pointer from the host
    // loader is a contract violation that must NOT result in a fresh
    // handle. Return 0 + LAST_ERROR with the macro-parity wording.
    src.push_str(
        r#"// =================================================================
// NOTE TO USERS REPLACING THE PLACEHOLDER BODY
// =================================================================
// The template's tick body is a no-op counter (`tick_count += 1`) and
// CANNOT FAIL. As a result, this raw-FFI template DOES NOT EMIT:
//   - `init failed: {e}` / `tick failed: {e}` Err-propagation wiring
//     (the placeholder body has no `Result<_, _>` return).
//   - `std::panic::catch_unwind(...)` panic-safety wrapper around
//     init/tick/shutdown (the placeholder body cannot panic).
//
// If your real logic CAN return Err OR CAN panic, you MUST either:
//   1. Migrate to `#[cerulion_node]` (drop `--raw-ffi` from
//      `cerulion node create`) — the recommended path; the macro
//      carries full panic-safety + Err propagation with no extra wiring.
//   2. Hand-roll the equivalent wiring in every entry point below:
//      wrap the body in `catch_unwind`, and on the error path call
//      `__cer_set_last_error(format!("init failed: {}", e))` before
//      returning the matching non-success code.
//
// Unwinding across the FFI boundary aborts the process (and was
// undefined behaviour on older toolchains pre-Rust-1.71). Either way,
// a panic must NOT cross the cdylib boundary back into the host.
// Failing to translate Err to LAST_ERROR silently strips the diagnostic
// — the host falls back to a generic "no detail provided" message that
// is nearly impossible to debug from operator logs.

"#,
    );
    let state_name = format!("{}State", to_pascal_case(&metadata.node_type));
    src.push_str(&format!(
        r#"#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {{
    if ctx_ptr.is_null() {{
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_init: NodeContext pointer was null",
        ));
        return 0;
    }}
    // SAFETY: `ctx_ptr` is a `NodeContext` the runtime boxed and handed to
    // this cdylib. Ownership transferred with it — the runtime will not free
    // it — so pairing with `Box::from_raw` is required to avoid leaking one
    // `NodeContext` per init call. This placeholder body doesn't use
    // transport, so dropping the context at end of scope is the correct
    // behavior. A node written with `#[cerulion_node]` instead STORES the
    // boxed context so it can publish / subscribe — and so should you, once
    // this node does real work.
    let ctx = unsafe {{
        ::std::boxed::Box::from_raw(ctx_ptr as *mut ::cerulion_core::graph::node::NodeContext)
    }};

    // KEEP THIS, and keep it in `init`. A cdylib statically links its OWN
    // copy of `iceoryx2-log`, whose level is a private static: the host
    // process setting `IOX2_LOG_LEVEL` does NOTHING for this copy, so
    // without this call iceoryx2 logs from THIS node at its crate default
    // (`Info`) no matter what you configured. That is not cosmetic — a node
    // logging at `Info` on a busy robot can emit thousands of lines a second
    // and fill the disk. The value comes from the node's FROZEN env
    // snapshot, never live `std::env`, so replay stays deterministic; empty
    // means unset, which leaves Cerulion's default (`error`). Nodes written
    // with `#[cerulion_node]` get this generated for them.
    {{
        let iox2_log = ctx.env_str("IOX2_LOG_LEVEL", "");
        ::cerulion_core::iceoryx_logger::init_iceoryx_log_level(if iox2_log.is_empty() {{
            ::std::option::Option::None
        }} else {{
            ::std::option::Option::Some(iox2_log.as_str())
        }});
    }}

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    if let Ok(mut guard) = NODES.lock() {{
        let map = guard.get_or_insert_with(HashMap::new);
        map.insert(handle, {state_name} {{ tick_count: 0 }});
        handle
    }} else {{
        // Mutex poisoned: do NOT recover via `into_inner()` — a panic
        // while holding the lock can leave per-node state inconsistent.
        // Surface code + LAST_ERROR; let the host decide to restart.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_init: NODES mutex poisoned",
        ));
        0
    }}
}}

"#
    ));

    // Tick function (handle-based FFI: takes u64 handle)
    src.push_str(
        r#"#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    if let Ok(mut guard) = NODES.lock() {
        match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
            Some(state) => {
                state.tick_count += 1;
                0
            }
            None => {
                __cer_set_last_error(format!("cerulion_node_tick: handle {} not found", handle));
                4
            }
        }
    } else {
        // Mutex poisoned: do NOT recover via `into_inner()` — see init.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_tick: NODES mutex poisoned",
        ));
        3
    }
}

"#,
    );

    // Pump-history function (handle-based FFI: takes u64 handle).
    // ABI v7: the host calls this on every live step
    // so cdylib nodes ALSO service quiescent late joiners (the in-process
    // `pump_history` path cannot reach a cdylib node across the FFI). The
    // placeholder body has no transport, so the pump is a no-op — but the
    // symbol is REQUIRED by the loader, and the handle-lookup + error
    // codes (4 not found / 3 NODES poisoned) mirror `cerulion_node_tick`.
    // A user replacing the placeholder with a real publisher body should
    // call their node's `pump_history()` here.
    src.push_str(
        r#"#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    if let Ok(mut guard) = NODES.lock() {
        match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
            Some(_state) => {
                // Placeholder node has no publishers — nothing to re-pump.
                0
            }
            None => {
                __cer_set_last_error(format!(
                    "cerulion_node_pump_history: handle {} not found",
                    handle
                ));
                4
            }
        }
    } else {
        // Mutex poisoned: do NOT recover via `into_inner()` — see init.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_pump_history: NODES mutex poisoned",
        ));
        3
    }
}

"#,
    );

    // Shutdown function (handle-based FFI: takes u64 handle)
    // Shutdown returns code 4 on missing handle
    // (never a silent 0). Every non-success path also
    // stashes a human-readable message in LAST_ERROR, mirroring the
    // macro-form FFI in `cerulion_macros/src/codegen.rs` so the host
    // surfaces real diagnostics rather than bare codes.
    src.push_str(
        r#"#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => {
            // Mutex poisoned: do NOT recover via `into_inner()` — see init.
            __cer_set_last_error(::std::string::String::from(
                "cerulion_node_shutdown: NODES mutex poisoned",
            ));
            return 3;
        }
    };
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_node) => 0,
        None => {
            __cer_set_last_error(format!(
                "cerulion_node_shutdown: handle {} not found (already shut down or never registered)",
                handle
            ));
            4
        }
    }
}

"#,
    );

    // Take/free the most recent error message off the
    // thread-local. Required by `DylibNodeEntry::load` — the host
    // pulls the message via `cerulion_take_last_error` after any
    // non-zero return from init/tick/shutdown and frees it via
    // `cerulion_free_error`. Allocator pairing is critical: the
    // CString MUST be freed by the same cdylib that allocated it.
    // `cerulion_free_error` is `unsafe` because it dereferences a raw
    // pointer (matching the hand-written
    // `crates/test_fixtures/test_node_cdylib/src/lib.rs` precedent and
    // satisfying clippy::not_unsafe_ptr_arg_deref under the strict
    // workspace lint contract).
    src.push_str(
        r#"#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
        Some(cstr) => cstr.into_raw(),
        None => std::ptr::null_mut(),
    })
}

/// # Safety
///
/// `ptr` must be a pointer previously returned by
/// `cerulion_take_last_error` from this same cdylib (allocator pairing
/// requirement). Null is permitted and is a no-op. Calling with any
/// other pointer is undefined behaviour.
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: pointer originated from `CString::into_raw` in
    // `cerulion_take_last_error`. Reclaiming via `from_raw`
    // returns ownership and the CString drops here.
    let _ = ::std::ffi::CString::from_raw(ptr);
}
"#,
    );

    src
}

/// Generate the `cerulion_node_info()` function with marker comments.
///
/// Uses a static byte array with null terminator — zero allocation, zero leak.
/// This matches the pattern from `cerulion_macros/src/codegen.rs`. The
/// JSON carries no `node_type` field; the type is the folder name and
/// is resolved at graph-load time, not by the cdylib.
pub fn generate_info_fn(metadata: &NodeMetadata) -> String {
    let inputs_json: Vec<String> = metadata
        .inputs
        .iter()
        .map(|p| format!("\\\"{}\\\"", json_escape(&p.name)))
        .collect();
    let outputs_json: Vec<String> = metadata
        .outputs
        .iter()
        .map(|p| format!("\\\"{}\\\"", json_escape(&p.name)))
        .collect();

    // Emit `policy:` JSON when metadata carries a non-default
    // trigger policy. Serialize the shared `PolicyJson` (the same
    // wire-shape struct the cdylib FFI emits and the host parser
    // consumes), then escape the result for embedding in a Rust
    // byte-string literal.
    let policy_json: String = match metadata.policy.as_ref() {
        Some(p) => {
            let wire = cerulion_core::PolicyJson::from(p);
            let json = serde_json::to_string(&wire)
                .expect("PolicyJson serialization is infallible for the documented variants");
            // Escape the JSON for embedding in `b"..."`: every `"`
            // becomes `\\\"` in the source string (one backslash in
            // the file, escaped twice for the Rust string literal).
            // `\\` is already absent from `serde_json` output for
            // these variants; if a future variant ever embeds a
            // backslash, the same `json_escape` helper handles it.
            format!(",\\\"policy\\\":{}", json_escape_for_byte_string(&json))
        }
        None => String::new(),
    };

    let info_json = format!(
        "{{\\\"inputs\\\":[{}],\\\"outputs\\\":[{}]{}}}",
        inputs_json.join(","),
        outputs_json.join(","),
        policy_json,
    );

    let mut s = String::new();
    s.push_str("// CERULION:INFO_START\n");
    s.push_str(&format!(
        "static INFO_BYTES: &[u8] = b\"{}\\0\";\n\n",
        info_json
    ));
    s.push_str("#[no_mangle]\n");
    s.push_str("pub extern \"C\" fn cerulion_node_info() -> *const std::ffi::c_char {\n");
    s.push_str("    // SAFETY: INFO_BYTES is a static &[u8] with a trailing null byte.\n");
    s.push_str(
        "    // Casting *const u8 to *const c_char is safe: identical layout, valid and 'static.\n",
    );
    s.push_str("    INFO_BYTES.as_ptr() as *const std::ffi::c_char\n");
    s.push_str("}\n");
    s.push_str("// CERULION:INFO_END\n");
    s
}

/// Check whether a source file uses the `#[cerulion_node]` macro pattern.
///
/// Both `#[cerulion_node]` (bare form, when the node has only
/// declarative `#[input]`/`#[output]` field attrs without trigger
/// hints) and `#[cerulion_node(...)]` are recognised.
///
/// AST-based via `syn`: parses the file with `syn::parse_file` and
/// walks every reachable `Item::Struct` (descending into
/// `Item::Mod` so sources that wrap a node in an organisational
/// `mod inner { ... }` block still match), returning `true` only
/// when the struct carries an attribute whose path is exactly the
/// single bare ident `cerulion_node` — no leading `::`, no
/// qualification, no path arguments. See `attr_is_cerulion_node`
/// for the precise rejection rules. Substring scans on the raw
/// source are NOT used because they false-match `cerulion_node`
/// inside line comments, doc comments, block comments, raw string
/// literals, and similar-named macro paths
/// (`#[other::cerulion_node]`).
///
/// Fallback: if `syn::parse_file` returns `Err` (e.g. the user is
/// in the middle of an edit and the file does not yet parse), we
/// fall back to the legacy substring check on the raw source. This
/// is conservative — false positives on partial source are
/// preferable to silently misclassifying a node mid-edit. Note that
/// the substring check itself has no further fallback, but
/// substring-matched syntactically-incomplete source is the only
/// case where we ever return `true` without a `syn`-confirmed
/// struct attribute, so the conservative default is bounded.
pub fn is_macro_based(source: &str) -> bool {
    match syn::parse_file(source) {
        Ok(file) => file.items.iter().any(item_carries_cerulion_node_struct),
        Err(_) => {
            // Conservative fallback for unparseable Rust (e.g. mid-edit
            // source). A more lenient tokenization is possible;
            // the accepted trade-off is that the
            // substring check may over-match in pathological cases —
            // the alternative (returning `false` and silently routing
            // the file as raw-FFI) is the worse failure mode.
            source.contains("#[cerulion_node(") || source.contains("#[cerulion_node]")
        }
    }
}

/// Walk an item recursively, returning `true` iff it is (or
/// contains, for `mod` items with inline content) a `struct` with a
/// bare `#[cerulion_node]` attribute. The recursion lets us match
/// hand-written sources that wrap a node in an organisational
/// `mod inner { ... }` or a `#[cfg(test)] mod tests { ... }` —
/// a substring match catches these by accident; the
/// AST walk matches them intentionally.
fn item_carries_cerulion_node_struct(item: &syn::Item) -> bool {
    match item {
        syn::Item::Struct(item_struct) => item_struct.attrs.iter().any(attr_is_cerulion_node),
        syn::Item::Mod(item_mod) => item_mod
            .content
            .as_ref()
            .map(|(_, items)| items.iter().any(item_carries_cerulion_node_struct))
            .unwrap_or(false),
        _ => false,
    }
}

/// Return `true` iff the given attribute is exactly
/// `#[cerulion_node]` or `#[cerulion_node(...)]` — single-segment
/// path with the ident `cerulion_node` and no leading `::`.
///
/// Uses `syn::Path::is_ident`, which is the canonical syn API
/// for "exactly this single bare ident": it returns `true` iff
/// `leading_colon.is_none()` AND there is exactly one segment
/// with no path arguments AND the ident matches. This rejects
/// every off-canonical attribute syntax that does parse —
/// `#[other::cerulion_node]`, `#[::cerulion_node]`,
/// `#[cerulion_node::extra]` — without us having to enumerate
/// them. Forms like `#[cerulion_node<T>]` aren't valid attribute
/// syntax in the first place: `syn::parse_file` returns `Err`
/// before this helper ever runs, and the substring fallback in
/// `is_macro_based` decides routing for those.
fn attr_is_cerulion_node(attr: &syn::Attribute) -> bool {
    attr.path().is_ident("cerulion_node")
}

/// Inject a `#[input]` or `#[output]` field declaration into the
/// `#[cerulion_node]`-annotated struct in `source`.
///
/// Strategy: parse `source` via `syn::parse_file`, find the
/// `#[cerulion_node]`-annotated `ItemStruct`, and use
/// `proc_macro2::Span::byte_range()` (stable since proc-macro2
/// 1.0.62) to get the EXACT byte position of the struct's
/// `{ ... }` body in the raw source. No substring search, no
/// brace-depth counting — syn's AST is the source of truth, and
/// its tokenizer has already stripped comments and resolved
/// strings/chars before producing the byte ranges. Splice the new
/// field declaration before the closing `}` via pure byte-offset
/// arithmetic.
///
/// Limitations:
/// - Tuple structs and unit structs aren't supported (the macro
///   itself rejects them; we treat them as a hard error here too).
pub fn inject_macro_field(
    source: &str,
    field_name: &str,
    field_type: &str,
    is_output: bool,
    is_trigger: bool,
) -> Result<String, String> {
    // 1. Parse the source and locate the macro struct AST node.
    let file = syn::parse_file(source).map_err(|e| {
        format!(
            "could not parse source as Rust to locate #[cerulion_node] struct: {}",
            e
        )
    })?;
    let item_struct = find_macro_item_struct(&file.items)
        .ok_or_else(|| "no `#[cerulion_node]`-annotated struct found in source".to_string())?;

    // 2. Get exact body-brace byte positions from syn. `DelimSpan`
    // exposes `.open()` and `.close()` Spans for the `{` and `}`
    // tokens individually; each `Span::byte_range()` is stable
    // since proc-macro2 1.0.62. The body interior starts AFTER
    // the open `{`'s last byte; the close `}` starts at the
    // close Span's first byte.
    let (body_start, close_byte) = match &item_struct.fields {
        syn::Fields::Named(named) => {
            let open_range = named.brace_token.span.open().byte_range();
            let close_range = named.brace_token.span.close().byte_range();
            (open_range.end, close_range.start)
        }
        syn::Fields::Unnamed(_) | syn::Fields::Unit => {
            return Err(format!(
                "struct `{}` is not a braced struct (tuple/unit structs not supported by `#[cerulion_node]`)",
                item_struct.ident
            ));
        }
    };
    if close_byte > source.len() || body_start > close_byte {
        // Defensive: should never trip because syn produced
        // these spans from `source` itself, but guard anyway so
        // downstream slice indexing can't panic on a malformed
        // Span.
        return Err(format!(
            "syn-reported span for `struct {}` body is out of bounds for the source",
            item_struct.ident
        ));
    }

    // 5. Build the field declaration text. Match the indentation
    // `generate_macro_lib_rs` emits, so a freshly-created node and a
    // `node modify`-extended node are textually homogeneous. Neither
    // writes a lint suppression on a port: the macro's generated code
    // uses every port field, so an untouched scaffold builds without a
    // `dead_code` warning (measured), and the shape matches the nodes
    // the README and the examples show.
    let attr = if is_output {
        "#[output]".to_string()
    } else if is_trigger {
        "#[input(trigger)]".to_string()
    } else {
        "#[input]".to_string()
    };
    let field_decl = format!("    {}\n    {}: {},\n", attr, field_name, field_type);

    // 6. Splice the new field in before the closing `}`. Look
    // backwards from `close_byte` to find the start of the line
    // containing `}` so we splice on a clean line boundary.
    //
    // Edge case: when the struct body is single-line — e.g.
    // `struct Foo {}` or `struct Foo { x: u32 }` — the `\n`
    // preceding `close_byte` is the one BEFORE `struct Foo`,
    // and `line_start` would land before the struct declaration
    // itself. Splicing there produces invalid Rust (the new
    // field becomes a top-level item). Detect this case
    // (`line_start <= body_start`) and fall back to inserting
    // immediately before `close_byte`, prepending a newline so
    // the new field is on its own line.
    let line_start = source[..close_byte].rfind('\n').map(|i| i + 1).unwrap_or(0);
    let mut result = String::with_capacity(source.len() + field_decl.len() + 1);
    if line_start > body_start {
        // Multi-line body: splice on the indented line containing `}`.
        //
        // Trailing-comma check: a hand-written struct with no
        // trailing comma on the last field (e.g.
        // `struct Foo {\n    x: u32\n}`) would produce invalid
        // Rust when we splice `    #[input] new: T,` between the
        // last field and `}` — `x: u32\n    #[input]\n    new: T,`
        // is a parse error. Look at the last non-whitespace char
        // in the body region BEFORE `line_start`; if it isn't `,`
        // or `{` (empty body — handled by the single-line branch),
        // inject a comma after that char before our splice.
        let body_before_close = &source[body_start..line_start];
        let trimmed = body_before_close.trim_end();
        let needs_comma = !trimmed.is_empty() && !trimmed.ends_with(',') && !trimmed.ends_with('{');
        if needs_comma {
            // Position the comma right after the last non-whitespace
            // char so we don't accidentally insert it inside trailing
            // whitespace / comments.
            let comma_pos = body_start + trimmed.len();
            result.push_str(&source[..comma_pos]);
            result.push(',');
            result.push_str(&source[comma_pos..line_start]);
        } else {
            result.push_str(&source[..line_start]);
        }
        result.push_str(&field_decl);
        result.push_str(&source[line_start..]);
    } else {
        // Single-line body: insert immediately before `close_byte`,
        // prepending a newline so the new field starts on its own
        // line. The closing `}` ends up on its own line too.
        //
        // If the existing body has a non-comma-terminated final
        // field (e.g. `struct Foo { x: u32 }` — `x: u32` has no
        // trailing comma), Rust requires the comma between `u32`
        // and our new field. Detect by trimming the body and
        // checking the last non-whitespace char.
        let body_str = &source[body_start..close_byte];
        let trimmed_body = body_str.trim_end();
        let needs_comma = !trimmed_body.is_empty() && !trimmed_body.ends_with(',');
        result.push_str(&source[..close_byte]);
        if needs_comma {
            result.push(',');
        }
        result.push('\n');
        result.push_str(&field_decl);
        result.push_str(&source[close_byte..]);
    }
    Ok(result)
}

/// Find the first `#[cerulion_node]`-annotated `ItemStruct`
/// reachable from `items` (recursing into modules to mirror
/// `is_macro_based`'s walk). Returns the AST node so callers
/// can read its span, ident, and fields directly — instead of
/// re-finding the struct via substring search.
fn find_macro_item_struct(items: &[syn::Item]) -> Option<&syn::ItemStruct> {
    for item in items {
        if let syn::Item::Struct(item_struct) = item {
            if item_struct
                .attrs
                .iter()
                .any(|a| a.path().is_ident("cerulion_node"))
            {
                return Some(item_struct);
            }
        }
        if let syn::Item::Mod(item_mod) = item {
            if let Some((_, nested)) = &item_mod.content {
                if let Some(found) = find_macro_item_struct(nested) {
                    return Some(found);
                }
            }
        }
    }
    None
}

/// Toggle the `external` token in a `#[cerulion_node(...)]` macro
/// attribute. Adds it if `enable` is true and not present; removes
/// it if `enable` is false and present. Bare `#[cerulion_node]`
/// (no parens) gets converted to `#[cerulion_node(external)]`
/// when `enable` is true. Raw-FFI nodes are not supported (the
/// macro arg has no equivalent in raw-FFI source); the caller
/// errors with a descriptive message in that case.
pub fn toggle_macro_external_flag(source: &str, enable: bool) -> Result<String, String> {
    // Bare `#[cerulion_node]` form first.
    let bare = "#[cerulion_node]";
    let parens_open = "#[cerulion_node(";
    let has_bare = source.contains(bare) && !source.contains(parens_open);
    if has_bare {
        if !enable {
            // Already off; nothing to do.
            return Ok(source.to_string());
        }
        // Convert bare → `#[cerulion_node(external)]`.
        return Ok(source.replacen(bare, "#[cerulion_node(external)]", 1));
    }

    // Parens form: locate the args, edit them.
    let macro_start = source
        .find(parens_open)
        .ok_or_else(|| "could not find `#[cerulion_node(` in source".to_string())?;
    let after_open = macro_start + parens_open.len();
    let mut depth = 1u32;
    let mut paren_close: Option<usize> = None;
    for (i, ch) in source[after_open..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    paren_close = Some(after_open + i);
                    break;
                }
            }
            _ => {}
        }
    }
    let paren_close = paren_close
        .ok_or_else(|| "could not find closing `)` for `#[cerulion_node(...)]`".to_string())?;
    let args_str = &source[after_open..paren_close];
    let trimmed = args_str.trim();

    // Tokenise existing args by `,`, looking for an `external`
    // token. The macro's full grammar is more permissive
    // (key-value pairs, `external`), but for this function's purposes
    // a comma-split + token-trim suffices — every arg the macro
    // accepts is either a plain ident (`external`) or a
    // `key = value` pair (`period_ms = 100`); both forms are
    // safe to trim and rejoin.
    let parts: Vec<String> = if trimmed.is_empty() {
        Vec::new()
    } else {
        trimmed.split(',').map(|s| s.trim().to_string()).collect()
    };
    let has_external = parts.iter().any(|p| p == "external");
    let new_parts: Vec<String> = if enable {
        if has_external {
            parts // already enabled, keep verbatim
        } else {
            // External is mutually exclusive with period_ms /
            // sync_window_ms (the macro's compile-time
            // validator rejects the combo). Strip any conflicting
            // policy-attr key=value pair and keep only `external`
            // so the resulting source is buildable AND
            // `parse_node_metadata` reports policy = External
            // post-toggle. Args that aren't policy keys (none today,
            // but defensively kept) pass through unchanged.
            const POLICY_KEYS: &[&str] = &["period_ms", "sync_window_ms"];
            let mut p: Vec<String> = parts
                .into_iter()
                .filter(|arg| {
                    let key = arg.split('=').next().map(str::trim).unwrap_or("");
                    !POLICY_KEYS.contains(&key)
                })
                .collect();
            p.push("external".to_string());
            p
        }
    } else {
        parts.into_iter().filter(|p| p != "external").collect()
    };
    let mut result = String::with_capacity(source.len());
    if new_parts.is_empty() {
        // Stripping the only arg would leave `#[cerulion_node()]` —
        // syntactically valid Rust but semantically invalid macro
        // input (the macro requires SOME trigger policy hint:
        // `period_ms`, `external`, or a field
        // `#[input(trigger)]`).
        //
        // - If the source has a `#[input(trigger)]` field already,
        //   declarative trigger inference picks up the policy and
        //   bare `#[cerulion_node]` is buildable. Drop the parens
        //   entirely.
        // - Otherwise, fall back to the same default
        //   `generate_macro_lib_rs` uses for fresh nodes
        //   (`period_ms = 100`) so the source stays buildable. The
        //   user can then `node modify --policy external` again
        //   or hand-edit if they want a different trigger.
        if source_has_input_trigger_field(source) {
            let macro_close = paren_close + 1; // include the `)`
            let bracket_close = source[macro_close..]
                .find(']')
                .map(|i| macro_close + i)
                .ok_or_else(|| {
                    "could not find closing `]` for `#[cerulion_node(...)]`".to_string()
                })?;
            result.push_str(&source[..macro_start]);
            result.push_str("#[cerulion_node]");
            result.push_str(&source[bracket_close + 1..]);
        } else {
            // Replace `(external)` with `(period_ms = 100)`.
            result.push_str(&source[..after_open]);
            result.push_str("period_ms = 100");
            result.push_str(&source[paren_close..]);
        }
    } else {
        let new_args = new_parts.join(", ");
        result.push_str(&source[..after_open]);
        result.push_str(&new_args);
        result.push_str(&source[paren_close..]);
    }
    Ok(result)
}

/// Return `true` iff `source` contains a `#[input(trigger)]`
/// field attribute. Used by `toggle_macro_external_flag` to pick
/// between bare `#[cerulion_node]` (valid when a trigger field
/// exists) and the `period_ms = 100` fallback (when no trigger
/// field exists, so bare form would fail macro expansion).
///
/// Substring-based check is sufficient because:
/// - The caller already verified the source parses via syn (the
///   detection is a downstream consumer of `is_macro_based`).
/// - `#[input(trigger)]` is a distinctive substring; over-detection
///   from a doc-string example is benign (we'd emit the bare form,
///   which is syntactically valid; macro expansion then either
///   finds the real `#[input(trigger)]` or fails — same failure
///   the user had pre-modify).
fn source_has_input_trigger_field(source: &str) -> bool {
    source.contains("#[input(trigger)]")
}

/// Strip every trigger-policy arg (`period_ms = N`,
/// `sync_window_ms = N`, `external`) from the
/// `#[cerulion_node(...)]` attribute. If the result is an empty
/// arg list, drops to bare `#[cerulion_node]`. Used by
/// `node_modify_add_port` when designating an input as the
/// trigger — the new `#[input(trigger)]` field provides the
/// policy via declarative inference, so the macro args MUST NOT
/// carry a competing hint.
pub fn clear_macro_policy_args(source: &str) -> Result<String, String> {
    if source.contains("#[cerulion_node]") && !source.contains("#[cerulion_node(") {
        return Ok(source.to_string());
    }
    let parens_open = "#[cerulion_node(";
    let macro_start = source
        .find(parens_open)
        .ok_or_else(|| "could not find `#[cerulion_node(` in source".to_string())?;
    let after_open = macro_start + parens_open.len();
    let mut depth = 1u32;
    let mut paren_close: Option<usize> = None;
    for (i, ch) in source[after_open..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    paren_close = Some(after_open + i);
                    break;
                }
            }
            _ => {}
        }
    }
    let paren_close = paren_close
        .ok_or_else(|| "could not find closing `)` for `#[cerulion_node(...)]`".to_string())?;
    let args_str = &source[after_open..paren_close];
    let kept: Vec<String> = args_str
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|p| !p.is_empty())
        .filter(|p| {
            // Extract the policy-key ident regardless of formatting:
            // `period_ms = 100` (spaces around `=`) and `period_ms=100`
            // (compact) and bare `external` all collapse to their key
            // ident here. Split on `=` first to handle compact form,
            // then strip surrounding whitespace.
            let head = p.split('=').next().unwrap_or("").trim();
            !matches!(head, "period_ms" | "sync_window_ms" | "external")
        })
        .collect();
    let mut result = String::with_capacity(source.len());
    if kept.is_empty() {
        let macro_close = paren_close + 1;
        let bracket_close = source[macro_close..]
            .find(']')
            .map(|i| macro_close + i)
            .ok_or_else(|| "could not find closing `]` for `#[cerulion_node(...)]`".to_string())?;
        result.push_str(&source[..macro_start]);
        result.push_str("#[cerulion_node]");
        result.push_str(&source[bracket_close + 1..]);
    } else {
        result.push_str(&source[..after_open]);
        result.push_str(&kept.join(", "));
        result.push_str(&source[paren_close..]);
    }
    Ok(result)
}

/// Remove the `(trigger)` arg from any `#[input(trigger)]` field
/// attribute on the `#[cerulion_node]` struct, leaving a bare
/// `#[input]`. AST-based — a global `str::replace` would mutate
/// doc comments and string literals containing the same text.
pub fn clear_input_trigger_attr(source: &str) -> Result<String, String> {
    use syn::spanned::Spanned;
    let file = syn::parse_file(source).map_err(|e| format!("source did not parse as Rust: {e}"))?;
    // Collect every `#[input(trigger)]` attribute byte-range on a
    // `#[cerulion_node]`-annotated struct in source order; rewrite
    // them in reverse so each splice doesn't shift the subsequent
    // ranges.
    let mut ranges: Vec<std::ops::Range<usize>> = Vec::new();
    walk_items_for_struct(&file.items, &mut |item_struct| {
        let is_macro_node = item_struct
            .attrs
            .iter()
            .any(|a| a.path().is_ident("cerulion_node"));
        if !is_macro_node {
            return;
        }
        for field in &item_struct.fields {
            for attr in &field.attrs {
                if !attr.path().is_ident("input") {
                    continue;
                }
                let syn::Meta::List(meta_list) = &attr.meta else {
                    continue;
                };
                // Only touch attrs whose token stream is exactly
                // the bare `trigger` ident — drop `(trigger)`
                // entirely. Multi-arg lists like `(trigger,
                // depth = 4)` keep their other args; we leave them
                // alone here because removing only `trigger` from
                // such a list isn't a job for the simple
                // "clear-all-triggers" helper (and the macro
                // rejects mixing `trigger` with most other attrs
                // anyway).
                use proc_macro2::TokenTree;
                let tokens: Vec<TokenTree> = meta_list.tokens.clone().into_iter().collect();
                let is_bare_trigger = tokens.len() == 1
                    && matches!(&tokens[0], TokenTree::Ident(id) if id == "trigger");
                if is_bare_trigger {
                    ranges.push(attr.meta.span().byte_range());
                }
            }
        }
    });
    if ranges.is_empty() {
        return Ok(source.to_string());
    }
    let mut result = source.to_string();
    for range in ranges.into_iter().rev() {
        result.replace_range(range, "input");
    }
    Ok(result)
}

/// Rewrite the `#[input]` field attribute on field `field_name`
/// to `#[input(trigger)]`. Idempotent if the field already carries
/// `#[input(trigger)]`.
///
/// Errors if:
/// - No field with that name is found.
/// - The field's `#[input]` attribute carries non-trigger args
///   (e.g. `#[input(depth = 4, backpressure = block)]`) — silently dropping them
///   when we rewrite would lose user configuration. Caller must
///   reconcile manually.
pub fn promote_input_field_to_trigger(source: &str, field_name: &str) -> Result<String, String> {
    use syn::spanned::Spanned;
    let file = syn::parse_file(source).map_err(|e| format!("source did not parse as Rust: {e}"))?;
    let mut attr_byte_range: Option<std::ops::Range<usize>> = None;
    let mut had_args: bool = false;
    walk_items_for_struct(&file.items, &mut |item_struct| {
        if attr_byte_range.is_some() {
            return;
        }
        let is_macro_node = item_struct
            .attrs
            .iter()
            .any(|a| a.path().is_ident("cerulion_node"));
        if !is_macro_node {
            return;
        }
        for field in &item_struct.fields {
            let Some(ident) = field.ident.as_ref() else {
                continue;
            };
            if ident != field_name {
                continue;
            }
            for f_attr in &field.attrs {
                if !f_attr.path().is_ident("input") {
                    continue;
                }
                attr_byte_range = Some(f_attr.meta.span().byte_range());
                had_args = matches!(&f_attr.meta, syn::Meta::List(_));
                return;
            }
        }
    });
    let Some(range) = attr_byte_range else {
        return Err(format!(
            "could not locate `#[input]` attribute for field `{field_name}`"
        ));
    };
    if had_args {
        let existing = &source[range.clone()];
        if existing.contains("trigger") {
            return Ok(source.to_string());
        }
        return Err(format!(
            "field `{field_name}` already carries `#[{}]` with non-trigger args; \
             clear them by hand before promoting to trigger",
            existing.trim()
        ));
    }
    let mut result = String::with_capacity(source.len());
    result.push_str(&source[..range.start]);
    result.push_str("input(trigger)");
    result.push_str(&source[range.end..]);
    Ok(result)
}

/// Walk every reachable struct in `items`, invoking `cb` per
/// struct. Recurses into `Item::Mod` so module-nested macro nodes
/// are visited, mirroring `is_macro_based`'s walker.
fn walk_items_for_struct<'a>(items: &'a [syn::Item], cb: &mut dyn FnMut(&'a syn::ItemStruct)) {
    for item in items {
        if let syn::Item::Struct(s) = item {
            cb(s);
        }
        if let syn::Item::Mod(item_mod) = item {
            if let Some((_, nested)) = &item_mod.content {
                walk_items_for_struct(nested, cb);
            }
        }
    }
}

/// Set the trigger-policy arg in a `#[cerulion_node(...)]` macro
/// attribute to exactly `new_arg`, stripping any conflicting
/// `period_ms` / `sync_window_ms` / `external`
/// args. Bare `#[cerulion_node]` (no parens) is converted to
/// `#[cerulion_node(<new_arg>)]`. Non-policy args (none today, but
/// defensively kept) pass through unchanged.
pub fn set_macro_policy_arg(source: &str, new_arg: &str) -> Result<String, String> {
    let cleared = clear_macro_policy_args(source)?;
    // After clear, the source either has `#[cerulion_node]` (no
    // parens) or `#[cerulion_node(<non-policy args>)]`.
    let bare = "#[cerulion_node]";
    if cleared.contains(bare) && !cleared.contains("#[cerulion_node(") {
        return Ok(cleared.replacen(bare, &format!("#[cerulion_node({})]", new_arg), 1));
    }
    let parens_open = "#[cerulion_node(";
    let macro_start = cleared
        .find(parens_open)
        .ok_or_else(|| "could not find `#[cerulion_node(` in source".to_string())?;
    let after_open = macro_start + parens_open.len();
    let mut depth = 1u32;
    let mut paren_close: Option<usize> = None;
    for (i, ch) in cleared[after_open..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    paren_close = Some(after_open + i);
                    break;
                }
            }
            _ => {}
        }
    }
    let paren_close = paren_close
        .ok_or_else(|| "could not find closing `)` for `#[cerulion_node(...)]`".to_string())?;
    let args_str = cleared[after_open..paren_close].trim();
    let merged = if args_str.is_empty() {
        new_arg.to_string()
    } else {
        format!("{args_str}, {new_arg}")
    };
    let mut result = String::with_capacity(cleared.len());
    result.push_str(&cleared[..after_open]);
    result.push_str(&merged);
    result.push_str(&cleared[paren_close..]);
    Ok(result)
}

/// Ensure the source has SOME trigger-policy hint after a
/// `--no-trigger` mutation. If a `#[input(trigger)]` field still
/// exists OR any policy keyword (`period_ms`,
/// `sync_window_ms`, `external`) appears in the macro args,
/// returns the source unchanged. Otherwise inserts
/// `period_ms = 100` so the macro has a valid policy.
///
/// The policy-keyword check scans ONLY the
/// `#[cerulion_node(...)]` argument string, not the whole file —
/// a comment like `// reads from external sensor` must not
/// suppress the fallback insertion.
pub fn ensure_macro_policy_hint(source: &str) -> Result<String, String> {
    if source_has_input_trigger_field(source) {
        return Ok(source.to_string());
    }
    let bare_form = source.contains("#[cerulion_node]") && !source.contains("#[cerulion_node(");
    if bare_form {
        return Ok(source.replacen("#[cerulion_node]", "#[cerulion_node(period_ms = 100)]", 1));
    }
    let parens_open = "#[cerulion_node(";
    let after_open = source
        .find(parens_open)
        .map(|p| p + parens_open.len())
        .ok_or_else(|| {
            "could not find `#[cerulion_node]` or `#[cerulion_node(` in source".to_string()
        })?;
    let mut depth = 1u32;
    let mut paren_close: Option<usize> = None;
    for (i, ch) in source[after_open..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    paren_close = Some(after_open + i);
                    break;
                }
            }
            _ => {}
        }
    }
    let paren_close = paren_close
        .ok_or_else(|| "could not find closing `)` for `#[cerulion_node(...)]`".to_string())?;
    let args_str = &source[after_open..paren_close];
    let known_policy_args = ["period_ms", "sync_window_ms", "external"];
    // Scope the keyword scan to the macro args only — a comment
    // elsewhere in the file containing one of these keywords (e.g.
    // `// reads from external sensor`) must not be mistaken for an
    // existing policy arg.
    if known_policy_args.iter().any(|kw| args_str.contains(kw)) {
        return Ok(source.to_string());
    }
    let existing = args_str.trim();
    let new_args = if existing.is_empty() {
        "period_ms = 100".to_string()
    } else {
        format!("period_ms = 100, {}", existing)
    };
    let mut result = String::with_capacity(source.len());
    result.push_str(&source[..after_open]);
    result.push_str(&new_args);
    result.push_str(&source[paren_close..]);
    Ok(result)
}

/// Escape a string for safe embedding in a JSON string literal.
fn json_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Escape a JSON-formatted string for embedding inside a Rust
/// byte-string literal (`b"..."`). The Rust source compiler will
/// run its own escape pass once on the raw source: `\\\"` in the
/// file becomes `\"` in the in-memory bytes, which is what the
/// JSON parser needs to see. This helper performs ONLY the
/// per-character escapes required by the Rust byte-string syntax;
/// it does NOT touch `\n` / `\t` / control bytes (not produced by
/// `serde_json` for our `PolicyJson` shape).
fn json_escape_for_byte_string(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Convert a schema reference to its Rust type identifier — i.e.
/// the last component after the package separator. Handles both the
/// canonical slash form (`sensor_msgs/Image` → `Image`) and the
/// legacy `::` form (`sensor_msgs::Image` → `Image`), AND the
/// rare mixed form (`sensor_msgs::sub/Name` and
/// `sensor_msgs/sub::Name`) — anywhere a `/` or `::` appears, the
/// function takes the portion after the LAST occurrence of either
/// separator. For an unqualified type (`MyType`), returns it
/// unchanged.
///
/// Centralising the split here keeps every codegen site in sync
/// regardless of which separator the input uses, including
/// pathological mixed-separator inputs that bypass the CLI's
/// `parse_port_args` canonicalisation (e.g. hand-edited graph YAML
/// strings).
pub fn schema_to_ident(schema: &str) -> &str {
    // Find the LAST occurrence of either separator, take what
    // follows. `rfind('/')` gives the byte index just before the
    // slash; `rfind("::")` gives the byte index just before the
    // first colon. Take whichever is later, then advance past the
    // separator length.
    let last_slash = schema.rfind('/').map(|i| i + 1);
    let last_colons = schema.rfind("::").map(|i| i + 2);
    let cut = match (last_slash, last_colons) {
        (Some(a), Some(b)) => a.max(b),
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0,
    };
    &schema[cut..]
}

/// Convert a schema reference like `sensor_msgs::Image` or
/// `sensor_msgs/Image` to a Rust import path under
/// `native_ros2_messages`.
///
/// Maps `sensor_msgs/Image` → `native_ros2_messages::sensor_msgs::Image`
/// (the canonical slash form, per `docs/user-api.md`).
/// Maps `sensor_msgs::Image` → `native_ros2_messages::sensor_msgs::Image`
/// (legacy colon form, accepted for backward compatibility).
/// Unqualified types (`MyType`) pass through unchanged.
///
/// Defends against malformed input by filtering empty path segments
/// (`/Foo`, `Foo/`, `""`, `"//"`). When filtering leaves zero or
/// one segment the input is treated as unqualified and the user's
/// literal string passes through unchanged — the resulting compile
/// error then points back at the typo's source rather than at
/// CLI-emitted garbage like `native_ros2_messages::::Foo`.
pub fn schema_to_import(schema: &str) -> String {
    // Normalise both separator forms to `/` so the segment split
    // and empty-segment filter work uniformly.
    let normalised = schema.replace("::", "/");
    let segments: Vec<&str> = normalised.split('/').filter(|s| !s.is_empty()).collect();
    if segments.len() <= 1 {
        // Unqualified or a single non-empty segment after filtering
        // (e.g. just `MyType`, or `/Foo` → just `Foo`). Pass the
        // raw input through unchanged so callers see what the user
        // typed; they can render it as-is in the import line, and
        // the user gets a localised compile error rather than
        // CLI-emitted garbage.
        return schema.to_string();
    }
    format!("native_ros2_messages::{}", segments.join("::"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_metadata::PortDef;
    use cerulion_core::graph::node::BackpressurePolicy;

    #[test]
    fn test_generate_cargo_toml() {
        let toml = generate_cargo_toml("camera");
        assert!(toml.contains("name = \"camera\""));
        assert!(toml.contains("crate-type = [\"cdylib\"]"));
        assert!(toml.contains("cerulion_core = { workspace = true }"));
        assert!(toml.contains("native_ros2_messages = { workspace = true }"));
        assert!(toml.contains(r#"default = ["cdylib"]"#));
    }

    #[test]
    fn test_generate_lib_rs_does_not_pre_import_prelude() {
        // The raw-FFI placeholder body
        // never references `cerulion_core::prelude::*`, so pre-importing
        // it would trip `unused_imports = deny` when the generated source
        // is compiled under the strict workspace lint contract. Users
        // adding real logic should add the imports they need.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            !src.contains("use cerulion_core::prelude::*;"),
            "raw-FFI template must not pre-import the prelude (unused → -D unused-imports)"
        );
    }

    #[test]
    fn test_generate_lib_rs_has_entry_points() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // 8 required FFI exports (handle-based FFI). Without the two LAST_ERROR
        // symbols a raw-FFI-generated
        // cdylib is unloadable via `DylibNodeEntry::load`. `cerulion_node_pump_history`
        // (ABI v7) lets the host service quiescent late
        // joiners on cdylib nodes across the FFI.
        assert!(src.contains("cerulion_abi_version"));
        assert!(src.contains("cerulion_node_info"));
        assert!(src.contains("cerulion_node_init"));
        assert!(src.contains("cerulion_node_tick"));
        assert!(src.contains("cerulion_node_pump_history"));
        assert!(src.contains("cerulion_node_shutdown"));
        assert!(src.contains("cerulion_take_last_error"));
        assert!(src.contains("cerulion_free_error"));
    }

    #[test]
    fn test_generate_lib_rs_has_marker_comments() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(src.contains("// CERULION:INFO_START"));
        assert!(src.contains("// CERULION:INFO_END"));
    }

    #[test]
    fn test_generate_lib_rs_with_ports() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let src = generate_lib_rs(&metadata);
        // Info JSON uses escaped quotes in byte string literal — port
        // names DO flow into the info JSON regardless of whether their
        // schemas get pre-imported.
        assert!(src.contains("\\\"image\\\""));
        // Schema imports are not
        // emitted (placeholder body never uses them, would trip
        // `unused_imports = deny`). Users replacing the body add the
        // imports they need.
        assert!(
            !src.contains("use native_ros2_messages::sensor_msgs::Image;"),
            "raw-FFI template must not pre-import port schema types (unused → -D unused-imports)"
        );
    }

    #[test]
    fn test_generate_info_fn_uses_static_bytes() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: None,
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let info_fn = generate_info_fn(&metadata);
        assert!(info_fn.contains("INFO_BYTES"));
        assert!(info_fn.contains("static INFO_BYTES: &[u8]"));
        assert!(!info_fn.contains("CString"));
        assert!(!info_fn.contains("into_raw"));
    }

    #[test]
    fn test_generate_lib_rs_handle_based_signatures() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // handle-based FFI signatures
        assert!(src.contains("ctx_ptr: *mut u8"), "init should take *mut u8");
        assert!(src.contains("-> u64"), "init should return u64");
        assert!(
            src.contains("handle: u64"),
            "tick/shutdown should take handle"
        );
        assert!(
            src.contains("NEXT_HANDLE"),
            "should use atomic handle counter"
        );
        assert!(
            src.contains("HashMap"),
            "should use HashMap for multi-instance"
        );
        // CString usage is expected, for the LAST_ERROR
        // thread-local. The "no CString" rule is about
        // INFO_BYTES not leaking via `CString::into_raw` — that contract
        // lives in `test_generate_info_fn_uses_static_bytes`, which
        // pins INFO_BYTES to the static-byte-array shape.
        assert!(
            src.contains("static INFO_BYTES"),
            "info function must use static bytes (no leak)"
        );
    }

    // ── LAST_ERROR machinery structural tests ──
    //
    // These tests are structural pins — they assert the generator emits
    // the specific code patterns the host loader and downstream tooling
    // depend on. End-to-end loadability is proven by the
    // `test_node_raw_ffi_template_cdylib` fixture + `DylibNodeEntry::load`
    // tests in `crates/cerulion_core/tests/node_raw_ffi_template_test.rs`;
    // the structural tests here let us pinpoint *which*
    // generator change broke loadability if both layers fire together.

    #[test]
    fn test_generate_lib_rs_has_last_error_machinery() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains("thread_local!"),
            "LAST_ERROR must be thread-local (per-cdylib, per-thread isolation)"
        );
        assert!(
            src.contains("LAST_ERROR"),
            "thread-local name must be LAST_ERROR"
        );
        assert!(
            src.contains("__cer_set_last_error"),
            "set helper must be present so init/tick/shutdown can stash messages"
        );
        assert!(
            src.contains("CString::new"),
            "set helper must allocate via CString::new (paired with from_raw in free)"
        );
        assert!(
            src.contains("into_raw"),
            "take must transfer ownership via CString::into_raw"
        );
        assert!(
            src.contains("CString::from_raw"),
            "free must reclaim via CString::from_raw (allocator pairing)"
        );
    }

    #[test]
    fn test_generate_lib_rs_thread_local_uses_const_init() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // `const { ::std::cell::RefCell::new(None) }` keeps first-touch
        // zero-cost. The macro form uses the same pattern; do not drift.
        // RefCell is fully-qualified for shadow-proofing.
        assert!(
            src.contains("const { ::std::cell::RefCell::new(None) }"),
            "thread_local should use const init for zero-cost first touch (with fully-qualified RefCell)"
        );
    }

    #[test]
    fn test_generate_lib_rs_populates_last_error_on_mutex_poison() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // This test asserts the
        // `__cer_set_last_error(::std::string::String::from(...))` call
        // envelope is wired, not just the message string. A maintainer
        // commenting out the call but leaving the literal as a
        // doc-comment would pass a substring-only
        // assertion. The envelope spans two lines after `cargo fmt`'s
        // 100-char wrap; assert the open-paren prefix + the literal
        // separately so a future formatting shift (e.g. wider column
        // limit collapsing back to single-line) doesn't false-trip.
        for path in [
            "cerulion_node_init",
            "cerulion_node_tick",
            "cerulion_node_pump_history",
            "cerulion_node_shutdown",
        ] {
            let msg = format!("{}: NODES mutex poisoned", path);
            assert!(
                src.contains(&msg),
                "{} mutex-poison must emit the LAST_ERROR message",
                path,
            );
            // Locate the message and walk backwards over the prior
            // lines to confirm the call envelope wraps it. The envelope
            // is the literal `__cer_set_last_error(::std::string::String::from(`
            // somewhere in the previous few lines.
            let msg_pos = src.find(&msg).expect("message must be present");
            let window_start = msg_pos.saturating_sub(200);
            let window = &src[window_start..msg_pos];
            assert!(
                window.contains("__cer_set_last_error(::std::string::String::from("),
                "{} mutex-poison message must be wrapped by the \
                 __cer_set_last_error call envelope; got window:\n{}",
                path,
                window,
            );
        }
    }

    #[test]
    fn test_generate_lib_rs_populates_last_error_on_missing_handle() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // Assert the `__cer_set_last_error(format!(...))`
        // CALL envelope, not just the format-string template. Shutdown
        // wording matches the macro form so operators grepping logs see
        // the same string from both cdylib types.
        assert!(
            src.contains(
                r#"__cer_set_last_error(format!("cerulion_node_tick: handle {} not found", handle))"#
            ),
            "tick missing-handle must call __cer_set_last_error with the format envelope"
        );
        // Shutdown's longer message is multi-line in the emitted source;
        // assert both halves of the format envelope in the right order.
        assert!(
            src.contains("__cer_set_last_error(format!("),
            "shutdown missing-handle must call __cer_set_last_error(format!(...))"
        );
        assert!(
            src.contains("cerulion_node_shutdown: handle {} not found (already shut down or never registered)"),
            "shutdown missing-handle template must match the macro form wording"
        );
        // `pump_history` mirrors tick's missing-handle
        // path. Its format string wraps across lines after `cargo fmt`,
        // so assert the message literal + the format envelope separately
        // (same shape as the shutdown assertion above).
        assert!(
            src.contains("cerulion_node_pump_history: handle {} not found"),
            "pump_history missing-handle template must match the tick form wording"
        );
    }

    #[test]
    fn test_generate_lib_rs_init_drops_node_context_via_box_from_raw() {
        // Init must reclaim the host's
        // `Box::into_raw(NodeContext)` via `Box::from_raw` to avoid
        // leaking one NodeContext per init call. A future maintainer
        // reverting to `let _ = ctx_ptr;` (the leak pattern)
        // would silently regress the user-API contract documented at
        // `crates/cerulion_core/src/graph/node.rs`, where `DylibNodeEntry` hands the
        // context over with `Box::into_raw`.
        //
        // The structural pin asserts the exact `Box::from_raw(ctx_ptr
        // as *mut NodeContext)` shape; the runtime regression test in
        // `crates/cerulion_core/tests/node_raw_ffi_template_test.rs` (specifically
        // `test_raw_ffi_template_cdylib_init_drops_context_no_leak`)
        // exercises 16 init+shutdown cycles to catch the leak end-to-end.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // The leak pattern is `let _ = ctx_ptr;` — explicitly NOT
        // present in the emit. (Comments may mention it as an
        // anti-example; the code-line filter excludes them.)
        let code_lines: Vec<&str> = src
            .lines()
            .filter(|line| !line.trim_start().starts_with("//"))
            .collect();
        let leak_pattern_count: usize = code_lines
            .iter()
            .map(|line| line.matches("let _ = ctx_ptr").count())
            .sum();
        assert_eq!(
            leak_pattern_count, 0,
            "leak pattern `let _ = ctx_ptr;` must NOT appear in code; \
             init must reclaim the context via Box::from_raw"
        );
        // And the fix pattern must be present in the emit.
        assert!(
            src.contains(
                "Box::from_raw(ctx_ptr as *mut ::cerulion_core::graph::node::NodeContext)"
            ),
            "init must reclaim the host's Box::into_raw'd NodeContext via Box::from_raw"
        );
    }

    #[test]
    fn test_generate_lib_rs_init_null_check_present() {
        // Init must null-check
        // `ctx_ptr` and populate LAST_ERROR with the macro-parity
        // wording. A silent fresh-handle return on a null ctx hides
        // a loader contract violation behind a "success" return value.
        //
        // Assertions are whitespace-tolerant.
        // Pinning a 12-space indent prefix would be fragile: a
        // `cargo fmt` could rewrite the multi-line `String::from(...)`
        // onto a single line and silently break the test even though
        // the contract was preserved.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains("if ctx_ptr.is_null()"),
            "init must null-check ctx_ptr before allocating a handle"
        );
        // The call envelope `__cer_set_last_error(::std::string::String::from(...))`
        // is already pinned by `test_generate_lib_rs_set_last_error_call_sites`
        // (count) + `test_generate_lib_rs_populates_last_error_on_mutex_poison`
        // (envelope shape). Here we only need the macro-parity wording.
        assert!(
            src.contains(r#""cerulion_node_init: NodeContext pointer was null""#),
            "init's null-ctx branch must use the macro-parity LAST_ERROR wording"
        );
    }

    #[test]
    fn test_generate_lib_rs_set_last_error_call_sites() {
        // Exactly 8 error-path call sites, wired into every reachable
        // non-success branch in init/tick/pump_history/shutdown:
        //   1. cerulion_node_init:         null ctx_ptr
        //   2. cerulion_node_init:         NODES mutex poisoned
        //   3. cerulion_node_tick:         missing handle
        //   4. cerulion_node_tick:         NODES mutex poisoned
        //   5. cerulion_node_pump_history: missing handle
        //   6. cerulion_node_pump_history: NODES mutex poisoned
        //   7. cerulion_node_shutdown:     NODES mutex poisoned
        //   8. cerulion_node_shutdown:     missing handle
        //
        // Plus 1 definition (`fn __cer_set_last_error(...)`) — 9 total
        // CODE occurrences in the generated source.
        //
        // Count only code occurrences, not mentions
        // in `//` comments. The NOTE-TO-USERS block in the emitted source
        // references `__cer_set_last_error(format!(...))` as an example
        // of what users should hand-roll; that mention is documentary,
        // not a call site, and must not inflate the count.
        //
        // If you ADD a new error path: wire LAST_ERROR + bump this count.
        // If you REMOVE one: bump this count down AND ensure the removed
        // path is no longer reachable.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        let code_lines: Vec<&str> = src
            .lines()
            .filter(|line| !line.trim_start().starts_with("//"))
            .collect();
        let count: usize = code_lines
            .iter()
            .map(|line| line.matches("__cer_set_last_error").count())
            .sum();
        assert_eq!(
            count, 9,
            "expected 9 code occurrences of __cer_set_last_error (1 definition + 8 error-path call sites); \
             got {} — a new non-success return probably needs LAST_ERROR wiring, or one was removed unsafely",
            count
        );
    }

    #[test]
    fn test_generate_lib_rs_does_not_import_shadow_prone_std_types() {
        // `String`, `CString`, `RefCell` are
        // referenced via fully-qualified `::std::...` paths in the
        // LAST_ERROR machinery so a `use native_ros2_messages::...;`
        // schema import (specifically `std_msgs/String`) cannot shadow
        // them and break compilation. The previous form imported
        // `RefCell` + `CString` and used bare names; that worked for
        // empty-port nodes but broke for any node with a String port.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            !src.contains("use std::cell::RefCell;"),
            "RefCell must be fully-qualified `::std::cell::RefCell` in the emit, not imported"
        );
        assert!(
            !src.contains("use std::ffi::CString;"),
            "CString must be fully-qualified `::std::ffi::CString` in the emit, not imported"
        );
        // The fully-qualified form must actually be present in the
        // LAST_ERROR thread-local declaration.
        assert!(
            src.contains("::std::cell::RefCell"),
            "thread_local must use fully-qualified ::std::cell::RefCell"
        );
        assert!(
            src.contains("::std::ffi::CString"),
            "LAST_ERROR machinery must use fully-qualified ::std::ffi::CString"
        );
    }

    #[test]
    fn test_generate_lib_rs_string_from_is_fully_qualified() {
        // Every `String::from(...)` call in the
        // emit must be fully-qualified to `::std::string::String::from`.
        // Bare `String::from(...)` would be shadowed by a
        // `use native_ros2_messages::std_msgs::String;` import emitted
        // alongside for any raw-FFI node with a `std_msgs/String` port,
        // resolving to `std_msgs::String::from` (which doesn't exist) at
        // compile time. The macro form qualifies for the same reason.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        // Count fully-qualified vs bare `String::from` occurrences in
        // CODE lines (excluding comments which may reference the bare
        // form as an anti-example in the NOTE-TO-USERS block).
        let code_lines: Vec<&str> = src
            .lines()
            .filter(|line| !line.trim_start().starts_with("//"))
            .collect();
        // Match the full call site `String::from(` (with open paren)
        // to exclude `CString::from_raw(` (whose substring contains
        // `String::from_raw` but not `String::from(`).
        let qualified_count: usize = code_lines
            .iter()
            .map(|line| line.matches("::std::string::String::from(").count())
            .sum();
        let bare_count: usize = code_lines
            .iter()
            .map(|line| {
                // `::std::string::String::from(` also contains `String::from(`,
                // so count bare occurrences by subtracting the qualified
                // matches from the total.
                line.matches("String::from(").count()
                    - line.matches("::std::string::String::from(").count()
            })
            .sum();
        // `pump_history` adds one more mutex-poison
        // `::std::string::String::from(...)` call site (init null-ctx +
        // init/tick/pump_history/shutdown mutex-poison = 5).
        assert_eq!(
            qualified_count, 5,
            "expected 5 fully-qualified ::std::string::String::from call sites; got {}",
            qualified_count
        );
        assert_eq!(
            bare_count, 0,
            "no bare `String::from(...)` allowed in emit (shadow risk vs std_msgs/String); got {} bare occurrences",
            bare_count
        );
        // And the helper signature must take the fully-qualified String.
        assert!(
            src.contains("fn __cer_set_last_error(msg: ::std::string::String)"),
            "set helper must take fully-qualified ::std::string::String parameter"
        );
    }

    #[test]
    fn test_generate_lib_rs_compiles_with_std_msgs_string_port() {
        // Defense in depth:
        // emit a node with a `std_msgs/String` output port. The
        // schema import is not emitted (the shadow scenario is
        // structurally eliminated), but the fully-qualified
        // `::std::string::String::from(...)` paths remain — so if a
        // future maintainer re-introduces schema-import emission, the
        // shadow case is automatically immune. This test pins BOTH:
        // (a) schema import is absent, AND (b) the qualification is
        // still in place.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "stringnode".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![PortDef {
                name: "msg".to_string(),
                schema: Some("std_msgs/String".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let src = generate_lib_rs(&metadata);
        // The schema import is not emitted.
        assert!(
            !src.contains("use native_ros2_messages::std_msgs::String"),
            "schema import for std_msgs/String must NOT be emitted (unused → -D unused-imports)"
        );
        // Defense in depth: every String::from in the emit
        // must still be fully-qualified, even though the shadow scenario
        // does not apply. If schema imports are ever
        // introduced, the shadow case is automatically immune.
        let code_lines: Vec<&str> = src
            .lines()
            .filter(|line| !line.trim_start().starts_with("//"))
            .collect();
        // Match `String::from(` with open paren to exclude
        // `CString::from_raw(` which shares the substring `String::from`.
        let bare_count: usize = code_lines
            .iter()
            .map(|line| {
                line.matches("String::from(").count()
                    - line.matches("::std::string::String::from(").count()
            })
            .sum();
        assert_eq!(
            bare_count, 0,
            "shadow-prone bare `String::from(...)` in emit with std_msgs/String port — \
             would fail to compile; got {} bare occurrences",
            bare_count
        );
    }

    #[test]
    fn test_generate_lib_rs_free_error_is_unsafe() {
        // `cerulion_free_error`
        // dereferences a raw pointer arg via `CString::from_raw`. Per
        // `clippy::not_unsafe_ptr_arg_deref` (deny-by-default), the
        // function MUST be marked `unsafe` to compile under the
        // workspace lint contract. The hand-written
        // `crates/test_fixtures/test_node_cdylib/src/lib.rs:57` is the
        // precedent. Drift back to safe-fn form would silently break
        // the template fixture's compile and any user `cerulion node build`.
        //
        // The redundancy check uses token
        // count rather than a whitespace-pinned substring. The earlier
        // form pinned an 8-space indent, which a `cargo fmt` could
        // silently invalidate.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains(r#"pub unsafe extern "C" fn cerulion_free_error"#),
            "cerulion_free_error must be `pub unsafe extern \"C\"` (raw-ptr deref + clippy lint)"
        );
        // The fn-body's inner `unsafe { ... }` block is redundant once
        // the function itself is unsafe (clippy::unused_unsafe). Walk
        // the function body and count `unsafe` tokens: there must be
        // exactly one (the fn-level keyword) — any inner block would
        // bump the count to 2 and trip this assertion regardless of
        // indentation.
        let body_start = src
            .find(r#"pub unsafe extern "C" fn cerulion_free_error"#)
            .expect("cerulion_free_error must be present");
        // Body ends at the next `\n}\n` (the function's closing brace
        // on its own line). Falling back to end-of-string is sound for
        // the trailing function in the emit; the assertion still fires
        // correctly on the wider slice.
        let body_end = src[body_start..]
            .find("\n}\n")
            .map(|i| body_start + i)
            .unwrap_or(src.len());
        let body = &src[body_start..body_end];
        let unsafe_count = body.matches("unsafe").count();
        assert_eq!(
            unsafe_count, 1,
            "cerulion_free_error must have exactly 1 `unsafe` keyword (fn-level only); got {} — \
             a redundant inner `unsafe block` would trip clippy::unused_unsafe and fail the fixture compile",
            unsafe_count
        );
        // And `cerulion_take_last_error` only RETURNS a raw pointer
        // (no arg-deref), so it stays plain `extern "C"`.
        assert!(
            src.contains(r#"pub extern "C" fn cerulion_take_last_error"#),
            "cerulion_take_last_error must remain plain extern \"C\" (no raw-ptr arg deref)"
        );
    }

    #[test]
    fn test_generate_lib_rs_does_not_recover_poisoned_mutex() {
        // Pin that no `.into_inner()` call
        // ever appears in the generated source. A future "resilience"
        // patch that recovers state via `into_inner()` after a
        // panic-while-locked would visibly corrupt per-node state and
        // surface as data corruption in user nodes rather than as a
        // loud FFI failure. The non-recovery contract is documented
        // in three places (one in each of init/tick/shutdown's
        // mutex-poison branches) — this test ensures the contract
        // can't drift silently.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            !src.contains(".into_inner()"),
            "do NOT add into_inner() recovery on the mutex-poison branches — \
             a panic while holding NODES can leave per-node state inconsistent; \
             surface the poison via LAST_ERROR and let the host decide to restart"
        );
        // Belt-and-braces: the educational comment must remain above
        // each of the three branches. (Once in init, twice as "see init"
        // cross-references in tick / shutdown.)
        assert!(
            src.contains("do NOT recover via `into_inner()`"),
            "the non-recovery rationale must remain in the emitted source so future readers see it"
        );
    }

    #[test]
    fn test_generate_lib_rs_emits_placeholder_body_warning() {
        // The warning block lives in the
        // emitted source, so users opening their generated `lib.rs`
        // see it. The first form put the warning in the generator
        // file only (a Rust source comment in `templates.rs`), where
        // users never look. Pinning the in-emit form here ensures the
        // warning is delivered to the actual audience.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains("NOTE TO USERS REPLACING THE PLACEHOLDER BODY"),
            "users must see the placeholder-body warning in their generated lib.rs"
        );
        // Each load-bearing claim is independently pinned so a partial
        // edit of the warning (e.g. removing the catch_unwind mention)
        // is loud at test time.
        //
        // `catch_unwind` is named twice in the
        // warning — once in the "DOES NOT EMIT" bullet list, once in
        // the "Hand-roll the equivalent wiring" reference. Requiring
        // a count >= 2 catches a maintainer collapsing the bullet
        // list (which would otherwise leave the second mention and
        // still pass a `contains("catch_unwind")` check).
        let catch_unwind_count = src.matches("catch_unwind").count();
        assert!(
            catch_unwind_count >= 2,
            "warning must name catch_unwind in BOTH the 'DOES NOT EMIT' list AND \
             the hand-roll-wiring reference; got {} occurrence(s)",
            catch_unwind_count
        );
        assert!(
            src.contains("init failed:") && src.contains("tick failed:"),
            "warning must name the Err-propagation patterns users would otherwise miss"
        );
        assert!(
            src.contains("Migrate to `#[cerulion_node]`"),
            "warning must offer the recommended migration path"
        );
        assert!(
            src.contains("Unwinding across the FFI boundary aborts the process"),
            "warning must explicitly call out the UB consequence of an un-trapped panic"
        );
    }

    #[test]
    fn test_generate_lib_rs_free_error_handles_null() {
        // The null guard is a host-contract invariant. A drift back to
        // unconditional `CString::from_raw(ptr)` would crash on the
        // documented "no error to free" case.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains("if ptr.is_null()"),
            "cerulion_free_error must guard against null (idempotent per host contract)"
        );
    }

    #[test]
    fn test_generate_lib_rs_take_last_error_returns_null_when_clean() {
        // The "no error buffered" path returns null. Structurally pin
        // the match-arm so a drift that always returns a CString-style
        // pointer is loud.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains("None => std::ptr::null_mut()"),
            "cerulion_take_last_error must return null when LAST_ERROR is empty"
        );
    }

    #[test]
    fn test_generate_lib_rs_set_last_error_has_nul_free_fallback() {
        // The `__cer_set_last_error` fallback `unwrap()` is sound only
        // because the fallback string contains no embedded nul. Pin
        // the exact fallback string so a future maintainer can't
        // silently introduce a nul that would panic the cdylib at the
        // worst possible moment (while it's trying to surface an
        // error to the host).
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_lib_rs(&metadata);
        assert!(
            src.contains(r#"CString::new("error message contained nul byte").unwrap()"#),
            "fallback message must be the exact nul-free literal so unwrap is provably infallible"
        );
        // Belt-and-braces: assert the literal itself contains no nul.
        // If a future maintainer edits the string, this assertion
        // catches it independently of the surrounding code.
        const FALLBACK: &str = "error message contained nul byte";
        assert!(
            !FALLBACK.contains('\0'),
            "fallback literal must not contain a nul byte (invariant under the unwrap())"
        );
    }

    #[test]
    fn test_generate_lib_rs_matches_oracle_fixture() {
        // Drift-detection oracle vector. The fixture at
        // `crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`
        // is the byte-for-byte output of `generate_lib_rs(canonical)`,
        // committed as a frozen artifact. Any future change to the
        // generator that changes the emit (intentional or otherwise)
        // will fail this assertion. The remediation is documented in
        // the failure message: regenerate the fixture from the new
        // output via the `dump_raw_ffi_emit` example.
        //
        // This is the load-bearing test for the acceptance
        // criterion `cerulion node create X --raw-ffi && cerulion node
        // build X && cerulion node run X`: the fixture is a workspace
        // member, so `cargo build` builds it as a real cdylib, and the
        // matching `crates/cerulion_core/tests/node_raw_ffi_template_test.rs`
        // tests prove `DylibNodeEntry::load` succeeds on the artifact.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "raw_ffi_template".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![],
        };
        let generated = generate_lib_rs(&metadata);
        let fixture =
            include_str!("../../test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs");
        if generated != fixture {
            // Compute the first diverging line so the failure message
            // points the maintainer at the change.
            let mut line_diff = None;
            for (i, (g, f)) in generated.lines().zip(fixture.lines()).enumerate() {
                if g != f {
                    line_diff = Some((i + 1, g.to_string(), f.to_string()));
                    break;
                }
            }
            let len_diff = generated.lines().count() as isize - fixture.lines().count() as isize;
            panic!(
                "generator output drifted from oracle fixture.\n\
                 first diverging line: {:?}\n\
                 generated line count - fixture line count = {}\n\
                 \n\
                 Remediation: if the generator change is intentional,\n\
                 regenerate the fixture and commit BOTH the templates.rs\n\
                 change AND the fixture diff:\n\
                 \n\
                     cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit \\\n\
                         > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs\n\
                 \n\
                 If the change is unintentional, revert it.",
                line_diff, len_diff,
            );
        }
    }

    #[test]
    fn test_schema_to_import() {
        // Legacy `::` form (still accepted for backward compat).
        assert_eq!(
            schema_to_import("sensor_msgs::Image"),
            "native_ros2_messages::sensor_msgs::Image"
        );
        assert_eq!(schema_to_import("MyType"), "MyType");
    }

    // ─── slash-form schema canonicalization ───────────────────────
    //
    // `schema_to_ident` must take everything after the LAST
    // separator (`/` or `::`) regardless of which form is used. A
    // pure `rsplit("::")` would return the WHOLE slash-form string,
    // so the generated `lib.rs` would carry invalid Rust like
    // line `image: sensor_msgs/Image,` — a Quick-Start user
    // following `docs/user-api.md` verbatim hit a compile error inside
    // CLI-generated code on first contact.
    //
    // The fix has two parts:
    // 1. `parse_port_args` (in `crates/cerulion_cli/src/main.rs`)
    //    canonicalizes via `schema_cmd::normalize_schema` at the
    //    input boundary, so every downstream consumer sees the
    //    slash form regardless of which separator the user typed.
    // 2. The codegen helpers `schema_to_ident` and
    //    `schema_to_import` understand both forms — the centralised
    //    helpers replace the open-coded `rsplit("::")` calls at
    //    every codegen site (3 in `generate_macro_lib_rs`).

    #[test]
    fn test_schema_to_ident_slash_form() {
        assert_eq!(schema_to_ident("sensor_msgs/Image"), "Image");
        assert_eq!(
            schema_to_ident("geometry_msgs/Twist"),
            "Twist",
            "slash-form must extract the type ident"
        );
    }

    #[test]
    fn test_schema_to_ident_colon_form() {
        // Colon form must still work — `parse_port_args`
        // canonicalises to slash form at the input boundary, but
        // other code paths (e.g. graph YAML deserialisation, hand-
        // edited graph YAML strings) may still feed colon form here.
        assert_eq!(schema_to_ident("sensor_msgs::Image"), "Image");
        assert_eq!(schema_to_ident("std_msgs::Header"), "Header");
    }

    #[test]
    fn test_schema_to_ident_unqualified() {
        assert_eq!(schema_to_ident("MyType"), "MyType");
        assert_eq!(schema_to_ident("u8"), "u8");
        assert_eq!(schema_to_ident(""), "");
    }

    #[test]
    fn test_schema_to_ident_mixed_separators() {
        // Pathological input bypassing CLI canonicalisation
        // (hand-edited graph YAML, future Rust API consumer). The
        // helper must take everything after the LAST separator
        // regardless of which form it is.
        assert_eq!(
            schema_to_ident("pkg/sub::Name"),
            "Name",
            "slash-then-colon mixed form must extract `Name`, not `sub::Name`"
        );
        assert_eq!(
            schema_to_ident("pkg::sub/Name"),
            "Name",
            "colon-then-slash mixed form must extract `Name`"
        );
        assert_eq!(
            schema_to_ident("a/b/c/Name"),
            "Name",
            "multi-slash form takes the last component"
        );
        assert_eq!(
            schema_to_ident("a::b::c::Name"),
            "Name",
            "multi-colon form takes the last component"
        );
    }

    #[test]
    fn test_schema_to_import_malformed_input() {
        // A leading- or trailing-slash typo today routes empty
        // segments through `replace('/', "::")`, producing
        // invalid Rust like `native_ros2_messages::::Foo` or
        // `native_ros2_messages::Foo::`. `schema_to_import`
        // filters empty segments and treats single-segment input
        // as unqualified (returns the user's literal string,
        // letting them get a localised compile error rather than
        // CLI-emitted garbage).
        assert_eq!(
            schema_to_import("/Foo"),
            "/Foo",
            "leading-slash typo passes through unchanged (not `::::Foo`)"
        );
        assert_eq!(
            schema_to_import("Foo/"),
            "Foo/",
            "trailing-slash typo passes through unchanged (not `Foo::`)"
        );
        assert_eq!(
            schema_to_import(""),
            "",
            "empty input passes through unchanged"
        );
        assert_eq!(
            schema_to_import("//"),
            "//",
            "all-empty-segments input passes through unchanged"
        );
    }

    #[test]
    fn test_schema_to_import_slash_form() {
        assert_eq!(
            schema_to_import("sensor_msgs/Image"),
            "native_ros2_messages::sensor_msgs::Image",
            "slash-form must map to the same Rust path as the legacy colon form"
        );
        assert_eq!(
            schema_to_import("geometry_msgs/Twist"),
            "native_ros2_messages::geometry_msgs::Twist"
        );
    }

    #[test]
    fn test_schema_to_import_unqualified() {
        // No separator in either form → unchanged. Used for
        // tier-3 in-workspace schemas defined in `schemas/`.
        assert_eq!(schema_to_import("MyType"), "MyType");
    }

    #[test]
    fn test_macro_lib_rs_slash_form_emits_valid_rust() {
        // Codegen oracle: metadata configured with the slash form
        // (the canonical `docs/user-api.md` form) must produce a `lib.rs`
        // whose struct-field type is the bare `Image` ident and
        // whose `use` statement is
        // `native_ros2_messages::sensor_msgs::Image`.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs/Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(
            src.contains("image: Image,"),
            "expected `image: Image,` field declaration, got:\n{}",
            src
        );
        assert!(
            src.contains("use native_ros2_messages::sensor_msgs::Image;"),
            "expected `use native_ros2_messages::sensor_msgs::Image;`, got:\n{}",
            src
        );
        assert!(
            !src.contains("sensor_msgs/Image"),
            "generated source must NOT contain the unqualified slash-form schema string"
        );
    }

    #[test]
    fn test_macro_lib_rs_colon_form_still_works() {
        // Backward-compat regression test: colon-form schemas must
        // still produce valid Rust. Mirrors
        // `test_macro_lib_rs_with_outputs` but pins the colon-form
        // path explicitly.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
    }

    #[test]
    fn test_macro_lib_rs_input_with_slash_form() {
        // Input ports go through the same codegen path; pin them
        // independently.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "detector".to_string(),
            policy: None,
            inputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs/Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
            outputs: vec![],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
    }

    // ── Macro template tests ───────────────────────────────────

    #[test]
    fn test_macro_lib_rs_no_ports() {
        // Source-only node (no inputs, no outputs) — the explicit
        // `Some(Period { 100 })` policy is required because the
        // template generator has no 100ms
        // fallback default. The strict gate lives in
        // `node_create_with_options`; the template is a pure
        // codegen function that trusts its inputs.
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(src.contains("use cerulion_core::prelude::*;"));
        assert!(src.contains("#[cerulion_node(period_ms = 100)]"));
        assert!(src.contains("#[cerulion_node_impl]"));
        assert!(src.contains("#[derive(Default)]"));
        assert!(src.contains("struct CameraNode"));
        assert!(src.contains("fn tick(&mut self) -> Result<(), NodeError>"));
        // Should NOT contain FFI entry points or legacy macro args.
        assert!(!src.contains("extern \"C\""));
        assert!(!src.contains("cerulion_node_info"));
        assert!(!src.contains("type_name"));
        assert!(!src.contains("inputs("));
        assert!(!src.contains("outputs("));
    }

    #[test]
    fn test_macro_lib_rs_external_scaffolds_external_source() {
        // The External arm must scaffold the
        // required `external_source` method — an `#[cerulion_node(external)]`
        // impl block without it is a hard compile error. Pin BOTH halves in one
        // body: the External scaffold HAS the method (HostDriven), and a
        // non-external (Period) scaffold does NOT (so a regression that always
        // emits it, or never emits it, fails here).
        let external = NodeMetadata {
            throttle_ms: None,
            node_type: "camera_driver".to_string(),
            policy: Some(cerulion_core::MacroPolicy::External),
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs/Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let ext_src = generate_macro_lib_rs(&external, None);
        assert!(
            ext_src.contains("#[cerulion_node(external)]"),
            "external node must carry the `external` macro arg, got:\n{ext_src}"
        );
        assert!(
            ext_src.contains("fn external_source(&mut self) -> ExternalSource"),
            "external scaffold must define the required `external_source` method, got:\n{ext_src}"
        );
        assert!(
            ext_src.contains("ExternalSource::HostDriven"),
            "external scaffold must return the HostDriven default source, got:\n{ext_src}"
        );

        // Non-external control: a Period node must NOT emit `external_source`.
        let periodic = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![],
        };
        let per_src = generate_macro_lib_rs(&periodic, None);
        assert!(
            !per_src.contains("external_source"),
            "a non-external scaffold must NOT emit an `external_source` method, got:\n{per_src}"
        );
    }

    #[test]
    fn test_macro_lib_rs_with_outputs() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        // Output declared as a struct field with `#[output]`.
        assert!(src.contains("#[output]"));
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
        // No legacy ctx.publisher_mut() in the new template.
        assert!(!src.contains("ctx.publisher_mut"));
        assert!(!src.contains("type_name"));
    }

    #[test]
    fn test_macro_lib_rs_with_inputs() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "detector".to_string(),
            policy: None,
            inputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
            outputs: vec![],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(src.contains("#[input]"));
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
        assert!(!src.contains("type_name"));
    }

    #[test]
    fn test_macro_lib_rs_with_trigger() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "fusion".to_string(),
            policy: None,
            inputs: vec![
                PortDef {
                    name: "lidar".to_string(),
                    schema: Some("sensor_msgs::LaserScan".to_string()),
                    schema_alternatives: Vec::new(),
                    trigger: false,
                    backpressure: BackpressurePolicy::DropOldest,
                },
                PortDef {
                    name: "camera".to_string(),
                    schema: Some("sensor_msgs::Image".to_string()),
                    schema_alternatives: Vec::new(),
                    trigger: false,
                    backpressure: BackpressurePolicy::DropOldest,
                },
            ],
            outputs: vec![],
        };
        let src = generate_macro_lib_rs(&metadata, Some("lidar"));
        // The lidar input is the trigger; the macro emits #[input(trigger)].
        assert!(src.contains("#[input(trigger)]"));
        assert!(src.contains("lidar: LaserScan,"));
        // Camera (non-trigger) gets bare #[input].
        assert!(src.contains("#[input]"));
        assert!(src.contains("camera: Image,"));
    }

    #[test]
    fn test_macro_lib_rs_multiple_outputs() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "driver".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![
                PortDef {
                    name: "velocity".to_string(),
                    schema: Some("geometry_msgs::Twist".to_string()),
                    schema_alternatives: Vec::new(),
                    trigger: false,
                    backpressure: BackpressurePolicy::DropOldest,
                },
                PortDef {
                    name: "status".to_string(),
                    schema: Some("std_msgs::String".to_string()),
                    schema_alternatives: Vec::new(),
                    trigger: false,
                    backpressure: BackpressurePolicy::DropOldest,
                },
            ],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        // Both outputs as struct fields with #[output].
        assert!(src.contains("velocity: Twist,"));
        assert!(src.contains("status: String,"));
        assert!(src.contains("use native_ros2_messages::geometry_msgs::Twist;"));
        assert!(src.contains("use native_ros2_messages::std_msgs::String;"));
    }

    #[test]
    fn test_macro_lib_rs_pascal_case_struct_name() {
        let metadata = NodeMetadata {
            throttle_ms: None,
            node_type: "imu_fusion".to_string(),
            policy: Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            inputs: vec![],
            outputs: vec![],
        };
        let src = generate_macro_lib_rs(&metadata, None);
        assert!(src.contains("struct ImuFusionNode"));
        assert!(src.contains("impl ImuFusionNode"));
    }

    // ─── inject_macro_field + toggle_macro_external_flag tests ──

    #[test]
    fn test_inject_macro_field_appends_input() {
        let source = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct FusionNode {
    #[input(trigger)]
    lidar: Image,
    tick_count: u32,
}
"#;
        let result = inject_macro_field(source, "camera", "Image", false, false).unwrap();
        assert!(
            result.contains("#[input]\n    camera: Image,"),
            "expected the new `#[input]` field to be injected, got:\n{}",
            result
        );
        // Existing fields preserved unchanged.
        assert!(result.contains("#[input(trigger)]\n    lidar: Image,"));
        assert!(result.contains("tick_count: u32"));
        // Inserted before the closing brace, not after.
        let close_brace_pos = result.find("\n}").expect("no closing brace");
        let camera_pos = result.find("camera: Image").expect("no camera field");
        assert!(camera_pos < close_brace_pos);
    }

    #[test]
    fn test_inject_macro_field_appends_output() {
        let source = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CameraNode {
    tick_count: u32,
}
"#;
        let result = inject_macro_field(source, "image", "Image", true, false).unwrap();
        assert!(result.contains("#[output]\n    image: Image,"));
        assert!(result.contains("tick_count: u32"));
    }

    #[test]
    fn test_inject_macro_field_appends_trigger_input() {
        let source = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node]
#[derive(Default)]
struct DetectorNode {
    tick_count: u32,
}
"#;
        let result = inject_macro_field(source, "raw", "Image", false, true).unwrap();
        assert!(result.contains("#[input(trigger)]\n    raw: Image,"));
    }

    #[test]
    fn test_inject_macro_field_errors_without_macro_struct() {
        let source = "fn main() {}";
        let err = inject_macro_field(source, "x", "u32", false, false).unwrap_err();
        assert!(
            err.contains("no `#[cerulion_node]`"),
            "expected no-macro-struct error, got: {}",
            err
        );
    }

    #[test]
    fn test_inject_macro_field_errors_on_unparseable_source() {
        let source = "fn foo( {";
        let err = inject_macro_field(source, "x", "u32", false, false).unwrap_err();
        assert!(err.contains("could not parse"));
    }

    #[test]
    fn test_toggle_macro_external_flag_adds_to_bare_form() {
        let source = "#[cerulion_node]\nstruct Foo {}\n";
        let result = toggle_macro_external_flag(source, true).unwrap();
        assert!(result.contains("#[cerulion_node(external)]"));
    }

    #[test]
    fn test_toggle_macro_external_flag_replaces_period_ms() {
        // External is mutually exclusive with
        // period_ms (the macro rejects the combo). Toggling external
        // ON when period_ms is present must STRIP period_ms so the
        // resulting source builds AND `parse_node_metadata` reports
        // policy = External. Appending `external` after
        // `period_ms = 100` instead makes the macro hit the validator error, and
        // the parser silently picks Period (precedence rule).
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        let result = toggle_macro_external_flag(source, true).unwrap();
        assert!(
            result.contains("#[cerulion_node(external)]"),
            "expected the period_ms key to be stripped; got: {result}"
        );
        assert!(!result.contains("period_ms"));
    }

    #[test]
    fn test_toggle_macro_external_flag_removes_when_present() {
        // Inverse of the strip-on-add path: turning external OFF
        // removes only the `external` token; sibling args (none here,
        // but the legacy fixture had `period_ms = 100`) stay
        // untouched. With no remaining args, the function falls back
        // to the `period_ms = 100` default per the no-trigger-field
        // branch.
        let source = "#[cerulion_node(external)]\nstruct Foo {}\n";
        let result = toggle_macro_external_flag(source, false).unwrap();
        assert!(!result.contains("external"));
        // Falls back to default period_ms hint so the source stays
        // buildable (no `#[input(trigger)]` field in the fixture).
        assert!(result.contains("period_ms = 100"));
    }

    #[test]
    fn test_toggle_macro_external_flag_idempotent_on_bare_disable() {
        // Bare `#[cerulion_node]` with `enable=false` is already off.
        let source = "#[cerulion_node]\nstruct Foo {}\n";
        let result = toggle_macro_external_flag(source, false).unwrap();
        assert_eq!(result, source);
    }

    #[test]
    fn test_toggle_macro_external_flag_no_op_when_already_enabled() {
        // Enabling on an already-enabled source must not double-add.
        let source = "#[cerulion_node(external)]\nstruct Foo {}\n";
        let result = toggle_macro_external_flag(source, true).unwrap();
        let count = result.matches("external").count();
        assert_eq!(count, 1, "external must appear exactly once");
    }

    // ─── set_macro_policy_arg: replace policy in macro args ──────

    #[test]
    fn test_set_macro_policy_arg_replaces_period_with_sync() {
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        let result = set_macro_policy_arg(source, "sync_window_ms = 50").unwrap();
        assert!(result.contains("#[cerulion_node(sync_window_ms = 50)]"));
        assert!(!result.contains("period_ms"));
    }

    #[test]
    fn test_set_macro_policy_arg_replaces_period_with_external() {
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        let result = set_macro_policy_arg(source, "external").unwrap();
        assert!(result.contains("#[cerulion_node(external)]"));
        assert!(!result.contains("period_ms"));
    }

    #[test]
    fn test_set_macro_policy_arg_replaces_external_with_period() {
        let source = "#[cerulion_node(external)]\nstruct Foo {}\n";
        let result = set_macro_policy_arg(source, "period_ms = 33").unwrap();
        assert!(result.contains("#[cerulion_node(period_ms = 33)]"));
        assert!(!result.contains("external"));
    }

    #[test]
    fn test_set_macro_policy_arg_converts_bare_form_to_attr() {
        let source = "#[cerulion_node]\nstruct Foo {}\n";
        let result = set_macro_policy_arg(source, "period_ms = 10").unwrap();
        assert!(result.contains("#[cerulion_node(period_ms = 10)]"));
    }

    #[test]
    fn test_clear_macro_policy_args_strips_compact_keyvalue() {
        // Regression: `period_ms=100` (no spaces around `=`) used
        // to slip past the filter because `split_whitespace`
        // returned the whole token unchanged.
        let source = "#[cerulion_node(period_ms=100)]\nstruct Foo {}\n";
        let result = clear_macro_policy_args(source).unwrap();
        assert!(
            !result.contains("period_ms"),
            "compact `period_ms=100` must be stripped; got: {result}"
        );
    }

    #[test]
    fn test_clear_macro_policy_args_strips_spaced_keyvalue() {
        // Sanity: the spaced form already worked, but pin it so a
        // future refactor of the splitter can't regress one shape
        // without the other.
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        let result = clear_macro_policy_args(source).unwrap();
        assert!(!result.contains("period_ms"));
    }

    #[test]
    fn test_set_macro_policy_arg_replaces_compact_period() {
        // End-to-end: `set_macro_policy_arg` builds on
        // `clear_macro_policy_args`. Compact input must round-trip
        // correctly through the full helper.
        let source = "#[cerulion_node(period_ms=100)]\nstruct Foo {}\n";
        let result = set_macro_policy_arg(source, "external").unwrap();
        assert!(result.contains("#[cerulion_node(external)]"));
        assert!(!result.contains("period_ms"));
    }

    // ─── ensure_macro_policy_hint scoping tests ──────────────────

    #[test]
    fn test_ensure_macro_policy_hint_ignores_keyword_in_comment() {
        // Regression: a keyword scan run over the WHOLE
        // file lets a comment mentioning `external` short-
        // circuit before the fallback insertion, leaving the bare
        // `#[cerulion_node]` form (no policy) — which fails macro
        // expansion at compile time.
        let source = "\
// This node reads from an external sensor.
#[cerulion_node]
#[derive(Default)]
struct CamNode {
    #[input]
    image: u8,
}
";
        let result = ensure_macro_policy_hint(source).unwrap();
        assert!(
            result.contains("#[cerulion_node(period_ms = 100)]"),
            "the keyword `external` in a comment must NOT suppress the fallback; got:\n{result}"
        );
    }

    #[test]
    fn test_ensure_macro_policy_hint_preserves_existing_policy() {
        // Sanity: when the macro args actually carry a policy, the
        // function leaves the source unchanged.
        let source = "#[cerulion_node(period_ms = 33)]\nstruct Foo {}\n";
        let result = ensure_macro_policy_hint(source).unwrap();
        assert_eq!(result, source);
    }

    // ─── promote_input_field_to_trigger tests ────────────────────

    #[test]
    fn test_promote_input_field_to_trigger_basic() {
        let source = r#"
#[cerulion_node]
struct Consumer {
    #[input]
    count: u32,
}
"#;
        let result = promote_input_field_to_trigger(source, "count").unwrap();
        assert!(
            result.contains("#[input(trigger)]"),
            "expected `#[input(trigger)]`, got: {result}"
        );
    }

    #[test]
    fn test_promote_input_field_to_trigger_idempotent() {
        let source = r#"
#[cerulion_node]
struct Consumer {
    #[input(trigger)]
    count: u32,
}
"#;
        let result = promote_input_field_to_trigger(source, "count").unwrap();
        assert_eq!(result, source, "already-trigger field must round-trip");
    }

    #[test]
    fn test_promote_input_field_to_trigger_missing_field_errors() {
        let source = r#"
#[cerulion_node]
struct Consumer {
    #[input]
    count: u32,
}
"#;
        let err = promote_input_field_to_trigger(source, "ghost").unwrap_err();
        assert!(
            err.contains("could not locate"),
            "expected missing-field error, got: {err}"
        );
    }

    #[test]
    fn test_promote_input_field_to_trigger_picks_correct_field() {
        // Multiple #[input] fields — only the named one gets promoted.
        let source = r#"
#[cerulion_node]
struct Consumer {
    #[input]
    a: u32,
    #[input]
    b: u32,
}
"#;
        let result = promote_input_field_to_trigger(source, "b").unwrap();
        // Only the second `#[input]` becomes `#[input(trigger)]`.
        let trigger_count = result.matches("#[input(trigger)]").count();
        assert_eq!(trigger_count, 1, "exactly one field must be promoted");
        // Verify the right one — `b` should be the trigger, `a` shouldn't.
        let after_a = &result[result.find("a:").unwrap()..];
        let after_b = &result[result.find("b:").unwrap()..];
        assert!(after_a.starts_with("a:"));
        assert!(after_b.starts_with("b:"));
        // Find the `#[input...]` immediately preceding each field.
        let a_block = &result[..result.find("a:").unwrap()];
        let b_block = &result[..result.find("b:").unwrap()];
        // `a`'s preceding attr should be plain `#[input]`; `b`'s should
        // be `#[input(trigger)]`.
        assert!(
            a_block.rfind("#[input(trigger)]") < a_block.rfind("#[input]")
                || !a_block.contains("#[input(trigger)]"),
            "`a` must NOT carry `#[input(trigger)]`"
        );
        assert!(
            b_block.contains("#[input(trigger)]"),
            "`b` must carry `#[input(trigger)]`"
        );
    }

    // ─── clear_input_trigger_attr AST-based tests ────────────────

    #[test]
    fn test_clear_input_trigger_attr_preserves_doc_comment_mention() {
        // A `str::replace` would mutate the `#[input(trigger)]`
        // mention inside the doc comment, silently corrupting the
        // user's source. AST-based rewrite must leave the comment
        // verbatim.
        let source = r#"
/// Use `#[input(trigger)]` on a field to mark it as the data trigger.
#[cerulion_node]
struct Consumer {
    #[input(trigger)]
    count: u32,
}
"#;
        let result = clear_input_trigger_attr(source).unwrap();
        assert!(
            result.contains("Use `#[input(trigger)]` on a field"),
            "doc comment mention must be preserved verbatim; got:\n{result}"
        );
        // The actual field attribute should be cleared.
        assert!(
            !result.contains("#[input(trigger)]\n    count"),
            "the real `#[input(trigger)]` attr on `count` must be cleared"
        );
    }

    #[test]
    fn test_clear_input_trigger_attr_preserves_raw_string_mention() {
        // A `str::replace` would mutate the `#[input(trigger)]`
        // substring inside a raw string literal.
        let source = r##"
const EXAMPLE: &str = r#"#[input(trigger)]"#;
#[cerulion_node]
struct Consumer {
    #[input(trigger)]
    count: u32,
}
"##;
        let result = clear_input_trigger_attr(source).unwrap();
        assert!(
            result.contains(r##"r#"#[input(trigger)]"#"##),
            "raw-string mention must be preserved verbatim; got:\n{result}"
        );
    }

    #[test]
    fn test_clear_input_trigger_attr_noop_when_no_trigger_field() {
        // No `#[input(trigger)]` anywhere on the struct: source
        // round-trips unchanged.
        let source = r#"
#[cerulion_node(period_ms = 100)]
struct Consumer {
    #[input]
    count: u32,
}
"#;
        let result = clear_input_trigger_attr(source).unwrap();
        assert_eq!(result, source);
    }

    // ─── inject_macro_field edge-case tests ──────────────────────

    #[test]
    fn test_inject_macro_field_inserts_comma_when_last_field_lacks_one() {
        // Multi-line body whose last field has no trailing comma.
        // The splice must insert a comma before the new field;
        // otherwise the resulting source is a parse error.
        let source = "\
#[cerulion_node(period_ms = 100)]
struct Foo {
    x: u32
}
";
        let result = inject_macro_field(source, "y", "u32", false, false).unwrap();
        syn::parse_file(&result).unwrap_or_else(|e| {
            panic!(
                "inject must produce valid Rust even when the last field has no trailing \
                 comma, got error: {e}\nresult was:\n{result}"
            )
        });
    }

    #[test]
    fn test_inject_macro_field_preserves_existing_trailing_comma() {
        // Sanity: when the existing last field has a trailing
        // comma, we must NOT insert a duplicate.
        let source = "\
#[cerulion_node(period_ms = 100)]
struct Foo {
    x: u32,
}
";
        let result = inject_macro_field(source, "y", "u32", false, false).unwrap();
        // No double-comma.
        assert!(!result.contains(",,"));
        syn::parse_file(&result).unwrap();
    }

    #[test]
    fn test_inject_macro_field_handles_empty_struct_body() {
        // Single-line `struct Foo {}` — the closing `}` is on the
        // same line as the opening `{`, so a naive line-based
        // splice would land BEFORE `struct Foo` and produce invalid
        // Rust. The implementation detects this and inserts before
        // `}` directly.
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        let result = inject_macro_field(source, "x", "u32", false, false).unwrap();
        // Oracle: result must parse as valid Rust.
        let parsed = syn::parse_file(&result).unwrap_or_else(|e| {
            panic!(
                "inject must produce valid Rust, got: {}\nresult was:\n{}",
                e, result
            )
        });
        // Oracle: struct exists and has exactly one field.
        let struct_item = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    Some(s)
                } else {
                    None
                }
            })
            .expect("expected a struct in the result");
        assert_eq!(struct_item.fields.iter().count(), 1);
    }

    #[test]
    fn test_inject_macro_field_handles_single_line_body() {
        // `struct Foo { x: u32 }` — also single-line, with one
        // existing field. The result must be valid Rust with
        // exactly two fields.
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo { x: u32 }\n";
        let result = inject_macro_field(source, "y", "u32", true, false).unwrap();
        let parsed =
            syn::parse_file(&result).unwrap_or_else(|e| panic!("must parse: {}\n{}", e, result));
        let struct_item = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(struct_item.fields.iter().count(), 2);
    }

    #[test]
    fn test_inject_macro_field_ignores_struct_name_in_raw_string_literal() {
        // A naive substring search for `struct CameraNode` would
        // match inside a raw string literal above the real macro
        // struct, splicing into the wrong location. Span-based
        // positioning sees only the AST's real struct.
        let source = r##"
const EXAMPLE: &str = r#"
    struct CameraNode { something: u32 }
"#;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CameraNode {
    tick_count: u32,
}
"##;
        let result = inject_macro_field(source, "image", "Image", true, false).unwrap();
        // Result must parse and the new field must land in the
        // real (annotated) struct.
        let parsed = syn::parse_file(&result).unwrap();
        let real = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    if s.attrs.iter().any(|a| a.path().is_ident("cerulion_node")) {
                        Some(s)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .unwrap();
        // `tick_count` + new `image` field.
        assert_eq!(real.fields.iter().count(), 2);
        // The raw string's content must remain untouched.
        assert!(result.contains("    struct CameraNode { something: u32 }"));
    }

    #[test]
    fn test_inject_macro_field_ignores_braces_in_doc_comment() {
        // A naive brace-depth counter would walk `{` inside a
        // doc-comment example and land in the wrong place. syn's
        // Span strips comments before AST construction, so the
        // inner braces don't enter the count.
        let source = r#"
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
/// Example body: { x: 1, y: { 2 } }
struct CameraNode {
    tick_count: u32,
}
"#;
        let result = inject_macro_field(source, "image", "Image", true, false).unwrap();
        let parsed = syn::parse_file(&result).unwrap();
        let real = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(real.fields.iter().count(), 2);
    }

    #[test]
    fn test_inject_macro_field_disambiguates_struct_name_prefix_collision() {
        // `struct CameraNodeBuilder` is a substring of
        // `struct CameraNode` if we're naive about word boundaries.
        // The fix walks past prefix matches whose next char is an
        // identifier continuation.
        let source = r#"
struct CameraNodeBuilder { x: u32 }

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CameraNode {
    tick_count: u32,
}
"#;
        let result = inject_macro_field(source, "image", "Image", true, false).unwrap();
        let parsed = syn::parse_file(&result).unwrap();
        // Oracle: helper struct unchanged (still 1 field).
        let helper = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    if s.ident == "CameraNodeBuilder" {
                        Some(s)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(
            helper.fields.iter().count(),
            1,
            "helper struct must NOT be touched"
        );
        // Oracle: macro struct has tick_count + new image field.
        let camera = parsed
            .items
            .iter()
            .find_map(|i| {
                if let syn::Item::Struct(s) = i {
                    if s.ident == "CameraNode" {
                        Some(s)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(camera.fields.iter().count(), 2);
    }

    #[test]
    fn test_toggle_macro_external_flag_disable_sole_external_no_trigger_field_uses_period_fallback()
    {
        // No `#[input(trigger)]` field → the macro requires a
        // policy hint, so dropping `external` would leave
        // `#[cerulion_node()]` which fails to compile. The fix-up
        // falls back to `period_ms = 100` (matching what
        // `generate_macro_lib_rs` emits for fresh source nodes).
        let source = "#[cerulion_node(external)]\nstruct Foo { tick_count: u32 }\n";
        let result = toggle_macro_external_flag(source, false).unwrap();
        assert!(
            !result.contains("#[cerulion_node()]"),
            "must NOT produce empty parens; got:\n{}",
            result
        );
        assert!(
            result.contains("#[cerulion_node(period_ms = 100)]"),
            "must fall back to period_ms = 100 when no trigger field exists; got:\n{}",
            result
        );
        syn::parse_file(&result).unwrap();
    }

    #[test]
    fn test_toggle_macro_external_flag_disable_sole_external_with_trigger_field_drops_to_bare() {
        // A `#[input(trigger)]` field already supplies the trigger
        // policy via declarative inference, so bare
        // `#[cerulion_node]` is buildable. Drop the parens
        // entirely.
        let source =
            "#[cerulion_node(external)]\nstruct Foo {\n    #[input(trigger)]\n    cmd: u8,\n}\n";
        let result = toggle_macro_external_flag(source, false).unwrap();
        assert!(
            result.contains("#[cerulion_node]\n"),
            "must drop to bare form when a trigger field exists; got:\n{}",
            result
        );
        assert!(
            !result.contains("#[cerulion_node()]"),
            "must NOT produce empty parens"
        );
        assert!(
            !result.contains("period_ms"),
            "must NOT inject the period fallback when a trigger field exists"
        );
        syn::parse_file(&result).unwrap();
    }

    // ─── AST-based `is_macro_based` tests ─────────────────────────
    //
    // A naive `str::contains("#[cerulion_node")` would match the
    // substring inside line/doc/block comments and raw string
    // literals — `cerulion node modify` on a raw-FFI
    // node whose source happened to mention `#[cerulion_node]` in
    // a comment or in a `r#"..."#` example string would route through
    // `update_macro_source`, corrupting the source.
    //
    // The implementation parses the source via `syn::parse_file`
    // and walks every reachable `Item::Struct` (recursing into
    // `Item::Mod` for module-nested nodes) for an attribute whose
    // path is exactly the single ident `cerulion_node`. The tests
    // below pin the positive happy paths AND the false-positive
    // classes (line/doc/block comment
    // mentions, raw string literals), plus the additional
    // adversarial classes that fall in the same category:
    // `#[other::cerulion_node]` and `#[::cerulion_node]` qualified
    // paths, `#[derive(cerulion_node)]` (path is `derive`, not
    // `cerulion_node`), `#[cfg_attr(test, cerulion_node)]`
    // (smuggled inside `cfg_attr` args), `#[cerulion_node]` on a
    // non-struct item, and an unparseable-source fallback test.
    // The `Item::Mod` recursion has its own happy-path test.

    #[test]
    fn test_is_macro_based_bare_form_matches() {
        let source = "#[cerulion_node]\nstruct Foo {}\n";
        assert!(
            is_macro_based(source),
            "bare `#[cerulion_node]` on a struct must be detected"
        );
    }

    #[test]
    fn test_is_macro_based_parens_form_matches() {
        let source = "#[cerulion_node(period_ms = 100)]\nstruct Foo {}\n";
        assert!(
            is_macro_based(source),
            "`#[cerulion_node(period_ms = 100)]` on a struct must be detected"
        );
    }

    #[test]
    fn test_is_macro_based_raw_ffi_template_does_not_match() {
        // Mirrors what `generate_lib_rs` (raw FFI) emits: a State
        // struct, FFI fns, no `#[cerulion_node]` anywhere on the
        // struct.
        let source = r#"
use cerulion_core::prelude::*;

struct CameraState { tick_count: u64 }

#[no_mangle]
pub extern "C" fn cerulion_node_init(_ctx: *mut u8) -> u64 { 1 }
"#;
        assert!(
            !is_macro_based(source),
            "raw-FFI source must NOT be detected as macro-based"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_line_comment_mention() {
        let source = r#"
struct Foo {
    // #[cerulion_node] looks like an attribute but it's in a comment
    x: u32,
}
"#;
        assert!(
            !is_macro_based(source),
            "`// #[cerulion_node]` line comment must NOT trigger the detector"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_doc_comment_mention() {
        let source = r#"
/// Example node:
/// `#[cerulion_node]`
/// `struct Camera { ... }`
struct Foo;
"#;
        assert!(
            !is_macro_based(source),
            "`/// #[cerulion_node]` doc comment must NOT trigger the detector"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_block_comment_mention() {
        let source = r#"
/* #[cerulion_node] */
struct Foo;
"#;
        assert!(
            !is_macro_based(source),
            "`/* #[cerulion_node] */` block comment must NOT trigger the detector"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_raw_string_literal() {
        let source = r##"
fn example_template() -> &'static str {
    r#"#[cerulion_node]
struct Camera {}"#
}
"##;
        assert!(
            !is_macro_based(source),
            "`#[cerulion_node]` inside a raw string literal must NOT trigger the detector"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_qualified_path() {
        // Single-segment `cerulion_node` is the only form the CLI
        // template emits. Fully-qualified paths like
        // `#[other::cerulion_node]` are out of the canonical
        // template and must NOT match — routing through
        // `update_macro_source` is safer to skip for non-canonical
        // sources.
        let source = "#[other::cerulion_node]\nstruct Foo {}\n";
        assert!(
            !is_macro_based(source),
            "qualified-path attribute `#[other::cerulion_node]` must NOT match"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_attribute_on_non_struct() {
        // syn allows `#[cerulion_node]` syntactically on non-struct
        // items even though our proc-macro would reject it. We must
        // still return `false` so a non-struct source isn't
        // misrouted.
        let source = r#"
#[cerulion_node]
fn not_a_struct() {}
"#;
        assert!(
            !is_macro_based(source),
            "`#[cerulion_node]` on a non-struct item must NOT trigger the detector"
        );
    }

    #[test]
    fn test_is_macro_based_empty_source() {
        assert!(!is_macro_based(""), "empty source must not match");
    }

    #[test]
    fn test_is_macro_based_use_only() {
        let source = "use cerulion_core::prelude::*;\n";
        assert!(
            !is_macro_based(source),
            "source with only `use` statements must not match"
        );
    }

    #[test]
    fn test_is_macro_based_falls_back_on_parse_error() {
        // Mid-edit source that doesn't parse as Rust. The
        // documented fallback returns the legacy substring
        // result. With `#[cerulion_node]` mentioned in the
        // partial source, the fallback returns `true` —
        // conservative: better to over-route to the (now
        // near-no-op) `update_macro_source` than to silently
        // route a real macro source through `update_info_fn`,
        // which would no-op on a macro file (no
        // `// CERULION:INFO_START` markers to find) and silently
        // skip the modification.
        let source = "#[cerulion_node]\nstruct Foo { x: u32, // unclosed";
        assert!(
            is_macro_based(source),
            "unparseable source with `#[cerulion_node]` mention should fall back to substring match (conservative)"
        );

        // A truly garbage source without the substring → false.
        let source = "fn foo( {";
        assert!(
            !is_macro_based(source),
            "unparseable source without the substring → false"
        );
    }

    #[test]
    fn test_is_macro_based_walks_past_non_cerulion_struct() {
        // Multi-item file: a non-cerulion struct followed by a
        // cerulion struct. The detector must walk every item, not
        // bail after the first.
        let source = r#"
struct Helper { x: u32 }

#[cerulion_node]
struct Real;
"#;
        assert!(
            is_macro_based(source),
            "must detect `#[cerulion_node]` even when an unrelated struct precedes it"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_leading_colon_path() {
        // `#[::cerulion_node]` is a 1-segment path with
        // `leading_colon: Some(_)`. `Path::is_ident` rejects it
        // (requires `leading_colon.is_none()`). Same conservative
        // routing as the qualified-path case.
        let source = "#[::cerulion_node]\nstruct Foo {}\n";
        assert!(
            !is_macro_based(source),
            "leading-`::` path attribute must NOT match"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_derive_form() {
        // `#[derive(cerulion_node)]` carries the substring but the
        // outer attribute path is `derive`, not `cerulion_node`.
        // The proc-macro is an attribute macro, not a derive — a
        // future contributor "improving" detection by walking
        // derive lists for the `cerulion_node` ident would silently
        // re-introduce a false-positive class. This test pins
        // against that.
        let source = "#[derive(cerulion_node)]\nstruct Foo {}\n";
        assert!(
            !is_macro_based(source),
            "`#[derive(cerulion_node)]` must NOT match — `cerulion_node` is an attribute macro, not a derive"
        );
    }

    #[test]
    fn test_is_macro_based_ignores_cfg_attr_smuggling() {
        // `#[cfg_attr(test, cerulion_node)]` makes `cerulion_node`
        // conditional on a cfg gate. The outer attribute path is
        // `cfg_attr`, not `cerulion_node`. Same false-positive
        // class as the derive form.
        let source = "#[cfg_attr(test, cerulion_node)]\nstruct Foo {}\n";
        assert!(
            !is_macro_based(source),
            "`#[cfg_attr(test, cerulion_node)]` must NOT match — `cerulion_node` is smuggled inside `cfg_attr`'s args"
        );
    }

    #[test]
    fn test_is_macro_based_walks_into_inline_modules() {
        // A user wrapping a node in an organisational module
        // `mod inner { ... }` is unusual but not wrong. A
        // substring match catches these by accident; the AST
        // walk matches them intentionally via the
        // `Item::Mod` recursion in `item_carries_cerulion_node_struct`.
        let source = r#"
mod inner {
    #[cerulion_node]
    pub struct Inner;
}
"#;
        assert!(
            is_macro_based(source),
            "module-nested `#[cerulion_node]` struct must still match"
        );
    }
}
