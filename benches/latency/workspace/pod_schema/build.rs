// See pod_codegen.rs — the build-script body that generates the
// PodPayload schema type for the type-class axis. It lives in a
// separate file because it is the single source of the schema text, the
// baked array length and the package qualifier; this crate is its only
// consumer now that the three node crates depend on the crate instead of
// include!'ing the body each.
include!("pod_codegen.rs");
