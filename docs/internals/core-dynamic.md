# cerulion_core internals - `cerulion_core::dynamic`

The public, stable-intent facade over the runtime schema machinery for **language
bindings** (Python first): decode and encode Cerulion wire frames by `schema_hash`
with no generated Rust message types. Companions: `core-transport.md` (how frames
move), `core-testing.md` (the test map). Code on `main` beats this document.

Source: `crates/cerulion_core/src/dynamic/` - `schema_yaml.rs`, `schema_set.rs`,
`encoder.rs`, `view.rs`, `error.rs`, `tests.rs`; integration pins in
`tests/dynamic_generated_parity_test.rs` and `tests/dynamic_zero_alloc_test.rs`.

## Contract

| Item | Guarantee |
|---|---|
| `SchemaSet`, `FrameEncoder`/`FrameCursor`, `FrameView`, `DynamicError` | Public API; a change to a listed method signature or an error-variant *meaning* is a breaking change and needs a PR-title `!`. |
| Re-exports (`MessageSchema`, `FieldType`, `WireLayout`, `FrameWalker`, `FrameValueKind`, `PrimArray`, `WireHeader`, `OffsetEntry`, …) | The binding surface for those types. Their *definitions* live in `codegen`/`wire`; the re-export path is the stable name. |
| `WireLayout::to_json()` | Deterministic: serde field order = struct declaration order, vectors in declaration order; pinned by an oracle string. |
| Bytes produced by `FrameEncoder` | Byte-identical to the generated `<Name>Shm` writer + `OutputProxy` path for the same values, `sequence` excepted (see below). Pinned against generated `ChannelFloat32`, `Image` and `ChannelFloat32Shm`. |
| `FrameEncoder::required_len`/`begin`, every `FrameCursor` method and every `FrameView` method except `FrameView::decode` (which materialises a `FrameValue` via `FrameWalker::walk_by_hash` and allocates) | No heap allocation on the SUCCESS path (`dynamic_zero_alloc_test`). Error arms may allocate (they carry the field/schema name as a `String`). |
| `DynamicError` | ONE `#[non_exhaustive]` `thiserror` enum; every malformed-frame class is a distinct variant (table below). Adding a variant is non-breaking for `match _ =>` users; removing/renaming is breaking. |

Not stable (may change without notice): `Debug` output, error `Display` wording,
the exact set of resolution *warning* strings, and anything under `codegen::` that is
not re-exported here. `docs/user-api.md` does not mention this module on purpose: it is
a contributor/binding surface, not end-user surface yet.

## Schema loading - `SchemaSet`

```rust
let (mut set, warnings) = SchemaSet::from_schemas(vec![])?;     // owns Vec<MessageSchema>
set.add_yaml_str(yaml)?;                                       // `schemas:` mapping
set.add_rosmsg_str(text, "Go2FrontVideoData", Some("unitree_go"))?;
let (set, warnings) = SchemaSet::from_workspace_dir(root)?;   // <root>/schemas/*.yaml + <pkg>/msg/*.msg
set.walker(); set.layout("Probe"); set.layout_for_hash(h); set.schema_hash("Probe");
```

- **One parser.** `dynamic::parse_yaml_schemas` is THE workspace-YAML parser; the CLI
  (`cerulion_cli_engine::schema_cmd::parse_message_schemas`) delegates to it, so the CLI
  and a binding can never disagree on which files parse or on a hash
  (`dynamic_generated_parity_test::yaml_schema_set_matches_cli_parser_on_example_workspaces`).
  Rules: top-level `schemas:` mapping; entry keys non-empty strings; field keys exactly
  `<type> <name>`; `FixedArray`/`StringFixed` lengths ≤ `MAX_FIXED_ARRAY_LEN` (2^20),
  recursively - this bound is what keeps layout arithmetic from overflowing on a hostile
  file. `checked_wire_fixed_size()` is the final representability gate.
- **Warnings are loud twice.** Every `LayoutResolver` warning (unresolved nested target,
  skipped workspace file, …) is `tracing::warn!`ed AND returned in the `Vec<String>`.
  A failed `add_*` leaves the set unchanged.
- **Nested references bind on identity.** `resolve_fixed_nested`, `LayoutResolver`,
  `FrameWalker` and the capture-time `header.stamp` locator (`codegen/header_stamp.rs`)
  all bind a nested reference through one registry
  (`codegen/nested_index.rs`) keyed on each schema's own `(package, name)`, never on
  its qualified-name text. A package-less YAML entry named `pkg/Type` keeps that text
  for name lookups (`layout(name)`, `walk`), where the later definition wins; a store
  schema's reference to `Type` still binds the store's `(Some("pkg"), "Type")`, the
  definition the resolver inlined, and both twins stay reachable by their own
  `schema_hash` (`walk_by_hash`, `layout_for_hash`). A consumer that resolved a name from
  a hash decodes with `walk_by_hash`, never `walk(name)`, so a twin never decodes under
  the other twin's layout. A nested reference that does not bind is never a panic, but
  it is not always an error: `resolve_fixed_nested` warns and leaves the field variable,
  so the layout still succeeds, and an unresolved array element can decode as
  `NestedArrayOpaque`. `LayoutError::UnresolvedNested` covers only a fixed stamp that no
  longer binds in the set, and `WalkError::NestedResolutionFailed` a nested frame whose
  reference does not bind at walk time. A fixed section whose inlined size overflows
  `usize` is `LayoutError::FixedSectionOverflow`, and the walker leaves that schema out.
- `from_schemas` returns `Result` and preflights every build through three gates:
  declared fixed size, composed overflow via `composed_overflow_indices`, and
  resolved frame prefix ≤ `u32` for every layout a frame can reach, by name or by hash
  (a twin hidden from name lookups included). `add_*` is all-or-nothing. The workspace loader
  skips offenders to a fixpoint with warnings. `layout(name)` is a direct name
  lookup, never a hash-index lookup.
- `dynamic::parse_rosmsg` is the checked parser (`codegen::parse_rosmsg` is the raw
  grammar parser); composed overflow is checked when constructing a `SchemaSet`.
- `from_workspace_dir` sorts directory entries, so load order - and therefore
  `schemas()` order - is deterministic across filesystems.
- Workspace YAML outranks the `.msg` store on a qualified-name collision (the
  CLI's rule). A schema, YAML or store, that referenced the shadowed definition
  bound it under the resolver's `(package, name)` key, which the package-less
  YAML twin never satisfies, so it is dropped with a warning (transitively, like
  a parent of a skipped schema) rather than loaded with the field re-resolved as
  opaque bytes and a changed hash. Dependents are bound by registry key, so a
  slash-named YAML schema is never a bare-name candidate and a bare YAML name is.

## Layout - `WireLayout`

```text
WireLayout { qualified_name, schema_hash, fixed_size, fixed_align,
             fixed_fields:    [FieldLayout { name, offset, size, align, field_type }],
             variable_fields: [VariableFieldLayout { name, field_type }] }
offset_table_offset() = fixed_size            (payload-relative)
offset_table_bytes()  = 8 * variable_fields.len()
data_floor()          = fixed_size + offset_table_bytes()
```

Fixed fields are `#[repr(C)]`-placed (natural alignment, tail padding to
`fixed_align`); fixed-nested schemas inline. `string`, `bytes`, `T[]` and
variable nested values are variable fields in declaration order - the index of a
variable field is its offset-table row.

## Frame - header, offset table, payload

```text
[WireHeader 32 B][fixed section fixed_size B][offset table 8 B × n][variable payloads]
header (LE): schema_hash u64 | total_size u32 | offset_table_offset u32 (= 32 + fixed_size)
             | offset_table_count u32 (= n) | sequence u32 | timestamp_ns u64
offset entry (LE): offset u32 (payload-relative, i.e. from byte 32) | length u32
```

Placement rule (shared by generated writers, `rmw_cerulion` and `FrameEncoder`):
cursor starts at `data_floor()`; for each variable field in declaration order,
align the cursor up to `variable_payload_align(field_type)` - 2/4/8 for
`i16/u16`, `i32/u32/f32`, `i64/u64/f64` dynamic arrays, 1 for everything else -
record `(cursor, len)`, advance by `len`. Every byte not written by a field
(fixed-section padding, alignment padding) is **zero**. `total_size = 32 + cursor`.

`sequence`: the transport stamps it at commit (`OutputProxy::Drop`), so `begin`
writes 0 and `FrameCursor::set_sequence` exists for replay and for parity pins (the
parity test copies the generated frame's `sequence` in, then asserts all 80+ bytes).

### Worked byte example

Schema `Probe { uint32 id; uint8 flag; string name; float64[] samples }`, values
`id = 0x01020304, flag = 7, name = "ab", samples = [1.0, 2.0]`, `timestamp_ns =
0x1122334455667788`, `schema_hash = H`:

```text
off  bytes                                     meaning
 0   H (8 B LE)                                schema_hash
 8   50 00 00 00                               total_size = 80
12   28 00 00 00                               offset_table_offset = 32 + 8
16   02 00 00 00                               offset_table_count = 2
20   00 00 00 00                               sequence (transport stamps)
24   88 77 66 55 44 33 22 11                   timestamp_ns
32   04 03 02 01                               id
36   07                                        flag
37   00 00 00                                  fixed tail padding (align 4)
40   18 00 00 00  02 00 00 00                  name   @24 len 2   (24 = data_floor)
48   20 00 00 00  10 00 00 00                  samples @32 len 16 (26 aligned up to 8)
56   61 62                                     "ab"
58   00 00 00 00 00 00                         alignment padding
64   00 00 00 00 00 00 F0 3F                   1.0f64
72   00 00 00 00 00 00 00 40                   2.0f64
80                                             total_size
```

This is `tests.rs::probe_oracle_frame`, asserted against `FrameEncoder` output and
read back field-by-field through `FrameView`.

## Encoder contract - `FrameEncoder` / `FrameCursor`

```rust
let enc = FrameEncoder::new(layout)?;                // the only allocation
let n   = enc.required_len(&[2, 16])?;               // 80
let mut cur = enc.begin(&mut slot, &[2, 16], ts)?;   // zeroes [0,n), writes header + table
cur.fixed_field_mut("id")?.copy_from_slice(&id.to_le_bytes());
cur.variable_field_mut("samples")?...;               // exactly the length you declared
let written = cur.finish();                          // == n
```

- `var_lens` is one byte length per variable field in declaration order; a typed
  array's length must be a whole number of elements (`LengthNotElementMultiple`).
- `begin` refuses `total_size > u32::MAX` (`FrameTooLarge`) and a short buffer
  (`BufferTooSmall { need, have }`); bytes past `total_size` are untouched, so a
  caller may hand over an oversized transport loan and publish `finish()` bytes.
- A cursor on which nothing is written is still a well-formed frame with zeroed
  values (empty strings, zero primitives).

## View contract - `FrameView`

`FrameView::new(walker, frame)` resolves the header's `schema_hash` through the
`FrameWalker` and validates the header and top-level offset table once (the inner
structure of a nested variable entry is checked only by `decode`); every accessor
afterwards is a bounds-safe slice of `frame[..total_size]`. `with_layout` skips the hash
lookup for a cached layout. `str_field` is the loud UTF-8 arm (the walker degrades
to bytes), `prim_array_field` hands out an element-aligned `PrimArray`, and
`decode` is the convenience over `FrameWalker::walk_by_hash` for the full typed
tree (nested values, element arrays - this path allocates).

`FrameView` is stricter than `FrameWalker`: the walker accepts out-of-order,
overlapping and misaligned top-level entries as legal wire; the view refuses them
because a binding hands its slices out as in-place typed arrays. `(0,0)` and
zero-length entries are accepted as empty. Typed-array entries must be aligned in
memory too, so the frame buffer must start at an 8-aligned address (the widest
element). A transport loan does. A fresh `Vec<u8>` from every system allocator
does in practice, but that is an allocator property, not a Rust guarantee (`u8`
allocations are promised 1-byte alignment only), and a sub-slice of a byte buffer
at an offset that is not a multiple of 8 does not. A binding that sees
`MisalignedBuffer` copies the frame into an 8-aligned buffer (or aligns its own)
and retries; `FrameEncoder::begin` refuses such a buffer up front for the same
reason. Both `with_layout` and `FrameEncoder::new` validate a supplied layout once
(`InvalidLayout`). `new` and `with_layout` parse the header ONCE per frame
(`dynamic_zero_alloc_test` counts it through `wire::header_parse_count`).

### Error arms (each pinned by an adversarial test in `tests.rs`)

| Class | `DynamicError` variant |
|---|---|
| frame shorter than 32 B | `FrameTooShort { have, need }` |
| hash unknown to the walker / hash ≠ supplied layout | `UnknownSchemaHash(u64)` / `SchemaHashMismatch { expected, found }` |
| `total_size` > supplied buffer | `TotalSizeExceedsBuffer { total_size, have }` |
| `total_size` < header + fixed + table | `TotalSizeBelowPrefix { schema, total_size, need }` |
| header table position/count ≠ layout | `OffsetTableMismatch { .. }` |
| entry offset < `data_floor` | `OffsetBelowDataFloor { field, offset, data_floor }` |
| entry end > `total_size` | `VariableFieldOutOfBounds { field, offset, length, payload_len }` |
| two non-empty entries overlap | `OverlappingEntries { first, second }` |
| typed-array offset or length not an element multiple | `MisalignedElements { field, offset, length, elem_size }` |
| typed-array payload offset is not aligned in memory | `MisalignedBuffer { field, offset, elem_size }` |
| supplied layout is malformed | `InvalidLayout { schema, detail }` |
| `string` field not UTF-8 | `InvalidUtf8 { field, valid_up_to }` |
| nested resolution failure and other walker structural errors (via `decode`) | `Walk(WalkError)` (transparent) |
| schema/field lookup | `UnknownSchema`, `UnknownSchemaHash`, `UnknownFixedField`, `UnknownVariableField`, `NotAStringField`, `NotAPrimitiveArrayField` |
| encoding | `VariableCountMismatch`, `LengthNotElementMultiple`, `FrameTooLarge`, `BufferTooSmall` |
| loading | `Yaml`, `MissingSchemasKey`, `InvalidSchemaName`, `SchemaNotMapping`, `FieldsNotMapping`, `InvalidFieldKey`, `FixedLengthTooLarge`, `Rosmsg`, `Io`, `SchemaNotWireRepresentable` |

## Message YAML - `message_to_yaml` / `yaml_to_message`

```rust
let yaml  = message_to_yaml(set.walker(), &frame)?;                       // full form
let shown = message_to_yaml_with(walker, &frame, YamlEncodeOptions { truncate_arrays: Some(128), max_lines: Some(2000) })?;
let frame = yaml_to_message(walker, "geometry_msgs/Pose", yaml.as_bytes(), timestamp_ns)?;
```

The user-facing form is documented in `docs/user-api.md` ("Message YAML"); its
keys, layout and error text are a contract. Source: `dynamic/message_yaml/`
(`encode.rs`, `decode.rs`, `parse.rs`, `scan.rs`, `tests.rs`).

- **Encode** walks the frame with `FrameWalker::walk_by_hash_keeping`, holding at most
  `truncate_arrays` decoded elements per array (every element is still decoded once and
  checked, so a bad element past the cut refuses the array as the full walk does; the
  full count comes from the schema or the array's raw bytes). A zero-size message (no
  wire bytes: an empty message, or one made only of fixed arrays of them) is printed
  from the schema, and the keeping walk holds none of a fixed array of them, so a
  32-byte frame that declares 2^40 empty elements costs only the output cap. It then
  prints fields in DECLARATION order from the resolved schema of the frame's own hash
  (`FrameWalker::schema_for_hash`; a twin that owns the same qualified name never stands in)
  and of each nested target (`FrameWalker::nested_target`): a `WireLayout` splits fields
  into a fixed and a variable group and loses that order. The walker's degradations
  become loud `YamlUnencodable` arms (a string that is not UTF-8, an array that is not
  canonically framed, a primitive array whose bytes end partway through an element,
  a `bool[]` byte other than 0/1). The walker reads an empty `bool[]` blob as an
  empty element array, which prints `[]` like every other empty array. The walker refuses a frame whose messages nest past
  `MAX_WALK_NESTING` (256; only a recursive schema can) with `NestingTooDeep`. The empty variable-nested blob
  idiom prints the target's zero values, so the YAML is complete; decoding that YAML
  writes the full sub-frame, so that one frame shape is not byte-preserved.
- **Decode** builds every body through `CanonicalBodyBuilder` (the shared placement
  rule) and resolves nested types through `FrameWalker::nested_target` (the walker's
  own identity binding on the parent's package, which gives the layout and the schema
  from one slot). `MAX_BODY_BYTES` counts every byte the unfinished bodies and arrays on
  the build stack hold, since all of them end up in the frame: a body whose fixed
  section plus offset table does not fit what is left is refused before the builder
  allocates it, and a string or scalar array that does not fit is refused before its
  bytes are built. So a few bytes of YAML naming many large fixed sections (unset
  `string_fixed` fields) are refused at the first one past the cap, not after all are
  held. Input past `MAX_YAML_INPUT_BYTES` (the encoder's cap plus 1 MiB for echo's
  comment lines and separator) is refused before parsing, which bounds the value tree. One final `---` line (echo's frame separator) is dropped before parsing.
  `parse.rs` builds the `serde_yaml::Value` tree with its own visitor so an integer
  past 64 bits (which `Value` cannot hold, and serde_yaml refuses with no path) is
  kept as its decimal text: an integer field refuses it as `YamlOutOfRange` at its
  path, a float field reads it like any integer. A typed tag that spells the
  parser's mark for such an integer (`!integer%20past%2064%20bits`) is refused as a tag.
  serde_yaml expands aliases silently, so `scan.rs` refuses an anchor or
  alias before parsing: `&`/`*` at the start of a node, outside quotes, comments and
  block scalars. It splits lines at every break the parser reads (LF, CR, CRLF, NEL,
  LS, PS), so a CR-only document is not one long line. A block scalar's content is what is more indented than the node that
  owns it (the key of `- key: |`, not the dash), so a sibling key is still scanned. A multi-line plain scalar whose continuation line starts with `&` or
  `*` is a false refusal (quote the string).
- **Floats**: shortest round-trip form; an `f32` is recovered by picking the neighbour
  of `x as f32` whose shortest form reads back to the parsed `f64` (two `f32` values
  round twice under a plain cast; bit pattern `363742205` is pinned). Every NaN prints
  `.nan` and reads back as the quiet NaN, so a NaN's sign and payload bits are the one
  thing the round trip does not keep.
- **Bounds**: `MAX_YAML_DEPTH` (64) on both directions, a message (a fieldless `{}`
  too) and an array of messages a level each (each is one YAML mapping or sequence, so printed YAML stays
  inside the reader's own nesting limit); `MAX_YAML_ARRAY_DEPTH` (the
  walker's `MAX_NESTED_ARRAY_DEPTH`, 8) on decode: a nonempty element array inside
  that many variable-message elements is refused, since the walker hands it back
  opaque, and an empty one there is written as the empty blob the walker reads as zero
  elements (not a zero count), so it prints back; `MAX_YAML_BYTES` (256 MiB) on
  encoder output, because an unset nested field expands to its schema's zero values and a
  40-byte frame can name gigabytes (a zero-default fixed array is counted, never built).
  A quoted string is escaped against the remaining cap, so a long string of control bytes
  is refused before its escaped form is built. Every write goes through `Emitter::put`, which counts it against the byte cap and the
  optional `max_lines` budget first; `max_lines` stops the output at a whole line and ends
  it with a column-0 `!truncated` line;
  `string_fixed[n]` and fixed array lengths are enforced. ROS `<=N` bounds ride on
  `FieldDef::bounds` (kept by the `.msg` parser, outside the wire type and the schema
  hash) and are enforced on decode only (`YamlTooManyElements`, `YamlStringTooLong`);
  the encoder prints an over-bound frame as it is. `wstring<=N` counts UTF-16 units, so
  its bound is not kept.
- Neither direction is a publish path (both allocate); the module is outside the
  hot-path allocation sweep.

| Class | `DynamicError` variant |
|---|---|
| input bytes / parse / anchor | `YamlInputTooLarge`, `YamlInputNotUtf8`, `YamlSyntax`, `YamlAnchorOrAlias` |
| value | `YamlWrongType`, `YamlOutOfRange`, `YamlUnknownField`, `YamlMissingField`, `YamlWrongLength`, `YamlTooManyElements`, `YamlStringTooLong`, `YamlNulInFixedString`, `YamlTruncated`, `YamlTag` |
| schema shape | `YamlUnsupportedField`, `YamlUnresolvedNested`, `YamlTooDeep`, `YamlMessageTooLarge` |
| encode | `YamlUnencodable`, `YamlOutputTooLarge` (plus `Walk` for a frame the walker refuses) |

`YamlSyntax` text past its `message YAML does not parse: ` prefix is serde_yaml's own
(only the trailing ` at line L column C` is ours): tests pin the prefix and the location,
never the parser's wording.

Tests use independent producers only: hand-written frames, and hand-written CDR bodies
turned into frames by `CdrCodec`, against hand-written YAML. `every_corpus_type_matches_the_oracle_both_ways`
covers one message of every vendored type against two oracles: the frame from `CdrCodec`
and the YAML from the test's own `Sample` writer, which spells out the documented form
with no call into the codec, so encode must print that YAML and decode must build that
frame; `encoding_is_deterministic_across_runs` re-runs the corpus in the same process, and
`encoding_is_deterministic_across_processes` (Unix only) re-runs it in a second process.

## Tests

```bash
cargo test -p cerulion_core --lib dynamic::                        # unit: happy/edge/adversarial/determinism/error arms
cargo test -p cerulion_core --test dynamic_generated_parity_test   # bytes == generated writers; YAML == CLI parser
cargo test -p cerulion_core --test dynamic_zero_alloc_test         # no alloc after FrameEncoder::new / in FrameView
cargo test -p cerulion_core --lib dynamic::message_yaml            # message YAML codec vectors + corpus oracle
```

The parity binary compares against generated `native_ros2_messages` writers - never a
`FrameEncoder` run against itself. New encoding rules land with a hand-written byte
oracle here AND a generated-type pin there.
