// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE error type of the dynamic facade.

use crate::codegen::WalkError;

/// Every failure the dynamic facade can report. One variant per distinct
/// cause, so a language binding can map each to its own exception and a
/// test can pin each arm.
///
/// This enum is `#[non_exhaustive]`; downstream matches must include a
/// wildcard arm so new failure variants can be added compatibly.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum DynamicError {
    // ---- schema loading -------------------------------------------------
    /// A YAML document failed to parse (the underlying `serde_yaml` message).
    #[error("schema YAML is not valid YAML: {0}")]
    Yaml(String),

    /// A YAML document has no top-level `schemas:` mapping.
    #[error("invalid schema format: missing 'schemas' key")]
    MissingSchemasKey,

    /// A `schemas:` entry key is not a non-empty string.
    #[error(
        "schema name key {0:?} is not a string or is empty; schema names must be non-empty \
         strings (e.g. 'Image')"
    )]
    InvalidSchemaName(String),

    /// A schema definition is not a mapping.
    #[error("schema '{schema}': definition must be a mapping (`description:` / `fields:`), got a non-mapping YAML value")]
    SchemaNotMapping {
        /// The schema whose definition was rejected.
        schema: String,
    },

    /// A schema's `fields:` value is not a mapping.
    #[error("schema '{schema}': `fields:` must be a mapping of '<type> <name>' keys")]
    FieldsNotMapping {
        /// The schema whose fields value was rejected.
        schema: String,
    },

    /// A `fields:` key of `schema` is not of the form `<type> <name>`.
    #[error("schema '{schema}': invalid field key {key:?}: {reason}")]
    InvalidFieldKey {
        /// The schema whose field key was rejected.
        schema: String,
        /// The offending key verbatim.
        key: String,
        /// Why it was rejected.
        reason: String,
    },

    /// An inline fixed length (`T[n]` / `string_fixed[n]`) exceeds
    /// [`MAX_FIXED_ARRAY_LEN`](super::MAX_FIXED_ARRAY_LEN).
    #[error(
        "schema '{schema}': field key {key:?} declares a {variant} of length {length}: \
         the maximum supported length is {max}"
    )]
    FixedLengthTooLarge {
        /// The schema whose field was rejected.
        schema: String,
        /// The offending key verbatim.
        key: String,
        /// `"FixedArray"` or `"StringFixed"`.
        variant: &'static str,
        /// The declared length.
        length: usize,
        /// The cap.
        max: usize,
    },

    /// A ROS 2 `.msg` text failed to parse (the parser's own message).
    #[error("rosmsg '{name}' failed to parse: {detail}")]
    Rosmsg {
        /// The message name the caller supplied.
        name: String,
        /// The parser's error rendering.
        detail: String,
    },

    /// A file under the workspace `schemas/` directory could not be read.
    #[error("could not read '{path}': {detail}")]
    Io {
        /// The path that failed.
        path: String,
        /// The I/O error rendering.
        detail: String,
    },

    /// A schema's fixed section cannot be sized (a hostile `FixedArray` /
    /// `string_fixed` length overflows `usize`), so it has no wire hash.
    #[error("schema '{schema}' is not wire-representable: {detail}")]
    SchemaNotWireRepresentable {
        /// The rejected schema's qualified name.
        schema: String,
        /// The sizing error.
        detail: String,
    },

    // ---- lookups --------------------------------------------------------
    /// No schema in the set has this qualified name.
    #[error("unknown schema '{0}' (not in the schema set)")]
    UnknownSchema(String),

    /// No schema in the set has this `schema_hash`.
    #[error("unknown schema_hash 0x{0:016X} (no schema in the set matches)")]
    UnknownSchemaHash(u64),

    /// `name` is not a fixed-section field of the layout.
    #[error("'{0}' is not a fixed-section field of this schema")]
    UnknownFixedField(String),

    /// `name` is not a variable (offset-table) field of the layout.
    #[error("'{0}' is not a variable field of this schema")]
    UnknownVariableField(String),

    /// A typed string accessor was used on a field that is not `string`.
    #[error("'{0}' is not a string field")]
    NotAStringField(String),

    /// A typed array accessor was used on a field that is not a primitive
    /// dynamic array.
    #[error("'{0}' is not a primitive dynamic-array field")]
    NotAPrimitiveArrayField(String),

    // ---- encoding -------------------------------------------------------
    /// `var_lens.len()` does not equal the layout's variable-field count.
    #[error("variable length list has {got} entries, layout declares {expected}")]
    VariableCountMismatch {
        /// `layout.variable_fields.len()`.
        expected: usize,
        /// `var_lens.len()`.
        got: usize,
    },

    /// A typed-array variable field's byte length is not a whole number of
    /// elements.
    #[error(
        "variable field '{field}': {len} bytes is not a multiple of the {elem_size}-byte element"
    )]
    LengthNotElementMultiple {
        /// The field.
        field: String,
        /// The requested byte length.
        len: usize,
        /// The element size in bytes.
        elem_size: usize,
    },

    /// The frame would not fit the wire: `total_size` must fit a `u32`;
    /// saturates at `usize::MAX` when the count overflows `usize`.
    #[error("frame of {needed} bytes exceeds the u32 total_size ceiling")]
    FrameTooLarge {
        /// The byte count the frame would need.
        needed: usize,
    },

    /// The caller's buffer is shorter than the frame needs.
    #[error("buffer too small: have {have} bytes, need {need}")]
    BufferTooSmall {
        /// Bytes the frame needs.
        need: usize,
        /// Bytes the buffer holds.
        have: usize,
    },

    // ---- layout ----------------------------------------------------------
    /// A caller-supplied layout is malformed.
    #[error("layout of '{schema}' is malformed: {detail}")]
    InvalidLayout {
        /// The layout's qualified schema name.
        schema: String,
        /// The violated layout invariant.
        detail: &'static str,
    },

    // ---- decoding: header -----------------------------------------------
    /// The buffer is shorter than the 32-byte `WireHeader`.
    #[error("frame too short: have {have} bytes, need at least {need} for the WireHeader")]
    FrameTooShort {
        /// Bytes the buffer holds.
        have: usize,
        /// Bytes required.
        need: usize,
    },

    /// The frame's `schema_hash` is not the hash of the layout the caller
    /// supplied to [`FrameView::with_layout`](super::FrameView::with_layout).
    #[error("frame schema_hash 0x{found:016X} does not match layout 0x{expected:016X}")]
    SchemaHashMismatch {
        /// `layout.schema_hash`.
        expected: u64,
        /// The header's value.
        found: u64,
    },

    /// `WireHeader::total_size` claims more bytes than the buffer holds.
    #[error("header total_size {total_size} exceeds the {have}-byte buffer")]
    TotalSizeExceedsBuffer {
        /// The header's claim.
        total_size: u32,
        /// Bytes the buffer holds.
        have: usize,
    },

    /// `WireHeader::total_size` is smaller than the frame prefix the layout
    /// requires (header + fixed section + offset table).
    #[error("header total_size {total_size} is below the {need}-byte frame prefix of '{schema}'")]
    TotalSizeBelowPrefix {
        /// The schema.
        schema: String,
        /// The header's claim.
        total_size: u32,
        /// Header + fixed section + offset table.
        need: usize,
    },

    /// The header's offset-table position or count disagrees with the
    /// layout the hash resolved to.
    #[error(
        "header offset table (offset {offset}, count {count}) does not match layout \
         (offset {expected_offset}, count {expected_count})"
    )]
    OffsetTableMismatch {
        /// `WireHeader::SIZE + fixed_size`.
        expected_offset: u32,
        /// The header's value.
        offset: u32,
        /// `layout.variable_fields.len()`.
        expected_count: u32,
        /// The header's value.
        count: u32,
    },

    // ---- decoding: offset table -----------------------------------------
    /// An offset-table entry points into the fixed section or the table
    /// itself.
    #[error("variable field '{field}' offset {offset} is below the data floor {data_floor}")]
    OffsetBelowDataFloor {
        /// The field.
        field: String,
        /// The entry's payload-relative offset.
        offset: u32,
        /// `layout.data_floor()`.
        data_floor: usize,
    },

    /// An offset-table entry extends past `total_size`.
    #[error(
        "variable field '{field}' out of bounds: offset {offset} len {length} vs payload len {payload_len}"
    )]
    VariableFieldOutOfBounds {
        /// The field.
        field: String,
        /// The entry's payload-relative offset.
        offset: u32,
        /// The entry's byte length.
        length: u32,
        /// `total_size - WireHeader::SIZE`.
        payload_len: usize,
    },

    /// Two offset-table entries with non-zero length share bytes.
    #[error("variable fields '{first}' and '{second}' overlap")]
    OverlappingEntries {
        /// The earlier-declared field.
        first: String,
        /// The later-declared field.
        second: String,
    },

    /// A typed-array entry is not element-aligned or not a whole number of
    /// elements - it could not be viewed in place as `&[T]`.
    #[error(
        "variable field '{field}' (offset {offset}, len {length}) is not aligned to its \
         {elem_size}-byte element"
    )]
    MisalignedElements {
        /// The field.
        field: String,
        /// The entry's payload-relative offset.
        offset: u32,
        /// The entry's byte length.
        length: u32,
        /// The element size (== required alignment).
        elem_size: usize,
    },

    /// A typed-array entry is element-aligned relative to the payload but not
    /// in memory because the frame buffer itself is not suitably aligned.
    #[error("variable field '{field}' at payload offset {offset} is not {elem_size}-byte aligned in memory (the frame buffer itself must be aligned to the widest element)")]
    MisalignedBuffer {
        /// The field.
        field: String,
        /// The entry's payload-relative offset.
        offset: u32,
        /// The element size.
        elem_size: usize,
    },

    /// A `string` field's bytes are not valid UTF-8.
    #[error("string field '{field}' is not valid UTF-8 (error at byte {valid_up_to})")]
    InvalidUtf8 {
        /// The field.
        field: String,
        /// `Utf8Error::valid_up_to`.
        valid_up_to: usize,
    },

    /// The full structural walk ([`crate::codegen::FrameWalker`]) refused
    /// the frame.
    #[error(transparent)]
    Walk(#[from] WalkError),

    // ---- message YAML (the `message_to_yaml` / `yaml_to_message` codec) --
    // `path` is the full field path (`pose.position.x`, `poses[2].x`,
    // `(message)` for the root). The Display text of these arms is a
    // user-facing contract: `topic pub` prints it as is.
    /// The message YAML bytes are not UTF-8.
    #[error("message YAML is not valid UTF-8 (invalid byte at offset {valid_up_to})")]
    YamlInputNotUtf8 {
        /// `Utf8Error::valid_up_to`.
        valid_up_to: usize,
    },

    /// The message YAML does not parse (the parser's message: bad syntax,
    /// a duplicate key, more than one document, nesting past its limit).
    #[error("message YAML does not parse: {0}")]
    YamlSyntax(String),

    /// The message YAML uses an anchor (`&name`) or an alias (`*name`).
    #[error(
        "message YAML uses an anchor or alias at line {line}, column {column}; write every \
         value out in full"
    )]
    YamlAnchorOrAlias {
        /// 1-based line.
        line: usize,
        /// 1-based column (in characters).
        column: usize,
    },

    /// A value has the wrong YAML kind for its field.
    #[error("{path}: expected {expected}, got {got}")]
    YamlWrongType {
        /// The field path.
        path: String,
        /// The field type (`float64`, `geometry_msgs/Point`, `uint8[]`).
        expected: String,
        /// The YAML kind found (`string`, `integer`, `float`, `boolean`,
        /// `null`, `sequence`, `mapping`, `tagged value`).
        got: String,
    },

    /// A number does not fit its field type.
    #[error("{path}: {value} is out of range for {expected}")]
    YamlOutOfRange {
        /// The field path.
        path: String,
        /// The number as written.
        value: String,
        /// The type with its bounds (`uint8 (0 to 255)`).
        expected: String,
    },

    /// A mapping key names no field of the message.
    #[error("{path}: unknown field; {schema} has fields: {known}")]
    YamlUnknownField {
        /// The field path, ending in the unknown key.
        path: String,
        /// The message type.
        schema: String,
        /// The type's field names, in declaration order.
        known: String,
    },

    /// A field of the message is not in the mapping.
    #[error("{path}: missing field required by {schema}")]
    YamlMissingField {
        /// The field path, ending in the missing field.
        path: String,
        /// The message type.
        schema: String,
    },

    /// A fixed-size array has the wrong number of elements.
    #[error("{path}: expected exactly {expected} elements, got {got}")]
    YamlWrongLength {
        /// The field path.
        path: String,
        /// The declared length.
        expected: usize,
        /// The number of elements written.
        got: usize,
    },

    /// A string is longer than its `string_fixed[n]` bound.
    #[error("{path}: string is {got} bytes, over the bound of {bound} bytes")]
    YamlStringTooLong {
        /// The field path.
        path: String,
        /// The bound `n`, in bytes.
        bound: usize,
        /// The UTF-8 length written.
        got: usize,
    },

    /// A `string_fixed[n]` value contains a NUL character (the wire form
    /// ends the string at the first NUL).
    #[error("{path}: a string_fixed value cannot contain a NUL character")]
    YamlNulInFixedString {
        /// The field path.
        path: String,
    },

    /// The YAML is display output with an array cut short.
    #[error(
        "{path}: this array was cut short for display (elements left out: {omitted}); print \
         the message again with a larger --truncate-length"
    )]
    YamlTruncated {
        /// The path of the `!truncated` element.
        path: String,
        /// The count the tag carries.
        omitted: String,
    },

    /// A YAML tag (`!name`) on a value.
    #[error("{path}: YAML tag {tag} is not supported")]
    YamlTag {
        /// The field path.
        path: String,
        /// The tag as written.
        tag: String,
    },

    /// The schema has a field type the YAML form does not cover.
    #[error("{path}: field type {field_type} has no YAML form")]
    YamlUnsupportedField {
        /// The field path.
        path: String,
        /// The field type.
        field_type: String,
    },

    /// A nested type the message refers to is not in the schema set.
    #[error("{path}: nested type {target} is not in the schema set")]
    YamlUnresolvedNested {
        /// The field path.
        path: String,
        /// The unresolved type name.
        target: String,
    },

    /// Messages nest deeper than the codec allows.
    #[error("{path}: messages nest deeper than {max} levels")]
    YamlTooDeep {
        /// The field path where the bound was passed.
        path: String,
        /// The bound.
        max: usize,
    },

    /// The message would pass the 1 GiB frame cap.
    #[error("{path}: the message is larger than the 1 GiB frame cap")]
    YamlMessageTooLarge {
        /// The field path being built when the cap was passed.
        path: String,
    },

    /// A frame value has no YAML form.
    #[error("{path}: {detail}; the frame has no YAML form")]
    YamlUnencodable {
        /// The field path.
        path: String,
        /// What is wrong with the value.
        detail: String,
    },
}
