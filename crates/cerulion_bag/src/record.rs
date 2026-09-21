// SPDX-License-Identifier: AGPL-3.0-only
//! MCAP framing primitives — opcodes, magic bytes, and record encoders.
//!
//! Every encoder appends a fully-framed record (`opcode: u8`, then
//! `content_length: u64` little-endian, then content) into a caller-provided
//! reusable scratch [`Vec<u8>`] arena. All multi-byte integers are
//! little-endian, per the MCAP spec (<https://mcap.dev/spec>).
//!
//! # Zero-copy contract
//!
//! The two hot-path record shapes — a [`Message`](encode_message_frame) and a
//! [`Chunk`](encode_chunk_header) — encode only their *framing* into scratch.
//! The message payload and the chunk body never enter a scratch buffer: they
//! ride to `writev(2)` as separate iovec entries pointing straight at the
//! caller's (shared-memory) buffers. Only cold-path config records
//! (attachments, the summary section) are fully materialised in scratch.

/// The 8-byte MCAP magic at the start and end of every file.
/// `0x89 M C A P 0x30 \r \n` — `0x30` is the ASCII major version `'0'`.
pub const MAGIC: [u8; 8] = [0x89, b'M', b'C', b'A', b'P', 0x30, b'\r', b'\n'];

/// MCAP record opcodes.
pub mod op {
    /// Header record.
    pub const HEADER: u8 = 0x01;
    /// Footer record.
    pub const FOOTER: u8 = 0x02;
    /// Schema record.
    pub const SCHEMA: u8 = 0x03;
    /// Channel record.
    pub const CHANNEL: u8 = 0x04;
    /// Message record.
    pub const MESSAGE: u8 = 0x05;
    /// Chunk record.
    pub const CHUNK: u8 = 0x06;
    /// Message Index record.
    pub const MESSAGE_INDEX: u8 = 0x07;
    /// Chunk Index record.
    pub const CHUNK_INDEX: u8 = 0x08;
    /// Attachment record.
    pub const ATTACHMENT: u8 = 0x09;
    /// Attachment Index record.
    pub const ATTACHMENT_INDEX: u8 = 0x0A;
    /// Statistics record.
    pub const STATISTICS: u8 = 0x0B;
    /// Summary Offset record.
    pub const SUMMARY_OFFSET: u8 = 0x0E;
    /// Data End record.
    pub const DATA_END: u8 = 0x0F;
}

// ---------------------------------------------------------------------------
// Little-endian primitive appenders. Each appends to the end of `out`.
// ---------------------------------------------------------------------------

#[inline]
fn put_u8(out: &mut Vec<u8>, v: u8) {
    out.push(v);
}
#[inline]
fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}
#[inline]
fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}
#[inline]
fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}
/// MCAP string: `u32` byte-length prefix + UTF-8 bytes.
#[inline]
fn put_str(out: &mut Vec<u8>, s: &str) {
    put_u32(out, s.len() as u32);
    out.extend_from_slice(s.as_bytes());
}
/// MCAP length-prefixed bytes: `u32` byte-length prefix + bytes.
#[inline]
fn put_bytes_u32(out: &mut Vec<u8>, b: &[u8]) {
    put_u32(out, b.len() as u32);
    out.extend_from_slice(b);
}

/// A framed record header: opcode + `content_len` (u64, little-endian).
#[inline]
fn put_frame(out: &mut Vec<u8>, opcode: u8, content_len: u64) {
    put_u8(out, opcode);
    put_u64(out, content_len);
}

// ---------------------------------------------------------------------------
// Fully-materialised records (framing + content in scratch).
// ---------------------------------------------------------------------------

/// Header record: `profile`, `library`.
pub fn encode_header(out: &mut Vec<u8>, profile: &str, library: &str) {
    let content_len = (4 + profile.len() + 4 + library.len()) as u64;
    put_frame(out, op::HEADER, content_len);
    put_str(out, profile);
    put_str(out, library);
}

/// Schema record: `id`, `name`, `encoding`, length-prefixed `data`.
pub fn encode_schema(out: &mut Vec<u8>, id: u16, name: &str, encoding: &str, data: &[u8]) {
    let content_len = (2 + 4 + name.len() + 4 + encoding.len() + 4 + data.len()) as u64;
    put_frame(out, op::SCHEMA, content_len);
    put_u16(out, id);
    put_str(out, name);
    put_str(out, encoding);
    put_bytes_u32(out, data);
}

/// Channel record: `id`, `schema_id`, `topic`, `message_encoding`, `metadata`.
///
/// `metadata` is emitted VERBATIM in the order given — the caller owns
/// determinism (see [`crate::provisioning::ChannelProvisioning::to_metadata`],
/// which renders sorted). An EMPTY slice reproduces the earlier encoding
/// byte for byte, which is what keeps every existing bag and every existing
/// byte-determinism oracle valid.
pub fn encode_channel(
    out: &mut Vec<u8>,
    id: u16,
    schema_id: u16,
    topic: &str,
    message_encoding: &str,
    metadata: &[(String, String)],
) {
    // MCAP `Map<string, string>`: a u32 BYTE length, then (string, string)
    // pairs, each string itself u32-length-prefixed.
    let map_bytes: usize = metadata
        .iter()
        .map(|(k, v)| 4 + k.len() + 4 + v.len())
        .sum();
    let content_len = (2 + 2 + 4 + topic.len() + 4 + message_encoding.len() + 4 + map_bytes) as u64;
    put_frame(out, op::CHANNEL, content_len);
    put_u16(out, id);
    put_u16(out, schema_id);
    put_str(out, topic);
    put_str(out, message_encoding);
    put_u32(out, map_bytes as u32);
    for (k, v) in metadata {
        put_str(out, k);
        put_str(out, v);
    }
}

/// Message Index record: `channel_id`, then an array of `(log_time, offset)`
/// tuples. `entries` are `(log_time, offset)` where `offset` is the byte
/// position of the message record WITHIN the uncompressed chunk body.
pub fn encode_message_index(out: &mut Vec<u8>, channel_id: u16, entries: &[(u64, u64)]) {
    let array_bytes = (entries.len() * 16) as u32; // 8 + 8 per entry
    let content_len = (2 + 4 + array_bytes as usize) as u64;
    put_frame(out, op::MESSAGE_INDEX, content_len);
    put_u16(out, channel_id);
    put_u32(out, array_bytes);
    for (log_time, offset) in entries {
        put_u64(out, *log_time);
        put_u64(out, *offset);
    }
}

/// One entry of a Chunk Index's `message_index_offsets` map:
/// `channel_id -> file offset of that channel's MessageIndex record`.
pub type MessageIndexOffset = (u16, u64);

/// Chunk Index record (summary section). `message_index_offsets` must be sorted
/// ascending by `channel_id` (MCAP maps are order-insensitive on read, but
/// sorting keeps the bytes deterministic).
#[allow(clippy::too_many_arguments)]
pub fn encode_chunk_index(
    out: &mut Vec<u8>,
    message_start_time: u64,
    message_end_time: u64,
    chunk_start_offset: u64,
    chunk_length: u64,
    message_index_offsets: &[MessageIndexOffset],
    message_index_length: u64,
    compressed_size: u64,
    uncompressed_size: u64,
) {
    // message_index_offsets map: u32 byte-length + (u16 + u64) per entry.
    let map_bytes = (message_index_offsets.len() * 10) as u32;
    // compression string is empty ("") → 4 bytes.
    let content_len = (8 + 8 + 8 + 8 + 4 + map_bytes as usize + 8 + 4 + 8 + 8) as u64;
    put_frame(out, op::CHUNK_INDEX, content_len);
    put_u64(out, message_start_time);
    put_u64(out, message_end_time);
    put_u64(out, chunk_start_offset);
    put_u64(out, chunk_length);
    put_u32(out, map_bytes);
    for (channel_id, offset) in message_index_offsets {
        put_u16(out, *channel_id);
        put_u64(out, *offset);
    }
    put_u64(out, message_index_length);
    put_str(out, ""); // compression = ""
    put_u64(out, compressed_size);
    put_u64(out, uncompressed_size);
}

/// Statistics record (summary section). `channel_message_counts` must be sorted
/// ascending by `channel_id`.
#[allow(clippy::too_many_arguments)]
pub fn encode_statistics(
    out: &mut Vec<u8>,
    message_count: u64,
    schema_count: u16,
    channel_count: u32,
    attachment_count: u32,
    metadata_count: u32,
    chunk_count: u32,
    message_start_time: u64,
    message_end_time: u64,
    channel_message_counts: &[(u16, u64)],
) {
    let map_bytes = (channel_message_counts.len() * 10) as u32; // u16 + u64
    let content_len = (8 + 2 + 4 + 4 + 4 + 4 + 8 + 8 + 4 + map_bytes as usize) as u64;
    put_frame(out, op::STATISTICS, content_len);
    put_u64(out, message_count);
    put_u16(out, schema_count);
    put_u32(out, channel_count);
    put_u32(out, attachment_count);
    put_u32(out, metadata_count);
    put_u32(out, chunk_count);
    put_u64(out, message_start_time);
    put_u64(out, message_end_time);
    put_u32(out, map_bytes);
    for (channel_id, count) in channel_message_counts {
        put_u16(out, *channel_id);
        put_u64(out, *count);
    }
}

/// Attachment Index record (summary section).
#[allow(clippy::too_many_arguments)]
pub fn encode_attachment_index(
    out: &mut Vec<u8>,
    offset: u64,
    length: u64,
    log_time: u64,
    create_time: u64,
    data_size: u64,
    name: &str,
    media_type: &str,
) {
    let content_len = (8 + 8 + 8 + 8 + 8 + 4 + name.len() + 4 + media_type.len()) as u64;
    put_frame(out, op::ATTACHMENT_INDEX, content_len);
    put_u64(out, offset);
    put_u64(out, length);
    put_u64(out, log_time);
    put_u64(out, create_time);
    put_u64(out, data_size);
    put_str(out, name);
    put_str(out, media_type);
}

/// Summary Offset record (summary-offset section).
pub fn encode_summary_offset(
    out: &mut Vec<u8>,
    group_opcode: u8,
    group_start: u64,
    group_length: u64,
) {
    put_frame(out, op::SUMMARY_OFFSET, 1 + 8 + 8);
    put_u8(out, group_opcode);
    put_u64(out, group_start);
    put_u64(out, group_length);
}

/// Data End record: `data_section_crc`.
pub fn encode_data_end(out: &mut Vec<u8>, data_section_crc: u32) {
    put_frame(out, op::DATA_END, 4);
    put_u32(out, data_section_crc);
}

/// Attachment record: `log_time`, `create_time`, `name`, `media_type`,
/// length-prefixed `data`, and a trailing `crc32` that covers everything from
/// `log_time` through the last data byte (i.e. all content except the CRC
/// field itself). Returns the number of framed bytes appended.
pub fn encode_attachment(
    out: &mut Vec<u8>,
    log_time: u64,
    create_time: u64,
    name: &str,
    media_type: &str,
    data: &[u8],
) -> usize {
    let content_len = (8 + 8 + 4 + name.len() + 4 + media_type.len() + 8 + data.len() + 4) as u64;
    let start = out.len();
    put_frame(out, op::ATTACHMENT, content_len);
    let content_start = out.len();
    put_u64(out, log_time);
    put_u64(out, create_time);
    put_str(out, name);
    put_str(out, media_type);
    put_u64(out, data.len() as u64); // data_size
    out.extend_from_slice(data);
    // CRC covers [log_time .. end of data] (everything appended since
    // content_start), NOT the opcode/length prefix and NOT the CRC field.
    let crc = crc32fast::hash(&out[content_start..]);
    put_u32(out, crc);
    out.len() - start
}

/// The fixed serialized size of a Message record's header (channel_id +
/// sequence + log_time + publish_time), excluding the payload.
pub const MESSAGE_HEADER_LEN: usize = 2 + 4 + 8 + 8;

/// Message record FRAME only (opcode + content-length + header fields). The
/// payload is NOT appended — it rides to `writev` as separate iovec(s). Returns
/// the number of frame bytes appended (always `9 + MESSAGE_HEADER_LEN`).
pub fn encode_message_frame(
    out: &mut Vec<u8>,
    channel_id: u16,
    sequence: u32,
    log_time: u64,
    publish_time: u64,
    payload_len: usize,
) -> usize {
    let content_len = (MESSAGE_HEADER_LEN + payload_len) as u64;
    let start = out.len();
    put_frame(out, op::MESSAGE, content_len);
    put_u16(out, channel_id);
    put_u32(out, sequence);
    put_u64(out, log_time);
    put_u64(out, publish_time);
    out.len() - start
}

/// The fixed serialized size of a Chunk record's header (message_start_time +
/// message_end_time + uncompressed_size + uncompressed_crc + empty compression
/// string + compressed_size), excluding the body.
pub const CHUNK_HEADER_LEN: usize = 8 + 8 + 8 + 4 + 4 + 8;

/// Chunk record header FRAME only (opcode + content-length + chunk header
/// fields). The chunk body (the framed message records + their payloads) is NOT
/// appended — it rides to `writev` after this frame. `compression` is always
/// the empty string and `compressed_size == uncompressed_size == body_len`
/// (this crate never compresses). Returns the number of frame bytes appended.
pub fn encode_chunk_header(
    out: &mut Vec<u8>,
    message_start_time: u64,
    message_end_time: u64,
    uncompressed_crc: u32,
    body_len: u64,
) {
    let content_len = (CHUNK_HEADER_LEN as u64) + body_len;
    put_frame(out, op::CHUNK, content_len);
    put_u64(out, message_start_time);
    put_u64(out, message_end_time);
    put_u64(out, body_len); // uncompressed_size
    put_u32(out, uncompressed_crc);
    put_str(out, ""); // compression = "" (uncompressed)
    put_u64(out, body_len); // compressed_size == uncompressed_size
}

/// Footer PREFIX: opcode + content-length + `summary_start` +
/// `summary_offset_start`. The `summary_crc` (u32) is appended by the caller
/// AFTER computing it over the summary section up to and including this prefix
/// (the footer CRC is self-referencing). The footer content length is always
/// 20 (8 + 8 + 4).
pub fn encode_footer_prefix(out: &mut Vec<u8>, summary_start: u64, summary_offset_start: u64) {
    put_frame(out, op::FOOTER, 8 + 8 + 4);
    put_u64(out, summary_start);
    put_u64(out, summary_offset_start);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn magic_is_the_spec_bytes() {
        assert_eq!(MAGIC, [0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A]);
    }

    #[test]
    fn header_byte_oracle() {
        let mut out = Vec::new();
        encode_header(&mut out, "cerulion", "lib");
        // opcode 0x01, len = 4+8 + 4+3 = 19, "cerulion"(8), "lib"(3)
        let mut expected = vec![0x01];
        expected.extend_from_slice(&19u64.to_le_bytes());
        expected.extend_from_slice(&8u32.to_le_bytes());
        expected.extend_from_slice(b"cerulion");
        expected.extend_from_slice(&3u32.to_le_bytes());
        expected.extend_from_slice(b"lib");
        assert_eq!(out, expected);
    }

    #[test]
    fn message_frame_len_and_reclen() {
        let mut out = Vec::new();
        let n = encode_message_frame(&mut out, 7, 42, 1000, 2000, 5);
        assert_eq!(n, 9 + MESSAGE_HEADER_LEN);
        // opcode
        assert_eq!(out[0], op::MESSAGE);
        // reclen = header(22) + payload(5) = 27
        assert_eq!(u64::from_le_bytes(out[1..9].try_into().unwrap()), 27);
        assert_eq!(u16::from_le_bytes(out[9..11].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(out[11..15].try_into().unwrap()), 42);
        assert_eq!(u64::from_le_bytes(out[15..23].try_into().unwrap()), 1000);
        assert_eq!(u64::from_le_bytes(out[23..31].try_into().unwrap()), 2000);
    }

    #[test]
    fn chunk_header_reclen_includes_body() {
        let mut out = Vec::new();
        encode_chunk_header(&mut out, 10, 20, 0xABCD, 100);
        assert_eq!(out[0], op::CHUNK);
        // reclen = CHUNK_HEADER_LEN(40) + body(100) = 140
        assert_eq!(u64::from_le_bytes(out[1..9].try_into().unwrap()), 140);
        assert_eq!(out.len(), 9 + CHUNK_HEADER_LEN);
    }

    #[test]
    fn attachment_crc_covers_content_not_frame() {
        let mut out = Vec::new();
        let n = encode_attachment(&mut out, 1, 2, "g", "text/plain", b"hi");
        assert_eq!(out.len(), n);
        assert_eq!(out[0], op::ATTACHMENT);
        let content_len = u64::from_le_bytes(out[1..9].try_into().unwrap()) as usize;
        assert_eq!(content_len, n - 9);
        // Recompute CRC over content minus the trailing 4-byte crc; must match.
        let content = &out[9..out.len() - 4];
        let stored = u32::from_le_bytes(out[out.len() - 4..].try_into().unwrap());
        assert_eq!(crc32fast::hash(content), stored);
    }

    #[test]
    fn footer_prefix_is_fixed_len() {
        let mut out = Vec::new();
        encode_footer_prefix(&mut out, 0x1111, 0x2222);
        assert_eq!(out[0], op::FOOTER);
        assert_eq!(u64::from_le_bytes(out[1..9].try_into().unwrap()), 20);
        // opcode + len + summary_start + summary_offset_start = 1 + 8 + 8 + 8
        assert_eq!(out.len(), 25);
    }
}
