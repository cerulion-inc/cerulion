// SPDX-License-Identifier: AGPL-3.0-only
//! Test support: rewrite a current bag as the release with the 32-byte wire
//! header would have written it, so an end-to-end test can drive a version 1
//! recording through `bag play`, `bag play --resim` and `bag migrate`
//! without that release's build.
//!
//! The rewrite is mechanical and independent of the reader under test: it
//! re-encodes every `cerulion` schema descriptor in the version 1 layout
//! (18 bytes, no wire header version), strips the step tag from every user
//! frame (`total_size` and an `offset_table_offset` past the header shrink by
//! 8; the body is copied), and copies reserved channels, channel metadata and
//! attachments unchanged. The container is rewritten by the upstream `mcap`
//! crate's writer.

use std::borrow::Cow;
use std::collections::BTreeMap;

use cerulion_core::wire::{WireHeader, V1_HEADER_SIZE};

use crate::schema::{SchemaDescriptor, RESERVED_PREFIX, SCHEMA_ENCODING};

/// The version 1 encoding of `d`: 18 bytes, recipe, hash and fixed size.
pub fn v1_descriptor_bytes(d: &SchemaDescriptor) -> Vec<u8> {
    let mut b = Vec::with_capacity(18);
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&d.hash_recipe.to_le_bytes());
    b.extend_from_slice(&d.schema_hash.to_le_bytes());
    b.extend_from_slice(&d.wire_fixed_size.to_le_bytes());
    b
}

/// One current frame in the version 1 layout: the step tag (bytes 32 to 40)
/// is dropped. Panics on a frame that carries a step tag, which version 1
/// cannot express, or that is not a whole frame.
pub fn downgrade_frame(frame: &[u8]) -> Vec<u8> {
    let h = WireHeader::read_from_buf(frame).expect("a whole current frame");
    assert_eq!(h.total_size as usize, frame.len(), "a whole current frame");
    assert!(h.step_tag.is_untagged(), "version 1 has no step tag");
    let grow = (WireHeader::SIZE - V1_HEADER_SIZE) as u32;
    let mut out = Vec::with_capacity(frame.len() - grow as usize);
    out.extend_from_slice(&frame[..8]);
    out.extend_from_slice(&(h.total_size - grow).to_le_bytes());
    let oto = if h.offset_table_offset as usize >= WireHeader::SIZE {
        h.offset_table_offset - grow
    } else {
        h.offset_table_offset
    };
    out.extend_from_slice(&oto.to_le_bytes());
    out.extend_from_slice(&frame[16..V1_HEADER_SIZE]);
    out.extend_from_slice(&frame[WireHeader::SIZE..]);
    out
}

/// Rewrite a finalized current bag as a version 1 bag (see the module docs).
pub fn downgrade_bag_to_v1(bytes: &[u8]) -> Vec<u8> {
    downgrade_bag_to_v1_editing(bytes, |_, _| {})
}

/// [`downgrade_bag_to_v1`], handing each downgraded user frame to `edit`
/// (with its topic) before it is written, so a test can damage one.
pub fn downgrade_bag_to_v1_editing(
    bytes: &[u8],
    mut edit: impl FnMut(&str, &mut Vec<u8>),
) -> Vec<u8> {
    let summary = mcap::Summary::read(bytes)
        .expect("a readable summary")
        .expect("a finalized bag");
    let mut out = std::io::Cursor::new(Vec::new());
    {
        let mut w = mcap::WriteOptions::new()
            .compression(None)
            .profile(crate::writer::PROFILE)
            .library("cerulion_bag")
            .create(&mut out)
            .expect("writer");
        let mut channel_ids: Vec<u16> = summary.channels.keys().copied().collect();
        channel_ids.sort_unstable();
        let mut new_id: BTreeMap<u16, (u16, bool)> = BTreeMap::new();
        for id in channel_ids {
            let ch = &summary.channels[&id];
            let mut user = false;
            let schema_id = match &ch.schema {
                Some(s) if s.encoding == SCHEMA_ENCODING => {
                    let d = SchemaDescriptor::decode(&s.data).expect("a cerulion descriptor");
                    user = !ch.topic.starts_with(RESERVED_PREFIX);
                    w.add_schema(&s.name, &s.encoding, &v1_descriptor_bytes(&d))
                        .expect("schema")
                }
                Some(s) => w.add_schema(&s.name, &s.encoding, &s.data).expect("schema"),
                None => 0,
            };
            let cid = w
                .add_channel(schema_id, &ch.topic, &ch.message_encoding, &ch.metadata)
                .expect("channel");
            new_id.insert(id, (cid, user));
        }
        for m in mcap::MessageStream::new(bytes).expect("message stream") {
            let m = m.expect("a whole message");
            let (cid, user) = new_id[&m.channel.id];
            let data: Cow<'_, [u8]> = if user {
                let mut f = downgrade_frame(&m.data);
                edit(&m.channel.topic, &mut f);
                Cow::Owned(f)
            } else {
                m.data
            };
            w.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: cid,
                    sequence: m.sequence,
                    log_time: m.log_time,
                    publish_time: m.publish_time,
                },
                &data,
            )
            .expect("message");
        }
        for index in &summary.attachment_indexes {
            let a = mcap::read::attachment(bytes, index).expect("attachment");
            w.attach(&a).expect("attach");
        }
        w.finish().expect("finish");
    }
    out.into_inner()
}
