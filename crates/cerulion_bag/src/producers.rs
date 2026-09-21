// SPDX-License-Identifier: AGPL-3.0-only
//! The record-time PRODUCER-LABEL record carried on the
//! [`FRAME_PRODUCERS_TOPIC`](crate::FRAME_PRODUCERS_TOPIC) reserved channel.
//!
//! # Why the labels have to be recorded
//!
//! A `multi_publisher_topics` topic is written by several publishers onto one
//! iceoryx2 service, and the 32-byte wire header's `sequence` is a
//! PER-PUBLISHER commit counter — so a bag's user channel holds an interleaving
//! no reader can attribute after the fact. The recorder CAN see it (iceoryx2
//! reports the sample's `UniquePublisherId` at drain time), so the attribution
//! is written down where it is still knowable rather than guessed at on replay.
//!
//! # Wire format
//!
//! ONE fixed-size, versioned, little-endian record — both kinds share the
//! layout, so a reader frames the stream without first knowing the kind:
//!
//! ```text
//! offset  size  field           type   value
//! 0       1     version         u8     = 1  (PRODUCER_RECORD_VERSION)
//! 1       1     kind            u8     0 = FrameLabel, 1 = CatchUpPrefix
//! 2       2     channel_id      u16    the DATA channel this record annotates
//! 4       8     ordinal         u64    see below — meaning depends on `kind`
//! 12      16    publisher_id    u128   iceoryx2 `UniquePublisherId::value()`
//! ```
//!
//! Total: [`PRODUCER_RECORD_SIZE`] (28) bytes.
//!
//! `ordinal` carries the one quantity whose meaning is kind-dependent, and the
//! two readings are deliberately not two fields: a `FrameLabel` names ONE frame
//! by its 0-based index in that channel's stream of frames written to THIS bag
//! (file order — not the wire `sequence`, which belongs to a publisher rather
//! than to the bag), while a `CatchUpPrefix` names a COUNT `k`, meaning the
//! ordinals `[0, k)` on that channel all belong to `publisher_id`. The prefix
//! form exists because a recorder that attaches to a single-publisher stretch
//! can label it in one record instead of one per frame.
//!
//! [`ProducerRecord::decode`] REFUSES anything it cannot read exactly — a wrong
//! length, an unknown version, an unknown kind — naming the offending value.
//! Silently accepting one would put a fabricated attribution on a replayed
//! frame, which is worse than having none.

use crate::error::{BagError, BagResult};

/// The current [`ProducerRecord`] wire version.
pub const PRODUCER_RECORD_VERSION: u8 = 1;

/// The exact byte length of an encoded [`ProducerRecord`].
pub const PRODUCER_RECORD_SIZE: usize = 28;

/// The kind of attribution a [`ProducerRecord`] carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerRecordKind {
    /// Label ONE frame: `ordinal` is that frame's 0-based index in its
    /// channel's stream of frames written to this bag.
    FrameLabel,
    /// Label a leading RUN of frames: `ordinal` is a count `k`, and the
    /// ordinals `[0, k)` on that channel all belong to this publisher.
    CatchUpPrefix,
}

impl ProducerRecordKind {
    /// The wire discriminant.
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::FrameLabel => 0,
            Self::CatchUpPrefix => 1,
        }
    }

    /// Decode a wire discriminant, or `None` for a value this build does not
    /// know (the caller turns that into the loud [`BagError::ProducerRecord`]).
    pub const fn from_u8(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::FrameLabel),
            1 => Some(Self::CatchUpPrefix),
            _ => None,
        }
    }
}

/// What a [`ProducerRecord`] attributes, and to how much.
///
/// The wire keeps ONE `ordinal` word whose meaning depends on `kind` (see the
/// module docs), because both readings are a single `u64` and framing the stream
/// must not require knowing the kind first. In Rust the two readings are
/// SEPARATE FIELDS of separate variants, so a caller cannot build a record whose
/// number means one thing and whose kind says the other — the mistake the shared
/// wire word makes easy and which no decoder could catch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerAttribution {
    /// Label ONE frame: its 0-based index in its channel's stream of frames
    /// written to this bag (file order — not the wire `sequence`, which belongs
    /// to a publisher rather than to the bag).
    FrameLabel {
        /// The frame's 0-based index in this channel's stream.
        frame_index: u64,
    },
    /// Label a leading RUN of frames: the ordinals `[0, prefix_len)` on this
    /// channel all belong to this publisher.
    ///
    /// A decoded `prefix_len` of `0` attributes NOTHING — the empty range — and
    /// a reader must treat it that way rather than as a claim about frame 0.
    CatchUpPrefix {
        /// How many leading frames of this channel the publisher wrote.
        prefix_len: u64,
    },
}

impl ProducerAttribution {
    /// The wire `kind` discriminant this attribution encodes as.
    const fn kind(self) -> ProducerRecordKind {
        match self {
            Self::FrameLabel { .. } => ProducerRecordKind::FrameLabel,
            Self::CatchUpPrefix { .. } => ProducerRecordKind::CatchUpPrefix,
        }
    }

    /// The wire `ordinal` word this attribution encodes as.
    const fn ordinal(self) -> u64 {
        match self {
            Self::FrameLabel { frame_index } => frame_index,
            Self::CatchUpPrefix { prefix_len } => prefix_len,
        }
    }

    /// Rebuild from a decoded `(kind, ordinal)` pair.
    const fn from_wire(kind: ProducerRecordKind, ordinal: u64) -> Self {
        match kind {
            ProducerRecordKind::FrameLabel => Self::FrameLabel {
                frame_index: ordinal,
            },
            ProducerRecordKind::CatchUpPrefix => Self::CatchUpPrefix {
                prefix_len: ordinal,
            },
        }
    }
}

/// One decoded producer label (see the module docs for the byte layout).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProducerRecord {
    /// What this record attributes, and to how much.
    pub attribution: ProducerAttribution,
    /// The DATA channel this record annotates.
    pub channel_id: u16,
    /// The producing publisher's `iceoryx2` `UniquePublisherId::value()`.
    pub publisher_id: u128,
}

impl ProducerRecord {
    /// Encode to the fixed [`PRODUCER_RECORD_SIZE`]-byte little-endian record.
    pub fn encode(&self) -> [u8; PRODUCER_RECORD_SIZE] {
        let mut b = [0u8; PRODUCER_RECORD_SIZE];
        b[0] = PRODUCER_RECORD_VERSION;
        b[1] = self.attribution.kind().as_u8();
        b[2..4].copy_from_slice(&self.channel_id.to_le_bytes());
        b[4..12].copy_from_slice(&self.attribution.ordinal().to_le_bytes());
        b[12..28].copy_from_slice(&self.publisher_id.to_le_bytes());
        b
    }

    /// Decode one record. Rejects a wrong length, an unknown
    /// [`PRODUCER_RECORD_VERSION`], and an unknown
    /// [`ProducerRecordKind`] discriminant — each naming the offending value.
    pub fn decode(data: &[u8]) -> BagResult<Self> {
        if data.len() != PRODUCER_RECORD_SIZE {
            return Err(BagError::ProducerRecord {
                reason: format!(
                    "record is {} bytes, expected exactly {PRODUCER_RECORD_SIZE}",
                    data.len()
                ),
            });
        }
        let version = data[0];
        if version != PRODUCER_RECORD_VERSION {
            return Err(BagError::ProducerRecord {
                reason: format!(
                    "unknown version {version} (this build writes/reads v{PRODUCER_RECORD_VERSION})"
                ),
            });
        }
        let kind_byte = data[1];
        let Some(kind) = ProducerRecordKind::from_u8(kind_byte) else {
            return Err(BagError::ProducerRecord {
                reason: format!(
                    "unknown kind {kind_byte} (this build reads 0 = FrameLabel, 1 = CatchUpPrefix)"
                ),
            });
        };
        // The three `try_into`s are INFALLIBLE — the length was checked above, so
        // every span is in bounds and each is exactly its target's width. House
        // style is `expect` with the reason, so a future edit that moves a span
        // fails with the invariant it broke rather than a bare `Option::unwrap`.
        Ok(Self {
            attribution: ProducerAttribution::from_wire(
                kind,
                u64::from_le_bytes(
                    data[4..12]
                        .try_into()
                        .expect("bytes 4..12 of a length-checked record are 8 bytes"),
                ),
            ),
            channel_id: u16::from_le_bytes(
                data[2..4]
                    .try_into()
                    .expect("bytes 2..4 of a length-checked record are 2 bytes"),
            ),
            publisher_id: u128::from_le_bytes(
                data[12..28]
                    .try_into()
                    .expect("bytes 12..28 of a length-checked record are 16 bytes"),
            ),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-written byte oracle for a `FrameLabel`. Every field carries a
    /// byte-recognisable value so a swapped or mis-sized field is visible in the
    /// diff rather than hidden by a zero.
    #[test]
    fn frame_label_byte_oracle() {
        let r = ProducerRecord {
            attribution: ProducerAttribution::FrameLabel {
                frame_index: 0x1112_1314_1516_1718,
            },
            channel_id: 0x0201,
            publisher_id: 0x2122_2324_2526_2728_2930_3132_3334_3536,
        };
        let expected: [u8; PRODUCER_RECORD_SIZE] = [
            0x01, // version = 1
            0x00, // kind = FrameLabel
            0x01, 0x02, // channel_id LE
            0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, // ordinal LE
            0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x29, // publisher_id LE
            0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,
        ];
        assert_eq!(r.encode(), expected, "record byte layout is a contract");
    }

    /// The same oracle for a `CatchUpPrefix` — only the kind byte differs, which
    /// is what pins that the two kinds SHARE one layout.
    #[test]
    fn catch_up_prefix_byte_oracle() {
        let r = ProducerRecord {
            attribution: ProducerAttribution::CatchUpPrefix {
                prefix_len: 0x1112_1314_1516_1718,
            },
            channel_id: 0x0201,
            publisher_id: 0x2122_2324_2526_2728_2930_3132_3334_3536,
        };
        let expected: [u8; PRODUCER_RECORD_SIZE] = [
            0x01, // version = 1
            0x01, // kind = CatchUpPrefix
            0x01, 0x02, // channel_id LE
            0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, // ordinal LE
            0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x29, // publisher_id LE
            0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,
        ];
        assert_eq!(r.encode(), expected, "record byte layout is a contract");
    }

    #[test]
    fn roundtrip_is_identity_for_both_kinds_and_the_edges() {
        for ordinal_as in [
            (|n| ProducerAttribution::FrameLabel { frame_index: n }) as fn(u64) -> _,
            (|n| ProducerAttribution::CatchUpPrefix { prefix_len: n }) as fn(u64) -> _,
        ] {
            for (ordinal, publisher_id) in [
                (0u64, 0u128),
                (1, 1),
                (u64::MAX, u128::MAX),
                (12_345, 0xDEAD_BEEF_CAFE_F00D),
            ] {
                let r = ProducerRecord {
                    attribution: ordinal_as(ordinal),
                    channel_id: 7,
                    publisher_id,
                };
                assert_eq!(ProducerRecord::decode(&r.encode()).unwrap(), r);
            }
        }
    }

    #[test]
    fn decode_rejects_a_wrong_length() {
        for len in [0usize, 27, 29, 56] {
            let e = ProducerRecord::decode(&vec![0u8; len]).expect_err("wrong length");
            let msg = e.to_string();
            assert!(
                matches!(e, BagError::ProducerRecord { .. }),
                "got {e:?} for len {len}"
            );
            assert!(
                msg.contains(&len.to_string()) && msg.contains("28"),
                "the refusal must name both lengths, got {msg:?}"
            );
        }
    }

    #[test]
    fn decode_rejects_an_unknown_version() {
        let mut bad = ProducerRecord {
            attribution: ProducerAttribution::FrameLabel { frame_index: 2 },
            channel_id: 1,
            publisher_id: 3,
        }
        .encode();
        bad[0] = 99;
        let e = ProducerRecord::decode(&bad).expect_err("unknown version");
        assert!(matches!(e, BagError::ProducerRecord { .. }), "got {e:?}");
        assert!(
            e.to_string().contains("99"),
            "the refusal must name the offending version, got {e}"
        );
    }

    #[test]
    fn decode_rejects_an_unknown_kind() {
        let mut bad = ProducerRecord {
            attribution: ProducerAttribution::FrameLabel { frame_index: 2 },
            channel_id: 1,
            publisher_id: 3,
        }
        .encode();
        bad[1] = 7;
        let e = ProducerRecord::decode(&bad).expect_err("unknown kind");
        assert!(matches!(e, BagError::ProducerRecord { .. }), "got {e:?}");
        assert!(
            e.to_string().contains('7'),
            "the refusal must name the offending kind, got {e}"
        );
    }

    /// The two discriminants are a WIRE contract: renumbering them silently
    /// re-attributes every already-recorded label.
    #[test]
    fn kind_discriminants_are_pinned() {
        assert_eq!(ProducerRecordKind::FrameLabel.as_u8(), 0);
        assert_eq!(ProducerRecordKind::CatchUpPrefix.as_u8(), 1);
        assert_eq!(
            ProducerRecordKind::from_u8(0),
            Some(ProducerRecordKind::FrameLabel)
        );
        assert_eq!(
            ProducerRecordKind::from_u8(1),
            Some(ProducerRecordKind::CatchUpPrefix)
        );
        assert_eq!(ProducerRecordKind::from_u8(2), None);
    }
}
