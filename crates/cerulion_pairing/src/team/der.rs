// SPDX-License-Identifier: MIT OR Apache-2.0
//! A minimal DER writer and a strict DER reader for the fixed team certificate
//! profile. Only the universal types that profile uses are covered.
//!
//! The writer always emits the shortest length form, so the same inputs give
//! the same bytes. The reader refuses indefinite lengths, non-minimal lengths
//! and trailing bytes, and every read is bounds checked.

use crate::PairingError;

pub(crate) const TAG_BOOLEAN: u8 = 0x01;
pub(crate) const TAG_INTEGER: u8 = 0x02;
pub(crate) const TAG_BIT_STRING: u8 = 0x03;
pub(crate) const TAG_OCTET_STRING: u8 = 0x04;
pub(crate) const TAG_OID: u8 = 0x06;
pub(crate) const TAG_UTF8_STRING: u8 = 0x0C;
pub(crate) const TAG_PRINTABLE_STRING: u8 = 0x13;
pub(crate) const TAG_UTC_TIME: u8 = 0x17;
pub(crate) const TAG_GENERALIZED_TIME: u8 = 0x18;
pub(crate) const TAG_SEQUENCE: u8 = 0x30;
pub(crate) const TAG_SET: u8 = 0x31;
/// `[0] EXPLICIT`, constructed (the certificate version).
pub(crate) const TAG_CTX0_CONSTRUCTED: u8 = 0xA0;
/// `[3] EXPLICIT`, constructed (the extensions).
pub(crate) const TAG_CTX3_CONSTRUCTED: u8 = 0xA3;
/// `[6] IMPLICIT IA5String`, primitive (a `uniformResourceIdentifier` name).
pub(crate) const TAG_CTX6_PRIMITIVE: u8 = 0x86;

/// Content octets of the OIDs the profile uses.
pub(crate) mod oid {
    /// 1.3.101.112, id-Ed25519 (RFC 8410).
    pub(crate) const ED25519: &[u8] = &[0x2B, 0x65, 0x70];
    /// 2.5.4.3, id-at-commonName.
    pub(crate) const COMMON_NAME: &[u8] = &[0x55, 0x04, 0x03];
    /// 2.5.29.15, id-ce-keyUsage.
    pub(crate) const KEY_USAGE: &[u8] = &[0x55, 0x1D, 0x0F];
    /// 2.5.29.17, id-ce-subjectAltName.
    pub(crate) const SUBJECT_ALT_NAME: &[u8] = &[0x55, 0x1D, 0x11];
    /// 2.5.29.19, id-ce-basicConstraints.
    pub(crate) const BASIC_CONSTRAINTS: &[u8] = &[0x55, 0x1D, 0x13];
    /// 2.5.29.37, id-ce-extKeyUsage.
    pub(crate) const EXT_KEY_USAGE: &[u8] = &[0x55, 0x1D, 0x25];
    /// 1.3.6.1.5.5.7.3.1, id-kp-serverAuth.
    pub(crate) const KP_SERVER_AUTH: &[u8] = &[0x2B, 0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x01];
    /// 1.3.6.1.5.5.7.3.2, id-kp-clientAuth.
    pub(crate) const KP_CLIENT_AUTH: &[u8] = &[0x2B, 0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x02];
}

/// Append one TLV: `tag`, the shortest length form, then `content`.
pub(crate) fn tlv(out: &mut Vec<u8>, tag: u8, content: &[u8]) {
    out.push(tag);
    let len = content.len();
    if len < 0x80 {
        out.push(len as u8);
    } else {
        let bytes = len.to_be_bytes();
        let skip = bytes.iter().take_while(|b| **b == 0).count();
        let significant = &bytes[skip..];
        out.push(0x80 | significant.len() as u8);
        out.extend_from_slice(significant);
    }
    out.extend_from_slice(content);
}

/// One TLV as a fresh buffer.
pub(crate) fn tlv_vec(tag: u8, content: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(content.len() + 4);
    tlv(&mut out, tag, content);
    out
}

/// A constructed value whose content is the concatenation of `parts`.
pub(crate) fn constructed(tag: u8, parts: &[&[u8]]) -> Vec<u8> {
    let content: Vec<u8> = parts.concat();
    tlv_vec(tag, &content)
}

/// One element read by [`Reader::read`].
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tlv<'a> {
    pub(crate) tag: u8,
    pub(crate) content: &'a [u8],
    /// The whole element, header included (what a signature covers).
    pub(crate) raw: &'a [u8],
}

/// A cursor over DER bytes. Every malformed input maps to
/// [`PairingError::TeamCertMalformed`].
pub(crate) struct Reader<'a> {
    rest: &'a [u8],
}

fn malformed(what: &'static str) -> PairingError {
    PairingError::TeamCertMalformed(what)
}

impl<'a> Reader<'a> {
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Reader { rest: bytes }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.rest.is_empty()
    }

    /// The next tag without consuming it.
    pub(crate) fn peek_tag(&self) -> Option<u8> {
        self.rest.first().copied()
    }

    /// Read the next element of any tag.
    pub(crate) fn read(&mut self) -> Result<Tlv<'a>, PairingError> {
        let all = self.rest;
        let (&tag, after_tag) = all.split_first().ok_or(malformed("truncated tag"))?;
        if tag & 0x1F == 0x1F {
            return Err(malformed("multi-byte tags are not part of the profile"));
        }
        let (&first, mut after_len) = after_tag
            .split_first()
            .ok_or(malformed("truncated length"))?;
        let len = if first < 0x80 {
            first as usize
        } else if first == 0x80 {
            return Err(malformed("indefinite length"));
        } else {
            let n = (first & 0x7F) as usize;
            if n > core::mem::size_of::<usize>() || n > 4 {
                return Err(malformed("length too large"));
            }
            if after_len.len() < n {
                return Err(malformed("truncated length"));
            }
            let (len_bytes, tail) = after_len.split_at(n);
            if len_bytes[0] == 0 {
                return Err(malformed("non-minimal length"));
            }
            let mut v = 0usize;
            for b in len_bytes {
                v = (v << 8) | *b as usize;
            }
            if v < 0x80 {
                return Err(malformed("non-minimal length"));
            }
            after_len = tail;
            v
        };
        if after_len.len() < len {
            return Err(malformed("truncated content"));
        }
        let (content, tail) = after_len.split_at(len);
        let header = all.len() - after_len.len();
        self.rest = tail;
        Ok(Tlv {
            tag,
            content,
            raw: &all[..header + len],
        })
    }

    /// Read the next element and require `tag`.
    pub(crate) fn expect(&mut self, tag: u8, what: &'static str) -> Result<Tlv<'a>, PairingError> {
        let t = self.read()?;
        if t.tag != tag {
            return Err(malformed(what));
        }
        Ok(t)
    }

    /// Require that nothing is left.
    pub(crate) fn finish(&self, what: &'static str) -> Result<(), PairingError> {
        if self.rest.is_empty() {
            Ok(())
        } else {
            Err(malformed(what))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lengths_use_the_shortest_form() {
        assert_eq!(tlv_vec(0x04, &[]), [0x04, 0x00]);
        assert_eq!(tlv_vec(0x04, &[7; 0x7F])[..2], [0x04, 0x7F]);
        assert_eq!(tlv_vec(0x04, &[7; 0x80])[..3], [0x04, 0x81, 0x80]);
        assert_eq!(tlv_vec(0x04, &[7; 0x100])[..4], [0x04, 0x82, 0x01, 0x00]);
    }

    #[test]
    fn reader_round_trips_and_reports_raw() {
        let a = tlv_vec(0x04, &[1, 2, 3]);
        let b = tlv_vec(0x02, &[5; 200]);
        let both = [a.clone(), b.clone()].concat();
        let mut r = Reader::new(&both);
        let t = r.read().unwrap();
        assert_eq!((t.tag, t.content, t.raw), (0x04, &[1u8, 2, 3][..], &a[..]));
        let t = r.read().unwrap();
        assert_eq!((t.tag, t.content.len(), t.raw), (0x02, 200, &b[..]));
        assert!(r.is_empty());
    }

    #[test]
    fn reader_refuses_malformed_lengths() {
        for bad in [
            &[0x04][..],                  // no length
            &[0x04, 0x80],                // indefinite
            &[0x04, 0x81, 0x05, 0, 0],    // long form for a short length
            &[0x04, 0x82, 0x00, 0x80],    // leading zero length byte
            &[0x04, 0x03, 0x00],          // content shorter than the length
            &[0x1F, 0x01, 0x00],          // multi-byte tag
            &[0x04, 0x85, 1, 1, 1, 1, 1], // five length bytes
        ] {
            assert!(
                matches!(
                    Reader::new(bad).read(),
                    Err(PairingError::TeamCertMalformed(_))
                ),
                "{bad:02x?}"
            );
        }
    }
}
