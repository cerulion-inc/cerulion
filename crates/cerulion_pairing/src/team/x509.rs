// SPDX-License-Identifier: MIT OR Apache-2.0
//! The X.509 profile for team leaves and team issuing CAs: issuance with a
//! fixed, deterministic DER layout, and a strict profile checker.
//!
//! Every certificate is X.509 v3, ed25519 (RFC 8410) for both the key and the
//! signature, with a single common name in the subject and the issuer.
//!
//! A leaf carries, in this order: basicConstraints (critical, not a CA),
//! keyUsage (critical, digitalSignature), extKeyUsage (serverAuth and
//! clientAuth, always written), and subjectAltName (the device URI, then the
//! account URI when there is one). A team CA carries basicConstraints
//! (critical, CA, pathLenConstraint 0) and keyUsage (critical, keyCertSign and
//! cRLSign).
//!
//! The serial number is 16 bytes of SHA-256 over the subject name, the subject
//! key, the issuer name and notBefore, so the same inputs give the same bytes.

use ed25519_dalek::SigningKey;
use sha2::{Digest, Sha256};

use super::der::{self, oid, Reader, Tlv};
use super::{
    account_uri, ca_common_name, common_name, device_uri, parse_ca_common_name, parse_common_name,
    uri_bytes32, TeamRole, TeamShort,
};
use crate::crypto::{self, VerifyResult};
use crate::format::{AccountId, PublicKey, Signature};
use crate::{CertKind, PairingError};

/// notBefore is written this many seconds before the issuance time.
pub const BACKDATE_SECS: u64 = 3600;
/// Owner, operator and viewer leaf lifetime from issuance: 7 days.
pub const DESK_LIFETIME_SECS: u64 = 7 * 24 * 3600;
/// Continuous integration leaf lifetime from issuance: 24 hours.
pub const CI_LIFETIME_SECS: u64 = 24 * 3600;
/// `99991231235959Z` as Unix seconds: the RFC 5280 notAfter for a
/// certificate with no well defined expiration. Robot leaves and team CAs
/// carry it.
pub const NO_EXPIRY_UNIX_SECS: u64 = 253_402_300_799;

const SERIAL_DOMAIN: &[u8] = b"cerulion-pairing:team-cert-serial:v1";
const NANOS_PER_SEC: u64 = 1_000_000_000;

/// Inputs for one machine leaf.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LeafParams {
    /// The team short name.
    pub team: TeamShort,
    /// The team generation the issuing CA belongs to.
    pub generation: u32,
    /// The role the leaf grants.
    pub role: TeamRole,
    /// The machine's ed25519 public key (the leaf's subject key).
    pub device_key: PublicKey,
    /// The account the machine was enrolled under, written as a SAN URI.
    pub account: Option<AccountId>,
    /// Issuance time, Unix nanoseconds. Floored to whole seconds.
    pub issued_at_ns: u64,
}

/// What [`check_leaf`] read from a leaf that satisfies the profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LeafFacts {
    /// The role named in the common name.
    pub role: TeamRole,
    /// The generation named in the common name.
    pub generation: u32,
    /// The team named in the common name.
    pub team: TeamShort,
    /// The subject ed25519 public key.
    pub device_key: PublicKey,
    /// The account SAN URI, when present.
    pub account: Option<AccountId>,
    /// notBefore, Unix seconds.
    pub not_before_unix_secs: u64,
    /// notAfter, Unix seconds ([`NO_EXPIRY_UNIX_SECS`] for a robot).
    pub not_after_unix_secs: u64,
    tbs: Vec<u8>,
    signature: Signature,
}

/// What [`check_team_ca`] read from a team issuing CA certificate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TeamCaFacts {
    /// The generation named in the common name.
    pub generation: u32,
    /// The team named in the common name.
    pub team: TeamShort,
    /// The CA's ed25519 public key.
    pub ca_key: PublicKey,
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

/// Days since 1970-01-01 to (year, month, day), proleptic Gregorian.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// (year, month, day) to days since 1970-01-01, proleptic Gregorian.
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let m = m as i64;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// RFC 5280 section 4.1.2.5: UTCTime through 2049, GeneralizedTime from 2050.
fn encode_time(unix_secs: u64) -> Result<Vec<u8>, PairingError> {
    if unix_secs > NO_EXPIRY_UNIX_SECS {
        return Err(PairingError::TeamCertTimeOutOfRange);
    }
    let days = (unix_secs / 86_400) as i64;
    let rem = unix_secs % 86_400;
    let (y, mo, d) = civil_from_days(days);
    let (h, mi, s) = (rem / 3600, (rem / 60) % 60, rem % 60);
    let body = if y < 2050 {
        format!("{:02}{mo:02}{d:02}{h:02}{mi:02}{s:02}Z", y % 100)
    } else {
        format!("{y:04}{mo:02}{d:02}{h:02}{mi:02}{s:02}Z")
    };
    let tag = if y < 2050 {
        der::TAG_UTC_TIME
    } else {
        der::TAG_GENERALIZED_TIME
    };
    Ok(der::tlv_vec(tag, body.as_bytes()))
}

fn decode_time(t: &Tlv<'_>) -> Result<u64, PairingError> {
    let bad = || PairingError::TeamCertMalformed("time");
    let s = core::str::from_utf8(t.content).map_err(|_| bad())?;
    let digits = s.strip_suffix('Z').ok_or_else(bad)?;
    if !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(bad());
    }
    let num = |r: core::ops::Range<usize>| -> Result<i64, PairingError> {
        digits.get(r).ok_or_else(bad)?.parse().map_err(|_| bad())
    };
    let (year, rest) = match (t.tag, digits.len()) {
        (der::TAG_UTC_TIME, 12) => {
            let yy = num(0..2)?;
            (if yy < 50 { 2000 + yy } else { 1900 + yy }, 2)
        }
        (der::TAG_GENERALIZED_TIME, 14) => (num(0..4)?, 4),
        _ => return Err(bad()),
    };
    let mo = num(rest..rest + 2)? as u32;
    let d = num(rest + 2..rest + 4)? as u32;
    let h = num(rest + 4..rest + 6)?;
    let mi = num(rest + 6..rest + 8)?;
    let sec = num(rest + 8..rest + 10)?;
    let days_in_month = match mo {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if (year % 4 == 0 && year % 100 != 0) || year % 400 == 0 => 29,
        2 => 28,
        _ => return Err(bad()),
    };
    if d == 0 || d > days_in_month || h > 23 || mi > 59 || sec > 59 || year < 1970 {
        return Err(bad());
    }
    let secs = days_from_civil(year, mo, d) * 86_400 + h * 3600 + mi * 60 + sec;
    u64::try_from(secs).map_err(|_| bad())
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

fn alg_ed25519() -> Vec<u8> {
    der::constructed(
        der::TAG_SEQUENCE,
        &[&der::tlv_vec(der::TAG_OID, oid::ED25519)],
    )
}

fn name(cn: &str) -> Vec<u8> {
    let atv = der::constructed(
        der::TAG_SEQUENCE,
        &[
            &der::tlv_vec(der::TAG_OID, oid::COMMON_NAME),
            &der::tlv_vec(der::TAG_UTF8_STRING, cn.as_bytes()),
        ],
    );
    let rdn = der::tlv_vec(der::TAG_SET, &atv);
    der::tlv_vec(der::TAG_SEQUENCE, &rdn)
}

fn spki(key: &PublicKey) -> Vec<u8> {
    let mut bits = Vec::with_capacity(33);
    bits.push(0);
    bits.extend_from_slice(&key.0);
    der::constructed(
        der::TAG_SEQUENCE,
        &[&alg_ed25519(), &der::tlv_vec(der::TAG_BIT_STRING, &bits)],
    )
}

fn extension(ext_oid: &[u8], critical: bool, value: &[u8]) -> Vec<u8> {
    let mut parts = vec![der::tlv_vec(der::TAG_OID, ext_oid)];
    if critical {
        parts.push(der::tlv_vec(der::TAG_BOOLEAN, &[0xFF]));
    }
    parts.push(der::tlv_vec(der::TAG_OCTET_STRING, value));
    let refs: Vec<&[u8]> = parts.iter().map(Vec::as_slice).collect();
    der::constructed(der::TAG_SEQUENCE, &refs)
}

fn serial(subject_cn: &str, key: &PublicKey, issuer_cn: &str, not_before: u64) -> [u8; 16] {
    let mut h = Sha256::new();
    for part in [
        SERIAL_DOMAIN,
        subject_cn.as_bytes(),
        &key.0,
        issuer_cn.as_bytes(),
    ] {
        h.update((part.len() as u32).to_le_bytes());
        h.update(part);
    }
    h.update(not_before.to_le_bytes());
    let digest: [u8; 32] = h.finalize().into();
    let mut s = [0u8; 16];
    s.copy_from_slice(&digest[..16]);
    // Positive, and the high byte nonzero, so the INTEGER is always 16 bytes.
    s[0] = (s[0] & 0x7F) | 0x40;
    s
}

struct CertInputs<'a> {
    subject_cn: &'a str,
    issuer_cn: &'a str,
    subject_key: &'a PublicKey,
    not_before: u64,
    not_after: u64,
    extensions: Vec<Vec<u8>>,
}

fn build_cert(inputs: CertInputs<'_>, signer: &SigningKey) -> Result<Vec<u8>, PairingError> {
    let version = der::tlv_vec(
        der::TAG_CTX0_CONSTRUCTED,
        &der::tlv_vec(der::TAG_INTEGER, &[2]),
    );
    let serial = der::tlv_vec(
        der::TAG_INTEGER,
        &serial(
            inputs.subject_cn,
            inputs.subject_key,
            inputs.issuer_cn,
            inputs.not_before,
        ),
    );
    let validity = der::constructed(
        der::TAG_SEQUENCE,
        &[
            &encode_time(inputs.not_before)?,
            &encode_time(inputs.not_after)?,
        ],
    );
    let ext_refs: Vec<&[u8]> = inputs.extensions.iter().map(Vec::as_slice).collect();
    let exts = der::tlv_vec(
        der::TAG_CTX3_CONSTRUCTED,
        &der::constructed(der::TAG_SEQUENCE, &ext_refs),
    );
    let tbs = der::constructed(
        der::TAG_SEQUENCE,
        &[
            &version,
            &serial,
            &alg_ed25519(),
            &name(inputs.issuer_cn),
            &validity,
            &name(inputs.subject_cn),
            &spki(inputs.subject_key),
            &exts,
        ],
    );
    let sig = crypto::ed25519_sign(signer, &tbs);
    let mut sig_bits = Vec::with_capacity(65);
    sig_bits.push(0);
    sig_bits.extend_from_slice(&sig.0);
    Ok(der::constructed(
        der::TAG_SEQUENCE,
        &[
            &tbs,
            &alg_ed25519(),
            &der::tlv_vec(der::TAG_BIT_STRING, &sig_bits),
        ],
    ))
}

/// Issue a machine leaf signed by the team CA key `ca_key` of
/// `params.generation`. Returns the certificate DER.
///
/// notBefore is the issuance second minus [`BACKDATE_SECS`]; notAfter is the
/// issuance second plus the role lifetime, or [`NO_EXPIRY_UNIX_SECS`] for a
/// robot.
pub fn issue_leaf(params: &LeafParams, ca_key: &SigningKey) -> Result<Vec<u8>, PairingError> {
    // `u64::MAX` ns is 18_446_744_073 s (year 2554); adding a lifetime stays in range.
    let issued = params.issued_at_ns / NANOS_PER_SEC;
    let not_before = issued.saturating_sub(BACKDATE_SECS);
    let not_after = match params.role.lifetime_secs() {
        Some(life) => issued + life,
        None => NO_EXPIRY_UNIX_SECS,
    };
    let subject_cn = common_name(params.role, params.generation, &params.team);
    let issuer_cn = ca_common_name(params.generation, &params.team);

    let mut san = der::tlv_vec(
        der::TAG_CTX6_PRIMITIVE,
        device_uri(&params.device_key).as_bytes(),
    );
    if let Some(account) = &params.account {
        der::tlv(
            &mut san,
            der::TAG_CTX6_PRIMITIVE,
            account_uri(account).as_bytes(),
        );
    }
    let eku = der::constructed(
        der::TAG_SEQUENCE,
        &[
            &der::tlv_vec(der::TAG_OID, oid::KP_SERVER_AUTH),
            &der::tlv_vec(der::TAG_OID, oid::KP_CLIENT_AUTH),
        ],
    );
    let extensions = vec![
        extension(
            oid::BASIC_CONSTRAINTS,
            true,
            &der::tlv_vec(der::TAG_SEQUENCE, &[]),
        ),
        // digitalSignature is bit 0: one significant bit, seven unused.
        extension(
            oid::KEY_USAGE,
            true,
            &der::tlv_vec(der::TAG_BIT_STRING, &[0x07, 0x80]),
        ),
        extension(oid::EXT_KEY_USAGE, false, &eku),
        extension(
            oid::SUBJECT_ALT_NAME,
            false,
            &der::tlv_vec(der::TAG_SEQUENCE, &san),
        ),
    ];
    build_cert(
        CertInputs {
            subject_cn: &subject_cn,
            issuer_cn: &issuer_cn,
            subject_key: &params.device_key,
            not_before,
            not_after,
            extensions,
        },
        ca_key,
    )
}

/// Issue the self-signed issuing CA certificate for `team` at `generation`.
/// notBefore is backdated like a leaf; notAfter is [`NO_EXPIRY_UNIX_SECS`],
/// because a generation ends by rotation, not by time.
pub fn issue_team_ca(
    team: &TeamShort,
    generation: u32,
    ca_key: &SigningKey,
    issued_at_ns: u64,
) -> Result<Vec<u8>, PairingError> {
    let cn = ca_common_name(generation, team);
    let key = PublicKey(ca_key.verifying_key().to_bytes());
    let bc = der::constructed(
        der::TAG_SEQUENCE,
        &[
            &der::tlv_vec(der::TAG_BOOLEAN, &[0xFF]),
            &der::tlv_vec(der::TAG_INTEGER, &[0]),
        ],
    );
    let extensions = vec![
        extension(oid::BASIC_CONSTRAINTS, true, &bc),
        // keyCertSign is bit 5 and cRLSign bit 6: seven significant bits, one unused.
        extension(
            oid::KEY_USAGE,
            true,
            &der::tlv_vec(der::TAG_BIT_STRING, &[0x01, 0x06]),
        ),
    ];
    build_cert(
        CertInputs {
            subject_cn: &cn,
            issuer_cn: &cn,
            subject_key: &key,
            not_before: (issued_at_ns / NANOS_PER_SEC).saturating_sub(BACKDATE_SECS),
            not_after: NO_EXPIRY_UNIX_SECS,
            extensions,
        },
        ca_key,
    )
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Extensions {
    /// (cA, pathLenConstraint)
    basic_constraints: Option<(bool, Option<u64>)>,
    /// Key usage bits, bit 0 = digitalSignature as the most significant bit.
    key_usage: Option<u16>,
    eku_server: bool,
    eku_client: bool,
    eku_present: bool,
    san_uris: Vec<String>,
}

struct Parsed {
    tbs: Vec<u8>,
    issuer_cn: String,
    subject_cn: String,
    not_before: u64,
    not_after: u64,
    key: PublicKey,
    ext: Extensions,
    signature: Signature,
}

fn malformed(what: &'static str) -> PairingError {
    PairingError::TeamCertMalformed(what)
}

fn expect_alg_ed25519(t: &Tlv<'_>) -> Result<(), PairingError> {
    if t.raw == alg_ed25519().as_slice() {
        Ok(())
    } else {
        Err(PairingError::TeamCertNotEd25519)
    }
}

fn parse_name(t: &Tlv<'_>) -> Result<String, PairingError> {
    let mut seq = Reader::new(t.content);
    let rdn = seq.expect(der::TAG_SET, "name")?;
    seq.finish("a name has exactly one relative distinguished name")?;
    let mut set = Reader::new(rdn.content);
    let atv = set.expect(der::TAG_SEQUENCE, "name attribute")?;
    set.finish("a name has exactly one attribute")?;
    let mut atv = Reader::new(atv.content);
    let attr = atv.expect(der::TAG_OID, "name attribute type")?;
    if attr.content != oid::COMMON_NAME {
        return Err(malformed("the only name attribute is the common name"));
    }
    let value = atv.read()?;
    atv.finish("name attribute")?;
    if value.tag != der::TAG_UTF8_STRING && value.tag != der::TAG_PRINTABLE_STRING {
        return Err(malformed("common name string type"));
    }
    String::from_utf8(value.content.to_vec()).map_err(|_| malformed("common name encoding"))
}

/// A DER INTEGER content that is nonnegative and minimal, as a u64.
fn small_uint(content: &[u8]) -> Result<u64, PairingError> {
    let minimal = match content {
        [] => false,
        [0, next, ..] => next & 0x80 != 0,
        [first, ..] => first & 0x80 == 0,
    };
    if !minimal || content.len() > 8 {
        return Err(malformed("integer"));
    }
    Ok(content.iter().fold(0u64, |acc, b| (acc << 8) | *b as u64))
}

fn parse_extensions(content: &[u8]) -> Result<Extensions, PairingError> {
    let mut out = Extensions::default();
    let mut seen: Vec<&[u8]> = Vec::new();
    let mut list = Reader::new(content);
    if list.is_empty() {
        return Err(malformed("empty extensions"));
    }
    while !list.is_empty() {
        let ext = list.expect(der::TAG_SEQUENCE, "extension")?;
        let mut r = Reader::new(ext.content);
        let ext_oid = r.expect(der::TAG_OID, "extension id")?.content;
        if seen.contains(&ext_oid) {
            return Err(malformed("duplicate extension"));
        }
        seen.push(ext_oid);
        let critical = if r.peek_tag() == Some(der::TAG_BOOLEAN) {
            let b = r.read()?;
            if b.content != [0xFF] {
                // DER omits a FALSE default and writes TRUE as 0xFF.
                return Err(malformed("extension critical flag"));
            }
            true
        } else {
            false
        };
        let value = r.expect(der::TAG_OCTET_STRING, "extension value")?.content;
        r.finish("extension")?;
        match ext_oid {
            o if o == oid::BASIC_CONSTRAINTS => {
                let mut outer = Reader::new(value);
                let seq = outer.expect(der::TAG_SEQUENCE, "basic constraints")?;
                outer.finish("basic constraints")?;
                let mut bc = Reader::new(seq.content);
                let mut ca = false;
                if bc.peek_tag() == Some(der::TAG_BOOLEAN) {
                    if bc.read()?.content != [0xFF] {
                        return Err(malformed("basic constraints cA flag"));
                    }
                    ca = true;
                }
                let path_len = if bc.peek_tag() == Some(der::TAG_INTEGER) {
                    Some(small_uint(bc.read()?.content)?)
                } else {
                    None
                };
                bc.finish("basic constraints")?;
                out.basic_constraints = Some((ca, path_len));
            }
            o if o == oid::KEY_USAGE => {
                let mut outer = Reader::new(value);
                let bits = outer.expect(der::TAG_BIT_STRING, "key usage")?;
                outer.finish("key usage")?;
                let (unused, data) = bits.content.split_first().ok_or(malformed("key usage"))?;
                if *unused > 7 || data.is_empty() || data.len() > 2 {
                    return Err(malformed("key usage"));
                }
                let mut v: u16 = 0;
                for (i, b) in data.iter().enumerate() {
                    v |= (*b as u16) << (8 * (1 - i));
                }
                if v & ((1u16 << (*unused as u16 + 8 * (2 - data.len()) as u16)) - 1) != 0 {
                    return Err(malformed("key usage unused bits"));
                }
                out.key_usage = Some(v);
            }
            o if o == oid::EXT_KEY_USAGE => {
                let mut outer = Reader::new(value);
                let seq = outer.expect(der::TAG_SEQUENCE, "extended key usage")?;
                outer.finish("extended key usage")?;
                let mut r = Reader::new(seq.content);
                out.eku_present = true;
                while !r.is_empty() {
                    let purpose = r.expect(der::TAG_OID, "key purpose")?.content;
                    out.eku_server |= purpose == oid::KP_SERVER_AUTH;
                    out.eku_client |= purpose == oid::KP_CLIENT_AUTH;
                }
            }
            o if o == oid::SUBJECT_ALT_NAME => {
                let mut outer = Reader::new(value);
                let seq = outer.expect(der::TAG_SEQUENCE, "subject alt name")?;
                outer.finish("subject alt name")?;
                let mut r = Reader::new(seq.content);
                while !r.is_empty() {
                    let gn = r.expect(der::TAG_CTX6_PRIMITIVE, "only URI alt names")?;
                    let uri = core::str::from_utf8(gn.content)
                        .map_err(|_| malformed("alt name encoding"))?;
                    out.san_uris.push(uri.to_string());
                }
            }
            _ if critical => return Err(PairingError::TeamCertUnknownCriticalExtension),
            _ => {}
        }
    }
    Ok(out)
}

fn parse_cert(cert_der: &[u8]) -> Result<Parsed, PairingError> {
    let mut top = Reader::new(cert_der);
    let cert = top.expect(der::TAG_SEQUENCE, "certificate")?;
    top.finish("trailing bytes after the certificate")?;
    let mut c = Reader::new(cert.content);
    let tbs = c.expect(der::TAG_SEQUENCE, "tbsCertificate")?;
    let outer_alg = c.expect(der::TAG_SEQUENCE, "signature algorithm")?;
    let sig = c.expect(der::TAG_BIT_STRING, "signature")?;
    c.finish("certificate")?;
    expect_alg_ed25519(&outer_alg)?;
    let signature = match sig.content {
        [0, rest @ ..] if rest.len() == 64 => {
            let mut s = [0u8; 64];
            s.copy_from_slice(rest);
            Signature(s)
        }
        _ => return Err(malformed("signature length")),
    };

    let mut t = Reader::new(tbs.content);
    let version = t.expect(der::TAG_CTX0_CONSTRUCTED, "version must be v3")?;
    if version.content != [der::TAG_INTEGER, 1, 2] {
        return Err(malformed("version must be v3"));
    }
    let serial = t.expect(der::TAG_INTEGER, "serial")?.content;
    let serial_ok = match serial {
        [] => false,
        [0, next, ..] => next & 0x80 != 0,
        [first, ..] => first & 0x80 == 0,
    };
    if !serial_ok || serial.len() > 20 {
        return Err(malformed("serial"));
    }
    let inner_alg = t.expect(der::TAG_SEQUENCE, "signature algorithm")?;
    expect_alg_ed25519(&inner_alg)?;
    let issuer_cn = parse_name(&t.expect(der::TAG_SEQUENCE, "issuer")?)?;
    let validity = t.expect(der::TAG_SEQUENCE, "validity")?;
    let mut v = Reader::new(validity.content);
    let not_before = decode_time(&v.read()?)?;
    let not_after = decode_time(&v.read()?)?;
    v.finish("validity")?;
    let subject_cn = parse_name(&t.expect(der::TAG_SEQUENCE, "subject")?)?;
    let spki = t.expect(der::TAG_SEQUENCE, "subject public key info")?;
    let mut s = Reader::new(spki.content);
    expect_alg_ed25519(&s.expect(der::TAG_SEQUENCE, "key algorithm")?)?;
    let key_bits = s.expect(der::TAG_BIT_STRING, "subject key")?;
    s.finish("subject public key info")?;
    let key = match key_bits.content {
        [0, rest @ ..] if rest.len() == 32 => {
            let mut k = [0u8; 32];
            k.copy_from_slice(rest);
            PublicKey(k)
        }
        _ => return Err(PairingError::TeamCertNotEd25519),
    };
    let exts = t.expect(der::TAG_CTX3_CONSTRUCTED, "extensions are required")?;
    t.finish("unexpected field after the extensions")?;
    let mut e = Reader::new(exts.content);
    let ext_list = e.expect(der::TAG_SEQUENCE, "extensions")?;
    e.finish("extensions")?;
    let ext = parse_extensions(ext_list.content)?;

    Ok(Parsed {
        tbs: tbs.raw.to_vec(),
        issuer_cn,
        subject_cn,
        not_before,
        not_after,
        key,
        ext,
        signature,
    })
}

const KU_DIGITAL_SIGNATURE: u16 = 0x8000;
const KU_KEY_CERT_SIGN: u16 = 0x0400;

/// Check a machine leaf against the profile and return what it names.
///
/// Refuses: malformed DER, a key or signature algorithm other than ed25519,
/// a CA certificate, a keyUsage without digitalSignature or with
/// keyCertSign, a missing extKeyUsage, an extKeyUsage without both
/// serverAuth and clientAuth, a common name that is not a role name, an
/// issuer name that is not the CA name of the same team and generation, a
/// lifetime longer than the role allows (a robot must carry
/// [`NO_EXPIRY_UNIX_SECS`]), a device URI that does not name the subject key,
/// and any other alt name. It does not check the signature; see
/// [`verify_leaf_issued_by`].
pub fn check_leaf(cert_der: &[u8]) -> Result<LeafFacts, PairingError> {
    let p = parse_cert(cert_der)?;
    match p.ext.basic_constraints {
        Some((true, _)) => return Err(PairingError::TeamCertIsCa),
        Some((false, Some(_))) => return Err(malformed("path length on a leaf")),
        _ => {}
    }
    match p.ext.key_usage {
        Some(ku) if ku & KU_DIGITAL_SIGNATURE != 0 && ku & KU_KEY_CERT_SIGN == 0 => {}
        _ => return Err(PairingError::TeamCertKeyUsage),
    }
    if !p.ext.eku_present {
        return Err(PairingError::TeamCertMissingEku);
    }
    if !(p.ext.eku_server && p.ext.eku_client) {
        return Err(PairingError::TeamCertIncompleteEku);
    }
    let (role, generation, team) = parse_common_name(&p.subject_cn)?;
    if p.issuer_cn != ca_common_name(generation, &team) {
        return Err(PairingError::TeamCertIssuerName);
    }
    let lifetime_ok = match role.lifetime_secs() {
        None => p.not_after == NO_EXPIRY_UNIX_SECS,
        Some(life) => {
            p.not_after > p.not_before && p.not_after - p.not_before <= life + BACKDATE_SECS
        }
    };
    if !lifetime_ok {
        return Err(PairingError::TeamCertLifetime { role });
    }

    let mut device_seen = false;
    let mut account = None;
    for uri in &p.ext.san_uris {
        if let Some(k) = uri_bytes32(uri, "urn:cerulion:device:") {
            if device_seen || k != p.key.0 {
                return Err(PairingError::TeamCertDeviceUriMismatch);
            }
            device_seen = true;
        } else if let Some(a) = uri_bytes32(uri, "urn:cerulion:account:") {
            if account.is_some() {
                return Err(malformed("more than one account alt name"));
            }
            account = Some(AccountId(a));
        } else {
            return Err(malformed("unexpected alt name"));
        }
    }
    if !device_seen {
        return Err(PairingError::TeamCertDeviceUriMismatch);
    }

    Ok(LeafFacts {
        role,
        generation,
        team,
        device_key: p.key,
        account,
        not_before_unix_secs: p.not_before,
        not_after_unix_secs: p.not_after,
        tbs: p.tbs,
        signature: p.signature,
    })
}

/// Check a team issuing CA certificate against the profile: self-issued under
/// a `ca.g<N>.<team>` name, basicConstraints CA with pathLenConstraint 0,
/// keyUsage with keyCertSign, and a valid self-signature.
pub fn check_team_ca(cert_der: &[u8]) -> Result<TeamCaFacts, PairingError> {
    let p = parse_cert(cert_der)?;
    let (generation, team) = parse_ca_common_name(&p.subject_cn)?;
    if p.issuer_cn != p.subject_cn {
        return Err(PairingError::TeamCertIssuerName);
    }
    if p.ext.basic_constraints != Some((true, Some(0))) {
        return Err(PairingError::TeamCaProfile(
            "basicConstraints must be CA with path length 0",
        ));
    }
    match p.ext.key_usage {
        Some(ku) if ku & KU_KEY_CERT_SIGN != 0 => {}
        _ => {
            return Err(PairingError::TeamCaProfile(
                "keyUsage must include keyCertSign",
            ))
        }
    }
    if !p.ext.san_uris.is_empty() {
        return Err(PairingError::TeamCaProfile("a CA carries no alt names"));
    }
    verify_tbs(&p.key, &p.tbs, &p.signature)?;
    Ok(TeamCaFacts {
        generation,
        team,
        ca_key: p.key,
    })
}

fn verify_tbs(key: &PublicKey, tbs: &[u8], sig: &Signature) -> Result<(), PairingError> {
    match crypto::ed25519_verify(key, tbs, sig) {
        VerifyResult::Ok => Ok(()),
        VerifyResult::BadKey => Err(PairingError::BadKey),
        VerifyResult::BadSig => Err(PairingError::BadSignature(CertKind::TeamCert)),
    }
}

/// Check a leaf with [`check_leaf`], require it to name the CA's team and
/// generation, and verify its signature with the CA key.
pub fn verify_leaf_issued_by(cert_der: &[u8], ca: &TeamCaFacts) -> Result<LeafFacts, PairingError> {
    let facts = check_leaf(cert_der)?;
    if facts.team != ca.team || facts.generation != ca.generation {
        return Err(PairingError::TeamCertIssuerMismatch);
    }
    verify_tbs(&ca.ca_key, &facts.tbs, &facts.signature)?;
    Ok(facts)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_encoding_switches_at_2050_and_round_trips() {
        // 2049-12-31T23:59:59Z and 2050-01-01T00:00:00Z.
        let last_utc = 2_524_607_999;
        let first_gen = 2_524_608_000;
        assert_eq!(
            encode_time(last_utc).unwrap(),
            [&[0x17, 13][..], b"491231235959Z"].concat()
        );
        assert_eq!(
            encode_time(first_gen).unwrap(),
            [&[0x18, 15][..], b"20500101000000Z"].concat()
        );
        assert_eq!(
            encode_time(NO_EXPIRY_UNIX_SECS).unwrap(),
            [&[0x18, 15][..], b"99991231235959Z"].concat()
        );
        assert!(matches!(
            encode_time(NO_EXPIRY_UNIX_SECS + 1),
            Err(PairingError::TeamCertTimeOutOfRange)
        ));
        for secs in [0, 951_782_400, last_utc, first_gen, NO_EXPIRY_UNIX_SECS] {
            let enc = encode_time(secs).unwrap();
            let t = Reader::new(&enc).read().unwrap();
            assert_eq!(decode_time(&t).unwrap(), secs);
        }
    }

    #[test]
    fn decode_time_refuses_impossible_dates() {
        for bad in [
            &b"490230000000Z"[..], // 30 February
            b"491301000000Z",      // month 13
            b"491231240000Z",      // hour 24
            b"4912312359590",      // no Z
            b"49123123595Z",       // short
        ] {
            let enc = der::tlv_vec(der::TAG_UTC_TIME, bad);
            let t = Reader::new(&enc).read().unwrap();
            assert!(decode_time(&t).is_err(), "{:?}", core::str::from_utf8(bad));
        }
        // 2000 is a leap year, 2100 is not.
        let ok = der::tlv_vec(der::TAG_UTC_TIME, b"000229000000Z");
        assert!(decode_time(&Reader::new(&ok).read().unwrap()).is_ok());
        let bad = der::tlv_vec(der::TAG_GENERALIZED_TIME, b"21000229000000Z");
        assert!(decode_time(&Reader::new(&bad).read().unwrap()).is_err());
    }
}
