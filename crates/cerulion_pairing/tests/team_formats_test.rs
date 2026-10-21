// SPDX-License-Identifier: MIT OR Apache-2.0
//! Team trust formats: the X.509 leaf and CA profile, the trust bundle, the
//! status list, the freshness proof and the anti-rollback decisions.
//!
//! Oracles are independent of the encoder: hand-written DER byte strings for
//! each extension, x509-parser reading the issued certificates field by
//! field, rustls-webpki accepting them for client and server authentication,
//! data-encoding for base32, rcgen for certificates that leave the profile,
//! and hand-built signing bytes for each statement.

mod common;

use std::time::Duration;

use cerulion_pairing::format::{AccountId, PublicKey, RootSet, SignedIntermediateCert};
use cerulion_pairing::team::rollback::{
    check_clock, decide_bundle, decide_status_list, BundleHighWater, RollbackDecision,
    StatusHighWater,
};
use cerulion_pairing::team::*;
use cerulion_pairing::{CertKind, PairingError};
use common::{make_intermediate, pk, sk, wide_validity, ISSUED, T_NOW};
use ed25519_dalek::SigningKey;
use rustls_pki_types::{CertificateDer, UnixTime};

// 2026-10-20T00:00:00Z.
const ISSUE_SECS: u64 = 1_792_454_400;
const ISSUE_NS: u64 = ISSUE_SECS * 1_000_000_000;

fn acme() -> TeamShort {
    TeamShort::new("acme").unwrap()
}

fn ca_key(generation: u8) -> SigningKey {
    sk(0x40 + generation)
}

fn device(seed: u8) -> SigningKey {
    sk(seed)
}

fn leaf_params(role: TeamRole, generation: u32, seed: u8) -> LeafParams {
    LeafParams {
        team: acme(),
        generation,
        role,
        device_key: pk(&device(seed)),
        account: Some(AccountId([0xAC; 32])),
        issued_at_ns: ISSUE_NS,
    }
}

fn ca_der(generation: u8) -> Vec<u8> {
    issue_team_ca(&acme(), generation as u32, &ca_key(generation), ISSUE_NS).unwrap()
}

fn leaf(role: TeamRole, generation: u8, seed: u8) -> Vec<u8> {
    issue_leaf(
        &leaf_params(role, generation as u32, seed),
        &ca_key(generation),
    )
    .unwrap()
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.windows(needle.len()).any(|w| w == needle)
}

fn b32(bytes: &[u8]) -> String {
    data_encoding::BASE32_NOPAD.encode(bytes).to_lowercase()
}

// ---- intermediate fixture ---------------------------------------------------

fn roots() -> (RootSet, SigningKey, SignedIntermediateCert) {
    let (r1, r2) = (sk(1), sk(2));
    let root_set = RootSet::new(vec![pk(&r1), pk(&r2)], 2).unwrap();
    let int = sk(9);
    let signed = make_intermediate(pk(&int), wide_validity(), ISSUED).sign_by_roots(&[&r1, &r2]);
    (root_set, int, signed)
}

fn team_id() -> TeamId {
    TeamId([0x7E; 32])
}

fn bundle(generation: u8, robots: Vec<Vec<u8>>) -> TeamTrustBundle {
    let (_, int, _) = roots();
    TeamTrustBundle {
        version: 1,
        team_id: team_id(),
        team_short: acme(),
        generation: generation as u32,
        issued_at_ns: ISSUED,
        issuing_ca_der: ca_der(generation),
        retired: (1..generation)
            .rev()
            .map(|g| RetiredGeneration {
                generation: g as u32,
                ca_der: ca_der(g),
            })
            .collect(),
        robot_leaves: robots,
        issuer_key: pk(&int),
    }
}

fn sign_bundle(b: TeamTrustBundle) -> SignedTeamTrustBundle {
    let (_, int, signed_int) = roots();
    b.sign(&int, signed_int)
}

fn fp(seed: u8) -> DeviceFingerprint {
    DeviceFingerprint::of(&pk(&device(seed)))
}

fn status_list(seq: u64, mut entries: Vec<StatusEntry>) -> TeamStatusList {
    let (_, int, _) = roots();
    entries.sort_by_key(|e| e.device);
    TeamStatusList {
        version: 1,
        team_id: team_id(),
        seq,
        issued_at_ns: ISSUED,
        entries,
        issuer_key: pk(&int),
    }
}

fn entry(seed: u8, status: DeviceStatus, entry_seq: u64) -> StatusEntry {
    StatusEntry {
        device: fp(seed),
        status,
        entry_seq,
        since_ns: ISSUED,
    }
}

fn freshness() -> StatusFreshness {
    let (_, int, _) = roots();
    StatusFreshness {
        version: 1,
        team_id: team_id(),
        asker: fp(20),
        nonce: [0x5A; 16],
        list_seq: 4,
        service_time_ns: ISSUED,
        issuer_key: pk(&int),
    }
}

fn webpki_verify(
    leaf_der: &[u8],
    ca: &[u8],
    at_secs: u64,
    client: bool,
) -> Result<(), webpki::Error> {
    let ca = CertificateDer::from(ca);
    let anchor = webpki::anchor_from_trusted_cert(&ca)?.to_owned();
    let leaf = CertificateDer::from(leaf_der);
    let ee = webpki::EndEntityCert::try_from(&leaf)?;
    let usage = if client {
        webpki::KeyUsage::client_auth()
    } else {
        webpki::KeyUsage::server_auth()
    };
    ee.verify_for_usage(
        &[webpki::ring::ED25519],
        &[anchor],
        &[],
        UnixTime::since_unix_epoch(Duration::from_secs(at_secs)),
        usage,
        None,
        None,
    )
    .map(|_| ())
}

// ============================================================================
// Names
// ============================================================================

#[test]
fn common_names_are_the_exact_strings_the_link_matches() {
    assert_eq!(
        common_name(TeamRole::Robot, 2, &acme()),
        "robot.g2.acme.cerulion.invalid"
    );
    assert_eq!(
        common_names(7, &acme()),
        [
            "robot.g7.acme.cerulion.invalid",
            "owner.g7.acme.cerulion.invalid",
            "operator.g7.acme.cerulion.invalid",
            "viewer.g7.acme.cerulion.invalid",
            "ci.g7.acme.cerulion.invalid",
        ]
    );
    assert_eq!(ca_common_name(3, &acme()), "ca.g3.acme.cerulion.invalid");
    assert_eq!(
        common_name(TeamRole::Ci, u32::MAX, &acme()),
        "ci.g4294967295.acme.cerulion.invalid"
    );
}

#[test]
fn common_names_parse_back_and_ca_names_are_not_roles() {
    for g in [0, 1, 9, 10, u32::MAX] {
        for (role, cn) in TeamRole::ALL.into_iter().zip(common_names(g, &acme())) {
            assert_eq!(parse_common_name(&cn).unwrap(), (role, g, acme()));
        }
    }
    assert!(matches!(
        parse_common_name("ca.g1.acme.cerulion.invalid"),
        Err(PairingError::TeamNameMismatch)
    ));
    assert_eq!(
        parse_ca_common_name("ca.g1.acme.cerulion.invalid").unwrap(),
        (1, acme())
    );
    assert!(matches!(
        parse_ca_common_name("robot.g1.acme.cerulion.invalid"),
        Err(PairingError::TeamNameMismatch)
    ));
}

#[test]
fn team_short_is_one_dns_label() {
    let (long, too_long) = ("x".repeat(63), "x".repeat(64));
    for good in ["a", "acme", "team-7", "0", long.as_str()] {
        assert_eq!(TeamShort::new(good).unwrap().as_str(), good);
    }
    for bad in [
        "",
        "-acme",
        "acme-",
        "Acme",
        "ac.me",
        "ac_me",
        "acmé",
        too_long.as_str(),
    ] {
        assert!(
            matches!(TeamShort::new(bad), Err(PairingError::InvalidTeamShort)),
            "{bad}"
        );
    }
    // The serde form validates too.
    let bad: Result<TeamShort, _> = postcard::from_bytes(&postcard::to_allocvec("A").unwrap());
    assert!(bad.is_err());
}

#[test]
fn device_fingerprint_is_sha256_of_the_raw_key() {
    use sha2::{Digest, Sha256};
    let key = pk(&device(5));
    let want: [u8; 32] = Sha256::digest(key.0).into();
    assert_eq!(DeviceFingerprint::of(&key).0, want);
    // SHA-256 of 32 zero bytes, a published constant.
    assert_eq!(
        DeviceFingerprint::of(&PublicKey([0; 32])).0[..4],
        [0x66, 0x68, 0x7a, 0xad]
    );
}

#[test]
fn san_uris_are_lowercase_unpadded_base32() {
    let key = pk(&device(5));
    assert_eq!(
        device_uri(&key),
        format!("urn:cerulion:device:{}", b32(&key.0))
    );
    assert_eq!(
        account_uri(&AccountId([0xAC; 32])),
        format!("urn:cerulion:account:{}", b32(&[0xAC; 32]))
    );
    assert_eq!(b32(&key.0).len(), 52);
}

// ============================================================================
// Leaf profile: issued bytes
// ============================================================================

#[test]
fn leaf_carries_the_profile_extensions_byte_for_byte() {
    let der = leaf(TeamRole::Operator, 2, 5);
    // basicConstraints, critical, empty SEQUENCE (not a CA).
    assert!(contains(
        &der,
        &[0x30, 0x0C, 0x06, 0x03, 0x55, 0x1D, 0x13, 0x01, 0x01, 0xFF, 0x04, 0x02, 0x30, 0x00]
    ));
    // keyUsage, critical, digitalSignature only.
    assert!(contains(
        &der,
        &[
            0x30, 0x0E, 0x06, 0x03, 0x55, 0x1D, 0x0F, 0x01, 0x01, 0xFF, 0x04, 0x04, 0x03, 0x02,
            0x07, 0x80
        ]
    ));
    // extKeyUsage, serverAuth then clientAuth.
    assert!(contains(
        &der,
        &[
            0x30, 0x1D, 0x06, 0x03, 0x55, 0x1D, 0x25, 0x04, 0x16, 0x30, 0x14, 0x06, 0x08, 0x2B,
            0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x01, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x05, 0x05,
            0x07, 0x03, 0x02
        ]
    ));
    // Ed25519 algorithm identifier, absent parameters.
    assert!(contains(&der, &[0x30, 0x05, 0x06, 0x03, 0x2B, 0x65, 0x70]));
    // v3 then a 16 byte serial.
    assert!(contains(&der, &[0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x10]));
    // The subject key as the SPKI bit string.
    let mut spki_bits = vec![0x03, 0x21, 0x00];
    spki_bits.extend_from_slice(&pk(&device(5)).0);
    assert!(contains(&der, &spki_bits));
}

#[test]
fn leaf_times_follow_rfc5280_and_the_role_lifetime() {
    // 2026-10-19T23:00:00Z as UTCTime, then +7 d from issuance.
    let desk = leaf(TeamRole::Viewer, 1, 5);
    assert!(contains(
        &desk,
        &[&[0x17, 0x0D][..], b"261019230000Z"].concat()
    ));
    assert!(contains(
        &desk,
        &[&[0x17, 0x0D][..], b"261027000000Z"].concat()
    ));
    let ci = leaf(TeamRole::Ci, 1, 5);
    assert!(contains(
        &ci,
        &[&[0x17, 0x0D][..], b"261021000000Z"].concat()
    ));
    // A robot never expires: GeneralizedTime 99991231235959Z.
    let robot = leaf(TeamRole::Robot, 1, 5);
    assert!(contains(
        &robot,
        &[&[0x18, 0x0F][..], b"99991231235959Z"].concat()
    ));
}

#[test]
fn x509_parser_reads_every_leaf_field_independently() {
    use x509_parser::prelude::*;
    let der = leaf(TeamRole::Robot, 2, 5);
    let (rest, cert) = X509Certificate::from_der(&der).unwrap();
    assert!(rest.is_empty());
    assert_eq!(cert.version(), X509Version::V3);
    let cn = |n: &X509Name<'_>| {
        n.iter_common_name()
            .next()
            .unwrap()
            .as_str()
            .unwrap()
            .to_string()
    };
    assert_eq!(cn(cert.subject()), "robot.g2.acme.cerulion.invalid");
    assert_eq!(cn(cert.issuer()), "ca.g2.acme.cerulion.invalid");
    assert_eq!(
        cert.validity().not_before.timestamp(),
        (ISSUE_SECS - 3600) as i64
    );
    assert_eq!(cert.validity().not_after.timestamp(), 253_402_300_799);
    assert_eq!(
        &cert.public_key().subject_public_key.data[..],
        &pk(&device(5)).0
    );
    assert_eq!(cert.raw_serial().len(), 16);

    let eku = cert.extended_key_usage().unwrap().unwrap();
    assert!(eku.value.server_auth && eku.value.client_auth);
    assert!(!eku.critical);
    let ku = cert.key_usage().unwrap().unwrap();
    assert!(ku.critical && ku.value.digital_signature() && !ku.value.key_cert_sign());
    let bc = cert.basic_constraints().unwrap().unwrap();
    assert!(bc.critical && !bc.value.ca);
    let san = cert.subject_alternative_name().unwrap().unwrap();
    let uris: Vec<String> = san
        .value
        .general_names
        .iter()
        .map(|g| match g {
            GeneralName::URI(u) => u.to_string(),
            other => panic!("unexpected alt name {other:?}"),
        })
        .collect();
    assert_eq!(
        uris,
        [
            format!("urn:cerulion:device:{}", b32(&pk(&device(5)).0)),
            format!("urn:cerulion:account:{}", b32(&[0xAC; 32])),
        ]
    );
}

#[test]
fn x509_parser_reads_the_team_ca_profile() {
    use x509_parser::prelude::*;
    let der = ca_der(3);
    assert!(contains(
        &der,
        &[
            0x30, 0x12, 0x06, 0x03, 0x55, 0x1D, 0x13, 0x01, 0x01, 0xFF, 0x04, 0x08, 0x30, 0x06,
            0x01, 0x01, 0xFF, 0x02, 0x01, 0x00
        ]
    ));
    assert!(contains(
        &der,
        &[
            0x30, 0x0E, 0x06, 0x03, 0x55, 0x1D, 0x0F, 0x01, 0x01, 0xFF, 0x04, 0x04, 0x03, 0x02,
            0x01, 0x06
        ]
    ));
    let (_, cert) = X509Certificate::from_der(&der).unwrap();
    let bc = cert.basic_constraints().unwrap().unwrap().value;
    assert!(bc.ca && bc.path_len_constraint == Some(0));
    let ku = cert.key_usage().unwrap().unwrap().value;
    assert!(ku.key_cert_sign() && ku.crl_sign() && !ku.digital_signature());
    assert_eq!(cert.subject(), cert.issuer());
}

#[test]
fn webpki_accepts_every_role_for_client_and_server_auth() {
    let ca = ca_der(2);
    for role in TeamRole::ALL {
        let der = leaf(role, 2, 5);
        for client in [true, false] {
            webpki_verify(&der, &ca, ISSUE_SECS + 60, client)
                .unwrap_or_else(|e| panic!("{role} client={client}: {e:?}"));
        }
    }
    // A robot leaf is still valid in the year 9000.
    webpki_verify(&leaf(TeamRole::Robot, 2, 5), &ca, 221_845_392_000, true).unwrap();
}

#[test]
fn webpki_refuses_an_expired_desk_leaf_and_a_foreign_ca() {
    let ca = ca_der(2);
    let desk = leaf(TeamRole::Operator, 2, 5);
    let after = ISSUE_SECS + DESK_LIFETIME_SECS + 1;
    assert!(matches!(
        webpki_verify(&desk, &ca, after, true),
        Err(webpki::Error::CertExpired { .. })
    ));
    let other = issue_team_ca(&TeamShort::new("other").unwrap(), 2, &sk(0x77), ISSUE_NS).unwrap();
    assert!(webpki_verify(&desk, &other, ISSUE_SECS + 60, true).is_err());
}

// ============================================================================
// Leaf profile: checker
// ============================================================================

#[test]
fn check_leaf_returns_what_the_leaf_names() {
    let facts = check_leaf(&leaf(TeamRole::Operator, 4, 5)).unwrap();
    assert_eq!(facts.role, TeamRole::Operator);
    assert_eq!(facts.generation, 4);
    assert_eq!(facts.team, acme());
    assert_eq!(facts.device_key, pk(&device(5)));
    assert_eq!(facts.account, Some(AccountId([0xAC; 32])));
    assert_eq!(facts.not_before_unix_secs, ISSUE_SECS - BACKDATE_SECS);
    assert_eq!(facts.not_after_unix_secs, ISSUE_SECS + DESK_LIFETIME_SECS);

    let ca = check_team_ca(&ca_der(4)).unwrap();
    assert_eq!(
        (ca.generation, ca.team.clone(), ca.ca_key),
        (4, acme(), pk(&ca_key(4)))
    );
    assert_eq!(
        verify_leaf_issued_by(&leaf(TeamRole::Operator, 4, 5), &ca).unwrap(),
        facts
    );
}

#[test]
fn a_leaf_without_an_account_carries_only_the_device_uri() {
    let mut p = leaf_params(TeamRole::Robot, 1, 5);
    p.account = None;
    let der = issue_leaf(&p, &ca_key(1)).unwrap();
    assert_eq!(check_leaf(&der).unwrap().account, None);
    assert!(!contains(&der, b"urn:cerulion:account:"));
}

#[test]
fn issuance_near_the_epoch_and_after_2050_encodes() {
    let mut p = leaf_params(TeamRole::Viewer, 1, 5);
    p.issued_at_ns = 600 * 1_000_000_000;
    let early = check_leaf(&issue_leaf(&p, &ca_key(1)).unwrap()).unwrap();
    assert_eq!(
        early.not_before_unix_secs, 0,
        "the backdate saturates at 1970"
    );

    // 2051-01-01T00:00:00Z: both times are GeneralizedTime.
    p.issued_at_ns = 2_556_144_000 * 1_000_000_000;
    let der = issue_leaf(&p, &ca_key(1)).unwrap();
    assert!(contains(
        &der,
        &[&[0x18, 0x0F][..], b"20501231230000Z"].concat()
    ));
    assert!(contains(
        &der,
        &[&[0x18, 0x0F][..], b"20510108000000Z"].concat()
    ));
    check_leaf(&der).unwrap();
}

#[test]
fn the_latest_representable_issuance_encodes() {
    // u64::MAX nanoseconds is 2554-07-21T23:34:33Z, so no issuance time can
    // push a desk leaf past 9999.
    let mut p = leaf_params(TeamRole::Owner, 1, 5);
    p.issued_at_ns = u64::MAX;
    let der = issue_leaf(&p, &ca_key(1)).unwrap();
    assert!(contains(
        &der,
        &[&[0x18, 0x0F][..], b"25540721223433Z"].concat()
    ));
    assert!(contains(
        &der,
        &[&[0x18, 0x0F][..], b"25540728233433Z"].concat()
    ));
    check_leaf(&der).unwrap();
}

#[test]
fn issuance_is_deterministic() {
    assert_eq!(leaf(TeamRole::Robot, 2, 5), leaf(TeamRole::Robot, 2, 5));
    assert_eq!(ca_der(2), ca_der(2));
    // The serial depends on the subject key.
    let (da, db) = (leaf(TeamRole::Robot, 2, 5), leaf(TeamRole::Robot, 2, 6));
    let a = x509_parser::parse_x509_certificate(&da).unwrap().1;
    let b = x509_parser::parse_x509_certificate(&db).unwrap().1;
    assert_ne!(a.raw_serial(), b.raw_serial());
}

// ---- refusals, built with rcgen ----------------------------------------------

fn pkcs8_ed25519(seed: &[u8; 32]) -> Vec<u8> {
    let mut v = vec![
        0x30, 0x2E, 0x02, 0x01, 0x00, 0x30, 0x05, 0x06, 0x03, 0x2B, 0x65, 0x70, 0x04, 0x22, 0x04,
        0x20,
    ];
    v.extend_from_slice(seed);
    v
}

fn rcgen_key(seed: u8) -> rcgen::KeyPair {
    rcgen::KeyPair::try_from(pkcs8_ed25519(&[seed; 32])).unwrap()
}

/// A leaf rcgen builds that is otherwise in profile, signed by the generation
/// 2 team CA key, for `edit` to push out of profile.
fn rcgen_leaf(edit: impl FnOnce(&mut rcgen::CertificateParams)) -> Vec<u8> {
    let ca_kp = rcgen_key(0x42);
    let mut ca_params = rcgen::CertificateParams::new(Vec::<String>::new()).unwrap();
    ca_params.distinguished_name = rcgen::DistinguishedName::new();
    ca_params
        .distinguished_name
        .push(rcgen::DnType::CommonName, "ca.g2.acme.cerulion.invalid");
    let issuer = rcgen::Issuer::from_params(&ca_params, &ca_kp);

    let dev = rcgen_key(5);
    let mut p = rcgen::CertificateParams::new(Vec::<String>::new()).unwrap();
    p.distinguished_name = rcgen::DistinguishedName::new();
    p.distinguished_name.push(
        rcgen::DnType::CommonName,
        "operator.g2.acme.cerulion.invalid",
    );
    p.key_usages = vec![rcgen::KeyUsagePurpose::DigitalSignature];
    p.extended_key_usages = vec![
        rcgen::ExtendedKeyUsagePurpose::ServerAuth,
        rcgen::ExtendedKeyUsagePurpose::ClientAuth,
    ];
    p.not_before = rcgen::date_time_ymd(2026, 10, 19);
    p.not_after = rcgen::date_time_ymd(2026, 10, 26);
    p.subject_alt_names = vec![rcgen::SanType::URI(
        rcgen::string::Ia5String::try_from(device_uri(&pk(&device(5)))).unwrap(),
    )];
    edit(&mut p);
    p.signed_by(&dev, &issuer).unwrap().der().to_vec()
}

#[test]
fn an_in_profile_rcgen_leaf_passes_the_checker() {
    // The control for every rcgen refusal below: only the edit differs.
    let der = rcgen_leaf(|_| {});
    let facts = check_leaf(&der).unwrap();
    assert_eq!(facts.role, TeamRole::Operator);
    let ca = check_team_ca(&ca_der(2)).unwrap();
    verify_leaf_issued_by(&der, &ca).unwrap();
}

#[test]
fn a_leaf_without_eku_is_refused() {
    // webpki reads a missing extKeyUsage as any purpose; the profile does not.
    let der = rcgen_leaf(|p| p.extended_key_usages.clear());
    webpki_verify(&der, &ca_der(2), ISSUE_SECS, true).unwrap();
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertMissingEku)
    ));
}

#[test]
fn a_leaf_with_one_auth_purpose_is_refused() {
    for only in [
        rcgen::ExtendedKeyUsagePurpose::ServerAuth,
        rcgen::ExtendedKeyUsagePurpose::ClientAuth,
    ] {
        let der = rcgen_leaf(|p| p.extended_key_usages = vec![only]);
        assert!(matches!(
            check_leaf(&der),
            Err(PairingError::TeamCertIncompleteEku)
        ));
    }
}

#[test]
fn a_ca_presented_as_a_leaf_is_refused() {
    let der = rcgen_leaf(|p| p.is_ca = rcgen::IsCa::Ca(rcgen::BasicConstraints::Unconstrained));
    assert!(matches!(check_leaf(&der), Err(PairingError::TeamCertIsCa)));
    // Our own CA certificate fails as a leaf too.
    assert!(check_leaf(&ca_der(2)).is_err());
}

#[test]
fn a_leaf_without_digital_signature_is_refused() {
    let der = rcgen_leaf(|p| p.key_usages = vec![rcgen::KeyUsagePurpose::KeyEncipherment]);
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertKeyUsage)
    ));
    let der = rcgen_leaf(|p| p.key_usages.clear());
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertKeyUsage)
    ));
}

#[test]
fn a_non_ed25519_leaf_is_refused() {
    let ca_kp = rcgen_key(0x42);
    let mut ca_params = rcgen::CertificateParams::new(Vec::<String>::new()).unwrap();
    ca_params.distinguished_name = rcgen::DistinguishedName::new();
    ca_params
        .distinguished_name
        .push(rcgen::DnType::CommonName, "ca.g2.acme.cerulion.invalid");
    let issuer = rcgen::Issuer::from_params(&ca_params, &ca_kp);
    let p256 = rcgen::KeyPair::generate_for(&rcgen::PKCS_ECDSA_P256_SHA256).unwrap();
    let mut p = rcgen::CertificateParams::new(Vec::<String>::new()).unwrap();
    p.distinguished_name = rcgen::DistinguishedName::new();
    p.distinguished_name.push(
        rcgen::DnType::CommonName,
        "operator.g2.acme.cerulion.invalid",
    );
    let der = p.signed_by(&p256, &issuer).unwrap().der().to_vec();
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertNotEd25519)
    ));
}

#[test]
fn a_foreign_common_name_is_refused() {
    for cn in [
        "www.example.com",
        "operator.g2.acme.cerulion.example",
        "admin.g2.acme.cerulion.invalid",
        "operator.g02.acme.cerulion.invalid",
    ] {
        let der = rcgen_leaf(|p| {
            p.distinguished_name = rcgen::DistinguishedName::new();
            p.distinguished_name.push(rcgen::DnType::CommonName, cn);
        });
        assert!(
            matches!(check_leaf(&der), Err(PairingError::TeamNameMismatch)),
            "{cn}"
        );
    }
    // A subject with a second attribute leaves the profile.
    let der = rcgen_leaf(|p| {
        p.distinguished_name
            .push(rcgen::DnType::OrganizationName, "Acme")
    });
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertMalformed(_))
    ));
}

#[test]
fn a_leaf_naming_another_generation_than_its_issuer_is_refused() {
    let der = rcgen_leaf(|p| {
        p.distinguished_name = rcgen::DistinguishedName::new();
        p.distinguished_name.push(
            rcgen::DnType::CommonName,
            "operator.g3.acme.cerulion.invalid",
        );
    });
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertIssuerName)
    ));
}

#[test]
fn a_lifetime_outside_the_role_is_refused() {
    // Operator for 30 days.
    let der = rcgen_leaf(|p| p.not_after = rcgen::date_time_ymd(2026, 11, 18));
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertLifetime {
            role: TeamRole::Operator
        })
    ));
    // Operator that never expires.
    let der = rcgen_leaf(|p| p.not_after = rcgen::date_time_ymd(9999, 12, 31));
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertLifetime { .. })
    ));
    // Robot with a finite expiry.
    let der = rcgen_leaf(|p| {
        p.distinguished_name = rcgen::DistinguishedName::new();
        p.distinguished_name
            .push(rcgen::DnType::CommonName, "robot.g2.acme.cerulion.invalid");
    });
    assert!(matches!(
        check_leaf(&der),
        Err(PairingError::TeamCertLifetime {
            role: TeamRole::Robot
        })
    ));
    // Exactly the role lifetime plus the backdate is accepted.
    let der = rcgen_leaf(|p| {
        p.not_before = rcgen::date_time_ymd(2026, 10, 19);
        p.not_after = rcgen::date_time_ymd(2026, 10, 26)
            .checked_add(time::Duration::hours(1))
            .unwrap();
    });
    check_leaf(&der).unwrap();
}

#[test]
fn device_uri_must_name_the_subject_key_exactly_once() {
    let uri = |s: String| rcgen::SanType::URI(rcgen::string::Ia5String::try_from(s).unwrap());
    let other = rcgen_leaf(|p| p.subject_alt_names = vec![uri(device_uri(&pk(&device(6))))]);
    let none = rcgen_leaf(|p| p.subject_alt_names.clear());
    let twice = rcgen_leaf(|p| {
        p.subject_alt_names = vec![
            uri(device_uri(&pk(&device(5)))),
            uri(device_uri(&pk(&device(5)))),
        ]
    });
    for der in [other, none, twice] {
        assert!(matches!(
            check_leaf(&der),
            Err(PairingError::TeamCertDeviceUriMismatch)
        ));
    }
    let dns = rcgen_leaf(|p| {
        p.subject_alt_names
            .push(rcgen::SanType::DnsName("robot.example".try_into().unwrap()))
    });
    assert!(matches!(
        check_leaf(&dns),
        Err(PairingError::TeamCertMalformed(_))
    ));
    let stray = rcgen_leaf(|p| p.subject_alt_names.push(uri("urn:example:x".into())));
    assert!(matches!(
        check_leaf(&stray),
        Err(PairingError::TeamCertMalformed(_))
    ));
}

#[test]
fn an_unknown_critical_extension_is_refused_and_a_non_critical_one_ignored() {
    let critical = rcgen_leaf(|p| {
        let mut e = rcgen::CustomExtension::from_oid_content(
            &[1, 3, 6, 1, 4, 1, 99999, 1],
            vec![0x05, 0x00],
        );
        e.set_criticality(true);
        p.custom_extensions.push(e);
    });
    assert!(matches!(
        check_leaf(&critical),
        Err(PairingError::TeamCertUnknownCriticalExtension)
    ));
    let plain = rcgen_leaf(|p| {
        p.custom_extensions
            .push(rcgen::CustomExtension::from_oid_content(
                &[1, 3, 6, 1, 4, 1, 99999, 1],
                vec![0x05, 0x00],
            ));
    });
    check_leaf(&plain).unwrap();
}

#[test]
fn malformed_der_is_refused() {
    let good = leaf(TeamRole::Robot, 2, 5);
    let mut trailing = good.clone();
    trailing.push(0);
    for bad in [&trailing[..], &good[..good.len() - 1], &[], &[0x30, 0x00]] {
        assert!(matches!(
            check_leaf(bad),
            Err(PairingError::TeamCertMalformed(_))
        ));
    }
}

#[test]
fn signatures_bind_leaf_and_ca() {
    let ca2 = check_team_ca(&ca_der(2)).unwrap();
    // A leaf signed by another key under the right names.
    let forged = issue_leaf(&leaf_params(TeamRole::Robot, 2, 5), &sk(0x99)).unwrap();
    assert!(matches!(
        verify_leaf_issued_by(&forged, &ca2),
        Err(PairingError::BadSignature(CertKind::TeamCert))
    ));
    // A leaf of generation 1 checked against the generation 2 CA.
    assert!(matches!(
        verify_leaf_issued_by(&leaf(TeamRole::Robot, 1, 5), &ca2),
        Err(PairingError::TeamCertIssuerMismatch)
    ));
    // A CA whose self-signature does not verify.
    let mut ca = ca_der(2);
    let last = ca.len() - 1;
    ca[last] ^= 1;
    assert!(matches!(
        check_team_ca(&ca),
        Err(PairingError::BadSignature(CertKind::TeamCert))
    ));
}

#[test]
fn team_ca_profile_refusals() {
    assert!(matches!(
        check_team_ca(&leaf(TeamRole::Robot, 2, 5)),
        Err(PairingError::TeamNameMismatch)
    ));
    let kp = rcgen_key(0x42);
    let mut p = rcgen::CertificateParams::new(Vec::<String>::new()).unwrap();
    p.distinguished_name = rcgen::DistinguishedName::new();
    p.distinguished_name
        .push(rcgen::DnType::CommonName, "ca.g2.acme.cerulion.invalid");
    p.key_usages = vec![rcgen::KeyUsagePurpose::KeyCertSign];
    p.is_ca = rcgen::IsCa::Ca(rcgen::BasicConstraints::Unconstrained);
    let unconstrained = p.self_signed(&kp).unwrap().der().to_vec();
    assert!(matches!(
        check_team_ca(&unconstrained),
        Err(PairingError::TeamCaProfile(_))
    ));
    p.is_ca = rcgen::IsCa::Ca(rcgen::BasicConstraints::Constrained(0));
    p.key_usages = vec![rcgen::KeyUsagePurpose::DigitalSignature];
    let no_cert_sign = p.self_signed(&kp).unwrap().der().to_vec();
    assert!(matches!(
        check_team_ca(&no_cert_sign),
        Err(PairingError::TeamCaProfile(_))
    ));
    p.key_usages = vec![rcgen::KeyUsagePurpose::KeyCertSign];
    let good = p.self_signed(&kp).unwrap().der().to_vec();
    assert_eq!(check_team_ca(&good).unwrap().generation, 2);
}

// ============================================================================
// Trust bundle
// ============================================================================

fn oracle_bundle_payload(b: &TeamTrustBundle) -> Vec<u8> {
    let mut o = Vec::new();
    let le32 = |n: usize| (n as u32).to_le_bytes();
    let domain = b"cerulion-pairing:team-trust:v1";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&b.version.to_le_bytes());
    o.extend_from_slice(&b.team_id.0);
    o.extend_from_slice(&le32(b.team_short.as_str().len()));
    o.extend_from_slice(b.team_short.as_str().as_bytes());
    o.extend_from_slice(&b.generation.to_le_bytes());
    o.extend_from_slice(&b.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&le32(b.issuing_ca_der.len()));
    o.extend_from_slice(&b.issuing_ca_der);
    o.extend_from_slice(&le32(b.retired.len()));
    for r in &b.retired {
        o.extend_from_slice(&r.generation.to_le_bytes());
        o.extend_from_slice(&le32(r.ca_der.len()));
        o.extend_from_slice(&r.ca_der);
    }
    o.extend_from_slice(&le32(b.robot_leaves.len()));
    for l in &b.robot_leaves {
        o.extend_from_slice(&le32(l.len()));
        o.extend_from_slice(l);
    }
    o.extend_from_slice(&b.issuer_key.0);
    o
}

#[test]
fn bundle_signing_bytes_match_the_documented_layout() {
    use cerulion_pairing::format::SignedPayload;
    let b = bundle(3, vec![leaf(TeamRole::Robot, 3, 5)]);
    assert_eq!(b.signing_payload(), oracle_bundle_payload(&b));
    assert_eq!(TEAM_TRUST_DOMAIN, b"cerulion-pairing:team-trust:v1");
}

#[test]
fn a_signed_bundle_verifies_and_round_trips() {
    let (root_set, _, _) = roots();
    let signed = sign_bundle(bundle(
        3,
        vec![leaf(TeamRole::Robot, 3, 5), leaf(TeamRole::Robot, 3, 6)],
    ));
    let v = verify_team_trust_bundle(&root_set, &signed, T_NOW).unwrap();
    assert_eq!(v.get(), &signed);
    let bytes = postcard::to_allocvec(&signed).unwrap();
    let back: SignedTeamTrustBundle = postcard::from_bytes(&bytes).unwrap();
    assert_eq!(back, signed);
    // Generation 1 with no retired generations and no robots.
    verify_team_trust_bundle(&root_set, &sign_bundle(bundle(1, vec![])), T_NOW).unwrap();
}

#[test]
fn bundle_chain_refusals() {
    let (root_set, int, signed_int) = roots();
    // Tampered after signing.
    let mut tampered = sign_bundle(bundle(2, vec![]));
    tampered.bundle.issued_at_ns += 1;
    assert!(matches!(
        verify_team_trust_bundle(&root_set, &tampered, T_NOW),
        Err(PairingError::BadSignature(CertKind::TeamTrustBundle))
    ));
    // Signed by a key that is not the intermediate.
    let mut b = bundle(2, vec![]);
    b.issuer_key = pk(&sk(0x55));
    let wrong = b.sign(&sk(0x55), signed_int.clone());
    assert!(matches!(
        verify_team_trust_bundle(&root_set, &wrong, T_NOW),
        Err(PairingError::IssuerMismatch(CertKind::TeamTrustBundle))
    ));
    // An intermediate the pinned roots did not sign.
    let rogue_int = make_intermediate(pk(&int), wide_validity(), ISSUED).sign_by_roots(&[&sk(3)]);
    let rogue = bundle(2, vec![]).sign(&int, rogue_int);
    assert!(matches!(
        verify_team_trust_bundle(&root_set, &rogue, T_NOW),
        Err(PairingError::RootThresholdNotMet { .. })
    ));
    // Issued in the future.
    let mut b = bundle(2, vec![]);
    b.issued_at_ns = T_NOW + 3_600_000_000_000;
    let future = b.sign(&int, signed_int);
    assert!(matches!(
        verify_team_trust_bundle(&root_set, &future, T_NOW),
        Err(PairingError::IssuedInFuture {
            kind: CertKind::TeamTrustBundle,
            ..
        })
    ));
}

#[test]
fn bundle_structure_refusals() {
    let (root_set, _, _) = roots();
    let check = |b: TeamTrustBundle| verify_team_trust_bundle(&root_set, &sign_bundle(b), T_NOW);

    let mut b = bundle(3, vec![]);
    b.issuing_ca_der = ca_der(2);
    assert!(matches!(
        check(b),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let mut b = bundle(3, vec![]);
    b.retired.reverse();
    assert!(matches!(
        check(b),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let mut b = bundle(3, vec![]);
    b.retired.insert(
        0,
        RetiredGeneration {
            generation: 3,
            ca_der: ca_der(3),
        },
    );
    assert!(matches!(
        check(b),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let mut b = bundle(3, vec![]);
    b.retired[0].ca_der = ca_der(1);
    assert!(matches!(
        check(b),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let b = bundle(3, vec![leaf(TeamRole::Operator, 3, 5)]);
    assert!(matches!(
        check(b),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let b = bundle(3, vec![leaf(TeamRole::Robot, 2, 5)]);
    assert!(matches!(
        check(b),
        Err(PairingError::TeamCertIssuerMismatch)
    ));

    let forged = issue_leaf(&leaf_params(TeamRole::Robot, 3, 5), &sk(0x99)).unwrap();
    assert!(matches!(
        check(bundle(3, vec![forged])),
        Err(PairingError::BadSignature(CertKind::TeamCert))
    ));

    let mut b = bundle(3, vec![]);
    b.issuing_ca_der = vec![0x30, 0x00];
    assert!(matches!(check(b), Err(PairingError::TeamCertMalformed(_))));
}

// ============================================================================
// Status list and freshness
// ============================================================================

fn oracle_list_payload(l: &TeamStatusList) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:team-status:v1";
    o.extend_from_slice(&(domain.len() as u32).to_le_bytes());
    o.extend_from_slice(domain);
    o.push(1);
    o.extend_from_slice(&l.version.to_le_bytes());
    o.extend_from_slice(&l.team_id.0);
    o.extend_from_slice(&l.seq.to_le_bytes());
    o.extend_from_slice(&l.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&(l.entries.len() as u32).to_le_bytes());
    for e in &l.entries {
        o.extend_from_slice(&e.device.0);
        o.push(match e.status {
            DeviceStatus::Stolen => 1,
            DeviceStatus::Revoked => 2,
        });
        o.extend_from_slice(&e.entry_seq.to_le_bytes());
        o.extend_from_slice(&e.since_ns.to_le_bytes());
    }
    o.extend_from_slice(&l.issuer_key.0);
    o
}

fn oracle_freshness_payload(f: &StatusFreshness) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:team-status:v1";
    o.extend_from_slice(&(domain.len() as u32).to_le_bytes());
    o.extend_from_slice(domain);
    o.push(2);
    o.extend_from_slice(&f.version.to_le_bytes());
    o.extend_from_slice(&f.team_id.0);
    o.extend_from_slice(&f.asker.0);
    o.extend_from_slice(&f.nonce);
    o.extend_from_slice(&f.list_seq.to_le_bytes());
    o.extend_from_slice(&f.service_time_ns.to_le_bytes());
    o.extend_from_slice(&f.issuer_key.0);
    o
}

#[test]
fn status_signing_bytes_match_the_documented_layout() {
    use cerulion_pairing::format::SignedPayload;
    let l = status_list(
        5,
        vec![
            entry(5, DeviceStatus::Stolen, 4),
            entry(6, DeviceStatus::Revoked, 5),
        ],
    );
    assert_eq!(l.signing_payload(), oracle_list_payload(&l));
    let f = freshness();
    assert_eq!(f.signing_payload(), oracle_freshness_payload(&f));
    assert_eq!((STATUS_LIST_KIND, STATUS_FRESHNESS_KIND), (1, 2));
}

#[test]
fn a_signed_status_list_verifies_and_finds_entries() {
    let (root_set, int, signed_int) = roots();
    let l = status_list(
        5,
        vec![
            entry(5, DeviceStatus::Stolen, 4),
            entry(6, DeviceStatus::Revoked, 5),
        ],
    );
    let signed = l.sign(&int, signed_int.clone());
    let v = verify_status_list(&root_set, &signed, T_NOW).unwrap();
    let got = v.get().list.entry_for(&fp(5)).unwrap();
    assert_eq!((got.status, got.entry_seq), (DeviceStatus::Stolen, 4));
    assert_eq!(v.get().list.entry_for(&fp(7)), None);
    // An empty list at sequence 0 is valid.
    verify_status_list(
        &root_set,
        &status_list(0, vec![]).sign(&int, signed_int),
        T_NOW,
    )
    .unwrap();
}

#[test]
fn status_list_refusals() {
    let (root_set, int, signed_int) = roots();
    let check =
        |l: TeamStatusList| verify_status_list(&root_set, &l.sign(&int, signed_int.clone()), T_NOW);

    let mut l = status_list(
        5,
        vec![
            entry(5, DeviceStatus::Stolen, 4),
            entry(6, DeviceStatus::Stolen, 4),
        ],
    );
    l.entries.reverse();
    assert!(matches!(
        check(l),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let mut l = status_list(5, vec![entry(5, DeviceStatus::Stolen, 4)]);
    l.entries.push(entry(5, DeviceStatus::Revoked, 5));
    assert!(matches!(
        check(l),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let l = status_list(5, vec![entry(5, DeviceStatus::Stolen, 6)]);
    assert!(matches!(
        check(l),
        Err(PairingError::TeamStatementInvalid(_))
    ));

    let mut signed = status_list(5, vec![]).sign(&int, signed_int.clone());
    signed.list.seq = 6;
    assert!(matches!(
        verify_status_list(&root_set, &signed, T_NOW),
        Err(PairingError::BadSignature(CertKind::TeamStatusList))
    ));
}

#[test]
fn a_status_signature_never_verifies_as_freshness() {
    let (root_set, int, signed_int) = roots();
    let list = status_list(4, vec![]).sign(&int, signed_int.clone());
    let mut f = freshness().sign(&int, signed_int);
    f.signature = list.signature;
    assert!(matches!(
        verify_status_freshness(&root_set, &f, &team_id(), &fp(20), &[0x5A; 16], T_NOW),
        Err(PairingError::BadSignature(CertKind::StatusFreshness))
    ));
}

#[test]
fn freshness_binds_team_asker_and_nonce() {
    let (root_set, int, signed_int) = roots();
    let f = freshness().sign(&int, signed_int);
    let v =
        verify_status_freshness(&root_set, &f, &team_id(), &fp(20), &[0x5A; 16], T_NOW).unwrap();
    assert_eq!(v.get().freshness.list_seq, 4);
    assert!(matches!(
        verify_status_freshness(&root_set, &f, &TeamId([1; 32]), &fp(20), &[0x5A; 16], T_NOW),
        Err(PairingError::TeamMismatch)
    ));
    assert!(matches!(
        verify_status_freshness(&root_set, &f, &team_id(), &fp(21), &[0x5A; 16], T_NOW),
        Err(PairingError::FreshnessMismatch)
    ));
    assert!(matches!(
        verify_status_freshness(&root_set, &f, &team_id(), &fp(20), &[0x5B; 16], T_NOW),
        Err(PairingError::FreshnessMismatch)
    ));
}

// ============================================================================
// Anti-rollback
// ============================================================================

fn verified_bundle(generation: u8, issued_at_ns: u64) -> Verified<SignedTeamTrustBundle> {
    let (root_set, _, _) = roots();
    let mut b = bundle(generation, vec![]);
    b.issued_at_ns = issued_at_ns;
    verify_team_trust_bundle(&root_set, &sign_bundle(b), T_NOW).unwrap()
}

#[test]
fn bundle_rollback_orders_by_generation_then_issuance() {
    let held = BundleHighWater {
        team_id: team_id(),
        generation: 2,
        issued_at_ns: ISSUED,
    };
    let d = |g, t| decide_bundle(Some(&held), &verified_bundle(g, t));
    assert_eq!(
        decide_bundle(None, &verified_bundle(1, ISSUED)).unwrap(),
        RollbackDecision::Apply
    );
    assert_eq!(d(3, ISSUED - 1).unwrap(), RollbackDecision::Apply);
    assert_eq!(d(2, ISSUED + 1).unwrap(), RollbackDecision::Apply);
    assert_eq!(d(2, ISSUED).unwrap(), RollbackDecision::NoChange);
    assert!(matches!(
        d(2, ISSUED - 1),
        Err(PairingError::TeamRollback {
            what: "bundle issuance time",
            ..
        })
    ));
    assert!(matches!(
        d(1, ISSUED + 1),
        Err(PairingError::TeamRollback {
            what: "bundle generation",
            held: 2,
            offered: 1
        })
    ));
    let other = BundleHighWater {
        team_id: TeamId([1; 32]),
        ..held
    };
    assert!(matches!(
        decide_bundle(Some(&other), &verified_bundle(3, ISSUED)),
        Err(PairingError::TeamMismatch)
    ));
}

#[test]
fn status_rollback_orders_by_sequence() {
    let (root_set, int, signed_int) = roots();
    let v = |seq| {
        verify_status_list(
            &root_set,
            &status_list(seq, vec![]).sign(&int, signed_int.clone()),
            T_NOW,
        )
        .unwrap()
    };
    let held = StatusHighWater {
        team_id: team_id(),
        seq: 5,
    };
    assert_eq!(
        decide_status_list(None, &v(0)).unwrap(),
        RollbackDecision::Apply
    );
    assert_eq!(
        decide_status_list(Some(&held), &v(6)).unwrap(),
        RollbackDecision::Apply
    );
    assert_eq!(
        decide_status_list(Some(&held), &v(5)).unwrap(),
        RollbackDecision::NoChange
    );
    assert!(matches!(
        decide_status_list(Some(&held), &v(4)),
        Err(PairingError::TeamRollback {
            held: 5,
            offered: 4,
            ..
        })
    ));
    let other = StatusHighWater {
        team_id: TeamId([1; 32]),
        seq: 0,
    };
    assert!(matches!(
        decide_status_list(Some(&other), &v(6)),
        Err(PairingError::TeamMismatch)
    ));
}

#[test]
fn clock_must_not_be_below_the_high_water_or_the_build_time() {
    check_clock(100, 100, 50).unwrap();
    check_clock(100, 50, 100).unwrap();
    assert!(matches!(
        check_clock(99, 100, 50),
        Err(PairingError::RollbackDetected {
            now_ns: 99,
            high_water_ns: 100
        })
    ));
    assert!(matches!(
        check_clock(99, 50, 100),
        Err(PairingError::RollbackDetected {
            now_ns: 99,
            high_water_ns: 100
        })
    ));
}

#[test]
fn signed_statements_are_deterministic() {
    assert_eq!(
        sign_bundle(bundle(2, vec![])),
        sign_bundle(bundle(2, vec![]))
    );
    let (_, int, signed_int) = roots();
    assert_eq!(
        freshness().sign(&int, signed_int.clone()),
        freshness().sign(&int, signed_int)
    );
}
