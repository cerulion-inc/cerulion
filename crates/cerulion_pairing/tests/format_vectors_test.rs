// SPDX-License-Identifier: MIT OR Apache-2.0
//! Byte-exact canonical signing-bytes vectors + serde round-trips.
//!
//! Each vector is built by an **independent** hand-serializer (raw byte ops that
//! mirror the documented layout) — NOT by calling the production encoder — so a
//! drift in `signing_payload()` fails loudly. A format change must update BOTH
//! the encoder and these oracles (a deliberate, visible act).

mod common;
use cerulion_pairing::format::*;

// ---- independent hand-serializers (the oracles) ----------------------------

fn le32(n: usize) -> [u8; 4] {
    (n as u32).to_le_bytes()
}

fn oracle_device_cert(c: &DeviceCert) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:device-cert:v1";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&c.version.to_le_bytes());
    o.extend_from_slice(&c.device_key.0);
    o.extend_from_slice(&c.account.0);
    o.push(c.principal_kind as u8);
    o.extend_from_slice(&c.scope.role.0.to_le_bytes());
    o.extend_from_slice(&c.scope.caps.to_le_bytes());
    o.extend_from_slice(&c.validity.not_before_ns.to_le_bytes());
    o.extend_from_slice(&c.validity.not_after_ns.to_le_bytes());
    o.extend_from_slice(&c.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&c.issuer_key.0);
    o
}

fn oracle_grant(g: &Grant) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:grant:v1";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&g.version.to_le_bytes());
    o.extend_from_slice(&g.subject.0);
    o.extend_from_slice(&g.robot.0);
    o.extend_from_slice(&g.scope.role.0.to_le_bytes());
    o.extend_from_slice(&g.scope.caps.to_le_bytes());
    o.push(g.principal_kind as u8);
    o.push(g.delegation_depth);
    o.extend_from_slice(&g.validity.not_before_ns.to_le_bytes());
    o.extend_from_slice(&g.validity.not_after_ns.to_le_bytes());
    o.extend_from_slice(&g.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&g.issuer.0);
    o.extend_from_slice(&g.issuer_key.0);
    o
}

fn oracle_epoch(e: &AccessListEpoch) -> Vec<u8> {
    let mut o = Vec::new();
    // The domain was bumped to :v2 when `revoked_devices` was folded in.
    let domain = b"cerulion-pairing:access-epoch:v2";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&e.version.to_le_bytes());
    o.extend_from_slice(&e.robot.0);
    o.extend_from_slice(&e.epoch.to_le_bytes());
    o.extend_from_slice(&le32(e.revoked_accounts.len()));
    for a in &e.revoked_accounts {
        o.extend_from_slice(&a.0);
    }
    // The revoked-device set rides the SAME payload (length prefix
    // then each 32-byte key), appended after the accounts collection.
    o.extend_from_slice(&le32(e.revoked_devices.len()));
    for d in &e.revoked_devices {
        o.extend_from_slice(&d.0);
    }
    o.extend_from_slice(&e.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&e.issuer_key.0);
    o
}

fn oracle_intermediate(i: &IntermediateCert) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:intermediate-cert:v1";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&i.version.to_le_bytes());
    o.extend_from_slice(&i.intermediate_key.0);
    o.extend_from_slice(&i.validity.not_before_ns.to_le_bytes());
    o.extend_from_slice(&i.validity.not_after_ns.to_le_bytes());
    o.extend_from_slice(&i.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&i.max_scope.role.0.to_le_bytes());
    o.extend_from_slice(&i.max_scope.caps.to_le_bytes());
    o
}

// Independent hand-serializer for the owner-signed access grant.
fn oracle_access_grant(g: &AccessGrant) -> Vec<u8> {
    let mut o = Vec::new();
    let domain = b"cerulion-pairing:access-grant:v1";
    o.extend_from_slice(&le32(domain.len()));
    o.extend_from_slice(domain);
    o.extend_from_slice(&g.version.to_le_bytes());
    o.extend_from_slice(&g.subject.0);
    o.extend_from_slice(&g.robot.0);
    o.extend_from_slice(&g.scope.role.0.to_le_bytes());
    o.extend_from_slice(&g.scope.caps.to_le_bytes());
    o.push(g.principal_kind as u8);
    o.extend_from_slice(&g.validity.not_before_ns.to_le_bytes());
    o.extend_from_slice(&g.validity.not_after_ns.to_le_bytes());
    o.extend_from_slice(&g.issued_at_ns.to_le_bytes());
    o.extend_from_slice(&g.owner.0);
    o.extend_from_slice(&g.owner_device_key.0);
    o
}

fn fixed_access_grant() -> AccessGrant {
    AccessGrant {
        version: 1,
        subject: AccountId([0xB1; 32]),
        robot: RobotId([0xB2; 32]),
        scope: Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
        },
        principal_kind: PrincipalKind::Human,
        validity: Validity {
            not_before_ns: 0x100,
            not_after_ns: 0x200,
        },
        issued_at_ns: 0x150,
        owner: AccountId([0xB3; 32]),
        owner_device_key: PublicKey([0xB4; 32]),
    }
}

fn fixed_intermediate() -> IntermediateCert {
    IntermediateCert {
        version: 1,
        intermediate_key: PublicKey([0x77; 32]),
        validity: Validity {
            not_before_ns: 0x10,
            not_after_ns: 0x20,
        },
        issued_at_ns: 0x15,
        max_scope: Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_TELEOP,
        },
    }
}

fn fixed_device_cert() -> DeviceCert {
    DeviceCert {
        version: 1,
        device_key: PublicKey([0xAA; 32]),
        account: AccountId([0xBB; 32]),
        principal_kind: PrincipalKind::Human,
        scope: Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
        },
        validity: Validity {
            not_before_ns: 0x100,
            not_after_ns: 0x200,
        },
        issued_at_ns: 0x150,
        issuer_key: PublicKey([0xCC; 32]),
    }
}

// ---- vectors ---------------------------------------------------------------

#[test]
fn device_cert_signing_bytes_match_hand_oracle() {
    let c = fixed_device_cert();
    let got = c.signing_payload();
    let want = oracle_device_cert(&c);
    assert_eq!(got, want, "device cert signing bytes drifted from the spec");
    // 4(len) + 31(domain) + 2(ver) + 32(dev) + 32(acct) + 1(princ)
    //   + 2+8(scope) + 8+8(validity) + 8(issued) + 32(issuer) = 168.
    assert_eq!(got.len(), 168);
    // First 4 bytes = the domain length prefix (31 = len of the domain tag).
    assert_eq!(&got[..4], &31u32.to_le_bytes()[..]);
}

#[test]
fn device_cert_first_bytes_are_the_length_prefixed_domain() {
    let c = fixed_device_cert();
    let got = c.signing_payload();
    let domain = b"cerulion-pairing:device-cert:v1";
    assert_eq!(&got[..4], &(domain.len() as u32).to_le_bytes());
    assert_eq!(&got[4..4 + domain.len()], domain);
}

#[test]
fn grant_signing_bytes_match_hand_oracle() {
    let g = Grant {
        version: 1,
        subject: AccountId([0x11; 32]),
        robot: RobotId([0x22; 32]),
        scope: Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
        principal_kind: PrincipalKind::Machine,
        delegation_depth: 1,
        validity: Validity {
            not_before_ns: 7,
            not_after_ns: 9_999,
        },
        issued_at_ns: 42,
        issuer: AccountId([0x33; 32]),
        issuer_key: PublicKey([0x44; 32]),
    };
    assert_eq!(g.signing_payload(), oracle_grant(&g));
}

#[test]
fn epoch_signing_bytes_match_hand_oracle_including_the_revoked_collection() {
    let e = AccessListEpoch {
        version: 1,
        robot: RobotId([0x55; 32]),
        epoch: 7,
        revoked_accounts: vec![AccountId([0x01; 32]), AccountId([0x02; 32])],
        // A non-empty device set exercises the appended collection.
        revoked_devices: vec![PublicKey([0x0A; 32]), PublicKey([0x0B; 32])],
        issued_at_ns: 123,
        issuer_key: PublicKey([0x66; 32]),
    };
    assert_eq!(e.signing_payload(), oracle_epoch(&e));

    // Empty revoked-account list => count prefix 0, no elements (the device set
    // stays non-empty, so the payloads still differ).
    let e0 = AccessListEpoch {
        revoked_accounts: vec![],
        ..e.clone()
    };
    assert_eq!(e0.signing_payload(), oracle_epoch(&e0));
    assert_ne!(e.signing_payload(), e0.signing_payload());

    // Dropping the DEVICE set (accounts unchanged) also changes the
    // payload — the device collection is genuinely part of the signed bytes.
    let e_no_dev = AccessListEpoch {
        revoked_devices: vec![],
        ..e.clone()
    };
    assert_eq!(e_no_dev.signing_payload(), oracle_epoch(&e_no_dev));
    assert_ne!(e.signing_payload(), e_no_dev.signing_payload());
}

#[test]
fn intermediate_signing_bytes_start_with_its_own_domain_and_have_expected_length() {
    let i = IntermediateCert {
        version: 1,
        intermediate_key: PublicKey([0x77; 32]),
        validity: Validity {
            not_before_ns: 1,
            not_after_ns: 2,
        },
        issued_at_ns: 3,
        max_scope: Scope::OWNER_FULL,
    };
    let got = i.signing_payload();
    let domain = b"cerulion-pairing:intermediate-cert:v1";
    assert_eq!(&got[..4], &(domain.len() as u32).to_le_bytes());
    assert_eq!(&got[4..4 + domain.len()], domain);
    // 4(len) + 37(domain) + 2(ver) + 32(key) + 8+8(validity) + 8(issued)
    //   + 2+8(max_scope) = 109.
    assert_eq!(got.len(), 109);
}

#[test]
fn intermediate_cert_signing_bytes_match_hand_oracle() {
    let i = fixed_intermediate();
    assert_eq!(
        i.signing_payload(),
        oracle_intermediate(&i),
        "intermediate cert signing bytes drifted from the spec (reorder/drop?)"
    );
    assert_eq!(i.signing_payload().len(), 109);
}

#[test]
fn every_intermediate_cert_field_is_covered_by_the_signing_bytes() {
    let base = fixed_intermediate();
    let base_bytes = base.signing_payload();
    let mutations: Vec<IntermediateCert> = vec![
        IntermediateCert {
            version: 2,
            ..base.clone()
        },
        IntermediateCert {
            intermediate_key: PublicKey([0x01; 32]),
            ..base.clone()
        },
        IntermediateCert {
            validity: Validity {
                not_before_ns: 1,
                not_after_ns: base.validity.not_after_ns,
            },
            ..base.clone()
        },
        IntermediateCert {
            validity: Validity {
                not_before_ns: base.validity.not_before_ns,
                not_after_ns: 1,
            },
            ..base.clone()
        },
        IntermediateCert {
            issued_at_ns: 0,
            ..base.clone()
        },
        IntermediateCert {
            max_scope: Scope {
                role: Role::VIEWER,
                caps: base.max_scope.caps,
            },
            ..base.clone()
        },
        IntermediateCert {
            max_scope: Scope {
                role: base.max_scope.role,
                caps: 0,
            },
            ..base.clone()
        },
    ];
    for m in mutations {
        assert_ne!(
            m.signing_payload(),
            base_bytes,
            "an intermediate-cert field mutation left the signing bytes unchanged"
        );
    }
}

#[test]
fn domains_are_distinct_across_types_so_signatures_cannot_be_confused() {
    // A device cert and a grant with overlapping content must not collide: the
    // domain tag makes their signing bytes disjoint.
    let dev = fixed_device_cert().signing_payload();
    let grant = Grant {
        version: 1,
        subject: AccountId([0xAA; 32]),
        robot: RobotId([0xBB; 32]),
        scope: Scope {
            role: Role::OPERATOR,
            caps: 6,
        },
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: Validity {
            not_before_ns: 0x100,
            not_after_ns: 0x200,
        },
        issued_at_ns: 0x150,
        issuer: AccountId([0xCC; 32]),
        issuer_key: PublicKey([0xDD; 32]),
    }
    .signing_payload();
    assert_ne!(&dev[..4 + 31], &grant[..4 + 25]);
    assert!(dev.starts_with(&{
        let d = b"cerulion-pairing:device-cert:v1";
        let mut v = (d.len() as u32).to_le_bytes().to_vec();
        v.extend_from_slice(d);
        v
    }));
    assert!(grant.starts_with(&{
        let d = b"cerulion-pairing:grant:v1";
        let mut v = (d.len() as u32).to_le_bytes().to_vec();
        v.extend_from_slice(d);
        v
    }));
}

#[test]
fn every_device_cert_field_is_covered_by_the_signing_bytes() {
    // Flipping any single field must change the signing bytes (no field is
    // silently excluded from the signature).
    let base = fixed_device_cert();
    let base_bytes = base.signing_payload();

    let mut mutations: Vec<DeviceCert> = Vec::new();
    mutations.push(DeviceCert {
        version: 2,
        ..base.clone()
    });
    mutations.push(DeviceCert {
        device_key: PublicKey([0x01; 32]),
        ..base.clone()
    });
    mutations.push(DeviceCert {
        account: AccountId([0x02; 32]),
        ..base.clone()
    });
    mutations.push(DeviceCert {
        principal_kind: PrincipalKind::Machine,
        ..base.clone()
    });
    mutations.push(DeviceCert {
        scope: Scope {
            role: Role::VIEWER,
            caps: base.scope.caps,
        },
        ..base.clone()
    });
    mutations.push(DeviceCert {
        scope: Scope {
            role: base.scope.role,
            caps: 0,
        },
        ..base.clone()
    });
    mutations.push(DeviceCert {
        validity: Validity {
            not_before_ns: 1,
            not_after_ns: base.validity.not_after_ns,
        },
        ..base.clone()
    });
    mutations.push(DeviceCert {
        validity: Validity {
            not_before_ns: base.validity.not_before_ns,
            not_after_ns: 1,
        },
        ..base.clone()
    });
    mutations.push(DeviceCert {
        issued_at_ns: 0,
        ..base.clone()
    });
    mutations.push(DeviceCert {
        issuer_key: PublicKey([0x03; 32]),
        ..base.clone()
    });

    for m in mutations {
        assert_ne!(
            m.signing_payload(),
            base_bytes,
            "a field mutation left the signing bytes unchanged"
        );
    }
}

// ---- serde container round-trips (postcard) --------------------------------

#[test]
fn signed_containers_round_trip_through_postcard() {
    let root0 = common::sk(1);
    let root1 = common::sk(2);
    let inter = common::sk(3);

    let signed_int = common::make_intermediate(common::pk(&inter), common::wide_validity(), 5)
        .sign_by_roots(&[&root0, &root1]);
    let signed_dev = common::make_device_cert(
        common::pk(&common::sk(9)),
        common::acct(0x0A),
        common::pk(&inter),
        common::op_scope(),
        common::wide_validity(),
        5,
    )
    .sign(&inter);
    let signed_grant = common::make_grant(
        common::acct(0x0A),
        common::rob(0x0B),
        common::op_scope(),
        0,
        common::acct(0x0C),
        common::pk(&inter),
        common::wide_validity(),
        5,
    )
    .sign(&inter);
    let signed_epoch = common::make_epoch(
        common::rob(0x0B),
        3,
        vec![common::acct(0x0A)],
        common::pk(&inter),
        6,
    )
    .sign(&inter);

    // Round-trip each and assert full structural equality (Signature's 64-byte
    // custom serde is exercised here).
    let r_int: SignedIntermediateCert =
        postcard::from_bytes(&postcard::to_stdvec(&signed_int).unwrap()).unwrap();
    assert_eq!(r_int, signed_int);
    let r_dev: SignedDeviceCert =
        postcard::from_bytes(&postcard::to_stdvec(&signed_dev).unwrap()).unwrap();
    assert_eq!(r_dev, signed_dev);
    let r_grant: SignedGrant =
        postcard::from_bytes(&postcard::to_stdvec(&signed_grant).unwrap()).unwrap();
    assert_eq!(r_grant, signed_grant);
    let r_epoch: SignedEpoch =
        postcard::from_bytes(&postcard::to_stdvec(&signed_epoch).unwrap()).unwrap();
    assert_eq!(r_epoch, signed_epoch);
}

// ---- owner-signed access grant ---------------------------------------------

#[test]
fn access_grant_signing_bytes_match_hand_oracle() {
    let g = fixed_access_grant();
    let got = g.signing_payload();
    assert_eq!(
        got,
        oracle_access_grant(&g),
        "access-grant signing bytes drifted from the spec (reorder/drop?)"
    );
    // 4(len) + 32(domain) + 2(ver) + 32(subject) + 32(robot) + 2+8(scope)
    //   + 1(princ) + 8+8(validity) + 8(issued) + 32(owner) + 32(owner_key) = 201.
    assert_eq!(got.len(), 201);
    let domain = b"cerulion-pairing:access-grant:v1";
    assert_eq!(&got[..4], &(domain.len() as u32).to_le_bytes());
    assert_eq!(&got[4..4 + domain.len()], domain);
}

#[test]
fn access_grant_domain_is_distinct_from_the_cloud_grant_domain() {
    // The owner-signed access grant must never share signing bytes with a
    // cloud-issued `Grant` — the distinct domain tag makes a cross-replay impossible
    // even if every account/robot/scope field coincided.
    let ag = fixed_access_grant().signing_payload();
    assert!(ag.starts_with(&{
        let d = b"cerulion-pairing:access-grant:v1";
        let mut v = (d.len() as u32).to_le_bytes().to_vec();
        v.extend_from_slice(d);
        v
    }));
    // The cloud-grant domain prefix must NOT appear at the head of an access grant.
    let cloud_prefix = {
        let d = b"cerulion-pairing:grant:v1";
        let mut v = (d.len() as u32).to_le_bytes().to_vec();
        v.extend_from_slice(d);
        v
    };
    assert!(!ag.starts_with(&cloud_prefix));
}

#[test]
fn every_access_grant_field_is_covered_by_the_signing_bytes() {
    // Flipping any single field must change the signing bytes — no field silently
    // excluded from the owner's signature.
    let base = fixed_access_grant();
    let base_bytes = base.signing_payload();
    let mutations: Vec<AccessGrant> = vec![
        AccessGrant {
            version: 2,
            ..base.clone()
        },
        AccessGrant {
            subject: AccountId([0x01; 32]),
            ..base.clone()
        },
        AccessGrant {
            robot: RobotId([0x02; 32]),
            ..base.clone()
        },
        AccessGrant {
            scope: Scope {
                role: Role::VIEWER,
                caps: base.scope.caps,
            },
            ..base.clone()
        },
        AccessGrant {
            scope: Scope {
                role: base.scope.role,
                caps: 0,
            },
            ..base.clone()
        },
        AccessGrant {
            principal_kind: PrincipalKind::Machine,
            ..base.clone()
        },
        AccessGrant {
            validity: Validity {
                not_before_ns: 1,
                not_after_ns: base.validity.not_after_ns,
            },
            ..base.clone()
        },
        AccessGrant {
            validity: Validity {
                not_before_ns: base.validity.not_before_ns,
                not_after_ns: 1,
            },
            ..base.clone()
        },
        AccessGrant {
            issued_at_ns: 0,
            ..base.clone()
        },
        AccessGrant {
            owner: AccountId([0x03; 32]),
            ..base.clone()
        },
        AccessGrant {
            owner_device_key: PublicKey([0x04; 32]),
            ..base.clone()
        },
    ];
    for (i, m) in mutations.iter().enumerate() {
        assert_ne!(
            m.signing_payload(),
            base_bytes,
            "access-grant field mutation #{i} left the signing bytes unchanged"
        );
    }
}

#[test]
fn signed_access_grant_round_trips_through_postcard() {
    let owner_key = common::sk(0x1A);
    let signed = fixed_access_grant().sign(&owner_key);
    let r: SignedAccessGrant =
        postcard::from_bytes(&postcard::to_stdvec(&signed).unwrap()).unwrap();
    assert_eq!(
        r, signed,
        "SignedAccessGrant did not round-trip through postcard"
    );
}

#[test]
fn a_real_owner_signature_verifies_against_the_access_grant_payload_and_tamper_fails() {
    use ed25519_dalek::Verifier;
    let owner_key = common::sk(0x1A);
    // The grant must name the SAME owner device key the signer holds (the verifier
    // asserts this at chain time; here we cross-check the raw signature).
    let mut g = fixed_access_grant();
    g.owner_device_key = common::pk(&owner_key);
    let signed = g.clone().sign(&owner_key);

    let vk = owner_key.verifying_key();
    let sig = ed25519_dalek::Signature::from_bytes(&signed.signature.0);
    // Verifies against exactly the canonical signing bytes.
    assert!(vk.verify(&g.signing_payload(), &sig).is_ok());
    // A tampered subject → the SAME signature no longer verifies (the field is
    // covered; a byte-flip is caught).
    let tampered = AccessGrant {
        subject: AccountId([0xEE; 32]),
        ..g.clone()
    };
    assert!(vk.verify(&tampered.signing_payload(), &sig).is_err());
    // ...and not some other bytes.
    assert!(vk.verify(b"not the payload", &sig).is_err());
}

#[test]
fn a_real_signature_verifies_against_the_signing_payload() {
    use ed25519_dalek::Verifier;
    let inter = common::sk(3);
    let signed = common::make_device_cert(
        common::pk(&common::sk(9)),
        common::acct(0x0A),
        common::pk(&inter),
        common::op_scope(),
        common::wide_validity(),
        5,
    )
    .sign(&inter);

    let vk = inter.verifying_key();
    let sig = ed25519_dalek::Signature::from_bytes(&signed.signature.0);
    // The signature covers exactly the canonical signing bytes.
    assert!(vk.verify(&signed.cert.signing_payload(), &sig).is_ok());
    // ...and not some other bytes.
    assert!(vk.verify(b"not the payload", &sig).is_err());
}

// ---- The epoch-SYNC wire artifact ---------------------------------

/// Build the sync artifact (a real signed intermediate + a real signed
/// epoch, both chained to seed 3) the desk carries and the robot applies.
fn epoch_sync_fixture() -> cerulion_pairing::verify::EpochSyncWire {
    let inter = common::sk(3);
    cerulion_pairing::verify::EpochSyncWire::new(
        common::make_intermediate(common::pk(&inter), common::wide_validity(), 5)
            .sign_by_roots(&[&common::sk(1), &common::sk(2)]),
        common::make_epoch_with_devices(
            common::rob(0x0B),
            9,
            vec![common::acct(0x0A)],
            vec![common::pk(&common::sk(20))],
            common::pk(&inter),
            6,
        )
        .sign(&inter),
    )
}

#[test]
fn epoch_sync_wire_postcard_round_trips_both_halves() {
    let w = epoch_sync_fixture();
    let decoded = cerulion_pairing::verify::EpochSyncWire::from_postcard(&w.to_postcard().unwrap())
        .expect("a well-formed blob decodes");
    // Full structural equality: the intermediate AND the epoch (including its
    // revoked-account + revoked-device sets and the 64-byte Signature serde).
    assert_eq!(decoded, w);
    // Against HAND-written expectations of every carried field — not merely a
    // self-compare: each value must survive the trip exactly.
    assert_eq!(decoded.signed_epoch.epoch_data.epoch, 9);
    assert_eq!(
        decoded.signed_epoch.epoch_data.revoked_accounts,
        vec![common::acct(0x0A)]
    );
    assert_eq!(
        decoded.signed_epoch.epoch_data.revoked_devices,
        vec![common::pk(&common::sk(20))]
    );
    assert_eq!(decoded.signed_epoch.epoch_data.robot, common::rob(0x0B));
    assert_eq!(
        decoded.intermediate.cert.intermediate_key,
        common::pk(&common::sk(3))
    );
}

#[test]
fn epoch_sync_wire_encoding_is_deterministic() {
    // Two encodes of the same artifact are byte-identical (the desk cache and the
    // control frame must not drift run-to-run).
    let w = epoch_sync_fixture();
    assert_eq!(w.to_postcard().unwrap(), w.to_postcard().unwrap());
}

#[test]
fn epoch_sync_wire_refuses_a_malformed_blob_loudly() {
    // Garbage, truncation, and an EMPTY blob must all be loud errors — never a
    // silently-default artifact the robot would then try to apply.
    for bad in [
        b"not a postcard blob at all".to_vec(),
        epoch_sync_fixture().to_postcard().unwrap()[..12].to_vec(),
        Vec::new(),
    ] {
        assert!(
            cerulion_pairing::verify::EpochSyncWire::from_postcard(&bad).is_err(),
            "a malformed epoch-sync blob must never decode (fail-open)"
        );
    }
}

/// The envelope carries a VERSION discriminant, and it is the FIRST
/// thing on the wire. postcard is not self-describing, so without it a future
/// artifact with a changed shape would decode silently (and wrongly) on an older
/// peer. An unknown version must be a LOUD refusal naming both versions.
#[test]
fn epoch_sync_wire_refuses_an_unknown_envelope_version_loudly() {
    use cerulion_pairing::verify::{EpochSyncWire, EPOCH_SYNC_WIRE_VERSION};

    // `new` stamps the current version (the constructor is the only way a producer
    // should build one).
    let w = epoch_sync_fixture();
    assert_eq!(w.version, EPOCH_SYNC_WIRE_VERSION);

    // A future artifact: same fields, a bumped version. Hand-built, then encoded
    // through the SAME serde — so the only difference is the discriminant.
    let future = EpochSyncWire {
        version: EPOCH_SYNC_WIRE_VERSION + 1,
        intermediate: w.intermediate.clone(),
        signed_epoch: w.signed_epoch.clone(),
    };
    let err = EpochSyncWire::from_postcard(&future.to_postcard().unwrap())
        .expect_err("an unknown envelope version must be refused, never re-interpreted");
    let msg = err.to_string();
    assert!(
        msg.contains(&(EPOCH_SYNC_WIRE_VERSION + 1).to_string())
            && msg.contains(&EPOCH_SYNC_WIRE_VERSION.to_string()),
        "the refusal must name the found AND expected versions: {msg}"
    );

    // The version is FIRST on the wire: the encoding starts with the varint-encoded
    // version, so a peer reads it before touching any credential bytes.
    assert_eq!(
        w.to_postcard().unwrap()[0],
        EPOCH_SYNC_WIRE_VERSION as u8,
        "the version discriminant must lead the encoding"
    );

    // The refusal leads with the shared needle, so a
    // desk classifies a version skew as an UPGRADE-shaped outcome instead of "this
    // epoch is forged / your clock is skewed". Reverting the message to prose that
    // does not carry the needle fails here AND silently re-misdirects every operator.
    assert!(
        msg.contains(cerulion_pairing::verify::EPOCH_VERSION_UNSUPPORTED_NEEDLE),
        "the version refusal must carry the shared classification needle: {msg}"
    );

    // The version guard must fire from the peeked leading varint,
    // before the rest of the struct decodes — a v2 whose later fields changed layout
    // must get the loud version refusal, not a cryptic postcard error from those
    // fields. Blob = varint version 2 + garbage that cannot decode as the v1 tail.
    let v2_with_garbage_tail: &[u8] = &[2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    let err = cerulion_pairing::verify::EpochSyncWire::from_postcard(v2_with_garbage_tail)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains(cerulion_pairing::verify::EPOCH_VERSION_UNSUPPORTED_NEEDLE)
            && err.contains('2'),
        "a future-version blob with an undecodable tail must be refused BY VERSION, \
         not by a tail decode error: {err}"
    );
}

/// The leading `u16` discriminant is NOT a magic:
/// it cannot tell a versioned envelope apart from a hypothetical UNVERSIONED one whose
/// first bytes happen to parse as a version. That artifact never existed (version 1 is
/// the first shape the epoch sync ever produced), and the guarantee that DOES hold is the one
/// that matters: an unversioned blob is refused LOUDLY rather than mis-read as a valid
/// artifact. Pins that, so the const's doc cannot overclaim.
#[test]
fn an_unversioned_epoch_envelope_is_refused_loudly() {
    use cerulion_pairing::verify::EpochSyncWire;

    let w = epoch_sync_fixture();
    // The exact pre-version encoding: the two payload fields, in order, with NO
    // leading discriminant (a 2-tuple encodes identically to a 2-field struct).
    let unversioned = postcard::to_stdvec(&(&w.intermediate, &w.signed_epoch))
        .expect("the pre-version shape encodes");
    assert_ne!(
        unversioned,
        w.to_postcard().unwrap(),
        "the unversioned encoding must genuinely differ from the versioned one"
    );
    let err = EpochSyncWire::from_postcard(&unversioned)
        .expect_err("an unversioned envelope must be refused, never mis-read as valid");
    // Loud: a real error message, not an empty/defaulted success.
    assert!(
        !err.to_string().is_empty(),
        "the refusal must carry a diagnosable message"
    );
}

/// Postcard's `from_bytes` IGNORES trailing bytes, which would let a
/// future artifact carrying an appended field decode "successfully" here while
/// silently dropping the new data. Strict decode refuses any remainder.
#[test]
fn epoch_sync_wire_refuses_trailing_bytes_rather_than_ignoring_them() {
    use cerulion_pairing::verify::EpochSyncWire;

    let good = epoch_sync_fixture().to_postcard().unwrap();
    // Sanity: the exact bytes still decode (the strict path did not break the happy
    // case) — the anti-tautology half.
    assert!(EpochSyncWire::from_postcard(&good).is_ok());

    let mut appended = good.clone();
    appended.extend_from_slice(&[0xAA, 0xBB, 0xCC]);
    let err = EpochSyncWire::from_postcard(&appended)
        .expect_err("trailing bytes must be refused, never silently ignored");
    assert!(
        err.to_string().contains("trailing"),
        "the refusal must name the cause: {err}"
    );
}

/// The one desk-wide epoch-cache DIRECTORY rule, which
/// every desk path resolves through — `cerulion connect`'s binary, `cerulion-netd`'s
/// `WanRegistry::from_env`, and the account-page cache WRITER. Hand oracle.
///
/// Before this, netd honored a `CERULION_NETD_EPOCH_DIR` override that the connect
/// path did NOT, so relocating the cache made an epoch cached by one path invisible to
/// the other with NO symptom at either end (the writer says "cached", the other reader
/// says "nothing cached"). One resolver, one env var, one answer.
#[test]
fn resolve_epoch_dir_is_the_one_desk_wide_rule() {
    use cerulion_pairing::verify::{resolve_epoch_dir, EPOCH_DIR_ENV};
    use std::path::{Path, PathBuf};

    // The env var is DESK-wide, not netd-scoped: a name only one path honored is the
    // divergence this whole rule exists to prevent.
    assert_eq!(EPOCH_DIR_ENV, "CERULION_EPOCH_DIR");

    let key = Path::new("/home/u/.cerulion/desk.key");

    // An explicit override WINS over the key-file sibling — on every path.
    assert_eq!(
        resolve_epoch_dir(Some("/etc/cerulion/epochs"), Some(key)),
        Some(PathBuf::from("/etc/cerulion/epochs"))
    );
    // Blank / whitespace-only is NOT an override (an unset-but-exported env var must
    // not silently relocate the cache to the current directory).
    assert_eq!(
        resolve_epoch_dir(Some("   "), Some(key)),
        Some(PathBuf::from("/home/u/.cerulion/epochs"))
    );
    // No override ⇒ the SIBLING `epochs/` next to the desk key (the `~/.cerulion`
    // convention `device.cert` and `grants/` already follow).
    assert_eq!(
        resolve_epoch_dir(None, Some(key)),
        Some(PathBuf::from("/home/u/.cerulion/epochs"))
    );
    assert_eq!(
        resolve_epoch_dir(None, Some(Path::new("/opt/keys/desk.key"))),
        Some(PathBuf::from("/opt/keys/epochs"))
    );
    // A bare key filename (no parent component) ⇒ `epochs/` in the current directory.
    assert_eq!(
        resolve_epoch_dir(None, Some(Path::new("desk.key"))),
        Some(PathBuf::from("epochs"))
    );
    // An ephemeral desk key with no override ⇒ no cache location at all (a no-op push,
    // never a refusal).
    assert_eq!(resolve_epoch_dir(None, None), None);
    // …but an override alone is enough, even with no key file.
    assert_eq!(
        resolve_epoch_dir(Some("/tmp/e"), None),
        Some(PathBuf::from("/tmp/e"))
    );
}
