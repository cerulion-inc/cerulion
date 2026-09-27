// SPDX-License-Identifier: AGPL-3.0-only
//! Fixed public protocol vectors shared with the account web app.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde_json::Value;
use sha2::{Digest, Sha256};

fn fixtures() -> Value {
    serde_json::from_str(include_str!("fixtures/pairing-v1.json")).unwrap()
}

fn unhex(value: &str) -> Vec<u8> {
    assert!(value.len().is_multiple_of(2));
    value
        .as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}

fn bytes(value: &Value, field: &str) -> Vec<u8> {
    unhex(value[field].as_str().unwrap())
}

#[test]
fn supabase_pairing_identity_uses_the_pinned_namespace_and_network_order_bytes() {
    let fixture = fixtures();
    let namespace = bytes(&fixture, "accountNamespaceHex");
    assert_eq!(namespace, b"cerulion:supabase-account:v1\0");
    for account in fixture["accounts"].as_array().unwrap() {
        let uuid = unhex(&account["uuid"].as_str().unwrap().replace('-', ""));
        assert_eq!(uuid, bytes(account, "uuidBytesHex"));
        assert_eq!(uuid.len(), 16);
        let digest = Sha256::new()
            .chain_update(&namespace)
            .chain_update(uuid)
            .finalize();
        assert_eq!(digest.as_slice(), bytes(account, "accountHex"));
        assert_eq!(URL_SAFE_NO_PAD.encode(digest), account["accountBase64url"]);
    }
}

#[test]
fn proof_of_possession_matches_the_independent_five_field_oracle() {
    let fixture = fixtures();
    let proof = &fixture["proofOfPossession"];
    let actual = cerulion_pairing::pop::pop_signing_message(
        proof["purpose"].as_str().unwrap(),
        &bytes(proof, "accountHex").try_into().unwrap(),
        &bytes(proof, "publicKeyHex").try_into().unwrap(),
        proof["challenge"].as_str().unwrap(),
    );
    assert_eq!(actual, bytes(proof, "payloadHex"));
}

#[test]
fn strict_ed25519_pins_small_order_and_cofactored_equation_boundaries() {
    let fixture = fixtures();
    for vector in fixture["ed25519"].as_array().unwrap() {
        let name = vector["name"].as_str().unwrap();
        let key_bytes = bytes(vector, "publicKeyHex");
        let signature_bytes = bytes(vector, "signatureHex");
        let key = key_bytes
            .as_slice()
            .try_into()
            .ok()
            .and_then(|bytes| VerifyingKey::from_bytes(bytes).ok());
        let signature = Signature::try_from(signature_bytes.as_slice()).ok();
        let message = bytes(vector, "messageHex");
        let accepted = key
            .as_ref()
            .zip(signature.as_ref())
            .is_some_and(|(key, signature)| key.verify_strict(&message, signature).is_ok());
        assert_eq!(
            accepted,
            vector["strictAccepted"].as_bool().unwrap(),
            "{name}"
        );
        if let Some(decodes) = vector["publicKeyDecodes"].as_bool() {
            assert_eq!(key.is_some(), decodes, "{name}: key decoding");
        }
        if let Some(weak) = vector["publicKeySmallOrder"].as_bool() {
            assert_eq!(
                key.as_ref().map(VerifyingKey::is_weak),
                Some(weak),
                "{name}: key order"
            );
        }
        if let Some(seed) = vector["testOnlySeedHex"].as_str() {
            let signing = SigningKey::from_bytes(&unhex(seed).try_into().unwrap());
            assert_eq!(
                signing.verifying_key().as_bytes().as_slice(),
                key_bytes.as_slice(),
                "{name}"
            );
            assert_eq!(Some(signing.sign(&message)), signature, "{name}");
        }
    }
}
