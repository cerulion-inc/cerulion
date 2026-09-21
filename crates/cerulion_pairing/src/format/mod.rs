// SPDX-License-Identifier: MIT OR Apache-2.0
//! Wire/storage **formats** for the identity layer: the primitive types, the
//! certificate / grant / epoch structures, their **canonical deterministic
//! signing bytes**, and serde container (de)serialization.
//!
//! ## What is signed vs. how it is stored
//!
//! Signatures cover the hand-rolled canonical [`canonical::CanonicalWriter`]
//! bytes ([`SignedPayload::signing_payload`]), **not** the serde/postcard output.
//! This decouples "what is signed" (a stable, domain-separated, length-prefixed
//! byte layout, pinned by byte-exact test vectors) from "how it is stored/sent"
//! (postcard, free to evolve). A serialization change can therefore never alter
//! a signature, and a signing-byte-layout change fails a vector test loudly.

pub mod canonical;
mod types;

pub use types::{
    AccessGrant, AccessListEpoch, Delegation, DeviceCert, Grant, IntermediateCert, PrincipalKind,
    Role, RootSet, RootSignature, Scope, SignedAccessGrant, SignedDeviceCert, SignedEpoch,
    SignedGrant, SignedIntermediateCert, Validity,
};

use serde::{Deserialize, Serialize};

/// Format version stamped into every signable structure. Bumped on a
/// breaking canonical-layout change (each type also carries its own domain tag).
pub const FORMAT_VERSION: u16 = 1;

/// An ed25519 public key. This 32-byte value **is** the transport identity — it
/// doubles as the iroh `EndpointId`.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PublicKey(pub [u8; 32]);

/// An ed25519 signature.
///
/// Serialized as a length-prefixed byte string (serde does not derive for
/// `[u8; 64]`), which postcard encodes compactly.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Signature(pub [u8; 64]);

/// A stable account identifier. Distinct from an account's signing *key(s)* so
/// that account-key rotation does not change the identifier — a rotated account
/// never strands the robots that trust it.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AccountId(pub [u8; 32]);

/// A stable robot identifier.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RobotId(pub [u8; 32]);

impl core::fmt::Debug for PublicKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "PublicKey({})", crate::crypto::to_hex(&self.0))
    }
}
impl core::fmt::Debug for Signature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Signature({})", crate::crypto::to_hex(&self.0))
    }
}
impl core::fmt::Debug for AccountId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "AccountId({})", crate::crypto::to_hex(&self.0))
    }
}
impl core::fmt::Debug for RobotId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "RobotId({})", crate::crypto::to_hex(&self.0))
    }
}

// serde does not derive for `[u8; 64]`; hand-impl over a length-prefixed byte
// string so the container format stays compact and unambiguous.
impl Serialize for Signature {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_bytes(&self.0)
    }
}
impl<'de> Deserialize<'de> for Signature {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        struct V;
        impl serde::de::Visitor<'_> for V {
            type Value = Signature;
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str("a 64-byte ed25519 signature")
            }
            fn visit_bytes<E: serde::de::Error>(self, v: &[u8]) -> Result<Signature, E> {
                let arr: [u8; 64] = v
                    .try_into()
                    .map_err(|_| E::invalid_length(v.len(), &self))?;
                Ok(Signature(arr))
            }
        }
        d.deserialize_bytes(V)
    }
}

/// A signable structure: it can produce its canonical signing bytes and be
/// verified against an ed25519 public key.
pub trait SignedPayload {
    /// The domain-separation tag written first into the signing bytes. Prevents
    /// a signature over one type from being replayed as another.
    const DOMAIN: &'static [u8];

    /// The exact, deterministic, domain-separated bytes that a signature covers.
    fn signing_payload(&self) -> Vec<u8>;
}
