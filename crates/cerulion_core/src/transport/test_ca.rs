// SPDX-License-Identifier: AGPL-3.0-only
//! A test team issuer: team issuing CAs and machine leaves in the
//! `cerulion_pairing::team` profile, built in memory from fixed seeds, so any
//! networked test can open authenticated sessions without files.
//!
//! Every key comes from SHA-256 over a label, the team name and the
//! generation (or the machine seed byte), so two calls with the same inputs
//! give the same keys, and two teams or two generations never share one.

use cerulion_pairing::identity::MachineKey;
use cerulion_pairing::team::{
    issue_leaf, issue_team_ca, LeafParams, RetiredGeneration, TeamRole, TeamShort,
};
use ed25519_dalek::SigningKey;
use sha2::{Digest, Sha256};

use super::team_auth::{TeamAnchors, TeamAuth};

/// One generation of a test team: its issuing CA key and certificate, and the
/// retired generations below it.
pub struct TestTeam {
    team: TeamShort,
    generation: u32,
    ca_key: SigningKey,
    ca_der: Vec<u8>,
    retired: Vec<RetiredGeneration>,
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

fn derive(label: &str, team: &str, n: u32) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(b"cerulion-test-ca:");
    h.update(label.as_bytes());
    h.update([0]);
    h.update(team.as_bytes());
    h.update(n.to_le_bytes());
    h.finalize().into()
}

impl TestTeam {
    /// Team `team` at `generation`, with no retired generations.
    ///
    /// # Panics
    ///
    /// When `team` is not a valid team short name.
    pub fn new(team: &str, generation: u32) -> TestTeam {
        let short = TeamShort::new(team).expect("a valid team short name");
        let ca_key = SigningKey::from_bytes(&derive("ca", team, generation));
        let ca_der =
            issue_team_ca(&short, generation, &ca_key, now_ns()).expect("issue the test team CA");
        TestTeam {
            team: short,
            generation,
            ca_key,
            ca_der,
            retired: Vec::new(),
        }
    }

    /// The next generation of this team, carrying this one (and every
    /// generation this one retired) as retired.
    pub fn rotated(&self) -> TestTeam {
        let mut next = TestTeam::new(self.team.as_str(), self.generation + 1);
        next.retired.push(self.retired());
        next.retired.extend(self.retired.iter().cloned());
        next
    }

    /// This generation as a retired entry of a later one.
    pub fn retired(&self) -> RetiredGeneration {
        RetiredGeneration {
            generation: self.generation,
            ca_der: self.ca_der.clone(),
        }
    }

    /// The issuing CA certificate, DER.
    pub fn ca_der(&self) -> &[u8] {
        &self.ca_der
    }

    /// The generation.
    pub fn generation(&self) -> u32 {
        self.generation
    }

    /// The retired generations this one carries.
    pub fn retired_list(&self) -> &[RetiredGeneration] {
        &self.retired
    }

    /// The issuing CA certificate of the next generation, DER: the CA
    /// [`Self::rotated`] signs with (same key, issued now).
    pub fn next_ca_der(&self) -> Vec<u8> {
        TestTeam::new(self.team.as_str(), self.generation + 1).ca_der
    }

    /// Anchors with this generation, every retired one and `next` as the
    /// next generation's CA.
    pub fn anchors<'a>(&'a self, next: Option<&'a [u8]>) -> TeamAnchors<'a> {
        TeamAnchors {
            issuing_ca_der: &self.ca_der,
            retired: &self.retired,
            next_ca_der: next,
        }
    }

    /// A leaf for `key` in `role`, issued now.
    pub fn leaf(&self, role: TeamRole, key: &MachineKey) -> Vec<u8> {
        issue_leaf(
            &LeafParams {
                team: self.team.clone(),
                generation: self.generation,
                role,
                device_key: cerulion_pairing::format::PublicKey(key.public_key()),
                account: None,
                issued_at_ns: now_ns(),
            },
            &self.ca_key,
        )
        .expect("issue a test leaf")
    }

    /// The machine key a test machine numbered `machine` uses in this team.
    pub fn machine_key(&self, machine: u8) -> MachineKey {
        MachineKey::from_seed(derive("machine", self.team.as_str(), u32::from(machine)))
    }

    /// Team credentials for machine `machine` in `role`: a leaf issued now
    /// over [`Self::machine_key`], trusting this generation, every retired
    /// one and the next one.
    pub fn machine(&self, role: TeamRole, machine: u8) -> TeamAuth {
        let key = self.machine_key(machine);
        let leaf = self.leaf(role, &key);
        let next = self.next_ca_der();
        TeamAuth::new(&key, &leaf, self.anchors(Some(&next))).expect("valid test credentials")
    }
}

/// The `(header length, content length)` of the DER element at `der[0..]`.
fn der_header(der: &[u8]) -> (usize, usize) {
    match der[1] {
        n @ 0..=0x7f => (2, n as usize),
        0x81 => (3, der[2] as usize),
        0x82 => (4, ((der[2] as usize) << 8) | der[3] as usize),
        other => panic!("unexpected DER length byte {other:#x}"),
    }
}

impl TestTeam {
    /// A leaf whose subject names `role` of generation `as_generation` while
    /// its issuer is this generation's CA and this CA's key signs it: the
    /// certificate a holder of this CA key could mint for another generation.
    /// `as_generation` must print with as many digits as this generation.
    ///
    /// # Panics
    ///
    /// When the digit counts differ or the issued certificate does not have
    /// the expected shape.
    pub fn forged_leaf(&self, as_generation: u32, role: TeamRole, key: &MachineKey) -> Vec<u8> {
        let claimed = cerulion_pairing::team::ca_common_name(as_generation, &self.team);
        let actual = cerulion_pairing::team::ca_common_name(self.generation, &self.team);
        assert_eq!(claimed.len(), actual.len(), "same digit count");
        let mut der = issue_leaf(
            &LeafParams {
                team: self.team.clone(),
                generation: as_generation,
                role,
                device_key: cerulion_pairing::format::PublicKey(key.public_key()),
                account: None,
                issued_at_ns: now_ns(),
            },
            &self.ca_key,
        )
        .expect("issue a leaf");
        // Certificate = SEQUENCE { tbsCertificate, signatureAlgorithm, signature BIT STRING }.
        let (outer, _) = der_header(&der);
        let (tbs_h, tbs_len) = der_header(&der[outer..]);
        let tbs = outer..outer + tbs_h + tbs_len;
        let hits: Vec<usize> = der[tbs.clone()]
            .windows(claimed.len())
            .enumerate()
            .filter(|(_, w)| *w == claimed.as_bytes())
            .map(|(i, _)| i + outer)
            .collect();
        assert_eq!(hits.len(), 1, "the issuer name appears once in the TBS");
        let at = hits[0];
        der[at..at + actual.len()].copy_from_slice(actual.as_bytes());
        use ed25519_dalek::Signer as _;
        let signature = self.ca_key.sign(&der[tbs]).to_bytes();
        let len = der.len();
        der[len - 64..].copy_from_slice(&signature);
        der
    }

    /// The PEM text of every DER in `ders` under `label`, concatenated and
    /// base64-encoded the way zenoh's `*_base64` keys take it.
    pub fn pem_base64(label: &str, ders: &[&[u8]]) -> String {
        use base64::Engine as _;
        let b64 = base64::engine::general_purpose::STANDARD;
        let mut pem = String::new();
        for der in ders {
            let body = b64.encode(der);
            pem.push_str(&format!("-----BEGIN {label}-----\n"));
            for chunk in body.as_bytes().chunks(64) {
                pem.push_str(std::str::from_utf8(chunk).expect("base64 is ASCII"));
                pem.push('\n');
            }
            pem.push_str(&format!("-----END {label}-----\n"));
        }
        b64.encode(pem)
    }
}
