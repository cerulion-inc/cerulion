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

use super::team_auth::TeamAuth;

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
    /// over [`Self::machine_key`], trusting this generation and every retired
    /// one.
    pub fn machine(&self, role: TeamRole, machine: u8) -> TeamAuth {
        let key = self.machine_key(machine);
        let leaf = self.leaf(role, &key);
        TeamAuth::new(&key, &leaf, &self.ca_der, &self.retired).expect("valid test credentials")
    }
}
