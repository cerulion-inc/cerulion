// SPDX-License-Identifier: AGPL-3.0-only
//! The account database — a stateless API over SQLite (`rusqlite`, bundled).
//!
//! The full resource model is created on open so every feature (robots,
//! grants/epochs, service accounts) builds on a stable schema; the endpoints exercise
//! the **active** tables: `users`, `devices`, `sessions`, `device_codes`,
//! `magic_links`, `oauth_flows`, `robots`, `access_list_epochs`. The rest (`orgs`,
//! `org_members`, `grants`, `service_accounts`) are created but **dormant**
//! — no endpoint reads or writes them.
//!
//! One `Connection` behind a `Mutex` (SQLite calls are microseconds; the guard is
//! never held across an `.await`). Only SHA-256 token *hashes* are stored, never
//! plaintext secrets.

use std::sync::{Arc, Mutex, MutexGuard};

use rusqlite::{params, Connection, OptionalExtension};

use crate::device_code::{
    insert_with_collision_retry, AttemptOutcome, DeviceCodeSnapshot, DeviceCodeState, RetryGiveUp,
};
use crate::error::{AccountdError, Result};
use crate::session::SessionSnapshot;

const SCHEMA: &str = include_str!("schema.sql");

/// Which session token-hash column a lookup keys on. An enum (not a `&str`) so
/// the SQL is a hardcoded literal per variant — the column name is never
/// string-interpolated into a query (defense against a future injection rot).
enum SessionHashColumn {
    Session,
    Refresh,
}

/// A device-code INSERT failure, split so a retryable `user_code` collision is
/// distinguished from every fatal error.
enum DeviceCodeInsertError {
    /// The `user_code` UNIQUE constraint was violated — retry with a fresh code.
    UserCodeCollision,
    /// Any other failure (incl. a `device_code_hash` primary-key collision) — fatal.
    Other(AccountdError),
}

/// Whether a rusqlite error is a UNIQUE-constraint violation specifically on
/// `device_codes.user_code` (the only collision the `device/start` allocator
/// retries — a `device_code_hash` PK collision, astronomically unlikely on a
/// 256-bit token, stays fatal).
fn is_user_code_collision(err: &rusqlite::Error) -> bool {
    matches!(
        err,
        rusqlite::Error::SqliteFailure(e, Some(msg))
            if e.code == rusqlite::ErrorCode::ConstraintViolation
                && msg.contains("device_codes.user_code")
    )
}

/// Whether a rusqlite error is a UNIQUE/constraint violation (matched on the error
/// KIND, not message text). Used to map a race-lost robot INSERT — a concurrent
/// registration of the SAME transport key from ANOTHER connection that slipped
/// between our SELECT pre-check and INSERT (the per-process `Mutex` only serializes
/// this process's connection) — to the same loud [`AccountdError::Conflict`] the
/// pre-checked cross-account path returns, instead of a bare 500.
fn is_constraint_violation(err: &rusqlite::Error) -> bool {
    matches!(
        err,
        rusqlite::Error::SqliteFailure(e, _)
            if e.code == rusqlite::ErrorCode::ConstraintViolation
    )
}

/// Idempotent, additive schema migrations run at open (AFTER `CREATE TABLE IF NOT
/// EXISTS`). SQLite's `CREATE TABLE IF NOT EXISTS` never ADDS a column to an already-
/// existing table, so a table created by an older build (one whose
/// `access_list_epochs` has fewer columns) does NOT gain a new column just by re-running the schema —
/// every migration here is a guarded `ALTER TABLE ... ADD COLUMN` that runs only when
/// the column is missing.
fn run_migrations(conn: &Connection) -> Result<()> {
    // `access_list_epochs.revoked_devices`. On a FRESH DB the CREATE TABLE
    // already added it (column_exists → true → no-op); on an older DB whose table
    // predates the column, this ALTER adds it so the INSERT/SELECTs naming it work.
    if !column_exists(conn, "access_list_epochs", "revoked_devices")? {
        conn.execute(
            "ALTER TABLE access_list_epochs ADD COLUMN revoked_devices BLOB NOT NULL DEFAULT x''",
            [],
        )?;
    }
    Ok(())
}

/// Whether `table` has a column named `column` (via `PRAGMA table_info`).
fn column_exists(conn: &Connection, table: &str, column: &str) -> Result<bool> {
    // The table name is a fixed internal literal (never user input), so the format! is
    // safe (PRAGMA does not accept a bound parameter for the table name).
    let mut stmt = conn.prepare(&format!("PRAGMA table_info({table})"))?;
    let names = stmt
        .query_map([], |row| row.get::<_, String>(1))?
        .collect::<rusqlite::Result<Vec<String>>>()?;
    Ok(names.iter().any(|c| c == column))
}

/// Encode a slice of 32-byte ids as the flat `access_list_epochs` blob (32*N bytes,
/// count implicit from the length).
fn encode_ids(ids: &[[u8; 32]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ids.len() * 32);
    for id in ids {
        out.extend_from_slice(id);
    }
    out
}

/// Decode the flat `access_list_epochs` blob back into 32-byte ids. A length that is
/// not a multiple of 32 is a corrupt row — a LOUD [`AccountdError::Internal`], never
/// a silent truncation.
fn decode_ids(blob: &[u8]) -> Result<Vec<[u8; 32]>> {
    if !blob.len().is_multiple_of(32) {
        return Err(AccountdError::Internal(format!(
            "access-list-epoch id blob length {} is not a multiple of 32 (corrupt row)",
            blob.len()
        )));
    }
    // `as_chunks` yields `&[u8; 32]` directly, so the staging buffer the slice-typed
    // chunks forced is gone; the length guard above already refused a partial tail.
    Ok(blob.as_chunks::<32>().0.to_vec())
}

/// The single conflict message both the SELECT pre-check and the INSERT
/// constraint-violation branch of [`Db::register_robot`] return, so a robot
/// transport key owned by another account surfaces the SAME loud 409 whether the
/// pre-check catches it or a concurrent registration wins the race.
const ROBOT_KEY_CONFLICT_MSG: &str =
    "robot transport key is already registered to a different account \
     (one transport key maps to exactly one robot owner)";

/// A user row.
#[derive(Clone, Debug)]
pub struct UserRow {
    /// Opaque user id (primary key).
    pub user_id: String,
    /// Identity provider (`google` | `github` | `email` | `supabase`).
    pub provider: String,
    /// Provider-scoped subject (the external identity's stable id).
    pub subject: String,
    /// Email, if the provider supplied one.
    pub email: Option<String>,
    /// The stable 32-byte `AccountId` (distinct from any signing key).
    pub account_id: [u8; 32],
    /// Principal kind discriminant (1 = Human, 2 = Machine).
    pub principal_kind: u8,
}

/// A device row.
#[derive(Clone, Debug)]
pub struct DeviceRow {
    /// Opaque device id.
    pub device_id: String,
    /// The device (transport) public key = iroh EndpointId.
    pub public_key: [u8; 32],
    /// The owning account.
    pub account_id: [u8; 32],
    /// Principal kind discriminant.
    pub principal_kind: u8,
    /// Registration time (Unix ns).
    pub created_at_ns: u64,
    /// Whether the device was revoked.
    pub revoked: bool,
}

/// A robot row (login-gated ownership at install).
#[derive(Clone, Debug)]
pub struct RobotRow {
    /// The stable 32-byte `RobotId` minted at registration.
    pub robot_id: [u8; 32],
    /// The robot's human name (its stamped hostname — the robot identity).
    pub hostname: String,
    /// The account that owns the robot (the installer's account).
    pub owner_account_id: [u8; 32],
    /// The owning org, if any (bound verbatim — org membership is NOT
    /// validated here).
    pub org_id: Option<String>,
    /// The robot's transport public key (its iroh EndpointId); one key = one robot.
    pub robot_transport_key: [u8; 32],
    /// Registration time (Unix ns).
    pub created_at_ns: u64,
}

/// A stored access-list revocation epoch. The account service owns the
/// per-robot epoch; the robot syncs the latest one + applies it via
/// `TrustStore::apply_epoch`. Ids are decoded from the flat 32*N-byte blobs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EpochRow {
    /// The monotonic epoch number.
    pub epoch: u64,
    /// The revoked account ids (32 bytes each).
    pub revoked_accounts: Vec<[u8; 32]>,
    /// The revoked device (transport) keys (32 bytes each).
    pub revoked_devices: Vec<[u8; 32]>,
    /// Issuance time (Unix ns).
    pub issued_at_ns: u64,
}

/// One target of a robot access-list revocation: a whole account OR one
/// device (transport) key. Both are 32-byte ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RevocationTarget {
    /// Revoke a whole account's access to the robot.
    Account([u8; 32]),
    /// Revoke ONE device key from the robot (the account's other devices keep access).
    Device([u8; 32]),
}

/// A session identity + its validity snapshot.
#[derive(Clone, Debug)]
pub struct SessionLookup {
    /// The session row id.
    pub id: String,
    /// The owning user id.
    pub user_id: String,
    /// The validity snapshot (for the pure session/refresh decisions).
    pub snapshot: SessionSnapshot,
}

/// The account database handle (cheap to clone — shares one connection).
#[derive(Clone)]
pub struct Db {
    conn: Arc<Mutex<Connection>>,
}

impl std::fmt::Debug for Db {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Db(sqlite)")
    }
}

impl Db {
    /// Open (or create) a file-backed database and apply the schema.
    pub fn open(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let conn = Connection::open(path)?;
        Self::init(conn)
    }

    /// Open a private in-memory database (hermetic tests).
    pub fn open_in_memory() -> Result<Self> {
        Self::init(Connection::open_in_memory()?)
    }

    fn init(conn: Connection) -> Result<Self> {
        conn.execute_batch("PRAGMA journal_mode = WAL; PRAGMA foreign_keys = ON;")?;
        conn.execute_batch(SCHEMA)?;
        run_migrations(&conn)?;
        Ok(Db {
            conn: Arc::new(Mutex::new(conn)),
        })
    }

    fn lock(&self) -> Result<MutexGuard<'_, Connection>> {
        self.conn
            .lock()
            .map_err(|_| AccountdError::Internal("db mutex poisoned".into()))
    }

    // -- users ---------------------------------------------------------------

    /// Find a user by (provider, subject), or create one with a fresh
    /// `AccountId`. The email is refreshed on an existing user.
    pub fn upsert_user_by_identity(
        &self,
        provider: &str,
        subject: &str,
        email: Option<&str>,
        principal_kind: u8,
        now_ns: u64,
    ) -> Result<UserRow> {
        let conn = self.lock()?;
        let existing: Option<UserRow> = conn
            .query_row(
                "SELECT user_id, provider, subject, email, account_id, principal_kind \
                 FROM users WHERE provider = ?1 AND subject = ?2",
                params![provider, subject],
                Self::map_user,
            )
            .optional()?;
        if let Some(mut row) = existing {
            if email.is_some() && email != row.email.as_deref() {
                conn.execute(
                    "UPDATE users SET email = ?1 WHERE user_id = ?2",
                    params![email, row.user_id],
                )?;
                row.email = email.map(str::to_string);
            }
            return Ok(row);
        }
        let user_id = crate::rng::opaque_token()?;
        let account_id = crate::rng::id_32()?;
        conn.execute(
            "INSERT INTO users (user_id, provider, subject, email, account_id, principal_kind, created_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                user_id,
                provider,
                subject,
                email,
                &account_id[..],
                principal_kind as i64,
                now_ns as i64
            ],
        )?;
        Ok(UserRow {
            user_id,
            provider: provider.to_string(),
            subject: subject.to_string(),
            email: email.map(str::to_string),
            account_id,
            principal_kind,
        })
    }

    /// Find an external identity without creating it.
    pub fn user_by_identity(&self, provider: &str, subject: &str) -> Result<Option<UserRow>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT user_id, provider, subject, email, account_id, principal_kind \
                 FROM users WHERE provider = ?1 AND subject = ?2",
                params![provider, subject],
                Self::map_user,
            )
            .optional()?)
    }

    /// Upsert an identity and authorize its pending device code atomically.
    ///
    /// An invalid, expired, or already-consumed code returns `None` and rolls
    /// back the identity write, so a rejected device exchange cannot create an
    /// account as a side effect.
    pub fn upsert_user_for_device_code(
        &self,
        provider: &str,
        subject: &str,
        email: Option<&str>,
        principal_kind: u8,
        user_code: &str,
        now_ns: u64,
    ) -> Result<Option<UserRow>> {
        let mut conn = self.lock()?;
        let tx = conn.transaction()?;
        let code_exists: Option<i64> = tx
            .query_row(
                "SELECT 1 FROM device_codes \
                 WHERE user_code = ?1 AND state = 'pending' AND expires_at_ns > ?2",
                params![user_code, now_ns as i64],
                |row| row.get(0),
            )
            .optional()?;
        if code_exists.is_none() {
            return Ok(None);
        }

        let existing: Option<UserRow> = tx
            .query_row(
                "SELECT user_id, provider, subject, email, account_id, principal_kind \
                 FROM users WHERE provider = ?1 AND subject = ?2",
                params![provider, subject],
                Self::map_user,
            )
            .optional()?;
        let user = if let Some(mut row) = existing {
            if email.is_some() && email != row.email.as_deref() {
                tx.execute(
                    "UPDATE users SET email = ?1 WHERE user_id = ?2",
                    params![email, row.user_id],
                )?;
                row.email = email.map(str::to_string);
            }
            row
        } else {
            let user_id = crate::rng::opaque_token()?;
            let account_id = crate::rng::id_32()?;
            tx.execute(
                "INSERT INTO users (user_id, provider, subject, email, account_id, principal_kind, created_at_ns) \
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
                params![
                    user_id,
                    provider,
                    subject,
                    email,
                    &account_id[..],
                    principal_kind as i64,
                    now_ns as i64
                ],
            )?;
            UserRow {
                user_id,
                provider: provider.to_string(),
                subject: subject.to_string(),
                email: email.map(str::to_string),
                account_id,
                principal_kind,
            }
        };
        let authorized = tx.execute(
            "UPDATE device_codes SET state = 'authorized', user_id = ?1 \
             WHERE user_code = ?2 AND state = 'pending' AND expires_at_ns > ?3",
            params![user.user_id, user_code, now_ns as i64],
        )?;
        if authorized != 1 {
            return Ok(None);
        }
        tx.commit()?;
        Ok(Some(user))
    }

    /// Fetch a user by id.
    pub fn get_user(&self, user_id: &str) -> Result<Option<UserRow>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT user_id, provider, subject, email, account_id, principal_kind \
                 FROM users WHERE user_id = ?1",
                params![user_id],
                Self::map_user,
            )
            .optional()?)
    }

    fn map_user(row: &rusqlite::Row<'_>) -> rusqlite::Result<UserRow> {
        Ok(UserRow {
            user_id: row.get(0)?,
            provider: row.get(1)?,
            subject: row.get(2)?,
            email: row.get(3)?,
            account_id: row
                .get::<_, Vec<u8>>(4)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            principal_kind: row.get::<_, i64>(5)? as u8,
        })
    }

    // -- devices -------------------------------------------------------------

    /// Register a device key for an account.
    ///
    /// A device key maps to EXACTLY ONE account (the invariant: accounts
    /// authorize, device keys authenticate). Re-registration is idempotent ONLY
    /// for the SAME account (returns the existing row unchanged, e.g. at
    /// login/refresh); a key already owned by a DIFFERENT account is refused with
    /// a LOUD [`AccountdError::Conflict`] — never a silent foreign-row return that
    /// would let the caller cert a key it does not own.
    pub fn register_device(
        &self,
        account_id: &[u8; 32],
        public_key: &[u8; 32],
        principal_kind: u8,
        now_ns: u64,
    ) -> Result<DeviceRow> {
        let conn = self.lock()?;
        let existing: Option<DeviceRow> = conn
            .query_row(
                "SELECT device_id, public_key, account_id, principal_kind, created_at_ns, revoked \
                 FROM devices WHERE public_key = ?1",
                params![&public_key[..]],
                Self::map_device,
            )
            .optional()?;
        if let Some(row) = existing {
            if row.account_id != *account_id {
                return Err(AccountdError::Conflict(
                    "device key is already registered to a different account \
                     (one device key maps to exactly one account)"
                        .to_string(),
                ));
            }
            // Same-account re-registration: idempotent — return the stored row.
            return Ok(row);
        }
        let device_id = crate::rng::opaque_token()?;
        conn.execute(
            "INSERT INTO devices (device_id, public_key, account_id, principal_kind, created_at_ns, revoked) \
             VALUES (?1, ?2, ?3, ?4, ?5, 0)",
            params![
                device_id,
                &public_key[..],
                &account_id[..],
                principal_kind as i64,
                now_ns as i64
            ],
        )?;
        Ok(DeviceRow {
            device_id,
            public_key: *public_key,
            account_id: *account_id,
            principal_kind,
            created_at_ns: now_ns,
            revoked: false,
        })
    }

    /// List the devices registered to an account.
    pub fn list_devices(&self, account_id: &[u8; 32]) -> Result<Vec<DeviceRow>> {
        let conn = self.lock()?;
        let mut stmt = conn.prepare(
            "SELECT device_id, public_key, account_id, principal_kind, created_at_ns, revoked \
             FROM devices WHERE account_id = ?1 ORDER BY created_at_ns ASC",
        )?;
        let rows = stmt
            .query_map(params![&account_id[..]], Self::map_device)?
            .collect::<rusqlite::Result<Vec<_>>>()?;
        Ok(rows)
    }

    fn map_device(row: &rusqlite::Row<'_>) -> rusqlite::Result<DeviceRow> {
        Ok(DeviceRow {
            device_id: row.get(0)?,
            public_key: row
                .get::<_, Vec<u8>>(1)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            account_id: row
                .get::<_, Vec<u8>>(2)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            principal_kind: row.get::<_, i64>(3)? as u8,
            created_at_ns: row.get::<_, i64>(4)? as u64,
            revoked: row.get::<_, i64>(5)? != 0,
        })
    }

    // -- robots ------------------------------------------------------------------

    /// Register a robot owned by `owner_account`, keyed on its transport key.
    ///
    /// Ownership is login-gated at install, by design: the installer's account owns
    /// the robot. A robot's transport key (its iroh EndpointId) maps to EXACTLY ONE
    /// robot, so re-registration is idempotent ONLY for the SAME owner (returns the
    /// stored row unchanged — a re-run / retried install); a transport key already
    /// owned by a DIFFERENT account is refused with a LOUD
    /// [`AccountdError::Conflict`] — never a silent foreign-row return that would let
    /// one account mint an owner grant for a robot it does not own. Mirrors
    /// [`Db::register_device`]'s one-key-one-owner discipline.
    ///
    /// `org_id` is stored verbatim (org membership is not validated
    /// here); a `None` org is a personally-owned robot.
    pub fn register_robot(
        &self,
        owner_account: &[u8; 32],
        hostname: &str,
        robot_transport_key: &[u8; 32],
        org_id: Option<&str>,
        now_ns: u64,
    ) -> Result<RobotRow> {
        let conn = self.lock()?;
        let existing: Option<RobotRow> = conn
            .query_row(
                "SELECT robot_id, hostname, owner_account_id, org_id, robot_transport_key, created_at_ns \
                 FROM robots WHERE robot_transport_key = ?1",
                params![&robot_transport_key[..]],
                Self::map_robot,
            )
            .optional()?;
        if let Some(row) = existing {
            if row.owner_account_id != *owner_account {
                return Err(AccountdError::Conflict(ROBOT_KEY_CONFLICT_MSG.to_string()));
            }
            // Same-owner re-registration: idempotent — return the stored row.
            return Ok(row);
        }
        // Not found by the pre-check — INSERT. A concurrent registration from ANOTHER
        // connection could have inserted the same transport key between our SELECT
        // and this INSERT (the `Mutex` only serializes THIS process's connection);
        // the `idx_robots_transport_key` UNIQUE index catches it and the constraint
        // violation is mapped to the SAME loud 409 the cross-account pre-check
        // returns, never a bare 500.
        Self::insert_robot_row(
            &conn,
            owner_account,
            hostname,
            robot_transport_key,
            org_id,
            now_ns,
        )
    }

    /// The single robots INSERT, classifying a UNIQUE-constraint violation
    /// (mapped to [`AccountdError::Conflict`], the same loud 409 the pre-check
    /// returns) distinctly from any other DB error (fatal). Split out so the
    /// race-lost mapping is testable directly (bypassing the SELECT pre-check).
    fn insert_robot_row(
        conn: &Connection,
        owner_account: &[u8; 32],
        hostname: &str,
        robot_transport_key: &[u8; 32],
        org_id: Option<&str>,
        now_ns: u64,
    ) -> Result<RobotRow> {
        let robot_id = crate::rng::id_32()?;
        match conn.execute(
            "INSERT INTO robots (robot_id, hostname, owner_account_id, org_id, robot_transport_key, created_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            params![
                &robot_id[..],
                hostname,
                &owner_account[..],
                org_id,
                &robot_transport_key[..],
                now_ns as i64
            ],
        ) {
            Ok(_) => Ok(RobotRow {
                robot_id,
                hostname: hostname.to_string(),
                owner_account_id: *owner_account,
                org_id: org_id.map(str::to_string),
                robot_transport_key: *robot_transport_key,
                created_at_ns: now_ns,
            }),
            Err(e) if is_constraint_violation(&e) => {
                Err(AccountdError::Conflict(ROBOT_KEY_CONFLICT_MSG.to_string()))
            }
            Err(e) => Err(AccountdError::Db(e)),
        }
    }

    /// Fetch a robot by its 32-byte id.
    pub fn get_robot(&self, robot_id: &[u8; 32]) -> Result<Option<RobotRow>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT robot_id, hostname, owner_account_id, org_id, robot_transport_key, created_at_ns \
                 FROM robots WHERE robot_id = ?1",
                params![&robot_id[..]],
                Self::map_robot,
            )
            .optional()?)
    }

    /// Every robot OWNED by `account_id` (the device self-revoke fan-out
    /// enumerates the account's robots to add the device to each one's revocation
    /// epoch). Uses the `idx_robots_owner` index.
    pub fn list_robots_by_owner(&self, account_id: &[u8; 32]) -> Result<Vec<RobotRow>> {
        let conn = self.lock()?;
        let mut stmt = conn.prepare(
            "SELECT robot_id, hostname, owner_account_id, org_id, robot_transport_key, created_at_ns \
             FROM robots WHERE owner_account_id = ?1 ORDER BY created_at_ns ASC",
        )?;
        let rows = stmt
            .query_map(params![&account_id[..]], Self::map_robot)?
            .collect::<rusqlite::Result<Vec<_>>>()?;
        Ok(rows)
    }

    fn map_robot(row: &rusqlite::Row<'_>) -> rusqlite::Result<RobotRow> {
        Ok(RobotRow {
            robot_id: row
                .get::<_, Vec<u8>>(0)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            hostname: row.get(1)?,
            owner_account_id: row
                .get::<_, Vec<u8>>(2)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            org_id: row.get(3)?,
            robot_transport_key: row
                .get::<_, Vec<u8>>(4)?
                .try_into()
                .map_err(|_| rusqlite::Error::InvalidQuery)?,
            created_at_ns: row.get::<_, i64>(5)? as u64,
        })
    }

    // -- devices (self-revoke) -----------------------------------------------

    /// Revoke a device the CALLER owns (a lost/decommissioned desk). Flips
    /// `devices.revoked = 1` guarded on BOTH the device id AND the owning account, so
    /// one account can never revoke another's device. Returns the updated row, or
    /// `None` if no such device belongs to the account (a caller cannot even learn
    /// that another account's device exists). Idempotent — re-revoking is a no-op
    /// that still returns the (already-revoked) row.
    pub fn revoke_device(
        &self,
        account_id: &[u8; 32],
        device_id: &str,
    ) -> Result<Option<DeviceRow>> {
        let conn = self.lock()?;
        // The UPDATE is account-scoped: it flips `revoked` only when the row belongs to
        // the caller (0 rows changed for a foreign / absent device). The SELECT below —
        // also account-scoped — is the source of truth for the return: an owned device
        // (even one already revoked) returns its row; a foreign/absent one returns None.
        conn.execute(
            "UPDATE devices SET revoked = 1 WHERE device_id = ?1 AND account_id = ?2",
            params![device_id, &account_id[..]],
        )?;
        let row = conn
            .query_row(
                "SELECT device_id, public_key, account_id, principal_kind, created_at_ns, revoked \
                 FROM devices WHERE device_id = ?1 AND account_id = ?2",
                params![device_id, &account_id[..]],
                Self::map_device,
            )
            .optional()?;
        Ok(row)
    }

    // -- access-list revocation epochs ---------------------------------------

    /// The LATEST (highest-numbered) revocation epoch for a robot, or `None` if the
    /// robot has never had one (a fresh robot with no revocations).
    pub fn latest_epoch(&self, robot_id: &[u8; 32]) -> Result<Option<EpochRow>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT epoch, revoked_accounts, revoked_devices, issued_at_ns \
                 FROM access_list_epochs WHERE robot_id = ?1 ORDER BY epoch DESC LIMIT 1",
                params![&robot_id[..]],
                Self::map_epoch,
            )
            .optional()?)
    }

    /// Insert a NEW revocation epoch for a robot. `epoch` MUST be strictly greater
    /// than any existing epoch for the robot (the caller derives it from
    /// [`Db::latest_epoch`]); a collision on `(robot_id, epoch)` is a
    /// [`AccountdError::Conflict`] (a concurrent revoke lost the race — the caller
    /// re-reads + retries).
    pub fn insert_epoch(
        &self,
        robot_id: &[u8; 32],
        epoch: u64,
        revoked_accounts: &[[u8; 32]],
        revoked_devices: &[[u8; 32]],
        issued_at_ns: u64,
    ) -> Result<()> {
        let conn = self.lock()?;
        conn.execute(
            "INSERT INTO access_list_epochs \
             (robot_id, epoch, revoked_accounts, revoked_devices, issued_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5)",
            params![
                &robot_id[..],
                epoch as i64,
                encode_ids(revoked_accounts),
                encode_ids(revoked_devices),
                issued_at_ns as i64,
            ],
        )
        .map(|_| ())
        .map_err(|e| {
            if is_constraint_violation(&e) {
                AccountdError::Conflict(format!(
                    "epoch {epoch} already exists for this robot (a concurrent revoke won the \
                     race); re-read the latest epoch and retry"
                ))
            } else {
                AccountdError::Db(e)
            }
        })
    }

    /// ATOMICALLY add a revocation `target` to a robot's set and mint the next epoch,
    /// under ONE connection lock — the read-modify-write (SELECT latest → compute next
    /// → INSERT) never yields the lock between the read and the write, so a concurrent
    /// `add_revocation` on the SAME robot cannot slip an epoch in between and force a
    /// spurious `Conflict` (the two-acquisition race).
    /// Concurrent calls serialize on the connection mutex: the first mints epoch N+1,
    /// the second reads N+1 and mints N+2 (monotonic, no conflict). Returns the NEW
    /// [`EpochRow`], or `None` if the target was already revoked (idempotent — no new
    /// epoch minted). `now_ns` stamps the new epoch's `issued_at_ns`.
    pub fn add_revocation(
        &self,
        robot_id: &[u8; 32],
        target: RevocationTarget,
        now_ns: u64,
    ) -> Result<Option<EpochRow>> {
        let conn = self.lock()?;

        // Read the latest epoch UNDER THE LOCK (no gap before the INSERT below).
        let latest: Option<EpochRow> = conn
            .query_row(
                "SELECT epoch, revoked_accounts, revoked_devices, issued_at_ns \
                 FROM access_list_epochs WHERE robot_id = ?1 ORDER BY epoch DESC LIMIT 1",
                params![&robot_id[..]],
                Self::map_epoch,
            )
            .optional()?;
        let cur_epoch = latest.as_ref().map(|e| e.epoch).unwrap_or(0);
        let mut accounts = latest
            .as_ref()
            .map(|e| e.revoked_accounts.clone())
            .unwrap_or_default();
        let mut devices = latest
            .as_ref()
            .map(|e| e.revoked_devices.clone())
            .unwrap_or_default();

        let already_present = match target {
            RevocationTarget::Account(a) => {
                if accounts.contains(&a) {
                    true
                } else {
                    accounts.push(a);
                    false
                }
            }
            RevocationTarget::Device(d) => {
                if devices.contains(&d) {
                    true
                } else {
                    devices.push(d);
                    false
                }
            }
        };
        if already_present {
            return Ok(None);
        }

        let next = cur_epoch
            .checked_add(1)
            .ok_or_else(|| AccountdError::Internal("epoch number overflow".into()))?;
        // INSERT under the SAME lock. A `(robot_id, epoch)` conflict is now impossible
        // within a process (all Db ops serialize on the connection mutex), so a
        // constraint violation here is a genuine unexpected error, not the race.
        conn.execute(
            "INSERT INTO access_list_epochs \
             (robot_id, epoch, revoked_accounts, revoked_devices, issued_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5)",
            params![
                &robot_id[..],
                next as i64,
                encode_ids(&accounts),
                encode_ids(&devices),
                now_ns as i64,
            ],
        )?;
        Ok(Some(EpochRow {
            epoch: next,
            revoked_accounts: accounts,
            revoked_devices: devices,
            issued_at_ns: now_ns,
        }))
    }

    fn map_epoch(row: &rusqlite::Row<'_>) -> rusqlite::Result<EpochRow> {
        Ok(EpochRow {
            epoch: row.get::<_, i64>(0)? as u64,
            revoked_accounts: decode_ids(&row.get::<_, Vec<u8>>(1)?).map_err(|e| {
                tracing::error!(error = %e, "corrupt access_list_epochs revoked_accounts blob");
                rusqlite::Error::InvalidQuery
            })?,
            revoked_devices: decode_ids(&row.get::<_, Vec<u8>>(2)?).map_err(|e| {
                tracing::error!(error = %e, "corrupt access_list_epochs revoked_devices blob");
                rusqlite::Error::InvalidQuery
            })?,
            issued_at_ns: row.get::<_, i64>(3)? as u64,
        })
    }

    // -- sessions ------------------------------------------------------------

    /// Create a session/refresh pair (both stored as SHA-256 hashes).
    #[allow(clippy::too_many_arguments)]
    pub fn insert_session(
        &self,
        user_id: &str,
        session_hash: &str,
        refresh_hash: &str,
        session_expires_at_ns: u64,
        refresh_expires_at_ns: u64,
        now_ns: u64,
    ) -> Result<String> {
        let id = crate::rng::opaque_token()?;
        let conn = self.lock()?;
        conn.execute(
            "INSERT INTO sessions \
             (id, user_id, session_token_hash, refresh_token_hash, session_expires_at_ns, refresh_expires_at_ns, revoked, created_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, 0, ?7)",
            params![
                id,
                user_id,
                session_hash,
                refresh_hash,
                session_expires_at_ns as i64,
                refresh_expires_at_ns as i64,
                now_ns as i64
            ],
        )?;
        Ok(id)
    }

    /// Look up a session by its session-token hash.
    pub fn session_by_session_hash(&self, session_hash: &str) -> Result<Option<SessionLookup>> {
        self.session_by_hash(SessionHashColumn::Session, session_hash)
    }

    /// Look up a session by its refresh-token hash.
    pub fn session_by_refresh_hash(&self, refresh_hash: &str) -> Result<Option<SessionLookup>> {
        self.session_by_hash(SessionHashColumn::Refresh, refresh_hash)
    }

    fn session_by_hash(
        &self,
        column: SessionHashColumn,
        hash: &str,
    ) -> Result<Option<SessionLookup>> {
        let conn = self.lock()?;
        // The full SQL is a hardcoded literal per column — NO string interpolation
        // of a column name, so this can never rot into a SQL-injection surface.
        let sql = match column {
            SessionHashColumn::Session => {
                "SELECT id, user_id, session_expires_at_ns, refresh_expires_at_ns, revoked \
                 FROM sessions WHERE session_token_hash = ?1"
            }
            SessionHashColumn::Refresh => {
                "SELECT id, user_id, session_expires_at_ns, refresh_expires_at_ns, revoked \
                 FROM sessions WHERE refresh_token_hash = ?1"
            }
        };
        Ok(conn
            .query_row(sql, params![hash], |row| {
                Ok(SessionLookup {
                    id: row.get(0)?,
                    user_id: row.get(1)?,
                    snapshot: SessionSnapshot {
                        session_expires_at_ns: row.get::<_, i64>(2)? as u64,
                        refresh_expires_at_ns: row.get::<_, i64>(3)? as u64,
                        revoked: row.get::<_, i64>(4)? != 0,
                    },
                })
            })
            .optional()?)
    }

    /// Rotate a session's token hashes + expiries — an atomic compare-and-swap
    /// guarded on `id`, `revoked = 0`, AND the OLD refresh-token hash. Returns
    /// whether THIS call won (rows-affected == 1). The two guards close two races:
    /// - `refresh_token_hash = old` makes refresh single-use — two concurrent
    ///   refreshes with the same token cannot both rotate (the second finds the
    ///   hash already replaced, `false`), so one client is never silently
    ///   invalidated by the other.
    /// - `revoked = 0` stops a concurrent revoke (which flips `revoked = 1` between
    ///   the snapshot read and this write) from being clobbered back to valid — the
    ///   guard lives IN the statement because the Mutex is per-statement, not a
    ///   transaction spanning the read.
    #[allow(clippy::too_many_arguments)]
    pub fn rotate_session(
        &self,
        id: &str,
        old_refresh_hash: &str,
        new_session_hash: &str,
        new_refresh_hash: &str,
        session_expires_at_ns: u64,
        refresh_expires_at_ns: u64,
    ) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE sessions SET session_token_hash = ?1, refresh_token_hash = ?2, \
             session_expires_at_ns = ?3, refresh_expires_at_ns = ?4 \
             WHERE id = ?5 AND revoked = 0 AND refresh_token_hash = ?6",
            params![
                new_session_hash,
                new_refresh_hash,
                session_expires_at_ns as i64,
                refresh_expires_at_ns as i64,
                id,
                old_refresh_hash
            ],
        )?;
        Ok(n == 1)
    }

    /// Revoke the session whose session- OR refresh-token hash matches. Returns
    /// whether a row was affected.
    pub fn revoke_session_by_token_hash(&self, hash: &str) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE sessions SET revoked = 1 \
             WHERE session_token_hash = ?1 OR refresh_token_hash = ?1",
            params![hash],
        )?;
        Ok(n > 0)
    }

    // -- device codes --------------------------------------------------------

    /// Insert a fresh (pending) device code, allocating a UNIQUE `user_code` with
    /// bounded retry: `gen_user_code` proposes candidates and a rare
    /// birthday-collision on the short human code is retried (up to `max_attempts`)
    /// with a fresh candidate before it becomes an error — a collision no longer
    /// surfaces as a 500. Returns the `user_code` that was actually stored.
    pub fn insert_device_code_retrying(
        &self,
        device_code_hash: &str,
        expires_at_ns: u64,
        interval_secs: u64,
        now_ns: u64,
        max_attempts: usize,
        mut gen_user_code: impl FnMut() -> Result<String>,
    ) -> Result<String> {
        let outcome = insert_with_collision_retry(max_attempts, &mut gen_user_code, |user_code| {
            match self.try_insert_device_code_raw(
                device_code_hash,
                user_code,
                expires_at_ns,
                interval_secs,
                now_ns,
            ) {
                Ok(()) => AttemptOutcome::Inserted(user_code.to_string()),
                Err(DeviceCodeInsertError::UserCodeCollision) => AttemptOutcome::Collision,
                Err(DeviceCodeInsertError::Other(e)) => AttemptOutcome::Fatal(e),
            }
        });
        match outcome {
            Ok(code) => Ok(code),
            Err(RetryGiveUp::Fatal(e)) => Err(e),
            Err(RetryGiveUp::Exhausted { attempts }) => Err(AccountdError::Internal(format!(
                "could not allocate a unique user_code after {attempts} attempts"
            ))),
        }
    }

    /// The single INSERT, classifying a `user_code` UNIQUE collision (retryable)
    /// distinctly from any other error (fatal — including a `device_code_hash`
    /// primary-key collision, which is NOT a user_code collision).
    fn try_insert_device_code_raw(
        &self,
        device_code_hash: &str,
        user_code: &str,
        expires_at_ns: u64,
        interval_secs: u64,
        now_ns: u64,
    ) -> std::result::Result<(), DeviceCodeInsertError> {
        let conn = self.lock().map_err(DeviceCodeInsertError::Other)?;
        match conn.execute(
            "INSERT INTO device_codes \
             (device_code_hash, user_code, state, user_id, expires_at_ns, interval_secs, last_poll_at_ns, created_at_ns) \
             VALUES (?1, ?2, 'pending', NULL, ?3, ?4, NULL, ?5)",
            params![
                device_code_hash,
                user_code,
                expires_at_ns as i64,
                interval_secs as i64,
                now_ns as i64
            ],
        ) {
            Ok(_) => Ok(()),
            Err(e) if is_user_code_collision(&e) => Err(DeviceCodeInsertError::UserCodeCollision),
            Err(e) => Err(DeviceCodeInsertError::Other(AccountdError::Db(e))),
        }
    }

    /// Snapshot a device code by its hash (for the pure poll decision).
    pub fn device_code_snapshot(
        &self,
        device_code_hash: &str,
    ) -> Result<Option<DeviceCodeSnapshot>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT state, user_id, expires_at_ns, interval_secs, last_poll_at_ns \
                 FROM device_codes WHERE device_code_hash = ?1",
                params![device_code_hash],
                |row| {
                    let state_str: String = row.get(0)?;
                    let user_id: Option<String> = row.get(1)?;
                    let state = match state_str.as_str() {
                        // An authorized row ALWAYS has a user_id (authorize_device_code
                        // sets it in the same UPDATE). A NULL here is a violated
                        // invariant — error, never fabricate an empty user_id (which
                        // would permanently redeem the code and mint a session for "",
                        // unrecoverably breaking the login).
                        "authorized" => DeviceCodeState::Authorized {
                            user_id: user_id.ok_or(rusqlite::Error::InvalidQuery)?,
                        },
                        "consumed" => DeviceCodeState::Consumed,
                        _ => DeviceCodeState::Pending,
                    };
                    Ok(DeviceCodeSnapshot {
                        state,
                        expires_at_ns: row.get::<_, i64>(2)? as u64,
                        interval_secs: row.get::<_, i64>(3)? as u64,
                        last_poll_at_ns: row.get::<_, Option<i64>>(4)?.map(|v| v as u64),
                    })
                },
            )
            .optional()?)
    }

    /// Authorize a pending, UNEXPIRED device code by its human `user_code`.
    /// Returns whether a still-authorizable code matched (a wrong /
    /// already-authorized / already-consumed / EXPIRED code → `false`). The expiry
    /// predicate keeps this consistent with the poll side (`poll_outcome` also
    /// treats an expired code as dead) — an expired pending code cannot be
    /// authorized into a live session.
    pub fn authorize_device_code(
        &self,
        user_code: &str,
        user_id: &str,
        now_ns: u64,
    ) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE device_codes SET state = 'authorized', user_id = ?1 \
             WHERE user_code = ?2 AND state = 'pending' AND expires_at_ns > ?3",
            params![user_id, user_code, now_ns as i64],
        )?;
        Ok(n == 1)
    }

    /// Record a poll instant (drives the slow-down gate).
    pub fn touch_device_code_poll(&self, device_code_hash: &str, now_ns: u64) -> Result<()> {
        let conn = self.lock()?;
        conn.execute(
            "UPDATE device_codes SET last_poll_at_ns = ?1 WHERE device_code_hash = ?2",
            params![now_ns as i64, device_code_hash],
        )?;
        Ok(())
    }

    /// Atomically redeem an AUTHORIZED device code — the compare-and-swap that
    /// makes single-use hold under concurrency. Flips `authorized`→`consumed` in
    /// ONE statement guarded by `WHERE state = 'authorized'` and returns whether
    /// THIS call won (rows-affected == 1). Two concurrent polls race here: exactly
    /// one flips the row (and issues a session); the loser sees `false` and is told
    /// the code was already redeemed. A non-atomic unconditional
    /// consume would let two polls both mint a session from one authorization.
    pub fn redeem_device_code(&self, device_code_hash: &str) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE device_codes SET state = 'consumed' \
             WHERE device_code_hash = ?1 AND state = 'authorized'",
            params![device_code_hash],
        )?;
        Ok(n == 1)
    }

    // -- magic links ---------------------------------------------------------

    /// Store a magic-link token (hash), bound to an email + the device-flow
    /// `user_code` it authorizes.
    pub fn insert_magic_link(
        &self,
        token_hash: &str,
        email: &str,
        user_code: &str,
        expires_at_ns: u64,
    ) -> Result<()> {
        let conn = self.lock()?;
        conn.execute(
            "INSERT INTO magic_links (token_hash, email, user_code, expires_at_ns, consumed) \
             VALUES (?1, ?2, ?3, ?4, 0)",
            params![token_hash, email, user_code, expires_at_ns as i64],
        )?;
        Ok(())
    }

    /// Consume a magic link atomically (rows-affected CAS): flip `consumed`→1 in
    /// ONE statement guarded by `consumed = 0 AND expires_at_ns > now`, and only
    /// the winner (rows-affected == 1) reads back `(email, user_code)`. Single-use
    /// and expiry-gated; two concurrent consumes can never both win (the same
    /// conditional-UPDATE discipline as [`Db::redeem_device_code`]).
    pub fn consume_magic_link(
        &self,
        token_hash: &str,
        now_ns: u64,
    ) -> Result<Option<(String, String)>> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE magic_links SET consumed = 1 \
             WHERE token_hash = ?1 AND consumed = 0 AND expires_at_ns > ?2",
            params![token_hash, now_ns as i64],
        )?;
        if n != 1 {
            return Ok(None);
        }
        // Won the CAS — read back the bound email + user_code.
        let row = conn.query_row(
            "SELECT email, user_code FROM magic_links WHERE token_hash = ?1",
            params![token_hash],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )?;
        Ok(Some(row))
    }

    // -- oauth flows ---------------------------------------------------------

    /// Store a pending OAuth authorization flow (PKCE verifier + the device-flow
    /// `user_code` to authorize on callback) with an expiry — the CSRF/lookup
    /// `state` is time-bounded, matching the magic-link path.
    pub fn insert_oauth_flow(
        &self,
        state: &str,
        provider: &str,
        pkce_verifier: &str,
        user_code: Option<&str>,
        expires_at_ns: u64,
        now_ns: u64,
    ) -> Result<()> {
        let conn = self.lock()?;
        conn.execute(
            "INSERT INTO oauth_flows (state, provider, pkce_verifier, user_code, expires_at_ns, created_at_ns) \
             VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            params![
                state,
                provider,
                pkce_verifier,
                user_code,
                expires_at_ns as i64,
                now_ns as i64
            ],
        )?;
        Ok(())
    }

    /// PEEK a pending, UNEXPIRED OAuth flow by its `state` WITHOUT consuming it —
    /// so the caller can run the token exchange FIRST and only [`Db::consume_oauth_flow`]
    /// on success. A failed exchange therefore leaves the flow row RETRYABLE (it was
    /// never deleted), while the original expiry/CSRF guard is preserved: an expired or
    /// unknown state returns `None`, so a leaked state cannot be replayed after expiry.
    pub fn peek_oauth_flow(
        &self,
        state: &str,
        now_ns: u64,
    ) -> Result<Option<(String, String, Option<String>)>> {
        let conn = self.lock()?;
        Ok(conn
            .query_row(
                "SELECT provider, pkce_verifier, user_code FROM oauth_flows \
                 WHERE state = ?1 AND expires_at_ns > ?2",
                params![state, now_ns as i64],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .optional()?)
    }

    /// Consume (delete) an OAuth flow by its `state` after a SUCCESSFUL exchange —
    /// an atomic single-use compare-and-swap: returns whether THIS call deleted the
    /// row (rows-affected == 1). Two concurrent callbacks that both peeked + exchanged
    /// race here; exactly one wins and proceeds, the loser is rejected.
    pub fn consume_oauth_flow(&self, state: &str) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute("DELETE FROM oauth_flows WHERE state = ?1", params![state])?;
        Ok(n == 1)
    }

    // -- device PoP challenges -----------------------------------------------

    /// Store a fresh proof-of-possession challenge (hash), bound to the issuing
    /// account with an expiry. Only the SHA-256 hash of the bearer is stored, never
    /// the plaintext (a DB read never yields a spendable challenge).
    pub fn insert_device_challenge(
        &self,
        challenge_hash: &str,
        account_id: &[u8; 32],
        expires_at_ns: u64,
        now_ns: u64,
    ) -> Result<()> {
        let conn = self.lock()?;
        conn.execute(
            "INSERT INTO device_challenges (challenge_hash, account_id, expires_at_ns, consumed, created_at_ns) \
             VALUES (?1, ?2, ?3, 0, ?4)",
            params![
                challenge_hash,
                &account_id[..],
                expires_at_ns as i64,
                now_ns as i64
            ],
        )?;
        Ok(())
    }

    /// Atomically SPEND a PoP challenge (single-use): flip `consumed`→1 in ONE
    /// statement guarded by `consumed = 0`, `expires_at_ns > now`, AND
    /// `account_id = the spending account`, returning whether THIS call won
    /// (rows-affected == 1). A `false` therefore covers EVERY failure the caller must
    /// reject identically — unknown / already-spent (replay) / expired / wrong-account
    /// challenge — so a challenge can never be reused and one account can never spend
    /// another's. Mirrors the [`Db::redeem_device_code`] / [`Db::consume_magic_link`]
    /// conditional-UPDATE single-use discipline.
    pub fn consume_device_challenge(
        &self,
        challenge_hash: &str,
        account_id: &[u8; 32],
        now_ns: u64,
    ) -> Result<bool> {
        let conn = self.lock()?;
        let n = conn.execute(
            "UPDATE device_challenges SET consumed = 1 \
             WHERE challenge_hash = ?1 AND consumed = 0 AND expires_at_ns > ?2 AND account_id = ?3",
            params![challenge_hash, now_ns as i64, &account_id[..]],
        )?;
        Ok(n == 1)
    }

    /// Opportunistic housekeeping: delete expired device codes, magic links
    /// (expired OR already consumed), and OAuth flows so these tables do not grow
    /// unbounded. Best-effort — invoked on a frequent entry point (`device/start`).
    /// Returns the number of rows deleted.
    pub fn sweep_expired(&self, now_ns: u64) -> Result<usize> {
        let conn = self.lock()?;
        let now = now_ns as i64;
        let mut deleted = 0usize;
        deleted += conn.execute(
            "DELETE FROM device_codes WHERE expires_at_ns <= ?1",
            params![now],
        )?;
        deleted += conn.execute(
            "DELETE FROM magic_links WHERE expires_at_ns <= ?1 OR consumed = 1",
            params![now],
        )?;
        deleted += conn.execute(
            "DELETE FROM oauth_flows WHERE expires_at_ns <= ?1",
            params![now],
        )?;
        deleted += conn.execute(
            "DELETE FROM device_challenges WHERE expires_at_ns <= ?1 OR consumed = 1",
            params![now],
        )?;
        Ok(deleted)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn column_names(db: &Db, table: &str) -> Vec<String> {
        let conn = db.lock().unwrap();
        let mut stmt = conn
            .prepare(&format!("PRAGMA table_info({table})"))
            .unwrap();
        stmt.query_map([], |row| row.get::<_, String>(1))
            .unwrap()
            .collect::<rusqlite::Result<Vec<_>>>()
            .unwrap()
    }

    /// A DB created from an older schema (its
    /// `access_list_epochs` predates `revoked_devices`) MUST gain the column via the
    /// open-time migration, so the INSERT/SELECTs naming it work. Seeds a file DB
    /// with the older table (NO `revoked_devices`), then `Db::open` migrates it.
    #[test]
    fn a6_migration_adds_revoked_devices_to_an_a0_era_db_through_open() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("a0era.db");
        // Seed the older table shape: `CREATE TABLE IF NOT EXISTS` (what an older build ran) with
        // NO `revoked_devices` column.
        {
            let conn = Connection::open(&path).unwrap();
            conn.execute_batch(
                "CREATE TABLE access_list_epochs (
                    robot_id         BLOB NOT NULL,
                    epoch            INTEGER NOT NULL,
                    revoked_accounts BLOB NOT NULL,
                    issued_at_ns     INTEGER NOT NULL,
                    PRIMARY KEY (robot_id, epoch)
                );",
            )
            .unwrap();
            assert!(
                !column_exists(&conn, "access_list_epochs", "revoked_devices").unwrap(),
                "precondition: the older table lacks the column"
            );
        }

        // Db::open runs SCHEMA (CREATE TABLE IF NOT EXISTS is a NO-OP for the existing
        // table — so it does NOT add the column) then run_migrations (which does).
        let db = Db::open(&path).unwrap();
        assert!(
            column_names(&db, "access_list_epochs")
                .iter()
                .any(|c| c == "revoked_devices"),
            "the migration must add revoked_devices to the older table"
        );

        // The write path (which names revoked_devices) works — pre-migration it
        // would fail with "no such column: revoked_devices".
        let robot = [0x11u8; 32];
        let row = db
            .add_revocation(&robot, RevocationTarget::Device([0x99; 32]), 5)
            .unwrap()
            .expect("mints an epoch");
        assert_eq!(row.epoch, 1);
        assert_eq!(row.revoked_devices, vec![[0x99u8; 32]]);
    }

    /// The migration is idempotent: running it on a FRESH DB (where SCHEMA already
    /// added the column) is a no-op, and running it twice never errors.
    #[test]
    fn a6_migration_is_idempotent_on_a_fresh_db() {
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch(SCHEMA).unwrap();
        // Fresh SCHEMA already has the column.
        assert!(column_exists(&conn, "access_list_epochs", "revoked_devices").unwrap());
        // Running the migration (twice) is a harmless no-op.
        run_migrations(&conn).unwrap();
        run_migrations(&conn).unwrap();
        assert!(column_exists(&conn, "access_list_epochs", "revoked_devices").unwrap());
    }

    #[test]
    fn dormant_grants_table_carries_the_issuer_column() {
        // Grant.issuer — present in the dormant schema so a SignedGrant can be
        // persisted without a migration.
        let db = Db::open_in_memory().unwrap();
        assert!(
            column_names(&db, "grants").iter().any(|c| c == "issuer"),
            "grants table must carry the issuer AccountId column"
        );
    }

    #[test]
    fn oauth_flows_carries_the_expires_at_column() {
        let db = Db::open_in_memory().unwrap();
        assert!(
            column_names(&db, "oauth_flows")
                .iter()
                .any(|c| c == "expires_at_ns"),
            "oauth_flows must carry an expires_at_ns column (the state is time-bounded)"
        );
    }

    #[test]
    fn robots_transport_key_is_uniquely_indexed() {
        // The uniqueness rides a SEPARATE `CREATE UNIQUE INDEX IF NOT EXISTS`
        // (not an inline column UNIQUE) so it applies to a DB created before the index too. Introspect
        // the index list to prove the unique index is actually present.
        let db = Db::open_in_memory().unwrap();
        let conn = db.lock().unwrap();
        let mut stmt = conn.prepare("PRAGMA index_list(robots)").unwrap();
        // PRAGMA index_list columns: seq(0) name(1) unique(2) origin(3) partial(4).
        let found = stmt
            .query_map([], |row| {
                Ok((row.get::<_, String>(1)?, row.get::<_, i64>(2)? != 0))
            })
            .unwrap()
            .collect::<rusqlite::Result<Vec<_>>>()
            .unwrap();
        assert!(
            found
                .iter()
                .any(|(name, unique)| name == "idx_robots_transport_key" && *unique),
            "robots must carry a UNIQUE index on robot_transport_key, got {found:?}"
        );
    }

    #[test]
    fn a_raced_duplicate_transport_key_insert_maps_to_a_loud_conflict() {
        // The INSERT-path UNIQUE violation (a concurrent registration that
        // slipped past the SELECT pre-check) maps to a loud 409 `Conflict`, not a
        // 500. Exercised deterministically by calling the raw INSERT helper twice
        // with the SAME transport key (each mints its own random robot_id, so the
        // collision is the transport-key index) — bypassing the SELECT pre-check.
        // This also proves the transport-key index is ACTIVE (a missing constraint
        // would let the second INSERT succeed).
        let db = Db::open_in_memory().unwrap();
        let conn = db.lock().unwrap();
        Db::insert_robot_row(&conn, &ACCT_A, "orin-01", &ROBOT_KEY, None, NOW)
            .expect("the first insert succeeds");
        let err = Db::insert_robot_row(&conn, &ACCT_B, "impostor", &ROBOT_KEY, None, NOW)
            .expect_err("a duplicate transport key must hit the UNIQUE index");
        assert!(
            matches!(err, AccountdError::Conflict(_)),
            "a duplicate-key INSERT maps to Conflict (409), not a bare Db 500; got {err:?}"
        );
    }

    const ACCT_A: [u8; 32] = [0xAA; 32];
    const ACCT_B: [u8; 32] = [0xBB; 32];
    const ROBOT_KEY: [u8; 32] = [0x22; 32];
    const NOW: u64 = 1_000_000_000_000;

    #[test]
    fn a_pop_challenge_is_single_use_account_bound_and_expiry_gated() {
        // The challenge consume CAS is the single-use + account-bound +
        // expiry gate the PoP rests on. Hand oracles, no HTTP.
        let db = Db::open_in_memory().unwrap();
        let far_future = NOW + 1_000_000_000_000;

        // (1) A fresh challenge for ACCT_A spends EXACTLY ONCE.
        db.insert_device_challenge("c1", &ACCT_A, far_future, NOW)
            .unwrap();
        assert!(
            db.consume_device_challenge("c1", &ACCT_A, NOW).unwrap(),
            "the first spend wins"
        );
        assert!(
            !db.consume_device_challenge("c1", &ACCT_A, NOW).unwrap(),
            "a replayed (already-consumed) challenge is rejected"
        );

        // (2) A DIFFERENT account cannot spend ACCT_A's challenge.
        db.insert_device_challenge("c2", &ACCT_A, far_future, NOW)
            .unwrap();
        assert!(
            !db.consume_device_challenge("c2", &ACCT_B, NOW).unwrap(),
            "a wrong-account spend is rejected"
        );
        // ...and the challenge is still spendable by its OWN account (the failed
        // wrong-account attempt did not consume it).
        assert!(
            db.consume_device_challenge("c2", &ACCT_A, NOW).unwrap(),
            "the owning account can still spend after a foreign attempt"
        );

        // (3) An EXPIRED challenge cannot be spent (now == expiry is expired: the
        // guard is `expires_at_ns > now`).
        db.insert_device_challenge("c3", &ACCT_A, NOW, NOW).unwrap();
        assert!(
            !db.consume_device_challenge("c3", &ACCT_A, NOW).unwrap(),
            "an expired challenge is rejected"
        );

        // (4) An UNKNOWN challenge hash is rejected.
        assert!(
            !db.consume_device_challenge("never-issued", &ACCT_A, NOW)
                .unwrap(),
            "an unknown challenge is rejected"
        );
    }

    #[test]
    fn sweep_expired_purges_expired_and_consumed_challenges() {
        let db = Db::open_in_memory().unwrap();
        let far_future = NOW + 1_000_000_000_000;
        // expired, consumed, and live-unconsumed.
        db.insert_device_challenge("expired", &ACCT_A, NOW, NOW)
            .unwrap();
        db.insert_device_challenge("consumed", &ACCT_A, far_future, NOW)
            .unwrap();
        db.insert_device_challenge("live", &ACCT_A, far_future, NOW)
            .unwrap();
        assert!(db
            .consume_device_challenge("consumed", &ACCT_A, NOW)
            .unwrap());

        db.sweep_expired(NOW).unwrap();

        // The live-unconsumed one survives (spendable); expired + consumed are gone.
        assert!(
            db.consume_device_challenge("live", &ACCT_A, NOW).unwrap(),
            "a live challenge survives the sweep"
        );
        // A second insert with the swept hashes proves the rows were deleted (a
        // lingering PRIMARY-KEY row would make the insert fail).
        db.insert_device_challenge("expired", &ACCT_A, far_future, NOW)
            .expect("the expired row was swept, so the hash is reusable");
        db.insert_device_challenge("consumed", &ACCT_A, far_future, NOW)
            .expect("the consumed row was swept, so the hash is reusable");
    }

    #[test]
    fn an_authorized_row_with_null_user_id_errors_never_fabricates_empty() {
        // A NULL user_id on an 'authorized' row is a violated invariant.
        // device_code_snapshot must ERROR, not fabricate Authorized{user_id:""}
        // (which would permanently redeem the code + mint a session for "").
        let db = Db::open_in_memory().unwrap();
        {
            let conn = db.lock().unwrap();
            conn.execute(
                "INSERT INTO device_codes \
                 (device_code_hash, user_code, state, user_id, expires_at_ns, interval_secs, last_poll_at_ns, created_at_ns) \
                 VALUES ('h', 'UC', 'authorized', NULL, ?1, 0, NULL, 0)",
                params![i64::MAX],
            )
            .unwrap();
        }
        assert!(
            db.device_code_snapshot("h").is_err(),
            "a NULL user_id on an authorized row must error, not fabricate an empty user_id"
        );
    }
}
