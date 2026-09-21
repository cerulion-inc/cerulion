-- SPDX-License-Identifier: AGPL-3.0-only
-- A0 account-service schema (§A resource model).
--
-- ACTIVE in A0: users, devices, sessions, device_codes, magic_links, oauth_flows.
-- DORMANT (created so A2/A4/A7 build on a stable schema; no A0 endpoint touches
-- them): orgs, org_members, robots, grants, service_accounts, access_list_epochs.
--
-- 32-byte ids/keys are BLOBs; timestamps are INTEGER Unix nanoseconds. Only
-- token HASHES are stored (never plaintext secrets).

-- ===================== ACTIVE =====================

CREATE TABLE IF NOT EXISTS users (
    user_id        TEXT PRIMARY KEY,
    provider       TEXT NOT NULL,             -- google | github | email | supabase
    subject        TEXT NOT NULL,             -- provider-scoped stable subject
    email          TEXT,
    account_id     BLOB NOT NULL UNIQUE,      -- stable 32-byte AccountId
    principal_kind INTEGER NOT NULL,          -- 1 = Human, 2 = Machine
    created_at_ns  INTEGER NOT NULL,
    UNIQUE (provider, subject)
);

CREATE TABLE IF NOT EXISTS devices (
    device_id      TEXT PRIMARY KEY,
    public_key     BLOB NOT NULL UNIQUE,      -- ed25519 device key = iroh EndpointId
    account_id     BLOB NOT NULL,
    principal_kind INTEGER NOT NULL,
    created_at_ns  INTEGER NOT NULL,
    revoked        INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_devices_account ON devices (account_id);

CREATE TABLE IF NOT EXISTS sessions (
    id                    TEXT PRIMARY KEY,
    user_id               TEXT NOT NULL,
    session_token_hash    TEXT NOT NULL UNIQUE,  -- SHA-256(session token)
    refresh_token_hash    TEXT NOT NULL UNIQUE,  -- SHA-256(refresh token)
    session_expires_at_ns INTEGER NOT NULL,
    refresh_expires_at_ns INTEGER NOT NULL,
    revoked               INTEGER NOT NULL DEFAULT 0,
    created_at_ns         INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS device_codes (
    device_code_hash TEXT PRIMARY KEY,          -- SHA-256(device_code bearer)
    user_code        TEXT NOT NULL UNIQUE,       -- human code entered in the browser
    state            TEXT NOT NULL,              -- pending | authorized | consumed
    user_id          TEXT,                       -- set when authorized
    expires_at_ns    INTEGER NOT NULL,
    interval_secs    INTEGER NOT NULL,
    last_poll_at_ns  INTEGER,
    created_at_ns    INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS magic_links (
    token_hash    TEXT PRIMARY KEY,             -- SHA-256(magic-link token)
    email         TEXT NOT NULL,
    user_code     TEXT NOT NULL,                -- the device-flow code it authorizes
    expires_at_ns INTEGER NOT NULL,
    consumed      INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS oauth_flows (
    state         TEXT PRIMARY KEY,             -- CSRF/lookup state
    provider      TEXT NOT NULL,
    pkce_verifier TEXT NOT NULL,
    user_code     TEXT,                          -- device-flow code to authorize on callback
    expires_at_ns INTEGER NOT NULL,              -- the state is time-bounded (no indefinite replay)
    created_at_ns INTEGER NOT NULL
);

-- Proof-of-possession challenges. A session-authed client requests a
-- fresh challenge, signs it with its device private key, and presents the signature
-- at a PoP-gated endpoint (POST /v1/robots) to prove it holds the private half of
-- the transport key it is registering. The challenge is SINGLE-USE (consumed on the
-- first successful spend), account-bound (only the issuing account may spend it),
-- and time-bounded. Only the SHA-256 HASH of the challenge bearer is stored.
CREATE TABLE IF NOT EXISTS device_challenges (
    challenge_hash TEXT PRIMARY KEY,            -- SHA-256(challenge bearer)
    account_id     BLOB NOT NULL,               -- the account the challenge was issued to
    expires_at_ns  INTEGER NOT NULL,
    consumed       INTEGER NOT NULL DEFAULT 0,  -- single-use (CAS to 1 on spend)
    created_at_ns  INTEGER NOT NULL
);

-- ===================== DORMANT (A2 / A4 / A7) =====================

CREATE TABLE IF NOT EXISTS orgs (
    org_id       TEXT PRIMARY KEY,
    name         TEXT NOT NULL,
    created_at_ns INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS org_members (
    org_id  TEXT NOT NULL,
    user_id TEXT NOT NULL,
    role    INTEGER NOT NULL,
    PRIMARY KEY (org_id, user_id)
);

CREATE TABLE IF NOT EXISTS robots (
    robot_id            BLOB PRIMARY KEY,        -- 32-byte RobotId
    hostname            TEXT NOT NULL,
    owner_account_id    BLOB NOT NULL,
    org_id              TEXT,
    robot_transport_key BLOB NOT NULL,           -- the robot's iroh EndpointId
    created_at_ns       INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_robots_owner ON robots (owner_account_id);
-- one key = one robot. The uniqueness is a SEPARATE index, not an
-- inline column UNIQUE, so it also applies to an older DB whose `robots` table
-- was already created (a bare `CREATE TABLE IF NOT EXISTS` never adds a column
-- constraint to an existing table). `CREATE UNIQUE INDEX IF NOT EXISTS` is
-- idempotent on a fresh AND an existing DB, so the constraint is never inert.
CREATE UNIQUE INDEX IF NOT EXISTS idx_robots_transport_key ON robots (robot_transport_key);

CREATE TABLE IF NOT EXISTS grants (
    grant_id         TEXT PRIMARY KEY,
    subject_account  BLOB NOT NULL,
    robot_id         BLOB NOT NULL,
    role             INTEGER NOT NULL,
    caps             INTEGER NOT NULL,
    principal_kind   INTEGER NOT NULL,
    delegation_depth INTEGER NOT NULL,
    not_before_ns    INTEGER NOT NULL,
    not_after_ns     INTEGER NOT NULL,
    issued_at_ns     INTEGER NOT NULL,
    issuer           BLOB NOT NULL,              -- the issuing AccountId (§A Grant.issuer; A4 persists it)
    revoked          INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS service_accounts (
    service_account_id TEXT PRIMARY KEY,
    org_id             TEXT NOT NULL,
    account_id         BLOB NOT NULL UNIQUE,
    created_at_ns      INTEGER NOT NULL
);

-- A6 ACTIVATED this dormant table: the per-robot revocation epoch the
-- robot syncs + applies (`TrustStore::apply_epoch`). `revoked_accounts` /
-- `revoked_devices` are a FLAT concatenation of 32-byte ids (N ids => 32*N bytes;
-- the count is implicit from the blob length) — the encoding the DB owns on both
-- read and write. `PRIMARY KEY (robot_id, epoch)` keeps the epoch numbers
-- monotonic per robot.
CREATE TABLE IF NOT EXISTS access_list_epochs (
    robot_id         BLOB NOT NULL,
    epoch            INTEGER NOT NULL,
    revoked_accounts BLOB NOT NULL,              -- flat 32*N bytes (Vec<AccountId>)
    revoked_devices  BLOB NOT NULL DEFAULT x'',  -- flat 32*N bytes (Vec<PublicKey>)
    issued_at_ns     INTEGER NOT NULL,
    PRIMARY KEY (robot_id, epoch)
);
