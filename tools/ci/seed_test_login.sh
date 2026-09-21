#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-only
#
# Provision a Cerulion config directory that the login gate accepts, for a test
# or CI job that runs the `cerulion` binary on a machine which has never signed
# in.
#
# Usage: tools/ci/seed_test_login.sh <dir>
#        CERULION_HOME=<dir> cerulion ...
#
# <dir> is used verbatim as the config directory, exactly as CERULION_HOME is.
# It is created if it does not exist.
#
# This writes the same `auth.json` that a real sign-in writes, minus the tokens:
# a fixed fake account id, empty session and refresh tokens, and the durable
# `logged_in_ever` marker the gate actually reads. It talks to nothing, so a CI
# job that uses it never reaches the account service.
#
# The shape is pinned against the type it has to parse as: a unit test in
# `crates/cerulion_cli_engine/src/auth.rs` runs this script and requires the
# result to load as an `AuthState` the gate lets through. A Rust test can call
# `cerulion_cli_engine::auth::seed_logged_in_at` instead and get the same file.
set -eu

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <dir>" >&2
    exit 2
fi

dir=$1
mkdir -p "$dir"
chmod 700 "$dir" 2>/dev/null || true

# `expires_at_ns` is u64::MAX so the seeded session never reads as stale.
cat > "$dir/auth.json" <<'JSON'
{
  "account_id": "seeded-test-account",
  "session_token": "",
  "refresh_token": "",
  "expires_at_ns": 18446744073709551615,
  "logged_in_ever": true
}
JSON
chmod 600 "$dir/auth.json"
