// SPDX-License-Identifier: AGPL-3.0-only
//! Explicit prior-login context for tests that actually serve a network graph.
use std::path::{Path, PathBuf};

pub fn expired_login(root: &Path) -> PathBuf {
    let home = root.join("serving-login");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::write(home.join("auth.json"), br#"{"account_id":"00112233-4455-4677-8899-aabbccddeeff","session_token":"expired-test-session","refresh_token":"offline-test-refresh","expires_at_ns":0,"logged_in_ever":true}"#).unwrap();
    home
}
