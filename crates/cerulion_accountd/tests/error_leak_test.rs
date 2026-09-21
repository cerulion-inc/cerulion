// SPDX-License-Identifier: AGPL-3.0-only
//! 5xx responses must not leak internal detail (raw rusqlite text — table names,
//! constraint names, internal messages) to the client. The client gets an opaque
//! message + a correlation `error_id`; the detail goes to the server log only.

use axum::body::to_bytes;
use axum::response::IntoResponse;
use serde_json::Value;

use cerulion_accountd::{AccountdError, Db};

const FUTURE: u64 = 2_000_000_000_000;

#[tokio::test]
async fn a_forced_db_error_5xx_body_leaks_no_schema_or_table_strings() {
    let db = Db::open_in_memory().unwrap();
    db.insert_magic_link("dup-token", "a@b.c", "UC", FUTURE)
        .unwrap();
    // Re-inserting the SAME primary key forces a real rusqlite constraint error
    // whose Display carries the table + constraint names.
    let err = db
        .insert_magic_link("dup-token", "a@b.c", "UC", FUTURE)
        .unwrap_err();
    assert!(
        matches!(err, AccountdError::Db(_)),
        "expected a Db error, got {err:?}"
    );

    // The RAW detail genuinely carries internals — so the leak surface is real
    // (this arm is the anti-tautology: the opaque body below is hiding something).
    let raw_detail = err.to_string().to_lowercase();
    assert!(
        raw_detail.contains("magic_links")
            || raw_detail.contains("constraint")
            || raw_detail.contains("unique"),
        "the raw rusqlite detail should carry table/constraint text: {raw_detail}"
    );

    let resp = err.into_response();
    assert_eq!(resp.status(), 500);
    let bytes = to_bytes(resp.into_body(), usize::MAX).await.unwrap();
    let body: Value = serde_json::from_slice(&bytes).unwrap();

    assert_eq!(body["error"], "db_error");
    assert_eq!(body["error_description"], "an internal error occurred");
    assert!(
        body["error_id"]
            .as_str()
            .map(|s| !s.is_empty())
            .unwrap_or(false),
        "a correlation error_id must be present"
    );
    // NONE of the internal strings reach the client body.
    let client = String::from_utf8_lossy(&bytes).to_lowercase();
    assert!(
        !client.contains("magic_links"),
        "table name leaked: {client}"
    );
    assert!(
        !client.contains("constraint"),
        "constraint text leaked: {client}"
    );
    assert!(!client.contains("unique"), "unique text leaked: {client}");
}

#[tokio::test]
async fn internal_5xx_is_opaque_but_4xx_keeps_its_actionable_detail() {
    // A 5xx Internal error carrying a sensitive marker → opaque body + error_id.
    let err = AccountdError::Internal("SENSITIVE_users_table_constraint_marker".to_string());
    let resp = err.into_response();
    assert_eq!(resp.status(), 500);
    let bytes = to_bytes(resp.into_body(), usize::MAX).await.unwrap();
    let body: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(body["error"], "internal_error");
    assert_eq!(body["error_description"], "an internal error occurred");
    assert!(
        !String::from_utf8_lossy(&bytes).contains("SENSITIVE_users_table"),
        "the internal marker must not reach the client"
    );

    // A 4xx, by contrast, KEEPS its client-actionable detail (no opaquing).
    let bad = AccountdError::BadRequest("a valid email is required".to_string());
    let bad_resp = bad.into_response();
    assert_eq!(bad_resp.status(), 400);
    let bad_bytes = to_bytes(bad_resp.into_body(), usize::MAX).await.unwrap();
    let bad_body: Value = serde_json::from_slice(&bad_bytes).unwrap();
    // The 4xx keeps the actionable detail verbatim (the full Display, prefix and all).
    assert!(
        bad_body["error_description"]
            .as_str()
            .unwrap()
            .contains("a valid email is required"),
        "4xx must keep its actionable detail: {bad_body}"
    );
}
