// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-accountd` binary: the hosted account service.
//!
//! Configuration is read from the environment (all optional except where noted):
//!
//! | Env var | Default | Purpose |
//! |---|---|---|
//! | `CERULION_ACCOUNTD_BIND` | [`DEFAULT_BIND`] (loopback) | Listen address |
//! | `CERULION_ACCOUNTD_DB` | `./accountd.sqlite` | SQLite path (`:memory:` = ephemeral) |
//! | `CERULION_ACCOUNTD_VERIFICATION_BASE_URI` | `http://` + the address the listener actually bound ([`default_verification_base_uri`]) — so `:0` prints the port the kernel picked | Device-flow verification base |
//! | `CERULION_ACCOUNTD_{GOOGLE,GITHUB}_CLIENT_ID/SECRET/REDIRECT` | — | OAuth creds (a provider with no id is unconfigured → its start endpoint refuses loudly) |
//! | `CERULION_ACCOUNTD_SUPABASE_ISSUER` | — | Enables Supabase Auth JWT verification |
//! | `CERULION_ACCOUNTD_SUPABASE_AUDIENCE` | `authenticated` | Accepted Supabase JWT audience |
//! | `CERULION_ACCOUNTD_SUPABASE_JWKS_URL` | `<issuer>/.well-known/jwks.json` | Supabase signing-key endpoint |
//! | `CERULION_ACCOUNTD_SUPABASE_JWT_SECRET` | — | Optional legacy HS256 secret |
//!
//! **Note:** the CA is **dev-provisioned** (an in-process root ceremony) — the
//! production air-gapped M-of-N root ceremony + loading only the rotatable
//! intermediate is not implemented here (fork 7, deferred to a security review). A dev
//! CA's roots are ephemeral (a restart re-provisions), so certs it issued do not
//! verify across restarts — fine for bring-up, never production.

use std::sync::Arc;

use cerulion_accountd::{
    default_verification_base_uri, AppState, Clock, LoggingEmailSender, OAuthConfig, ProviderCreds,
    ServiceConfig, SupabaseConfig, UnconfiguredResolver, DEFAULT_BIND,
};

fn provider_creds(prefix: &str) -> Option<ProviderCreds> {
    let client_id = std::env::var(format!("{prefix}_CLIENT_ID")).ok()?;
    let client_secret = std::env::var(format!("{prefix}_SECRET")).unwrap_or_default();
    let redirect_uri = std::env::var(format!("{prefix}_REDIRECT")).unwrap_or_default();
    Some(ProviderCreds {
        client_id,
        client_secret,
        redirect_uri,
    })
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let bind = std::env::var("CERULION_ACCOUNTD_BIND").unwrap_or_else(|_| DEFAULT_BIND.to_string());
    let db_path =
        std::env::var("CERULION_ACCOUNTD_DB").unwrap_or_else(|_| "./accountd.sqlite".into());

    // Bind BEFORE the config is built. The codes this daemon prints must name the
    // socket it is listening on, and with `:0` the kernel picks the port — the
    // requested address does not know it yet, so a URI derived from the request
    // would say `:0`, which nothing serves. `local_addr()` is the only statement
    // of where this daemon actually is.
    let listener = tokio::net::TcpListener::bind(&bind).await?;
    let listening_on = listener.local_addr()?;

    let config = ServiceConfig {
        verification_base_uri: std::env::var("CERULION_ACCOUNTD_VERIFICATION_BASE_URI")
            .unwrap_or_else(|_| default_verification_base_uri(&listening_on.to_string())),
        oauth: OAuthConfig {
            google: provider_creds("CERULION_ACCOUNTD_GOOGLE"),
            github: provider_creds("CERULION_ACCOUNTD_GITHUB"),
        },
        supabase: SupabaseConfig::from_env_values(
            std::env::var("CERULION_ACCOUNTD_SUPABASE_ISSUER").ok(),
            std::env::var("CERULION_ACCOUNTD_SUPABASE_AUDIENCE").ok(),
            std::env::var("CERULION_ACCOUNTD_SUPABASE_JWKS_URL").ok(),
            std::env::var("CERULION_ACCOUNTD_SUPABASE_JWT_SECRET").ok(),
        )?,
        ..ServiceConfig::default()
    };

    // The account DB.
    let db = if db_path == ":memory:" {
        cerulion_accountd::Db::open_in_memory()?
    } else {
        cerulion_accountd::Db::open(&db_path)?
    };

    // DEV CA (see the module docs). Loud so nobody mistakes it for production.
    let clock = Clock::System;
    let ca = cerulion_accountd::Ca::dev_provision(config.ca.clone(), clock.now_ns())?;
    tracing::warn!(
        "cerulion-accountd started with a DEV-PROVISIONED CA (ephemeral in-process roots). \
         The production air-gapped M-of-N root ceremony is not implemented here — do NOT deploy this \
         as a production issuer."
    );
    if config.oauth.google.is_none() {
        tracing::info!("google oauth: not configured (its start endpoint will refuse loudly)");
    }
    if config.oauth.github.is_none() {
        tracing::info!("github oauth: not configured (its start endpoint will refuse loudly)");
    }

    let state = Arc::new(AppState::new(
        ca,
        db,
        config,
        clock,
        Arc::new(UnconfiguredResolver),
        Arc::new(LoggingEmailSender),
    ));

    tracing::info!(addr = %listening_on, "cerulion-accountd listening");

    axum::serve(listener, cerulion_accountd::build_router(state))
        .with_graceful_shutdown(shutdown_signal())
        .await?;
    tracing::info!("cerulion-accountd shut down");
    Ok(())
}

/// Resolve on SIGINT (Ctrl-C) or SIGTERM.
async fn shutdown_signal() {
    let ctrl_c = async {
        let _ = tokio::signal::ctrl_c().await;
    };
    #[cfg(unix)]
    let terminate = async {
        // If SIGTERM registration FAILS, this future must NEVER resolve (else it
        // would win the select! at boot and shut the server down immediately).
        // Park forever, mirroring the non-unix branch, so only a real signal wakes.
        match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
            Ok(mut sig) => {
                sig.recv().await;
            }
            Err(e) => {
                tracing::warn!(error = %e, "SIGTERM handler registration failed; ignoring SIGTERM");
                std::future::pending::<()>().await
            }
        }
    };
    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        _ = ctrl_c => {},
        _ = terminate => {},
    }
}
