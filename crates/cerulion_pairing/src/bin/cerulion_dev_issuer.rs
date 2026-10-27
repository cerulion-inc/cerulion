// SPDX-License-Identifier: MIT OR Apache-2.0
//! `cerulion-dev-issuer`: enroll this machine in a local development team.
//!
//! ```text
//! cerulion-dev-issuer init [--team <short>] [--role <role>] [--import <state file>]
//! cerulion-dev-issuer status
//! ```
//!
//! `init` loads `<config>/dev-issuer/state`, or copies it from `--import`, or
//! creates it for `--team` (default `dev`); then issues this machine a leaf in
//! `--role` (one of robot, owner, operator, viewer, ci; default: the role in
//! this machine's leaf file for that team, expired or not, else `operator`),
//! writes the leaf
//! and the team bundle under `<config>/identity/<team>/` and makes the team
//! active. `<config>` is `CERULION_HOME`, else `~/.cerulion`. A second machine
//! joins the same team with `init --import <a copy of the first machine's
//! state file>`.
//!
//! `status` prints the active team and whether its credentials verify.
//!
//! Exit status: 0 on success, 2 for a usage error, 1 for any other failure.

use std::process::ExitCode;

use cerulion_pairing::identity::{unix_date, IdentityLayout, TeamDirName};
use cerulion_pairing::team::dev_issuer::{dev_issuer_dir, DevIssuer, DEV_ISSUER_STATE_FILENAME};
use cerulion_pairing::team::{
    check_leaf, trusted_root_sets, TeamRole, TeamShort, NO_EXPIRY_UNIX_SECS,
};

const USAGE: &str = "usage:
  cerulion-dev-issuer init [--team <short>] [--role <role>] [--import <state file>]
  cerulion-dev-issuer status";

enum Failure {
    Usage(String),
    Run(String),
}

fn run<E: std::fmt::Display>(e: E) -> Failure {
    Failure::Run(e.to_string())
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

fn init(args: &[String]) -> Result<String, Failure> {
    let mut team: Option<String> = None;
    let mut role: Option<TeamRole> = None;
    let mut import: Option<String> = None;
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let Some(value) = it.next().cloned() else {
            return Err(Failure::Usage(format!("{flag} needs a value")));
        };
        let twice = || Failure::Usage(format!("{flag} is given twice"));
        match flag.as_str() {
            "--team" if team.is_none() => team = Some(value),
            "--role" if role.is_none() => {
                role = Some(TeamRole::from_label(&value).ok_or_else(|| {
                    Failure::Usage(format!(
                        "unknown role '{value}'; use robot, owner, operator, viewer or ci"
                    ))
                })?);
            }
            "--import" if import.is_none() => import = Some(value),
            "--team" | "--role" | "--import" => return Err(twice()),
            other => return Err(Failure::Usage(format!("unknown argument '{other}'"))),
        }
    }
    let layout = IdentityLayout::resolve().map_err(run)?;
    let now = now_ns();
    let wrong_team = |found: &TeamShort, wanted: &str| {
        Failure::Run(format!(
            "the development issuer is for team '{found}', not '{wanted}'"
        ))
    };
    let issuer = match (DevIssuer::load(&layout).map_err(run)?, import) {
        (Some(_), Some(_)) => {
            return Err(Failure::Run(format!(
                "{} already holds a development issuer; remove it to import another",
                dev_issuer_dir(&layout).display()
            )))
        }
        (Some(existing), None) => existing,
        (None, Some(path)) => {
            let found = DevIssuer::team_of_state_file(path.as_ref()).map_err(run)?;
            if let Some(t) = &team {
                if t != found.as_str() {
                    return Err(wrong_team(&found, t));
                }
            }
            DevIssuer::import(&layout, path.as_ref()).map_err(run)?
        }
        (None, None) => {
            let short =
                TeamShort::new(team.clone().unwrap_or_else(|| "dev".into())).map_err(run)?;
            DevIssuer::create(&layout, short, now).map_err(run)?
        }
    };
    if let Some(t) = &team {
        if t != issuer.team().as_str() {
            return Err(wrong_team(issuer.team(), t));
        }
    }
    let roots = trusted_root_sets(&layout).map_err(run)?;
    let role = match role {
        Some(r) => r,
        None => current_role(&layout, issuer.team()).unwrap_or(TeamRole::Operator),
    };
    issuer.enroll(&layout, role, now).map_err(run)?;
    let creds = layout
        .load_team_credentials(&roots, now)
        .map_err(run)?
        .ok_or_else(|| Failure::Run("the team was written but does not read back".into()))?;
    let valid = if creds.leaf.not_after_unix_secs == NO_EXPIRY_UNIX_SECS {
        "does not expire".to_string()
    } else {
        format!("valid to {}", unix_date(creds.leaf.not_after_unix_secs))
    };
    Ok(format!(
        "Joined development team {} as {role}, machine {} (certificate {valid}).\n\
         To join another machine, copy {} to it and run there: \
         cerulion-dev-issuer init --import <copied file> --role <role>",
        issuer.team(),
        creds.key.short_fingerprint(),
        dev_issuer_dir(&layout)
            .join(DEV_ISSUER_STATE_FILENAME)
            .display()
    ))
}

/// The role in this machine's leaf file for `team`, read with the profile
/// check only (no clock and no chain), when the leaf names `team` and this
/// machine's key.
fn current_role(layout: &IdentityLayout, team: &TeamShort) -> Option<TeamRole> {
    let dir = TeamDirName::new(team.as_str()).ok()?;
    let leaf = check_leaf(&std::fs::read(layout.leaf_path(&dir)).ok()?).ok()?;
    let key = layout.load_machine_key().ok()??;
    (leaf.team == *team && leaf.device_key.0 == key.public_key()).then_some(leaf.role)
}

fn status(args: &[String]) -> Result<String, Failure> {
    if let Some(extra) = args.first() {
        return Err(Failure::Usage(format!("unknown argument '{extra}'")));
    }
    let layout = IdentityLayout::resolve().map_err(run)?;
    let roots = trusted_root_sets(&layout).map_err(run)?;
    match layout
        .load_team_credentials(&roots, now_ns())
        .map_err(run)?
    {
        None => Ok("no active team".into()),
        Some(c) => Ok(format!(
            "active team {} as {} at generation {}, machine {}",
            c.team.as_str(),
            c.leaf.role,
            c.leaf.generation,
            c.key.short_fingerprint()
        )),
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("init") => init(&args[1..]),
        Some("status") => status(&args[1..]),
        Some(other) => Err(Failure::Usage(format!("unknown command '{other}'"))),
        None => Err(Failure::Usage("no command".into())),
    };
    match result {
        Ok(line) => {
            println!("{line}");
            ExitCode::SUCCESS
        }
        Err(Failure::Usage(line)) => {
            eprintln!("cerulion-dev-issuer: {line}\n{USAGE}");
            ExitCode::from(2)
        }
        Err(Failure::Run(line)) => {
            eprintln!("cerulion-dev-issuer: {line}");
            ExitCode::from(1)
        }
    }
}
