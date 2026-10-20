// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion schema`: create, delete, inspect and list message schemas.
//!
//! A verb family (see [`super`]): this module is the whole of the verb, from
//! its clap definition to its dispatch. The command logic lives in
//! `cerulion_cli_engine::schema_cmd`.

use std::process::ExitCode;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};
use clap_complete::engine::ArgValueCandidates;

use cerulion_cli_engine::error::CliResult;

use super::VerbFamily;
use crate::cli::VerbLogClass;
use crate::completion;

// The family's top-level verb, declared as a one-variant `Subcommand` so the
// derive generates the same `schema` command the `Commands` enum would. A
// plain comment, not a doc comment: the derive copies an enum doc comment onto
// the ROOT command's `about`, which `build` refuses.
#[derive(Subcommand)]
enum SchemaCmd {
    /// Schema management
    Schema {
        #[command(subcommand)]
        action: SchemaAction,
    },
}

#[derive(Subcommand)]
enum SchemaAction {
    /// Create a new schema
    Create {
        /// Schema name
        // No completer, and this arm was the worst of the three.
        // The workspace half is what the verb rejects (already-exists), and
        // the 254 built-in `pkg/Type` names are worse than useless here: a
        // `sensor_msgs/Image` either fails on the `/` or silently creates a
        // workspace schema that SHADOWS the built-in for the whole workspace.
        name: String,
    },
    /// Delete a schema
    Delete {
        /// Schema name
        #[arg(add = ArgValueCandidates::new(completion::schema_names))]
        name: String,
    },
    /// Show info about a schema
    Info {
        /// Schema name
        #[arg(add = ArgValueCandidates::new(completion::schema_names))]
        name: String,
    },
    /// List all schemas: workspace-local (schemas/*.yaml) and built-in ROS 2 types by package
    List,
}

/// The `schema` verb family.
pub(crate) struct Schema;

impl VerbFamily for Schema {
    fn name(&self) -> &'static str {
        "schema"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("topic")
    }

    fn augment(&self, root: Command) -> Command {
        SchemaCmd::augment_subcommands(root)
    }

    /// Every schema verb runs and exits, so all of them default quiet.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = SchemaAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        super::exit_code_of(run(action))
    }
}

fn run(action: SchemaAction) -> CliResult<()> {
    match action {
        // Create/Delete WRITE into the workspace `schemas/` dir, so a
        // workspace is required (loud error naming `workspace create`).
        SchemaAction::Create { name } => {
            let ws = crate::discover_workspace()?;
            cerulion_cli_engine::schema_cmd::schema_create(&ws.schemas_dir, &name)?;
            println!("Created schema '{}'", name);
            Ok(())
        }
        SchemaAction::Delete { name } => {
            let ws = crate::discover_workspace()?;
            cerulion_cli_engine::schema_cmd::schema_delete(&ws.schemas_dir, &name)?;
            println!("Deleted schema '{}'", name);
            Ok(())
        }
        SchemaAction::Info { name } => {
            // `schema info` does NOT require a
            // workspace - the intended use is a brand-new laptop
            // resolving a robot's types with zero setup. A missing
            // workspace ⇒ skip the workspace + `.msg`-store tiers and
            // resolve builtin → remote exactly as designed; a workspace
            // present ⇒ the full resolution ladder (workspace
            // wins). `schema_info_unified_opt` keeps the provenance
            // accurate either way.
            let ws = crate::discover_workspace().ok();
            let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
            // Unified lookup - workspace schemas first,
            // then the embedded built-in ROS 2 registry (the exact
            // .msg text the generated types compiled against), with
            // ONE renderer for both sources. Replaces the sibling
            // `../native_ros2_messages/msg` path probe, which only
            // worked for workspaces created inside this repository.
            let info = match cerulion_cli_engine::schema_cmd::schema_info_unified_opt(
                schemas_dir,
                &name,
            ) {
                Ok(info) => info,
                // Unresolvable locally (neither workspace
                // nor built-in). If it is a QUALIFIED `pkg/Type` and robots
                // are discoverable, ask THEM - the automagic bar: a desk
                // with zero local knowledge of a robot's custom types can
                // still `schema info` them. Best-effort: a failed/empty
                // fetch re-raises the original local error (exit code
                // unchanged). A found remote schema prints with a loud
                // provenance header, then returns Ok.
                Err(local_err @ cerulion_cli_engine::error::CliError::SchemaNotFound { .. }) => {
                    match cerulion_cli_engine::schema_cmd::remote_fetch_target(&name) {
                        Some(requested) => {
                            let opts = cerulion_cli_engine::topic_cmd::RemoteTopicsOptions {
                                connect: vec![],
                                listen: vec![],
                                scouting: true,
                            };
                            // An empty remote answer means two different
                            // things. Only a SETTLED discovery licenses re-raising
                            // the terminal local "schema not found"; a cold
                            // `cerulion-netd` (or a transient gather that read
                            // nothing) searched NOTHING, so saying the type does
                            // not exist would be a claim with no evidence - the
                            // same false absence already removed from the topic
                            // path.
                            //
                            // The variant match lives in
                            // `cerulion_cli_engine::topic_cmd::schema_info_remote_outcome`, NOT here -
                            // as a `match` in this file the mapping has no test,
                            // and a mutation reverting its cold-start arm would pass
                            // the whole suite. One `?` leaves no arm to mis-route.
                            let fetched = cerulion_cli_engine::topic_cmd::fetch_remote_schema(
                                &requested, &opts,
                            )?;
                            let reply = cerulion_cli_engine::topic_cmd::schema_info_remote_outcome(
                                &requested, fetched, local_err,
                            )?;
                            print!(
                                "{}",
                                cerulion_cli_engine::schema_cmd::render_remote_schema(&reply)
                            );
                            return Ok(());
                        }
                        None => return Err(local_err),
                    }
                }
                Err(e) => return Err(e),
            };
            // A workspace schema colliding with a
            // built-in name is LOUD - one stderr warning per
            // (workspace schema, shadowed built-in) pair, BEFORE
            // the rendered output. `shadowed_builtin` is only
            // `Some` on workspace results; the ", "-joined list
            // splits back losslessly (names carry no commas).
            if let Some(shadowed) = &info.shadowed_builtin {
                for builtin in shadowed.split(", ") {
                    let bare = builtin.rsplit('/').next().unwrap_or(builtin);
                    for entry in &info.result.entries {
                        // The ONE canonical identity (the engine's
                        // `shadowed_builtin_names` decided the shadow
                        // that way): an entry declared `pkg::Type`
                        // must get the warning its `pkg/Type` twin
                        // gets, or the loud line goes missing on
                        // exactly the spelling the engine flagged.
                        let declared =
                            cerulion_cli_engine::schema_cmd::normalize_schema(&entry.name);
                        if declared == builtin || declared == bare {
                            eprintln!(
                                "WARNING: workspace schema '{}' shadows built-in '{}' \
                                 — workspace wins for schema info", // dash-ok: user-facing text, pinned byte for byte
                                entry.name, builtin
                            );
                        }
                    }
                }
            }
            print!("{}", info);
            Ok(())
        }
        SchemaAction::List => {
            // Workspace-local schemas + the embedded
            // built-in ROS 2 registry grouped by package, with
            // shadowed built-ins marked inline. One renderer.
            // Workspace-OPTIONAL - no workspace ⇒ the
            // built-in registry alone (never a stray cwd `schemas/`).
            let ws = crate::discover_workspace().ok();
            let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
            let listing = cerulion_cli_engine::schema_cmd::schema_list_opt(schemas_dir);
            print!("{}", listing);
            Ok(())
        }
    }
}
