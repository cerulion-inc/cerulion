// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-rcpd`: the Robot Context Protocol daemon.
//!
//! An MCP server named [`SERVER_NAME`] that exposes the live robot as MCP
//! resources. This crate ships the read surface's first resource,
//! [`GRAPH_URI`]: the runs on this robot, the nodes its manifest declares, the
//! live topic catalog (publisher counts, liveness, integral rate estimates)
//! and the topic monitor's flags.
//!
//! - [`manifest`]: the `rcp.yaml` manifest (robot identity, graph YAML).
//! - [`graph`]: the `robot://graph` JSON document and its pure composer.
//! - [`observer`]: the std thread that samples the transport and refreshes the
//!   cached document at most once per [`observer::GRAPH_REFRESH_INTERVAL_NS`].
//! - [`server`]: the MCP handler.
//! - [`http`]: the bearer-token streamable HTTP transport.
//!
//! The stdio transport is the binary's default; every log line goes to stderr
//! so stdout carries only the MCP stream.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod error;
pub mod graph;
pub mod http;
pub mod manifest;
pub mod observer;
pub mod server;

pub use error::RcpdError;

/// The MCP server name reported to clients.
pub const SERVER_NAME: &str = "cerulion-rcp";

/// The URI of the robot graph resource.
pub const GRAPH_URI: &str = "robot://graph";

/// The address `--http` binds when given no value: loopback only.
pub const DEFAULT_HTTP_ADDR: &str = "127.0.0.1:7801";

/// The environment variable carrying the HTTP bearer token. `--http` refuses
/// to start without it.
pub const TOKEN_ENV: &str = "CERULION_RCP_TOKEN";
