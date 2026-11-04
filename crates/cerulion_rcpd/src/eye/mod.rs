// SPDX-License-Identifier: AGPL-3.0-only
//! Jev, the gate's fast second eye: a typed look at one wrist-camera frame.
//!
//! A node in the graph publishes an ask on the manifest's ask topic (default
//! `/rcp/eye/ask`), naming a question set and a frame it has seen:
//!
//! ```json
//! {"ask_id": 7, "question_set": "press_gate",
//!  "frame": {"topic": "/wrist/image", "seq": 412}, "deadline_ms": 1500,
//!  "numbers": {"jaw_gap_mm": 3.5}}
//! ```
//!
//! With an `eyes.jev` section in the manifest, the `rcp-eye` thread looks the
//! frame up among the newest frames it holds from the manifest's frame topic
//! (`sensor_msgs/CompressedImage`, JPEG), checks the request shape and the
//! robot's limits ([`limits`]), sends one
//! System One request to the configured route ([`client`]) and publishes one
//! answer frame on the answer topic (default `/rcp/eye/answer`) for every
//! ask, with an `outcome` ([`verdict::Outcome`]). Every answer frame has a
//! receipt (`eye.jev`, caller `rcp-eye`) written before it is published.
//! Without the section no eye ports open: an ask goes unanswered and the
//! gate's own deadline makes Jev absent.
//!
//! The answer frame carries what was sent apart from the image bytes and the
//! key: the questions, the state with the image placeholder, the image's
//! SHA-256 and size, and the SHA-256 of the request body. A recording holds
//! the frame on its own topic, so the request can be checked against it.
//!
//! The answer topic has no producer inside the graph, so a replay re-injects
//! it as recorded and nothing calls the network; [`replay`] serves recorded
//! answers by `ask_id` and answers an ask it never saw with `not_recorded`.
//! [`verify`] joins a recording's answers to the receipt log.
//!
//! The gate reads an answer through [`verdict::Look`]: `Agree`, `Disagree`
//! or `Absent(outcome)`. There is no default and no conversion to `bool`, so
//! a missing answer cannot be read as agreement.
//!
//! - [`config`]: the manifest's `eyes.jev` section.
//! - [`limits`]: the v1 rate, in-flight and request shape limits.
//! - [`credential`]: the per-robot key file.
//! - [`client`]: the HTTP client and the request body.
//! - [`answer`]: the answer frame and how a reply maps to an outcome.
//! - [`host`]: the `rcp-eye` thread.
//! - [`switch`]: the owner's on/off switch, the data notice and the status
//!   file.

pub mod answer;
pub mod client;
pub mod config;
pub mod credential;
pub mod host;
pub mod limits;
pub mod replay;
pub mod switch;
pub mod verdict;
pub mod verify;

/// The caller every eye receipt names.
pub const EYE_CALLER: &str = "rcp-eye";

/// The verb every Jev receipt records.
pub const JEV_VERB: &str = "eye.jev";

/// The environment variable that names the Jev credential file, ahead of
/// the manifest's `credential_file`.
pub const JEV_KEY_FILE_ENV: &str = "CERULION_JEV_KEY_FILE";
