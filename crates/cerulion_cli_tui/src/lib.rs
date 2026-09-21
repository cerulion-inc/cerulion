// SPDX-License-Identifier: AGPL-3.0-only
//! Cerulion TUI — interactive terminal dashboard for monitoring nodes, topics,
//! and message flow in real-time.
//!
//! Built on ratatui + crossterm with a clean separation between state (`app`),
//! rendering (`ui`), and input handling (`event`).

// P12 (the AGENTS.md logging convention): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `crates/cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod app;
pub mod event;
pub mod terminal;
pub mod ui;

use std::time::Duration;

use app::{App, EchoEntry};
use event::EventResult;

/// Run the TUI dashboard main loop.
///
/// Discovers the workspace (if any) to populate node/topic data, then enters
/// the render-poll-update loop until the user presses `q` or Ctrl+C.
///
/// Terminal is always restored on exit, even if the main loop panics.
pub fn run() -> std::io::Result<()> {
    let mut terminal = terminal::init()?;
    let mut app = App::new();

    // Load initial data (best-effort; errors just leave lists empty)
    refresh_data(&mut app);

    // Install a panic hook that restores the terminal before printing the panic.
    let original_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = crossterm::terminal::disable_raw_mode();
        let _ = crossterm::execute!(
            std::io::stdout(),
            crossterm::terminal::LeaveAlternateScreen,
            crossterm::event::DisableMouseCapture
        );
        original_hook(info);
    }));

    let result = main_loop(&mut terminal, &mut app);

    // Restore panic hook before restoring terminal (avoid double-restore on future panics)
    let _ = std::panic::take_hook();

    terminal::restore(&mut terminal)?;
    result
}

fn main_loop(terminal: &mut terminal::Tui, app: &mut App) -> std::io::Result<()> {
    let mut last_refresh = std::time::Instant::now();

    // Echo subscriber: lazily created (and re-created) when the user
    // selects a topic via Enter on the Topics tab. Held across iterations
    // so we drain new samples on every loop turn rather than missing
    // messages between renders.
    let mut echo_state: Option<EchoSubscription> = None;

    while app.running {
        terminal.draw(|f| ui::draw(f, app))?;

        // Poll for keyboard events (50ms — responsive but not busy)
        if let Some(key) = event::poll_event(Duration::from_millis(50))? {
            if let EventResult::Quit = event::handle_key(app, key) {
                app.running = false;
            }
        }

        // Refresh data every 2 seconds
        if last_refresh.elapsed() >= Duration::from_secs(2) {
            refresh_data(app);
            last_refresh = std::time::Instant::now();
        }

        // Drive the Echo tab: keep a live iceoryx2 subscriber on whichever
        // topic the user selected, and drain any new samples into the app
        // ring buffer before the next render. Mirrors `topic_cmd::topic_echo`
        // — same schema-aware decoding for the well-known message types.
        pump_echo(&mut echo_state, app);
    }

    Ok(())
}

/// Active subscription for the Echo tab.
struct EchoSubscription {
    topic: String,
    subscriber: cerulion_core::transport::subscriber::CerulionSubscriber,
}

/// Drain any new echo messages into `app.echo_messages`, creating /
/// re-creating the subscriber when the selected topic changes.
fn pump_echo(state: &mut Option<EchoSubscription>, app: &mut App) {
    let target = app.echo_topic.clone();

    match (&state, &target) {
        (Some(active), Some(want)) if &active.topic == want => {}
        _ => {
            *state = target
                .as_deref()
                .and_then(|topic| new_subscription(topic).ok());
        }
    }

    let Some(sub) = state.as_ref() else { return };

    let _ = sub.subscriber.try_receive(
        |msg: cerulion_core::transport::subscriber::ReceivedMessage<'_>| {
            let header = msg.header();
            let payload = msg.payload();
            // Recipe-3 (layout + FQN): single source of truth for the
            // pinned schema hashes — a `const fn` over the name alone cannot
            // reproduce a layout-sensitive hash. Kept in sync with the
            // generated constants by `pinned_hashes_match_generated_constants`
            // in cerulion_cli_engine/tests/integration_test.rs.
            const STD_MSGS_STRING: u64 =
                cerulion_cli_engine::topic_cmd::STD_MSGS_STRING_SCHEMA_HASH;
            const SENSOR_MSGS_IMAGE: u64 =
                cerulion_cli_engine::topic_cmd::SENSOR_MSGS_IMAGE_SCHEMA_HASH;

            let preview = match header.schema_hash {
                STD_MSGS_STRING => decode_string(payload)
                    .map(|s| format!("{:?}", s))
                    .unwrap_or_else(|| "<undecodable string>".to_string()),
                SENSOR_MSGS_IMAGE => decode_image_meta(payload)
                    .map(|(w, h)| format!("Image {}×{} ({} bytes)", w, h, payload.len()))
                    .unwrap_or_else(|| "<undecodable image>".to_string()),
                _ => payload
                    .iter()
                    .take(32)
                    .map(|b| format!("{:02x}", b))
                    .collect::<Vec<_>>()
                    .join(" "),
            };

            app.push_echo(EchoEntry {
                sequence: header.sequence,
                timestamp_ns: header.timestamp_ns,
                schema_hash: header.schema_hash,
                size: header.total_size,
                payload_hex: preview,
            });
        },
    );
}

fn new_subscription(topic: &str) -> Result<EchoSubscription, cerulion_core::TransportError> {
    let transport = cerulion_core::TransportManager::get_or_init()?;
    let subscriber = transport.create_subscriber(topic)?;
    Ok(EchoSubscription {
        topic: topic.to_string(),
        subscriber,
    })
}

fn decode_string(payload: &[u8]) -> Option<String> {
    if payload.len() < 8 {
        return None;
    }
    let off = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
    let len = u32::from_le_bytes(payload[4..8].try_into().ok()?) as usize;
    let end = off.checked_add(len)?;
    if end > payload.len() {
        return None;
    }
    std::str::from_utf8(&payload[off..end])
        .ok()
        .map(str::to_string)
}

fn decode_image_meta(payload: &[u8]) -> Option<(u32, u32)> {
    if payload.len() < 16 + 24 {
        return None;
    }
    let height = u32::from_le_bytes(payload[0..4].try_into().ok()?);
    let width = u32::from_le_bytes(payload[4..8].try_into().ok()?);
    Some((width, height))
}

/// Refresh node and topic lists from the workspace / iceoryx2 services.
///
/// The TUI does not require a workspace — topics come from live iceoryx2
/// service discovery and work from any directory. Workspace discovery is
/// best-effort: when it succeeds, the Nodes pane is populated; when it
/// fails, the pane renders a hint pointing the user at the right command.
fn refresh_data(app: &mut App) {
    match cerulion_cli_engine::workspace::CerulionWorkspace::discover(
        &std::env::current_dir().unwrap_or_default(),
    ) {
        Ok(ws) => {
            app.nodes = cerulion_cli_engine::node_cmd::node_list(&ws.nodes_dir).unwrap_or_default();
            app.workspace_root = Some(ws.root);
        }
        Err(_) => {
            app.nodes.clear();
            app.workspace_root = None;
        }
    }

    // Topics come from iceoryx2 service discovery, independent of the workspace.
    app.topics = cerulion_cli_engine::topic_cmd::topic_list()
        .map(|topics| topics.into_iter().map(|t| t.name).collect())
        .unwrap_or_default();
}
