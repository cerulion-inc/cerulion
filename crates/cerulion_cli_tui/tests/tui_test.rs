// SPDX-License-Identifier: AGPL-3.0-only
//! Rendering and state tests for the TUI dashboard.
//!
//! Uses ratatui's `TestBackend` for rendering verification without a real terminal.

use cerulion_cli_engine::node_metadata::{NodeMetadata, PortDef};
use cerulion_cli_tui::app::{App, EchoEntry, Tab};
use cerulion_cli_tui::event::{handle_key, EventResult};
use cerulion_cli_tui::ui;
use cerulion_core::graph::node::BackpressurePolicy;
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::backend::TestBackend;
use ratatui::Terminal;

/// Helper: create a test terminal with the given dimensions.
fn test_terminal(width: u16, height: u16) -> Terminal<TestBackend> {
    let backend = TestBackend::new(width, height);
    Terminal::new(backend).unwrap()
}

/// Helper: render one frame and return the buffer as a string.
fn render_to_string(app: &App, width: u16, height: u16) -> String {
    let mut terminal = test_terminal(width, height);
    terminal.draw(|f| ui::draw(f, app)).unwrap();
    let buffer = terminal.backend().buffer().clone();
    let mut output = String::new();
    for y in 0..buffer.area.height {
        for x in 0..buffer.area.width {
            let cell = &buffer[(x, y)];
            output.push_str(cell.symbol());
        }
        output.push('\n');
    }
    output
}

/// Helper: make a KeyEvent from a KeyCode.
fn key(code: KeyCode) -> KeyEvent {
    KeyEvent::new(code, KeyModifiers::NONE)
}

// ─── Checkpoint 8.1: test_tui_initializes ─────────────────────────

#[test]
fn test_tui_initializes() {
    let app = App::new();
    assert!(app.running);
    assert_eq!(app.tab, Tab::Nodes);
    assert!(app.nodes.is_empty());
    assert!(app.topics.is_empty());
    assert!(app.echo_messages.is_empty());

    // Verify rendering doesn't panic with empty state
    let mut terminal = test_terminal(80, 24);
    terminal.draw(|f| ui::draw(f, &app)).unwrap();
}

// ─── Empty-state rendering: no workspace + workspace with no nodes ────

#[test]
fn test_tui_nodes_empty_no_workspace_shows_hint() {
    // When the TUI is launched outside a workspace, the Nodes pane must
    // explain *why* it's empty rather than silently showing nothing.
    let app = App::new();
    assert!(app.workspace_root.is_none());

    let output = render_to_string(&app, 100, 24);
    assert!(
        output.contains("Not inside a Cerulion workspace"),
        "off-workspace nodes pane should explain the situation; got:\n{}",
        output
    );
}

#[test]
fn test_tui_nodes_empty_in_workspace_shows_create_hint() {
    let mut app = App::new();
    app.workspace_root = Some(std::path::PathBuf::from("/tmp/fake_ws"));

    let output = render_to_string(&app, 100, 24);
    assert!(
        output.contains("cerulion node create"),
        "in-workspace empty pane should point at `cerulion node create`; got:\n{}",
        output
    );
}

#[test]
fn test_tui_topics_empty_shows_graph_run_hint() {
    let mut app = App::new();
    app.tab = Tab::Topics;
    let output = render_to_string(&app, 100, 24);
    assert!(
        output.contains("cerulion graph run"),
        "empty topics pane should point at `cerulion graph run`; got:\n{}",
        output
    );
}

// ─── Checkpoint 8.2: test_tui_node_list_renders ───────────────────

#[test]
fn test_tui_node_list_renders() {
    let mut app = App::new();

    // Populate with test node data
    app.nodes = vec![
        NodeMetadata {
            throttle_ms: None,
            node_type: "camera".to_string(),
            policy: None,
            inputs: vec![],
            outputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        },
        NodeMetadata {
            throttle_ms: None,
            node_type: "detector".to_string(),
            policy: None,
            inputs: vec![PortDef {
                name: "image".to_string(),
                schema: Some("sensor_msgs::Image".to_string()),
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
            outputs: vec![PortDef {
                name: "detections".to_string(),
                schema: None,
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            }],
        },
    ];

    let output = render_to_string(&app, 80, 24);

    // Verify node names appear in the rendered output
    assert!(output.contains("camera"), "Should show 'camera' node");
    assert!(output.contains("detector"), "Should show 'detector' node");
    assert!(output.contains("Nodes"), "Should show Nodes tab/header");
}

// ─── Checkpoint 8.3: test_tui_topic_echo_updates ──────────────────

#[test]
fn test_tui_topic_echo_updates() {
    let mut app = App::new();
    app.tab = Tab::Echo;
    app.echo_topic = Some("perception/image".to_string());

    // Push echo entries
    app.push_echo(EchoEntry {
        sequence: 1,
        timestamp_ns: 1_000_000_000,
        schema_hash: 0xabcd,
        size: 128,
        payload_hex: "de ad be ef".to_string(),
    });
    app.push_echo(EchoEntry {
        sequence: 2,
        timestamp_ns: 2_000_000_000,
        schema_hash: 0xabcd,
        size: 256,
        payload_hex: "ca fe ba be".to_string(),
    });

    let output = render_to_string(&app, 100, 24);

    // Verify echo data appears
    assert!(
        output.contains("perception/image"),
        "Should show echo topic name"
    );
    assert!(output.contains("128B"), "Should show message size");
    assert!(output.contains("256B"), "Should show second message size");
    assert!(output.contains("de ad be ef"), "Should show payload hex");
}

// ─── Checkpoint 8.4: test_tui_keyboard_navigation ─────────────────

#[test]
fn test_tui_keyboard_navigation() {
    let mut app = App::new();
    app.nodes = vec![
        NodeMetadata::new("camera", None),
        NodeMetadata::new("detector", None),
        NodeMetadata::new("tracker", None),
    ];
    app.topics = vec![
        "perception/image".to_string(),
        "perception/detections".to_string(),
    ];

    // Initial state
    assert_eq!(app.tab, Tab::Nodes);
    assert_eq!(app.node_scroll, 0);

    // Tab → Topics
    let result = handle_key(&mut app, key(KeyCode::Tab));
    assert!(matches!(result, EventResult::Continue));
    assert_eq!(app.tab, Tab::Topics);

    // Tab → Echo
    handle_key(&mut app, key(KeyCode::Tab));
    assert_eq!(app.tab, Tab::Echo);

    // Tab → wraps to Nodes
    handle_key(&mut app, key(KeyCode::Tab));
    assert_eq!(app.tab, Tab::Nodes);

    // BackTab → wraps to Echo
    handle_key(&mut app, key(KeyCode::BackTab));
    assert_eq!(app.tab, Tab::Echo);

    // Navigate back to Nodes, test scroll
    app.tab = Tab::Nodes;
    app.node_scroll = 0;
    handle_key(&mut app, key(KeyCode::Down));
    assert_eq!(app.node_scroll, 1);
    handle_key(&mut app, key(KeyCode::Down));
    assert_eq!(app.node_scroll, 2);
    // Should not scroll past end
    handle_key(&mut app, key(KeyCode::Down));
    assert_eq!(app.node_scroll, 2);
    // Scroll up
    handle_key(&mut app, key(KeyCode::Up));
    assert_eq!(app.node_scroll, 1);

    // Enter on Topics tab selects topic for echo
    app.tab = Tab::Topics;
    app.topic_scroll = 1;
    handle_key(&mut app, key(KeyCode::Enter));
    assert_eq!(app.tab, Tab::Echo);
    assert_eq!(app.echo_topic.as_deref(), Some("perception/detections"));

    // q quits
    let result = handle_key(&mut app, key(KeyCode::Char('q')));
    assert!(matches!(result, EventResult::Quit));
}

// ─── Additional: echo buffer overflow test ────────────────────────

#[test]
fn test_echo_buffer_capacity() {
    let mut app = App::new();
    for i in 0..250 {
        app.push_echo(EchoEntry {
            sequence: i,
            timestamp_ns: i as u64 * 1_000_000,
            schema_hash: 0,
            size: 64,
            payload_hex: String::new(),
        });
    }
    // Should cap at 200
    assert_eq!(app.echo_messages.len(), 200);
    // Oldest entries should have been evicted
    assert_eq!(app.echo_messages[0].sequence, 50);
}
