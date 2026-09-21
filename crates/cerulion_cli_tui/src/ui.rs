// SPDX-License-Identifier: AGPL-3.0-only
//! Rendering functions for the Cerulion TUI dashboard.
//!
//! Pure functions that build ratatui widget trees from `App` state.
//! No side effects — all rendering is immediate-mode.

use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, Table, Tabs};
use ratatui::Frame;

use cerulion_core::MacroPolicy;

use crate::app::{App, Tab};

/// Render a [`MacroPolicy`] as a short, human-readable label
/// for the dashboard's POLICY column. None falls back to a `-`
/// placeholder so the column stays aligned across rows.
fn format_policy(spec: Option<&MacroPolicy>) -> String {
    match spec {
        Some(MacroPolicy::Period { period_ms }) => format!("period {}ms", period_ms),
        Some(MacroPolicy::Sync { window_ms }) => format!("sync {}ms", window_ms),
        Some(MacroPolicy::UnboundedSync) => "sync ∞".to_string(),
        Some(MacroPolicy::External) => "external".to_string(),
        Some(MacroPolicy::DataTrigger { input_name }) => format!("trigger:{}", input_name),
        None => "-".to_string(),
    }
}

/// Render the full dashboard for one frame.
pub fn draw(f: &mut Frame, app: &App) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // tab bar
            Constraint::Min(0),    // content
            Constraint::Length(1), // status bar
        ])
        .split(f.area());

    draw_tabs(f, app, chunks[0]);

    match app.tab {
        Tab::Nodes => draw_nodes(f, app, chunks[1]),
        Tab::Topics => draw_topics(f, app, chunks[1]),
        Tab::Echo => draw_echo(f, app, chunks[1]),
    }

    draw_status_bar(f, app, chunks[2]);
}

fn draw_tabs(f: &mut Frame, app: &App, area: Rect) {
    let titles: Vec<Line> = Tab::ALL
        .iter()
        .map(|t| {
            let style = if *t == app.tab {
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::DarkGray)
            };
            Line::from(Span::styled(t.label(), style))
        })
        .collect();

    let tabs = Tabs::new(titles)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Cerulion Dashboard "),
        )
        .select(app.tab.index())
        .highlight_style(Style::default().fg(Color::Cyan));

    f.render_widget(tabs, area);
}

/// Compute viewport offset so the selected row is always visible.
/// Returns (start_index, visible_count) for slicing row data.
fn viewport_window(selected: usize, total: usize, area_height: u16) -> (usize, usize) {
    // 2 for borders, 1 for header row
    let visible = area_height.saturating_sub(3) as usize;
    if visible == 0 || total == 0 {
        return (0, 0);
    }
    let start = if selected >= visible {
        selected - visible + 1
    } else {
        0
    };
    (start, visible)
}

fn draw_nodes(f: &mut Frame, app: &App, area: Rect) {
    if app.nodes.is_empty() {
        let hint = if app.workspace_root.is_some() {
            "No node types in this workspace yet.\nCreate one with: cerulion node create <type> --policy period_ms=100 -o <schema> <name>"
        } else {
            "Not inside a Cerulion workspace.\n`cd` into one (a directory with Cargo.toml + graphs/), or run: cerulion workspace create <name>\n\nLive topics still show in the Topics tab: they don't need a workspace."
        };
        let msg =
            Paragraph::new(hint).block(Block::default().borders(Borders::ALL).title(" Nodes "));
        f.render_widget(msg, area);
        return;
    }

    let header = Row::new(vec![
        Cell::from("TYPE").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("INPUTS").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("OUTPUTS").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("POLICY").style(Style::default().add_modifier(Modifier::BOLD)),
    ]);

    let (start, _visible) = viewport_window(app.node_scroll, app.nodes.len(), area.height);

    let rows: Vec<Row> = app
        .nodes
        .iter()
        .enumerate()
        .skip(start)
        .map(|(i, node)| {
            let style = if i == app.node_scroll {
                Style::default().fg(Color::Black).bg(Color::Cyan)
            } else {
                Style::default()
            };
            Row::new(vec![
                Cell::from(node.node_type.clone()),
                Cell::from(node.inputs.len().to_string()),
                Cell::from(node.outputs.len().to_string()),
                Cell::from(format_policy(node.policy.as_ref())),
            ])
            .style(style)
        })
        .collect();

    let table = Table::new(
        rows,
        [
            Constraint::Percentage(40),
            Constraint::Percentage(15),
            Constraint::Percentage(15),
            Constraint::Percentage(30),
        ],
    )
    .header(header)
    .block(Block::default().borders(Borders::ALL).title(" Nodes "));

    f.render_widget(table, area);
}

fn draw_topics(f: &mut Frame, app: &App, area: Rect) {
    if app.topics.is_empty() {
        let msg = Paragraph::new(
            "No active topics. Start a graph in another terminal:\n  cerulion graph run <graph>\n\nTopics show up here as soon as their publishers come online.",
        )
        .block(Block::default().borders(Borders::ALL).title(" Topics "));
        f.render_widget(msg, area);
        return;
    }

    let header = Row::new(vec![
        Cell::from("TOPIC").style(Style::default().add_modifier(Modifier::BOLD))
    ]);

    let (start, _visible) = viewport_window(app.topic_scroll, app.topics.len(), area.height);

    let rows: Vec<Row> = app
        .topics
        .iter()
        .enumerate()
        .skip(start)
        .map(|(i, topic)| {
            let style = if i == app.topic_scroll {
                Style::default().fg(Color::Black).bg(Color::Cyan)
            } else {
                Style::default()
            };
            Row::new(vec![Cell::from(topic.as_str())]).style(style)
        })
        .collect();

    let table = Table::new(rows, [Constraint::Percentage(100)])
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Topics (Enter to echo) "),
        );

    f.render_widget(table, area);
}

fn draw_echo(f: &mut Frame, app: &App, area: Rect) {
    let title = match &app.echo_topic {
        Some(topic) => format!(" Echo: {} ", topic),
        None => " Echo: (no topic selected) ".to_string(),
    };

    if app.echo_messages.is_empty() {
        let hint = if app.echo_topic.is_some() {
            "Waiting for messages..."
        } else {
            "Select a topic from the Topics tab and press Enter."
        };
        let msg = Paragraph::new(hint)
            .block(Block::default().borders(Borders::ALL).title(title.as_str()));
        f.render_widget(msg, area);
        return;
    }

    let header = Row::new(vec![
        Cell::from("SEQ").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("TIMESTAMP").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("SIZE").style(Style::default().add_modifier(Modifier::BOLD)),
        Cell::from("PAYLOAD").style(Style::default().add_modifier(Modifier::BOLD)),
    ]);

    // Show latest messages, auto-scroll to bottom
    let visible_height = area.height.saturating_sub(4) as usize; // borders + header
    let start = app.echo_messages.len().saturating_sub(visible_height);

    let rows: Vec<Row> = app.echo_messages[start..]
        .iter()
        .enumerate()
        .map(|(i, entry)| {
            let abs_idx = start + i;
            let style = if abs_idx == app.echo_scroll {
                Style::default().fg(Color::Black).bg(Color::Cyan)
            } else {
                Style::default()
            };
            Row::new(vec![
                Cell::from(entry.sequence.to_string()),
                Cell::from(format!("{}ns", entry.timestamp_ns)),
                Cell::from(format!("{}B", entry.size)),
                Cell::from(entry.payload_hex.clone()),
            ])
            .style(style)
        })
        .collect();

    let table = Table::new(
        rows,
        [
            Constraint::Length(8),
            Constraint::Length(22),
            Constraint::Length(10),
            Constraint::Min(20),
        ],
    )
    .header(header)
    .block(Block::default().borders(Borders::ALL).title(title.as_str()));

    f.render_widget(table, area);
}

fn draw_status_bar(f: &mut Frame, _app: &App, area: Rect) {
    let help = Line::from(vec![
        Span::styled("Tab", Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(": switch  "),
        Span::styled(
            "\u{2191}/\u{2193}",
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(": navigate  "),
        Span::styled("Enter", Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(": select  "),
        Span::styled("q", Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(": quit"),
    ]);
    let bar = Paragraph::new(help).style(Style::default().fg(Color::DarkGray));
    f.render_widget(bar, area);
}
