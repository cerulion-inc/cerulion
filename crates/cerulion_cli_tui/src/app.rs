// SPDX-License-Identifier: AGPL-3.0-only
//! Application state for the Cerulion TUI dashboard.

use std::path::PathBuf;

use cerulion_cli_engine::node_metadata::NodeMetadata;

/// Active tab in the TUI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    Nodes,
    Topics,
    Echo,
}

impl Tab {
    pub const ALL: [Tab; 3] = [Tab::Nodes, Tab::Topics, Tab::Echo];

    pub fn label(self) -> &'static str {
        match self {
            Tab::Nodes => "Nodes",
            Tab::Topics => "Topics",
            Tab::Echo => "Echo",
        }
    }

    pub fn index(self) -> usize {
        match self {
            Tab::Nodes => 0,
            Tab::Topics => 1,
            Tab::Echo => 2,
        }
    }
}

/// A message received on a topic, stored for display.
#[derive(Debug, Clone)]
pub struct EchoEntry {
    pub sequence: u32,
    pub timestamp_ns: u64,
    pub schema_hash: u64,
    pub size: u32,
    pub payload_hex: String,
}

/// Central application state.
pub struct App {
    pub tab: Tab,
    pub running: bool,
    pub nodes: Vec<NodeMetadata>,
    pub topics: Vec<String>,
    pub echo_topic: Option<String>,
    pub echo_messages: Vec<EchoEntry>,
    pub node_scroll: usize,
    pub topic_scroll: usize,
    pub echo_scroll: usize,
    /// Workspace root if the TUI was launched inside one. `None` means
    /// the user can still browse live topics — the Nodes pane just
    /// shows a hint instead of an empty list.
    pub workspace_root: Option<PathBuf>,
}

impl App {
    pub fn new() -> Self {
        Self {
            tab: Tab::Nodes,
            running: true,
            nodes: Vec::new(),
            topics: Vec::new(),
            echo_topic: None,
            echo_messages: Vec::new(),
            node_scroll: 0,
            topic_scroll: 0,
            echo_scroll: 0,
            workspace_root: None,
        }
    }

    pub fn next_tab(&mut self) {
        let idx = self.tab.index();
        let next = (idx + 1) % Tab::ALL.len();
        self.tab = Tab::ALL[next];
    }

    pub fn prev_tab(&mut self) {
        let idx = self.tab.index();
        let prev = if idx == 0 {
            Tab::ALL.len() - 1
        } else {
            idx - 1
        };
        self.tab = Tab::ALL[prev];
    }

    pub fn scroll_up(&mut self) {
        let scroll = self.active_scroll_mut();
        *scroll = scroll.saturating_sub(1);
    }

    pub fn scroll_down(&mut self) {
        let max = self.active_list_len().saturating_sub(1);
        let scroll = self.active_scroll_mut();
        if *scroll < max {
            *scroll += 1;
        }
    }

    pub fn select_topic_for_echo(&mut self) {
        if self.tab == Tab::Topics {
            if let Some(topic) = self.topics.get(self.topic_scroll).cloned() {
                self.echo_topic = Some(topic);
                self.echo_messages.clear();
                self.echo_scroll = 0;
                self.tab = Tab::Echo;
            }
        }
    }

    pub fn push_echo(&mut self, entry: EchoEntry) {
        const MAX_ECHO: usize = 200;
        self.echo_messages.push(entry);
        if self.echo_messages.len() > MAX_ECHO {
            self.echo_messages.remove(0);
        }
    }

    fn active_scroll_mut(&mut self) -> &mut usize {
        match self.tab {
            Tab::Nodes => &mut self.node_scroll,
            Tab::Topics => &mut self.topic_scroll,
            Tab::Echo => &mut self.echo_scroll,
        }
    }

    fn active_list_len(&self) -> usize {
        match self.tab {
            Tab::Nodes => self.nodes.len(),
            Tab::Topics => self.topics.len(),
            Tab::Echo => self.echo_messages.len(),
        }
    }
}

impl Default for App {
    fn default() -> Self {
        Self::new()
    }
}
