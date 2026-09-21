// SPDX-License-Identifier: AGPL-3.0-only
//! Keyboard event handling for the TUI.

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use std::time::Duration;

use crate::app::App;

/// Result of processing an event.
pub enum EventResult {
    Continue,
    Quit,
}

/// Poll for a keyboard event with the given timeout.
/// Returns `None` if no event is available within the timeout.
pub fn poll_event(timeout: Duration) -> std::io::Result<Option<KeyEvent>> {
    if event::poll(timeout)? {
        if let Event::Key(key) = event::read()? {
            return Ok(Some(key));
        }
    }
    Ok(None)
}

/// Handle a key event, updating app state.
pub fn handle_key(app: &mut App, key: KeyEvent) -> EventResult {
    match key.code {
        KeyCode::Char('q') => EventResult::Quit,
        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => EventResult::Quit,
        KeyCode::Tab => {
            app.next_tab();
            EventResult::Continue
        }
        KeyCode::BackTab => {
            app.prev_tab();
            EventResult::Continue
        }
        KeyCode::Up => {
            app.scroll_up();
            EventResult::Continue
        }
        KeyCode::Down => {
            app.scroll_down();
            EventResult::Continue
        }
        KeyCode::Enter => {
            app.select_topic_for_echo();
            EventResult::Continue
        }
        _ => EventResult::Continue,
    }
}
