// SPDX-License-Identifier: AGPL-3.0-only
//! Where an ask or a wrist frame came from: the iceoryx2 node its publisher
//! runs under.
//!
//! A frame's origin ([`cerulion_core::ReceivedMessage::origin`]) is its
//! publisher's id. [`Publishers::source`] names that publisher through
//! [`TransportManager::topic_publisher_nodes`] on the first frame from it
//! and keeps the answer, so later frames cost one map lookup.
//!
//! `cerulion bag play` runs its transport under the node [`BAG_PLAY_NODE`],
//! and `cerulion bag play --resim` under [`RESIM_NODE_PREFIX`] followed by
//! the recording's identity. A frame either
//! one publishes is [`Source::Replay`] whatever its wire timestamp says, so a
//! replay is told apart from a live graph however its clock runs. A graph
//! run's own node is `cerulion_` followed by the graph's name (and, in a
//! multi-process run, `_` and the process group), so a graph named
//! `bag_play`, one whose name begins `replay_`, or a multi-process graph
//! named `replay` reads as a replay too.

use std::collections::HashMap;

use cerulion_core::{TopicPublisherNode, TransportManager};

/// The node `cerulion bag play` runs its transport under.
pub const BAG_PLAY_NODE: &str = "cerulion_bag_play";

/// The start of the node name a resim runs its transport under; the
/// recording's identity follows it.
pub const RESIM_NODE_PREFIX: &str = "cerulion_replay_";

/// The most publishers [`Publishers`] keeps; one more empties it first.
pub const PUBLISHERS_KEPT: usize = 256;

/// What the eye makes of the publisher of an ask or a frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    /// A live publisher: its asks are answered and its frames held.
    Live,
    /// A replay's publisher, with its node name: its asks get no request, no
    /// answer and no receipt, and its frames are not held.
    Replay(String),
    /// A publisher whose node's details cannot be read; its asks and frames
    /// are dropped as a replay's are.
    Unnamed,
}

impl Source {
    /// The source a publisher created under the node `name` is.
    pub fn of_node(name: Option<&str>) -> Self {
        match name {
            Some(name) if name == BAG_PLAY_NODE || name.starts_with(RESIM_NODE_PREFIX) => {
                Self::Replay(name.to_owned())
            }
            Some(_) => Self::Live,
            None => Self::Unnamed,
        }
    }
}

/// The publishers the eye has named, by id.
#[derive(Debug, Default)]
pub struct Publishers {
    known: HashMap<u128, Source>,
}

impl Publishers {
    /// The source of the publisher `origin`, if it was named.
    pub fn get(&self, origin: u128) -> Option<&Source> {
        self.known.get(&origin)
    }

    /// Name every publisher in `listed` not named yet, as listed on `topic`.
    /// Logs each replay and each unnamed publisher once.
    pub fn learn(&mut self, topic: &str, listed: &[TopicPublisherNode]) {
        for publisher in listed {
            if self.known.contains_key(&publisher.publisher) {
                continue;
            }
            if self.known.len() >= PUBLISHERS_KEPT {
                self.known.clear();
            }
            let source = Source::of_node(publisher.node_name.as_deref());
            match &source {
                Source::Replay(node) => tracing::warn!(
                    topic,
                    node = ?node,
                    "a replay publishes on an eye topic: its asks get no request, no answer \
                     and no receipt, and its frames are not held"
                ),
                Source::Unnamed => tracing::warn!(
                    topic,
                    "a publisher on an eye topic runs under a node whose details cannot be \
                     read: its asks get no request, no answer and no receipt, and its frames \
                     are not held"
                ),
                Source::Live => {}
            }
            self.known.insert(publisher.publisher, source);
        }
    }

    /// The source of a frame from `origin` read on `topic`: the one named
    /// before, else the one `manager` lists for it now. `None` when the
    /// publisher is not among the topic's publishers, such as one that has
    /// left, or the list cannot be read.
    pub fn source(
        &mut self,
        manager: &TransportManager,
        topic: &str,
        origin: u128,
    ) -> Option<Source> {
        if let Some(known) = self.known.get(&origin) {
            return Some(known.clone());
        }
        match manager.topic_publisher_nodes(topic) {
            Ok(listed) => self.learn(topic, &listed),
            Err(e) => tracing::debug!(topic, error = %e, "the topic's publishers were not listed"),
        }
        self.known.get(&origin).cloned()
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use cerulion_core::TopicPublisherNode;

    use super::{Publishers, Source, BAG_PLAY_NODE, PUBLISHERS_KEPT, RESIM_NODE_PREFIX};

    /// The names are the ones `bag play` and the resim mint their transports
    /// with in `cerulion_cli_engine`.
    #[test]
    fn the_names_are_the_ones_bag_play_and_the_resim_run_under() {
        let engine = Path::new(env!("CARGO_MANIFEST_DIR")).join("../cerulion_cli_engine/src");
        let read = |file: &str| {
            std::fs::read_to_string(engine.join(file)).unwrap_or_else(|e| panic!("{file}: {e}"))
        };
        let resim = read("replay_engine.rs");
        assert!(resim.contains(&format!(
            "pub const REPLAY_NODE_NAME_PREFIX: &str = \"{RESIM_NODE_PREFIX}\";"
        )));
        assert!(resim
            .contains(r#"node_name: format!("{REPLAY_NODE_NAME_PREFIX}{}", config.identity()),"#));
        let play = read("bag_cmd.rs");
        assert!(play.contains(&format!("node_name: \"{BAG_PLAY_NODE}\".to_string(),")));
    }

    #[test]
    fn the_two_replay_node_names_are_replays_and_any_other_name_is_live() {
        assert_eq!(
            Source::of_node(Some("cerulion_bag_play")),
            Source::Replay(String::from("cerulion_bag_play"))
        );
        assert_eq!(
            Source::of_node(Some("cerulion_replay_perception_20260101T000000Z")),
            Source::Replay(String::from("cerulion_replay_perception_20260101T000000Z"))
        );
        for live in [
            "cerulion_coffee",
            "cerulion_coffee_arm",
            "cerulion_bag_play_x",
            "cerulion_replay",
            "gate",
            "",
        ] {
            assert_eq!(Source::of_node(Some(live)), Source::Live, "{live}");
        }
        assert_eq!(Source::of_node(None), Source::Unnamed);
    }

    #[test]
    fn a_publisher_is_named_once_and_the_book_starts_over_when_full() {
        let mut book = Publishers::default();
        let listed = |ids: std::ops::Range<u128>| -> Vec<TopicPublisherNode> {
            ids.map(|publisher| TopicPublisherNode {
                publisher,
                node_name: Some(format!("cerulion_replay_{publisher}")),
            })
            .collect()
        };
        book.learn("/ask", &listed(0..2));
        assert!(matches!(book.get(1), Some(Source::Replay(_))));
        // A publisher already named keeps its name.
        book.learn(
            "/ask",
            &[TopicPublisherNode {
                publisher: 1,
                node_name: Some(String::from("gate")),
            }],
        );
        assert!(matches!(book.get(1), Some(Source::Replay(_))));
        let kept = u128::try_from(PUBLISHERS_KEPT).expect("kept");
        book.learn("/ask", &listed(2..kept));
        assert!(book.get(0).is_some());
        book.learn("/ask", &listed(kept..kept + 1));
        assert!(book.get(0).is_none(), "full: it starts over");
        assert!(book.get(kept).is_some());
    }
}
