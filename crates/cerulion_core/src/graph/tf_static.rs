// SPDX-License-Identifier: AGPL-3.0-only
//! `/tf_static` is latched: a static transform reaches a listener that
//! starts after its publisher.
//!
//! A static transform is published once, so on a volatile topic a node that
//! starts after the publisher never sees it. Two rules close that:
//!
//! - **Latched by default.** An output on `/tf_static` with the schema
//!   `tf2_msgs/TFMessage` keeps its last message for late listeners even
//!   when its `history_size` is 0: its history is at least 1
//!   ([`effective_history_size`]). The publisher writes its whole static
//!   set, merged by child frame, into every message it sends, so that one
//!   kept message holds every static frame it publishes.
//! - **Listener caps are checked at graph load.** A late listener receives
//!   the kept messages of every `/tf_static` publisher at once. Every
//!   in-graph consumer of `/tf_static` must hold them all: its queue depth
//!   (an accumulate input's `cap`) must be at least the number of
//!   `/tf_static` outputs times the topic's history. A smaller cap would
//!   drop static frames without a word, so the load refuses it with
//!   [`TransportError::TfStaticCapTooSmall`], which names the publishers and
//!   the cap.
//! - **Start order does not matter.** A listener graph that starts before
//!   its static publishers creates `/tf_static` itself. It creates the topic
//!   with the latch history ([`listener_create_history`]), because a
//!   publisher that requires history cannot open a topic that keeps less,
//!   and the transport cannot raise the history of a topic that exists. The
//!   listener requires nothing when the topic already exists: it attaches to
//!   whatever history the creator set.
//!
//! All three rules read the graph config and the declared input depths
//! only, so a replay applies them exactly as the live run did.

use crate::error::TransportError;

use super::config::{GraphConfig, OutputDef};
use super::resolve_output_topic;
use super::topology::{TopicFlow, MAX_CONSUMER_DEPTH};

/// The topic of static transforms.
pub const TF_STATIC_TOPIC: &str = "/tf_static";

/// Whether `schema` names `tf2_msgs/TFMessage`: `tf2_msgs/TFMessage` or
/// `tf2_msgs/msg/TFMessage` (`::` separators are normalized to `/` at
/// parse). A bare `TFMessage` is not enough: a workspace schema of that
/// name is another type.
fn is_tf_message(schema: &str) -> bool {
    matches!(schema, "tf2_msgs/TFMessage" | "tf2_msgs/msg/TFMessage")
}

/// Whether an output publishing `schema` on the resolved `topic` is a
/// static transform output, latched by default.
pub(crate) fn is_latched_static_output(topic: &str, schema: &str) -> bool {
    topic == TF_STATIC_TOPIC && is_tf_message(schema)
}

/// The history a latched static transform output keeps for late listeners
/// when its `history_size` is 0: the one message that holds its whole set.
pub(crate) const LATCH_HISTORY: usize = 1;

/// The history an output keeps for late listeners: its `history_size`,
/// and at least [`LATCH_HISTORY`] for a static transform output.
pub(crate) fn effective_history_size(topic: &str, output: &OutputDef) -> usize {
    if is_latched_static_output(topic, &output.schema) {
        output.history_size.max(LATCH_HISTORY)
    } else {
        output.history_size
    }
}

/// Whether `output` keeps history only because it is a latched static
/// transform output: its `history_size` is 0, so the one message it keeps
/// is the latch, not a declared history. A diagnostic that names
/// `history_size` as the setting to change must not say so for this
/// output, since it is already 0.
pub(crate) fn is_implicit_latch(topic: &str, output: &OutputDef) -> bool {
    is_latched_static_output(topic, &output.schema) && output.history_size == 0
}

/// The history a listener graph gives `topic` when it creates it: the
/// latch history for `/tf_static`, nothing for any other topic.
///
/// A listener's input names its topic, not the schema it carries, so this
/// keys on the topic alone where [`is_latched_static_output`] also checks
/// the schema. The value raises only what the listener creates; it is never
/// an open requirement, so a listener attaches to a `/tf_static` that
/// already exists with any history.
pub(crate) fn listener_create_history(topic: &str) -> Option<usize> {
    (topic == TF_STATIC_TOPIC).then_some(LATCH_HISTORY)
}

/// Refuses an in-graph consumer of `/tf_static` whose queue cannot hold
/// what a late start delivers: the kept messages of every `/tf_static`
/// output in the graph.
///
/// `flow` is the topology's view of one topic; any topic but `/tf_static`,
/// and a `/tf_static` that keeps no history, passes untouched. Publishers
/// outside this graph are not counted: a listener of another graph's
/// static publishers sizes its cap by hand.
// hot-path-alloc-ok-fn: cold: graph BUILD, runs once per process, before the first step
pub(crate) fn check_listener_caps(
    config: &GraphConfig,
    flow: &TopicFlow,
) -> Result<(), TransportError> {
    if flow.topic != TF_STATIC_TOPIC {
        return Ok(());
    }
    let mut publishers = Vec::new();
    let mut depth = 0;
    for node in &config.nodes {
        for output in &node.outputs {
            let topic = resolve_output_topic(&config.prefix, &node.id, output);
            if topic == TF_STATIC_TOPIC {
                publishers.push(format!("{}.{}", node.id, output.name));
                depth = depth.max(effective_history_size(&topic, output));
            }
        }
    }
    if depth == 0 {
        return Ok(());
    }
    let required = publishers.len().saturating_mul(depth);
    let limit = if config.is_multi_publisher(TF_STATIC_TOPIC) {
        crate::transport::MULTI_TOPIC_BUFFER_CEILING
    } else {
        MAX_CONSUMER_DEPTH
    };
    match flow.consumers.iter().find(|edge| edge.depth < required) {
        None => Ok(()),
        Some(edge) => Err(TransportError::TfStaticCapTooSmall {
            node_id: edge.node_id.clone(),
            input: edge.input.clone(),
            cap: edge.depth,
            required,
            limit,
            depth,
            publishers,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::config::NodeDef;
    use crate::graph::node::BackpressurePolicy;
    use crate::graph::topology::ConsumerEdge;

    fn output(name: &str, topic: Option<&str>, schema: &str, history_size: usize) -> OutputDef {
        OutputDef {
            name: name.to_string(),
            schema: schema.to_string(),
            max_slice_len: None,
            topic: topic.map(str::to_string),
            history_size,
        }
    }

    fn node(id: &str, outputs: Vec<OutputDef>) -> NodeDef {
        NodeDef {
            fuse: None,
            params: Default::default(),
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs: vec![],
            outputs,
        }
    }

    fn graph(nodes: Vec<NodeDef>, multi: &[&str]) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: multi.iter().map(|t| t.to_string()).collect(),
            name: None,
            identity: "tf_static".to_string(),
            prefix: "robot".to_string(),
            nodes,
        }
    }

    fn statics(ids: &[&str], history_size: usize) -> Vec<NodeDef> {
        ids.iter()
            .map(|id| {
                node(
                    id,
                    vec![output(
                        "tf_static",
                        Some(TF_STATIC_TOPIC),
                        "tf2_msgs/TFMessage",
                        history_size,
                    )],
                )
            })
            .collect()
    }

    fn flow(topic: &str, caps: &[usize]) -> TopicFlow {
        TopicFlow {
            topic: topic.to_string(),
            producers: vec![],
            consumers: caps
                .iter()
                .enumerate()
                .map(|(i, cap)| ConsumerEdge {
                    node_id: format!("listener{i}"),
                    input: "tf_static".to_string(),
                    policy: BackpressurePolicy::default(),
                    depth: *cap,
                })
                .collect(),
        }
    }

    #[test]
    fn a_static_transform_output_keeps_at_least_one_message() {
        let latched = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessage", 0);
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &latched), 1);
        let long = output(
            "tf_static",
            Some(TF_STATIC_TOPIC),
            "tf2_msgs/msg/TFMessage",
            0,
        );
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &long), 1);
        // A declared history above 1 is kept as declared.
        let deep = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessage", 3);
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &deep), 3);
    }

    #[test]
    fn other_outputs_keep_their_declared_history() {
        // `/tf` moves: it stays volatile.
        let tf = output("tf", Some("/tf"), "tf2_msgs/TFMessage", 0);
        assert_eq!(effective_history_size("/tf", &tf), 0);
        // Another schema on `/tf_static` is not a static transform output.
        let other = output("tf_static", Some(TF_STATIC_TOPIC), "Vector3", 0);
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &other), 0);
        let near = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessages", 0);
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &near), 0);
        // A bare `TFMessage` may be a workspace schema of that name.
        let bare = output("tf_static", Some(TF_STATIC_TOPIC), "TFMessage", 0);
        assert_eq!(effective_history_size(TF_STATIC_TOPIC, &bare), 0);
        // A relative output named `tf_static` resolves under the prefix.
        let relative = output("tf_static", None, "tf2_msgs/TFMessage", 2);
        assert_eq!(
            effective_history_size("/robot/mounts/tf_static", &relative),
            2
        );
    }

    #[test]
    fn only_a_latched_output_with_no_declared_history_is_an_implicit_latch() {
        let latched = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessage", 0);
        assert!(is_implicit_latch(TF_STATIC_TOPIC, &latched));
        // A declared history is the user's setting, not the latch.
        let declared = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessage", 1);
        assert!(!is_implicit_latch(TF_STATIC_TOPIC, &declared));
        // Not latched at all: `/tf`, and another schema on `/tf_static`.
        let tf = output("tf", Some("/tf"), "tf2_msgs/TFMessage", 0);
        assert!(!is_implicit_latch("/tf", &tf));
        let other = output("tf_static", Some(TF_STATIC_TOPIC), "Vector3", 0);
        assert!(!is_implicit_latch(TF_STATIC_TOPIC, &other));
    }

    #[test]
    fn a_listener_creates_tf_static_with_the_latch_history_and_other_topics_with_none() {
        assert_eq!(listener_create_history(TF_STATIC_TOPIC), Some(1));
        // The history a listener creates is the one a publisher with no
        // `history_size` requires, so the two cannot drift apart.
        let latched = output("tf_static", Some(TF_STATIC_TOPIC), "tf2_msgs/TFMessage", 0);
        assert_eq!(
            listener_create_history(TF_STATIC_TOPIC),
            Some(effective_history_size(TF_STATIC_TOPIC, &latched))
        );
        // `/tf` moves, and a prefixed `tf_static` is another topic.
        assert_eq!(listener_create_history("/tf"), None);
        assert_eq!(listener_create_history("/robot/tf_static"), None);
    }

    #[test]
    fn a_cap_that_holds_every_publisher_passes() {
        let config = graph(statics(&["mounts", "arm", "head"], 0), &[TF_STATIC_TOPIC]);
        assert!(check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[3, 16])).is_ok());
    }

    #[test]
    fn a_cap_below_the_publisher_count_is_refused_naming_them() {
        let config = graph(statics(&["mounts", "arm", "head"], 0), &[TF_STATIC_TOPIC]);
        let err = check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[16, 2])).unwrap_err();
        match &err {
            TransportError::TfStaticCapTooSmall {
                node_id,
                input,
                cap,
                required,
                limit,
                depth,
                publishers,
            } => {
                assert_eq!(node_id, "listener1");
                assert_eq!(input, "tf_static");
                assert_eq!((*cap, *required, *limit, *depth), (2, 3, 16, 1));
                assert_eq!(
                    publishers,
                    &vec![
                        "mounts.tf_static".to_string(),
                        "arm.tf_static".to_string(),
                        "head.tf_static".to_string()
                    ]
                );
            }
            other => panic!("expected TfStaticCapTooSmall, got {other:?}"),
        }
        assert_eq!(
            err.to_string(),
            "node 'listener1' input 'tf_static' has cap 2, but /tf_static has 3 publishers \
             in this graph (mounts.tf_static, arm.tf_static, head.tf_static) that keep 1 \
             message each, so a listener that starts after them receives 3 and would drop \
             static transforms: raise the input's cap to at least 3"
        );
    }

    #[test]
    fn a_cap_that_cannot_hold_every_publisher_names_the_limit() {
        // 17 publishers keep 17 messages, more than the cap limit of 16 on a
        // multi-publisher topic: no cap can hold them.
        let ids: Vec<String> = (0..17).map(|i| format!("p{i}")).collect();
        let refs: Vec<&str> = ids.iter().map(String::as_str).collect();
        let config = graph(statics(&refs, 0), &[TF_STATIC_TOPIC]);
        let err = check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[16])).unwrap_err();
        let text = err.to_string();
        assert!(
            text.starts_with(
                "node 'listener0' input 'tf_static' has cap 16, but /tf_static has 17 \
                 publishers in this graph (p0.tf_static, p1.tf_static,"
            ),
            "{text}"
        );
        assert!(
            text.ends_with(
                "that keep 1 message each, so a listener that starts after them receives \
                 17, more than the cap limit of 16: move static transforms into fewer \
                 publishers"
            ),
            "{text}"
        );
    }

    #[test]
    fn a_deep_history_past_the_limit_names_history_size_as_a_remedy() {
        // 6 publishers keeping 3 each need 18: lowering history_size to 1
        // would bring that to 6.
        let config = graph(
            statics(&["a", "b", "c", "d", "e", "f"], 3),
            &[TF_STATIC_TOPIC],
        );
        let err = check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[16])).unwrap_err();
        assert!(
            err.to_string().ends_with(
                "that keep 3 messages each, so a listener that starts after them receives \
                 18, more than the cap limit of 16: move static transforms into fewer \
                 publishers, or lower their history_size to 1"
            ),
            "{err}"
        );
    }

    #[test]
    fn the_required_cap_is_publishers_times_the_deepest_history() {
        // Two publishers, one keeping 3: a late listener receives 3 from each.
        let mut nodes = statics(&["mounts"], 0);
        nodes.extend(statics(&["arm"], 3));
        let config = graph(nodes, &[TF_STATIC_TOPIC]);
        assert!(check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[6])).is_ok());
        match check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[5])) {
            Err(TransportError::TfStaticCapTooSmall {
                required: 6,
                depth: 3,
                cap: 5,
                ..
            }) => {}
            other => panic!("expected a refusal requiring 6, got {other:?}"),
        }
    }

    #[test]
    fn one_unlisted_publisher_is_held_to_the_consumer_depth_limit() {
        let config = graph(statics(&["mounts"], 0), &[]);
        assert!(check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[1])).is_ok());
        match check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[0])) {
            Err(TransportError::TfStaticCapTooSmall {
                required: 1,
                limit: MAX_CONSUMER_DEPTH,
                ..
            }) => {}
            other => panic!("expected a refusal under the depth limit, got {other:?}"),
        }
    }

    #[test]
    fn other_topics_and_unlatched_static_topics_are_not_checked() {
        let config = graph(statics(&["mounts", "arm"], 0), &[TF_STATIC_TOPIC]);
        assert!(check_listener_caps(&config, &flow("/tf", &[1])).is_ok());
        // `/tf_static` carrying another schema keeps no history: nothing to hold.
        let config = graph(
            vec![node(
                "odd",
                vec![output("tf_static", Some(TF_STATIC_TOPIC), "Vector3", 0)],
            )],
            &[],
        );
        assert!(check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[1])).is_ok());
    }

    #[test]
    fn the_verdict_is_the_same_on_every_run() {
        let config = graph(statics(&["mounts", "arm", "head"], 0), &[TF_STATIC_TOPIC]);
        let a = check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[2])).unwrap_err();
        let b = check_listener_caps(&config, &flow(TF_STATIC_TOPIC, &[2])).unwrap_err();
        assert_eq!(a.to_string(), b.to_string());
    }
}
