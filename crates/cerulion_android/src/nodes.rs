// SPDX-License-Identifier: AGPL-3.0-only
//! Graph node types linked into the app library.
//!
//! An Android app runs its graph nodes from its own native library: the app
//! runtime crate depends on each node crate as a plain library
//! (`default-features = false`) and lists the node types once with
//! [`embed_nodes!`](crate::embed_nodes). The `JNI_OnLoad` that macro makes
//! fills this process-wide registry when Java loads the library, before any
//! runtime call. A graph start looks each YAML `type:` up here.

use std::collections::BTreeMap;
use std::ffi::{c_int, c_void};
use std::sync::{Mutex, MutexGuard};

pub use cerulion_core::graph::NodeEntry;

use crate::error::EmbedError;

/// Makes a fresh node entry. Each graph start calls it once per node.
pub type NodeFactory = fn() -> Box<dyn NodeEntry>;

/// One node type linked into the app library.
#[derive(Clone, Copy)]
pub struct LinkedNode {
    /// The graph YAML `type:` this node answers to.
    pub node_type: &'static str,
    /// Makes a fresh entry for the node.
    pub factory: NodeFactory,
}

/// The JNI version `JNI_OnLoad` reports.
const JNI_VERSION_1_6: c_int = 0x0001_0006;

#[derive(Default)]
struct Registry {
    nodes: BTreeMap<&'static str, NodeFactory>,
    /// The first registration error. Every later start with a graph reports
    /// it, so a bad `embed_nodes!` list cannot go unnoticed.
    error: Option<String>,
}

impl Registry {
    const fn new() -> Self {
        Self {
            nodes: BTreeMap::new(),
            error: None,
        }
    }

    /// Add `nodes` all together, or none of them when a type repeats.
    fn register(&mut self, nodes: &[LinkedNode]) -> Result<(), EmbedError> {
        for (index, node) in nodes.iter().enumerate() {
            let repeated = self.nodes.contains_key(node.node_type)
                || nodes[..index]
                    .iter()
                    .any(|earlier| earlier.node_type == node.node_type);
            if repeated {
                let message = format!(
                    "node type '{}' is registered twice; list each node type once in cerulion_android::embed_nodes!",
                    node.node_type
                );
                self.error.get_or_insert_with(|| message.clone());
                return Err(EmbedError::Graph(message));
            }
        }
        for node in nodes {
            self.nodes.insert(node.node_type, node.factory);
        }
        Ok(())
    }

    fn types(&self) -> Vec<String> {
        self.nodes.keys().map(|name| (*name).to_string()).collect()
    }

    fn check(&self) -> Result<(), EmbedError> {
        match &self.error {
            Some(error) => Err(EmbedError::Graph(error.clone())),
            None => Ok(()),
        }
    }

    fn make(&self, node_type: &str) -> Option<Box<dyn NodeEntry>> {
        self.nodes.get(node_type).map(|factory| factory())
    }

    fn unlinked(&self, node_type: &str) -> EmbedError {
        let linked = if self.nodes.is_empty() {
            "none".to_string()
        } else {
            self.types().join(", ")
        };
        EmbedError::Graph(format!(
            "node type '{node_type}' is not linked into this app library (linked: {linked}); add the node crate to the app runtime crate and list it in cerulion_android::embed_nodes!"
        ))
    }
}

static REGISTRY: Mutex<Registry> = Mutex::new(Registry::new());

fn registry() -> MutexGuard<'static, Registry> {
    REGISTRY.lock().unwrap_or_else(|error| error.into_inner())
}

/// Register node types linked into this library.
///
/// [`embed_nodes!`](crate::embed_nodes) calls it from `JNI_OnLoad`. An app
/// that writes its own `JNI_OnLoad` calls it there instead. A node type that
/// is already registered, or listed twice, is refused with a graph error,
/// nothing from `nodes` is added, and every later start with a graph fails
/// with the same error.
pub fn register(nodes: &[LinkedNode]) -> Result<(), EmbedError> {
    registry().register(nodes)
}

/// The registered node types, sorted.
pub fn linked_node_types() -> Vec<String> {
    registry().types()
}

/// The first registration error, as a graph error.
pub(crate) fn check() -> Result<(), EmbedError> {
    registry().check()
}

/// A fresh entry for `node_type`, or `None` when it is not linked.
pub(crate) fn make(node_type: &str) -> Option<Box<dyn NodeEntry>> {
    registry().make(node_type)
}

/// The graph error for a node type that is not linked; it names the type
/// and the linked types.
pub(crate) fn unlinked(node_type: &str) -> EmbedError {
    registry().unlinked(node_type)
}

/// The body of the `JNI_OnLoad` that [`embed_nodes!`](crate::embed_nodes)
/// makes.
///
/// `vm` and `reserved` are the `JNI_OnLoad` arguments; they are not used.
/// A registration error never fails the library load: it is logged, and
/// the first start with a graph reports it. Returns `JNI_VERSION_1_6`.
pub fn on_load(_vm: *mut c_void, _reserved: *mut c_void, nodes: &[LinkedNode]) -> c_int {
    if let Err(error) = register(nodes) {
        tracing::error!(error = %error, "node registration failed");
    }
    JNI_VERSION_1_6
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::codes;

    fn relay() -> Box<dyn NodeEntry> {
        Box::new(test_node_macro_relay_rlib::RelayNodeEntry::new())
    }

    fn period() -> Box<dyn NodeEntry> {
        Box::new(test_node_macro_period_cdylib::PeriodNodeEntry::new())
    }

    fn node(node_type: &'static str, factory: NodeFactory) -> LinkedNode {
        LinkedNode { node_type, factory }
    }

    const TWICE: &str = "node type 'timer' is registered twice; list each node type once in cerulion_android::embed_nodes!";

    #[test]
    fn registered_types_are_made_and_listed_sorted() {
        let mut registry = Registry::new();
        registry
            .register(&[node("relay", relay), node("period", period)])
            .expect("two distinct types");
        assert_eq!(registry.types(), ["period", "relay"]);
        assert!(registry.check().is_ok());
        let made = registry.make("relay").expect("relay is linked");
        let info = made.info().expect("relay info");
        assert_eq!(info.input_names(), ["cmd"]);
        assert_eq!(info.output_names(), ["echo"]);
        let made = registry.make("period").expect("period is linked");
        let info = made.info().expect("period info");
        assert!(info.input_names().is_empty());
        assert_eq!(info.output_names(), ["cmd"]);
        assert!(registry.make("timer").is_none());
    }

    #[test]
    fn a_type_registered_twice_is_refused_and_latched() {
        let mut registry = Registry::new();
        registry.register(&[node("timer", period)]).unwrap();
        let error = registry
            .register(&[node("relay", relay), node("timer", relay)])
            .expect_err("timer is already registered");
        assert_eq!(error.code(), codes::GRAPH);
        assert_eq!(error.to_string(), TWICE);
        // Nothing from the refused list was added.
        assert_eq!(registry.types(), ["timer"]);
        let latched = registry.check().expect_err("latched error");
        assert_eq!(latched.code(), codes::GRAPH);
        assert_eq!(latched.to_string(), TWICE);
        // A later, valid registration does not clear the first error.
        registry.register(&[node("relay", relay)]).unwrap();
        assert_eq!(registry.check().unwrap_err().to_string(), TWICE);
    }

    #[test]
    fn a_type_listed_twice_in_one_call_is_refused() {
        let mut registry = Registry::new();
        let error = registry
            .register(&[node("timer", relay), node("timer", period)])
            .expect_err("timer is listed twice");
        assert_eq!(error.to_string(), TWICE);
        assert!(registry.types().is_empty());
    }

    #[test]
    fn an_unlinked_type_names_itself_and_the_sorted_linked_list() {
        let mut registry = Registry::new();
        registry
            .register(&[node("relay", relay), node("printer", period)])
            .unwrap();
        let error = registry.unlinked("timer");
        assert_eq!(error.code(), codes::GRAPH);
        assert_eq!(
            error.to_string(),
            "node type 'timer' is not linked into this app library (linked: printer, relay); add the node crate to the app runtime crate and list it in cerulion_android::embed_nodes!"
        );
    }

    #[test]
    fn an_empty_registry_says_none_is_linked() {
        assert_eq!(
            Registry::new().unlinked("timer").to_string(),
            "node type 'timer' is not linked into this app library (linked: none); add the node crate to the app runtime crate and list it in cerulion_android::embed_nodes!"
        );
    }

    #[test]
    fn on_load_reports_jni_1_6() {
        assert_eq!(
            on_load(std::ptr::null_mut(), std::ptr::null_mut(), &[]),
            0x0001_0006
        );
    }
}
