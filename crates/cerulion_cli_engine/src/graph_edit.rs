// SPDX-License-Identifier: AGPL-3.0-only
//! Surgical wire and node removal edits on a graph file: `graph.wire`,
//! `graph.unwire` and `graph.unstage` for `cerulion-wsd`.
//!
//! A wire is one `inputs:` entry on the consuming node: `name` is the input
//! port, `source` is `<node>/<port>` (or the producing output's absolute
//! `topic:` override). Wiring appends that entry, unwiring deletes it, and
//! unstaging deletes a whole node entry. Like `node stage`, none of these
//! re-serializes the file: every line outside the edited entry is copied
//! verbatim, so comments, key order, quoting and the `network:` block
//! survive, and the write goes through the same atomic writer with a
//! `.bak` of the prior bytes.
//!
//! The refusal contract is typed ([`GraphEditError`]) and nothing is written
//! on any refusal:
//! * [`GraphEditError::SchemaMismatch`]: the output and the input name
//!   different schemas (the same identity rule `graph validate` applies);
//! * [`GraphEditError::WouldBreak`]: unstaging a node other nodes read from,
//!   unless the caller passes `force`;
//! * [`CliError::Validation`]: anything else the engine refuses (an unknown
//!   node or port, an input that is already wired, no such wire, a graph the
//!   edit would leave invalid). Loops and levelization are not judged here;
//!   `graph validate` and `graph levels` are the full check.
//!
//! THE TOTAL GUARD. Line-oriented splicing cannot understand every legal
//! YAML spelling (flow-style entries, a key on the dash line, anchors). So
//! after each splice the result is re-parsed and must equal the in-memory
//! edit it was meant to produce; any other shape is refused with the file
//! untouched, never silently rewritten wrong.

use cerulion_core::graph::config::{GraphConfig, InputDef};
use cerulion_core::graph::{
    default_prefix, parse_graph_raw, resolve_output_topic, resolve_source, validate_graph_with,
    ValidationOptions,
};

use crate::error::CliError;
use crate::graph_cmd::{
    detect_list_item_indent, detect_nested_list_step, graph_read_raw, spelling_verdict,
    write_yaml_atomically_with, ExpectedPrior, PriorFile, SpellingVerdict,
};
use crate::workspace::CerulionWorkspace;
use crate::workspace_lock::WorkspaceLock;
use crate::yaml_splice::{
    is_blank, is_block_sequence_item, render_scalar, scan_top_level_block, split_lines, BlockScan,
    Line,
};

/// One end of a wire: a node id in the graph and a port name on it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortRef {
    pub node: String,
    pub port: String,
}

/// A wire: the `from` output feeds the `to` input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Wire {
    pub from: PortRef,
    pub to: PortRef,
}

/// Why a graph edit was refused. Nothing was written in any case.
#[derive(Debug)]
pub enum GraphEditError {
    /// The output and the input name different schemas.
    SchemaMismatch {
        /// What the consuming input expects.
        expected: String,
        /// What the producing output provides.
        found: String,
        /// The engine's own explanation.
        detail: String,
    },
    /// Unstaging would leave these wires without a producer.
    WouldBreak { wires: Vec<Wire> },
    /// Any other engine refusal or failure.
    Cli(CliError),
}

impl From<CliError> for GraphEditError {
    fn from(error: CliError) -> Self {
        Self::Cli(error)
    }
}

impl From<std::io::Error> for GraphEditError {
    fn from(error: std::io::Error) -> Self {
        Self::Cli(error.into())
    }
}

impl std::fmt::Display for GraphEditError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SchemaMismatch { detail, .. } => f.write_str(detail),
            Self::WouldBreak { wires } => write!(
                f,
                "removing this node would break {} wire(s): {}",
                wires.len(),
                wires
                    .iter()
                    .map(|w| format!(
                        "{}.{} -> {}.{}",
                        w.from.node, w.from.port, w.to.node, w.to.port
                    ))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            Self::Cli(error) => write!(f, "{error}"),
        }
    }
}

/// A successful edit: the graph file's new bytes and, for `unstage`, the
/// wires that were removed together with the node (empty without `force`).
#[derive(Debug)]
pub struct EditOutcome {
    pub raw: String,
    pub removed_wires: Vec<Wire>,
}

fn refuse(message: impl Into<String>) -> GraphEditError {
    GraphEditError::Cli(CliError::Validation(message.into()))
}

/// The in-memory state every edit starts from: the lock, the file bytes and
/// a config parsed from exactly those bytes.
struct Loaded {
    _lock: WorkspaceLock,
    raw: String,
    config: GraphConfig,
}

fn load(workspace: &CerulionWorkspace, graph: &str) -> Result<Loaded, GraphEditError> {
    let lock = WorkspaceLock::acquire_and_track_gitignore(&workspace.root)?;
    let (_, raw) = graph_read_raw(&workspace.graphs_dir, graph)?;
    let config = parse_graph_raw(&raw).map_err(CliError::from)?;
    Ok(Loaded {
        _lock: lock,
        raw,
        config,
    })
}

/// The `source:` text a wire from `node`'s output `port` carries: the
/// absolute `topic:` override when the output has one (a relative
/// `<node>/<port>` would not resolve to it), else `<node>/<port>`.
fn wire_source(config: &GraphConfig, node: &str, port: &str) -> Result<String, GraphEditError> {
    let node_def = config
        .nodes
        .iter()
        .find(|n| n.id == node)
        .ok_or_else(|| refuse(format!("node '{node}' is not in this graph")))?;
    if node_def.is_ros2() {
        return Err(refuse(format!(
            "node '{node}' is a ROS 2 entry; it has no wirable ports"
        )));
    }
    let output = node_def
        .outputs
        .iter()
        .find(|o| o.name == port)
        .ok_or_else(|| {
            refuse(format!(
                "node '{node}' has no output '{port}' in this graph"
            ))
        })?;
    Ok(match &output.topic {
        Some(topic) => topic.clone(),
        None => format!("{node}/{port}"),
    })
}

/// Wire `from`'s output to `to`'s input: append one `inputs:` entry to the
/// consuming node.
pub fn graph_wire(
    workspace: &CerulionWorkspace,
    graph: &str,
    from: &PortRef,
    to: &PortRef,
) -> Result<EditOutcome, GraphEditError> {
    let Loaded {
        raw, mut config, ..
    } = load(workspace, graph)?;
    let source = wire_source(&config, &from.node, &from.port)?;
    let to_index = config
        .nodes
        .iter()
        .position(|n| n.id == to.node)
        .ok_or_else(|| refuse(format!("node '{}' is not in this graph", to.node)))?;
    let consumer = &config.nodes[to_index];
    if consumer.is_ros2() {
        return Err(refuse(format!(
            "node '{}' is a ROS 2 entry; it has no wirable ports",
            to.node
        )));
    }
    if let Some(existing) = consumer.inputs.iter().find(|i| i.name == to.port) {
        return Err(refuse(format!(
            "input '{}' of node '{}' is already wired to '{}'; unwire it first",
            to.port, to.node, existing.source
        )));
    }
    check_schemas(workspace, &config, from, to, to_index)?;

    let input = InputDef {
        name: to.port.clone(),
        source,
    };
    config.nodes[to_index].inputs.push(input.clone());
    validate_after_edit(&config)?;
    let out = splice_add_input(&raw, to_index, &input)?;
    commit(workspace, graph, &raw, out, &config, Vec::new())
}

/// Refuse a wire whose two ends name different schemas. Unknown on either
/// side (a raw-FFI node declares no schema name) is not a mismatch: the full
/// check is `graph validate`.
fn check_schemas(
    workspace: &CerulionWorkspace,
    config: &GraphConfig,
    from: &PortRef,
    to: &PortRef,
    to_index: usize,
) -> Result<(), GraphEditError> {
    let consumer = &config.nodes[to_index];
    let metadata = crate::node_cmd::node_info(&workspace.nodes_dir, &consumer.node_type)?;
    let input_port = metadata
        .inputs
        .iter()
        .find(|p| p.name == to.port)
        .ok_or_else(|| {
            refuse(format!(
                "node type '{}' declares no input '{}'",
                consumer.node_type, to.port
            ))
        })?;
    let Some(in_schema) = &input_port.schema else {
        return Ok(());
    };
    // The producer's schema as the graph states it; a graph that left it
    // empty falls back to what the producing node type declares.
    let producer = config.nodes.iter().find(|n| n.id == from.node);
    let yaml_schema = producer
        .and_then(|n| n.outputs.iter().find(|o| o.name == from.port))
        .map(|o| o.schema.clone())
        .filter(|s| !s.is_empty());
    let out_schema = match yaml_schema {
        Some(schema) => schema,
        None => match producer
            .and_then(|n| crate::node_cmd::node_info(&workspace.nodes_dir, &n.node_type).ok())
            .and_then(|m| m.outputs.into_iter().find(|p| p.name == from.port))
            .and_then(|p| p.schema)
        {
            Some(schema) => schema,
            None => return Ok(()),
        },
    };
    let verdict = spelling_verdict(&workspace.schemas_dir, in_schema, &out_schema);
    let agrees = match &verdict {
        SpellingVerdict::Same => true,
        SpellingVerdict::Different => input_port.schema_alternatives.iter().any(|alt| {
            spelling_verdict(&workspace.schemas_dir, alt, &out_schema) == SpellingVerdict::Same
        }),
        SpellingVerdict::Refused(_) => false,
    };
    if agrees {
        return Ok(());
    }
    let detail = match verdict {
        SpellingVerdict::Refused(reason) => reason,
        _ => format!(
            "input '{}' of node '{}' expects '{in_schema}' but output '{}' of node '{}' provides \
             '{out_schema}'",
            to.port, to.node, from.port, from.node
        ),
    };
    Err(GraphEditError::SchemaMismatch {
        expected: in_schema.clone(),
        found: out_schema,
        detail,
    })
}

/// The prefix the graph resolves topics under: its own `prefix:` line, else
/// the host-derived default it runs with. Only for matching topics; the
/// authored file never gains the line.
fn effective_prefix(config: &GraphConfig) -> String {
    if config.prefix.is_empty() {
        default_prefix("")
    } else {
        config.prefix.clone()
    }
}

/// The same validation `node stage` runs before it writes, with the engine's
/// refusal text carried as a refusal rather than a transport fault.
fn validate_after_edit(config: &GraphConfig) -> Result<(), GraphEditError> {
    // A graph that omits `prefix:` runs under a host-derived default, but the
    // reader here keeps it empty (so a rewrite never invents the line) and the
    // validator refuses an empty prefix. Validate a copy under the prefix the
    // graph resolves to at run time.
    let resolved;
    let config = if config.prefix.is_empty() {
        let mut copy = config.clone();
        copy.prefix = effective_prefix(config);
        resolved = copy;
        &resolved
    } else {
        config
    };
    validate_graph_with(
        config,
        ValidationOptions {
            require_output_schema: false,
            ..Default::default()
        },
    )
    .map_err(|error| refuse(error.to_string()))
}

/// Remove the wire `from` -> `to`: delete the matching `inputs:` entry of
/// the consuming node.
pub fn graph_unwire(
    workspace: &CerulionWorkspace,
    graph: &str,
    from: &PortRef,
    to: &PortRef,
) -> Result<EditOutcome, GraphEditError> {
    let Loaded {
        raw, mut config, ..
    } = load(workspace, graph)?;
    let source = wire_source(&config, &from.node, &from.port)?;
    let prefix = effective_prefix(&config);
    let wanted = resolve_source(&prefix, &source);
    let to_index = config
        .nodes
        .iter()
        .position(|n| n.id == to.node)
        .ok_or_else(|| refuse(format!("node '{}' is not in this graph", to.node)))?;
    let input_position = config.nodes[to_index]
        .inputs
        .iter()
        .position(|i| i.name == to.port && resolve_source(&prefix, &i.source) == wanted)
        .ok_or_else(|| {
            refuse(format!(
                "no wire from '{}.{}' to '{}.{}' in this graph",
                from.node, from.port, to.node, to.port
            ))
        })?;
    config.nodes[to_index].inputs.remove(input_position);
    let out = splice_remove_input(&raw, to_index, input_position)?;
    commit(workspace, graph, &raw, out, &config, Vec::new())
}

/// Remove the node `node` from the graph. Other nodes' inputs wired to its
/// outputs are listed in [`GraphEditError::WouldBreak`] and nothing is
/// written, unless `force` is set: then those inputs are removed too.
pub fn graph_unstage(
    workspace: &CerulionWorkspace,
    graph: &str,
    node: &str,
    force: bool,
) -> Result<EditOutcome, GraphEditError> {
    let Loaded {
        raw, mut config, ..
    } = load(workspace, graph)?;
    let index = config
        .nodes
        .iter()
        .position(|n| n.id == node)
        .ok_or_else(|| refuse(format!("node '{node}' is not in this graph")))?;
    let prefix = effective_prefix(&config);
    let produced: Vec<(String, String)> = config.nodes[index]
        .outputs
        .iter()
        .map(|o| (o.name.clone(), resolve_output_topic(&prefix, node, o)))
        .collect();
    // (consumer index, input position, wire) for every other node's input
    // fed by one of this node's outputs.
    let mut dependents: Vec<(usize, usize, Wire)> = Vec::new();
    // Topics another node still publishes (a `multi_publisher_topics` entry):
    // an input that reads one of them keeps a producer without this node.
    let still_produced: Vec<String> = config
        .nodes
        .iter()
        .enumerate()
        .filter(|(other, _)| *other != index)
        .flat_map(|(_, n)| {
            n.outputs
                .iter()
                .map(|o| resolve_output_topic(&prefix, &n.id, o))
                .collect::<Vec<_>>()
        })
        .collect();
    for (consumer_index, consumer) in config.nodes.iter().enumerate() {
        if consumer_index == index {
            continue;
        }
        for (input_position, input) in consumer.inputs.iter().enumerate() {
            let key = resolve_source(&prefix, &input.source);
            if still_produced.contains(&key) {
                continue;
            }
            if let Some((port, _)) = produced.iter().find(|(_, topic)| *topic == key) {
                dependents.push((
                    consumer_index,
                    input_position,
                    Wire {
                        from: PortRef {
                            node: node.to_string(),
                            port: port.clone(),
                        },
                        to: PortRef {
                            node: consumer.id.clone(),
                            port: input.name.clone(),
                        },
                    },
                ));
            }
        }
    }
    if !dependents.is_empty() && !force {
        return Err(GraphEditError::WouldBreak {
            wires: dependents.into_iter().map(|(_, _, wire)| wire).collect(),
        });
    }

    // Remove the dependent inputs back to front per node so earlier
    // positions stay valid, then the node entry itself.
    let mut out = raw.clone();
    let mut by_removal = dependents.clone();
    by_removal.sort_by_key(|dependent| std::cmp::Reverse((dependent.0, dependent.1)));
    for (consumer_index, input_position, _) in &by_removal {
        out = splice_remove_input(&out, *consumer_index, *input_position)?;
        config.nodes[*consumer_index].inputs.remove(*input_position);
    }
    out = splice_remove_node(&out, index)?;
    config.nodes.remove(index);
    validate_after_edit(&config)?;
    let removed = dependents.into_iter().map(|(_, _, wire)| wire).collect();
    commit(workspace, graph, &raw, out, &config, removed)
}

/// Enforce the total guard, then write atomically against the bytes the edit
/// was derived from.
fn commit(
    workspace: &CerulionWorkspace,
    graph: &str,
    raw: &str,
    out: String,
    intended: &GraphConfig,
    removed_wires: Vec<Wire>,
) -> Result<EditOutcome, GraphEditError> {
    let reparsed = parse_graph_raw(&out).map_err(|e| {
        refuse(format!(
            "this graph file's layout cannot be edited in place ({e}); nothing was written"
        ))
    })?;
    let same = serde_json::to_value(&reparsed).ok() == serde_json::to_value(intended).ok();
    if !same {
        return Err(refuse(
            "this graph file's layout cannot be edited in place (the spliced result does not \
             match the intended edit, for example a flow-style `nodes:` entry); nothing was \
             written. Edit the file by hand.",
        ));
    }
    let path = workspace.graphs_dir.join(format!("{graph}.yaml"));
    write_yaml_atomically_with(
        &path,
        &out,
        "graph file",
        ExpectedPrior::Contents(raw),
        PriorFile::BackUp,
    )?;
    Ok(EditOutcome {
        raw: out,
        removed_wires,
    })
}

// ---------------------------------------------------------------------------
// Pure line-oriented splices. Each takes the document and returns the new one;
// none writes. The caller's total guard decides whether the result is trusted.
// ---------------------------------------------------------------------------

/// Where one node entry sits in the document.
struct NodeSpan {
    /// Line index of the entry's `- ` line.
    first: usize,
    /// One past the last line that belongs to the entry (trailing blank and
    /// comment lines excluded: they are not attributable to it).
    end: usize,
}

fn indent_of(content: &str) -> usize {
    content.len() - content.trim_start().len()
}

fn is_comment_or_blank(content: &str) -> bool {
    is_blank(content) || content.trim_start().starts_with('#')
}

/// The newline this document uses on `line`.
fn eol_of(raw: &str, line: &Line<'_>) -> &'static str {
    if raw[line.start..line.end].ends_with("\r\n") {
        "\r\n"
    } else {
        "\n"
    }
}

/// Index of the last non-blank, non-comment line in `[from, to)`.
fn last_content_line(lines: &[Line<'_>], from: usize, to: usize) -> Option<usize> {
    (from..to)
        .rev()
        .find(|&i| !is_comment_or_blank(lines[i].content))
}

/// Node entry spans of the sole top-level `nodes:` block, plus the document's
/// node-item indent.
fn node_spans(raw: &str, lines: &[Line<'_>]) -> Result<(Vec<NodeSpan>, usize), GraphEditError> {
    let (start, end) = match scan_top_level_block(lines, "nodes") {
        BlockScan::One { start, end } => (start, end),
        BlockScan::Absent => return Err(refuse("this graph has no top-level `nodes:` block")),
        BlockScan::Duplicate { .. } => {
            return Err(refuse(
                "this graph has more than one top-level `nodes:` key",
            ))
        }
    };
    let key_line = lines
        .iter()
        .position(|l| l.start == start)
        .ok_or_else(|| refuse("this graph's `nodes:` block could not be located"))?;
    let block_end = lines
        .iter()
        .rposition(|l| l.end == end)
        .map(|i| i + 1)
        .ok_or_else(|| refuse("this graph's `nodes:` block could not be located"))?;
    let item_indent = detect_list_item_indent(raw, start, end)
        .map(str::len)
        .ok_or_else(|| refuse("this graph's `nodes:` block holds no node entries"))?;
    let firsts: Vec<usize> = (key_line + 1..block_end)
        .filter(|&i| {
            is_block_sequence_item(lines[i].content) && indent_of(lines[i].content) == item_indent
        })
        .collect();
    let mut spans = Vec::with_capacity(firsts.len());
    for (n, &first) in firsts.iter().enumerate() {
        let limit = firsts.get(n + 1).copied().unwrap_or(block_end);
        let last = last_content_line(lines, first, limit).unwrap_or(first);
        spans.push(NodeSpan {
            first,
            end: last + 1,
        });
    }
    Ok((spans, item_indent))
}

fn join(raw: &str, lines: &[Line<'_>], skip: impl Fn(usize) -> bool) -> String {
    let mut out = String::with_capacity(raw.len());
    for (i, line) in lines.iter().enumerate() {
        if !skip(i) {
            out.push_str(&raw[line.start..line.end]);
        }
    }
    out
}

/// Delete the node entry at `index` (position in `nodes:`).
fn splice_remove_node(raw: &str, index: usize) -> Result<String, GraphEditError> {
    let lines = split_lines(raw);
    let (spans, _) = node_spans(raw, &lines)?;
    let span = spans
        .get(index)
        .ok_or_else(|| refuse("the node entry could not be located in the file"))?;
    Ok(join(raw, &lines, |i| i >= span.first && i < span.end))
}

/// The `inputs:` key line of the node entry, if it has one. A key sharing
/// the dash line (`- inputs:`) is not edited in place.
fn find_inputs_key(
    lines: &[Line<'_>],
    span: &NodeSpan,
    item_indent: usize,
) -> Result<Option<usize>, GraphEditError> {
    let dash = lines[span.first].content.trim_start();
    if dash
        .trim_start_matches('-')
        .trim_start()
        .starts_with("inputs:")
    {
        return Err(refuse(
            "this node entry starts with `inputs:` on its dash line, which is not edited in \
             place; nothing was written",
        ));
    }
    let key_indent = item_indent + 2;
    let found: Vec<usize> = (span.first + 1..span.end)
        .filter(|&i| {
            indent_of(lines[i].content) == key_indent
                && lines[i].content.trim_start().starts_with("inputs:")
        })
        .collect();
    match found.as_slice() {
        [] => Ok(None),
        [one] => Ok(Some(*one)),
        _ => Err(refuse("this node entry has more than one `inputs:` key")),
    }
}

/// End (exclusive) of the `inputs:` block opened at `key_line`: members are
/// lines indented past the key, or sequence items at the key's own column.
/// Blank and comment lines never end the block.
fn inputs_block_end(lines: &[Line<'_>], key_line: usize, limit: usize) -> usize {
    let key_indent = indent_of(lines[key_line].content);
    let mut last = key_line;
    for (i, line) in lines.iter().enumerate().take(limit).skip(key_line + 1) {
        let content = line.content;
        if is_comment_or_blank(content) {
            continue;
        }
        let indent = indent_of(content);
        if indent > key_indent || (indent == key_indent && is_block_sequence_item(content)) {
            last = i;
        } else {
            break;
        }
    }
    last + 1
}

/// Append `input` to node `index`'s `inputs:`, creating the key when absent.
fn splice_add_input(raw: &str, index: usize, input: &InputDef) -> Result<String, GraphEditError> {
    let lines = split_lines(raw);
    let (spans, item_indent) = node_spans(raw, &lines)?;
    let span = spans
        .get(index)
        .ok_or_else(|| refuse("the node entry could not be located in the file"))?;
    let nl = eol_of(raw, &lines[span.first]);
    let key_indent = item_indent + 2;
    let nested = detect_nested_list_step(raw).unwrap_or(2);
    let name = render_scalar(&input.name);
    let source = render_scalar(&input.source);

    let entry = |item: &str| format!("{item}- name: {name}{nl}{item}  source: {source}{nl}");

    match find_inputs_key(&lines, span, item_indent)? {
        None => {
            let pad = " ".repeat(key_indent);
            let item = " ".repeat(key_indent + nested);
            let text = format!("{pad}inputs:{nl}{}", entry(&item));
            // Inputs sit before outputs, as `node stage` renders an entry.
            let outputs_key = (span.first + 1..span.end).find(|&i| {
                indent_of(lines[i].content) == key_indent
                    && lines[i].content.trim_start().starts_with("outputs:")
            });
            match outputs_key {
                Some(outputs) => {
                    // Own-line comments right above `outputs:` describe it, so
                    // the new block goes above them, not between.
                    let mut at = outputs;
                    while at > span.first + 1 && lines[at - 1].content.trim_start().starts_with('#')
                    {
                        at -= 1;
                    }
                    insert_after(raw, &lines, at - 1, &text, nl)
                }
                None => insert_after(raw, &lines, span.end - 1, &text, nl),
            }
        }
        Some(key_line) => {
            let key_content = lines[key_line].content;
            let after_key = key_content.trim_start()["inputs:".len()..].trim();
            let block_end = inputs_block_end(&lines, key_line, span.end);
            if after_key.is_empty() || after_key.starts_with('#') {
                // An existing block: copy the first item's own indentation.
                let item_indent_str = (key_line + 1..block_end)
                    .find(|&i| is_block_sequence_item(lines[i].content))
                    .map(|i| " ".repeat(indent_of(lines[i].content)))
                    .unwrap_or_else(|| " ".repeat(key_indent + nested));
                insert_after(raw, &lines, block_end - 1, &entry(&item_indent_str), nl)
            } else if after_key.starts_with("[]")
                && (after_key[2..].trim().is_empty() || after_key[2..].trim().starts_with('#'))
            {
                // `inputs: []`: replace the empty flow sequence by a block.
                let comment = after_key[2..].trim();
                let item = " ".repeat(key_indent + nested);
                let key_text = if comment.is_empty() {
                    format!("{}inputs:{nl}", " ".repeat(key_indent))
                } else {
                    format!("{}inputs: {comment}{nl}", " ".repeat(key_indent))
                };
                let mut out = String::with_capacity(raw.len() + 64);
                for (i, line) in lines.iter().enumerate() {
                    if i == key_line {
                        out.push_str(&key_text);
                        out.push_str(&entry(&item));
                    } else {
                        out.push_str(&raw[line.start..line.end]);
                    }
                }
                Ok(out)
            } else {
                Err(refuse(
                    "this node's `inputs:` is a flow-style value, which is not edited in place; \
                     nothing was written",
                ))
            }
        }
    }
}

/// Insert `text` after line `after`, adding a newline first when that line is
/// the newline-less last line of the file.
fn insert_after(
    raw: &str,
    lines: &[Line<'_>],
    after: usize,
    text: &str,
    nl: &str,
) -> Result<String, GraphEditError> {
    let mut out = String::with_capacity(raw.len() + text.len() + 2);
    for (i, line) in lines.iter().enumerate() {
        out.push_str(&raw[line.start..line.end]);
        if i == after {
            if !out.ends_with('\n') {
                out.push_str(nl);
            }
            out.push_str(text);
        }
    }
    Ok(out)
}

/// Delete the `position`-th entry of node `index`'s `inputs:`; when it is the
/// only one, the `inputs:` key goes with it.
fn splice_remove_input(raw: &str, index: usize, position: usize) -> Result<String, GraphEditError> {
    let lines = split_lines(raw);
    let (spans, item_indent) = node_spans(raw, &lines)?;
    let span = spans
        .get(index)
        .ok_or_else(|| refuse("the node entry could not be located in the file"))?;
    let key_line = find_inputs_key(&lines, span, item_indent)?
        .ok_or_else(|| refuse("the node entry has no `inputs:` key to remove from"))?;
    let after_key = lines[key_line].content.trim_start()["inputs:".len()..].trim();
    if !(after_key.is_empty() || after_key.starts_with('#')) {
        return Err(refuse(
            "this node's `inputs:` is a flow-style value, which is not edited in place; nothing \
             was written",
        ));
    }
    let block_end = inputs_block_end(&lines, key_line, span.end);
    let item_indent_len = (key_line + 1..block_end)
        .find(|&i| is_block_sequence_item(lines[i].content))
        .map(|i| indent_of(lines[i].content))
        .ok_or_else(|| refuse("the `inputs:` block holds no entries"))?;
    let firsts: Vec<usize> = (key_line + 1..block_end)
        .filter(|&i| {
            is_block_sequence_item(lines[i].content)
                && indent_of(lines[i].content) == item_indent_len
        })
        .collect();
    let first = *firsts
        .get(position)
        .ok_or_else(|| refuse("the input entry could not be located in the file"))?;
    let limit = firsts.get(position + 1).copied().unwrap_or(block_end);
    let last = last_content_line(&lines, first, limit).unwrap_or(first);
    let only_one = firsts.len() == 1;
    // Own-line comments between the key and its only entry describe that
    // entry, so they go with it rather than being left stranded.
    let first = if only_one { key_line + 1 } else { first };
    if only_one && after_key.starts_with('#') {
        // `inputs: # note`: the comment is the author's, so the key stays as an
        // empty list carrying it.
        let nl = eol_of(raw, &lines[key_line]);
        let pad = " ".repeat(indent_of(lines[key_line].content));
        let mut out = String::with_capacity(raw.len());
        for (i, line) in lines.iter().enumerate() {
            if i == key_line {
                out.push_str(&format!("{pad}inputs: [] {after_key}{nl}"));
            } else if !(i >= first && i <= last) {
                out.push_str(&raw[line.start..line.end]);
            }
        }
        return Ok(out);
    }
    Ok(join(raw, &lines, |i| {
        (i >= first && i <= last) || (only_one && i == key_line)
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(name: &str, source: &str) -> InputDef {
        InputDef {
            name: name.to_string(),
            source: source.to_string(),
        }
    }

    const DOC: &str = "\
# header
prefix: robot
nodes:
  # the camera
  - id: camera
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
  - id: det # trailing
    type: detector
    inputs:
      - name: image
        source: camera/image
    outputs:
      - name: boxes
        schema: vision/Boxes
  - id: sink
    type: sink

network:
  mode: disabled
";

    #[test]
    fn add_input_appends_to_an_existing_block_and_creates_a_missing_one() {
        let out = splice_add_input(DOC, 1, &input("extra", "camera/image")).unwrap();
        assert!(out.contains(
            "      - name: image\n        source: camera/image\n      - name: extra\n        source: camera/image\n    outputs:\n      - name: boxes"
        ));
        let out = splice_add_input(DOC, 2, &input("boxes", "det/boxes")).unwrap();
        assert!(out.contains(
            "  - id: sink\n    type: sink\n    inputs:\n      - name: boxes\n        source: det/boxes\n\nnetwork:"
        ));
        // Every other line is byte-identical.
        let rest: String = DOC.lines().take(14).collect::<Vec<_>>().join("\n");
        assert!(out.starts_with(&rest));
    }

    #[test]
    fn a_new_inputs_key_goes_before_outputs() {
        let out = splice_add_input(DOC, 0, &input("x", "other/y")).unwrap();
        assert!(out.contains(
            "  - id: camera\n    type: camera\n    inputs:\n      - name: x\n        source: other/y\n    outputs:\n"
        ));
    }

    #[test]
    fn remove_input_deletes_the_entry_and_the_key_when_it_was_the_last() {
        let out = splice_remove_input(DOC, 1, 0).unwrap();
        assert!(!out.contains("source: camera/image"));
        assert!(!out.contains("    inputs:\n"));
        assert!(out.contains("  - id: det # trailing\n    type: detector\n    outputs:"));
    }

    #[test]
    fn remove_node_keeps_the_comment_above_and_everything_after() {
        let out = splice_remove_node(DOC, 0).unwrap();
        assert!(out.starts_with("# header\nprefix: robot\nnodes:\n  # the camera\n  - id: det"));
        assert!(out.ends_with("    type: sink\n\nnetwork:\n  mode: disabled\n"));
    }

    #[test]
    fn crlf_and_missing_final_newline_are_kept() {
        let crlf = "prefix: r\r\nnodes:\r\n  - id: a\r\n    type: a\r\n";
        let out = splice_add_input(crlf, 0, &input("x", "b/y")).unwrap();
        assert_eq!(
            out,
            "prefix: r\r\nnodes:\r\n  - id: a\r\n    type: a\r\n    inputs:\r\n      - name: x\r\n        source: b/y\r\n"
        );
        let bare = "prefix: r\nnodes:\n  - id: a\n    type: a";
        let out = splice_add_input(bare, 0, &input("x", "b/y")).unwrap();
        assert_eq!(
            out,
            "prefix: r\nnodes:\n  - id: a\n    type: a\n    inputs:\n      - name: x\n        source: b/y\n"
        );
    }

    #[test]
    fn serde_style_column_zero_lists_are_edited_in_their_own_indentation() {
        let doc = "prefix: r\nnodes:\n- id: a\n  type: a\n  inputs:\n  - name: p\n    source: b/o\n- id: b\n  type: b\n";
        let out = splice_add_input(doc, 0, &input("q", "b/o2")).unwrap();
        assert_eq!(
            out,
            "prefix: r\nnodes:\n- id: a\n  type: a\n  inputs:\n  - name: p\n    source: b/o\n  - name: q\n    source: b/o2\n- id: b\n  type: b\n"
        );
        let out = splice_remove_input(doc, 0, 0).unwrap();
        assert_eq!(
            out,
            "prefix: r\nnodes:\n- id: a\n  type: a\n- id: b\n  type: b\n"
        );
    }

    #[test]
    fn an_empty_flow_inputs_value_becomes_a_block() {
        let doc = "prefix: r\nnodes:\n  - id: a\n    type: a\n    inputs: []\n";
        let out = splice_add_input(doc, 0, &input("x", "b/y")).unwrap();
        assert_eq!(
            out,
            "prefix: r\nnodes:\n  - id: a\n    type: a\n    inputs:\n      - name: x\n        source: b/y\n"
        );
    }

    #[test]
    fn a_flow_inputs_value_is_refused_not_rewritten() {
        let doc =
            "prefix: r\nnodes:\n  - id: a\n    type: a\n    inputs: [{name: p, source: b/o}]\n";
        assert!(splice_add_input(doc, 0, &input("x", "b/y")).is_err());
        assert!(splice_remove_input(doc, 0, 0).is_err());
    }
}
