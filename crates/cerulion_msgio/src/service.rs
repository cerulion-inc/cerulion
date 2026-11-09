// SPDX-License-Identifier: AGPL-3.0-only
//! Call a service once and render its response as one JSON document: the
//! engine of `cerulion service call` and the robot-side `service-call` verb.
//!
//! The call speaks the protocol `rmw_cerulion` serves ROS 2 services with
//! (`cerulion_core::transport::service`): the request goes out on the topic
//! `svc/<service>/request` (the ROS name with its slashes collapsed, no
//! leading slash), the response comes back on this client's own
//! `svc/<service>/reply/<guid>` topic, each frame carries the
//! `[client guid][sequence]` envelope, and the payload is the message body
//! in the Cerulion wire layout. Calls to one service share 16 client ids, and
//! each call numbers its request below every sequence an earlier call sent
//! (counting down from the wall clock), so a late response to an earlier
//! call on the same id matches no later one. The request and response types
//! are the two parts of the service's `.srv` file, `<pkg>/<Name>_Request` and
//! `<pkg>/<Name>_Response`, read from the workspace store
//! (`schemas/<pkg>/srv/<Name>.srv`), each nested type named with its package
//! as ROS 2 introspection names it. A field-less part, and every field-less
//! message they use, gets the one `uint8` member ROS 2's IDL generation adds
//! ([`MEMBER_PLACEHOLDER`]), which `rmw_cerulion` lays out and hashes as a
//! field.
//!
//! The server must run on this machine: the request and reply topics are
//! local shared memory.

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cerulion_core::codegen::{FrameWalker, MessageSchema};
use cerulion_core::transport::service::{
    derive_client_guid, reply_topic, request_topic, ServiceClient,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::FullQueueSend;

use crate::text::{body_to_json, message_frame, qualified_type_name, InterfaceKind, VerbError};

/// The default `--timeout`: how long the call waits for a server and then
/// for its response.
pub const DEFAULT_CALL_TIMEOUT: Duration = Duration::from_secs(5);

/// The request publisher's slot size: the slice `rmw_cerulion` gives its own
/// service clients and servers.
const SERVICE_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(4 * 1024 * 1024);

/// How often the call checks for a server and for the response.
const POLL: Duration = Duration::from_millis(2);

/// The node identity this verb's client GUIDs are derived from, with a slot
/// number from 0 below [`CLIENT_SLOTS`] as the instance.
const CLIENT_NODE_ID: &str = "cerulion-cli-service-call";

/// How many client ids calls to one service share.
const CLIENT_SLOTS: u64 = 16;

/// What `service call` was asked to do.
#[derive(Debug, Clone)]
pub struct CallRequest<'a> {
    /// The service name as ROS spells it, `/controller_manager/list_controllers`.
    pub service: &'a str,
    /// `pkg/srv/Type` or `pkg/Type`.
    pub type_name: &'a str,
    /// The request as a JSON or YAML mapping.
    pub request: &'a str,
    /// How long to wait for a server, and then for the response.
    pub timeout: Duration,
}

/// The two message types of a service and the walker that encodes them.
pub struct ServiceTypes {
    /// Knows the request, the response and every type they nest.
    pub walker: FrameWalker,
    /// `pkg/<Name>_Request`.
    pub request: String,
    /// `pkg/<Name>_Response`.
    pub response: String,
}

/// Resolve the service type `type_name` from the store's `.srv` file under
/// `schemas_dir`, laid out over `base` (the message schemas its parts may
/// use: the built-ins, the store's messages, any workspace schemas) by
/// `build`, which turns the full set into a walker.
pub fn resolve_service_types(
    type_name: &str,
    schemas_dir: Option<&Path>,
    base: Vec<MessageSchema>,
    build: impl FnOnce(Vec<MessageSchema>) -> Result<FrameWalker, VerbError>,
) -> Result<ServiceTypes, VerbError> {
    let qualified = qualified_type_name(type_name, InterfaceKind::Service)?;
    let Some((package, name)) = qualified.split_once('/') else {
        return Err(VerbError::Usage(format!(
            "service type '{qualified}' names no package: write <package>/srv/<Type>, for \
             example std_srvs/srv/Trigger"
        )));
    };
    let not_found = |dir: &str| {
        VerbError::Usage(format!(
            "unknown service type '{package}/srv/{name}': no {dir}{package}/srv/{name}.srv. Copy \
             the package's .srv file there (and the .msg files it uses to \
             schemas/<pkg>/msg/)"
        ))
    };
    let Some(schemas_dir) = schemas_dir else {
        return Err(not_found("workspace, so no schemas/"));
    };
    let definition = crate::schema_store::load_store_service(schemas_dir, package, name)
        .map_err(VerbError::Usage)?
        .ok_or_else(|| not_found("schemas/"))?;
    let request = definition.request.qualified_name();
    let response = definition.response.qualified_name();
    let mut schemas = base;
    schemas.push(definition.request);
    schemas.push(definition.response);
    add_member_placeholders(&mut schemas);
    let walker = build(schemas)?;
    for part in [&request, &response] {
        let missing = match walker.schema_for_name(part) {
            Some(schema) => unresolved_nested(&walker, schema, 0),
            None => Some(part.clone()),
        };
        if let Some(missing) = missing {
            return Err(VerbError::Usage(format!(
                "service type '{qualified}': {part} uses {missing}, which is neither a built-in \
                 type nor in schemas/<pkg>/msg/"
            )));
        }
    }
    Ok(ServiceTypes {
        walker,
        request,
        response,
    })
}

/// The member every field-less message gets from ROS 2's IDL generation
/// (an IDL struct cannot be empty). `rmw_cerulion` builds a type's schema
/// from the introspection members, so its schema of a field-less request or
/// response (`ListControllers_Request`, `Trigger_Request`) holds this one
/// `uint8`, and so does its schema hash.
pub const MEMBER_PLACEHOLDER: &str = "structure_needs_at_least_one_member";

/// Give every field-less message in `schemas` the [`MEMBER_PLACEHOLDER`]
/// `uint8` field, the shape `rmw_cerulion` lays it out with.
fn add_member_placeholders(schemas: &mut [MessageSchema]) {
    use cerulion_core::codegen::{FieldDef, FieldType};
    for schema in schemas.iter_mut().filter(|s| s.fields.is_empty()) {
        schema.add_field(FieldDef::new(MEMBER_PLACEHOLDER, FieldType::U8));
    }
}

/// The first message type `schema` refers to, at any depth, that `walker`
/// cannot resolve, as `pkg/Type`; `None` when every one resolves.
fn unresolved_nested(walker: &FrameWalker, schema: &MessageSchema, depth: usize) -> Option<String> {
    use cerulion_core::codegen::FieldType;
    if depth > cerulion_core::dynamic::MAX_YAML_DEPTH {
        return None;
    }
    let parent = schema.package.as_deref();
    for field in &schema.fields {
        let mut ft = &field.field_type;
        while let FieldType::FixedArray { element_type, .. }
        | FieldType::DynamicArray { element_type } = ft
        {
            ft = element_type;
        }
        if let FieldType::Nested {
            schema_name,
            package,
            ..
        } = ft
        {
            match walker.nested_target(schema_name, package, parent) {
                Some((_, nested)) => {
                    if let Some(missing) = unresolved_nested(walker, nested, depth + 1) {
                        return Some(missing);
                    }
                }
                None => {
                    return Some(format!(
                        "{}/{schema_name}",
                        package.as_deref().or(parent).unwrap_or("")
                    ))
                }
            }
        }
    }
    None
}

/// Refuse a service name that is empty once its slashes are dropped.
pub fn check_service_name(service: &str) -> Result<(), VerbError> {
    if service.split('/').all(str::is_empty) {
        return Err(VerbError::Usage(format!(
            "service name '{}' is empty: write the full ROS name, for example \
             /controller_manager/list_controllers",
            service.escape_debug()
        )));
    }
    Ok(())
}

/// [`resolve_service_types`] over the built-in corpus and the `.msg` store
/// under `schemas_dir`, laid out through the checked
/// [`cerulion_core::dynamic::SchemaSet`] construction.
pub fn resolve_store_service_types(
    type_name: &str,
    schemas_dir: Option<&Path>,
) -> Result<ServiceTypes, VerbError> {
    resolve_service_types(
        type_name,
        schemas_dir,
        crate::store_schemas(schemas_dir),
        crate::checked_walker,
    )
}

/// Call the service once with `types` (from [`resolve_service_types`]
/// for `req.type_name`) and return its response as one line of JSON.
///
/// Errors: a malformed or unknown type, or request text that is not one
/// JSON or YAML document, is [`VerbError::Usage`]; a request value that does
/// not fit its field, or a response that does not decode, is
/// [`VerbError::Encode`]; no server within the timeout, or no response
/// within what is left of it, is [`VerbError::Unavailable`]. An interrupted
/// call returns [`VerbError::Unavailable`] too; a `running` cleared before
/// the request goes out sends no request, even to a server that is ready.
///
/// The client's request publisher is built with [`FullQueueSend::Discard`],
/// so the send returns without waiting on a server whose request queue is
/// full (a request topic created without safe overflow) and skips that
/// server. A request that reaches none of the servers subscribed before and
/// after the send is [`VerbError::Unavailable`] naming the full request
/// queue; one that reaches some of them waits for its response, and an error
/// after that says how many servers it skipped.
pub fn service_call(
    req: &CallRequest<'_>,
    types: &ServiceTypes,
    running: &AtomicBool,
) -> Result<String, VerbError> {
    service_call_gated(req, types, running, |send| send())
}

/// [`service_call`] with `send_gate` around the send: once a server is up
/// and `running` is still set, the call passes the send to `send_gate` as a
/// closure, and the request goes out only if `send_gate` runs it. A gate
/// that returns without running it ends the call as
/// [`VerbError::Unavailable`] with no request sent.
pub fn service_call_gated(
    req: &CallRequest<'_>,
    types: &ServiceTypes,
    running: &AtomicBool,
    send_gate: impl FnOnce(&mut dyn FnMut()),
) -> Result<String, VerbError> {
    check_service_name(req.service)?;
    let request_frame = message_frame(&types.walker, &types.request, req.request, 0)?;
    let request_hash = types
        .walker
        .schema_hash_for(&types.request)
        .ok_or_else(|| VerbError::Usage(format!("unknown type {}", types.request)))?;
    let response_hash = types
        .walker
        .schema_hash_for(&types.response)
        .ok_or_else(|| VerbError::Usage(format!("unknown type {}", types.response)))?;

    let transport = cerulion_core::TransportManager::get_or_init()?;
    let mut client = open_client(&transport, req.service, request_hash, response_hash)?;
    client.start_sequence_at(next_call_sequence());

    let deadline = Instant::now() + req.timeout;
    if !wait_until(deadline, running, || client.server_available()) {
        return Err(VerbError::Unavailable(format!(
            "no server for service '{}' on this machine within {} (nothing subscribes to {})",
            req.service,
            format_secs(req.timeout),
            request_topic(req.service)
        )));
    }
    let mut sent = None;
    let servers_before = client.server_count();
    if running.load(Ordering::Relaxed) {
        send_gate(&mut || {
            sent = Some(client.send_request_counted(&request_frame[WireHeader::SIZE..]));
        });
    }
    let Some(sent) = sent else {
        return Err(VerbError::Unavailable(format!(
            "the call to '{}' was stopped before its request was sent",
            req.service
        )));
    };
    let (sequence, reached) = sent?;
    let skipped = request_reach(
        req.service,
        reached,
        servers_before.min(client.server_count()),
    )?;
    if let Some(note) = &skipped {
        tracing::warn!(
            service = %req.service,
            reached,
            note = %note,
            "the request did not reach every server: a request queue was full"
        );
    }
    let skipped = skipped.map(|note| format!(". {note}")).unwrap_or_default();
    let mut response: Option<Vec<u8>> = None;
    let answered = wait_until(deadline, running, || {
        take_response(&mut client, sequence, &mut response)
    });
    let Some(body) = response.filter(|_| answered) else {
        if !running.load(Ordering::Relaxed) {
            return Err(VerbError::Unavailable(format!(
                "the call to '{}' was stopped before its response arrived{skipped}",
                req.service
            )));
        }
        return Err(VerbError::Unavailable(format!(
            "service '{}' sent no response within {}{skipped}. A server that refuses the \
             request type drops it: check that schemas/ holds the same {} the server was built \
             with (request schema hash 0x{request_hash:016x}; {} dropped on the response hash)",
            req.service,
            format_secs(req.timeout),
            req.type_name,
            client.schema_mismatch_count()
        )));
    };
    body_to_json(&types.walker, response_hash, &body)
}

/// What a request that reached `reached` servers means for the call, given
/// `servers`, the smaller of the server counts read before and after its
/// send: an error when it reached none, the servers it skipped in words when
/// it reached fewer than `servers`, `None` when it reached them all.
fn request_reach(
    service: &str,
    reached: usize,
    servers: usize,
) -> Result<Option<String>, VerbError> {
    match (reached, servers.saturating_sub(reached)) {
        (0, 0) => Err(VerbError::Unavailable(format!(
            "the request to '{service}' reached no server: nothing subscribes to {} now",
            request_topic(service)
        ))),
        (0, _) => Err(VerbError::Unavailable(format!(
            "the request to '{service}' was not delivered: the server's request queue was full \
             ({servers} server(s) on this machine, none took the request)"
        ))),
        (_, 0) => Ok(None),
        (_, 1) => Ok(Some(format!(
            "The request reached {reached} of the {servers} servers on this machine; the \
             request queue of the other was full"
        ))),
        (_, n) => Ok(Some(format!(
            "The request reached {reached} of the {servers} servers on this machine; the \
             request queues of the other {n} were full"
        ))),
    }
}

/// Open a client on the first free one of [`CLIENT_SLOTS`] client ids. A
/// server keeps one reply publisher per client id it has answered, so calls
/// that reuse a small set of ids leave at most that many on a server; a slot
/// is taken while its reply topic has a subscriber, and a client whose reply
/// topic another call opened at the same moment is dropped for the next slot.
fn open_client(
    transport: &std::sync::Arc<cerulion_core::TransportManager>,
    service: &str,
    request_hash: u64,
    response_hash: u64,
) -> Result<ServiceClient, VerbError> {
    for slot in 0..CLIENT_SLOTS {
        let guid = derive_client_guid(CLIENT_NODE_ID, service, slot);
        let reply = reply_topic(service, &guid);
        if transport.topic_subscriber_count(&reply) > 0 {
            continue;
        }
        let client = transport.create_service_client_with(
            service,
            guid,
            SERVICE_SLICE_LEN,
            request_hash,
            response_hash,
            FullQueueSend::Discard,
        )?;
        if transport.topic_subscriber_count(&reply) == 1 {
            return Ok(client);
        }
    }
    Err(VerbError::Unavailable(format!(
        "all {CLIENT_SLOTS} service call client ids for '{service}' are in use by calls \
         running now"
    )))
}

/// The first sequence of a call's client: below every sequence an earlier
/// call in this process took (an atomic minimum) and at most `i64::MAX` less
/// the wall clock's nanoseconds since the Unix epoch, so, on a clock that
/// moved forward, below what a call in an earlier process sent too. A server
/// answers on the client id's reply topic with the request's sequence, so a
/// late answer to an earlier call on the same id matches no later call. A
/// server reads a sequence below the last one it saw from an id as a client
/// restarted with a reused id, and one above it by more than one as requests
/// lost from its queue.
fn next_call_sequence() -> i64 {
    static LAST: AtomicI64 = AtomicI64::new(i64::MAX);
    let since_epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_nanos()).unwrap_or(i64::MAX));
    let below = |last: i64| (i64::MAX - since_epoch).min(last - 1);
    let last = LAST
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |last| {
            Some(below(last))
        })
        .unwrap_or_else(|last| last);
    below(last)
}

/// Take queued responses until the one answering `sequence` arrives; true
/// once it has (its body is then in `out`). A response to another sequence
/// is dropped.
fn take_response(client: &mut ServiceClient, sequence: i64, out: &mut Option<Vec<u8>>) -> bool {
    loop {
        let mut matched = false;
        let took = client.try_take_one_response(|seq, payload| {
            if seq == sequence {
                *out = Some(payload.to_vec());
                matched = true;
            }
        });
        match took {
            Ok(true) if matched => return true,
            Ok(true) => continue,
            Ok(false) => return false,
            Err(e) => {
                tracing::warn!(service = %client.service(), error = %e, "service response take failed");
                return false;
            }
        }
    }
}

/// Poll `ready` every [`POLL`] until it is true (returns true), the deadline
/// passes or `running` clears (returns false).
fn wait_until(deadline: Instant, running: &AtomicBool, mut ready: impl FnMut() -> bool) -> bool {
    loop {
        if ready() {
            return true;
        }
        if !running.load(Ordering::Relaxed) || Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL); // ALLOW: a one-shot CLI wait for a server or a response, bounded by --timeout
    }
}

fn format_secs(d: Duration) -> String {
    format!("{} s", d.as_secs_f64())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(files: &[(&str, &str)]) -> tempfile::TempDir {
        let dir = tempfile::tempdir().expect("tempdir");
        for (rel, text) in files {
            let path = dir.path().join(rel);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, text).unwrap();
        }
        dir
    }

    #[test]
    fn a_store_srv_resolves_both_parts_with_their_nested_store_messages() {
        let dir = store(&[
            ("demo_msgs/msg/Item.msg", "string name\nuint8 state\n"),
            (
                "demo_msgs/srv/ListItems.srv",
                "string prefix\n---\nItem[] items\nbool ok\n",
            ),
        ]);
        let types =
            resolve_store_service_types("demo_msgs/srv/ListItems", Some(dir.path())).unwrap();
        assert_eq!(types.request, "demo_msgs/ListItems_Request");
        assert_eq!(types.response, "demo_msgs/ListItems_Response");
        let frame = message_frame(
            &types.walker,
            &types.response,
            r#"{"items": [{"name": "a", "state": 2}], "ok": true}"#,
            0,
        )
        .unwrap();
        let hash = types.walker.schema_hash_for(&types.response).unwrap();
        assert_eq!(
            body_to_json(&types.walker, hash, &frame[WireHeader::SIZE..]).unwrap(),
            r#"{"items":[{"name":"a","state":2}],"ok":true}"#
        );
        // The pkg/Type spelling names the same service.
        let short = resolve_store_service_types("demo_msgs/ListItems", Some(dir.path())).unwrap();
        assert_eq!(short.request, types.request);
    }

    #[test]
    fn the_part_hashes_are_the_names_ros_gives_them() {
        // The same `.srv` read as two `.msg` bodies named the way rmw_cerulion
        // names a ROS 2 service's parts gives the same hashes.
        let dir = store(&[(
            "std_srvs/srv/SetBool.srv",
            "bool data\n---\nbool success\nstring message\n",
        )]);
        let types = resolve_store_service_types("std_srvs/srv/SetBool", Some(dir.path())).unwrap();
        let mut schemas = crate::builtin_schemas();
        schemas.push(
            cerulion_core::codegen::parse_rosmsg(
                "bool data\n",
                "SetBool_Request",
                Some("std_srvs"),
            )
            .unwrap(),
        );
        schemas.push(
            cerulion_core::codegen::parse_rosmsg(
                "bool success\nstring message\n",
                "SetBool_Response",
                Some("std_srvs"),
            )
            .unwrap(),
        );
        let (reference, _) = FrameWalker::new(schemas);
        for part in ["std_srvs/SetBool_Request", "std_srvs/SetBool_Response"] {
            assert_eq!(
                types.walker.schema_hash_for(part),
                reference.schema_hash_for(part),
                "{part}"
            );
        }
    }

    #[test]
    fn a_bare_nested_type_in_a_srv_hashes_as_ros_2_introspection_names_it() {
        // ROS 2 reads `Level`, `Span` and `Item`, written without a package
        // in demo_msgs' files, as `demo_msgs/Level`, `demo_msgs/Span` and
        // `demo_msgs/Item`; the oracle is the schema rmw_cerulion builds from
        // those introspection members, nested types included. `Span` is fixed,
        // so its own `Level` members reach the request hash.
        use crate::introspection::{hash, message, nested};
        use cerulion_core::codegen::FieldType;
        let dir = store(&[
            ("demo_msgs/msg/Level.msg", "uint8 value\n"),
            ("demo_msgs/msg/Span.msg", "Level lo\nLevel hi\n"),
            ("demo_msgs/msg/Item.msg", "string name\nLevel level\n"),
            (
                "demo_msgs/srv/ListItems.srv",
                "Level min\nSpan span\n---\nItem[] items\nItem first\nLevel[2] range\n",
            ),
        ]);
        let types =
            resolve_store_service_types("demo_msgs/srv/ListItems", Some(dir.path())).unwrap();
        let level = || message("demo_msgs", "Level", vec![("value", FieldType::U8)]);
        let span = message(
            "demo_msgs",
            "Span",
            vec![
                ("lo", nested("demo_msgs", "Level")),
                ("hi", nested("demo_msgs", "Level")),
            ],
        );
        let item = message(
            "demo_msgs",
            "Item",
            vec![
                ("name", FieldType::String),
                ("level", nested("demo_msgs", "Level")),
            ],
        );
        let request = message(
            "demo_msgs",
            "ListItems_Request",
            vec![
                ("min", nested("demo_msgs", "Level")),
                ("span", nested("demo_msgs", "Span")),
            ],
        );
        let response = message(
            "demo_msgs",
            "ListItems_Response",
            vec![
                (
                    "items",
                    FieldType::DynamicArray {
                        element_type: Box::new(nested("demo_msgs", "Item")),
                    },
                ),
                ("first", nested("demo_msgs", "Item")),
                (
                    "range",
                    FieldType::FixedArray {
                        element_type: Box::new(nested("demo_msgs", "Level")),
                        length: 2,
                    },
                ),
            ],
        );
        assert_eq!(
            types.walker.schema_hash_for("demo_msgs/ListItems_Request"),
            Some(hash(
                vec![level(), span, request],
                "demo_msgs/ListItems_Request"
            ))
        );
        assert_eq!(
            types.walker.schema_hash_for("demo_msgs/ListItems_Response"),
            Some(hash(
                vec![level(), item, response],
                "demo_msgs/ListItems_Response"
            ))
        );
    }

    #[test]
    fn a_field_less_part_carries_the_idl_placeholder_member() {
        // Oracle: rmw_cerulion builds a schema from the introspection
        // members, and ROS 2 gives a field-less message exactly one member,
        // `uint8 structure_needs_at_least_one_member`.
        let dir = store(&[("controller_manager_msgs/srv/Ping.srv", "---\nbool ok\n")]);
        let types =
            resolve_store_service_types("controller_manager_msgs/srv/Ping", Some(dir.path()))
                .unwrap();
        let mut schemas = crate::builtin_schemas();
        schemas.push(
            cerulion_core::codegen::parse_rosmsg(
                "uint8 structure_needs_at_least_one_member\n",
                "Ping_Request",
                Some("controller_manager_msgs"),
            )
            .unwrap(),
        );
        let (reference, _) = FrameWalker::new(schemas);
        assert_eq!(
            types
                .walker
                .schema_hash_for("controller_manager_msgs/Ping_Request"),
            reference.schema_hash_for("controller_manager_msgs/Ping_Request"),
        );
        let frame = message_frame(&types.walker, &types.request, "{}", 0).unwrap();
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[0u8],
            "one zero byte, as rosidl initializes it"
        );
    }

    #[test]
    fn an_empty_service_name_is_a_usage_error() {
        for name in ["", "/", "//"] {
            let err = check_service_name(name).unwrap_err();
            assert_eq!(err.exit_code(), crate::text::EXIT_USAGE, "{name:?}");
        }
        assert!(check_service_name("/controller_manager/list_controllers").is_ok());
    }

    #[test]
    fn a_missing_srv_and_a_bare_name_are_usage_errors() {
        let dir = store(&[]);
        let err = resolve_store_service_types("demo_msgs/srv/Nope", Some(dir.path()))
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE);
        assert!(err.to_string().contains("demo_msgs/srv/Nope.srv"), "{err}");
        let err = resolve_store_service_types("Nope", Some(dir.path()))
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE);
        let err = resolve_store_service_types("demo_msgs/srv/Nope", None)
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE);
    }

    #[test]
    fn a_srv_without_a_separator_is_a_usage_error_naming_the_file() {
        let dir = store(&[("demo_msgs/srv/Bad.srv", "bool data\n")]);
        let err = resolve_store_service_types("demo_msgs/srv/Bad", Some(dir.path()))
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE);
        assert!(err.to_string().contains("demo_msgs/srv/Bad.srv"), "{err}");
    }

    #[test]
    fn a_call_sequence_is_below_the_wall_clock_countdown_and_the_last_call() {
        // next_call_sequence is at most i64::MAX less the wall clock's
        // nanoseconds it reads, and this reading is taken first, so it is no
        // later than that one. The atomic minimum alone gives i64::MAX - 1 to
        // the first call of every process.
        let before = i64::try_from(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        )
        .unwrap();
        let first = next_call_sequence();
        assert!(
            first <= i64::MAX - before,
            "{first} is above i64::MAX less the {before} ns read before the call"
        );
        let second = next_call_sequence();
        assert!(second < first, "{second} is not below {first}");
    }

    #[test]
    fn a_request_that_reached_no_server_queue_names_the_full_queue() {
        let full = request_reach("/arm/home", 0, 1).unwrap_err();
        assert_eq!(full.exit_code(), crate::text::EXIT_UNAVAILABLE);
        assert_eq!(
            full.to_string(),
            "the request to '/arm/home' was not delivered: the server's request queue was full \
             (1 server(s) on this machine, none took the request)"
        );
        let gone = request_reach("/arm/home", 0, 0).unwrap_err();
        assert_eq!(gone.exit_code(), crate::text::EXIT_UNAVAILABLE);
        assert!(gone.to_string().contains("reached no server"), "{gone}");
        assert_eq!(request_reach("/arm/home", 1, 1).unwrap(), None);
        assert_eq!(request_reach("/arm/home", 2, 1).unwrap(), None);
        assert_eq!(
            request_reach("/arm/home", 1, 2).unwrap().unwrap(),
            "The request reached 1 of the 2 servers on this machine; the request queue of the \
             other was full"
        );
        assert_eq!(
            request_reach("/arm/home", 1, 4).unwrap().unwrap(),
            "The request reached 1 of the 4 servers on this machine; the request queues of the \
             other 3 were full"
        );
    }

    #[test]
    fn a_part_using_a_missing_message_is_a_usage_error() {
        let dir = store(&[("demo_msgs/srv/Get.srv", "---\nMissing[] things\n")]);
        let err = resolve_store_service_types("demo_msgs/srv/Get", Some(dir.path()))
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE, "{err}");
        assert!(err.to_string().contains("demo_msgs/Missing"), "{err}");
    }
}
