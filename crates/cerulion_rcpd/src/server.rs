// SPDX-License-Identifier: AGPL-3.0-only
//! The MCP handler: `robot://graph` as a readable, subscribable resource,
//! `robot://state/{name}` for the manifest's state topics, and the tools of
//! [`crate::tools`].
//!
//! Both subscription shapes are served for `robot://graph`:
//! `subscriptions/listen` (protocol 2026-07-28) and the legacy
//! `resources/subscribe`. Either way a subscriber gets
//! `notifications/resources/updated` after each document refresh. A handler
//! built with [`RcpServer::new`] serves no tools and no state resources.

use std::sync::{Arc, Mutex};

use rmcp::model::{
    CallToolRequestParams, CallToolResponse, Implementation, ListResourcesResult, ListToolsResult,
    PaginatedRequestParams, ReadResourceRequestParams, ReadResourceResponse, ReadResourceResult,
    Resource, ResourceContents, ResourceUpdatedNotification, ResourceUpdatedNotificationParam,
    ServerCapabilities, ServerConfig, ServerNotification, SubscribeRequestParams,
    SubscriptionFilter, UnsubscribeRequestParams,
};
use rmcp::service::{RequestContext, SubscriptionContext};
use rmcp::{ErrorData, RoleServer, ServerHandler};
use tokio::task::AbortHandle;

use crate::manifest::StateResource;
use crate::observer::GraphFeed;
use crate::tools::ToolHost;
use crate::{GRAPH_URI, SERVER_NAME, STATE_URI_PREFIX};

const GRAPH_MIME: &str = "application/json";

/// One MCP session's handler. Each session gets its own clone of the feed.
#[derive(Debug, Clone)]
pub struct RcpServer {
    feed: GraphFeed,
    legacy_subscription: Arc<Mutex<Option<AbortHandle>>>,
    tools: Option<Arc<ToolHost>>,
    state: Arc<Vec<StateResource>>,
}

impl RcpServer {
    /// A handler serving the document behind `feed`, with no tools.
    pub fn new(feed: GraphFeed) -> Self {
        Self {
            feed,
            legacy_subscription: Arc::new(Mutex::new(None)),
            tools: None,
            state: Arc::new(Vec::new()),
        }
    }

    /// A handler that also serves `tools` and the `state` resources.
    pub fn with_tools(feed: GraphFeed, tools: Arc<ToolHost>, state: Vec<StateResource>) -> Self {
        Self {
            feed,
            legacy_subscription: Arc::new(Mutex::new(None)),
            tools: Some(tools),
            state: Arc::new(state),
        }
    }

    fn replace_legacy_subscription(&self, next: Option<AbortHandle>) {
        let mut slot = self
            .legacy_subscription
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(previous) = std::mem::replace(&mut *slot, next) {
            previous.abort();
        }
    }
}

impl Drop for RcpServer {
    fn drop(&mut self) {
        if Arc::strong_count(&self.legacy_subscription) == 1 {
            self.replace_legacy_subscription(None);
        }
    }
}

fn graph_resource() -> Resource {
    Resource::new(GRAPH_URI, "robot graph")
        .with_description(
            "Runs, declared nodes, live topics with liveness and rates, and monitor flags",
        )
        .with_mime_type(GRAPH_MIME)
}

fn unknown_resource(uri: &str) -> ErrorData {
    ErrorData::resource_not_found("unknown resource", Some(serde_json::json!({ "uri": uri })))
}

impl ServerHandler for RcpServer {
    fn get_info(&self) -> ServerConfig {
        let capabilities = if self.tools.is_some() {
            ServerCapabilities::builder()
                .enable_resources()
                .enable_resources_subscribe()
                .enable_tools()
                .build()
        } else {
            ServerCapabilities::builder()
                .enable_resources()
                .enable_resources_subscribe()
                .build()
        };
        ServerConfig::new(capabilities)
            .with_server_info(Implementation::new(SERVER_NAME, env!("CARGO_PKG_VERSION")))
    }

    async fn list_resources(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListResourcesResult, ErrorData> {
        let mut resources = vec![graph_resource()];
        resources.extend(self.state.iter().map(|entry| {
            Resource::new(
                format!("{STATE_URI_PREFIX}{}", entry.name),
                entry.name.clone(),
            )
            .with_description(format!("The newest frame on {}", entry.topic))
            .with_mime_type(GRAPH_MIME)
        }));
        Ok(ListResourcesResult::with_all_items(resources))
    }

    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        let tools = self
            .tools
            .as_ref()
            .map(|host| host.list())
            .unwrap_or_default();
        Ok(ListToolsResult::with_all_items(tools))
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResponse, ErrorData> {
        let Some(host) = self.tools.as_ref() else {
            return Err(ErrorData::invalid_params("this server has no tools", None));
        };
        Ok(host.call(&request.name, request.arguments).await.into())
    }

    async fn read_resource(
        &self,
        request: ReadResourceRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<ReadResourceResponse, ErrorData> {
        if let Some(name) = request.uri.strip_prefix(STATE_URI_PREFIX) {
            let known = self.state.iter().any(|entry| entry.name == name);
            let host = self.tools.as_ref().filter(|_| known);
            let Some(host) = host else {
                return Err(unknown_resource(&request.uri));
            };
            let Some(text) = host.journal().state(name) else {
                return Err(ErrorData::resource_not_found(
                    "no frame seen on this state topic yet",
                    Some(serde_json::json!({ "uri": request.uri })),
                ));
            };
            return Ok(ReadResourceResult::new(vec![ResourceContents::text(
                text,
                request.uri.clone(),
            )
            .with_mime_type(GRAPH_MIME)])
            .into());
        }
        if request.uri != GRAPH_URI {
            return Err(unknown_resource(&request.uri));
        }
        let snapshot = self.feed.current();
        Ok(ReadResourceResult::new(vec![ResourceContents::text(
            snapshot.json.clone(),
            GRAPH_URI,
        )
        .with_mime_type(GRAPH_MIME)])
        .into())
    }

    fn accepted_subscription_filter(
        &self,
        requested: &SubscriptionFilter,
    ) -> Option<SubscriptionFilter> {
        let wants_graph = requested
            .resource_subscriptions
            .as_ref()
            .is_some_and(|uris| uris.iter().any(|uri| uri == GRAPH_URI));
        let mut accepted = SubscriptionFilter::new();
        if wants_graph {
            accepted.resource_subscriptions = Some(vec![GRAPH_URI.to_string()]);
        }
        Some(accepted)
    }

    async fn listen(&self, context: SubscriptionContext) -> Result<(), ErrorData> {
        let wants_graph = context
            .accepted()
            .resource_subscriptions
            .as_ref()
            .is_some_and(|uris| uris.iter().any(|uri| uri == GRAPH_URI));
        if !wants_graph {
            context.cancelled().await;
            return Ok(());
        }
        let mut feed = self.feed.clone();
        loop {
            tokio::select! {
                () = context.cancelled() => return Ok(()),
                alive = feed.changed() => {
                    if !alive {
                        return Ok(());
                    }
                    let notification = ServerNotification::ResourceUpdatedNotification(
                        ResourceUpdatedNotification::new(ResourceUpdatedNotificationParam::new(
                            GRAPH_URI,
                        )),
                    );
                    if context.sink().send(notification).await.is_err() {
                        return Ok(());
                    }
                }
            }
        }
    }

    #[allow(
        deprecated,
        reason = "legacy clients still subscribe with resources/subscribe"
    )]
    async fn subscribe(
        &self,
        request: SubscribeRequestParams,
        context: RequestContext<RoleServer>,
    ) -> Result<(), ErrorData> {
        if request.uri != GRAPH_URI {
            return Err(unknown_resource(&request.uri));
        }
        let mut feed = self.feed.clone();
        let peer = context.peer.clone();
        let task = tokio::spawn(async move {
            while feed.changed().await {
                let param = ResourceUpdatedNotificationParam::new(GRAPH_URI);
                if peer.notify_resource_updated(param).await.is_err() {
                    break;
                }
            }
        });
        self.replace_legacy_subscription(Some(task.abort_handle()));
        Ok(())
    }

    #[allow(
        deprecated,
        reason = "legacy clients still subscribe with resources/subscribe"
    )]
    async fn unsubscribe(
        &self,
        request: UnsubscribeRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<(), ErrorData> {
        if request.uri != GRAPH_URI {
            return Err(unknown_resource(&request.uri));
        }
        self.replace_legacy_subscription(None);
        Ok(())
    }
}
