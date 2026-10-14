// SPDX-License-Identifier: AGPL-3.0-only
//! The MCP handler: `robot://graph` as a readable, subscribable resource.
//!
//! Both subscription shapes are served: `subscriptions/listen` (protocol
//! 2026-07-28) and the legacy `resources/subscribe`. Either way a subscriber
//! gets `notifications/resources/updated` after each document refresh.
//! `tools/list` is the empty trait default.

use std::sync::{Arc, Mutex};

use rmcp::model::{
    Implementation, ListResourcesResult, PaginatedRequestParams, ReadResourceRequestParams,
    ReadResourceResponse, ReadResourceResult, Resource, ResourceContents,
    ResourceUpdatedNotification, ResourceUpdatedNotificationParam, ServerCapabilities,
    ServerConfig, ServerNotification, SubscribeRequestParams, SubscriptionFilter,
    UnsubscribeRequestParams,
};
use rmcp::service::{RequestContext, SubscriptionContext};
use rmcp::{ErrorData, RoleServer, ServerHandler};
use tokio::task::AbortHandle;

use crate::observer::GraphFeed;
use crate::{GRAPH_URI, SERVER_NAME};

const GRAPH_MIME: &str = "application/json";

/// One MCP session's handler. Each session gets its own clone of the feed.
#[derive(Debug, Clone)]
pub struct RcpServer {
    feed: GraphFeed,
    legacy_subscription: Arc<Mutex<Option<AbortHandle>>>,
}

impl RcpServer {
    /// A handler serving the document behind `feed`.
    pub fn new(feed: GraphFeed) -> Self {
        Self {
            feed,
            legacy_subscription: Arc::new(Mutex::new(None)),
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
        ServerConfig::new(
            ServerCapabilities::builder()
                .enable_resources()
                .enable_resources_subscribe()
                .build(),
        )
        .with_server_info(Implementation::new(SERVER_NAME, env!("CARGO_PKG_VERSION")))
    }

    async fn list_resources(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListResourcesResult, ErrorData> {
        Ok(ListResourcesResult::with_all_items(vec![graph_resource()]))
    }

    async fn read_resource(
        &self,
        request: ReadResourceRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<ReadResourceResponse, ErrorData> {
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
