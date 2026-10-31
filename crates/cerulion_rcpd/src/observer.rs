// SPDX-License-Identifier: AGPL-3.0-only
//! The graph observer: one std thread owning a [`TopicLivenessObserver`] and a
//! [`MonitorEngine`], refreshing the cached `robot://graph` document.
//!
//! Every [`GraphSource::tick`] sweeps the liveness taps (self-throttled to
//! [`LIVENESS_SWEEP_INTERVAL_NS`](cerulion_core::transport::liveness::LIVENESS_SWEEP_INTERVAL_NS)), feeds the monitor every
//! [`MONITOR_SAMPLE_INTERVAL_NS`], and recomposes the document at most once
//! per [`GRAPH_REFRESH_INTERVAL_NS`]. Readers get the cached document through
//! a [`GraphFeed`] and never touch the transport. All pacing reads the
//! transport manager's clock.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_core::monitor::{MonitorEngine, MonitorSample, MONITOR_SAMPLE_INTERVAL_NS};
use cerulion_core::transport::liveness::{read_liveness, TopicLivenessObserver};
use cerulion_core::transport::run_registry::RUN_GATHER_WINDOW;
use cerulion_core::TransportManager;
use tokio::sync::watch;

use crate::graph::{compose, GraphInputs, RobotGraph, TopicObservation};
use crate::manifest::RobotIdentity;
use crate::RcpdError;

/// The minimum transport-clock interval between two document refreshes.
pub const GRAPH_REFRESH_INTERVAL_NS: u64 = 1_000_000_000;

/// How long the observer thread sleeps between ticks.
const TICK_PERIOD: Duration = Duration::from_millis(50);

/// One composed document and its serialized form.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GraphSnapshot {
    /// The document.
    pub graph: RobotGraph,
    /// The document as served: `serde_json` output of `graph`.
    pub json: String,
}

impl GraphSnapshot {
    fn new(graph: RobotGraph) -> Self {
        let json = serde_json::to_string(&graph).unwrap_or_else(|e| {
            tracing::error!(error = %e, "robot graph did not serialize");
            String::from("{}")
        });
        Self { graph, json }
    }
}

/// A read handle on the cached document. Cheap to clone.
#[derive(Debug, Clone)]
pub struct GraphFeed {
    rx: watch::Receiver<Arc<GraphSnapshot>>,
}

impl GraphFeed {
    /// The newest cached document.
    pub fn current(&self) -> Arc<GraphSnapshot> {
        Arc::clone(&self.rx.borrow())
    }

    /// Wait for the next refresh. Returns `false` once the source is gone.
    pub async fn changed(&mut self) -> bool {
        self.rx.changed().await.is_ok()
    }
}

/// The transport side of the observer.
pub struct GraphSource {
    manager: Arc<TransportManager>,
    identity: RobotIdentity,
    liveness: TopicLivenessObserver,
    engine: MonitorEngine,
    topics: Vec<String>,
    topics_error: Option<String>,
    last_sample_ns: Option<u64>,
    last_refresh_ns: u64,
    tx: watch::Sender<Arc<GraphSnapshot>>,
}

impl GraphSource {
    /// Open the source and compose the first document.
    pub fn new(manager: Arc<TransportManager>, identity: RobotIdentity) -> Self {
        let liveness = TopicLivenessObserver::new(Arc::clone(&manager));
        let now_ns = manager.clock().now_ns();
        let placeholder = Arc::new(GraphSnapshot::new(compose(
            &identity,
            GraphInputs {
                now_ns,
                runs: Err(String::from("not gathered yet")),
                topics: Ok(Vec::new()),
                monitors: Vec::new(),
            },
        )));
        let (tx, _rx) = watch::channel(placeholder);
        let mut source = Self {
            manager,
            identity,
            liveness,
            engine: MonitorEngine::new(),
            topics: Vec::new(),
            topics_error: None,
            last_sample_ns: None,
            last_refresh_ns: now_ns,
            tx,
        };
        source.discover_topics();
        source.liveness.sweep();
        source.refresh(now_ns);
        source
    }

    /// A read handle on the cached document.
    pub fn feed(&self) -> GraphFeed {
        GraphFeed {
            rx: self.tx.subscribe(),
        }
    }

    /// Advance the observer once: sweep, sample the monitor when due, and
    /// refresh the document when due.
    pub fn tick(&mut self) {
        let now_ns = self.manager.clock().now_ns();
        let refresh_due = now_ns.saturating_sub(self.last_refresh_ns) >= GRAPH_REFRESH_INTERVAL_NS;
        if refresh_due {
            self.discover_topics();
        }
        self.liveness.sweep();
        let sample_due = self
            .last_sample_ns
            .is_none_or(|last| now_ns.saturating_sub(last) >= MONITOR_SAMPLE_INTERVAL_NS);
        if sample_due {
            self.sample_monitor(now_ns);
        }
        if refresh_due {
            self.refresh(now_ns);
        }
    }

    /// Run [`Self::tick`] on a dedicated thread until the handle drops.
    ///
    /// # Errors
    ///
    /// [`RcpdError::ObserverSpawn`] when the thread cannot be spawned.
    pub fn spawn(mut self) -> Result<ObserverHandle, RcpdError> {
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = Arc::clone(&stop);
        let join = std::thread::Builder::new()
            .name(String::from("rcpd-graph"))
            .spawn(move || {
                while !thread_stop.load(Ordering::Acquire) {
                    self.tick();
                    std::thread::sleep(TICK_PERIOD);
                }
            })
            .map_err(RcpdError::ObserverSpawn)?;
        Ok(ObserverHandle {
            stop,
            join: Some(join),
        })
    }

    fn discover_topics(&mut self) {
        match self.manager.list_topics() {
            Ok(topics) => {
                self.liveness.track_all(topics.iter().map(String::as_str));
                self.topics = topics;
                self.topics_error = None;
            }
            Err(e) => {
                tracing::warn!(error = %e, "topic enumeration failed");
                self.topics_error = Some(e.to_string());
            }
        }
    }

    fn sample_monitor(&mut self, now_ns: u64) {
        self.last_sample_ns = Some(now_ns);
        let table = self.liveness.table();
        for topic in &self.topics {
            let liveness = read_liveness(&table, topic, now_ns);
            self.engine.observe(&MonitorSample::new(
                topic.as_str(),
                None,
                now_ns,
                liveness,
                true,
            ));
        }
    }

    fn refresh(&mut self, now_ns: u64) {
        self.last_refresh_ns = now_ns;
        let runs = self
            .manager
            .gather_live_runs(RUN_GATHER_WINDOW)
            .map_err(|e| e.to_string());
        let topics = match &self.topics_error {
            Some(error) => Err(error.clone()),
            None => {
                let table = self.liveness.table();
                Ok(self
                    .topics
                    .iter()
                    .map(|topic| TopicObservation {
                        topic: topic.clone(),
                        producer_count: self.manager.topic_publisher_count_checked(topic),
                        liveness: read_liveness(&table, topic, now_ns),
                    })
                    .collect())
            }
        };
        let graph = compose(
            &self.identity,
            GraphInputs {
                now_ns,
                runs,
                topics,
                monitors: self.engine.rows(now_ns),
            },
        );
        self.tx.send_replace(Arc::new(GraphSnapshot::new(graph)));
    }
}

/// Owns the observer thread; dropping it stops and joins the thread.
pub struct ObserverHandle {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl Drop for ObserverHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(join) = self.join.take() {
            if join.join().is_err() {
                tracing::error!("graph observer thread panicked");
            }
        }
    }
}
