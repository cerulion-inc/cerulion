// SPDX-License-Identifier: AGPL-3.0-only
//! SPIKE: Cerulion publisher + in-process network gateway for Android arm64.
//!
//! One process holds everything (iceoryx2 0.10.0 has no cross-process shared
//! memory on Android): a producer manager `P`, a local subscriber on `P` (the
//! proof that a frame crossed the transport), and optionally a gateway manager
//! `G` that owns the zenoh session and forwards `P`'s frames to a remote
//! demander. On the desk the same binary runs `sub`, the demander.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, SchemaHashName, SchemaServing, TopicSchema,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

/// The one topic of the spike.
pub const TOPIC: &str = "/temi/spike/vec";

/// Options of the `pub` loop.
#[derive(Debug, Clone)]
pub struct PubOpts {
    pub secs: u64,
    pub hz: u64,
    pub listen: Vec<String>,
    pub connect: Vec<String>,
    pub robot: String,
}

impl Default for PubOpts {
    fn default() -> Self {
        Self {
            secs: 10,
            hz: 5,
            listen: Vec::new(),
            connect: Vec::new(),
            robot: "temi-spike".to_string(),
        }
    }
}

/// Counters a caller (CLI or JNI) can read while the loop runs.
#[derive(Debug, Default)]
pub struct Counters {
    pub published: AtomicU64,
    pub received_local: AtomicU64,
    pub forwarded: AtomicU64,
}

fn ix_config() -> iceoryx2::config::Config {
    iceoryx2::config::Config::default()
}

fn err<E: std::fmt::Display>(what: &'static str) -> impl FnOnce(E) -> String {
    move |e| format!("{what}: {e}")
}

/// Publish `TOPIC` until `stop` is set or `opts.secs` passed.
pub fn run_pub(opts: &PubOpts, stop: &AtomicBool, counters: &Counters) -> Result<(), String> {
    let p = TransportManager::detached_with_config(
        TransportConfig {
            node_name: "temi_spike_p".to_string(),
            ..Default::default()
        },
        ix_config(),
    )
    .map_err(err("init producer manager"))?;
    let mut publisher = p
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(256))
        .map_err(err("create publisher"))?;
    let sub = p
        .create_subscriber(TOPIC)
        .map_err(err("create local subscriber"))?;
    println!("spike: publisher + local subscriber up on {TOPIC}");

    let mut gateway: Option<GatewayRuntime> = None;
    if !opts.listen.is_empty() || !opts.connect.is_empty() {
        let g = TransportManager::detached_with_config(
            TransportConfig {
                node_name: "temi_spike_g".to_string(),
                network: Some(NetworkConfig {
                    listen_endpoints: opts.listen.clone(),
                    connect_endpoints: opts.connect.clone(),
                    robot_identity: Some(opts.robot.clone()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            ix_config(),
        )
        .map_err(err("init gateway manager"))?;
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![TOPIC.to_string()],
            ingress: vec![],
        };
        // The desk daemon (cerulion-netd) refuses a catalog row with no schema
        // name, so the gateway is told the name of the one built-in type here.
        let serving = SchemaServing {
            topic_schemas: vec![TopicSchema {
                topic: TOPIC.to_string(),
                schema_name: "geometry_msgs/Vector3".to_string(),
            }],
            schema_docs: Vec::new(),
            schema_hashes: vec![SchemaHashName {
                schema_hash: Vector3::SCHEMA_HASH,
                qualified: "geometry_msgs/Vector3".to_string(),
            }],
        };
        gateway = Some(
            GatewayRuntime::new_with_schema_serving(g, plan, serving)
                .map_err(err("gateway boot"))?,
        );
        println!(
            "spike: gateway up robot={} listen={:?} connect={:?}",
            opts.robot, opts.listen, opts.connect
        );
    }

    let period = Duration::from_millis(1000 / opts.hz.max(1));
    let end = Instant::now() + Duration::from_secs(opts.secs);
    let mut next = Instant::now();
    let mut last_print = Instant::now();
    let mut i: u64 = 0;
    while Instant::now() < end && !stop.load(Ordering::Relaxed) {
        if Instant::now() >= next {
            next += period;
            {
                let mut proxy = publisher.loan_proxy::<Vector3>().map_err(err("loan"))?;
                proxy.x = i as f64;
                proxy.y = (i as f64 * 0.1).sin();
                proxy.z = 42.0;
            }
            i += 1;
            counters.published.fetch_add(1, Ordering::Relaxed);
        }
        if let Some(g) = gateway.as_mut() {
            g.drive_once().map_err(err("gateway drive"))?;
            counters
                .forwarded
                .store(g.forwarded_count(TOPIC), Ordering::Relaxed);
        }
        sub.try_receive(|msg| {
            let n = counters.received_local.fetch_add(1, Ordering::Relaxed);
            if n < 3 {
                let h = msg.header();
                let pl = msg.payload();
                let x = f64::from_le_bytes(pl[0..8].try_into().unwrap_or([0; 8]));
                println!(
                    "spike: local rx seq={} bytes={} schema={:#018x} x={x}",
                    h.sequence, h.total_size, h.schema_hash
                );
            }
        })
        .map_err(err("local receive"))?;
        if last_print.elapsed() >= Duration::from_secs(1) {
            last_print = Instant::now();
            println!(
                "spike: published={} received_local={} forwarded_to_network={} taps={:?}",
                counters.published.load(Ordering::Relaxed),
                counters.received_local.load(Ordering::Relaxed),
                counters.forwarded.load(Ordering::Relaxed),
                gateway.as_ref().map(GatewayRuntime::active_tap_topics)
            );
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    println!(
        "spike: DONE published={} received_local={} forwarded_to_network={}",
        counters.published.load(Ordering::Relaxed),
        counters.received_local.load(Ordering::Relaxed),
        counters.forwarded.load(Ordering::Relaxed)
    );
    Ok(())
}

/// The desk side: connect to a gateway, demand `TOPIC`, print what arrives.
pub fn run_sub(connect: &[String], secs: u64, want: u64) -> Result<u64, String> {
    let b = TransportManager::detached_with_config(
        TransportConfig {
            node_name: "temi_spike_b".to_string(),
            network: Some(NetworkConfig {
                connect_endpoints: connect.to_vec(),
                ..Default::default()
            }),
            ..Default::default()
        },
        ix_config(),
    )
    .map_err(err("init demander manager"))?;
    let sub = b
        .create_subscriber(TOPIC)
        .map_err(err("create subscriber"))?;
    b.register_ingress_topic(TOPIC, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .map_err(err("register ingress (the demand token)"))?;
    println!("spike-sub: demanding {TOPIC} from {connect:?}");
    let end = Instant::now() + Duration::from_secs(secs);
    let mut got: u64 = 0;
    while Instant::now() < end && got < want {
        sub.try_receive(|msg| {
            got += 1;
            let h = msg.header();
            let pl = msg.payload();
            let f = |o: usize| f64::from_le_bytes(pl[o..o + 8].try_into().unwrap_or([0; 8]));
            println!(
                "spike-sub: rx #{got} seq={} bytes={} schema={:#018x} x={} y={:.3} z={}",
                h.sequence,
                h.total_size,
                h.schema_hash,
                f(0),
                f(8),
                f(16)
            );
        })
        .map_err(err("receive"))?;
        std::thread::sleep(Duration::from_millis(20));
    }
    println!("spike-sub: DONE received={got}");
    Ok(got)
}

// ---------------------------------------------------------------------------
// JNI shape. Primitive-only signatures, so no `jni` crate and no JNIEnv call.
// Kotlin side:
//   package com.cerulion.spike
//   object Native {
//       init { System.loadLibrary("cerulion_android_spike") }
//       external fun start(listenPort: Int, secs: Int): Int   // 0 = started
//       external fun published(): Long
//       external fun receivedLocal(): Long
//       external fun forwarded(): Long
//       external fun stop()
//   }
// ---------------------------------------------------------------------------

static JNI_STOP: AtomicBool = AtomicBool::new(false);
static JNI_RUNNING: AtomicBool = AtomicBool::new(false);
static JNI_COUNTERS: std::sync::OnceLock<Arc<Counters>> = std::sync::OnceLock::new();

fn jni_counters() -> &'static Arc<Counters> {
    JNI_COUNTERS.get_or_init(|| Arc::new(Counters::default()))
}

/// `Native.start(listenPort, secs)`: 0 = started, 1 = already running.
#[no_mangle]
pub extern "system" fn Java_com_cerulion_spike_Native_start(
    _env: *mut core::ffi::c_void,
    _this: *mut core::ffi::c_void,
    listen_port: i32,
    secs: i32,
) -> i32 {
    if JNI_RUNNING.swap(true, Ordering::SeqCst) {
        return 1;
    }
    JNI_STOP.store(false, Ordering::SeqCst);
    let counters = Arc::clone(jni_counters());
    let opts = PubOpts {
        secs: u64::try_from(secs).unwrap_or(10),
        listen: if listen_port > 0 {
            vec![format!("tcp/0.0.0.0:{listen_port}")]
        } else {
            Vec::new()
        },
        ..Default::default()
    };
    std::thread::Builder::new()
        .name("cerulion-spike".to_string())
        .spawn(move || {
            if let Err(e) = run_pub(&opts, &JNI_STOP, &counters) {
                eprintln!("spike: FAILED {e}");
            }
            JNI_RUNNING.store(false, Ordering::SeqCst);
        })
        .map_or(2, |_| 0)
}

/// `Native.stop()`.
#[no_mangle]
pub extern "system" fn Java_com_cerulion_spike_Native_stop(
    _env: *mut core::ffi::c_void,
    _this: *mut core::ffi::c_void,
) {
    JNI_STOP.store(true, Ordering::SeqCst);
}

/// `Native.published()`.
#[no_mangle]
pub extern "system" fn Java_com_cerulion_spike_Native_published(
    _env: *mut core::ffi::c_void,
    _this: *mut core::ffi::c_void,
) -> i64 {
    jni_counters().published.load(Ordering::Relaxed) as i64
}

/// `Native.receivedLocal()`.
#[no_mangle]
pub extern "system" fn Java_com_cerulion_spike_Native_receivedLocal(
    _env: *mut core::ffi::c_void,
    _this: *mut core::ffi::c_void,
) -> i64 {
    jni_counters().received_local.load(Ordering::Relaxed) as i64
}

/// `Native.forwarded()`.
#[no_mangle]
pub extern "system" fn Java_com_cerulion_spike_Native_forwarded(
    _env: *mut core::ffi::c_void,
    _this: *mut core::ffi::c_void,
) -> i64 {
    jni_counters().forwarded.load(Ordering::Relaxed) as i64
}
