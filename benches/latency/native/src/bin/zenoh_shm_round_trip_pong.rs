//! zenoh-SHM round-trip responder (mirrors zenoh's
//! [`z_pong.rs`](https://github.com/eclipse-zenoh/zenoh/blob/main/examples/examples/z_pong.rs)).
//!
//! Subscribes to `test/ping`, echoes every received payload to `test/pong`.
//! Runs forever; the parent process (`zenoh_shm_round_trip`) kills it once
//! the size sweep completes.
//!
//! The callback re-publishes `sample.payload().clone()` — for SHM-backed
//! payloads the clone is an Arc bump on the same SHM ref, so the bytes
//! round-trip without a copy in either direction.

use cerulion_latency_bench::zenoh_shm_config;
use zenoh::{key_expr::keyexpr, qos::CongestionControl, Wait};

fn main() {
    let _lock = cerulion_latency_bench::acquire_dma_lock();
    zenoh::init_log_from_env_or("error");

    // Pong listens on the fixed loopback locator; ping connects to it.
    let session = zenoh::open(zenoh_shm_config(/* listen */ true))
        .wait()
        .expect("open zenoh session (pong)");

    let key_ping = keyexpr::new("test/ping").expect("ping key");
    let key_pong = keyexpr::new("test/pong").expect("pong key");

    let publisher = session
        .declare_publisher(key_pong)
        .congestion_control(CongestionControl::Block)
        .wait()
        .expect("declare pong publisher");

    session
        .declare_subscriber(key_ping)
        .callback(move |sample| {
            publisher
                .put(sample.payload().clone())
                .wait()
                .expect("pong republish");
        })
        .background()
        .wait()
        .expect("declare ping subscriber");

    eprintln!("zenoh_shm_round_trip_pong: ready, parking thread");
    std::thread::park();
}
