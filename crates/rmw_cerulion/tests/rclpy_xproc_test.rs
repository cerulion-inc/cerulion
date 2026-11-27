// SPDX-License-Identifier: AGPL-3.0-only
//! The PERMANENT cross-process rmw pin: a REAL `rclpy` process
//! (`RMW_IMPLEMENTATION=rmw_cerulion`) talking to IN-PROCESS native Cerulion
//! transport in BOTH directions over ONE shared iceoryx2 SHM region, robot-free.
//!
//! Every other rmw test in this crate drives the extern "C" surface
//! IN-PROCESS with hand-built introspection typesupports (`rmw_e2e_test.rs`):
//! it never exercises the actual `RMW_IMPLEMENTATION=rmw_cerulion` dlopen path,
//! never crosses the Python/C/Rust boundary, and never proves a NATIVE Cerulion
//! node and a ROS node meet on the same shared-memory service. This file closes
//! that gap. It pins the sensors leg (a native reader decodes
//! an rclpy `String` publisher's frames with the correct schema
//! hash, Direction A) and the
//! reverse leg (Direction B, the actuator path).
//!
//! ## The two directions
//!
//! - **Direction B (headline, actuator path)**: an in-process native
//!   [`TransportManager`] publisher publishes `N` known frames (hand oracle) →
//!   a SUBPROCESS `python3` rclpy listener subscribes with `rmw_cerulion` and
//!   prints each received value. Two message shapes: `geometry_msgs/Twist`
//!   (fixed-POD, nested `Vector3`s) and `std_msgs/String` (the variable-length
//!   wire shape). Twist parity is pinned BEHAVIORALLY here: if the native
//!   `Twist::SCHEMA_HASH` disagreed with the rmw's introspection-derived hash,
//!   the rmw subscriber would reject every frame (`SchemaMismatch`) and the
//!   child would receive nothing → the value oracle fails. The Twist value
//!   oracle probes a field beyond offset 0 in EACH of the two nested-`Vector3`
//!   copy spans (`linear.x = i`, `linear.z = i + 100.0`, `angular.z = i * 10.0`
//!   all hand-derivable): the schema hash catches DECLARATION drift but not a
//!   runtime offset/copy bug in the rmw unflatten's second span, which would
//!   leave the hash intact while corrupting fields an x-only oracle never
//!   reads.
//! - **Direction A (sensors path)**: a
//!   SUBPROCESS rclpy `std_msgs/String` talker publishes → an in-process native
//!   subscriber ([`create_subscriber_open_only`], a late-join to the
//!   rmw-created service) receives. Asserts the payload oracle, GAP-FREE wire
//!   sequences (the late joiner sees a contiguous run, starting at whatever
//!   sequence it joined on), and EXPLICITLY that the rmw-written frame's
//!   `WireHeader::schema_hash` equals the native generated
//!   `std_msgs::String::SCHEMA_HASH`.
//! - **Graph listing**: TWO SUBPROCESS rclpy processes, one publishing a
//!   `std_msgs/String` topic and one subscribing to a `geometry_msgs/Twist`
//!   topic, each list the topics of the host through
//!   `node.get_topic_names_and_types()`. Each must print the same hand-written
//!   line naming both topics with their exact ROS types: the listing of a
//!   topic another process owns, which `rmw_get_topic_names_and_types` merges
//!   from the shared-memory rmw graph registry.
//! - **Schema hash capture**: a SUBPROCESS rclpy publisher per pinned type
//!   (`CAPTURE_TYPES`) → one native subscriber per topic reads
//!   `WireHeader::schema_hash` off the first frame, prints a
//!   `MEASURED <package>/<Name> 0x<hash>` line per type, and asserts each
//!   equals the generated `SCHEMA_HASH`. The Jazzy row of the distro lane
//!   runs it with `--show-output`; its lines are the source for the wire
//!   hash table in `native_ros2_messages`' `schema_hash_pin_test.rs`.
//!
//! ## Topic + schema meeting point
//!
//! The rmw maps ROS topic names to Cerulion topics via
//! [`rmw_cerulion::runtime::ros_topic_to_cerulion`] is the IDENTITY: the
//! fully-qualified ROS name IS the canonical Cerulion name, so the native side
//! addresses the SAME iceoryx2 service by the same name (ROS `/<topic>` ↔
//! native `/<topic>`, service `/<topic>/data`). Both the native manager and the
//! child's rmw run over the DEFAULT iceoryx2 root, so they share the SHM
//! registry (the runner cleans that root between tests). iceoryx2 matches the
//! service by name + byte-slice type; the Cerulion schema hash is validated at
//! the read layer, so a hash mismatch does not break iceoryx2 connection, it
//! drops the DATA (which is exactly what Direction A asserts against).
//!
//! ## Scope + how to run (runs in the rmw distro lanes)
//!
//! The rmw distro lanes run these tests: `tools/ci/rmw-distros/gate.sh` runs them
//! with `--ignored` inside each `ros:<distro>-ros-base` container, across the
//! supported distros, against a freshly staged `librmw_cerulion.so`. A plain
//! `cargo test` still skips them (they stay `#[ignore]`'d and the module is
//! `#![cfg(unix)]`); to run one by hand, use the recipe below. Each test PRECONDITION-CHECKS
//! (loud panic with the recipe) that `python3 -c "import rclpy"` works,
//! `RMW_IMPLEMENTATION=rmw_cerulion` is set, and `AMENT_PREFIX_PATH` is set:
//! the child inherits the parent env, so the RUNNER (not the test) must source
//! ROS + stage the prefix. The recipe (the same steps as
//! `ensure_rmw_cerulion` in `benches/latency/ros2/run_bench.sh`):
//!
//! ```bash
//! # inside a ros:<distro>-ros-base container (what the rmw distro lanes use):
//! set +u; source /opt/ros/<distro>/setup.bash     # setup.bash is nounset-hostile
//! cargo build --release -p rmw_cerulion            # build the .so
//! PREFIX=/tmp/rmw_prefix; mkdir -p "$PREFIX/lib"
//! cp target/release/librmw_cerulion.so "$PREFIX/lib/"
//! export AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH"
//! export LD_LIBRARY_PATH="$PREFIX/lib:$LD_LIBRARY_PATH"
//! export RMW_IMPLEMENTATION=rmw_cerulion
//! # each direction in its OWN process, with iox state cleaned BEFORE each (what
//! # the rmw distro lanes do in tools/ci/rmw-distros/gate.sh):
//! for t in direction_a_rclpy_string_talker_to_native_subscriber \
//!          direction_b_native_twist_publisher_to_rclpy_listener \
//!          direction_b_native_string_publisher_to_rclpy_listener \
//!          rclpy_processes_list_each_others_topics; do
//!   rm -rf /tmp/iceoryx2 /dev/shm/iox2_*           # fresh iox state per test
//!   cargo test -p rmw_cerulion --test rclpy_xproc_test -- --ignored --test-threads=1 --exact "$t" --nocapture
//! done
//! ```
//!
//! Run each test with clean iox state in a FRESH process: the native
//! [`TransportManager`] is a process singleton over the default root; sweeping
//! `/dev/shm` under a live singleton corrupts it. `#[serial]` + unique per-test
//! topics/node-names are belt-and-suspenders if the runner batches them.
//!
//! No fake data (Principle #13): every oracle is a HAND-written value vector
//! (never a self-compare); the frames cross a REAL iceoryx2 SHM region between a
//! REAL rclpy process and REAL native Cerulion transport. All child waits are
//! HARD-bounded; a [`PyChild`]'s `Drop` SIGKILLs + reaps so a parent panic never
//! orphans the Python process.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Read};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber};
use native_ros2_messages::geometry_msgs::Twist;
use native_ros2_messages::std_msgs::String as RosString;
use serial_test::serial;

// ===========================================================================
// Tunables.
// ===========================================================================

/// Number of oracle frames the native side publishes / the listener collects
/// (Direction B). Small + spaced so the child never overflows its QoS-10 queue.
const N: usize = 5;

/// Contiguous frames the native late-joiner collects in Direction A (gap-free
/// run length). The talker publishes indefinitely; the native side reads a
/// contiguous window and then the [`PyChild`] `Drop` reaps the talker.
const COLLECT: usize = 8;

/// Warmup sentinel frames the native publisher sends BEFORE the oracle frames
/// (Direction B): the iceoryx2 connection-establishment latency after the child
/// subscribes lands on these throwaway frames, never on an oracle value.
const WARMUP_FRAMES: usize = 12;

/// Inter-frame gap for Direction B publishes, slow enough that the child's
/// per-wake `rmw_take` keeps up with no queue buildup.
const PUBLISH_GAP: Duration = Duration::from_millis(120);

/// Post-READY settle before the native publisher starts: lets the cross-process
/// iceoryx2 pub↔sub connection establish (same-machine SHM ⇒ generous).
const SETTLE: Duration = Duration::from_millis(1500);

/// The child's own spin/publish deadline (seconds). Well past the parent's
/// bounded waits so the parent's deadlines fire first on any wedge.
const CHILD_TIMEOUT_SECS: u64 = 40;

/// Warmup sentinel string (Direction B, `std_msgs/String` arm). Distinct from
/// every oracle value so the child filters it out cleanly.
const STRING_SENTINEL: &str = "__warmup__";

/// Fault-injection seam (SELF-TEST ONLY). When
/// `CERULION_RCLPY_XPROC_FORCE_WRONG_PAYLOAD` holds a non-empty value, the
/// Direction-B `String` arm publishes THAT value in place of every `b-{i}`
/// oracle frame, so the distro-lane gate can prove the exchange oracle detects a
/// mismatch (`tools/ci/rmw-distros/gate.sh`, self-test arm 2a). Read at ONE site
/// (the Direction-B String publish loop) through this ONE helper; nothing else in
/// the harness reads the variable. Unset in every real run, so the hand oracle
/// `b-{i}` holds.
fn forced_wrong_payload() -> Option<String> {
    match std::env::var("CERULION_RCLPY_XPROC_FORCE_WRONG_PAYLOAD") {
        Ok(v) if !v.is_empty() => Some(v),
        _ => None,
    }
}

/// The seam is never inert by accident: unset selects the oracle, set selects the
/// injected literal, and a blank value is treated as unset. Proven WITHOUT rclpy
/// or a container, so a plain `cargo test` runs it (the exchange tests themselves
/// stay `#[ignore]`'d). This arm and `forced_wrong_payload` are the only readers
/// of the seam variable.
#[test]
#[serial]
fn wrong_payload_seam_selects_literal_only_when_set() {
    const VAR: &str = "CERULION_RCLPY_XPROC_FORCE_WRONG_PAYLOAD";
    let oracle = "b-1";
    std::env::remove_var(VAR);
    assert_eq!(forced_wrong_payload(), None);
    assert_eq!(forced_wrong_payload().as_deref().unwrap_or(oracle), oracle);
    let injected = "seam-unit-literal";
    std::env::set_var(VAR, injected);
    assert_eq!(forced_wrong_payload().as_deref(), Some(injected));
    assert_eq!(
        forced_wrong_payload().as_deref().unwrap_or(oracle),
        injected
    );
    std::env::set_var(VAR, "");
    assert_eq!(forced_wrong_payload(), None);
    std::env::remove_var(VAR);
}

// ===========================================================================
// Precondition: the container-only environment contract (loud panic + recipe).
// ===========================================================================

/// The exact remediation recipe, embedded in every precondition panic so a
/// misconfigured environment fails LOUDLY with actionable guidance rather than a cryptic
/// spawn/connect error.
const RECIPE: &str = "\
rclpy cross-process test PRECONDITION failed. It runs in the rmw distro \
lanes (ros:<distro>-ros-base); to run by hand, inside a ROS container with \
librmw_cerulion.so staged:\n  \
set +u; source /opt/ros/<distro>/setup.bash\n  \
cargo build --release -p rmw_cerulion\n  \
PREFIX=/tmp/rmw_prefix; mkdir -p \"$PREFIX/lib\"; cp target/release/librmw_cerulion.so \"$PREFIX/lib/\"\n  \
export AMENT_PREFIX_PATH=\"$PREFIX:$AMENT_PREFIX_PATH\"\n  \
export LD_LIBRARY_PATH=\"$PREFIX/lib:$LD_LIBRARY_PATH\"\n  \
export RMW_IMPLEMENTATION=rmw_cerulion\n  \
# each direction in its own process, iox state cleaned before each:\n  \
for t in direction_a_rclpy_string_talker_to_native_subscriber direction_b_native_twist_publisher_to_rclpy_listener direction_b_native_string_publisher_to_rclpy_listener rclpy_processes_list_each_others_topics; do\n  \
  rm -rf /tmp/iceoryx2 /dev/shm/iox2_*; cargo test -p rmw_cerulion --test rclpy_xproc_test -- --ignored --test-threads=1 --exact \"$t\" --nocapture\n  \
done\n\
(the same steps as ensure_rmw_cerulion in benches/latency/ros2/run_bench.sh)";

/// Assert the inherited env is a real ROS 2 + rmw_cerulion environment. Panics
/// (with [`RECIPE`]) if `RMW_IMPLEMENTATION` != `rmw_cerulion`, `AMENT_PREFIX_PATH`
/// is unset, or `python3 -c "import rclpy"` fails under the inherited env.
fn precondition_or_panic() {
    let rmw = std::env::var("RMW_IMPLEMENTATION").unwrap_or_default();
    assert_eq!(
        rmw, "rmw_cerulion",
        "RMW_IMPLEMENTATION must be rmw_cerulion (got {rmw:?}).\n{RECIPE}"
    );
    assert!(
        std::env::var("AMENT_PREFIX_PATH").is_ok(),
        "AMENT_PREFIX_PATH must be set (the ament prefix staging librmw_cerulion.so).\n{RECIPE}"
    );
    let out = Command::new("python3")
        .args(["-c", "import rclpy"])
        .output()
        .unwrap_or_else(|e| panic!("failed to spawn python3: {e}.\n{RECIPE}"));
    assert!(
        out.status.success(),
        "`python3 -c \"import rclpy\"` failed ({}).\nstderr:\n{}\n{RECIPE}",
        out.status,
        String::from_utf8_lossy(&out.stderr),
    );
}

// ===========================================================================
// Native transport helpers.
// ===========================================================================

/// The process-singleton native [`TransportManager`] over the DEFAULT iceoryx2
/// root (the root the child's `rmw_cerulion` also uses, which is what lets them
/// share SHM). `subscriber_buffer_size = 16` provisions Direction B's publisher
/// service with enough subscriber ceiling for the child's QoS-10 subscription;
/// it is inert for the open-only Direction A subscriber (which opens at the
/// service's own ceiling). The singleton is per-PROCESS, so the runner must run
/// each `#[ignore]` test in a fresh process with clean iox state.
fn native_manager(node_name: &str) -> Arc<TransportManager> {
    let cfg = TransportConfig {
        node_name: node_name.to_string(),
        subscriber_buffer_size: 16,
        ..Default::default()
    };
    TransportManager::init(cfg)
        .or_else(|_| TransportManager::get())
        .expect("native TransportManager over the default iceoryx2 root")
}

/// The Cerulion topic name the rmw derives from a ROS topic, the exact meeting
/// point (`ros_topic_to_cerulion` is the identity on a fully-qualified name).
fn cerulion_topic(ros_topic: &str) -> String {
    rmw_cerulion::runtime::ros_topic_to_cerulion(ros_topic)
        .expect("a fully-qualified ROS name maps to itself")
}

/// Publish one `Twist` carrying `linear.x = x`, `linear.z = lz`,
/// `angular.z = az` (the other three components 0.0). `linear` and `angular`
/// are TWO SEPARATE nested-`Vector3` copy spans in the rmw unflatten, so the
/// Direction-B oracle stamps a non-zero probe beyond offset 0 in EACH span:
/// a runtime offset/copy bug in the second span (invisible to the schema
/// hash, which pins the DECLARATION only) corrupts a value the child reads.
/// The direct-path proxy publishes on `Drop`.
fn publish_twist(pubr: &mut CerulionPublisher, x: f64, lz: f64, az: f64) {
    let mut proxy = pubr.loan_proxy::<Twist>().expect("loan Twist");
    proxy.linear.x = x;
    proxy.linear.y = 0.0;
    proxy.linear.z = lz;
    proxy.angular.x = 0.0;
    proxy.angular.y = 0.0;
    proxy.angular.z = az;
}

/// Publish one `std_msgs/String` carrying `s`.
fn publish_string(pubr: &mut CerulionPublisher, s: &str) {
    let mut proxy = pubr.loan_proxy::<RosString>().expect("loan String");
    proxy.set_data(s).expect("set_data must fit the loan");
}

// ===========================================================================
// PyChild: a `python3 -c` rclpy subprocess with a line-stream + stderr drain
// and RAII SIGKILL-on-Drop (a wedged Python process must NEVER hang a test run).
// ===========================================================================

struct PyChild {
    child: Child,
    lines: mpsc::Receiver<String>,
    stderr: Arc<Mutex<String>>,
    reaped: bool,
}

impl PyChild {
    /// Spawn `python3 -u -c <script>` inheriting the parent env (so
    /// RMW_IMPLEMENTATION / AMENT_PREFIX_PATH / LD_LIBRARY_PATH reach the child),
    /// with stdout piped into a line channel and stderr drained on a thread
    /// (never block the child on a full pipe).
    fn spawn(script: &str, tag: &str) -> PyChild {
        let mut child = Command::new("python3")
            .arg("-u") // unbuffered stdio (plus the scripts flush=True)
            .arg("-c")
            .arg(script)
            .env("PYTHONUNBUFFERED", "1")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap_or_else(|e| {
                panic!("failed to spawn python3 rclpy child ({tag}): {e}.\n{RECIPE}")
            });

        let stdout = child.stdout.take().expect("child stdout piped");
        let (tx, rx) = mpsc::channel::<String>();
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                match line {
                    Ok(l) => {
                        if tx.send(l).is_err() {
                            break; // receiver dropped
                        }
                    }
                    Err(_) => break,
                }
            }
        });

        let stderr_pipe = child.stderr.take().expect("child stderr piped");
        let stderr_buf = Arc::new(Mutex::new(String::new()));
        let sb = Arc::clone(&stderr_buf);
        thread::spawn(move || {
            let mut s = String::new();
            let _ = BufReader::new(stderr_pipe).read_to_string(&mut s);
            *sb.lock().unwrap() = s;
        });

        PyChild {
            child,
            lines: rx,
            stderr: stderr_buf,
            reaped: false,
        }
    }

    /// Next stdout line, or `None` at the deadline / on child EOF.
    fn next_line(&self, deadline: Instant) -> Option<String> {
        let now = Instant::now();
        if now >= deadline {
            return self.lines.try_recv().ok();
        }
        self.lines.recv_timeout(deadline - now).ok()
    }

    /// Bounded wait for a line starting with `prefix` (e.g. `"READY"`).
    fn wait_for(&self, prefix: &str, deadline: Instant) -> bool {
        loop {
            match self.next_line(deadline) {
                Some(l) if l.starts_with(prefix) => return true,
                Some(_) => continue,
                None => return false,
            }
        }
    }

    /// Kill + reap the child and return whatever it wrote to stderr (used to
    /// enrich a failure panic with the Python/rmw side's diagnostics). After
    /// `kill()` + `wait()` the stderr pipe closes; a short settle lets the drain
    /// thread finish reading it.
    fn kill_and_stderr(&mut self) -> String {
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.reaped = true;
        thread::sleep(Duration::from_millis(200));
        self.stderr.lock().unwrap().clone()
    }
}

impl Drop for PyChild {
    fn drop(&mut self) {
        if !self.reaped {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

// ===========================================================================
// Python scripts (templated in Rust to avoid `{}` brace conflicts, `.replace`
// the `__TOKEN__` markers).
// ===========================================================================

const TWIST_LISTENER_PY: &str = r#"
import time, rclpy
from geometry_msgs.msg import Twist

N = __N__
recv = []

def cb(msg):
    # All three oracle fields, one per nested-Vector3 span beyond offset 0
    # (linear.x, linear.z) plus the SECOND span (angular.z). Warmup frames are
    # all-zero and filtered by linear.x on both sides.
    print("RECV %r %r %r" % (msg.linear.x, msg.linear.z, msg.angular.z), flush=True)
    recv.append(msg.linear.x)

rclpy.init()
node = rclpy.create_node("__NODE__")
sub = node.create_subscription(Twist, "__TOPIC__", cb, 10)
print("READY", flush=True)
deadline = time.time() + __TIMEOUT__
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
    if sum(1 for v in recv if v != 0.0) >= N:
        break
print("DONE", flush=True)
node.destroy_node()
rclpy.shutdown()
"#;

const STRING_LISTENER_PY: &str = r#"
import time, rclpy
from std_msgs.msg import String

N = __N__
SENTINEL = "__SENTINEL__"
recv = []

def cb(msg):
    print("RECV " + msg.data, flush=True)
    recv.append(msg.data)

rclpy.init()
node = rclpy.create_node("__NODE__")
sub = node.create_subscription(String, "__TOPIC__", cb, 10)
print("READY", flush=True)
deadline = time.time() + __TIMEOUT__
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
    if sum(1 for v in recv if v != SENTINEL) >= N:
        break
print("DONE", flush=True)
node.destroy_node()
rclpy.shutdown()
"#;

const STRING_TALKER_PY: &str = r#"
import time, rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("__NODE__")
pub = node.create_publisher(String, "__TOPIC__", 10)
print("READY", flush=True)
i = 0
deadline = time.time() + __TIMEOUT__
while rclpy.ok() and time.time() < deadline:
    m = String()
    # Content encodes the publish index, which equals the wire sequence the rmw
    # stamps (0-based, one per publish), so frame seq s carries "a-s".
    m.data = "a-%d" % i
    pub.publish(m)
    i += 1
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
"#;

/// One process of the graph-listing arm. It creates ONE endpoint (`__MAKE__`),
/// then lists the host's topics under `__PREFIX__/` until the listing equals
/// `__WANT__`, prints it as `LISTED <line>` and keeps spinning (so its own
/// endpoint stays alive for the peer) until the parent kills it. A listing
/// that never reaches `__WANT__` prints the last one seen, so a failure shows
/// what the registry returned.
const GRAPH_LISTER_PY: &str = r#"
import time, rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

PREFIX = "__PREFIX__/"
WANT = "__WANT__"

def listing(node):
    rows = []
    for name, types in node.get_topic_names_and_types():
        if name.startswith(PREFIX):
            rows.append("%s=%s" % (name, ",".join(sorted(types))))
    return ";".join(sorted(rows))

rclpy.init()
node = rclpy.create_node("__NODE__")
__MAKE__
print("READY", flush=True)
deadline = time.time() + __TIMEOUT__
last = ""
reported = False
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.25)
    if reported:
        continue
    last = listing(node)
    if last == WANT:
        print("LISTED " + last, flush=True)
        reported = True
if not reported:
    print("LISTED " + last, flush=True)
node.destroy_node()
rclpy.shutdown()
"#;

fn render(template: &str, node: &str, topic: &str) -> String {
    template
        .replace("__N__", &N.to_string())
        .replace("__TIMEOUT__", &CHILD_TIMEOUT_SECS.to_string())
        .replace("__SENTINEL__", STRING_SENTINEL)
        .replace("__NODE__", node)
        .replace("__TOPIC__", topic)
}

// ===========================================================================
// Direction B: native publisher → rclpy listener (the actuator path).
//
// The native side creates the publisher FIRST (provisioning the iceoryx2
// service with subscriber headroom), spawns the child listener (which OPENS the
// service as an rmw subscriber), waits for READY, settles the connection, then
// publishes a warmup burst followed by the N oracle frames. The child prints
// each received value; the parent filters the warmup sentinel and asserts the
// remainder equals the hand oracle in order. The Twist arm's oracle stamps a
// distinct non-zero field in EACH nested-Vector3 copy span (linear AND
// angular), see `publish_twist`.
// ===========================================================================

#[test]
#[ignore = "distro-lane exchange: run by tools/ci/rmw-distros/gate.sh inside ros:<distro>-ros-base with a staged librmw_cerulion.so (--ignored); a plain cargo test skips it. Direction B: native Twist publisher -> rclpy listener"]
#[serial]
fn direction_b_native_twist_publisher_to_rclpy_listener() {
    precondition_or_panic();

    const ROS_TOPIC: &str = "/b_twist";
    let mgr = native_manager("b_twist_native");
    let topic = cerulion_topic(ROS_TOPIC);

    // Native publisher creates the service (single-writer + subscriber headroom).
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("native Twist publisher must create the shared service");

    // Spawn the rclpy listener; it OPENS the service as an rmw subscriber.
    let script = render(TWIST_LISTENER_PY, "b_twist_listener", ROS_TOPIC);
    let mut child = PyChild::spawn(&script, "twist-listener");
    if !child.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = child.kill_and_stderr();
        panic!("rclpy Twist listener never printed READY (rmw subscription failed?).\nstderr:\n{stderr}");
    }

    // Settle the cross-process connection, then warmup (all-zero) + oracle
    // frames i = 1..=N stamping BOTH nested-Vector3 spans beyond offset 0:
    // linear.x = i, linear.z = i + 100.0, angular.z = i * 10.0.
    thread::sleep(SETTLE);
    for _ in 0..WARMUP_FRAMES {
        publish_twist(&mut pubr, 0.0, 0.0, 0.0);
        thread::sleep(PUBLISH_GAP);
    }
    for i in 1..=N {
        let i = i as f64;
        publish_twist(&mut pubr, i, i + 100.0, i * 10.0);
        thread::sleep(PUBLISH_GAP);
    }

    // Collect the child's RECV lines (three floats each) until DONE / EOF;
    // filter warmup frames by linear.x == 0.0.
    let mut got: Vec<(f64, f64, f64)> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    while let Some(line) = child.next_line(deadline) {
        if line == "DONE" {
            break;
        }
        if let Some(rest) = line.strip_prefix("RECV ") {
            let parts: Vec<f64> = rest
                .split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect();
            if parts.len() == 3 {
                got.push((parts[0], parts[1], parts[2]));
            }
        }
    }

    // Hand oracle: (i, i + 100.0, i * 10.0), a distinct non-zero value in
    // EACH span so a second-span (angular) offset/copy bug in the rmw
    // unflatten shows as a wrong VALUE, not an always-0.0 blind spot.
    let oracle: Vec<(f64, f64, f64)> = (1..=N)
        .map(|i| (i as f64, i as f64 + 100.0, i as f64 * 10.0))
        .collect();
    let received: Vec<(f64, f64, f64)> = got.into_iter().filter(|&(x, _, _)| x != 0.0).collect();
    if received != oracle {
        let stderr = child.kill_and_stderr();
        panic!(
            "rclpy listener must receive the native Twist span oracle \
             (linear.x, linear.z, angular.z) = {oracle:?} in order, got {received:?} \
             (a schema-hash mismatch drops frames silently; a second-span offset/copy \
             bug corrupts linear.z / angular.z while linear.x still matches).\nstderr:\n{stderr}"
        );
    }
    drop(pubr);
}

#[test]
#[ignore = "distro-lane exchange: run by tools/ci/rmw-distros/gate.sh inside ros:<distro>-ros-base with a staged librmw_cerulion.so (--ignored); a plain cargo test skips it. Direction B: native String publisher -> rclpy listener"]
#[serial]
fn direction_b_native_string_publisher_to_rclpy_listener() {
    precondition_or_panic();

    const ROS_TOPIC: &str = "/b_string";
    let mgr = native_manager("b_string_native");
    let topic = cerulion_topic(ROS_TOPIC);

    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("native String publisher must create the shared service");

    let script = render(STRING_LISTENER_PY, "b_string_listener", ROS_TOPIC);
    let mut child = PyChild::spawn(&script, "string-listener");
    if !child.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = child.kill_and_stderr();
        panic!("rclpy String listener never printed READY (rmw subscription failed?).\nstderr:\n{stderr}");
    }

    thread::sleep(SETTLE);
    for _ in 0..WARMUP_FRAMES {
        publish_string(&mut pubr, STRING_SENTINEL);
        thread::sleep(PUBLISH_GAP);
    }
    let oracle: Vec<String> = (1..=N).map(|i| format!("b-{i}")).collect();
    // Self-test seam: with CERULION_RCLPY_XPROC_FORCE_WRONG_PAYLOAD set, publish
    // the injected literal for every frame instead of the oracle, so the assertion
    // below MUST reject it (the distro-lane gate's Arm 2a drives this to prove the
    // exchange detects a mismatch). Unset in every real run, so `s` is published.
    let forced = forced_wrong_payload();
    for s in &oracle {
        publish_string(&mut pubr, forced.as_deref().unwrap_or(s));
        thread::sleep(PUBLISH_GAP);
    }

    let mut received = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    while let Some(line) = child.next_line(deadline) {
        if line == "DONE" {
            break;
        }
        if let Some(rest) = line.strip_prefix("RECV ") {
            if rest != STRING_SENTINEL {
                received.push(rest.to_string());
            }
        }
    }

    if received != oracle {
        let stderr = child.kill_and_stderr();
        panic!(
            "rclpy listener must receive the native String oracle {oracle:?} in order, \
             got {received:?} (the variable-length wire shape).\nstderr:\n{stderr}"
        );
    }
    drop(pubr);
}

// ===========================================================================
// Direction A: rclpy talker → native subscriber (the sensors
// path). The child publishes `std_msgs/String` indefinitely; the native
// side late-joins via `create_subscriber_open_only` and collects a contiguous
// window. Asserts: explicit schema-hash parity, gap-free wire sequences (from
// first received), and the payload oracle (frame seq s carries "a-s").
// ===========================================================================

#[test]
#[ignore = "distro-lane exchange: run by tools/ci/rmw-distros/gate.sh inside ros:<distro>-ros-base with a staged librmw_cerulion.so (--ignored); a plain cargo test skips it. Direction A: rclpy String talker -> native subscriber"]
#[serial]
fn direction_a_rclpy_string_talker_to_native_subscriber() {
    precondition_or_panic();

    const ROS_TOPIC: &str = "/a_string";
    let mgr = native_manager("a_string_native");
    let topic = cerulion_topic(ROS_TOPIC);

    // The talker creates the rmw publisher (service). Wait for READY so the
    // service exists before the native side opens it read-only.
    let script = render(STRING_TALKER_PY, "a_string_talker", ROS_TOPIC);
    let mut child = PyChild::spawn(&script, "string-talker");
    if !child.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = child.kill_and_stderr();
        panic!(
            "rclpy String talker never printed READY (rmw publisher failed?).\nstderr:\n{stderr}"
        );
    }

    // Late-join the rmw-created service (bounded retry: the service may be a
    // beat behind READY). open-only never creates a phantom service.
    let open_deadline = Instant::now() + Duration::from_secs(10);
    let sub: CerulionSubscriber = loop {
        match mgr.create_subscriber_open_only(&topic) {
            Ok(s) => break s,
            Err(_) if Instant::now() < open_deadline => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                let stderr = child.kill_and_stderr();
                panic!("native subscriber could not open the rmw-created topic {topic:?}: {e}.\nstderr:\n{stderr}");
            }
        }
    };

    // Poll for a contiguous window of frames. Poll fast so the QoS queue never
    // overflows (which would drop_oldest and break gap-freeness).
    let mut frames: Vec<(u32, u64, Vec<u8>)> = Vec::new();
    let collect_deadline = Instant::now() + Duration::from_secs(30);
    while frames.len() < COLLECT && Instant::now() < collect_deadline {
        sub.try_receive(|msg| {
            frames.push((
                msg.header().sequence,
                msg.header().schema_hash,
                msg.payload().to_vec(),
            ));
        })
        .expect("native try_receive on the rmw topic");
        thread::sleep(Duration::from_millis(3));
    }

    if frames.len() < COLLECT {
        let n = frames.len();
        let stderr = child.kill_and_stderr();
        panic!(
            "native subscriber collected only {n} frames (needed {COLLECT}): the rclpy talker \
             (RMW_IMPLEMENTATION=rmw_cerulion) never reached the shared iceoryx2 service.\nstderr:\n{stderr}"
        );
    }

    // (1) EXPLICIT schema-hash parity: the rmw-written frame's hash equals the
    // native generated std_msgs/String SCHEMA_HASH.
    let expected_hash = <RosString as ShmMessage>::SCHEMA_HASH;
    // (2) Payload oracle: frame seq s carries the talker's "a-s".
    for (seq, hash, payload) in &frames {
        assert_eq!(
            *hash, expected_hash,
            "schema-hash parity: the rmw-written frame (seq {seq}) must carry the native \
             std_msgs/String SCHEMA_HASH {expected_hash:#x}, got {hash:#x}"
        );
        let expected = format!("a-{seq}");
        assert!(
            payload
                .windows(expected.len())
                .any(|w| w == expected.as_bytes()),
            "frame seq {seq} payload must carry the talker's value {expected:?} \
             (the UTF-8 string appears verbatim in the wire frame)"
        );
    }

    // (3) Gap-free wire sequences from first received (the late joiner sees a
    // contiguous run, starting at whatever sequence it joined on).
    for w in frames.windows(2) {
        assert_eq!(
            w[1].0,
            w[0].0 + 1,
            "wire sequences must be gap-free from first received (the rmw stamps 0-based \
             per publish; the native late-joiner reads a contiguous run): {:?}",
            frames.iter().map(|f| f.0).collect::<Vec<_>>()
        );
    }

    // The talker keeps publishing; PyChild::Drop SIGKILLs + reaps it.
}

// ===========================================================================
// Graph listing: two rclpy processes list each other's topics.
//
// Child P publishes `std_msgs/String` on `<prefix>/pub`; child S subscribes to
// `geometry_msgs/Twist` on `<prefix>/sub`. Each then lists the host's topics
// under the per-run prefix. The oracle is one hand-written line, the same for
// both: a listing that omitted the peer's topic, or reported the wrong type,
// differs from it. The prefix filter drops `/parameter_events` and `/rosout`.
// ===========================================================================

#[test]
#[ignore = "distro-lane exchange: run by tools/ci/rmw-distros/gate.sh inside ros:<distro>-ros-base with a staged librmw_cerulion.so (--ignored); a plain cargo test skips it. Graph listing: two rclpy processes list each other's topics"]
#[serial]
fn rclpy_processes_list_each_others_topics() {
    precondition_or_panic();

    let prefix = format!("/xproc_graph_{}", std::process::id());
    let want = format!("{prefix}/pub=std_msgs/msg/String;{prefix}/sub=geometry_msgs/msg/Twist");
    let script = |node: &str, make: &str| {
        render(GRAPH_LISTER_PY, node, "")
            .replace("__PREFIX__", &prefix)
            .replace("__WANT__", &want)
            .replace("__MAKE__", make)
    };
    let publisher_make = format!("pub = node.create_publisher(String, \"{prefix}/pub\", 10)");
    let subscriber_make =
        format!("sub = node.create_subscription(Twist, \"{prefix}/sub\", lambda m: None, 10)");

    // P first and READY, so its topic is live before S starts listing.
    let mut publisher = PyChild::spawn(&script("graph_pub", &publisher_make), "graph-publisher");
    if !publisher.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = publisher.kill_and_stderr();
        panic!("rclpy publisher never printed READY.\nstderr:\n{stderr}");
    }
    let mut subscriber = PyChild::spawn(&script("graph_sub", &subscriber_make), "graph-subscriber");
    if !subscriber.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = subscriber.kill_and_stderr();
        panic!("rclpy subscriber never printed READY.\nstderr:\n{stderr}");
    }

    let want_line = format!("LISTED {want}");
    for (who, child) in [
        ("publisher", &mut publisher),
        ("subscriber", &mut subscriber),
    ] {
        let deadline = Instant::now() + Duration::from_secs(60);
        let got = loop {
            match child.next_line(deadline) {
                Some(l) if l.starts_with("LISTED ") => break Some(l),
                Some(_) => continue,
                None => break None,
            }
        };
        if got.as_deref() != Some(want_line.as_str()) {
            let stderr = child.kill_and_stderr();
            panic!(
                "the rclpy {who} must list exactly {want_line:?} (its own topic and the \
                 other process's, with exact ROS types), got {got:?}.\nstderr:\n{stderr}"
            );
        }
    }
}

// ===========================================================================
// Schema hash capture: one rclpy publisher per pinned type → native
// subscribers that read `WireHeader::schema_hash` off the first frame.
//
// The hashes in `LIVE_RMW_MEASURED_HASHES`
// (`crates/native_ros2_messages/tests/schema_hash_pin_test.rs`) must come off
// the wire, never from `schema_hash()`. This test is the instrument: stock
// rclpy hands each type to the rmw through its rosidl introspection
// typesupport, so the rmw derives the schema from the installed ROS packages,
// not from the vendored corpus. Each hash is printed as a `MEASURED` line
// BEFORE any assertion, so a red run still shows what the wire carried.
// The assertion is wire == the generated constant for every type; the pin
// test asserts generated == the captured table, so a green lane and a green
// pin test together mean wire == table.
// ===========================================================================

/// The pinned types the capture measures, in the order of
/// `LIVE_RMW_MEASURED_HASHES`, plus `geometry_msgs/Pose`: a type built from
/// fixed nested fields only, the control whose hash a nested-hash recipe
/// change must leave alone.
const CAPTURE_TYPES: &[(&str, &str, u64)] = &[
    (
        "vision_msgs",
        "ObjectHypothesisWithPose",
        <native_ros2_messages::vision_msgs::ObjectHypothesisWithPose as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "vision_msgs",
        "Detection2DArray",
        <native_ros2_messages::vision_msgs::Detection2DArray as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "control_msgs",
        "DynamicJointState",
        <native_ros2_messages::control_msgs::DynamicJointState as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "grid_map_msgs",
        "GridMap",
        <native_ros2_messages::grid_map_msgs::GridMap as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "object_recognition_msgs",
        "Table",
        <native_ros2_messages::object_recognition_msgs::Table as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "geometry_msgs",
        "PoseStamped",
        <native_ros2_messages::geometry_msgs::PoseStamped as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "sensor_msgs",
        "Imu",
        <native_ros2_messages::sensor_msgs::Imu as ShmMessage>::SCHEMA_HASH,
    ),
    (
        "geometry_msgs",
        "Pose",
        <native_ros2_messages::geometry_msgs::Pose as ShmMessage>::SCHEMA_HASH,
    ),
];

/// The ROS topic the capture publishes `<package>/<Name>` on: one topic per
/// type, so each frame's hash belongs to exactly one type.
fn capture_topic(package: &str, name: &str) -> String {
    format!("/hash_capture/{package}/{name}")
}

/// The rclpy child: one default-constructed message per type, published every
/// 100 ms on its own topic until the deadline. A default message is enough,
/// because the schema hash depends on the type only, never on the values.
fn capture_publisher_script() -> String {
    let mut py = String::from("import time, rclpy\n");
    for (package, name, _) in CAPTURE_TYPES {
        py.push_str(&format!("from {package}.msg import {name}\n"));
    }
    py.push_str("rclpy.init()\nnode = rclpy.create_node(\"hash_capture_talker\")\npubs = []\n");
    for (package, name, _) in CAPTURE_TYPES {
        let topic = capture_topic(package, name);
        py.push_str(&format!(
            "pubs.append((node.create_publisher({name}, \"{topic}\", 10), {name}))\n"
        ));
    }
    py.push_str(&format!(
        "print(\"READY\", flush=True)\n\
         deadline = time.time() + {CHILD_TIMEOUT_SECS}\n\
         while rclpy.ok() and time.time() < deadline:\n    \
         for p, t in pubs:\n        \
         p.publish(t())\n    \
         time.sleep(0.1)\n\
         node.destroy_node()\n\
         rclpy.shutdown()\n"
    ));
    py
}

/// The import probe the capture runs before it spawns the publisher, so a
/// message package missing from the container fails with its name.
fn capture_import_probe() -> String {
    let mut py = String::new();
    for (package, name, _) in CAPTURE_TYPES {
        py.push_str(&format!("from {package}.msg import {name}\n"));
    }
    py
}

/// The capture's inputs, proven without rclpy or a container: every type is
/// named once, each on its own topic that the rmw maps to itself, and the
/// publisher script imports and publishes each type exactly once.
#[test]
#[serial]
fn capture_publishes_every_type_once_on_its_own_topic() {
    let names: std::collections::BTreeSet<(&str, &str)> =
        CAPTURE_TYPES.iter().map(|(p, n, _)| (*p, *n)).collect();
    assert_eq!(names.len(), 8, "eight distinct types: {names:?}");
    assert!(names.contains(&("geometry_msgs", "Pose")));
    assert_eq!(
        capture_topic("geometry_msgs", "Pose"),
        "/hash_capture/geometry_msgs/Pose"
    );

    let script = capture_publisher_script();
    let probe = capture_import_probe();
    for (package, name, _) in CAPTURE_TYPES {
        let topic = capture_topic(package, name);
        assert_eq!(cerulion_topic(&topic), topic);
        let import = format!("from {package}.msg import {name}\n");
        assert_eq!(script.matches(&import).count(), 1, "{import:?}");
        assert_eq!(probe.matches(&import).count(), 1, "{import:?}");
        let publisher = format!("node.create_publisher({name}, \"{topic}\", 10)");
        assert_eq!(script.matches(&publisher).count(), 1, "{publisher:?}");
    }
    assert!(
        script.contains("\n    for p, t in pubs:\n        p.publish(t())\n    time.sleep(0.1)\n")
    );
}

#[test]
#[ignore = "distro-lane capture: run by tools/ci/rmw-distros/gate.sh on the Jazzy row inside ros:jazzy-ros-base with a staged librmw_cerulion.so and the vision, control, grid map and object recognition message packages (--ignored --show-output); a plain cargo test skips it. Prints one MEASURED line per type"]
#[serial]
fn schema_hash_capture_rclpy_publishers_to_native_subscribers() {
    precondition_or_panic();
    let probe = Command::new("python3")
        .args(["-c", &capture_import_probe()])
        .output()
        .unwrap_or_else(|e| panic!("failed to spawn python3: {e}.\n{RECIPE}"));
    assert!(
        probe.status.success(),
        "the capture needs every message package importable under the sourced ROS env \
         (ros-<distro>-vision-msgs, -control-msgs, -grid-map-msgs, -object-recognition-msgs).\n\
         stderr:\n{}\n{RECIPE}",
        String::from_utf8_lossy(&probe.stderr),
    );

    let mgr = native_manager("hash_capture_native");
    let mut child = PyChild::spawn(&capture_publisher_script(), "hash-capture-talker");
    if !child.wait_for("READY", Instant::now() + Duration::from_secs(30)) {
        let stderr = child.kill_and_stderr();
        panic!("rclpy capture publisher never printed READY (rmw publisher failed?).\nstderr:\n{stderr}");
    }

    // Late-join every rmw-created service (bounded retry, as in Direction A).
    let open_deadline = Instant::now() + Duration::from_secs(10);
    let mut subs: Vec<CerulionSubscriber> = Vec::with_capacity(CAPTURE_TYPES.len());
    for (package, name, _) in CAPTURE_TYPES {
        let topic = cerulion_topic(&capture_topic(package, name));
        let sub = loop {
            match mgr.create_subscriber_open_only(&topic) {
                Ok(s) => break s,
                Err(_) if Instant::now() < open_deadline => {
                    thread::sleep(Duration::from_millis(50));
                }
                Err(e) => {
                    let stderr = child.kill_and_stderr();
                    panic!("native subscriber could not open the rmw-created topic {topic:?}: {e}.\nstderr:\n{stderr}");
                }
            }
        };
        subs.push(sub);
    }

    // The first frame on each topic gives that type's wire hash. A receive
    // error ends that topic's polling and is reported with the rest, so every
    // other type still gets its MEASURED line.
    let mut measured: Vec<Result<Option<u64>, String>> = vec![Ok(None); CAPTURE_TYPES.len()];
    let collect_deadline = Instant::now() + Duration::from_secs(30);
    while measured.iter().any(|m| matches!(m, Ok(None))) && Instant::now() < collect_deadline {
        for (sub, slot) in subs.iter().zip(measured.iter_mut()) {
            if let Ok(None) = slot {
                let mut first = None;
                match sub.try_receive(|msg| {
                    first.get_or_insert(msg.header().schema_hash);
                }) {
                    Ok(_) => *slot = Ok(first),
                    Err(e) => *slot = Err(e.to_string()),
                }
            }
        }
        thread::sleep(Duration::from_millis(3));
    }

    for ((package, name, _), hash) in CAPTURE_TYPES.iter().zip(&measured) {
        match hash {
            Ok(Some(h)) => println!("MEASURED {package}/{name} 0x{h:016x}"),
            Ok(None) => println!("NOT MEASURED {package}/{name}: no frame within the deadline"),
            Err(e) => println!("NOT MEASURED {package}/{name}: receive failed: {e}"),
        }
    }

    let mut problems: Vec<String> = Vec::new();
    for ((package, name, generated), hash) in CAPTURE_TYPES.iter().zip(&measured) {
        match hash {
            Ok(None) => problems.push(format!(
                "  {package}/{name}: no frame reached the native subscriber"
            )),
            Err(e) => problems.push(format!("  {package}/{name}: receive failed: {e}")),
            Ok(Some(h)) if h != generated => problems.push(format!(
                "  {package}/{name}: wire 0x{h:016x}, generated SCHEMA_HASH 0x{generated:016x}"
            )),
            Ok(Some(_)) => {}
        }
    }
    if !problems.is_empty() {
        let stderr = child.kill_and_stderr();
        panic!(
            "{} of {} captured type(s) did not match: the rmw, reading the installed ROS \
             packages, disagrees with the vendored corpus about the schema hash, or no frame \
             arrived.\n{}\nstderr:\n{stderr}",
            problems.len(),
            CAPTURE_TYPES.len(),
            problems.join("\n"),
        );
    }
}
