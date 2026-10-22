import os
import shutil
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest

import cerulion

DYLIB = ".dylib" if sys.platform == "darwin" else ".so"


CLI = os.environ.get("CERULION_CLI_BIN")
PYTHON = sys.executable
ROOT_TARGET = os.environ.get("CERULION_ROOT_TARGET_DIR")

pytestmark = pytest.mark.skipif(
    not CLI,
    reason="set CERULION_CLI_BIN to run the CLI Python-node e2e",
)


def _run(cli, *args, cwd, env, check=True):
    return subprocess.run(
        [cli, *args],
        cwd=cwd,
        env=env,
        check=check,
        capture_output=True,
        text=True,
    )


def _python_libdir():
    result = subprocess.run(
        [
            PYTHON,
            "-c",
            "import sysconfig; print(sysconfig.get_config_var('LIBDIR') or '')",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _echo_topic(topic, graph, *, cwd, env, marker="x:"):
    """Stream ``topic echo`` for a few seconds while ``graph`` runs; return its stdout."""
    out = ""
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and graph.poll() is None:
        echo = subprocess.Popen(
            [CLI, "topic", "echo", topic],
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            out, _ = echo.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            echo.send_signal(signal.SIGINT)
            out, _ = echo.communicate(timeout=10)
        if marker in out:
            return out
        time.sleep(0.2)
    return out


def test_cli_python_node_build_info_and_graph_run(tmp_path):
    env = os.environ.copy()
    env.pop("CERULION_PYTHON", None)
    env.pop("CERULION_PY_PATH", None)
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("DYLD_LIBRARY_PATH", None)
    env["PATH"] = f"{Path(PYTHON).parent}{os.pathsep}{env.get('PATH', '')}"
    env["CERULION_LOGIN_GATE"] = "off"
    _run(CLI, "workspace", "create", "ws", cwd=tmp_path, env=env)
    workspace = tmp_path / "ws"

    rejected = _run(
        CLI,
        "node",
        "new",
        "--lang",
        "python",
        "echo",
        "-i",
        "geometry_msgs/Vector3",
        "inp",
        "-o",
        "geometry_msgs/Vector3",
        "out",
        cwd=workspace,
        env=env,
        check=False,
    )
    assert rejected.returncode != 0
    assert (
        "Python nodes need a trigger policy: pass -T SCHEMA NAME or "
        "--policy period_ms=N (Python nodes cannot be modified after creation)"
        in rejected.stderr
    )
    assert not (workspace / "nodes" / "echo").exists()

    created = _run(
        CLI,
        "node",
        "new",
        "--lang",
        "python",
        "echo",
        "-T",
        "geometry_msgs/Vector3",
        "inp",
        "-o",
        "geometry_msgs/Vector3",
        "out",
        cwd=workspace,
        env=env,
    )
    assert created.stdout == "Created node type 'echo'\n"

    node_py = workspace / "nodes" / "echo" / "node.py"
    source = node_py.read_text()
    assert '@cerulion.node(trigger="inp")' in source
    assert "from cerulion.msgs.geometry_msgs import Vector3\n" in source
    assert "inp = cerulion.input(Vector3, trigger=True)" in source
    source = source.replace(
        "        # copy fields here, e.g. out.x = msg.x",
        "        out.x = msg.x + 1.5\n        out.y = msg.y - 2.0\n        out.z = msg.z + 0.25",
    )
    node_py.write_text(source)

    built = _run(CLI, "node", "build", "echo", cwd=workspace, env=env)
    assert built.returncode == 0
    lib_source = (workspace / "nodes" / "echo" / "src" / "lib.rs").read_text()
    assert r'\"schema_hash\":15293913555552287199' in lib_source
    libdir = _python_libdir()
    assert libdir
    cdylib = workspace / "target" / "debug" / ("libecho" + DYLIB)
    if sys.platform == "darwin":
        dynamic = subprocess.run(
            ["otool", "-l", str(cdylib)], check=True, capture_output=True, text=True
        ).stdout
        runpath_lines = [
            line.strip() for line in dynamic.splitlines() if line.strip().startswith("path ")
        ]
    else:
        dynamic = subprocess.run(
            ["readelf", "-d", str(cdylib)], check=True, capture_output=True, text=True
        ).stdout
        runpath_lines = [
            line for line in dynamic.splitlines() if "RUNPATH" in line or "RPATH" in line
        ]
    assert any(libdir in line for line in runpath_lines), (
        f"expected {libdir!r} in cdylib rpath, got:\n{dynamic}"
    )

    info = _run(CLI, "node", "info", "echo", cwd=workspace, env=env)
    assert info.stdout == (
        "Node type: echo\n"
        "Policy: trigger:inp\n"
        "Inputs:\n"
        "  inp (geometry_msgs/Vector3)\n"
        "Outputs:\n"
        "  out (geometry_msgs/Vector3)\n"
        "Params: none\n"
    )

    if not ROOT_TARGET:
        pytest.fail("CERULION_ROOT_TARGET_DIR is required for the Rust peer graph")
    producer_src = (
        Path(__file__).parents[3]
        / "crates/test_fixtures/test_node_macro_period_cdylib/src/lib.rs"
    )
    producer = _run(
        CLI,
        "node",
        "new",
        "producer",
        "--policy",
        "period_ms=1",
        "-o",
        "geometry_msgs/Vector3",
        "cmd",
        cwd=workspace,
        env=env,
    )
    assert producer.stdout == "Created node type 'producer'\n"
    (workspace / "nodes" / "producer" / "src" / "lib.rs").write_text(
        producer_src.read_text()
    )
    fixture_candidates = list(
        Path(ROOT_TARGET).rglob("libtest_node_macro_period_cdylib" + DYLIB)
    )
    assert fixture_candidates, "build test_node_macro_period_cdylib first"
    target_debug = workspace / "target" / "debug"
    target_debug.mkdir(parents=True, exist_ok=True)
    shutil.copy2(fixture_candidates[0], target_debug / ("libproducer" + DYLIB))

    graph = workspace / "graphs"
    graph.mkdir(exist_ok=True)
    (graph / "echo.yaml").write_text(
        """\
prefix: echo
nodes:
  - id: producer
    type: producer
    outputs:
      - name: cmd
        schema: geometry_msgs/Vector3
  - id: echo
    type: echo
    inputs:
      - name: inp
        source: producer/cmd
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
"""
    )
    run = subprocess.Popen(
        [CLI, "graph", "run", "echo", "--single-process", "--network", "off"],
        cwd=workspace,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    echo = _echo_topic("/echo/echo/out", run, cwd=workspace, env=env)
    bag_path = workspace / "journey.mcap"
    recorded = None
    try:
        if run.poll() is None:
            recorded = subprocess.run(
                [CLI, "bag", "record", "/echo/echo/out", "-o", str(bag_path), "--duration", "2"],
                cwd=workspace,
                env=env,
                capture_output=True,
                text=True,
                timeout=60,
            )
    finally:
        # A recorder that hangs or fails must not leave the graph running.
        try:
            stdout, stderr = run.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            run.send_signal(signal.SIGINT)
            try:
                stdout, stderr = run.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                run.kill()
                stdout, stderr = run.communicate()
    assert run.returncode in (0, -signal.SIGINT), stderr
    # The producer publishes a zero vector; only the Python tick adds the offsets.
    assert "x: 1.5" in echo and "y: -2" in echo and "z: 0.25" in echo, echo
    combined = stdout + stderr
    assert "schema 'producer'.cmd: geometry_msgs/Vector3" in combined
    assert "schema 'echo'.out: geometry_msgs/Vector3" in combined

    assert recorded is not None, "graph exited before the bag was recorded"
    assert recorded.returncode == 0, recorded.stderr
    schemas = cerulion.SchemaSet.builtins()
    vector_hash = schemas.schema_hash("geometry_msgs/Vector3")
    with cerulion.open_bag(bag_path) as bag:
        topics = bag.topics()
        records = list(bag.messages("/echo/echo/out"))
    assert len(topics) == 1
    assert topics[0][:3] == ("/echo/echo/out", "geometry_msgs/Vector3", vector_hash)
    assert topics[0].count == len(records) > 0
    sequences = [record.sequence for _, record in records]
    assert sequences == sorted(set(sequences))
    for topic, record in records:
        assert topic == "/echo/echo/out"
        assert record.schema_hash == vector_hash
        message = record.view("geometry_msgs/Vector3", schemas=schemas)
        assert (message.x, message.y, message.z) == (1.5, -2.0, 0.25)


def test_cli_python_sync_node_takes_repeated_inputs_and_rejects_one(tmp_path):
    env = os.environ.copy()
    env["CERULION_LOGIN_GATE"] = "off"
    _run(CLI, "workspace", "create", "ws", cwd=tmp_path, env=env)
    workspace = tmp_path / "ws"

    single = _run(
        CLI, "node", "create", "--lang", "python", "lonely",
        "-i", "std_msgs/Int32", "a",
        "--policy", "sync_window_ms=10",
        cwd=workspace, env=env, check=False,
    )
    assert single.returncode != 0
    assert (
        "a sync_window_ms Python node aligns two or more inputs: "
        "repeat `-i SCHEMA NAME` for each" in single.stderr
    )
    assert not (workspace / "nodes" / "lonely").exists()

    _run(
        CLI, "node", "create", "--lang", "python", "pair",
        "-i", "std_msgs/Int32", "a",
        "-i", "std_msgs/Int32", "b",
        "-o", "std_msgs/Int32", "x",
        "-o", "std_msgs/Int32", "y",
        "--policy", "sync_window_ms=10",
        cwd=workspace, env=env,
    )
    source = (workspace / "nodes" / "pair" / "node.py").read_text()
    for line in (
        "import cerulion\nfrom cerulion.msgs.std_msgs import Int32\n",
        "@cerulion.node(sync_window_ms=10)",
        "a = cerulion.input(Int32, trigger=True)",
        "b = cerulion.input(Int32, trigger=True)",
        "x = cerulion.output(Int32)",
        "y = cerulion.output(Int32)",
    ):
        assert line in source, source


def test_cli_python_node_builds_builtin_schema_without_workspace_schema(tmp_path):
    env = os.environ.copy()
    env.pop("CERULION_PYTHON", None)
    env.pop("CERULION_PY_PATH", None)
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("DYLD_LIBRARY_PATH", None)
    env["PATH"] = f"{Path(PYTHON).parent}{os.pathsep}{env.get('PATH', '')}"
    env["CERULION_LOGIN_GATE"] = "off"
    _run(CLI, "workspace", "create", "ws", cwd=tmp_path, env=env)
    workspace = tmp_path / "ws"
    shutil.rmtree(workspace / "schemas")
    assert not (workspace / "schemas").exists()

    created = _run(
        CLI,
        "node",
        "new",
        "--lang",
        "python",
        "echo",
        "-T",
        "geometry_msgs/Vector3",
        "inp",
        "-o",
        "geometry_msgs/Vector3",
        "out",
        cwd=workspace,
        env=env,
    )
    assert created.returncode == 0

    built = _run(CLI, "node", "build", "echo", cwd=workspace, env=env)
    assert built.returncode == 0, built.stderr
    lib_source = (workspace / "nodes" / "echo" / "src" / "lib.rs").read_text()
    assert r'\"schema_hash\":15293913555552287199' in lib_source


POSE_YAML = """\
header:
  stamp:
    sec: 7
    nanosec: 8
  frame_id: "base_link"
pose:
  position: {x: 1.5, y: -2.0, z: 0.25}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""

POSE_NODE = f'''\
import cerulion
from cerulion.msgs.geometry_msgs import PoseStamped

POSE = """{POSE_YAML}"""


@cerulion.node(period_ms=10)
class PosePub:
    out = cerulion.output(PoseStamped)

    def tick(self):
        self.publish_yaml("out", POSE)
'''

# geometry_msgs/PoseStamped body, packed by hand: the fixed Pose (seven
# float64), one offset entry for the variable Header (payload offset 64,
# 25 bytes), then the Header body: stamp (int32 sec, uint32 nanosec), its
# own offset entry for frame_id (offset 16, 9 bytes) and the string bytes.
POSE_BODY = (
    struct.pack("<7d", 1.5, -2.0, 0.25, 0.0, 0.0, 0.0, 1.0)
    + struct.pack("<II", 64, 25)
    + struct.pack("<iI", 7, 8)
    + struct.pack("<II", 16, 9)
    + b"base_link"
)


def _python_node_env():
    env = os.environ.copy()
    env.pop("CERULION_PY_PATH", None)
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("DYLD_LIBRARY_PATH", None)
    env["PATH"] = f"{Path(PYTHON).parent}{os.pathsep}{env.get('PATH', '')}"
    env["CERULION_LOGIN_GATE"] = "off"
    return env


def _record_pose_run(workspace, env, *, echo=False):
    """Run the pose graph with --record for a few seconds; return the bag
    path and, with ``echo``, what ``topic echo`` printed meanwhile."""
    recordings = workspace / "recordings"
    shutil.rmtree(recordings, ignore_errors=True)
    run = subprocess.Popen(
        [CLI, "graph", "run", "pose", "--single-process", "--network", "off",
         "--record=recordings"],
        cwd=workspace,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    printed = ""
    try:
        if echo:
            printed = _echo_topic("/pose/posepub/out", run, cwd=workspace, env=env,
                                  marker="frame_id:")
        else:
            time.sleep(3)
    finally:
        run.send_signal(signal.SIGINT)
        try:
            stdout, stderr = run.communicate(timeout=60)
        except subprocess.TimeoutExpired:
            run.kill()
            stdout, stderr = run.communicate()
    assert run.returncode in (0, -signal.SIGINT), stdout + stderr
    bags = sorted(recordings.glob("*.mcap"))
    assert len(bags) == 1, (bags, stderr)
    return bags[0], printed


def test_python_node_publishes_yaml_built_message_recorded_and_resimmed(tmp_path):
    env = _python_node_env()
    _run(CLI, "workspace", "create", "ws", cwd=tmp_path, env=env)
    workspace = tmp_path / "ws"
    _run(
        CLI, "node", "new", "--lang", "python", "posepub",
        "--policy", "period_ms=10", "-o", "geometry_msgs/PoseStamped", "out",
        cwd=workspace, env=env,
    )
    (workspace / "nodes" / "posepub" / "node.py").write_text(POSE_NODE)
    built = _run(CLI, "node", "build", "posepub", cwd=workspace, env=env, check=False)
    assert built.returncode == 0, built.stderr
    (workspace / "graphs").mkdir(exist_ok=True)
    (workspace / "graphs" / "pose.yaml").write_text(
        """\
prefix: pose
nodes:
  - id: posepub
    type: posepub
    outputs:
      - name: out
        schema: geometry_msgs/PoseStamped
"""
    )

    schemas = cerulion.SchemaSet.builtins()
    pose_hash = schemas.schema_hash("geometry_msgs/PoseStamped")
    # The binding's codec builds the hand-packed body from the same YAML.
    assert schemas.yaml_to_message("geometry_msgs/PoseStamped", POSE_YAML)[32:] == POSE_BODY

    bodies = []
    for attempt in range(2):
        bag, printed = _record_pose_run(workspace, env, echo=attempt == 0)
        with cerulion.open_bag(bag) as opened:
            records = [record for _, record in opened.messages("/pose/posepub/out")]
        assert records, "the run recorded no frame on /pose/posepub/out"
        for record in records:
            raw = bytes(record.raw)
            assert struct.unpack_from("<QIII", raw) == (pose_hash, 32 + len(POSE_BODY), 88, 1)
            assert raw[32:] == POSE_BODY
        bodies.append({bytes(record.raw)[32:] for record in records})

        resim = _run(
            CLI, "bag", "play", "--resim", "all", "--verify", str(bag),
            cwd=workspace, env=env, check=False,
        )
        assert resim.returncode == 0, resim.stdout + resim.stderr

        if attempt == 0:
            # What `topic echo` printed reads back through the Python API
            # unchanged, to the same bytes.
            # One document: the `# <type>` comment line, the message, `---`
            # (the capture also holds the CLI's log lines).
            lines = printed.splitlines()
            start = lines.index("# geometry_msgs/PoseStamped") + 1
            text = "".join(line + "\n" for line in lines[start : lines.index("---", start)])
            assert "frame_id:" in text, printed
            frame = schemas.yaml_to_message("geometry_msgs/PoseStamped", text)
            assert frame[32:] == POSE_BODY
    # Two runs publish bit-identical frames.
    assert bodies[0] == bodies[1] == {POSE_BODY}


def test_cli_one_msg_file_types_a_rust_and_a_python_port(tmp_path):
    # One `.msg` file in the workspace store is the only definition of
    # `demo_pkg/Reading`: the Rust producer imports the type the message crate
    # generated from it, the Python node the class `cerulion.msgs` built from
    # it, and the graph carries frames between them.
    env = os.environ.copy()
    env.pop("CERULION_PYTHON", None)
    env.pop("CERULION_PY_PATH", None)
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("DYLD_LIBRARY_PATH", None)
    env["PATH"] = f"{Path(PYTHON).parent}{os.pathsep}{env.get('PATH', '')}"
    env["CERULION_LOGIN_GATE"] = "off"
    _run(CLI, "workspace", "create", "ws", cwd=tmp_path, env=env)
    workspace = tmp_path / "ws"
    store = workspace / "schemas" / "demo_pkg" / "msg"
    store.mkdir(parents=True)
    (store / "Reading.msg").write_text("float64 value\ngeometry_msgs/Point position\n")
    # YAML types the producer never uses still generate into its message
    # crate: a package-less one and a packaged one that embeds it.
    (workspace / "schemas" / "extra.yaml").write_text(
        'schemas:\n  Note:\n    fields:\n      "float64 level":\n'
        '  yam/Wrapper:\n    fields:\n      "Note inner":\n      "uint8 z":\n'
    )

    _run(
        CLI, "node", "new", "producer", "--policy", "period_ms=5",
        "-o", "demo_pkg/Reading", "cmd",
        cwd=workspace, env=env,
    )
    producer_lib = workspace / "nodes" / "producer" / "src" / "lib.rs"
    scaffold = producer_lib.read_text()
    assert "use native_ros2_messages::demo_pkg::Reading;" in scaffold, scaffold
    producer_lib.write_text(
        """\
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::demo_pkg::Reading;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct ProducerNode {
    #[output]
    cmd: Reading,
}

#[cerulion_node_impl]
impl ProducerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.value = 40.0;
        self.cmd.position.x = 2.0;
        Ok(())
    }
}
"""
    )
    built = _run(CLI, "node", "build", "producer", cwd=workspace, env=env, check=False)
    assert built.returncode == 0, built.stdout + built.stderr

    _run(
        CLI, "node", "new", "--lang", "python", "relay",
        "-T", "Reading", "inp", "-o", "demo_pkg/Reading", "out",
        cwd=workspace, env=env,
    )
    node_py = workspace / "nodes" / "relay" / "node.py"
    source = node_py.read_text()
    assert "from cerulion.msgs.demo_pkg import Reading\n" in source, source
    node_py.write_text(
        source.replace(
            "        # copy fields here, e.g. out.x = msg.x",
            "        assert isinstance(msg, Reading)\n"
            "        out.value = msg.value + 1.5\n"
            "        out.position.x = msg.position.x * 3",
        )
    )
    built = _run(CLI, "node", "build", "relay", cwd=workspace, env=env, check=False)
    assert built.returncode == 0, built.stdout + built.stderr
    stub = workspace / ".cerulion" / "python" / "cerulion-stubs" / "msgs" / "demo_pkg.pyi"
    assert "class Reading(_Message):" in stub.read_text()
    assert (stub.parent.parent / "py.typed").read_text() == "partial\n"

    graph = workspace / "graphs"
    (graph / "relay.yaml").write_text(
        """\
prefix: relay
nodes:
  - id: producer
    type: producer
    outputs:
      - name: cmd
        schema: demo_pkg/Reading
  - id: relay
    type: relay
    inputs:
      - name: inp
        source: producer/cmd
    outputs:
      - name: out
        schema: demo_pkg/Reading
"""
    )
    run = subprocess.Popen(
        [CLI, "graph", "run", "relay", "--single-process", "--network", "off"],
        cwd=workspace,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        echo = ""
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and run.poll() is None and "value:" not in echo:
            echo_proc = subprocess.Popen(
                [CLI, "topic", "echo", "/relay/relay/out"],
                cwd=workspace,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                echo, _ = echo_proc.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                echo_proc.send_signal(signal.SIGINT)
                echo, _ = echo_proc.communicate(timeout=10)
    finally:
        run.send_signal(signal.SIGINT)
        try:
            stdout, stderr = run.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            run.kill()
            stdout, stderr = run.communicate()
    assert "value: 41.5" in echo and "x: 6" in echo, echo + stdout + stderr
    assert run.returncode in (0, -signal.SIGINT), stderr

    # Delivery already proves the two hashes agree: the Python host refuses an
    # input frame whose hash differs from its class's (SchemaMismatch), so no
    # relayed value would reach the echo. The class also carries the hash its
    # node build baked.
    check = subprocess.run(
        [
            PYTHON,
            "-c",
            "from cerulion.msgs.demo_pkg import Reading; "
            "print(Reading.__cerulion_schema_hash__)",
        ],
        cwd=workspace,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    lib_source = (workspace / "nodes" / "relay" / "src" / "lib.rs").read_text()
    assert f'\\"schema_hash\\":{check.stdout.strip()}' in lib_source
