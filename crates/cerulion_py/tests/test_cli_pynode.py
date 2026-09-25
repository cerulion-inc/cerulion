import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

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


def _echo_topic(topic, graph, *, cwd, env):
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
        if "x:" in out:
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
    assert '@cer.node(trigger="inp")' in source
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
    try:
        stdout, stderr = run.communicate(timeout=1)
    except subprocess.TimeoutExpired:
        run.send_signal(signal.SIGINT)
        stdout, stderr = run.communicate(timeout=10)
    assert run.returncode in (0, -signal.SIGINT), stderr
    # The producer publishes a zero vector; only the Python tick adds the offsets.
    assert "x: 1.5" in echo and "y: -2" in echo and "z: 0.25" in echo, echo
    combined = stdout + stderr
    assert "schema 'producer'.cmd: geometry_msgs/Vector3" in combined
    assert "schema 'echo'.out: geometry_msgs/Vector3" in combined


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
        "@cer.node(sync_window_ms=10)",
        'a = cer.input("std_msgs/Int32", trigger=True)',
        'b = cer.input("std_msgs/Int32", trigger=True)',
        'x = cer.output("std_msgs/Int32")',
        'y = cer.output("std_msgs/Int32")',
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
