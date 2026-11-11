import ctypes
import json
import os
import subprocess
import sys
import sysconfig

import pytest

DYLIB = ".dylib" if sys.platform == "darwin" else ".so"


FIXTURE = os.environ.get("CERULION_PY_FIXTURE")
_DEFAULT_PYNODE_DIR = os.path.join(os.path.dirname(__file__), "..", "target", "pynodes")
PYNODE_DIR = os.environ.get("CERULION_PYNODE_DIR") or (
    _DEFAULT_PYNODE_DIR if os.path.isdir(_DEFAULT_PYNODE_DIR) else None
)


pytestmark = pytest.mark.skipif(
    not FIXTURE or not PYNODE_DIR,
    reason="set CERULION_PY_FIXTURE and build the fixtures/pynodes crates into "
    "target/pynodes (or set CERULION_PYNODE_DIR) to run embedded pynode fixtures",
)


def test_host_pynode_harness_is_deterministic():
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    env = _node_env("counter")
    first = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    second = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    first_ticks = [line for line in first.stdout.splitlines() if line.startswith("tick=")]
    second_ticks = [line for line in second.stdout.splitlines() if line.startswith("tick=")]
    assert first_ticks == second_ticks == [
        "tick=0 code=0 out=0100000000000000",
        "tick=1 code=0 out=0300000000000000",
    ]


def test_host_pynode_continues_a_restored_output_sequence():
    # A restored replay seeds each output publisher; the Python node's first
    # frame must carry the seed, the next seed + 1, exactly as a Rust node's.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--seed", "41"],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("counter"),
    )
    assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000 seq=41",
        "tick=1 code=0 out=0300000000000000 seq=42",
    ]


@pytest.mark.parametrize(
    "extra, message",
    [
        (["--also"], "--also requires a node path"),
        (["--seed"], "--seed requires a sequence number"),
        (["--seed", "-1"], "--seed:"),
        (["--bogus"], "unknown host-pynode option '--bogus'"),
    ],
)
def test_host_pynode_refuses_malformed_options(extra, message):
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "1", *extra],
        capture_output=True,
        text=True,
        env=_node_env("counter"),
    )
    assert result.returncode != 0
    assert message in result.stderr
    assert "node_info=" not in result.stdout


def test_host_pynode_loans_builtin_output_without_workspace_schemas():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_builtin" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "1"],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("builtin"),
    )
    lines = result.stdout.splitlines()
    assert json.loads(lines[0].removeprefix("node_info=")) == {
        "inputs": [],
        "outputs": [{"name": "out", "schema_hash": 15293913555552287199,
                     "max_slice_len_default": 64, "promise_within_ms": None,
                     "wire_fixed_size": 24}],
        "policy": {"period_ms": 1},
    }
    assert lines[1:] == [
        "tick=0 code=0 out=000000000000f83f00000000000000c0000000000000d03f",
        "shutdown code=0",
    ]


def _node_env(name):
    env = os.environ.copy()
    env["CERULION_WORKSPACE"] = os.path.join(
        os.path.dirname(__file__), "..", "fixtures", "pynodes", name
    )
    env["CERULION_PY_PATH"] = os.pathsep.join(
        [sysconfig.get_path("purelib")]
    )
    libdir = sysconfig.get_config_var("LIBDIR")
    if libdir:
        var = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        env[var] = os.pathsep.join(value for value in (libdir, env.get(var)) if value)
    return env


def _run(name, case, ticks=1):
    path = os.path.join(PYNODE_DIR, "release", f"libcerulion_pynode_{name}{DYLIB}")
    env = _node_env(name)
    env["CERULION_PYNODE_CASE"] = case
    env.pop("CERULION_PYNODE_ABSENT_KEY", None)
    return subprocess.run(
        [FIXTURE, "host-pynode", path, str(ticks)],
        capture_output=True,
        text=True,
        env=env,
    )


def test_tick_exception_reports_traceback():
    result = _run("errors", "tick_exception")
    assert result.returncode == 0
    assert "tick=0 code=1 err=" in result.stdout
    assert "RuntimeError: fixture tick failure" in result.stdout
    assert "Traceback" in result.stdout


def test_loan_rejects_non_integer_variable_lengths():
    result = _run("errors", "loan_length_type")
    assert result.returncode == 0, result.stderr
    assert "tick=0 code=0" in result.stdout


def test_env_returns_none_for_an_absent_key_without_default():
    result = _run("errors", "env_lookup")
    assert result.returncode == 0, result.stderr
    assert "tick=0 code=0" in result.stdout


def test_import_failure_is_an_init_error():
    result = _run("errors", "import_error")
    assert result.returncode != 0
    assert "init failed: " in result.stderr
    assert "ImportError: fixture import failure" in result.stderr


def test_missing_tick_is_a_decorator_error():
    result = _run("errors", "missing_tick")
    assert result.returncode != 0
    assert "init failed: " in result.stderr
    assert "TypeError" in result.stderr


def test_retained_input_view_is_rejected_and_next_tick_survives():
    result = _run("errors", "retain_view", ticks=2)
    assert result.returncode == 0
    lines = [line for line in result.stdout.splitlines() if line.startswith("tick=")]
    assert lines[0].startswith(
        "tick=0 code=1 err=retained view of input 'inp' escaped tick()"
    )
    assert lines[1].startswith("tick=1 code=1 err=")


def test_retained_loan_export_is_rejected():
    result = _run("errors", "retain_loan", ticks=2)
    assert result.returncode == 0
    assert "tick=0 code=1 err=output loan 'out' retained" in result.stdout
    assert "tick=1 code=0 out=01000000" in result.stdout


def test_retained_tick_is_inert_after_tick_end():
    result = _run("errors", "retained_tick", ticks=2)
    assert result.returncode == 0
    assert "tick=1 code=1 err=" in result.stdout
    assert "tick is no longer active" in result.stdout


def test_retained_tick_loan_is_inert_after_failed_tick():
    result = _run("errors", "retained_tick_loan_after_error", ticks=3)
    assert result.returncode == 0
    assert "tick=1 code=1 err=" in result.stdout
    assert "fixture retained tick loan failure" in result.stdout
    assert "tick=2 code=1 err=" in result.stdout
    assert "tick is no longer active" in result.stdout


def test_retained_input_after_failed_tick_pins_view():
    result = _run("errors", "retain_then_raise")
    assert result.returncode == 0
    assert "RuntimeError: fixture retained input then raised" in result.stdout
    assert "input view retained across a failed tick" in result.stdout


def test_retaining_second_output_publishes_nothing():
    result = _run("errors", "retain_second_output")
    assert result.returncode == 0
    assert "output loan 'out2' retained" in result.stdout
    assert "out_frames=0" in result.stdout


def test_wrong_metadata_is_rejected_at_init():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_wrongmeta" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "1"],
        capture_output=True,
        text=True,
        env=_node_env("wrongmeta"),
    )
    assert result.returncode != 0
    assert (
        "node metadata is stale for output 'out': INFO schema_hash "
        "5093796653891464802, declaration 5093796653891464803; run `cerulion node build`"
        in result.stderr
    )


def test_two_node_types_share_one_process():
    # Both fixtures import a sibling `helpers.py` of their own inside tick();
    # the doubler's lines below are its own helper's arithmetic, so a host that
    # served the counter's cached `helpers` to the doubler would print 1 and 3.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_counter" + DYLIB)
    doubler = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_doubler" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--also", doubler],
        check=True,
        capture_output=True,
        text=True,
        env=_shared_node_env(),
    )
    ticks = [line for line in result.stdout.splitlines() if line.startswith("tick=")]
    assert ticks == [
        "tick=0 code=0 out=0100000000000000",
        "tick=1 code=0 out=0300000000000000",
        "tick=0 code=0 out=00000000",
        "tick=1 code=0 out=02000000",
    ]


def test_host_pynode_ticks_on_a_thread_other_than_the_initializing_one():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    env = _node_env("counter")
    env["CERULION_PYNODE_TICK_THREAD"] = "1"
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "2"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
    )
    assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000",
        "tick=1 code=0 out=0300000000000000",
    ]


def test_node_context_is_usable_from_a_tick_on_another_thread():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_errors" + DYLIB)
    env = _node_env("errors")
    env["CERULION_PYNODE_CASE"] = "ctx_in_tick"
    env["CERULION_PYNODE_TICK_THREAD"] = "1"
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "2"],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    ticks = [line for line in result.stdout.splitlines() if line.startswith("tick=")]
    assert [line.split(" out=")[0] for line in ticks] == ["tick=0 code=0", "tick=1 code=0"]


def test_extra_thread_warning_is_latched():
    result = _run("errors", "spawn_thread", ticks=3)
    assert result.returncode == 0
    assert result.stderr.count("additional threads") == 1


def test_stable_python_adapter_delivers_oracle_frames_and_host_sequences():
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    expected = [
        "tick=0 code=0 out=0100000000000000 seq=41",
        "tick=1 code=0 out=0300000000000000 seq=42",
        "tick=2 code=0 out=0500000000000000 seq=43",
    ]
    for _ in range(2):
        result = subprocess.run([FIXTURE, "host-pynode", counter, "3", "--seed", "41"],
                                capture_output=True, text=True, check=True, env=_node_env("counter"))
        metadata = _bootstrap_metadata("counter")
        assert metadata["inputs"][0]["schema_hash"] == 15583965953746584896
        assert metadata["inputs"][0]["wire_fixed_size"] == 8
        assert metadata["inputs"][0]["variable_count"] == 0
        assert metadata["inputs"][0]["alignment"] == 8
        assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == expected


class _Status(ctypes.Structure):
    _fields_ = [("error", ctypes.c_uint64), ("code", ctypes.c_uint32), ("kind", ctypes.c_uint32)]


class _TableHeader(ctypes.Structure):
    _fields_ = [("layout_id", ctypes.c_uint64), ("size_bytes", ctypes.c_uint32), ("major", ctypes.c_uint32)]


class _Bytes(ctypes.Structure):
    _fields_ = [("ptr", ctypes.c_void_p), ("len", ctypes.c_uint64)]


class _NodeApi(ctypes.Structure):
    _fields_ = [("header", _TableHeader), ("metadata", _Bytes),
                ("required", ctypes.c_uint64), ("capabilities", ctypes.c_uint64)] + [
                    (name, ctypes.c_void_p) for name in
                    ("create", "tick", "shutdown", "destroy", "copy_error", "query_extension", "state", "external")]


def _bootstrap_metadata(name, env=None):
    # A fresh process resolves workspace schemas before the metadata OnceLock
    # is initialized; the in-process ctypes tests deliberately have no workspace.
    path = os.path.join(PYNODE_DIR, "release", f"libcerulion_pynode_{name}{DYLIB}")
    script = """
import ctypes, runpy, sys
namespace = runpy.run_path(sys.argv[1])
library = ctypes.CDLL(sys.argv[2])
library.cerulion_node_api_v1.restype = ctypes.POINTER(namespace["_NodeApi"])
api = library.cerulion_node_api_v1().contents
assert api.header.major == 1
assert api.header.size_bytes == ctypes.sizeof(namespace["_NodeApi"])
print(ctypes.string_at(api.metadata.ptr, api.metadata.len).decode())
"""
    result = subprocess.run([sys.executable, "-c", script, __file__, path],
                            check=True, capture_output=True, text=True,
                            env=env or _node_env(name), timeout=60)
    return json.loads(result.stdout)


def _shared_node_env():
    env = _node_env("shared")
    for name, schema_hash, fixed_size in (
        ("shared_counter", 15583965953746584896, 8),
        ("shared_doubler", 9549963880961276720, 4),
        ("shared_errors", 9549963880961276720, 4),
    ):
        metadata = _bootstrap_metadata(name, env)
        for direction in ("inputs", "outputs"):
            for port in metadata[direction]:
                assert (port["schema_hash"], port["wire_fixed_size"],
                        port["variable_count"], port["alignment"]) == (
                            schema_hash, fixed_size, 0, 8
                        ), (name, direction, port)
    return env


def test_python_maintenance_exports_are_absent():
    library = _counter_library()
    for name in ("pump_history", "set_snapshot_inputs", "snapshot_inputs",
                 "drain_trigger_input", "refill_trigger_input", "sync_head_op"):
        assert not hasattr(library, f"cerulion_node_{name}")


def test_stable_python_bootstrap_refuses_null_host_and_stale_handles():
    library = _counter_library()
    library.cerulion_node_api_v1_stabbied_v3_report.restype = ctypes.c_void_p
    assert library.cerulion_node_api_v1_stabbied_v3_report()
    assert library.cerulion_node_api_v1_stabbied_v3
    library.cerulion_node_api_v1.restype = ctypes.POINTER(_NodeApi)
    pointer = library.cerulion_node_api_v1()
    assert pointer
    api = pointer.contents
    assert api.header.major == 1
    assert api.header.size_bytes == ctypes.sizeof(_NodeApi)
    assert api.header.layout_id != 0
    assert api.capabilities == api.required == 0
    assert api.state is None and api.external is None
    raw_metadata = ctypes.string_at(api.metadata.ptr, api.metadata.len)
    assert b"\0" not in raw_metadata
    assert json.loads(raw_metadata)["retain_process"] is True
    create = ctypes.CFUNCTYPE(_Status, ctypes.c_void_p, ctypes.c_uint64, _Bytes,
                              ctypes.POINTER(ctypes.c_uint64))(api.create)
    handle = ctypes.c_uint64(999)
    assert create(None, 1, _Bytes(None, 0), ctypes.byref(handle)).code == 5
    assert handle.value == 0
    for name in ("tick", "shutdown", "destroy"):
        operation = ctypes.CFUNCTYPE(_Status, ctypes.c_uint64)(getattr(api, name))
        assert operation(0).code == operation(999).code == 4


def _counter_library():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    library = ctypes.CDLL(path)
    library.cerulion_node_init.argtypes = [ctypes.c_void_p]
    library.cerulion_node_init.restype = ctypes.c_uint64
    for name in ("cerulion_node_tick", "cerulion_node_shutdown"):
        function = getattr(library, name)
        function.argtypes = [ctypes.c_uint64]
        function.restype = ctypes.c_int
    library.cerulion_take_last_error.restype = ctypes.c_void_p
    library.cerulion_free_error.argtypes = [ctypes.c_void_p]
    return library


def _last_error(library):
    pointer = library.cerulion_take_last_error()
    assert pointer
    try:
        return ctypes.string_at(pointer).decode()
    finally:
        library.cerulion_free_error(pointer)


def test_counter_abi_error_arms_report_stable_codes_and_errors():
    library = _counter_library()
    assert library.cerulion_node_tick(999) == 4
    assert _last_error(library).startswith("handle 999 not found")
    assert library.cerulion_node_init(None) == 0
    assert _last_error(library).startswith("cerulion_node_init: NodeContext pointer was null")
    assert library.cerulion_node_shutdown(999) == 4
    assert _last_error(library).startswith("handle 999 not found")


def test_shutdown_exception_reports_error_code():
    result = _run("errors", "shutdown_exception")
    assert result.returncode != 0
    assert "shutdown code=1 err=" in result.stdout
    assert "RuntimeError: fixture shutdown failure" in result.stdout


def test_interleaved_node_types_each_import_their_own_helpers():
    # Ticks alternate between the two node types, as a single-process graph
    # schedules them, and each tick imports `helpers` afresh: the counter's
    # tick after the doubler's must still get the counter's helper (2n+1), the
    # doubler's after the counter's its own (2n).
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_counter" + DYLIB)
    doubler = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_doubler" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--also", doubler, "--interleave"],
        check=True,
        capture_output=True,
        text=True,
        env=_shared_node_env(),
    )
    assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000",
        "tick=0 code=0 out=00000000",
        "tick=1 code=0 out=0300000000000000",
        "tick=1 code=0 out=02000000",
    ]


def test_shutdown_runs_with_the_node_types_own_helpers():
    # Shutdown comes after every node has ticked, so the node type that ticked
    # last is the one whose helpers are in `sys.modules`. Each fixture's
    # `shutdown()` imports `helpers` and raises unless it is the very module
    # object its ticks used: the other type's helper, or a fresh copy of its
    # own without its state, fails the run. Interleaved, the doubler ticks
    # last and the counter shuts down first. The harness prints one
    # `shutdown` line per node, so two `code=0` lines prove both hooks ran
    # and returned.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_counter" + DYLIB)
    doubler = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_doubler" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--also", doubler, "--interleave"],
        capture_output=True,
        text=True,
        env=_shared_node_env(),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert [line for line in lines if line.startswith("shutdown ")] == [
        "shutdown code=0",
        "shutdown code=0",
    ], result.stdout
    assert [line for line in lines if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000",
        "tick=0 code=0 out=00000000",
        "tick=1 code=0 out=0300000000000000",
        "tick=1 code=0 out=02000000",
    ]


def test_variable_length_output_is_written_through_element_counts():
    # `self.loan("out", name=5, samples=3)` reserves five characters and three
    # doubles. Hand-written oracle for the `Samples` body: two offset-table
    # entries (offset, length) at 0..16, "laser" at 16 padded to the 8-byte
    # boundary, then 1.5, -2.0 and 0.25 as little-endian doubles at 24..48.
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_varlen" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "1"],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("varlen"),
    )
    lines = result.stdout.splitlines()
    assert '"schema_hash":3245531966109359144' in lines[0]
    assert lines[1:] == [
        "tick=0 code=0 out="
        "1000000005000000"
        "1800000018000000"
        "6c61736572000000"
        "000000000000f83f"
        "00000000000000c0"
        "000000000000d03f",
        "shutdown code=0",
    ]


def test_host_snapshots_hold_the_last_input_on_quiet_ticks():
    # The scheduler freezes a periodic node's inputs at every step boundary
    # through host-owned snapshots; with one input frame and three ticks the
    # node reads that frame on every tick. Without the freeze (the control
    # run) the quiet ticks read None and publish nothing.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    env = _node_env("counter")
    held = subprocess.run(
        [FIXTURE, "host-pynode", counter, "3", "--snapshot", "--input-ticks", "1"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    assert [line for line in held.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000",
        "tick=1 code=0 out=0100000000000000",
        "tick=2 code=0 out=0100000000000000",
    ]
    live = subprocess.run(
        [FIXTURE, "host-pynode", counter, "3", "--input-ticks", "1"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    assert [line for line in live.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=0100000000000000",
        "tick=1 code=0 out=",
        "tick=2 code=0 out=",
    ]


def test_host_sync_heads_fill_and_probe_each_input():
    # Host-owned trigger drain and Sync head operations inspect each port:
    # a boundary fill on a tick with a new frame answers Head and the
    # tick reads that frame; a quiet tick answers Nothing and reads nothing.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--sync-probe", "--input-ticks", "1"],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("counter"),
    )
    assert result.stdout.splitlines()[1:] == [
        "sync_head_ops=true unified_drain=true",
        "sync=inp fill=Head probe=Nothing",
        "tick=0 code=0 out=0100000000000000",
        "sync=inp fill=Nothing probe=Nothing",
        "tick=1 code=0 out=",
        "shutdown code=0",
    ]


def test_two_instances_of_one_node_type_share_its_helpers():
    # The counter imports `helpers` in init() and keeps it; its ticks and its
    # shutdown raise unless the import returns that very module. A second
    # instance of the same type (the same cdylib loaded for a second node)
    # must not evict the first instance's helpers as a rebuilt node would:
    # both instances run on the one module the running type already uses.
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--also", counter, "--interleave"],
        capture_output=True,
        text=True,
        env=_node_env("counter"),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    ticks = [line for line in lines if line.startswith("tick=")]
    # Every tick ran on the shared helpers (a reloaded one raises, code=1).
    # Both instances share the harness topics, so only the first tick's
    # value is pinned: later ticks read whichever instance's feed won.
    assert len(ticks) == 4 and all(" code=0 " in line for line in ticks), result.stdout
    assert ticks[:2] == ["tick=0 code=0 out=0100000000000000"] * 2, result.stdout
    assert [line for line in lines if line.startswith("shutdown ")] == [
        "shutdown code=0",
        "shutdown code=0",
    ], result.stdout


def test_a_cached_foreign_module_does_not_shadow_a_node_types_sibling(tmp_path):
    # The errors fixture ships no helpers.py: with CERULION_PY_PATH pointing at
    # a directory that has one, its init() caches a FOREIGN `helpers` in
    # `sys.modules`, which the import system consults before any finder. The
    # counter, loaded next, ships its own helpers.py (2n+1): served the cached
    # foreign module it would publish 1000n. Interleaved, the errors node then
    # gets its foreign module back on every one of its ticks.
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    (foreign / "helpers.py").write_text("def transform(value):\n    return value * 1000\n")
    errors = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_errors" + DYLIB)
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_shared_counter" + DYLIB)
    env = _shared_node_env()
    env["CERULION_PYNODE_CASE"] = "import_foreign_helpers"
    env["CERULION_PY_PATH"] = os.pathsep.join([str(foreign), env["CERULION_PY_PATH"]])
    result = subprocess.run(
        [FIXTURE, "host-pynode", errors, "2", "--also", counter, "--interleave"],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    ticks = [line for line in lines if line.startswith("tick=")]
    assert len(ticks) == 4 and all(" code=0 " in line for line in ticks), result.stdout
    assert ticks[1] == "tick=0 code=0 out=0100000000000000", result.stdout
    assert ticks[3] == "tick=1 code=0 out=0300000000000000", result.stdout
    assert [line for line in lines if line.startswith("shutdown ")] == [
        "shutdown code=0",
        "shutdown code=0",
    ], result.stdout


def test_failed_init_invalidates_a_retained_context():
    # The first load keeps its context handle and fails init; the second load
    # calls through that handle and must get "no longer alive", then run.
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_errors" + DYLIB)
    env = _node_env("errors")
    env["CERULION_PYNODE_CASE"] = "leak_ctx_then_fail"
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "1", "--also", path],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode != 0
    # One init failed (the loader's wrapper repeats the phrase inside the
    # message, so count report lines, not occurrences): the first load's.
    failures = [line for line in result.stderr.splitlines() if line.startswith("init failed: ")]
    assert len(failures) == 1, result.stderr
    assert "RuntimeError: fixture init failure" in result.stderr
    assert "stale context answered" not in result.stderr
    assert "tick=0 code=0 out=00000000" in result.stdout, result.stdout


def test_escaped_python_memoryview_survives_destroy_and_never_reuses_its_lease():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_errors" + DYLIB)
    env = _node_env("errors")
    env["CERULION_PYNODE_CASE"] = "retain_after_destroy"
    result = subprocess.run([FIXTURE, "host-pynode-pinned", path],
                            capture_output=True, text=True,
                            env=env, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "pinned after_destroy=1 outstanding=1 refused=9" in result.stdout
    assert "pinned after_unload=1 bytes=d4c3b2a1 outstanding=1" in result.stdout


def test_python_node_error_reason_crosses_the_stable_slot_exactly():
    result = _run("errors", "retain_loan", ticks=1)
    assert result.returncode == 0, result.stderr
    assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=1 err=output loan 'out' retained a buffer export at tick end out_frames=0"
    ]


def test_baked_node_directory_finds_workspace_schemas_without_the_variable():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_errors" + DYLIB)
    env = _node_env("errors")
    env.pop("CERULION_WORKSPACE", None)
    env["CERULION_PYNODE_CASE"] = "workspace_fallback"
    for _ in range(2):
        result = subprocess.run([FIXTURE, "host-pynode", path, "2"],
                                capture_output=True, text=True, env=env, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
        assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
            "tick=0 code=0 out=00000000", "tick=1 code=0 out=01000000"
        ]
        assert "panicked" not in result.stderr


def test_stale_metadata_uses_the_init_diagnostic_without_a_bootstrap_panic():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_wrongmeta" + DYLIB)
    result = subprocess.run([FIXTURE, "host-pynode", path, "1"],
                            capture_output=True, text=True, env=_node_env("wrongmeta"), timeout=60)
    assert result.returncode != 0
    assert ("node metadata is stale for output 'out': INFO schema_hash "
            "5093796653891464802, declaration 5093796653891464803; run `cerulion node build`"
            in result.stderr)
    assert "panicked" not in result.stderr


def test_unwired_python_outputs_allow_init_and_two_graph_ticks():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_errors" + DYLIB)
    env = _node_env("errors")
    env["CERULION_PYNODE_CASE"] = "unwired_output"
    result = subprocess.run([FIXTURE, "host-pynode", path, "2", "--unwired-outputs"],
                            capture_output=True, text=True, env=env, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "unwired publisher_none=1" in result.stdout
    assert [line for line in result.stdout.splitlines() if line.startswith("tick=")] == [
        "tick=0 code=0 out=", "tick=1 code=0 out="
    ]
    assert "shutdown code=0" in result.stdout
