import ctypes
import os
import subprocess
import sys
import sysconfig

import pytest

DYLIB = ".dylib" if sys.platform == "darwin" else ".so"


FIXTURE = os.environ.get("CERULION_PY_FIXTURE")
PYNODE_DIR = os.environ.get("CERULION_PYNODE_DIR")


pytestmark = pytest.mark.skipif(
    not FIXTURE or not PYNODE_DIR,
    reason="set CERULION_PY_FIXTURE and CERULION_PYNODE_DIR to run embedded pynode fixtures",
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


def test_host_pynode_loans_builtin_output_without_workspace_schemas():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_builtin" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", path, "1"],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("builtin"),
    )
    assert result.stdout.splitlines() == [
        "node_info={\"inputs\":[],\"outputs\":[{\"name\":\"out\",\"schema_hash\":15293913555552287199,\"max_slice_len_default\":56,\"promise_within_ms\":null,\"wire_fixed_size\":24}],\"policy\":{\"period_ms\":1}}",
        "tick=0 code=0 out=000000000000f83f00000000000000c0000000000000d03f",
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
    counter = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    doubler = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_doubler" + DYLIB)
    result = subprocess.run(
        [FIXTURE, "host-pynode", counter, "2", "--also", doubler],
        check=True,
        capture_output=True,
        text=True,
        env=_node_env("counter"),
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


def test_extra_thread_warning_is_latched():
    result = _run("errors", "spawn_thread", ticks=3)
    assert result.returncode == 0
    assert result.stderr.count("additional threads") == 1


def _counter_library():
    path = os.path.join(PYNODE_DIR, "release", "libcerulion_pynode_counter" + DYLIB)
    library = ctypes.CDLL(path)
    library.cerulion_node_init.argtypes = [ctypes.c_void_p]
    library.cerulion_node_init.restype = ctypes.c_uint64
    for name in ("cerulion_node_tick", "cerulion_node_pump_history", "cerulion_node_shutdown"):
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
    assert library.cerulion_node_pump_history(999) == 4
    assert _last_error(library).startswith("handle 999 not found")


def test_shutdown_exception_reports_error_code():
    result = _run("errors", "shutdown_exception")
    assert result.returncode != 0
    assert "shutdown code=1 err=" in result.stdout
    assert "RuntimeError: fixture shutdown failure" in result.stdout
