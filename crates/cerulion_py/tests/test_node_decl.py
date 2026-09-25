import json
import ctypes
import importlib.util
import os
import re
import sys
from pathlib import Path

import pytest

import cerulion

DYLIB = ".dylib" if sys.platform == "darwin" else ".so"


def _schema_set():
    schemas = cerulion.SchemaSet()
    assert schemas.add_yaml(
        """\
schemas:
  Probe:
    fields:
      uint32 value: {}
"""
    ) == []
    return schemas


def test_declarations_preserve_port_order_and_emit_compact_metadata():
    schemas = _schema_set()

    @cerulion.node(period_ms=10)
    class Counter:
        first = cerulion.input("Probe", depth=1)
        result = cerulion.output("Probe")

        def tick(self):
            pass

    document = json.loads(Counter.__cerulion_info__(schemas))
    assert [port["name"] for port in document["inputs"]] == ["first"]
    assert [port["name"] for port in document["outputs"]] == ["result"]
    assert document["policy"] == {"period_ms": 10}
    assert document["inputs"][0]["depth"] == 1
    assert document["outputs"][0]["promise_within_ms"] is None


def test_two_input_one_output_metadata_matches_handwritten_oracle():
    schemas = _schema_set()

    @cerulion.node(period_ms=25)
    class Mixer:
        left = cerulion.input("Probe", depth=1, backpressure="drop_oldest")
        right = cerulion.input("Probe", depth=2, backpressure="block")
        result = cerulion.output("Probe")

        def tick(self):
            self.result.value = self.left.value + self.right.value

    assert json.loads(Mixer.__cerulion_info__(schemas)) == {
        "inputs": [
            {
                "name": "left",
                "schema": "Probe",
                "schema_hash": 5093796653891464803,
                "depth": 1,
                "backpressure": "drop_oldest",
            },
            {
                "name": "right",
                "schema": "Probe",
                "schema_hash": 5093796653891464803,
                "depth": 2,
                "backpressure": "block",
            },
        ],
        "outputs": [
            {
                "name": "result",
                "schema": "Probe",
                "schema_hash": 5093796653891464803,
                "max_slice_len_default": 36,
                "promise_within_ms": None,
                "wire_fixed_size": 4,
            }
        ],
        "policy": {"period_ms": 25},
    }


@pytest.mark.parametrize("kwargs", [{"period_ms": 1, "trigger": "x"}, {"period_ms": 1, "sync_window_ms": 2}])
def test_invalid_policy_combination_is_rejected(kwargs):
    with pytest.raises((TypeError, ValueError)):
        decorator = cerulion.node(**kwargs)
        @decorator
        class Invalid:
            def tick(self):
                pass


def test_no_policy_without_trigger_is_rejected():
    with pytest.raises(
        TypeError,
        match=(
            r"^no trigger policy: mark an input with trigger=True, or specify "
            r"period_ms or sync_window_ms$"
        ),
    ):

        @cerulion.node()
        class External:
            inp = cerulion.input("Probe")

            def tick(self):
                pass


def test_single_trigger_input_infers_data_policy():
    @cerulion.node()
    class Triggered:
        inp = cerulion.input("Probe", trigger=True)

        def tick(self):
            pass

    assert Triggered.__cerulion_policy__ == {
        "data_trigger": {"input_name": "inp"}
    }


def test_multiple_trigger_inputs_require_sync_policy():
    with pytest.raises(
        TypeError, match=r"^multiple trigger inputs require sync_window_ms$"
    ):

        @cerulion.node()
        class Synchronized:
            left = cerulion.input("Probe", trigger=True)
            right = cerulion.input("Probe", trigger=True)

            def tick(self):
                pass


def test_missing_tick_and_reserved_host_methods_are_rejected():
    with pytest.raises(TypeError):

        @cerulion.node(period_ms=1)
        class Missing:
            pass

    with pytest.raises(TypeError):

        @cerulion.node(period_ms=1)
        class Reserved:
            def tick(self):
                pass

            def now_ns(self):
                pass


@pytest.mark.parametrize("depth", [0, 65, True, "1"])
def test_depth_validation(depth):
    with pytest.raises(ValueError):
        cerulion.input("Probe", depth=depth)


def test_depth_64_is_accepted():
    port = cerulion.input("Probe", depth=64)

    @cerulion.node(period_ms=1)
    class Deep:
        inp = port

        def tick(self):
            pass

    assert json.loads(Deep.__cerulion_info__(_schema_set()))["inputs"][0]["depth"] == 64


def test_runtime_reserved_port_names_are_rejected():
    for name in ("_cer_ctx", "__cerulion_ports__"):
        with pytest.raises(TypeError, match=rf"reserved port name: {name}"):
            cerulion.node(period_ms=1)(
                type(
                    "Reserved",
                    (),
                    {name: cerulion.input("Probe"), "tick": lambda self: None},
                )
            )

    cerulion.node(period_ms=1)(
        type(
            "Normal",
            (),
            {"normal": cerulion.input("Probe"), "tick": lambda self: None},
        )
    )


@pytest.mark.parametrize(
    ("factory", "argument", "name"),
    [
        (cerulion.node, "period_ms", "period_ms"),
        (cerulion.node, "sync_window_ms", "sync_window_ms"),
        (cerulion.node, "tick_within_ms", "tick_within_ms"),
        (cerulion.node, "throttle_ms", "throttle_ms"),
        (cerulion.input, "expect_within_ms", "expect_within_ms"),
        (cerulion.output, "promise_within_ms", "promise_within_ms"),
    ],
)
@pytest.mark.parametrize("value", [0, -1, True, 1.5])
def test_timing_values_must_be_positive_integers(factory, argument, name, value):
    with pytest.raises(ValueError, match=f"{name} must be a positive integer"):
        factory("Probe", **{argument: value}) if factory is not cerulion.node else factory(**{argument: value})


def test_timing_values_round_trip():
    schemas = _schema_set()

    @cerulion.node(period_ms=1, tick_within_ms=2, throttle_ms=3)
    class Timed:
        inp = cerulion.input("Probe", expect_within_ms=4)
        out = cerulion.output("Probe", promise_within_ms=5)

        def tick(self):
            pass

    document = json.loads(Timed.__cerulion_info__(schemas))
    assert document["inputs"][0]["expect_within_ms"] == 4
    assert document["outputs"][0]["promise_within_ms"] == 5
    assert document["policy"] == {"period_ms": 1}
    assert document["tick_within_ms"] == 2
    assert document["throttle_ms"] == 3


def test_backpressure_validation():
    with pytest.raises(ValueError):
        cerulion.input("Probe", backpressure="unknown")


def test_workspace_fallback_requires_explicit_schema_set(monkeypatch):
    monkeypatch.delenv("CERULION_WORKSPACE", raising=False)

    @cerulion.node(period_ms=1)
    class NoWorkspace:
        inp = cerulion.input("Probe")

        def tick(self):
            pass

    with pytest.raises(RuntimeError, match="pass a SchemaSet"):
        NoWorkspace.__cerulion_info__()


def test_rust_period_node_metadata_matches_python_declaration():
    target_dir = os.environ.get("CERULION_ROOT_TARGET_DIR")
    if not target_dir:
        pytest.skip("set CERULION_ROOT_TARGET_DIR to run Rust/Python metadata parity")
    candidates = list(Path(target_dir).rglob("libtest_node_macro_period_cdylib" + DYLIB))
    if not candidates:
        pytest.skip(
            "libtest_node_macro_period_cdylib" + DYLIB + " not found; build it with "
            "cargo build -p test_node_macro_period_cdylib"
        )

    library = ctypes.CDLL(str(candidates[0]))
    library.cerulion_node_info.restype = ctypes.c_char_p
    rust_info = json.loads(library.cerulion_node_info().decode("utf-8"))

    schemas = cerulion.SchemaSet()
    assert schemas.add_rosmsg(
        "float64 x\nfloat64 y\nfloat64 z\n", "geometry_msgs/Vector3"
    ) == []

    @cerulion.node(period_ms=50)
    class PeriodNode:
        cmd = cerulion.output(
            "geometry_msgs/Vector3", max_slice_len_default=56
        )

        def tick(self):
            pass

    python_info = json.loads(PeriodNode.__cerulion_info__(schemas))
    assert python_info["outputs"][0].pop("schema") == "geometry_msgs/Vector3"
    assert rust_info == python_info


def test_fixture_info_bytes_match_python_declarations(monkeypatch):
    root = Path(__file__).parents[1] / "fixtures" / "pynodes"
    for name in ("counter", "doubler", "errors", "wrongmeta"):
        workspace = root / name
        monkeypatch.setenv("CERULION_WORKSPACE", str(workspace))
        source = (workspace / "src/lib.rs").read_text()
        match = re.search(r'static INFO_BYTES: &\[u8\] = b"((?:\\.|[^"])*)\\0";', source)
        assert match, name
        info_text = match.group(1).replace(r"\"", '"').replace(r"\\", "\\")
        info = json.loads(info_text)
        marker = re.search(r"^// CERULION:PORT_SCHEMAS (.+)$", source, re.MULTILINE)
        assert marker, name
        port_schemas = json.loads(marker.group(1))
        for section in ("inputs", "outputs"):
            for port in info[section]:
                port["schema"] = port_schemas[section][port["name"]]

        spec = importlib.util.spec_from_file_location(
            f"fixture_node_{name}", workspace / "node.py"
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        classes = [
            value
            for value in vars(module).values()
            if isinstance(value, type) and hasattr(value, "__cerulion_info__")
        ]
        assert len(classes) == 1
        declared = json.loads(classes[0].__cerulion_info__())
        if name == "wrongmeta":
            stale_hash = info["outputs"][0]["schema_hash"]
            info["outputs"][0]["schema_hash"] = declared["outputs"][0]["schema_hash"]
            assert info == declared
            assert stale_hash != declared["outputs"][0]["schema_hash"]
        else:
            assert info == declared
