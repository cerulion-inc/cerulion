"""`cerulion.msgs`: message classes resolved from one message set, and the
class-typed API that takes them."""

import importlib.util
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

import cerulion
from cerulion.msgs import _stubs
from cerulion.msgs.geometry_msgs import Point, Pose, Vector3

# The wire hash of geometry_msgs/Vector3, as the Rust message crate stamps it
# (pinned by the node host fixtures and the CLI e2e).
VECTOR3_HASH = 15293913555552287199

MSGS_DIR = Path(cerulion.msgs.__file__).parent

THING_MSG = "float64 x\nstring label\nuint8[] blob\ngeometry_msgs/Point p\n"
READING_YAML = """\
schemas:
  Reading:
    fields:
      "float64 value":
"""
OVERRIDE_YAML = """\
schemas:
  geometry_msgs/Vector3:
    fields:
      "float32 x":
"""


def _workspace(root, *, override=False):
    """A minimal workspace the CLI's discovery rule finds: a `[workspace]`
    manifest beside `graphs/`, a `.msg` store package and a YAML file."""
    (root / "graphs").mkdir(parents=True)
    (root / "Cargo.toml").write_text('[workspace]\nmembers = ["nodes/*"]\n')
    store = root / "schemas" / "demo_pkg" / "msg"
    store.mkdir(parents=True)
    (store / "Thing.msg").write_text(THING_MSG)
    (root / "schemas" / "reading.yaml").write_text(READING_YAML)
    if override:
        (root / "schemas" / "vector3.yaml").write_text(OVERRIDE_YAML)
    return root


def _python(code, *, cwd, env_extra=None):
    env = os.environ.copy()
    env.pop("CERULION_WORKSPACE", None)
    env.update(env_extra or {})
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(code)],
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


def test_a_builtin_class_names_its_schema_and_hash():
    assert Vector3.__module__ == "cerulion.msgs.geometry_msgs"
    assert Vector3.__qualname__ == "Vector3"
    assert Vector3.__cerulion_schema__ == "geometry_msgs/Vector3"
    assert Vector3.__cerulion_schema_hash__ == VECTOR3_HASH
    assert issubclass(Vector3, cerulion.Message)
    assert cerulion.msgs.geometry_msgs.Vector3 is Vector3


def test_a_message_class_cannot_be_called():
    with pytest.raises(TypeError, match="Vector3 is a message type"):
        Vector3()


def test_an_unknown_package_or_message_is_an_import_error():
    with pytest.raises(ImportError):
        from cerulion.msgs.no_such_msgs import Thing  # noqa: F401
    with pytest.raises(ImportError):
        from cerulion.msgs.geometry_msgs import NoSuchType  # noqa: F401
    with pytest.raises(AttributeError):
        cerulion.msgs.NoSuchRootType


def test_views_are_instances_of_their_class_nested_ones_too():
    session = cerulion.connect()
    publisher = session.publisher("/cerulion_py/msgs/pose", schema=Pose)
    subscriber = session.subscriber("/cerulion_py/msgs/pose", schema=Pose)
    publisher.publish(
        {"position": {"x": 1.0, "y": 2.0, "z": 3.0},
         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
    )
    frame = subscriber.receive(1000)
    pose = frame.view()
    assert type(pose) is Pose
    assert type(pose.position) is Point
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, 2.0, 3.0)
    assert type(frame.view(Pose)) is Pose
    frame.release()
    with publisher.loan() as loan:
        assert type(loan) is Pose
        loan.position.x = 4.0
    frame = subscriber.receive(1000)
    assert frame.view().position.x == 4.0
    frame.release()


def test_a_set_hands_out_one_class_per_name_until_it_changes():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Probe:\n    fields:\n      uint32 value: {}\n")
    Probe = schemas.message_type("Probe")
    assert schemas.message_type("Probe") is Probe
    assert Probe.__module__ == "cerulion"
    assert Probe.__cerulion_schemas__ is schemas
    with pytest.raises(cerulion.SchemaError):
        schemas.message_type("Missing")
    publisher = cerulion.connect().publisher("/cerulion_py/msgs/probe", schema=Probe)
    assert publisher.schema_hash == Probe.__cerulion_schema_hash__
    # Redefining Probe changes its hash: the class taken before is retired.
    schemas.add_yaml("schemas:\n  Probe:\n    fields:\n      uint64 value: {}\n")
    with pytest.raises(cerulion.SchemaMismatch, match="take the class from the set again"):
        cerulion.connect().publisher("/cerulion_py/msgs/probe2", schema=Probe)
    assert schemas.message_type("Probe") is not Probe


def test_the_schema_argument_takes_a_class_alone_or_a_name_with_its_set():
    session = cerulion.connect()
    other = cerulion.SchemaSet.builtins()
    with pytest.raises(TypeError, match="belongs to its own schema set"):
        session.subscriber("/cerulion_py/msgs/arg", schema=Vector3, schemas=other)
    with pytest.raises(TypeError, match="a schema name needs schemas="):
        session.subscriber("/cerulion_py/msgs/arg", schema="geometry_msgs/Vector3")
    with pytest.raises(TypeError, match="expected a message class"):
        session.subscriber("/cerulion_py/msgs/arg", schema=42)
    with pytest.raises(TypeError, match="schemas requires schema"):
        session.subscriber("/cerulion_py/msgs/arg", schemas=other)
    publisher = session.publisher("/cerulion_py/msgs/arg", schema=Vector3)
    subscriber = session.subscriber(
        "/cerulion_py/msgs/arg", schema="geometry_msgs/Vector3", schemas=other
    )
    publisher.publish({"x": 1.0, "y": 2.0, "z": 3.0})
    frame = subscriber.receive(1000)
    # A by-name view is an instance of ITS set's class, not cerulion.msgs'.
    by_name = frame.view()
    assert type(by_name) is other.message_type("geometry_msgs/Vector3")
    assert type(by_name) is not Vector3
    with pytest.raises(TypeError, match=r"view\(LaserScan\)"):
        frame.view(other)
    frame.release()


def test_a_port_takes_a_class_and_refuses_one_from_another_set():
    with pytest.raises(TypeError, match="takes a message class from cerulion.msgs"):
        cerulion.input(42)
    with pytest.raises(TypeError, match="takes a message class from cerulion.msgs"):
        cerulion.output(cerulion.SchemaSet)

    first = cerulion.SchemaSet.builtins()
    first.add_yaml("schemas:\n  Probe:\n    fields:\n      uint32 value: {}\n")
    second = cerulion.SchemaSet.builtins()
    second.add_yaml("schemas:\n  Probe:\n    fields:\n      int64 value: {}\n")
    Probe = first.message_type("Probe")

    @cerulion.node(period_ms=5)
    class Node:
        inp = cerulion.input(Probe)
        out = cerulion.output(Vector3)

        def tick(self):
            pass

    assert Node.inp.schema == "Probe" and Node.inp.msg_type is Probe
    info = json.loads(Node.__cerulion_info__(first))
    assert info["inputs"][0]["schema"] == "Probe"
    assert info["outputs"][0]["schema"] == "geometry_msgs/Vector3"
    assert info["outputs"][0]["schema_hash"] == VECTOR3_HASH
    with pytest.raises(cerulion.SchemaMismatch, match="port 'inp': Probe carries schema hash") as exc:
        Node.__cerulion_info__(second)
    assert exc.value.kind == "SchemaMismatch"

    @cerulion.node(period_ms=5)
    class Writer:
        out = cerulion.output(Probe)

        def tick(self):
            pass

    with pytest.raises(cerulion.SchemaMismatch, match="port 'out': Probe carries schema hash"):
        Writer.__cerulion_info__(second)


def test_a_workspace_joins_the_namespace_by_package(tmp_path):
    root = _workspace(tmp_path / "ws")
    expected = cerulion.SchemaSet.from_workspace(str(root))
    out = _python(
        """
        import json
        import cerulion.msgs
        from cerulion.msgs.demo_pkg import Thing
        from cerulion.msgs import Reading
        from cerulion.msgs.geometry_msgs import Vector3
        print(json.dumps({
            "thing": [Thing.__module__, Thing.__cerulion_schema__, Thing.__cerulion_schema_hash__],
            "reading": [Reading.__module__, Reading.__cerulion_schema__, Reading.__cerulion_schema_hash__],
            "vector3": Vector3.__cerulion_schema_hash__,
            "root": cerulion.msgs._workspace_root(),
        }))
        """,
        cwd=root / "graphs",
    )
    seen = json.loads(out)
    assert seen["thing"] == [
        "cerulion.msgs.demo_pkg",
        "demo_pkg/Thing",
        expected.schema_hash("demo_pkg/Thing"),
    ]
    assert seen["reading"] == ["cerulion.msgs", "Reading", expected.schema_hash("Reading")]
    assert seen["vector3"] == VECTOR3_HASH
    assert os.path.realpath(seen["root"]) == os.path.realpath(root)
    # The same workspace through the environment, from a directory outside it.
    out = _python(
        "from cerulion.msgs.demo_pkg import Thing; print(Thing.__cerulion_schema_hash__)",
        cwd=tmp_path,
        env_extra={"CERULION_WORKSPACE": str(root)},
    )
    assert int(out) == expected.schema_hash("demo_pkg/Thing")


def test_outside_a_workspace_only_the_builtins_are_there(tmp_path):
    out = _python(
        """
        import cerulion.msgs
        print(cerulion.msgs._workspace_root())
        try:
            import cerulion.msgs.demo_pkg
        except ImportError:
            print("no demo_pkg")
        """,
        cwd=tmp_path,
    )
    assert out == "None\nno demo_pkg\n"


def test_a_workspace_override_replaces_the_builtin_and_drops_its_dependents(tmp_path):
    root = _workspace(tmp_path / "ws", override=True)
    expected = cerulion.SchemaSet.from_workspace(str(root))
    out = _python(
        """
        from cerulion.msgs import geometry_msgs
        print(geometry_msgs.Vector3.__cerulion_schema_hash__)
        print(hasattr(geometry_msgs, "Vector3Stamped"), hasattr(geometry_msgs, "Point"))
        """,
        cwd=root,
    )
    vector3, flags = out.splitlines()
    assert int(vector3) == expected.schema_hash("geometry_msgs/Vector3") != VECTOR3_HASH
    assert flags == "False True"


def test_the_default_binding_refuses_a_second_workspace(tmp_path):
    first = _workspace(tmp_path / "a")
    second = _workspace(tmp_path / "b")
    out = _python(
        f"""
        import cerulion.msgs
        cerulion.msgs._bind_workspace({str(first)!r})
        cerulion.msgs._bind_workspace({str(first)!r})
        try:
            cerulion.msgs._bind_workspace({str(second)!r})
        except ImportError as error:
            print("refused", "cannot rebind" in str(error))
        """,
        cwd=tmp_path,
    )
    assert out == "refused True\n"


def test_two_node_types_keep_their_own_workspace_modules(tmp_path):
    first = _workspace(tmp_path / "a")
    second = _workspace(tmp_path / "b")
    (second / "schemas" / "demo_pkg" / "msg" / "Thing.msg").write_text("int8 only\n")
    out = _python(
        f"""
        import cerulion
        from cerulion._node import _activate_node_dir
        def load(root):
            _activate_node_dir(root + "/nodes/n", forget=False)
            cerulion.msgs._bind_workspace(root)
            from cerulion.msgs.demo_pkg import Thing
            from cerulion.msgs import demo_pkg
            assert demo_pkg.Thing is Thing
            return Thing
        a = load({str(first)!r})
        b = load({str(second)!r})
        again = load({str(first)!r})
        print(a is again, a is not b, a.__cerulion_schema_hash__ != b.__cerulion_schema_hash__)
        """,
        cwd=tmp_path,
    )
    assert out == "True True True\n"


def test_the_wheel_carries_the_generated_builtin_package_files():
    expected = _stubs.builtin_files()
    shipped = {
        path.name: path.read_text()
        for path in MSGS_DIR.iterdir()
        if path.suffix in (".py", ".pyi") and not path.name.startswith(("_", "__main__"))
    }
    shipped["__init__.pyi"] = (MSGS_DIR / "__init__.pyi").read_text()
    regenerate = (
        "regenerate with: python -c \"from cerulion.msgs._stubs import write_builtin_stubs; "
        "write_builtin_stubs('crates/cerulion_py/python/cerulion/msgs')\""
    )
    assert sorted(shipped) == sorted(expected), regenerate
    for name, text in expected.items():
        assert shipped[name] == text, f"{name} is stale; {regenerate}"


def test_a_stub_types_each_field_as_the_view_returns_it(tmp_path):
    root = _workspace(tmp_path / "ws")
    schemas = cerulion.SchemaSet.from_workspace(str(root))
    schemas._class_module = "cerulion.msgs"
    assert _stubs.render_module(schemas, "demo_pkg") == (
        "# Generated by cerulion.msgs from the resolved message set. Do not edit.\n"
        "from typing import Any as _Any, Mapping as _Mapping\n"
        "\n"
        "import numpy as _np\n"
        "from numpy.typing import ArrayLike as _ArrayLike, NDArray as _NDArray\n"
        "\n"
        "from cerulion import Message as _Message, SchemaSet as _SchemaSet\n"
        "from cerulion.msgs import geometry_msgs as _m_geometry_msgs\n"
        "\n"
        "\n"
        "class Thing(_Message):\n"
        '    """The demo_pkg/Thing message."""\n'
        "    @property\n"
        "    def x(self) -> float: ...\n"
        "    @x.setter\n"
        "    def x(self, value: float) -> None: ...\n"
        "    @property\n"
        "    def p(self) -> _m_geometry_msgs.Point: ...\n"
        "    @property\n"
        "    def label(self) -> str: ...\n"
        "    @label.setter\n"
        "    def label(self, value: str | bytes) -> None: ...\n"
        "    @property\n"
        "    def blob(self) -> _NDArray[_np.uint8]: ...\n"
        "    @blob.setter\n"
        "    def blob(self, value: _ArrayLike) -> None: ...\n"
    )


def test_a_stub_skips_what_it_cannot_name_and_never_shadows_its_imports(tmp_path):
    schemas = cerulion.SchemaSet.builtins()
    schemas.add_yaml(
        "schemas:\n"
        "  odd/Message:\n    fields:\n      uint8 a: {}\n"
        "  odd/Any:\n    fields:\n      uint8 b: {}\n"
        "  odd/class:\n    fields:\n      uint8 a: {}\n"
        "  odd/Keys:\n    fields:\n      uint8 copy: {}\n      uint8 lambda: {}\n"
    )
    text = _stubs.render_module(schemas, "odd")
    assert "class Message(_Message):" in text
    assert "class Any(_Message):" in text
    assert "class class" not in text
    assert "# field 'copy': read and write it as message['copy']" in text
    assert "# field 'lambda': read and write it as message['lambda']" in text
    compile(text, "odd.pyi", "exec")


def test_workspace_stubs_hold_every_package(tmp_path):
    root = _workspace(tmp_path / "ws", override=True)
    schemas = cerulion.SchemaSet.from_workspace(str(root))
    schemas._class_module = "cerulion.msgs"
    out = tmp_path / "out"
    stale = out / "cerulion-stubs" / "msgs" / "gone_pkg.pyi"
    stale.parent.mkdir(parents=True)
    stale.write_text("")
    written = _stubs.write_workspace_stubs(str(out), schemas)
    base = out / "cerulion-stubs"
    names = {Path(path).relative_to(base).as_posix() for path in written}
    # Every built-in package is written, the ones the override leaves alone
    # (std_msgs) as the wheel's stub and the ones it changes (geometry_msgs,
    # sensor_msgs) as the workspace renders them.
    builtins = cerulion.SchemaSet.builtins()
    expected = {f"msgs/{module}.pyi" for module in _stubs._modules(builtins)}
    expected |= {"msgs/__init__.pyi", "msgs/demo_pkg.pyi", "py.typed"}
    assert names == expected
    std = (base / "msgs" / "std_msgs.pyi").read_text()
    assert std == (MSGS_DIR / "std_msgs.pyi").read_text()
    for module in ("geometry_msgs", "sensor_msgs"):
        assert _stubs.render_module(schemas, module) != _stubs.render_module(builtins, module)
    assert (base / "py.typed").read_text() == "partial\n"
    assert not stale.exists()
    init = (base / "msgs" / "__init__.pyi").read_text()
    assert "class Reading(_Message):" in init and "def _bind_workspace(" in init
    geometry = (base / "msgs" / "geometry_msgs.pyi").read_text()
    assert "class Vector3(_Message):" in geometry
    assert "class Vector3Stamped(" not in geometry


def test_pyright_types_builtin_and_workspace_imports_through_the_stubs(tmp_path):
    # The documented setup: `python -m cerulion.msgs`, then the stub
    # directory on pyright's extraPaths. A built-in import and a workspace
    # import both resolve to their classes.
    if importlib.util.find_spec("pyright") is None:
        pytest.skip("pyright is not installed")
    root = _workspace(tmp_path / "ws")
    env = {k: v for k, v in os.environ.items() if k != "CERULION_WORKSPACE"}
    made = subprocess.run(
        [sys.executable, "-m", "cerulion.msgs"], cwd=root, env=env, capture_output=True, text=True
    )
    assert made.returncode == 0, made.stderr
    (root / "pyrightconfig.json").write_text('{"extraPaths": [".cerulion/python"]}\n')
    (root / "check.py").write_text(
        "from cerulion.msgs.std_msgs import Int32\n"
        "from cerulion.msgs.demo_pkg import Thing\n"
        "\n"
        "def f(i: Int32, t: Thing) -> None:\n"
        "    reveal_type(i.data)\n"
        "    reveal_type(t.x)\n"
        "    reveal_type(t.label)\n"
    )
    result = subprocess.run(
        [sys.executable, "-m", "pyright", "--outputjson", "--pythonpath", sys.executable,
         "check.py"],
        cwd=root,
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )
    report = json.loads(result.stdout)
    diagnostics = report["generalDiagnostics"]
    errors = [d["message"] for d in diagnostics if d["severity"] == "error"]
    assert errors == [], errors
    revealed = [d["message"] for d in diagnostics if d["severity"] == "information"]
    assert revealed == [
        'Type of "i.data" is "int"',
        'Type of "t.x" is "float"',
        'Type of "t.label" is "str"',
    ], revealed


def test_python_dash_m_writes_the_workspace_stubs(tmp_path):
    root = _workspace(tmp_path / "ws")
    result = subprocess.run(
        [sys.executable, "-m", "cerulion.msgs"],
        cwd=root,
        env={k: v for k, v in os.environ.items() if k != "CERULION_WORKSPACE"},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    stub = root / ".cerulion" / "python" / "cerulion-stubs" / "msgs" / "demo_pkg.pyi"
    assert str(stub) in result.stdout.splitlines()
    assert "class Thing(_Message):" in stub.read_text()
    outside = subprocess.run(
        [sys.executable, "-m", "cerulion.msgs"],
        cwd=tmp_path,
        env={k: v for k, v in os.environ.items() if k != "CERULION_WORKSPACE"},
        capture_output=True,
        text=True,
    )
    assert outside.returncode == 2
    assert "no Cerulion workspace" in outside.stderr


def test_a_mutation_keeps_a_class_whose_hash_is_unchanged():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Point:\n    fields:\n      float64 x: {}\n")
    Point = schemas.message_type("Point")
    schemas.add_yaml("schemas:\n  Other:\n    fields:\n      uint8 a: {}\n")
    assert schemas.message_type("Point") is Point
    session = cerulion.connect()
    publisher = session.publisher("/cerulion_py/msgs/kept", schema=Point)
    subscriber = session.subscriber("/cerulion_py/msgs/kept", schema=Point)
    publisher.publish({"x": 2.5})
    frame = subscriber.receive(1000)
    assert isinstance(frame.view(), Point) and frame.view().x == 2.5
    frame.release()


def test_the_cerulion_msgs_set_is_read_only():
    with pytest.raises(TypeError, match="read-only"):
        Vector3.__cerulion_schemas__.add_yaml("schemas:\n  X:\n    fields:\n      uint8 a: {}\n")
    with pytest.raises(TypeError, match="read-only"):
        Vector3.__cerulion_schemas__.add_rosmsg("uint8 a", "X", "pkg")


def test_the_old_view_argument_order_is_refused_with_the_new_spelling():
    schemas = cerulion.SchemaSet.builtins()
    session = cerulion.connect()
    publisher = session.publisher("/cerulion_py/msgs/order", schema=Vector3)
    subscriber = session.subscriber("/cerulion_py/msgs/order")
    publisher.publish({"x": 1.0, "y": 2.0, "z": 3.0})
    frame = subscriber.receive(1000)
    with pytest.raises(TypeError, match=r"view\(LaserScan\).*view\('pkg/Name', schemas=schemas\)"):
        frame.view(schemas, "geometry_msgs/Vector3")
    record = cerulion._bag.Record(frame.raw)
    frame.release()
    with pytest.raises(TypeError, match=r"view\(LaserScan\).*view\('pkg/Name', schemas=schemas\)"):
        record.view(schemas, "geometry_msgs/Vector3")
    assert record.view(Vector3).z == 3.0


def test_a_rebuilt_node_reads_its_changed_workspace(tmp_path):
    root = _workspace(tmp_path / "ws")
    node_dir = str(root / "nodes" / "n")
    out = _python(
        f"""
        import os, time
        import cerulion
        from cerulion._node import _activate_node_dir, _IMPORTS
        def load():
            _activate_node_dir({node_dir!r})
            cerulion.msgs._bind_workspace({str(root)!r})
            from cerulion.msgs.demo_pkg import Thing
            return Thing
        first = load()
        path = {str(root / "schemas" / "demo_pkg" / "msg" / "Thing.msg")!r}
        with open(path, "w") as handle:
            handle.write("int32 only\\n")
        os.utime(path, ns=(time.time_ns() + 10**9, time.time_ns() + 10**9))
        _IMPORTS.forget({node_dir!r})
        second = load()
        print(first.__cerulion_schema_hash__)
        print(second.__cerulion_schema_hash__)
        print(second.__cerulion_schemas__.layout("demo_pkg/Thing").fixed_fields[0].name)
        """,
        cwd=tmp_path,
    )
    first, second, field = out.splitlines()
    assert int(first) != int(second)
    assert int(second) == cerulion.SchemaSet.from_workspace(str(root)).schema_hash("demo_pkg/Thing")
    assert field == "only"


def test_a_module_cached_outside_a_node_is_not_served_to_one(tmp_path):
    root = _workspace(tmp_path / "ws", override=True)
    out = _python(
        f"""
        import sys
        import cerulion
        import cerulion.msgs.geometry_msgs as outside
        from cerulion._node import _activate_node_dir, _IMPORTS
        _activate_node_dir({str(root / "nodes" / "n")!r})
        cerulion.msgs._bind_workspace({str(root)!r})
        from cerulion.msgs.geometry_msgs import Vector3
        print(Vector3.__cerulion_schema_hash__ != outside.Vector3.__cerulion_schema_hash__)
        print(hasattr(sys.modules["cerulion.msgs.geometry_msgs"], "Vector3Stamped"))
        _IMPORTS.forget({str(root / "nodes" / "n")!r})
        print(sys.modules["cerulion.msgs.geometry_msgs"] is outside)
        """,
        cwd=tmp_path,
    )
    assert out == "True\nFalse\nTrue\n"
