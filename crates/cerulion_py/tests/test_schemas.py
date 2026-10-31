import numpy as np
import pytest

import cerulion


SCHEMA = """\
schemas:
  Probe:
    fields:
      uint32 id: {}
      uint8 flag: {}
"""


def test_probe_layout_hash_and_dtype_cache():
    schemas = cerulion.SchemaSet()
    assert schemas.add_yaml(SCHEMA) == []
    layout = schemas.layout("Probe")

    assert layout.qualified_name == "Probe"
    assert [(f.name, f.offset, f.size) for f in layout.fixed_fields] == [
        ("id", 0, 4),
        ("flag", 4, 1),
    ]
    assert layout.fixed_size == 8
    assert layout.schema_hash == schemas.schema_hash("Probe")
    assert schemas.layout("Probe") is schemas.layout("Probe")
    assert layout.dtype is layout.dtype
    assert layout.dtype.itemsize == layout.fixed_size
    assert np.dtype(layout.dtype.fields["id"][0]) == np.dtype("<u4")


def test_padded_schema_uses_alignment_and_explicit_dtype_offsets():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        """\
schemas:
  Padded:
    fields:
      uint8 tag: {}
      uint64 value: {}
"""
    )
    layout = schemas.layout("Padded")
    assert layout.fixed_fields[0].offset == 0
    assert layout.fixed_fields[1].offset == 8
    assert layout.fixed_size == 16
    assert layout.dtype.fields["value"][1] == 8
    assert layout.dtype.itemsize == 16


def test_schema_cache_invalidates_and_names_preserve_declaration_order():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    old = schemas.layout("Probe")
    first_hash = schemas.schema_hash("Probe")
    schemas.add_yaml(
        """\
schemas:
  Later:
    fields:
      uint16 value: {}
"""
    )
    assert schemas.layout("Probe") is not old
    assert schemas.schema_hash("Probe") == first_hash
    assert schemas.names() == ["Probe", "Later"]


def test_workspace_and_rosmsg_loading(tmp_path):
    (tmp_path / "schemas").mkdir()
    (tmp_path / "schemas" / "base.yaml").write_text(SCHEMA)
    (tmp_path / "schemas" / "pkg" / "msg").mkdir(parents=True)
    (tmp_path / "schemas" / "pkg" / "msg" / "X.msg").write_text("uint32 id\n")
    schemas = cerulion.SchemaSet.from_workspace(tmp_path)
    (tmp_path / "empty").mkdir()
    builtin = cerulion.SchemaSet.from_workspace(tmp_path / "empty").names()
    assert len(builtin) == 254
    assert {
        "builtin_interfaces/Time",
        "geometry_msgs/Vector3",
        "sensor_msgs/LaserScan",
        "std_msgs/Header",
    } <= set(builtin)
    assert not {"Probe", "pkg/X"} & set(builtin)
    assert schemas.names() == builtin + ["Probe", "pkg/X"]
    assert schemas.layout("pkg/X").fixed_size == 4

    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg("uint32 id\n", "pkg/Msg")
    assert "pkg/Msg" in schemas.names()


@pytest.mark.parametrize(
    ("document", "kind"),
    [
        ("not: [yaml", "Yaml"),
        ("other: {}", "MissingSchemasKey"),
        ("schemas:\n  X:\n    fields:\n      nope: {}\n", "InvalidFieldKey"),
    ],
)
def test_schema_errors_are_typed_and_failed_additions_are_atomic(document, kind):
    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg("uint32 id\n", "pkg/Kept")
    before = schemas.names()
    assert "pkg/Kept" in before
    with pytest.raises(cerulion.SchemaError) as exc:
        schemas.add_yaml(document)
    assert exc.value.kind == kind
    assert schemas.names() == before


def test_unknown_schema_error_kind():
    schemas = cerulion.SchemaSet()
    with pytest.raises(cerulion.SchemaError) as exc:
        schemas.layout("Missing")
    assert exc.value.kind == "UnknownSchema"
