"""Schema-aware typed views over Cerulion wire frames."""

from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

from cerulion import _native


_SCALARS = {
    "Bool": ("?", 1),
    "I8": ("<i1", 1),
    "U8": ("<u1", 1),
    "I16": ("<i2", 2),
    "U16": ("<u2", 2),
    "I32": ("<i4", 4),
    "U32": ("<u4", 4),
    "I64": ("<i8", 8),
    "U64": ("<u8", 8),
    "F32": ("<f4", 4),
    "F64": ("<f8", 8),
}
_UNSET = object()


class _OwnedArray(np.ndarray):
    pass


def _owned_array(value, owner):
    result = np.asarray(value).view(_OwnedArray)
    result._cerulion_owner = owner
    return result


class _ScratchOwner:
    def _check_alive(self):
        return None


_SCRATCH_OWNER = _ScratchOwner()


@dataclass(frozen=True)
class FieldLayout:
    name: str
    offset: int
    size: int
    align: int
    field_type: object


@dataclass(frozen=True)
class VariableFieldLayout:
    name: str
    field_type: object


class SchemaSet:
    def __init__(self, native=None):
        self._native = native or _native.SchemaSet()
        self._generation = 0
        self._layouts = {}

    @classmethod
    def builtins(cls):
        """Create a schema set containing the vendored ROS 2 messages."""
        return cls(_native.SchemaSet.builtins())

    @classmethod
    def from_workspace(cls, path):
        """Load ROS 2 builtins plus workspace schemas, with workspace overrides."""
        return cls(_native.SchemaSet.from_workspace(path))

    def add_yaml(self, text):
        result = self._native.add_yaml(text)
        self._generation += 1
        self._layouts.clear()
        return result

    def add_rosmsg(self, text, name, package=None):
        result = self._native.add_rosmsg(text, name, package)
        self._generation += 1
        self._layouts.clear()
        return result

    def names(self):
        return self._native.names()

    def schema_hash(self, name):
        return self._native.schema_hash(name)

    def output_meta(self, name):
        return self._native.output_meta(name)

    @property
    def warnings(self):
        return self._native.warnings

    def layout(self, name):
        cached = self._layouts.get(name)
        if cached is not None and cached[0] == self._generation:
            return cached[1]
        raw = json.loads(self._native.layout_json(name))
        layout = Layout(self, raw)
        self._layouts[name] = (self._generation, layout)
        return layout


class Layout:
    def __init__(self, schemas, raw):
        self._schemas = schemas
        self._raw = raw
        self.qualified_name = raw["qualified_name"]
        self.schema_hash = raw["schema_hash"]
        self.fixed_size = raw["fixed_size"]
        self.fixed_align = raw["fixed_align"]
        self.fixed_fields = [
            FieldLayout(
                item["name"],
                item["offset"],
                item["size"],
                item["align"],
                _parse_type(item["field_type"]),
            )
            for item in raw["fixed_fields"]
        ]
        self.variable_fields = [
            VariableFieldLayout(item["name"], _parse_type(item["field_type"]))
            for item in raw["variable_fields"]
        ]
        self._fixed = {field.name: field for field in self.fixed_fields}
        self._variable = {field.name: field for field in self.variable_fields}
        self._dtype = _UNSET

    @property
    def dtype(self):
        if self.fixed_size == 0:
            return None
        if self._dtype is _UNSET:
            names, formats, offsets = [], [], []
            for field in self.fixed_fields:
                names.append(field.name)
                formats.append(_numpy_format(self, field.field_type))
                offsets.append(field.offset)
            try:
                dtype = np.dtype(
                    {
                        "names": names,
                        "formats": formats,
                        "offsets": offsets,
                        "itemsize": self.fixed_size,
                    }
                )
            except (TypeError, ValueError) as exc:
                raise _native.SchemaError(str(exc)) from exc
            if dtype.itemsize != self.fixed_size:
                raise _native.SchemaError(
                    f"layout {self.qualified_name} itemsize mismatch"
                )
            self._dtype = dtype
        return self._dtype

    def _resolve_nested(self, field_type):
        candidates = []
        if field_type["package"]:
            candidates.append(f"{field_type['package']}/{field_type['schema_name']}")
        package = self.qualified_name.rpartition("/")[0]
        if package:
            candidates.append(f"{package}/{field_type['schema_name']}")
        candidates.append(field_type["schema_name"])
        for candidate in candidates:
            if candidate not in self._schemas.names():
                continue
            target = self._schemas.layout(candidate)
            fixed = field_type.get("fixed")
            if fixed is not None and target.schema_hash != fixed["target_hash"]:
                continue
            return target
        raise _native.SchemaError(
            f"cannot resolve nested schema {field_type['schema_name']!r}"
        )


def _parse_type(value):
    if isinstance(value, str):
        return value
    if len(value) != 1:
        return value
    key, nested = next(iter(value.items()))
    if key in ("FixedArray", "DynamicArray"):
        return {key: {k: _parse_type(v) if k == "element_type" else v for k, v in nested.items()}}
    if key == "Nested":
        return {key: nested}
    return value


def _numpy_format(layout, field_type):
    if isinstance(field_type, str):
        if field_type not in _SCALARS:
            raise _native.SchemaError(f"unsupported fixed field type {field_type}")
        return _SCALARS[field_type][0]
    if "StringFixed" in field_type:
        return f"S{field_type['StringFixed']}"
    if "FixedArray" in field_type:
        item = field_type["FixedArray"]
        return (_numpy_format(layout, item["element_type"]), (item["length"],))
    if "Nested" in field_type:
        target = layout._resolve_nested(field_type["Nested"])
        if field_type["Nested"].get("fixed") is None or target.dtype is None:
            raise _native.SchemaError("nested field is not fixed-size")
        return target.dtype
    raise _native.SchemaError(f"unsupported fixed field type {field_type}")


def _element_dtype(field_type):
    if isinstance(field_type, str):
        try:
            return np.dtype(_SCALARS[field_type][0])
        except KeyError:
            return None
    return None


def _wire_length(field_type, count):
    if isinstance(field_type, dict) and "DynamicArray" in field_type:
        elem = field_type["DynamicArray"]["element_type"]
        if isinstance(elem, str) and elem in _SCALARS:
            return count * _SCALARS[elem][1]
    return count


def _dynamic_descriptor(field_type, offset, byte_len):
    """One descriptor for a variable DynamicArray/String/Bytes field.

    Scalar-element arrays view in place as ``("prim", dtype, offset,
    count)``; everything else (``string[]``, nested arrays, `bytes`,
    `string`) is the raw ``("raw", offset, byte_len)`` slice. `elem` may
    be a dict (nested element type) - `_SCALARS.get` must never see it.
    """
    if isinstance(field_type, dict) and "DynamicArray" in field_type:
        elem = field_type["DynamicArray"]["element_type"]
        scalar = _SCALARS.get(elem) if isinstance(elem, str) else None
        dtype = scalar[0] if scalar is not None else None
        if dtype is not None:
            return ("prim", dtype, offset, byte_len // np.dtype(dtype).itemsize)
    return ("raw", offset, byte_len)


def _value_wire_length(schemas, field_type, value, parent=None, name=None):
    if isinstance(field_type, str):
        if field_type in ("String", "Bytes"):
            return len(value.encode() if isinstance(value, str) else bytes(value))
    if isinstance(field_type, dict) and "DynamicArray" in field_type:
        elem = field_type["DynamicArray"]["element_type"]
        if isinstance(elem, str) and elem in _SCALARS:
            try:
                length = len(value)
            except TypeError as exc:
                raise _native.EncodeError(
                    "dynamic array value must be a sized sequence"
                ) from exc
            return length * _SCALARS[elem][1]
        # string[] / nested[] carry FRAMED elements; element-wise encoding
        # is not supported - the value must be pre-framed bytes.
        if isinstance(value, (bytes, bytearray, memoryview)):
            return len(bytes(value))
        raise _native.EncodeError(
            f"dynamic array field {name!r} of {elem} elements must be given "
            "as pre-framed bytes; element-wise encoding of "
            "string[]/nested[] is not supported"
        )
    if isinstance(field_type, dict) and "Nested" in field_type:
        if not isinstance(value, dict):
            return len(bytes(value))
        target = schemas.layout(_nested_name(schemas, field_type["Nested"], parent))
        return len(_encode_message(schemas, target.qualified_name, value, None)) - _native.WIRE_HEADER_SIZE
    raise _native.EncodeError("cannot determine variable field length")


def _nested_name(schemas, field_type, parent):
    package = field_type.get("package")
    candidates = []
    if package:
        candidates.append(f"{package}/{field_type['schema_name']}")
    if parent and "/" in parent:
        candidates.append(f"{parent.rpartition('/')[0]}/{field_type['schema_name']}")
    candidates.append(field_type["schema_name"])
    for candidate in candidates:
        if candidate in schemas.names():
            return candidate
    raise _native.SchemaError(f"unknown nested schema {field_type['schema_name']}")


def _encode_message(schemas, name, values, timestamp_ns):
    if isinstance(values, Message):
        values = values.copy()
    if not isinstance(values, dict):
        raise _native.EncodeError("typed publish expects a dict or Message")
    layout = schemas.layout(name)
    expected = {field.name for field in layout.fixed_fields + layout.variable_fields}
    missing = expected - set(values)
    extra = set(values) - expected
    if missing:
        raise _native.EncodeError(f"missing field(s): {', '.join(sorted(missing))}")
    if extra:
        raise _native.EncodeError(f"extra field(s): {', '.join(sorted(extra))}")
    encoded_values = {}
    var_lens = []
    for field in layout.variable_fields:
        value = values[field.name]
        if isinstance(field.field_type, dict) and "Nested" in field.field_type and isinstance(value, dict):
            target = schemas.layout(_nested_name(schemas, field.field_type["Nested"], layout.qualified_name))
            encoded = _encode_message(schemas, target.qualified_name, value, None)
            encoded_values[field.name] = bytes(encoded[_native.WIRE_HEADER_SIZE:])
            var_lens.append(len(encoded_values[field.name]))
        else:
            encoded_values[field.name] = value
            var_lens.append(
                _value_wire_length(
                    schemas,
                    field.field_type,
                    value,
                    layout.qualified_name,
                    field.name,
                )
            )
    frame = bytearray(
        schemas._native.begin_frame(name, var_lens, 0 if timestamp_ns is None else timestamp_ns)
    )
    body = memoryview(frame)[_native.WIRE_HEADER_SIZE:]
    descriptors = _descriptors_from_body(layout, body)
    message = Message(body, layout, schemas, descriptors)
    for field in layout.fixed_fields:
        _assign_value(message, field.name, values[field.name])
    for field in layout.variable_fields:
        _assign_value(message, field.name, encoded_values[field.name])
    return frame


def _assign_value(message, name, value):
    field = message._field(name)
    if isinstance(field, FieldLayout) and isinstance(field.field_type, dict) and "Nested" in field.field_type:
        nested = getattr(message, name)
        if not isinstance(value, dict):
            raise _native.EncodeError(f"nested field {name} expects a dict")
        for key, nested_value in value.items():
            setattr(nested, key, nested_value)
        return
    setattr(message, name, value)


def _descriptors_from_body(layout, body):
    result = {}
    table = layout.fixed_size
    for index, field in enumerate(layout.variable_fields):
        start = table + index * 8
        offset = int.from_bytes(body[start : start + 4], "little")
        length = int.from_bytes(body[start + 4 : start + 8], "little")
        result[field.name] = _dynamic_descriptor(field.field_type, offset, length)
    return result


class Message:
    def __init__(
        self, payload, layout, schemas, resolved_variables=None, owner=None, resolved_fields=None
    ):
        object.__setattr__(self, "_payload", payload)
        object.__setattr__(self, "_layout", layout)
        object.__setattr__(self, "_schemas", schemas)
        object.__setattr__(self, "_variables", resolved_variables or {})
        object.__setattr__(self, "_owner", owner or _SCRATCH_OWNER)
        object.__setattr__(self, "_resolved_fields", resolved_fields)
        writable = (
            not payload.readonly if isinstance(payload, memoryview) else isinstance(payload, bytearray)
        )
        object.__setattr__(self, "_writable", writable)
        object.__setattr__(self, "_rec", None)

    def _check_alive(self):
        owner = self._owner
        if owner is None:
            raise _native.ReleasedFrame("loan is closed")
        owner._check_alive()

    def _record(self):
        self._check_alive()
        if self._rec is None and self._layout.dtype is not None:
            object.__setattr__(
                self,
                "_rec",
                np.frombuffer(self._payload, dtype=self._layout.dtype, count=1)[0],
            )
        return self._rec

    def _field(self, name):
        if name in self._layout._fixed:
            return self._layout._fixed[name]
        if name in self._layout._variable:
            return self._layout._variable[name]
        raise AttributeError(name)

    def __getattr__(self, name):
        self._check_alive()
        if self._resolved_fields is not None and name in self._resolved_fields:
            value = self._resolved_fields[name]
            if isinstance(value, dict) and "nested" in value:
                value = value["nested"]
            if isinstance(value, dict) and "fields" in value:
                nested_type = self._field(name).field_type["Nested"]
                nested = self._layout._resolve_nested(nested_type)
                return Message(
                    self._payload,
                    nested,
                    self._schemas,
                    owner=self._owner,
                    resolved_fields=value["fields"],
                )
            if isinstance(value, (tuple, list)):
                # Walker-resolved variable fields carry descriptors
                # (("prim"/"raw") tuples, lists for string[]/nested[]),
                # not data - decode them like top-level variables.
                return self._variable_value(name, self._field(name), value)
            return value
        field = self._field(name)
        if isinstance(field, FieldLayout):
            value = self._record()[name]
            if isinstance(field.field_type, dict) and "StringFixed" in field.field_type:
                try:
                    return bytes(value).split(b"\0", 1)[0].decode()
                except UnicodeDecodeError as exc:
                    raise _native.DecodeError(
                        f"fixed string field {name!r}: invalid UTF-8"
                    ) from exc
            if isinstance(field.field_type, dict) and "Nested" in field.field_type:
                target = self._layout._resolve_nested(field.field_type["Nested"])
                return Message(
                    self._payload[field.offset : field.offset + field.size],
                    target,
                    self._schemas,
                    owner=self._owner,
                )
            if isinstance(value, np.ndarray):
                return _owned_array(value, self._owner)
            if isinstance(value, np.generic):
                return value.item()
            return value
        descriptor = self._variables.get(name)
        if descriptor is None:
            raise _native.DecodeError(f"missing resolved variable {name}")
        return self._variable_value(name, field, descriptor)

    def _variable_value(self, name, field, descriptor):
        field_type = field.field_type
        if isinstance(field_type, str) and field_type == "String":
            try:
                return bytes(
                    self._payload[_offset(descriptor) : _end(descriptor)]
                ).decode()
            except UnicodeDecodeError as exc:
                raise _native.DecodeError(
                    f"string field {name!r}: invalid UTF-8"
                ) from exc
        if isinstance(field_type, str) and field_type == "Bytes":
            return self._payload[_offset(descriptor) : _end(descriptor)]
        if isinstance(field_type, dict) and "DynamicArray" in field_type:
            elem = field_type["DynamicArray"]["element_type"]
            if isinstance(descriptor, tuple):
                if descriptor[0] == "prim":
                    return _owned_array(np.frombuffer(
                        self._payload[_offset(descriptor) : _offset(descriptor) + _byte_len(descriptor)],
                        dtype=np.dtype(descriptor[1]),
                        count=descriptor[3],
                    ), self._owner)
                # "raw": byte arrays (incl. bool[], one byte per element)
                # view in place; on a LOAN a string[]/nested[] field is
                # pre-framed bytes, so hand back the raw writable slice.
                if elem in ("I8", "U8", "Bool"):
                    return _owned_array(np.frombuffer(
                        self._payload[_offset(descriptor) : _end(descriptor)],
                        dtype=np.dtype(_SCALARS[elem][0]),
                    ), self._owner)
                # A raw string[]/nested[] slice is pre-framed (loan) or
                # opaque (walker could not resolve the element schema):
                # either way the caller gets the bytes, never a fake
                # DecodeError.
                return self._payload[_offset(descriptor) : _end(descriptor)]
            if isinstance(descriptor, list):
                # A decoded string[] arrives as a list of str already.
                if elem == "String":
                    return descriptor
                return self._nested_value(descriptor, elem)
            return self._nested_value(descriptor, elem)
        if isinstance(field_type, dict) and "Nested" in field_type:
            if isinstance(descriptor, tuple):
                # Loan path: variable nested fields carry a ("raw", offset,
                # len) descriptor - the pre-framed bytes contract, same as
                # string[]/nested[].
                return self._payload[_offset(descriptor) : _end(descriptor)]
            nested_data = descriptor.get("nested", descriptor)
            nested = self._layout._resolve_nested(field_type["Nested"])
            return Message(
                self._payload,
                nested,
                self._schemas,
                owner=self._owner,
                resolved_fields=nested_data.get("fields", {}),
            )
        raise _native.DecodeError(f"unsupported variable field {name}")

    def _nested_value(self, descriptor, elem):
        if isinstance(descriptor, list):
            return [self._nested_value(item, elem) for item in descriptor]
        if isinstance(descriptor, dict):
            nested = self._layout._resolve_nested(elem["Nested"])
            data = descriptor.get("nested", descriptor)
            return Message(
                self._payload,
                nested,
                self._schemas,
                owner=self._owner,
                resolved_fields=data.get("fields", {}),
            )
        raise _native.DecodeError("invalid nested variable descriptor")

    def __setattr__(self, name, value):
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        self._check_alive()
        if not self._writable:
            raise TypeError("frame views are read-only")
        field = self._field(name)
        try:
            if isinstance(field, FieldLayout):
                rec = self._record()
                if isinstance(field.field_type, dict) and "Nested" in field.field_type:
                    raise TypeError("assign nested messages through their fields")
                if (
                    isinstance(field.field_type, dict)
                    and "FixedArray" in field.field_type
                    and isinstance(field.field_type["FixedArray"]["element_type"], dict)
                ):
                    target = rec[name]
                    source = np.asarray(value)
                    if target.dtype.names is None:
                        rec[name] = source
                    else:
                        if source.shape != target.shape + (len(target.dtype.names),):
                            raise ValueError(
                                f"nested array {name} has shape {source.shape}, "
                                f"expected {target.shape + (len(target.dtype.names),)}"
                            )
                        for index, child in enumerate(target.dtype.names):
                            target[child] = source[..., index]
                    return
                if isinstance(field.field_type, dict) and "StringFixed" in field.field_type:
                    raw = str(value).encode()
                    size = field.field_type["StringFixed"]
                    if len(raw) > size:
                        raise _native.EncodeError(
                            f"string field {name} requires at most {size} bytes"
                        )
                    rec[name] = raw
                    return
                rec[name] = value
                return
            descriptor = self._variables[name]
            if field.field_type == "String":
                raw = str(value).encode()
                if len(raw) != _byte_len(descriptor):
                    raise _native.EncodeError(
                        f"string field {name} requires {_byte_len(descriptor)} bytes"
                    )
                self._payload[_offset(descriptor) : _end(descriptor)] = raw
                return
            if field.field_type == "Bytes":
                raw = bytes(value)
                if len(raw) != _byte_len(descriptor):
                    raise _native.EncodeError(
                        f"bytes field {name} requires {_byte_len(descriptor)} bytes"
                    )
                self._payload[_offset(descriptor) : _end(descriptor)] = raw
                return
            if isinstance(field.field_type, dict) and (
                "Nested" in field.field_type
                or (
                    "DynamicArray" in field.field_type
                    and not (
                        isinstance(
                            field.field_type["DynamicArray"]["element_type"], str
                        )
                        and field.field_type["DynamicArray"]["element_type"] in _SCALARS
                    )
                )
            ):
                if isinstance(value, dict) and "Nested" in field.field_type:
                    target = self._layout._resolve_nested(field.field_type["Nested"])
                    raw = bytes(
                        _encode_message(
                            self._schemas, target.qualified_name, value, 0
                        )[_native.WIRE_HEADER_SIZE :]
                    )
                else:
                    # A variable nested field takes its encoded body; a
                    # string[]/nested[] array takes pre-framed bytes only
                    # (element-wise encoding is unsupported).
                    raw = bytes(value)
                if len(raw) != _byte_len(descriptor):
                    raise _native.EncodeError(
                        f"field {name} requires {_byte_len(descriptor)} bytes"
                    )
                self._payload[_offset(descriptor) : _end(descriptor)] = raw
                return
            array = self.__getattr__(name)
            array[...] = value
        except (ValueError, TypeError) as exc:
            raise _native.EncodeError(str(exc)) from exc

    def __dir__(self):
        return sorted(set(super().__dir__()) | set(self._layout._fixed) | set(self._layout._variable))

    def __repr__(self):
        return f"Message({self._layout.qualified_name})"

    def copy(self):
        self._check_alive()
        result = {}
        for field in self._layout.fixed_fields:
            value = getattr(self, field.name)
            result[field.name] = value.copy() if hasattr(value, "copy") else value
        for field in self._layout.variable_fields:
            value = getattr(self, field.name)
            if isinstance(value, Message):
                value = value.copy()
            elif isinstance(value, np.ndarray):
                value = value.copy()
            elif isinstance(value, memoryview):
                value = bytes(value)
            elif isinstance(value, list):
                value = [
                    v.copy() if isinstance(v, Message) else v for v in value
                ]
            result[field.name] = value
        return result

    def _detach(self):
        object.__setattr__(self, "_rec", None)
        object.__setattr__(self, "_payload", None)
        object.__setattr__(self, "_owner", None)
        object.__setattr__(self, "_variables", {})
        object.__setattr__(self, "_owner", None)


def _offset(descriptor):
    return descriptor[2] if descriptor[0] == "prim" else descriptor[1]


def _byte_len(descriptor):
    return descriptor[3] * np.dtype(descriptor[1]).itemsize if descriptor[0] == "prim" else descriptor[2]


def _end(descriptor):
    return _offset(descriptor) + _byte_len(descriptor)
