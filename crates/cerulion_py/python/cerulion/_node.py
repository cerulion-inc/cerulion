"""Python-authored node declarations and their host-side runtime protocol."""

from __future__ import annotations

import json
import os

from cerulion._native import SchemaMismatch
from cerulion._typed import Message, SchemaSet, _dynamic_descriptor


class NodeContext:
    """Host context passed to a Python node."""


class _Port:
    def __init__(self, schema, *, input_port, **kwargs):
        self.schema = schema
        self.input_port = input_port
        self.kwargs = kwargs
        self.name = None
        self.owner = None

    def __set_name__(self, owner, name):
        self.owner = owner
        self.name = name
        ports = list(getattr(owner, "__cerulion_declared_ports__", ()))
        ports.append(self)
        owner.__cerulion_declared_ports__ = ports

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        if self.input_port:
            if not hasattr(instance, "_cer_inputs"):
                raise RuntimeError("input views are only valid inside tick()")
            return instance._cer_inputs.get(self.name)
        if not hasattr(instance, "_cer_tick"):
            raise RuntimeError("output views are only valid inside tick()")
        if self.name not in instance._cer_outputs:
            count = len(instance._cer_layouts[self.name].variable_fields)
            instance._cer_outputs[self.name] = instance._cer_make_output(
                self.name, [0] * count
            )
        return instance._cer_outputs[self.name]

    def __set__(self, instance, value):
        if self.input_port:
            raise AttributeError(f"cannot assign input '{self.name}'")
        raise AttributeError(f"cannot assign output '{self.name}' directly")


def input(
    schema,
    *,
    depth=None,
    trigger=False,
    expect_within_ms=None,
    backpressure=None,
):
    if depth is not None and (
        isinstance(depth, bool) or not isinstance(depth, int) or not 1 <= depth <= 64
    ):
        raise ValueError("depth must be between 1 and 64")
    if not isinstance(trigger, bool):
        raise TypeError(f"trigger must be True or False, got {trigger!r}")
    if expect_within_ms is not None:
        _positive_int("expect_within_ms", expect_within_ms)
    if backpressure not in (None, "drop_oldest", "block") and not (
        isinstance(backpressure, dict)
        and set(backpressure) == {"sample"}
        and isinstance(backpressure["sample"], int)
        and not isinstance(backpressure["sample"], bool)
        and backpressure["sample"] > 0
    ):
        raise ValueError(f"unknown backpressure policy: {backpressure}")
    return _Port(
        schema,
        input_port=True,
        depth=depth,
        trigger=trigger,
        expect_within_ms=expect_within_ms,
        backpressure=backpressure,
    )


def output(schema, *, promise_within_ms=None, max_slice_len_default=None):
    if promise_within_ms is not None:
        _positive_int("promise_within_ms", promise_within_ms)
    if max_slice_len_default is not None and (
        isinstance(max_slice_len_default, bool)
        or not isinstance(max_slice_len_default, int)
        or max_slice_len_default < 32
    ):
        raise ValueError("max_slice_len_default must be at least 32")
    return _Port(
        schema,
        input_port=False,
        promise_within_ms=promise_within_ms,
        max_slice_len_default=max_slice_len_default,
    )


def _positive_int(name, value):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _policy(period_ms, trigger, sync_window_ms):
    selected = sum(value is not None for value in (period_ms, trigger, sync_window_ms))
    if selected > 1:
        raise TypeError("exactly one scheduling policy must be selected")
    if selected == 0:
        return {}
    if period_ms is not None:
        return {"period_ms": _positive_int("period_ms", period_ms)}
    if trigger is not None:
        return {"data_trigger": {"input_name": trigger}}
    return {"sync_window_ms": _positive_int("sync_window_ms", sync_window_ms)}


def node(
    period_ms=None,
    *,
    trigger=None,
    sync_window_ms=None,
    tick_within_ms=None,
    throttle_ms=None,
):
    policy = _policy(period_ms, trigger, sync_window_ms)
    for name, value in (
        ("tick_within_ms", tick_within_ms),
        ("throttle_ms", throttle_ms),
    ):
        if value is not None:
            _positive_int(name, value)

    def decorate(cls):
        if not callable(cls.__dict__.get("tick")):
            raise TypeError("Python node must define a callable tick method")
        for reserved in ("now_ns", "request_shutdown", "env", "loan"):
            if reserved in cls.__dict__:
                raise TypeError(f"reserved method name: {reserved}")
        ports = list(getattr(cls, "__cerulion_declared_ports__", ()))
        seen = set()
        for port in ports:
            if port.name in seen:
                raise TypeError(f"duplicate port name: {port.name}")
            seen.add(port.name)
            if port.name.startswith("__"):
                raise TypeError(f"reserved port name: {port.name}")
            if port.name.startswith("_cer_") or port.name.startswith("__cerulion") or port.name in (
                "now_ns",
                "request_shutdown",
                "env",
                "loan",
                "tick",
                "init",
                "shutdown",
            ):
                raise TypeError(f"reserved port name: {port.name}")
        inputs = [port for port in ports if port.input_port]
        outputs = [port for port in ports if not port.input_port]
        if trigger is not None and trigger not in {port.name for port in inputs}:
            raise TypeError(f"unknown trigger input: {trigger}")
        trigger_inputs = [port.name for port in inputs if port.kwargs["trigger"]]
        if "period_ms" in policy and trigger_inputs:
            raise TypeError(
                "cannot combine input(trigger=True) with period_ms: trigger inputs "
                "define a data-driven policy, which conflicts with a time-driven one"
            )
        if "data_trigger" in policy and any(name != trigger for name in trigger_inputs):
            raise TypeError(
                f"node(trigger={trigger!r}) conflicts with input(trigger=True) on "
                f"{', '.join(name for name in trigger_inputs if name != trigger)}"
            )
        effective_policy = policy
        if not effective_policy:
            if len(trigger_inputs) > 1:
                raise TypeError("multiple trigger inputs require sync_window_ms")
            if not trigger_inputs:
                raise TypeError(
                    "no trigger policy: mark an input with trigger=True, or specify "
                    "period_ms or sync_window_ms"
                )
            effective_policy = {"data_trigger": {"input_name": trigger_inputs[0]}}

        cls.__cerulion_ports__ = {"inputs": inputs, "outputs": outputs}
        cls.__cerulion_policy__ = effective_policy

        def info(schemas=None):
            if schemas is None:
                workspace = os.environ.get("CERULION_WORKSPACE")
                if workspace is None:
                    raise RuntimeError("pass a SchemaSet when CERULION_WORKSPACE is unset")
                schemas = SchemaSet.from_workspace(workspace)
            input_info = []
            for port in inputs:
                schema_hash, _, _ = schemas.output_meta(port.schema)
                input_info.append(
                    {
                        "name": port.name,
                        "schema": port.schema,
                        "schema_hash": schema_hash,
                    }
                )
                if port.kwargs["trigger"]:
                    input_info[-1]["trigger"] = True
                for key in ("expect_within_ms", "depth", "backpressure"):
                    value = port.kwargs[key]
                    if value is not None:
                        input_info[-1][key] = value
            output_info = []
            for port in outputs:
                schema_hash, fixed_size, max_slice = schemas.output_meta(port.schema)
                configured_max_slice = port.kwargs["max_slice_len_default"]
                if configured_max_slice is not None:
                    smallest = schemas._native.min_frame_len(port.schema)
                    if configured_max_slice < smallest:
                        raise ValueError(
                            f"output {port.name!r}: max_slice_len_default "
                            f"{configured_max_slice} is smaller than the {smallest}-byte "
                            f"{port.schema} frame"
                        )
                output_info.append(
                    {
                        "name": port.name,
                        "schema": port.schema,
                        "schema_hash": schema_hash,
                        "max_slice_len_default": configured_max_slice
                        if configured_max_slice is not None
                        else max_slice,
                        "promise_within_ms": port.kwargs["promise_within_ms"],
                        "wire_fixed_size": fixed_size,
                    }
                )
            document = {
                "inputs": input_info,
                "outputs": output_info,
                "policy": effective_policy,
            }
            if tick_within_ms is not None:
                document["tick_within_ms"] = tick_within_ms
            if throttle_ms is not None:
                document["throttle_ms"] = throttle_ms
            return json.dumps(document, separators=(",", ":"))

        cls.__cerulion_info__ = staticmethod(info)

        def now_ns(self):
            return self._cer_ctx.now_ns()

        def env(self, name, default=None):
            return self._cer_ctx.env(name, default)

        def request_shutdown(self):
            return self._cer_ctx.request_shutdown()

        def loan(self, name, **variable_lengths):
            if name not in {port.name for port in outputs}:
                raise KeyError(name)
            if name in self._cer_outputs:
                raise RuntimeError(f"output '{name}' was already touched this tick")
            port = next(port for port in outputs if port.name == name)
            for field, length in variable_lengths.items():
                if isinstance(length, bool) or not isinstance(length, int) or length < 0:
                    raise TypeError(
                        f"variable length for '{field}' must be a non-negative int"
                    )
            if set(variable_lengths) - {
                field.name for field in self._cer_layouts[name].variable_fields
            }:
                raise TypeError(f"unknown variable field for output '{name}'")
            lengths = [
                variable_lengths.get(field.name, 0)
                for field in self._cer_layouts[name].variable_fields
            ]
            self._cer_outputs[name] = self._cer_make_output(name, lengths)
            return self._cer_outputs[name]

        cls.now_ns = now_ns
        cls.env = env
        cls.request_shutdown = request_shutdown
        cls.loan = loan
        return cls

    return decorate


class _Runtime:
    def __init__(self, cls, ctx, schemas=None, workspace=None):
        self.cls = cls
        self.ctx = ctx
        root = (workspace or ctx.env("CERULION_WORKSPACE", None)) if schemas is None else None
        self.schemas = schemas or SchemaSet.from_workspace(root or os.getcwd())
        self.instance = cls()
        self.instance._cer_ctx = ctx
        self.instance._cer_layouts = {
            port.name: self.schemas.layout(port.schema)
            for port in cls.__cerulion_ports__["outputs"]
        }
        self.instance._cer_make_output = self._make_output
        if hasattr(self.instance, "init"):
            self.instance.init(ctx)

    def begin_tick(self, tick_handle, frames):
        inputs = {}
        self.instance._cer_tick = tick_handle
        self.instance._cer_outputs = {}
        self.instance._cer_loans = {}
        self.instance._cer_input_views = {}
        for port, frame in zip(self.cls.__cerulion_ports__["inputs"], frames):
            if frame is None:
                inputs[port.name] = None
                continue
            if frame.schema_hash != self.schemas.schema_hash(port.schema):
                raise SchemaMismatch(
                    f"schema mismatch on input '{port.name}'"
                )
            raw = memoryview(frame)
            self.instance._cer_input_views[port.name] = raw
            resolved = self.schemas._native.resolve_frame(raw, port.schema)
            payload = raw[32:]
            layout = self.schemas.layout(port.schema)
            inputs[port.name] = Message(
                payload,
                layout,
                self.schemas,
                resolved.get("variables"),
                owner=frame,
            )
        self.instance._cer_inputs = inputs

    def _make_output(self, name, lengths):
        loan = self.instance._cer_tick.loan(name, lengths)
        entries = loan.variable_entries() or []
        descriptors = {}
        for field, entry in zip(self.instance._cer_layouts[name].variable_fields, entries):
            _, offset, byte_len = entry
            descriptors[field.name] = _dynamic_descriptor(
                field.field_type, offset, byte_len
            )
        message = Message(
            memoryview(loan),
            self.instance._cer_layouts[name],
            self.schemas,
            descriptors,
            owner=loan,
        )
        self.instance._cer_loans[name] = loan
        return message

    def run_tick(self):
        return self.instance.tick()

    def end_tick(self):
        touched = []
        messages = list(self.instance._cer_inputs.values()) + list(
            self.instance._cer_outputs.values()
        )
        for name in self.instance._cer_outputs:
            touched.append((name, self.instance._cer_loans[name]))
        for message in messages:
            if message is not None:
                message._detach()
        self.instance._cer_input_views.clear()
        self.instance._cer_inputs = {}
        self.instance._cer_input_views = {}
        self.instance._cer_outputs = {}
        del self.instance._cer_tick
        return touched

    def abort_tick(self):
        touched = []
        outputs = getattr(self.instance, "_cer_outputs", {})
        loans = getattr(self.instance, "_cer_loans", {})
        for name in outputs:
            loan = loans.get(name)
            if loan is not None:
                touched.append((name, loan))
        messages = list(getattr(self.instance, "_cer_inputs", {}).values()) + list(
            outputs.values()
        )
        for message in messages:
            if message is not None:
                message._detach()
        self.instance._cer_input_views = {}
        self.instance._cer_inputs = {}
        self.instance._cer_outputs = {}
        self.instance._cer_loans = {}
        if hasattr(self.instance, "_cer_tick"):
            del self.instance._cer_tick
        return touched

    def shutdown(self):
        if hasattr(self.instance, "shutdown"):
            self.instance.shutdown()
