"""Python-authored node declarations and their host-side runtime protocol."""

from __future__ import annotations

import importlib.machinery
import json
import os
import sys

from cerulion._native import SchemaMismatch
from cerulion._typed import Message, SchemaSet, _dynamic_descriptor, _wire_length


class NodeContext:
    """Host context passed to a Python node."""


class _NodeImports:
    """Sibling modules (the files beside a node type's ``node.py``) are private
    to that node type, however many node types one interpreter hosts and
    whenever they import them.

    A meta path finder, first on ``sys.meta_path``: while a node type is
    ACTIVE (its ``init``, each of its ticks, its ``shutdown``), a top-level
    import that names a file or package in the active node directory resolves
    there, ahead of every ``sys.path`` entry, and the name is recorded as that
    node's. A recorded name owns its whole dotted subtree: ``helpers.sub``
    goes wherever ``helpers`` goes. On the switch to another node type the
    previous type's recorded modules leave ``sys.modules`` for a stash and the
    next type's stash comes back, so a cached ``helpers`` is always the active
    node's own.

    The import system answers from ``sys.modules`` before it asks any finder,
    so a module cached by OTHER code (a ``utils`` some dependency imported, a
    ``helpers`` another node type found on ``sys.path`` rather than beside
    itself) would be served for the active node's own sibling of that name.
    While a node type is active, every such cached module that one of its
    sibling files would resolve is displaced out of ``sys.modules`` and put
    back when the type deactivates; the modules themselves are untouched, so
    whoever holds them keeps working.

    The registry lives in this one ``cerulion`` package, shared by every node
    cdylib in the process, where a static in the host crate would be one per
    cdylib.
    """

    def __init__(self):
        self.active = None
        self.names = {}
        self.stashes = {}
        self.siblings = {}
        self.displaced = {}
        self.installed = False

    def find_spec(self, name, path, target=None):
        if self.active is None or path is not None or "." in name:
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, [self.active])
        if spec is None:
            return None
        self.names.setdefault(self.active, set()).add(name)
        return spec

    def _owned(self, node_dir):
        """Every ``sys.modules`` key a node type's recorded names own: the
        names themselves and every submodule under them."""
        names = self.names.get(node_dir, ())
        prefixes = tuple(f"{name}." for name in names)
        return [key for key in sys.modules if key in names or key.startswith(prefixes)]

    def _sibling_names(self, node_dir):
        """The top-level module names the files and directories in
        ``node_dir`` answer to, listed once per load of the node type
        (``forget`` clears the list, so a rebuilt node is listed again)."""
        names = self.siblings.get(node_dir)
        if names is None:
            names = set()
            try:
                entries = os.listdir(node_dir)
            except OSError:
                entries = []
            for entry in entries:
                for suffix in importlib.machinery.all_suffixes():
                    if entry.endswith(suffix):
                        entry = entry[: -len(suffix)]
                        break
                if entry.isidentifier():
                    names.add(entry)
            self.siblings[node_dir] = names
        return names

    def _displace(self, node_dir):
        """Move every cached module that one of ``node_dir``'s sibling files
        would shadow out of ``sys.modules``, with its submodules, until the
        type deactivates. It runs on activation, before the type's own stash
        comes back, so whatever sits under a sibling name at that point is
        another party's. A bare directory (a namespace portion) displaces
        nothing: a regular module anywhere on ``sys.path`` outranks it in
        Python's own search too."""
        displaced = self.displaced.setdefault(node_dir, {})
        for name in self._sibling_names(node_dir):
            if name not in sys.modules:
                continue
            spec = importlib.machinery.PathFinder.find_spec(name, [node_dir])
            if spec is None or spec.origin is None:
                continue
            prefix = f"{name}."
            for key in [key for key in sys.modules if key == name or key.startswith(prefix)]:
                displaced[key] = sys.modules.pop(key)

    def _deactivate(self):
        """Stash the active type's own modules and give back the cached
        modules its siblings displaced."""
        stash = self.stashes.setdefault(self.active, {})
        for name in self._owned(self.active):
            stash[name] = sys.modules.pop(name)
        for name, module in self.displaced.pop(self.active, {}).items():
            sys.modules[name] = module
        self.active = None

    def activate(self, node_dir):
        """Make ``node_dir`` the active node type, swapping the private
        modules of the previous one out of ``sys.modules`` and this one's in."""
        if node_dir == self.active:
            return
        if self.active is not None:
            self._deactivate()
        self.active = node_dir
        # Displace first: a foreign module under a sibling name must be kept
        # for the deactivation that gives it back, not overwritten by the
        # stash.
        self._displace(node_dir)
        for name, module in self.stashes.pop(node_dir, {}).items():
            sys.modules[name] = module
        if not self.installed:
            sys.meta_path.insert(0, self)
            self.installed = True

    def forget(self, node_dir):
        """Drop every module recorded for ``node_dir`` so its next import
        loads the files afresh (a rebuilt node, or the same type loaded
        again). Returns the names dropped.

        A type's own modules are in ``sys.modules`` only while it is active;
        deactivating stashes them and gives back the modules its siblings
        displaced. So the drop is the stash, never a name match against
        ``sys.modules``: under a recorded name there may now sit the foreign
        module that was just restored, which is nobody's to delete."""
        if node_dir == self.active:
            self._deactivate()
        dropped = sorted(self.stashes.pop(node_dir, {}))
        self.names.pop(node_dir, None)
        self.siblings.pop(node_dir, None)
        return dropped


_IMPORTS = _NodeImports()


def _activate_node_dir(node_dir, forget=True):
    """The host's entry at ``init``. With ``forget`` (the first instance of
    the node type in this process, or one loaded after every earlier instance
    shut down) what an earlier load of this node directory imported is
    dropped first, so a rebuilt node loads afresh. Another instance of a node
    type that is still running keeps the modules the running one uses: both
    import the same ``helpers``, as two instances of one type should. Either
    way the directory becomes the active node type."""
    dropped = _IMPORTS.forget(node_dir) if forget else []
    _IMPORTS.activate(node_dir)
    return dropped


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
        if "sync_window_ms" in policy and len(trigger_inputs) < 2:
            raise TypeError(
                "sync_window_ms aligns two or more inputs: mark each one with "
                "input(trigger=True)"
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

        # The output is positional-only, so a variable field named `name` (or
        # any other parameter name) is still reachable as a keyword length.
        def loan(self, name, /, **variable_lengths):
            if name not in {port.name for port in outputs}:
                raise KeyError(name)
            if name in self._cer_outputs:
                raise RuntimeError(f"output '{name}' was already touched this tick")
            for field, length in variable_lengths.items():
                if isinstance(length, bool) or not isinstance(length, int) or length < 0:
                    raise TypeError(
                        f"variable length for '{field}' must be a non-negative int"
                    )
            if set(variable_lengths) - {
                field.name for field in self._cer_layouts[name].variable_fields
            }:
                raise TypeError(f"unknown variable field for output '{name}'")
            # Element counts at the API, wire bytes at the host: a `float64[]`
            # of three elements reserves 24 bytes, as `Publisher.loan()` does.
            lengths = [
                _wire_length(field.field_type, variable_lengths.get(field.name, 0))
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
    def __init__(self, cls, ctx, schemas=None, workspace=None, node_dir=None):
        self.cls = cls
        self.ctx = ctx
        self.node_dir = node_dir
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
        if self.node_dir is not None:
            _IMPORTS.activate(self.node_dir)
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
            resolved = self.schemas._native.resolve_frame_buffer(raw, port.schema)
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
        # Shutdown runs after every node type in the process has ticked, so
        # the active type is whichever ticked last: switch back first, or an
        # import here resolves to that type's helpers.
        if self.node_dir is not None:
            _IMPORTS.activate(self.node_dir)
        if hasattr(self.instance, "shutdown"):
            self.instance.shutdown()
