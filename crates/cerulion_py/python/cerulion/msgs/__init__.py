"""Message classes, one module per package: ``cerulion.msgs.<package>``.

``from cerulion.msgs.sensor_msgs import LaserScan`` names the ROS 2 built-in
message; a workspace's own messages (``schemas/<package>/msg/*.msg``, and
``schemas/*.yaml``) join the same namespace under their package, and a
package-less YAML schema is ``cerulion.msgs.<Name>``. A Rust node names the
same types ``native_ros2_messages::<package>::<Type>``.

The classes come from ONE resolved message set: the built-ins under the bound
workspace, by the rule the Rust build and the node host use (a workspace
definition replaces the built-in of the same name, and the built-ins that
embed it are absent). The workspace is, in order: the one the node host binds
before it imports ``node.py``; ``$CERULION_WORKSPACE``; the workspace that
contains the current directory (a ``Cargo.toml`` with ``[workspace]`` beside a
``graphs/`` directory); else none, and only the built-ins are here.

A class names a type. Every view of that schema the set opens (an input, a
loan, ``Frame.view()``, a nested field, a bag record) is an instance of it.
"""


# Module-level names are underscored: a package-less schema is an attribute
# of this module, so no import may sit under a name a message could take.
import importlib as _importlib
import importlib.abc as _importlib_abc
import importlib.machinery as _machinery
import os as _os
import sys as _sys
import types as _types

from cerulion._typed import SchemaSet as _SchemaSet, _module_name

_MODULE = __name__
# The process default: bound on first use outside a node, or by
# `_bind_workspace` when no node type is active.
_default = {"root": None, "set": None}
# One set per workspace root (realpath), shared by every node type of that
# workspace, so their classes are the same objects: `{key: (fingerprint, set)}`.
_sets = {}
# The workspace each node type (its node directory) was bound to.
_node_roots = {}


def _find_workspace(start):
    """The workspace that contains ``start``: the first directory upward with
    a ``Cargo.toml`` declaring ``[workspace]`` beside a ``graphs/`` directory
    (the ``cerulion`` CLI's rule), or ``None``."""
    current = _os.path.abspath(start)
    while True:
        manifest = _os.path.join(current, "Cargo.toml")
        if _os.path.isfile(manifest) and _os.path.isdir(_os.path.join(current, "graphs")):
            try:
                with open(manifest, encoding="utf-8") as handle:
                    if "[workspace]" in handle.read():
                        return current
            except OSError:
                pass
        parent = _os.path.dirname(current)
        if parent == current:
            return None
        current = parent


def _fingerprint(root):
    """What the message set of ``root`` was read from: every file under its
    ``schemas/`` with its size and modification time."""
    if root is None:
        return ()
    entries = []
    for directory, _, files in _os.walk(_os.path.join(root, "schemas")):
        for name in files:
            path = _os.path.join(directory, name)
            try:
                stat = _os.stat(path)
            except OSError:
                continue
            entries.append((path, stat.st_size, stat.st_mtime_ns))
    return tuple(sorted(entries))


def _set_for(root, reload=False):
    """The message set of workspace ``root`` (``None``: the built-ins alone),
    loaded once per process. With ``reload`` the workspace's files are
    checked again and a changed workspace is read afresh (a rebuilt node);
    classes handed out from the earlier set stay that set's."""
    key = _os.path.realpath(root) if root is not None else None
    cached = _sets.get(key)
    if cached is not None and not reload:
        return cached[1]
    fingerprint = _fingerprint(key)
    if cached is not None and cached[0] == fingerprint:
        return cached[1]
    schemas = _SchemaSet.from_workspace(root) if root is not None else _SchemaSet.builtins()
    schemas._class_module = _MODULE
    schemas._frozen = True
    _sets[key] = (fingerprint, schemas)
    return schemas


def _active_node():
    node = _sys.modules.get("cerulion._node")
    return None if node is None else node._IMPORTS.active


def _bind_workspace(root, reload=True):
    """Bind ``root`` as the workspace ``cerulion.msgs`` resolves against. The
    node host calls this with the node type active, before it imports
    ``node.py``: that node type's imports then resolve against its own
    workspace, and its ``cerulion.msgs.<pkg>`` modules are its own, swapped
    with its ``helpers`` when another node type runs; ``reload`` is the
    host's first-instance flag (see below). With no node type active it
    binds the process default, once: rebinding the default to a
    different workspace raises ``ImportError``."""
    root = _os.path.abspath(root)
    active = _active_node()
    if active is not None:
        # A node type is loading: with `reload` (its first instance in the
        # process) its workspace is read afresh if a file under `schemas/`
        # changed since the set was loaded; a further instance keeps the set
        # its running sibling's modules came from.
        _node_roots[active] = root
        return _set_for(root, reload=reload)
    if _default["set"] is not None:
        if _default["root"] is not None and _os.path.realpath(_default["root"]) == _os.path.realpath(root):
            return _default["set"]
        raise ImportError(
            f"cerulion.msgs is bound to "
            f"{_default['root'] or 'the built-in messages only'}; it cannot rebind to {root}"
        )
    _default["root"] = root
    _default["set"] = _set_for(root)
    return _default["set"]


def _message_set():
    """The set ``cerulion.msgs`` resolves against now: the active node type's
    workspace, else the process default, bound on first use from
    ``$CERULION_WORKSPACE`` or the workspace around the current directory."""
    active = _active_node()
    if active is not None and active in _node_roots:
        return _set_for(_node_roots[active])
    if _default["set"] is None:
        root = _os.environ.get("CERULION_WORKSPACE") or _find_workspace(_os.getcwd())
        _default["root"] = _os.path.abspath(root) if root else None
        _default["set"] = _set_for(_default["root"])
    return _default["set"]


def _workspace_root():
    """The workspace root of the set ``cerulion.msgs`` resolves against now,
    or ``None`` for the built-ins alone."""
    active = _active_node()
    if active is not None and active in _node_roots:
        return _node_roots[active]
    _message_set()
    return _default["root"]


def _schemas_for(root):
    """The set a node running in workspace ``root`` resolves against: the
    same object ``cerulion.msgs`` hands that workspace's classes from, so the
    node's views are instances of the classes it declared."""
    return _set_for(root)


def _split(qualified):
    """``(module, leaf)`` of a qualified name: ``pkg/Name`` is ``Name`` in
    module ``pkg``; a name without ``/`` sits at the top of ``cerulion.msgs``
    (module ``None``). A name with several ``/`` has no class (``None``)."""
    package, _, leaf = qualified.rpartition("/")
    if "/" in package:
        return None
    return (_module_name(package) if package else None), leaf


def _index(schemas=None):
    """``{module or None: {leaf: qualified}}`` over ``schemas`` (default: the
    current set), computed once per set: a ``cerulion.msgs`` set is never
    mutated."""
    if schemas is None:
        schemas = _message_set()
    index = getattr(schemas, "_msgs_index", None)
    if index is not None:
        return index
    index = {}
    for qualified in schemas.names():
        split = _split(qualified)
        if split is None:
            continue
        module, leaf = split
        index.setdefault(module, {})[leaf] = qualified
    schemas._msgs_index = index
    return index


def _fill(namespace, schemas=None):
    """Fill the module namespace ``namespace`` (its ``globals()``) of
    ``cerulion.msgs.<package>`` with the package's classes from ``schemas``
    (default: the current set). Inside a node type the module is recorded as
    that type's own, so it leaves ``sys.modules`` with the type's
    ``helpers`` and the next type imports its own. The wheel's built-in
    package modules are one call of this; a workspace package's module is
    made by the finder below."""
    if schemas is None:
        schemas = _message_set()
    name = namespace["__name__"]
    package = name.rpartition(".")[2]
    active = _active_node()
    if active is not None:
        _sys.modules["cerulion._node"]._IMPORTS.names.setdefault(active, set()).add(name)
    members = _index(schemas).get(package, {})
    for leaf, qualified in sorted(members.items()):
        namespace[leaf] = schemas.message_type(qualified)
    namespace["__all__"] = sorted(members)
    namespace["__cerulion_schemas__"] = schemas


class _PackageLoader(_importlib_abc.Loader):
    def __init__(self, schemas):
        self.schemas = schemas

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        package = module.__name__.rpartition(".")[2]
        module.__doc__ = f"The {package} message classes."
        _fill(module.__dict__, self.schemas)


class _PackageFinder(_importlib_abc.MetaPathFinder):
    """Answers ``cerulion.msgs.<package>`` for every package of the current
    set that has no module file in the wheel (a workspace package); nothing
    else."""

    def find_spec(self, fullname, path, target=None):
        prefix = _MODULE + "."
        if not fullname.startswith(prefix):
            return None
        package = fullname[len(prefix) :]
        if "." in package or package.startswith("_"):
            return None
        schemas = _message_set()
        if package not in _index(schemas):
            return None
        return _machinery.ModuleSpec(fullname, _PackageLoader(schemas))


class _MessagesModule(_types.ModuleType):
    """``cerulion.msgs`` itself. A package module is never cached as an
    attribute here (the import system would pin the first node type's one):
    ``cerulion.msgs.<pkg>`` resolves through ``sys.modules`` every time."""

    def __setattr__(self, name, value):
        if (
            not name.startswith("_")
            and isinstance(value, _types.ModuleType)
            and value.__name__ == f"{_MODULE}.{name}"
        ):
            return
        super().__setattr__(name, value)


_sys.modules[_MODULE].__class__ = _MessagesModule
_FINDER = _PackageFinder()
if not any(isinstance(finder, _PackageFinder) for finder in _sys.meta_path):
    _sys.meta_path.append(_FINDER)


def __getattr__(name):
    if name.startswith("_"):
        raise AttributeError(name)
    index = _index()
    root = index.get(None, {})
    if name in root:
        return _message_set().message_type(root[name])
    if name in index:
        return _importlib.import_module(f"{_MODULE}.{name}")
    raise AttributeError(f"module {_MODULE!r} has no message or package {name!r}")


def __dir__():
    index = _index()
    names = set(index.get(None, {}))
    names.update(module for module in index if module is not None)
    return sorted(names | {name for name in globals() if name.startswith("__")})
