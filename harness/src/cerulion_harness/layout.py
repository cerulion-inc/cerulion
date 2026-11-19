# SPDX-License-Identifier: AGPL-3.0-only
"""Where the harness's parts live, and the plain setup's code tree. Standard library only.

Every path is inside the harness folder, the folder that holds this package's src/:

    measured/rcp            the Cerulion harness setup's code tree, run as it is
    measured/assemble.py    builds the plain setup's tree from measured/rcp, measured/plain-overlay and
                            measured/plain-absent.txt
    body_records/           the robot's calibration records: the shipped ones, and the ones measured on this
                            machine
    macos/                  the macOS simulator launcher (Apple's OpenGL)
    bx/                     written by install.sh: the simulator environment and the benchmark scenes; and the
                            plain setup's trees (bx/trees/plain/DIGEST, one for each state of their sources),
                            each built by measured/assemble.py the first time a run needs it
    venv/                   written by install.sh: the episode server environment (Python 3.12)
"""
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import tempfile

SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HARNESS = os.path.dirname(SRC)
BX = os.path.join(HARNESS, "bx")
SIM_PYTHON = os.path.join(BX, "libero-env", "bin", "python")
VENV_PYTHON = os.path.join(HARNESS, "venv", "bin", "python")
RECORDS = os.path.join(HARNESS, "body_records")
MEASURED = os.path.join(HARNESS, "measured")
RCP_TREE = os.path.join(MEASURED, "rcp")
PLAIN_TREES = os.path.join(BX, "trees", "plain")
MACOS_ENVHOST = os.path.join(HARNESS, "macos", "envhost_macos.sh")
STAMP = ".assembled-from"


def simulator_env(env):
    """env with the settings every process of an episode reads: where the simulator and the scenes are, and on
    macOS the launcher that renders through Apple's OpenGL."""
    env = dict(env)
    env["LIBERO_BX"] = BX
    if sys.platform == "darwin":
        env["RCPX_EXT_ENVHOST"] = json.dumps([MACOS_ENVHOST, "{runtime}", "{env_sock}", "{scoring}", "{gl}"])
    else:
        env.setdefault("MUJOCO_GL", "osmesa")
        env.setdefault("PYOPENGL_PLATFORM", "osmesa")
    return env


def assembler(measured=MEASURED):
    """measured/assemble.py, loaded from its path: the one builder of the plain tree, the same one verify checks."""
    path = os.path.join(measured, "assemble.py")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"no plain tree builder at {path}")
    spec = importlib.util.spec_from_file_location("cerulion_harness_assemble", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source_digest(measured, builder):
    """SHA-256 over what a plain tree is built from: every file of measured/rcp and measured/plain-overlay as the
    builder lists them (run output left out), the absent list and the builder itself, by path and bytes."""
    h = hashlib.sha256()
    parts = [("rcp", os.path.join(measured, "rcp"), builder.tree_files(os.path.join(measured, "rcp"))),
             ("overlay", os.path.join(measured, "plain-overlay"),
              builder.tree_files(os.path.join(measured, "plain-overlay"))),
             ("measured", measured, [builder.ABSENT_LIST, "assemble.py"])]
    for label, root, rels in parts:
        for rel in rels:
            h.update(f"{label}\0{rel}\0".encode("utf-8"))
            with open(os.path.join(root, *rel.split("/")), "rb") as f:
                h.update(hashlib.sha256(f.read()).digest())
    return h.hexdigest()


def _stamped(tree, digest):
    """tree holds a stamp that names digest."""
    try:
        with open(os.path.join(tree, STAMP), encoding="utf-8") as f:
            return f.read().strip() == digest
    except OSError:
        return False


def plain_tree(measured=MEASURED, trees=PLAIN_TREES):
    """The plain setup's code tree of the current sources: trees/DIGEST, DIGEST the first 16 hex digits of
    source_digest (a stamp in the tree holds all of it). The first run that needs it builds it with
    measured/assemble.py in a folder of its own and renames that folder into place whole; no tree in place is
    written over or removed, so an episode that runs from one keeps it while other runs build theirs. Two runs
    that build the same tree at once both use the one renamed into place first. Changed sources get a new tree
    beside the earlier ones; while no run is going, trees/ may be removed whole. Returns the tree's path. Raises
    FileNotFoundError when the builder or a tree is missing, and ValueError when the builder refuses or
    trees/DIGEST holds something else."""
    builder = assembler(measured)
    try:
        digest = source_digest(measured, builder)
    except builder.AssemblyError as exc:
        raise ValueError(str(exc)) from None
    dest = os.path.join(trees, digest[:16])
    if _stamped(dest, digest):
        return dest
    os.makedirs(trees, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix=".building.", dir=trees)
    try:
        try:
            builder.assemble_plain(measured, tmp)
        except builder.AssemblyError as exc:
            raise ValueError(str(exc)) from None
        with open(os.path.join(tmp, STAMP), "w", encoding="utf-8") as f:
            f.write(digest + "\n")
        try:
            os.rename(tmp, dest)   # refused when dest is a folder that holds anything: it is left as it is
        except OSError:
            if _stamped(dest, digest):
                pass   # another run renamed the same tree into place first: that one is used
            elif os.path.lexists(dest):
                raise ValueError(f"{dest} holds files that are not the plain tree of these sources: remove "
                                 f"that folder") from None
            else:
                raise
    finally:
        shutil.rmtree(tmp, ignore_errors=True)   # after the rename nothing is left here; else the unused copy
    return dest
