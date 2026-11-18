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
                            plain setup's tree, built by measured/assemble.py the first time a run needs it
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
PLAIN_TREE = os.path.join(BX, "trees", "plain")
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


def plain_tree(measured=MEASURED, dest=PLAIN_TREE):
    """The plain setup's code tree, built by measured/assemble.py under dest the first time it is needed, and built
    again only when what it is built from changed (a stamp in dest holds source_digest). Returns dest. Raises
    FileNotFoundError when the builder or a tree is missing and ValueError when the builder refuses."""
    builder = assembler(measured)
    try:
        digest = source_digest(measured, builder)
    except builder.AssemblyError as exc:
        raise ValueError(str(exc)) from None
    try:
        with open(os.path.join(dest, STAMP), encoding="utf-8") as f:
            if f.read().strip() == digest:
                return dest
    except OSError:
        pass
    parent = os.path.dirname(dest)
    os.makedirs(parent, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix=".plain.", dir=parent)
    try:
        try:
            builder.assemble_plain(measured, tmp)
        except builder.AssemblyError as exc:
            raise ValueError(str(exc)) from None
        with open(os.path.join(tmp, STAMP), "w", encoding="utf-8") as f:
            f.write(digest + "\n")
        if os.path.isdir(dest):
            shutil.rmtree(dest)
        os.replace(tmp, dest)
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return dest
