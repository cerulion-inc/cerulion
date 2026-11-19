# SPDX-License-Identifier: AGPL-3.0-only
"""Build the plain arm's code tree from the shipped one. Standard library only.

The benchmark ran two code trees that differ in a few files. rcp/ is the arm with the Cerulion harness's tools and
ships whole. The plain arm (the paper's three-tool interface, without the harness) is rcp/ with every file of
plain-overlay/ put in place of rcp/'s file of the same path, and every path plain-absent.txt names left out.

    python3 harness/measured/assemble.py DEST

writes the plain tree to DEST, which must be missing or empty. What a run leaves inside a tree (Python caches, the
run outputs in RUNTIME_OUTPUT, the .DS_Store file macOS writes) is never copied. provenance/ledger.json holds the hash of
every file of the result, and cerulion_harness.verify assembles the tree and checks it against those hashes.
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# Folders, relative to a tree's root, that the measured code writes its run outputs into: the derived robot models
# (bench/derive_spec.py) and the grid's default results folder (runs/tools/grid_ext.py).
RUNTIME_OUTPUT = ("runs/derived_mjcf", "runs/benchmarks")
SKIPPED_DIRS = ("__pycache__",)
SKIPPED_FILES = (".DS_Store",)
ABSENT_LIST = "plain-absent.txt"


class AssemblyError(Exception):
    """The overlay, the absent list or the destination does not allow a plain tree to be built."""


def tree_files(root):
    """The sorted relative paths ("/" separated) of the regular files under root, with what a run leaves in a tree
    left out. A symbolic link anywhere is an error: the shipped trees hold none."""
    if not os.path.isdir(root):
        raise AssemblyError("%s is not a folder" % root)
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root).replace(os.sep, "/")
        rel_dir = "" if rel_dir == "." else rel_dir + "/"
        for name in dirnames + filenames:
            if os.path.islink(os.path.join(dirpath, name)):
                raise AssemblyError("%s%s is a symbolic link" % (rel_dir, name))
        dirnames[:] = sorted(d for d in dirnames
                             if d not in SKIPPED_DIRS and rel_dir + d not in RUNTIME_OUTPUT)
        for name in filenames:
            if name in SKIPPED_FILES or name.endswith(".pyc"):
                continue
            out.append(rel_dir + name)
    return sorted(out)


def absent_paths(measured):
    """The paths plain-absent.txt names, relative to rcp/: one per line, blank lines and # lines skipped."""
    path = os.path.join(measured, ABSENT_LIST)
    with open(path, encoding="utf-8") as f:
        lines = [line.strip() for line in f]
    paths = [line for line in lines if line and not line.startswith("#")]
    for rel in paths:
        parts = rel.split("/")
        if rel.startswith("/") or "\\" in rel or any(p in ("", ".", "..") for p in parts):
            raise AssemblyError("%s: %r is not a plain relative path" % (ABSENT_LIST, rel))
    if len(set(paths)) != len(paths):
        raise AssemblyError("%s names a path twice" % ABSENT_LIST)
    return paths


def assemble_plain(measured, dest):
    """Write the plain arm's tree to dest from measured/rcp, measured/plain-overlay and measured/plain-absent.txt.
    Returns the sorted relative paths written. Raises AssemblyError, and writes nothing, when an absent path is not a
    file of rcp/, an overlay file replaces no file of rcp/ or is also absent, or dest holds anything."""
    rcp = os.path.join(measured, "rcp")
    overlay = os.path.join(measured, "plain-overlay")
    base = tree_files(rcp)
    have = set(base)
    absent = absent_paths(measured)
    for rel in absent:
        if rel not in have:
            raise AssemblyError("%s names %s, which is not a file of rcp/" % (ABSENT_LIST, rel))
    replaced = tree_files(overlay)
    for rel in replaced:
        if rel not in have:
            raise AssemblyError("plain-overlay/%s replaces no file of rcp/" % rel)
        if rel in absent:
            raise AssemblyError("plain-overlay/%s is also named in %s" % (rel, ABSENT_LIST))
    if os.path.lexists(dest) and (not os.path.isdir(dest) or os.listdir(dest)):
        raise AssemblyError("%s is not an empty folder" % dest)
    left_out, from_overlay = set(absent), set(replaced)
    written = []
    for rel in base:
        if rel in left_out:
            continue
        src = os.path.join(overlay if rel in from_overlay else rcp, *rel.split("/"))
        dst = os.path.join(dest, *rel.split("/"))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src, dst)
        shutil.copymode(src, dst)
        written.append(rel)
    return written


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or argv[0].startswith("-"):
        print("usage: python3 assemble.py DEST   (DEST: a missing or empty folder)", file=sys.stderr)
        return 2
    try:
        written = assemble_plain(HERE, argv[0])
    except (AssemblyError, OSError) as e:
        print("assemble: %s" % e, file=sys.stderr)
        return 1
    print("assemble: wrote %d files of the plain tree to %s" % (len(written), argv[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
