# SPDX-License-Identifier: AGPL-3.0-only
"""The hash of a code tree, the same for every builder. Standard library only.

    python -m bench.instruments.cells snapshot [--root TREE] [--dir rcpx --dir bench ...]

The hash is the SHA-256 of a manifest: one line "FILEHASH  relative/path" per regular file under the named folders
of the tree (default: rcpx and bench), sorted by path, with forward slashes; compiled files, caches and symbolic
links are left out. Two trees have the same hash exactly when those files are byte-equal. The printed form is
"sha256:<hex>"; a cell log line carries it so that a run can be tied to the code that made it.
"""

import argparse
import hashlib
import os
import sys

DEFAULT_DIRS = ("rcpx", "bench")
SKIP_DIRS = ("__pycache__", ".git", ".pytest_cache", ".mypy_cache")
SKIP_SUFFIXES = (".pyc", ".pyo")
SKIP_NAMES = (".DS_Store",)


def _file_hash(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def manifest(root, dirs=DEFAULT_DIRS):
    """The manifest text of a tree. ValueError when a named folder is missing."""
    lines = []
    for d in dirs:
        top = os.path.join(root, d)
        if not os.path.isdir(top):
            raise ValueError(f"no folder {d} in {root}")
        for dirpath, dirnames, filenames in os.walk(top, followlinks=False):
            dirnames[:] = sorted(x for x in dirnames
                                 if x not in SKIP_DIRS and not os.path.islink(os.path.join(dirpath, x)))
            for name in sorted(filenames):
                if name in SKIP_NAMES or name.endswith(SKIP_SUFFIXES):
                    continue
                path = os.path.join(dirpath, name)
                if os.path.islink(path) or not os.path.isfile(path):
                    continue
                rel = os.path.relpath(path, root).replace(os.sep, "/")
                lines.append((rel, _file_hash(path)))
    lines.sort()
    return "".join(f"{h}  {rel}\n" for rel, h in lines)


def snapshot_hash(root, dirs=DEFAULT_DIRS):
    """'sha256:<hex>' of the tree's manifest."""
    return "sha256:" + hashlib.sha256(manifest(root, dirs).encode("utf-8")).hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m bench.instruments.cells snapshot",
                                 description="Print the hash of a code tree.")
    ap.add_argument("--root", default=None, help="the tree (default: the tree this file lies in)")
    ap.add_argument("--dir", action="append", default=None, help="a folder of the tree to hash (repeatable)")
    ap.add_argument("--manifest", action="store_true", help="print the manifest lines too")
    ns = ap.parse_args(argv)
    root = ns.root or os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    dirs = tuple(ns.dir) if ns.dir else DEFAULT_DIRS
    try:
        text = manifest(root, dirs)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if ns.manifest:
        sys.stdout.write(text)
    print("sha256:" + hashlib.sha256(text.encode("utf-8")).hexdigest(), f"files={text.count(chr(10))}",
          "dirs=" + ",".join(dirs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
