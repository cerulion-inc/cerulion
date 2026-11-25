# SPDX-License-Identifier: AGPL-3.0-only
"""What git tracks under `harness/`: no fetched scene, no built environment.

`install.sh` writes under `bx/` and `venv/`, and a run writes body records
measured on its machine beside the shipped ones. A tracked file under `bx/` or
`venv/`, or a tracked file of a kind the benchmark ships (a mesh, a texture, a
stored start state, a dataset), means a scene or an environment was committed.
"""

import pathlib
import subprocess
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
REPO = HARNESS.parent

# What install.sh builds or fetches, relative to harness/.
NEVER_TRACKED_DIRS = ("bx", "venv", "out")
# File kinds the benchmark trees carry and this folder never does.
SCENE_SUFFIXES = (".stl", ".obj", ".mtl", ".msh", ".hdf5", ".pruned_init", ".init", ".bddl", ".xml")
# The largest tracked file here is a digest list; one megabyte is far above it.
MAX_TRACKED_BYTES = 1024 * 1024


def tracked():
    out = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "-z", "--", "harness"],
        check=True, stdout=subprocess.PIPE).stdout
    return [name for name in out.decode("utf-8").split("\0") if name]


class TrackedFiles(unittest.TestCase):
    def setUp(self):
        self.names = tracked()

    def test_git_tracks_this_folder(self):
        # An empty answer would pass every rule below while checking nothing.
        self.assertIn("harness/install.sh", self.names)
        self.assertIn("harness/benchmarks/fetch.sh", self.names)

    def test_nothing_install_writes_is_tracked(self):
        for name in self.names:
            top = pathlib.PurePosixPath(name).parts[1]
            self.assertNotIn(top, NEVER_TRACKED_DIRS, name)

    def test_no_scene_file_is_tracked(self):
        for name in self.names:
            self.assertFalse(name.lower().endswith(SCENE_SUFFIXES), name)

    def test_no_tracked_file_is_large(self):
        for name in self.names:
            size = (REPO / name).stat().st_size
            self.assertLessEqual(size, MAX_TRACKED_BYTES, "%s is %d bytes" % (name, size))

    def test_the_ignore_file_names_what_install_writes(self):
        ignore = (HARNESS / ".gitignore").read_text(encoding="utf-8").splitlines()
        for entry in ("/bx/", "/venv/", "/out/", "/body_records/*", "/measured/*/runs/derived_mjcf/"):
            self.assertIn(entry, ignore, entry)

    def test_the_ignore_file_keeps_every_shipped_body_record_tracked(self):
        ignore = (HARNESS / ".gitignore").read_text(encoding="utf-8").splitlines()
        shipped = sorted(n for n in self.names if n.startswith("harness/body_records/"))
        self.assertEqual(len(shipped), 3, shipped)
        for name in shipped:
            self.assertIn("!/" + name[len("harness/"):], ignore, name)


if __name__ == "__main__":
    unittest.main()
