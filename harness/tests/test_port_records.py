# SPDX-License-Identifier: AGPL-3.0-only
"""python -m cerulion_harness.port_records: what it leaves in the --work folder. Standard library only, no
simulator: the probe that starts the simulator is replaced by one that writes into the folder it is given as a
probe does (an episode folder and its scoring folder), so what is tested is what the command removes at the end."""
import contextlib
import io
import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import port_records  # noqa: E402


def files_under(root):
    """Sorted relative paths of every file and folder under root."""
    out = []
    for dp, dn, fn in os.walk(root):
        out += [os.path.relpath(os.path.join(dp, n), root) for n in dn + fn]
    return sorted(out)


class WorkFolder(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = self.tmp.name
        sim = os.path.join(root, "sim", "python")
        os.makedirs(os.path.dirname(sim))
        open(sim, "w").close()
        tree = os.path.join(root, "rcp")
        os.makedirs(tree)
        self.given = []
        cwd, path = os.getcwd(), list(sys.path)
        self.addCleanup(os.chdir, cwd)
        self.addCleanup(setattr, sys, "path", path)
        for patch in (mock.patch.object(port_records.layout, "SIM_PYTHON", sim),
                      mock.patch.object(port_records.layout, "RCP_TREE", tree),
                      mock.patch.object(port_records.layout, "RECORDS", os.path.join(root, "records")),
                      mock.patch.dict(os.environ)):
            patch.start()
            self.addCleanup(patch.stop)
        self.addCleanup(self.tmp.cleanup)

    def probe(self, probe, work, wait_s):
        self.given.append(work)
        ep_dir = tempfile.mkdtemp(prefix="p", dir=work)
        scoring = os.path.join(work, "_scoring", os.path.basename(ep_dir))
        os.makedirs(scoring)
        for folder in (ep_dir, scoring):
            with open(os.path.join(folder, "episode.json"), "w", encoding="utf-8") as f:
                f.write("{}")
        return {"body_sha1": "0" * 40}, "1" * 40

    def run_main(self, work, probe=None):
        with mock.patch.object(port_records, "identity_here", probe or self.probe), \
                contextlib.redirect_stdout(io.StringIO()):
            return port_records.main(["--probe", "--work", work])

    def test_what_was_in_the_work_folder_stays_and_only_the_run_folder_goes(self):
        work = os.path.join(self.tmp.name, "work")
        os.makedirs(os.path.join(work, "mine"))
        for rel in ("keep.txt", os.path.join("mine", "data.txt")):
            with open(os.path.join(work, rel), "w", encoding="utf-8") as f:
                f.write("a file the command did not make")
        self.assertEqual(self.run_main(work), 0)
        self.assertEqual(files_under(work), ["keep.txt", "mine", os.path.join("mine", "data.txt")])
        self.assertEqual(len(self.given), len(port_records.PROBES))

    def test_a_work_folder_the_run_made_is_removed_when_it_is_empty(self):
        work = os.path.join(self.tmp.name, "made", "here")
        self.assertEqual(self.run_main(work), 0)
        self.assertFalse(os.path.exists(work))
        self.assertTrue(os.path.isdir(os.path.dirname(work)))

    def test_a_probe_that_fails_still_leaves_the_files_it_did_not_make(self):
        work = os.path.join(self.tmp.name, "work")
        os.makedirs(work)
        with open(os.path.join(work, "keep.txt"), "w", encoding="utf-8") as f:
            f.write("a file the command did not make")

        def failing(probe, folder, wait_s):
            self.probe(probe, folder, wait_s)
            raise RuntimeError("the simulator did not start: a test stand-in")

        with self.assertRaises(RuntimeError):
            self.run_main(work, failing)
        self.assertEqual(files_under(work), ["keep.txt"])


if __name__ == "__main__":
    unittest.main()
