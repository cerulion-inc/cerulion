# SPDX-License-Identifier: AGPL-3.0-only
"""The shipped measured code against its provenance files, and the plain arm's assembly.

cerulion_harness.verify and measured/assemble.py are loaded from their paths. The oracles are written here: the two
snapshot ids the results package cites, the file counts of the ledger, the paths of the plain overlay and its absent
list, and the hashes of the two texts the model reads unchanged from the measured trees. Every tampering test works on
a copy of the folder in a temporary directory.
"""

import hashlib
import importlib.util
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
MEASURED = HARNESS / "measured"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


V = load("cerulion_harness_verify_under_test", HARNESS / "src" / "cerulion_harness" / "verify.py")
A = load("cerulion_harness_assemble_under_test", MEASURED / "assemble.py")

RCP_SNAPSHOT = "sha256:7cb9f322c10f6fa7e64a3f8da7c22c67d67280eca436a03d2d87683496b4d5b8"
PLAIN_SNAPSHOT = "sha256:b73c548a1f0436f52ce41cffc7d4a4a2784e5d6170e2dd2c757c9cabdf146637"
OVERLAY = sorted([
    "rcpx/bounds_book.py", "rcpx/control_expect.py", "rcpx/control_hold.py", "rcpx/control_hold_close.py",
    "rcpx/control_sense.py", "rcpx/r2/device_bounds.py", "rcpx/r2/epochs_book.py", "rcpx/r2/probe.py",
])
ABSENT = ["rcpx/bounds_book_read.py"]
# The operator brief and the practices text reach the model as they are: both are the measured bytes.
MODEL_TEXTS = {
    "bench/operators/operator_brief.md": "92ea6c6bc2f614aed15296f2ed041fd69011c9d496a6ad3bf7de209a823e3cfd",
    "rcpx/practices.md": "829d7e7f810488c42037a9a884bf38cd2e7da373ec074a1010d276491dde4b81",
}
FACTS = {
    "rcp": {"snapshot": RCP_SNAPSHOT, "measured_files": 645, "shipped_files": 321, "left_out": 325,
            "shipped_by_status": {"code_edited": 1, "comments_and_docstrings": 315, "identical": 2,
                                  "strings_edited": 2, "text_edited": 1}},
    "plain": {"snapshot": PLAIN_SNAPSHOT, "measured_files": 643, "shipped_files": 320, "left_out": 324,
              "shipped_by_status": {"code_edited": 1, "comments_and_docstrings": 314, "identical": 2,
                                    "strings_edited": 2, "text_edited": 1}},
}


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def listing(root):
    """[(relative path, sha256)] of every file under root, sorted: an oracle walk that does not use assemble.py."""
    root = pathlib.Path(root)
    return sorted((p.relative_to(root).as_posix(), sha(p)) for p in root.rglob("*") if p.is_file())


class Copy:
    """A copy of the harness folder's measured/ and provenance/ in a temporary directory."""

    def __enter__(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="harness-verify-test-")
        self.home = pathlib.Path(self.tmp.name) / "harness"
        for part in ("measured", "provenance"):
            shutil.copytree(HARNESS / part, self.home / part, ignore=shutil.ignore_patterns("__pycache__"))
        return self.home

    def __exit__(self, *exc):
        self.tmp.cleanup()


class ShippedTree(unittest.TestCase):
    def test_the_shipped_tree_matches_its_provenance(self):
        problems, facts = V.verify(str(HARNESS))
        self.assertEqual(problems, [])
        self.assertEqual(facts, FACTS)

    def test_the_manifests_hash_to_the_cited_snapshot_ids(self):
        self.assertEqual("sha256:" + sha(HARNESS / "provenance" / "measured-rcp.manifest"), RCP_SNAPSHOT)
        self.assertEqual("sha256:" + sha(HARNESS / "provenance" / "measured-plain.manifest"), PLAIN_SNAPSHOT)
        self.assertEqual(V.SNAPSHOTS, {"rcp": RCP_SNAPSHOT, "plain": PLAIN_SNAPSHOT})

    def test_the_texts_the_model_reads_are_the_measured_bytes(self):
        for rel, want in MODEL_TEXTS.items():
            self.assertEqual(sha(MEASURED / "rcp" / rel), want, rel)

    def test_the_overlay_and_absent_list_are_the_arm_difference(self):
        overlay = sorted(p.relative_to(MEASURED / "plain-overlay").as_posix()
                         for p in (MEASURED / "plain-overlay").rglob("*") if p.is_file())
        self.assertEqual(overlay, OVERLAY)
        self.assertEqual(A.absent_paths(str(MEASURED)), ABSENT)
        for rel in OVERLAY + ABSENT:
            self.assertTrue((MEASURED / "rcp" / rel).is_file(), rel)
        for rel in OVERLAY:
            self.assertNotEqual(sha(MEASURED / "plain-overlay" / rel), sha(MEASURED / "rcp" / rel), rel)


class Assembly(unittest.TestCase):
    def test_the_plain_tree_is_rcp_with_the_overlay_and_without_the_absent_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = os.path.join(tmp, "plain")
            written = A.assemble_plain(str(MEASURED), dest)
            rcp = dict(listing(MEASURED / "rcp"))
            got = dict(listing(dest))
            self.assertEqual(sorted(got), sorted(set(rcp) - set(ABSENT)))
            self.assertEqual(written, sorted(got))
            self.assertEqual(len(got), 320)
            for rel, digest in got.items():
                source = MEASURED / ("plain-overlay" if rel in OVERLAY else "rcp") / rel
                self.assertEqual(digest, sha(source), rel)
            for rel in ("rcpx/eyes_service/start.sh", "rcpx/eyes_service/stop.sh"):
                self.assertTrue(os.access(os.path.join(dest, rel), os.X_OK), rel)

    def test_two_assemblies_are_identical(self):
        with tempfile.TemporaryDirectory() as tmp:
            one, two = os.path.join(tmp, "one"), os.path.join(tmp, "two")
            A.assemble_plain(str(MEASURED), one)
            A.assemble_plain(str(MEASURED), two)
            self.assertEqual(listing(one), listing(two))

    def test_what_a_run_leaves_in_the_tree_is_neither_copied_nor_checked(self):
        with Copy() as home:
            rcp = home / "measured" / "rcp"
            for rel in ("rcpx/__pycache__/server.cpython-312.pyc", "bench/stray.pyc", ".DS_Store",
                        "runs/derived_mjcf/panda.xml", "runs/benchmarks/libero_10/x.json"):
                (rcp / rel).parent.mkdir(parents=True, exist_ok=True)
                (rcp / rel).write_bytes(b"run output\n")
            problems, facts = V.verify(str(home))
            self.assertEqual(problems, [])
            self.assertEqual(facts, FACTS)
            dest = home / "plain"
            A.assemble_plain(str(home / "measured"), str(dest))
            self.assertFalse((dest / "runs" / "derived_mjcf").exists())
            self.assertFalse((dest / "rcpx" / "__pycache__").exists())
            self.assertFalse((dest / ".DS_Store").exists())

    def refused(self, measured, dest, words):
        with self.assertRaises(A.AssemblyError) as caught:
            A.assemble_plain(str(measured), str(dest))
        self.assertIn(words, str(caught.exception))

    def test_a_destination_that_holds_anything_is_refused_and_left_as_it_was(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = pathlib.Path(tmp) / "plain"
            dest.mkdir()
            (dest / "mine.txt").write_text("keep\n")
            self.refused(MEASURED, dest, "is not an empty folder")
            self.assertEqual([p.name for p in dest.iterdir()], ["mine.txt"])
            self.assertEqual((dest / "mine.txt").read_text(), "keep\n")

    def test_a_bad_absent_list_or_overlay_writes_nothing(self):
        cases = [
            ("rcpx/no_such_file.py\n", None, "which is not a file of rcp/"),
            ("../rcp/rcpx/server.py\n", None, "is not a plain relative path"),
            ("rcpx/bounds_book_read.py\nrcpx/bounds_book_read.py\n", None, "names a path twice"),
            ("rcpx/bounds_book_read.py\n", "rcpx/new_file.py", "replaces no file of rcp/"),
            ("rcpx/bounds_book_read.py\nrcpx/probe_not_here.py\n", None, "rcpx/probe_not_here.py"),
            ("rcpx/bounds_book_read.py\nrcpx/r2/probe.py\n", None, "is also named in"),
        ]
        for absent, extra, words in cases:
            with self.subTest(absent=absent, extra=extra), Copy() as home:
                (home / "measured" / "plain-absent.txt").write_text(absent)
                if extra:
                    (home / "measured" / "plain-overlay" / extra).write_text("x = 1\n")
                dest = home / "plain"
                self.refused(home / "measured", dest, words)
                self.assertFalse(dest.exists())

    def test_a_symbolic_link_in_the_tree_is_refused(self):
        with Copy() as home:
            os.symlink("server.py", str(home / "measured" / "rcp" / "rcpx" / "link.py"))
            dest = home / "plain"
            self.refused(home / "measured", dest, "rcpx/link.py is a symbolic link")
            self.assertFalse(dest.exists())

    def test_the_command_line(self):
        script = str(MEASURED / "assemble.py")
        with tempfile.TemporaryDirectory() as tmp:
            dest = os.path.join(tmp, "plain")
            run = subprocess.run([sys.executable, "-I", "-B", script, dest], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(run.stdout, "assemble: wrote 320 files of the plain tree to %s\n" % dest)
            again = subprocess.run([sys.executable, "-I", "-B", script, dest], capture_output=True, text=True)
            self.assertEqual(again.returncode, 1)
            self.assertIn("is not an empty folder", again.stderr)
            usage = subprocess.run([sys.executable, "-I", "-B", script], capture_output=True, text=True)
            self.assertEqual(usage.returncode, 2)
            self.assertIn("usage:", usage.stderr)


class Tampering(unittest.TestCase):
    def check(self, change, *expected):
        with Copy() as home:
            change(home)
            problems, _facts = V.verify(str(home))
        for words in expected:
            self.assertTrue(any(words in p for p in problems), "%r not in %r" % (words, problems))
        return problems

    def test_a_changed_byte_in_a_shipped_file(self):
        def change(home):
            path = home / "measured" / "rcp" / "rcpx" / "server.py"
            data = bytearray(path.read_bytes())
            data[-2] ^= 1
            path.write_bytes(bytes(data))
        problems = self.check(change, "rcp: rcpx/server.py differs from the ledger",
                              "plain: rcpx/server.py differs from the ledger")
        self.assertEqual(len(problems), 2)

    def test_a_changed_overlay_file_fails_the_plain_arm_only(self):
        def change(home):
            with open(home / "measured" / "plain-overlay" / "rcpx" / "control_hold.py", "a") as f:
                f.write("\n")
        problems = self.check(change, "plain: rcpx/control_hold.py differs from the ledger")
        self.assertEqual(len(problems), 1)

    def test_an_added_and_a_removed_file(self):
        def change(home):
            (home / "measured" / "rcp" / "rcpx" / "extra.py").write_text("x = 1\n")
            (home / "measured" / "rcp" / "rcpx" / "surfaces.py").unlink()
        self.check(change, "rcp: rcpx/extra.py is not in the ledger", "rcp: rcpx/surfaces.py is missing",
                   "plain: rcpx/extra.py is not in the ledger", "plain: rcpx/surfaces.py is missing")

    def test_a_changed_manifest(self):
        def change(home):
            path = home / "provenance" / "measured-rcp.manifest"
            path.write_text(path.read_text().replace("bench/accident.py", "bench/accident2.py"))
        self.check(change, "rcp: the measured manifest hashes to sha256:",
                   "rcp: dropped file bench/accident.py is not in the measured manifest",
                   "rcp: measured file bench/accident2.py is neither shipped nor dropped")

    def test_a_malformed_manifest_line(self):
        def change(home):
            with open(home / "provenance" / "measured-plain.manifest", "a") as f:
                f.write("not a manifest line\n")
        self.check(change, "plain: the measured manifest hashes to", "plain: manifest line 644 is not 'SHA256  path'")

    def ledger(self, edit):
        def change(home):
            path = home / "provenance" / "ledger.json"
            data = json.loads(path.read_text())
            edit(data)
            path.write_text(json.dumps(data))
        return change

    def row(self, data, arm, rel):
        return next(r for r in data["arms"][arm]["files"] if r["path"] == rel)

    def test_a_ledger_measured_hash_that_is_not_the_manifests(self):
        def edit(data):
            self.row(data, "rcp", "rcpx/server.py")["measured_sha256"] = "0" * 64
        self.check(self.ledger(edit), "rcp: rcpx/server.py: the ledger's measured hash is not the manifest's")

    def test_an_identical_row_whose_hashes_differ(self):
        def edit(data):
            self.row(data, "rcp", "rcpx/practices.md")["measured_sha256"] = "0" * 64
        self.check(self.ledger(edit), "rcp: rcpx/practices.md is marked identical but its two hashes differ")

    def test_an_unknown_status(self):
        def edit(data):
            self.row(data, "plain", "rcpx/server.py")["status"] = "licence_line_only"
        self.check(self.ledger(edit), "plain: rcpx/server.py: unknown status 'licence_line_only'")

    def test_a_declared_text_that_is_not_in_the_shipped_file(self):
        def edit(data):
            self.row(data, "rcp", "rcpx/r2/core.py")["strings_changed"][0]["public"] = "a string nobody wrote"
        self.check(self.ledger(edit), "rcp: rcpx/r2/core.py: a text the ledger declares is not in the shipped file")

    def test_an_edited_row_that_lists_no_edit(self):
        def edit(data):
            self.row(data, "rcp", "rcpx/mcp_stdio.py")["code_changed"]["edits"] = []
            self.row(data, "plain", "rcpx/r2/core.py")["strings_changed"] = []
        self.check(self.ledger(edit), "rcp: rcpx/mcp_stdio.py is marked code_edited but lists no edit",
                   "plain: rcpx/r2/core.py is marked strings_edited but lists no string")

    def test_a_shipped_file_that_is_also_dropped_and_a_repeated_row(self):
        def edit(data):
            data["arms"]["rcp"]["dropped_files"].append("rcpx/server.py")
            rows = data["arms"]["plain"]["files"]
            rows.append(dict(rows[0]))
        self.check(self.ledger(edit), "rcp: rcpx/server.py is both shipped and dropped",
                   "plain: bench/__init__.py has two ledger rows")

    def test_a_row_outside_the_manifest(self):
        def edit(data):
            row = dict(self.row(data, "rcp", "rcpx/server.py"), path="rcpx/not_measured.py")
            data["arms"]["rcp"]["files"].append(row)
        self.check(self.ledger(edit), "rcp: rcpx/not_measured.py is in the ledger but not in the measured manifest",
                   "rcp: rcpx/not_measured.py is missing")

    def test_an_absent_list_the_tree_cannot_meet(self):
        def change(home):
            (home / "measured" / "plain-absent.txt").write_text("rcpx/gone.py\n")
        problems = self.check(change, "plain: the tree cannot be assembled: plain-absent.txt names rcpx/gone.py")
        self.assertFalse(any(p.startswith("rcp:") for p in problems), problems)

    def test_unreadable_provenance(self):
        for name in ("ledger.json", "measured-rcp.manifest"):
            with self.subTest(name=name):
                def change(home, name=name):
                    (home / "provenance" / name).unlink()
                problems = self.check(change, "cannot read")
                if name == "ledger.json":
                    self.assertEqual(len(problems), 1)

    def test_two_runs_agree(self):
        self.assertEqual(V.verify(str(HARNESS)), V.verify(str(HARNESS)))


if __name__ == "__main__":
    unittest.main()
