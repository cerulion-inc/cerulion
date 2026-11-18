# SPDX-License-Identifier: AGPL-3.0-only
"""The plain setup's code tree, which run-cell builds once with measured/assemble.py and reuses while its sources
are unchanged. Standard library only. The measured folder here is a hand-written fixture with a small builder of
the same interface (tree_files, ABSENT_LIST, AssemblyError, assemble_plain), so what is tested is when the tree
is built, where it goes, what a refusal leaves and that a tree in place is never changed, not the measured
builder's own rules."""
import hashlib
import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import layout  # noqa: E402

BUILDER = '''
import os, shutil
ABSENT_LIST = "plain-absent.txt"
class AssemblyError(Exception):
    pass
def tree_files(root):
    if not os.path.isdir(root):
        raise AssemblyError("%s is not a folder" % root)
    out = []
    for dp, dn, fn in os.walk(root):
        dn[:] = [d for d in dn if d not in ("__pycache__", "derived_mjcf")]
        out += [os.path.relpath(os.path.join(dp, n), root).replace(os.sep, "/") for n in fn]
    return sorted(out)
def assemble_plain(measured, dest):
    if os.listdir(dest):
        raise AssemblyError("%s is not an empty folder" % dest)
    with open(os.path.join(measured, ABSENT_LIST)) as f:
        absent = [line.strip() for line in f if line.strip()]
    if "refuse" in absent:
        raise AssemblyError("the absent list says refuse")
    overlay = tree_files(os.path.join(measured, "plain-overlay"))
    for rel in tree_files(os.path.join(measured, "rcp")):
        if rel in absent:
            continue
        src = os.path.join(measured, "plain-overlay" if rel in overlay else "rcp", rel)
        os.makedirs(os.path.dirname(os.path.join(dest, rel)), exist_ok=True)
        shutil.copyfile(src, os.path.join(dest, rel))
'''


def write(root, rel, text):
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def listing(root):
    """{relative path: SHA-256 of its bytes} of every file under root."""
    out = {}
    for dp, _, fn in os.walk(root):
        for name in fn:
            path = os.path.join(dp, name)
            with open(path, "rb") as f:
                out[os.path.relpath(path, root).replace(os.sep, "/")] = hashlib.sha256(f.read()).hexdigest()
    return out


def sha(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


class PlainTree(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.measured = os.path.join(self.tmp.name, "measured")
        self.trees = os.path.join(self.tmp.name, "bx", "trees", "plain")
        write(self.measured, "assemble.py", BUILDER)
        write(self.measured, "rcp/a.py", "a = 1\n")
        write(self.measured, "rcp/pkg/b.py", "b = 'rcp'\n")
        write(self.measured, "rcp/pkg/c.py", "c = 'rcp only'\n")
        write(self.measured, "plain-overlay/pkg/b.py", "b = 'plain'\n")
        write(self.measured, "plain-absent.txt", "pkg/c.py\n")

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, trees=None):
        return layout.plain_tree(self.measured, trees or self.trees)

    def test_the_tree_is_what_the_builder_wrote_plus_the_stamp(self):
        tree = self.build()
        self.assertEqual(os.path.dirname(tree), self.trees)
        self.assertRegex(os.path.basename(tree), r"^[0-9a-f]{16}$")
        got = listing(tree)
        self.assertTrue(got.pop(layout.STAMP))
        self.assertEqual(got, {"a.py": sha("a = 1\n"), "pkg/b.py": sha("b = 'plain'\n")})
        with open(os.path.join(tree, layout.STAMP), encoding="utf-8") as f:
            stamp = f.read().strip()
        self.assertRegex(stamp, r"^[0-9a-f]{64}$")
        self.assertTrue(stamp.startswith(os.path.basename(tree)))

    def test_a_built_tree_is_reused_while_its_sources_are_unchanged(self):
        tree = self.build()
        write(tree, "kept.txt", "only a reuse keeps this file")
        self.assertEqual(self.build(), tree)
        self.assertTrue(os.path.exists(os.path.join(tree, "kept.txt")))

    def test_any_changed_source_builds_a_new_tree_and_leaves_the_old_one_as_it_was(self):
        for rel, text in (("plain-overlay/pkg/b.py", "b = 'plain, changed'\n"), ("rcp/a.py", "a = 2\n"),
                          ("plain-absent.txt", "pkg/c.py\n\n"), ("assemble.py", BUILDER + "\n# changed\n")):
            old = self.build()
            write(old, "kept.txt", "an episode runs from this tree")
            before = listing(old)
            write(self.measured, rel, text)
            new = self.build()
            self.assertNotEqual(new, old, rel)
            self.assertEqual(listing(old), before, rel)
            self.assertFalse(os.path.exists(os.path.join(new, "kept.txt")), rel)

    def test_run_output_inside_the_rcp_tree_does_not_count_as_a_change(self):
        tree = self.build()
        write(tree, "kept.txt", "only a reuse keeps this file")
        write(self.measured, "rcp/runs/derived_mjcf/robot.xml", "<a derived model/>")
        write(self.measured, "rcp/pkg/__pycache__/b.cpython-312.pyc", "bytecode")
        self.assertEqual(self.build(), tree)
        self.assertTrue(os.path.exists(os.path.join(tree, "kept.txt")))

    def test_two_builds_are_byte_identical(self):
        one = self.build(os.path.join(self.tmp.name, "one"))
        two = self.build(os.path.join(self.tmp.name, "two"))
        self.assertEqual(os.path.basename(one), os.path.basename(two))
        self.assertEqual(listing(one), listing(two))

    def test_two_runs_that_build_at_once_both_use_the_first_tree_and_it_is_never_replaced(self):
        # While this run builds its copy, another run builds the same tree, renames it into place and starts an
        # episode in it. This run then finds the place taken by that tree, uses it, and drops its own copy.
        real = layout.assembler(self.measured)
        first = {}
        test = self

        class Racing:
            ABSENT_LIST, AssemblyError, tree_files = real.ABSENT_LIST, real.AssemblyError, real.tree_files

            @staticmethod
            def assemble_plain(measured, dest):
                with mock.patch.object(layout, "assembler", lambda m: real):
                    first["tree"] = layout.plain_tree(test.measured, test.trees)
                write(first["tree"], "in-use.txt", "an episode runs from this tree")
                first["listing"] = listing(first["tree"])
                real.assemble_plain(measured, dest)

        with mock.patch.object(layout, "assembler", lambda m: Racing):
            got = self.build()
        self.assertEqual(got, first["tree"])
        self.assertEqual(listing(got), first["listing"])
        self.assertEqual(os.listdir(self.trees), [os.path.basename(got)])

    def test_a_folder_in_the_place_of_the_tree_that_is_not_it_is_refused_and_left_alone(self):
        tree = self.build()
        os.unlink(os.path.join(tree, layout.STAMP))
        write(tree, "other.txt", "not a tree this builder made")
        before = listing(tree)
        with self.assertRaises(ValueError) as ctx:
            self.build()
        self.assertIn("not the plain tree of these sources", str(ctx.exception))
        self.assertEqual(listing(tree), before)
        self.assertEqual(os.listdir(self.trees), [os.path.basename(tree)])

    def test_a_refusal_is_a_value_error_and_leaves_the_old_tree_and_no_partial_one(self):
        tree = self.build()
        before = listing(tree)
        write(self.measured, "plain-absent.txt", "refuse\n")
        with self.assertRaises(ValueError) as ctx:
            self.build()
        self.assertIn("the absent list says refuse", str(ctx.exception))
        self.assertEqual(listing(tree), before)
        self.assertEqual(os.listdir(self.trees), [os.path.basename(tree)])

    def test_a_missing_tree_is_a_value_error_from_the_builder(self):
        os.rename(os.path.join(self.measured, "plain-overlay"), os.path.join(self.measured, "elsewhere"))
        with self.assertRaises(ValueError):
            self.build()

    def test_a_missing_builder_is_named(self):
        os.unlink(os.path.join(self.measured, "assemble.py"))
        with self.assertRaises(FileNotFoundError) as ctx:
            self.build()
        self.assertIn("assemble.py", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
