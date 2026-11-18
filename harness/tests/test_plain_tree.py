# SPDX-License-Identifier: AGPL-3.0-only
"""The plain setup's code tree, which run-cell builds once with measured/assemble.py and reuses while its sources
are unchanged. Standard library only. The measured folder here is a hand-written fixture with a small builder of
the same interface (tree_files, ABSENT_LIST, AssemblyError, assemble_plain), so what is tested is when the tree
is built, where it goes and what a refusal leaves, not the measured builder's own rules."""
import hashlib
import os
import pathlib
import sys
import tempfile
import unittest

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
        self.dest = os.path.join(self.tmp.name, "bx", "trees", "plain")
        write(self.measured, "assemble.py", BUILDER)
        write(self.measured, "rcp/a.py", "a = 1\n")
        write(self.measured, "rcp/pkg/b.py", "b = 'rcp'\n")
        write(self.measured, "rcp/pkg/c.py", "c = 'rcp only'\n")
        write(self.measured, "plain-overlay/pkg/b.py", "b = 'plain'\n")
        write(self.measured, "plain-absent.txt", "pkg/c.py\n")

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, dest=None):
        return layout.plain_tree(self.measured, dest or self.dest)

    def test_the_tree_is_what_the_builder_wrote_plus_the_stamp(self):
        self.assertEqual(self.build(), self.dest)
        got = listing(self.dest)
        self.assertTrue(got.pop(layout.STAMP))
        self.assertEqual(got, {"a.py": sha("a = 1\n"), "pkg/b.py": sha("b = 'plain'\n")})

    def test_a_built_tree_is_reused_while_its_sources_are_unchanged(self):
        self.build()
        write(self.dest, "kept.txt", "only a reuse keeps this file")
        self.build()
        self.assertTrue(os.path.exists(os.path.join(self.dest, "kept.txt")))

    def test_any_changed_source_builds_the_tree_again(self):
        for rel, text in (("plain-overlay/pkg/b.py", "b = 'plain, changed'\n"), ("rcp/a.py", "a = 2\n"),
                          ("plain-absent.txt", "pkg/c.py\n\n"), ("assemble.py", BUILDER + "\n# changed\n")):
            self.build()
            write(self.dest, "kept.txt", "only a reuse keeps this file")
            write(self.measured, rel, text)
            self.build()
            self.assertFalse(os.path.exists(os.path.join(self.dest, "kept.txt")), rel)

    def test_run_output_inside_the_rcp_tree_does_not_count_as_a_change(self):
        self.build()
        write(self.dest, "kept.txt", "only a reuse keeps this file")
        write(self.measured, "rcp/runs/derived_mjcf/robot.xml", "<a derived model/>")
        write(self.measured, "rcp/pkg/__pycache__/b.cpython-312.pyc", "bytecode")
        self.build()
        self.assertTrue(os.path.exists(os.path.join(self.dest, "kept.txt")))

    def test_two_builds_are_byte_identical(self):
        one = listing(self.build(os.path.join(self.tmp.name, "one")))
        two = listing(self.build(os.path.join(self.tmp.name, "two")))
        self.assertEqual(one, two)

    def test_a_refusal_is_a_value_error_and_leaves_the_old_tree_and_no_partial_one(self):
        self.build()
        before = listing(self.dest)
        write(self.measured, "plain-absent.txt", "refuse\n")
        with self.assertRaises(ValueError) as ctx:
            self.build()
        self.assertIn("the absent list says refuse", str(ctx.exception))
        self.assertEqual(listing(self.dest), before)
        self.assertEqual(sorted(os.listdir(os.path.dirname(self.dest))), ["plain"])

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
