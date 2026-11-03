#!/usr/bin/env python3
"""Oracle tests for two pieces of release text that a reader acts on.

1. The Android app copy step in docs/android.md (`Graph nodes in an app`):
   the example `Cargo.toml` already has a `[patch.crates-io]` header, so the
   reader must add entries under it and must not paste a second header.
2. The CHANGELOG entry for the empty trust-store MAC key: it states the fix
   once, and it states no key length the code does not enforce.

Every expected string is written out by hand here. The root `Cargo.toml` and
`workspace.rs` are read as the facts the documents describe, not as expected
values for the documents' own wording.
"""
import pathlib
import re
import sys
import tomllib
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
ANDROID_DOC = ROOT / "docs" / "android.md"
CHANGELOG = ROOT / "CHANGELOG.md"
ROOT_MANIFEST = ROOT / "Cargo.toml"
WORKSPACE_RS = ROOT / "crates" / "cerulion_cli_engine" / "src" / "workspace.rs"


def squash(text):
    """Collapse every run of whitespace so a line wrap never decides a result."""
    return " ".join(text.split())


def app_runtime_example(doc):
    """The first toml fence under `Graph nodes in an app` that has a patch header."""
    section = doc.split("### Graph nodes in an app", 1)[1]
    for fence in re.findall(r"```toml\n(.*?)```", section, flags=re.DOTALL):
        if "[patch.crates-io]" in fence:
            return fence
    raise AssertionError("no toml example with a [patch.crates-io] header")


def copy_step_one(doc):
    """Step 1 of the numbered list of things an app copies, as one line."""
    start = doc.index("1. The iceoryx2 `[patch.crates-io]`")
    end = doc.index("\n2. The Java facade", start)
    return squash(doc[start:end])


def iceoryx2_entry_lines(patch_table):
    return [
        f'{name} = {{ git = "{spec["git"]}", rev = "{spec["rev"]}" }}'
        for name, spec in patch_table.items()
        if name.startswith("iceoryx2")
    ]


def unreleased_bullets(changelog):
    section = changelog.split("## [Unreleased]", 1)[1].split("\n## [", 1)[0]
    return [line for line in section.splitlines() if line.startswith("- ")]


class AppPatchCopyStep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = ANDROID_DOC.read_text(encoding="utf-8")
        cls.example = app_runtime_example(cls.doc)
        root = tomllib.loads(ROOT_MANIFEST.read_text(encoding="utf-8"))
        cls.root_patch = root["patch"]["crates-io"]
        cls.entries = iceoryx2_entry_lines(cls.root_patch)
        cls.step = copy_step_one(cls.doc)

    def test_the_example_already_has_the_patch_header(self):
        self.assertEqual(self.example.count("[patch.crates-io]"), 1)
        parsed = tomllib.loads(self.example)
        self.assertEqual(
            sorted(parsed["patch"]["crates-io"]),
            ["cerulion_core", "native_ros2_messages"],
        )

    def test_the_root_table_has_iceoryx2_entries_to_add(self):
        self.assertGreaterEqual(len(self.entries), 20)

    def test_adding_the_entries_under_the_existing_header_parses(self):
        merged = self.example.rstrip("\n") + "\n" + "\n".join(self.entries) + "\n"
        table = tomllib.loads(merged)["patch"]["crates-io"]
        self.assertIn("cerulion_core", table)
        self.assertIn("native_ros2_messages", table)
        self.assertEqual(
            sorted(k for k in table if k.startswith("iceoryx2")),
            sorted(k for k in self.root_patch if k.startswith("iceoryx2")),
        )

    def test_pasting_the_whole_table_with_its_header_does_not_parse(self):
        pasted = (
            self.example.rstrip("\n")
            + "\n\n[patch.crates-io]\n"
            + "\n".join(self.entries)
            + "\n"
        )
        with self.assertRaises(tomllib.TOMLDecodeError):
            tomllib.loads(pasted)

    def test_the_root_table_holds_more_than_the_iceoryx2_family(self):
        # The step tells the reader to leave the other entries out. If the root
        # table ever holds only the family, that sentence needs to go.
        others = sorted(k for k in self.root_patch if not k.startswith("iceoryx2"))
        self.assertIn("re_grpc_server", others)

    def test_a_generated_node_workspace_patches_the_same_family(self):
        # `cerulion workspace create` is described as writing these entries and
        # no others; its list must be the root table's iceoryx2 family.
        source = WORKSPACE_RS.read_text(encoding="utf-8")
        block = re.search(
            r"const ICEORYX2_FORK_CRATES: &\[&str\] = &\[(.*?)\];", source, re.DOTALL
        )
        self.assertIsNotNone(block)
        generated = sorted(re.findall(r'"([^"]+)"', block.group(1)))
        family = sorted(k for k in self.root_patch if k.startswith("iceoryx2"))
        self.assertEqual(generated, family)

    def test_the_step_says_entries_under_the_existing_header(self):
        self.assertIn("Add the `iceoryx2*` entries", self.step)
        self.assertIn(
            "under the `[patch.crates-io]` header this file already has", self.step
        )
        self.assertIn("a second header makes the file fail to parse", self.step)

    def test_the_step_leaves_out_the_other_root_entries(self):
        self.assertIn("Leave out the table's other entries", self.step)
        self.assertIn("`re_grpc_server`", self.step)

    def test_the_step_does_not_say_complete_block_or_same_block(self):
        self.assertNotIn("complete block", self.step)
        self.assertNotIn("below the entries above", self.step)
        self.assertNotIn("carries the same block", self.step)


class MacKeyChangelogEntry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        text = CHANGELOG.read_text(encoding="utf-8")
        cls.bullets = unreleased_bullets(text)
        cls.about_key = [b for b in cls.bullets if "MAC key" in b]

    def test_the_empty_key_fix_appears_once(self):
        empty_key = [b for b in self.about_key if "empty" in b]
        self.assertEqual(len(empty_key), 1, empty_key)

    def test_the_one_entry_says_the_daemon_used_to_load_with_an_empty_key(self):
        entry = squash(self.about_key[0])
        self.assertIn("`cerulion-remoted` used to load with an empty key", entry)
        self.assertIn("refuses to start", entry)

    def test_no_entry_claims_a_key_length(self):
        # `store_mac_key` returns a `Vec<u8>` and the MAC takes any length.
        for bullet in self.about_key:
            self.assertNotIn("32-byte", bullet)
            self.assertNotIn("32 byte", bullet)


if __name__ == "__main__":
    sys.exit(unittest.main(verbosity=2))
