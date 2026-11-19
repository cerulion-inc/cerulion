# SPDX-License-Identifier: AGPL-3.0-only
"""Every entry under `harness/` is a regular file or a folder. Standard library only.

The walk reads each entry's own kind with lstat: a symbolic link is reported and never followed, so a link to a
folder is not walked into. A symbolic link, a named pipe, a socket or a device file under the folder fails the
check. What git ignores there (what install.sh and a run write: bx/, venv/, Python's caches, a machine's own body
records) is not walked. The index is read as well, so an entry git records as a symbolic link or a submodule fails
even in a checkout that wrote it out as a plain file or an empty folder. Each rule also runs on a tree made here in
a temporary folder, with the entries it must find.
"""

import os
import pathlib
import shutil
import socket
import stat
import subprocess
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
REPO = HARNESS.parent

FILE, FOLDER = "a regular file", "a folder"
KINDS = ((stat.S_ISREG, FILE), (stat.S_ISDIR, FOLDER), (stat.S_ISLNK, "a symbolic link"),
         (stat.S_ISFIFO, "a named pipe"), (stat.S_ISSOCK, "a socket"), (stat.S_ISCHR, "a character device"),
         (stat.S_ISBLK, "a block device"))
# The index modes of a regular file, plain and executable; the others are named by INDEX_KINDS.
REGULAR_MODES = ("100644", "100755")
INDEX_KINDS = {"120000": "a symbolic link", "160000": "a submodule"}


def kind_of(mode):
    for test, name in KINDS:
        if test(mode):
            return name
    return "an entry of mode %o" % stat.S_IFMT(mode)


def walk(root, skipped=frozenset()):
    """[(path relative to root, kind)] of every entry under root, sorted by path. skipped: relative paths that are
    neither reported nor walked into."""
    seen = []

    def visit(folder, prefix):
        with os.scandir(folder) as entries:
            for entry in sorted(entries, key=lambda e: e.name):
                rel = prefix + entry.name
                if rel in skipped:
                    continue
                kind = kind_of(entry.stat(follow_symlinks=False).st_mode)
                seen.append((rel, kind))
                if kind == FOLDER:
                    visit(entry.path, rel + "/")

    visit(str(root), "")
    return sorted(seen)


def odd(entries):
    """The entries that are neither a regular file nor a folder."""
    return [(rel, kind) for rel, kind in entries if kind not in (FILE, FOLDER)]


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, stdout=subprocess.PIPE).stdout.decode("utf-8")


def ignored(listing, folder):
    """The paths, relative to folder, of `git ls-files -z --others --ignored --exclude-standard --directory`: an
    ignored folder is one entry, its path without the closing slash."""
    prefix = folder.rstrip("/") + "/"
    return {p[len(prefix):].rstrip("/") for p in listing.split("\0") if p.startswith(prefix)}


def odd_in_index(listing):
    """[(path, kind)] of the entries of `git ls-files -z -s` whose mode is not a regular file's, sorted."""
    found = []
    for record in listing.split("\0"):
        if not record:
            continue
        head, _, path = record.partition("\t")
        mode = head.split(" ")[0]
        if mode not in REGULAR_MODES:
            found.append((path, INDEX_KINDS.get(mode, "an entry of mode " + mode)))
    return sorted(found)


def check(repo, folder):
    """(every entry the walk saw, the odd ones on disk, the odd ones in the index) for folder (relative to repo) of
    a git checkout, with what git ignores there left out of the walk."""
    skipped = ignored(git(repo, "ls-files", "-z", "--others", "--ignored", "--exclude-standard", "--directory",
                          "--", folder), folder)
    entries = walk(pathlib.Path(repo) / folder, frozenset(skipped))
    return entries, odd(entries), odd_in_index(git(repo, "ls-files", "-z", "-s", "--", folder))


def in_a_checkout(path):
    if shutil.which("git") is None:
        return False
    done = subprocess.run(["git", "-C", str(path), "rev-parse", "--is-inside-work-tree"], stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL)
    return done.returncode == 0 and done.stdout.strip() == b"true"


class Walk(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = pathlib.Path(tmp.name) / "tree"
        (self.root / "sub").mkdir(parents=True)
        (self.root / "a.txt").write_text("a\n", encoding="utf-8")
        (self.root / "sub" / "b.txt").write_text("b\n", encoding="utf-8")

    def test_a_tree_of_files_and_folders_has_nothing_odd(self):
        entries = walk(self.root)
        self.assertEqual(entries, [("a.txt", FILE), ("sub", FOLDER), ("sub/b.txt", FILE)])
        self.assertEqual(odd(entries), [])

    def test_a_symbolic_link_and_a_named_pipe_are_both_found(self):
        os.symlink("../a.txt", str(self.root / "sub" / "link.txt"))
        os.mkfifo(str(self.root / "sub" / "pipe"))
        self.assertEqual(odd(walk(self.root)), [("sub/link.txt", "a symbolic link"), ("sub/pipe", "a named pipe")])

    def test_a_dangling_link_and_a_link_to_a_folder_are_found_and_not_followed(self):
        os.symlink("nowhere", str(self.root / "dangling"))
        os.symlink("sub", str(self.root / "to-sub"))
        entries = walk(self.root)
        self.assertEqual(odd(entries), [("dangling", "a symbolic link"), ("to-sub", "a symbolic link")])
        self.assertNotIn("to-sub/b.txt", [rel for rel, _ in entries])

    @unittest.skipUnless(hasattr(socket, "AF_UNIX"), "this system has no unix sockets")
    def test_a_socket_is_found(self):
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.addCleanup(listener.close)
        listener.bind(str(self.root / "sock"))
        self.assertEqual(odd(walk(self.root)), [("sock", "a socket")])

    def test_a_skipped_path_is_neither_reported_nor_walked(self):
        (self.root / "venv" / "bin").mkdir(parents=True)
        os.symlink("/usr/bin/python3", str(self.root / "venv" / "bin" / "python"))
        os.symlink("a.txt", str(self.root / "sub" / "skipped-link"))
        entries = walk(self.root, frozenset({"venv", "sub/skipped-link"}))
        self.assertEqual(entries, [("a.txt", FILE), ("sub", FOLDER), ("sub/b.txt", FILE)])


class GitListings(unittest.TestCase):
    def test_ignored_paths_are_read_relative_to_the_folder(self):
        listing = "\x00".join(["harness/bx/", "harness/venv/", "harness/src/cerulion_harness/__pycache__/",
                                "harness/body_records/0123.json", "other/harness/x", ""])
        self.assertEqual(ignored(listing, "harness"),
                         {"bx", "venv", "src/cerulion_harness/__pycache__", "body_records/0123.json"})
        self.assertEqual(ignored("", "harness"), set())

    def test_a_link_and_a_submodule_in_the_index_are_named(self):
        blob = "0" * 40
        listing = "\x00".join([f"100644 {blob} 0\tharness/a.txt", f"100755 {blob} 0\tharness/run.sh",
                                f"120000 {blob} 0\tharness/link", f"160000 {blob} 0\tharness/vendored", ""])
        self.assertEqual(odd_in_index(listing), [("harness/link", "a symbolic link"),
                                                 ("harness/vendored", "a submodule")])
        self.assertEqual(odd_in_index(f"100644 {blob} 0\tharness/a.txt\x00"), [])

    @unittest.skipUnless(shutil.which("git"), "git is not on PATH")
    def test_a_checkout_with_an_added_link_and_an_ignored_environment(self):
        # A repository made here: an added link and an untracked pipe are found, an ignored environment's links
        # are not.
        with tempfile.TemporaryDirectory() as tmp:
            repo = pathlib.Path(tmp)
            subprocess.run(["git", "init", "-q", str(repo)], check=True)
            folder = repo / "harness"
            (folder / "venv" / "bin").mkdir(parents=True)
            (folder / ".gitignore").write_text("/venv/\n", encoding="utf-8")
            (folder / "install.sh").write_text("#!/bin/sh\n", encoding="utf-8")
            os.symlink("/usr/bin/python3", str(folder / "venv" / "bin" / "python"))
            os.symlink("install.sh", str(folder / "again.sh"))
            os.mkfifo(str(folder / "pipe"))
            subprocess.run(["git", "-C", str(repo), "-c", "core.symlinks=true", "add", "harness/.gitignore",
                            "harness/install.sh", "harness/again.sh"], check=True)
            entries, on_disk, in_index = check(repo, "harness")
        self.assertIn(("install.sh", FILE), entries)
        self.assertNotIn("venv", [rel for rel, _ in entries])
        self.assertEqual(on_disk, [("again.sh", "a symbolic link"), ("pipe", "a named pipe")])
        self.assertEqual(in_index, [("harness/again.sh", "a symbolic link")])


class HarnessFolder(unittest.TestCase):
    def test_every_entry_is_a_regular_file_or_a_folder(self):
        if not in_a_checkout(HARNESS):
            self.skipTest("needs a git checkout, which tells the shipped entries from what an install wrote")
        entries, on_disk, in_index = check(REPO, "harness")
        # An empty walk would pass while checking nothing.
        self.assertIn(("install.sh", FILE), entries)
        self.assertIn(("tests/test_entry_kinds.py", FILE), entries)
        self.assertEqual(on_disk, [])
        self.assertEqual(in_index, [])


if __name__ == "__main__":
    unittest.main()
