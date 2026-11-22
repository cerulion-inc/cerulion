# SPDX-License-Identifier: AGPL-3.0-only
"""The benchmark pins: what `benchmarks/fetch.sh` fetches and checks.

Standard library only, no network. The expected values below are written by
hand: a pin that changes in `fetch.sh` or in the lists fails here until the
value is changed here too.
"""

import pathlib
import re
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
BENCHMARKS = HARNESS / "benchmarks"

# Hand-written oracle: the revisions the measured runs used.
LIBERO_COMMIT = "8f1084e3132a39270c3a13ebe37270a43ece2a01"
LIBERO_PRO_COMMIT = "eafdb809426b13153aa1e4c42d6601844217dfec"
DATA_REVISION = "c86fc3b8293185a6f373677018ff3e37f8391602"
DATA_FILE_COUNT = 160
# The first and the last entry of the sorted list, with their digests.
FIRST_FILE = "bddl_files/libero_10_swap/KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it.bddl"
FIRST_SHA256 = "1104896ca46963b0b3b97caf785f84d70635b72cca0efdfe724167dbc9f19926"


def shell_assignment(text, name):
    """The value of the one `NAME=value` line of a shell script."""
    found = re.findall(r"^%s=(\S+)$" % re.escape(name), text, flags=re.MULTILINE)
    if len(found) != 1:
        raise AssertionError("%s is assigned %d times, wanted once" % (name, len(found)))
    return found[0]


class ScenePins(unittest.TestCase):
    def setUp(self):
        self.fetch = (BENCHMARKS / "fetch.sh").read_text(encoding="utf-8")
        self.files = (BENCHMARKS / "LIBERO-PRO-data.files.txt").read_text(encoding="utf-8").splitlines()
        self.digests = (BENCHMARKS / "LIBERO-PRO-data.sha256").read_text(encoding="utf-8").splitlines()

    def test_fetch_pins_the_three_revisions_of_the_measured_runs(self):
        self.assertEqual(shell_assignment(self.fetch, "LIBERO_SHA"), LIBERO_COMMIT)
        self.assertEqual(shell_assignment(self.fetch, "PRO_SHA"), LIBERO_PRO_COMMIT)
        self.assertEqual(shell_assignment(self.fetch, "DATA_REV"), DATA_REVISION)

    def test_fetch_never_follows_a_branch(self):
        # A branch or a tag name moves; a pinned commit does not.
        for word in ("main", "master", "HEAD~", "--branch"):
            self.assertNotIn(word, self.fetch, word)
        self.assertIn('test "$(git -C "$2" rev-parse HEAD)" = "$3"', self.fetch)

    def test_the_file_list_has_160_sorted_unique_relative_paths(self):
        self.assertEqual(len(self.files), DATA_FILE_COUNT)
        self.assertEqual(self.files, sorted(set(self.files)))
        self.assertEqual(self.files[0], FIRST_FILE)
        for rel in self.files:
            self.assertFalse(rel.startswith("/"), rel)
            self.assertNotIn("..", pathlib.PurePosixPath(rel).parts, rel)
            self.assertEqual(rel, rel.strip(), rel)

    def test_every_listed_file_has_one_sha256_and_nothing_else_does(self):
        self.assertEqual(len(self.digests), DATA_FILE_COUNT)
        seen = {}
        for line in self.digests:
            match = re.fullmatch(r"([0-9a-f]{64})  \./(\S.*)", line)
            self.assertIsNotNone(match, line)
            self.assertNotIn(match.group(2), seen, line)
            seen[match.group(2)] = match.group(1)
        self.assertEqual(sorted(seen), self.files)
        self.assertEqual(seen[FIRST_FILE], FIRST_SHA256)

    def test_fetch_checks_every_file_against_the_digest_list(self):
        self.assertIn('sha256sum --quiet -c "$HERE/LIBERO-PRO-data.sha256"', self.fetch)
        self.assertIn('shasum -a 256 -s -c "$HERE/LIBERO-PRO-data.sha256"', self.fetch)


if __name__ == "__main__":
    unittest.main()
