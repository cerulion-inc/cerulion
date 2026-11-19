# SPDX-License-Identifier: AGPL-3.0-only
"""The note of which start states were checked on this machine: a start state checked once never starts the
simulator for its body record again. Standard library only, no simulator: a test that reached the measured code
would fail on its import, since no measured tree is on the path here."""
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import body_record  # noqa: E402

CELL = "pro_spatial_swap_t00_i00"
KEY = "726d4e8cc7d8227186ce25893edfbeb2d71458ce"


class Notes(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.records = self.tmp.name
        self.record = os.path.join(self.records, KEY + ".json")
        with open(self.record, "w", encoding="utf-8") as f:
            f.write("{}")

    def tearDown(self):
        self.tmp.cleanup()

    def note(self):
        body_record._note(CELL, self.records, {"key": KEY, "path": self.record})

    def test_a_start_state_never_checked_has_no_note(self):
        self.assertIsNone(body_record.checked(CELL, self.records))

    def test_a_noted_start_state_reads_its_key_and_record(self):
        self.note()
        row = body_record.checked(CELL, self.records)
        self.assertEqual(row["key"], KEY)
        self.assertEqual(row["record"], KEY + ".json")
        self.assertIsNone(body_record.checked("pro_object_swap_t00_i00", self.records))

    def test_a_note_whose_record_file_is_gone_does_not_count(self):
        self.note()
        os.unlink(self.record)
        self.assertIsNone(body_record.checked(CELL, self.records))

    def test_a_note_written_on_another_kind_of_machine_does_not_count(self):
        with open(os.path.join(self.records, body_record.CHECKED), "w", encoding="utf-8") as f:
            json.dump({"machine": "Plan9 mips", "cells": {CELL: {"key": KEY, "record": KEY + ".json"}}}, f)
        self.assertIsNone(body_record.checked(CELL, self.records))

    def test_a_note_that_is_not_json_does_not_count(self):
        with open(os.path.join(self.records, body_record.CHECKED), "w", encoding="utf-8") as f:
            f.write("{not json")
        self.assertIsNone(body_record.checked(CELL, self.records))
        self.note()   # and the next note replaces it
        self.assertEqual(body_record.checked(CELL, self.records)["key"], KEY)

    def test_no_note_is_kept_for_a_row_without_a_stored_record(self):
        body_record._note(CELL, self.records, {"key": KEY, "path": os.path.join(self.records, "missing.json")})
        body_record._note(CELL, self.records, {"key": None, "path": self.record})
        self.assertFalse(os.path.exists(os.path.join(self.records, body_record.CHECKED)))

    def test_notes_of_two_start_states_are_both_kept(self):
        self.note()
        body_record._note("l10_t09_i00", self.records, {"key": KEY, "path": self.record})
        self.assertIsNotNone(body_record.checked(CELL, self.records))
        self.assertIsNotNone(body_record.checked("l10_t09_i00", self.records))
        leftovers = [n for n in os.listdir(self.records) if n.endswith(".tmp")]
        self.assertEqual(leftovers, [])

    def test_a_noted_start_state_is_answered_without_the_simulator(self):
        self.note()
        row = body_record.for_cell(CELL, self.records, None, None, False, None)
        self.assertEqual((row["record"], row["measured_now"], row["checked"], row["key"]),
                         ("found", False, "earlier", KEY))

    def test_the_command_answers_a_noted_start_state_with_one_found_row(self):
        self.note()
        out = os.path.join(self.records, "out.json")
        with open(out, "w", encoding="utf-8") as f:
            saved, sys.stdout = sys.stdout, f
            try:
                rc = body_record.main(["--for-cell", CELL, "--records", self.records])
            finally:
                sys.stdout = saved
        with open(out, encoding="utf-8") as f:
            doc = json.load(f)
        self.assertEqual(rc, 0)
        self.assertEqual((doc["found"], doc["measured"], doc["problems"]), (1, 0, 0))

    def test_the_command_refuses_an_unknown_cell_before_anything_starts(self):
        saved, sys.stderr = sys.stderr, open(os.devnull, "w")
        try:
            self.assertEqual(body_record.main(["--for-cell", "not_a_cell", "--records", self.records]), 2)
        finally:
            sys.stderr.close()
            sys.stderr = saved


if __name__ == "__main__":
    unittest.main()
