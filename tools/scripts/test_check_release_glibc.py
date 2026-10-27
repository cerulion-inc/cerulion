#!/usr/bin/env python3
"""Hand-written compatibility oracles: never infer expectations from a build."""
import contextlib
import io
import subprocess
import sys
import unittest
from unittest.mock import patch

import check_release_glibc as gate


class GlibcCompatibilityTest(unittest.TestCase):
    def test_numeric_versions_are_compared_as_numbers(self):
        self.assertEqual(gate.required_glibc("Name: GLIBC_2.9\nName: GLIBC_2.35\nName: GLIBC_2.2.5"), (2, 35, 0))

    def test_missing_requirements_do_not_claim_compatibility(self):
        with self.assertRaisesRegex(ValueError, "compatibility is unproven"):
            gate.required_glibc("Version symbols section has no entries")

    def test_unknown_abi_requirements_are_rejected_even_with_old_numeric_versions(self):
        for abi in ("GLIBC_ABI_DT_RELR", "GLIBC_PRIVATE", "GLIBC_2.35.1.2"):
            with self.subTest(abi=abi), self.assertRaisesRegex(ValueError, "unsupported glibc ABI"):
                gate.required_glibc(f"Name: GLIBC_2.34\nName: {abi}")

    def test_newer_glibc_fails_the_command(self):
        for version in ("2.32", "2.31.1", "2.35", "3.0"):
            with self.subTest(version=version), patch.object(sys, "argv", ["gate", "cerulion"]), \
                    patch.object(subprocess, "check_output", return_value=f"Name: GLIBC_{version}"), \
                    self.assertRaisesRegex(SystemExit, "maximum is 2.31"):
                gate.main()

    def test_every_binary_is_checked_and_locale_is_fixed(self):
        with patch.object(sys, "argv", ["gate", "cli", "netd", "connectd"]), \
                patch.object(subprocess, "check_output", return_value="Name: GLIBC_2.31") as readelf, \
                contextlib.redirect_stdout(io.StringIO()):
            gate.main()
        self.assertEqual([call.args[0] for call in readelf.call_args_list], [
            ["readelf", "--version-info", "cli"],
            ["readelf", "--version-info", "netd"],
            ["readelf", "--version-info", "connectd"],
        ])
        self.assertTrue(all(call.kwargs["env"]["LC_ALL"] == "C" for call in readelf.call_args_list))

    def test_the_ubuntu_20_04_floor_is_accepted_and_one_above_is_refused(self):
        readelf = (
            "Version needs section '.gnu.version_r' contains 2 entries:\n"
            " Addr: 0x0000000000001a2c  Offset: 0x001a2c  Link: 7 (.dynstr)\n"
            "  000000: Version: 1  File: libc.so.6  Cnt: 3\n"
            "  0x0010:   Name: GLIBC_2.2.5  Flags: none  Version: 4\n"
            "  0x0020:   Name: GLIBC_2.30  Flags: none  Version: 3\n"
            "  0x0030:   Name: {top}  Flags: none  Version: 2\n"
        )
        for top, accepted in (("GLIBC_2.30", True), ("GLIBC_2.31", True), ("GLIBC_2.32", False)):
            text = readelf.format(top=top)
            with self.subTest(top=top), patch.object(sys, "argv", ["gate", "libcerulion_heaphook.so"]), \
                    patch.object(subprocess, "check_output", return_value=text):
                if accepted:
                    out = io.StringIO()
                    with contextlib.redirect_stdout(out):
                        gate.main()
                    self.assertIn("<= 2.31", out.getvalue())
                else:
                    with self.assertRaisesRegex(SystemExit, "requires GLIBC_2.32.0; maximum is 2.31"):
                        gate.main()

    def test_readelf_failure_is_not_a_pass(self):
        with patch.object(sys, "argv", ["gate", "not-an-elf"]), \
                patch.object(subprocess, "check_output", side_effect=subprocess.CalledProcessError(1, "readelf")), \
                self.assertRaises(subprocess.CalledProcessError):
            gate.main()


if __name__ == "__main__":
    unittest.main()
