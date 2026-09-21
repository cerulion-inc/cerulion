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
        for version in ("2.36", "2.35.1", "3.0"):
            with self.subTest(version=version), patch.object(sys, "argv", ["gate", "cerulion"]), \
                    patch.object(subprocess, "check_output", return_value=f"Name: GLIBC_{version}"), \
                    self.assertRaisesRegex(SystemExit, "maximum is 2.35"):
                gate.main()

    def test_every_binary_is_checked_and_locale_is_fixed(self):
        with patch.object(sys, "argv", ["gate", "cli", "netd", "connectd"]), \
                patch.object(subprocess, "check_output", return_value="Name: GLIBC_2.35") as readelf, \
                contextlib.redirect_stdout(io.StringIO()):
            gate.main()
        self.assertEqual([call.args[0] for call in readelf.call_args_list], [
            ["readelf", "--version-info", "cli"],
            ["readelf", "--version-info", "netd"],
            ["readelf", "--version-info", "connectd"],
        ])
        self.assertTrue(all(call.kwargs["env"]["LC_ALL"] == "C" for call in readelf.call_args_list))

    def test_readelf_failure_is_not_a_pass(self):
        with patch.object(sys, "argv", ["gate", "not-an-elf"]), \
                patch.object(subprocess, "check_output", side_effect=subprocess.CalledProcessError(1, "readelf")), \
                self.assertRaises(subprocess.CalledProcessError):
            gate.main()


if __name__ == "__main__":
    unittest.main()
