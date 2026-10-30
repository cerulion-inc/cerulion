#!/usr/bin/env python3
"""Oracle tests for package_android_java.py, the Java facade release asset.

Every expected member list and every expected byte string is written out by
hand here; nothing is derived from a run of the script under test.
"""
import hashlib
import os
import pathlib
import subprocess
import sys
import tarfile
import tempfile
import unittest

SCRIPTS = pathlib.Path(__file__).resolve().parent
ROOT = SCRIPTS.parent.parent
SCRIPT = SCRIPTS / "package_android_java.py"
CRATE = ROOT / "crates" / "cerulion_android"


def run(*args):
    return subprocess.run(
        [sys.executable, "-I", str(SCRIPT), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def fixture_crate(root):
    """A crate directory with two Java files and a license, all hand-written."""
    crate = root / "crate"
    write(crate / "java" / "com" / "example" / "b" / "Second.java", "class Second {}\n")
    write(crate / "java" / "com" / "example" / "a" / "First.java", "class First {}\n")
    write(crate / "LICENSE", "license text\n")
    write(crate / "README.md", "not shipped in the facade asset\n")
    return crate


def members(archive):
    with tarfile.open(archive, "r:gz") as tar:
        return [(m.name, m.type, m.mode, m.uid, m.gid, m.mtime) for m in tar.getmembers()]


def member_bytes(archive, name):
    with tarfile.open(archive, "r:gz") as tar:
        return tar.extractfile(name).read()


class RealCrateAsset(unittest.TestCase):
    def test_the_asset_carries_exactly_the_facade_and_the_license(self):
        with tempfile.TemporaryDirectory() as out:
            result = run("--version", "1.1.0", "--out", out)
            self.assertEqual(result.returncode, 0, result.stderr)
            archive = pathlib.Path(out) / "cerulion-android-java-1.1.0.tar.gz"
            names = [m[0] for m in members(archive)]
            self.assertEqual(names, [
                "cerulion-android-java-1.1.0",
                "cerulion-android-java-1.1.0/LICENSE",
                "cerulion-android-java-1.1.0/java",
                "cerulion-android-java-1.1.0/java/com",
                "cerulion-android-java-1.1.0/java/com/cerulion",
                "cerulion-android-java-1.1.0/java/com/cerulion/android",
                "cerulion-android-java-1.1.0/java/com/cerulion/android/CerulionRuntime.java",
            ])
            facade = CRATE / "java" / "com" / "cerulion" / "android" / "CerulionRuntime.java"
            self.assertEqual(
                member_bytes(archive, "cerulion-android-java-1.1.0/java/com/cerulion/android/CerulionRuntime.java"),
                facade.read_bytes(),
            )
            self.assertEqual(
                member_bytes(archive, "cerulion-android-java-1.1.0/LICENSE"),
                (CRATE / "LICENSE").read_bytes(),
            )
            self.assertEqual(sorted(os.listdir(out)), [
                "cerulion-android-java-1.1.0.tar.gz",
                "cerulion-android-java-1.1.0.tar.gz.sha256",
            ])


class FixtureAsset(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.tmp.name)
        self.crate = fixture_crate(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def package(self, version="2.0.0-rc.1", out="out"):
        out_dir = self.root / out
        result = run("--version", version, "--out", str(out_dir), "--crate-dir", str(self.crate))
        return result, out_dir

    def test_members_are_sorted_with_fixed_owner_mode_and_time(self):
        result, out = self.package()
        self.assertEqual(result.returncode, 0, result.stderr)
        p = "cerulion-android-java-2.0.0-rc.1"
        d, f = tarfile.DIRTYPE, tarfile.REGTYPE
        self.assertEqual(members(out / f"{p}.tar.gz"), [
            (p, d, 0o755, 0, 0, 0),
            (f"{p}/LICENSE", f, 0o644, 0, 0, 0),
            (f"{p}/java", d, 0o755, 0, 0, 0),
            (f"{p}/java/com", d, 0o755, 0, 0, 0),
            (f"{p}/java/com/example", d, 0o755, 0, 0, 0),
            (f"{p}/java/com/example/a", d, 0o755, 0, 0, 0),
            (f"{p}/java/com/example/a/First.java", f, 0o644, 0, 0, 0),
            (f"{p}/java/com/example/b", d, 0o755, 0, 0, 0),
            (f"{p}/java/com/example/b/Second.java", f, 0o644, 0, 0, 0),
        ])
        self.assertEqual(member_bytes(out / f"{p}.tar.gz", f"{p}/java/com/example/b/Second.java"),
                         b"class Second {}\n")
        self.assertEqual(member_bytes(out / f"{p}.tar.gz", f"{p}/LICENSE"), b"license text\n")

    def test_the_checksum_file_is_sha256sum_format_for_the_archive(self):
        result, out = self.package()
        self.assertEqual(result.returncode, 0, result.stderr)
        name = "cerulion-android-java-2.0.0-rc.1.tar.gz"
        digest = hashlib.sha256((out / name).read_bytes()).hexdigest()
        self.assertEqual((out / f"{name}.sha256").read_text(encoding="ascii"), f"{digest}  {name}\n")

    def test_the_gzip_header_carries_no_time_and_no_name(self):
        result, out = self.package()
        self.assertEqual(result.returncode, 0, result.stderr)
        header = (out / "cerulion-android-java-2.0.0-rc.1.tar.gz").read_bytes()[:10]
        # magic, deflate, no flags (so no file name), mtime 0, any xfl, any os
        self.assertEqual(header[:8], b"\x1f\x8b\x08\x00\x00\x00\x00\x00")

    def test_two_runs_are_byte_identical_whatever_the_source_times(self):
        first, out_a = self.package(out="a")
        self.assertEqual(first.returncode, 0, first.stderr)
        for path in self.crate.rglob("*"):
            os.utime(path, (1_700_000_000, 1_700_000_000))
        second, out_b = self.package(out="b")
        self.assertEqual(second.returncode, 0, second.stderr)
        name = "cerulion-android-java-2.0.0-rc.1.tar.gz"
        self.assertEqual((out_a / name).read_bytes(), (out_b / name).read_bytes())
        self.assertEqual((out_a / f"{name}.sha256").read_bytes(), (out_b / f"{name}.sha256").read_bytes())

    def test_a_version_that_is_not_semver_is_refused(self):
        for version in ("v1.1.0", "1.1", "", "1.1.0/../x", "1.1.0 "):
            with self.subTest(version=version):
                result, out = self.package(version=version)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("version", result.stderr)
                self.assertFalse(out.exists())

    def test_a_crate_without_a_java_tree_is_refused(self):
        for path in sorted(self.crate.joinpath("java").rglob("*"), reverse=True):
            path.unlink() if path.is_file() else path.rmdir()
        self.crate.joinpath("java").rmdir()
        result, out = self.package()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("java", result.stderr)
        self.assertFalse(out.exists())

    def test_a_java_tree_without_a_java_source_is_refused(self):
        for path in self.crate.joinpath("java").rglob("*.java"):
            path.unlink()
        result, out = self.package()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(".java", result.stderr)
        self.assertFalse(out.exists())

    def test_a_crate_without_a_license_is_refused(self):
        self.crate.joinpath("LICENSE").unlink()
        result, out = self.package()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("LICENSE", result.stderr)
        self.assertFalse(out.exists())

    def test_a_symbolic_link_in_the_java_tree_is_refused(self):
        outside = self.root / "outside.java"
        write(outside, "class Outside {}\n")
        self.crate.joinpath("java", "com", "Link.java").symlink_to(outside)
        result, out = self.package()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("symbolic link", result.stderr)
        self.assertFalse(out.exists())

    def test_missing_arguments_are_refused(self):
        for args in ((), ("--version", "1.0.0"), ("--out", str(self.root / "out"))):
            with self.subTest(args=args):
                result = run(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("usage", result.stderr)


if __name__ == "__main__":
    unittest.main()
