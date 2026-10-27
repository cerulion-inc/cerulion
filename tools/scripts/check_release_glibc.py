#!/usr/bin/env python3
"""Reject GNU/Linux release binaries requiring a newer glibc than Ubuntu 20.04 (2.31)."""
import os
import re
import subprocess
import sys

MAXIMUM = (2, 31, 0)
MAXIMUM_TEXT = "2.31"


def required_glibc(text: str) -> tuple[int, int, int]:
    versions = []
    for requirement in re.findall(r"Name: (GLIBC_\S+)", text):
        numeric = re.fullmatch(r"GLIBC_(\d+)\.(\d+)(?:\.(\d+))?", requirement)
        if numeric is None:
            raise ValueError(f"unsupported glibc ABI requirement: {requirement}")
        major, minor, patch = numeric.groups()
        versions.append((int(major), int(minor), int(patch or 0)))
    if not versions:
        raise ValueError("no GLIBC version requirements found; compatibility is unproven")
    return max(versions)


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: check_release_glibc.py BINARY ...")
    for binary in sys.argv[1:]:
        output = subprocess.check_output(["readelf", "--version-info", binary], text=True, env={**os.environ, "LC_ALL": "C"})
        version = required_glibc(output)
        if version > MAXIMUM:
            raise SystemExit(f"error: {binary} requires GLIBC_{'.'.join(map(str, version))}; maximum is {MAXIMUM_TEXT}")
        print(f"{binary}: maximum required GLIBC_{'.'.join(map(str, version))} <= {MAXIMUM_TEXT}")


if __name__ == "__main__":
    main()
