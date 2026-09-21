#!/usr/bin/env python3
"""Reject GNU/Linux release binaries requiring a newer glibc than Ubuntu 22.04."""
import os
import re
import subprocess
import sys


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
        if version > (2, 35, 0):
            raise SystemExit(f"error: {binary} requires GLIBC_{'.'.join(map(str, version))}; maximum is 2.35")
        print(f"{binary}: maximum required GLIBC_{'.'.join(map(str, version))} <= 2.35")


if __name__ == "__main__":
    main()
