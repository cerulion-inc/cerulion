#!/usr/bin/env python3
"""Package the cerulion_android Java facade as a versioned release asset.

    package_android_java.py --version <X.Y.Z> --out <dir> [--crate-dir <dir>]

Writes `<dir>/cerulion-android-java-<version>.tar.gz` and its `.sha256` line
(the `sha256sum` format the other release assets use). The archive holds the
crate's whole `java/` tree and its `LICENSE` under one
`cerulion-android-java-<version>/` directory, the same files the crates.io
package of `cerulion_android` carries for them.

The archive is reproducible: members are sorted, owned by 0:0 with modes
0755/0644 and time 0, and the gzip header carries no time and no name, so two
runs over the same sources with the same zlib give the same bytes and the
same checksum.

Every refusal happens before anything is written: a version that is not
`X.Y.Z` or `X.Y.Z-<pre>`, a crate with no `java/` directory, a `java/` tree
with no `.java` file, a missing `LICENSE`, and a symbolic link anywhere in
the tree (the asset never carries a file from outside the crate).
"""
import argparse
import gzip
import hashlib
import io
import pathlib
import re
import sys
import tarfile

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
DEFAULT_CRATE = ROOT / "crates" / "cerulion_android"
VERSION = re.compile(r"[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?")


def refuse(message):
    print(f"package_android_java: {message}", file=sys.stderr)
    sys.exit(1)


def tree_entries(crate):
    """(archive-relative path, source path or None for a directory), sorted."""
    java = crate / "java"
    if java.is_symlink() or not java.is_dir():
        refuse(f"{java} is not a directory: the crate carries no java tree")
    license_file = crate / "LICENSE"
    if license_file.is_symlink() or not license_file.is_file():
        refuse(f"{license_file} is missing: the asset must carry the crate's LICENSE")
    entries = [("LICENSE", license_file), ("java", None)]
    sources = 0
    for path in sorted(java.rglob("*")):
        relative = path.relative_to(crate).as_posix()
        if path.is_symlink():
            refuse(f"{path} is a symbolic link: the asset carries files of the crate only")
        if path.is_dir():
            entries.append((relative, None))
        elif path.is_file():
            entries.append((relative, path))
            sources += path.suffix == ".java"
        else:
            refuse(f"{path} is neither a file nor a directory")
    if sources == 0:
        refuse(f"{java} holds no .java file")
    return sorted(entries)


def member(name, size=None):
    info = tarfile.TarInfo(name)
    info.uid = info.gid = 0
    info.mtime = 0
    if size is None:
        info.type = tarfile.DIRTYPE
        info.mode = 0o755
    else:
        info.size = size
        info.mode = 0o644
    return info


def archive_bytes(prefix, entries):
    tar_buffer = io.BytesIO()
    with tarfile.open(fileobj=tar_buffer, mode="w", format=tarfile.PAX_FORMAT) as tar:
        tar.addfile(member(prefix))
        for relative, source in entries:
            name = f"{prefix}/{relative}"
            if source is None:
                tar.addfile(member(name))
            else:
                data = source.read_bytes()
                tar.addfile(member(name, len(data)), io.BytesIO(data))
    gz_buffer = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=gz_buffer, compresslevel=9, mtime=0) as gz:
        gz.write(tar_buffer.getvalue())
    return gz_buffer.getvalue()


class Parser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        refuse(message)


def main():
    parser = Parser(description="Package the cerulion_android Java facade as a release asset.")
    parser.add_argument("--version", required=True, help="the release version, X.Y.Z or X.Y.Z-<pre>")
    parser.add_argument("--out", required=True, type=pathlib.Path, help="directory to write the asset into")
    parser.add_argument("--crate-dir", type=pathlib.Path, default=DEFAULT_CRATE,
                        help="the cerulion_android crate directory (default: this checkout's)")
    args = parser.parse_args()
    if not VERSION.fullmatch(args.version):
        refuse(f"version {args.version!r} is not X.Y.Z or X.Y.Z-<pre>")

    prefix = f"cerulion-android-java-{args.version}"
    data = archive_bytes(prefix, tree_entries(args.crate_dir))
    args.out.mkdir(parents=True, exist_ok=True)
    archive = args.out / f"{prefix}.tar.gz"
    archive.write_bytes(data)
    digest = hashlib.sha256(data).hexdigest()
    (args.out / f"{archive.name}.sha256").write_text(f"{digest}  {archive.name}\n", encoding="ascii")
    print(f"{archive}: sha256 {digest}")


if __name__ == "__main__":
    main()
