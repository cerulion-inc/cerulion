# SPDX-License-Identifier: AGPL-3.0-only
"""Check the shipped harness code against its provenance files. Standard library only.

provenance/measured-rcp.manifest and measured-plain.manifest list every file of the two code trees the published
benchmark grid ran, one "SHA256  path" line each. The SHA-256 of a manifest's bytes is that tree's snapshot id,
the id the results package cites; SNAPSHOTS below holds both.

One tree ships: measured/rcp/, the arm with the Cerulion harness's tools. The plain arm's tree is built from it by
measured/assemble.py (plain-overlay/ replaces a few files, plain-absent.txt leaves one out); verify builds it in a
temporary folder and checks the result.

provenance/ledger.json holds, per arm, one row for every shipped file: the hash of the measured file it came from,
the hash of the shipped file, and a status:

    identical                byte for byte the measured file
    comments_and_docstrings  the measured file's code and strings; the licence line, comments and docstrings differ
    text_edited              a page of prose that no code reads
    strings_edited           strings_changed lists each changed string literal: the measured string's hash and the
                             shipped text
    code_edited              code_changed lists each edit (the measured and the shipped text) and why

and dropped_files, the measured files the shipped tree leaves out. Every measured file is a row or a dropped file.
OUTSIDE_SNAPSHOT names the one shipped file that is in neither measured tree: the launcher that ran the grid, whose
measured hash only the ledger holds.
"""
import hashlib
import importlib.util
import json
import os
import re
import tempfile

SNAPSHOTS = {
    "rcp": "sha256:7cb9f322c10f6fa7e64a3f8da7c22c67d67280eca436a03d2d87683496b4d5b8",
    "plain": "sha256:b73c548a1f0436f52ce41cffc7d4a4a2784e5d6170e2dd2c757c9cabdf146637",
}
OUTSIDE_SNAPSHOT = ("runs/tools/grid_ext.py",)
STATUSES = ("identical", "comments_and_docstrings", "text_edited", "strings_edited", "code_edited")
MANIFEST_LINE = re.compile(r"^([0-9a-f]{64})  (\S(?:.*\S)?)$")


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _assembler(home):
    """measured/assemble.py of this harness folder, loaded from its path."""
    path = os.path.join(home, "measured", "assemble.py")
    spec = importlib.util.spec_from_file_location("cerulion_harness_assemble", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _manifest(arm, text, problems):
    """{path: sha256} of a manifest's lines (the last one ends with a newline); a malformed or repeated line is a
    problem."""
    measured = {}
    lines = text.decode("utf-8", errors="replace").split("\n")
    if lines[-1] == "":
        lines.pop()
    for no, line in enumerate(lines, 1):
        m = MANIFEST_LINE.match(line)
        if m is None:
            problems.append(f"{arm}: manifest line {no} is not 'SHA256  path'")
        elif m.group(2) in measured:
            problems.append(f"{arm}: manifest line {no} names {m.group(2)} again")
        else:
            measured[m.group(2)] = m.group(1)
    return measured


def _rows(arm, entry, measured, problems):
    """{path: row} of the arm's ledger rows, each row checked against the manifest and its own status."""
    rows = {}
    for row in entry.get("files", []):
        rel, status = row.get("path"), row.get("status")
        if rel in rows:
            problems.append(f"{arm}: {rel} has two ledger rows")
            continue
        rows[rel] = row
        if rel in OUTSIDE_SNAPSHOT:
            pass
        elif rel not in measured:
            problems.append(f"{arm}: {rel} is in the ledger but not in the measured manifest")
        elif measured[rel] != row.get("measured_sha256"):
            problems.append(f"{arm}: {rel}: the ledger's measured hash is not the manifest's")
        if status not in STATUSES:
            problems.append(f"{arm}: {rel}: unknown status {status!r}")
        elif status == "identical" and row.get("public_sha256") != row.get("measured_sha256"):
            problems.append(f"{arm}: {rel} is marked identical but its two hashes differ")
        elif status == "strings_edited" and not row.get("strings_changed"):
            problems.append(f"{arm}: {rel} is marked strings_edited but lists no string")
        elif status == "code_edited" and not (row.get("code_changed") or {}).get("edits"):
            problems.append(f"{arm}: {rel} is marked code_edited but lists no edit")
    dropped = entry.get("dropped_files", [])
    for rel in dropped:
        if rel not in measured:
            problems.append(f"{arm}: dropped file {rel} is not in the measured manifest")
        elif rel in rows:
            problems.append(f"{arm}: {rel} is both shipped and dropped")
    for rel in sorted(set(measured) - set(rows) - set(dropped)):
        problems.append(f"{arm}: measured file {rel} is neither shipped nor dropped")
    return rows


def _declared_texts(arm, row, path, problems):
    """Every shipped text a strings_edited or code_edited row declares is in the shipped file."""
    texts = [s.get("public", "") for s in row.get("strings_changed", [])]
    texts += [e.get("public", "") for e in (row.get("code_changed") or {}).get("edits", [])]
    if not texts:
        return
    with open(path, encoding="utf-8") as f:
        source = f.read()
    for text in texts:
        if not text or text not in source:
            problems.append(f"{arm}: {row['path']}: a text the ledger declares is not in the shipped file")


def _tree(arm, tree, files, rows, problems):
    for rel in sorted(set(rows) - set(files)):
        problems.append(f"{arm}: {rel} is missing")
    for rel in files:
        row = rows.get(rel)
        if row is None:
            problems.append(f"{arm}: {rel} is not in the ledger")
            continue
        path = os.path.join(tree, *rel.split("/"))
        if _sha(path) != row.get("public_sha256"):
            problems.append(f"{arm}: {rel} differs from the ledger")
        else:
            _declared_texts(arm, row, path, problems)


def verify(home):
    """(problems, facts) for the harness folder home: problems is empty when everything matches; facts gives, per
    arm, the snapshot id, the measured and shipped file counts, the shipped files by status and the files left
    out."""
    problems, facts = [], {}
    prov = os.path.join(home, "provenance")
    try:
        with open(os.path.join(prov, "ledger.json"), encoding="utf-8") as f:
            ledger = json.load(f)
        arms = ledger["arms"]
        assembler = _assembler(home)
    except (OSError, ValueError, KeyError, TypeError, ImportError, SyntaxError) as e:
        return [f"cannot read the provenance files or load measured/assemble.py: {e}"], facts
    with tempfile.TemporaryDirectory(prefix="cerulion-harness-plain-") as scratch:
        for arm, want in SNAPSHOTS.items():
            try:
                with open(os.path.join(prov, f"measured-{arm}.manifest"), "rb") as f:
                    text = f.read()
            except OSError as e:
                problems.append(f"{arm}: cannot read the measured manifest: {e}")
                continue
            got = "sha256:" + hashlib.sha256(text).hexdigest()
            if got != want:
                problems.append(f"{arm}: the measured manifest hashes to {got}, expected {want}")
            measured = _manifest(arm, text, problems)
            entry = arms.get(arm) or {}
            rows = _rows(arm, entry, measured, problems)
            if arm == "rcp":
                tree = os.path.join(home, "measured", "rcp")
            else:
                tree = os.path.join(scratch, "plain")
                try:
                    assembler.assemble_plain(os.path.join(home, "measured"), tree)
                except (assembler.AssemblyError, OSError) as e:
                    problems.append(f"{arm}: the tree cannot be assembled: {e}")
                    continue
            try:
                files = assembler.tree_files(tree)
            except (assembler.AssemblyError, OSError) as e:
                problems.append(f"{arm}: cannot list the shipped tree: {e}")
                continue
            _tree(arm, tree, files, rows, problems)
            by_status = {}
            for row in rows.values():
                by_status[row.get("status")] = by_status.get(row.get("status"), 0) + 1
            facts[arm] = {"snapshot": want, "measured_files": len(measured), "shipped_files": len(files),
                          "shipped_by_status": dict(sorted(by_status.items(), key=lambda kv: str(kv[0]))),
                          "left_out": len(entry.get("dropped_files", []))}
    return problems, facts
