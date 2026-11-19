# SPDX-License-Identifier: AGPL-3.0-only
"""Check the shipped harness code against its provenance files. Standard library only.

provenance/measured-rcp.manifest and measured-plain.manifest list every file of the two measured code trees, one
"SHA256  path" line each. The SHA-256 of a manifest's bytes is that tree's snapshot id; SNAPSHOTS below holds both.
Both trees carry the two picture modes of bench/operators/chat_support.py: swap, the operator's default, and append,
which run-cell gives a Claude model with the harness tools (cerulion_harness.cli.picture_mode).

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

    python -m cerulion_harness.verify [HARNESS]

checks HARNESS (by default the harness folder this file is in) and prints each arm's snapshot id and file count and
every problem. It exits 0 only when both arms were checked and no problem was found, 1 when there is a problem, 2 on
a usage error.
"""
import hashlib
import importlib.util
import json
import os
import re
import sys
import tempfile

SNAPSHOTS = {
    "rcp": "sha256:4f8bbe79fdf38df1770c068ec470fd3d5a629fd371f7288887cb3d275f9f747e",
    "plain": "sha256:6f040ca78bae0efd437005afc04ead78c4295e1e471d854b3e7ff0f9ea01f2a4",
}
OUTSIDE_SNAPSHOT = ("runs/tools/grid_ext.py",)
STATUSES = ("identical", "comments_and_docstrings", "text_edited", "strings_edited", "code_edited")
MANIFEST_LINE = re.compile(r"^([0-9a-f]{64})  (\S(?:.*\S)?)$")
# The harness folder: src/cerulion_harness/ is two levels below it.
HARNESS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
USAGE = "usage: python -m cerulion_harness.verify [HARNESS]   (HARNESS: a harness folder, by default this file's)"


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


def main(argv=None):
    """The command line: verify one folder and print the result. An arm with no checked file is a problem of its
    own, so the exit is 0 only when every arm was checked and no problem was found."""
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv in (["-h"], ["--help"]):
        print(USAGE)
        return 0
    if len(argv) > 1 or (argv and argv[0].startswith("-")):
        print(USAGE, file=sys.stderr)
        return 2
    problems, facts = verify(argv[0] if argv else HARNESS)
    problems = list(problems)
    for arm in SNAPSHOTS:
        if not (facts.get(arm) or {}).get("shipped_files"):
            problems.append(f"{arm}: no shipped file was checked")
    for arm in sorted(facts):
        fact = facts[arm]
        print(f"{arm}: snapshot {fact.get('snapshot')}, {fact.get('shipped_files')} shipped files")
    for problem in problems:
        print("PROBLEM: " + problem, file=sys.stderr)
    if problems:
        print(f"verify: {len(problems)} problem(s); this folder is not the measured code")
        return 1
    print("verify: ok, this folder holds the code the benchmark measured")
    return 0


if __name__ == "__main__":
    sys.exit(main())
