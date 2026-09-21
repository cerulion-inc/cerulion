#!/usr/bin/env python3
"""render_public_surface_verdict.py: read the public-surface review's verdict
file, render the pull request comment on stdout, and decide the job's exit.

The model writes JSON and nothing else (see public-surface-review.md). This
script is what the workflow ACTS on, so the outcome never depends on the
session choosing to call a tool, and the blocking rule is written down in one
place rather than inferred from prose.

EXIT CODES (the workflow maps them):
  0  no findings                      -> pass,  post nothing
  1  at least one LEAK or EMBARRASSING -> FAIL,  post the comment
  2  only CONFUSING or COSMETIC        -> pass,  post the comment
  3  the verdict could not be read     -> FAIL,  post the comment

3 is a FAILURE, not a pass: a semantic check that could not run looks exactly
like a clean one, which is the whole reason the other gates here fail closed.

USAGE
  render_public_surface_verdict.py <verdict.json>
  render_public_surface_verdict.py --self-test

Stdlib only.
"""
import io
import json
import os
import sys

BLOCKING = ("LEAK", "EMBARRASSING")
ADVISORY = ("CONFUSING", "COSMETIC")
SEVERITIES = BLOCKING + ADVISORY
MAX_ROWS = 50
TITLE = "### Public surface review"
DOC = "tools/review/public-surface-review.md"


def cell(text, limit=220):
    """One table cell: no line break can break the row, no pipe can add one."""
    flat = " ".join(str(text).split())
    if len(flat) > limit:
        flat = flat[: limit - 1] + "…"
    return flat.replace("|", "\\|")


def render(verdict):
    """(markdown, exit code) for a parsed verdict."""
    findings = verdict.get("findings")
    if not isinstance(findings, list):
        return cannot_run("the verdict has no `findings` list")
    rows = []
    for f in findings:
        if not isinstance(f, dict):
            return cannot_run("a finding is not an object")
        sev = str(f.get("severity", "")).upper()
        if sev not in SEVERITIES:
            return cannot_run("a finding carries severity %r, which is not one of %s" % (f.get("severity"), ", ".join(SEVERITIES)))
        rows.append((sev, f))
    blocking = [r for r in rows if r[0] in BLOCKING]
    notes = verdict.get("notes")
    if not rows:
        return ("%s\n\nNo shipped-text tells in this diff.\n" % TITLE, 0)
    out = [TITLE, ""]
    if blocking:
        out.append("**%d finding(s) block this check** (LEAK or EMBARRASSING), %d advisory. "
                   "Shipped text describes the product to a stranger: what it does, what is "
                   "experimental, what is not supported, what to do. The standard is %s."
                   % (len(blocking), len(rows) - len(blocking), DOC))
    else:
        out.append("%d advisory finding(s); nothing blocks this check. The standard is %s."
                   % (len(rows), DOC))
    out += ["", "| Severity | Where | Quoted | Write instead |", "|---|---|---|---|"]
    for sev, f in rows[:MAX_ROWS]:
        where = "`%s`" % cell(f.get("path", "?"), 120)
        line = f.get("line")
        if isinstance(line, int) and line > 0:
            where += ":%d" % line
        out.append("| %s | %s | %s | %s |" % (sev, where, cell(f.get("quote", "")), cell(f.get("rewrite", ""))))
    if len(rows) > MAX_ROWS:
        out.append("")
        out.append("_%d further finding(s) not listed._" % (len(rows) - MAX_ROWS))
    if isinstance(notes, str) and notes.strip():
        out += ["", "_%s_" % cell(notes, 600)]
    out += ["", "This check is advisory until it is a required check on `main`."]
    return ("\n".join(out) + "\n", 1 if blocking else 2)


def cannot_run(why):
    return ("%s\n\n**This check could not run:** %s. It fails rather than passes, because a "
            "semantic review that did not happen looks exactly like a clean one. Re-run the "
            "job; if it keeps happening, the prompt in %s and this renderer have drifted "
            "apart.\n" % (TITLE, why, DOC), 3)


def main(argv):
    if len(argv) == 1 and argv[0] == "--self-test":
        return self_test()
    if len(argv) != 1:
        sys.stderr.write("usage: render_public_surface_verdict.py <verdict.json>\n")
        return 2
    path = argv[0]
    if not os.path.isfile(path):
        text, code = cannot_run("the model wrote no %s" % os.path.basename(path))
    else:
        try:
            with open(path, encoding="utf-8") as fh:
                verdict = json.load(fh)
        except (OSError, ValueError) as exc:
            text, code = cannot_run("%s is not readable JSON (%s)" % (os.path.basename(path), exc))
        else:
            text, code = (cannot_run("the verdict is not an object")
                          if not isinstance(verdict, dict) else render(verdict))
    sys.stdout.write(text)
    return code


def self_test():
    """Oracle vectors: one per exit code, plus the shapes that must not break a
    table row or slip a severity past the blocking rule."""
    arms = 0

    def arm(name, ok, detail=""):
        nonlocal arms
        arms += 1
        if not ok:
            sys.stdout.write("render_public_surface_verdict --self-test: FAIL at %s %s\n" % (name, detail))
            sys.exit(1)

    text, code = render({"findings": []})
    arm("empty-findings-pass", code == 0 and "No shipped-text tells" in text, text)

    text, code = render({"findings": [{"path": "a.md", "line": 3, "severity": "COSMETIC",
                                       "quote": "q", "rewrite": "r"}]})
    arm("advisory-only-passes", code == 2 and "nothing blocks" in text and "| COSMETIC |" in text, text)

    for sev in BLOCKING:
        text, code = render({"findings": [{"path": "a.md", "severity": sev, "quote": "q", "rewrite": "r"}]})
        arm("blocking:" + sev, code == 1 and "block this check" in text, text)

    text, code = render({"findings": [{"path": "a.md", "severity": "embarrassing", "quote": "q"}]})
    arm("severity-is-case-insensitive", code == 1, text)

    text, code = render({"findings": [{"path": "a.md", "severity": "MILD", "quote": "q"}]})
    arm("unknown-severity-cannot-run", code == 3 and "could not run" in text, text)

    for bad in ({}, {"findings": "none"}, {"findings": [42]}):
        _t, code = render(bad)
        arm("malformed-cannot-run:%r" % (bad,), code == 3)

    # A quote that carries a pipe or a newline must not add or break a row.
    text, _code = render({"findings": [{"path": "a.md", "severity": "COSMETIC",
                                        "quote": "a | b\nc", "rewrite": "x\ny"}]})
    body = [l for l in text.split("\n") if l.startswith("| COSMETIC")]
    # The escaped pipe is data, so the column count is read off the UNESCAPED ones.
    columns = body[0].replace("\\|", "").count("|") if body else 0
    arm("one-row-per-finding", len(body) == 1 and columns == 5, repr(body))
    arm("pipe-is-escaped", "a \\| b c" in body[0], body[0])

    # Every finding is rendered up to the cap, and the overflow is named.
    many = [{"path": "a.md", "severity": "COSMETIC", "quote": str(i)} for i in range(MAX_ROWS + 7)]
    text, code = render({"findings": many})
    arm("cap-and-overflow", text.count("| COSMETIC |") == MAX_ROWS and "7 further finding(s)" in text, text)

    # A blocking finding past the cap still blocks: the exit reads the LIST,
    # never the rendered table.
    text, code = render({"findings": many + [{"path": "z.md", "severity": "LEAK", "quote": "q"}]})
    arm("blocking-past-the-cap-still-blocks", code == 1, text)

    # Through main(), where a missing or unreadable file is decided.
    import contextlib
    import tempfile

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = main([os.path.join(tempfile.gettempdir(), "no-such-public-surface-verdict.json")])
    arm("missing-file-is-exit-3", code == 3 and "could not run" in buf.getvalue(), buf.getvalue())
    with tempfile.TemporaryDirectory(prefix="public-surface-verdict-") as tmp:
        broken = os.path.join(tmp, "verdict.json")
        with open(broken, "w", encoding="utf-8") as fh:
            fh.write("{not json")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = main([broken])
        arm("unparseable-file-is-exit-3", code == 3 and "not readable JSON" in buf.getvalue(), buf.getvalue())
        with open(broken, "w", encoding="utf-8") as fh:
            fh.write('{"findings": [{"path": "a.md", "severity": "LEAK", "quote": "q"}]}')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = main([broken])
        arm("a-real-file-blocks", code == 1 and "| LEAK |" in buf.getvalue(), buf.getvalue())

    sys.stdout.write("render_public_surface_verdict --self-test: OK (%d arms)\n" % arms)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
