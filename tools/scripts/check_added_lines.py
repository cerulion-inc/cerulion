#!/usr/bin/env python3
"""check_added_lines.py: no line a pull request ADDS carries a typographic dash.

WHY THE GATE IS DIFF-SCOPED, and not a tree scan. The shipped tree carries an em
or en dash in 1,266 files (measured on the public `main` at 423ce9b6c1: 1,137
Rust, of which 489 non-test sources, 76 toml, 25 shell, 12 python, 9 workflow, 7
markdown), almost all of it in comments and doc comments. A prose-wide tree gate
would red `main` on its first run and say nothing about the change under review.
So this gate reads the DIFF: any line the pull request adds, in any file type,
must carry no U+2014 and no U+2013. Untouched lines stay as they are, and a
tree-wide scrub is a separate decision.

WHY IT EXISTS AT ALL. `check_public_surface.py`'s `dash_surface` reads string
literals in non-test Rust files and no Python at all, so prose in
`crates/*/tests/*.rs` and `tools/scripts/*.py` escaped the rule entirely;
fifteen em dashes reached one pull request that way.

THE ESCAPE HATCH. A line that legitimately needs one, a code span quoting text
that carries a dash or a test fixture that must contain one, carries the marker
`dash-ok` on that same line, spelled as a comment in whatever syntax the file
uses (`# dash-ok`, `// dash-ok`, `<!-- dash-ok -->`). Per LINE, never per file:
a file-level waiver would exempt every line added to it afterwards.

WHAT IT READS. `git diff <base>...HEAD` with the three-dot spelling, which
diffs against the MERGE BASE of `<base>` and HEAD. Two dots would report every
line `main` gained since the branch left it as a line this branch added, so a
dash somebody else landed would red this pull request.

A file the diff reports as DELETED contributes nothing: its `+` lines are the
`+++` header, which is not an added line.

Usage:
  check_added_lines.py [--base REF] [--repo DIR]
  check_added_lines.py --diff FILE
  check_added_lines.py --self-test

`--base` defaults to `origin/main`. `--diff` reads a unified diff from a file or
from `-` (standard input) instead of running git, which is what the self-test
drives.

Exit codes:
  0  no added line carries a typographic dash, or the self-test passed
  1  an added line carries one, or the self-test found a miss
  2  malformed invocation, or git could not produce the diff

Stdlib only. Python 3.8 or newer.
"""
import argparse
import os
import subprocess
import sys
import tempfile

# The two characters, by code point rather than by glyph, so this file carries
# neither of them and can be read by its own gate.
EM_DASH = "—"  # dash-ok: the character this gate refuses
EN_DASH = "–"  # dash-ok: the character this gate refuses
# The two, by the name a refusal prints, so the report says WHICH character a
# line carries rather than showing a glyph a terminal may not render.
BANNED = (("U+2014", EM_DASH), ("U+2013", EN_DASH))

# The per-line waiver. Spelled once so the self-test and the scanner cannot
# drift apart.
DASH_OK_MARKER = "dash-ok"

DEFAULT_BASE = "origin/main"


class DiffError(Exception):
    """A refusal this script reports as exit 2."""


def added_lines(diff_text):
    """Every ADDED line of a unified diff, as `(path, line number, text)`.

    The line number is the line's position in the NEW file, tracked from each
    hunk header, so the report names a line a maintainer can open. A `+++`
    header is not an added line, and neither is anything outside a hunk.
    """
    out = []
    path = None
    line_number = 0
    in_hunk = False
    for raw in diff_text.splitlines():
        if raw.startswith("+++ "):
            target = raw[4:].strip()
            # `+++ /dev/null` is a deletion; `+++ b/<path>` is everything else.
            path = None if target == "/dev/null" else target[2:] if target.startswith("b/") else target
            in_hunk = False
            continue
        if raw.startswith("--- "):
            in_hunk = False
            continue
        if raw.startswith("@@"):
            # `@@ -<old>,<n> +<new>,<m> @@ <heading>`; the heading is context
            # from the file and must never be read as a diff line.
            marker = raw.split("@@")
            if len(marker) < 2:
                raise DiffError("check_added_lines: malformed hunk header: %r" % raw)
            fields = marker[1].split()
            new_field = next((f for f in fields if f.startswith("+")), None)
            if new_field is None:
                raise DiffError("check_added_lines: hunk header names no new range: %r" % raw)
            start = new_field[1:].split(",")[0]
            if not start.isdigit():
                raise DiffError("check_added_lines: hunk header has no line number: %r" % raw)
            line_number = int(start)
            in_hunk = True
            continue
        if not in_hunk:
            continue
        if raw.startswith("+"):
            if path is not None:
                out.append((path, line_number, raw[1:]))
            line_number += 1
        elif raw.startswith("-"):
            continue
        elif raw.startswith("\\"):
            # `\ No newline at end of file` belongs to the line before it.
            continue
        else:
            line_number += 1
    return out


def offending_lines(diff_text):
    """Every added line carrying a banned dash without the per-line waiver."""
    out = []
    for path, number, text in added_lines(diff_text):
        if DASH_OK_MARKER in text:
            continue
        found = [name for name, ch in BANNED if ch in text]
        if found:
            out.append((path, number, ", ".join(found), text))
    return out


def git_diff(base, repo):
    """`git diff <base>...HEAD` in `repo`, as text."""
    command = ["git", "diff", "--no-color", "%s...HEAD" % base]
    try:
        completed = subprocess.run(
            command, cwd=repo, capture_output=True, text=True, check=False)
    except OSError as error:
        raise DiffError("check_added_lines: cannot run git: %r" % error)
    if completed.returncode != 0:
        raise DiffError(
            "check_added_lines: `%s` failed in %s (exit %d): %s; fetch the base "
            "ref before running this gate"
            % (" ".join(command), repo, completed.returncode, completed.stderr.strip()))
    return completed.stdout


def report(offenders, where):
    """Print the refusal a maintainer reads. Returns an exit code."""
    if not offenders:
        print("check_added_lines: no added line carries a typographic dash (%s)" % where)
        return 0
    print("check_added_lines: %d added line(s) carry a typographic dash:"
          % len(offenders), file=sys.stderr)
    for path, number, which, text in offenders:
        print("  %s:%d  %s  %s" % (path, number, which, text.strip()), file=sys.stderr)
    print(
        "\nEvery line a pull request ADDS must spell a dash as a hyphen, a "
        "comma or a full stop. A line that genuinely needs one carries the "
        "marker `%s` on that line, as a comment in the file's own syntax."
        % DASH_OK_MARKER,
        file=sys.stderr)
    return 1


# ---------------------------------------------------------------------------
# Self-test.
#
# The diffs below are written out by hand, so no arm compares one run of the
# parser with another. The last arm is the other half: it builds a real
# repository, commits, edits, and runs the real `git diff`, which is the only
# arm that can catch a parser that is self-consistently wrong about the format
# git actually emits.
# ---------------------------------------------------------------------------

def _hunk(path, body_lines, start=1):
    """A one-file, one-hunk unified diff over `body_lines`."""
    added = sum(1 for line in body_lines if line.startswith("+"))
    kept = sum(1 for line in body_lines if line.startswith(" "))
    head = ["diff --git a/%s b/%s" % (path, path),
            "--- a/%s" % path,
            "+++ b/%s" % path,
            "@@ -%d,%d +%d,%d @@" % (start, kept, start, kept + added)]
    return "\n".join(head + body_lines) + "\n"


def self_test():
    """Both sides of the rule, then a real `git diff`. Returns an exit code."""
    failures = []
    arms = [0]

    def arm(name, ok, detail=""):
        arms[0] += 1
        if not ok:
            failures.append("%s %s" % (name, detail))

    # A planted dash on an ADDED line fails, in every file type the tree
    # carries, and each type is spelled with its own comment syntax so the
    # rows say what the answer is rather than sharing one fixture.
    planted = [
        ("test source", "crates/cerulion_core/tests/planted_test.rs",
         "+// a comment %s with a dash" % EM_DASH),
        ("script", "tools/scripts/planted.sh",
         "+# a comment %s with a dash" % EM_DASH),
        ("toml", "crates/cerulion_core/Cargo.toml",
         "+# a manifest comment %s with a dash" % EN_DASH),
        ("python", "tools/scripts/planted.py",
         "+PROSE = \"a string %s with a dash\"" % EM_DASH),
        ("workflow", ".github/workflows/planted.yml",
         "+      # a step note %s with a dash" % EM_DASH),
        ("markdown", "docs/planted.md",
         "+A sentence %s with a dash." % EM_DASH),
    ]
    for name, path, line in planted:
        got = offending_lines(_hunk(path, [" unchanged", line]))
        arm("planted-dash-on-an-added-%s-line" % name,
            len(got) == 1 and got[0][0] == path and got[0][1] == 2,
            "-> %r" % (got,))

    # The other side: an UNCHANGED line carrying a dash is not an added line,
    # and neither is a REMOVED one.
    untouched = _hunk("docs/planted.md",
                      [" A sentence %s with a dash." % EM_DASH,
                       "-A removed sentence %s with a dash." % EM_DASH,
                       "+A plain added sentence."])
    arm("an-unchanged-or-removed-line-with-a-dash-is-not-an-added-line",
        offending_lines(untouched) == [], "-> %r" % (offending_lines(untouched),))

    # The waiver, on the line and nowhere else.
    waived = _hunk("docs/planted.md",
                   ["+A quoted span `a %s b` <!-- %s -->" % (EM_DASH, DASH_OK_MARKER)])
    arm("a-waived-added-line-passes", offending_lines(waived) == [],
        "-> %r" % (offending_lines(waived),))
    # A waiver on one line does not cover the next one.
    neighbour = _hunk("docs/planted.md",
                      ["+A quoted span `a %s b` <!-- %s -->" % (EM_DASH, DASH_OK_MARKER),
                       "+A second sentence %s with a dash." % EM_DASH])
    got = offending_lines(neighbour)
    arm("a-waiver-covers-only-its-own-line", len(got) == 1 and got[0][1] == 2,
        "-> %r" % (got,))

    # Both dashes, named separately, so a rule that lost one is not hidden by
    # the other.
    for name, ch, which in (("em", EM_DASH, "U+2014"), ("en", EN_DASH, "U+2013")):
        got = offending_lines(_hunk("docs/planted.md", ["+A sentence %s here." % ch]))
        arm("the-%s-dash-is-named" % name,
            len(got) == 1 and got[0][2] == which, "-> %r" % (got,))
    both = offending_lines(_hunk("docs/planted.md",
                                 ["+A sentence %s and %s here." % (EM_DASH, EN_DASH)]))
    arm("a-line-with-both-names-both",
        len(both) == 1 and both[0][2] == "U+2014, U+2013", "-> %r" % (both,))

    # A hyphen is not a dash: the gate must not red an ordinary line.
    plain = _hunk("docs/planted.md", ["+A well-formed sentence with a hyphen."])
    arm("a-hyphen-is-not-a-dash", offending_lines(plain) == [],
        "-> %r" % (offending_lines(plain),))

    # A DELETED file's `+++ /dev/null` header contributes no added line, and a
    # hunk heading (the text after the second `@@`) is context, never a line.
    deleted = ("diff --git a/docs/gone.md b/docs/gone.md\n"
               "--- a/docs/gone.md\n"
               "+++ /dev/null\n"
               "@@ -1,1 +0,0 @@\n"
               "-A sentence %s with a dash.\n" % EM_DASH)
    arm("a-deleted-file-adds-nothing", offending_lines(deleted) == [],
        "-> %r" % (offending_lines(deleted),))
    heading = ("diff --git a/src/lib.rs b/src/lib.rs\n"
               "--- a/src/lib.rs\n"
               "+++ b/src/lib.rs\n"
               "@@ -10,2 +10,3 @@ fn f() %s the heading\n"
               " keep\n"
               "+added\n" % EM_DASH)
    arm("a-hunk-heading-is-not-an-added-line", offending_lines(heading) == [],
        "-> %r" % (offending_lines(heading),))

    # The NEW-file line number is tracked from the hunk header, so the report
    # names a line a maintainer can open.
    numbered = _hunk("docs/planted.md",
                     [" keep", " keep", "+A sentence %s here." % EM_DASH], start=40)
    got = offending_lines(numbered)
    arm("the-line-number-comes-from-the-hunk-header",
        len(got) == 1 and got[0][1] == 42, "-> %r" % (got,))

    # Two files in one diff are both read, and the second file's numbering
    # restarts from its own hunk header.
    two_files = (_hunk("a.md", ["+A sentence %s here." % EM_DASH], start=5)
                 + _hunk("b.md", [" keep", "+A sentence %s here." % EN_DASH], start=9))
    got = offending_lines(two_files)
    arm("two-files-in-one-diff",
        [(p, n) for p, n, _, _ in got] == [("a.md", 5), ("b.md", 10)], "-> %r" % (got,))

    # A malformed hunk header is a refusal, never a quiet zero: a diff this
    # reader cannot place is a diff whose added lines it cannot find.
    for name, text in (("no-new-range", "+++ b/a.md\n@@ -1,1 @@\n+x\n"),
                       ("no-line-number", "+++ b/a.md\n@@ -1,1 +x,1 @@\n+x\n")):
        try:
            added_lines(text)
        except DiffError:
            arm("a-malformed-hunk-header-is-refused-%s" % name, True)
        else:
            arm("a-malformed-hunk-header-is-refused-%s" % name, False,
                "-> parsed without a refusal")

    # THE REAL GIT ARM. Everything above is a hand-written diff; this one
    # proves the parser reads the format git actually writes, and that the
    # three-dot spelling reports only what THIS branch added.
    with tempfile.TemporaryDirectory() as scratch:
        def git(*args):
            done = subprocess.run(["git"] + list(args), cwd=scratch,
                                  capture_output=True, text=True, check=False)
            if done.returncode != 0:
                raise DiffError("git %s failed: %s" % (" ".join(args), done.stderr.strip()))
            return done.stdout

        def write(name, text):
            with open(os.path.join(scratch, name), "w", encoding="utf-8") as handle:
                handle.write(text)

        git("init", "--quiet", "-b", "base")
        git("config", "user.email", "gate@example.invalid")
        git("config", "user.name", "gate")
        git("config", "commit.gpgsign", "false")
        write("kept.md", "A base sentence %s with a dash.\n" % EM_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "base")
        git("checkout", "--quiet", "-b", "work")
        write("added.md", "A clean sentence.\nA sentence %s with a dash.\n" % EM_DASH)
        write("waived.md", "A span `a %s b` <!-- %s -->\n" % (EM_DASH, DASH_OK_MARKER))
        git("add", "-A")
        git("commit", "--quiet", "-m", "work")
        got = offending_lines(git_diff("base", scratch))
        arm("a-real-git-diff-reports-the-added-dash-only",
            [(p, n) for p, n, _, _ in got] == [("added.md", 2)], "-> %r" % (got,))

        # The base moves on with a dash of its own; the three-dot diff must
        # still report only this branch's line.
        git("checkout", "--quiet", "base")
        write("base_moved.md", "A later base sentence %s with a dash.\n" % EN_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "base moves")
        git("checkout", "--quiet", "work")
        got = offending_lines(git_diff("base", scratch))
        arm("the-three-dot-diff-ignores-what-the-base-gained",
            [(p, n) for p, n, _, _ in got] == [("added.md", 2)], "-> %r" % (got,))

    for failure in failures:
        print("SELF-TEST FAILED: " + failure, file=sys.stderr)
    if failures:
        print("check_added_lines: %d of %d self-test arm(s) failed"
              % (len(failures), arms[0]), file=sys.stderr)
        return 1
    print("check_added_lines: self-test OK (%d arms)" % arms[0])
    return 0


def run(argv):
    """The command line. Returns an exit code."""
    parser = argparse.ArgumentParser(
        prog="check_added_lines.py",
        description="Refuse a typographic dash on any line a pull request adds.")
    parser.add_argument("--base", default=DEFAULT_BASE, metavar="REF",
                        help="the base ref to diff against (default: %s)" % DEFAULT_BASE)
    parser.add_argument("--repo", default=".", metavar="DIR",
                        help="the repository to diff in (default: the working directory)")
    parser.add_argument("--diff", metavar="FILE",
                        help="read a unified diff from FILE, or `-` for stdin, instead of git")
    parser.add_argument("--self-test", action="store_true",
                        help="run the self-test and exit")
    args = parser.parse_args(argv)

    if args.self_test:
        if args.diff or args.base != DEFAULT_BASE or args.repo != ".":
            parser.error("--self-test takes no other argument")
        return self_test()

    try:
        if args.diff:
            text = sys.stdin.read() if args.diff == "-" else open(
                args.diff, encoding="utf-8").read()
            where = args.diff
        else:
            text = git_diff(args.base, args.repo)
            where = "%s...HEAD" % args.base
        return report(offending_lines(text), where)
    except (DiffError, OSError) as error:
        print(error if isinstance(error, DiffError)
              else "check_added_lines: cannot read the diff: %r" % error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))
