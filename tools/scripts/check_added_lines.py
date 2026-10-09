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
a file-level waiver would exempt every line added to it afterwards. INSIDE the
comment, and enforced: the marker in a string literal, a data column or a
heading is text the change ships rather than a decision its author recorded, and
a file type with no comment syntax at all (`.json`) has no waiver.

WHAT IT READS. `git diff <base>...HEAD`, which diffs against the MERGE BASE of
`<base>` and HEAD. The three dots matter only when the base is NOT an ancestor
of HEAD, which is a base on a target branch somebody rewrote and a base that is
an ordinary branch tip: two dots there report every line that branch gained
since this one left it as a line this branch added. Whenever the base IS an
ancestor of HEAD, and the first parent this file resolves to always is, the
three dots collapse to two and change nothing.

THE BASE IS RESOLVED HERE, not taken on trust. `base.sha` on a `pull_request`
event is a snapshot of the target branch that GitHub does not keep in step with
the merge the checkout is built on, and diffing against it reports every line
the target branch gained in between as a line this pull request added. GitHub
builds the `refs/pull/N/merge` commit server-side and `actions/checkout` checks
it out; its FIRST parent is the tip the branch was merged onto, so that is the
base the gate uses. The value handed in is the fallback, and it is what a branch
checkout uses. Which of the two applies is decided by an ancestor test,
described at `resolve_base`. The gate prints the base it uses and where that
base comes from.

A file the diff reports as DELETED contributes nothing: its `+` lines are the
`+++` header, which is not an added line.

Usage:
  check_added_lines.py [--base REF] [--repo DIR]
  check_added_lines.py --diff FILE
  check_added_lines.py --self-test

`--base` defaults to `origin/main` and is the FALLBACK: on the merge checkout of
a pull request the first parent usually wins and the diff does not run against
this value. The probe runs once HEAD^2 and HEAD^1 resolve; either way a value it
cannot read becomes the base and refuses with exit 2. `--diff` reads a diff from
a file or from `-` (standard input) instead of running git, which is what the
self-test drives.
THE INPUT IS `git diff` OUTPUT in either mode: a hunk that arrives before any
`diff --git` file header is refused with exit 2, never read as an empty diff.

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

# The two characters, spelled by CODE POINT rather than by glyph, so this file
# carries neither of them: the gate reads its own source like any other, and a
# glyph here would need the per-line waiver to survive it. An escape needs
# nothing, and the added-line count over a diff that touches this file reads
# zero.
EM_DASH = "\u2014"
EN_DASH = "\u2013"
# The two, by the name a refusal prints, so the report says WHICH character a
# line carries rather than showing a glyph a terminal may not render.
BANNED = (("U+2014", EM_DASH), ("U+2013", EN_DASH))

# The per-line waiver. Spelled once so the self-test and the scanner cannot
# drift apart.
DASH_OK_MARKER = "dash-ok"

# THE COMMENT SYNTAX THE WAIVER HAS TO SIT IN, by file extension.
#
# The marker is a note to the reader, so it belongs in a comment. As a bare
# substring of the line it would also match a string literal, a data column or
# a heading, which are shipped text: a `.tsv` row or a Rust string carrying the
# words `dash-ok` would waive a dash no comment marks.
#
# An extension this table does not name has NO waiver: a `.json` line that needs
# a dash has nowhere to put a comment, and a token accepted anywhere is no
# waiver rule at all. A path with NO extension is read as shell, which is what
# the extensionless hooks under `tools/hooks` are.
#
# The reader does not tokenise strings, so an opener spelled EARLIER in a string
# literal on the same line is read as opening a comment. That is a named
# limitation and it is strictly narrower than accepting the token anywhere: the
# author has to write a comment opener in front of the marker either way.
COMMENT_OPENERS = {
    "py": ("#",),
    "sh": ("#",),
    "bash": ("#",),
    "zsh": ("#",),
    "yml": ("#",),
    "yaml": ("#",),
    "toml": ("#",),
    "txt": ("#",),
    "tsv": ("#",),
    "rs": ("//", "/*"),
    "c": ("//", "/*"),
    "h": ("//", "/*"),
    "cpp": ("//", "/*"),
    "hpp": ("//", "/*"),
    "js": ("//", "/*"),
    "ts": ("//", "/*"),
    "md": ("<!--",),
    "html": ("<!--",),
    "xml": ("<!--",),
    "sql": ("--",),
    "lua": ("--",),
    "ini": (";",),
    "cfg": (";",),
}

# What a path with no extension is read as.
SHELL_OPENERS = ("#",)


def comment_openers_for(path):
    """The comment openers this path's file type uses, empty for no waiver."""
    name = path.rsplit("/", 1)[-1]
    if "." not in name:
        return SHELL_OPENERS
    return COMMENT_OPENERS.get(name.rsplit(".", 1)[-1].lower(), ())


def waived(path, text):
    """Does this added line carry the waiver INSIDE a comment?

    The marker has to follow a comment opener the file type uses. The first
    opener on the line is the one that counts: everything after it is comment as
    far as this reader can tell.
    """
    at = text.find(DASH_OK_MARKER)
    if at < 0:
        return False
    for opener in comment_openers_for(path):
        opened = text.find(opener)
        if 0 <= opened < at:
            return True
    return False

DEFAULT_BASE = "origin/main"

# The environment the SELF-TEST's fixture repositories run git in, and only
# them: the real run keeps the host environment, because `actions/checkout`
# writes `safe.directory` into the GLOBAL configuration and a run that parked it
# could stop reading the repository it was pointed at wherever the workspace
# ownership differs from the step's user (a container job, a self-hosted runner).
#
# A fixture that reads the machine it runs on proves nothing about the gate, and
# git takes host settings by TWO routes. The configuration FILES are parked by
# pointing both paths at an empty device: `merge.verifySignatures = true` makes
# the fixture merges refuse and a `diff.external` sends the fixture diffs
# through somebody else's program. The ENVIRONMENT carries its own:
# a `GIT_EXTERNAL_DIFF` does what `diff.external` does, and a
# `GIT_CONFIG_PARAMETERS` carries whole settings no file holds, so both are
# REMOVED rather than emptied: an empty `GIT_EXTERNAL_DIFF` is a program git
# tries to run and fails on.
# `GIT_CONFIG_COUNT=0` says the environment carries no settings of its own.
#
# THE LIMIT: the configuration routes are parked; the location variables
# (`GIT_DIR`, `GIT_WORK_TREE`) are not, and a host that sets them reds the
# self-test rather than passing it; a `git` that is an old version or a wrapper
# on `PATH` still runs the fixtures.
FIXTURE_GIT_ENV = {k: v for k, v in os.environ.items()
                   if k not in ("GIT_EXTERNAL_DIFF", "GIT_CONFIG_PARAMETERS")}
FIXTURE_GIT_ENV.update(GIT_CONFIG_GLOBAL=os.devnull,
                       GIT_CONFIG_SYSTEM=os.devnull,
                       GIT_CONFIG_COUNT="0")


class DiffError(Exception):
    """A refusal this script reports as exit 2."""


def added_lines(diff_text):
    """Every ADDED line of a unified diff, as `(path, line number, text)`.

    The line number is the line's position in the NEW file, tracked from each
    hunk header, so the report names a line a maintainer can open. A `+++`
    header is not an added line, and neither is anything outside a hunk.

    A hunk that arrives before any `diff --git` line is a REFUSAL. This reader
    takes `git diff` output, and a diff it cannot place is a diff whose added
    lines it cannot find, so it says so rather than reporting nothing.
    """
    out = []
    path = None
    line_number = 0
    in_hunk = False
    in_header = False
    # A `diff --git` line opened the current file. That is a different fact
    # from holding a path: a deletion is `+++ /dev/null`, so `path` is None
    # there too, and a reader that keeps only the path cannot tell a deletion
    # apart from a diff that names no file at all.
    header_seen = False
    for raw in diff_text.splitlines():
        if raw.startswith("diff --git "):
            # A file's header opens here and runs to that file's first `@@`.
            in_header = True
            in_hunk = False
            path = None
            header_seen = True
            continue
        if in_header and raw.startswith("+++ "):
            target = raw[4:].strip()
            # `+++ /dev/null` is a deletion; `+++ b/<path>` is everything else.
            path = None if target == "/dev/null" else target[2:] if target.startswith("b/") else target
            continue
        if in_header and raw.startswith("--- "):
            continue
        if raw.startswith("@@"):
            if not header_seen:
                raise DiffError(
                    "check_added_lines: hunk header before any `diff --git` file "
                    "header; this reader takes `git diff` output")
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
            in_header = False
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
        if waived(path, text):
            continue
        found = [name for name, ch in BANNED if ch in text]
        if found:
            out.append((path, number, ", ".join(found), text))
    return out


def decode_diff(raw):
    """A diff's bytes as text, whatever bytes they are.

    BYTES, DECODED HERE, never `text=True`. A diff can carry a line that is not
    valid UTF-8: a source file in another encoding, a binary hunk git spelled as
    text, a pasted byte. `text=True` decodes with the locale codec and raises
    `UnicodeDecodeError`, which is neither `DiffError` nor `OSError`, so the gate
    died with a traceback instead of reporting. `surrogateescape` keeps every
    byte, so the scan still reads every other line and still names the dash on
    it: an undecodable byte on one line is not a reason to stop reading the
    diff, and it is not a pass either.
    """
    return raw.decode("utf-8", errors="surrogateescape")


def git_output(args, repo, env=None):
    """One git command in `repo`, as text, or None when it exits non-zero.

    `env` is the whole environment to run in, or None for this process's own,
    which is what the real run uses.
    """
    try:
        completed = subprocess.run(
            ["git"] + args, cwd=repo, env=env, capture_output=True, check=False)
    except OSError as error:
        raise DiffError("check_added_lines: cannot run git: %r" % error)
    if completed.returncode != 0:
        return None
    return decode_diff(completed.stdout).strip()


def resolve_base(given, repo, env=None):
    """The base to diff against, and the phrase saying where it comes from.

    THE PRECONDITION. On a merge commit, `HEAD^1` is the base when `given` is a
    commit the branch has NOT itself merged. On the `refs/pull/N/merge` commit
    GitHub builds and `actions/checkout` checks out, `given` is `base.sha`, a
    snapshot of the target branch, and the branch side of that merge sits under
    `HEAD^2`, so the precondition holds by construction. Where it does not hold,
    taking `HEAD^1` would hide every line the branch added before its own last
    merge, so the value handed in stays the base.

    THE ANCESTOR TEST is how the precondition is checked. `given` REACHABLE from
    the first parent is the case this run acts on: this run then has not seen the
    first parent as the branch's own work and takes it as the base; a branch that
    has already merged `given` into itself defeats the test, which the merge-ref
    shape rules out.
    UNREACHABLE, and this run cannot tell the merge ref apart from a merge the
    pull request's branch made itself, so it stops trusting the first parent and
    the value handed in stays the base. Each reason string below reports what
    was observed and names where the base came from.
    """
    if git_output(["rev-parse", "--verify", "-q", "HEAD^2"], repo, env) is None:
        return given, "the value handed to --base (HEAD^2 did not resolve)"
    parent = git_output(["rev-parse", "--verify", "HEAD^1"], repo, env)
    if parent is None:
        return given, "the value handed to --base (HEAD^1 did not resolve)"
    # PROBED SEPARATELY, because `merge-base --is-ancestor` exits non-zero both
    # for "no ancestor" and for a revision it cannot resolve, so one reason
    # string for the two would state a branch-history fact this run never saw.
    if git_output(["rev-parse", "--verify", "-q", "%s^{commit}" % given], repo, env) is None:
        return given, "the value handed to --base (%s did not resolve)" % given
    if git_output(["merge-base", "--is-ancestor", given, parent], repo, env) is None:
        return given, "the value handed to --base (%s is not reachable from HEAD^1)" % given
    return parent, "the first parent of the merge commit HEAD"


def git_diff(base, repo, env=None):
    """`git diff <base>...HEAD` in `repo`, as text.

    `env` as in `git_output`: None for this process's own environment.
    """
    command = ["git", "diff", "--no-color", "%s...HEAD" % base]
    try:
        completed = subprocess.run(command, cwd=repo, env=env,
                                   capture_output=True, check=False)
    except OSError as error:
        raise DiffError("check_added_lines: cannot run git: %r" % error)
    if completed.returncode != 0:
        raise DiffError(
            "check_added_lines: `%s` failed in %s (exit %d): %s; fetch the base "
            "ref before running this gate"
            % (" ".join(command), repo, completed.returncode,
               decode_diff(completed.stderr).strip()))
    return decode_diff(completed.stdout)


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
# parser with another. Two blocks near the end are the other half, and they
# build real repositories: the first commits, edits and runs the real
# `git diff`, and the second also merges.
# `a-real-git-diff-reports-the-added-dash-only` is the arm whose job is to catch
# a parser self-consistently wrong about the format git actually emits, and the
# base arms in the second block are the ones whose job is to catch a base
# resolved from the wrong commit.
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

    def silently(thunk):
        """Run `thunk` with both streams captured: `(what it returned, output)`."""
        with tempfile.TemporaryDirectory() as parked_dir:
            path = os.path.join(parked_dir, "streams")
            out, err = sys.stdout, sys.stderr
            with open(path, "w", encoding="utf-8") as parked:
                try:
                    sys.stdout, sys.stderr = parked, parked
                    code = thunk()
                finally:
                    sys.stdout, sys.stderr = out, err
            with open(path, encoding="utf-8") as parked:
                return code, parked.read()

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

    # THE WAIVER HAS TO SIT IN A COMMENT, in the syntax the file type uses. Each
    # row is the same sentence twice: once with the marker behind that type's
    # comment opener, once with the same token in a string literal or a data
    # column, where it records no decision and waives nothing.
    for path, commented, in_content in (
        ("docs/planted.md",
         "+A span `a %s b` <!-- %s -->",
         "+A heading about the %s and the word %s"),
        ("crates/cerulion_core/src/planted.rs",
         "+let s = \"plain\"; // a span a %s b, %s",
         "+let s = \"a span a %s b, %s\";"),
        ("tools/scripts/planted.py",
         "+VALUE = 1  # a span a %s b, %s",
         "+VALUE = \"a span a %s b, %s\""),
        ("tools/ci/planted.tsv",
         "+# a span a %s b, %s",
         "+column\ta span a %s b\t%s"),
        ("db/planted.sql",
         "+SELECT 1; -- a span a %s b, %s",
         "+INSERT INTO t VALUES ('a span a %s b, %s');"),
        ("tools/planted.cfg",
         "+key = 1  ; a span a %s b, %s",
         "+key = a span a %s b, %s"),
    ):
        kind = path.rsplit(".", 1)[-1]
        inside = _hunk(path, [commented % (EM_DASH, DASH_OK_MARKER)])
        arm("a-%s-waiver-inside-a-comment-passes" % kind,
            offending_lines(inside) == [], "-> %r" % (offending_lines(inside),))
        outside = _hunk(path, [in_content % (EM_DASH, DASH_OK_MARKER)])
        arm("a-%s-marker-outside-a-comment-waives-nothing" % kind,
            len(offending_lines(outside)) == 1, "-> %r" % (offending_lines(outside),))

    # A FILE TYPE WITH NO COMMENT SYNTAX HAS NO WAIVER: there is nowhere to put
    # the marker, so accepting it anywhere on the line is the rule this replaces.
    json_line = _hunk("tools/ci/planted.json",
                      ["+  \"note\": \"a span a %s b, %s\"" % (EM_DASH, DASH_OK_MARKER)])
    arm("a-file-type-with-no-comment-syntax-has-no-waiver",
        len(offending_lines(json_line)) == 1, "-> %r" % (offending_lines(json_line),))
    # And a path with NO extension is read as shell, which is what the
    # extensionless hooks are.
    hook = _hunk("tools/hooks/pre-commit",
                 ["+true  # a span a %s b, %s" % (EM_DASH, DASH_OK_MARKER)])
    arm("an-extensionless-path-waives-behind-a-hash",
        offending_lines(hook) == [], "-> %r" % (offending_lines(hook),))

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

    # A REMOVED LINE IS NOT A FILE HEADER, and this is the arm that matters. A
    # line whose CONTENT begins `-- ` (a SQL or Lua comment, a signature
    # separator, a `--` flag in a shell block) is spelled `--- ` in a unified
    # diff, byte for byte a file header. Read as one it closed the hunk, and
    # the rest of THAT HUNK escaped the gate: the next `@@` reopens the reader
    # and restores the line number, so a later hunk of the same file is read
    # again and a one-hunk file loses every added line after the marker. A
    # false green in the one direction this gate exists to close. A reader that
    # closes a hunk on such a line reports nothing at all for this fixture.
    removed_marker = _hunk("db/schema.sql",
                           [" keep",
                            "--- a comment the change removes",
                            "+A sentence %s here." % EM_DASH])
    got = offending_lines(removed_marker)
    arm("a-removed-line-spelled-like-a-file-header-does-not-close-the-hunk",
        [(p, n) for p, n, _, _ in got] == [("db/schema.sql", 2)], "-> %r" % (got,))
    # The other side: the same hunk, same removed line, no dash on the added
    # one, passes. Without this the arm above is satisfied by a reader that
    # reports every line of every hunk.
    clean = _hunk("db/schema.sql",
                  [" keep", "--- a comment the change removes", "+A plain sentence."])
    arm("the-same-hunk-without-a-dash-passes", offending_lines(clean) == [],
        "-> %r" % (offending_lines(clean),))
    # One hyphen fewer is content too.
    two = _hunk("db/schema.sql",
                [" keep", "-- a list item the change removes",
                 "+A sentence %s here." % EM_DASH])
    got = offending_lines(two)
    arm("a-removed-line-opening-with-two-hyphens-does-not-close-the-hunk",
        [(p, n) for p, n, _, _ in got] == [("db/schema.sql", 2)], "-> %r" % (got,))
    # And an ADDED line beginning `++ ` is content too, for the same reason.
    plus = _hunk("db/schema.sql",
                 [" keep", "++ an added line %s that opens with two plus signs" % EN_DASH,
                  "+A sentence %s here." % EM_DASH])
    got = offending_lines(plus)
    arm("an-added-line-opening-with-plus-signs-stays-in-the-file",
        [(p, n) for p, n, _, _ in got] == [("db/schema.sql", 2), ("db/schema.sql", 3)],
        "-> %r" % (got,))

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
    # reader cannot place is a diff whose added lines it cannot find. Each
    # fixture opens with the `diff --git` line so the refusal it proves is the
    # RANGE one and not the missing-header one the arms below cover.
    header = "diff --git a/a.md b/a.md\n"
    for name, text in (("no-new-range", header + "+++ b/a.md\n@@ -1,1 @@\n+x\n"),
                       ("no-line-number", header + "+++ b/a.md\n@@ -1,1 +x,1 @@\n+x\n")):
        try:
            added_lines(text)
        except DiffError:
            arm("a-malformed-hunk-header-is-refused-%s" % name, True)
        else:
            arm("a-malformed-hunk-header-is-refused-%s" % name, False,
                "-> parsed without a refusal")

    # A DIFF THAT NAMES NO FILE is the same refusal for the same reason. The
    # `--- `/`+++ ` pair alone is a unified diff any patch tool writes; this
    # reader takes `git diff`, so a hunk opening with no `diff --git` line in
    # front of it is a diff it cannot place. Reading it as zero added lines
    # printed a pass over a diff nothing had scanned.
    headerless = ("--- a/docs/planted.md\n"
                  "+++ b/docs/planted.md\n"
                  "@@ -1,1 +1,2 @@\n"
                  " keep\n"
                  "+A sentence %s here.\n" % EM_DASH)
    try:
        added_lines(headerless)
    except DiffError:
        arm("a-diff-with-no-git-file-header-is-refused", True)
    else:
        arm("a-diff-with-no-git-file-header-is-refused", False,
            "-> parsed without a refusal")
    # The other side: the same bytes behind the header report the dash, so the
    # refusal above is about the missing header and not about the content.
    placed = offending_lines("diff --git a/docs/planted.md b/docs/planted.md\n"
                             + headerless)
    arm("the-same-diff-behind-a-git-file-header-reports-the-dash",
        [(p, n, w) for p, n, w, _ in placed] == [("docs/planted.md", 2, "U+2014")],
        "-> %r" % (placed,))
    # A DELETION is not a headerless diff: its header opened the file, its path
    # is None because the new side is `/dev/null`, and its removed lines are
    # read and contribute nothing. Refusing on the path rather than on the
    # header would red every pull request that deletes a file.
    deletion = ("diff --git a/docs/gone.md b/docs/gone.md\n"
                "--- a/docs/gone.md\n"
                "+++ /dev/null\n"
                "@@ -1,2 +0,0 @@\n"
                "-A sentence %s here.\n"
                "-keep\n" % EM_DASH)
    arm("a-deletion-only-diff-is-clean-and-not-a-refusal",
        offending_lines(deletion) == [], "-> %r" % (offending_lines(deletion),))

    # AN UNDECODABLE BYTE IS NOT A REASON TO STOP READING. A diff can carry a
    # line that is not valid UTF-8, and `text=True` raised `UnicodeDecodeError`
    # out of the reader, past a handler that catches only `DiffError` and
    # `OSError`: the gate died with a traceback rather than reporting. The bytes
    # are decoded with `surrogateescape` instead, so the dash on ANOTHER line is
    # still found and `report()` still returns an exit code.
    raw = _hunk("docs/planted.md",
                [" keep",
                 "+A latin-1 byte: BYTE here.",
                 "+A sentence %s here." % EM_DASH]).encode("utf-8")
    planted = raw.replace(b"BYTE", b"\xe9")
    try:
        got = offending_lines(decode_diff(planted))
    except UnicodeDecodeError as error:
        arm("an-undecodable-byte-on-another-line-still-reports-the-dash", False,
            "-> the reader raised %r instead of reading the diff" % (error,))
    else:
        arm("an-undecodable-byte-on-another-line-still-reports-the-dash",
            [(p, n, w) for p, n, w, _ in got] == [("docs/planted.md", 3, "U+2014")],
            "-> %r" % (got,))
    # And the WHOLE command line over that diff returns an exit code rather
    # than raising: the handler in `run()` catches `DiffError` and `OSError`, so
    # a decode that raised anything else came out as a traceback.
    with tempfile.TemporaryDirectory() as scratch:
        undecodable = os.path.join(scratch, "undecodable.diff")
        with open(undecodable, "wb") as handle:
            handle.write(planted)
        try:
            code, _ = silently(lambda: run(["--diff", undecodable]))
        except UnicodeDecodeError as error:
            code = "raised %r" % (error,)
    arm("the-command-line-over-an-undecodable-diff-returns-an-exit-code",
        code == 1, "-> %r" % (code,))

    # THE REAL GIT ARM. Everything above is a hand-written diff; this one
    # proves the parser reads the format git actually writes, and that the
    # three-dot spelling reports only what THIS branch added.
    with tempfile.TemporaryDirectory() as scratch:
        def git(*args):
            done = subprocess.run(["git"] + list(args), cwd=scratch, env=FIXTURE_GIT_ENV,
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
        got = offending_lines(git_diff("base", scratch, FIXTURE_GIT_ENV))
        arm("a-real-git-diff-reports-the-added-dash-only",
            [(p, n) for p, n, _, _ in got] == [("added.md", 2)], "-> %r" % (got,))

        # The base moves on with a dash of its own; the three-dot diff must
        # still report only this branch's line.
        git("checkout", "--quiet", "base")
        write("base_moved.md", "A later base sentence %s with a dash.\n" % EN_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "base moves")
        git("checkout", "--quiet", "work")
        got = offending_lines(git_diff("base", scratch, FIXTURE_GIT_ENV))
        arm("the-three-dot-diff-ignores-what-the-base-gained",
            [(p, n) for p, n, _, _ in got] == [("added.md", 2)], "-> %r" % (got,))

    # THE BASE ARMS, on a real repository shaped like the checkout a pull request
    # gets. `main` carries a file, a branch adds two lines with a dash, `main`
    # then advances with a DIFFERENT dashed line, and the checkout is the MERGE
    # of the branch onto that newer `main`. The value handed in is the older
    # snapshot of `main`, which is the shape `base.sha` has: a diff against it
    # reports the line `main` gained as a line this branch added, because the
    # merge base of that snapshot and a merge commit descending from it IS the
    # snapshot and the three dots collapse to two.
    with tempfile.TemporaryDirectory() as scratch:
        def git(*args):
            done = subprocess.run(["git"] + list(args), cwd=scratch, env=FIXTURE_GIT_ENV,
                                  capture_output=True, text=True, check=False)
            if done.returncode != 0:
                raise DiffError("git %s failed: %s" % (" ".join(args), done.stderr.strip()))
            return done.stdout.strip()

        def write(name, text):
            with open(os.path.join(scratch, name), "w", encoding="utf-8") as handle:
                handle.write(text)

        git("init", "--quiet", "-b", "main")
        git("config", "user.email", "gate@example.invalid")
        git("config", "user.name", "gate")
        git("config", "commit.gpgsign", "false")
        write("kept.md", "A base sentence.\n")
        git("add", "-A")
        git("commit", "--quiet", "-m", "base")
        stale = git("rev-parse", "HEAD")

        # TWO commits on the branch, so its head's FIRST parent is not the base
        # handed in: an arm whose fallback happens to equal `HEAD^1` says
        # nothing about which of the two the resolver read.
        git("checkout", "--quiet", "-b", "work")
        write("branch.md", "A branch sentence %s here.\n" % EM_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "the branch")
        write("branch_again.md", "A second branch sentence %s here.\n" % EM_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "the branch again")
        branch_head = git("rev-parse", "HEAD")

        git("checkout", "--quiet", "main")
        write("landed.md", "A sentence somebody else landed %s here.\n" % EN_DASH)
        git("add", "-A")
        git("commit", "--quiet", "-m", "main moves on")
        main_tip = git("rev-parse", "HEAD")

        git("merge", "--quiet", "--no-ff", "-m", "merge the branch", branch_head)
        base, why = resolve_base(stale, scratch, FIXTURE_GIT_ENV)
        arm("a-merge-checkout-resolves-its-base-to-the-first-parent",
            base == git("rev-parse", "HEAD^1") and "first parent" in why,
            "-> %r %r" % (base, why))
        got = offending_lines(git_diff(base, scratch, FIXTURE_GIT_ENV))
        arm("a-merge-checkout-reports-only-the-branchs-own-added-dashes",
            [(p, n) for p, n, _, _ in got]
            == [("branch.md", 1), ("branch_again.md", 1)], "-> %r" % (got,))
        # What the base handed in would have reported on that same checkout, so
        # the arm above is not satisfied by a gate that reports nothing: the
        # line somebody ELSE landed comes back as a line this branch added.
        stale_got = offending_lines(git_diff(stale, scratch, FIXTURE_GIT_ENV))
        arm("the-base-handed-in-would-report-what-the-target-branch-gained",
            [(p, n) for p, n, _, _ in stale_got]
            == [("branch.md", 1), ("branch_again.md", 1), ("landed.md", 1)],
            "-> %r" % (stale_got,))

        # THE OTHER SIDE: a checkout that is no merge keeps the base handed in,
        # which here is neither `HEAD` nor `HEAD^1`, so the arm says which of
        # the two the resolver read.
        git("checkout", "--quiet", branch_head)
        fallback, why = resolve_base(stale, scratch, FIXTURE_GIT_ENV)
        arm("a-non-merge-checkout-keeps-the-base-handed-in",
            fallback == stale
            and fallback != git("rev-parse", "HEAD^1")
            and "HEAD^2 did not resolve" in why,
            "-> %r %r" % (fallback, why))
        got = offending_lines(git_diff(fallback, scratch, FIXTURE_GIT_ENV))
        arm("a-non-merge-checkout-reports-every-line-its-given-base-makes-added",
            [(p, n) for p, n, _, _ in got]
            == [("branch.md", 1), ("branch_again.md", 1)], "-> %r" % (got,))

        # A BRANCH THAT MERGES THE TARGET BRANCH INTO ITSELF is a merge commit
        # too, and its first parent is the branch's own previous head. Taking
        # that as the base would report NOTHING for every line the branch added
        # before the merge, which is the fail-open direction. The snapshot
        # handed in is a target-branch commit the branch had not got, so it is
        # no ancestor of that first parent, and that is what tells the two
        # merges apart.
        git("checkout", "--quiet", "work")
        git("merge", "--quiet", "--no-ff", "-m",
            "merge the target branch into the branch", main_tip)
        inward, why = resolve_base(main_tip, scratch, FIXTURE_GIT_ENV)
        arm("a-target-branch-merged-into-the-branch-keeps-the-base-handed-in",
            inward == main_tip
            and inward != git("rev-parse", "HEAD^1")
            and "not reachable from HEAD^1" in why,
            "-> %r %r" % (inward, why))
        got = offending_lines(git_diff(inward, scratch, FIXTURE_GIT_ENV))
        arm("a-target-branch-merged-into-the-branch-still-reports-the-branchs-dashes",
            [(p, n) for p, n, _, _ in got]
            == [("branch.md", 1), ("branch_again.md", 1)], "-> %r" % (got,))

        # A BASE THAT DOES NOT RESOLVE says so. `merge-base --is-ancestor` exits
        # non-zero for a revision it cannot read as well as for one that is no
        # ancestor, so without its own probe the run reports a missing ref as a
        # branch-history fact. The whole command line refuses with exit 2 rather
        # than reading an empty diff.
        git("checkout", "--quiet", "main")
        missing, why = resolve_base("origin/no-such-ref", scratch, FIXTURE_GIT_ENV)
        arm("a-base-that-does-not-resolve-is-reported-as-not-resolving",
            missing == "origin/no-such-ref" and "did not resolve" in why
            and "reachable" not in why, "-> %r %r" % (missing, why))
        code, _ = silently(
            lambda: run(["--base", "origin/no-such-ref", "--repo", scratch]))
        arm("a-base-that-does-not-resolve-refuses-rather-than-reading-nothing",
            code == 2, "-> %r" % (code,))
        for where, head in (("merge", "main"), ("non-merge", branch_head)):
            git("checkout", "--quiet", head)
            code, said = silently(lambda: run(["--base", "", "--repo", scratch]))
            arm("an-empty-base-refuses-rather-than-reading-nothing-on-a-%s-head" % where,
                code == 2 and "no added line" not in said, "-> %r %r" % (code, said))

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
                        help="the base ref this run falls back to; on the merge "
                             "checkout of a pull request the first parent "
                             "usually wins, but an unreadable value here still "
                             "refuses (default: %s)" % DEFAULT_BASE)
    parser.add_argument("--repo", default=".", metavar="DIR",
                        help="the repository to diff in (default: the working directory)")
    parser.add_argument("--diff", metavar="FILE",
                        help="read `git diff` output from FILE, or `-` for stdin, instead of git")
    parser.add_argument("--self-test", action="store_true",
                        help="run the self-test and exit")
    args = parser.parse_args(argv)

    if args.self_test and (args.diff or args.base != DEFAULT_BASE
                           or args.repo != "."):
        parser.error("--self-test takes no other argument")

    try:
        # INSIDE the try, so a fixture repository whose git cannot run is a
        # refusal with exit 2 and one line, never a traceback.
        if args.self_test:
            return self_test()
        if args.diff:
            raw = (sys.stdin.buffer.read() if args.diff == "-"
                   else open(args.diff, "rb").read())
            text = decode_diff(raw)
            where = args.diff
        else:
            base, why = resolve_base(args.base, args.repo)
            # FLUSHED, because Python block-buffers stdout into a pipe and the
            # refusal below goes to stderr: without this the log shows the
            # offending lines before the base they were measured against.
            print("check_added_lines: base %s, %s" % (base, why))
            sys.stdout.flush()
            # The EMPTY string is the one unreadable value `git diff` accepts:
            # `...HEAD` is a valid range over HEAD alone, so it reads clean.
            if git_output(["rev-parse", "--verify", "-q", "%s^{commit}" % base],
                          args.repo) is None:
                raise DiffError(
                    "check_added_lines: the base %r names no commit in %s; pass a "
                    "base this repository carries" % (base, args.repo))
            text = git_diff(base, args.repo)
            where = "%s...HEAD" % base
        return report(offending_lines(text), where)
    except (DiffError, OSError) as error:
        print(error if isinstance(error, DiffError)
              else "check_added_lines: cannot read the diff: %r" % error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))
