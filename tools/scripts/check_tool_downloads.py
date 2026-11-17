#!/usr/bin/env python3
"""check_tool_downloads.py: curl downloads in the workflows and the tool fetchers are retried and bounded.

    python3 tools/scripts/check_tool_downloads.py
    python3 tools/scripts/check_tool_downloads.py PATH...
    python3 tools/scripts/check_tool_downloads.py --self-test

With no PATH it reads every workflow under `.github/workflows` and the tool
and toolchain fetchers under `tools/scripts` (`install_*.sh` and
`build_*_container.sh`), relative to the repository this file sits in. It
reads no Dockerfile: a `curl` in a `RUN` line is outside what it checks.

A DOWNLOAD is a `curl` command whose bytes are kept: written with
`-o`/`--output`/`-O`, or sent to stdout, a pipe or a command substitution. A
`curl` word followed by a URL is one wherever it stands, behind `timeout N`,
`env`, `sudo`, `docker run IMAGE` or a wrapper function as much as at the
start of a command; a word holding `$` counts as a URL. One with no URL
counts only at the start of a command, where its URL can come from `--config`,
so the package name in `apt-get install -y curl git` is not one. The script of
`sh -c '...'` or
`bash -c "..."` is read as commands too. A curl whose output goes to /dev/null
(a readiness probe), or that carries neither a URL nor a kept output
(`curl --version`), is not one. Backslash continuations are joined first, so a
download split over several lines is read as one command.

Rules, one line per violation:

  BOUNDED    a download carries `--retry`, `--connect-timeout` and `--max-time`
             (or `-m`), none of them given a literal 0. `--retry` makes a
             transient failure, a timeout included, a retry; the two timeouts
             bound one attempt, so a connection that opens and then stops
             sending times out and is retried instead of holding the job until
             the job's own timeout.
  TO_A_FILE  a download writes its bytes with `-o`, `--output` or `-O`. curl
             discards a partial file it opened itself before a retry; bytes it
             already sent to stdout, into a pipe or a command substitution
             cannot be taken back.
  PARSE      a curl command whose words cannot be split (an unbalanced quote).
             It is refused rather than skipped: a check that cannot read a
             command reports nothing about it.
  FLOOR      every fetcher (`install_*.sh`, `build_*_container.sh`) holds at
             least one download, except the ones NO_DOWNLOAD names, which must
             hold none. A fetcher whose only download moves into a shape this
             reader does not know drops to zero and fails here, rather than
             leaving the other files to keep the run green. A workflow has no
             floor of its own.

Exit 0 every rule holds, 1 one line per violation on stdout, 2 usage, a file
it cannot read, or no download found in the files it read.

`--self-test` runs the rules over in-memory commands, one per rule and one per
shape that must NOT count as a download, and asserts the verdict of each.
"""

import os
import re
import shlex
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# curl options that take the next word as their value. Short ones are listed
# by letter; a value-taking letter ends a short cluster such as `-sSLo FILE`.
VALUE_LONG = {
    "--output", "--url", "--write-out", "--header", "--user-agent", "--user",
    "--data", "--data-binary", "--data-raw", "--request", "--max-time",
    "--connect-timeout", "--retry", "--retry-delay", "--retry-max-time",
    "--proto", "--proto-redir", "--referer", "--cacert", "--cert", "--config",
    "--upload-file", "--range", "--cookie", "--cookie-jar", "--limit-rate",
    "--proxy", "--speed-time", "--speed-limit", "--continue-at", "--form",
    "--resolve", "--connect-to", "--output-dir", "--interface", "--ciphers",
}
VALUE_SHORT = set("owHAudXmeKTrbcxyYCEFQtz")

# What may stand before `curl` for it to be the command word.
PREFIX_WORDS = {
    "if", "then", "do", "else", "elif", "while", "until", "exec", "command",
    "time", "sudo", "!", "run:", "-",
}
ASSIGNMENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=\S*$")
CURL_RE = re.compile(r"(?<![\w./-])curl(?=\s)")


def logical_lines(text):
    """Yield (first line number, text) with backslash continuations joined."""
    lines = text.split("\n")
    i = 0
    while i < len(lines):
        start = i
        cur = lines[i]
        while cur.rstrip().endswith("\\") and i + 1 < len(lines):
            i += 1
            cur = cur.rstrip()[:-1] + " " + lines[i].strip()
        i += 1
        yield start + 1, cur


def context_at(line, end):
    """What index `end` of `line` sits in: None, ("comment", start), or
    (quote character, index of the opening quote).

    A `$(` opens a command context again, inside double quotes too, so the
    curl in `sum="$(curl URL)"` is a command and not quoted text. An unquoted
    `#` at the start of a word opens a comment that runs to the end of the line.
    """
    stack = []
    i = 0
    while i < end:
        ch = line[i]
        top = stack[-1][0] if stack else None
        if top == "'":
            if ch == "'":
                stack.pop()
            i += 1
            continue
        if ch == "\\":
            i += 2
            continue
        if ch == "$" and line[i + 1:i + 2] == "(":
            stack.append(("(", i))
            i += 2
            continue
        if top == '"':
            if ch == '"':
                stack.pop()
        elif ch in "'\"":
            stack.append((ch, i))
        elif ch == "(":
            stack.append(("(", i))
        elif ch == ")" and top == "(":
            stack.pop()
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            return ("comment", i)
        i += 1
    if stack and stack[-1][0] in ("'", '"'):
        return stack[-1]
    return None


# A quoted string that is the script of a shell's `-c`: `sh -c '...'`,
# `bash -lc "..."`, `bash -euo pipefail -c '...'`.
SHELL_C_RE = re.compile(r"(?<![\w./-])(?:ba|da|k|z)?sh(?:\s+\S+)*?\s+-[A-Za-z]*c\s*$")
MAX_SHELL_C_DEPTH = 3


def quoted_body(line, start):
    """The text inside the quote that opens at `start`, unescaped."""
    quote = line[start]
    if quote == "'":
        stop = line.find("'", start + 1)
        return line[start + 1:] if stop < 0 else line[start + 1:stop]
    out = []
    i = start + 1
    while i < len(line):
        ch = line[i]
        if ch == "\\" and i + 1 < len(line) and line[i + 1] in '"\\$`':
            out.append(line[i + 1])
            i += 2
            continue
        if ch == '"':
            break
        out.append(ch)
        i += 1
    return "".join(out)


def is_command_position(line, index):
    before = line[:index].rstrip()
    if before == "":
        return True
    if before[-1] in "|&;(`{":
        return True
    word = before.split()[-1]
    return word in PREFIX_WORDS or bool(ASSIGNMENT_RE.match(word))


def segment_from(line, index):
    """The curl command starting at `index`, and how it ends.

    Ends at the first unquoted `|`, `||`, `&&`, `;`, `&`, backtick, a `)` that
    closes nothing opened inside the segment, or a ` #` comment. Returns the
    segment text and the terminator (`pipe` for a single `|`, else `other`).
    """
    quote = None
    depth = 0
    i = index
    while i < len(line):
        ch = line[i]
        if quote is not None:
            if ch == "\\" and quote == '"':
                i += 2
                continue
            if ch == quote:
                quote = None
            i += 1
            continue
        if ch == "\\":
            i += 2
            continue
        if ch in "'\"":
            quote = ch
        elif ch == "(":
            depth += 1
        elif ch == ")":
            if depth == 0:
                return line[index:i], "other"
            depth -= 1
        elif depth == 0 and ch == "|":
            if line[i + 1:i + 2] == "|":
                return line[index:i], "other"
            return line[index:i], "pipe"
        elif depth == 0 and ch in ";&`":
            return line[index:i], "other"
        elif depth == 0 and ch == "#" and i > 0 and line[i - 1] in " \t":
            return line[index:i], "other"
        i += 1
    return line[index:], "other"


def read_command(words):
    """(has_url, outputs, writes_remote_name, stdout_null, flags, values) for
    a curl; `values` maps each value-taking option to its last value."""
    outputs = []
    remote_name = False
    stdout_null = False
    flags = set()
    values = {}
    positional = []
    i = 1
    while i < len(words):
        word = words[i]
        # Redirections: `>/dev/null`, `1> x`, `2>/dev/null`, `&>/dev/null`.
        mo = re.match(r"^(\d*|&)(>>?)(.*)$", word)
        if mo:
            fd, target = mo.group(1), mo.group(3)
            if target == "" and i + 1 < len(words):
                i += 1
                target = words[i]
            if fd in ("", "1", "&") and target == "/dev/null":
                stdout_null = True
            i += 1
            continue
        if word.startswith("--"):
            name, eq, value = word.partition("=")
            flags.add(name)
            if name in VALUE_LONG:
                if not eq:
                    i += 1
                    value = words[i] if i < len(words) else ""
                values[name] = value
                if name == "--output":
                    outputs.append(value)
                elif name == "--url":
                    positional.append(value)
            elif name in ("--remote-name", "--remote-name-all"):
                remote_name = True
            i += 1
            continue
        if word.startswith("-") and len(word) > 1:
            j = 1
            while j < len(word):
                letter = word[j]
                flags.add("-" + letter)
                if letter == "O":
                    remote_name = True
                if letter in VALUE_SHORT:
                    value = word[j + 1:]
                    if value == "":
                        i += 1
                        value = words[i] if i < len(words) else ""
                    values["-" + letter] = value
                    if letter == "o":
                        outputs.append(value)
                    break
                j += 1
            i += 1
            continue
        positional.append(word)
        i += 1
    has_url = any(re.match(r"^[a-z]+://", w) or "$" in w for w in positional)
    return has_url, outputs, remote_name, stdout_null, flags, values


def is_zero(value):
    """A literal 0 (`0`, `00`, `0.0`): no retry, or no limit on the attempt."""
    return bool(re.match(r"^0+(?:\.0*)?$", value or ""))


def check_line(path, lineno, line, depth=0):
    problems = []
    downloads = 0
    read_quotes = set()
    for mo in CURL_RE.finditer(line):
        index = mo.start()
        where = context_at(line, index)
        if where is not None:
            kind, start = where
            # The script of `sh -c '...'` is read as its own command line,
            # once per quoted string however many curls it holds.
            if (kind in ("'", '"') and start not in read_quotes
                    and depth < MAX_SHELL_C_DEPTH and SHELL_C_RE.search(line[:start])):
                read_quotes.add(start)
                found, count = check_line(path, lineno, quoted_body(line, start), depth + 1)
                problems.extend(found)
                downloads += count
            continue
        at_command = is_command_position(line, index)
        before = line[:index].rstrip()
        in_substitution = before.endswith("$(") or before.endswith("`")
        segment, end = segment_from(line, index)
        try:
            words = shlex.split(segment, posix=True)
        except ValueError as err:
            problems.append("%s:%d: PARSE cannot split the curl command (%s): %s"
                            % (path, lineno, err, segment.strip()))
            continue
        has_url, outputs, remote_name, stdout_null, flags, values = read_command(words)
        kept = [o for o in outputs if o != "/dev/null"]
        if not has_url and not (at_command and (kept or remote_name)):
            continue
        if outputs and not kept and not remote_name:
            continue
        if not outputs and not remote_name and stdout_null and end != "pipe":
            continue
        downloads += 1
        missing = [f for f in ("--retry", "--connect-timeout", "--max-time")
                   if f not in flags and not (f == "--max-time" and "-m" in flags)]
        zero = [f for f in ("--retry", "--connect-timeout", "--max-time", "-m")
                if f in values and is_zero(values[f])]
        if missing:
            problems.append("%s:%d: BOUNDED a download without %s: %s"
                            % (path, lineno, ", ".join(missing), segment.strip()))
        if zero:
            problems.append("%s:%d: BOUNDED a download with %s given 0, which turns it off: %s"
                            % (path, lineno, ", ".join(zero), segment.strip()))
        if not kept and not remote_name:
            sink = ("into a pipe" if end == "pipe"
                    else "into a command substitution" if in_substitution
                    else "to stdout")
            problems.append("%s:%d: TO_A_FILE a download writes %s instead of a file "
                            "named with -o: %s" % (path, lineno, sink, segment.strip()))
    return problems, downloads


def check_text(path, text):
    problems = []
    downloads = 0
    for lineno, line in logical_lines(text):
        if line.lstrip().startswith("#"):
            continue
        found, count = check_line(path, lineno, line)
        problems.extend(found)
        downloads += count
    return problems, downloads


# Fetchers that hold no download. They are read like every other fetcher and
# must stay at zero.
NO_DOWNLOAD = {"install_hooks.sh"}


def is_fetcher(name):
    return (name.startswith("install_") and name.endswith(".sh")) or (
        name.startswith("build_") and name.endswith("_container.sh"))


def check_floor(path, count):
    name = os.path.basename(path)
    if not is_fetcher(name):
        return []
    if name in NO_DOWNLOAD:
        if count:
            return ["%s: FLOOR %s is listed in NO_DOWNLOAD but holds %d download(s); "
                    "remove it from the list" % (path, name, count)]
        return []
    if count == 0:
        return ["%s: FLOOR a fetcher holds no download this reader can see; a download "
                "in a shape it does not read is not checked, so teach the reader the shape "
                "or add the file to NO_DOWNLOAD" % path]
    return []


def default_paths(root):
    paths = []
    workflows = os.path.join(root, ".github", "workflows")
    for name in sorted(os.listdir(workflows)):
        if name.endswith((".yml", ".yaml")):
            paths.append(os.path.join(workflows, name))
    scripts = os.path.join(root, "tools", "scripts")
    for name in sorted(os.listdir(scripts)):
        if not name.endswith(".sh"):
            continue
        if is_fetcher(name):
            paths.append(os.path.join(scripts, name))
    return paths


SELF_TEST = [
    # (name, text, expected rule list)
    ("bounded download to a file",
     'curl -fsSL --retry 5 --connect-timeout 30 --max-time 180 -o x.tgz "$url"\n', []),
    ("-m counts as --max-time",
     "curl -fsSL --retry 5 --connect-timeout 30 -m 60 -o x https://example.com/x\n", []),
    ("--output= and --url forms",
     'curl --retry=3 --connect-timeout=9 --max-time=9 --output=x --url "$u"\n', []),
    ("a short cluster ending in o",
     "curl -fsSLo x --retry 3 --connect-timeout 9 --max-time 9 https://example.com/x\n", []),
    ("no --connect-timeout",
     'curl -fsSL --retry 5 --max-time 180 -o x.tgz "$url"\n', ["BOUNDED"]),
    ("no --max-time",
     'curl -fsSL --retry 5 --connect-timeout 30 -o x.tgz "$url"\n', ["BOUNDED"]),
    ("no --retry",
     'curl -fsSL --connect-timeout 30 --max-time 180 -o x.tgz "$url"\n', ["BOUNDED"]),
    ("a continuation is one command",
     'curl -fsSL --retry 5 \\\n    --connect-timeout 30 \\\n    -o x.tgz "$url"\n', ["BOUNDED"]),
    ("a workflow one-liner",
     "      - run: curl -fsSL -o x https://example.com/x\n", ["BOUNDED"]),
    ("an assignment before curl",
     "HOME=/tmp/h curl -fsSL -o x https://example.com/x\n", ["BOUNDED"]),
    ("a curl under if !",
     'if ! curl -fsSL -o x "$url"; then exit 1; fi\n', ["BOUNDED"]),
    ("into a pipe",
     "curl -sSf --retry 5 --connect-timeout 30 --max-time 60 https://sh.rustup.rs | sh -s -- -y\n",
     ["TO_A_FILE"]),
    ("into a command substitution",
     'sum="$(curl -fsSL --retry 5 --connect-timeout 30 --max-time 60 "$url.sha256")"\n',
     ["TO_A_FILE"]),
    ("to stdout",
     "curl -fsSL --retry 5 --connect-timeout 30 --max-time 60 https://example.com/x\n",
     ["TO_A_FILE"]),
    ("unbounded into a pipe breaks both rules",
     "curl -sSf https://sh.rustup.rs | sh\n", ["BOUNDED", "TO_A_FILE"]),
    ("an || handler is not a pipe",
     'curl -fsSL --retry 5 --connect-timeout 30 --max-time 60 -o x "$url" || die "no"\n', []),
    ("a probe to -o /dev/null",
     'if curl -fs "$mirror_url/SHA256SUMS" -o /dev/null; then exit 0; fi\n', []),
    ("a probe to >/dev/null",
     "curl -fsS http://127.0.0.1:8000/dists/stable/InRelease >/dev/null\n", []),
    ("curl --version",
     "v=\"$(curl --version 2>/dev/null | awk 'NR == 1 {print $2}')\"\n", []),
    ("a package name",
     "apt-get install -y build-essential curl ca-certificates git\n", []),
    ("command -v",
     "command -v curl >/dev/null 2>&1 || exit 1\n", []),
    ("a quoted mention",
     "printf 'fetch it with curl -o x https://example.com/x'\n", []),
    ("a comment",
     "# curl -fsSL https://example.com/x | sh\n", []),
    ("an unbalanced quote",
     'curl -fsSL -o x "$url\n', ["PARSE"]),
    # A curl with a URL counts wherever it stands.
    ("behind timeout N",
     "timeout 300 curl -fsSL -o x https://example.com/x\n", ["BOUNDED"]),
    ("behind env",
     "env HOME=/tmp curl -fsSL -o x https://example.com/x\n", ["BOUNDED"]),
    ("behind sudo -E",
     'sudo -E curl -fsSL -o x "$url"\n', ["BOUNDED"]),
    ("inside docker run IMAGE",
     'docker run --rm img:1 curl -fsSL -o /out/x "$url"\n', ["BOUNDED"]),
    ("behind a wrapper function",
     'run_foreground curl -fsSL -o x "$url"\n', ["BOUNDED"]),
    ("bounded behind timeout N",
     "timeout 300 curl -fsSL --retry 5 --connect-timeout 30 --max-time 180 -o x https://example.com/x\n",
     []),
    ("the script of bash -c",
     'bash -c "curl -fsSL -o x https://example.com/x"\n', ["BOUNDED"]),
    ("the script of sh -c, piped",
     "sh -c 'curl -sSf https://sh.rustup.rs | sh'\n", ["BOUNDED", "TO_A_FILE"]),
    ("the script of bash -euo pipefail -c",
     "bash -euo pipefail -c 'curl -fsSL --retry 5 --connect-timeout 30 --max-time 60 -o x \"$u\"'\n",
     []),
    ("an escaped quote inside bash -c",
     'bash -lc "curl -fsSL -o \\"$out\\" https://example.com/x"\n', ["BOUNDED"]),
    ("a command-position curl with no URL keeps its output",
     "curl --config urls.conf -o x\n", ["BOUNDED"]),
    ("an apt option after the package name",
     "apt-get install -y curl -o Dpkg::Options::=--force-confold\n", []),
    ("a trailing comment",
     "echo done # curl -fsSL https://example.com/x | sh\n", []),
    # Literal zero bounds turn the bound off.
    ("--retry 0",
     'curl -fsSL --retry 0 --connect-timeout 30 --max-time 180 -o x "$url"\n', ["BOUNDED"]),
    ("--max-time 0",
     'curl -fsSL --retry 5 --connect-timeout 30 --max-time 0 -o x "$url"\n', ["BOUNDED"]),
    ("-m 0",
     'curl -fsSL --retry 5 --connect-timeout 30 -m 0 -o x "$url"\n', ["BOUNDED"]),
    ("--connect-timeout=0",
     'curl -fsSL --retry 5 --connect-timeout=0 --max-time 9 -o x "$url"\n', ["BOUNDED"]),
    ("a variable bound",
     'curl -fsSL --retry "$n" --connect-timeout 30 --max-time 9 -o x "$url"\n', []),
]

GOOD_DOWNLOAD = 'curl -fsSL --retry 5 --connect-timeout 30 --max-time 180 -o x "$url"\n'

# (name, texts, expected exit, a substring of the FLOOR line or None)
FLOOR_TEST = [
    ("every fetcher holds a download",
     {"tools/scripts/install_a.sh": GOOD_DOWNLOAD, "tools/scripts/build_b_container.sh": GOOD_DOWNLOAD,
      "tools/scripts/install_hooks.sh": "echo no download\n"}, 0, None),
    ("a fetcher whose download the reader cannot see",
     {"tools/scripts/install_a.sh": GOOD_DOWNLOAD,
      "tools/scripts/install_b.sh": 'xargs -n 1 curl -fsSL -O < urls\n'}, 1, "install_b.sh: FLOOR"),
    ("a container fetcher with no download",
     {"tools/scripts/install_a.sh": GOOD_DOWNLOAD,
      "tools/scripts/build_b_container.sh": "echo none\n"}, 1, "build_b_container.sh: FLOOR"),
    ("a NO_DOWNLOAD file that gained a download",
     {"tools/scripts/install_a.sh": GOOD_DOWNLOAD, "tools/scripts/install_hooks.sh": GOOD_DOWNLOAD},
     1, "install_hooks.sh is listed in NO_DOWNLOAD"),
    ("a workflow with no download is not a fetcher",
     {"tools/scripts/install_a.sh": GOOD_DOWNLOAD, ".github/workflows/x.yml": "jobs: {}\n"}, 0, None),
]


def self_test():
    failures = 0
    for name, text, expected in SELF_TEST:
        problems, _ = check_text("<%s>" % name, text)
        rules = sorted(p.split(": ", 1)[1].split(" ", 1)[0] for p in problems)
        if rules != sorted(expected):
            failures += 1
            print("self-test FAIL %r: expected %s, got %s" % (name, sorted(expected), rules))
            for p in problems:
                print("  " + p)
    # A file that holds no download fails the run, so a reader that stopped
    # finding curl commands cannot report a pass.
    if run([], texts={"<empty>": "echo nothing here\n"}, say=lambda _line: None) != 2:
        failures += 1
        print("self-test FAIL: a run that found no download did not exit 2")
    for name, texts, want_exit, want_line in FLOOR_TEST:
        lines = []
        got = run([], texts=texts, say=lines.append)
        floor = [line for line in lines if ": FLOOR " in line]
        ok = got == want_exit and (
            (want_line is None and not floor) or
            (want_line is not None and len(floor) == 1 and want_line in floor[0]))
        if not ok:
            failures += 1
            print("self-test FAIL %r: expected exit %d and %s, got exit %d and %s"
                  % (name, want_exit, want_line or "no FLOOR line", got, floor))
    if failures:
        print("check_tool_downloads self-test: %d case(s) failed" % failures)
        return 1
    print("check_tool_downloads self-test: %d cases passed"
          % (len(SELF_TEST) + 1 + len(FLOOR_TEST)))
    return 0


def run(paths, texts=None, say=print):
    if texts is None:
        texts = {}
        for path in paths:
            try:
                with open(path, encoding="utf-8") as fh:
                    texts[os.path.relpath(path, ROOT)] = fh.read()
            except OSError as err:
                say("check_tool_downloads: cannot read %s: %s" % (path, err))
                return 2
    problems = []
    downloads = 0
    for path, text in texts.items():
        found, count = check_text(path, text)
        problems.extend(found)
        problems.extend(check_floor(path, count))
        downloads += count
    for p in problems:
        say(p)
    if downloads == 0:
        say("check_tool_downloads: no download found in %d file(s); the reader found nothing to check"
              % len(texts))
        return 2
    if problems:
        say("check_tool_downloads: %d violation(s) in %d download(s) across %d file(s)"
              % (len(problems), downloads, len(texts)))
        return 1
    say("check_tool_downloads: %d download(s) across %d file(s) are retried, bounded and written to a file"
          % (downloads, len(texts)))
    return 0


def main(argv):
    if argv == ["--self-test"]:
        return self_test()
    if any(a.startswith("-") for a in argv):
        print(__doc__.strip().split("\n\n")[1])
        return 2
    return run(argv or default_paths(ROOT))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
