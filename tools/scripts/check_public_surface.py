#!/usr/bin/env python3
"""The public-surface gate: the launch-week scrub classes, made unrepeatable.

Driven by `tools/scripts/check_public_surface.sh` (which documents the classes,
the exit codes and the data files); this file is the implementation and the
self-test. Standard library only, python3.8+.

Output contract, shared with the wrapper:
  every finding      `<file>:<line>: <class>: <message>`
  every note         `note: <class>: <message>`   (never a failure)
  per failing class  `remedy: <class>: <one line naming the rule it enforces>`
  the LAST line      `check_public_surface: OK (...)` or `check_public_surface: FAIL (...)`
Exit 0 = clean, 1 = findings, 2 = usage, 3 = could not run (a gate must not
fail open, so an unreadable input is a 3, never a silent pass).
"""

import os
import re
import subprocess
import unicodedata
import sys
import tempfile

DASHES = "\u2013\u2014"  # en dash, em dash
DASH_RE = re.compile("[" + DASHES + "]")

# The tracker-id shape is spelled as a bracket class so this file never matches
# its own scan (the same trick the sibling gates use for their banned tokens).
TRACKER_RE = re.compile(r"\b[C]ER-[0-9]+\b")
TODO_OWNER_RE = re.compile(r"\b(TODO|FIXME)\(([A-Za-z][A-Za-z0-9_.]*)\)")
# A pictograph on a shipped page. "Emoji" is not one range, so this is three
# shapes: the astral emoji planes; a curated BMP set of characters that exist
# only as a sign or as the emoji twin of a mark (the warning sign, and the heavy
# and emoji forms of the check mark and the cross); and U+FE0F, the variation
# selector that forces emoji presentation onto any base character, which is how
# a plain check mark becomes one.
# The BARE check mark and ballot X (U+2713, U+2717) STAY: a support table draws
# its yes and no with them and 28 such cells ship, so refusing them would be a
# rule against a table rather than against decoration.
PICTOGRAPH_RE = re.compile(
    "["
    "\U0001F000-\U0001FAFF"                 # the emoji planes
    "\u26a0"                                 # warning sign
    "\u2705\u2714\u274c\u2716\u2718"        # emoji and heavy check, cross, ballot X
    "\u2b50\u2757\u2764\u2728"              # star, exclamation, heart, sparkles
    "\ufe0f"                                 # the emoji variation selector
    "]")
# The project's legal name is DECLARED in a data file, never parsed out of
# prose. A notice is a list of OTHER people's copyright lines, so a rule that
# reads one to find its own input takes whichever line comes first: a rename, a
# reordering, a translated line or a deleted line hands it a vendor's name and
# it goes on policing that, green. Declared inputs, parsed outputs.
ENTITY_FILE = "tools/scripts/public_surface_entity.txt"
# The suffix family. `Co.` carries its period, because a bare `Co` matches the
# first half of a hyphenated word. The trailing guard is what keeps `Inc` out of
# `Incidentally`; the order of the alternatives is cosmetic, since the guard
# forces the backtrack either way.
# THE ONE LIST. The family a mention may end in, the family a refusal names, and
# the characters a declared name may hold all come from here, so the three can
# never disagree again. `Co.` is spelled only with its period, because a bare
# `Co` matches the first half of a hyphenated word.
LEGAL_SUFFIX_WORDS = (
    "Inc", "Inc.", "Incorporated", "Corp", "Corp.", "Corporation", "Co.",
    "LLC", "LLP", "Ltd", "Ltd.", "Limited", "PLC", "GmbH", "AG",
    "S.A.", "B.V.", "AB", "Oy", "A/S")
# LONGEST FIRST, so `Inc.` wins over `Inc` and the match carries the period a
# page actually wrote. Without that order the pattern would read a correct
# `<name> Inc.` as `<name> Inc` and report the right spelling as wrong.
LEGAL_SUFFIX = (r"(?:" + "|".join(re.escape(w) for w in sorted(LEGAL_SUFFIX_WORDS, key=len, reverse=True))
                + r")(?![A-Za-z])")

# Widening this set by adding CHARACTERS is safe. Widening it by admitting a
# unicode CATEGORY is not: that is how the invisible characters got in, and a
# category always holds more than the letters it was reached for.
# Every character the suffix family needs is folded in from the list above, so a
# suffix the rule accepts can never be a suffix a declaration may not spell.
LEGAL_NAME_CHARS = frozenset(
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 &.,-'/()"
) | frozenset("".join(LEGAL_SUFFIX_WORDS))
# What may stand between the words of a mention: spaces (a no-break space among
# them, which a typesetter inserts and a reader cannot see), a comma, or ONE
# line break with the prefix a wrapped line carries. The prefix set is every
# line-lead marker this tree writes: a doc comment `///` or `//!`, a block
# comment `*`, a shell or heading `#`, a quote `>`, a table cell `|`, a list
# `-`, and `;` `%` `!` from the other comment syntaxes. One break and no blank
# line, because a blank line ends the sentence and the two sides are then not
# one mention.
LEGAL_SPACE = r"[ \t\u00a0\u202f]"
TRAILING_SUFFIX_RE = None       # built below, once LEGAL_GAP and LEGAL_SUFFIX exist
LEGAL_GAP = (r"(?:" + LEGAL_SPACE + r"+|" + LEGAL_SPACE + r"*," + LEGAL_SPACE + r"*|"
             + LEGAL_SPACE + r"*,?" + LEGAL_SPACE + r"*\r?\n" + LEGAL_SPACE + r"*(?:[>#*/!;%]+" + LEGAL_SPACE + r"*)?)")

ALLOW_FILE = "tools/scripts/public_surface_allow.txt"
PHRASES_FILE = "tools/scripts/public_surface_phrases.txt"
LEDGER_FILE = "tools/scripts/public_surface_dash_ledger.txt"
WORKSTATE_FILE = "tools/scripts/public_surface_workstate.txt"
WORKSTATE_LEDGER_FILE = "tools/scripts/public_surface_workstate_ledger.txt"
CLI_FILE = "crates/cerulion_cli/src/cli.rs"
DATA_FILES = (ALLOW_FILE, PHRASES_FILE, LEDGER_FILE, WORKSTATE_FILE, WORKSTATE_LEDGER_FILE, ENTITY_FILE)

CLASSES = (
    "examples-shape",
    "docs-refs",
    "unreferenced-media",
    "bench-citation",
    "shipped-text",
    "work-state",
    "numbers-without-data",
    "string-literal-rewrite",
)

REMEDIES = {
    "examples-shape": "an example is a workspace (Cargo.toml, graphs/*.yaml, one #[cerulion_node] "
    "type per nodes/<type>/ crate) run through the cerulion verbs; several node types in one "
    "file, a graph built in code and runtime-API construction belong under tests/ only "
    "(AGENTS.md: 'Shipped surface' rules)",
    "docs-refs": "docs name only verbs, flags, paths and files that exist: fix the link or "
    "path, spell the verb as the CLI defines it, and write post-move paths (crates/..., "
    "tools/scripts/..., docs/user-api.md) (AGENTS.md: 'Shipped surface' rules)",
    "unreferenced-media": "every file under docs/media/ is used by README.md or a docs page; "
    "delete an unused asset (a removal is a maintainer ruling) or reference it "
    "(AGENTS.md: 'Shipped surface' rules)",
    "bench-citation": "every results package and every bench tree is named by the pages that "
    "cite benchmarks (README.md, docs/PERFORMANCE.md, docs/benchmarks/README.md, "
    "benches/README.md); cite it or remove it by ruling (AGENTS.md: 'Shipped surface' rules)",
    "shipped-text": "shipped text carries no typographic dash, no tracker id outside "
    "CHANGELOG.md, no TODO(<owner>) and no claim the README contradicts; fix each site by hand "
    "and, for a dash you removed, lower that file's line in " + LEDGER_FILE + " "
    "(AGENTS.md: 'Shipped surface' rules)",
    "work-state": "shipped text describes the product to its user: what works, what is "
    "experimental, what is not supported, what to do. It never reports how the project was "
    "built: who decided, which plan step or review round a line came from, which machine a "
    "test ran on, or what is still on a to-do list. Rewrite each line by hand (the note per "
    "key says what to write instead; the patterns are " + WORKSTATE_FILE + "), then lower the "
    "file's line in " + WORKSTATE_LEDGER_FILE + "; a user-facing page reads zero "
    "(AGENTS.md: 'Shipped surface' rules)",
    "numbers-without-data": "a figure on a user-facing page must appear, spelled the same, in "
    "a shipped package under docs/benchmarks/results/ (a CSV, HEADLINE.md or the package "
    "README.md); print the package's spelling or ship the package "
    "(AGENTS.md: 'Shipped surface' rules)",
    "string-literal-rewrite": "a bulk edit never rewrites the inside of a string literal; fix "
    "each literal by hand and run the tests that read it (AGENTS.md: 'Shipped surface' rules)",
}


class CannotRun(Exception):
    """An input the gate needs is missing or unreadable: exit 3, never a silent pass."""


class Finding:
    __slots__ = ("cls", "path", "line", "message")

    def __init__(self, cls, path, line, message):
        self.cls = cls
        self.path = path
        self.line = line
        self.message = message

    def render(self):
        return "%s:%d: %s: %s" % (self.path, self.line, self.cls, self.message)


# ---------------------------------------------------------------------------
# Tree access
# ---------------------------------------------------------------------------


def tracked_files(root):
    """`git ls-files` at `root`: the set of files that ship."""
    try:
        out = subprocess.run(
            ["git", "-C", root, "ls-files", "-z", "--cached"],
            capture_output=True,
            check=False,
        )
    except OSError as exc:
        raise CannotRun("git is not runnable: %s" % exc)
    if out.returncode != 0:
        raise CannotRun("git ls-files failed at %s: %s" % (root, out.stderr.decode("utf-8", "replace").strip()))
    files = [p.decode("utf-8", "surrogateescape") for p in out.stdout.split(b"\0") if p]
    # A path listed by git but absent on disk (deleted, not yet staged) is not
    # shipped text; skip it rather than fail on it.
    return sorted(f for f in files if os.path.isfile(os.path.join(root, f)))


_text_cache = {}


def read_text(root, rel):
    """The file's text, or None when it is binary. Unreadable => CannotRun."""
    key = (root, rel)
    if key in _text_cache:
        return _text_cache[key]
    full = os.path.join(root, rel)
    try:
        with open(full, "rb") as fh:
            raw = fh.read()
    except OSError as exc:
        raise CannotRun("cannot read %s: %s" % (rel, exc))
    if b"\0" in raw[:8192]:
        _text_cache[key] = None
        return None
    txt = raw.decode("utf-8", "replace")
    _text_cache[key] = txt
    return txt


def is_test_path(rel):
    return bool(re.search(r"(^|/)tests/|^crates/test_fixtures/|(^|/)tests\.rs$", rel))


def in_docs_set(rel):
    """README.md, docs/**/*.md, examples/**/*.md, every AGENTS.md, every README.md."""
    if not rel.endswith(".md"):
        return False
    base = os.path.basename(rel)
    return base in ("README.md", "AGENTS.md") or rel.startswith("docs/") or rel.startswith("examples/")


# ---------------------------------------------------------------------------
# Rust lexing (comments out, strings located)
# ---------------------------------------------------------------------------


def lex_rust(src):
    """Return (code_view, literals). `code_view` is `src` with every comment and
    every string body blanked to spaces (same length, newlines kept) so brace
    counting, pattern search and line numbers stay exact; `literals` is a list
    of (line, body) for every string literal outside comments."""
    n = len(src)
    out = list(src)
    literals = []
    i = 0
    line = 1

    def blank(a, b):
        for k in range(a, b):
            if out[k] != "\n":
                out[k] = " "

    while i < n:
        c = src[i]
        if c == "\n":
            line += 1
            i += 1
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            blank(i, j)
            i = j
            continue
        if src.startswith("/*", i):
            depth = 1
            j = i + 2
            while j < n and depth:
                if src.startswith("/*", j):
                    depth += 1
                    j += 2
                elif src.startswith("*/", j):
                    depth -= 1
                    j += 2
                else:
                    j += 1
            line += src.count("\n", i, j)
            blank(i, j)
            i = j
            continue
        m = re.match(r'b?r(#*)"', src[i:i + 8])
        if m:
            hashes = m.group(1)
            start = i + len(m.group(0))
            end = src.find('"' + hashes, start)
            end = n if end < 0 else end
            body = src[start:end]
            literals.append((line, body))
            line += body.count("\n")
            blank(start, end)
            i = end + 1 + len(hashes)
            continue
        if c == '"' or (c == "b" and i + 1 < n and src[i + 1] == '"'):
            i += 2 if c == "b" else 1
            start = i
            while i < n and src[i] != '"':
                if src[i] == "\\":
                    i += 1
                if i < n and src[i] == "\n":
                    line += 1
                i += 1
            literals.append((line - src.count("\n", start, i), src[start:i]))
            blank(start, i)
            i += 1
            continue
        if c == "'":
            # A char literal ('x', '\n', '\u{..}') versus a lifetime ('a).
            if i + 2 < n and (src[i + 2] == "'" or src[i + 1] == "\\"):
                j = src.find("'", i + 2 if src[i + 1] != "\\" else i + 3)
                if j > 0:
                    blank(i + 1, j)
                    i = j + 1
                    continue
            i += 1
            continue
        i += 1
    return "".join(out), literals


def cfg_test_regions(code_view):
    """Line ranges (1-based, inclusive) of `#[cfg(test)] mod ... { ... }` blocks."""
    lines = code_view.split("\n")
    regions = []
    i = 0
    while i < len(lines):
        if re.match(r"\s*#\[cfg\(test\)\]", lines[i]):
            j = i + 1
            while j < len(lines) and re.match(r"\s*#\[", lines[j]):
                j += 1
            if j < len(lines) and re.match(r"\s*(pub(\([^)]*\))?\s+)?mod\s+\w+", lines[j]):
                depth = 0
                opened = False
                k = j
                while k < len(lines):
                    depth += lines[k].count("{") - lines[k].count("}")
                    if "{" in lines[k]:
                        opened = True
                    if opened and depth <= 0:
                        break
                    k += 1
                regions.append((i + 1, k + 1))
                i = k + 1
                continue
        i += 1
    return regions


def in_regions(line, regions):
    return any(a <= line <= b for a, b in regions)


# ---------------------------------------------------------------------------
# Markdown helpers
# ---------------------------------------------------------------------------

FENCE_RE = re.compile(r"^(```|~~~).*?^\1[ \t]*$", re.M | re.S)
SPAN_RE = re.compile(r"`[^`\n]+`")


def md_code_regions(txt):
    """Every fenced block and inline span as (start, end, text) spans."""
    out = []
    for m in FENCE_RE.finditer(txt):
        out.append((m.start(), m.end(), m.group(0)))
    covered = [(a, b) for a, b, _ in out]
    for m in SPAN_RE.finditer(txt):
        if not any(a <= m.start() < b for a, b in covered):
            out.append((m.start(), m.end(), m.group(0)))
    return out


def md_prose(txt):
    """`txt` with every code region blanked (length preserved)."""
    chars = list(txt)
    for a, b, _ in md_code_regions(txt):
        for k in range(a, b):
            if chars[k] != "\n":
                chars[k] = " "
    return "".join(chars)


def line_of(txt, offset):
    return txt.count("\n", 0, offset) + 1


# ---------------------------------------------------------------------------
# Data files: allow list, phrase list, dash ledger
# ---------------------------------------------------------------------------


def load_allow(root):
    """`<path> | <class> | <match> | <reason>` per line; `*` matches any message."""
    rel = ALLOW_FILE
    full = os.path.join(root, rel)
    if not os.path.isfile(full):
        return []
    entries = []
    with open(full, encoding="utf-8") as fh:
        for no, raw in enumerate(fh, 1):
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            parts = [p.strip() for p in s.split("|")]
            if len(parts) != 4 or not all(parts):
                raise CannotRun("%s:%d: an allow entry is `<path> | <class> | <match> | <reason>`" % (rel, no))
            path, cls, match, reason = parts
            if cls not in CLASSES:
                raise CannotRun("%s:%d: unknown class %r" % (rel, no, cls))
            if len(reason) < 12:
                raise CannotRun("%s:%d: the reason must say why (at least 12 characters)" % (rel, no))
            if cls == "work-state" and match == "*":
                raise CannotRun("%s:%d: a work-state entry names the one finding it excuses; `*` would waive the whole file" % (rel, no))
            entries.append({"path": path, "cls": cls, "match": match, "reason": reason, "line": no, "used": 0})
    return entries


def load_phrases(root):
    """One contradicting phrase per line, matched case-insensitively as a substring."""
    candidates = [os.path.join(root, PHRASES_FILE), os.path.join(os.path.dirname(os.path.abspath(__file__)), "public_surface_phrases.txt")]
    for full in candidates:
        if os.path.isfile(full):
            phrases = []
            with open(full, encoding="utf-8") as fh:
                for raw in fh:
                    s = raw.strip()
                    if s and not s.startswith("#"):
                        phrases.append(s)
            if not phrases:
                raise CannotRun("%s holds no phrase: an empty list would pass every claim" % full)
            return phrases
    raise CannotRun("no phrase list found (%s)" % PHRASES_FILE)


def load_ledger(root):
    """`<count> <path>` per line: the dashes each shipped file is known to carry."""
    full = os.path.join(root, LEDGER_FILE)
    if not os.path.isfile(full):
        return {}
    ledger = {}
    with open(full, encoding="utf-8") as fh:
        for no, raw in enumerate(fh, 1):
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            m = re.match(r"(\d+)\s+(\S.*)$", s)
            if not m:
                raise CannotRun("%s:%d: a ledger line is `<count> <path>`" % (LEDGER_FILE, no))
            ledger[m.group(2)] = int(m.group(1))
    return ledger


LEDGER_HEADER = """# Dash ledger for tools/scripts/check_public_surface.sh (class shipped-text).
# `<count> <path>`: the typographic dashes (U+2013, U+2014) each shipped file
# carries in its scanned surface today. The gate fails a file whose count is
# ABOVE its line (a new dash) or BELOW it (a stale line pre-authorises the next
# dash): after removing dashes from a file, lower its line or delete it. Lines
# only ever go down. `check_public_surface.sh --regenerate-dash-ledger`
# rewrites this file from the tree, for the maintainer-ruled scrub of a crate.
"""


# ---------------------------------------------------------------------------
# Class 1: examples-shape
# ---------------------------------------------------------------------------

GRAPH_IN_CODE_RE = re.compile(
    r"GraphRuntime::build|TransportManager::(?:init|get_or_init)|Box<dyn NodeEntry>|\bparse_graph\("
)
CARGO_EXAMPLE_RE = re.compile(r"cargo\s+(?:run|test)\b[^\n`]*--example\b")
NODE_DECL_RE = re.compile(r"^\s*#\[cerulion_node(?:\(|\])", re.M)


def check_examples_shape(root, files):
    findings = []
    ex_dirs = sorted({f.split("/")[1] for f in files if f.startswith("examples/") and f.count("/") >= 2})
    for name in ex_dirs:
        base = "examples/%s/" % name
        members = [f for f in files if f.startswith(base)]
        anchor = base + "Cargo.toml"
        if anchor not in members:
            findings.append(Finding("examples-shape", base.rstrip("/"), 1, "no Cargo.toml: an example is a workspace"))
        if not any(re.match(re.escape(base) + r"graphs/[^/]+\.yaml$", f) for f in members):
            findings.append(Finding("examples-shape", base.rstrip("/"), 1, "no graphs/*.yaml: wiring lives in graph YAML"))
        libs = [f for f in members if re.match(re.escape(base) + r"nodes/[^/]+/src/lib\.rs$", f)]
        if not libs:
            findings.append(Finding("examples-shape", base.rstrip("/"), 1, "no nodes/<type>/src/lib.rs: one node type per crate"))
        for lib in libs:
            txt = read_text(root, lib)
            if txt is None:
                continue
            code, _ = lex_rust(txt)
            count = len(NODE_DECL_RE.findall(code))
            if count != 1:
                findings.append(Finding("examples-shape", lib, 1, "declares %d #[cerulion_node] types; exactly one per nodes/<type>/ crate" % count))
    # A graph built in code, outside tests, anywhere an example lives.
    for f in files:
        if not f.endswith(".rs") or is_test_path(f):
            continue
        if not (f.startswith("examples/") or re.match(r"crates/[^/]+/examples/", f)):
            continue
        txt = read_text(root, f)
        if txt is None:
            continue
        code, _ = lex_rust(txt)
        regions = cfg_test_regions(code)
        for m in GRAPH_IN_CODE_RE.finditer(code):
            ln = line_of(code, m.start())
            if in_regions(ln, regions):
                continue
            findings.append(Finding("examples-shape", f, ln, "graph built in code (%s); a user never drives the runtime, only tests do" % m.group(0)))
    # `cargo run --example` in any README, docs page or AGENTS.md.
    for f in files:
        if not in_docs_set(f):
            continue
        txt = read_text(root, f)
        if txt is None:
            continue
        for m in CARGO_EXAMPLE_RE.finditer(txt):
            findings.append(Finding("examples-shape", f, line_of(txt, m.start()), "`cargo ... --example` is not how a user runs an example; examples run through the cerulion verbs"))
    return findings


# ---------------------------------------------------------------------------
# Class 2: docs-refs
# ---------------------------------------------------------------------------

LINK_RE = re.compile(r"!?\[[^\]]*\]\(\s*(<[^>]*>|[^)\s]+)(?:\s+\"[^\"]*\")?\s*\)|^[ \t]*\[[^\]]+\]:[ \t]+(\S+)", re.M)
# Raw HTML in a page (a <picture> block, an <img>, an <a>) names files too: `src`, `srcset` and `href`.
# A `srcset` holds comma-separated candidates, each a path with an optional width or density descriptor.
HTML_ATTR_RE = re.compile(r"\b(src|srcset|href)\s*=\s*\"([^\"]*)\"", re.I)
PLACEHOLDER_RE = re.compile(r"^[A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+$")  # STUDIO_MACOS_DOWNLOAD_URL, never LICENSE


def kebab(name):
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "-", name).lower()


def parse_cli_tree(src):
    """The clap verb tree from `#[derive(Subcommand)] pub enum ...` definitions."""
    enums = {}
    for m in re.finditer(r"#\[derive\(Subcommand\)\]\s*\n(?:[^\n]*\n)*?pub enum (\w+) \{\n(.*?)\n\}\n", src, re.S):
        name, body = m.group(1), m.group(2)
        variants = {}
        pending_name = None
        pending_hidden = False
        depth = 0
        cur = None
        prev = ""
        for line in body.split("\n"):
            s = line.strip()
            if depth == 0:
                mc = re.match(r"#\[command\((.*)\)\]", s)
                if mc:
                    attrs = mc.group(1)
                    mn = re.search(r"\bname\s*=\s*\"([^\"]+)\"", attrs)
                    if mn:
                        pending_name = mn.group(1)
                    if re.search(r"\bhide\b", attrs):
                        pending_hidden = True
                mv = re.match(r"([A-Z][A-Za-z0-9]*)\s*(\{|,|\(|$)", s)
                if mv and not s.startswith("#"):
                    cur = pending_name or kebab(mv.group(1))
                    variants[cur] = {"hidden": pending_hidden, "sub": None}
                    pending_name = None
                    pending_hidden = False
            if cur and depth >= 1:
                ms = re.match(r"\w+\s*:\s*(\w+),", s)
                if ms and "subcommand" in prev:
                    variants[cur]["sub"] = ms.group(1)
            prev = s
            depth += s.count("{") - s.count("}")
        enums[name] = variants
    if "Commands" not in enums:
        raise CannotRun("%s: no `#[derive(Subcommand)] pub enum Commands` found; the verb parser no longer understands the CLI" % CLI_FILE)

    def build(enum):
        node = {}
        for verb, d in enums[enum].items():
            node[verb] = build(d["sub"]) if d["sub"] and d["sub"] in enums else {}
        return node

    return build("Commands")


VERB_RE = re.compile(r"(?<![\w/.:$-])cerulion((?:[ \t]+[a-z][a-z0-9-]*(?=[\s`|)\]]|$))+)")


def verb_findings(path, txt, tree):
    out = []
    for a, _b, region in md_code_regions(txt):
        for m in VERB_RE.finditer(region):
            tokens = m.group(1).split()
            node = tree
            walked = ["cerulion"]
            for tok in tokens:
                if tok == "help":
                    break
                if tok not in node:
                    out.append(Finding("docs-refs", path, line_of(txt, a + m.start()), "`%s %s` is not a cerulion verb (the CLI defines: %s)" % (" ".join(walked), tok, ", ".join(sorted(node)) or "no subcommand here")))
                    break
                walked.append(tok)
                node = node[tok]
                if not node:
                    break
    return out


def check_docs_refs(root, files):
    findings, notes = [], []
    cli_src = read_text(root, CLI_FILE) if CLI_FILE in files else None
    if cli_src is None:
        raise CannotRun("%s is not in the tree: the verb check cannot run" % CLI_FILE)
    tree = parse_cli_tree(cli_src)
    crate_names = sorted({f.split("/")[1] for f in files if f.startswith("crates/") and f.count("/") >= 2})
    stale_names = crate_names + ["test_fixtures", "scripts"]
    stale_re = re.compile(r"(?<![\w/.-])(" + "|".join(re.escape(n) for n in stale_names) + r")/(?=[\w.])")
    user_api_re = re.compile(r"\bUSER_API\.md\b")
    for f in files:
        if not in_docs_set(f):
            continue
        txt = read_text(root, f)
        if txt is None:
            continue
        prose = md_prose(txt)
        targets = [((m.group(1) or m.group(2) or "").strip("<>"), m.start()) for m in LINK_RE.finditer(prose)]
        for m in HTML_ATTR_RE.finditer(prose):
            values = [c.strip().split()[0] for c in m.group(2).split(",") if c.strip()] if m.group(1).lower() == "srcset" else [m.group(2).strip()]
            targets.extend((v, m.start()) for v in values)
        for target, at in targets:
            if not target or re.match(r"^(#|https?:|mailto:|tel:|data:|//)", target):
                continue
            target = target.split("#", 1)[0].split("?", 1)[0]
            if not target:
                continue
            if PLACEHOLDER_RE.match(target):
                notes.append("docs-refs: %s:%d: link target %s is a placeholder token, replaced at the cut (cut.sh refuses a leftover)" % (f, line_of(prose, at), target))
                continue
            resolved = os.path.normpath(os.path.join(os.path.dirname(f), target)) if not target.startswith("/") else target.lstrip("/")
            if not os.path.exists(os.path.join(root, resolved)):
                findings.append(Finding("docs-refs", f, line_of(prose, at), "link target %s resolves to %s, which does not exist" % (target, resolved)))
        findings.extend(verb_findings(f, txt, tree))
        for m in stale_re.finditer(txt):
            findings.append(Finding("docs-refs", f, line_of(txt, m.start()), "pre-move path spelling `%s/`; the tree keeps crates under crates/ and scripts under tools/scripts/" % m.group(1)))
        for m in user_api_re.finditer(txt):
            findings.append(Finding("docs-refs", f, line_of(txt, m.start()), "`USER_API.md` no longer exists; the user API reference is docs/user-api.md"))
    return findings, notes, tree


# ---------------------------------------------------------------------------
# Class 3: unreferenced-media
# ---------------------------------------------------------------------------


def check_unreferenced_media(root, files):
    findings = []
    media = [f for f in files if f.startswith("docs/media/") and f != "docs/media/README.md" and not f.startswith("docs/media/charts/")]
    if not media:
        return findings
    referrers = [f for f in files if f == "README.md" or (f.startswith("docs/") and f.endswith(".md") and f != "docs/media/README.md")]
    corpus = "\n".join(t for t in (read_text(root, f) for f in referrers) if t)
    for m in media:
        if os.path.basename(m) not in corpus:
            findings.append(Finding("unreferenced-media", m, 1, "not referenced by README.md or any docs page (the media index does not count as a use)"))
    return findings


# ---------------------------------------------------------------------------
# Class 4: bench-citation
# ---------------------------------------------------------------------------


def check_bench_citation(root, files):
    findings = []
    pkgs = sorted({f.split("/")[3] for f in files if f.startswith("docs/benchmarks/results/") and f.count("/") >= 4})
    citers = "\n".join(t for t in (read_text(root, f) for f in ("README.md", "docs/PERFORMANCE.md", "docs/benchmarks/README.md") if f in files) if t)
    for p in pkgs:
        if p not in citers:
            findings.append(Finding("bench-citation", "docs/benchmarks/results/" + p, 1, "results package not linked from README.md, docs/PERFORMANCE.md or docs/benchmarks/README.md"))
    benches = sorted({f.split("/")[1] for f in files if f.startswith("benches/") and f.count("/") >= 2})
    named = "\n".join(t for t in (read_text(root, f) for f in ("README.md", "benches/README.md") if f in files) if t)
    for b in benches:
        if b not in named:
            findings.append(Finding("bench-citation", "benches/" + b, 1, "bench tree not named in README.md or benches/README.md"))
    return findings


# ---------------------------------------------------------------------------
# Class 5: shipped-text
# ---------------------------------------------------------------------------


def snippet(text):
    return re.sub(r"\s+", " ", text.strip())[:60]


def dash_surface(rel, txt):
    """(dash count, [(line, snippet)]) over the file's scanned surface."""
    hits = []
    if rel.endswith(".rs"):
        _code, literals = lex_rust(txt)
        for ln, body in literals:
            for _ in DASH_RE.finditer(body):
                hits.append((ln, snippet(body)))
    elif rel.endswith(".md"):
        for no, line in enumerate(txt.split("\n"), 1):
            for _ in DASH_RE.finditer(line):
                hits.append((no, snippet(line)))
    elif os.path.basename(rel) == "Cargo.toml":
        for no, line in enumerate(txt.split("\n"), 1):
            if re.match(r"\s*description\s*=", line):
                for _ in DASH_RE.finditer(line):
                    hits.append((no, snippet(line)))
    elif rel.startswith(".github/workflows/") and rel.endswith((".yml", ".yaml")):
        for no, line in enumerate(txt.split("\n"), 1):
            if re.match(r"\s*(?:-\s+)?name:", line):
                for _ in DASH_RE.finditer(line):
                    hits.append((no, snippet(line)))
    return len(hits), hits


def shipped_files(files):
    return [f for f in files if not is_test_path(f) and f not in DATA_FILES]


def compute_dash_counts(root, files):
    counts = {}
    for f in shipped_files(files):
        txt = read_text(root, f)
        if txt is None:
            continue
        n, _ = dash_surface(f, txt)
        if n:
            counts[f] = n
    return counts


TRAILING_SUFFIX_RE = re.compile(LEGAL_GAP + LEGAL_SUFFIX)


def legal_name(root):
    """The project's legal name, as DECLARED in ENTITY_FILE. Every defect of
    that file is exit 3 rather than a guess: the class compares every mention in
    the tree against this one string, so a name nobody chose makes every later
    comparison meaningless, and a name carrying whitespace can never be matched
    byte for byte by anything."""
    full = os.path.join(root, ENTITY_FILE)
    if not os.path.isfile(full):
        raise CannotRun("%s is missing: it declares the legal name every mention is compared against" % ENTITY_FILE)
    try:
        with open(full, encoding="utf-8-sig") as fh:
            raw = fh.read().split("\n")
    except (OSError, UnicodeDecodeError) as exc:
        raise CannotRun("cannot read %s: %s" % (ENTITY_FILE, exc))
    lines = [l for l in raw if l.strip() and not l.lstrip().startswith("#")]
    if not lines:
        raise CannotRun("%s declares no name: one line holding the legal name, and comments" % ENTITY_FILE)
    if len(lines) > 1:
        raise CannotRun("%s declares %d names: the legal name is ONE line, so a second is a name nobody chose between" % (ENTITY_FILE, len(lines)))
    if lines[0] != lines[0].strip():
        raise CannotRun("%s declares a name with leading or trailing whitespace: the comparison is byte for byte, so the spelling must be exact" % ENTITY_FILE)
    # WHAT A NAME MAY HOLD, listed positively. A character a reader cannot see
    # is the one defect the matchability check below cannot catch: the
    # declaration matches ITSELF, and every mention written the visible way then
    # fails to match, so the class reads green over an unchecked tree. Listing
    # the invisible characters to refuse is unbounded (a combining mark, a
    # variation selector, a filler letter and a homoglyph are all invisible or
    # indistinguishable and live in four different categories), so this lists
    # what a name MAY hold instead. A name outside it is refused loudly, and a
    # project whose name needs more can widen this line deliberately.
    for c in lines[0]:
        if c not in LEGAL_NAME_CHARS:
            raise CannotRun("%s declares a name holding %s, which is outside the characters a declared name may "
                            "hold (letters, digits, space, and %s): a character a reader cannot see, or cannot "
                            "tell from another, would leave every mention unmatched and the class green over an "
                            "unchecked tree"
                            % (ENTITY_FILE, "U+%04X" % ord(c),
                               " ".join(sorted(x for x in LEGAL_NAME_CHARS if not x.isalnum() and x != " "))))
    return lines[0]


def legal_name_mentions(canonical):
    """The regex that finds every MENTION of the declared name, however it is
    spelled. It keys on the FIRST word of the name, and the words between that
    and the suffix may only be the OTHER words of the declared name, in any
    number up to how many there are. That is what lets a mention which drops a
    name word still be judged (a rename leaves those behind) while `Cerulion
    works with Foo Inc.` is ordinary prose about two companies and not a
    mention of either. Case is ignored, the gap covers a comma and one hard
    wrap, and the suffix is the family in LEGAL_SUFFIX."""
    name_part = re.sub(LEGAL_SPACE + r"*,?" + LEGAL_SPACE + r"*" + LEGAL_SUFFIX + r"\s*$", "", canonical).strip()
    # Split on the SAME punctuation the gap treats as a separator, so a comma
    # inside the declared name does not get glued to a word and make the
    # comma-less spelling, the likeliest defect, invisible.
    words = [w for w in re.split(r"[\s,]+", name_part) if w]
    if not words:
        raise CannotRun("%s declares `%s`, which is a suffix and no name: the rule would have nothing to look for"
                        % (ENTITY_FILE, canonical))
    head = re.escape(words[0])
    interior = ""
    if len(words) > 1:
        # GREEDY: the longest run of the name's own words before a suffix is the
        # mention. Lazy would stop at the first word that happens to be a suffix,
        # so a name of the `Acme Robotics Co., Ltd.` shape would match its own
        # head and report its own correct spelling as wrong.
        interior = (r"(?:" + LEGAL_GAP + r"(?:" + "|".join(re.escape(w) for w in words[1:]) + r")){0,%d}" % len(words[1:]))
    return re.compile(r"(?i)\b" + head + interior + LEGAL_GAP + LEGAL_SUFFIX)


def visible(text):
    """`text` with every character a reader cannot see written out, so a message
    never prints two strings that differ and look identical. The declared name
    is printable ASCII by LEGAL_NAME_CHARS, so passing IT through here is
    provably the identity today; it stays because the set is one line and a
    project that widens it should not have to remember this call site."""
    out = []
    for c in text:
        if c == "\\":
            out.append("\\\\")
        elif " " <= c <= "~":
            out.append(c)
        elif c == "\n":
            out.append("\\n")
        elif c == "\t":
            out.append("\\t")
        elif c == "\r":
            out.append("\\r")
        elif ord(c) > 0xFFFF:
            out.append("\\U%08x" % ord(c))
        else:
            out.append("\\u%04x" % ord(c))
    return "".join(out)


def legal_name_mentions_checked(canonical):
    """The mention regex, proven to find the declared name itself. A declaration
    the rule cannot match is the one defect this class cannot survive: it reads
    green over a tree where every mention is wrong, or red over one where every
    mention is right, and both look like a working gate. An invisible character
    inside the name, a byte-order mark, a name with no suffix the family knows,
    and a name longer than the pattern can span all land here."""
    rx = legal_name_mentions(canonical)
    # Proved with the operation the rule PERFORMS. `fullmatch` would accept a
    # declaration whose own text the scan reads as a SHORTER mention, and the
    # class would then report the declared spelling as wrong on every page.
    found = [m.group(0) for m in rx.finditer(canonical)]
    if found != [canonical]:
        raise CannotRun("%s declares `%s`, which the rule that looks for it reads as %s: a name the rule cannot find "
                        "whole would make the class refuse its own spelling. Check it for a suffix inside the name "
                        "and for a suffix the rule knows (%s)"
                        % (ENTITY_FILE, visible(canonical),
                           ("nothing" if not found else ", ".join("`%s`" % visible(f) for f in found)),
                           ", ".join(LEGAL_SUFFIX_WORDS)))
    return rx


def legal_name_findings(root, files, canonical):
    """(findings, mentions). Every mention of the declared name in shipped text
    reads exactly as ENTITY_FILE declares it: an upper-case holder line, an
    added comma, a wrap and a different suffix are one defect, because a reader
    cannot tell which spelling binds. A word in front of the name is not a
    defect, because the scan looks for the name and not for what precedes it.

    The COUNT is returned because ZERO of them is the one result this class
    cannot tell from success: a declaration whose first word was retyped matches
    nothing, and a scan that matches nothing prints the line a perfect tree
    prints. The floor below makes that a finding, and the count reaches the
    summary so a reader of a green run can see what was judged."""
    rx = legal_name_mentions_checked(canonical)
    findings, mentions = [], 0
    for f in sorted(files):
        txt = read_text(root, f)
        if txt is None:
            continue
        for m in rx.finditer(txt):
            # EVERY mention counts, right or wrong: the count exists to say the
            # scan found the name at all, not to say how often it was misspelled.
            mentions += 1
            if m.group(0) == canonical:
                # A SECOND suffix behind a correct mention is its own defect:
                # `<name> Inc. LLC` reads as two forms of the company and a
                # reader cannot tell which one binds.
                after = TRAILING_SUFFIX_RE.match(txt, m.end())
                if after is None:
                    continue
                findings.append(Finding("shipped-text", f, line_of(txt, m.start()),
                                        "the legal name reads `%s` here, and `%s` follows it: one form of the name, "
                                        "not two" % (visible(m.group(0)), visible(after.group(0).strip()))))
                continue
            findings.append(Finding("shipped-text", f, line_of(txt, m.start()),
                                    "the legal name reads `%s` here and `%s` in %s: one spelling everywhere"
                                    % (visible(m.group(0)), visible(canonical), ENTITY_FILE)))
    if mentions == 0 and files:
        findings.append(Finding("shipped-text", ENTITY_FILE, 1,
                                "the declared legal name `%s` is written nowhere in shipped text: a name the scan "
                                "never finds makes this class read the same green as a tree where every mention is "
                                "right, so either the declaration is wrong or the project never states its own name"
                                % visible(canonical)))
    return findings, mentions


def check_shipped_text(root, files, phrases, ledger):
    """(findings, one line for the summary naming what the legal-name rule
    judged). A summary that does not say WHAT was judged cannot tell a reader
    that the rule ran over nothing."""
    findings = []
    phrase_res = [(p, re.compile(re.escape(p), re.I)) for p in phrases]
    for f in shipped_files(files):
        txt = read_text(root, f)
        if txt is None:
            continue
        n, hits = dash_surface(f, txt)
        known = ledger.get(f, 0)
        if n > known:
            first = hits[known] if known < len(hits) else hits[-1]
            findings.append(Finding("shipped-text", f, first[0], "typographic dash in shipped text (%d in the file, ledger allows %d): `%s`" % (n, known, first[1])))
        elif n < known:
            findings.append(Finding("shipped-text", f, 1, "the dash ledger says %d but the file carries %d: lower its line in %s (ledgers burn down)" % (known, n, LEDGER_FILE)))
        if f != "CHANGELOG.md":
            for m in TRACKER_RE.finditer(txt):
                findings.append(Finding("shipped-text", f, line_of(txt, m.start()), "tracker id `%s` in shipped text (only CHANGELOG.md may carry one)" % m.group(0)))
        for m in TODO_OWNER_RE.finditer(txt):
            findings.append(Finding("shipped-text", f, line_of(txt, m.start()), "`%s(%s)` names an owner; a shipped TODO carries a reason, not a person" % (m.group(1), m.group(2))))
        for phrase, rx in phrase_res:
            for m in rx.finditer(txt):
                findings.append(Finding("shipped-text", f, line_of(txt, m.start()), "`%s` is a claim the README contradicts (listed in %s)" % (phrase, PHRASES_FILE)))
        if f.endswith(".md"):
            for m in PICTOGRAPH_RE.finditer(txt):
                findings.append(Finding("shipped-text", f, line_of(txt, m.start()),
                                        "a pictograph on a shipped page (U+%04X): prose says what the product does in words"
                                        % ord(m.group(0))))
    canonical = legal_name(root)
    legal, mentions = legal_name_findings(root, shipped_files(files), canonical)
    findings.extend(legal)
    legal_tail = "legal-name=`%s` in %d mention(s)" % (visible(canonical), mentions)
    for path in sorted(ledger):
        if path not in files or is_test_path(path):
            findings.append(Finding("shipped-text", LEDGER_FILE, 1, "the dash ledger names %s, which is not a shipped file: delete its line" % path))
    return findings, legal_tail


# ---------------------------------------------------------------------------
# Class 6: work-state
# ---------------------------------------------------------------------------

# What the class never reads: raw benchmark evidence, the vendored message and
# binding trees, lockfiles, the legal texts, and the gate's own scripts and data
# files (they spell the patterns in order to refuse them).
WORKSTATE_EXCLUDE_RE = re.compile(
    r"^(?:docs/benchmarks/results/"
    r"|docs/legal/"
    r"|\.github/CLA/"
    r"|crates/native_ros2_messages/(?:msg/|upstream_msg_manifest\.txt$)"
    r"|crates/rmw_cerulion/src/ffi/vendored_bindings\.rs$"
    r"|tools/scripts/(?:check_public_surface\.(?:py|sh)|leak_scan\.py|leak_scan_allow\.txt|public_surface_[a-z_]+\.txt|publish_preflight\.sh)$"
    r")|(?:^|/)(?:Cargo\.lock|LICENSE(?:-[A-Z0-9]+)?)$"
)
WORKSTATE_KEY_RE = re.compile(r"^[a-z][a-z0-9]*(?:-[a-z0-9]+)*$")
# Keys that may never be ledgered: the wording is a hard failure wherever it
# appears, so the only way past it is a rewrite or an allow entry naming the one
# line. A ledger row would pre-authorise the next one, which for these is the
# whole point of the key.
WORKSTATE_NEVER_LEDGERED = ("approval-voice",)
# Below this many files the scan runs in this process; above it, in a pool.
WORKSTATE_POOL_MIN_FILES = 400
JOBS_ENV = "CHECK_PUBLIC_SURFACE_JOBS"

WORKSTATE_LEDGER_HEADER = """# Work-state ledger for tools/scripts/check_public_surface.sh (class work-state).
# `<count> <key> <path>`: the lines of a file that match one pattern key of
# tools/scripts/public_surface_workstate.txt today, after the allow list. The
# gate fails a file whose count is ABOVE its line (new wording: every matching
# line is printed) or BELOW it (a stale line pre-authorises the next one): after
# rewriting lines in a file, lower its line or delete it. Lines only ever go
# down, and a file that is not listed allows none. A USER-FACING page
# (README.md, CHANGELOG.md, docs/ outside docs/internals/, examples/**/*.md,
# crates/**/README.md, .github/**/*.md) is never listed here: it reads zero, or
# tools/scripts/public_surface_allow.txt names its one legitimate line.
# `check_public_surface.sh --regenerate-workstate-ledger` rewrites this file
# from the tree.
"""


def is_user_facing(rel):
    """The pages a user reads first. They may never be ledgered."""
    if rel in ("README.md", "CHANGELOG.md"):
        return True
    if rel.startswith("docs/"):
        return not rel.startswith("docs/internals/")
    if not rel.endswith(".md"):
        return False
    if rel.startswith("examples/") or rel.startswith(".github/"):
        return True
    return rel.startswith("crates/") and os.path.basename(rel) == "README.md"


def load_workstate_patterns(root):
    """`<key> | <python regex> | <what to write instead>` per line, in file order.
    The file is the single source of truth, so every defect in it stops the run:
    a missing file, a line of another shape, a regex that does not compile or
    that matches the empty string, a repeated key, a file with no entry. A
    skipped line would be a class that silently stopped matching."""
    full = os.path.join(root, WORKSTATE_FILE)
    if not os.path.isfile(full):
        raise CannotRun("%s is missing: the work-state class cannot run without its patterns" % WORKSTATE_FILE)
    patterns = []
    seen = set()
    try:
        with open(full, encoding="utf-8") as fh:
            raw_lines = fh.read().split("\n")
    except (OSError, UnicodeDecodeError) as exc:
        raise CannotRun("cannot read %s: %s" % (WORKSTATE_FILE, exc))
    for no, raw in enumerate(raw_lines, 1):
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        parts = [p.strip() for p in s.split(" | ")]
        if len(parts) != 3 or not all(parts):
            raise CannotRun("%s:%d: a pattern line is `<key> | <python regex> | <what to write instead>` (exactly two ` | ` separators)" % (WORKSTATE_FILE, no))
        key, source, instead = parts
        if not WORKSTATE_KEY_RE.match(key):
            raise CannotRun("%s:%d: the key %r is not lower-case words joined by hyphens" % (WORKSTATE_FILE, no, key))
        if key in seen:
            raise CannotRun("%s:%d: the key %r appears twice" % (WORKSTATE_FILE, no, key))
        try:
            rx = re.compile(source)
        except re.error as exc:
            raise CannotRun("%s:%d: the regex for %r does not compile: %s" % (WORKSTATE_FILE, no, key, exc))
        if rx.search(""):
            raise CannotRun("%s:%d: the regex for %r matches the empty string, so it would match every line" % (WORKSTATE_FILE, no, key))
        seen.add(key)
        patterns.append((key, source, instead))
    if not patterns:
        raise CannotRun("%s holds no pattern: an empty list would pass every line" % WORKSTATE_FILE)
    return patterns


def load_workstate_ledger(root, keys):
    """`<count> <key> <path>` per line -> {(path, key): (count, line number)}."""
    full = os.path.join(root, WORKSTATE_LEDGER_FILE)
    if not os.path.isfile(full):
        return {}
    ledger = {}
    with open(full, encoding="utf-8") as fh:
        for no, raw in enumerate(fh, 1):
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            m = re.match(r"([1-9]\d*)\s+(\S+)\s+(\S.*)$", s)
            if not m:
                raise CannotRun("%s:%d: a ledger line is `<count> <key> <path>` with a count of at least 1" % (WORKSTATE_LEDGER_FILE, no))
            count, key, path = int(m.group(1)), m.group(2), m.group(3)
            if key not in keys:
                raise CannotRun("%s:%d: %r is not a key of %s" % (WORKSTATE_LEDGER_FILE, no, key, WORKSTATE_FILE))
            if (path, key) in ledger:
                raise CannotRun("%s:%d: %s is listed twice for %r" % (WORKSTATE_LEDGER_FILE, no, path, key))
            ledger[(path, key)] = (count, no)
    return ledger


def around(line, m, width=96):
    """The line, or a window of it around the match, on one line."""
    flat = re.sub(r"\s+", " ", line.strip())
    if len(flat) <= width:
        return flat
    at = flat.find(re.sub(r"\s+", " ", m.group(0)))
    at = 0 if at < 0 else at
    start = max(0, min(at - width // 3, len(flat) - width))
    return flat[start:start + width]


def workstate_scan_text(txt, compiled):
    """[(line, key, matched text, snippet)]: at most one hit per line per key."""
    hits = []
    for no, line in enumerate(txt.split("\n"), 1):
        for key, rx in compiled:
            m = rx.search(line)
            if m:
                hits.append((no, key, m.group(0), around(line, m)))
    return hits


def _workstate_scan_chunk(job):
    """One pool task: scan `rels` under `root`. Module level so it pickles."""
    root, rels, specs = job
    compiled = [(key, re.compile(source)) for key, source in specs]
    out = []
    for rel in rels:
        txt = read_text(root, rel)
        if txt is not None:
            out.append((rel, workstate_scan_text(txt, compiled)))
    return out


def workstate_jobs():
    raw = os.environ.get(JOBS_ENV, "").strip()
    if raw:
        if not re.match(r"^[1-9]\d?$", raw):
            raise CannotRun("%s=%r is not a job count between 1 and 99" % (JOBS_ENV, raw))
        return int(raw), True
    return max(1, min(os.cpu_count() or 1, 8)), False


def workstate_scan(root, scanned, patterns):
    """({path: hits}, mode). The regexes are the cost (nine passes over every
    line of the tree), so a large tree is split over a process pool; a pool that
    cannot start (a sandbox without semaphores) falls back to this process. Both
    paths run the same function over the same files, so the result is the same."""
    specs = [(key, source) for key, source, _ in patterns]
    jobs, forced = workstate_jobs()
    if jobs > 1 and (forced or len(scanned) >= WORKSTATE_POOL_MIN_FILES):
        def size(rel):
            try:
                return os.path.getsize(os.path.join(root, rel))
            except OSError:
                return 0
        chunks = [[] for _ in range(min(len(scanned), jobs * 8) or 1)]
        for i, rel in enumerate(sorted(scanned, key=size, reverse=True)):
            chunks[i % len(chunks)].append(rel)
        try:
            from concurrent.futures import ProcessPoolExecutor
            from concurrent.futures.process import BrokenProcessPool
            pool_errors = (OSError, ImportError, NotImplementedError, BrokenProcessPool)
        except ImportError:
            ProcessPoolExecutor = None
            pool_errors = ()
        if ProcessPoolExecutor is not None:
            try:
                result = {}
                with ProcessPoolExecutor(max_workers=jobs) as pool:
                    for part in pool.map(_workstate_scan_chunk, [(root, c, specs) for c in chunks if c]):
                        result.update(part)
                return result, "pool of %d" % jobs
            except pool_errors:
                pass
    return dict(_workstate_scan_chunk((root, list(scanned), specs))), "one process"


def workstate_message(key, matched, shown):
    return "`%s` wording `%s`: `%s`" % (key, matched, shown)


def workstate_live_hits(root, files, patterns, allow):
    """(scanned files, {path: {key: [(line, message)]}}, mode): every hit the allow
    list does not excuse. The allow list works on the HIT, before the count, so a
    legitimate line (an MCAP chunk number, a bounding box) is never ledger debt."""
    scanned = [f for f in files if not WORKSTATE_EXCLUDE_RE.search(f)]
    raw, mode = workstate_scan(root, scanned, patterns)
    entries = [e for e in allow if e["cls"] == "work-state"]
    live = {}
    for path in sorted(raw):
        for line, key, matched, shown in raw[path]:
            message = workstate_message(key, matched, shown)
            excused = False
            for e in entries:
                if e["path"] == path and e["match"] in message:
                    e["used"] += 1
                    excused = True
                    break
            if not excused:
                live.setdefault(path, {}).setdefault(key, []).append((line, message))
    return scanned, live, mode


def check_workstate(root, files, patterns, ledger, allow):
    findings, notes = [], []
    scanned, live, _mode = workstate_live_hits(root, files, patterns, allow)
    fired = set()
    scanned_set = set(scanned)
    for path in sorted(set(live) | {p for p, _ in ledger}):
        if path not in scanned_set:
            continue  # a ledger line for a file that is not scanned is reported below
        user_facing = is_user_facing(path)
        for key, _source, _instead in patterns:
            hits = live.get(path, {}).get(key, [])
            known = 0 if (user_facing or key in WORKSTATE_NEVER_LEDGERED) else ledger.get((path, key), (0, 0))[0]
            if len(hits) > known:
                fired.add(key)
                tail = "" if not known else " (%d such lines in the file, the ledger allows %d)" % (len(hits), known)
                for line, message in hits:
                    findings.append(Finding("work-state", path, line, message + tail))
            elif len(hits) < known:
                findings.append(Finding("work-state", path, 1, "the work-state ledger says %d `%s` line(s) but the file carries %d: lower its line in %s (ledgers burn down)" % (known, key, len(hits), WORKSTATE_LEDGER_FILE)))
    for (path, key), (_count, no) in sorted(ledger.items(), key=lambda kv: kv[1][1]):
        if path not in scanned_set:
            findings.append(Finding("work-state", WORKSTATE_LEDGER_FILE, no, "the work-state ledger names %s, which is not a scanned shipped file: delete its line" % path))
        elif is_user_facing(path):
            findings.append(Finding("work-state", WORKSTATE_LEDGER_FILE, no, "%s is a user-facing page and may never be ledgered: delete its line, then rewrite the page to read zero or name its one legitimate line in %s" % (path, ALLOW_FILE)))
        elif key in WORKSTATE_NEVER_LEDGERED:
            findings.append(Finding("work-state", WORKSTATE_LEDGER_FILE, no, "`%s` may never be ledgered: delete its line, then rewrite the line in %s or name it in %s" % (key, path, ALLOW_FILE)))
    for key, _source, instead in patterns:
        if key in fired:
            notes.append("work-state: %s: write instead: %s" % (key, instead))
    return findings, notes


# ---------------------------------------------------------------------------
# Class 7: numbers-without-data
# ---------------------------------------------------------------------------

NUM = r"\d{1,3}(?:,\d{3})+(?:\.\d+)?|\d+(?:\.\d+)?"
UNITS = r"(?:µs|μs|us|ns|ms|Hz|kHz|MiB/s|replies/s)"
NUM_UNIT_RE = re.compile(r"(" + NUM + r")\s*" + UNITS + r"(?![A-Za-z])")
NUM_RANGE_RE = re.compile(r"(" + NUM + r")\s+(?:to|and|or)\s+(?:" + NUM + r")\s*" + UNITS + r"(?![A-Za-z])")
NUM_P_RE = re.compile(r"(" + NUM + r")\s*(?:" + UNITS + r"\s*)?(?:p50|p99)\b|\bp(?:50|99)s?\s*(?:of|is|at|=|:)?\s*(" + NUM + r")")
NUM_ANY_RE = re.compile(NUM)
TABLE_HEADER_RE = re.compile(r"p50|p99|µs|μs|\bms\b|\bns\b|\bHz\b", re.I)
ONE_SIG_FIG_RE = re.compile(r"^[1-9]0*$")


def doc_figures(txt):
    """[(line, number-string)] for every measured figure the page states."""
    out = []
    lines = txt.split("\n")
    in_table = False
    for no, line in enumerate(lines, 1):
        nums = set()
        for rx in (NUM_UNIT_RE, NUM_RANGE_RE):
            nums.update(m.group(1) for m in rx.finditer(line))
        nums.update((m.group(1) or m.group(2)) for m in NUM_P_RE.finditer(line))
        stripped = line.strip()
        if stripped.startswith("|"):
            cells = [c.strip() for c in stripped.strip("|").split("|")]
            if no < len(lines) and re.match(r"^\|?\s*:?-{3,}", lines[no].strip()):
                in_table = bool(TABLE_HEADER_RE.search(line))
            elif in_table and not re.match(r"^\|?\s*:?-{3,}", stripped):
                for cell in cells[1:]:
                    nums.update(NUM_ANY_RE.findall(cell))
        else:
            in_table = False
        for n in nums:
            plain = n.replace(",", "")
            if ONE_SIG_FIG_RE.match(plain):
                continue
            out.append((no, n))
    return out


def package_numbers(root, files):
    """Every number a shipped package states, plus the µs/ms renderings of its
    nanosecond CSV cells (µs at two decimals and as an integer, ms at one decimal:
    the HEADLINE convention)."""
    nums = set()
    for f in files:
        if not f.startswith("docs/benchmarks/results/"):
            continue
        base = os.path.basename(f)
        is_pkg_readme = base == "README.md" and f.count("/") == 4
        if not (f.endswith(".csv") or base == "HEADLINE.md" or is_pkg_readme):
            continue
        txt = read_text(root, f)
        if txt is None:
            continue
        nums.update(n.replace(",", "") for n in NUM_ANY_RE.findall(txt))
        if f.endswith(".csv"):
            rows = [l for l in txt.split("\n") if l.strip() and not l.startswith("#")]
            if not rows:
                continue
            header = [h.strip() for h in rows[0].split(",")]
            ns_cols = [i for i, h in enumerate(header) if h.endswith("_ns")]
            for row in rows[1:]:
                cells = row.split(",")
                for i in ns_cols:
                    if i < len(cells) and re.match(r"^\d+$", cells[i].strip()):
                        v = int(cells[i])
                        nums.add("%.2f" % (v / 1000.0))
                        nums.add("%d" % round(v / 1000.0))
                        nums.add("%.1f" % (v / 1e6))
    return nums


def check_numbers(root, files):
    findings = []
    docs = [f for f in ("README.md", "docs/PERFORMANCE.md") if f in files]
    if not docs:
        return findings
    pkg = package_numbers(root, files)
    for f in docs:
        txt = read_text(root, f)
        if txt is None:
            continue
        seen = set()
        for ln, n in doc_figures(txt):
            if (ln, n) in seen:
                continue
            seen.add((ln, n))
            if n.replace(",", "") not in pkg:
                findings.append(Finding("numbers-without-data", f, ln, "figure %s is in no shipped package (CSV, HEADLINE.md or package README under docs/benchmarks/results/); spell it as the package does or ship the package" % n))
    return findings


# ---------------------------------------------------------------------------
# Class 8: string-literal-rewrite (a rule with a named shape, not a tree check)
# ---------------------------------------------------------------------------

PREFIX_RE = re.compile(r'prefix\s*[:=]\s*"([^"]+)"')


def literal_rewrite_shape(src):
    """The shape of the launch-week incident, as an oracle: every absolute topic
    literal that sits under the graph prefix must still start with `/<prefix>/`.
    Returns the offending literals. A tree cannot know which literals belong
    together, which is why this is a rule and a fixture, never a gate."""
    code, literals = lex_rust(src)
    prefixes = PREFIX_RE.findall(src)
    if not prefixes:
        return []
    bad = []
    for _ln, body in literals:
        if body.startswith("/") and body.count("/") >= 2 and not any(body.startswith("/" + p + "/") for p in prefixes):
            bad.append(body)
    return bad


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------


def run_tree(root, out=sys.stdout):
    # The text cache is keyed by (root, path) and lives for the process, so a
    # SECOND run over a tree whose files changed in between reads what the first
    # run saw. One run per process is the shipped case and never notices; the
    # self-test drives many runs over one tree, where a stale read decides an arm
    # on content nobody wrote. The regenerators below clear it for the same
    # reason: a ledger written from stale text is checked in.
    _text_cache.clear()
    root = os.path.abspath(root)
    files = tracked_files(root)
    if not files:
        raise CannotRun("%s tracks no files" % root)
    allow = load_allow(root)
    phrases = load_phrases(root)
    ledger = load_ledger(root)
    patterns = load_workstate_patterns(root)
    ws_ledger = load_workstate_ledger(root, {key for key, _source, _instead in patterns})
    findings = []
    notes = []
    findings.extend(check_examples_shape(root, files))
    f2, n2, _tree = check_docs_refs(root, files)
    findings.extend(f2)
    notes.extend(n2)
    findings.extend(check_unreferenced_media(root, files))
    findings.extend(check_bench_citation(root, files))
    f5, legal_tail = check_shipped_text(root, files, phrases, ledger)
    findings.extend(f5)
    f6, n6 = check_workstate(root, files, patterns, ws_ledger, allow)
    findings.extend(f6)
    notes.extend(n6)
    findings.extend(check_numbers(root, files))
    # Allow entries excuse a finding by path + class + message substring.
    kept = []
    for fd in findings:
        excused = False
        for e in allow:
            if e["cls"] == fd.cls and e["path"] == fd.path and (e["match"] == "*" or e["match"] in fd.message):
                e["used"] += 1
                excused = True
                break
        if not excused:
            kept.append(fd)
    for e in allow:
        if not e["used"]:
            kept.append(Finding(e["cls"], ALLOW_FILE, e["line"], "stale allow entry for %s (%s): it excused nothing, delete it (a stale waiver pre-authorises the next finding)" % (e["path"], e["cls"])))
    kept.sort(key=lambda x: (CLASSES.index(x.cls), x.path, x.line, x.message))
    for fd in kept:
        out.write(fd.render() + "\n")
    for n in notes:
        out.write("note: " + n + "\n")
    failing = []
    for c in CLASSES:
        if any(fd.cls == c for fd in kept):
            failing.append(c)
            out.write("remedy: %s: %s\n" % (c, REMEDIES[c]))
    per_class = ", ".join("%s=%d" % (c, sum(1 for fd in kept if fd.cls == c)) for c in failing)
    tail = "files=%d, allowlist=%d, ledger=%d file(s), workstate-ledger=%d line(s), %s" % (
        len(files), len(allow), len(ledger), len(ws_ledger), legal_tail)
    if kept:
        out.write("check_public_surface: FAIL (%d finding(s) in %d class(es): %s; %s)\n" % (len(kept), len(failing), per_class, tail))
        return 1
    out.write("check_public_surface: OK (%d classes over %s)\n" % (len(CLASSES), tail))
    return 0


def regenerate_ledger(root, out=sys.stdout):
    """Rewrite the dash ledger from the tree: the live dash count per shipped
    file."""
    _text_cache.clear()
    root = os.path.abspath(root)
    files = tracked_files(root)
    counts = compute_dash_counts(root, files)
    full = os.path.join(root, LEDGER_FILE)
    with open(full, "w", encoding="utf-8") as fh:
        fh.write(LEDGER_HEADER)
        for path in sorted(counts):
            fh.write("%d %s\n" % (counts[path], path))
    out.write("check_public_surface: wrote %s (%d file(s), %d dash(es))\n" % (LEDGER_FILE, len(counts), sum(counts.values())))
    return 0


def regenerate_workstate_ledger(root, out=sys.stdout):
    """Rewrite the work-state ledger from the tree: the live count per file and
    key, after the allow list. A user-facing page is never written: it keeps
    failing until it reads zero."""
    _text_cache.clear()
    root = os.path.abspath(root)
    files = tracked_files(root)
    patterns = load_workstate_patterns(root)
    allow = load_allow(root)
    _scanned, live, mode = workstate_live_hits(root, files, patterns, allow)
    rows = []
    refused = 0
    for path in sorted(live):
        for key, _source, _instead in patterns:
            n = len(live[path].get(key, []))
            if not n:
                continue
            if is_user_facing(path) or key in WORKSTATE_NEVER_LEDGERED:
                refused += n
            else:
                rows.append((n, key, path))
    with open(os.path.join(root, WORKSTATE_LEDGER_FILE), "w", encoding="utf-8") as fh:
        fh.write(WORKSTATE_LEDGER_HEADER)
        for n, key, path in rows:
            fh.write("%d %s %s\n" % (n, key, path))
    out.write("check_public_surface: wrote %s (%d line(s) over %d file(s), %d matching line(s) in the tree; %d more on user-facing pages and never-ledgered keys stay unlisted and keep failing; scanned in %s)\n"
              % (WORKSTATE_LEDGER_FILE, len(rows), len({r[2] for r in rows}), sum(r[0] for r in rows), refused, mode))
    return 0


# ---------------------------------------------------------------------------
# Self-test: one fixture tree with a positive (caught) and a negative (not
# caught) arm per class, checked against a hand-written expected set.
# ---------------------------------------------------------------------------

FIXTURE_CLI = '''use clap::{Parser, Subcommand};

#[derive(Parser)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Graph management
    Graph {
        #[command(subcommand)]
        action: GraphAction,
    },
    /// Visualize
    Viz {
        topic: String,
    },
    /// Removed spelling, kept so the old form fails loudly.
    #[command(hide = true, disable_help_flag = true)]
    Ros {
        args: Vec<String>,
    },
    #[command(name = "make-it-so")]
    MakeItSo,
    Account {
        #[command(subcommand)]
        action: AccountAction,
    },
}

#[derive(Subcommand)]
pub enum GraphAction {
    /// Run a graph
    Run {
        name: String,
    },
    Validate,
    #[command(hide = true)]
    RunWorker,
}

#[derive(Subcommand)]
pub enum AccountAction {
    Devices {
        #[command(subcommand)]
        action: DevicesAction,
    },
}

#[derive(Subcommand)]
pub enum DevicesAction {
    List,
    Revoke {
        device_id: String,
    },
}
'''

EXPECTED_TREE = {
    "graph": {"run": {}, "validate": {}, "run-worker": {}},
    "viz": {},
    "ros": {},
    "make-it-so": {},
    "account": {"devices": {"list": {}, "revoke": {}}},
}

FIXTURE_NODE_OK = '''use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct TimerNode {
    #[output]
    tick: Vector3,
}

#[cerulion_node_impl]
impl TimerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // GraphRuntime::build in a comment is not a graph built in code
        tracing::info!("no dash here");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn harness() {
        let _r = GraphRuntime::build(cfg, nodes, &t, clock);
    }
}
'''

FIXTURE_NODE_BAD = '''use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct OneNode {
    #[output]
    a: Vector3,
}

#[cerulion_node]
#[derive(Default)]
struct TwoNode {
    #[input(trigger)]
    a: Vector3,
}

fn main() {
    let cfg = parse_graph(r#"prefix: x"#).unwrap();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let rt = GraphRuntime::build(cfg, factories, &mgr, clock).unwrap();
}
'''

FIXTURE_HEADLINE = """# fixture package

| cell | size | p50 | p99 (pooled) |
|---|---|---|---|
| split | 64 B | 4.08 µs | 7.38 µs |
| stock | 1 MiB | 11.0 ms | 43.4 ms |
"""

FIXTURE_CSV = """# schema note
payload_bytes,iterations,round_trip_p50_ns,round_trip_p99_ns,rep_count
64,10000,5537,15894,5
16777216,10000,14361,45781,5
"""

FIXTURE_README = """# Fixture

![logo](docs/media/used.svg)

A 1 MiB round trip measures 4.08 µs median and 7.38 µs p99; the stock line reads 11.0 ms / 43.4 ms.
Lockstep p50 is 5.54 µs at 64 B with a p99 of 15.89 µs; at 16 MiB the p99 is 45.78 µs.
A rounded 15.9 µs is a miss, and so is an invented 9.99 µs. A nominal 100 Hz load and a 10 ms interval are configuration.

| Payload | p50 / p99 in µs |
|---|---:|
| 64 B | 4.08 / 7.77 |

Benchmarks: [latency](benches/latency/README.md), results in [cited](docs/benchmarks/results/pkg-cited/HEADLINE.md).
"""

FIXTURE_PAGE = """# Page

See [the README](../README.md), [a missing page](missing.md), [external](https://example.invalid/x), [an anchor](#top),
and [a placeholder](STUDIO_MACOS_DOWNLOAD_URL), while [an all-caps file](../NOTICE) is a link like any other.

Run `cerulion graph run demo`, then `cerulion graph validate demo`; never `cerulion graph frobnicate demo`,
`cerulion replay bag.mcap` or `cerulion account devices purge`. The old `cerulion ros attach` spelling still exists as a stub,
`cerulion viz /topic` takes a positional, `cerulion make-it-so` is a renamed verb, and `cerulion account devices list` is three deep.
Prose that says cerulion graph bogus outside a code span is not a command.

```bash
cerulion graph run-worker --rank 0
cerulion bogus
cargo run -p cerulion_core --example basic_timer
cargo build -p cerulion_cli
```

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="media/used.svg, media/missing-dark.svg 2x">
  <img src="media/missing-fallback.svg" alt="a themed image">
</picture>

A markdown TABLE cell is where README.md and docs/user-api.md spell their verbs, so the verb walk reaches one:

| Verb | What it does |
|---|---|
| `cerulion graph run demo` | a table cell that names a real verb |
| `cerulion graph liftoff demo` | a table cell that names one the CLI does not define |

Paths: `crates/cerulion_core/src/wire.rs` is fine, `cerulion_core/src/wire.rs` is stale, `tools/scripts/x.sh` is fine,
`scripts/x.sh` is stale, and USER_API.md moved to docs/user-api.md.

RUSTFLOOR is a claim the README contradicts; this page is still UNPUBLISHED. A dash here: a \u2014 b.

Support: yes \u2713 and no \u2717 are typography and stay; a rocket \U0001F680 is not. Join the WAITWORD today.

\u26a0\ufe0f A sign with a variation selector on a heading is decoration, not a warning.
""".replace("WAITWORD", "wait" + "list").replace("RUSTFLOOR", "Rust 1." + "88").replace("UNPUBLISHED", "not yet published" + " to crates.io")

FIXTURE_DAMAGED = '''fn topics() {
    let prefix = "pos";
    let _out = "/producer/out";
}
'''

FIXTURE_CAREFUL = '''fn topics() {
    let prefix = "pos";
    let _out = "/pos/producer/out";
}
'''


# Work-state fixtures. One CAUGHT line per pattern key, and beside each key a
# legitimate neighbour that must stay silent. This file is outside the class's
# scan (the gate spells the wording in order to refuse it), so the fixture
# text is written plainly.
WORKSTATE_CAUGHT = [
    ("hardware-status", "[ROBOT-GATED] the pair has never run over a real link", "ROBOT-GATED"),
    ("who-decided", "the default changed by founder ruling", "founder"),
    ("plan-step", "the SUPERVISOR (chunk 3) plans the split", "chunk 3"),
    ("review-round", "review-train round 4 moved this check", "review-train"),
    ("session-id", "measured in pt42 on a quiet machine", "pt42"),
    ("deferred-work", "graceful forward there is a follow-up", "follow-up"),
    ("proof-tag", "MUTATION-VERIFIED: deleting the drain fails it", "MUTATION-VERIFIED"),
    ("our-machines", "structurally immune to the box's load", "the box"),
    ("candour-voice", "an honest FLOOR while the AGE stays exact", "honest"),
    ("change-ref", "the root cause named in PR #219 is a futile wake", "PR #219"),
    ("history-voice", "Pre-fix the wrapper only dropped the runtime", "Pre-fix"),
    ("design-ref", "the fence from design §3 stays closed", "design §3"),
    ("our-machines", "MEASURED on the authoring desk in August", "authoring desk"),
    ("plan-label", "an expired session is refused server-side (I4)", "(I4)"),
    ("promise-voice", "the Windows port is coming soon", "coming soon"),
    ("approval-voice", "the verb is one-shot by decision", "by decision"),
    ("review-actor", "REVIEW ITEM 7: a corrupt value must not be adopted", "REVIEW ITEM"),
    ("plan-step", "the sprint after this one widens the window", "sprint"),
    ("plan-label", "milestone M2 closes the encoder", "milestone M2"),
    ("plan-label", "the A0 era shipped without the guard", "A0 era"),
    ("change-ref", "the encoder change closes #482", "closes #482"),
    ("review-actor", "a subagent reads the page before an edit", "subagent"),
    ("review-actor", "As an AI a model cannot open the file", "As an AI"),
    ("review-actor", "Let me walk through the frame layout", "Let me"),
    ("approval-voice", "we agreed to keep the flag", "we agreed"),
    ("deferred-work", "XXX: the parser drops a byte", "XXX:"),
    ("deferred-work", "HACK: reach through the mutex", "HACK:"),
    ("deferred-work", "the launch branch carries the encoder", "launch branch"),
    ("deferred-work", "WIP: the encoder", "WIP"),
    ("deferred-work", "the wire size is to be confirmed", "to be confirmed"),
    ("our-machines", "our robot carries the driver", "our robot"),
    ("our-machines", "the publisher role is arn:aws:iam", "arn:aws:"),
    ("our-machines", "account 123456789012 owns the archive", "account 123456789012"),
    ("our-machines", "the bucket id is in the deploy notes", "bucket id"),
    ("proof-tag", "every mutant RUN under the cap", "mutant RUN"),
    ("history-voice", "it bit us twice before the guard landed", "bit us"),
    ("history-voice", "the flag was renamed from an older spelling", "was renamed from"),
    ("history-voice", "the old version dropped a byte", "the old version"),
    ("history-voice", "this commit renames the flag", "this commit"),
    ("history-voice", "in this change the flag is renamed", "in this change"),
    ("candour-voice", "we found the parser dropped a byte", "we found"),
    ("candour-voice", "I measured the round trip", "I measured"),
]
WORKSTATE_NEIGHBOURS = [
    ("hardware-status", "gated on a feature flag, and the hardware gate stays shut"),
    ("who-decided", "ruled out by the type system; a removal is a maintainer ruling"),
    ("plan-step", "two chunks of 4 MiB each, then stage 2 of the pipeline, and a sprinting counter"),
    ("review-round", "ground 3 of the pin header, the bypass-2 valve, a wave-shaped plot"),
    ("session-id", "the opt3 preset and a 12 pt font"),
    ("deferred-work", "follow the link, look up the value, keep a TODO list, WIPE the ring, an XXX redaction"),
    ("proof-tag", "UNVERIFIED coverage is reported as such, and every mutant runs under the cap"),
    ("our-machines", "the desk, a bounding box, a Box<dyn Trait> value, our own account 12345 and a bucket of tokens"),
    ("candour-voice", "a dishonest peer is refused; we find the parser sound and I run the suite"),
    ("change-ref", "colour #2196f3, issue number 12 upstream, a PRNG seed, and the writer closes 482 handles"),
    ("history-voice", "a prefix match, a fixed point, postfix notation, this committed value, the oldest version"),
    ("design-ref", "the designated section 3 of this page"),
    ("plan-label", "a C1 processor state, a NEL (C1) control character, WS1 as a shell variable, the A0 error code and milestone tracking"),
    ("promise-voice", "the soonest deadline wins, and the gap is acknowledged in the limits table"),
    ("approval-voice", "the merge re-sorts by decision position before publishing, and we agree on the wire format"),
    ("review-actor", "each Kahn peel wave is one level, a tolerant compare waves it through, and a \
false alarm would train an operator to skim the line"),
]
FIXTURE_WS_LEDGER = (
    "3 deferred-work crates/lib_clean/src/todo.rs\n"
    "2 plan-step crates/ws_ledger/src/at.rs\n"
    "1 plan-step crates/ws_ledger/src/above.rs\n"
    "3 plan-step crates/ws_ledger/src/below.rs\n"
    "1 plan-step crates/ws_gone/src/lib.rs\n"
    "1 candour-voice docs/ws_page.md\n"
    "1 approval-voice crates/ws/src/caught.rs\n"
)


def _put(root, rel, content):
    full = os.path.join(root, rel)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "w", encoding="utf-8") as fh:
        fh.write(content)


def build_fixture(root):
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "public_surface_phrases.txt"), encoding="utf-8") as fh:
        phrases = fh.read()
    _put(root, PHRASES_FILE, phrases)
    # The REAL pattern file: the arms below prove the shipped patterns still match.
    with open(os.path.join(here, os.path.basename(WORKSTATE_FILE)), encoding="utf-8") as fh:
        _put(root, WORKSTATE_FILE, fh.read())
    _put(root, ENTITY_FILE, "# the declared legal name\nAcme Systems Inc.\n")
    _put(root, WORKSTATE_LEDGER_FILE, WORKSTATE_LEDGER_HEADER + FIXTURE_WS_LEDGER)
    _put(root, "crates/ws/src/caught.rs", "".join("// %s\n" % line for _key, line, _m in WORKSTATE_CAUGHT) + "pub fn f() {}\n")
    _put(root, "crates/ws/src/controls.rs", "".join("// %s\n" % line for _key, line in WORKSTATE_NEIGHBOURS) + "pub fn g() {}\n")
    _put(root, "crates/ws/tests/it.rs", "// a test comment ships too: see Phase 4 of the plan\n#[test] fn t() {}\n")
    # At its count: two ledgered lines, plus a legitimate MCAP line the allow list names (never ledger debt).
    _put(root, "crates/ws_ledger/src/at.rs", "// chunk 1 wired the seam\n// chunk 2 wired the drain\n// the reader finds seq 0 in chunk 7 of the recording\npub fn f() {}\n")
    _put(root, "crates/ws_ledger/src/above.rs", "// chunk 1 wired the seam\n// chunk 2 wired the drain\npub fn f() {}\n")
    _put(root, "crates/ws_ledger/src/below.rs", "// chunk 1 wired the seam\npub fn f() {}\n")
    _put(root, "docs/ws_page.md", "# Limits\n\nThe honest wall is client isolation.\n")
    _put(root, "docs/ws_allowed.md", "# Recording\n\nA reader seeks to seq 0 in chunk 2 of the file.\n")
    # An allow entry names ONE line: two waivers here, and a third line of the
    # same wording that nothing excuses and the gate must still report.
    _put(root, "crates/ws_allowed/src/lib.rs",
         "// Undo: git revert <this commit> to put the file back\n"
         "// the tool does not reverse this commit either\n"
         "// this commit renames the flag with nothing excusing it\n"
         "pub fn k() {}\n")
    _put(root, "docs/benchmarks/results/pkg-cited/run.log", "pass-5 on the box: raw evidence is never rewritten\n")
    _put(root, "docs/legal/HISTORY.md", "# History\n\nThe founders relicensed the tree.\n")
    _put(root, ALLOW_FILE, "# fixture allow list\n"
         "crates/ws_ledger/src/at.rs | work-state | seq 0 in chunk 7 | an MCAP chunk index is the recording format's own number\n"
         "docs/ws_allowed.md | work-state | seq 0 in chunk 2 | an MCAP chunk index is the recording format's own number\n"
         "examples/gate_only | examples-shape | no Cargo.toml | a container gate, not a workspace: the maintainer ruling is pending\n"
         "crates/ws_allowed/src/lib.rs | work-state | git revert <this commit> | the undo instruction names the reader's own commit\n"
         "crates/ws_allowed/src/lib.rs | work-state | does not reverse this commit | the refusal names the reader's own commit\n"
         "docs/never.md | docs-refs | * | a stale entry that excuses nothing and must be reported\n")
    _put(root, LEDGER_FILE, LEDGER_HEADER + "1 crates/lib_ledgered/src/lib.rs\n3 crates/lib_stale/src/lib.rs\n2 crates/gone/src/lib.rs\n")
    _put(root, CLI_FILE, FIXTURE_CLI)
    _put(root, "crates/cerulion_core/src/lib.rs", "pub fn core() {}\n")
    _put(root, "crates/lib_ledgered/src/lib.rs", 'pub fn f() -> &\'static str { "one \u2014 dash" }\n')
    _put(root, "crates/lib_stale/src/lib.rs", 'pub fn f() -> &\'static str { "one \u2013 dash" }\n')
    _put(root, "crates/lib_new/src/lib.rs", 'pub fn f() -> &\'static str { "a \u2014 b" } // a \u2014 comment dash is exempt\n')
    _put(root, "crates/lib_clean/src/lib.rs", "// a comment \u2014 with a dash is exempt\npub fn f() {}\n")
    _put(root, "crates/lib_clean/src/todo.rs", "// TODO(needs-calibration): a reason, not a person\n// TODO(" + "someone): an owner\n// FIXME(" + "x): an owner\npub fn g() {}\n")
    _put(root, "crates/lib_clean/src/id.rs", "// see " + "C" + "ER" + "-" + "123" + " for history\npub fn h() {}\n")
    _put(root, "crates/lib_clean/Cargo.toml", '[package]\nname = "lib_clean"\ndescription = "clean \u2014 description"\n')
    _put(root, "crates/lib_new/Cargo.toml", '[package]\nname = "lib_new"\ndescription = "plain description"\n')
    _put(root, "crates/lib_clean/tests/it.rs",
         '#[test] fn t() { let _ = "a \u2014 dash in a test is exempt"; }\n'
         '// Acme Systems Corp. in a test path is outside the shipped set.\n')
    _put(root, "crates/test_fixtures/fx/src/lib.rs", 'pub const S: &str = "fixture \u2014 exempt";\n')
    _put(root, ".github/workflows/ci.yml", "name: CI \u2014 dashed\non: push\njobs:\n  a:\n    steps:\n      - name: plain step\n        run: echo hi # a \u2014 comment\n")
    _put(root, "CHANGELOG.md", "## 0.2.0\n- C" + "ER-999 may be cited here\n")
    # The legal name, one variant per file so each fails on its own arm, beside
    # the shapes that must stay silent: the declared spelling, the same name with
    # a word in front, a longer word the name only prefixes, and the word the
    # trailing guard exists for.
    _put(root, "README.md", FIXTURE_README
         + "\nPublished by Acme Systems Inc., and by The Acme Systems Inc., the same name with a word in front.\n"
         + "ACME SYSTEMS INC. shouts it, while MyAcme Systems, Inc. and Acme Systems Incidentally are other words.\n"
         + "\nAcme Systems\n\nIncorporated feedback shaped the API: a blank line ends the sentence.\n"
         + "Acme Systems Co-operative is another word, Acme works with Foo Inc. is two companies,\n"
         + "and Acme Robotics Systems Inc. adds a word the name does not have.\n"
         + "Acme Robotics Inc. is a different company that shares one word.\n")
    _put(root, "CITATION.cff",
         'cff-version: 1.2.0\nauthors:\n  - name: "Acme Systems, Inc. and contributors"\n')
    # OUTSIDE the four files the old rule read: the scan is the shipped set now.
    _put(root, "crates/lib_clean/src/legal.rs",
         "//! Shipped by Acme Systems Ltd.\n"
         "//! Earlier releases said Acme Inc., which DROPS a word of the name.\n"
         "//! The notes wrap it as Acme\n//! Systems Corp. behind a comment prefix.\n"
         "//! A narrow column wraps it over Acme\n//! Systems\n//! Ltd. three lines.\n"
         "pub fn legal() {}\n")
    # The suffixes no other fixture reaches, one per line.
    _put(root, "docs/legal/AUTHORS",
         "Acme Systems Incorporated\nAcme Systems Corporation\nAcme Systems Limited\n"
         "Acme Systems GmbH\nAcme Systems Co.\n"
         # the short forms without their period, and a no-break space no reader sees
         "Acme Systems Inc\nAcme\u00a0Systems Inc.\n"
         # a wrap behind a quote marker and behind a hash, the two prefixes no
         # other fixture reaches
         "> Acme\n> Systems Ltd.\n"
         "# Acme\n# Systems Corp.\n"
         # a correct mention with a SECOND suffix behind it
         "Acme Systems Inc. LLC\n")
    # A HARD WRAP inside the NAME, which a pattern joining the name words with a
    # plain space would not see.
    _put(root, ".github/CLA/individual.md", "# Agreement\n\nThis agreement is with Acme\nSystems Inc. and nobody else.\n")
    # A notice shaped like a real one: the project's own line first, an INDENTED
    # third-party copyright line, and a lowercase attribution. None of it is
    # consulted, because the name is declared rather than parsed out of prose,
    # and none of it is refused, because a vendor does not carry the name.
    _put(root, "docs/legal/NOTICE",
         "Acme Systems\n"
         "Copyright (c) 2026 Acme Systems Inc. and the Acme Systems Contributors\n"
         "\n"
         "            Copyright (c) 2008, Willow Garage, Inc. The repository LICENSE applies.\n"
         "Portions copyright Eclipse Foundation Inc.\n"
         "A later line writes it Acme Systems LLC instead.\n")
    _put(root, "docs/page.md", FIXTURE_PAGE)
    _put(root, "docs/PERFORMANCE.md", "# Performance\n\nThe 64 B p50 is 4.08 µs.\n")
    _put(root, "docs/media/used.svg", "<svg/>\n")
    _put(root, "docs/media/orphan.png", "PNG-ish text\n")
    _put(root, "docs/media/index-only.svg", "<svg/>\n")
    _put(root, "docs/media/README.md", "| `index-only.svg` | listed only here |\n")
    _put(root, "docs/media/charts/gen.py", "print('generator')\n")
    _put(root, "docs/benchmarks/README.md", "Packages: pkg-cited\n")
    _put(root, "docs/benchmarks/results/pkg-cited/HEADLINE.md", FIXTURE_HEADLINE)
    _put(root, "docs/benchmarks/results/pkg-cited/lockstep/results.csv", FIXTURE_CSV)
    _put(root, "docs/benchmarks/results/pkg-uncited/HEADLINE.md", "# nobody links this\n")
    _put(root, "benches/README.md", "Trees: latency\n")
    _put(root, "benches/latency/README.md", "# latency bench\n")
    _put(root, "benches/orphanbench/run.sh", "#!/bin/sh\n")
    _put(root, "examples/good_ws/Cargo.toml", "[workspace]\nmembers = [\"nodes/*\"]\n")
    _put(root, "examples/good_ws/graphs/good.yaml", "prefix: good\nnodes: []\n")
    _put(root, "examples/good_ws/nodes/timer/src/lib.rs", FIXTURE_NODE_OK)
    _put(root, "examples/good_ws/nodes/timer/tests/harness.rs", "fn h() { let _ = GraphRuntime::build(a, b, &c, d); }\n")
    _put(root, "examples/good_ws/README.md", "Run: `cerulion graph run good`\n")
    _put(root, "examples/bad_ws/Cargo.toml", "[workspace]\n")
    _put(root, "examples/bad_ws/nodes/two/src/lib.rs", FIXTURE_NODE_BAD)
    _put(root, "examples/gate_only/Dockerfile", "FROM scratch\n")
    _put(root, "examples/gate_only/graphs/g.yaml", "prefix: g\n")
    _put(root, "examples/gate_only/nodes/n/src/lib.rs", "#[cerulion_node]\nstruct N;\n")
    _put(root, "crates/lib_clean/examples/prog.rs", "fn main() { let cfg = parse_graph(\"prefix: p\"); let _m = TransportManager::get_or_init(); }\n")
    _put(root, "AGENTS.md", "# guide\n\nRun `cargo test -p lib_clean --example prog` never.\n")
    _put(root, "shape/damaged.rs", FIXTURE_DAMAGED)
    _put(root, "shape/careful.rs", FIXTURE_CAREFUL)
    for cmd in (["git", "init", "-q"], ["git", "add", "-A"]):
        r = subprocess.run(cmd, cwd=root, capture_output=True)
        if r.returncode != 0:
            raise CannotRun("fixture git %s failed: %s" % (cmd[1], r.stderr.decode("utf-8", "replace")))


# (class, path, substring of the message) for every finding the fixture must raise.
EXPECTED = [
    ("examples-shape", "examples/bad_ws", "no graphs/*.yaml"),
    ("examples-shape", "examples/bad_ws/nodes/two/src/lib.rs", "declares 2 #[cerulion_node]"),
    ("examples-shape", "examples/bad_ws/nodes/two/src/lib.rs", "graph built in code (parse_graph()"),
    ("examples-shape", "examples/bad_ws/nodes/two/src/lib.rs", "graph built in code (Box<dyn NodeEntry>)"),
    ("examples-shape", "examples/bad_ws/nodes/two/src/lib.rs", "graph built in code (GraphRuntime::build)"),
    ("examples-shape", "crates/lib_clean/examples/prog.rs", "graph built in code (parse_graph()"),
    ("examples-shape", "crates/lib_clean/examples/prog.rs", "graph built in code (TransportManager::get_or_init)"),
    ("examples-shape", "docs/page.md", "`cargo ... --example`"),
    ("examples-shape", "AGENTS.md", "`cargo ... --example`"),
    ("docs-refs", "docs/page.md", "link target missing.md"),
    ("docs-refs", "docs/page.md", "link target ../NOTICE resolves to NOTICE"),
    ("docs-refs", "docs/page.md", "link target media/missing-dark.svg resolves to docs/media/missing-dark.svg"),
    ("docs-refs", "docs/page.md", "link target media/missing-fallback.svg resolves to docs/media/missing-fallback.svg"),
    ("docs-refs", "docs/page.md", "`cerulion graph frobnicate` is not a cerulion verb"),
    ("docs-refs", "docs/page.md", "`cerulion replay` is not a cerulion verb"),
    ("docs-refs", "docs/page.md", "`cerulion account devices purge` is not a cerulion verb"),
    ("docs-refs", "docs/page.md", "`cerulion bogus` is not a cerulion verb"),
    # README.md and docs/user-api.md spell every verb in a table cell, so the
    # class has to reach one: this is that arm.
    ("docs-refs", "docs/page.md", "`cerulion graph liftoff` is not a cerulion verb"),
    ("docs-refs", "docs/page.md", "pre-move path spelling `cerulion_core/`"),
    ("docs-refs", "docs/page.md", "pre-move path spelling `scripts/`"),
    ("docs-refs", "docs/page.md", "`USER_API.md` no longer exists"),
    ("docs-refs", ALLOW_FILE, "stale allow entry for docs/never.md"),
    ("unreferenced-media", "docs/media/orphan.png", "not referenced"),
    ("unreferenced-media", "docs/media/index-only.svg", "not referenced"),
    ("bench-citation", "docs/benchmarks/results/pkg-uncited", "not linked"),
    ("bench-citation", "benches/orphanbench", "not named"),
    ("shipped-text", "crates/lib_new/src/lib.rs", "typographic dash in shipped text (1 in the file, ledger allows 0)"),
    ("shipped-text", "crates/lib_stale/src/lib.rs", "the dash ledger says 3 but the file carries 1"),
    ("shipped-text", LEDGER_FILE, "names crates/gone/src/lib.rs"),
    ("shipped-text", "crates/lib_clean/Cargo.toml", "typographic dash"),
    ("shipped-text", ".github/workflows/ci.yml", "typographic dash"),
    ("shipped-text", "docs/page.md", "typographic dash"),
    ("shipped-text", "crates/lib_clean/src/todo.rs", "`TODO(" + "someone)` names an owner"),
    ("shipped-text", "crates/lib_clean/src/todo.rs", "`FIXME(" + "x)` names an owner"),
    ("shipped-text", "crates/lib_clean/src/id.rs", "tracker id"),
    ("shipped-text", "docs/page.md", "`Rust 1." + "88` is a claim"),
    ("shipped-text", "docs/page.md", "`not yet published" + " to crates.io` is a claim"),
    ("shipped-text", "docs/page.md", "`" + "wait" + "list` is a claim"),
    ("shipped-text", "docs/page.md", "a pictograph on a shipped page (U+1F680)"),
    ("shipped-text", "docs/page.md", "a pictograph on a shipped page (U+26A0)"),
    ("shipped-text", "docs/page.md", "a pictograph on a shipped page (U+FE0F)"),
    ("shipped-text", "README.md", "the legal name reads `ACME SYSTEMS INC.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "CITATION.cff", "the legal name reads `Acme Systems, Inc.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    # the only row that pins a LINE as well as a message: the class gains a position oracle
    (":3: shipped-text", ".github/CLA/individual.md", "the legal name reads `Acme\\nSystems Inc.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "docs/legal/NOTICE", "the legal name reads `Acme Systems LLC` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "crates/lib_clean/src/legal.rs", "the legal name reads `Acme Systems Ltd.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "crates/lib_clean/src/legal.rs", "the legal name reads `Acme Inc.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "crates/lib_clean/src/legal.rs", "the legal name reads `Acme\\n//! Systems Corp.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "crates/lib_clean/src/legal.rs", "the legal name reads `Acme\\n//! Systems\\n//! Ltd.` here and `Acme Systems Inc.` in " + ENTITY_FILE),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Inc.` here, and `LLC` follows it"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Incorporated` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Corporation` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Limited` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems GmbH` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Co.` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme Systems Inc` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme\\u00a0Systems Inc.` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme\\n> Systems Ltd.` here"),
    ("shipped-text", "docs/legal/AUTHORS", "the legal name reads `Acme\\n# Systems Corp.` here"),
    ("work-state", "crates/ws_allowed/src/lib.rs", "`history-voice` wording `this commit`: `// this commit renames the flag with nothing excusing it`"),
    ("work-state", "crates/ws/tests/it.rs", "`plan-step` wording `Phase 4`"),
    ("work-state", "crates/ws_ledger/src/above.rs", "`plan-step` wording `chunk 1`: `// chunk 1 wired the seam` (2 such lines in the file, the ledger allows 1)"),
    ("work-state", "crates/ws_ledger/src/above.rs", "`plan-step` wording `chunk 2`: `// chunk 2 wired the drain` (2 such lines in the file, the ledger allows 1)"),
    ("work-state", "crates/ws_ledger/src/below.rs", "the work-state ledger says 3 `plan-step` line(s) but the file carries 1"),
    ("work-state", WORKSTATE_LEDGER_FILE, "names crates/ws_gone/src/lib.rs"),
    ("work-state", WORKSTATE_LEDGER_FILE, "docs/ws_page.md is a user-facing page and may never be ledgered"),
    ("work-state", WORKSTATE_LEDGER_FILE, "`approval-voice` may never be ledgered"),
    ("work-state", "docs/ws_page.md", "`candour-voice` wording `honest`: `The honest wall is client isolation.`"),
    ("numbers-without-data", "README.md", "figure 15.9 is in no shipped package"),
    ("numbers-without-data", "README.md", "figure 9.99 is in no shipped package"),
    ("numbers-without-data", "README.md", "figure 7.77 is in no shipped package"),
]


EXPECTED.extend(("work-state", "crates/ws/src/caught.rs", "`%s` wording `%s`: `// %s`" % (key, matched, line)) for key, line, matched in WORKSTATE_CAUGHT)


def self_test(out=sys.stdout):
    arms = 0

    def arm(name, ok, detail=""):
        nonlocal arms
        arms += 1
        if not ok:
            out.write("check_public_surface --self-test: FAIL at arm %r %s\n" % (name, detail))
            sys.exit(1)

    tree = parse_cli_tree(FIXTURE_CLI)
    arm("verb-tree-oracle", tree == EXPECTED_TREE, repr(tree))
    arm("kebab", kebab("RunWorker") == "run-worker" and kebab("Ros2") == "ros2")
    # The lexer: strings located, comments blanked, char literals and lifetimes skipped.
    code, lits = lex_rust('fn f<\'a>(x: &\'a str) { let c = \'"\'; let s = "q \u2014 \\" e"; /* c \u2014 */ // t \u2014\n let r = r#"raw \u2014 "#; }')
    arm("lexer-literals", [b for _, b in lits] == ['q \u2014 \\" e', 'raw \u2014 '], repr(lits))
    arm("lexer-code-view", "\u2014" not in code and "r#" in code and len(code) > 0)
    regions = cfg_test_regions(lex_rust(FIXTURE_NODE_OK)[0])
    arm("cfg-test-region", len(regions) == 1 and regions[0][0] == 19 and regions[0][1] >= 24, repr(regions))
    arm("literal-rewrite-damaged", literal_rewrite_shape(FIXTURE_DAMAGED) == ["/producer/out"])
    arm("literal-rewrite-careful", literal_rewrite_shape(FIXTURE_CAREFUL) == [])
    # Work-state: which pages are user-facing, and what the class never reads.
    arm("user-facing-oracle",
        all(is_user_facing(f) for f in ("README.md", "CHANGELOG.md", "docs/page.md", "docs/media/a.svg", "docs/AGENTS.md", "examples/go2/README.md",
                                        "crates/cerulion_bag/README.md", ".github/CONTRIBUTING.md"))
        and not any(is_user_facing(f) for f in ("AGENTS.md", "docs/internals/design.md", "examples/go2/nodes/n/src/lib.rs", "crates/cerulion_bag/AGENTS.md",
                                                "crates/cerulion_bag/src/lib.rs", ".github/workflows/ci.yml", "tools/scripts/install.sh")))
    arm("work-state-exclusions",
        all(WORKSTATE_EXCLUDE_RE.search(f) for f in ("docs/benchmarks/results/pkg/run.log", "Cargo.lock", "examples/go2/Cargo.lock", "docs/legal/HISTORY.md",
                                                     ".github/CLA/individual.md", "LICENSE", "tools/scripts/check_public_surface.py",
                                                     "tools/scripts/check_public_surface.sh", "tools/scripts/leak_scan.py", WORKSTATE_FILE,
                                                     WORKSTATE_LEDGER_FILE, ALLOW_FILE, "crates/native_ros2_messages/msg/std_msgs/String.msg",
                                                     "crates/rmw_cerulion/src/ffi/vendored_bindings.rs"))
        and not any(WORKSTATE_EXCLUDE_RE.search(f) for f in ("crates/cerulion_core/src/lib.rs", "tools/scripts/check_agents_md.sh", "docs/benchmarks/README.md",
                                                             "crates/native_ros2_messages/src/lib.rs", "docs/page.md", "AGENTS.md", "benches/latency/bench.py")))
    # The fixture writes its own declaration, so nothing above proves the SHIPPED
    # one parses. This drives the REAL functions over the real file: a copy of
    # their parsing here would pass while the run against the tree exits 3.
    here_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    shipped_name, shipped_err = None, None
    try:
        shipped_name = legal_name(here_root)
        legal_name_mentions_checked(shipped_name)
    except CannotRun as exc:
        shipped_err = str(exc)
    arm("the-shipped-declaration-parses-and-is-matchable", shipped_err is None,
        shipped_err or repr(shipped_name))
    # Not scanned by the text classes, and not read by work-state: the file
    # spells the name in order to refuse every other spelling of it.
    arm("the-declaration-is-a-data-file",
        ENTITY_FILE in DATA_FILES and ENTITY_FILE not in shipped_files([ENTITY_FILE])
        and WORKSTATE_EXCLUDE_RE.search(ENTITY_FILE) is not None)
    # `visible` writes out what a reader cannot see. Every branch of it, and the
    # promise that it is the identity over a declared name.
    arm("visible-writes-out-what-cannot-be-seen",
        visible("a\\b") == "a\\\\b" and visible("a\nb") == "a\\nb" and visible("a\tb") == "a\\tb"
        and visible("a\rb") == "a\\rb" and visible("a\u00a0b") == "a\\u00a0b"
        and visible("a\U0001F680b") == "a\\U0001f680b"
        and visible("a\nb") != visible("a\\nb"),
        repr([visible("a\\b"), visible("a\nb"), visible("a\U0001F680b")]))
    arm("visible-is-the-identity-over-a-declared-name",
        all(visible(c) == c for c in LEGAL_NAME_CHARS), repr(sorted(LEGAL_NAME_CHARS)))
    # Every character the set permits really is usable in a declaration, and the
    # suffix family a message names is the family the pattern matches.
    arm("every-permitted-character-builds-a-usable-name",
        all(legal_name_mentions_checked("A%sB Inc." % c).search("A%sB Inc." % c) is not None
            for c in sorted(LEGAL_NAME_CHARS) if c not in " .,"),
        "one of the permitted characters cannot appear in a declared name")
    arm("the-refusal-names-the-suffixes-the-pattern-holds",
        all(re.compile(LEGAL_SUFFIX).fullmatch(w) for w in LEGAL_SUFFIX_WORDS), repr(LEGAL_SUFFIX_WORDS))
    arm("the-character-set-holds-every-suffix-the-family-names",
        all(c in LEGAL_NAME_CHARS for w in LEGAL_SUFFIX_WORDS for c in w), repr(LEGAL_SUFFIX_WORDS))
    # One per suffix: a name DECLARED with it is found whole, a variant of that
    # name is found and is not the declaration, and the near miss stays silent.
    for _w in LEGAL_SUFFIX_WORDS:
        _name = "Acme Systems " + _w
        try:
            _rx = legal_name_mentions_checked(_name)
        except CannotRun as _exc:
            arm("suffix:" + _w, False, str(_exc))
            continue
        _whole = _rx.search(_name)
        _variant = _rx.search(_name.upper())
        _near = _rx.search("MyAcme Systems " + _w)
        arm("suffix:" + _w,
            _whole is not None and _whole.group(0) == _name
            and _variant is not None and _variant.group(0) != _name
            and _near is None,
            repr([_whole and _whole.group(0), _variant and _variant.group(0), _near and _near.group(0)]))
    # A name whose head carries a regex character is a literal, not a pattern.
    arm("a-head-with-a-regex-character-is-a-literal",
        legal_name_mentions("A.B Inc.").search("AxB Inc.") is None
        and legal_name_mentions("A.B Inc.").search("A.B Inc.") is not None)
    arm("an-interior-word-with-a-regex-character-is-a-literal",
        legal_name_mentions("Acme S.T Inc.").search("Acme SxT Inc.") is None
        and legal_name_mentions("Acme S.T Inc.").search("Acme S.T Inc.") is not None)
    # 9: a comma inside the declared name is a separator, so the comma-less
    # spelling, the likeliest defect, is still a mention.
    arm("a-comma-in-the-declared-name-is-a-separator",
        legal_name_mentions("Acme, Systems Inc.").search("Acme Systems Inc.") is not None
        and legal_name_mentions("Acme, Systems Inc.").search("Acme, Systems Inc.") is not None)
    # 1: the per-suffix arms below are GENERATED from this list, so a member that
    # is deleted deletes its own arm. This second copy is deliberate: it is the
    # only thing that makes a deletion loud.
    arm("the-suffix-family-is-the-list-it-is-meant-to-be",
        LEGAL_SUFFIX_WORDS == ("Inc", "Inc.", "Incorporated", "Corp", "Corp.", "Corporation", "Co.",
                               "LLC", "LLP", "Ltd", "Ltd.", "Limited", "PLC", "GmbH", "AG",
                               "S.A.", "B.V.", "AB", "Oy", "A/S"), repr(LEGAL_SUFFIX_WORDS))
    # 6a: the same for the punctuation the set permits, which the loop below
    # iterates over and therefore cannot notice losing.
    arm("the-character-set-holds-the-punctuation-a-name-takes",
        frozenset("&.,-'/()") <= LEGAL_NAME_CHARS, repr(sorted(LEGAL_NAME_CHARS)))
    # 8: the refusal names the family, rather than a copy of it that can drift.
    arm("the-refusal-message-names-every-suffix",
        all(w in ", ".join(LEGAL_SUFFIX_WORDS) for w in LEGAL_SUFFIX_WORDS))
    # GREEDY: the longest run of the name's own words before a suffix.
    arm("the-longest-run-of-name-words-is-the-mention",
        [x.group(0) for x in legal_name_mentions("Acme Robotics Co., Ltd.").finditer("Acme Robotics Co., Ltd.")]
        == ["Acme Robotics Co., Ltd."])
    arm("one-sig-fig", all(ONE_SIG_FIG_RE.match(x) for x in ("100", "10", "2", "500")) and not any(ONE_SIG_FIG_RE.match(x) for x in ("87", "104", "4.08", "311")))
    with tempfile.TemporaryDirectory(prefix="public-surface-selftest-") as tmp:
        root = os.path.join(tmp, "tree")
        os.makedirs(root)
        build_fixture(root)
        import io

        buf = io.StringIO()
        rc = run_tree(root, buf)
        text = buf.getvalue()
        lines = text.rstrip("\n").split("\n")
        arm("fixture-exit-1", rc == 1, "rc=%d" % rc)
        arm("summary-last", lines[-1].startswith("check_public_surface: FAIL ("), lines[-1])
        findings = [l for l in lines if re.match(r"^\S+:\d+: [a-z-]+: ", l)]
        arm("every-finding-has-file-line-class", all(re.match(r"^[^:]+:\d+: (%s): .+" % "|".join(re.escape(c) for c in CLASSES), l) for l in findings))
        for cls, path, sub in EXPECTED:
            # a `cls` written as ":<line>: <class>" pins the line number too
            marker = (cls + ": ") if cls.startswith(":") else (": %s: " % cls)
            hit = [l for l in findings if l.startswith(path + ":") and marker in l and sub in l]
            arm("expected:%s:%s:%s" % (cls, path, sub), len(hit) >= 1, "\n" + text)
        arm("no-unexpected-findings", len(findings) == len(EXPECTED), "expected %d findings, got %d:\n%s" % (len(EXPECTED), len(findings), text))
        # Negative controls, each named: the clean arm of every class must stay silent.
        for path in ("examples/good_ws", "examples/good_ws/nodes/timer/src/lib.rs", "examples/good_ws/nodes/timer/tests/harness.rs",
                     "crates/lib_ledgered/src/lib.rs", "crates/lib_clean/src/lib.rs", "crates/lib_clean/tests/it.rs",
                     "crates/test_fixtures/fx/src/lib.rs", "crates/lib_new/Cargo.toml", "CHANGELOG.md", "docs/media/used.svg",
                     "docs/benchmarks/results/pkg-cited", "benches/latency", "docs/PERFORMANCE.md", "examples/good_ws/README.md"):
            arm("control:" + path, not any(l.startswith(path + ":") for l in findings), "\n" + text)
        # A control is a substring that must NOT appear in the findings. Deleting
        # the fixture text it controls would make it pass while proving nothing,
        # so each control's text is asserted PRESENT in the tree first.
        for text, rel in ((" Acme Systems Co-operative ", "README.md"),
                          (" Acme works with Foo Inc. ", "README.md"),
                          (" Acme Robotics Systems Inc. ", "README.md"),
                          ("Acme Robotics Inc. is a different company", "README.md"),
                          ("MyAcme Systems, Inc.", "README.md"),
                          ("The Acme Systems Inc.", "README.md"),
                          ("Acme Systems Incidentally", "README.md"),
                          ("Acme Systems\n\nIncorporated feedback", "README.md"),
                          ("Acme Systems Corp.", "crates/lib_clean/tests/it.rs")):
            body = open(os.path.join(root, rel), encoding="utf-8").read()
            arm("control-text-is-in-the-fixture:" + text.strip()[:34], text.strip() in body, rel)
        for sub in ("`cerulion graph run`", "`cerulion graph validate`", "`cerulion graph run-worker`", "`cerulion ros`", "`cerulion viz`",
                    "`cerulion make-it-so`", "`cerulion account devices list`", "graph bogus", "figure 4.08", "figure 5.54", "figure 15.89",
                    "figure 45.78", "figure 11.0", "figure 43.4", "figure 100", "figure 10 ", "`TODO(needs-calibration)`", "crates/cerulion_core/`", "\u2713", "\u2717",
                    "Eclipse Foundation", "Willow Garage", "MyAcme", "Incidentally", "Acme Systems Corp.",
                    "Co-operative", "works with Foo", "Acme Robotics Systems", "Acme Robotics Inc.",
                    "git revert <this commit>", "does not reverse this commit"):
            arm("control-message:" + sub, not any(sub in l for l in findings), "\n" + text)
        # Work-state, key by key, against the REAL pattern file: the caught line fires
        # its own key, and the legitimate neighbour fires no key at all.
        ws_patterns = load_workstate_patterns(root)
        ws_compiled = dict((key, re.compile(source)) for key, source, _instead in ws_patterns)
        arm("work-state-key-set", sorted(ws_compiled) == sorted(set(key for key, _l, _m in WORKSTATE_CAUGHT)), repr(sorted(ws_compiled)))
        for key, line, matched in WORKSTATE_CAUGHT:
            m = ws_compiled[key].search(line)
            arm("work-state-caught:" + key, m is not None and m.group(0) == matched, repr(m and m.group(0)))
        for key, line in WORKSTATE_NEIGHBOURS:
            arm("work-state-neighbour:" + key, not any(rx.search(line) for rx in ws_compiled.values()), line)
        ws_lines = [l for l in findings if ": work-state: " in l]
        for path in ("crates/ws/src/controls.rs", "crates/ws_ledger/src/at.rs", "crates/lib_clean/src/todo.rs", "docs/ws_allowed.md",
                     "docs/benchmarks/results/pkg-cited/run.log", "docs/legal/HISTORY.md", ALLOW_FILE):
            arm("work-state-control:" + path, not any(l.startswith(path + ":") for l in ws_lines), "\n" + text)
        arm("work-state-note-per-fired-key", all(any(l.startswith("note: work-state: %s: write instead: " % key) for l in lines) for key in ws_compiled), text)
        arm("placeholder-is-a-note", any(l.startswith("note: docs-refs: docs/page.md:") and "STUDIO_MACOS_DOWNLOAD_URL" in l for l in lines), text)
        arm("allow-entry-excused-gate-only", not any(l.startswith("examples/gate_only:") and "no Cargo.toml" in l for l in findings))
        arm("remedy-per-failing-class", all(any(l.startswith("remedy: %s: " % c) for l in lines) for c in CLASSES if c != "string-literal-rewrite"), text)
        # A tree with nothing wrong is reported OK with exit 0 (the run cannot be vacuous).
        clean = os.path.join(tmp, "clean")
        os.makedirs(clean)
        _put(clean, CLI_FILE, FIXTURE_CLI)
        _put(clean, PHRASES_FILE, open(os.path.join(root, PHRASES_FILE), encoding="utf-8").read())
        real_patterns = open(os.path.join(root, WORKSTATE_FILE), encoding="utf-8").read()
        _put(clean, WORKSTATE_FILE, real_patterns)
        _put(clean, ENTITY_FILE, "# the declared legal name\nAcme Systems Inc.\n")
        _put(clean, "README.md", "# clean\n\nRun `cerulion graph run demo`.\n\n"
             "Acme Systems Inc. ships it, and The Acme Systems Inc. is the same name with a word in front.\n")
        for cmd in (["git", "init", "-q"], ["git", "add", "-A"]):
            subprocess.run(cmd, cwd=clean, capture_output=True, check=True)
        buf2 = io.StringIO()
        rc2 = run_tree(clean, buf2)
        arm("clean-tree-exit-0", rc2 == 0 and buf2.getvalue().startswith("check_public_surface: OK ("), buf2.getvalue())
        # The summary has to SAY what the legal-name rule judged. A tail nobody
        # reads is a tail that can report the wrong name, or none.
        arm("the-summary-names-what-the-legal-name-rule-judged",
            "legal-name=`Acme Systems Inc.` in 2 mention(s)" in buf2.getvalue(), buf2.getvalue())
        # And the floor FIRES: a tree that never writes the declared name is not
        # a clean tree, it is a rule that found nothing.
        named = open(os.path.join(clean, "README.md"), encoding="utf-8").read()
        _put(clean, "README.md", "# clean\n\nRun `cerulion graph run demo`.\n")
        buf2c = io.StringIO()
        rc2c = run_tree(clean, buf2c)
        arm("a-tree-that-never-writes-the-name-is-a-finding",
            rc2c == 1 and "is written nowhere in shipped text" in buf2c.getvalue()
            and "in 0 mention(s)" in buf2c.getvalue(), buf2c.getvalue())
        _put(clean, "README.md", named)
        # The text cache is keyed by (root, path) for the life of the process, so a
        # SECOND pass over this tree has to read each file as it is NOW. Each of the
        # three entry points that fills the cache gets an arm: a run, and both
        # regenerators, which would otherwise write a ledger from what a run saw.
        clean_readme = open(os.path.join(clean, "README.md"), encoding="utf-8").read()
        _put(clean, "README.md", clean_readme + "\nAcme Systems, Inc. is the wrong spelling.\n")
        buf2b = io.StringIO()
        rc2b = run_tree(clean, buf2b)
        arm("a-second-run-reads-the-tree-as-it-is-now",
            rc2b == 1 and "Acme Systems, Inc." in buf2b.getvalue(), buf2b.getvalue())
        _put(clean, "README.md", clean_readme + "\nA dash \u2014 here.\n")
        regenerate_ledger(clean, io.StringIO())
        arm("the-dash-regenerator-reads-the-tree-as-it-is-now",
            "1 README.md" in open(os.path.join(clean, LEDGER_FILE), encoding="utf-8").read(),
            open(os.path.join(clean, LEDGER_FILE), encoding="utf-8").read())
        os.remove(os.path.join(clean, LEDGER_FILE))
        clean_cli = open(os.path.join(clean, CLI_FILE), encoding="utf-8").read()
        _put(clean, CLI_FILE, "// Phase 4 wired the seam\n" + clean_cli)
        regenerate_workstate_ledger(clean, io.StringIO())
        arm("the-work-state-regenerator-reads-the-tree-as-it-is-now",
            ("1 plan-step " + CLI_FILE) in open(os.path.join(clean, WORKSTATE_LEDGER_FILE), encoding="utf-8").read(),
            open(os.path.join(clean, WORKSTATE_LEDGER_FILE), encoding="utf-8").read())
        _put(clean, CLI_FILE, clean_cli)
        os.remove(os.path.join(clean, WORKSTATE_LEDGER_FILE))
        _put(clean, "README.md", clean_readme)
        # Fail closed, work-state: a pattern line of another shape stops the run (it is
        # never skipped: a skipped line is a key that silently stopped matching), and so
        # does every other defect of the pattern file, the ledger and the allow list.
        def cannot_run(name, needle, call):
            try:
                call()
                arm(name, False, "ran")
            except CannotRun as exc:
                arm(name, needle in str(exc), str(exc))

        # The legal name is DECLARED, so every defect of the file that declares it
        # is exit 3: a name nobody chose makes every later comparison meaningless,
        # and a name carrying whitespace can never match anything byte for byte.
        entity_body = open(os.path.join(clean, ENTITY_FILE), encoding="utf-8").read()
        for name, body, needle in (
                ("legal-name-file-empty-stops-the-run", "# only a comment\n", "declares no name"),
                ("legal-name-file-two-names-stops-the-run", "Acme Systems Inc.\nOther Holdings Ltd.\n", "declares 2 names"),
                ("legal-name-file-leading-whitespace-stops-the-run", "  Acme Systems Inc.\n", "leading or trailing whitespace"),
                ("legal-name-file-trailing-whitespace-stops-the-run", "Acme Systems Inc.  \n", "leading or trailing whitespace"),
                ("legal-name-that-is-only-a-suffix-stops-the-run", "LLC\n", "a suffix and no name"),
                ("legal-name-with-no-known-suffix-stops-the-run", "Acme Systems Robotics\n", "reads as nothing"),
                ("legal-name-the-rule-reads-as-shorter-stops-the-run", "Acme Inc. of Delaware\n", "reads as `Acme Inc.`"),
                ("legal-name-with-an-invisible-character-stops-the-run", "Acme\u200bSystems Inc.\n", "U+200B, which is outside"),
                ("legal-name-with-a-no-break-space-stops-the-run", "Acme\u00a0Systems Inc.\n", "U+00A0, which is outside"),
                ("legal-name-with-a-combining-mark-stops-the-run", "Acme\u034fSystems Inc.\n", "U+034F, which is outside"),
                ("legal-name-with-a-variation-selector-stops-the-run", "Acme\ufe0fSystems Inc.\n", "U+FE0F, which is outside"),
                ("legal-name-with-a-homoglyph-stops-the-run", "Acme Systems \u0406nc.\n", "U+0406, which is outside")):
            _put(clean, ENTITY_FILE, body)
            cannot_run(name, needle, lambda: run_tree(clean, io.StringIO()))
        # Unreadable is the branch a hardcoded fallback would hide: it is the one
        # defect of this file that does not change the file's TEXT.
        entity_path = os.path.join(clean, ENTITY_FILE)
        # UNREADABLE without a file mode: a mode keeps nobody out when the run is
        # root, which is the ordinary container, and an arm that skips in that
        # case passes without testing anything. Bytes that are not text reach the
        # same branch whoever runs it.
        with open(entity_path, "wb") as fh:
            fh.write(b"\xff\xfe" + "Acme Systems Inc.".encode("utf-16-le"))
        cannot_run("legal-name-file-unreadable-stops-the-run", "cannot read",
                   lambda: run_tree(clean, io.StringIO()))
        os.remove(entity_path)
        cannot_run("legal-name-file-missing-stops-the-run", "it declares the legal name",
                   lambda: run_tree(clean, io.StringIO()))
        _put(clean, ENTITY_FILE, entity_body)
        _put(clean, WORKSTATE_FILE, real_patterns + "session-id-two \\bqt\\d+\\b drop it\n")
        cannot_run("malformed-pattern-line-stops-the-run", "%s:%d: a pattern line is" % (WORKSTATE_FILE, real_patterns.count("\n") + 1),
                   lambda: run_tree(clean, io.StringIO()))
        for name, body, needle in (
                ("pattern-two-fields", "plan-step | \\bchunk\n", "a pattern line is"),
                ("pattern-four-fields", "plan-step | a | b | c\n", "a pattern line is"),
                ("pattern-empty-field", "plan-step |  | c\n", "a pattern line is"),
                ("pattern-bad-key", "Plan Step | a | c\n", "is not lower-case words"),
                ("pattern-regex-does-not-compile", "plan-step | ( | c\n", "does not compile"),
                ("pattern-regex-matches-empty", "plan-step | a* | c\n", "matches the empty string"),
                ("pattern-key-twice", "plan-step | a | c\nplan-step | b | c\n", "appears twice"),
                ("pattern-file-empty", "# only a comment\n", "holds no pattern")):
            _put(clean, WORKSTATE_FILE, body)
            cannot_run(name, needle, lambda: load_workstate_patterns(clean))
        # A MISSING pattern file cannot run: exit 3 through main(), never a pass.
        os.remove(os.path.join(clean, WORKSTATE_FILE))
        cannot_run("missing-pattern-file-cannot-run", WORKSTATE_FILE + " is missing", lambda: run_tree(clean, io.StringIO()))
        import contextlib

        buf_main = io.StringIO()
        with contextlib.redirect_stdout(buf_main):
            rc_main = main(["--root", clean])
        arm("missing-pattern-file-exit-3", rc_main == 3 and buf_main.getvalue().startswith("check_public_surface: CANNOT RUN: "), "rc=%d %s" % (rc_main, buf_main.getvalue()))
        _put(clean, WORKSTATE_FILE, real_patterns)
        for name, body, needle in (
                ("ledger-line-shape", "many plan-step a.rs\n", "a ledger line is"),
                ("ledger-zero-count", "0 plan-step a.rs\n", "a ledger line is"),
                ("ledger-unknown-key", "1 plan-stair a.rs\n", "is not a key of"),
                ("ledger-listed-twice", "1 plan-step a.rs\n2 plan-step a.rs\n", "is listed twice")):
            _put(clean, WORKSTATE_LEDGER_FILE, body)
            cannot_run(name, needle, lambda: load_workstate_ledger(clean, set(ws_compiled)))
        os.remove(os.path.join(clean, WORKSTATE_LEDGER_FILE))
        _put(clean, ALLOW_FILE, "README.md | work-state | * | a whole-file waiver is refused for this class\n")
        cannot_run("work-state-allow-star-refused", "names the one finding", lambda: load_allow(clean))
        os.remove(os.path.join(clean, ALLOW_FILE))
        # Fail closed: a tree without the CLI definition cannot run (exit 3), never passes.
        os.remove(os.path.join(clean, CLI_FILE))
        subprocess.run(["git", "add", "-A"], cwd=clean, capture_output=True, check=True)
        cannot_run("no-cli-fails-closed", CLI_FILE, lambda: run_tree(clean, io.StringIO()))
        # The ledger regenerator writes exactly the counts the gate reads back as clean.
        regenerate_ledger(root, io.StringIO())
        buf3 = io.StringIO()
        run_tree(root, buf3)
        arm("regenerated-ledger-clears-dash-findings", not any(re.match(r"^\S+:\d+: shipped-text: .*dash", l) for l in buf3.getvalue().split("\n")), buf3.getvalue())
        # The pool and the single process read the same hits (same function, same files).
        scanned = [f for f in tracked_files(root) if not WORKSTATE_EXCLUDE_RE.search(f)]
        saved_jobs = os.environ.get(JOBS_ENV)
        try:
            os.environ[JOBS_ENV] = "1"
            serial, serial_mode = workstate_scan(root, scanned, ws_patterns)
            os.environ[JOBS_ENV] = "2"
            pooled, pooled_mode = workstate_scan(root, scanned, ws_patterns)
            os.environ[JOBS_ENV] = "lots"
            cannot_run("jobs-env-is-validated", JOBS_ENV, lambda: workstate_scan(root, scanned, ws_patterns))
        finally:
            if saved_jobs is None:
                os.environ.pop(JOBS_ENV, None)
            else:
                os.environ[JOBS_ENV] = saved_jobs
        arm("serial-scan-found-the-fixture", serial_mode == "one process" and sum(len(v) for v in serial.values()) >= len(WORKSTATE_CAUGHT), serial_mode)
        arm("pool-equals-serial", pooled == serial, "%s: %r" % (pooled_mode, pooled))
        # The work-state regenerator writes the counts the gate reads back as clean, never
        # lists a user-facing page (it keeps failing), and never counts an allowed line.
        regenerate_workstate_ledger(root, io.StringIO())
        regenerated = open(os.path.join(root, WORKSTATE_LEDGER_FILE), encoding="utf-8").read()
        buf4 = io.StringIO()
        run_tree(root, buf4)
        ws_after = [l for l in buf4.getvalue().split("\n") if ": work-state: " in l and not l.startswith(("note:", "remedy:"))]
        # What a regenerated ledger CANNOT silence: the user-facing page, and the
        # key that may never be ledgered at all.
        never_ledgered_rows = sum(1 for key, _l, _m in WORKSTATE_CAUGHT if key in WORKSTATE_NEVER_LEDGERED)
        arm("regenerated-work-state-ledger-leaves-the-user-facing-page-and-the-never-ledgered-key",
            len(ws_after) == never_ledgered_rows + 1 and any(l.startswith("docs/ws_page.md:3: ") for l in ws_after)
            and any("`approval-voice` wording" in l for l in ws_after), "\n".join(ws_after))
        arm("regenerated-work-state-ledger-never-writes-a-never-ledgered-key",
            not any(" %s " % key in regenerated for key in WORKSTATE_NEVER_LEDGERED), regenerated)
        arm("regenerated-work-state-ledger-rows",
            "\n2 plan-step crates/ws_ledger/src/at.rs\n" in regenerated and "\n2 plan-step crates/ws_ledger/src/above.rs\n" in regenerated
            and "\n1 plan-step crates/ws_ledger/src/below.rs\n" in regenerated
            and "\n%d plan-step crates/ws/src/caught.rs\n" % sum(1 for key, _l, _m in WORKSTATE_CAUGHT if key == "plan-step") in regenerated
            and "docs/" not in regenerated.split("#")[-1] and "ws_gone" not in regenerated, regenerated)
    out.write("check_public_surface --self-test: OK (%d arms: %d expected findings, the negative controls, the verb-tree oracle, the lexer, the fail-closed and ledger arms, "
              "and for work-state every key caught and their neighbours silent, the ledger at, above and below its count, the user-facing page nobody may ledger, "
              "the malformed and the missing pattern file, the allow list before the count, and the pool against one process [%s])\n" % (arms, len(EXPECTED), pooled_mode))
    return 0


def main(argv):
    mode = "tree"
    root = None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--self-test":
            mode = "self-test"
        elif a == "--regenerate-dash-ledger":
            mode = "regenerate"
        elif a == "--regenerate-workstate-ledger":
            mode = "regenerate-workstate"
        elif a == "--root":
            if i + 1 >= len(argv):
                sys.stderr.write("check_public_surface: --root needs a directory\n")
                return 2
            root = argv[i + 1]
            i += 1
        else:
            sys.stderr.write("check_public_surface: unknown argument %s\n" % a)
            return 2
        i += 1
    if root is None:
        root = os.getcwd()
    try:
        if mode == "self-test":
            return self_test()
        if mode == "regenerate":
            return regenerate_ledger(root)
        if mode == "regenerate-workstate":
            return regenerate_workstate_ledger(root)
        return run_tree(root)
    except CannotRun as exc:
        sys.stdout.write("check_public_surface: CANNOT RUN: %s\n" % exc)
        return 3


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
