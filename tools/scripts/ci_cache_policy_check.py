#!/usr/bin/env python3
"""ci_cache_policy_check.py — hold the cache-save policy in the workflows.

    python3 tools/scripts/ci_cache_policy_check.py .github/workflows
    python3 tools/scripts/ci_cache_policy_check.py .github/workflows/ci.yml ...
    python3 tools/scripts/ci_cache_policy_check.py --self-test

WHY THIS EXISTS. The repository's GitHub Actions cache store has a 10 GB free
allowance, and above it saves are refused while the account carries a failed
payment. The workflows now gate every `actions/cache/save` step behind a policy
-- a list of namespaces that may save (`CACHE_SAVE_NAMESPACES`, default
`macOS Linux`), main-only unless `CACHE_SAVE_ON_PULL_REQUEST` lifts it -- and a
prune step runs directly before each save of a lockfile-keyed archive so the
namespace holds exactly the key the job is about to write. Every one of those
conditions lives in an `if:` expression, which no runner evaluates until the job
is already running and which nothing else in the tree reads. This file reads
them.

The failure this guards against is not a wrong expression, it is a MISSING one:
a new job copied from an old one, a save step whose gate was dropped during a
rebase, or a namespace token that no longer matches the key it gates. All three
are silent -- the workflow is still valid YAML, the job still runs, and the only
symptom is the store filling up again a week later.

IT READS THE WHOLE DIRECTORY. A path that names a directory is walked for
`*.yml`/`*.yaml`, and the Lint step passes `.github/workflows`. Naming the files
by hand is what let two workflows write into the same store with the combined
`actions/cache@` action, ungated and unpruned, while this checker reported the
two files it had been told about as clean. Every file is read; the policy rules
themselves apply to the files that bear the policy (a save step, or the policy
`env` block), and the structural rules apply everywhere.

Stdlib only: PyYAML is not installed on the runners, so the reader below is a
purpose-built one for the YAML subset these workflows use. It refuses anything
outside that subset with a parse error naming the line rather than skipping it,
because a reader that silently ignores what it cannot understand reports a
green file it never read. A job that calls a reusable workflow (`uses:` at job
level), or that carries no `steps:`, is refused for the same reason: the reader
would otherwise contribute zero steps for it and every rule would pass on a job
nobody read.

Exit 0 every rule holds, 1 one line per violation on stdout, 2 usage or a file
it cannot parse.

`--self-test` builds workflows in memory, asserts they pass, then applies one
mutation per rule and asserts each mutant reports exactly that rule. The CI step
that runs it is the only thing that proves this checker is not inert.
"""

import os
import re
import sys

# ---------------------------------------------------------------------------
# YAML subset reader
# ---------------------------------------------------------------------------


class ParseError(Exception):
    def __init__(self, line, msg):
        Exception.__init__(self, "line %d: %s" % (line, msg))
        self.line = line
        self.msg = msg


class Scalar(object):
    __slots__ = ("value", "line")

    def __init__(self, value, line):
        self.value = value
        self.line = line


class Seq(object):
    __slots__ = ("items", "line")

    def __init__(self, line):
        self.items = []
        self.line = line


class Map(object):
    __slots__ = ("line", "_order", "_vals", "_lines")

    def __init__(self, line):
        self.line = line
        self._order = []
        self._vals = {}
        self._lines = {}

    def add(self, key, node, line):
        if key in self._vals:
            raise ParseError(line, "duplicate key %r" % key)
        self._order.append(key)
        self._vals[key] = node
        self._lines[key] = line

    def keys(self):
        return list(self._order)

    def has(self, key):
        return key in self._vals

    def get(self, key):
        return self._vals.get(key)

    def line_of(self, key):
        return self._lines.get(key, self.line)


KEY_LINE_RE = re.compile(
    r"^(?P<ind> *)(?P<key>'[^']*'|\"[^\"]*\"|[A-Za-z0-9_][A-Za-z0-9_.\-]*):(?P<rest>.*)$"
)
BLOCK_HEAD_RE = re.compile(r"^(?P<style>[|>])(?P<chomp>[+-]?)(?:\s+#.*)?$")


def _is_key_line(text):
    mo = KEY_LINE_RE.match(text)
    if not mo:
        return False
    rest = mo.group("rest")
    return rest == "" or rest.startswith(" ")


def _strip_plain_comment(text):
    """Cut a plain scalar at the first ` #` that is not inside a quoted run.

    `uses: actions/cache/save@<sha> # v4.3.0` must not keep the comment, and
    `${{ hashFiles('**/Cargo.lock') }}` must not be cut at a `#` that does not
    exist -- the quote tracking is what keeps the two apart.
    """
    quote = None
    for i, ch in enumerate(text):
        if quote is not None:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == "#" and i > 0 and text[i - 1] in " \t":
            return text[:i].rstrip()
    return text.rstrip()


def _read_quoted(text, line):
    """Return (value, remainder) for a single- or double-quoted scalar."""
    quote = text[0]
    out = []
    i = 1
    while i < len(text):
        ch = text[i]
        if quote == "'":
            if ch == "'":
                if i + 1 < len(text) and text[i + 1] == "'":
                    out.append("'")
                    i += 2
                    continue
                return "".join(out), text[i + 1:]
            out.append(ch)
            i += 1
            continue
        if ch == "\\":
            if i + 1 >= len(text):
                raise ParseError(line, "dangling escape in a double-quoted scalar")
            nxt = text[i + 1]
            out.append({"n": "\n", "t": "\t", '"': '"', "\\": "\\"}.get(nxt, nxt))
            i += 2
            continue
        if ch == '"':
            return "".join(out), text[i + 1:]
        out.append(ch)
        i += 1
    raise ParseError(line, "unterminated quoted scalar")


def _scalar_value(text, line):
    text = text.strip()
    if text[:1] in ("'", '"'):
        value, rest = _read_quoted(text, line)
        rest = rest.strip()
        if rest and not rest.startswith("#"):
            raise ParseError(line, "trailing content after a quoted scalar: %r" % rest)
        return value
    return _strip_plain_comment(text)


def _unquote_key(raw, line):
    if raw[:1] in ("'", '"'):
        return _read_quoted(raw, line)[0]
    return raw


class Parser(object):
    """An indentation reader for the workflow subset, with line numbers.

    Understands: mappings, block sequences, plain/single/double-quoted scalars,
    the `|`, `|-`, `>` and `>-` block scalars, simple inline lists (`[main]`),
    and comment or blank lines anywhere outside a block scalar. Everything else
    -- anchors, aliases, tags, flow mappings, tab indentation, `+` chomping --
    is a parse error naming its line.
    """

    def __init__(self, text):
        self.lines = text.split("\n")
        self.n = len(self.lines)
        for i, raw in enumerate(self.lines):
            lead = raw[: len(raw) - len(raw.lstrip(" \t"))]
            if "\t" in lead:
                raise ParseError(i + 1, "tab in indentation")
            if raw.strip() in ("---", "..."):
                raise ParseError(i + 1, "multi-document YAML is outside this reader's subset")

    def parse(self):
        i = self._next_content(0)
        if i >= self.n:
            raise ParseError(1, "the file is empty")
        if self._indent(i) != 0:
            raise ParseError(i + 1, "the document must start at column 0")
        node, i = self._parse_map(i, 0)
        i = self._next_content(i)
        if i < self.n:
            raise ParseError(i + 1, "trailing content after the document mapping")
        return node

    # -- line helpers -------------------------------------------------------

    def _indent(self, i):
        raw = self.lines[i]
        return len(raw) - len(raw.lstrip(" "))

    def _skippable(self, i):
        stripped = self.lines[i].strip()
        return stripped == "" or stripped.startswith("#")

    def _next_content(self, i):
        while i < self.n and self._skippable(i):
            i += 1
        return i

    def _is_seq_line(self, i, indent):
        body = self.lines[i][indent:]
        return body.startswith("-") and (len(body) == 1 or body[1] == " ")

    # -- structure ----------------------------------------------------------

    def _parse_block(self, i, indent):
        if self._is_seq_line(i, indent):
            return self._parse_seq(i, indent)
        return self._parse_map(i, indent)

    def _parse_map(self, i, indent):
        node = Map(i + 1)
        while True:
            i = self._next_content(i)
            if i >= self.n:
                break
            ind = self._indent(i)
            if ind < indent:
                break
            if ind > indent:
                raise ParseError(
                    i + 1, "unexpected indentation (%d spaces, expected %d)" % (ind, indent)
                )
            if self._is_seq_line(i, indent):
                break
            raw = self.lines[i]
            mo = KEY_LINE_RE.match(raw)
            if not mo or len(mo.group("ind")) != indent:
                raise ParseError(i + 1, "expected a mapping key, found: %r" % raw.strip())
            rest = mo.group("rest")
            if rest and not rest.startswith(" "):
                raise ParseError(i + 1, "a mapping key needs a space after its colon")
            key = _unquote_key(mo.group("key"), i + 1)
            if key == "<<":
                raise ParseError(i + 1, "YAML merge keys are outside this reader's subset")
            key_line = i + 1
            value, i = self._parse_value(i, indent, rest)
            node.add(key, value, key_line)
        return node, i

    def _parse_seq(self, i, indent):
        node = Seq(i + 1)
        while True:
            i = self._next_content(i)
            if i >= self.n:
                break
            ind = self._indent(i)
            if ind < indent:
                break
            if ind > indent:
                raise ParseError(
                    i + 1, "unexpected indentation (%d spaces, expected %d)" % (ind, indent)
                )
            if not self._is_seq_line(i, indent):
                break
            rest = self.lines[i][indent + 1:]
            stripped = rest.strip()
            if stripped == "" or stripped.startswith("#"):
                j = self._next_content(i + 1)
                if j < self.n and self._indent(j) > indent:
                    item, i = self._parse_block(j, self._indent(j))
                    node.items.append(item)
                    continue
                node.items.append(Scalar(None, i + 1))
                i += 1
                continue
            pad = len(rest) - len(rest.lstrip(" "))
            inner = indent + 1 + pad
            probe = " " * inner + rest.lstrip(" ")
            if _is_key_line(probe):
                # Rewrite `- key: v` to `  key: v` in place so the item parses as
                # an ordinary mapping. The line number is untouched.
                self.lines[i] = probe
                item, i = self._parse_map(i, inner)
                node.items.append(item)
                continue
            node.items.append(Scalar(_scalar_value(stripped, i + 1), i + 1))
            i += 1
        return node, i

    def _parse_value(self, i, indent, rest):
        line = i + 1
        text = rest.strip()
        if text.startswith("#"):
            text = ""
        if text == "":
            j = self._next_content(i + 1)
            if j < self.n and self._indent(j) > indent:
                return self._parse_block(j, self._indent(j))
            if j < self.n and self._indent(j) == indent and self._is_seq_line(j, indent):
                return self._parse_seq(j, indent)
            return Scalar(None, line), i + 1
        mo = BLOCK_HEAD_RE.match(text)
        if mo:
            if mo.group("chomp") == "+":
                raise ParseError(line, "`+` block chomping is outside this reader's subset")
            return self._read_block_scalar(i, indent, mo.group("style"), mo.group("chomp"))
        if text[0] in "&*":
            raise ParseError(line, "YAML anchors and aliases are outside this reader's subset")
        if text[0] == "{":
            # `permissions: {}` and `workflow_dispatch: {}` are the only flow
            # mappings the workflows use, and an EMPTY one has an unambiguous
            # meaning the rules below need (a job or workflow that grants no
            # permission at all). A non-empty one is still refused.
            head = text.split("#", 1)[0].strip()
            if head == "{}":
                return Map(line), i + 1
            raise ParseError(line, "a non-empty YAML flow mapping is outside this reader's subset")
        if text[0] == "!":
            raise ParseError(line, "YAML tags are outside this reader's subset")
        if text[0] == "[":
            return self._parse_flow_seq(text, line), i + 1
        return Scalar(_scalar_value(text, line), line), i + 1

    def _parse_flow_seq(self, text, line):
        depth = 0
        quote = None
        end = None
        for i, ch in enumerate(text):
            if quote is not None:
                if ch == quote:
                    quote = None
                continue
            if ch in "'\"":
                quote = ch
            elif ch == "[":
                depth += 1
            elif ch == "]":
                depth -= 1
                if depth == 0:
                    end = i
                    break
            elif ch == "{":
                raise ParseError(line, "a flow mapping inside a list is outside this reader's subset")
        if end is None:
            raise ParseError(line, "unterminated inline list")
        tail = text[end + 1:].strip()
        if tail and not tail.startswith("#"):
            raise ParseError(line, "trailing content after an inline list: %r" % tail)
        inner = text[1:end]
        node = Seq(line)
        if inner.strip() == "":
            return node
        part = []
        quote = None
        parts = []
        for ch in inner:
            if quote is not None:
                part.append(ch)
                if ch == quote:
                    quote = None
                continue
            if ch in "'\"":
                quote = ch
                part.append(ch)
            elif ch == ",":
                parts.append("".join(part))
                part = []
            else:
                if ch == "[":
                    raise ParseError(line, "nested inline lists are outside this reader's subset")
                part.append(ch)
        parts.append("".join(part))
        for raw in parts:
            node.items.append(Scalar(_scalar_value(raw, line), line))
        return node

    def _read_block_scalar(self, i, indent, style, chomp):
        line = i + 1
        body = []
        block_indent = None
        j = i + 1
        while j < self.n:
            raw = self.lines[j]
            if raw.strip() == "":
                body.append("")
                j += 1
                continue
            ind = self._indent(j)
            if ind <= indent:
                break
            if block_indent is None:
                block_indent = ind
            elif ind < block_indent:
                break
            body.append(raw[block_indent:])
            j += 1
        while body and body[-1] == "":
            body.pop()
        if style == "|":
            text = "\n".join(body)
        else:
            # Folding, with the more-indented rule. A line indented further
            # than the block keeps its own line break on both sides (YAML
            # folds only the lines at the block's own indentation), which is
            # how a hand-wrapped `if:` expression reads. Every rule here
            # compares whitespace-normalised text, so the distinction changes
            # no verdict -- it is here so the reader reports what the file
            # says rather than refusing a shape GitHub accepts.
            chunks = []
            prev_more = False
            for entry in body:
                if entry == "":
                    chunks.append("\n")
                    prev_more = False
                    continue
                more = entry[:1] == " "
                if chunks:
                    if more or prev_more:
                        if not chunks[-1].endswith("\n"):
                            chunks.append("\n")
                    elif not chunks[-1].endswith("\n"):
                        chunks.append(" ")
                chunks.append(entry)
                prev_more = more
            text = "".join(chunks)
        if chomp != "-" and text:
            text += "\n"
        return Scalar(text, line), j


# ---------------------------------------------------------------------------
# workflow model
# ---------------------------------------------------------------------------


def _scalar_of(node, line, what):
    if node is None:
        return None
    if not isinstance(node, Scalar):
        raise ParseError(line, "%s: expected a scalar" % what)
    return node.value


def _get_str(mapping, key, what):
    if not mapping.has(key):
        return None
    return _scalar_of(mapping.get(key), mapping.line_of(key), "%s: %s" % (what, key))


def _get_map(mapping, key, what):
    if not mapping.has(key):
        return None
    node = mapping.get(key)
    if isinstance(node, Scalar) and node.value is None:
        return Map(mapping.line_of(key))
    if not isinstance(node, Map):
        raise ParseError(mapping.line_of(key), "%s: expected a mapping for %r" % (what, key))
    return node


class Step(object):
    def __init__(self, node, job_id, index):
        if not isinstance(node, Map):
            raise ParseError(node.line, "a step must be a mapping")
        what = "job %s step %d" % (job_id, index + 1)
        self.node = node
        self.line = node.line
        self.index = index
        self.name = _get_str(node, "name", what)
        self.step_id = _get_str(node, "id", what)
        self.cond = _get_str(node, "if", what)
        self.uses = _get_str(node, "uses", what)
        self.run = _get_str(node, "run", what)
        self.with_ = _get_map(node, "with", what)
        self.env = _get_map(node, "env", what)
        self.env_line = node.line_of("env") if node.has("env") else node.line
        self.cond_line = node.line_of("if") if node.has("if") else node.line

    @property
    def is_save(self):
        return bool(self.uses) and self.uses.startswith(SAVE_USES_PREFIX)

    @property
    def is_combined_cache(self):
        return bool(self.uses) and self.uses.startswith(COMBINED_USES_PREFIX)

    @property
    def is_prune(self):
        return bool(self.run) and self.run.strip().startswith(PRUNE_CMD)

    @property
    def key(self):
        if self.with_ is None:
            return None
        return _get_str(self.with_, "key", "step at line %d" % self.line)

    @property
    def path(self):
        if self.with_ is None:
            return None
        return _get_str(self.with_, "path", "step at line %d" % self.line)

    def label(self):
        return self.name or (self.uses or "step") or "step"


class Job(object):
    def __init__(self, job_id, node):
        if not isinstance(node, Map):
            raise ParseError(node.line, "job %s must be a mapping" % job_id)
        self.job_id = job_id
        self.node = node
        self.line = node.line
        # A job this reader cannot enumerate is REFUSED, not skipped. Before
        # this rule a reusable-workflow call contributed zero steps and every
        # rule below passed on it in silence.
        if node.has("uses"):
            raise ParseError(
                node.line_of("uses"),
                "job %s calls a reusable workflow (`uses:` at job level); this reader cannot see "
                "the steps it runs, so a cache save inside it would be invisible to every rule "
                "here" % job_id,
            )
        self.permissions = _get_map(node, "permissions", "job %s" % job_id)
        self.permissions_line = node.line_of("permissions") if node.has("permissions") else node.line
        self.cond = _get_str(node, "if", "job %s" % job_id)
        self.cond_line = node.line_of("if") if node.has("if") else node.line
        self.env = _get_map(node, "env", "job %s" % job_id)
        self.env_line = node.line_of("env") if node.has("env") else node.line
        self.container = node.has("container")
        self.steps = []
        steps_node = node.get("steps")
        if steps_node is None or (isinstance(steps_node, Scalar) and steps_node.value is None):
            raise ParseError(
                node.line_of("steps") if node.has("steps") else node.line,
                "job %s has no `steps`; a job whose steps this reader cannot enumerate is refused "
                "rather than silently contributing none" % job_id,
            )
        if not isinstance(steps_node, Seq):
            raise ParseError(node.line_of("steps"), "job %s: `steps` must be a list" % job_id)
        for index, item in enumerate(steps_node.items):
            self.steps.append(Step(item, job_id, index))


# ---------------------------------------------------------------------------
# policy vocabulary
# ---------------------------------------------------------------------------

SAVE_USES_PREFIX = "actions/cache/save@"
COMBINED_USES_PREFIX = "actions/cache@"
PRUNE_CMD = "bash tools/scripts/ci_cache_prune.sh"
PRUNE_RUN_RE = re.compile(
    r'^bash tools/scripts/ci_cache_prune\.sh "(?P<key>[^"]*)"(?: "(?P<prefix>[^"]*)")?$'
)
PRUNE_TOOLS_STEP_NAME = "Install the cache prune tools (gh, jq)"
# The fork half is load-bearing: a pull request from a fork gets a read-only
# token, so the prune's delete would fail the job outright.
MAIN_ONLY_CLAUSE = (
    "(github.ref == 'refs/heads/main' || (env.CACHE_SAVE_ON_PULL_REQUEST != '' "
    "&& github.event.pull_request.head.repo.full_name == github.repository))"
)
MERGE_GROUP_CLAUSE = "github.event_name != 'merge_group'"
SHARD_CLAUSE = "matrix.shard == 0"
NS_ALL_CLAUSE = "env.CACHE_SAVE_NAMESPACES == 'all'"
NS_CONTAINS_HEAD = "contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), "
CACHE_HIT_RE = re.compile(r"^steps\.[A-Za-z0-9_.\-]+\.outputs\.cache-hit != 'true'$")
CACHE_HIT_MARKER = ".outputs.cache-hit != 'true' && "
CACHE_HIT_PREFIX_RE = re.compile(r"^steps\.[A-Za-z0-9_.\-]+\.outputs\.cache-hit != 'true' && $")
# A job-shape guard such as `steps.present.outputs.present == 'true'`. It can
# only ever NARROW the gate (every clause is a conjunct), which is why it is in
# the closed set; anything else in an `if:` here is refused.
STEP_OUTPUT_GUARD_RE = re.compile(
    r"^steps\.[A-Za-z0-9_.\-]+\.outputs\.[A-Za-z0-9_.\-]+ == '[A-Za-z0-9_.\-]+'$"
)
TOOL_CACHE_PATH_RE = re.compile(r"^~/\.cargo/bin/[A-Za-z0-9_.\-]+$")
EVENT_NE_PULL_REQUEST = "github.event_name != 'pull_request'"
EVENT_EQ_RE = re.compile(r"^github\.event_name == '([A-Za-z0-9_]+)'$")
SCOPE_SUFFIX = "-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}"
HASHFILES_RE = re.compile(r"-\$\{\{\s*hashFiles\(")
STEPS_OUTPUT_SEG_RE = re.compile(r"-\$\{\{\s*steps\.[A-Za-z0-9_.\-]+\.outputs\.[A-Za-z0-9_.\-]+\s*\}\}")
STEPS_OUTPUT_EXPR_RE = re.compile(r"\$\{\{\s*steps\.[A-Za-z0-9_.\-]+\.outputs\.[A-Za-z0-9_.\-]+\s*\}\}")
EXPR_RE = re.compile(r"\$\{\{(.*?)\}\}", re.S)
POLICY_ENV = {
    "CACHE_SAVE_NAMESPACES": "${{ vars.CACHE_SAVE_NAMESPACES || 'macOS Linux' }}",
    "CACHE_SAVE_ON_PULL_REQUEST": "${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}",
}
POLICY_ENV_NAMES = tuple(sorted(POLICY_ENV))


def norm(text):
    """Collapse every whitespace run to a single space, as the rules compare."""
    return " ".join((text or "").split())


def is_lockhash_key(key):
    return bool(key) and "${{ hashFiles(" in key.replace("${{hashFiles(", "${{ hashFiles(")


def namespace_templates(key):
    """The namespace template(s) of a lockhash key, widest match last.

    The key is the template plus an optional `-<scope>` and the lockfile hash;
    the leading `cargo-` is shared by every key and carries no namespace. The
    rmw-distros keys additionally carry a `-${{ steps.<id>.outputs.<name> }}`
    header-tree hash which is a cache generation, not a namespace, so a variant
    with it dropped is offered too.
    """
    hits = list(HASHFILES_RE.finditer(key))
    if not hits:
        return []
    head = key[: hits[-1].start()]
    cut = head.rfind("-${{")
    if cut >= 0 and norm(head[cut:]) == SCOPE_SUFFIX:
        head = head[:cut]
    if head.startswith("cargo-"):
        head = head[len("cargo-"):]
    variants = [head]
    dropped = STEPS_OUTPUT_SEG_RE.sub("", head)
    if dropped != head:
        variants.append(dropped)
    return variants


def token_fragment(template):
    """The `if:` text that tests one namespace token against the policy list."""
    args = []

    def replace(mo):
        args.append(norm(mo.group(1)))
        return "{%d}" % (len(args) - 1)

    body = EXPR_RE.sub(replace, template)
    if not args:
        return "' %s '" % body
    return "format(' %s ', %s)" % (body, ", ".join(args))


def namespace_clause(fragment):
    return "(%s || %s%s))" % (NS_ALL_CLAUSE, NS_CONTAINS_HEAD, fragment)


def namespace_clauses(key):
    return [namespace_clause(token_fragment(t)) for t in namespace_templates(key)]


def _split_top(text, op, other):
    """Split `text` at every depth-0 `op`, reporting whether `other` appears there.

    Returns `(parts, saw_other)`. For `&&`, `saw_other` is the whole point:
    `... || true` appended to a gate keeps every required substring and every
    equality the other rules test, and turns the gate off. A parenthesised
    alternative (the main-only clause, the namespace gate) is inside one
    conjunct and is not a depth-0 `||`.
    """
    parts = []
    current = []
    depth = 0
    quote = None
    saw_other = False
    index = 0
    while index < len(text):
        ch = text[index]
        if quote is not None:
            current.append(ch)
            if ch == quote:
                quote = None
            index += 1
            continue
        if ch in "'\"":
            quote = ch
            current.append(ch)
            index += 1
            continue
        if ch == "(":
            depth += 1
            current.append(ch)
            index += 1
            continue
        if ch == ")":
            depth -= 1
            current.append(ch)
            index += 1
            continue
        if depth == 0 and text[index:index + 2] == op:
            parts.append("".join(current).strip())
            current = []
            index += 2
            continue
        if depth == 0 and text[index:index + 2] == other:
            saw_other = True
        current.append(ch)
        index += 1
    parts.append("".join(current).strip())
    return [part for part in parts if part], saw_other


def split_conjuncts(text):
    return _split_top(text, "&&", "||")


def split_disjuncts(text):
    return _split_top(text, "||", "&&")


def _strip_outer_parens(text):
    """`(a || b)` -> `a || b`, only when the first `(` closes on the last char."""
    while text.startswith("(") and text.endswith(")"):
        depth = 0
        for index, ch in enumerate(text):
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
        if index != len(text) - 1:
            return text
        text = text[1:-1].strip()
    return text


def is_unscoped_key(key):
    """True when the key carries no `-<main|pr>-` scope segment before its hash.

    Derived the same way `namespace_templates` finds the scope, so the two can
    never disagree about which keys are scoped.
    """
    hits = list(HASHFILES_RE.finditer(key))
    if not hits:
        return False
    head = key[: hits[-1].start()]
    cut = head.rfind("-${{")
    return not (cut >= 0 and norm(head[cut:]) == SCOPE_SUFFIX)


def job_excludes_pull_request(cond):
    """True when a job-level `if:` can never be true on a pull-request run.

    Two shapes, both in ci.yml today: a depth-0 conjunct
    `github.event_name != 'pull_request'` (fuzz, miri, msrv, cross-aarch64), and
    an allowlist -- a disjunction of `github.event_name == '<event>'` terms
    naming no pull request (the two latency jobs). The allowlist is accepted
    inside any depth-0 conjunct, not only as the whole expression: a conjunct
    can only narrow the job, so one that excludes pull requests excludes them
    for the job.
    """
    cond = norm(cond)
    if not cond:
        return False
    conjuncts, _ = split_conjuncts(cond)
    for part in conjuncts:
        if part == EVENT_NE_PULL_REQUEST:
            return True
        terms, saw_and = split_disjuncts(_strip_outer_parens(part))
        if saw_and or not terms:
            continue
        events = []
        for term in terms:
            mo = EVENT_EQ_RE.match(_strip_outer_parens(term))
            if mo is None:
                events = None
                break
            events.append(mo.group(1))
        if events and "pull_request" not in events:
            return True
    return False


def classify_conjunct(part, ns_clauses):
    if CACHE_HIT_RE.match(part):
        return "cache-hit"
    if part == SHARD_CLAUSE:
        return "shard"
    if part == MERGE_GROUP_CLAUSE:
        return "merge-group"
    if part == MAIN_ONLY_CLAUSE:
        return "main-only"
    if part in ns_clauses:
        return "namespace"
    if STEP_OUTPUT_GUARD_RE.match(part):
        return "step-output guard"
    return None


def _check_closed_conjunction(add, line, what, cond, allowed, ns_clauses):
    """R8. The gate is a conjunction of clauses from a CLOSED set, each once."""
    conjuncts, saw_or = split_conjuncts(cond)
    if saw_or:
        add(
            line,
            "R8_CLOSED_CONJUNCTION",
            "%s carries a top-level `||`; this gate must be a conjunction only, because one "
            "`|| true` turns it off while every other rule here still passes" % what,
        )
    seen = []
    for part in conjuncts:
        name = classify_conjunct(part, ns_clauses)
        if name is None or name not in allowed:
            add(
                line,
                "R8_CLOSED_CONJUNCTION",
                "%s carries the conjunct %r, which is not one of the clauses this gate may "
                "hold (%s)" % (what, part, ", ".join(allowed)),
            )
            continue
        if name in seen:
            add(
                line,
                "R8_CLOSED_CONJUNCTION",
                "%s carries the %s clause twice" % (what, name),
            )
        seen.append(name)
    return seen


# ---------------------------------------------------------------------------
# the rules
# ---------------------------------------------------------------------------

# The gates, as conjunctions:
#
#   prune  = [matrix.shard == 0 &&] [a step-output guard &&]
#            github.event_name != 'merge_group' && <main-only>
#   save   = steps.<restore id>.outputs.cache-hit != 'true' && <the prune's if>
#            && <namespace gate>
#
# The prune deliberately does NOT carry the namespace gate. When it did, a
# namespace the policy does not name was never pruned and its stale archives
# stayed for ever (a pull request's restore refreshes an entry's last-access
# time, so GitHub's idle eviction never fires on one). Now every default-branch
# run of a job that has a prune step prunes, keeping exactly that job's current
# key -- which empties a restore-only namespace, and that is the intent.
PRUNE_ALLOWED = ("shard", "step-output guard", "merge-group", "main-only")
SAVE_ALLOWED = ("cache-hit",) + PRUNE_ALLOWED + ("namespace",)
TOOL_CACHE_ALLOWED = ("cache-hit", "merge-group")


def check_text(path, text):
    """Return (violations, stats). Raises ParseError for a file outside the subset."""
    doc = Parser(text).parse()
    problems = []

    def add(line, code, msg):
        problems.append("%s:%d: %s: %s" % (path, line, code, msg))

    jobs_node = doc.get("jobs")
    if not isinstance(jobs_node, Map):
        raise ParseError(doc.line_of("jobs") if doc.has("jobs") else 1, "no `jobs` mapping")
    jobs = [Job(job_id, jobs_node.get(job_id)) for job_id in jobs_node.keys()]

    save_count = sum(1 for job in jobs for step in job.steps if step.is_save)
    prune_count = sum(1 for job in jobs for step in job.steps if step.is_prune)

    # ---- rules that apply to EVERY workflow in the directory ---------------
    for job in jobs:
        _check_local_policy_env(add, job)
        for step in job.steps:
            if step.is_combined_cache:
                add(
                    step.line,
                    "NO_COMBINED_CACHE_ACTION",
                    "step %r uses the combined `actions/cache` action, which saves at the end of "
                    "the job with no gate and no prune; split it into `actions/cache/restore` and "
                    "a gated `actions/cache/save`" % step.label(),
                )

    wf_env = _get_map(doc, "env", "workflow")
    bears_policy = save_count > 0 or (
        wf_env is not None and any(wf_env.has(name) for name in POLICY_ENV_NAMES)
    )
    stats = {"saves": save_count, "prunes": prune_count, "policy": 1 if bears_policy else 0}
    if not bears_policy:
        # A workflow that neither saves nor declares the policy has nothing to
        # hold: restore-only caching costs the store nothing.
        return problems, stats

    # R4, workflow half: the file itself may not hand `actions: write` to jobs
    # that do not prune. A job that prunes opts in on its own line.
    wf_perm = _get_map(doc, "permissions", "workflow")
    if wf_perm is None:
        add(1, "R4_PERMISSIONS", "no workflow-level `permissions`; expected exactly {contents: read}")
    else:
        got = _permission_pairs(wf_perm, "workflow")
        if got != {"contents": "read"}:
            add(
                doc.line_of("permissions"),
                "R4_PERMISSIONS",
                "workflow-level permissions are %s; expected exactly {contents: read}" % _fmt(got),
            )

    # R5: the two policy variables and their defaults.
    if wf_env is None:
        add(1, "R5_POLICY_DEFAULT", "no workflow-level `env`; the cache-save policy lives there")
    else:
        for name, want in sorted(POLICY_ENV.items()):
            if not wf_env.has(name):
                add(
                    doc.line_of("env"),
                    "R5_POLICY_DEFAULT",
                    "workflow env is missing %s (expected %s)" % (name, want),
                )
                continue
            got = _get_str(wf_env, name, "workflow env")
            if got != want:
                add(
                    wf_env.line_of(name),
                    "R5_POLICY_DEFAULT",
                    "%s is %r; expected %r" % (name, got, want),
                )

    for job in jobs:
        steps = job.steps
        has_prune = any(step.is_prune for step in steps)

        # R4, job half.
        got = _permission_pairs(job.permissions, "job %s" % job.job_id) if job.permissions else {}
        if has_prune:
            if got != {"contents": "read", "actions": "write"}:
                add(
                    job.permissions_line,
                    "R4_PERMISSIONS",
                    "job %s prunes but its permissions are %s; expected exactly "
                    "{contents: read, actions: write}" % (job.job_id, _fmt(got)),
                )
        elif "actions" in got:
            add(
                job.permissions_line,
                "R4_PERMISSIONS",
                "job %s has no prune step but takes `actions: %s`; only a pruning job needs it"
                % (job.job_id, got["actions"]),
            )

        for index, step in enumerate(steps):
            if step.is_prune:
                _check_prune_step(add, job, steps, index, step)
            if not step.is_save:
                continue
            key = step.key
            if key is None:
                add(
                    step.line,
                    "R1_NAMESPACE_GATE",
                    "save step %r has no `with.key`" % step.label(),
                )
                continue

            # R7 applies to every save, tool cache included.
            if index != len(steps) - 1:
                add(
                    step.line,
                    "R7_SAVE_IS_LAST",
                    "save step %r is followed by %r; a save must be the last step of its job"
                    % (step.label(), steps[index + 1].label()),
                )

            if not is_lockhash_key(key):
                _check_tool_cache(add, steps, index, step, key)
                continue

            cond = norm(step.cond)
            _check_namespace_gate(add, step, key)

            # R2.
            if MAIN_ONLY_CLAUSE not in cond:
                add(
                    step.cond_line,
                    "R2_MAIN_ONLY",
                    "save step %r does not carry the clause %s" % (step.label(), MAIN_ONLY_CLAUSE),
                )

            # R6.
            if MERGE_GROUP_CLAUSE not in cond:
                add(
                    step.cond_line,
                    "R6_NO_QUEUE_SAVE",
                    "save step %r does not carry %s; a merge-queue branch is deleted after "
                    "the batch, so nothing could ever restore what it saved"
                    % (step.label(), MERGE_GROUP_CLAUSE),
                )

            # R8.
            _check_closed_conjunction(
                add, step.cond_line, "save step %r" % step.label(), cond,
                SAVE_ALLOWED, namespace_clauses(key),
            )

            # R3, the pairing half.
            _check_prune_before_save(add, steps, index, step, key)

    if save_count == 0:
        add(1, "NO_SAVE_STEPS", "this workflow declares the cache-save policy but has no "
                                "`actions/cache/save` step; the policy checks have nothing to hold")

    return problems, stats


def _check_local_policy_env(add, job):
    """R10. The policy lives at workflow level and nowhere else.

    R5 pins the workflow-level values. A job- or step-level `env` of the same
    name overrides them for exactly the steps whose gate reads them, which is a
    policy change invisible to every other rule here.
    """
    for name in POLICY_ENV_NAMES:
        if job.env is not None and job.env.has(name):
            add(
                job.env.line_of(name),
                "R10_NO_LOCAL_POLICY_OVERRIDE",
                "job %s sets %s in its own `env`; the policy is the workflow-level value and "
                "nothing may shadow it" % (job.job_id, name),
            )
        for step in job.steps:
            if step.env is not None and step.env.has(name):
                add(
                    step.env.line_of(name),
                    "R10_NO_LOCAL_POLICY_OVERRIDE",
                    "step %r in job %s sets %s in its own `env`; the policy is the workflow-level "
                    "value and nothing may shadow it" % (step.label(), job.job_id, name),
                )


def _check_tool_cache(add, steps, index, step, key):
    """TOOL_CACHE.

    A key with no `hashFiles(...)` has no lockfile generation, so
    `ci_cache_prune.sh` refuses it and there is nothing for the namespace policy
    to name -- gating it into a namespace the default never lists means the tool
    is never cached and gets reinstalled on every run. Exactly one shape is
    allowed to be keyed that way: a TOOL binary under `~/.cargo/bin/`, a few MB,
    whose key names the pinned version. It keeps the cache-hit test (do not
    re-upload what was restored) and the merge-queue test (nothing could restore
    it), and carries no namespace gate and no prune step. Any other
    lockhash-free key is a violation: a per-commit key inside a namespace the
    policy does name would drift generation on generation with no prune.
    """
    label = step.label()
    path = (step.path or "").strip()
    if "\n" in path or not TOOL_CACHE_PATH_RE.match(path):
        add(
            step.line,
            "TOOL_CACHE",
            "save step %r has a key with no hashFiles(...) lockfile hash (%r), so no prune can "
            "reclaim its generations; only a tool binary cache (one `with.path` under "
            "`~/.cargo/bin/`) may be keyed that way, and this one caches %r"
            % (label, key, step.path),
        )
        return
    if index > 0 and steps[index - 1].is_prune:
        add(
            step.line,
            "TOOL_CACHE",
            "save step %r is a tool binary cache and needs no prune step, but %r precedes it; "
            "the prune script refuses a key with no lockfile hash"
            % (label, steps[index - 1].label()),
        )
    _check_closed_conjunction(
        add, step.cond_line, "tool-cache save step %r" % label, norm(step.cond),
        TOOL_CACHE_ALLOWED, (),
    )
    for wanted in (CACHE_HIT_MARKER.rstrip(" &"), MERGE_GROUP_CLAUSE):
        if wanted not in norm(step.cond):
            add(
                step.cond_line,
                "TOOL_CACHE",
                "tool-cache save step %r does not carry %s" % (label, wanted),
            )


def _check_namespace_gate(add, step, key):
    """R1."""
    cond = norm(step.cond)
    wanted = namespace_clauses(key)
    if not any(clause in cond for clause in wanted):
        add(
            step.cond_line,
            "R1_NAMESPACE_GATE",
            "save step %r does not carry the namespace gate for its key; expected %s"
            % (step.label(), " or ".join(wanted)),
        )


def _check_prune_step(add, job, steps, index, step):
    """R3's orphan half, R2, R6, R8, R9 and R11 for one prune step."""
    label = step.label()
    # R11. An unscoped keep key (`cargo-fuzz-<os>-<lockhash>`, the push-only
    # namespaces) has no `-main-`/`-pr-` segment, so the prune's scope rule does
    # not narrow it: it clears the WHOLE namespace, `main`'s archive included. A
    # pull-request run must therefore never reach such a prune, and the only
    # thing that can promise that is the job's own `if:`.
    mo = PRUNE_RUN_RE.match((step.run or "").strip())
    if mo is not None and is_unscoped_key(mo.group("key")) \
            and not job_excludes_pull_request(job.cond):
        add(
            job.cond_line,
            "R11_UNSCOPED_KEY_NEVER_ON_PULL_REQUEST",
            "prune step %r keeps the unscoped key %r, which clears its whole namespace "
            "(`main`'s archive included, since the scope rule cannot narrow a key with no "
            "scope); job %s must then never run on a pull request, but its `if:` is %r. "
            "Expected a `%s` conjunct, or an `if` that is a disjunction of "
            "`github.event_name == '<event>'` terms naming no pull_request"
            % (label, mo.group("key"), job.job_id, norm(job.cond) or None, EVENT_NE_PULL_REQUEST),
        )
    nxt = steps[index + 1] if index + 1 < len(steps) else None
    if nxt is None or not nxt.is_save:
        add(
            step.line,
            "R3_PRUNE_BEFORE_SAVE",
            "step %r prunes but the step after it is %s; a prune step must sit "
            "directly before the save it makes room for"
            % (label, "the end of the job" if nxt is None else repr(nxt.label())),
        )
    cond = norm(step.cond)
    if MAIN_ONLY_CLAUSE not in cond:
        add(
            step.cond_line,
            "R2_MAIN_ONLY",
            "prune step %r does not carry the clause %s" % (label, MAIN_ONLY_CLAUSE),
        )
    if MERGE_GROUP_CLAUSE not in cond:
        add(
            step.cond_line,
            "R6_NO_QUEUE_SAVE",
            "prune step %r does not carry %s; a merge-queue run must not delete what it cannot "
            "replace" % (label, MERGE_GROUP_CLAUSE),
        )
    _check_closed_conjunction(
        add, step.cond_line, "prune step %r" % label, cond, PRUNE_ALLOWED, (),
    )
    # R9: `gh` and `jq` are on every hosted runner image and on none of the
    # container images the rmw lanes run in, where the prune died at its first
    # `gh cache list` the moment the policy enabled that namespace.
    if not job.container:
        return
    prev = steps[index - 1] if index > 0 else None
    if prev is None or norm(prev.name) != PRUNE_TOOLS_STEP_NAME:
        add(
            step.line,
            "R9_CONTAINER_TOOLS",
            "job %s runs in a container, so prune step %r must be directly preceded by a step "
            "named %r; it is preceded by %s"
            % (job.job_id, label, PRUNE_TOOLS_STEP_NAME,
               "the start of the job" if prev is None else repr(prev.label())),
        )
        return
    if norm(prev.cond) != cond:
        add(
            prev.cond_line,
            "R9_CONTAINER_TOOLS",
            "step %r runs under %r but the prune step after it runs under %r; the install step "
            "must carry the prune's own gate so it costs nothing when the prune does not run"
            % (prev.label(), norm(prev.cond), cond),
        )


def _check_prune_before_save(add, steps, index, step, key):
    """R3.

    A lockhash key with no `-main-`/`-pr-` scope (`cargo-cross-aarch64-<lock
    hash>`, the push-only namespaces) is accepted here and by
    `ci_cache_prune.sh`, which derives the namespace prefix from the hash
    boundary in that case; the two tools agree on every key shape the
    workflows carry.
    """
    label = step.label()
    if index == 0:
        add(step.line, "R3_PRUNE_BEFORE_SAVE",
            "save step %r is the first step of its job; no prune step precedes it" % label)
        return
    prune = steps[index - 1]
    if not prune.is_prune:
        add(
            step.line,
            "R3_PRUNE_BEFORE_SAVE",
            "save step %r is preceded by %r, not by a `%s` step" % (label, prune.label(), PRUNE_CMD),
        )
        return
    mo = PRUNE_RUN_RE.match((prune.run or "").strip())
    if mo is None:
        add(
            prune.line,
            "R3_PRUNE_BEFORE_SAVE",
            "prune step %r does not run exactly `%s \"<key>\"` (optionally plus a prefix); it runs %r"
            % (prune.label(), PRUNE_CMD, (prune.run or "").strip()),
        )
        return
    if mo.group("key") != key:
        add(
            prune.line,
            "R3_PRUNE_BEFORE_SAVE",
            "prune step %r keeps %r but the save writes %r; the prune would delete the entry "
            "the save is about to create" % (prune.label(), mo.group("key"), key),
        )
    prefix = mo.group("prefix")
    if prefix is not None:
        # The explicit prefix widens the sweep across the generations of a key
        # that carries a `${{ steps.<id>.outputs.<name> }}` segment (the rmw
        # header hash). It must be the key text up to that segment EXACTLY: any
        # shorter leading substring reaches into sibling namespaces, and "is a
        # leading substring" alone does not say where the namespace ends.
        gen = STEPS_OUTPUT_EXPR_RE.search(key)
        if gen is None:
            add(
                prune.line,
                "R3_PRUNE_BEFORE_SAVE",
                "prune step %r passes the explicit prefix %r, but the save key %r carries no "
                "`${{ steps.<id>.outputs.<name> }}` generation segment for it to stop at"
                % (prune.label(), prefix, key),
            )
        elif prefix != key[: gen.start()]:
            add(
                prune.line,
                "R3_PRUNE_BEFORE_SAVE",
                "prune step %r sweeps prefix %r; for this key the only prefix that names the "
                "namespace and no more is %r (the text before the generation segment)"
                % (prune.label(), prefix, key[: gen.start()]),
            )
    token = _get_str(prune.env, "GH_TOKEN", "prune step") if prune.env else None
    if token != "${{ github.token }}":
        add(
            prune.line,
            "R3_PRUNE_BEFORE_SAVE",
            "prune step %r has env.GH_TOKEN = %r; expected '${{ github.token }}'"
            % (prune.label(), token),
        )
    save_cond = norm(step.cond)
    prune_cond = norm(prune.cond)
    if not save_cond.startswith("steps.") or CACHE_HIT_MARKER not in save_cond:
        add(
            step.cond_line,
            "R3_PRUNE_BEFORE_SAVE",
            "save step %r must start with a `steps.<restore id>%s` test" % (label, CACHE_HIT_MARKER),
        )
        return
    head = save_cond[: save_cond.index(CACHE_HIT_MARKER) + len(CACHE_HIT_MARKER)]
    if not CACHE_HIT_PREFIX_RE.match(head):
        add(
            step.cond_line,
            "R3_PRUNE_BEFORE_SAVE",
            "save step %r starts with %r, not a bare `steps.<restore id>%s`"
            % (label, head, CACHE_HIT_MARKER),
        )
        return
    # The save is the prune's own gate, plus the cache-hit test in front and the
    # namespace gate behind: the save runs on a subset of the runs that prune.
    expected = [head + prune_cond + " && " + clause for clause in namespace_clauses(key)]
    if save_cond not in expected:
        add(
            step.cond_line,
            "R3_PRUNE_BEFORE_SAVE",
            "save step %r is not the prune step's condition plus the cache-hit test and the "
            "namespace gate: save %r, expected %r" % (label, save_cond, expected[0]),
        )


def _permission_pairs(mapping, what):
    out = {}
    if mapping is None:
        return out
    for key in mapping.keys():
        out[key] = _get_str(mapping, key, what)
    return out


def _fmt(pairs):
    if not pairs:
        return "{}"
    return "{" + ", ".join("%s: %s" % (k, v) for k, v in sorted(pairs.items())) + "}"


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

CLEAN_WORKFLOW = """name: CI
'on':
  push:
    branches: [main]
  pull_request:
  merge_group:

permissions:
  contents: read

env:
  CACHE_SAVE_NAMESPACES: ${{ vars.CACHE_SAVE_NAMESPACES || 'macOS Linux' }}
  CACHE_SAVE_ON_PULL_REQUEST: ${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}

jobs:
  test-linux:
    name: Test (Linux) shard ${{ matrix.shard }}
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    strategy:
      fail-fast: false
      matrix:
        shard: [0, 1, 2, 3]
    steps:
      - name: Checkout
        uses: actions/checkout@v5

      # Restoring is free against the allowance, so it is unconditional.
      - name: Cache cargo (restore)
        id: cache-restore-1
        uses: actions/cache/restore@v4
        with:
          path: |
            ~/.cargo/registry
            ~/.cargo/git
            target
          key: cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}
          restore-keys: |
            cargo-${{ runner.os }}-main-
            cargo-${{ runner.os }}-

      - name: Build and test
        run: |
          cargo build --locked
          cargo test --locked

      - name: Prune the cache namespace, Linux shard
        if: >-
          matrix.shard == 0
          && github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"

      - name: Cache cargo, Linux shard (save)
        if: >-
          steps.cache-restore-1.outputs.cache-hit != 'true'
          && matrix.shard == 0
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))
        uses: actions/cache/save@v4
        with:
          path: |
            ~/.cargo/registry
            ~/.cargo/git
            target
          key: cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}

  lint:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    steps:
      - name: Checkout
        uses: actions/checkout@v5
      - name: Cache cargo (restore)
        id: cache-restore-lint
        uses: actions/cache/restore@v4
        with:
          key: cargo-${{ runner.os }}-lint-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}
      - name: Clippy
        run: cargo clippy --locked --all-targets
      - name: Prune the cache namespace, lint
        if: >-
          github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-lint-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, lint (save)
        if: >-
          steps.cache-restore-lint.outputs.cache-hit != 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0}-lint ', runner.os)))
        uses: actions/cache/save@v4
        with:
          key: cargo-${{ runner.os }}-lint-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}

  viz-tests:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    strategy:
      matrix:
        lane: [studio, gateway]
    steps:
      - name: Cache cargo (restore)
        id: cache-restore-viz
        uses: actions/cache/restore@v4
        with:
          key: cargo-viz-${{ matrix.lane }}-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}
      - name: Viz tests
        run: |-
          cargo test -p cerulion_vizd --locked
      - name: Prune the cache namespace, viz
        if: >-
          github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-viz-${{ matrix.lane }}-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, viz (save)
        if: >-
          steps.cache-restore-viz.outputs.cache-hit != 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' viz-{0}-{1} ', matrix.lane, runner.os)))
        uses: actions/cache/save@v4
        with:
          key: cargo-viz-${{ matrix.lane }}-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}

  cross:
    runs-on: ubuntu-latest
    # `cargo-cross-aarch64-<lockhash>` carries no scope segment, so its prune
    # clears the whole namespace: R11 needs this job off pull requests.
    if: github.event_name != 'pull_request' && github.event_name != 'merge_group'
    permissions:
      contents: read
      actions: write
    steps:
      - name: Skip if the target is absent on this ref
        id: present
        run: echo "present=true" >> "$GITHUB_OUTPUT"
      - name: Cache cargo (restore)
        id: cache-restore-cross
        uses: actions/cache/restore@v4
        with:
          key: cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}
      - name: Cross build
        run: cargo build --locked --target aarch64-unknown-linux-gnu
      - name: Prune the cache namespace, cross
        if: >
          steps.present.outputs.present == 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, cross (save)
        if: >
          steps.cache-restore-cross.outputs.cache-hit != 'true'
          && steps.present.outputs.present == 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))
        uses: actions/cache/save@v4
        with:
          key: cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}

  latency:
    runs-on: ubuntu-latest
    # The other shape R11 accepts for an unscoped key: an allowlist of events,
    # none of them a pull request.
    if: github.event_name == 'push' || github.event_name == 'workflow_dispatch'
    permissions:
      contents: read
      actions: write
    steps:
      - name: Cache cargo (restore)
        id: cache-restore-latency
        uses: actions/cache/restore@v4
        with:
          key: cargo-release-${{ runner.os }}-${{ hashFiles('**/Cargo.lock') }}
      - name: Measure
        run: cargo run --release -p bench
      - name: Prune the cache namespace, latency
        if: >-
          github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-release-${{ runner.os }}-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, latency (save)
        if: >-
          steps.cache-restore-latency.outputs.cache-hit != 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' release-{0} ', runner.os)))
        uses: actions/cache/save@v4
        with:
          key: cargo-release-${{ runner.os }}-${{ hashFiles('**/Cargo.lock') }}

  lane:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    strategy:
      matrix:
        distro: [jazzy, humble]
    container:
      image: ros:${{ matrix.distro }}-ros-base
    steps:
      - name: Header tree hash
        id: headers
        run: echo "hash=deadbeefdeadbeef" >> "$GITHUB_OUTPUT"
      - name: Cache cargo (restore)
        id: cache-restore-lane
        uses: actions/cache/restore@v4
        with:
          key: cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}
          restore-keys: |
            cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-main-
            cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-
      - name: Gate (expected state for this distro)
        run: bash tools/ci/rmw-distros/gate.sh "${{ matrix.distro }}"
      - name: Install the cache prune tools (gh, jq)
        if: >-
          github.event_name != 'merge_group'
          && <MAIN>
        run: apt-get install -y gh jq
      - name: Prune the cache namespace, rmw distro
        if: >-
          github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}" "cargo-rmw-distros-${{ matrix.distro }}-"
      - name: Cache cargo, rmw distro (save)
        if: >-
          steps.cache-restore-lane.outputs.cache-hit != 'true'
          && github.event_name != 'merge_group'
          && <MAIN>
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' rmw-distros-{0} ', matrix.distro)))
        uses: actions/cache/save@v4
        with:
          key: cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}

  machete:
    runs-on: ubuntu-latest
    permissions:
      contents: read
    steps:
      - name: Cache the machete binary (restore)
        id: cache-restore-machete
        uses: actions/cache/restore@v4
        with:
          path: ~/.cargo/bin/cargo-machete
          key: cargo-machete-bin-${{ runner.os }}-v0.9.2
      - name: Install cargo-machete
        if: steps.cache-restore-machete.outputs.cache-hit != 'true'
        run: cargo install cargo-machete --version 0.9.2 --locked
      - name: Cache the machete binary (save)
        if: >-
          steps.cache-restore-machete.outputs.cache-hit != 'true'
          && github.event_name != 'merge_group'
        uses: actions/cache/save@v4
        with:
          path: ~/.cargo/bin/cargo-machete
          key: cargo-machete-bin-${{ runner.os }}-v0.9.2
""".replace("<MAIN>", MAIN_ONLY_CLAUSE)

# A workflow that restores and never saves costs the store nothing, so it
# carries no policy `env`, no `actions: write` and no prune step -- and the
# policy rules must NOT fire on it. The structural rules still do.
RESTORE_ONLY_WORKFLOW = """name: Release
'on':
  push:
    tags: ["v*"]
  workflow_dispatch: {}

permissions:
  contents: read

jobs:
  publish:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      # A tag run's save is restorable by no other ref, so this one only reads.
      - name: Cache cargo (restore)
        uses: actions/cache/restore@v4
        with:
          path: |
            ~/.cargo/registry
            target
          key: cargo-release-${{ runner.os }}-${{ hashFiles('**/Cargo.lock') }}
          restore-keys: |
            cargo-release-${{ runner.os }}-
      - name: Publish
        run: cargo publish --workspace
"""

REUSABLE_CALL_WORKFLOW = """name: Caller
'on':
  push:
    branches: [main]

permissions:
  contents: read

jobs:
  delegate:
    uses: ./.github/workflows/ci.yml
    secrets: inherit
"""

NO_STEPS_WORKFLOW = """name: Empty
'on':
  push:
    branches: [main]

permissions:
  contents: read

jobs:
  nothing:
    runs-on: ubuntu-latest
    timeout-minutes: 5
"""

STRAY_PRUNE_STEP = """      - name: Stray prune with nothing to guard
        if: >-
          matrix.shard == 0
          && github.event_name != 'merge_group'
          && <MAIN>
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"

""".replace("<MAIN>", MAIN_ONLY_CLAUSE)

# The `cross` job's two gates, quoted exactly as the fixture writes them. They
# are the only gates in the fixture that carry a step-output guard, so each of
# these anchors occurs exactly once and a mutation can move prune and save in
# step (which is what an attack on R8 looks like: R3's equality still holds).
CROSS_PRUNE_IF = (
    "          steps.present.outputs.present == 'true'\n"
    "          && github.event_name != 'merge_group'\n"
    "          && " + MAIN_ONLY_CLAUSE + "\n"
)
CROSS_SAVE_IF = (
    "          && steps.present.outputs.present == 'true'\n"
    "          && github.event_name != 'merge_group'\n"
    "          && " + MAIN_ONLY_CLAUSE + "\n"
)
CROSS_NS_CLAUSE = (
    "          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
    "env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))\n"
)


def _mut(text, old, new, total=1, occurrence=1):
    """Replace one occurrence of `old`, asserting the anchor count first.

    The count assertion is the point: a mutation whose anchor silently stopped
    matching would test nothing and report a pass.
    """
    found = text.count(old)
    if found != total:
        raise AssertionError(
            "mutation anchor occurs %d times, expected %d: %r" % (found, total, old[:90])
        )
    if occurrence == "all":
        return text.replace(old, new)
    at = -1
    for _ in range(occurrence):
        at = text.index(old, at + 1)
    return text[:at] + new + text[at + len(old):]


def _drop_all_save_steps(text):
    out = []
    lines = text.split("\n")
    i = 0
    while i < len(lines):
        line = lines[i]
        if not line.startswith("      - name: "):
            out.append(line)
            i += 1
            continue
        block = [line]
        j = i + 1
        while j < len(lines):
            nxt = lines[j]
            if nxt.strip() == "":
                block.append(nxt)
                j += 1
                continue
            indent = len(nxt) - len(nxt.lstrip(" "))
            if indent < 8 and nxt.strip() != "":
                break
            block.append(nxt)
            j += 1
        if any(("uses: " + SAVE_USES_PREFIX) in b for b in block):
            i = j
            continue
        out.extend(block)
        i = j
    return "\n".join(out)


def _codes(problems):
    seen = []
    for line in problems:
        mo = re.match(r"^[^:]*:\d+: ([A-Z0-9_]+): ", line)
        if mo is None:
            raise AssertionError("violation line is not in the reporting format: %r" % line)
        if mo.group(1) not in seen:
            seen.append(mo.group(1))
    return set(seen)


# The pre-fork-condition main-only clause, used by the mutant that drops the
# fork half: a fork pull request's token cannot hold `actions: write`, so the
# prune's delete would fail the job.
MAIN_ONLY_WITHOUT_FORK = "(github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')"


def _mutants():
    """(name, mutated text, expected rule codes).

    `None` means the file must be refused with a parse error; an EMPTY set
    means the variant must still pass (the near-miss beside a mutation, which
    is what keeps a rule from passing by never firing).
    """
    clean = CLEAN_WORKFLOW
    out = []

    # R1: the lint job's token no longer matches its own key template.
    out.append((
        "R1 wrong token",
        _mut(clean, "format(' {0}-lint ', runner.os)))", "format(' {0}-lintx ', runner.os)))"),
        {"R1_NAMESPACE_GATE", "R3_PRUNE_BEFORE_SAVE", "R8_CLOSED_CONJUNCTION"},
    ))
    # R1: the shard job keeps the shard key but carries the lint job's token.
    # There is no `-<job id>` allowance, so the two must match exactly.
    out.append((
        "R1 shard key with the lint token",
        _mut(clean, "format(' {0} ', runner.os)))", "format(' {0}-lint ', runner.os)))"),
        {"R1_NAMESPACE_GATE", "R3_PRUNE_BEFORE_SAVE", "R8_CLOSED_CONJUNCTION"},
    ))
    # R1: the `all` escape dropped from the literal-token job.
    out.append((
        "R1 missing the == 'all' alternative",
        _mut(
            clean,
            CROSS_NS_CLAUSE,
            "          && (contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), "
            "' cross-aarch64 '))\n",
        ),
        {"R1_NAMESPACE_GATE", "R3_PRUNE_BEFORE_SAVE", "R8_CLOSED_CONJUNCTION"},
    ))
    # R2 on a save. The save and prune conditions must stay in step, so removing
    # the clause from one side necessarily trips R3 as well.
    out.append((
        "R2 clause removed from a save",
        _mut(
            clean,
            "          steps.cache-restore-viz.outputs.cache-hit != 'true'\n"
            "          && github.event_name != 'merge_group'\n"
            "          && " + MAIN_ONLY_CLAUSE + "\n",
            "          steps.cache-restore-viz.outputs.cache-hit != 'true'\n"
            "          && github.event_name != 'merge_group'\n",
        ),
        {"R2_MAIN_ONLY", "R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R2 clause removed from a prune",
        _mut(
            clean,
            "          && " + MAIN_ONLY_CLAUSE + "\n        env:\n"
            "          GH_TOKEN: ${{ github.token }}\n"
            '        run: bash tools/scripts/ci_cache_prune.sh "cargo-viz-',
            "        env:\n          GH_TOKEN: ${{ github.token }}\n"
            '        run: bash tools/scripts/ci_cache_prune.sh "cargo-viz-',
        ),
        {"R2_MAIN_ONLY", "R3_PRUNE_BEFORE_SAVE"},
    ))
    # R2: the fork half dropped from BOTH sides, so R3's equality still holds
    # and only the clause itself is wrong. A fork pull request's token is
    # read-only, so the prune's delete would fail the job.
    forkless = _mut(clean, CROSS_PRUNE_IF,
                    CROSS_PRUNE_IF.replace(MAIN_ONLY_CLAUSE, MAIN_ONLY_WITHOUT_FORK))
    forkless = _mut(forkless, CROSS_SAVE_IF,
                    CROSS_SAVE_IF.replace(MAIN_ONLY_CLAUSE, MAIN_ONLY_WITHOUT_FORK))
    out.append((
        "R2 fork condition dropped from the main-only clause",
        forkless,
        {"R2_MAIN_ONLY", "R8_CLOSED_CONJUNCTION"},
    ))
    # R3: the prune keeps a key one character away from the one the save writes,
    # so it would delete the entry the save is about to upload.
    out.append((
        "R3 prune key differs by one character",
        _mut(
            clean,
            'run: bash tools/scripts/ci_cache_prune.sh "cargo-rmw-distros-${{ matrix.distro }}-'
            "${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || "
            "'pr' }}-${{ hashFiles('**/Cargo.lock') }}\" \"cargo-rmw-distros-${{ matrix.distro }}-\"",
            'run: bash tools/scripts/ci_cache_prune.sh "cargo-rmw-distros-${{ matrix.distro }}-'
            "${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || "
            "'pr' }}-${{ hashFiles('**/Cargo.locl') }}\" \"cargo-rmw-distros-${{ matrix.distro }}-\"",
        ),
        {"R3_PRUNE_BEFORE_SAVE"},
    ))
    # R3: the explicit prefix is a leading substring but not the structural one.
    # `cargo-rmw-distros-` sweeps every distro's namespace, not this lane's.
    out.append((
        "R3 explicit prefix wider than the generation segment",
        _mut(clean, '" "cargo-rmw-distros-${{ matrix.distro }}-"', '" "cargo-rmw-distros-"'),
        {"R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R3 a step inserted between prune and save",
        _mut(clean, "      - name: Cache cargo, Linux shard (save)\n",
             "      - name: Interloper\n        run: echo interloper\n\n"
             "      - name: Cache cargo, Linux shard (save)\n"),
        {"R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R3 prune GH_TOKEN missing",
        _mut(
            clean,
            "        env:\n          GH_TOKEN: ${{ github.token }}\n"
            '        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-'
            "${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-"
            "${{ hashFiles('**/Cargo.lock') }}\"",
            '        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-'
            "${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-"
            "${{ hashFiles('**/Cargo.lock') }}\"",
        ),
        {"R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R3 save if is not cache-hit plus the prune if plus the namespace gate",
        _mut(clean,
             "          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
             "env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))\n"
             "        uses: actions/cache/save@v4\n",
             "          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
             "env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))\n"
             "          && true\n        uses: actions/cache/save@v4\n"),
        {"R3_PRUNE_BEFORE_SAVE", "R8_CLOSED_CONJUNCTION"},
    ))
    # R3 + R8: the namespace gate back on the PRUNE, which is the shape this
    # change removed. It left a namespace the policy does not name unpruned for
    # ever, because nothing else ever deletes a cache entry a pull request keeps
    # restoring.
    out.append((
        "R3 namespace gate back on the prune",
        _mut(clean, CROSS_PRUNE_IF, CROSS_PRUNE_IF + CROSS_NS_CLAUSE),
        {"R3_PRUNE_BEFORE_SAVE", "R8_CLOSED_CONJUNCTION"},
    ))
    out.append((
        "R3 prune present but no save after it",
        _mut(clean, "      - name: Build and test\n", STRAY_PRUNE_STEP + "      - name: Build and test\n"),
        {"R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R4 pruning job missing actions: write",
        _mut(clean,
             "  test-linux:\n    name: Test (Linux) shard ${{ matrix.shard }}\n"
             "    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n"
             "      actions: write\n",
             "  test-linux:\n    name: Test (Linux) shard ${{ matrix.shard }}\n"
             "    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n"),
        {"R4_PERMISSIONS"},
    ))
    out.append((
        "R4 non-pruning job with actions: write",
        _mut(clean,
             "  machete:\n    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n",
             "  machete:\n    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n"
             "      actions: write\n"),
        {"R4_PERMISSIONS"},
    ))
    out.append((
        "R4 workflow permissions widened",
        _mut(clean, "\npermissions:\n  contents: read\n",
             "\npermissions:\n  contents: read\n  actions: write\n"),
        {"R4_PERMISSIONS"},
    ))
    out.append((
        "R5 default literal changed",
        _mut(clean, "vars.CACHE_SAVE_NAMESPACES || 'macOS Linux'",
             "vars.CACHE_SAVE_NAMESPACES || 'Linux'"),
        {"R5_POLICY_DEFAULT"},
    ))
    out.append((
        "R5 policy key missing",
        _mut(clean, "  CACHE_SAVE_ON_PULL_REQUEST: ${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}\n", ""),
        {"R5_POLICY_DEFAULT"},
    ))
    # R6 on both sides of the cross job, so R3's equality still holds.
    queue_ok = _mut(clean, CROSS_PRUNE_IF,
                    CROSS_PRUNE_IF.replace("          && github.event_name != 'merge_group'\n", ""))
    queue_ok = _mut(queue_ok, CROSS_SAVE_IF,
                    CROSS_SAVE_IF.replace("          && github.event_name != 'merge_group'\n", ""))
    out.append(("R6 merge-queue clause removed", queue_ok, {"R6_NO_QUEUE_SAVE"}))
    out.append((
        "R7 a step appended after the save",
        clean + "      - name: Report\n        run: echo done\n",
        {"R7_SAVE_IS_LAST"},
    ))
    # R8: `|| true` appended to the prune AND the save. Every required substring
    # is still there and R3's equality still holds -- the gate is simply off.
    or_true = _mut(clean, CROSS_PRUNE_IF,
                   CROSS_PRUNE_IF.replace(MAIN_ONLY_CLAUSE + "\n", MAIN_ONLY_CLAUSE + " || true\n"))
    or_true = _mut(or_true, CROSS_SAVE_IF,
                   CROSS_SAVE_IF.replace(MAIN_ONLY_CLAUSE + "\n", MAIN_ONLY_CLAUSE + " || true\n"))
    out.append(("R8 || true appended to both gates", or_true, {"R8_CLOSED_CONJUNCTION"}))
    # R8: an unknown conjunct, added to both sides so nothing else fires.
    unknown = _mut(clean, CROSS_PRUNE_IF, CROSS_PRUNE_IF + "          && success()\n")
    unknown = _mut(unknown, CROSS_SAVE_IF, CROSS_SAVE_IF + "          && success()\n")
    out.append(("R8 an unknown conjunct", unknown, {"R8_CLOSED_CONJUNCTION"}))
    # R8, the passing side: a complete conjunction in a different ORDER is fine,
    # so the rule is about membership and not about the text of one file.
    reordered = _mut(
        clean, CROSS_PRUNE_IF,
        "          github.event_name != 'merge_group'\n"
        "          && " + MAIN_ONLY_CLAUSE + "\n"
        "          && steps.present.outputs.present == 'true'\n",
    )
    reordered = _mut(
        reordered, CROSS_SAVE_IF,
        "          && github.event_name != 'merge_group'\n"
        "          && " + MAIN_ONLY_CLAUSE + "\n"
        "          && steps.present.outputs.present == 'true'\n",
    )
    out.append(("R8 a reordered but complete conjunction passes", reordered, set()))
    # R9: the container lane without the install step. `gh` and `jq` are on
    # every hosted image and on none of the ros: images.
    install_step = (
        "      - name: Install the cache prune tools (gh, jq)\n"
        "        if: >-\n"
        "          github.event_name != 'merge_group'\n"
        "          && " + MAIN_ONLY_CLAUSE + "\n"
        "        run: apt-get install -y gh jq\n"
    )
    out.append((
        "R9 container job without the tool install step",
        _mut(clean, install_step, ""),
        {"R9_CONTAINER_TOOLS"},
    ))
    out.append((
        "R9 install step under a different condition",
        _mut(clean, install_step,
             "      - name: Install the cache prune tools (gh, jq)\n"
             "        if: github.event_name != 'merge_group'\n"
             "        run: apt-get install -y gh jq\n"),
        {"R9_CONTAINER_TOOLS"},
    ))
    # R10: the policy shadowed on a job, and on a step.
    out.append((
        "R10 job-level policy override",
        _mut(clean,
             "  lint:\n    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n"
             "      actions: write\n",
             "  lint:\n    runs-on: ubuntu-latest\n    permissions:\n      contents: read\n"
             "      actions: write\n    env:\n      CACHE_SAVE_NAMESPACES: Linux-lint\n"),
        {"R10_NO_LOCAL_POLICY_OVERRIDE"},
    ))
    out.append((
        "R10 step-level policy override",
        _mut(clean,
             "        env:\n          GH_TOKEN: ${{ github.token }}\n"
             '        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-'
             "${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-"
             "${{ hashFiles('**/Cargo.lock') }}\"",
             "        env:\n          GH_TOKEN: ${{ github.token }}\n"
             "          CACHE_SAVE_ON_PULL_REQUEST: yes\n"
             '        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-'
             "${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-"
             "${{ hashFiles('**/Cargo.lock') }}\""),
        {"R10_NO_LOCAL_POLICY_OVERRIDE"},
    ))
    # TOOL_CACHE: the same lockhash-free key over a cargo target directory. That
    # key has no generation to prune by, so the archive would accumulate.
    out.append((
        "TOOL_CACHE a lockhash-free key over target/",
        _mut(clean,
             "        uses: actions/cache/save@v4\n        with:\n"
             "          path: ~/.cargo/bin/cargo-machete\n"
             "          key: cargo-machete-bin-${{ runner.os }}-v0.9.2\n",
             "        uses: actions/cache/save@v4\n        with:\n"
             "          path: target\n"
             "          key: cargo-machete-bin-${{ runner.os }}-v0.9.2\n"),
        {"TOOL_CACHE"},
    ))
    # R11: the unscoped key reachable from a pull request, three ways.
    out.append((
        "R11 the job condition removed",
        _mut(clean,
             "    # `cargo-cross-aarch64-<lockhash>` carries no scope segment, so its prune\n"
             "    # clears the whole namespace: R11 needs this job off pull requests.\n"
             "    if: github.event_name != 'pull_request' && github.event_name != 'merge_group'\n",
             ""),
        {"R11_UNSCOPED_KEY_NEVER_ON_PULL_REQUEST"},
    ))
    out.append((
        "R11 pull_request added to the allowlist",
        _mut(clean,
             "    if: github.event_name == 'push' || github.event_name == 'workflow_dispatch'\n",
             "    if: github.event_name == 'push' || github.event_name == 'workflow_dispatch'"
             " || github.event_name == 'pull_request'\n"),
        {"R11_UNSCOPED_KEY_NEVER_ON_PULL_REQUEST"},
    ))
    out.append((
        "R11 the pull_request conjunct narrowed to merge_group",
        _mut(clean,
             "    if: github.event_name != 'pull_request' && github.event_name != 'merge_group'\n",
             "    if: github.event_name != 'merge_group'\n"),
        {"R11_UNSCOPED_KEY_NEVER_ON_PULL_REQUEST"},
    ))
    # Removing every save leaves five prune steps guarding nothing, so this
    # mutation necessarily trips R3 beside NO_SAVE_STEPS.
    out.append((
        "NO_SAVE_STEPS every save removed",
        _drop_all_save_steps(clean),
        {"NO_SAVE_STEPS", "R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "parse error: a YAML anchor",
        _mut(clean, "    runs-on: ubuntu-latest\n", "    runs-on: &ru ubuntu-latest\n",
             total=7, occurrence=1),
        None,
    ))
    # The restore-only workflow and its two structural failures.
    out.append(("restore-only workflow passes", RESTORE_ONLY_WORKFLOW, set()))
    out.append((
        "NO_COMBINED_CACHE_ACTION the combined action saves ungated",
        _mut(RESTORE_ONLY_WORKFLOW, "uses: actions/cache/restore@v4", "uses: actions/cache@v4"),
        {"NO_COMBINED_CACHE_ACTION"},
    ))
    out.append(("parse error: a reusable-workflow job", REUSABLE_CALL_WORKFLOW, None))
    out.append(("parse error: a job with no steps", NO_STEPS_WORKFLOW, None))
    return out


def self_test():
    fails = []
    cases = 0

    cases += 1
    problems, stats = check_text("mem.yml", CLEAN_WORKFLOW)
    if problems:
        fails.append("clean workflow reported %d violation(s): %s" % (len(problems), problems))
    # Guard against a fixture or reader that silently stopped seeing the steps:
    # every rule below would pass vacuously on an empty step list.
    if stats != {"saves": 7, "prunes": 6, "policy": 1}:
        fails.append("clean workflow parsed to %r, expected 7 saves and 6 prunes" % (stats,))

    cases += 1
    problems, stats = check_text("mem.yml", RESTORE_ONLY_WORKFLOW)
    if problems:
        fails.append("restore-only workflow reported %d violation(s): %s" % (len(problems), problems))
    if stats != {"saves": 0, "prunes": 0, "policy": 0}:
        fails.append("restore-only workflow parsed to %r, expected no saves and no policy" % (stats,))

    for name, text, expected in _mutants():
        cases += 1
        try:
            problems, _ = check_text("mem.yml", text)
        except ParseError as exc:
            if expected is not None:
                fails.append("%s: unexpected parse error: %s" % (name, exc))
            continue
        if expected is None:
            fails.append("%s: expected a parse error, got %r" % (name, problems))
            continue
        got = _codes(problems)
        if got != expected:
            fails.append(
                "%s: reported %s, expected %s\n    %s"
                % (name, sorted(got), sorted(expected), "\n    ".join(problems) or "(nothing)")
            )

    # Usage half: a file with no jobs is a parse error, not a quiet pass.
    cases += 1
    try:
        check_text("mem.yml", "name: CI\npermissions:\n  contents: read\nenv:\n"
                              "  CACHE_SAVE_NAMESPACES: ${{ vars.CACHE_SAVE_NAMESPACES || 'macOS Linux' }}\n"
                              "  CACHE_SAVE_ON_PULL_REQUEST: ${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}\n")
    except ParseError:
        pass
    else:
        fails.append("a file with no `jobs` mapping did not raise a parse error")

    if fails:
        for line in fails:
            sys.stderr.write("ci_cache_policy_check: self-test FAILED: %s\n" % line)
        sys.stderr.write("ci_cache_policy_check: %d self-test case(s) failed\n" % len(fails))
        return 1
    sys.stdout.write("ci_cache_policy_check: self-test OK (%d cases)\n" % cases)
    return 0


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

USAGE = (
    "usage: python3 tools/scripts/ci_cache_policy_check.py <workflow.yml|directory>...\n"
    "       python3 tools/scripts/ci_cache_policy_check.py --self-test\n"
)


def expand(paths):
    """Every argument that names a directory contributes its workflow files.

    A directory is the form the Lint step uses, so a new workflow is covered the
    day it lands rather than the day somebody remembers to add it to a list.
    """
    out = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".yml") or name.endswith(".yaml"):
                    out.append(os.path.join(path, name))
            continue
        out.append(path)
    return out


def main(argv):
    if len(argv) == 1 and argv[0] == "--self-test":
        return self_test()
    if not argv or any(a.startswith("-") for a in argv):
        sys.stderr.write(USAGE)
        return 2
    files = expand(argv)
    if not files:
        sys.stderr.write("%s: no workflow files found under %s\n" % (USAGE, ", ".join(argv)))
        return 2
    violations = []
    saves = 0
    for path in files:
        try:
            with open(path, "r") as handle:
                text = handle.read()
        except OSError as exc:
            sys.stderr.write("%s: cannot read: %s\n" % (path, exc))
            return 2
        try:
            problems, stats = check_text(path, text)
        except ParseError as exc:
            sys.stderr.write("%s:%d: PARSE_ERROR: %s\n" % (path, exc.line, exc.msg))
            return 2
        violations.extend(problems)
        saves += stats["saves"]
    if saves == 0:
        sys.stderr.write(
            "%s: no `actions/cache/save` step in any of the %d file(s) read; this checker would "
            "pass on anything\n" % (", ".join(argv), len(files))
        )
        return 2
    if violations:
        for line in violations:
            sys.stdout.write(line + "\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
