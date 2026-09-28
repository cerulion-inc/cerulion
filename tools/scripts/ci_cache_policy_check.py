#!/usr/bin/env python3
"""ci_cache_policy_check.py — hold the cache-save policy in the workflows.

    python3 tools/scripts/ci_cache_policy_check.py .github/workflows/ci.yml ...
    python3 tools/scripts/ci_cache_policy_check.py --self-test

WHY THIS EXISTS. The repository's GitHub Actions cache store has a 10 GB free
allowance, and above it saves are refused while the account carries a failed
payment. The workflows now gate every `actions/cache/save` step behind a policy
-- a list of namespaces that may save (`CACHE_SAVE_NAMESPACES`, default
`macOS Linux`), main-only unless `CACHE_SAVE_ON_PULL_REQUEST` lifts it -- and a
prune step runs directly before each save so the namespace holds exactly the
key the job is about to write. Every one of those conditions lives in an `if:`
expression, which no runner evaluates until the job is already running and
which nothing else in the tree reads. This file reads them.

The failure this guards against is not a wrong expression, it is a MISSING one:
a new job copied from an old one, a save step whose gate was dropped during a
rebase, or a namespace token that no longer matches the key it gates. All three
are silent -- the workflow is still valid YAML, the job still runs, and the only
symptom is the store filling up again a week later.

Stdlib only: PyYAML is not installed on the runners, so the reader below is a
purpose-built one for the YAML subset these workflows use. It refuses anything
outside that subset with a parse error naming the line rather than skipping it,
because a reader that silently ignores what it cannot understand reports a
green file it never read.

Exit 0 every rule holds, 1 one line per violation on stdout, 2 usage or a file
it cannot parse.

`--self-test` builds a workflow in memory, asserts it passes, then applies one
mutation per rule and asserts each mutant reports exactly that rule. The CI step
that runs it is the only thing that proves this checker is not inert.
"""

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
            raise ParseError(line, "YAML flow mappings are outside this reader's subset")
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
            if style == ">" and ind > block_indent:
                raise ParseError(
                    j + 1,
                    "a more-indented line inside a folded scalar is outside this reader's subset",
                )
            body.append(raw[block_indent:])
            j += 1
        while body and body[-1] == "":
            body.pop()
        if style == "|":
            text = "\n".join(body)
        else:
            chunks = []
            for entry in body:
                if entry == "":
                    chunks.append("\n")
                else:
                    if chunks and not chunks[-1].endswith("\n"):
                        chunks.append(" ")
                    chunks.append(entry)
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
        self.cond_line = node.line_of("if") if node.has("if") else node.line

    @property
    def is_save(self):
        return bool(self.uses) and self.uses.startswith(SAVE_USES_PREFIX)

    @property
    def is_prune(self):
        return bool(self.run) and self.run.strip().startswith(PRUNE_CMD)

    @property
    def key(self):
        if self.with_ is None:
            return None
        return _get_str(self.with_, "key", "step at line %d" % self.line)

    def label(self):
        return self.name or (self.uses or "step") or "step"


class Job(object):
    def __init__(self, job_id, node):
        if not isinstance(node, Map):
            raise ParseError(node.line, "job %s must be a mapping" % job_id)
        self.job_id = job_id
        self.node = node
        self.line = node.line
        self.permissions = _get_map(node, "permissions", "job %s" % job_id)
        self.permissions_line = node.line_of("permissions") if node.has("permissions") else node.line
        self.steps = []
        steps_node = node.get("steps")
        if steps_node is None:
            return
        if not isinstance(steps_node, Seq):
            raise ParseError(node.line_of("steps"), "job %s: `steps` must be a list" % job_id)
        for index, item in enumerate(steps_node.items):
            self.steps.append(Step(item, job_id, index))


# ---------------------------------------------------------------------------
# policy vocabulary
# ---------------------------------------------------------------------------

SAVE_USES_PREFIX = "actions/cache/save@"
PRUNE_CMD = "bash tools/scripts/ci_cache_prune.sh"
PRUNE_RUN_RE = re.compile(
    r'^bash tools/scripts/ci_cache_prune\.sh "(?P<key>[^"]*)"(?: "(?P<prefix>[^"]*)")?$'
)
MAIN_ONLY_CLAUSE = "(github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')"
MERGE_GROUP_CLAUSE = "github.event_name != 'merge_group'"
NS_ALL_CLAUSE = "env.CACHE_SAVE_NAMESPACES == 'all'"
NS_CONTAINS_HEAD = "contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), "
CACHE_HIT_PREFIX_RE = re.compile(r"^steps\.[A-Za-z0-9_.\-]+\.outputs\.cache-hit != 'true' && $")
CACHE_HIT_MARKER = ".outputs.cache-hit != 'true' && "
SCOPE_SUFFIX = "-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}"
HASHFILES_RE = re.compile(r"-\$\{\{\s*hashFiles\(")
STEPS_OUTPUT_SEG_RE = re.compile(r"-\$\{\{\s*steps\.[A-Za-z0-9_.\-]+\.outputs\.[A-Za-z0-9_.\-]+\s*\}\}")
EXPR_RE = re.compile(r"\$\{\{(.*?)\}\}", re.S)
POLICY_ENV = {
    "CACHE_SAVE_NAMESPACES": "${{ vars.CACHE_SAVE_NAMESPACES || 'macOS Linux' }}",
    "CACHE_SAVE_ON_PULL_REQUEST": "${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}",
}


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


# ---------------------------------------------------------------------------
# the rules
# ---------------------------------------------------------------------------


def check_text(path, text):
    """Return (violations, stats). Raises ParseError for a file outside the subset."""
    doc = Parser(text).parse()
    problems = []

    def add(line, code, msg):
        problems.append("%s:%d: %s: %s" % (path, line, code, msg))

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
    wf_env = _get_map(doc, "env", "workflow")
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

    jobs_node = doc.get("jobs")
    if not isinstance(jobs_node, Map):
        raise ParseError(doc.line_of("jobs") if doc.has("jobs") else 1, "no `jobs` mapping")

    save_count = 0
    prune_count = 0
    for job_id in jobs_node.keys():
        job = Job(job_id, jobs_node.get(job_id))
        steps = job.steps
        has_prune = any(step.is_prune for step in steps)
        prune_count += sum(1 for step in steps if step.is_prune)

        # R4, job half.
        got = _permission_pairs(job.permissions, "job %s" % job_id) if job.permissions else {}
        if has_prune:
            if got != {"contents": "read", "actions": "write"}:
                add(
                    job.permissions_line,
                    "R4_PERMISSIONS",
                    "job %s prunes but its permissions are %s; expected exactly "
                    "{contents: read, actions: write}" % (job_id, _fmt(got)),
                )
        elif "actions" in got:
            add(
                job.permissions_line,
                "R4_PERMISSIONS",
                "job %s has no prune step but takes `actions: %s`; only a pruning job needs it"
                % (job_id, got["actions"]),
            )

        for index, step in enumerate(steps):
            # R3, the orphan half: a prune that guards nothing is a prune that
            # was left behind when its save moved or was deleted.
            if step.is_prune:
                nxt = steps[index + 1] if index + 1 < len(steps) else None
                if nxt is None or not nxt.is_save:
                    add(
                        step.line,
                        "R3_PRUNE_BEFORE_SAVE",
                        "step %r prunes but the step after it is %s; a prune step must sit "
                        "directly before the save it makes room for"
                        % (step.label(), "the end of the job" if nxt is None else repr(nxt.label())),
                    )
            if not step.is_save:
                continue
            save_count += 1
            cond = norm(step.cond)
            key = step.key
            if key is None:
                add(
                    step.line,
                    "R1_NAMESPACE_GATE",
                    "save step %r has no `with.key`" % step.label(),
                )
                continue

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

            # R7.
            if index != len(steps) - 1:
                add(
                    step.line,
                    "R7_SAVE_IS_LAST",
                    "save step %r is followed by %r; a save must be the last step of its job"
                    % (step.label(), steps[index + 1].label()),
                )

            # R3, the pairing half.
            if is_lockhash_key(key):
                _check_prune_before_save(add, steps, index, step, key)

        # R2 also applies to the prune steps: pruning off `main` without the
        # variable set would delete a cache nothing is about to replace.
        for step in steps:
            if not step.is_prune:
                continue
            if MAIN_ONLY_CLAUSE not in norm(step.cond):
                add(
                    step.cond_line,
                    "R2_MAIN_ONLY",
                    "prune step %r does not carry the clause %s"
                    % (step.label(), MAIN_ONLY_CLAUSE),
                )

    if save_count == 0:
        add(1, "NO_SAVE_STEPS", "no `actions/cache/save` step in this file; the policy checks "
                                "below have nothing to hold")

    return problems, {"saves": save_count, "prunes": prune_count}


def _check_namespace_gate(add, step, key):
    """R1."""
    cond = norm(step.cond)
    if not is_lockhash_key(key):
        # One key today has no lockfile hash (`cargo-machete-bin-...-v0.9.2`),
        # so it has no template to derive a token from. It must still be behind
        # the policy: the list alternative and the `all` escape, both present.
        if NS_ALL_CLAUSE not in cond or NS_CONTAINS_HEAD not in cond:
            add(
                step.cond_line,
                "R1_NAMESPACE_GATE",
                "save step %r is not behind the namespace policy; expected "
                "(%s || %s<token>)) in its `if`" % (step.label(), NS_ALL_CLAUSE, NS_CONTAINS_HEAD),
            )
        return
    wanted = [namespace_clause(token_fragment(t)) for t in namespace_templates(key)]
    if not any(clause in cond for clause in wanted):
        add(
            step.cond_line,
            "R1_NAMESPACE_GATE",
            "save step %r does not carry the namespace gate for its key; expected %s"
            % (step.label(), " or ".join(wanted)),
        )


def _check_prune_before_save(add, steps, index, step, key):
    """R3.

    A lockhash key with no `-main-`/`-pr-` scope (`cargo-cross-aarch64-<lock
    hash>`, the push-only namespaces) is accepted here and by
    `ci_cache_prune.sh`, which derives the namespace prefix from the hash
    boundary in that case; the two tools agree on every key shape the
    workflows carry. A key with no lockfile hash at all (the machete binary
    cache) needs no prune step and gets none.
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
    if prefix is not None and not key.startswith(prefix):
        add(
            prune.line,
            "R3_PRUNE_BEFORE_SAVE",
            "prune step %r sweeps prefix %r, which is not a leading substring of the save key %r"
            % (prune.label(), prefix, key),
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
    if save_cond != head + prune_cond:
        add(
            step.cond_line,
            "R3_PRUNE_BEFORE_SAVE",
            "save step %r runs under a different condition from the prune step before it: "
            "save %r, expected %r" % (label, save_cond, head + prune_cond),
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
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && matrix.shard == 0
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"

      - name: Cache cargo, Linux shard (save)
        if: >-
          steps.cache-restore-1.outputs.cache-hit != 'true'
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && matrix.shard == 0
          && github.event_name != 'merge_group'
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
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0}-lint ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-lint-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, lint (save)
        if: >-
          steps.cache-restore-lint.outputs.cache-hit != 'true'
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0}-lint ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
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
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' viz-{0}-{1} ', matrix.lane, runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-viz-${{ matrix.lane }}-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, viz (save)
        if: >-
          steps.cache-restore-viz.outputs.cache-hit != 'true'
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' viz-{0}-{1} ', matrix.lane, runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        uses: actions/cache/save@v4
        with:
          key: cargo-viz-${{ matrix.lane }}-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}

  cross:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    steps:
      - name: Cache cargo (restore)
        id: cache-restore-cross
        uses: actions/cache/restore@v4
        with:
          key: cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}
      - name: Cross build
        run: cargo build --locked --target aarch64-unknown-linux-gnu
      - name: Prune the cache namespace, cross
        if: >
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}"
      - name: Cache cargo, cross (save)
        if: >
          steps.cache-restore-cross.outputs.cache-hit != 'true'
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        uses: actions/cache/save@v4
        with:
          key: cargo-cross-aarch64-${{ hashFiles('**/Cargo.lock') }}

  lane:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      actions: write
    strategy:
      matrix:
        distro: [jazzy, humble]
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
      - name: Prune the cache namespace, rmw distro
        if: >-
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' rmw-distros-{0} ', matrix.distro)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-rmw-distros-${{ matrix.distro }}-${{ steps.headers.outputs.hash }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}" "cargo-rmw-distros-${{ matrix.distro }}-"
      - name: Cache cargo, rmw distro (save)
        if: >-
          steps.cache-restore-lane.outputs.cache-hit != 'true'
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' rmw-distros-{0} ', matrix.distro)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
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
          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' machete-bin-{0} ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && github.event_name != 'merge_group'
        uses: actions/cache/save@v4
        with:
          path: ~/.cargo/bin/cargo-machete
          key: cargo-machete-bin-${{ runner.os }}-v0.9.2
"""

STRAY_PRUNE_STEP = """      - name: Stray prune with nothing to guard
        if: >-
          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), format(' {0} ', runner.os)))
          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')
          && matrix.shard == 0
          && github.event_name != 'merge_group'
        env:
          GH_TOKEN: ${{ github.token }}
        run: bash tools/scripts/ci_cache_prune.sh "cargo-${{ runner.os }}-${{ github.ref == 'refs/heads/main' && 'main' || 'pr' }}-${{ hashFiles('**/Cargo.lock') }}"

"""


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


def _mutants():
    """(name, mutated text, expected rule codes). `None` codes means exit 2."""
    clean = CLEAN_WORKFLOW
    out = []

    # R1: the lint job's token no longer matches its own key template.
    out.append((
        "R1 wrong token",
        _mut(clean, "format(' {0}-lint ', runner.os)))", "format(' {0}-lintx ', runner.os)))",
             total=2, occurrence="all"),
        {"R1_NAMESPACE_GATE"},
    ))
    # R1: the shard job keeps the shard key but carries the lint job's token.
    # There is no `-<job id>` allowance, so the two must match exactly.
    out.append((
        "R1 shard key with the lint token",
        _mut(clean, "format(' {0} ', runner.os)))", "format(' {0}-lint ', runner.os)))",
             total=2, occurrence="all"),
        {"R1_NAMESPACE_GATE"},
    ))
    # R1: the `all` escape dropped from the literal-token job.
    out.append((
        "R1 missing the == 'all' alternative",
        _mut(
            clean,
            "(env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
            "env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))",
            "(contains(format(' {0} ', env.CACHE_SAVE_NAMESPACES), ' cross-aarch64 '))",
            total=2, occurrence="all",
        ),
        {"R1_NAMESPACE_GATE"},
    ))
    # R2 on a save. The save and prune conditions must stay in step, so removing
    # the clause from one side necessarily trips R3 as well.
    viz_frag = "format(' viz-{0}-{1} ', matrix.lane, runner.os)))"
    main_only_line = "          && (github.ref == 'refs/heads/main' || env.CACHE_SAVE_ON_PULL_REQUEST != '')\n"
    out.append((
        "R2 clause removed from a save",
        _mut(clean, "          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
                    "env.CACHE_SAVE_NAMESPACES), " + viz_frag + "\n" + main_only_line,
             "          && (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
             "env.CACHE_SAVE_NAMESPACES), " + viz_frag + "\n"),
        {"R2_MAIN_ONLY", "R3_PRUNE_BEFORE_SAVE"},
    ))
    out.append((
        "R2 clause removed from a prune",
        _mut(clean, "\n          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
                    "env.CACHE_SAVE_NAMESPACES), " + viz_frag + "\n" + main_only_line,
             "\n          (env.CACHE_SAVE_NAMESPACES == 'all' || contains(format(' {0} ', "
             "env.CACHE_SAVE_NAMESPACES), " + viz_frag + "\n"),
        {"R2_MAIN_ONLY", "R3_PRUNE_BEFORE_SAVE"},
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
        "R3 save if is not cache-hit plus the prune if",
        _mut(clean,
             "          && matrix.shard == 0\n          && github.event_name != 'merge_group'\n"
             "        uses: actions/cache/save@v4\n",
             "          && matrix.shard == 0\n          && github.event_name != 'merge_group'\n"
             "          && true\n        uses: actions/cache/save@v4\n"),
        {"R3_PRUNE_BEFORE_SAVE"},
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
    out.append((
        "R6 merge-queue clause removed",
        _mut(clean,
             "' cross-aarch64 '))\n" + main_only_line
             + "          && github.event_name != 'merge_group'\n",
             "' cross-aarch64 '))\n" + main_only_line,
             total=2, occurrence="all"),
        {"R6_NO_QUEUE_SAVE"},
    ))
    out.append((
        "R7 a step appended after the save",
        clean + "      - name: Report\n        run: echo done\n",
        {"R7_SAVE_IS_LAST"},
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
             total=6, occurrence=1),
        None,
    ))
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
    if stats != {"saves": 6, "prunes": 5}:
        fails.append("clean workflow parsed to %r, expected 6 saves and 5 prunes" % (stats,))

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
        if not problems:
            fails.append("%s: mutant passed; expected %s" % (name, sorted(expected)))
            continue
        got = _codes(problems)
        if got != expected:
            fails.append(
                "%s: reported %s, expected %s\n    %s"
                % (name, sorted(got), sorted(expected), "\n    ".join(problems))
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
    "usage: python3 tools/scripts/ci_cache_policy_check.py <workflow.yml>...\n"
    "       python3 tools/scripts/ci_cache_policy_check.py --self-test\n"
)


def main(argv):
    if len(argv) == 1 and argv[0] == "--self-test":
        return self_test()
    if not argv or any(a.startswith("-") for a in argv):
        sys.stderr.write(USAGE)
        return 2
    violations = []
    for path in argv:
        try:
            with open(path, "r") as handle:
                text = handle.read()
        except OSError as exc:
            sys.stderr.write("%s: cannot read: %s\n" % (path, exc))
            return 2
        try:
            problems, _ = check_text(path, text)
        except ParseError as exc:
            sys.stderr.write("%s:%d: PARSE_ERROR: %s\n" % (path, exc.line, exc.msg))
            return 2
        violations.extend(problems)
    if violations:
        for line in violations:
            sys.stdout.write(line + "\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
