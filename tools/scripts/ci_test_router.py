#!/usr/bin/env python3
"""ci_test_router.py: the test binaries a change can affect, in SHADOW MODE.

    python3 -B tools/scripts/ci_test_router.py --base BASE --head HEAD [--json OUT]
    python3 -B tools/scripts/ci_test_router.py --self-test
    python3 -B tools/scripts/ci_test_router.py --live-check

SHADOW MODE. This script computes a selection and nothing reads it: no job,
step or required context is gated on its answer. A binary that failed in the
full run while this selection skipped it is an ESCAPE, and routing stays off
until the shadow period shows none.

WHAT IT ANSWERS. Given a base commit and a head commit it lists every test
binary of the root workspace (package plus target, named the way nextest names
them: `pkg` for the library's unit tests, `pkg::name` for an integration test,
`pkg::bin/name`, `pkg::example/name`, `pkg::bench/name`, and
`pkg::doctest/<lib>` for the doctests) and says, for each one, whether the
change can affect it and why. Separate workspaces under the tree (`examples/*`,
`benches/*`, `crates/cerulion_py`, the fuzz and Android runtime workspaces) are
listed as one unit each.

THE RULE. A binary runs when a GLOBAL input changed, or when the diff touches
one of ITS inputs. Its inputs are:

  1. Its own sources: the target file, every module file reachable through
     `mod` (with `#[path]`, inline modules and `#[cfg(test)]` tracked), and
     every file named by `include!`, `include_str!` or `include_bytes!` with a
     literal or `concat!(env!("CARGO_MANIFEST_DIR"), "...")` path. This is a
     STATIC APPROXIMATION of rustc's dep-info, computed from the source text so
     the job needs no build. A `mod` or `include` it cannot resolve widens that
     target to every file under its package directory.
  2. Its crate closure: the normal and build closure of its package, plus the
     closure of each dev-dependency the binary names in its own sources (one
     hop; every dev-dependency for a `trybuild` driver, which compiles cases
     that name crates the driver never does). A library's build inputs are
     its build script, the files the script names in `rerun-if-changed` (the
     whole package directory when it names none or computes one) and the
     literal paths it opens.
  3. Every binary it spawns, cdylib it loads, and nested cargo build it runs,
     with the full inputs of each, TRANSITIVELY. A spawn found in LIBRARY code
     belongs to every binary that links that library:
     `crates/cerulion_netd/src/client.rs` spawns `cerulion-netd`,
     `crates/cerulion_cli_engine/src/viz_client.rs` spawns `cerulion-vizd`,
     and `crates/cerulion_cli_engine/src/doctor/daemons.rs` looks for
     `cerulion-remoted` beside the running executable. A spawned binary brings
     its own sources (`crates/cerulion_netd/src/main.rs`) and its own spawns.
     `crates/cerulion_android/src/runtime.rs` runs its recorder, gateway and
     graph as threads named like daemons: library code the closure carries,
     not a spawn. A binary's name is a spawn where `Command`, an `exec`, or
     `current_exe` sits in the same file, or where a `const` or `static` bound
     to it is named in a file of the context that runs a command.
  4. The files it reads at run time: the observation edges in
     `tools/ci/observation_edges.tsv` and the doc-pin markers in `ci.yml`,
     narrowed to the binaries whose own sources carry the edge's shape, plus
     its own package tree when its sources name a path, and every file inside
     its package a literal names (a test's working directory is its package).
  5. Whole-tree walkers, unattributable readers, and binaries whose own
     sources run cargo (or drive the CLI's `build` verb) always run: an
     unlocked resolve reads crates.io, which no input key covers. Library code
     that runs cargo on a scaffold adds the umbrella crate's closure, which a
     scaffold links through a path dependency, to every binary that links it.

WHAT THE STATIC READING DOES NOT SEE. A doctest's body sits in a comment, so
its spawns and reads are not scanned: a doctest runs when its library, or any
dev-dependency, changes. A path or a binary name assembled at run time, a
crate reference a macro from another crate expands to, and a file a test finds
through `PATH` are not edges here either. The shadow period and the escape
scorer are what measure those misses.

MEMBER MANIFESTS. A member's `Cargo.toml` change selects every binary of every
package whose test build includes that member (its normal, build and EVERY
dev-dependency closure, not the per-binary refinement), every binary of a
package that shares a `cargo test` command with one in `ci.yml`, and every
binary that spawns or loads a workspace artifact, because `ci.yml` builds those
with `cargo build --workspace` and features unify across the workspace there.
That is why a `crates/cerulion_cli_engine/Cargo.toml` change selects all of
`cerulion_core` (core dev-depends on cli_engine, and cli_engine's features merge
into the core library every core test links), while a cli_engine `.rs` change
selects only the core binaries that name cli_engine. No comment-only refinement
applies to a member manifest: `crates/cerulion_macros/src/crate_root.rs` reads the
CONSUMER's `Cargo.toml` with `std::fs` while it expands, so the manifest's bytes
are a compile input of every crate that expands the macro, and rustc's dep-info
does not record that read.

GLOBAL INPUTS select everything:

  * `Cargo.lock`, NARROWED: the entries that differ, then every member whose
    normal or build dependency reaches one through the resolved graph (its
    library changes, and every binary that spawns or loads a workspace
    artifact runs, since a new version can request other features of a crate
    `cargo build --workspace` shares) and every member that dev-depends on
    one (all of its binaries). A lock this script cannot parse is global.
  * root `Cargo.toml`: any change to a table's content (`[workspace]`,
    `[workspace.*]`, `[patch.*]`, `[profile.*]`, or any other) is global; a
    change to comments and blank lines alone selects the binaries that read
    the manifest's text.
  * `.github/workflows/ci.yml`, `.github/actions/**`, `.cargo/**`, `.config/**`,
    `rust-toolchain*`;
  * this script, `tools/ci/observation_edges.tsv`, the path classifier and its
    selector, and every `tools/` path a test job in `ci.yml` names;
  * metadata this script cannot reconcile with the head tree (a target file
    the head lacks, or a file cargo would make a target that the metadata
    does not list), and any path no rule claims (fail closed).

OUTPUT. A summary on stdout, and with `--json` the selection as one JSON
document: `selected` (each binary with its reasons), `skipped`, `global` (the
triggers), the foreign workspace units, the dirty packages with their first
cause, and `built_packages`, the packages the selected binaries compile, which
the escape scorer reads to judge a library that failed to compile. The document
is a function of the two commits alone: no clock, no hash order, no random.

`--self-test` reads only its hand-built fixture. `--live-check` reads this
checkout and pins shapes the real tree keeps; it runs in no CI job.

Stdlib only. Python 3.8 or newer. Exit 0 with an answer, 1 when the self-test
or the live check found a miss, 2 on a usage error or a repository it cannot
read.
"""

import argparse
import bisect
import contextlib
import io
import json
import os
import posixpath
import re
import subprocess
import sys

SCHEMA_VERSION = 1

# This script's own path, relative to the repository root: a change to it is a
# global input.
ROUTER_REL = 'tools/scripts/ci_test_router.py'

# Paths whose change selects everything, matched exactly or as a prefix when
# they end in `/`. The `tools/` paths a test job in ci.yml names are added to
# this list at run time.
GLOBAL_PATHS = (
    ('.github/workflows/ci.yml', 'ci.yml'),
    ('.github/actions/', 'composite-action'),
    ('.cargo/', '.cargo'),
    ('.config/', '.config'),
    (ROUTER_REL, 'router'),
    ('tools/ci/observation_edges.tsv', 'router-input'),
    ('tools/scripts/ci_changed_paths.sh', 'ci-machinery'),
    ('tools/scripts/ci_selected_packages.py', 'ci-machinery'),
    ('tools/scripts/ci_test_shard.sh', 'ci-machinery'),
)

# The shared trees outside every member, and the root markdown files, that a
# test may read at run time. A change under one selects the binaries that read
# it (doc-pin markers and this script's own literal scan) and the whole-tree
# walkers. The names match `ci_doc_pin_walk_test`'s DOC_ROOT_DIRS and
# ROOT_MARKDOWN_STEMS.
DOC_ROOT_DIRS = ('.github', 'benches', 'docs', 'examples', 'harness', 'tools')
ROOT_MARKDOWN = ('AGENTS.md', 'CHANGELOG.md', 'CLAUDE.md', 'README.md')

CI_WORKFLOW = '.github/workflows/ci.yml'
# The CLI binary, and the umbrella crate a scaffold built through it links.
CLI_BINARY = 'cerulion'
UMBRELLA = 'cerulion'
OBSERVATION_EDGES = 'tools/ci/observation_edges.tsv'
EVERYTHING = 'all'
# The four row kinds `ci_doc_pin_walk_test` derives.
EDGE_KINDS = ('crate-path', 'dlopen', 'whole-tree', 'unattributable')

# Words in code that make a string naming a workspace binary a SPAWN.
SPAWN_WORDS = frozenset(('Command', 'current_exe', 'exec', 'execv', 'execvp', 'execve',
                         'posix_spawn', 'cargo_bin'))
# Words that make a test read its own package tree at run time.
OWN_TREE_WORDS = frozenset(('read_dir', 'canonicalize', 'current_dir', 'WalkDir',
                            'walkdir', 'glob', 'include_dir'))
ROOT_WORDS = frozenset(('repo_root', 'workspace_root'))
# Words of a test that compiles sources at run time against the package's
# dependencies.
COMPILES_AT_RUN_TIME = frozenset(('trybuild',))
CLIMB_WORDS = frozenset(('parent', 'ancestors', 'pop'))
OWN_TREE_PREFIXES = ('tests/', 'src/', 'benches/', 'examples/', './')


class RouterError(Exception):
    """A repository or document this script cannot read: exit 2."""


# ---------------------------------------------------------------------------
# Trees: the files at one commit
# ---------------------------------------------------------------------------


class GitTree:
    """The files of one commit, read through `git cat-file --batch`."""

    def __init__(self, repo, rev, reader):
        self.rev = rev
        self.reader = reader
        try:
            out = subprocess.run(
                ['git', '-C', repo, 'ls-tree', '-r', '-z', '--full-tree', rev],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).stdout
        except subprocess.CalledProcessError as err:
            raise RouterError('cannot list the tree of %s: %s'
                              % (rev, err.stderr.decode('utf-8', 'replace').strip()))
        self.blobs = {}
        for entry in out.split(b'\0'):
            if not entry:
                continue
            meta, path = entry.split(b'\t', 1)
            parts = meta.split()
            # A submodule (`commit`) has no blob to read; it is still a path.
            self.blobs[path.decode('utf-8', 'surrogateescape')] = (
                parts[2].decode() if parts[1] == b'blob' else None)
        self.dirs = set()
        for path in self.blobs:
            parent = posixpath.dirname(path)
            while parent and parent not in self.dirs:
                self.dirs.add(parent)
                parent = posixpath.dirname(parent)

    def exists(self, path):
        return path in self.blobs

    def is_dir(self, path):
        return path in self.dirs

    def blob(self, path):
        return self.blobs.get(path)

    def read(self, path):
        sha = self.blobs.get(path)
        if sha is None:
            return None
        return self.reader.read(sha)

    def files(self):
        return sorted(self.blobs)


class BlobReader:
    """One `git cat-file --batch` process, shared by every tree of a repo."""

    def __init__(self, repo):
        self.proc = subprocess.Popen(['git', '-C', repo, 'cat-file', '--batch'],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.cache = {}

    def read(self, sha):
        if sha in self.cache:
            return self.cache[sha]
        self.proc.stdin.write(sha.encode() + b'\n')
        self.proc.stdin.flush()
        header = self.proc.stdout.readline().split()
        if len(header) != 3 or header[1] != b'blob':
            raise RouterError('git cat-file returned %r for %s' % (header, sha))
        size = int(header[2])
        data = self.proc.stdout.read(size)
        self.proc.stdout.read(1)
        text = data.decode('utf-8', 'replace')
        self.cache[sha] = text
        return text

    def close(self):
        if self.proc.stdin:
            self.proc.stdin.close()
        self.proc.wait()


class DictTree:
    """An in-memory tree for the self-test: `{path: text}`."""

    def __init__(self, files):
        self.data = dict(files)
        self.dirs = set()
        for path in self.data:
            parent = posixpath.dirname(path)
            while parent:
                self.dirs.add(parent)
                parent = posixpath.dirname(parent)

    def exists(self, path):
        return path in self.data

    def is_dir(self, path):
        return path in self.dirs

    def blob(self, path):
        if path not in self.data:
            return None
        return 'dict:%d:%s' % (len(self.data[path]), hash_text(self.data[path]))

    def read(self, path):
        return self.data.get(path)

    def files(self):
        return sorted(self.data)


def hash_text(text):
    """A deterministic content key (Python's `hash` of a str is salted)."""
    value = 1469598103934665603
    for byte in text.encode('utf-8'):
        value = ((value ^ byte) * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return '%016x' % value


# ---------------------------------------------------------------------------
# Rust source scanning
# ---------------------------------------------------------------------------

# The next token that opens a comment, a string or a char literal. `\b` keeps
# `r#ident` and `br"` apart from an identifier that merely ends in `r` or `b`.
_OPENER = re.compile(r'//|/\*|\b(?:br|cr|r)(#*)"|\b[bc]"|"|\'')
_STR_BODY = re.compile(r'(?:[^"\\]|\\.)*"', re.S)
_CHAR = re.compile(r"'(?:[^'\\\n]|\\(?:x[0-9a-fA-F]{2}|u\{[0-9a-fA-F]{1,6}\}|.))'")
_ESCAPE = re.compile(r'\\(\n\s*|.)', re.S)
_WORD = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
_MOD = re.compile(r'\bmod\s+(?:r#)?([A-Za-z_][A-Za-z0-9_]*)\s*([;{])')
_INCLUDE = re.compile(r'\b(include|include_str|include_bytes)\s*!\s*\(')
_FILE_MACRO = re.compile(r'\bfile\s*!\s*\(')
_CONST_STR = re.compile(r"\b(?:const|static)\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*&\s*(?:'static\s+)?"
                        r'str\s*=\s*(?:br|cr|r)?#*"')
_ESCAPES = {'n': '\n', 't': '\t', 'r': '\r', '0': '\0', '\\': '\\', '"': '"', "'": "'"}


def _unescape(body):
    def repl(match):
        tail = match.group(1)
        if tail[0] == '\n':
            return ''
        return _ESCAPES.get(tail, tail)
    return _ESCAPE.sub(repl, body)


def lex(text):
    """Return `(code, literals)`.

    `code` is `text` with every comment blanked and every literal's body
    blanked (its quotes kept), so offsets line up with `text` and braces,
    `mod` and macro names in code are found without a false hit inside a
    string. `literals` is `[(start, end, value)]`, start at the literal's first
    character, value with escapes applied (raw strings verbatim).
    """
    out = []
    literals = []
    pos = 0
    length = len(text)
    while True:
        match = _OPENER.search(text, pos)
        if match is None:
            break
        start = match.start()
        token = match.group(0)
        if token == '//':
            end = text.find('\n', start)
            end = length if end < 0 else end
            out.append(text[pos:start])
            out.append(' ' * (end - start))
            pos = end
            continue
        if token == '/*':
            depth = 1
            cursor = start + 2
            while depth and cursor < length:
                nxt_open = text.find('/*', cursor)
                nxt_close = text.find('*/', cursor)
                if nxt_close < 0:
                    cursor = length
                    break
                if 0 <= nxt_open < nxt_close:
                    depth += 1
                    cursor = nxt_open + 2
                else:
                    depth -= 1
                    cursor = nxt_close + 2
            out.append(text[pos:start])
            out.append(re.sub(r'[^\n]', ' ', text[start:cursor]))
            pos = cursor
            continue
        if token == "'":
            char = _CHAR.match(text, start)
            if char is None:
                # A lifetime or a label: one quote and nothing to blank.
                out.append(text[pos:start + 1])
                pos = start + 1
                continue
            end = char.end()
            out.append(text[pos:start + 1])
            out.append(' ' * (end - start - 2))
            out.append("'")
            pos = end
            continue
        # A string literal of one of the four shapes.
        body_start = match.end()
        if match.group(1) is not None:
            closing = '"' + match.group(1)
            close = text.find(closing, body_start)
            if close < 0:
                close = length
            value = text[body_start:close]
            end = min(length, close + len(closing))
        else:
            body = _STR_BODY.match(text, body_start)
            if body is None:
                end = length
                value = text[body_start:]
            else:
                end = body.end()
                value = _unescape(text[body_start:end - 1])
        literals.append((start, end, value))
        out.append(text[pos:body_start])
        out.append(re.sub(r'[^\n]', ' ', text[body_start:max(body_start, end - 1)]))
        out.append(text[max(body_start, end - 1):end])
        pos = end
    out.append(text[pos:])
    return ''.join(out), literals


def _brace_pairs(code):
    pairs = {}
    stack = []
    for match in re.finditer(r'[{}]', code):
        if match.group(0) == '{':
            stack.append(match.start())
        elif stack:
            pairs[stack.pop()] = match.start()
    return pairs


def _attribute_window(code, at):
    """The attribute text directly in front of the item that starts at `at`."""
    cut = max(code.rfind(';', 0, at), code.rfind('{', 0, at), code.rfind('}', 0, at))
    return cut + 1, at


def _balanced(code, open_at):
    depth = 0
    for index in range(open_at, len(code)):
        char = code[index]
        if char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth == 0:
                return index
    return len(code) - 1


def _cfg_is_test(code, start, end):
    """Does a `#[cfg(...)]` in `code[start:end]` hold only under `cfg(test)`?

    `test` alone, or `all(...)` with `test` as one of its top-level terms.
    `any(test, ...)` and `not(test)` hold outside tests too, so they do not.
    """
    window = code[start:end]
    for match in re.finditer(r'#\s*\[\s*cfg\s*\(', window):
        close = _balanced(window, match.end() - 1)
        predicate = re.sub(r'\s+', '', window[match.end():close])
        if predicate == 'test':
            return True
        if predicate.startswith('all(') and predicate.endswith(')'):
            terms = _split_top(predicate[4:-1])
            if 'test' in terms:
                return True
    return False


def _split_top(text):
    terms, depth, current = [], 0, []
    for char in text:
        if char == ',' and depth == 0:
            terms.append(''.join(current))
            current = []
            continue
        if char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
        current.append(char)
    terms.append(''.join(current))
    return terms


class FileScan:
    """What one Rust source declares and names, split by `cfg(test)` spans."""

    __slots__ = ('mods', 'includes', 'lits_normal', 'lits_test', 'words_normal',
                 'words_test', 'file_macro_normal', 'file_macro_test', 'consts',
                 'root_args_normal', 'root_args_test')

    def __init__(self):
        self.mods = []
        self.includes = []
        self.lits_normal = set()
        self.lits_test = set()
        self.words_normal = set()
        self.words_test = set()
        self.file_macro_normal = False
        self.file_macro_test = False
        self.consts = []
        # Does the code hand the REPOSITORY ROOT to a call as an argument?
        self.root_args_normal = False
        self.root_args_test = False


class ModDecl:
    __slots__ = ('name', 'paths', 'keeps_default', 'test_only', 'inline_chain', 'inline_paths')

    def __init__(self, name, paths, keeps_default, test_only, inline_chain, inline_paths):
        self.name = name
        self.paths = paths
        self.keeps_default = keeps_default
        self.test_only = test_only
        self.inline_chain = inline_chain
        self.inline_paths = inline_paths


def scan_rust(text):
    """Scan one Rust source. Pure: the same text always gives the same scan."""
    code, literals = lex(text)
    scan = FileScan()
    lit_at = {}
    for start, end, value in literals:
        lit_at[start] = (end, value)
    starts = sorted(lit_at)

    def literals_between(lo, hi):
        index = bisect.bisect_left(starts, lo)
        found = []
        while index < len(starts) and starts[index] < hi:
            found.append((starts[index],) + lit_at[starts[index]])
            index += 1
        return found

    # Inline modules: name, span, and whether `cfg(test)` gates them.
    pairs = None
    inline = []
    for match in _MOD.finditer(code):
        if match.group(2) != '{':
            continue
        if pairs is None:
            pairs = _brace_pairs(code)
        open_at = match.end() - 1
        close_at = pairs.get(open_at, len(code))
        lo, hi = _attribute_window(code, match.start())
        test = _cfg_is_test(code, lo, hi)
        path_attr, _ = _path_attrs(code, lo, hi, literals_between(lo, hi))
        inline.append((match.group(1), open_at, close_at, test, tuple(path_attr)))
    test_spans = [(o, c) for _, o, c, t, _ in inline if t]
    test_spans += _test_item_spans(code, pairs if pairs is not None else None)

    def in_test(at):
        for lo, hi in test_spans:
            if lo < at < hi:
                return True
        return False

    def chain_at(at):
        names, attrs = [], []
        for name, lo, hi, _, path_attr in inline:
            if lo < at < hi:
                names.append(name)
                attrs.append(path_attr[0] if path_attr else None)
        return tuple(names), tuple(attrs)

    for match in _MOD.finditer(code):
        if match.group(2) != ';':
            continue
        lo, hi = _attribute_window(code, match.start())
        paths, keeps_default = _path_attrs(code, lo, hi, literals_between(lo, hi))
        test = _cfg_is_test(code, lo, hi) or in_test(match.start())
        names, attrs = chain_at(match.start())
        scan.mods.append(ModDecl(match.group(1), tuple(paths), keeps_default, test,
                                 names, attrs))

    for match in _INCLUDE.finditer(code):
        open_at = match.end() - 1
        close_at = _balanced(code, open_at)
        inner = code[open_at + 1:close_at]
        lits = literals_between(open_at, close_at)
        test = in_test(match.start())
        kind = match.group(1)
        stripped = re.sub(r'\s+', '', inner)
        if len(lits) == 1 and re.fullmatch(r'(?:br|cr|r)?#*""#*,?', stripped) is not None:
            scan.includes.append((kind, 'file', lits[0][2], test))
        elif stripped.startswith('concat!(') and lits:
            values = [value for _, _, value in lits]
            if values[0] == 'CARGO_MANIFEST_DIR' and 'env!(' in stripped:
                scan.includes.append((kind, 'manifest', ''.join(values[1:]), test))
            elif values[0] == 'OUT_DIR' and 'env!(' in stripped:
                scan.includes.append((kind, 'out_dir', ''.join(values[1:]), test))
            else:
                scan.includes.append((kind, 'dynamic', inner.strip(), test))
        elif '$' in inner:
            # A `macro_rules!` body: the path is the macro's argument, and the
            # call sites pass literals this scan sees as ordinary literals.
            scan.includes.append((kind, 'macro-arg', inner.strip(), test))
        else:
            scan.includes.append((kind, 'dynamic', inner.strip(), test))

    for start, end, value in literals:
        (scan.lits_test if in_test(start) else scan.lits_normal).add(value)
    for match in _CONST_STR.finditer(code):
        found = lit_at.get(match.end() - 1)
        if found is None:
            for prefix in range(match.end() - 1, max(-1, match.end() - 6), -1):
                if prefix in lit_at:
                    found = lit_at[prefix]
                    break
        if found is not None:
            scan.consts.append((match.group(1), found[1]))
    for at in _root_argument_offsets(code, literals):
        if in_test(at):
            scan.root_args_test = True
        else:
            scan.root_args_normal = True

    if test_spans:
        normal_parts, test_parts, cursor = [], [], 0
        for lo, hi in sorted(test_spans):
            if lo < cursor:
                continue
            normal_parts.append(code[cursor:lo])
            test_parts.append(code[lo:hi])
            cursor = hi
        normal_parts.append(code[cursor:])
        normal_code = ' '.join(normal_parts)
        test_code = ' '.join(test_parts)
    else:
        normal_code, test_code = code, ''
    scan.words_normal = set(_WORD.findall(normal_code))
    scan.words_test = set(_WORD.findall(test_code))
    scan.file_macro_normal = _FILE_MACRO.search(normal_code) is not None
    scan.file_macro_test = _FILE_MACRO.search(test_code) is not None
    return scan


_ROOT_CALL = re.compile(r'\b(?:repo_root|workspace_root)\s*\(\s*\)')
_BINDING = re.compile(r'\b(?:let|const|static)\s+(?:mut\s+)?([A-Za-z_][A-Za-z0-9_]*)[^=;]*=')
_CLIMB = re.compile(r'\.(?:parent|ancestors)\s*\(')


def _root_argument_offsets(code, literals):
    """Offsets where a source hands the repository root to a call.

    A SUPERSET of the shape `ci_doc_pin_walk_test` derives a `whole-tree` row
    from: a `repo_root()` or `workspace_root()` call, or a name bound to one
    or to a climb out of `CARGO_MANIFEST_DIR` (or to the manifest directory a
    later `.pop()` climbs), used as a call ARGUMENT (after `(` or `,`, through
    `&` or `&mut`) and not as the receiver of `.join(` or a declaration. The
    walk also excuses calls that only print or compare the root; this reader
    excuses none, so it can only flag more.
    """
    manifest_lits = [start for start, _, value in literals if value == 'CARGO_MANIFEST_DIR']
    dots = {start for start, _, value in literals if value and set(value) <= {'.', '/'}}
    names = set()
    for match in _BINDING.finditer(code):
        end = code.find(';', match.end())
        end = len(code) if end < 0 else end
        init = code[match.end():end]
        if _ROOT_CALL.search(init):
            names.add(match.group(1))
            continue
        lo, hi = match.end(), end
        if any(lo <= at < hi for at in manifest_lits):
            climbs = _CLIMB.search(init) or any(lo <= at < hi for at in dots)
            popped = re.search(r'\b%s\s*\.\s*pop\s*\(' % re.escape(match.group(1)), code)
            if climbs or popped:
                names.add(match.group(1))
    out = []
    patterns = [_ROOT_CALL]
    if names:
        patterns.append(re.compile(r'\b(?:%s)\b' % '|'.join(sorted(re.escape(n) for n in names))))
    for pattern in patterns:
        for match in pattern.finditer(code):
            after = code[match.end():].lstrip()
            if after.startswith('.join(') or after.startswith(':') or after.startswith('='):
                continue
            before = code[:match.start()].rstrip()
            before = re.sub(r'&\s*(?:mut\s+)?$', '', before).rstrip()
            if before.endswith('(') or before.endswith(','):
                out.append(match.start())
    return sorted(out)


_TEST_ATTR = re.compile(r'#\s*\[\s*(?:test|cfg\s*\(\s*test\s*\))\s*\]')


_ITEM_KEYWORDS = frozenset(('fn', 'mod', 'impl', 'use', 'const', 'static', 'struct', 'enum',
                            'trait', 'type', 'macro_rules', 'extern', 'unsafe', 'async',
                            'union'))
_ATTR_OR_VIS = re.compile(r'\s*(?:#\s*\[|pub\b\s*(?:\([^)]*\))?)')
_NEXT_WORD = re.compile(r'\s*([A-Za-z_][A-Za-z0-9_]*)(!?)')


def _gated_item_start(code, at):
    """The keyword of the item an attribute ending at `at` gates, or None.

    Further attributes and a visibility are skipped. A `#[cfg(test)]` on a
    struct field, an enum variant, a match arm or an expression gates no item:
    reading it as one would mark the next brace block (often a whole `impl`)
    test-only and drop what that code does from every linker.
    """
    cursor = at
    while True:
        match = _ATTR_OR_VIS.match(code, cursor)
        if match is None or match.end() == cursor:
            break
        if match.group(0).rstrip().endswith('['):
            depth, index = 1, match.end()
            while index < len(code) and depth:
                if code[index] == '[':
                    depth += 1
                elif code[index] == ']':
                    depth -= 1
                index += 1
            cursor = index
        else:
            cursor = match.end()
    word = _NEXT_WORD.match(code, cursor)
    if word is None:
        return None
    if word.group(1) == 'macro_rules' and word.group(2):
        return 'macro_rules'
    return word.group(1) if word.group(1) in _ITEM_KEYWORDS else None


def _test_item_spans(code, pairs):
    """Spans of the ITEMS a `#[test]` or `#[cfg(test)]` attribute gates.

    The item runs from the attribute to the close of its first brace block,
    or to its `;` when it has none (`#[cfg(test)] use ...;`). An attribute on
    anything that is not an item gates no span (see `_gated_item_start`). A
    `cfg(test)` inline module is found by the caller as well; finding it twice
    is no harm.
    """
    spans = []
    for match in _TEST_ATTR.finditer(code):
        if _gated_item_start(code, match.end()) is None:
            continue
        brace = code.find('{', match.end())
        semi = code.find(';', match.end())
        if brace < 0 or (0 <= semi < brace):
            end = semi if semi >= 0 else len(code)
            spans.append((match.start(), end))
            continue
        if pairs is None:
            pairs = _brace_pairs(code)
        spans.append((match.start(), pairs.get(brace, len(code))))
    return spans


def _path_attrs(code, lo, hi, lits):
    """The `#[path = "..."]` values in an attribute window.

    Returns `(paths, keeps_default)`. A path set through `cfg_attr` applies
    only under its condition, so the default location stays a candidate.
    """
    paths = []
    keeps_default = True
    for start, _, value in lits:
        before = code[lo:start]
        if re.search(r'\bpath\s*=\s*$', before):
            paths.append(value)
            if re.search(r'#\s*\[\s*cfg_attr\s*\(', before) is None:
                keeps_default = False
    return paths, keeps_default


# ---------------------------------------------------------------------------
# TOML: just enough for Cargo.lock and the root manifest
# ---------------------------------------------------------------------------


def strip_toml_comment(line):
    out, quote, index = [], None, 0
    while index < len(line):
        char = line[index]
        if quote:
            out.append(char)
            if quote == '"' and char == '\\' and index + 1 < len(line):
                out.append(line[index + 1])
                index += 2
                continue
            if char == quote:
                quote = None
        elif char in '"\'':
            quote = char
            out.append(char)
        elif char == '#':
            break
        else:
            out.append(char)
        index += 1
    return ''.join(out).strip()


def toml_tables(text):
    """`{table header: [normalised content lines]}`, comments dropped.

    Refuses multi-line strings: a line inside one can look like a header, and
    a reader that guessed would call a semantic change a comment.
    """
    if '"""' in text or "'''" in text:
        raise RouterError('a multi-line TOML string; this reader refuses it')
    tables = {}
    current = ''
    tables[current] = []
    for raw in text.splitlines():
        line = strip_toml_comment(raw)
        if not line:
            continue
        header = re.fullmatch(r'\[\[?\s*([^\[\]]+?)\s*\]\]?', line)
        if header:
            current = line
            tables.setdefault(current, [])
            if line.startswith('[['):
                tables[current].append('--- new array entry ---')
            continue
        tables[current].append(re.sub(r'\s+', ' ', line))
    return tables


def parse_lock(text):
    """`{(name, version, source): {field: value}}` for every `[[package]]`.

    Raises RouterError on anything outside the Cargo.lock shape.
    """
    packages = {}
    header = []
    current = None
    array_key = None
    array = None
    for number, raw in enumerate(text.splitlines(), 1):
        line = strip_toml_comment(raw)
        if not line:
            continue
        if array is not None:
            if line == ']':
                current[array_key] = tuple(array)
                array = None
                continue
            item = re.fullmatch(r'"((?:[^"\\]|\\.)*)",?', line)
            if item is None:
                raise RouterError('Cargo.lock:%d: unexpected array item %r' % (number, line))
            array.append(item.group(1))
            continue
        if line == '[[package]]':
            current = {}
            header.append(('package', len(packages) + len(header)))
            packages.setdefault('__entries__', []).append(current)
            continue
        if line.startswith('['):
            # `[metadata]`, `[[patch.unused]]` and any future table: keep its
            # text so a change to it is a change, and refuse nothing.
            current = {}
            packages.setdefault('__other__', []).append((line, current))
            continue
        pair = re.fullmatch(r'([A-Za-z0-9_-]+)\s*=\s*(.*)', line)
        if pair is None:
            raise RouterError('Cargo.lock:%d: unexpected line %r' % (number, line))
        key, value = pair.group(1), pair.group(2)
        if current is None:
            header.append((key, value))
            continue
        if value == '[':
            array_key, array = key, []
            continue
        inline = re.fullmatch(r'\[(.*)\]', value)
        if inline:
            current[key] = tuple(re.findall(r'"((?:[^"\\]|\\.)*)"', inline.group(1)))
            continue
        string = re.fullmatch(r'"((?:[^"\\]|\\.)*)"', value)
        current[key] = string.group(1) if string else value
    if array is not None:
        raise RouterError('Cargo.lock: an array never closes')
    entries = {}
    for entry in packages.get('__entries__', []):
        if 'name' not in entry or 'version' not in entry:
            raise RouterError('Cargo.lock: a [[package]] without name and version')
        key = (entry['name'], entry['version'], entry.get('source', ''))
        if key in entries:
            raise RouterError('Cargo.lock: %s %s listed twice' % key[:2])
        entries[key] = entry
    other = tuple((line, tuple(sorted(fields.items())))
                  for line, fields in packages.get('__other__', []))
    head = tuple(item for item in header if item[0] != 'package')
    return entries, (head, other)


def resolve_lock_dep(spec, by_name):
    """The lock keys one `dependencies` entry names."""
    match = re.fullmatch(r'(\S+)(?: (\S+))?(?: \((.+)\))?', spec)
    if match is None:
        raise RouterError('Cargo.lock: unreadable dependency %r' % spec)
    name, version, source = match.groups()
    keys = by_name.get(name, [])
    if version is not None:
        keys = [key for key in keys if key[1] == version]
    if source is not None:
        keys = [key for key in keys if key[2] == source]
    if not keys:
        raise RouterError('Cargo.lock: %r names no package' % spec)
    return keys


# ---------------------------------------------------------------------------
# The workspace model
# ---------------------------------------------------------------------------


class Target:
    __slots__ = ('package', 'name', 'kind', 'src', 'test', 'doctest', 'unit_id')

    def __init__(self, package, name, kind, src, test, doctest):
        self.package = package
        self.name = name
        self.kind = kind
        self.src = src
        self.test = test
        self.doctest = doctest
        self.unit_id = None


class Package:
    def __init__(self, name, directory):
        self.name = name
        self.dir = directory
        self.lib = None
        self.targets = []
        self.build_script = None
        # member name -> set of kinds ('normal', 'dev', 'build')
        self.member_deps = {}
        # every dependency name (registry and member) -> set of kinds
        self.dep_kinds = {}
        # crate identifier in source -> member name, for dev-dependencies
        self.dev_idents = {}
        self.proc_macro = False
        self.cdylib = False


LIB_KINDS = ('lib', 'rlib', 'dylib', 'cdylib', 'staticlib', 'proc-macro')


def build_model(metadata):
    """Packages of the root workspace from `cargo metadata --no-deps`."""
    if not isinstance(metadata, dict) or not isinstance(metadata.get('packages'), list) \
            or not isinstance(metadata.get('workspace_members'), list):
        raise RouterError('the metadata document is not a cargo metadata object')
    root = metadata.get('workspace_root', '').rstrip('/')
    members = set(metadata['workspace_members'])

    def rel(path):
        if path == root:
            return ''
        if not path.startswith(root + '/'):
            raise RouterError('%s is outside the workspace root %s' % (path, root))
        return path[len(root) + 1:]

    packages = {}
    dir_to_name = {}
    raw = [p for p in metadata['packages'] if p.get('id') in members]
    for entry in raw:
        directory = posixpath.dirname(rel(entry['manifest_path']))
        package = Package(entry['name'], directory)
        packages[package.name] = package
        dir_to_name[directory] = package.name
    for entry in raw:
        package = packages[entry['name']]
        for target in entry['targets']:
            kinds = target['kind']
            src = rel(target['src_path'])
            if 'custom-build' in kinds:
                package.build_script = src
                continue
            if any(kind in LIB_KINDS for kind in kinds):
                kind = 'lib'
                package.proc_macro = package.proc_macro or 'proc-macro' in kinds
                package.cdylib = package.cdylib or 'cdylib' in kinds
            elif kinds and kinds[0] in ('bin', 'test', 'example', 'bench'):
                kind = kinds[0]
            else:
                raise RouterError('%s: target %s has kinds %r this script does not know'
                                  % (package.name, target['name'], kinds))
            item = Target(package.name, target['name'], kind, src,
                          bool(target.get('test', kind in ('lib', 'bin', 'test'))),
                          bool(target.get('doctest', False)))
            package.targets.append(item)
            if kind == 'lib':
                package.lib = item
        package.targets.sort(key=lambda t: (t.kind, t.name))
        for dep in entry.get('dependencies', []):
            kind = dep.get('kind') or 'normal'
            package.dep_kinds.setdefault(dep['name'], set()).add(kind)
            path = dep.get('path')
            if not path:
                continue
            target_dir = rel(path)
            member = dir_to_name.get(target_dir)
            if member is None:
                raise RouterError('%s has a path dependency on %s, which is not a workspace '
                                  'member' % (package.name, target_dir))
            if member == package.name:
                continue
            package.member_deps.setdefault(member, set()).add(kind)
            if kind == 'dev':
                ident = (dep.get('rename') or '').replace('-', '_')
                lib = packages[member].lib if member in packages else None
                for name in (ident, lib.name.replace('-', '_') if lib else '',
                             member.replace('-', '_')):
                    if name:
                        package.dev_idents[name] = member
    return packages


def unit_ids(package, target):
    """The nextest-style id of a target's test binary."""
    if target.kind == 'lib':
        return package.name
    if target.kind == 'test':
        return '%s::%s' % (package.name, target.name)
    return '%s::%s/%s' % (package.name, target.kind, target.name)


# ---------------------------------------------------------------------------
# The router
# ---------------------------------------------------------------------------


class Unit:
    """One test binary this script can select."""

    __slots__ = ('id', 'package', 'target', 'kind')

    def __init__(self, unit_id, package, target, kind):
        self.id = unit_id
        self.package = package
        self.target = target
        self.kind = kind


class Router:
    def __init__(self, metadata, base, head, changed, scan_cache=None):
        """`changed` is an iterable of repository paths that differ."""
        self.base = base
        self.head = head
        self.changed = sorted(set(changed))
        self.changed_set = set(self.changed)
        self.cache = scan_cache if scan_cache is not None else {}
        self.warnings = []
        self.global_triggers = []
        self.packages = build_model(metadata)
        self.member_dirs = sorted(((p.dir, p.name) for p in self.packages.values()),
                                  key=lambda item: (-len(item[0]), item[0]))
        self.bin_names = {}
        for package in self.packages.values():
            for target in package.targets:
                if target.kind == 'bin':
                    self.bin_names[target.name] = (package.name, target.name)
        self.cdylib_stems = {}
        for package in self.packages.values():
            if package.cdylib and package.lib:
                self.cdylib_stems[package.lib.name.replace('-', '_')] = package.name
        self._closure = {}
        self._module_cache = {}

    # -- small helpers -----------------------------------------------------

    def note_global(self, path, reason):
        self.global_triggers.append((path, reason))

    def owner(self, path):
        for directory, name in self.member_dirs:
            if directory == '' or path == directory or path.startswith(directory + '/'):
                return name
        return None

    def scans(self, path):
        """The scans of `path` at head and, when it changed, at base too."""
        out = []
        for tree in (self.head, self.base) if path in self.changed_set else (self.head,):
            blob = tree.blob(path)
            if blob is None:
                continue
            scan = self.cache.get(blob)
            if scan is None:
                scan = scan_rust(tree.read(path))
                self.cache[blob] = scan
            out.append(scan)
        return out

    def exists(self, path):
        return self.head.exists(path) or self.base.exists(path)

    def closure(self, name):
        """Normal and build closure of a member, itself included."""
        if name in self._closure:
            return self._closure[name]
        seen = set()
        stack = [name]
        while stack:
            item = stack.pop()
            if item in seen:
                continue
            seen.add(item)
            for dep, kinds in self.packages[item].member_deps.items():
                if kinds & {'normal', 'build'}:
                    stack.append(dep)
        result = frozenset(seen)
        self._closure[name] = result
        return result

    def full_test_closure(self, name):
        """What `cargo test -p name` compiles: closure plus EVERY dev-dependency."""
        out = set(self.closure(name))
        for dep, kinds in self.packages[name].member_deps.items():
            if 'dev' in kinds:
                out |= self.closure(dep)
        return frozenset(out)

    # -- module trees ------------------------------------------------------

    def module_tree(self, root, package):
        """Files a crate root reaches: `{path: 'normal'|'test'}`, data, unresolved.

        `data` maps an `include_str!`/`include_bytes!` file to its mode. A
        `mod` that resolves to no file, or an include this scan cannot read,
        is listed in `unresolved` and the caller widens the target.
        """
        key = (root, package.dir)
        if key in self._module_cache:
            return self._module_cache[key]
        files = {}
        data = {}
        unresolved = []
        stack = [(root, (posixpath.dirname(root),), False)]
        while stack:
            path, child_dirs, test = stack.pop()
            have = files.get(path)
            if have == 'normal' or (have == 'test' and test):
                continue
            files[path] = 'test' if test else 'normal'
            here = posixpath.dirname(path)
            for scan in self.scans(path):
                for decl in scan.mods:
                    child_test = test or decl.test_only
                    candidates = []
                    for base_dir in child_dirs:
                        # An inline module's directory is its name, or the
                        # `#[path]` it carries; both readings are candidates.
                        inline_dirs = [posixpath.join(base_dir, *decl.inline_chain)
                                       if decl.inline_chain else base_dir]
                        if any(decl.inline_paths):
                            alt = base_dir
                            for part, attr in zip(decl.inline_chain, decl.inline_paths):
                                alt = posixpath.normpath(posixpath.join(alt, attr or part))
                            inline_dirs.append(alt)
                        for inline_dir in inline_dirs:
                            for attr in decl.paths:
                                anchor = inline_dir if decl.inline_chain else here
                                candidates.append((posixpath.normpath(
                                    posixpath.join(anchor, attr)), True))
                            if decl.keeps_default:
                                candidates.append((posixpath.join(inline_dir, decl.name + '.rs'),
                                                   False))
                                candidates.append((posixpath.join(inline_dir, decl.name,
                                                                  'mod.rs'), False))
                    found = False
                    for candidate, by_attr in candidates:
                        if not self.exists(candidate):
                            continue
                        found = True
                        stem = posixpath.splitext(posixpath.basename(candidate))[0]
                        parent = posixpath.dirname(candidate)
                        if by_attr:
                            dirs = (parent, posixpath.join(parent, stem))
                        elif posixpath.basename(candidate) == 'mod.rs':
                            dirs = (parent,)
                        else:
                            dirs = (posixpath.join(parent, stem),)
                        stack.append((candidate, dirs, child_test))
                    if not found:
                        unresolved.append('%s: mod %s' % (path, decl.name))
                for kind, shape, value, in_test in scan.includes:
                    mode_test = test or in_test
                    if shape == 'file':
                        target = posixpath.normpath(posixpath.join(here, value))
                    elif shape == 'manifest':
                        target = posixpath.normpath(posixpath.join(package.dir, value.lstrip('/')))
                    elif shape in ('out_dir', 'macro-arg'):
                        continue
                    else:
                        unresolved.append('%s: %s!(%s)' % (path, kind, value))
                        continue
                    if not self.exists(target):
                        if self.head.is_dir(target) or self.base.is_dir(target):
                            data[target + '/'] = 'test' if mode_test else 'normal'
                            continue
                        unresolved.append('%s: %s!("%s")' % (path, kind, value))
                        continue
                    if kind == 'include':
                        stack.append((target, child_dirs, mode_test))
                    else:
                        previous = data.get(target)
                        if previous != 'normal':
                            data[target] = 'test' if mode_test else 'normal'
        result = (files, data, sorted(set(unresolved)))
        self._module_cache[key] = result
        return result

    # -- the analysis ------------------------------------------------------

    def run(self):
        self.read_config()
        self.classify_paths()
        self.build_units()
        self.derive_sources()
        self.derive_behaviours()
        self.apply_observation_edges()
        self.propagate()
        return self.select()

    def read_config(self):
        ci_text = self.head.read(CI_WORKFLOW) or ''
        if not ci_text:
            self.warnings.append('%s is missing at head' % CI_WORKFLOW)
            self.note_global(CI_WORKFLOW, 'ci.yml-missing')
        self.doc_pins = parse_doc_pins(ci_text)
        self.machinery = sorted(set(ci_tools_paths(ci_text)))
        self.test_groups = ci_cotested_groups(ci_text, set(self.packages))
        edges_text = self.head.read(OBSERVATION_EDGES)
        if edges_text is None:
            self.warnings.append('%s is missing at head' % OBSERVATION_EDGES)
            self.note_global(OBSERVATION_EDGES, 'observation-edges-missing')
            self.edges = []
        else:
            try:
                self.edges = parse_observation_edges(edges_text)
            except RouterError as err:
                self.warnings.append(str(err))
                self.note_global(OBSERVATION_EDGES, 'observation-edges-unreadable')
                self.edges = []
        for package in self.packages.values():
            for target in package.targets:
                if not self.head.exists(target.src):
                    self.warnings.append('metadata names %s, which the head tree does not hold'
                                         % target.src)
                    self.note_global(target.src, 'metadata-mismatch')
        # The other direction: a target file cargo would discover at head that
        # the metadata does not list means the metadata is of another tree.
        for path in self.undiscovered_targets():
            self.warnings.append('the head tree holds %s, a target the metadata does not list'
                                 % path)
            self.note_global(path, 'metadata-mismatch')

    def undiscovered_targets(self):
        """Files at head cargo would make targets that no metadata target names."""
        known = set()
        for package in self.packages.values():
            known |= {target.src for target in package.targets}
            if package.build_script:
                known.add(package.build_script)
        out = []
        for path in self.head.files():
            if not path.endswith('.rs'):
                continue
            owner = self.owner(path)
            if owner is None or path in known:
                continue
            package = self.packages[owner]
            rel = path[len(package.dir) + 1:] if package.dir else path
            parts = rel.split('/')
            kind = None
            if rel in ('src/main.rs', 'src/lib.rs'):
                kind = 'bins' if rel == 'src/main.rs' else 'lib'
            elif parts[0] in ('tests', 'examples', 'benches') or rel.startswith('src/bin/'):
                top = 'bin' if rel.startswith('src/bin/') else parts[0]
                tail = parts[2:] if top == 'bin' else parts[1:]
                if len(tail) == 1 or (len(tail) == 2 and tail[1] == 'main.rs'):
                    kind = {'bin': 'bins', 'tests': 'tests', 'examples': 'examples',
                            'benches': 'benches'}[top]
            if kind is None:
                continue
            manifest = self.head.read(posixpath.join(package.dir, 'Cargo.toml')) or ''
            if kind != 'lib' and re.search(r'^\s*auto%s\s*=\s*false' % kind, manifest, re.M):
                continue
            if kind == 'lib' and package.lib is not None:
                continue
            out.append(path)
        return out

    def classify_paths(self):
        self.doc_dirty = set()
        self.foreign_hits = {}
        self.manifest_dirty = {}
        self.lock_lib_dirty = {}
        self.lock_dev_dirty = {}
        self.root_manifest_readers = False
        self.foreign = foreign_workspaces(self.head, self.base, self.packages)
        for path in self.changed:
            self.classify_one(path)

    def classify_one(self, path):
        for prefix, reason in GLOBAL_PATHS:
            if path == prefix or (prefix.endswith('/') and path.startswith(prefix)):
                self.note_global(path, reason)
                return
        if path.startswith('rust-toolchain'):
            self.note_global(path, 'rust-toolchain')
            return
        for token in self.machinery:
            if path == token or path.startswith(token.rstrip('/') + '/') or \
                    (not token.endswith('/') and path.startswith(token)):
                self.note_global(path, 'ci-machinery:%s' % token)
                return
        if path == 'Cargo.lock':
            self.classify_lock()
            return
        if path == 'Cargo.toml':
            self.classify_root_manifest()
            return
        claimed = False
        for directory in sorted(self.foreign):
            if path.startswith(directory + '/'):
                self.foreign_hits.setdefault(directory, path)
                claimed = True
        first = path.split('/', 1)[0]
        if first in DOC_ROOT_DIRS and '/' in path:
            self.doc_dirty.add(first)
            claimed = True
        elif path in ROOT_MARKDOWN:
            self.doc_dirty.add(path)
            claimed = True
        owner = self.owner(path)
        if owner is not None:
            package = self.packages[owner]
            if path == posixpath.join(package.dir, 'Cargo.toml'):
                self.manifest_dirty.setdefault(owner, path)
            claimed = True
        if not claimed:
            self.note_global(path, 'unknown-path')

    def classify_lock(self):
        try:
            base_entries, base_rest = parse_lock(self.base.read('Cargo.lock') or '')
            head_entries, head_rest = parse_lock(self.head.read('Cargo.lock') or '')
        except RouterError as err:
            self.warnings.append(str(err))
            self.note_global('Cargo.lock', 'lock-unreadable')
            return
        if base_rest != head_rest:
            self.note_global('Cargo.lock', 'lock-header')
            return
        changed = set(base_entries) ^ set(head_entries)
        for key in set(base_entries) & set(head_entries):
            if base_entries[key] != head_entries[key]:
                changed.add(key)
        if not changed:
            return
        reverse = {}
        try:
            for entries in (base_entries, head_entries):
                by_name = {}
                for key in entries:
                    by_name.setdefault(key[0], []).append(key)
                for key, entry in entries.items():
                    for spec in entry.get('dependencies', ()):
                        for dep in resolve_lock_dep(spec, by_name):
                            reverse.setdefault(dep, set()).add(key)
        except RouterError as err:
            self.warnings.append(str(err))
            self.note_global('Cargo.lock', 'lock-unreadable')
            return
        # Third-party entries only carry the walk: a member is reached through
        # its own direct dependencies, and member-to-member edges are the
        # workspace graph's, kinds and all.
        def is_member(key):
            return key[2] == '' and key[0] in self.packages
        reached = set(changed)
        stack = [key for key in changed if not is_member(key)]
        while stack:
            key = stack.pop()
            for parent in reverse.get(key, ()):
                if parent in reached:
                    continue
                reached.add(parent)
                if not is_member(parent):
                    stack.append(parent)
        for key in sorted(changed):
            if key[2] == '' and key[0] not in self.packages and key in head_entries:
                self.warnings.append('Cargo.lock: %s has no source and is no member' % key[0])
                self.note_global('Cargo.lock', 'lock-unplaced:%s' % key[0])
                return
        for name, package in sorted(self.packages.items()):
            member_keys = [key for key in list(head_entries) + list(base_entries)
                           if key[0] == name and key[2] == '']
            for key in member_keys:
                if key in changed:
                    self.lock_lib_dirty.setdefault(name, 'Cargo.lock entry %s' % name)
            for entries in (head_entries, base_entries):
                by_name = {}
                for key in entries:
                    by_name.setdefault(key[0], []).append(key)
                for key in member_keys:
                    entry = entries.get(key)
                    if entry is None:
                        continue
                    for spec in entry.get('dependencies', ()):
                        for dep in resolve_lock_dep(spec, by_name):
                            if dep not in reached or is_member(dep):
                                continue
                            kinds = package.dep_kinds.get(dep[0], {'normal'})
                            cause = 'Cargo.lock %s %s' % (dep[0], dep[1])
                            if kinds & {'normal', 'build'}:
                                self.lock_lib_dirty.setdefault(name, cause)
                            if 'dev' in kinds:
                                self.lock_dev_dirty.setdefault(name, cause)

    def classify_root_manifest(self):
        try:
            base_tables = toml_tables(self.base.read('Cargo.toml') or '')
            head_tables = toml_tables(self.head.read('Cargo.toml') or '')
        except RouterError as err:
            self.warnings.append(str(err))
            self.note_global('Cargo.toml', 'root-manifest-unreadable')
            return
        names = sorted(set(base_tables) | set(head_tables))
        differing = [name for name in names if base_tables.get(name) != head_tables.get(name)]
        if differing:
            self.note_global('Cargo.toml', 'root-manifest:%s' % (differing[0] or '<top level>'))
            return
        # Comments and blank lines only: the readers of the manifest's text.
        self.root_manifest_readers = True

    def build_units(self):
        self.units = []
        for name in sorted(self.packages):
            package = self.packages[name]
            for target in package.targets:
                if target.kind == 'example' or target.test or \
                        (target.kind == 'bench' and target.test):
                    unit = Unit(unit_ids(package, target), name, target, target.kind)
                    target.unit_id = unit.id
                    self.units.append(unit)
            if package.lib is not None and package.lib.doctest:
                self.units.append(Unit('%s::doctest/%s' % (name, package.lib.name), name,
                                       package.lib, 'doctest'))

    def derive_sources(self):
        """Source sets of every target and the build inputs of every package."""
        self.target_files = {}
        self.lib_normal_files = {}
        self.lib_test_files = {}
        self.widened = {}
        self.build_inputs = {}
        for name in sorted(self.packages):
            package = self.packages[name]
            for target in package.targets:
                files, data, unresolved = self.module_tree(target.src, package)
                normal = {path for path, mode in files.items() if mode == 'normal'}
                test = {path for path, mode in files.items() if mode == 'test'}
                normal |= {path for path, mode in data.items() if mode == 'normal'}
                test |= {path for path, mode in data.items() if mode == 'test'}
                if unresolved:
                    self.widened[(name, target.kind, target.name)] = unresolved
                    self.warnings.append('%s %s %s: %s; every file under %s counts'
                                         % (name, target.kind, target.name, unresolved[0],
                                            package.dir or '.'))
                self.target_files[(name, target.kind, target.name)] = (normal, test)
                if target.kind == 'lib':
                    self.lib_normal_files[name] = normal
                    self.lib_test_files[name] = test - normal
            self.build_inputs[name] = self.derive_build_inputs(package)

    def derive_build_inputs(self, package):
        if package.build_script is None:
            return set()
        files, data, unresolved = self.module_tree(package.build_script, package)
        inputs = set(files) | set(data)
        literals = set()
        for path in files:
            for scan in self.scans(path):
                literals |= scan.lits_normal | scan.lits_test
        directives = [value for value in literals if 'rerun-if-changed' in value]
        whole = not directives or unresolved
        for value in directives:
            tail = value.split('rerun-if-changed=', 1)[-1]
            if '{' in tail or not tail:
                whole = True
                continue
            path = posixpath.normpath(posixpath.join(package.dir, tail))
            # A directory cargo watches is watched whole, spelled with a slash
            # or not.
            if tail.endswith('/') or self.head.is_dir(path) or self.base.is_dir(path):
                path += '/'
            inputs.add(path)
        for value in literals:
            if not value or '\n' in value or ' ' in value or '{' in value:
                continue
            candidate = posixpath.normpath(posixpath.join(package.dir, value))
            if self.exists(candidate):
                inputs.add(candidate)
            elif self.head.is_dir(candidate) or self.base.is_dir(candidate):
                inputs.add(candidate + '/')
        if whole:
            inputs.add((package.dir + '/') if package.dir else '')
        return inputs

    # -- behaviours --------------------------------------------------------

    def context_text(self, files, portion):
        """Literals and words of `files`; portion is 'normal', 'test' or 'all'."""
        lits, words, file_macro = set(), set(), False
        for path in sorted(files):
            if not path.endswith('.rs'):
                continue
            for scan in self.scans(path):
                if portion in ('normal', 'all'):
                    lits |= scan.lits_normal
                    words |= scan.words_normal
                    file_macro = file_macro or scan.file_macro_normal
                if portion in ('test', 'all'):
                    lits |= scan.lits_test
                    words |= scan.words_test
                    file_macro = file_macro or scan.file_macro_test
        return lits, words, file_macro

    def file_behaviour(self, scan, portion, package_name, library):
        """What one file's portion does at run time.

        Judged PER FILE, because the signals that make a literal a spawn or a
        read (a `Command` beside a binary's name, a manifest-directory anchor
        beside a path) only mean that when they sit in the same source. A
        literal with whitespace in it is prose, never a path or a name.

        LIBRARY code (what every binary that links the package runs) reads the
        source tree only where the file anchors a path to it at build time:
        `CARGO_MANIFEST_DIR` or `file!()` (a `repo_root` or `workspace_root`
        there names a USER's workspace). Test code also counts those two
        helpers as anchors, and needs none for a path inside its package,
        because a test runs with its package directory as the working
        directory.
        """
        lits, words, file_macro = portion_view(scan, portion)
        lits = manifest_relative(lits)
        out = new_behaviour()
        spawn_context = bool(words & SPAWN_WORDS)
        if library:
            # `repo_root` and `workspace_root` name a USER's workspace in
            # product code, so only a build-time anchor counts there.
            anchored = 'CARGO_MANIFEST_DIR' in lits or file_macro
        else:
            anchored = 'CARGO_MANIFEST_DIR' in lits or file_macro or bool(words & ROOT_WORDS)
        own_dir = self.packages[package_name].dir
        own_paths = False
        for value in lits:
            if not value or any(ch.isspace() for ch in value):
                continue
            if value.startswith('CARGO_BIN_EXE_'):
                name = value[len('CARGO_BIN_EXE_'):]
                if name in self.bin_names:
                    out['spawns'].add(self.bin_names[name])
                continue
            if spawn_context:
                name = self.bin_name_in(value)
                if name is not None:
                    out['spawns'].add(self.bin_names[name])
            for stem, cdylib in self.cdylib_stems.items():
                if (stem in value or stem.replace('_', '-') in value) and cdylib != package_name:
                    out['dlopens'].add(cdylib)
            if library and not anchored:
                continue
            root = doc_root_of(value)
            if root is not None:
                out['docs'].add(root)
            keys = self.strict_tree_keys(value, package_name)
            if library:
                # Library code that names a member's ROOT directory hands it to
                # a nested cargo build as a path dependency (the scaffold's
                # `cerulion_core = { path = ... }`): what that reads is the
                # member's library, so it is linked rather than read.
                for kind, path in sorted(keys):
                    if kind == 'path' and path.endswith('/'):
                        member = self.owner(path.rstrip('/'))
                        if member is not None and self.packages[member].dir == path.rstrip('/'):
                            keys.discard((kind, path))
                            out['links'].add(member)
            for path in self.resolve_paths(value, own_dir):
                # A path inside the package itself: test code reads it
                # relative to the working directory, which is the package.
                if path_inside(path, own_dir):
                    out['trees'].add(('path', path))
                    own_paths = True
            out['trees'] |= keys
        if ('cargo' in lits and 'Command' in words) or \
                ('CARGO' in lits and words & {'var', 'var_os', 'env'}):
            out['cargo'] = True
        if library:
            if anchored and words & OWN_TREE_WORDS and not own_paths:
                out['trees'].add(('tree', package_name))
        elif ('CARGO_MANIFEST_DIR' in lits or file_macro or bool(words & OWN_TREE_WORDS) or
              any(value.startswith(OWN_TREE_PREFIXES) or value == 'Cargo.toml'
                  for value in lits)):
            out['trees'].add(('tree', package_name))
        # The ROOT manifest's text: a climb out of the package to it, or (in
        # test code) a repository-root helper beside its name. A bare
        # `Cargo.toml` beside `CARGO_MANIFEST_DIR` is the package's own.
        out['root_manifest'] = (
            (anchored and any(value.startswith(('../', './../')) and
                              strip_climb(value) == 'Cargo.toml' for value in lits)) or
            (not library and bool(words & ROOT_WORDS) and 'Cargo.toml' in lits))
        out['build_word'] = 'build' in lits
        return out

    def resolve_paths(self, value, package_dir):
        """Repository paths a literal can name: files as-is, directories with `/`.

        Every reading this tree uses: against the package directory, against
        the repository root, and with the climb stripped. A reading that names
        nothing in either tree is dropped.
        """
        if not value or '{' in value or any(ch.isspace() for ch in value):
            return []
        readings = []
        if value.startswith(('../', './')):
            readings.append(posixpath.normpath(posixpath.join(package_dir, value)))
            readings.append(posixpath.normpath(strip_climb(value) or '.'))
        else:
            readings.append(posixpath.normpath(value))
            readings.append(posixpath.normpath(posixpath.join(package_dir, value)))
        out = []
        for path in readings:
            if path in ('', '.') or path.startswith('..') or path.startswith('/'):
                continue
            if self.exists(path):
                out.append(path)
            elif self.head.is_dir(path) or self.base.is_dir(path):
                out.append(path + '/')
        return sorted(set(out))

    def strict_tree_keys(self, value, package_name):
        """Read keys for a PATH literal into another member's tree.

        Strict: only a literal shaped as a path (a climb, or rooted at
        `crates/`). A literal that resolves names its file or directory; one
        that does not names the whole tree of the member it points into.
        """
        if '{' in value or '/' not in value:
            return set()
        if not value.startswith(('../', './', 'crates/')):
            return set()
        own_dir = self.packages[package_name].dir
        keys = set()
        for path in self.resolve_paths(value, own_dir):
            owner = self.owner(path.rstrip('/'))
            if owner is not None and owner != package_name:
                keys.add(('path', path))
            elif owner is None or owner == package_name:
                for directory, name in self.member_dirs:
                    if name != package_name and directory and \
                            directory.startswith(path.rstrip('/') + '/'):
                        keys.add(('tree', name))
        if keys:
            return keys
        if value.startswith(('../', './')):
            target = posixpath.normpath(posixpath.join(own_dir, value))
        else:
            target = posixpath.normpath(value)
        owner = self.owner(target)
        if owner is not None and owner != package_name and self.packages[owner].dir:
            keys.add(('tree', owner))
        return keys

    def bin_name_in(self, value):
        for candidate in (value, value[:-4] if value.endswith('.exe') else None,
                          value.rsplit('/', 1)[-1] if '/' in value else None):
            if candidate and candidate in self.bin_names:
                return candidate
        return None

    def context_behaviour_of(self, parts, package_name, library):
        """Union of the per-file behaviours of `[(path, portion)]`.

        A binary's name bound to a `const` or `static` in one file and handed
        to `Command` in another is a spawn too: the binding's NAME is followed
        into every file of the context that runs a command.
        """
        out = new_behaviour()
        consts = {}
        command_words = set()
        for path, portion in sorted(parts):
            if not path.endswith('.rs'):
                continue
            for scan in self.scans(path):
                part = self.file_behaviour(scan, portion, package_name, library)
                merge_behaviour(out, part)
                for name, value in scan.consts:
                    target = self.bin_name_in(value)
                    if target is not None:
                        consts[name] = target
                if portion == 'test':
                    words = scan.words_test
                else:
                    words = portion_words(scan, portion)
                if words & SPAWN_WORDS:
                    command_words |= words
        for name, target in sorted(consts.items()):
            if name in command_words:
                out['spawns'].add(self.bin_names[target])
        return out

    def derive_behaviours(self):
        """Run-time behaviour of every library and every target context."""
        self.lib_behaviour = {}
        self.lib_test_behaviour = {}
        self.target_behaviour = {}
        self.bin_run_behaviour = {}
        for name in sorted(self.packages):
            package = self.packages[name]
            if package.lib is not None:
                files = self.lib_normal_files[name]
                self.lib_behaviour[name] = self.context_behaviour_of(
                    [(path, 'normal') for path in files], name, True)
                # The library's unit-test binary runs the library itself, which
                # its own links carry (`lib_behaviour`), plus the code only a
                # `cfg(test)` build compiles: that code is what it owns.
                parts = [(path, 'test') for path in files]
                parts += [(path, 'all') for path in self.lib_test_files[name]]
                self.lib_test_behaviour[name] = self.context_behaviour_of(parts, name, False)
            for target in package.targets:
                if target.kind == 'lib':
                    continue
                normal, test = self.target_files[(name, target.kind, target.name)]
                behaviour = self.context_behaviour_of(
                    [(path, 'all') for path in normal | test], name, False)
                self.target_behaviour[(name, target.kind, target.name)] = behaviour
                if target.kind == 'bin':
                    self.bin_run_behaviour[target.name] = self.context_behaviour_of(
                        [(path, 'normal') for path in normal], name, True)
        # A test that drives the CLI's build verbs runs cargo through it.
        for key, behaviour in sorted(self.target_behaviour.items()):
            if behaviour['build_word'] and any(name == CLI_BINARY for _, name in
                                               behaviour['spawns']):
                behaviour['cargo'] = True

    def apply_observation_edges(self):
        """Fold the table and the doc-pin markers in, narrowed per context.

        A row names a package. The contexts of that package whose files carry
        the row's shape get the edge; when the row's witness file is not among
        them, the narrowing failed for that row and every context of the
        package gets it (and its library too, when the witness is in `src/`).
        """
        contexts = self.package_contexts()
        for observing, observed, kind, witness in self.edges:
            if observing not in self.packages:
                self.warnings.append('observation edge names %s, no member' % observing)
                self.note_global(OBSERVATION_EDGES, 'observation-edge-unknown:%s' % observing)
                continue
            if observed != EVERYTHING and observed not in self.packages:
                self.warnings.append('observation edge names %s, no member' % observed)
                self.note_global(OBSERVATION_EDGES, 'observation-edge-unknown:%s' % observed)
                continue
            if kind not in EDGE_KINDS:
                self.warnings.append('observation edge kind %r is not one this script reads'
                                     % kind)
                self.note_global(OBSERVATION_EDGES, 'observation-edge-kind:%s' % kind)
                continue
            matcher = self.edge_matcher(observed, kind)
            # The walk keeps the FIRST witness by path, and `src/` sorts before
            # `tests/`: a witness outside `src/` means no library source of
            # the package carries the edge, so the library contexts are out.
            in_src = witness.startswith(posixpath.join(self.packages[observing].dir, 'src/'))
            candidates = [item for item in contexts.get(observing, [])
                          if in_src or item[0][0] not in ('lib', 'libtest')]
            hits = {}
            witness_hit = False
            for ctx_key, files, portion in candidates:
                # Code every linker runs reads the source tree only through a
                # build-time anchor (see `file_behaviour`); a dlopen finds its
                # artifact in the target directory and needs none.
                anchor = ctx_key[0] == 'lib' and kind != 'dlopen'
                matched = [path for path in sorted(files) if path.endswith('.rs') and
                           any(matcher(scan, portion) and (not anchor or library_anchored(scan))
                               for scan in self.scans(path))]
                if matched:
                    hits[ctx_key] = matched, portion
                    if witness in matched:
                        witness_hit = True
            if not witness_hit:
                # The narrowing could not see the row's own witness: every
                # context that could hold it gets the edge, unnarrowed.
                self.warnings.append('observation edge %s -> %s (%s) not narrowed: its witness '
                                     '%s carries no shape this script reads'
                                     % (observing, observed, kind, witness))
                hits = {ctx_key: (None, portion) for ctx_key, _, portion in candidates}
            for ctx_key, (matched, portion) in sorted(hits.items()):
                self.attach_edge(ctx_key, observing, observed, kind, matched, portion)
        for (package, binary), roots in sorted(self.doc_pins.items()):
            if package not in self.packages:
                self.warnings.append('doc-pin marker names %s, no member' % package)
                self.note_global(CI_WORKFLOW, 'doc-pin-unknown:%s' % package)
                continue
            keys = self.marker_contexts(package, binary)
            for ctx_key in keys:
                behaviour = self.context_behaviour(ctx_key)
                behaviour['docs'] |= roots

    def package_contexts(self):
        """`{package: [(context key, files, portion)]}` for the edge narrowing."""
        out = {}
        for name in sorted(self.packages):
            package = self.packages[name]
            items = []
            if package.lib is not None:
                items.append((('lib', name), self.lib_normal_files[name], 'normal'))
                items.append((('libtest', name),
                              self.lib_normal_files[name] | self.lib_test_files[name], 'all'))
            for target in package.targets:
                if target.kind == 'lib':
                    continue
                normal, test = self.target_files[(name, target.kind, target.name)]
                items.append((('target', name, target.kind, target.name), normal | test, 'all'))
            out[name] = items
        return out

    def context_behaviour(self, ctx_key):
        if ctx_key[0] == 'lib':
            return self.lib_behaviour[ctx_key[1]]
        if ctx_key[0] == 'libtest':
            return self.lib_test_behaviour[ctx_key[1]]
        return self.target_behaviour[ctx_key[1:]]

    def attach_edge(self, ctx_key, observing, observed, kind, matched, portion):
        behaviour = self.context_behaviour(ctx_key)
        if observed == EVERYTHING:
            behaviour['always'].add(kind)
            return
        if kind == 'dlopen':
            behaviour['dlopens'].add(observed)
            return
        for key in self.crate_path_keys(observed, observing, matched, portion):
            if key[0] == 'link':
                if ctx_key[0] == 'lib':
                    behaviour['links'].add(key[1])
                else:
                    behaviour['trees'].add(('tree', key[1]))
            else:
                behaviour['trees'].add(key)

    def crate_path_keys(self, observed, observing, matched, portion):
        """What a crate-path edge reads: the paths its literals name, or the tree.

        Every literal in the matched files that carries the observed member's
        shape is resolved. One that names a file or a directory inside the
        member reads that; one that names nothing this script can find reads
        the member's whole tree, and so does an edge it could not narrow.
        """
        if matched is None:
            return {('tree', observed)}
        directory = self.packages[observed].dir
        shape = self.crate_shape(observed)
        own_dir = self.packages[observing].dir
        keys = set()
        for path in matched:
            for scan in self.scans(path):
                lits, _, _ = portion_view(scan, portion)
                for value in sorted(manifest_relative(lits)):
                    # Prose and interpolated text are not a path (the walk
                    # reads an interpolation into a member tree as its own
                    # `unattributable` row).
                    if not shape(value) or '{' in value or \
                            any(ch.isspace() for ch in value):
                        continue
                    inside = [item for item in self.resolve_paths(value, own_dir)
                              if path_inside(item.rstrip('/'), directory)]
                    if not inside:
                        return {('tree', observed)}
                    for item in inside:
                        if item.rstrip('/') == directory:
                            # The member's root directory itself: a path
                            # dependency for a nested build in library code,
                            # a read of the whole tree anywhere else.
                            keys.add(('link', observed))
                        else:
                            keys.add(('path', item))
        return keys or {('tree', observed)}

    def crate_shape(self, observed):
        directory = self.packages[observed].dir
        base = posixpath.basename(directory)
        parts = directory.split('/')
        components = set(parts)
        prefixes = {'/'.join(parts[:index]) for index in range(1, len(parts) + 1)}

        def shape(value):
            # The member's own name anywhere in a literal, a literal that names
            # one of its directories (a `.join` chain spells the path one
            # component at a time), or a literal naming a directory above it.
            return base in value or value in components or strip_climb(value) in prefixes
        return shape

    def marker_contexts(self, package, binary):
        name = self.packages[package]
        if binary == package or (name.lib is not None and binary == name.lib.name):
            # The walk pools every `src/` read into `<package>::<package>`.
            # With a library that is its unit-test binary (a read in code
            # every linker runs is this script's own anchored scan's to find,
            # see `file_behaviour`); a package with no library has its `src/`
            # in its binaries, so every context of the package reads it.
            if name.lib is not None:
                return [('libtest', package)]
            return [key for key, _, _ in self.package_contexts()[package]]
        for target in name.targets:
            if target.kind == 'test' and (target.name == binary or
                                          posixpath.basename(target.src) == binary + '.rs'):
                return [('target', package, target.kind, target.name)]
        self.warnings.append('doc-pin marker %s::%s names no target; every context of %s '
                             'reads its roots' % (package, binary, package))
        return [key for key, _, _ in self.package_contexts()[package]]

    def edge_matcher(self, observed, kind):
        if kind == 'dlopen':
            stem = self.packages[observed].lib.name.replace('-', '_') \
                if self.packages[observed].lib else observed
            names = (stem, stem.replace('_', '-'), observed)

            def match(scan, portion):
                return any(n in value for value in portion_lits(scan, portion) for n in names)
            return match
        if kind == 'crate-path':
            shape = self.crate_shape(observed)

            def match(scan, portion):
                return any(shape(value) for value in portion_lits(scan, portion))
            return match
        multi = multi_member_dirs(self.packages)
        if kind == 'whole-tree':

            def match(scan, portion):
                # The root handed to a call, or a literal that names a
                # directory holding several members (`crates`).
                if portion == 'normal':
                    roots = scan.root_args_normal
                else:
                    roots = scan.root_args_normal or scan.root_args_test
                return roots or any(strip_climb(value) in multi
                                    for value in portion_lits(scan, portion))
            return match
        tops = {directory for directory in multi if '/' not in directory}

        def match(scan, portion):
            # An interpolated segment where a member tree or an artifact name
            # would be (`lib{name}.so`, `crates/{pkg}`), or an interpolation
            # beside a literal naming a member-holding directory, which a
            # `.join` chain composes into the same path.
            lits = portion_lits(scan, portion)
            chain = any(value in tops for value in lits)
            for value in lits:
                if '{' not in value:
                    continue
                last = value.rsplit('/', 1)[-1]
                if last.startswith('lib') and (last.endswith('.so') or last.endswith('.dylib')):
                    return True
                if chain or any(segment in tops for segment in value.split('/')):
                    return True
            return False
        return match

    # -- dirt --------------------------------------------------------------

    def propagate(self):
        """Seed the dirty libraries and targets, then walk reverse edges."""
        self.lib_dirty = {}
        self.target_dirty = {}
        self.libtest_dirty = {}
        changed = self.changed_set
        prefix_changed = sorted(changed)

        def touched(inputs):
            for item in sorted(inputs):
                if item.endswith('/') or item == '':
                    for path in prefix_changed:
                        if path.startswith(item):
                            return path
                elif item in changed:
                    return item
            return None

        for name in sorted(self.packages):
            package = self.packages[name]
            if name in self.manifest_dirty:
                self.lib_dirty.setdefault(name, 'manifest %s' % self.manifest_dirty[name])
            if name in self.lock_lib_dirty:
                self.lib_dirty.setdefault(name, self.lock_lib_dirty[name])
            hit = touched(self.build_inputs[name])
            if hit:
                self.lib_dirty.setdefault(name, 'build input %s' % hit)
            if package.lib is not None:
                hit = touched(self.lib_normal_files[name])
                if hit:
                    self.lib_dirty.setdefault(name, 'source %s' % hit)
                if (name, 'lib', package.lib.name) in self.widened:
                    hit = touched({(package.dir + '/') if package.dir else ''})
                    if hit:
                        self.lib_dirty.setdefault(name, 'widened source %s' % hit)
                hit = touched(self.lib_test_files[name])
                if hit:
                    self.libtest_dirty[name] = 'test-only source %s' % hit
            for target in package.targets:
                if target.kind == 'lib':
                    continue
                key = (name, target.kind, target.name)
                normal, test = self.target_files[key]
                hit = touched(normal | test)
                if hit:
                    self.target_dirty[key] = 'source %s' % hit
                elif key in self.widened:
                    hit = touched({(package.dir + '/') if package.dir else ''})
                    if hit:
                        self.target_dirty[key] = 'widened source %s' % hit
        # Reverse normal and build edges carry a library's dirt to its users.
        # The SEEDS are kept apart, so a reason names the change itself.
        self.lib_seeds = dict(self.lib_dirty)
        users = {}
        for name, package in self.packages.items():
            for dep, kinds in package.member_deps.items():
                if kinds & {'normal', 'build'}:
                    users.setdefault(dep, set()).add(name)
        stack = sorted(self.lib_dirty)
        while stack:
            dep = stack.pop()
            for user in sorted(users.get(dep, ())):
                if user not in self.lib_dirty:
                    self.lib_dirty[user] = 'depends on %s' % dep
                    stack.append(user)

    def unit_links(self, unit):
        """Packages a unit's test binary links."""
        package = self.packages[unit.package]
        out = set(self.closure(unit.package))
        if unit.kind == 'doctest':
            idents = set(package.dev_idents)
        else:
            if unit.kind == 'lib':
                files = self.lib_normal_files[unit.package] | self.lib_test_files[unit.package]
            else:
                normal, test = self.target_files[(unit.package, unit.target.kind,
                                                  unit.target.name)]
                files = normal | test
            _, words, _ = self.context_text(files, 'all')
            if words & COMPILES_AT_RUN_TIME:
                # A driver that compiles test cases at run time (trybuild)
                # compiles them against every dependency the package has, and
                # the cases name crates the driver file never does.
                idents = set(package.dev_idents)
            else:
                idents = {ident for ident in package.dev_idents if ident in words}
        for ident in sorted(idents):
            out |= self.closure(package.dev_idents[ident])
        return out

    def unit_direct_behaviour(self, unit):
        if unit.kind == 'lib':
            return [self.lib_test_behaviour[unit.package]]
        if unit.kind == 'doctest':
            return [self.lib_behaviour[unit.package]]
        return [self.target_behaviour[(unit.package, unit.target.kind, unit.target.name)]]

    def expand(self, links, direct):
        """Everything a binary reaches through its links, spawns and loads.

        Returns `(libs, bins, trees, docs, flags, artifacts)`, transitively.
        """
        libs = set(links)
        bins = set()
        cdylibs = set()
        trees = set()
        docs = set()
        flags = {}
        pending = list(direct)
        seen_libs = set()
        while True:
            for name in sorted(libs - seen_libs):
                seen_libs.add(name)
                if self.packages[name].lib is not None:
                    pending.append(self.lib_behaviour[name])
            if not pending:
                break
            behaviour = pending.pop()
            for always in sorted(behaviour.get('always', ())):
                flags.setdefault('always:%s' % always, True)
            if behaviour.get('cargo'):
                if any(behaviour is item for item in direct):
                    # The binary's own sources run cargo: an unlocked resolve
                    # against crates.io, which no input key covers.
                    flags.setdefault('always:nested-cargo', True)
                elif 'nested-build' not in flags:
                    # A library or a spawned binary runs cargo on a scaffold:
                    # what that build can compile is the umbrella crate.
                    flags['nested-build'] = True
                    if UMBRELLA in self.packages:
                        libs |= self.closure(UMBRELLA)
            if behaviour.get('root_manifest'):
                flags.setdefault('reads:Cargo.toml', True)
            docs |= behaviour['docs']
            trees |= behaviour['trees']
            for name in sorted(behaviour['links']):
                libs |= self.closure(name)
            for package_name, bin_name in sorted(behaviour['spawns']):
                if bin_name in bins:
                    continue
                bins.add(bin_name)
                libs |= self.closure(package_name)
                pending.append(self.bin_run_behaviour[bin_name])
            for cdylib in sorted(behaviour['dlopens']):
                if cdylib in cdylibs:
                    continue
                cdylibs.add(cdylib)
                libs |= self.closure(cdylib)
        return libs, bins, cdylibs, trees, docs, flags

    def select(self):
        selected = {}
        reasons_for = {}
        nonempty = bool(self.changed)
        global_reason = ['global:%s (%s)' % (reason, path)
                         for path, reason in sorted(set(self.global_triggers))]
        manifest_affected = {}
        for dirty in sorted(self.manifest_dirty):
            for name in sorted(self.packages):
                if dirty in self.full_test_closure(name):
                    manifest_affected.setdefault(name, 'manifest %s (test build of %s)'
                                                 % (self.manifest_dirty[dirty], name))
            for group in self.test_groups:
                members = set(group)
                reach = set()
                for name in members:
                    reach |= self.full_test_closure(name)
                if dirty in reach:
                    for name in sorted(members):
                        manifest_affected.setdefault(name, 'manifest %s (shares a cargo test '
                                                     'command)' % self.manifest_dirty[dirty])
        built = set()
        for unit in self.units:
            reasons = []
            links = self.unit_links(unit)
            libs, bins, cdylibs, trees, docs, flags = self.expand(
                links, self.unit_direct_behaviour(unit))
            reasons.extend(global_reason)
            if nonempty:
                reasons.extend(sorted(flag for flag in flags if flag.startswith('always:')))
            if unit.kind == 'lib' and unit.package in self.libtest_dirty:
                reasons.append(self.libtest_dirty[unit.package])
            if unit.kind not in ('lib', 'doctest'):
                key = (unit.package, unit.target.kind, unit.target.name)
                if key in self.target_dirty:
                    reasons.append(self.target_dirty[key])
            dirty = sorted(name for name in libs if name in self.lib_dirty)
            if dirty:
                seeds = [name for name in dirty if name in self.lib_seeds] or dirty
                reasons.append('links %s: %s' % (seeds[0], self.lib_dirty[seeds[0]]))
            for bin_name in sorted(bins):
                package_name, _ = self.bin_names[bin_name]
                key = (package_name, 'bin', bin_name)
                if key in self.target_dirty:
                    reasons.append('spawns %s: %s' % (bin_name, self.target_dirty[key]))
                    break
            if unit.package in manifest_affected:
                reasons.append(manifest_affected[unit.package])
            if self.manifest_dirty and (bins or cdylibs):
                first = sorted(self.manifest_dirty)[0]
                reasons.append('manifest %s (workspace-built artifact)'
                               % self.manifest_dirty[first])
            elif self.lock_lib_dirty and (bins or cdylibs):
                # A new version can request other features of a crate it
                # shares with an artifact `cargo build --workspace` builds.
                first = sorted(self.lock_lib_dirty)[0]
                reasons.append('%s (workspace-built artifact)' % self.lock_lib_dirty[first])
            if unit.package in self.lock_dev_dirty:
                reasons.append('dev-dependency %s' % self.lock_dev_dirty[unit.package])
            for key in sorted(trees):
                hit = self.tree_key_dirty(key)
                if hit:
                    reasons.append('reads %s: %s' % (key[1], hit))
                    break
            for root in sorted(docs & self.doc_dirty):
                reasons.append('reads %s' % root)
                break
            if flags.get('reads:Cargo.toml') and self.root_manifest_readers:
                reasons.append('reads the root Cargo.toml text')
            if reasons:
                selected[unit.id] = unit
                reasons_for[unit.id] = reasons
                # Cargo compiles every dev-dependency of the package for any
                # of its test binaries, named or not, so the built set takes
                # the full test closure; `libs` stays the narrower set the
                # selection above reads.
                built |= libs | self.full_test_closure(unit.package)
        foreign = self.select_foreign()
        return self.report(selected, reasons_for, built, foreign)

    def tree_key_dirty(self, key):
        """The changed path a read key covers, or None."""
        kind, value = key
        if kind == 'tree':
            directory = self.packages[value].dir
            for path in self.changed:
                if path_inside(path, directory):
                    return path
            return None
        if value.endswith('/'):
            for path in self.changed:
                if path.startswith(value):
                    return path
            return None
        return value if value in self.changed_set else None

    def select_foreign(self):
        out = {}
        any_lib = bool(self.lib_dirty) or any(key[1] == 'bin' for key in self.target_dirty)
        for directory in sorted(self.foreign):
            reasons = []
            if self.global_triggers:
                reasons.append('global')
            if directory in self.foreign_hits:
                reasons.append('changed %s' % self.foreign_hits[directory])
            if any_lib:
                reasons.append('a workspace library or binary changed')
            if self.doc_dirty:
                reasons.append('a shared tree changed')
            out[directory] = reasons
        return out

    def report(self, selected, reasons_for, built, foreign):
        units = sorted(self.units, key=lambda u: u.id)
        skipped = [unit.id for unit in units if unit.id not in selected]
        doc = {
            'schema': SCHEMA_VERSION,
            'base': getattr(self.base, 'rev', None),
            'head': getattr(self.head, 'rev', None),
            'changed_paths': self.changed,
            'global': [{'path': path, 'reason': reason}
                       for path, reason in sorted(set(self.global_triggers))],
            'selected': [{'binary': unit.id, 'package': unit.package, 'kind': unit.kind,
                          'target': unit.target.name, 'reasons': reasons_for[unit.id]}
                         for unit in units if unit.id in selected],
            'skipped': skipped,
            # Every binary, so a scorer can place a log line that names a
            # target by its file and crate name rather than by this id.
            'units': [{'binary': unit.id, 'package': unit.package, 'kind': unit.kind,
                       'target': unit.target.name, 'src': unit.target.src}
                      for unit in units],
            'foreign_workspaces': {
                'selected': [{'workspace': key, 'reasons': value}
                             for key, value in sorted(foreign.items()) if value],
                'skipped': sorted(key for key, value in foreign.items() if not value),
            },
            'dirty_packages': {name: cause for name, cause in sorted(self.lib_dirty.items())},
            'built_packages': sorted(built),
            'counts': {'units': len(units), 'selected': len(selected),
                       'skipped': len(skipped)},
            'warnings': sorted(set(self.warnings)),
        }
        return doc


def new_behaviour():
    return {'spawns': set(), 'dlopens': set(), 'docs': set(), 'trees': set(), 'always': set(),
            'links': set(), 'cargo': False, 'root_manifest': False, 'build_word': False}


def merge_behaviour(into, part):
    for key in ('spawns', 'dlopens', 'docs', 'trees', 'always', 'links'):
        into[key] |= part[key]
    for key in ('cargo', 'root_manifest', 'build_word'):
        into[key] = into[key] or part[key]


def portion_view(scan, portion):
    """`(literals, words, file_macro)` of one portion of a scan."""
    if portion == 'normal':
        return scan.lits_normal, scan.words_normal, scan.file_macro_normal
    if portion == 'test':
        return scan.lits_test, scan.words_test, scan.file_macro_test
    return (scan.lits_normal | scan.lits_test, scan.words_normal | scan.words_test,
            scan.file_macro_normal or scan.file_macro_test)


def library_anchored(scan):
    """Does the non-test code of a file anchor a path to the source tree?"""
    return 'CARGO_MANIFEST_DIR' in scan.lits_normal or scan.file_macro_normal


def manifest_relative(lits):
    """Literals with a `concat!(env!("CARGO_MANIFEST_DIR"), "/x")` tail read as `./x`.

    The tail is a separate literal that opens with `/`, which no relative or
    rooted reading would place. Applied only where the manifest directory is
    in the same literal set, so a real absolute path elsewhere stays one.
    """
    if 'CARGO_MANIFEST_DIR' not in lits:
        return lits
    return {'.' + value if value.startswith('/') and not value.startswith('//') else value
            for value in lits}


def path_inside(path, directory):
    if not directory:
        return True
    return path == directory or path.startswith(directory + '/')


def portion_lits(scan, portion):
    if portion == 'normal':
        return scan.lits_normal
    return scan.lits_normal | scan.lits_test


def portion_words(scan, portion):
    if portion == 'normal':
        return scan.words_normal
    return scan.words_normal | scan.words_test


def strip_climb(value):
    while value.startswith(('../', './')):
        value = value[3:] if value.startswith('../') else value[2:]
    return value.rstrip('/')


def doc_root_of(value):
    """The shared tree or root markdown file a path literal names, if any."""
    if not value or '\n' in value or ' ' in value:
        return None
    stripped = strip_climb(value)
    if stripped in ROOT_MARKDOWN:
        return stripped
    first = stripped.split('/', 1)[0]
    if first in DOC_ROOT_DIRS:
        return first
    return None


def multi_member_dirs(packages):
    """Directories that hold two or more members: a literal naming one walks them."""
    counts = {}
    for package in packages.values():
        parent = posixpath.dirname(package.dir)
        while parent:
            counts[parent] = counts.get(parent, 0) + 1
            parent = posixpath.dirname(parent)
    return {directory for directory, count in counts.items() if count >= 2}


def foreign_workspaces(head, base, packages):
    """Directories of manifests outside the root workspace, one unit per root."""
    member_dirs = {package.dir for package in packages.values()}
    dirs = set()
    for tree in (head, base):
        for path in tree.files():
            if posixpath.basename(path) != 'Cargo.toml' or path == 'Cargo.toml':
                continue
            directory = posixpath.dirname(path)
            if directory in member_dirs:
                continue
            dirs.add(directory)
    roots = set()
    for directory in sorted(dirs):
        parent = posixpath.dirname(directory)
        nested = False
        while parent:
            if parent in dirs:
                nested = True
                break
            parent = posixpath.dirname(parent)
        if not nested:
            roots.add(directory)
    return roots


# ---------------------------------------------------------------------------
# Readers of ci.yml and the observation-edge table
# ---------------------------------------------------------------------------


def parse_doc_pins(text):
    """`{(package, binary): {roots}}` from the `# doc-pin:` markers."""
    out = {}
    for line in text.splitlines():
        if '# doc-pin: ' not in line:
            continue
        body = line.split('# doc-pin: ', 1)[1]
        if ' reads ' not in body or '::' not in body:
            continue
        pair, roots = body.split(' reads ', 1)
        package, binary = pair.strip().split('::', 1)
        out.setdefault((package, binary), set()).update(
            root.strip() for root in roots.split(',') if root.strip())
    return out


def ci_jobs(text):
    """`[(job key, text)]` for the jobs of a workflow, by two-space indentation."""
    jobs = []
    inside = False
    current = None
    lines = []
    for line in text.splitlines():
        if line.startswith('jobs:'):
            inside = True
            continue
        if not inside:
            continue
        match = re.match(r'^  ([A-Za-z0-9_-]+):\s*$', line)
        if match:
            if current is not None:
                jobs.append((current, '\n'.join(lines)))
            current, lines = match.group(1), []
            continue
        if line and not line.startswith(' ') and not line.startswith('#'):
            break
        lines.append(line)
    if current is not None:
        jobs.append((current, '\n'.join(lines)))
    return jobs


def _code_lines(text):
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        yield line


# A job that runs test binaries. `ci_test_shard.sh --check` is Lint's static
# partition check and runs none.
TEST_JOB = re.compile(r'cargo (?:\+\S+ )?(?:test|nextest|miri test)|ci_test_shard\.sh(?! --check)')


def ci_tools_paths(text):
    """Every `tools/` path a job that runs test binaries names, outside comments."""
    out = []
    for _, body in ci_jobs(text):
        code = '\n'.join(_code_lines(body))
        if not TEST_JOB.search(code):
            continue
        for token in re.findall(r'tools/[A-Za-z0-9_./-]*', code):
            token = token.rstrip('.,')
            if token in ('tools/', 'tools'):
                # A bare `tools/` is a doc root, never a script a step runs.
                continue
            out.append(token)
    return out


def ci_cotested_groups(text, members):
    """Package sets one `cargo test`/`nextest` command in ci.yml tests together.

    A command that runs in ANOTHER directory (after a `cd` on its line, or in
    a step with `working-directory:`) tests another workspace and is skipped.
    """
    groups = []
    for _, body in ci_jobs(text):
        code = '\n'.join(_code_lines(body)).replace('\\\n', ' ')
        steps = re.split(r'\n(?=\s*- )', code)
        for step in steps:
            if re.search(r'^\s*-?\s*working-directory:', step, re.M):
                continue
            for line in step.splitlines():
                moved = False
                for command in re.split(r'&&|;|\|', line):
                    if re.match(r'^\s*(?:-\s*)?(?:run:\s*)?cd\s', command):
                        moved = True
                        continue
                    if moved or not re.search(r'cargo (?:\+\S+ )?(?:test|nextest run)', command):
                        continue
                    names = re.findall(r'(?:-p|--package)[ =]([A-Za-z0-9_-]+)', command)
                    names = sorted(set(name for name in names if name in members))
                    if '--workspace' in command:
                        names = sorted(members)
                    if len(names) >= 2:
                        groups.append(tuple(names))
    return sorted(set(groups))


def parse_observation_edges(text):
    rows = []
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split('\t')
        if len(fields) != 4 or any(not field or field != field.strip() for field in fields):
            raise RouterError('%s:%d: malformed observation edge %r'
                              % (OBSERVATION_EDGES, number, line))
        rows.append(tuple(fields))
    return rows


# ---------------------------------------------------------------------------
# Self-test: a hand-built workspace and hand-written selections
# ---------------------------------------------------------------------------
#
# The fixture is a small workspace whose files carry exactly the shapes the
# rules read. Every row names the paths that changed and the selection WRITTEN
# OUT BY HAND: the expected set says what the rule IS, so a rule that changes
# its mind fails here instead of agreeing with itself. The named sets below are
# typed out too, never computed by the router.

FX_ROOT = '/ws'

FX_REGISTRY = 'registry+https://github.com/rust-lang/crates.io-index'

FX_LOCK = """# This file is automatically @generated by Cargo.
version = 4

[[package]]
name = "cerulion"
version = "0.1.0"
dependencies = [
 "fx_core",
 "fx_macros",
]

[[package]]
name = "fx_alpha"
version = "0.1.0"
dependencies = [
 "fx_user",
 "serde",
]

[[package]]
name = "fx_android"
version = "0.1.0"
dependencies = [
 "fx_bagd",
 "fx_core",
]

[[package]]
name = "fx_bagd"
version = "0.1.0"
dependencies = [
 "fx_core",
]

[[package]]
name = "fx_beta"
version = "0.1.0"

[[package]]
name = "fx_cli"
version = "0.1.0"
dependencies = [
 "fx_engine",
]

[[package]]
name = "fx_core"
version = "0.1.0"
dependencies = [
 "fx_engine",
]

[[package]]
name = "fx_engine"
version = "0.1.0"
dependencies = [
 "fx_core",
 "fx_netd",
 "tempfile",
]

[[package]]
name = "fx_fixture"
version = "0.1.0"
dependencies = [
 "fx_core",
]

[[package]]
name = "fx_macros"
version = "0.1.0"

[[package]]
name = "fx_netd"
version = "0.1.0"
dependencies = [
 "fx_core",
]

[[package]]
name = "fx_user"
version = "0.1.0"
dependencies = [
 "fx_macros",
]

[[package]]
name = "fx_vizd"
version = "0.1.0"
dependencies = [
 "fx_core",
]

[[package]]
name = "fastrand"
version = "2.0.0"
source = "%(reg)s"
checksum = "dddd"
dependencies = [
 "libc",
]

[[package]]
name = "libc"
version = "0.2.1"
source = "%(reg)s"
checksum = "aaaa"

[[package]]
name = "serde"
version = "1.0.1"
source = "%(reg)s"
checksum = "bbbb"

[[package]]
name = "tempfile"
version = "3.0.0"
source = "%(reg)s"
checksum = "cccc"
dependencies = [
 "fastrand",
]
""" % {'reg': FX_REGISTRY}

FX_ROOT_MANIFEST = """[workspace]
# The fixture members.
members = [
    "crates/*",
]
resolver = "2"

[workspace.dependencies]
serde = "1"

[profile.dev]
debug = 1
"""

FX_CI = """name: CI
on:
  pull_request:
jobs:
  lint:
    runs-on: ubuntu-latest
    steps:
      - run: ./tools/scripts/other_lint.sh
      - run: ./tools/scripts/ci_test_shard.sh --check
  test:
    runs-on: ubuntu-latest
    steps:
      - run: ./tools/scripts/setup_tests.sh
      - run: cargo test -p fx_core
      # doc-pin: fx_core::doc_test reads docs
      # doc-pin: fx_user::user_test reads docs
      # doc-pin: fx_vizd::fx_vizd reads docs
      - run: cargo test -p fx_alpha -p fx_beta
      - run: cd examples/demo && cargo test --workspace
      - name: demo tests
        working-directory: examples/demo
        run: cargo test -p fx_core -p fx_user
"""

FX_EDGES = """# observing, observed, kind, witness
fx_beta\tall\tunattributable\tcrates/fx_beta/tests/beta_test.rs
fx_cli\tfx_fixture\tdlopen\tcrates/fx_cli/tests/load_fixture_test.rs
fx_core\tall\twhole-tree\tcrates/fx_core/tests/walk_test.rs
fx_engine\tfx_core\tcrate-path\tcrates/fx_engine/tests/engine_api_test.rs
"""

FX_FILES = {
    'Cargo.toml': FX_ROOT_MANIFEST,
    'Cargo.lock': FX_LOCK,
    'LICENSE': 'license text\n',
    'README.md': '# fixture\n',
    'rust-toolchain.toml': '[toolchain]\nchannel = "stable"\n',
    '.cargo/config.toml': '[env]\n',
    '.config/nextest.toml': '[profile.default]\n',
    '.github/CODEOWNERS': '* @owner\n',
    '.github/workflows/ci.yml': FX_CI,
    'docs/guide.md': '# guide\n',
    'examples/demo/Cargo.toml': '[workspace]\n[package]\nname = "demo"\n',
    'examples/demo/src/main.rs': 'fn main() {}\n',
    'crates/fx_py/Cargo.toml': '[workspace]\n[package]\nname = "fx_py"\n',
    'crates/fx_py/src/lib.rs': 'pub fn py() {}\n',
    'tools/ci/observation_edges.tsv': FX_EDGES,
    'tools/ci/thing.txt': 'thing\n',
    'tools/scripts/ci_test_router.py': '# the router\n',
    'tools/scripts/other_lint.sh': '#!/bin/sh\n',
    'tools/scripts/setup_tests.sh': '#!/bin/sh\n',
    'crates/cerulion/Cargo.toml': '[package]\nname = "cerulion"\n',
    'crates/cerulion/src/lib.rs': 'pub use fx_core;\npub use fx_macros;\n',
    'crates/fx_alpha/Cargo.toml': '[package]\nname = "fx_alpha"\n',
    'crates/fx_alpha/src/lib.rs': 'pub fn alpha() {}\n',
    'crates/fx_alpha/tests/ui_test.rs': (
        '#[test]\nfn ui() {\n    let t = trybuild::TestCases::new();\n'
        '    t.compile_fail("tests/ui/*.rs");\n}\n'),
    'crates/fx_alpha/tests/ui/case.rs': 'fn main() { fx_user::user(); }\n',
    'crates/fx_alpha/tests/alpha_test.rs': (
        '#[test]\nfn reads_tools() {\n'
        '    let _ = std::fs::read_to_string("../../tools/ci/thing.txt");\n}\n'),
    'crates/fx_android/Cargo.toml': '[package]\nname = "fx_android"\n',
    'crates/fx_android/src/lib.rs': (
        'pub const HELP: &str = "docs/manual.md";\n\n'
        'pub fn start() {\n'
        '    let _ = std::thread::Builder::new()\n'
        '        .name("fx-bagd".to_string())\n'
        '        .spawn(|| fx_bagd::run());\n}\n'),
    'crates/fx_android/tests/runtime_test.rs': (
        '#[test]\nfn starts() {\n    fx_android::start();\n'
        '    let _ = std::path::Path::new("libfx_fixture.so");\n}\n'),
    'crates/fx_bagd/Cargo.toml': '[package]\nname = "fx_bagd"\n',
    'crates/fx_bagd/build.rs': (
        'fn main() {\n'
        '    println!("cargo:rerun-if-changed=build.rs");\n'
        '    println!("cargo:rerun-if-changed=schema");\n}\n'),
    'crates/fx_bagd/notes.md': 'notes\n',
    'crates/fx_bagd/schema/a.yaml': 'name: a\n',
    'crates/fx_bagd/src/lib.rs': (
        'pub fn run() {}\n\n#[cfg(test)]\nmod tests {\n    #[test]\n    fn spawns_netd() {\n'
        '        let _ = std::process::Command::new("fx-netd").status();\n    }\n}\n'),
    'crates/fx_bagd/src/main.rs': 'fn main() { fx_bagd::run(); }\n',
    'crates/fx_beta/Cargo.toml': '[package]\nname = "fx_beta"\n',
    'crates/fx_beta/notes.txt': 'notes\n',
    'crates/fx_beta/src/lib.rs': 'mod gone;\npub fn beta() {}\n',
    'crates/fx_beta/tests/beta_test.rs': (
        '#[test]\nfn loads_by_pattern() {\n    let name = "x";\n'
        '    let _ = format!("crates/{name}/lib{name}.so");\n}\n'),
    'crates/fx_cli/Cargo.toml': '[package]\nname = "fx_cli"\n',
    'crates/fx_cli/src/main.rs': 'fn main() { fx_engine::run(); }\n',
    'crates/fx_cli/tests/build_test.rs': (
        '#[test]\nfn builds_a_scaffold() {\n'
        '    let _ = std::process::Command::new("cargo").arg("check").status();\n}\n'),
    'crates/fx_cli/tests/cli_e2e_test.rs': (
        '#[test]\nfn runs_the_cli() {\n'
        '    let _ = std::process::Command::new(env!("CARGO_BIN_EXE_cerulion"))'
        '.arg("status").status();\n}\n'),
    'crates/fx_cli/tests/load_fixture_test.rs': (
        '#[test]\nfn loads_the_fixture() {\n'
        '    let _ = std::path::Path::new("target/debug/libfx_fixture.so");\n}\n'),
    'crates/fx_cli/tests/node_build_test.rs': (
        '#[test]\nfn builds_a_node() {\n'
        '    let _ = std::process::Command::new(env!("CARGO_BIN_EXE_cerulion"))'
        '.args(["node", "build"]).status();\n}\n'),
    'crates/fx_core/AGENTS.md': '# core\n',
    'crates/fx_core/Cargo.toml': '[package]\nname = "fx_core"\n',
    'crates/fx_core/data/table.txt': 'a b c\n',
    'crates/fx_core/src/lib.rs': (
        'mod wire;\nmod net {\n    pub mod inner;\n}\n#[path = "netimpl"]\nmod net2 {\n'
        '    pub mod deep;\n}\n#[cfg(test)]\nmod unit_tests;\n\n'
        'pub const TABLE: &str = include_str!("../data/table.txt");\n\npub fn core() {}\n'),
    'crates/fx_core/src/net/inner.rs': 'pub fn inner() {}\n',
    'crates/fx_core/src/netimpl/deep.rs': 'pub fn deep() {}\n',
    'crates/fx_core/src/unit_tests.rs': '#[test]\nfn unit() {}\n',
    'crates/fx_core/src/wire.rs': 'pub fn wire() {}\n',
    'crates/fx_core/tests/common/mod.rs': 'pub fn helper() {}\n',
    'crates/fx_core/tests/data/x.json': '{}\n',
    'crates/fx_core/tests/doc_test.rs': (
        '#[test]\nfn reads_the_guide() {\n'
        '    let _ = std::fs::read_to_string("../../docs/guide.md");\n}\n'),
    'crates/fx_core/tests/engine_test.rs': '#[test]\nfn uses_the_engine() { fx_engine::run(); }\n',
    'crates/fx_core/tests/fixture_test.rs': (
        '#[test]\nfn reads_its_fixture() {\n'
        '    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));\n'
        '    let _ = std::fs::read_to_string(dir.join("tests/data/x.json"));\n'
        '    let _ = std::fs::read_to_string(dir.join("../../Cargo.toml"));\n}\n'),
    'crates/fx_core/tests/plain_test.rs': (
        'mod common;\n#[test]\nfn plain() { common::helper(); fx_core::core(); }\n'),
    'crates/fx_core/tests/walk_test.rs': (
        'fn repo_root() -> std::path::PathBuf { std::path::PathBuf::from("../..") }\n'
        '#[test]\nfn walks() { let _ = std::fs::read_dir(repo_root()); }\n'),
    'crates/fx_engine/Cargo.toml': '[package]\nname = "fx_engine"\n',
    'crates/fx_engine/src/doctor.rs': (
        'pub fn check() {\n    let _ = std::env::current_exe()\n        .ok()\n'
        '        .and_then(|exe| exe.parent().map(|d| d.join("fx-netd")));\n}\n'),
    'crates/fx_engine/src/lib.rs': (
        'mod doctor;\nmod node_build;\nmod viz_client;\n\n'
        'pub const PARSE: &str = include_str!("../../fx_macros/src/parse.rs");\n\n'
        'pub fn run() {\n    viz_client::start();\n    doctor::check();\n'
        '    node_build::build();\n}\n'),
    'crates/fx_engine/src/node_build.rs': (
        'pub fn build() {\n'
        '    let _ = std::process::Command::new("cargo").arg("build").status();\n}\n'),
    'crates/fx_engine/src/viz_client.rs': (
        'pub const VIZD_BINARY: &str = "fx-vizd";\n\npub fn start() {\n'
        '    let _ = std::process::Command::new(VIZD_BINARY).status();\n}\n'),
    'crates/fx_engine/tests/engine_api_test.rs': (
        '#[test]\nfn reads_core_data() {\n'
        '    let _ = std::fs::read_to_string("../fx_core/tests/data/x.json");\n'
        '    let _ = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), '
        '"/../fx_bagd/notes.md"));\n'
        '    fx_engine::run();\n}\n'),
    'crates/fx_fixture/Cargo.toml': '[package]\nname = "fx_fixture"\n',
    'crates/fx_fixture/src/lib.rs': '#[no_mangle]\npub extern "C" fn fixture() {}\n',
    'crates/fx_macros/Cargo.toml': '[package]\nname = "fx_macros"\n',
    'crates/fx_macros/src/crate_root.rs': (
        'pub fn consumer_manifest() -> Option<String> {\n'
        '    std::env::var_os("CARGO_MANIFEST_DIR")\n'
        '        .map(|dir| std::path::PathBuf::from(dir).join("Cargo.toml"))\n'
        '        .and_then(|path| std::fs::read_to_string(path).ok())\n}\n'),
    'crates/fx_macros/src/lib.rs': 'mod crate_root;\nmod parse;\n',
    'crates/fx_macros/src/parse.rs': 'pub fn parse() {}\n',
    'crates/fx_netd/Cargo.toml': '[package]\nname = "fx_netd"\n',
    'crates/fx_netd/src/client.rs': 'pub const NETD_BIN_NAME: &str = "fx-netd";\n',
    'crates/fx_netd/fixtures/n.bin': 'bytes\n',
    'crates/fx_netd/src/lib.rs': (
        'pub mod client;\n\npub fn spawn() {\n'
        '    let _ = std::process::Command::new(client::NETD_BIN_NAME).status();\n}\n\n'
        'pub fn serve() {}\n\n#[test]\nfn reads_a_fixture() {\n'
        '    let _ = std::fs::read("fixtures/n.bin");\n}\n'),
    'crates/fx_netd/src/main.rs': 'fn main() { fx_netd::serve(); }\n',
    'crates/fx_user/Cargo.toml': '[package]\nname = "fx_user"\n',
    'crates/fx_user/src/lib.rs': 'pub fn user() {}\n',
    'crates/fx_user/fixtures/a.bin': 'bytes\n',
    'crates/fx_user/tests/user_test.rs': (
        '#[test]\nfn uses() {\n    fx_user::user();\n'
        '    let _ = std::fs::read("fixtures/a.bin");\n}\n'),
    'crates/fx_vizd/Cargo.toml': '[package]\nname = "fx_vizd"\n',
    'crates/fx_vizd/src/extra.rs': 'pub fn extra() {}\n',
    'crates/fx_vizd/src/main.rs': 'include!("parts.rs");\nfn main() {}\n',
    'crates/fx_vizd/src/parts.rs': 'mod extra;\n',
}


def fx_package(name, targets, deps=()):
    """One `cargo metadata` package entry of the fixture."""
    directory = '%s/crates/%s' % (FX_ROOT, name)
    out_targets = []
    for kind, target_name, src, doctest in targets:
        out_targets.append({'kind': [kind] if isinstance(kind, str) else list(kind),
                            'name': target_name, 'src_path': '%s/%s' % (directory, src),
                            'test': kind not in ('custom-build', 'example'),
                            'doctest': doctest})
    out_deps = []
    for dep_name, kind, member in deps:
        entry = {'name': dep_name, 'kind': kind}
        if member:
            entry['path'] = '%s/crates/%s' % (FX_ROOT, dep_name)
        out_deps.append(entry)
    return {'name': name, 'id': 'path+file://%s#%s@0.1.0' % (directory, name),
            'manifest_path': directory + '/Cargo.toml', 'targets': out_targets,
            'dependencies': out_deps}


def fx_metadata(extra_targets=()):
    lib = lambda name, doctest=False: ('lib', name, 'src/lib.rs', doctest)  # noqa: E731
    test = lambda name: ('test', name, 'tests/%s.rs' % name, False)  # noqa: E731
    packages = [
        fx_package('cerulion', [lib('cerulion')],
                   [('fx_core', None, True), ('fx_macros', None, True)]),
        fx_package('fx_alpha', [lib('fx_alpha'), test('alpha_test'), test('ui_test')] +
                   list(extra_targets),
                   [('serde', None, False), ('fx_user', 'dev', True)]),
        fx_package('fx_android', [lib('fx_android'), test('runtime_test')],
                   [('fx_bagd', None, True), ('fx_core', None, True)]),
        fx_package('fx_bagd', [lib('fx_bagd'), ('bin', 'fx-bagd', 'src/main.rs', False),
                               ('custom-build', 'build-script-build', 'build.rs', False)],
                   [('fx_core', None, True)]),
        fx_package('fx_beta', [lib('fx_beta'), test('beta_test')]),
        fx_package('fx_cli', [('bin', 'cerulion', 'src/main.rs', False), test('build_test'),
                              test('cli_e2e_test'), test('load_fixture_test'),
                              test('node_build_test')],
                   [('fx_engine', None, True)]),
        fx_package('fx_core', [lib('fx_core', True), test('doc_test'), test('engine_test'),
                               test('fixture_test'), test('plain_test'), test('walk_test')],
                   [('fx_engine', 'dev', True)]),
        fx_package('fx_engine', [lib('fx_engine'), test('engine_api_test')],
                   [('fx_core', None, True), ('fx_netd', None, True),
                    ('tempfile', 'dev', False)]),
        fx_package('fx_fixture', [('cdylib', 'fx_fixture', 'src/lib.rs', False)],
                   [('fx_core', None, True)]),
        fx_package('fx_macros', [('proc-macro', 'fx_macros', 'src/lib.rs', False)]),
        fx_package('fx_netd', [lib('fx_netd'), ('bin', 'fx-netd', 'src/main.rs', False)],
                   [('fx_core', None, True)]),
        fx_package('fx_user', [lib('fx_user'), test('user_test')], [('fx_macros', None, True)]),
        fx_package('fx_vizd', [('bin', 'fx-vizd', 'src/main.rs', False)],
                   [('fx_core', None, True)]),
    ]
    return {'packages': packages, 'workspace_members': [p['id'] for p in packages],
            'workspace_root': FX_ROOT, 'version': 1}


# The fixture's 31 test binaries, typed out.
FX_ALL = frozenset((
    'cerulion', 'fx_alpha', 'fx_alpha::alpha_test', 'fx_alpha::ui_test', 'fx_android',
    'fx_android::runtime_test',
    'fx_bagd', 'fx_bagd::bin/fx-bagd', 'fx_beta', 'fx_beta::beta_test',
    'fx_cli::bin/cerulion', 'fx_cli::build_test', 'fx_cli::cli_e2e_test',
    'fx_cli::load_fixture_test', 'fx_cli::node_build_test', 'fx_core', 'fx_core::doc_test',
    'fx_core::doctest/fx_core', 'fx_core::engine_test', 'fx_core::fixture_test',
    'fx_core::plain_test', 'fx_core::walk_test', 'fx_engine', 'fx_engine::engine_api_test',
    'fx_fixture', 'fx_macros', 'fx_netd', 'fx_netd::bin/fx-netd', 'fx_user',
    'fx_user::user_test', 'fx_vizd::bin/fx-vizd',
))
# Run on any non-empty change: a whole-tree walker, an unattributable reader,
# a test that runs cargo, and a test that drives the CLI's build verb.
FX_ALWAYS = frozenset(('fx_core::walk_test', 'fx_beta::beta_test', 'fx_cli::build_test',
                       'fx_cli::node_build_test'))
# Every binary that links fx_engine: the engine's own, fx_core's two that name
# it (engine_test, and the doctests, which may use any dev-dependency), and
# fx_cli's, whose package depends on it.
FX_ENGINE = frozenset(('fx_engine', 'fx_engine::engine_api_test', 'fx_core::engine_test',
                       'fx_core::doctest/fx_core', 'fx_cli::bin/cerulion', 'fx_cli::build_test',
                       'fx_cli::cli_e2e_test', 'fx_cli::load_fixture_test',
                       'fx_cli::node_build_test'))
# Every binary that spawns or loads a workspace artifact: the engine's linkers
# (fx-vizd, fx-netd), fx_netd's (its library spawns fx-netd through a const),
# fx_bagd's library unit tests (a cfg(test) spawn of fx-netd), and
# fx_android's test, which loads the fixture cdylib.
FX_ARTIFACT = FX_ENGINE | frozenset(('fx_netd', 'fx_netd::bin/fx-netd', 'fx_bagd',
                                     'fx_android::runtime_test'))
FX_CORE_UNITS = frozenset(('fx_core', 'fx_core::doc_test', 'fx_core::doctest/fx_core',
                           'fx_core::engine_test', 'fx_core::fixture_test',
                           'fx_core::plain_test', 'fx_core::walk_test'))
# Every binary whose closure holds fx_core.
FX_CORE_LINKERS = FX_ALL - frozenset(('fx_alpha', 'fx_alpha::alpha_test', 'fx_alpha::ui_test',
                                      'fx_beta', 'fx_macros', 'fx_user', 'fx_user::user_test'))
FX_ALPHA = frozenset(('fx_alpha', 'fx_alpha::alpha_test', 'fx_alpha::ui_test'))

FX_PLAIN_WITH_HELPER = 'mod common;\nmod old_helper;\n#[test]\nfn plain() {}\n'

FX_DOCTOR_ANCHORED = (
    'pub fn check() {\n    let _ = std::env::current_exe()\n        .ok()\n'
    '        .and_then(|exe| exe.parent().map(|d| d.join("fx-netd")));\n'
    '    let schema = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fx_bagd/schema");\n'
    '    let _ = std::fs::read_dir(schema);\n}\n')

# The foreign workspace unit a row selects: any change to a workspace library or
# binary, to a shared tree, or to a path outside every member can reach it.
FX_FOREIGN = ['crates/fx_py', 'examples/demo']

# `(name, changed paths, base overrides, head overrides, expected selection,
#   [(binary or '*', reason substring)], expected selected foreign units)`.
# An override of `None` deletes the path from that tree. A reason row names a
# binary whose reasons must carry the substring, or `*` for the global list.
FX_ROWS = [
    ('a test file selects its binary and the readers of its tree',
     ['crates/fx_core/tests/plain_test.rs'], {}, {},
     FX_ALWAYS | {'fx_core::plain_test', 'fx_core::fixture_test'},
     [('fx_core::plain_test', 'source crates/fx_core/tests/plain_test.rs'),
      ('fx_core::fixture_test', 'reads fx_core')], []),
    ('a shared test module selects the binary that declares it, not its siblings',
     ['crates/fx_core/tests/common/mod.rs'], {}, {},
     FX_ALWAYS | {'fx_core::plain_test', 'fx_core::fixture_test'},
     [('fx_core::plain_test', 'source crates/fx_core/tests/common/mod.rs')], []),
    ('a cfg(test) module file selects only the library unit tests',
     ['crates/fx_core/src/unit_tests.rs'], {}, {},
     FX_ALWAYS | {'fx_core', 'fx_core::fixture_test'},
     [('fx_core', 'test-only source crates/fx_core/src/unit_tests.rs')], []),
    ('a library source selects every binary whose closure holds it',
     ['crates/fx_core/src/wire.rs'], {}, {},
     FX_CORE_LINKERS | FX_ALWAYS,
     [('fx_vizd::bin/fx-vizd', 'links fx_core: source crates/fx_core/src/wire.rs')], FX_FOREIGN),
    ('a file inside an inline module is a library source',
     ['crates/fx_core/src/net/inner.rs'], {}, {},
     FX_CORE_LINKERS | FX_ALWAYS, [], FX_FOREIGN),
    ('an include_str! file is a library source',
     ['crates/fx_core/data/table.txt'], {}, {},
     FX_CORE_LINKERS | FX_ALWAYS,
     [('fx_bagd', 'links fx_core: source crates/fx_core/data/table.txt')], FX_FOREIGN),
    ('hole 3: a dev-dependency source selects only the binaries that name it',
     ['crates/fx_engine/src/viz_client.rs'], {}, {},
     FX_ENGINE | FX_ALWAYS,
     [('fx_core::engine_test', 'links fx_engine')], FX_FOREIGN),
    ('hole 3: a dev-dependency manifest selects every binary of the package that dev-depends',
     ['crates/fx_engine/Cargo.toml'], {}, {},
     FX_ENGINE | FX_CORE_UNITS | FX_ARTIFACT | FX_ALWAYS,
     [('fx_core::plain_test', 'manifest crates/fx_engine/Cargo.toml (test build of fx_core)'),
      ('fx_netd', 'manifest crates/fx_engine/Cargo.toml (workspace-built artifact)')], FX_FOREIGN),
    ('hole 1: a daemon binary spawned from library code selects every linker, transitively',
     ['crates/fx_netd/src/main.rs'], {}, {},
     FX_ENGINE | FX_ALWAYS | {'fx_netd', 'fx_netd::bin/fx-netd', 'fx_bagd'},
     [('fx_cli::cli_e2e_test', 'spawns fx-netd'), ('fx_core::engine_test', 'spawns fx-netd'),
      ('fx_bagd', 'spawns fx-netd')], FX_FOREIGN),
    ('hole 1: a binary named by a const beside Command selects the spawners',
     ['crates/fx_vizd/src/main.rs'], {}, {},
     FX_ENGINE | FX_ALWAYS | {'fx_vizd::bin/fx-vizd'},
     [('fx_cli::bin/cerulion', 'spawns fx-vizd')], FX_FOREIGN),
    ('hole 1: a thread named like a daemon is a link, not a spawn',
     ['crates/fx_bagd/src/lib.rs'], {}, {},
     FX_ALWAYS | {'fx_bagd', 'fx_bagd::bin/fx-bagd', 'fx_android', 'fx_android::runtime_test'},
     [('fx_android::runtime_test', 'links fx_bagd')], FX_FOREIGN),
    ('hole 1: the thread-named binary itself is spawned by nobody',
     ['crates/fx_bagd/src/main.rs'], {}, {},
     FX_ALWAYS | {'fx_bagd::bin/fx-bagd'}, [], FX_FOREIGN),
    ('hole 2: a consumer manifest the proc macro reads selects the consumer',
     ['crates/fx_user/Cargo.toml'], {}, {},
     FX_ALWAYS | FX_ARTIFACT | FX_ALPHA | {'fx_user', 'fx_user::user_test', 'fx_beta'},
     [('fx_user::user_test', 'manifest crates/fx_user/Cargo.toml'),
      ('fx_alpha::alpha_test', 'manifest crates/fx_user/Cargo.toml (test build of fx_alpha)'),
      ('fx_beta', 'manifest crates/fx_user/Cargo.toml (shares a cargo test command)')],
     FX_FOREIGN),
    ('a proc-macro source selects its users and the library that includes it',
     ['crates/fx_macros/src/parse.rs'], {}, {},
     FX_ALWAYS | FX_ENGINE | {'fx_macros', 'cerulion', 'fx_user', 'fx_user::user_test',
                              'fx_alpha::ui_test'},
     [('fx_engine', 'links fx_engine: source crates/fx_macros/src/parse.rs')], FX_FOREIGN),
    ('a dlopened cdylib selects its loaders, by table row and by literal',
     ['crates/fx_fixture/src/lib.rs'], {}, {},
     FX_ALWAYS | {'fx_fixture', 'fx_cli::load_fixture_test', 'fx_android::runtime_test'},
     [('fx_cli::load_fixture_test', 'links fx_fixture'),
      ('fx_android::runtime_test', 'links fx_fixture')], FX_FOREIGN),
    ('a crate-path read of one file selects only the binary that names it',
     ['crates/fx_core/tests/data/x.json'], {}, {},
     FX_ALWAYS | {'fx_engine::engine_api_test', 'fx_core::fixture_test'},
     [('fx_engine::engine_api_test', 'reads crates/fx_core/tests/data/x.json')], []),
    ('a build-script input selects the package and its users',
     ['crates/fx_bagd/schema/a.yaml'], {}, {},
     FX_ALWAYS | {'fx_bagd', 'fx_bagd::bin/fx-bagd', 'fx_android', 'fx_android::runtime_test'},
     [('fx_bagd', 'links fx_bagd: build input crates/fx_bagd/schema/a.yaml')], FX_FOREIGN),
    ('a file the build script does not name is no build input, and a manifest-dir read is one',
     ['crates/fx_bagd/notes.md'], {}, {}, FX_ALWAYS | {'fx_engine::engine_api_test'},
     [('fx_engine::engine_api_test', 'reads crates/fx_bagd/notes.md')], []),
    ('an unresolvable mod widens its target to the package directory',
     ['crates/fx_beta/notes.txt'], {}, {},
     FX_ALWAYS | {'fx_beta'},
     [('fx_beta', 'links fx_beta: widened source crates/fx_beta/notes.txt')], FX_FOREIGN),
    ('a doc-pinned tree selects its readers, by marker and by literal',
     ['docs/guide.md'], {}, {},
     FX_ALWAYS | {'fx_core::doc_test', 'fx_user::user_test', 'fx_vizd::bin/fx-vizd'},
     [('fx_core::doc_test', 'reads docs'), ('fx_user::user_test', 'reads docs'),
      ('fx_vizd::bin/fx-vizd', 'reads docs')], FX_FOREIGN),
    ('a tools file no test job runs selects its readers',
     ['tools/scripts/other_lint.sh'], {}, {}, FX_ALWAYS | {'fx_alpha::alpha_test'},
     [('fx_alpha::alpha_test', 'reads tools')], FX_FOREIGN),
    ('a harness file selects its readers only',
     ['harness/run_checks.sh'], {}, {}, FX_ALWAYS, [], FX_FOREIGN),
    ('a .github file other than ci.yml selects its readers only',
     ['.github/CODEOWNERS'], {}, {}, FX_ALWAYS, [], FX_FOREIGN),
    ('a foreign workspace path under a shared tree selects every foreign workspace',
     ['examples/demo/src/main.rs'], {}, {}, FX_ALWAYS, [], FX_FOREIGN),
    ('a foreign workspace path under crates/ selects that workspace alone',
     ['crates/fx_py/src/lib.rs'], {}, {}, FX_ALWAYS, [], ['crates/fx_py']),
    ('a trybuild driver compiles its cases against every dev-dependency',
     ['crates/fx_user/src/lib.rs'], {}, {},
     FX_ALWAYS | {'fx_user', 'fx_user::user_test', 'fx_alpha::ui_test'},
     [('fx_alpha::ui_test', 'links fx_user: source crates/fx_user/src/lib.rs')], FX_FOREIGN),
    ('a trybuild case file is read from its own tree',
     ['crates/fx_alpha/tests/ui/case.rs'], {}, {}, FX_ALWAYS | {'fx_alpha::ui_test'},
     [('fx_alpha::ui_test', 'reads fx_alpha')], []),
    ('a path a test reads relative to its package selects it',
     ['crates/fx_user/fixtures/a.bin'], {}, {}, FX_ALWAYS | {'fx_user::user_test'},
     [('fx_user::user_test', 'reads crates/fx_user/fixtures/a.bin')], []),
    ('a #[test] function in a library file reads for the unit tests alone',
     ['crates/fx_netd/fixtures/n.bin'], {}, {}, FX_ALWAYS | {'fx_netd'},
     [('fx_netd', 'reads crates/fx_netd/fixtures/n.bin')], []),
    ('a module under an inline module with a path attribute is a library source',
     ['crates/fx_core/src/netimpl/deep.rs'], {}, {}, FX_CORE_LINKERS | FX_ALWAYS,
     [('fx_bagd', 'links fx_core: source crates/fx_core/src/netimpl/deep.rs')], FX_FOREIGN),
    ('a module an include!d file declares belongs to the including target',
     ['crates/fx_vizd/src/extra.rs'], {}, {}, FX_ENGINE | FX_ALWAYS | {'fx_vizd::bin/fx-vizd'},
     [('fx_vizd::bin/fx-vizd', 'source crates/fx_vizd/src/extra.rs'),
      ('fx_cli::cli_e2e_test', 'spawns fx-vizd: source crates/fx_vizd/src/extra.rs')],
     FX_FOREIGN),
    ('library code that runs cargo links the umbrella crate a scaffold builds',
     ['crates/cerulion/src/lib.rs'], {}, {}, FX_ENGINE | FX_ALWAYS | {'cerulion'},
     [('fx_engine::engine_api_test', 'links cerulion: source crates/cerulion/src/lib.rs')],
     FX_FOREIGN),
    ('the empty change selects nothing, not even the walkers',
     [], {}, {}, frozenset(), [], []),
    ('a file deleted in the head is still attributed through the base',
     ['crates/fx_core/tests/old_helper.rs'],
     {'crates/fx_core/tests/old_helper.rs': 'pub fn old() {}\n',
      'crates/fx_core/tests/plain_test.rs': FX_PLAIN_WITH_HELPER},
     {'crates/fx_core/tests/plain_test.rs': FX_PLAIN_WITH_HELPER},
     FX_ALWAYS | {'fx_core::plain_test', 'fx_core::fixture_test'},
     [('fx_core::plain_test', 'source crates/fx_core/tests/old_helper.rs')], []),
    ('a lock entry a member depends on selects that member and the artifact consumers',
     ['Cargo.lock'], {}, {'Cargo.lock': FX_LOCK.replace('1.0.1', '1.0.2')},
     FX_ALWAYS | FX_ARTIFACT | FX_ALPHA,
     [('fx_alpha', 'links fx_alpha: Cargo.lock serde 1.0.2'),
      ('fx_bagd', 'Cargo.lock serde 1.0.2 (workspace-built artifact)')], FX_FOREIGN),
    ('a lock entry two registry hops below a dev-dependency selects that package alone',
     ['Cargo.lock'], {}, {'Cargo.lock': FX_LOCK.replace('0.2.1', '0.2.2')},
     FX_ALWAYS | {'fx_engine', 'fx_engine::engine_api_test'},
     [('fx_engine', 'dev-dependency Cargo.lock tempfile 3.0.0')], []),
    ('a comment-only root manifest change selects the readers of its text',
     ['Cargo.toml'], {}, {'Cargo.toml': FX_ROOT_MANIFEST.replace(
         '# The fixture members.', '# The fixture members, listed.\n')},
     FX_ALWAYS | {'fx_core::fixture_test'},
     [('fx_core::fixture_test', 'reads the root Cargo.toml text')], []),
    ('a member manifest selects the packages that share its cargo test command',
     ['crates/fx_alpha/Cargo.toml'], {}, {},
     FX_ALWAYS | FX_ARTIFACT | FX_ALPHA | {'fx_beta'},
     [('fx_beta', 'manifest crates/fx_alpha/Cargo.toml (shares a cargo test command)')],
     FX_FOREIGN),
    ('an edge whose witness cannot be narrowed applies to every test of the package',
     ['crates/fx_core/tests/plain_test.rs'],
     {'tools/ci/observation_edges.tsv':
      FX_EDGES + 'fx_alpha\tfx_core\tcrate-path\tcrates/fx_alpha/tests/alpha_test.rs\n'},
     {'tools/ci/observation_edges.tsv':
      FX_EDGES + 'fx_alpha\tfx_core\tcrate-path\tcrates/fx_alpha/tests/alpha_test.rs\n'},
     FX_ALWAYS | {'fx_core::plain_test', 'fx_core::fixture_test', 'fx_alpha::alpha_test',
                  'fx_alpha::ui_test'},
     [('fx_alpha::alpha_test', 'reads fx_core')], []),
    ('an anchored read in library code binds every linker',
     ['crates/fx_bagd/schema/a.yaml'],
     {'crates/fx_engine/src/doctor.rs': FX_DOCTOR_ANCHORED,
      'tools/ci/observation_edges.tsv':
      FX_EDGES + 'fx_engine\tfx_bagd\tcrate-path\tcrates/fx_engine/src/doctor.rs\n'},
     {'crates/fx_engine/src/doctor.rs': FX_DOCTOR_ANCHORED,
      'tools/ci/observation_edges.tsv':
      FX_EDGES + 'fx_engine\tfx_bagd\tcrate-path\tcrates/fx_engine/src/doctor.rs\n'},
     FX_ALWAYS | FX_ENGINE | {'fx_bagd', 'fx_bagd::bin/fx-bagd', 'fx_android',
                              'fx_android::runtime_test'},
     [('fx_cli::cli_e2e_test', 'reads crates/fx_bagd/schema/')], FX_FOREIGN),
]

# Global inputs: `(name, changed paths, base overrides, head overrides, reason)`.
FX_GLOBAL_ROWS = [
    ('ci.yml', ['.github/workflows/ci.yml'], {}, {}, 'ci.yml'),
    ('a composite action', ['.github/actions/setup/action.yml'], {}, {}, 'composite-action'),
    ('.cargo', ['.cargo/config.toml'], {}, {}, '.cargo'),
    ('.config', ['.config/nextest.toml'], {}, {}, '.config'),
    ('the router', ['tools/scripts/ci_test_router.py'], {}, {}, 'router'),
    ('the observation edges', ['tools/ci/observation_edges.tsv'], {}, {}, 'router-input'),
    ('a script a test job runs', ['tools/scripts/setup_tests.sh'], {}, {},
     'ci-machinery:tools/scripts/setup_tests.sh'),
    ('the toolchain file', ['rust-toolchain.toml'], {}, {}, 'rust-toolchain'),
    ('an unknown root file', ['LICENSE'], {}, {}, 'unknown-path'),
    ('a crates path no member owns', ['crates/orphan.txt'], {}, {'crates/orphan.txt': 'x\n'},
     'unknown-path'),
    ('a root profile', ['Cargo.toml'], {},
     {'Cargo.toml': FX_ROOT_MANIFEST.replace('debug = 1', 'debug = 0')},
     'root-manifest:[profile.dev]'),
    ('root workspace dependencies', ['Cargo.toml'], {},
     {'Cargo.toml': FX_ROOT_MANIFEST.replace('serde = "1"', 'serde = "1.0.2"')},
     'root-manifest:[workspace.dependencies]'),
    ('a lock this reader cannot parse', ['Cargo.lock'], {},
     {'Cargo.lock': FX_LOCK + 'this is not toml\n'}, 'lock-unreadable'),
    ('a lock header', ['Cargo.lock'], {},
     {'Cargo.lock': FX_LOCK.replace('version = 4', 'version = 3')}, 'lock-header'),
    ('a source-less lock entry no member owns', ['Cargo.lock'], {},
     {'Cargo.lock': FX_LOCK + '\n[[package]]\nname = "stray"\nversion = "0.1.0"\n'},
     'lock-unplaced:stray'),
    ('an edge naming no member', ['docs/guide.md'],
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'fx_ghost\tfx_core\tcrate-path\tx.rs\n'},
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'fx_ghost\tfx_core\tcrate-path\tx.rs\n'},
     'observation-edge-unknown:fx_ghost'),
    ('an edge kind this reader does not know', ['docs/guide.md'],
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'fx_core\tfx_bagd\tteleport\tx.rs\n'},
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'fx_core\tfx_bagd\tteleport\tx.rs\n'},
     'observation-edge-kind:teleport'),
    ('a doc-pin marker naming no member', ['docs/guide.md'],
     {'.github/workflows/ci.yml': FX_CI + '      # doc-pin: fx_ghost::t reads docs\n'},
     {'.github/workflows/ci.yml': FX_CI + '      # doc-pin: fx_ghost::t reads docs\n'},
     'doc-pin-unknown:fx_ghost'),
    ('a target file the metadata does not list', ['docs/guide.md'], {},
     {'crates/fx_alpha/tests/new_test.rs': '#[test]\nfn new() {}\n'}, 'metadata-mismatch'),
    ('an unreadable observation-edge table', ['crates/fx_core/tests/plain_test.rs'],
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'broken row\n'},
     {'tools/ci/observation_edges.tsv': FX_EDGES + 'broken row\n'},
     'observation-edges-unreadable'),
]


def fx_trees(base_overrides, head_overrides):
    base = dict(FX_FILES)
    head = dict(FX_FILES)
    for tree, overrides in ((base, base_overrides), (head, head_overrides)):
        for path, text in overrides.items():
            if text is None:
                tree.pop(path, None)
            else:
                tree[path] = text
    if 'crates/fx_core/tests/old_helper.rs' in base_overrides:
        head.pop('crates/fx_core/tests/old_helper.rs', None)
    return DictTree(base), DictTree(head)


def fx_dump_all():
    """Every fixture row's document, in row order: the determinism witness."""
    return ''.join(dump(fx_route(changed, base_o, head_o))
                   for _, changed, base_o, head_o, _, _, _ in FX_ROWS)


def fx_route(changed, base_overrides, head_overrides, metadata=None):
    base, head = fx_trees(base_overrides, head_overrides)
    return Router(metadata or fx_metadata(), base, head, changed).run()


def self_test():
    fails = []
    cases = 0

    def check(label, ok, detail):
        if not ok:
            fails.append('%s: %s' % (label, detail))

    for name, changed, base_o, head_o, want, reasons, foreign in FX_ROWS:
        cases += 1
        doc = fx_route(changed, base_o, head_o)
        got = {item['binary'] for item in doc['selected']}
        check(name, got == set(want),
              'extra %s, missing %s' % (sorted(got - set(want)), sorted(set(want) - got)))
        check(name, set(doc['skipped']) == FX_ALL - set(want),
              'skipped %s' % sorted(doc['skipped']))
        by_id = {item['binary']: item['reasons'] for item in doc['selected']}
        for binary, fragment in reasons:
            cases += 1
            carried = by_id.get(binary, [])
            check(name, any(reason.startswith(fragment) for reason in carried),
                  '%s carries %r, wanted %r' % (binary, carried, fragment))
        got_foreign = [item['workspace'] for item in doc['foreign_workspaces']['selected']]
        check(name, got_foreign == foreign, 'foreign %s, wanted %s' % (got_foreign, foreign))
        check(name, not doc['global'], 'global %s' % doc['global'])
        # Determinism: the same input twice, and in reverse order, is the
        # same document byte for byte.
        cases += 1
        again = dump(fx_route(list(reversed(changed)), base_o, head_o))
        check(name, again == dump(doc), 'a second run in reverse order differs')

    for name, changed, base_o, head_o, reason in FX_GLOBAL_ROWS:
        cases += 1
        doc = fx_route(changed, base_o, head_o)
        got = {item['binary'] for item in doc['selected']}
        check(name, got == FX_ALL, 'missing %s' % sorted(FX_ALL - got))
        reasons = [item['reason'] for item in doc['global']]
        check(name, reason in reasons, 'global reasons %s, wanted %s' % (reasons, reason))
        check(name, doc['foreign_workspaces']['selected'] != [], 'a global skipped a workspace')

    # Metadata naming a target the head tree does not hold is a global input.
    cases += 1
    ghost = fx_metadata(extra_targets=[('test', 'ghost_test', 'tests/ghost_test.rs', False)])
    doc = fx_route(['docs/guide.md'], {}, {}, metadata=ghost)
    check('metadata mismatch', {i['binary'] for i in doc['selected']} ==
          FX_ALL | {'fx_alpha::ghost_test'}, 'not everything')
    check('metadata mismatch', any(i['reason'] == 'metadata-mismatch' for i in doc['global']),
          doc['global'])

    # A test binary that names none of its package's dev-dependencies still
    # has them compiled: the built set carries the full test closure.
    cases += 1
    doc = fx_route(['tools/scripts/other_lint.sh'], {}, {})
    check('built closure', 'fx_alpha::alpha_test' in {i['binary'] for i in doc['selected']}
          and 'fx_alpha::ui_test' not in {i['binary'] for i in doc['selected']},
          [i['binary'] for i in doc['selected']])
    check('built closure', 'fx_user' in doc['built_packages'], doc['built_packages'])

    # The member-test-only change leaves the foreign workspace skipped.
    cases += 1
    doc = fx_route(['crates/fx_core/tests/plain_test.rs'], {}, {})
    check('foreign skip', doc['foreign_workspaces']['skipped'] == FX_FOREIGN,
          doc['foreign_workspaces'])

    # The unresolvable `mod gone;` is reported, never silently dropped.
    cases += 1
    doc = fx_route(['crates/fx_beta/notes.txt'], {}, {})
    check('widen warning', any('mod gone' in w for w in doc['warnings']), doc['warnings'])

    # Lexer and scanner pins: a `mod` inside a string or a comment is no
    # module, a raw string with a quote inside ends where its hashes say, and
    # a `cfg(test)` inline module's literals are test-only.
    cases += 1
    scan = scan_rust('// mod a;\n/* mod b; /* nested */ mod c; */\nconst S: &str = "mod d;";\n'
                     'const R: &str = r#"a "quoted" mod e;"#;\nmod f;\n'
                     '#[cfg(all(test, unix))]\nmod tests { const X: &str = "fx-netd"; }\n'
                     '#[cfg(any(test, feature = "x"))]\nmod g;\n'
                     '#[path = "other/h.rs"]\nmod h;\nfn l<\'a>(x: &\'a str) -> char { \'{\' }\n'
                     '#[cfg_attr(feature = "x", path = "alt.rs")]\nmod k;\n')
    names = [(decl.name, decl.test_only, decl.paths, decl.keeps_default) for decl in scan.mods]
    check('scanner', names == [('f', False, (), True), ('g', False, (), True),
                               ('h', False, ('other/h.rs',), False),
                               ('k', False, ('alt.rs',), True)], names)
    check('scanner', 'fx-netd' in scan.lits_test and 'fx-netd' not in scan.lits_normal,
          (sorted(scan.lits_test), sorted(scan.lits_normal)))
    check('scanner', 'a "quoted" mod e;' in scan.lits_normal, sorted(scan.lits_normal))
    check('scanner', ('S', 'mod d;') in scan.consts and ('R', 'a "quoted" mod e;') in scan.consts,
          scan.consts)
    cases += 1
    scan = scan_rust('const A: &str = include_str!("x.txt");\n'
                     'const B: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), '
                     '"/y.txt"));\n'
                     'include!(concat!(env!("OUT_DIR"), "/gen.rs"));\n'
                     'const C: &str = include_str!(some_path());\n')
    shapes = [(kind, shape, value) for kind, shape, value, _ in scan.includes]
    check('includes', shapes == [('include_str', 'file', 'x.txt'),
                                 ('include_str', 'manifest', '/y.txt'),
                                 ('include', 'out_dir', '/gen.rs'),
                                 ('include_str', 'dynamic', 'some_path()')], shapes)

    # The unattributable and whole-tree shapes, judged on hand-written
    # sources: each spelling the walk derives a row from, and a near miss.
    cases += 1
    base, head = fx_trees({}, {})
    router = Router(fx_metadata(), base, head, [])
    unattributable = router.edge_matcher(None, 'unattributable')
    whole = router.edge_matcher(None, 'whole-tree')
    shapes = [
        (unattributable, 'let p = format!("lib{id}.so");', True),
        (unattributable, 'let p = format!("crates/{pkg}/src");', True),
        (unattributable, 'let p = root.join("crates").join(format!("{pkg}"));', True),
        (unattributable, 'let p = format!("{dir}/out.txt");', False),
        (whole, 'walk(&repo_root());', True),
        (whole, 'let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();\nscan(root);',
         True),
        (whole, 'let file = repo_root().join("Cargo.toml");', False),
        (whole, 'fn repo_root() -> PathBuf { todo() }', False),
        (whole, 'let all = root.join("crates");', True),
    ]
    for matcher, source, want in shapes:
        got = matcher(scan_rust(source), 'all')
        check('shape', got == want, '%r read as %s' % (source, got))

    # A `cfg(test)` attribute on a field or a match arm gates no item: the
    # code after it stays code every linker runs.
    cases += 1
    scan = scan_rust('struct S {\n    #[cfg(test)]\n    hook: u8,\n}\nimpl S {\n'
                     '    fn go() { let _ = std::process::Command::new("fx-netd"); }\n}\n'
                     'fn pick(x: u8) -> u8 {\n    match x {\n        #[cfg(test)]\n'
                     '        0 => 1,\n        _ => 2,\n    }\n}\nmod wire;\n'
                     '#[cfg(test)]\npub(crate) fn only_in_tests() { let _ = "tests-only"; }\n')
    check('field attribute', 'fx-netd' in scan.lits_normal, sorted(scan.lits_test))
    check('arm attribute', [(d.name, d.test_only) for d in scan.mods] == [('wire', False)],
          [(d.name, d.test_only) for d in scan.mods])
    check('item attribute', 'tests-only' in scan.lits_test, sorted(scan.lits_normal))

    # The lock reader resolves the three dependency spellings and refuses a
    # name it cannot place.
    cases += 1
    by_name = {'a': [('a', '1.0.0', 'r1'), ('a', '2.0.0', 'r1')], 'b': [('b', '1.0.0', '')]}
    check('lock deps', resolve_lock_dep('b', by_name) == [('b', '1.0.0', '')], 'bare name')
    check('lock deps', resolve_lock_dep('a 2.0.0', by_name) == [('a', '2.0.0', 'r1')], 'version')
    check('lock deps', resolve_lock_dep('a 1.0.0 (r1)', by_name) == [('a', '1.0.0', 'r1')],
          'source')
    try:
        resolve_lock_dep('zz', by_name)
        check('lock deps', False, 'an unknown name resolved')
    except RouterError:
        pass

    # The doc-pin marker reader and the test-job reader of ci.yml.
    cases += 1
    check('markers', parse_doc_pins(FX_CI) == {('fx_core', 'doc_test'): {'docs'},
                                               ('fx_user', 'user_test'): {'docs'},
                                               ('fx_vizd', 'fx_vizd'): {'docs'}},
          parse_doc_pins(FX_CI))
    check('machinery', sorted(set(ci_tools_paths(FX_CI))) == ['tools/scripts/setup_tests.sh'],
          ci_tools_paths(FX_CI))
    check('co-tested', ci_cotested_groups(FX_CI, {'fx_alpha', 'fx_beta', 'fx_core'}) ==
          [('fx_alpha', 'fx_beta')], ci_cotested_groups(FX_CI, {'fx_alpha', 'fx_beta'}))

    # Determinism across processes: two interpreters with different hash
    # seeds give this process's documents byte for byte, so no set or dict
    # order reaches the output.
    cases += 1
    here = fx_dump_all()
    for seed in ('1', '4242'):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        proc = subprocess.run([sys.executable, '-B', os.path.abspath(__file__), '--fixture-dump'],
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out = proc.stdout.decode('utf-8', 'replace')
        check('hash seed %s' % seed, proc.returncode == 0 and out == here,
              'exit %d, %d bytes against %d' % (proc.returncode, len(out), len(here)))

    if fails:
        for line in fails:
            print('ci_test_router self-test: %s' % line, file=sys.stderr)
        print('ci_test_router: %d self-test case(s) failed' % len(fails), file=sys.stderr)
        return 1
    if cases < 50:
        print('ci_test_router self-test: judged %d cases, under the floor of 50' % cases,
              file=sys.stderr)
        return 1
    print('ci_test_router: self-test OK (%d cases)' % cases)
    return 0


LIVE_BINARY = 'cerulion-vizd'


def live_self_check():
    """Shapes the real workspace must keep. Returns the complaints.

    `--live-check` runs this over the checkout: the wiring to git, to cargo
    metadata and to the committed table and markers. The fixture rows pin the
    rules; these pin shapes on the real tree no rule change should flip, so a
    reader that finds nothing in the real tree reports it. It runs in no CI
    job.
    """
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    metadata = load_metadata(repo, None)
    # The daemon's main file, read off the metadata rather than spelled here,
    # so moving the crate moves this arm with it.
    sources = [target.src for package in build_model(metadata).values()
               for target in package.targets if target.kind == 'bin' and
               target.name == LIVE_BINARY]
    if len(sources) != 1:
        return ['the workspace has %d `%s` binaries, not one' % (len(sources), LIVE_BINARY)]
    reader = BlobReader(repo)
    try:
        head = GitTree(repo, git(repo, 'rev-parse', '--verify', 'HEAD^{commit}').strip(), reader)
        docs = []
        cache = {}
        for _ in range(2):
            docs.append(Router(metadata, head, head, sources, cache).run())
        empty = Router(metadata, head, head, [], cache).run()
    finally:
        reader.close()
    doc = docs[0]
    out = []
    if dump(docs[0]) != dump(docs[1]):
        out.append('two runs over the same tree differ')
    selected = {item['binary']: item['reasons'] for item in doc['selected']}
    if doc['counts']['units'] < 500:
        out.append('only %d test binaries found; the reader is not reaching the tree'
                   % doc['counts']['units'])
    if doc['global']:
        out.append('a vizd source change hit a global input: %s' % doc['global'])
    if 'cerulion_vizd::bin/cerulion-vizd' not in selected:
        out.append('a vizd source change did not select the vizd binary')
    if not any(binary.startswith('cerulion_cli_engine::') and
               any(reason.startswith('spawns cerulion-vizd') for reason in reasons)
               for binary, reasons in selected.items()):
        out.append('no cli_engine test reaches cerulion-vizd through viz_client.rs')
    if 'cerulion_tf' in selected:
        out.append('a vizd source change selected cerulion_tf, which nothing connects to it')
    if empty['selected']:
        out.append('the empty change selected %d binaries' % len(empty['selected']))
    # The command line refuses a head that is not the checkout when it has to
    # read the metadata itself: `cargo metadata` would describe another tree.
    parent = subprocess.run(['git', '-C', repo, 'rev-parse', '--verify', '-q', 'HEAD^1'],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    if parent.returncode == 0:
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            code = main(['--repo', repo, '--base', 'HEAD', '--head',
                         parent.stdout.decode().strip()])
        if code != 2 or 'is not the checkout' not in err.getvalue():
            out.append('a head other than the checkout was not refused (exit %d)' % code)
    return out


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------


def git(repo, *args):
    try:
        return subprocess.run(['git', '-C', repo] + list(args), check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE
                              ).stdout.decode('utf-8', 'surrogateescape')
    except subprocess.CalledProcessError as err:
        raise RouterError('git %s: %s' % (' '.join(args),
                                         err.stderr.decode('utf-8', 'replace').strip()))


def changed_paths(repo, base, head):
    """`git diff --name-only --no-renames base...head`: deletes and adds, never a rename."""
    out = git(repo, '-c', 'core.quotePath=false', 'diff', '--name-only', '--no-renames', '-z',
              '%s...%s' % (base, head))
    return sorted(path for path in out.split('\0') if path)


def load_metadata(repo, path):
    if path:
        with open(path, encoding='utf-8') as handle:
            return json.load(handle)
    try:
        out = subprocess.run(['cargo', 'metadata', '--format-version', '1', '--no-deps'],
                             cwd=repo, check=True, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE).stdout
    except (OSError, subprocess.CalledProcessError) as err:
        raise RouterError('cargo metadata failed: %s' % err)
    return json.loads(out)


def route(repo, base, head, metadata, scan_cache=None, reader=None):
    own_reader = reader is None
    reader = reader or BlobReader(repo)
    try:
        base_sha = git(repo, 'rev-parse', '--verify', '%s^{commit}' % base).strip()
        head_sha = git(repo, 'rev-parse', '--verify', '%s^{commit}' % head).strip()
        base_tree = GitTree(repo, base_sha, reader)
        head_tree = GitTree(repo, head_sha, reader)
        changed = changed_paths(repo, base_sha, head_sha)
        router = Router(metadata, base_tree, head_tree, changed, scan_cache)
        return router.run()
    finally:
        if own_reader:
            reader.close()


def summary_lines(doc):
    counts = doc['counts']
    lines = ['router-shadow: %d of %d test binaries selected, %d skipped (shadow mode: '
             'nothing is gated on this)' % (counts['selected'], counts['units'],
                                            counts['skipped'])]
    for item in doc['global']:
        lines.append('router-shadow: global input %s (%s)' % (item['path'], item['reason']))
    for item in doc['selected']:
        lines.append('router-shadow: run  %s: %s'
                     % (item['binary'], '; '.join(item['reasons'][:3])))
    for name in doc['skipped']:
        lines.append('router-shadow: skip %s' % name)
    for item in doc['foreign_workspaces']['selected']:
        lines.append('router-shadow: run  workspace %s: %s'
                     % (item['workspace'], '; '.join(item['reasons'])))
    for name in doc['foreign_workspaces']['skipped']:
        lines.append('router-shadow: skip workspace %s' % name)
    for warning in doc['warnings']:
        lines.append('router-shadow: warning: %s' % warning)
    return lines


def dump(doc):
    return json.dumps(doc, indent=1, sort_keys=True) + '\n'


def main(argv):
    parser = argparse.ArgumentParser(description='Per-binary test selection, shadow mode.')
    parser.add_argument('--base', help='the base commit')
    parser.add_argument('--head', default='HEAD', help='the head commit (default HEAD)')
    parser.add_argument('--repo', default=None, help='repository root (default: this checkout)')
    parser.add_argument('--metadata', default=None,
                        help='a `cargo metadata --no-deps` document of the head tree')
    parser.add_argument('--json', default=None, help='write the selection document here')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--live-check', action='store_true',
                        help='check shapes the real workspace keeps, over this checkout')
    parser.add_argument('--fixture-dump', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.live_check:
        try:
            complaints = live_self_check()
        except RouterError as err:
            complaints = ['cannot read this checkout: %s' % err]
        for line in complaints:
            print('ci_test_router live check: %s' % line, file=sys.stderr)
        if complaints:
            return 1
        print('ci_test_router: live check OK')
        return 0
    if args.fixture_dump:
        sys.stdout.write(fx_dump_all())
        return 0
    if not args.base:
        parser.error('--base is required')
    repo = args.repo or os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    try:
        if not args.metadata:
            # `cargo metadata` reads the CHECKOUT, so it describes the head
            # only when the head is the checkout.
            checkout = git(repo, 'rev-parse', '--verify', 'HEAD^{commit}').strip()
            wanted = git(repo, 'rev-parse', '--verify', '%s^{commit}' % args.head).strip()
            if checkout != wanted:
                raise RouterError('the head %s is not the checkout %s; check it out, or pass '
                                  '--metadata from a checkout of it' % (wanted, checkout))
        metadata = load_metadata(repo, args.metadata)
        doc = route(repo, args.base, args.head, metadata)
    except RouterError as err:
        print('ci_test_router: %s' % err, file=sys.stderr)
        return 2
    # The document first, so a reader that closes stdout early cannot cost it.
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            handle.write(dump(doc))
    for line in summary_lines(doc):
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
