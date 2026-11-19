#!/usr/bin/env python3
"""ci_router_escapes.py: score a shadow selection against a full run's verdicts.

    python3 -B tools/scripts/ci_router_escapes.py --selection router-shadow.json \\
        [--reruns reruns.json] [--issues issues.json] LOG [LOG ...]
    python3 -B tools/scripts/ci_router_escapes.py --selection router-shadow.json \\
        --per-test per_test.tsv --per-binary per_binary.tsv [--run RUN_ID]
    python3 -B tools/scripts/ci_router_escapes.py --self-test

WHAT IT ANSWERS. `ci_test_router.py` writes a selection in shadow mode; the run
it rode on still ran every test the workflow runs. This script reads that run's
verdicts and sorts every test binary that FAILED in the run while the selection
SKIPPED it into one of two kinds:

  * a FLAKE when it passed on a later attempt at the same head (`--reruns`, a
    JSON object of the selection's `head` commit to the binary ids that passed
    on a rerun) or a flake issue, open or closed, names it (`--issues`, a JSON
    object of key to issue numbers; a key is a binary id, a target name only
    one binary carries, or a binary id and a full failing test path separated
    by a space, and an ambiguous key is refused);
  * an ESCAPE otherwise. Routing stays off until the shadow period shows none.

Both maps are files, so the score is offline and repeatable.

WHERE THE VERDICTS COME FROM. Job logs as GitHub serves them (timestamps and
colour codes are stripped here), or the tab-separated extracts
`per_test.tsv` (nextest: `run, job_id, job, status, seconds, binary, test`)
and `per_binary.tsv` (libtest: `run, job_id, job, source, binary, result,
passed, failed, seconds`). From a log it reads:

  * nextest status lines (`FAIL [ 0.4s] (12/300) pkg::binary test`). A test's
    LAST status decides it: `PASS`, `FLAKY`, `LEAK` and `SKIP` pass, `SLOW`
    never replaces a status (after the run's `Summary` it is a slow pass, before
    it a test still running, with no verdict), and every other word fails it,
    so a later `PASS` of a retried test is a pass and a status this reader has
    not seen is a failure. A run that
    printed `Starting` and no `Summary`, or a test list nextest could not
    build, is UNPLACED;
  * libtest blocks (`Running tests/x.rs (target/debug/deps/x-<hash>)` up to
    its `test result:` line, and `Doc-tests <crate>`), and cargo's own
    `error: test failed, to rerun pass `-p pkg --test x`` line;
  * `error: could not compile `pkg` (test "x")` and its `lib test`, `bin`,
    `example` and `lib` forms. A LIBRARY that failed to compile is an escape
    only when the selection compiles no binary that links it
    (`built_packages`).

WHAT IT REFUSES TO CLAIM. A failure this script cannot place on a binary of the
selection's universe is listed as UNPLACED, and the exit status counts it, so a
log line in a shape nobody taught it never reads as "no escapes". A skipped
binary with no verdict at all in the logs given is listed as UNVERIFIED: "no
escape" says nothing about a binary whose job never ran, so a scorer that only
read the aggregate result of a job could not tell that leg apart from a green
one. Read the logs of every job of the run, by name.

Exit 0 no escape, nothing unplaced and a verdict for every skipped binary; 1
escapes or unplaced failures; 3 no escape, but skipped binaries with no verdict
in the logs given (an example target, which `cargo test` only compiles, never
prints one); 2 usage or an unreadable input. Stdlib only, Python 3.8 or newer.
"""

import argparse
import collections
import contextlib
import csv
import io
import json
import os
import re
import sys

ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
TIMESTAMP = re.compile(r'^\d{4}-\d\d-\d\dT[\d:.]+Z ?')
# nextest: `STATUS [ secs] (n/m) binary-id test-name`, optionally `TRY n`.
NEXTEST = re.compile(r'^\s*(?:TRY\s+\d+\s+)?([A-Z][A-Z-]*(?:\s+\d+/\d+)?)\s+\[\s*[\d.]+s\]\s+'
                     r'(?:\(\s*\d+/\s*\d+\)\s+)?(\S+)\s+(\S+)\s*$')
# A test's LAST status decides it. `SLOW` never replaces a status: mid-run it
# says the test is still going, and in the closing list after `Summary` it is a
# slow PASS (the CI profile prints no `PASS` lines, so that is the only line a
# slow pass gets). A `SLOW` test whose step printed a `Summary` passed; one
# whose step never did has no verdict, and the step is UNPLACED. Every status
# word outside these lists counts as failing.
NEXTEST_PASSING = ('PASS', 'FLAKY', 'LEAK', 'SKIP')
NEXTEST_RUNNING = ('SLOW',)
NEXTEST_STARTING = re.compile(r'^\s*Starting \d+ tests? across \d+ binar')
NEXTEST_SUMMARY = re.compile(r'^\s*Summary \[')
TEST_LIST_FAILED = re.compile(r'error: creating test list failed')
STEP_FAILED = re.compile(r'^##\[error\](?:Process completed with exit code [1-9]\d*|'
                         r'The operation was canceled)')
STEP_END = re.compile(r'^##\[endgroup\]')
RUNNING = re.compile(r'^\s*Running (?:unittests )?(\S+) \((?:\S*/)?deps/([^)\s/]+?)-[0-9a-f]+'
                     r'(?:\.exe)?\)\s*$')
DOC_TESTS = re.compile(r'^\s*Doc-tests (\S+)\s*$')
RESULT = re.compile(r'^\s*test result: (ok|FAILED)\.')
LIBTEST_FAILED = re.compile(r'^\s*test (\S+) \.\.\. FAILED\s*$')
RERUN = re.compile(r'error: (?:test|doctest) failed, to rerun pass [`\'](.+?)[`\']')
COMPILE = re.compile(r'error: could not compile [`\']([^`\']+)[`\'](?: \(([^)]*)\))?')
BUILD_SCRIPT = re.compile(r'error: failed to run custom build command for [`\']([^`\' ]+)')
# The step header GitHub prints: `##[group]Run cargo test -p pkg ...`.
STEP = re.compile(r'^##\[group\]Run (.*)$')
STEP_PACKAGE = re.compile(r'(?:-p|--package)[ =]([A-Za-z0-9_-]+)')


class ScoreError(Exception):
    """An input this script cannot read: exit 2."""


class Universe:
    """The binaries a selection document names, and how logs spell them."""

    def __init__(self, selection):
        if not isinstance(selection, dict) or selection.get('schema') != 1:
            raise ScoreError('the selection is not a ci_test_router schema 1 document')
        units = selection.get('units')
        if not isinstance(units, list) or not units:
            raise ScoreError('the selection lists no units')
        self.units = {}
        for unit in units:
            self.units[unit['binary']] = unit
        self.selected = {item['binary'] for item in selection.get('selected', [])}
        self.skipped = set(selection.get('skipped', []))
        if self.selected | self.skipped != set(self.units) or self.selected & self.skipped:
            raise ScoreError('selected and skipped do not partition the units')
        self.built = set(selection.get('built_packages', []))
        self.head = selection.get('head') or ''
        self.packages = {unit['package'] for unit in self.units.values()}

    def by_rerun(self, args):
        """The unit cargo's `to rerun pass` arguments name."""
        words = args.split()
        package = None
        target = None
        for index, word in enumerate(words):
            if word in ('-p', '--package') and index + 1 < len(words):
                package = words[index + 1]
            elif word in ('--lib', '--doc'):
                target = (word[2:], None)
            elif word in ('--test', '--bin', '--example', '--bench') and index + 1 < len(words):
                target = (word[2:], words[index + 1])
        if package is None or target is None:
            return None
        kind, name = target
        if kind == 'lib':
            return self.lib_unit(package)
        if kind == 'doc':
            for unit_id, unit in self.units.items():
                if unit['package'] == package and unit['kind'] == 'doctest':
                    return unit_id
            return None
        return self.find(package, kind, name)

    def find(self, package, kind, name):
        for unit_id, unit in sorted(self.units.items()):
            if unit['package'] == package and unit['kind'] == kind and unit['target'] == name:
                return unit_id
        return None

    def lib_unit(self, package):
        if package in self.units and self.units[package]['kind'] == 'lib':
            return package
        return None

    def by_running(self, source, crate):
        """Units a libtest `Running <source> (deps/<crate>-<hash>)` line can be.

        The line carries the source relative to its package and the crate name
        with `-` turned into `_`; both must match.
        """
        out = []
        for unit_id, unit in sorted(self.units.items()):
            if unit['kind'] == 'doctest':
                continue
            if unit['target'].replace('-', '_') != crate:
                continue
            if unit['src'] == source or unit['src'].endswith('/' + source):
                out.append(unit_id)
        return out

    def by_doc_tests(self, crate):
        return [unit_id for unit_id, unit in sorted(self.units.items())
                if unit['kind'] == 'doctest' and unit['target'].replace('-', '_') == crate]

    def by_compile(self, package, what):
        """`(unit or None, library package or None)` for a compile failure."""
        what = (what or '').strip()
        if package not in self.packages:
            return None, None
        if what in ('lib', 'build script', ''):
            return None, package
        if what == 'lib test':
            return self.lib_unit(package), None
        match = re.fullmatch(r'(test|bin|example|bench) "([^"]+)"( test)?', what)
        if match:
            return self.find(package, match.group(1), match.group(2)), None
        if what in ('doctest',):
            return self.by_rerun('-p %s --doc' % package), None
        return None, None


class Verdicts:
    """Failed and observed units, with the evidence for each."""

    def __init__(self):
        self.failed = {}
        self.observed = set()
        self.failed_libs = {}
        self.unplaced = []
        # The failing test names a log names for a unit, for the issue map.
        self.tests = {}

    def fail(self, unit_id, evidence, test=None):
        self.failed.setdefault(unit_id, evidence)
        self.observed.add(unit_id)
        if test:
            self.tests.setdefault(unit_id, set()).add(test)


def clean(line):
    return TIMESTAMP.sub('', ANSI.sub('', line.rstrip('\r\n')))


def read_log(text, universe, verdicts, where):
    """Fold one job log's verdicts into `verdicts`.

    A libtest block is a `Running` (or `Doc-tests`) line and the next
    `test result:` line. Cargo writes the first to stderr and the test binary
    writes the second to stdout, and the log interleaves the two streams, so
    a `Running` line can land before the result of the binary ahead of it.
    The blocks are therefore paired in ORDER: each result closes the oldest
    open block. A block still open when its step ends, or at the end of the
    log, had no result at all: the binary died, hung until the job's limit,
    or was cancelled, and that is a failure of it.

    A STEP that failed with no failure placed on a binary (a nextest run that
    never printed its `Summary`, a test list nextest could not build, a
    cancel, a non-zero exit nothing here explains) is UNPLACED: its binaries
    have no verdict, and another job's passing log of the same binaries must
    not stand in for one.
    """
    nextest_final = {}
    pending = collections.deque()
    step = {'packages': set(), 'header': False, 'started': None, 'summary': False,
            'placed': False, 'failed': None, 'index': 0}
    summarized = set()

    def flush(at):
        while pending:
            units, unplaced, opened, identified = pending.popleft()
            step['placed'] = True
            if identified:
                # Already failed by the rerun line that named it.
                continue
            if units:
                for unit_id in units:
                    verdicts.fail(unit_id, '%s, and no test result followed (%s)'
                                  % (opened, at))
            else:
                verdicts.unplaced.append('%s, and no test result followed' % unplaced)
        if step['started'] and not step['summary']:
            verdicts.unplaced.append('%s: a nextest run ended with no Summary (%s)'
                                     % (step['started'], at))
            step['placed'] = True
        if step['failed'] and not step['placed']:
            verdicts.unplaced.append('%s, and no failure in the step names a binary'
                                     % step['failed'])

    def placed():
        step['placed'] = True

    for number, raw in enumerate(text.splitlines(), 1):
        line = clean(raw)
        at = '%s:%d' % (where, number)
        match = STEP.match(line)
        if match:
            flush('%s: the next step began' % at)
            step = {'packages': set(STEP_PACKAGE.findall(match.group(1))), 'header': True,
                    'started': None, 'summary': False, 'placed': False, 'failed': None,
                    'index': step['index'] + 1}
            continue
        if step['header']:
            # The step's script, printed between its `Run` line and
            # `##[endgroup]`: its `-p` arguments say which package a
            # same-named test binary belongs to. The lines are still read
            # below, so a log that never closes the group loses nothing.
            if STEP_END.match(line):
                step['header'] = False
                continue
            step['packages'] |= set(STEP_PACKAGE.findall(line))
        if STEP_FAILED.match(line):
            step['failed'] = '%s: %s' % (at, line.strip())
            continue
        if NEXTEST_STARTING.match(line):
            step['started'] = at
            continue
        if NEXTEST_SUMMARY.match(line):
            step['summary'] = True
            summarized.add(step['index'])
            continue
        if TEST_LIST_FAILED.search(line):
            verdicts.unplaced.append('%s: %s' % (at, line.strip()))
            placed()
            continue
        match = NEXTEST.match(line)
        if match:
            status = match.group(1).split()[0]
            binary, test = match.group(2), match.group(3)
            if status in NEXTEST_RUNNING and (binary, test) in nextest_final:
                continue
            nextest_final[(binary, test)] = (status, at, step['index'])
            if status not in NEXTEST_PASSING and status not in NEXTEST_RUNNING:
                placed()
            continue
        match = RUNNING.match(line) or DOC_TESTS.match(line)
        if match:
            if match.re is RUNNING:
                units = universe.by_running(match.group(1), match.group(2))
                unplaced = '%s: no unit runs %s as %s' % (at, match.group(1), match.group(2))
            else:
                units = universe.by_doc_tests(match.group(1))
                unplaced = '%s: no unit holds the doctests of %s' % (at, match.group(1))
            # Two packages can hold a test of the same file and crate name;
            # the step's own `-p` list says which one this block is.
            narrowed = [unit_id for unit_id in units
                        if universe.units[unit_id]['package'] in step['packages']]
            # The fourth field is set once cargo's rerun line names this
            # block's unit while the block is still open: that unit FAILED, so
            # a passing result that follows belongs to a later block.
            pending.append([narrowed or units, unplaced, '%s: %s' % (at, line.strip()), False])
            continue
        match = LIBTEST_FAILED.match(line)
        unnamed = [block for block in pending if not block[3]]
        if match and len(unnamed) == 1 and len(unnamed[0][0]) == 1:
            # Only when ONE block not yet named by a rerun line is open is it
            # the binary whose output this is: a block that crashed without a
            # result stays open while the next binary prints, and naming that
            # one's tests after it would let another binary's known flake
            # excuse a crash.
            verdicts.tests.setdefault(unnamed[0][0][0], set()).add(match.group(1))
            continue
        match = RESULT.match(line)
        if match and pending:
            if match.group(1) == 'ok':
                # A passing result never closes a block whose unit a rerun line
                # already named as failed.
                index = next((i for i, block in enumerate(pending) if not block[3]), None)
                if index is None:
                    continue
                block = pending[index]
                del pending[index]
            else:
                block = pending.popleft()
            units, unplaced, _, _ = block
            if match.group(1) == 'FAILED':
                placed()
            # A block no unit claims matters only when it FAILED: a passing
            # binary of another workspace is no verdict this score needs.
            if not units and match.group(1) == 'FAILED':
                verdicts.unplaced.append(unplaced)
            for unit_id in units:
                # A block that could be either of two units is a verdict on
                # neither when it passed: only a failure is charged to both.
                if len(units) == 1:
                    verdicts.observed.add(unit_id)
                if match.group(1) == 'FAILED':
                    verdicts.fail(unit_id, '%s: test result: FAILED' % at)
            continue
        match = RERUN.search(line)
        if match:
            placed()
            unit_id = universe.by_rerun(match.group(1))
            if unit_id is None:
                verdicts.unplaced.append('%s:%d: %s' % (where, number, line.strip()))
            else:
                verdicts.fail(unit_id, '%s:%d: %s' % (where, number, line.strip()))
                for block in pending:
                    if not block[3] and block[0] == [unit_id]:
                        block[3] = True
                        break
            continue
        match = BUILD_SCRIPT.search(line)
        if match:
            placed()
            package = match.group(1)
            if package in universe.packages:
                verdicts.failed_libs.setdefault(package, '%s:%d: %s'
                                                % (where, number, line.strip()))
            # A registry crate's build script failing fails every build that
            # needs it: no binary of the selection can be told apart, so it
            # is recorded, not placed.
            else:
                verdicts.unplaced.append('%s:%d: %s' % (where, number, line.strip()))
            continue
        match = COMPILE.search(line)
        if match:
            placed()
            unit_id, library = universe.by_compile(match.group(1), match.group(2))
            if unit_id is not None:
                verdicts.fail(unit_id, '%s:%d: %s' % (where, number, line.strip()))
            elif library is not None:
                verdicts.failed_libs.setdefault(library, '%s:%d: %s'
                                                % (where, number, line.strip()))
            else:
                verdicts.unplaced.append('%s:%d: %s' % (where, number, line.strip()))
    flush('%s: end of log' % where)
    for (binary, test), (status, evidence, index) in sorted(nextest_final.items()):
        if status in NEXTEST_RUNNING:
            if index not in summarized:
                # Still running when the log stopped: no verdict.
                continue
            status = 'PASS'
        if binary not in universe.units:
            if status not in NEXTEST_PASSING:
                verdicts.unplaced.append('%s: nextest binary %s is no unit' % (evidence, binary))
            continue
        verdicts.observed.add(binary)
        if status not in NEXTEST_PASSING:
            # `SLOW` as the last word: the test never finished.
            verdicts.fail(binary, '%s: %s %s %s' % (evidence, status, binary, test), test)


# Every column the two TSV readers read, so a file that lacks one is refused
# with a message rather than failing on a missing key.
PER_TEST_COLUMNS = ('run', 'job_id', 'status', 'binary', 'test')
PER_BINARY_COLUMNS = ('run', 'source', 'binary', 'result')


def read_tsv(path, columns):
    try:
        with open(path, encoding='utf-8', newline='') as handle:
            rows = list(csv.DictReader(handle, delimiter='\t'))
    except OSError as err:
        raise ScoreError('cannot read %s: %s' % (path, err))
    if rows and not set(columns) <= set(rows[0]):
        raise ScoreError('%s lacks the columns %s' % (path, ', '.join(columns)))
    return rows


def read_per_test(path, universe, verdicts, run):
    final = {}
    for number, row in enumerate(read_tsv(path, PER_TEST_COLUMNS), 2):
        if run and row['run'] != run:
            continue
        key = (row['run'], row['job_id'], row['binary'], row['test'])
        # `SLOW` says a test was still running when nextest printed the line,
        # and a finished run lists slow tests again after its Summary: it
        # never replaces a finished status, and a test with no finished
        # status has no verdict, as in `read_log` without a Summary.
        if row['status'] in NEXTEST_RUNNING and key in final:
            continue
        final[key] = (row['status'], '%s:%d' % (path, number))
    for (_, _, binary, test), (status, evidence) in sorted(final.items()):
        if status in NEXTEST_RUNNING:
            continue
        if binary not in universe.units:
            if status not in NEXTEST_PASSING:
                verdicts.unplaced.append('%s: nextest binary %s is no unit' % (evidence, binary))
            continue
        verdicts.observed.add(binary)
        if status not in NEXTEST_PASSING:
            verdicts.fail(binary, '%s: %s %s' % (evidence, status, test), test)


def read_per_binary(path, universe, verdicts, run):
    for number, row in enumerate(read_tsv(path, PER_BINARY_COLUMNS), 2):
        if run and row['run'] != run:
            continue
        units = universe.by_running(row['source'], row['binary'])
        where = '%s:%d' % (path, number)
        if not units:
            if row['result'] != 'ok':
                verdicts.unplaced.append('%s: no unit runs %s as %s'
                                         % (where, row['source'], row['binary']))
            continue
        for unit_id in units:
            if len(units) == 1:
                verdicts.observed.add(unit_id)
            if row['result'] != 'ok':
                verdicts.fail(unit_id, '%s: test result: %s' % (where, row['result']))


def flake_reason(unit_id, universe, verdicts, reruns, issues):
    """Why a failure outside the selection is a FLAKE, or None for an escape.

    A FLAKE either passed on a later attempt at the same head (`reruns`: the
    selection's own `head` commit, or a prefix of it of at least 7 hex
    characters, to the binaries that passed; on a pull request that head is the
    merge commit the run checked out), or is named in a flake issue, open or
    closed (`issues`, resolved by `resolve_issue_keys`). Everything else that
    failed outside the selection is an ESCAPE.
    """
    for head, binaries in sorted(reruns.items()):
        if len(head) >= 7 and universe.head.startswith(head) and unit_id in binaries:
            return {'reason': 'passed on a rerun at the same head', 'issues': []}
    tests = verdicts.tests.get(unit_id, set())
    found = set()
    for (unit, test), numbers in issues.items():
        if unit == unit_id and (test is None or test in tests):
            found |= set(numbers)
    if found:
        return {'reason': 'named in a flake issue', 'issues': sorted(found)}
    return None


def resolve_issue_keys(issues, universe):
    """`{(unit id, test path or None): [numbers]}` from the issue map's keys.

    A key is a binary id (`cerulion_core::wire_test`), a target name that ONE
    binary of the universe carries (`wire_test` names two packages' tests, so it
    is refused), or a binary id and a full failing test path separated by one
    space (`cerulion_netd client::bounded::tests::oversized_response`), matched
    exactly. A key no binary of this selection carries excuses nothing. An
    ambiguous key is refused: it would excuse a failure the issue never named.
    """
    by_target = {}
    for unit_id, unit in universe.units.items():
        if unit['kind'] != 'doctest':
            by_target.setdefault(unit['target'], []).append(unit_id)
    out = {}
    for key, numbers in sorted(issues.items()):
        unit, _, test = key.partition(' ')
        test = test or None
        if unit in universe.units:
            resolved = unit
        elif test is None and unit in by_target:
            if len(by_target[unit]) != 1:
                raise ScoreError('issue map key %r names %d binaries (%s); name one binary id'
                                 % (key, len(by_target[unit]), ', '.join(sorted(by_target[unit]))))
            resolved = by_target[unit][0]
        else:
            continue
        out.setdefault((resolved, test), set()).update(numbers)
    return {key: sorted(value) for key, value in out.items()}


def score(universe, verdicts, reruns=None, issues=None):
    reruns = reruns or {}
    issues = resolve_issue_keys(issues or {}, universe)
    escapes = []
    flakes = []
    for unit_id in sorted(verdicts.failed):
        if unit_id not in universe.skipped:
            continue
        item = {'binary': unit_id, 'evidence': verdicts.failed[unit_id],
                'tests': sorted(verdicts.tests.get(unit_id, ()))}
        flake = flake_reason(unit_id, universe, verdicts, reruns, issues)
        if flake is None:
            escapes.append(item)
        else:
            item.update(flake)
            flakes.append(item)
    for package in sorted(verdicts.failed_libs):
        if package not in universe.built:
            escapes.append({'binary': 'lib:%s' % package,
                            'evidence': verdicts.failed_libs[package], 'tests': []})
    caught = sorted(unit_id for unit_id in verdicts.failed if unit_id in universe.selected)
    unverified = sorted(universe.skipped - verdicts.observed)
    return {
        'escapes': escapes,
        'flakes': flakes,
        'caught': caught,
        'unplaced': sorted(set(verdicts.unplaced)),
        'failed_libraries': sorted(verdicts.failed_libs),
        'unverified_skips': unverified,
        'counts': {'failed': len(verdicts.failed), 'escapes': len(escapes),
                   'flakes': len(flakes), 'caught': len(caught),
                   'unplaced': len(set(verdicts.unplaced)),
                   'observed': len(verdicts.observed), 'skipped': len(universe.skipped),
                   'unverified_skips': len(unverified)},
    }


def run_score(selection, logs=(), per_test=None, per_binary=None, run=None, reruns=None,
              issues=None):
    universe = Universe(selection)
    verdicts = Verdicts()
    for name, text in logs:
        read_log(text, universe, verdicts, name)
    if per_test:
        read_per_test(per_test, universe, verdicts, run)
    if per_binary:
        read_per_binary(per_binary, universe, verdicts, run)
    return score(universe, verdicts, reruns, issues)


def load_reruns(path):
    """`{head: set(binary ids)}`: the binaries that passed on a later attempt."""
    doc = load_json(path)
    if not isinstance(doc, dict) or not all(
            isinstance(head, str) and re.fullmatch(r'[0-9a-f]{7,40}', head) and
            isinstance(binaries, list) and all(isinstance(b, str) for b in binaries)
            for head, binaries in doc.items()):
        raise ScoreError('%s is not a rerun map: an object of commit to binary ids' % path)
    return {head: set(binaries) for head, binaries in doc.items()}


def load_issues(path):
    """`{binary id, target or test name: [issue numbers]}`."""
    doc = load_json(path)
    if not isinstance(doc, dict) or not all(
            isinstance(key, str) and key and isinstance(numbers, list) and numbers and
            all(isinstance(n, int) and not isinstance(n, bool) and n > 0 for n in numbers)
            for key, numbers in doc.items()):
        raise ScoreError('%s is not an issue map: an object of name to issue numbers' % path)
    return {key: sorted(set(numbers)) for key, numbers in doc.items()}


def load_json(path):
    try:
        with open(path, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError) as err:
        raise ScoreError('cannot read %s: %s' % (path, err))


def report_lines(result):
    counts = result['counts']
    lines = ['router-escapes: %d escape(s), %d flake(s), %d failure(s) caught by the selection, '
             '%d unplaced, %d skipped binaries with no verdict in these logs'
             % (counts['escapes'], counts['flakes'], counts['caught'], counts['unplaced'],
                counts['unverified_skips'])]
    for item in result['escapes']:
        lines.append('router-escapes: ESCAPE %s (%s)' % (item['binary'], item['evidence']))
    for item in result['flakes']:
        lines.append('router-escapes: FLAKE %s, %s%s (%s)'
                     % (item['binary'], item['reason'],
                        ''.join(' #%d' % n for n in item['issues']), item['evidence']))
    for line in result['unplaced']:
        lines.append('router-escapes: UNPLACED %s' % line)
    return lines


# ---------------------------------------------------------------------------
# Self-test: hand-written logs and their hand-written scores
# ---------------------------------------------------------------------------

ST_HEAD = 'abcdef1234567890abcdef1234567890abcdef12'

ST_SELECTION = {
    'schema': 1,
    'head': ST_HEAD,
    'units': [
        {'binary': 'core', 'package': 'core', 'kind': 'lib', 'target': 'core',
         'src': 'crates/core/src/lib.rs'},
        {'binary': 'core::doctest/core', 'package': 'core', 'kind': 'doctest', 'target': 'core',
         'src': 'crates/core/src/lib.rs'},
        {'binary': 'core::wire_test', 'package': 'core', 'kind': 'test', 'target': 'wire_test',
         'src': 'crates/core/tests/wire_test.rs'},
        {'binary': 'core::slow_test', 'package': 'core', 'kind': 'test', 'target': 'slow_test',
         'src': 'crates/core/tests/slow_test.rs'},
        {'binary': 'cli::bin/cerulion', 'package': 'cli', 'kind': 'bin', 'target': 'cerulion',
         'src': 'crates/cli/src/main.rs'},
        {'binary': 'cli::e2e_test', 'package': 'cli', 'kind': 'test', 'target': 'e2e_test',
         'src': 'crates/cli/tests/e2e_test.rs'},
        {'binary': 'cli::flaky_test', 'package': 'cli', 'kind': 'test', 'target': 'flaky_test',
         'src': 'crates/cli/tests/flaky_test.rs'},
        {'binary': 'netd', 'package': 'netd', 'kind': 'lib', 'target': 'netd',
         'src': 'crates/netd/src/lib.rs'},
        {'binary': 'netd::client_test', 'package': 'netd', 'kind': 'test',
         'target': 'client_test', 'src': 'crates/netd/tests/client_test.rs'},
        {'binary': 'tf', 'package': 'tf', 'kind': 'lib', 'target': 'tf',
         'src': 'crates/tf/src/lib.rs'},
        {'binary': 'tf::wire_test', 'package': 'tf', 'kind': 'test', 'target': 'wire_test',
         'src': 'crates/tf/tests/wire_test.rs'},
        {'binary': 'cli::example/demo', 'package': 'cli', 'kind': 'example', 'target': 'demo',
         'src': 'crates/cli/examples/demo.rs'},
    ],
    'selected': [{'binary': 'core::wire_test'}, {'binary': 'cli::e2e_test'},
                 {'binary': 'cli::flaky_test'}, {'binary': 'core'}],
    'skipped': ['cli::bin/cerulion', 'cli::example/demo', 'core::doctest/core', 'core::slow_test',
                'netd', 'netd::client_test', 'tf', 'tf::wire_test'],
    'built_packages': ['cli', 'core'],
}

ESC = '\x1b'
ST_NEXTEST_LOG = '\n'.join([
    '2026-11-17T20:00:00.0000000Z ' + ESC + '[32;1m        PASS' + ESC + '[0m [   0.010s] '
    '(  1/300) core::wire_test encodes',
    '2026-11-17T20:00:01.0000000Z         FAIL [   0.419s] (440/768) core::wire_test decodes',
    '2026-11-17T20:00:02.0000000Z ' + ESC + '[31;1m     SIGSEGV' + ESC + '[0m [  20.637s] '
    '(2710/3035) ' + ESC + '[35;1mcore::slow_test' + ESC + '[0m ' + ESC + '[34;1mwaits' + ESC
    + '[0m',
    '2026-11-17T20:00:03.0000000Z   TRY 1 FAIL [   1.000s] cli::flaky_test races',
    '2026-11-17T20:00:04.0000000Z   TRY 2 PASS [   1.000s] cli::flaky_test races',
    '2026-11-17T20:00:05.0000000Z         PASS [   0.100s] core tests::unit',
    '2026-11-17T20:00:06.0000000Z      Summary [  29.603s] 768 tests run: 767 passed, 1 failed',
    '2026-11-17T20:00:07.0000000Z         FAIL [   0.419s] (440/768) core::wire_test decodes',
    '2026-11-17T20:00:08.0000000Z      TIMEOUT [  60.000s] ghost::gone_test hangs',
])
ST_LIBTEST_LOG = '\n'.join([
    '2026-11-17T20:01:00.0000000Z      Running tests/client_test.rs '
    '(target/debug/deps/client_test-0123456789abcdef)',
    '2026-11-17T20:01:00.5000000Z test reconnect::after_reset ... FAILED',
    '2026-11-17T20:01:01.0000000Z test result: FAILED. 20 passed; 1 failed; 0 ignored; '
    '0 measured; 0 filtered out; finished in 16.08s',
    '2026-11-17T20:01:02.0000000Z      Running unittests src/lib.rs '
    '(target/debug/deps/tf-00000000000000aa)',
    '2026-11-17T20:01:03.0000000Z test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured; '
    '0 filtered out; finished in 0.01s',
    '2026-11-17T20:01:03.5000000Z      Running src/main.rs (target/debug/deps/py_fixture-00ab)',
    '2026-11-17T20:01:03.6000000Z test result: ok. 1 passed; 0 failed',
    '2026-11-17T20:01:03.7000000Z      Running src/main.rs (target/debug/deps/py_other-00ab)',
    '2026-11-17T20:01:03.8000000Z test result: FAILED. 0 passed; 1 failed',
    '2026-11-17T20:01:04.0000000Z    Doc-tests core',
    '2026-11-17T20:01:05.0000000Z test result: ok. 1 passed; 0 failed',
    '2026-11-17T20:01:06.0000000Z ' + ESC + '[1m' + ESC + '[91merror' + ESC
    + '[0m: test failed, to rerun pass `-p netd --test client_test`',
    '2026-11-17T20:01:07.0000000Z error: test failed, to rerun pass `-p cli --test e2e_test`',
    '2026-11-17T20:01:08.0000000Z error: could not compile `netd` (lib) due to 2 previous errors',
    '2026-11-17T20:01:09.0000000Z error: could not compile `cli` (bin "cerulion" test) due to '
    '1 previous error',
    '2026-11-17T20:01:10.0000000Z error: could not compile `core` (lib) due to 1 previous error',
    '2026-11-17T20:01:11.0000000Z error: test failed, to rerun pass `-p nobody --lib`',
])

ST_BLOCKS_LOG = '\n'.join([
    '2026-11-17T20:02:00.0000000Z ##[group]Run cargo test -p tf -- --test-threads=1',
    '2026-11-17T20:02:01.0000000Z      Running tests/wire_test.rs '
    '(target/debug/deps/wire_test-00000000000000bb)',
    '2026-11-17T20:02:02.0000000Z test result: ok. 4 passed; 0 failed',
    '2026-11-17T20:02:03.0000000Z      Running tests/slow_test.rs '
    '(target/debug/deps/slow_test-00000000000000cc)',
    '2026-11-17T20:02:04.0000000Z test waits ... has been running for over 60 seconds',
    '2026-11-17T20:02:04.5000000Z ##[group]Run set -euo pipefail',
    '2026-11-17T20:02:04.5100000Z cargo test -p netd -p tf',
    '2026-11-17T20:02:04.5200000Z ##[endgroup]',
    '2026-11-17T20:02:04.6000000Z      Running unittests src/lib.rs '
    '(target/debug/deps/tf-00000000000000ee)',
    '2026-11-17T20:02:04.7000000Z      Running tests/wire_test.rs '
    '(target/debug/deps/wire_test-00000000000000bb)',
    '2026-11-17T20:02:04.8000000Z test result: ok. 3 passed; 0 failed',
    '2026-11-17T20:02:04.9000000Z test result: FAILED. 0 passed; 1 failed',
    '2026-11-17T20:02:05.0000000Z      Running unittests src/lib.rs '
    '(target/debug/deps/netd-00000000000000dd)',
    '2026-11-17T20:02:06.0000000Z error: failed to run custom build command for `core v2.0.0 '
    '(/work/crates/core)`',
    '2026-11-17T20:02:07.0000000Z error: failed to run custom build command for `rustix v1.1.6`',
    '2026-11-17T20:02:08.0000000Z ##[error]The operation was canceled.',
])

ST_NEXTEST_BROKEN_LOG = '\n'.join([
    '2026-11-17T21:00:00.0000000Z ##[group]Run ./tools/scripts/ci_test_shard.sh cerulion_core 0 4',
    '2026-11-17T21:00:00.1000000Z ##[endgroup]',
    '2026-11-17T21:00:01.0000000Z     Starting 300 tests across 120 binaries',
    '2026-11-17T21:00:02.0000000Z         PASS [   0.010s] core::wire_test encodes',
    '2026-11-17T21:00:03.0000000Z         SLOW [ 60.000s] core::slow_test waits',
    '2026-11-17T21:30:00.0000000Z ##[error]The operation was canceled.',
    '2026-11-17T21:30:01.0000000Z ##[group]Run cargo nextest run -p netd',
    '2026-11-17T21:30:01.1000000Z ##[endgroup]',
    '2026-11-17T21:30:02.0000000Z error: creating test list failed',
    '2026-11-17T21:30:03.0000000Z ##[error]Process completed with exit code 101.',
    '2026-11-17T21:30:04.0000000Z ##[group]Run ./tools/scripts/some_check.sh',
    '2026-11-17T21:30:04.1000000Z ##[endgroup]',
    '2026-11-17T21:30:05.0000000Z ##[error]Process completed with exit code 1.',
    '2026-11-17T21:30:06.0000000Z ##[group]Run cargo nextest run -p cli',
    '2026-11-17T21:30:06.1000000Z ##[endgroup]',
    '2026-11-17T21:30:07.0000000Z     Starting 2 tests across 1 binary',
    '2026-11-17T21:30:08.0000000Z      WEIRD [   0.100s] cli::e2e_test new_status',
    '2026-11-17T21:30:09.0000000Z      Summary [   0.200s] 2 tests run: 1 passed, 1 failed',
    '2026-11-17T21:30:10.0000000Z         SLOW [ 90.000s] (2/2) cli::flaky_test races',
])

ST_PER_TEST = ('run\tjob_id\tjob\tstatus\tseconds\tbinary\ttest\n'
               '1\t10\tTest\tPASS\t0.1\tcore::wire_test\tencodes\n'
               '1\t10\tTest\tFAIL\t0.2\tcore::slow_test\twaits\n'
               '2\t20\tTest\tFAIL\t0.2\tnetd::client_test\tother_run\n')
ST_PER_BINARY = ('run\tjob_id\tjob\tsource\tbinary\tresult\tpassed\tfailed\tseconds\n'
                 '1\t11\tCrates\tsrc/lib.rs\ttf\tok\t3\t0\t0.01\n'
                 '1\t11\tCrates\tsrc/lib.rs\tnetd\tFAILED\t1\t1\t0.02\n')


def self_test():
    fails = []
    cases = 0

    def check(label, got, want):
        if got != want:
            fails.append('%s: got %r, wanted %r' % (label, got, want))

    cases += 1
    result = run_score(ST_SELECTION, logs=[('nextest.log', ST_NEXTEST_LOG)])
    check('nextest escapes', [e['binary'] for e in result['escapes']], ['core::slow_test'])
    check('nextest caught', result['caught'], ['core::wire_test'])
    check('nextest retry passed', 'cli::flaky_test' in result['caught'], False)
    check('nextest unplaced', result['unplaced'],
          ['nextest.log:9: nextest binary ghost::gone_test is no unit'])
    check('nextest unverified', result['unverified_skips'],
          ['cli::bin/cerulion', 'cli::example/demo', 'core::doctest/core', 'netd',
           'netd::client_test', 'tf', 'tf::wire_test'])

    cases += 1
    result = run_score(ST_SELECTION, logs=[('libtest.log', ST_LIBTEST_LOG)])
    check('libtest escapes', [e['binary'] for e in result['escapes']],
          ['cli::bin/cerulion', 'netd::client_test', 'lib:netd'])
    check('libtest caught', result['caught'], ['cli::e2e_test'])
    check('libtest failed libraries', result['failed_libraries'], ['core', 'netd'])
    check('libtest unplaced', result['unplaced'],
          ['libtest.log:17: error: test failed, to rerun pass `-p nobody --lib`',
           'libtest.log:8: no unit runs src/main.rs as py_other'])
    check('libtest unverified', result['unverified_skips'],
          ['cli::example/demo', 'core::slow_test', 'netd', 'tf::wire_test'])

    cases += 1
    tsv_dir = os.environ.get('TMPDIR', '/tmp')
    per_test = os.path.join(tsv_dir, 'ci_router_escapes_selftest_%d_per_test.tsv' % os.getpid())
    per_binary = os.path.join(tsv_dir, 'ci_router_escapes_selftest_%d_per_binary.tsv'
                              % os.getpid())
    try:
        with open(per_test, 'w', encoding='utf-8') as handle:
            handle.write(ST_PER_TEST)
        with open(per_binary, 'w', encoding='utf-8') as handle:
            handle.write(ST_PER_BINARY)
        result = run_score(ST_SELECTION, per_test=per_test, per_binary=per_binary, run='1')
        check('tsv escapes', [e['binary'] for e in result['escapes']],
              ['core::slow_test', 'netd'])
        check('tsv run filter', 'netd::client_test' in [e['binary'] for e in result['escapes']],
              False)
        result = run_score(ST_SELECTION, per_test=per_test, per_binary=per_binary)
        check('tsv every run', [e['binary'] for e in result['escapes']],
              ['core::slow_test', 'netd', 'netd::client_test'])
    finally:
        for path in (per_test, per_binary):
            if os.path.exists(path):
                os.remove(path)

    # FLAKES and ESCAPES. A failure outside the selection is a flake when it
    # passed on a later attempt at the same head, or when a flake issue names
    # its binary, its target or one of its failing tests; otherwise it is an
    # escape. Each arm from both sides.
    cases += 1
    flake_rows = [
        ('rerun at the same head', {'abcdef1': ['core::slow_test']}, {}, [],
         [('core::slow_test', 'passed on a rerun at the same head', [])]),
        ('rerun at another head', {'1234567': ['core::slow_test']}, {}, ['core::slow_test'], []),
        ('a head prefix under 7 characters', {'abcdef': ['core::slow_test']}, {},
         ['core::slow_test'], []),
        ('rerun of another binary', {'abcdef1': ['core::wire_test']}, {}, ['core::slow_test'],
         []),
        ('issue naming the target', {}, {'slow_test': [684]}, [],
         [('core::slow_test', 'named in a flake issue', [684])]),
        ('issue naming the binary and its failing test', {}, {'core::slow_test waits': [586]},
         [], [('core::slow_test', 'named in a flake issue', [586])]),
        ('issue naming the binary', {}, {'core::slow_test': [5]}, [],
         [('core::slow_test', 'named in a flake issue', [5])]),
        ('issue naming a test path that only ends like the failing one', {},
         {'core::slow_test aits': [7]}, ['core::slow_test'], []),
        ('issue naming another binary', {}, {'core::wire_test': [1]}, ['core::slow_test'], []),
        ('issue naming a binary this selection does not carry', {}, {'nobody::x': [3]},
         ['core::slow_test'], []),
    ]
    for label, reruns, issues, want_escapes, want_flakes in flake_rows:
        result = run_score(ST_SELECTION, logs=[('nextest.log', ST_NEXTEST_LOG)], reruns=reruns,
                           issues=issues)
        check('flake: %s escapes' % label, [e['binary'] for e in result['escapes']],
              want_escapes)
        check('flake: %s flakes' % label,
              [(f['binary'], f['reason'], f['issues']) for f in result['flakes']], want_flakes)
    # A libtest failure is matched by the failing test's name its log prints.
    result = run_score(ST_SELECTION, logs=[('libtest.log', ST_LIBTEST_LOG)],
                       issues={'netd::client_test reconnect::after_reset': [949]})
    check('flake: libtest test name',
          [(f['binary'], f['tests'], f['issues']) for f in result['flakes']],
          [('netd::client_test', ['reconnect::after_reset'], [949])])
    check('flake: libtest leaves the rest', [e['binary'] for e in result['escapes']],
          ['cli::bin/cerulion', 'lib:netd'])
    # A target name two binaries carry is refused, never read as both.
    try:
        run_score(ST_SELECTION, logs=[('nextest.log', ST_NEXTEST_LOG)],
                  issues={'wire_test': [1]})
        fails.append('an ambiguous issue key was accepted')
    except ScoreError:
        pass
    # A binary that died without a result while the next one printed: the
    # next one's failing test names are not the first one's, so an issue that
    # names them cannot excuse the crash.
    crash = '\n'.join([
        '2026-11-17T22:00:00.0000000Z      Running tests/client_test.rs '
        '(target/debug/deps/client_test-00000000000000aa)',
        '2026-11-17T22:00:01.0000000Z      Running tests/slow_test.rs '
        '(target/debug/deps/slow_test-00000000000000bb)',
        '2026-11-17T22:00:02.0000000Z test b_case ... FAILED',
        '2026-11-17T22:00:03.0000000Z test result: FAILED. 0 passed; 1 failed',
    ])
    result = run_score(ST_SELECTION, logs=[('crash.log', crash)],
                       issues={'netd::client_test b_case': [9]})
    check('crash then a failing neighbour', [e['binary'] for e in result['escapes']],
          ['core::slow_test', 'netd::client_test'])
    check('crash then a failing neighbour names nothing',
          [e['tests'] for e in result['escapes']], [[], []])
    # A binary that died without a result, named by cargo's rerun line, then a
    # binary that passed: the pass closes its own block, not the crashed one.
    cases += 1
    crash_pass = '\n'.join([
        '2026-11-17T22:00:00.0000000Z      Running tests/client_test.rs '
        '(target/debug/deps/client_test-00000000000000aa)',
        '2026-11-17T22:00:01.0000000Z error: test failed, to rerun pass '
        '`-p netd --test client_test`',
        '2026-11-17T22:00:02.0000000Z      Running tests/slow_test.rs '
        '(target/debug/deps/slow_test-00000000000000bb)',
        '2026-11-17T22:00:03.0000000Z test waits ... ok',
        '2026-11-17T22:00:04.0000000Z test result: ok. 1 passed; 0 failed',
    ])
    result = run_score(ST_SELECTION, logs=[('crash.log', crash_pass)])
    check('crash then a passing binary escapes',
          [e['binary'] for e in result['escapes']], ['netd::client_test'])
    check('crash then a passing binary observes the pass',
          'core::slow_test' in result['unverified_skips'], False)

    # A test nextest lists as SLOW after its run finished is the pass it had,
    # from a log and from the per-test table, whatever order the rows come in;
    # a test with SLOW alone has no verdict.
    cases += 1
    slow_log = '\n'.join([
        '2026-11-17T22:10:00.0000000Z         PASS [  61.000s] core::slow_test waits',
        '2026-11-17T22:10:01.0000000Z      Summary [  61.100s] 1 test run: 1 passed',
        '2026-11-17T22:10:02.0000000Z         SLOW [> 60.000s] core::slow_test waits',
    ])
    result = run_score(ST_SELECTION, logs=[('slow.log', slow_log)])
    check('slow pass from a log', [e['binary'] for e in result['escapes']], [])
    check('slow pass from a log is observed', 'core::slow_test' in result['unverified_skips'],
          False)
    tmp = os.environ.get('TMPDIR', '/tmp')
    slow_tsv = os.path.join(tmp, 'ci_router_escapes_selftest_%d_slow.tsv' % os.getpid())
    head = 'run\tjob_id\tjob\tstatus\tseconds\tbinary\ttest\n'
    try:
        for label, rows, want_escapes, want_observed in (
                ('pass then slow', ['PASS', 'SLOW'], [], True),
                ('slow then pass', ['SLOW', 'PASS'], [], True),
                ('slow alone', ['SLOW'], [], False),
                ('slow then fail', ['SLOW', 'FAIL'], ['core::slow_test'], True)):
            with open(slow_tsv, 'w', encoding='utf-8') as handle:
                handle.write(head + ''.join('1\t10\tTest\t%s\t61.0\tcore::slow_test\twaits\n'
                                            % status for status in rows))
            result = run_score(ST_SELECTION, per_test=slow_tsv)
            check('slow from the table, %s' % label,
                  [e['binary'] for e in result['escapes']], want_escapes)
            check('slow from the table, %s, observed' % label,
                  'core::slow_test' not in result['unverified_skips'], want_observed)
        # A per-test table without a column the reader reads is refused with a
        # message, not a missing key.
        with open(slow_tsv, 'w', encoding='utf-8') as handle:
            handle.write('run\tjob\tstatus\tseconds\tbinary\ttest\n'
                         '1\tTest\tPASS\t0.1\tcore::wire_test\tencodes\n')
        try:
            run_score(ST_SELECTION, per_test=slow_tsv)
            fails.append('a per-test table without job_id was accepted')
        except ScoreError as err:
            check('a per-test table without job_id names it', 'job_id' in str(err), True)
    finally:
        if os.path.exists(slow_tsv):
            os.remove(slow_tsv)

    # The two maps are refused when they are not the shape they claim.
    cases += 1
    tmp = os.environ.get('TMPDIR', '/tmp')
    bad = os.path.join(tmp, 'ci_router_escapes_selftest_%d_map.json' % os.getpid())
    try:
        for loader, text in ((load_reruns, '{"abc": ["x"]}'), (load_reruns, '["x"]'),
                             (load_issues, '{"x": []}'), (load_issues, '{"x": [0]}'),
                             (load_issues, '{"x": [true]}'), (load_issues, 'not json')):
            with open(bad, 'w', encoding='utf-8') as handle:
                handle.write(text)
            try:
                loader(bad)
                fails.append('%s accepted %s' % (loader.__name__, text))
            except ScoreError:
                pass
        with open(bad, 'w', encoding='utf-8') as handle:
            handle.write('{"slow_test": [684, 684]}')
        check('issue map dedups', load_issues(bad), {'slow_test': [684]})
    finally:
        if os.path.exists(bad):
            os.remove(bad)

    # Blocks with no result fail their binary (a hang to the job's limit, a
    # crash, a cancel), closed at the next step or the end of the log; a
    # result closes the OLDEST open block, because a `Running` line can land
    # ahead of the result before it; a step's `-p` list tells two same-named
    # binaries apart; a member's build script failing fails its library; a
    # registry crate's build script is a failure no binary owns.
    cases += 1
    result = run_score(ST_SELECTION, logs=[('blocks.log', ST_BLOCKS_LOG)])
    check('blocks escapes', [e['binary'] for e in result['escapes']],
          ['core::slow_test', 'netd', 'tf::wire_test'])
    check('blocks observed tf only', 'core::wire_test' in result['caught'], False)
    check('blocks unverified', result['unverified_skips'],
          ['cli::bin/cerulion', 'cli::example/demo', 'core::doctest/core',
           'netd::client_test'])
    check('blocks failed libraries', result['failed_libraries'], ['core'])
    check('blocks unplaced', result['unplaced'],
          ['blocks.log:15: error: failed to run custom build command for `rustix v1.1.6`'])

    # A nextest run cut off with no `Summary` (here a test whose last word is
    # `SLOW`, then a cancel), a test list nextest could not build, a step
    # that failed with no binary named, and a status word this reader does not
    # know: each is a failure or unplaced, never a clean score. A `SLOW` in
    # the closing list after a `Summary` is a slow pass.
    cases += 1
    result = run_score(ST_SELECTION, logs=[('broken.log', ST_NEXTEST_BROKEN_LOG)])
    check('broken escapes', [e['binary'] for e in result['escapes']], [])
    check('broken slow without summary', 'core::slow_test' in result['unverified_skips'], True)
    check('broken caught', result['caught'], ['cli::e2e_test'])
    check('broken slow pass', 'cli::flaky_test' in result['caught'], False)
    check('broken unplaced', result['unplaced'],
          ['broken.log:13: ##[error]Process completed with exit code 1., and no failure in '
           'the step names a binary',
           'broken.log:3: a nextest run ended with no Summary (broken.log:7: the next step '
           'began)',
           'broken.log:9: error: creating test list failed'])

    # The same-named block with no step header is a verdict on neither unit.
    cases += 1
    headless = '\n'.join(ST_BLOCKS_LOG.split('\n')[1:3])
    result = run_score(ST_SELECTION, logs=[('headless.log', headless)])
    check('ambiguous block', 'tf::wire_test' in result['unverified_skips'], True)

    # The exit status: 1 for an escape, 3 for a clean score with unverified
    # skips, 0 only when every skipped binary has a verdict.
    cases += 1
    tmp = os.environ.get('TMPDIR', '/tmp')
    sel_path = os.path.join(tmp, 'ci_router_escapes_selftest_%d.json' % os.getpid())
    log_path = os.path.join(tmp, 'ci_router_escapes_selftest_%d.log' % os.getpid())
    quiet = io.StringIO()
    try:
        with open(sel_path, 'w', encoding='utf-8') as handle:
            json.dump(ST_SELECTION, handle)
        with open(log_path, 'w', encoding='utf-8') as handle:
            handle.write(ST_NEXTEST_LOG)
        with contextlib.redirect_stdout(quiet):
            check('exit escape', main(['--selection', sel_path, log_path]), 1)
        with open(log_path, 'w', encoding='utf-8') as handle:
            handle.write('2026-11-17T20:00:00.0000000Z         PASS [   0.010s] '
                         'core::wire_test encodes\n')
        with contextlib.redirect_stdout(quiet):
            check('exit unverified', main(['--selection', sel_path, log_path]), 3)
        everything_path = sel_path + '.all'
        with open(everything_path, 'w', encoding='utf-8') as handle:
            json.dump(dict(ST_SELECTION, selected=[{'binary': u['binary']}
                                                   for u in ST_SELECTION['units']],
                           skipped=[]), handle)
        with contextlib.redirect_stdout(quiet):
            check('exit clean', main(['--selection', everything_path, log_path]), 0)
        os.remove(everything_path)
    finally:
        for path in (sel_path, log_path):
            if os.path.exists(path):
                os.remove(path)

    # A selection that skips nothing has no escape whatever failed.
    cases += 1
    everything = dict(ST_SELECTION)
    everything['selected'] = [{'binary': u['binary']} for u in ST_SELECTION['units']]
    everything['skipped'] = []
    everything['built_packages'] = ['cli', 'core', 'netd', 'tf']
    result = run_score(everything, logs=[('a', ST_NEXTEST_LOG), ('b', ST_LIBTEST_LOG)])
    check('full selection', result['escapes'], [])

    # Refusals: a document that is not a selection, and one whose lists do not
    # partition its units.
    cases += 1
    for label, doc in (('not a selection', {'schema': 2}),
                       ('overlap', dict(ST_SELECTION, skipped=ST_SELECTION['skipped'] + ['core']))):
        try:
            Universe(doc)
            fails.append('%s: accepted' % label)
        except ScoreError:
            pass

    # Determinism: the same inputs twice give the same document.
    cases += 1
    one = json.dumps(run_score(ST_SELECTION, logs=[('a', ST_NEXTEST_LOG), ('b', ST_LIBTEST_LOG)]),
                     sort_keys=True)
    two = json.dumps(run_score(ST_SELECTION, logs=[('a', ST_NEXTEST_LOG), ('b', ST_LIBTEST_LOG)]),
                     sort_keys=True)
    check('determinism', one == two, True)

    if fails:
        for line in fails:
            print('ci_router_escapes self-test: %s' % line, file=sys.stderr)
        return 1
    print('ci_router_escapes: self-test OK (%d cases)' % cases)
    return 0


def main(argv):
    parser = argparse.ArgumentParser(description='Escapes of a shadow test selection.')
    parser.add_argument('--selection', help='the ci_test_router.py JSON document')
    parser.add_argument('--per-test', help='a per_test.tsv extract')
    parser.add_argument('--per-binary', help='a per_binary.tsv extract')
    parser.add_argument('--run', help='only the TSV rows of this run id')
    parser.add_argument('--reruns', help='JSON: head commit to the binaries that passed on a '
                        'later attempt at it')
    parser.add_argument('--issues', help='JSON: binary id, target or test name to the flake '
                        'issue numbers that name it')
    parser.add_argument('--json', help='write the score here')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('logs', nargs='*', help='job logs of the full run')
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.selection or not (args.logs or args.per_test or args.per_binary):
        parser.error('--selection and at least one log or TSV are required')
    try:
        with open(args.selection, encoding='utf-8') as handle:
            selection = json.load(handle)
        logs = []
        for path in args.logs:
            with open(path, encoding='utf-8', errors='replace') as handle:
                logs.append((path, handle.read()))
        reruns = load_reruns(args.reruns) if args.reruns else None
        issues = load_issues(args.issues) if args.issues else None
        result = run_score(selection, logs, args.per_test, args.per_binary, args.run, reruns,
                           issues)
    except (OSError, ValueError, ScoreError) as err:
        print('ci_router_escapes: %s' % err, file=sys.stderr)
        return 2
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, indent=1, sort_keys=True)
            handle.write('\n')
    for line in report_lines(result):
        print(line)
    if result['escapes'] or result['unplaced']:
        return 1
    return 3 if result['unverified_skips'] else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
